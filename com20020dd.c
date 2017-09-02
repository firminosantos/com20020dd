/**
 * @file   com20020dd.c
 * @author Firmino dos Santos Filho
 * @version 1.1
 * @date   14 October 2016
 * @brief  Linux loadable kernel module (LKM) for interfacing a COM20020 Arcnet Controller
 *         This module maps to /dev/arcchar
 *         for loading:	insmod com20020dd.ko
 *         for remove:  rmmod com20020dd.ko
 *         for loading at boot time:
 *  	      update /etc/modules *
 */

#include <linux/init.h>				// Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>			// Core header for loading LKMs into the kernel
#include <linux/kernel.h>			// Contains types, macros, functions for the kernel
#include <linux/gpio.h>             // for the GPIO interface
#include <linux/interrupt.h>        // for the IRQ code
#include <linux/timer.h>			// Interface to Kernel Timers.
#include <asm/atomic.h>				// Local atomic operations
#include <linux/delay.h>
#include <linux/spinlock.h>			// Spinlock - a way to protect a shared resource from being
									//  modified by two or more processes simultaneously
#include <linux/io.h>				// for ioremap
#include <linux/hrtimer.h>			// Usage of high-resolution timer API
#include <linux/sched.h>
#include <linux/fs.h>				// Header for the Linux file system support
#include <linux/poll.h>				// Poll file descriptor
#include <linux/device.h>			// Header to support the kernel Driver Model
#include <linux/jiffies.h>			// Access to internal kernel counter

#define  DEVICE_NAME "arcchar"    	///< The device will appear at /dev/arcchar using this value
#define  CLASS_NAME  "arc"        	///< The device class -- this is a arcnet device driver

MODULE_LICENSE("GPL");				///< The license type -- this affects available functionality
MODULE_AUTHOR("Firmino dos Santos Filho");
MODULE_DESCRIPTION("ARCNET Interface char driver for the SBC - Single Board Computers");
MODULE_VERSION("1.1");

static int    majorNumber;                  ///< Store the device number -- determined automatically
static char   message[256] = {'0','1','2','3','4','5','6','7','8','9'};  ///< Memory for the string that is passed from userspace
static int    numberOpens = 0;              	///< Counts the number of times the device is opened
static struct class*  arccharClass  = NULL; 	///< The device-driver class struct pointer
static struct device* arccharDevice = NULL; 	///< The device-driver device struct pointer

char buffstrd[17] = {"28/01/2017 08h00"};

static DEFINE_MUTEX(arcchar_mutex);	    ///< Macro to declare a new mutex

/// The prototype functions for the character driver
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
static unsigned int arc_p_poll(struct file *filp, poll_table *pt);
unsigned char readDatabus(unsigned int a0a1a2);
void writeDatabus(unsigned int a0a1a2, unsigned char data);
int configGPIO(void);
unsigned int rblockri (uint src, unsigned char *dest);
unsigned char rbyte(unsigned int add);
void wbyte(unsigned int add, unsigned char wdata);
void rblock (uint src, unsigned char *dest, uint nbytes);
unsigned short configCOM20020(unsigned char pnode);
void wblock (unsigned char *src, uint dest, uint nbytes);
/** @brief COM20020 Data Variables
 */
#define	COM_CONF		6		/* CONFIGURATION: read/write */
#define	COM_STATUS		0		/* STATUS: read only */
#define	GPIO16		16
#define	GPIOINTR			GPIO16
static unsigned int gpioIntr = GPIOINTR;    // GPIOINTR
unsigned char 	node = 0;
atomic_t stat_aux = ATOMIC_INIT(0);
atomic_t data_avail_to_read = ATOMIC_INIT(0);
static DECLARE_WAIT_QUEUE_HEAD(read_queue);

/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 * /linux/fs.h lists the callback functions associated with file operations.
 * Implemented operations: open, read, write, poll and release calls
 */
static struct file_operations fops =
{
   .owner =	THIS_MODULE,
   .open = dev_open,
   .read = dev_read,
   .write = dev_write,
   .poll  =	arc_p_poll,
   .release = dev_release
};

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @return returns 0 if successful
 */
static int dev_open(struct inode *inodep, struct file *filep){

   if(!mutex_trylock(&arcchar_mutex)){                  // Try to acquire the mutex (returns 0 on fail)
	printk(KERN_ALERT "ARCChar: Device in use by another process");
	return -EBUSY;
   }
   numberOpens++;
   printk(KERN_INFO "ARCChar: Device has been opened %d time(s)\n", numberOpens);
   return 0;
}

/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case it uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset is not required
 *  @return returns number of bytes read or
 *  @return returns a bad address message (i.e. -14)in case of failure
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
    int error_count = 0;
	unsigned char read_byte;
	unsigned char opd = 0;
	unsigned char reg = 0;
	unsigned int addr = 0;
	unsigned int addrl, addrh;
	#define		SINGLE_BYTE		1
	#define		HIGH_ORDER_BYTE		8

	//Operations
	#define		RD_REG		1		//Read register
	#define		RD_BYTE		2		//Read byte
	#define		RD_BLOCK	6		//Read block of data
	#define		RD_INTR		7		//Read if status interrupt pin

	//operation required
	opd = buffer[0] >> 4;

	//Arcnet COM20020 register to be accessed.
	reg = buffer[0] & 0x7;

	//Arcnet COM20020 address to be used.
	addrl = buffer[1];
	addrh = (buffer[0] & 0x7) << HIGH_ORDER_BYTE;

	if (len > 256)
	{
		return -EFAULT;
	}

	#ifdef WRITE_TEST
		printk(KERN_INFO "dev read buf[0]=%x opd=%x reg=%x\n", buffer[0], opd, reg);
	#endif

	//check operation requested
	if ( (opd == RD_REG) && (len == SINGLE_BYTE) )
	{
   		read_byte = readDatabus(reg);

	   // copy_to_user has the format ( * to, *from, size) and returns 0 on success
	   error_count = copy_to_user(buffer, &read_byte, 1);

	}
	else if (opd == RD_BYTE)
	{
		addr =  buffer[1] | (reg << HIGH_ORDER_BYTE);
		buffer[0] = rbyte(addr);
	}
	else if ( (opd == RD_BLOCK)  )
	{
		addr = addrh | addrl;

		#ifdef WRITE_TEST1
			printk(KERN_INFO "RD_BLOCK addr=%x opd=%x len=%x\n", addr, opd, len);
		#endif

		rblock(addr, message, len);

		/*copy_to_user() =>Its name speaks for itself: it simply transfers specific data
		  from the kernel buffer to the buffer allocated in the user space by copying it.
		  In addition, it also verifies if a pointer is valid and if the buffer size is
		  big enough.*/
		error_count = copy_to_user(buffer, message, len);
	}
	else if ( (opd == RD_INTR)  )
	{
		//read INTR
   		read_byte = gpio_get_value(gpioIntr);

	   // copy_to_user has the format ( * to, *from, size) and returns 0 on success
	   error_count = copy_to_user(buffer, &read_byte, SINGLE_BYTE);
	}
	else
	{
		error_count++;
	}

   if (error_count==0)           // success!
   {
      return (len);
   }
   else
   {
      return -EFAULT;      // Failed -- return a bad address message (i.e. -14)
   }
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset is not required.
 *  @return returns number of bytes write or
 *  @return returns a bad address message (i.e. -14)in case of failure
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
    int error_count = 0;
	unsigned char opd = 0;
	unsigned char reg = 0;
	unsigned int addr = 0;
	unsigned int addrl, addrh;
	unsigned char com_status_aux;
	#define		HIGH_ORDER_BYTE		8
	#define		MAX_BUFFER_SIZE		(256+2)

	//Operations
	#define		WR_REG		0		//Write register
	#define		WR_BYTE		3		//Write byte
	#define		WR_BLOCK	4		//Write block
	#define		WR_CONFIG	5		//Write configuration
	#define		WR_HARDWARE_RESET	0xA		//Send Hardware Reset
	#define		WR_FORCED_INTR		0xB		//Send command forced interrupt


	//data[0] format =>  0xoooo rrrr		where oooo operation rrrr register
	//operation required
	opd = buffer[0] >> 4;
	//Arcnet COM20020 register to be accessed.
	reg = buffer[0] & 0x7;
	//Arcnet COM20020 address to be used.
	addrl = buffer[1];
	addrh = (buffer[0] & 0x7) << HIGH_ORDER_BYTE;

	#ifdef WRITE_TEST
		printk(KERN_INFO "dev write buf[0]=%x opd=%x reg=%x\n", buffer[0], opd, reg);
	#endif

	if (len > MAX_BUFFER_SIZE)
	{
		return -EFAULT;
	}

    if ( (opd == WR_REG) && (len == 2) )
    {
   		writeDatabus(reg, buffer[1]);
    }
	else if (opd == WR_BYTE)
	{
		addr = (reg << 8) | buffer[1];
		wbyte(addr, buffer[2]);
	}
	else if (opd == WR_BLOCK)
	{
		addr = addrh | addrl;

		#ifdef WRITE_TEST1
			printk(KERN_INFO "WD_BLOCK addr=%x opd=%x len=%x\n", addr, opd, len-2);
		#endif

   		//void wblock (unsigned char *src, uint dest, uintnbytes)
		wblock ((unsigned char *)(buffer+2), addr, len-2);
	}
	else if ( (opd == WR_CONFIG) && (len == 2) && (reg == COM_CONF) )
	{
		error_count = configCOM20020(buffer[1]);
	}
	else if ( (opd == WR_HARDWARE_RESET) && (len == 2) && (reg == COM_CONF) )
	{
		//reset confirmed
		node = buffer[1];
		error_count = configGPIO();
	}
	else if ( (opd == WR_FORCED_INTR) && (len == 2) && (reg == COM_CONF) )
	{
		/* status do controlador */
		com_status_aux = readDatabus(COM_STATUS);
		atomic_set(&stat_aux, com_status_aux);
		atomic_set(&data_avail_to_read, atomic_read(&stat_aux));
		wake_up(&read_queue);

		node = buffer[1];
		error_count = 0;
	}
	else
	{
		//error, wrong write
		error_count = 1;
	}

   if (error_count==0)           // success!
   {
      return (len);
   }
   else
   {
      return -EFAULT;      // Failed -- return a bad address message (i.e. -14)
   }
}

/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @return returns 0 if successful
 */
static int dev_release(struct inode *inodep, struct file *filep){
   mutex_unlock(&arcchar_mutex);                      // release the mutex (i.e., lock goes up)
   printk(KERN_INFO "ARCChar: Device successfully closed\n");
   return 0;
}


/** @brief Applications that use nonblocking I/O use the poll, select calls; they have essentially the same
 *  functionality: each allow a process to determine whether it can read from or write to one or more open
 *  files without blocking.
 *  The device method is in charge of these two steps:
 *  1. Call poll waiton one wait queue that could indicate a change in the poll status.
 *     If no file descriptors are currently available for I/O, the kernel causes the process to wait on
 *     the wait queues for all file descriptors passed to the system call.
 *  2. Return a bit mask describing the operations (if any) that could be immediately performed without
 *     blocking.
 *
 *  @param filp	A pointer to file descriptor
 *  @param pt 	A pointer to to wait queues that will be used for waking the poll call.
 *  @return Return a bit mask describing the operations (if any) that could be immediately performed without blocking.
 *  @return Returns 0 if there is no data available.
 */
static unsigned int arc_p_poll(struct file *filp, poll_table *pt)
{
unsigned int mask = 0;

	//printk(KERN_INFO "ARCChar: arc_p_poll\n");

	//register interest in the available data information
	poll_wait(filp, &read_queue, pt);

	if (atomic_read(&data_avail_to_read))
	{
		atomic_set(&data_avail_to_read, 0);		//reset data available information
		mask |= POLLIN | POLLRDNORM; /* readable */
	}
	return mask;
}


/** @brief Basic I/O Interface
  *        GPIO Direct Access
  */
static volatile int piPeriphBase = 0x3F000000;		//for raspberry pi 2 and 3.

#ifndef GPIO_BASE
	#define GPIO_BASE  (piPeriphBase + 0x200000)
#endif

#define TIMER_BASE  (piPeriphBase + 0x003000)
#define TIMER_LEN  	0x1C

#define GPIO_LEN  0xB4

#define GPSET0 7
#define GPSET1 8

#define GPCLR0 10
#define GPCLR1 11

#define GPLEV0 13
#define GPLEV1 14

#define GPPUD     37
#define GPPUDCLK0 38
#define GPPUDCLK1 39

#define SYST_CS  0
#define SYST_CLO 1
#define SYST_CHI 2

// pin modes
#define INPUT  0b000
#define OUTPUT 0b001
#define ALT0   0b100
#define ALT1   0b101
#define ALT2   0b110
#define ALT3   0b111
#define ALT4   0b011
#define ALT5   0b010

static volatile unsigned int  *gpioReg = NULL;
static volatile unsigned int  *timerReg =  NULL;

volatile unsigned *timer =  NULL;

#define PI_BANK (gpio>>5)
#define PI_BIT  (1<<(gpio&0x1F))

/* gpio modes. */

#define PI_INPUT  0
#define PI_OUTPUT 1
#define PI_ALT0   4
#define PI_ALT1   5
#define PI_ALT2   6
#define PI_ALT3   7
#define PI_ALT4   3
#define PI_ALT5   2

/** @brief Function to Set gpio mode
 *  @param gpio address
 *  @param mode	set mode
 *  @return	none
 */
void gpioSetMode(unsigned gpio, unsigned mode)
{
   int reg, shift;

   reg   =  gpio/10;
   shift = (gpio%10) * 3;

   gpioReg[reg] = (gpioReg[reg] & ~(7<<shift)) | (mode<<shift);
}

/** @brief Function to read gpio mode
 *  @param gpio address
 *  @return	gpio mode
 */
int gpioGetMode(unsigned gpio)
{
   int reg, shift;

   reg   =  gpio/10;
   shift = (gpio%10) * 3;

   return (*(gpioReg + reg) >> shift) & 7;
}

/* Values for pull-ups/downs off, pull-down and pull-up. */
#define PI_PUD_OFF  0
#define PI_PUD_DOWN 1
#define PI_PUD_UP   2

/** @brief Function to Set Pullup or PullDown
 *  @param gpio address
 *  @param pud	Up or down
 *  @return	none
 */
void gpioSetPullUpDown(unsigned gpio, unsigned pud)
{
   *(gpioReg + GPPUD) = pud;
   udelay(20);
   *(gpioReg + GPPUDCLK0 + PI_BANK) = PI_BIT;
   udelay(20);
   *(gpioReg + GPPUD) = 0;
   *(gpioReg + GPPUDCLK0 + PI_BANK) = 0;
}


/** @brief Function to read gpio
 *  @param gpio address
 *  @return	gpio data
 */
int gpioRead(unsigned gpio)
{
   if ((*(gpioReg + GPLEV0 + PI_BANK) & PI_BIT) != 0) return 1;
   else                                         return 0;
}

/** @brief Function to write gpio
 *  @param gpio address
 *  @param level data to be set
 *  @return	none
 */
void gpioWrite(unsigned gpio, unsigned level)
{
   if (level == 0) *(gpioReg + GPCLR0 + PI_BANK) = PI_BIT;
   else            *(gpioReg + GPSET0 + PI_BANK) = PI_BIT;
}

/** @brief Function to set trigger level
 *  @param gpio address
 *  @param pulseLen pulse length (duration)
 *  @param level data to be set
 *  @return	none
 */
void gpioTrigger(unsigned gpio, unsigned pulseLen, unsigned level)
{
   if (level == 0) *(gpioReg + GPCLR0 + PI_BANK) = PI_BIT;
   else            *(gpioReg + GPSET0 + PI_BANK) = PI_BIT;

   udelay(pulseLen);

   if (level != 0) *(gpioReg + GPCLR0 + PI_BANK) = PI_BIT;
   else            *(gpioReg + GPSET0 + PI_BANK) = PI_BIT;
}

/** @brief Function to read bank of gpio's
 *         Bit (1<<x) will be set if gpio x is high
 *  @param gpio address
 *  @return	bank of gpio's
 */
unsigned int gpioReadBank1(void) { return (*(gpioReg + GPLEV0)); }
unsigned int gpioReadBank2(void) { return (*(gpioReg + GPLEV1)); }

/** @brief Function to clear bank of gpio's
 *         To clear gpio x bit or in (1<<x)
 *  @param gpio address
 *  @return	none
 */
void gpioClearBank1(int bits) { *(gpioReg + GPCLR0) = bits; }
void gpioClearBank2(int bits) { *(gpioReg + GPCLR1) = bits; }

/** @brief Function to set bank of gpio's
 *         To set gpio x bit or in (1<<x)
 *  @param gpio address
 *  @return	none
 */
void gpioSetBank1(unsigned int bits) { *(gpioReg + GPSET0) = bits; }	//bits 0..31
void gpioSetBank2(unsigned int bits) { *(gpioReg + GPSET1) = bits; }	//bits 32..53

/** @brief Function to set bank 1 of 8 gpio's
 *  @param bits
 *  @return	none
 */
void gpioSetPort8bits(unsigned int bits)
{
	gpioSetBank1(bits & 0xff);
	gpioClearBank1(~bits | 0xffffff00);
}

/** @brief COM20020 I/O Defines / Raspberry Pi 2/3
 */
#define	GPIOFORWRITE		GPIO06
#define	GPIO20		20	//bit0 databus.
#define	GPIO27		27	//bit7 databus.
#define	GPIO12		12
#define	GPIO07		7
#define	GPIO17		17
#define	GPIO04		4
#define	GPIO13		13
#define	GPIO06		6
#define	GPIO05		5
#define	GPIO18		18
#define	GPIO19		19
#define	GPIO08		8

/** @brief COM20020 Data Variables
 */
static unsigned int irqNumber;          // share IRQ num within file

atomic_t intr  = ATOMIC_INIT(0);		//atomic_set(&intr, 1);
atomic_t count = ATOMIC_INIT(0);		//atomic_set(&count, 1); atomic_add(1, &count);
										//atomic_inc(&count); atomic_read(&count);
atomic_t count2 = ATOMIC_INIT(0);


static DEFINE_SPINLOCK(mr_lock) ;

unsigned long flags;

// prototype for the custom IRQ handler function, function below
static irq_handler_t  erpi_gpio_irq_handler(unsigned int irq, void
                                    *dev_id, struct pt_regs *regs);

/** @brief COM20020 REGISTERS
 */
#define		COM_STATUS		0		/* STATUS: read only */
#define		COM_IMASK		0		/* INTERRUPT MASK: write only */
#define		COM_DIAG		1		/* DIAG. STATUS: read only */
#define		COM_CMD			1		/* COMMAND: write only */
#define		COM_PTRH		2		/* ADD PTR HIGH: read/write */
#define		COM_PTRL		3		/* ADD PTR LOW: read/write */
#define		COM_DATA		4		/* DATA: read/write */
#define		COM_SUBADR		5		/* SUBADR R/W */
#define		COM_CONF		6		/* CONFIGURATION: read/write */
#define		COM_SUB			7		/* SUBREG: read/write */

#define	DIS_TX		01				/* Disable Transmitter */
#define	BUFSIZE		256				/* buffer size to be handle by COM20020 */

/** @brief COM20020 Interrupt Masks.
 */
#define	RI_INT		0x80		/* Interrupt RI */
#define	ENAK_INT	0x08		/* Interrupt ExcNak */
#define	REC_INT		0x04		/* Interrupt for reconfiguration */
#define	TA_INT		0x01		/* Interrupt TA */

/** @brief timer base
 */
static unsigned int numerror = 0;
unsigned short configCOM20020(unsigned char pnode);

#ifdef INTERRUPT_TEST
	void intr_test(void);
#endif

static void gpio_cleanup(void);

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file.
 *  @return returns 0 if successful
 */
static int __init erpi_gpio_init(void)
{
   int result = 0;
   int ret = 0;

	printk(KERN_INFO "ARCChar: Initializing the ARCChar LKM\n");

	// Try to dynamically allocate a major number for the device
	majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
	if (majorNumber<0){
	  printk(KERN_ALERT "ARCChar failed to register a major number\n");
	  return majorNumber;
	}
	printk(KERN_INFO "ARCChar: registered correctly with major number %d\n", majorNumber);

	// Register the device class
	arccharClass = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(arccharClass)){           // Check for error and clean up if there is
	  unregister_chrdev(majorNumber, DEVICE_NAME);
	  printk(KERN_ALERT "Failed to register device class\n");
	  return PTR_ERR(arccharClass);     // Correct way to return an error on a pointer
	}
	printk(KERN_INFO "ARCChar: device class registered correctly\n");

	// Register the device driver
	arccharDevice = device_create(arccharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
	if (IS_ERR(arccharDevice)){          // Clean up if there is an error
	  class_destroy(arccharClass);      // Repeated code but the alternative is goto statements
	  unregister_chrdev(majorNumber, DEVICE_NAME);
	  printk(KERN_ALERT "Failed to create the device\n");
	  return PTR_ERR(arccharDevice);
	}
	printk(KERN_INFO "ARCChar: device class created correctly\n"); // Made it! device was initialized
	mutex_init(&arcchar_mutex);          // Initialize the mutex dynamically

	printk(KERN_INFO "GPIO_TEST: Initializing the GPIO_TEST LKM\n");

	//Intr - Interrupt Signal from COM20020
	if (!gpio_is_valid(gpioIntr)){
	  printk(KERN_INFO "GPIO_TEST: invalid gpioIntr\n");
	  return -ENODEV;
	}
	gpio_request(gpioIntr, "sysfs");       // set up gpioIntr
	gpio_direction_input(gpioIntr);        // set up as input
	gpio_export(gpioIntr, false);          // appears in /sys/class/gpio

	printk(KERN_INFO "GPIO_TEST: valor do GPIO_BASE: %#010x\n", GPIO_BASE);

	gpioReg  = (unsigned int *)ioremap(GPIO_BASE, GPIO_LEN);
	if (gpioReg == NULL)
	{
		//error
		printk(KERN_INFO "GPIO_TEST: error on ioremap\n");
		return -ENODEV;
	}

	timerReg  = (unsigned int *)ioremap(TIMER_BASE, TIMER_LEN);
	if (timerReg == NULL)
	{
		//error
		printk(KERN_INFO "GPIO_TEST: error on timer ioremap\n");
		return -ENODEV;
	}

	timer = (volatile unsigned *)timerReg;	//System Timer Control/Status  1Mhz.
    ++timer;    // timer lo 4 bytes System Timer Counter Lower 32 bits
                // timer hi 4 bytes available via *(timer+1)

	printk(KERN_INFO "Compiled on %s\n", buffstrd);

	//Direct gpio access
   ret = configGPIO();

   printk(KERN_INFO "GPIO_TEST: Intr value is currently: %d\n",
          gpio_get_value(gpioIntr));
   irqNumber = gpio_to_irq(gpioIntr);     // map GPIO to IRQ number
   printk(KERN_INFO "GPIO_TEST: gpioIntr mapped to IRQ: %d\n", irqNumber);

   // This next call requests an interrupt line
   result = request_irq(irqNumber,         // interrupt number requested
            (irq_handler_t) erpi_gpio_irq_handler, // handler function
            IRQF_TRIGGER_FALLING,  // on falling, oposite would be     IRQF_TRIGGER_RISING
            "erpi_gpio_handler",  // used in /proc/interrupts
            NULL);                // *dev_id for shared interrupt lines
   printk(KERN_INFO "GPIO_TEST: IRQ request result is: %d\n", result);
   printk(KERN_INFO "GPIO_TEST: HZ %d\n", HZ);

   #ifdef INTERRUPT_TEST
   		intr_test();
   		printk(KERN_INFO "ARCChar: INTR Test\n");
   #endif

   if ( (ret != 0) || (result != 0)	)
   {
    	gpio_cleanup();
    	result = -EIO;	/* I/O error */
   }

   return result;
}


/** @brief The GPIO Cleanup Handler function
 *  Release interrupt handler that was attached to the GPIO.
 * @param none
 * @return none
 */
static void gpio_cleanup(void)
{
   mutex_destroy(&arcchar_mutex);                       // destroy the dynamically-allocated mutex
   device_destroy(arccharClass, MKDEV(majorNumber, 0)); // remove the device
   class_unregister(arccharClass);                      // unregister the device class
   class_destroy(arccharClass);                         // remove the device class
   unregister_chrdev(majorNumber, DEVICE_NAME);         // unregister the major number
   printk(KERN_INFO "ARCChar: Goodbye from the LKM!\n");

   printk(KERN_INFO "GPIO_TEST: gpioIntr value is currently: %d\n",
          gpio_get_value(gpioIntr));
   free_irq(irqNumber, NULL);     // free the IRQ number, no *dev_id
   gpio_unexport(gpioIntr);     // unexport the gpioIntr GPIO
   gpio_free(gpioIntr);         // free the gpioIntr GPIO


	if (gpioReg)
	{
		/* release the mapping */
		iounmap(gpioReg);
	}

	if (timerReg)
	{
		/* release the mapping */
		iounmap(timerReg);
	}

   printk(KERN_INFO "GPIO_TEST: Goodbye from the LKM!\n");

}


/** @brief The LKM cleanup function  */
static void __exit erpi_gpio_exit(void)
{
	gpio_cleanup();
}


/** @brief The GPIO IRQ Handler function
 * A custom interrupt handler that is attached to the GPIO. The same
 * interrupt handler cannot be invoked concurrently as the line is
 * masked out until the function is complete. This function is static
 * as it should not be invoked directly from outside of this file.
 * @param irq    the IRQ number associated with the GPIO
 * @param dev_id the *dev_id that is provided - used to identify
 * which device caused the interrupt. Not used here.
 * @param regs   h/w specific register values -used for debugging.
 * return returns IRQ_HANDLED if successful - return IRQ_NONE otherwise.
 */
static irq_handler_t erpi_gpio_irq_handler(unsigned int irq,
                        void *dev_id, struct pt_regs *regs)
{
	if (atomic_read(&intr) == 0)
	{
		atomic_set(&intr, 1);
	}

	atomic_set(&data_avail_to_read, 1);
	wake_up(&read_queue);

   return (irq_handler_t) IRQ_HANDLED; // announce IRQ handled
}


/** @brief Read Data Bus
*/
#define GPIOOE				GPIO12
#define GPIODIR				GPIO07
#define	GPIORST				GPIO04
#define	GPIOFORREAD			GPIO05
#define	GPIOLED				GPIO08
#define	GPIOFORCS			GPIO13
#define PI_BITn(io)  (1<<(io&0x1F))
#define	RST_RD_WR	(0x7<<(GPIORST&0x1F))
#define	CS			PI_BITn(GPIOFORCS)
#define	DIR			PI_BITn(GPIODIR)
#define	OE			PI_BITn(GPIOOE)
#define	INTR		PI_BITn(GPIOINTR)
#define	DATABUS			(0xFF<<(GPIO20&0x1F))
#define	SETDATABUS(a)	((a&0xFF)<<(GPIO20&0x1F))
#define	CLRDATABUS(a)	((~a)&DATABUS)
#define		REG		(GPIO20/10)



/** @brief Function to read REG GPIO20
 *         Set Databus as Input
 *  @param none
 *  @return	none
 */
void gpioSetReadData(void)	//as input
{
   //int reg;

   gpioReg[REG] = (gpioReg[REG] & 0xff000000 );	//bits 0 a 23  FSEL27
}


/** @brief Function to write REG GPIO20
 *         Set Databus as Output
 *  @param none
 *  @return	none
 */
#define	OUTPUT_MODEFSEL27	0x249249		/* 001001001001001001001001 = 0x249249  output mode 001 */

void gpioSetWriteData(void)	//as output
{
   gpioReg[REG] = ( (gpioReg[REG] & 0xff000000) | OUTPUT_MODEFSEL27 );	//bits 0 a 23  FSEL27
}

/** @brief Inline Delay
 *  @param none
 *  @return	none
 */
__inline__ void fdelay(void)
{
	{
		gpioWrite(GPIOLED, 0);
		gpioWrite(GPIOLED, 1);
		gpioWrite(GPIOLED, 0);
		gpioWrite(GPIOLED, 1);
		gpioWrite(GPIOLED, 0);
		gpioWrite(GPIOLED, 1);
	}
	return;
}

/** @brief Write byte COM20020
 * @param a0a1a2  three bit	address if COM20020
 * @param data byte to be write
 * @return none
 */
void writeDatabus(unsigned int a0a1a2, unsigned char data)
{
	volatile unsigned int dt;
	volatile unsigned int addr, notaddr;

	//Set databus as output. databus as output
	gpioSetWriteData();

	//Set also RSTIN, RD, WR, CS, high
	//Set A1,A1,A2 accordingly.  a0a1a2
	addr = (a0a1a2<<(GPIO17&0x1F)) | DIR | RST_RD_WR | CS;
	notaddr = (((~a0a1a2)&0x7)<<(GPIO17&0x1F)) | OE ;

	spin_lock_irqsave(&mr_lock, flags);
	{
		// /OE implicit=0
		// Signals @ High level = RSTIN, RD, WR, CS, A0,A1,A2
		gpioSetBank1( /*dt1=*/((dt=SETDATABUS(data)) | addr) );
		gpioClearBank1( /*dt2=*/(CLRDATABUS(dt) | notaddr) );

		/* now write (GPIO06) and cs signals(GPIO13) low */
		gpioWrite(GPIOFORCS, 0);		//CS Setup to WR Active=5nS
		gpioWrite(GPIOFORWRITE, 0);		//aciona em separado para dar pequeno delay.

		fdelay();

		gpioSetBank1( RST_RD_WR | CS);

	}
	spin_unlock_irqrestore(&mr_lock, flags);
	gpioWrite(GPIOOE, 1);
}

/** @brief Read byte from COM20020 Databus
 * @param a0a1a2  three bit	address if COM20020
 * @return byte read
 */
unsigned char readDatabus(unsigned int a0a1a2)
{
	unsigned char data;
	unsigned int addr;

	//Set databus as input.
	gpioSetReadData();

	//Set also RSTIN, RD, WR, CS, high
	//Set A1,A1,A2 accordingly.  a0a1a2
	addr = (a0a1a2<<(GPIO17&0x1F));

	// /OE implicit=0
	// Signals @ High level = RSTIN, RD, WR, CS, A0,A1,A2
	gpioSetBank1( addr | RST_RD_WR | CS);
	addr = (((~a0a1a2)&0x7)<<(GPIO17&0x1F));
	gpioClearBank1( addr | OE | DIR);

	/* 	OE	DIR 	An		Bn
		L 	L 		An=Bn	AD0~AD7 COM20020C	read  COM20020
		L 	H 		GPBIO 	Bn=An				write COM20020
		H 	X 		Z 		Z					high impedance */

	spin_lock_irqsave(&mr_lock, flags);
	/* now write (GPIO06) and cs signals(GPIO13) low */
	gpioWrite(GPIOFORCS, 0);		//CS Setup to RD Active=-5nS
	gpioWrite(GPIOFORREAD, 0);		//small delay using separate read
	//delay
	fdelay();

	data = ( gpioReadBank1() >> (GPIO20&0x1F) ) & 0xff;
	gpioSetBank1( RST_RD_WR | CS | OE);
	spin_unlock_irqrestore(&mr_lock, flags);

	return(data);
}

void readRev(void);

/** @brief Config GPIO for COM20020 access
 * @param none
 * @return 0 if success
 * @return number of errors
 */
int configGPIO(void)
{
	int igpio;
	volatile unsigned int a = 0;
	volatile unsigned int b,i;
	unsigned char ret;

	//Set databus as input.
	gpioSetReadData();

	for (igpio = GPIO20; igpio <= GPIO27; igpio++)
	{
		gpioSetPullUpDown(igpio, PI_PUD_UP);
	}

	//just for test delay
	gpioSetMode(GPIOLED,  PI_OUTPUT);

	//Set OE, DIR,
	gpioSetMode(GPIOOE,  PI_OUTPUT);
	gpioSetMode(GPIODIR, PI_OUTPUT);

	//Set A1,A1,A2 output
	gpioSetMode(GPIO17, PI_OUTPUT);
	gpioSetMode(GPIO18, PI_OUTPUT);
	gpioSetMode(GPIO19, PI_OUTPUT);

	gpioSetMode(GPIORST, 	  PI_OUTPUT);
	gpioSetMode(GPIOFORREAD,  PI_OUTPUT);
	gpioSetMode(GPIOFORWRITE, PI_OUTPUT);
	//gpioSetPullUpDown(GPIOFORWRITE, PI_PUD_UP);
	gpioSetMode(GPIOFORCS,    PI_OUTPUT);

	//Set INTR as input. Set using gpio_request & gpio_direction_input(GPIOINTR)
	//gpioSetMode(GPIOINTR, PI_INPUT);
	gpioSetPullUpDown(GPIOINTR, PI_PUD_UP);

	gpioSetBank1( RST_RD_WR | CS | OE);

	//apply reset to COM20020 minimum 250nsec => 300nsec
	spin_lock_irqsave(&mr_lock, flags);
	gpioWrite(GPIORST, 0);
	//delay
	for (i=0 ; i < 0x06; i++)
	{
		b = i + 1;
		a += b*b + i;
	}
	gpioWrite(GPIORST, 1);
	spin_unlock_irqrestore(&mr_lock, flags);

	//check com20020 revision
	readRev();

	/* CONFIG COM20020 */
	ret = configCOM20020(node);

	if (ret != 0)
		numerror++;

	printk(KERN_INFO "ARCChar: Diag COM20020 err=%x\n", ret);

	return(ret);
}

/** @brief Read COM20020 revision
 *         Log COM20020 revision
 * @param none
 * @return none
 */
void readRev(void)
{
	volatile unsigned char rev;
	#define	WRT_RESET_CONTROLLER	0x98
	#define	WRT_COM_SUBADR			0X02
	#define	RD_COM20020_REVB		0x98
	#define	WRT_COM20020_REV		0x80

	//Revision Identification of COM20020
	//WRITE 0X98 REG-6 ADD=6
	writeDatabus(COM_CONF, WRT_RESET_CONTROLLER);		/* reset  controller */

	mdelay(16);		//delay miliseconds not interruptible.

	writeDatabus(COM_SUBADR, WRT_COM_SUBADR);

	mdelay(32);

	//read COM20020 revision
	rev = readDatabus(COM_CONF);

	if (rev == RD_COM20020_REVB)
	{
		printk(KERN_INFO "ARCChar: COM20020 rev B or earlier 0x98 read value=%x\n", rev);
	}
	else
	{
		printk(KERN_INFO "ARCChar: COM20020 first step result=%x\n", rev);

		writeDatabus(COM_SUBADR, 0x80);
		rev = readDatabus(COM_SUBADR);
		if (rev == 0)
		{
			printk(KERN_INFO "ARCChar: COM20020 rev C result=%x\n", rev);
		}
		else if (rev == WRT_COM20020_REV)
		{
			printk(KERN_INFO "ARCChar: COM20020 rev D result OK=%x\n", rev);
		}
		else
		{
			printk(KERN_INFO "ARCChar: COM20020 rev not identified result=%x\n", rev);
		}
	}
}

/** @brief COM20020 Configuracao
 *			static variables
 */
unsigned char	comok = 0;	/* default ‚ controller not OK */
#define	CON_OK		00	/* normal operation */
#define CON_ERR1	01	/* station address = FF */
#define	CON_ERR2	02	/* controller does not reset */
#define	CON_ERR3	03	/* does not exist */
#define	CON_ERR4	04	/* error on DPRAM test */

/* Access control to the DPRAM COM20020 */
	/* select read operation DPRAM COM20020 */
	#define	RD_DATA		0x80
	/* selet write operation DPRAM COM20020 */
	#define	WR_DATA		0x00
	/* select auto increment to access DPRAM COM20020 */
	#define	AUTO_INC	0x40

/** @brief COM20020 configuration
 * @param pnode  node number
 * @return 0 if success
 * @return error number
 */
unsigned short configCOM20020(unsigned char pnode)
{
unsigned int	ramptr;			/* position tested DPRAM */
unsigned char 	ramerr;			/* error flag DPRAM */
unsigned int	ident;			/* type of bus access */
unsigned char	aux1;			/* auxiliary variable  */
unsigned char	my_id;

#define	SEL_NODE_ID_REG		0x19
#define	DESEL_NODE_ID_REG	0x18
#define	CMD_RESET_CONTROLLER	0x98
#define	BYTE_RIGHT_CHECK	0xd1
#define	NO_TOKEN_REQUIRED	0x38

comok = 0;

/* write to odd address in order to identify kind of bus */
ident = 0;

node = pnode;

/* (@2) */
my_id = ~node;			/* node address */

/* check node address */
if (my_id == 00)
	return (CON_ERR1);

/* sel NODE ID REG */
writeDatabus(COM_CONF, SEL_NODE_ID_REG);

/* program station address  */
writeDatabus(COM_SUB, my_id);

/* unselect NODE ID REG */
writeDatabus(COM_CONF, DESEL_NODE_ID_REG);

/* reset  controller */
writeDatabus(COM_CONF, CMD_RESET_CONTROLLER);

mdelay(16);		//not interruptible.

writeDatabus(COM_CONF, DESEL_NODE_ID_REG);

mdelay(32);

/* read first byte buffer controller  */
aux1 = rbyte (0000);

printk(KERN_INFO "ARCChar: COM20020 1o.byte (0xd1)=%x\n", aux1);

/* se diferente de "D1" retorna erro */
if (aux1 != BYTE_RIGHT_CHECK)
		return (CON_ERR2);

/* read second byte buffer  controller */
aux1 = rbyte (0001);

printk(KERN_INFO "ARCChar: COM20020 2o.byte my_id=%x\n", aux1);

/* if not as programmed */
if (aux1 != my_id)
{
	return (CON_ERR2);
}

/* if error */
if ((readDatabus(COM_DIAG) & 0x40) != 0)
	return (CON_ERR3);


/* enable no token  */
writeDatabus(COM_CONF, NO_TOKEN_REQUIRED);

/* inicia teste da DPRAM */
ramptr = 0000;			/* posicao testada */
ramerr = 0;			/* flag de erro no teste */

	do
	{
		aux1 = (unsigned char)(ramptr & 0xff);

		wbyte (ramptr, aux1);	/* write to DPRAM */

		if (aux1 != rbyte(ramptr))
			ramerr = 0xff;	/* error */

		ramptr++;		/* increment test position */
	}
	while ((ramerr == 0) && (ramptr < 0x0800));
	/* end test DPRAM */

	printk(KERN_INFO "ARCChar: COM20020 DPRAM test result=%x numerror=%d\n", ramerr, numerror);

/* if error on DPRAM test */
if (ramerr != 0)
{
	return (CON_ERR4);
}

return (CON_OK);
}

#ifdef	INTERRUPT_TEST

void intr_test(void)
{
	unsigned char mask_copy;
	unsigned char com_status_aux;

writeDatabus(COM_CMD, DIS_TX);

mask_copy = (ENAK_INT | TA_INT);
writeDatabus(COM_IMASK, mask_copy);

com_status_aux = readDatabus(COM_STATUS);

printk(KERN_INFO "ARCChar: intr_test com_status_aux=%x\n", com_status_aux);

}
#endif


/** @brief read one byte of DPRAM COM20020
 * @param add  address
 * @return byte read
 */
unsigned char rbyte(unsigned int add)
{
unsigned char rdata;		/* dado lido do COM20020 */

writeDatabus(COM_PTRH, ((unsigned char)(add >> 8) | (RD_DATA)));

writeDatabus(COM_PTRL, (unsigned char)(add & 0xff));

rdata = readDatabus(COM_DATA);

return (rdata);
}


/** @brief read  "nbytes" bytes of DPRAM and write them to the "dest" system memory
 * @param src  address of DPRAM
 * @param pointer to the destination address
 * @paran nbytes number of bytes
 * @return none
 */
void rblock (uint src, unsigned char *dest, uint nbytes)
{
uint  i;		/* count number of transfered bytes */

	writeDatabus(COM_PTRH, ((unsigned char)(src >> 8) | (RD_DATA) | AUTO_INC));

	writeDatabus(COM_PTRL, (unsigned char)(src & 0xff));

	for (i = 0; i < nbytes; i++)
	{
		dest[i] = readDatabus(COM_DATA);
	}
}

/** @brief write byte to DPRAM
 * @param add  address of DPRAM
 * @param byte to be written
 * @return none
 */
void wbyte(unsigned int add, unsigned char wdata)
{
	writeDatabus(COM_PTRH, (unsigned char)(add >> 8) | (WR_DATA));

	writeDatabus(COM_PTRL, (unsigned char)(add & 0xff));

	writeDatabus(COM_DATA, wdata);
}


/** @brief write block of bytes to DPRAM
 * @param src pointer to the system memory position
 * @param dest address of DPRAM
 * @param nbytes number of bytes to be written.
 * @return none
 */
void wblock (unsigned char *src, uint dest, uint nbytes)
{

uint  i;		/* count number of bytes transfered */

	writeDatabus(COM_PTRH, ((unsigned char)(dest >> 8) | (WR_DATA) | AUTO_INC));

	writeDatabus(COM_PTRL, (unsigned char)(dest & 0xff));

	for ( i = 0; i < nbytes; i++ )
	{
		writeDatabus(COM_DATA, src[i]);
	}
}


/** @brief reads block of bytes from DPRAM
 * @param src address of DPRAM
 * @param dest pointer to the system memory position
 * @return number of transfered bytes.
 */
unsigned int rblockri (uint src, unsigned char *dest)
{
uint  count;
unsigned char trcnt;	/* counter bytes to be transfered  */
unsigned char trtot;	/* number of total bytes to be transfered */

	writeDatabus(COM_PTRH, ((unsigned char)(src >> 8) | (RD_DATA) | AUTO_INC));

	writeDatabus(COM_PTRL, (unsigned char)(src & 0xff));

	dest[0] = readDatabus(COM_DATA);
	dest[0] = ~dest[0];
	dest[1] = readDatabus(COM_DATA);
	dest[1] = ~dest[1];

	count = readDatabus(COM_DATA);

	trtot = BUFSIZE - count;

	dest[2] = trtot;

	src += count;

	writeDatabus(COM_PTRH, ((unsigned char)(src >> 8) | (RD_DATA) | AUTO_INC));

	writeDatabus(COM_PTRL, (unsigned char)(src & 0xff));

	trcnt = 0;

	while (trcnt < trtot)
	{
		dest[3 + trcnt] = readDatabus(COM_DATA);
		trcnt++;
	}

	return(trtot+3);
}


/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
 *  identify the initialization function at insertion time and the cleanup function (as
 *  listed above)
 */
module_init(erpi_gpio_init);
module_exit(erpi_gpio_exit);
