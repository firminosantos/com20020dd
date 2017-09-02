# com20020dd
Linux loadable kernel module (LKM) for interfacing a COM20020 Arcnet Controller

A kernel module for controlling a ARcnet COM20020. Actions like character driver
Character driver for Arcnet COM20020 controller device.
Implement the device interface

for loading:	insmod com20020dd.ko
for status:  lsmod
for remove:  rmmod com20020dd.ko
for viewing: watch -n 1 "dmesg | tail -15"
after boot load: dmesg | grep -iC 3 "com20020"

Requires a GPIO bus to interface to a Arcnet COM20020.

