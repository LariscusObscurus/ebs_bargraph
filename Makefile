
# to generate a lst file add the line below to CFLAGS
#-Wa,-ahlms=$(<:.c=.lst)

ifneq ($(KERNELRELEASE),)
#CFLAGS += -march=armv4t -I$(SUBDIRS)/../include
EXTRA_CFLAGS = -march=armv4t -I$(SUBDIRS)/../include

	obj-m := led_driver.o
	adc-objs := led_driver.o
else

KERNELDIR = ../../../../../../linux-2.6.x
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) SUBDIRS=$(PWD) modules

endif

clean:
	rm -f *.o *~ *.ko core *.mod.c *.ko.cmd *.o.cmd .*.cmd Module.symvers -r .tmp_versions
