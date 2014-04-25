
# to generate a lst file add the line below to CFLAGS
#-Wa,-ahlms=$(<:.c=.lst)

ifneq ($(KERNELRELEASE),)
#CFLAGS += -march=armv4t -I$(SUBDIRS)/../include
EXTRA_CFLAGS = -march=armv4t -I$(SUBDIRS)/../include

	obj-m := lpc2478_adc.o
	adc-objs := lpc2478_adc.o
else

KERNELDIR = ../../../../../../linux-2.6.x
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) SUBDIRS=$(PWD) modules

endif

clean:
	rm -f *.o *~ *.ko core *.mod.c *.ko.cmd *.o.cmd .*.cmd Module.symvers -r .tmp_versions
