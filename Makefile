.PHONY: lpc2468mmc ctest fh_adc timer2 gpio led_driver

CFLAGS=
LDFLAGS=

all: lpc2468mmc ctest fh_adc timer2 gpio led_driver

lpc2468mmc:
	make -C lpc2468mmc

ctest:
	make -C ctest

fh_adc:
	make -C fh_adc

timer2:
	make -C timer2

gpio:
	make -C gpio

led_driver:
	make -C led_driver
clean:
	make -C lpc2468mmc clean
	make -C ctest clean
	make -C fh_adc clean
	make -C timer2 clean
	make -C gpio clean
	make -C led_driver clean

