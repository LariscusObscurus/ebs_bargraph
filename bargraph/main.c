#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include "inc/lpc24xx.h"

#define m_reg_read(reg) (*(volatile unsigned long *)(reg))
#define m_reg_write(reg, data) ((*(volatile unsigned long *)(reg)) = (volatile unsigned long)(data))
#define m_reg_set(reg, data) (*(volatile unsigned long *)(reg)) |= (data)
#define m_reg_clear(reg, data) (*(volatile unsigned long *)(reg)) &= ~(data)

#define PID_TRANSACTION 48

#define GET_REG 49
#define READ_ADC 50
#define WAIT_FOR_ADC 51
#define LAST_VALUE 52


static int adc;
static int led_driver;

void set_pins();
void sig_handler(int signal);
void sig_handler_int(int signal);
void next(void);

int main(void) 
{
	set_pins();
	(void) signal(SIGUSR1, sig_handler);
	(void) signal(SIGINT, sig_handler_int);

	adc = open("/dev/lpc2478_adc", O_RDWR, 0);
	if(adc < 0) {
		(void)printf("Konnte adc device nicht öffnen: %i\n", adc);
		perror("adc:");
		return -1;
	}
	(void)printf("adc device erfolgreich geöffnet\n");

	led_driver = open("/dev/led_driver", O_RDWR, 0);
	if(led_driver < 0) {
		(void)printf("Konnte led_driver device nicht öffnen\n");
		goto error;
	}
	(void)printf("led_driver device erfolgreich geöffnet\n");
	ioctl(adc, PID_TRANSACTION, getpid());
	next();

	(void)getchar();

	(void)close(led_driver);
	(void)close(adc);
	return 0;
error:
	(void)close(adc);
	return -1;
}

void set_pins(void)
{
	m_reg_set(SCS, (1<<0));
	m_reg_set(PINSEL1, (1<<18));
}

void sig_handler(int signal) {
	char buf[32];
	static int lit_leds;
	static unsigned long old_data;
	unsigned long data = 0;

	data = ioctl(adc, LAST_VALUE, NULL);
	printf("data 0x%x\n", data);
	printf("stat 0x%x\n", ioctl(adc, GET_REG, AD0STAT));

	if((data > old_data) 
			&& ((data - old_data) >= 1000) 
			&& (lit_leds < 8)) {

		lit_leds++;
		old_data = data;

	} else if((data < old_data) 
			&& (old_data - data >= 1000)
			&& (lit_leds > 0)) {

		lit_leds--;
		old_data = data;
	}

	(void)snprintf(buf,32, "%i", lit_leds);
	(void)write(led_driver, buf, 1);
	next();
}
void sig_handler_int(int signal)
{
	close(adc);
	close(led_driver);
}

void next(void)
{
	char buf[32];
	unsigned int control_register = ioctl(adc, GET_REG, AD0CR);
	control_register &= ~(1<<0);
	control_register |= (1<<2); /* Select AD0.2 */
	control_register |= (1<<24); /* Start now */
	/*control_register |= (1<<16); Burst mode */

	(void)snprintf(buf, 256,"AD0CR 0x%x", control_register);
	(void)write(adc, buf, strlen(buf));
}
