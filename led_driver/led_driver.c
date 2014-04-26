/*
   This module implements a driver for LPC2468 led_driver peripheral.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *************                 version 1.0                    ************
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/interrupt.h>

/* char device */
#include <linux/fs.h>
#include <linux/cdev.h>

#include <asm/uaccess.h>

#include <asm/hardware/lpc24xx.h>

#define DEVICE_NAME		"led_driver"
#define LED_DRIVER_IRQ		9

#define BUFSIZE			32

/* Various Register Values */
#define I20CONSET_I2EN		0x00000040
#define I20CONSET_AA		0x00000004
#define I20CONSET_SI		0x00000008
#define I20CONSET_STO		0x00000010
#define I20CONSET_STA		0x00000020

#define I20CONCLR_AA		0x00000004
#define I20CONCLR_SI		0x00000008
#define I20CONCLR_STA		0x00000020
#define I20CONCLR_I2EN		0x00000040

#define I20SCLH_SCLH		0x00000080
#define I20SCLL_SCLL		0x00000080

/* PCA9532 Register */

#define PCA_SLA_W		0xC0
#define PCA_LS2			0x08
#define PCA_LS3			0x09
#define PCA_LS2_AI		0x18
#define PCA_LS3_AI		0x19

#define LED_ON_0		0x00
#define LED_ON_1		0x01
#define LED_ON_2		0x05
#define LED_ON_3		0x15
#define LED_ON_4		0x55

/* Write or read a whole register at once  */
#define m_reg_read(reg) (*(volatile unsigned long *)(reg))
#define m_reg_write(reg, data) ((*(volatile unsigned long *)(reg)) = (volatile unsigned long)(data))
/* Set or clear bits */
#define m_reg_set(reg, data) (*(volatile unsigned long *)(reg)) |= (data)
#define m_reg_clear(reg, data) (*(volatile unsigned long *)(reg)) &= ~(data)
/* Returns 1 if bit at pos is set*/
#define m_reg_check_bit(var, pos)(*(volatile unsigned long *)(var) & (1<<(pos)))

/* Prototypes for fops */
static int led_driver_open(struct inode* inode,
			struct file* file);

static int led_driver_close(struct inode* inode, 
			struct file* file);

ssize_t led_driver_read(struct file *p_file, 
			char __user *p_buf, 
			size_t count, 
			loff_t *p_pos);

ssize_t led_driver_write(struct file *p_file, 
			const char __user *p_buf, 
			size_t count, 
			loff_t *p_pos);

static long led_driver_ioctl (struct file *file, 
			unsigned int cmd, 
			unsigned long arg);

/* Variables */
struct led_driver {
	u8 inuse;
	u8 eof;
	struct cdev cdev;
};

static struct led_driver led_driver_dev;

struct file_operations led_driver_fops = {
	.owner   = THIS_MODULE,
	.read    = led_driver_read,
	.write   = led_driver_write,
	.open    = led_driver_open,
	.release = led_driver_close,
	.unlocked_ioctl	 = led_driver_ioctl,
};

static dev_t devno;
static int major_devno;

static unsigned char i2c_buffer[BUFSIZE];
static u32 i2c_buffer_index;
static u32 i2c_write_length;
static int lit_leds;
/* Functions */

static int led_driver_setup(struct led_driver *dev)
{
	int err;
	struct cdev *cdev = &dev->cdev;

	cdev_init(cdev, &led_driver_fops);
	cdev->owner = THIS_MODULE;
	cdev->ops = &led_driver_fops;
	err = cdev_add(cdev,devno, 1);
	if(err < 0) {
		printk(KERN_NOTICE "Error %d adding led_driver", err);
		return -1;
	}

	dev->eof = 0;
	dev->inuse = 0;
	dev->cdev = *cdev;
	return 0;

}

static irqreturn_t led_driver_isr(int irq, void *dev_id)
{
	unsigned char stat_value = *(volatile unsigned char*)I20STAT;
	switch(stat_value)
	{
	case 0x08:	/* STA issued */
#ifdef DEBUG
		(void)printk("DEBUG: IRQ STA\n");
#endif
		*(volatile unsigned char *)I20DAT = i2c_buffer[0]; /* SLA+W */
		m_reg_write(I20CONCLR, I20CONCLR_STA);
		break;
	case 0x10:	/* repeated STA */
#ifdef DEBUG
		(void)printk("DEBUG: IRQ RESTA\n");
#endif
		m_reg_write(I20CONCLR, I20CONCLR_STA);
		break;
	case 0x18:	/* ACK */
#ifdef DEBUG
		(void)printk("DEBUG: IRQ ACK1\n");
#endif
		*(volatile unsigned char *)I20DAT = i2c_buffer[1 + i2c_buffer_index];
		i2c_buffer_index++;
		break;
	case 0x28:	/* Data transmitted ACK */
#ifdef DEBUG
		(void)printk("DEBUG: IRQ ACK\n");
#endif
		if(i2c_buffer_index != i2c_write_length) {
			*(volatile unsigned char *)I20DAT = i2c_buffer[1 + i2c_buffer_index];
			i2c_buffer_index++;
		} else {
			goto stop;
		}
		break;
	case 0x20:	/* NACK */
	case 0x30:	/* Data transmitted NACK */
#ifdef DEBUG
		(void)printk("DEBUG: IRQ NACK\n");
#endif
		lit_leds = -1;
		/* fallthrough */
	stop:
		m_reg_set(I20CONSET, I20CONSET_STO);
		break;
	default:
#ifdef DEBUG
		(void)printk("DEBUG: stat_value = 0x%x\n", stat_value);
#endif
		break;
	}
	m_reg_write(I20CONCLR, I20CONCLR_SI);
	return IRQ_HANDLED;
}

static int i2c_start(void)
{
	m_reg_set(I20CONSET, I20CONSET_STA);
#ifdef DEBUG
	(void)printk("DEBUG: i2c_start CONSET 0x%x\n", (u32)m_reg_read(I20CONSET));
#endif

	return 0;
}

static int led_driver_open(struct inode* inode,
			struct file* file)
{
	u8 *inuse = &led_driver_dev.inuse;

	if(*inuse) {
		return -EBUSY;
	}

	inuse++;
	led_driver_dev.inuse = *inuse;
	led_driver_dev.eof = 0;

	m_reg_set(PCONP, (1 << 7)); /* Make sure I2C0 is powered */
	/*XXX: set bit 23 & 25 to 0*/
	m_reg_set(PINSEL1, (1<<22)); /* SDA0 */
	m_reg_set(PINSEL1, (1<<24)); /* SCL0 */
	
	/* Reset I20CONSET */
	m_reg_write(I20CONCLR, I20CONCLR_AA 
			| I20CONCLR_SI 
			| I20CONCLR_STA 
			| I20CONCLR_I2EN);

	m_reg_write(I20SCLL, I20SCLL_SCLL);
	m_reg_write(I20SCLH, I20SCLH_SCLH);
	m_reg_set(I20CONSET,  I20CONSET_I2EN);

	return 0;
}


static int led_driver_close(struct inode* inode, 
		struct file* file)
{
	u8 *inuse = &led_driver_dev.inuse;

	led_driver_dev.eof = 0;
	inuse--;
	led_driver_dev.inuse = *inuse;
	return 0;

}

ssize_t led_driver_read(struct file *p_file, 
			char __user *p_buf, 
			size_t count, 
			loff_t *p_pos)
{
	char buf[32];
	u32 bytes_read = 0;

	if(led_driver_dev.eof)
		return 0;/*EOF*/

	(void)snprintf(buf, 32, "%d LEDs are lit\n", lit_leds);

	bytes_read = strlen(buf) + 1;
	bytes_read -= copy_to_user(p_buf, buf, bytes_read);
	led_driver_dev.eof = 1;
	return bytes_read;
}

ssize_t led_driver_write(struct file *p_file, 
			const char __user *p_buf, 
			size_t count, 
			loff_t *p_pos)
{
	char *buf;
	u32 bytes_written;
	u32 val;

	buf = (char*) p_buf;
	bytes_written = strlen(buf);

	if(*(buf + bytes_written-1) == '\n') {
		*(buf + bytes_written-1) = '\0';
	}
	val = buf[0] - '0';

	if((val > 8) || (val < 0)) {
		(void)printk("Error: Value must be between 0 and 8.\n");
		return bytes_written; 
	}
	lit_leds = val;

	i2c_buffer_index = 0;
	i2c_write_length = 3;

	i2c_buffer[0] = PCA_SLA_W;
	i2c_buffer[1] = PCA_LS2_AI;
	switch(val) { /* XXX: Find better solution */
	case 0:
		i2c_buffer[2] = LED_ON_0;
		i2c_buffer[3] = LED_ON_0;
		break;
	case 1:
		i2c_buffer[2] = LED_ON_1;
		i2c_buffer[3] = LED_ON_0;
		break;
	case 2:
		i2c_buffer[2] = LED_ON_2;
		i2c_buffer[3] = LED_ON_0;
		break;
	case 3:
		i2c_buffer[2] = LED_ON_3;
		i2c_buffer[3] = LED_ON_0;
		break;
	case 4:
		i2c_buffer[2] = LED_ON_4;
		i2c_buffer[3] = LED_ON_0;
		break;
	case 5:
		i2c_buffer[2] = LED_ON_4;
		i2c_buffer[3] = LED_ON_1;
		break;
	case 6:
		i2c_buffer[2] = LED_ON_4;
		i2c_buffer[3] = LED_ON_2;
		break;
	case 7:
		i2c_buffer[2] = LED_ON_4;
		i2c_buffer[3] = LED_ON_3;
		break;
	case 8:
		i2c_buffer[2] = LED_ON_4;
		i2c_buffer[3] = LED_ON_4;
		break;
	}

	(void)i2c_start();
	return bytes_written;
}

static long led_driver_ioctl (struct file *file, 
			unsigned int cmd, 
			unsigned long arg)
{
	return 0;
}


static int __init led_driver_mod_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&devno, 0, 1, DEVICE_NAME);
	if(ret < 0) {
		(void)printk("led_driver: failed to register char device\n");
		return ret;
	}

	ret = led_driver_setup(&led_driver_dev);
	if(ret) {
		(void)printk("led_driver: Error adding led_driver\n");
		goto error;
	}
	major_devno = MAJOR(devno);

	ret = request_irq(LED_DRIVER_IRQ, led_driver_isr, 0, DEVICE_NAME, NULL);
	if(ret) {
		(void)printk("led_driver could not bind interrupt");
		goto error;
	}

	(void)printk("Hello LED\n");
	(void)printk("Call 'mknod /dev/%s c %i 0' to create a device file\n",DEVICE_NAME, major_devno);
	return 0;
error:
	unregister_chrdev_region(devno, 1);
	return ret;
}

static void __exit led_driver_mod_exit(void)
{
	struct cdev *cdev = &led_driver_dev.cdev;

	free_irq(LED_DRIVER_IRQ, NULL);
  	devno = MKDEV(major_devno, 0);
  	cdev_del(cdev);
	unregister_chrdev_region(devno, 1);

	(void)printk("Goodbye LED\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Leonhardt Schwarz");
MODULE_DESCRIPTION("PCA9532 LED driver");

module_init(led_driver_mod_init);
module_exit(led_driver_mod_exit);
