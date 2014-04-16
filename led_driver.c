#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/interrupt.h>

/* char device */
#include <linux/fs.h>
#include <linux/cdev.h>

#include <asm/uaccess.h>

#include <asm/hardware/lpc24xx.h>

#define DEVICE_NAME	"led_driver"
#define LED_DRIVER_IRQ	9 /* According to "/proc/interupts" this is correct*/

#define I20CONSET_I2EN	0x00000040
#define I20CONSET_AA	0x00000004
#define I20CONSET_SI	0x00000008
#define I20CONSET_STO	0x00000010
#define I20CONSET_STA	0x00000020

#define I20CONCLR_AA	0x00000004
#define I20CONCLR_SI	0x00000008
#define I20CONCLR_STA	0x00000020
#define I20CONCLR_I2EN	0x00000040

#define I20SCLH_SCLH	0x00000080
#define I20SCLL_SCLL	0x00000080

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
static int led_driver_open(struct inode* inode,
			struct file* file)
{
	volatile u32 tmp = 0;
	u8 *inuse = &led_driver_dev.inuse;

	if(*inuse)
		return -EBUSY;

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

	m_reg_set(I20SCLL, I20SCLL_SCLL);
	m_reg_set(I20SCLH, I20SCLH_SCLH);


	return 0;
}


static int led_driver_close(struct inode* inode, 
		struct file* file)
{
	return 0;

}

ssize_t led_driver_read(struct file *p_file, 
			char __user *p_buf, 
			size_t count, 
			loff_t *p_pos)
{
	return 0;
}

ssize_t led_driver_write(struct file *p_file, 
			const char __user *p_buf, 
			size_t count, 
			loff_t *p_pos)
{
	return 0;
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
		printk("led_driver: Error adding led_driver\n");
	}
	major_devno = MAJOR(devno);

	(void)printk("Hello LED\n");
	return 0;
}

static void __exit led_driver_mod_exit(void)
{
	struct cdev *cdev = &led_driver_dev.cdev;

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
