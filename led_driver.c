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
#define MAX_TIMEOUT		0x00FFFFFF

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
#define PCA_LS0			0x06
#define PCA_LS1			0x07
#define PCA_LS2			0x08
#define PCA_LS3			0x09

#define LED_ON_1		0x01
#define LED_ON_2		0x05
#define LED_ON_3		0x15
#define LED_ON_4		0x55


/* I2C State */
#define I2C_IDLE		0
#define I2C_STARTED		1
#define I2C_RESTARTED		2
#define I2C_REPEATED_START	3
#define DATA_ACK		4
#define DATA_NACK		5
#define DATA_FIN		6

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

static int i2c_state = I2C_IDLE;
static unsigned char i2c_buffer[BUFSIZE];
static u32 i2c_buffer_index;
static u32 i2c_write_length;
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
	u32 stat_value = m_reg_read(I20STAT);
	(void)printk("DEBUG: IRQ\n");
	switch(stat_value)
	{
	case 0x08:	/* STA issued */
		(void)printk("DEBUG: IRQ STA\n");
		m_reg_write(I20DAT, i2c_buffer[0]); /* SLA+W */
		m_reg_set(I20CONCLR, (I20CONCLR_SI | I20CONCLR_STA));
		i2c_state = I2C_STARTED;
		break;
	case 0x10:	/* repeated STA */
		(void)printk("DEBUG: IRQ RESTA\n");
		m_reg_set(I20CONCLR, (I20CONCLR_SI | I20CONCLR_STA));
		i2c_state = I2C_RESTARTED;
		break;
	case 0x18:	/* ACK */
		(void)printk("DEBUG: IRQ ACK1\n");
		if(i2c_state == I2C_STARTED) {
			m_reg_write(I20DAT, i2c_buffer[1 + i2c_buffer_index]);
			i2c_buffer_index++;
			i2c_state = DATA_ACK;
		}
		m_reg_set(I20CONCLR, I20CONCLR_SI);
		break;
	case 0x20:	/* NACK */
		(void)printk("DEBUG: IRQ NACK\n");
		m_reg_set(I20CONCLR, I20CONCLR_SI);
		i2c_state = DATA_NACK;
		break;
	case 0x28:	/* Data transmitted ACK */
	case 0x30:	/* Data transmitted NACK */
		(void)printk("DEBUG: IRQ ACK & NACK\n");
		if(i2c_buffer_index != i2c_write_length) {
			m_reg_write(I20DAT, i2c_buffer[1 + i2c_buffer_index]);
			i2c_buffer_index++;
			i2c_state = DATA_ACK;
		} else {
			i2c_state = DATA_FIN;
		}
		m_reg_set(I20CONCLR, I20CONCLR_SI);
		break;
	default:
		(void)printk("DEBUG: stat_value = 0x%x\n", stat_value);
		m_reg_set(I20CONCLR, I20CONCLR_SI);
		break;
	}
	return IRQ_HANDLED;
}

static int i2c_start(void)
{
	u32 timeout = 0;
	int ret = -1;

	m_reg_set(I20CONSET, I20CONSET_STA);

	while(1) {
		if(i2c_state == I2C_STARTED) {
			ret = 0;
			break;
		} else if(timeout >= MAX_TIMEOUT) {
			ret = -1;
			break;
		}
		timeout++;
	}
	return ret;
}

static int i2c_stop(void)
{
	m_reg_set(I20CONSET, I20CONSET_STO);
	m_reg_set(I20CONCLR, I20CONCLR_SI);
	return 0;
}

static int i2c_transaction(void)
{
	i2c_state = I2C_IDLE;
	i2c_buffer_index = 0;
	u32 timeout = 0;

	if(i2c_start() < 0) {
		i2c_stop();
		return -1;
	}
	(void)printk("DEBUG: STA ok\n");
	while(1) {
		if((i2c_state == DATA_NACK) || (i2c_state == DATA_FIN)) {
			i2c_stop();
			break;
		} else if(timeout >= MAX_TIMEOUT) {
			(void)printk("ERROR: I2C Transaction timed out\n");
			return -1;
		} else if((i2c_state == DATA_ACK)) {
			timeout = 0;
		}
		timeout++;
	}
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
	m_reg_set(I20CONSET,  I20CONSET_I2EN);

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
	char *buf;
	int ret;
	u32 bytes_written;
	u32 val;

	buf = (char*) p_buf;
	bytes_written = strlen(buf);

	if(*(buf + bytes_written-1) == '\n') {
		*(buf + bytes_written-1) = '\0';
	}
	val = buf[0] - '0';

	if(val >= 8) {
		(void)printk("Error: Value must be smaller than 9.\n");
		return -EFAULT;
	}

	i2c_buffer[0] = PCA_SLA_W;
	i2c_buffer[1] = PCA_LS0;
	i2c_buffer[2] = LED_ON_4;
	i2c_write_length = 2;

	ret = i2c_transaction();
	if(ret < 0) {
		(void)printk("Error: Start condition could not be issued.\n");
		return -EFAULT;
	}
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

	ret = request_irq(LED_DRIVER_IRQ, led_driver_isr, 0, DEVICE_NAME, NULL);
	if(ret) {
		printk("led_driver could not bind interrupt");
		return ret;
	}

	(void)printk("Hello LED\n");
	return 0;
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
