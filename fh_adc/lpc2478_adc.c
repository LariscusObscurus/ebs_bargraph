/*
   This module implements a driver for LPC2468 lpc2478_adc peripheral.

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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/fs.h> 		/*Needed by char devices*/
#include <linux/module.h> 
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/cdev.h>		/*Char device structures and functions*/
#include <asm/uaccess.h> 	/*Interaction with user space*/


/*Used for Timers and to get access to the jiffies variable*/
#include <linux/time.h>
#include <linux/timer.h>

/* Board specific definitions have to be added to include directory of linux kernel*/
#include <asm/hardware/lpc24xx.h>

#include <generated/utsrelease.h> /*defines UTS_RELEASE macro*/

#include <linux/signal.h>
#include <linux/sched.h>

/******************************************************************************
 * Typedefs and defines
 *****************************************************************************/

#define DEVICE_NAME		"lpc2478_adc"

#define SYSCLK			(48000000) 

#define ADC0_IRQ		18

#define MAX_REGNAME_LENGTH	9
#define MAX_REG_ENTRIES		16

/* ioctl */
#define PID_TRANSACTION		48
#define GET_REG	       		49
#define READ_ADC       		50
#define WAIT_FOR_ADC   		51
#define LAST_VALUE     		52

#define DEBUG			0

/* Register bit set, bit clear read and write macros */
#define m_reg_read(reg) (*(volatile unsigned long *)(reg))
#define m_reg_write(reg, data) ((*(volatile unsigned long *)(reg)) = (volatile unsigned long)(data))
#define m_reg_bfs(reg, data) (*(volatile unsigned long *)(reg)) |= (data)
#define m_reg_bfc(reg, data) (*(volatile unsigned long *)(reg)) &= ~(data)

/******************************************************************************
 * Driver specific function prototypes
 *****************************************************************************/

static int lpc2478_adc_open(struct inode* inode, 
		struct file* file);
static int lpc2478_adc_close(struct inode* inode, 
		struct file* file);

ssize_t lpc2478_adc_read(struct file *p_file, 
		char __user *p_buf, 
		size_t count, 
		loff_t *p_pos);

ssize_t lpc2478_adc_write(struct file *p_file, 
		const char __user *p_buf, 
		size_t count, 
		loff_t *p_pos);

static long lpc2478_adc_ioctl (struct file *file, unsigned int cmd, unsigned long arg);

/******************************************************************************
 * Local variables and function prototypes
 *****************************************************************************/

struct reg_map {
	char str[MAX_REGNAME_LENGTH];
	unsigned long reg;
};

static struct reg_map reg_map_ar[MAX_REG_ENTRIES] = { 
	{"PCLKSEL0",PCLKSEL0},
	{"AD0CR",AD0CR},
	{"AD0GDR",AD0GDR},
	{"AD0INTEN",AD0INTEN},
	{"AD0DR0",AD0DR0},
	{"AD0DR1",AD0DR1},
	{"AD0DR2",AD0DR2},
	{"AD0DR2",AD0DR2},
	{"AD0DR3",AD0DR3},
	{"AD0DR4",AD0DR4},
	{"AD0DR5",AD0DR5},
	{"AD0DR6",AD0DR6},
	{"AD0DR7",AD0DR7},
	{"AD0STAT",AD0STAT},
	{"PCON",PCON},
	{"PCONP",PCONP}
};

struct lpc2478_adc {
	u8 inuse;
	u8 eof;
	struct cdev cdev;
};

static struct lpc2478_adc lpc2478_adc_dev;

struct file_operations lpc2478_adc_fops = {
	.owner   = THIS_MODULE,
	.read    = lpc2478_adc_read,
	.write   = lpc2478_adc_write,
	.open    = lpc2478_adc_open,
	.release = lpc2478_adc_close,
	.unlocked_ioctl	 = lpc2478_adc_ioctl,
};

/* Variables */
static dev_t devno;
static int adc_major;

static pid_t user_pid;
static spinlock_t lock;
struct siginfo info;
struct task_struct *tsk;

/* last read value from AD0GDR */
static unsigned long last_value;

/******************************************************************************
 * Local functions
 *****************************************************************************/
static int lpc2478_adc_setup(struct lpc2478_adc *dev) {

	int err;

	struct cdev *cdev = &dev->cdev;

	cdev_init(cdev, &lpc2478_adc_fops);
	cdev->owner = THIS_MODULE;
	cdev->ops = &lpc2478_adc_fops;
	err = cdev_add(cdev,devno,1);
	if(err){
		printk(KERN_NOTICE "Error %d adding lpc2478_adc", err);
		return -1;
	}

	spin_lock_init(&lock);

	dev->eof = 0;
	dev->inuse = 0;
	dev->cdev = *cdev;

	return 0;
}

/******************************************************************************
 * Exponentiate
 *****************************************************************************/
static int n_pow(int base, int expo)
{
	int i = 1;
	int val = base;

	if(expo == 0)
		return 1;
	while(i<expo)
	{
		val=val*base;
		i++;
	}
	return val;
}

/******************************************************************************
 * Convert char to int
 *****************************************************************************/
static u32 artoi(char *buf,u8 *error) {

	char *tmp = buf;
	u32 val = 0;
	u8 size = 0;
	char c;
	u32 power;

	/*get size of array*/
	while(*tmp) {
		*tmp++; 
		size++;
	}

	tmp = buf;

	/*convert*/
	for(;size > 0;size--)
	{
		c = *tmp;
		power = n_pow(16,size-1);
		if(c >= '0' && c <= '9')
			val += (c - '0')*power;
		else if(c >= 'A' && c <= 'F')
			val += (c - 'A' + 10)*power;
		else if(c >= 'a' && c <= 'f')
			val += (c - 'a' + 10)*power;
		else
		{
			printk("errorchar:%c:\n",c);
			*error = 1;
			return 0;
		}
		tmp++;
	}

	*error = 0;
	return val;
}

/******************************************************************************
 * Case sensitive compare of two strings 
 *****************************************************************************/
static u8 cmp(char *str1, char *str2) {

	char *tmp1 = str1;
	char *tmp2 = str2;

	while(*tmp1)
	{
		if(*tmp1++ != *tmp2++)
			return 1;

	}
	//printk("cmp return 0\n");
	return 0;
}

/******************************************************************************
 * Identify given Registers and calculate the offset and return the Registersaddress
 *****************************************************************************/
static unsigned long get_reg(char *buf,u8 *offset) {

	char *tmp = buf;
	char tok[MAX_REGNAME_LENGTH+1];
	u8 i;
	u8 j;

	i = 0;
	while(*tmp != ' ')
	{
		if(*tmp == '\0')
			return 0;
		tok[i] = *tmp;
		i++; tmp++;
		if(i == MAX_REGNAME_LENGTH)
			return 0;
	}
	tok[i] = '\0';
	*offset = i+1;
	//printk("filtered string:%s:\n",tok);

	for(j = 0;j < MAX_REG_ENTRIES;j++)
	{
		//printk("reg_map_ar[%i].str:%s:\n",j,reg_map_ar[j].str);
		if(cmp(reg_map_ar[j].str,tok) == 0)
			return reg_map_ar[j].reg;
	}

	//printk("get_reg return 0:\n");
	return 0;
}

/******************************************************************************
 * Filter the given value for the Register and return the value in int
 *****************************************************************************/
static unsigned long get_val(char *buf,u8 *offset,u8 *error) {

	char *tmp = buf + *offset;
	u32 val;
	u8 err = 0;

	//printk("getval:tmp[0]:%c: tmp[1]:%c:\n",*tmp,*(tmp+1));
	//printk("getval:tmp:%s:\n",tmp);

	if((*tmp != '0') || ( (*(tmp+1) != 'x') && (*(tmp+1) != 'X') ) ) {
		*error = 1;
		return 0;
	}
	tmp = tmp +2;
	//printk("getval:tmp+2:%s:\n",tmp);
	val = artoi(tmp,&err);
	if(err) {
		printk("errorgetval:%i:\n",*error);
		*error = 1;	
		return 0;
	}
	*error = 0;
	return val;
}

/******************************************************************************
 * Device functions
 *****************************************************************************/
static irqreturn_t lpc2478_adc_isr(int irq, void *dev_id)
{
	int retVal;
	retVal = 0;

	/* AD0GDR must be read to reset irq */
	last_value = m_reg_read(AD0GDR);

	kill_proc_info(SIGUSR1, &info, user_pid);
	if(retVal < 0)
	{
		printk("unable to send signal SIGUSER1\n");
		return -1;
	}
	return IRQ_HANDLED;

}
/******************************************************************************
 * Open the device
 *****************************************************************************/
static int lpc2478_adc_open(struct inode* inode, 
		struct file* file) {
	volatile u32 tmp = 0;
	u8 *inuse = &lpc2478_adc_dev.inuse;

	/*if(*inuse)
		return -EBUSY; Does not work*/

	inuse++;
	lpc2478_adc_dev.inuse = *inuse;
	lpc2478_adc_dev.eof = 0;

	m_reg_bfs(PCONP, (1 << 12));

	m_reg_write(AD0CR, 			/**** configure the ADC 0 ***/
			(1 << 0)                             |  //SEL = 1, dummy channel #1
			((LPC24xx_Fpclk / 4500000) - 1) << 8 |  //set clock division factor, so ADC clock is 4.5MHz
			(0 << 16)                            |  //BURST = 0, conversions are SW controlled
			(0 << 17)                            |  //CLKS  = 0, 11 clocks = 10-bit result
			(1 << 21)                            |  //PDN   = 1, ADC is active
			(1 << 24)                            |  //START = 1, start a conversion now
			(0 << 27)                               //EDGE  = 0, not relevant when start=1
		   );

	//short delay and dummy read
	mdelay(10);
	tmp = m_reg_read(AD0GDR);

	return 0;
}

/******************************************************************************
 * Close the device
 *****************************************************************************/
static int lpc2478_adc_close(struct inode* inode, 
		struct file* file) {

	u8 *inuse = &lpc2478_adc_dev.inuse;

	/*should never happen*/
	if(!inuse)
		return -EFAULT;

	lpc2478_adc_dev.eof = 0;
	inuse--;
	lpc2478_adc_dev.inuse = *inuse;

	m_reg_bfc(AD0CR, (1<<21));	// reset PDN (power down mode)
	m_reg_bfc(PCONP, (1<<12));	// reset PCAD (power off)


	return 0;
}

/******************************************************************************
 * Read all the registers related to adc 1 unit
 *****************************************************************************/
ssize_t lpc2478_adc_read(struct file *p_file, 
		char __user *p_buf, 
		size_t count, 
		loff_t *p_pos) {

	char buf[256];
	char *ptr;
	u32 bytes_read;

	if(lpc2478_adc_dev.eof)
		return 0;/*EOF*/

	(void)sprintf(buf,"PCLKSEL0 0x%x\nAD0CR 0x%x\nAD0GDR 0x%x\nAD0STAT 0x%x\nAD0INTEN 0x%x\nAD0DR0 0x%x\nAD0DR1 0x%x\nAD0DR2 0x%x\nAD0DR3 0x%x\nAD0DR4 0x%x\nAD0DR5 0x%x\nAD0DR6 0x%x\nAD0DR7 0x%x\nPCON 0x%x\nPCONP 0x%x\n",
			(u32)m_reg_read(PCLKSEL0),
			(u32)m_reg_read(AD0CR),
			(u32)m_reg_read(AD0GDR),
			(u32)m_reg_read(AD0STAT),
			(u32)m_reg_read(AD0INTEN),
			(u32)m_reg_read(AD0DR0),
			(u32)m_reg_read(AD0DR1),
			(u32)m_reg_read(AD0DR2),
			(u32)m_reg_read(AD0DR3),
			(u32)m_reg_read(AD0DR4),
			(u32)m_reg_read(AD0DR5),
			(u32)m_reg_read(AD0DR6),
			(u32)m_reg_read(AD0DR7),
			(u32)m_reg_read(PCON),
			(u32)m_reg_read(PCONP));
	bytes_read = 0;
	ptr = buf;
	while(*ptr)
	{
		*p_buf++ = *ptr++;
		bytes_read++;
	}
	lpc2478_adc_dev.eof = 1;

	return bytes_read;
}

/******************************************************************************
 * Write to the given adc registers.
 *****************************************************************************/
ssize_t lpc2478_adc_write(struct file *p_file, 
		const char __user *p_buf, 
		size_t count, 
		loff_t *p_pos) {

	char *buf;
	unsigned long reg;
	unsigned long val;

	u32 bytes_written;
	u8 offset;
	u8 error;

	buf = (char *)p_buf;
	bytes_written = strlen(buf);

	/*cut away trailing new line*/
	if(*(buf+bytes_written-1) == '\n')
		*(buf+bytes_written-1) = '\0';

	//printk("\n**buf:%s**\n",buf);

	/*get register address*/
	reg = get_reg(buf,&offset);
	if(reg == 0){
		printk("Unknown register\n");
		return -EFAULT;
	}

	//printk("buf:%s:\n",buf);
	//printk("offset:%i:\n",offset);

	val = get_val(buf,&offset,&error);	//error handeling
	if(error) {
		printk("Usage: \"REGNAME<space>0xVALUE\"\n Where REGNAME is an element from the enumeration adc0_regs");
		return -EFAULT;
	}

	//printk("reg:0x%x:\n",(int)reg);
	//printk("val:0x%x:\n",(int)val);

	m_reg_write(reg,val);

	//printk("in write:");
	//while ((m_reg_read(AD0GDR) & 0x80000000) == 0);

	return bytes_written;
}

/******************************************************************************
 * ioctl - do some special operations
 *****************************************************************************/

static long lpc2478_adc_ioctl (struct file *file, 
			unsigned int cmd,
			unsigned long arg)
{

#if DEBUG
	printk("in ioctl\n");
#endif
	switch(cmd)
	{
	case PID_TRANSACTION:
		memset(&info, 0, sizeof(struct siginfo));
		info.si_signo = SIGUSR1;
		info.si_code = SI_KERNEL;
		info.si_int = 1234;
		/*without interrupt blocking*/
		spin_lock_bh(&lock);
		user_pid = arg;
		spin_unlock_bh(&lock);

		/* get the task, which called this driver (arg = pid */
		tsk = find_task_by_vpid(user_pid);	
		if(tsk == NULL)
		{
			printk("no such pid \n");
			return -ENODEV;
		}	
	/* reads a certain register 
	 * arg must be a pointer to the address of the register 
	 */
	case GET_REG:
		return (u32)m_reg_read(arg);
		break;
	/* waits till the ADC is finished with the conversion */
	case WAIT_FOR_ADC:			
		while ((m_reg_read(AD0GDR) & 0x80000000) == 0);
		break;
	case LAST_VALUE:
		return last_value;
	default:
		printk("no valid arg for ioctl\n");
		break;
	}
#if DEBUG
	printk("ioctl: user_pid: %i \n", user_pid);
#endif
	return 0;

}

/******************************************************************************
 * Called if the module is exited
 *****************************************************************************/
static void __exit lpc2478_adc_mod_exit(void)
{

	struct cdev *cdev = &lpc2478_adc_dev.cdev;

	m_reg_bfc(AD0CR, (1<<21));	// reset PDN (power down mode)
	m_reg_bfc(PCONP, (1<<12));	// reset PCAD (power off)

	devno = MKDEV(adc_major, 0);
	cdev_del(cdev);
	free_irq(ADC0_IRQ, NULL);
	unregister_chrdev_region(devno, 1);

	printk("Removed  device %s successfully.\n",DEVICE_NAME);
}

/******************************************************************************
 * Called to register and initialize the module
 *****************************************************************************/
static int __init lpc2478_adc_mod_init(void){
	int ret;

	ret = alloc_chrdev_region(&devno,0,1,DEVICE_NAME);
	if(ret)
	{
		(void)printk("lpc2478_adc: failed to register char device\n");
		return ret;
	}

	ret = lpc2478_adc_setup(&lpc2478_adc_dev);
	if (ret) {
		(void)printk("lpc2478_adc: Error adding lpc2478_adc\n");
		goto error;
	}

	adc_major = MAJOR(devno);	

	ret = request_irq(ADC0_IRQ, lpc2478_adc_isr, 0, DEVICE_NAME, NULL);
	if(ret) {
		(void)printk("lpc2478_adc: Error could not bind interrupt.\n");
		goto error;
	}

	printk("Registered new device %s with interrupt at source %i.\n",DEVICE_NAME,ADC0_IRQ);
	printk("In case the module is not already loaded:\n");
	printk("Load the driver with: 'insmod /drivers/lpc2478_%s.ko'\n",DEVICE_NAME);
	printk("Call 'mknod /dev/%s c %i 0' to create a device file\n",DEVICE_NAME,adc_major);
	printk("Remove the module with 'rmmod %s' when you are done. \n",DEVICE_NAME);
	printk("Remove the device file with 'rm /dev/%s'. \n",DEVICE_NAME);

	return ret;
error:
	unregister_chrdev_region(devno, 1);
	return ret;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Roman Beneder, Philipp Brejcha, Matthias Wenzl, Leonhardt Schwarz");
MODULE_DESCRIPTION("lpc2478_adc driver");

module_init(lpc2478_adc_mod_init);
module_exit(lpc2478_adc_mod_exit);
