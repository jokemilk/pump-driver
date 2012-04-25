#include <linux/module.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
//registers
#include <mach/map.h>

#include <asm/system.h>		/* cli(), *_flags */
#include <asm/uaccess.h>	/* copy_*_user */

#include <plat/regs-adc.h>
#include <mach/regs-irq.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/mach/time.h>
#include <asm/io.h>

#include <linux/sched.h>

#include "pump.h"	/* local definitions */

#define init_MUTEX(sem)	sema_init(sem,1)

/*
 * Our parameters which can be set at load time.
 */

int pump_major = PUMP_MAJOR;
int pump_minor = 0;

struct pump_dev *pump_device; /* allocated in pump_init_module */
/*
 * Functions
 */
void default_global_values(struct pump_dev *dev)
{
	dev->pump_state.compress_ratio = DEF_COMP_RATIO;
	dev->pump_state.direction = 0;
	dev->pump_state.heart_beat = DEF_HEART_BEAT;
	dev->pump_state.volume = DEF_VOL;
}
void Buzzer_Freq_Set( unsigned long freq )
{
	rGPBCON &= ~3;			//set GPB0 as tout0, pwm output
	rGPBCON |= 2;

	rTCFG0 &= ~0xff;
	rTCFG0 |= 15;			//prescaler = 15+1
	rTCFG1 &= ~0xf;
	rTCFG1 |= 2;			//mux = 1/8
	rTCNTB0 = (PCLK>>7)/freq;
	rTCMPB0 = rTCNTB0>>1;	// 50%
	rTCON &= ~0x1f;
	rTCON |= 0xb;			//disable deadzone, auto-reload, inv-off, update TCNTB0&TCMPB0, start timer 0
	rTCON &= ~2;			//clear manual update bit		//clear manual update bit
}

void Buzzer_Stop( void )
{
	rGPBCON &= ~3;			//set GPB0 as output
	rGPBCON |= 1;
	rGPBDAT &= ~1;
}

/*
 * irq handlers
 */
static irqreturn_t timer_handler_2(int irq, void *dev_id)
{
		static int cnt = 0;
		static int cnt1 = 0;
		if(cnt++ == 8333)
		{
			cnt = 0;
			cnt1++;
			printk("\rtimer_handler %d",cnt1);
//			Buzzer_Freq_Set(2000+cnt1*20);
		}
		return IRQ_HANDLED;
}
static irqreturn_t timer_handler_1(int irq, void *dev_id)
{

		return IRQ_HANDLED;
}
/*
 * Open and close
 */

int pump_open(struct inode *inode, struct file *filp)
{
	int result;
	struct pump_dev *dev; /* device information */

	dev = container_of(inode->i_cdev, struct pump_dev, cdev);
	filp->private_data = dev; /* for other methods */

	spin_lock(&dev->lock);
	if (dev->globle_cnt)
	{
		spin_unlock(&dev->lock);
		return -EBUSY;
	}
	dev->globle_cnt++;
	spin_unlock(&dev->lock);
//set timer 2
	rTCNTB2 = (PCLK/3/2/200);   //压缩的时间
	rTCMPB2 = 0;

	rTCON &= ~0x00F000;	//
	rTCON |= S3C2410_TCON_T2MANUALUPD;
	rTCON |= S3C2410_TCON_T2RELOAD; //
	rTCON &= ~S3C2410_TCON_T2MANUALUPD;

//set timer 1
	rTCNTB1 = 1;   //压缩的时间
	rTCMPB1 = 0;

	rTCON &= ~0x0000F0;	//
	rTCON |= S3C2410_TCON_T1MANUALUPD;
	rTCON |= S3C2410_TCON_T1INVERT;
	rTCON &= ~S3C2410_TCON_T1MANUALUPD;

//request irq
	result = request_irq(IRQ_TIMER2, timer_handler_2, IRQF_DISABLED, "pump",
			NULL);
	if (result < 0)
	{
		printk("irq timer2 request fail\n");
		return result;
	}
	result = request_irq(IRQ_TIMER1, timer_handler_1, IRQF_DISABLED, "pump",
			NULL);
	if (result < 0)
	{
		printk("irq timer1 request fail\n");
		return result;
	}
	return 0; /* success */
}

int pump_release(struct inode *inode, struct file *filp)
{

	return 0;
}
/*
 * Data management: read and write
 */
ssize_t pump_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos)
{
	return 0;
}

ssize_t pump_write(struct file *filp, const char __user *buf, size_t count,
		loff_t *f_pos)
{
	return 0;
}
/*
 * The ioctl() implementation
 */

long pump_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return 0;
}

struct file_operations pump_fops =
		{ .owner = THIS_MODULE, .read = pump_read, .write = pump_write,
				.unlocked_ioctl = pump_ioctl, .open = pump_open, .release =
						pump_release, };

/*
 * Finally, the module stuff
 */
/*
 * Set up the char_dev structure for this device.
 */
static void pump_setup_cdev(struct pump_dev *dev)
{
	int err, devno = MKDEV(pump_major, pump_minor);

	cdev_init(&dev->cdev, &pump_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &pump_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk("Error %d adding pump", err);
}
//static __init int pump_init(void)
static int pump_init(void)
{
	int result;
	dev_t dev = 0;
	printk("start initial driver pump!\n");
	/*
	 * Get a range of minor numbers to work with, asking for a dynamic
	 * major unless directed otherwise at load time.
	 */
	if (pump_major)
	{
		dev = MKDEV(pump_major, pump_minor);
		result = register_chrdev_region(dev, 1, "pump");
	}
	else
	{
		result = alloc_chrdev_region(&dev, pump_minor, 1, "pump");
		pump_major = MAJOR(dev);
	}
	if (result < 0)
	{
		printk("pump: can't get major %d\n", pump_major);
		return result;
	}
	/*
	 * allocate the devices -- we can't have them static, as the number
	 * can be specified at load time
	 */
	pump_device = kmalloc(sizeof(struct pump_dev), GFP_KERNEL);
	if (!pump_device)
	{
		result = -ENOMEM;
		goto fail;
		/* Make this more graceful */
	}

	memset(pump_device, 0, sizeof(struct pump_dev));
	spin_lock_init(&pump_device->lock);

	pump_setup_cdev(pump_device);

	base_addr_timer = ioremap(S3C2410_PA_TIMER, 0x20);
	if (base_addr_timer == NULL)
	{
		printk(KERN_ERR "failed to remap timer_register block\n");
		result = -ENOMEM;
		goto fail_map_1;
	}
	base_addr_io = ioremap(0x56000000, 0x40);
	if (base_addr_io == NULL)
	{
		printk(KERN_ERR "failed to remap io_register block\n");
		result = -ENOMEM;
		goto fail_map_2;
	}
	default_global_values(pump_device);
	printk("initial driver pump success!\n ");
	printk("major: %d minor: %d\n", pump_major, pump_minor);
	return 0;
	fail_map_2: iounmap(base_addr_timer);
	fail_map_1: kfree(pump_device);
	fail:
	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(dev, 1);
	return result;
}

//static __exit void pump_exit(void)
static void pump_exit(void)
{
	dev_t devno = MKDEV(pump_major, pump_minor);
	iounmap(base_addr_io);
	iounmap(base_addr_timer);
	kfree(pump_device);
	unregister_chrdev_region(devno, 1);
	printk("exit driver driver pump!\n");
}

module_init(pump_init);
module_exit(pump_exit);
MODULE_AUTHOR("jokemilk,jokemilk@yahoo.com.cn");
MODULE_LICENSE("Dual BSD/GPL");
