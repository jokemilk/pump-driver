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
#include <linux/clk.h>
//#include <linux/irq.h>
//registers
#include <plat/regs-timer.h>
#include <plat/regs-adc.h>
#include <mach/regs-irq.h>
#include <mach/gpio.h>
#include <mach/map.h>
#include <mach/irqs.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/mach/time.h>
#include <asm/io.h>
#include <asm/system.h>		/* cli(), *_flags */
#include <asm/uaccess.h>	/* copy_*_user */
#include <linux/sched.h>

#include "pump.h"	/* local definitions */
#define init_MUTEX(sem)	sema_init(sem,1)

/*
 * Our parameters which can be set at load time.
 */

int pump_major = PUMP_MAJOR;
int pump_minor = 0;

typedef struct
{
	wait_queue_head_t wait;
	int channel;
	int prescale;
} ADC_DEV;

static int ADC_enable = 0;

static ADC_DEV adcdev;
static volatile int ev_adc = 0;
static int adc_data;

static struct clk *adc_clock;

static void __iomem *base_addr;

#define ADCCON		(*(volatile unsigned long *)(base_addr + S3C2410_ADCCON))	//ADC control
#define ADCTSC		(*(volatile unsigned long *)(base_addr + S3C2410_ADCTSC))	//ADC touch screen control
#define ADCDLY		(*(volatile unsigned long *)(base_addr + S3C2410_ADCDLY))	//ADC start or Interval Delay
#define ADCDAT0		(*(volatile unsigned long *)(base_addr + S3C2410_ADCDAT0))	//ADC conversion data 0
#define ADCDAT1		(*(volatile unsigned long *)(base_addr + S3C2410_ADCDAT1))	//ADC conversion data 1
#define ADCUPDN		(*(volatile unsigned long *)(base_addr + 0x14))			//Stylus Up/Down interrupt status
#define PRESCALE_DIS		(0 << 14)
#define PRESCALE_EN		(1 << 14)
#define PRSCVL(x)		((x) << 6)
#define ADC_INPUT(x)		((x) << 3)
#define ADC_START		(1 << 0)
#define ADC_ENDCVT		(1 << 15)

#define START_ADC_AIN(ch, prescale) \
	do{ 	ADCCON = PRESCALE_EN | PRSCVL(prescale) | ADC_INPUT((ch)) ; \
		ADCCON |= ADC_START; \
	}while(0)

struct pump_dev *pump_device; /* allocated in pump_init_module */

/*
 * handler
 */
static irqreturn_t adcdone_int_handler(int irq, void *dev_id)
{
	if (ADC_enable)
	{
		adc_data = ADCDAT0 & 0x3ff;

		ev_adc = 1;
		wake_up_interruptible(&adcdev.wait);
	}

	return IRQ_HANDLED;
}
static irqreturn_t timer_handler(int irq, void *dev_id)
{
//	static int cnt = 0;
//	printk("\rtimer_handler %d", cnt++);
	if (down_trylock(&pump_device->ADC_LOCK) == 0)
	{
		ADC_enable = 1;
		START_ADC_AIN(adcdev.channel, adcdev.prescale);
		wait_event_interruptible(adcdev.wait, ev_adc);

		ev_adc = 0;

//			DPRINTK("AIN[%d] = 0x%04x, %d\n", adcdev.channel, adc_data, ((ADCCON & 0x80) ? 1:0));

		pump_device->ad_value = adc_data;
//			sprintf(str,"%5d", adc_data);
//			copy_to_user(buffer, (char *)&adc_data, sizeof(adc_data));

		ADC_enable = 0;
		up(&pump_device->ADC_LOCK);
	}
	else
	{
		pump_device->ad_value = -1;
	}
	return IRQ_HANDLED;
}
/*
 * Open and close
 */
int pump_open(struct inode *inode, struct file *filp)
{
	int result;
	unsigned long tcfg0, tcfg1, tcon;
	unsigned long tcntb0; //
	struct clk *clk_p;
	unsigned long pclk;
	struct pump_dev *dev; /* device information */

	dev = container_of(inode->i_cdev, struct pump_dev, cdev);
	filp->private_data = dev; /* for other methods */

	if (!down_trylock(&dev->lock))
	{

		clk_p = clk_get(NULL, "pclk");
		pclk = clk_get_rate(clk_p);
		tcfg0 = __raw_readl(S3C2410_TCFG0);
		tcfg1 = __raw_readl(S3C2410_TCFG1);
		tcon = __raw_readl(S3C2410_TCON);
		__raw_writel((tcfg0 &= ~0xff) | 1, S3C2410_TCFG0); //prescaler = 1+1
		__raw_writel((tcfg1 &= ~0xf) | 3, S3C2410_TCFG1); //mux = 1/16
		tcntb0 = (pclk / 32) / dev->freq;
		__raw_writel(tcntb0, S3C2410_TCNTB(0));
		__raw_writel(0, S3C2410_TCMPB(0));
		__raw_writel(tcon | S3C2410_TCON_T0MANUALUPD, S3C2410_TCON);
		tcon = __raw_readl(S3C2410_TCON) & ~S3C2410_TCON_T0MANUALUPD;
		__raw_writel(tcon | (S3C2410_TCON_T0START | S3C2410_TCON_T0RELOAD),
				S3C2410_TCON); // also start timer

		result = request_irq(IRQ_TIMER0, timer_handler, IRQF_DISABLED, "pump",
				NULL);
		if (result < 0)
		{
			__raw_writel(tcon & ~S3C2410_TCON_T0START, S3C2410_TCON); // stop timer
			printk("irq request fail\n");
		}
		init_waitqueue_head(&(adcdev.wait));
		adcdev.channel = CHL;
		adcdev.prescale = PRES;

		printk("ADC opened\n");
	}
	else
		return -EBUSY;

	return 0; /* success */
}

int pump_release(struct inode *inode, struct file *filp)
{
	struct pump_dev *dev; /* device information */
	dev = filp->private_data; /* for other methods */
	free_irq(IRQ_TIMER0, NULL);
	up(&dev->lock);
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
		goto fail_req;
		/* Make this more graceful */
	}

	memset(pump_device, 0, sizeof(struct pump_dev));
	init_MUTEX(&pump_device->lock);
	init_MUTEX(&pump_device->ADC_LOCK);
	pump_setup_cdev(pump_device);

	pump_device->freq = def_freq;

	base_addr = ioremap(S3C2410_PA_ADC, 0x20);
	if (base_addr == NULL)
	{
		printk(KERN_ERR "failed to remap register block\n");
		result = -ENOMEM;
		goto fail_map;
	}
	adc_clock = clk_get(NULL, "adc");
	if (!adc_clock)
	{
		printk(KERN_ERR "failed to get adc clock source\n");
		return -ENOENT;
	}
	clk_enable(adc_clock);

//	ADCTSC = 0;

	result = request_irq(IRQ_ADC, adcdone_int_handler, IRQF_SHARED, "pump",
			&adcdev);
	if (result)
		goto fail_ioumap;

	printk("initial driver pump success!\n ");
	return 0;
	fail_ioumap: iounmap(base_addr);
	fail_map: kfree(pump_device);
	fail_req:
	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(dev, 1);
	return result;
}

//static __exit void pump_exit(void)
static void pump_exit(void)
{
	dev_t dev = MKDEV(pump_major, pump_minor);
	kfree(pump_device);
	free_irq(IRQ_ADC, &adcdev);
	iounmap(base_addr);
	if (adc_clock)
	{
		clk_disable(adc_clock);
		clk_put(adc_clock);
		adc_clock = NULL;
	}
	unregister_chrdev_region(dev, 1);
	printk("exit driver driver pump!\n");
}

module_init(pump_init);
module_exit(pump_exit);
MODULE_AUTHOR("jokemilk,jokemilk@yahoo.com.cn");
MODULE_LICENSE("Dual BSD/GPL");

