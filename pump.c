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

static struct pump_dev *pump_device; /* allocated in pump_init_module */

/*typedef struct
{
	wait_queue_head_t wait;
	int channel;
	int prescale;
} ADC_DEV;
static int ADC_enable = 0;

static ADC_DEV adcdev;
static volatile int ev_adc = 0;*/
static int adc_data;

/*static struct clk *adc_clock;*/

/*
 * Functions
 */
void default_global_values(struct pump_dev *dev)
{
	dev->pump_state.compress_ratio = DEF_COMP_RATIO;
	dev->pump_state.direction = 0;
	dev->pump_state.heart_beat = DEF_HEART_BEAT;
	dev->pump_state.volume = DEF_VOL;
	dev->pump_state.step = 0;
	dev->pump_state.time_push_acc = DEF_ACC;
	dev->pump_state.time_push_avg = DEF_AVG;
	dev->pump_state.time_pull_acc = DEF_ACC;
	dev->pump_state.time_pull_avg = DEF_AVG;
	dev->pump_state.pwm_rate[0] = DEF_PWM;
	dev->pump_state.pwm_rate[1] = DEF_PWM;
}
void Buzzer_Freq_Set(unsigned long freq)
{
	rGPBCON &= ~0xc; //set GPB0 as tout0, pwm output
	rGPBCON |= 0x8;

//	rTCFG0 &= ~0xff;
//	rTCFG0 |= 4; //prescaler = 4+1
	rTCFG1 &= 0xffffff0f;
//	rTCFG1 |= 2<<4; //mux = 1/2
	rTCNTB1 = (PCLK /10) / freq;
	rTCMPB1 = rTCNTB1 >> 1; // 50%
	rTCON &= 0xfffff0ff;
	rTCON |= 0x00000b00; //disable deadzone, auto-reload, inv-off, update TCNTB0&TCMPB0, start timer 0
	rTCON &= 0xfffffdff; //clear manual update bit		//clear manual update bit
}

void Buzzer_Stop(void)
{
	rGPBCON &= ~0xc;			//set GPB1 as (output) input
/*	rGPBCON |= 1;
	rGPBDAT &= ~1;*/
}
/************************自动调整回零********************/

void AD_convert(void)
{
	unsigned long rADCCON_save = ADCCON;
	ADCCON = (1 << 14) | (19 << 6) | (0 << 3); //预分频有效、预分频值19、使用通道0
	ADCCON |= 0x1; //启动AD
	while (ADCCON & 0x1)
		; //check if Enable_start is low
	while (!(ADCCON & 0x8000))
		; //检测AD转换是否结束
	adc_data = ((int) ADCDAT0 & 0x3ff);
	ADCCON = rADCCON_save;
}
void Auto_zero(void)
{
	AD_convert();
	while(adc_data>(Position_zero+4))
	{
		rGPFDAT=M_n;
		Buzzer_Freq_Set(6000);
		AD_convert();
	}
	while(adc_data<(Position_zero-4))
	{
		rGPFDAT=M_p;
		Buzzer_Freq_Set(6000);
		AD_convert();
	}
	Buzzer_Stop();
//	Uart_Printf( "Return to zero!" );
}
/*
 * irq handlers
 */
/*static irqreturn_t timer_handler_2(int irq, void *dev_id)
{
	static int cnt = 0;
	static int cnt1 = 0;
	if (cnt++ == 200)
	{
		cnt = 0;
		cnt1++;
		printk("\rtimer_handler %d", cnt1);
		Buzzer_Freq_Set(2000 + cnt1 * 20);
	}
	return IRQ_HANDLED;
}*/
static irqreturn_t timer_handler_0(int irq, void *dev_id)
{
		unsigned long freq_1 = 0;
		static int cnt = 0;
		rTCON &= ~S3C2410_TCON_T0START;
		printk("\rtimer0_handler %d step %d",cnt,pump_device->pump_state.step);

		if (cnt++ == 25)
		{
			cnt = 0;
			Buzzer_Stop();
			if (pump_device->pump_state.direction == 0)
			{
				rGPFDAT = M_p;
				if (pump_device->pump_state.step == 0)
				{
					freq_1 = pump_device->pump_state.pwm_rate[0];
					pump_device->pump_state.timer_reload = pump_device->pump_state.time_push_acc;
				}
				else if ((pump_device->pump_state.step > 0)
						&& (pump_device->pump_state.step < 4))
				{
					freq_1 = pump_device->pump_state.pwm_rate[0]
							* (pump_device->pump_state.step + 1);
					pump_device->pump_state.timer_reload = pump_device->pump_state.time_push_acc;
				}
				else if (pump_device->pump_state.step == 4)
				{
					freq_1 = pump_device->pump_state.pwm_rate[0] * 5;
					pump_device->pump_state.timer_reload = pump_device->pump_state.time_push_avg;
				}
				else if (pump_device->pump_state.step <= 8)
				{
					freq_1 = pump_device->pump_state.pwm_rate[0]
							* (9 - pump_device->pump_state.step);
					pump_device->pump_state.timer_reload = pump_device->pump_state.time_push_acc;
				}
			}
			else
			{
				rGPFDAT = M_n;
				if (pump_device->pump_state.step == 0)
				{
					freq_1 = pump_device->pump_state.pwm_rate[1];
					pump_device->pump_state.timer_reload = pump_device->pump_state.time_pull_acc;
				}
				else if ((pump_device->pump_state.step > 0)
						&& (pump_device->pump_state.step < 4))
				{
					freq_1 = pump_device->pump_state.pwm_rate[1]
							* (pump_device->pump_state.step + 1);
					pump_device->pump_state.timer_reload = pump_device->pump_state.time_pull_acc;
				}
				else if (pump_device->pump_state.step == 4)
				{
					freq_1 = pump_device->pump_state.pwm_rate[1] * 5;
					pump_device->pump_state.timer_reload = pump_device->pump_state.time_pull_avg;
				}
				else if (pump_device->pump_state.step <= 8)
				{
					freq_1 = pump_device->pump_state.pwm_rate[1]
							* (9 - pump_device->pump_state.step);
					pump_device->pump_state.timer_reload = pump_device->pump_state.time_pull_acc;
				}
			}

/*			if (pump_device->pump_state.step == 9) //对正反脉冲的差值进行补偿阶段
			{
				if (pump_device->pump_state.direction == 1)
				{
					Buzzer_Stop();
					AD_convert();
					if (adc_data >= (Position_zero + 15)) //超出规定的范围就
					{
						freq_1 = 7000;
						rGPFDAT = M_n;
						pump_device->pump_state.timer_reload = 700; //补偿用时0.025s
					}
					else if (adc_data <= (Position_zero - 15))
					{
						freq_1 = 7000;
						rGPFDAT = M_p;
						pump_device->pump_state.timer_reload = 700;
					}
					else if (adc_data > (Position_zero - 20)
							&& adc_data < (Position_zero + 20))
					{
						pump_device->pump_state.timer_reload = 50;
						freq_1 = 0;
					}
				}
				else
				{
					pump_device->pump_state.timer_reload = 50;
					freq_1 = 0;
				}
			}
			else if (pump_device->pump_state.step == 10) //在最原始的起始端进行参数更换，以免导致绝对位置改变
			{
				Buzzer_Stop();
				pump_device->pump_state.step = -1;
				if (pump_device->pump_state.direction == 1)
				{
					pump_device->pump_state.direction = 0;
					pump_device->pump_state.timer_reload = 50;
					freq_1 = 0;
				}
				else
				{
					pump_device->pump_state.direction = 1;
					pump_device->pump_state.timer_reload = 50;
					freq_1 = 0;
				}
			}*/
//			printk("freq_1 %ld, reload %d\n",freq_1,pump_device->pump_state.timer_reload);
			pump_device->pump_state.step++;
			if(pump_device->pump_state.step == 9) //for test
			{
				pump_device->pump_state.step = 0;
				if(pump_device->pump_state.direction == 0)
					pump_device->pump_state.direction = 1;
				else
					pump_device->pump_state.direction = 0;
			}

//			freq_1 = 2500;

			Buzzer_Freq_Set(freq_1);
		}
/*		printk("reload %d\n",pump_device->pump_state.timer_reload);
		printk("rTCON %lx\n", rTCON);*/
		rTCNTB0 = pump_device->pump_state.timer_reload;

		rTCON |= S3C2410_TCON_T0MANUALUPD;

		rTCON &= ~S3C2410_TCON_T0MANUALUPD;

		rTCON |= S3C2410_TCON_T0START;
		return IRQ_HANDLED;
}
/*static irqreturn_t adcdone_int_handler(int irq, void *dev_id)
{
	if (ADC_enable)
	{
		adc_data = ADCDAT0 & 0x3ff;

		ev_adc = 1;
		wake_up_interruptible(&adcdev.wait);
	}

	return IRQ_HANDLED;
}*/
/*
 * Open and close
 */

int pump_open(struct inode *inode, struct file *filp)
{
	int result;
	struct pump_dev *dev; /* device information */

	dev = container_of(inode->i_cdev, struct pump_dev, cdev);
	filp->private_data = dev; /* for other methods */
	printk("pump ready open\n");
	spin_lock(&dev->lock);
	printk("globel_cnt is %d\n", dev->globle_cnt);
	if (dev->globle_cnt)
	{
		spin_unlock(&dev->lock);
		printk("you can open only one device.\n");
		return -EBUSY;
	}
	dev->globle_cnt++;
	spin_unlock(&dev->lock);
	printk("rTCFG0 %lx\n", rTCFG0);
	printk("rTCFG1 %lx\n", rTCFG1);
	printk("rTCON %lx\n", rTCON);
//set timer 2
/*
	rTCNTB2 = (PCLK / 3 / 2 / 200); //压缩的时间
	rTCMPB2 = 0;

	rTCON &= ~0x00F000; //
	rTCON |= S3C2410_TCON_T2MANUALUPD;
	rTCON |= S3C2410_TCON_T2RELOAD; //
	rTCON &= ~S3C2410_TCON_T2MANUALUPD;
*/

//set timer 0
	dev->pump_state.timer_reload =100;
	rTCNTB0 = 100; //压缩的时间
	rTCMPB0 = 0;

	rTCFG0 &= ~0xff;
	rTCFG0 |=4; //pres 5
	rTCFG1 &= ~(0xf);
	rTCFG1 |= 2;//8
	rTCON &= ~0x00000F; //
	rTCON |= S3C2410_TCON_T0MANUALUPD;
	rTCON |= S3C2410_TCON_T0INVERT;
	rTCON &= ~S3C2410_TCON_T0MANUALUPD;
//set adc
/*	init_waitqueue_head(&(adcdev.wait));
	adcdev.channel=2;	//ÉèÖÃADCµÄÍšµÀ
	adcdev.prescale=0xff;*/
//set io
	rGPFCON|=(5<<6);//engine control
//request irq
/*	result = request_irq(IRQ_TIMER2, timer_handler_2, IRQF_DISABLED, "pump",
			NULL);
	if (result < 0)
	{
		printk("irq timer2 request fail\n");
		return result;
	}*/
	result = request_irq(IRQ_TIMER0, timer_handler_0, IRQF_DISABLED, "pump",
			NULL);
	if (result < 0)
	{
		printk("irq timer0 request fail\n");
		return result;
	}
/*	result = request_irq(IRQ_ADC, adcdone_int_handler, IRQF_SHARED, "adc",
			&adcdev);
	if (result < 0)
	{
		printk("irq adc request fail\n");
		return result;
	}*/
	printk("pump opened\n");
	Auto_zero();
	return 0; /* success */
}

int pump_release(struct inode *inode, struct file *filp)
{
	struct pump_dev *dev; /* device information */
	dev = filp->private_data;
	spin_lock(&dev->lock);
	printk("globel_cnt is %d\n", dev->globle_cnt);
	dev->globle_cnt--;
	spin_unlock(&dev->lock);
	rTCON &= ~(S3C2410_TCON_T0START | S3C2410_TCON_T1START
			| S3C2410_TCON_T1START);
	Buzzer_Stop();
/*	free_irq(IRQ_ADC, &adcdev);*/
	free_irq(IRQ_TIMER0, NULL);
/*	free_irq(IRQ_TIMER2, NULL);*/
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
	int err = 0;
//	int retval = 0;
	struct pump_dev *dev; /* device information */
	dev = filp->private_data;
	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != PUMP_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > PUMP_IOC_MAXNR)
		return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *) arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *) arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	switch (cmd)
	{

	case PUMP_RESET:
		break;
	case PUMP_START_T1:
//		rTCON |= S3C2410_TCON_T1START;
//		printk("start timer_1\n");
		break;
	case PUMP_START_T2:
	{
/*		dev->pump_state.time_push = 60/(dev->pump_state.heart_beat*(1+dev->pump_state.compress_ratio));
		dev->pump_state.time_pull = (60/dev->pump_state.heart_beat)-dev->pump_state.time_push;
		dev->pump_state.time_push_acc = 2500*dev->pump_state.time_push;
		dev->pump_state.time_push_avg = 30000 *dev->pump_state.time_push;
		dev->pump_state.time_pull_acc = 2500*dev->pump_state.time_pull;
		dev->pump_state.time_pull_avg = 30000 *dev->pump_state.time_pull;
		dev->pump_state.pwm_rate[0] = dev->pump_state.volume*(1+dev->pump_state.compress_ratio)/4;
		dev->pump_state.pwm_rate[1] = dev->pump_state.volume*15/(60-dev->pump_state.heart_beat*dev->pump_state.time_push);*/
//		Auto_zero();
		rTCON |= S3C2410_TCON_T0START;
		printk("start timer_0\n");
	}
		break;
	case PUMP_STOP_T1:
//		rTCON &= ~S3C2410_TCON_T1START;
//		printk("stop timer_1\n");
		break;
	case PUMP_STOP_T2:
		Buzzer_Stop();
		rTCON &= ~S3C2410_TCON_T0START;
//		Auto_zero();
		printk("stop timer_0\n");
		break;
	default: /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}
//	return retval;
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
	base_addr = ioremap(S3C2410_PA_ADC, 0x40);
	if (base_addr == NULL)
	{
		printk(KERN_ERR "failed to remap AD_register block\n");
		result = -ENOMEM;
		goto fail_map_3;
	}
/*	adc_clock = clk_get(NULL, "adc");
	if (!adc_clock)
	{
		printk("failed to get adc clock source\n");
		goto fail_map_4;
	}
	clk_enable(adc_clock);*/
	default_global_values(pump_device);
	printk("initial driver pump success!\n ");
	printk("major: %d minor: %d\n", pump_major, pump_minor);
	return 0;

/*	fail_map_4: iounmap(base_addr);*/
	fail_map_3: iounmap(base_addr_io);
	fail_map_2: iounmap(base_addr_timer);
	fail_map_1: kfree(pump_device);
	fail: cdev_del(&pump_device->cdev);
	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(dev, 1);
	return result;
}

//static __exit void pump_exit(void)
static void pump_exit(void)
{
	dev_t devno = MKDEV(pump_major, pump_minor);
/*	if (adc_clock)
	{
		clk_disable(adc_clock);
		clk_put(adc_clock);
		adc_clock = NULL;
	}*/
	iounmap(base_addr);
	iounmap(base_addr_io);
	iounmap(base_addr_timer);
	cdev_del(&pump_device->cdev);
	kfree(pump_device);
	unregister_chrdev_region(devno, 1);
	printk("exit driver driver pump!\n");
}

module_init(pump_init);
module_exit(pump_exit);
MODULE_AUTHOR("jokemilk,jokemilk@yahoo.com.cn");
MODULE_LICENSE("Dual BSD/GPL");
