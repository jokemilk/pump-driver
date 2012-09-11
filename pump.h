/*
 * pump.h -- definitions for the char module
 *
 * Copyright (C) 2012 jokemilk
 *
 * The source code in this file can be freely used, adapted,
 * and redistributed in source or binary form, so long as an
 * acknowledgment appears in derived source files.No warranty
 *  is attached; we cannot take responsibility for errors or
 *  fitness for use.
 *
 * $Id: jokemilk.h,v 1.0 2012/4/10 17:51:18
 */

#ifndef _PUMP_H_
#define _PUMP_H_

#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */
#include "pump_command.h"
/*
 * Macros to help debugging
 */

#undef PDEBUG             /* undef it, just in case */
#ifdef PUMP_DEBUG
#  ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG "pump: " fmt, ## args)
#  else
/* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */

#ifndef PUMP_MAJOR
#define PUMP_MAJOR 0   /* dynamic major by default */
#endif


/*
 * Macros to default values
 */
#define def_freq 200
#define CHL	1
#define PRES 9
#define DEF_COMP_RATIO	1
#define DEF_VOL		5000
#define DEF_HEART_BEAT 70
#define PCLK	50000000L
#define Position_zero	266			//运动的原始零点
#define M_p 0x08				//正反转
#define M_n 0x10
#define DEF_ACC 1071
#define DEF_AVG 12857
#define DEF_PWM 2500
/*
 * Macros to ioremap
 */
#define S3C_TIMERREG(x) (x)
#define S3C_TIMERREG2(tmr,reg) S3C_TIMERREG((reg)+0x0c+((tmr)*0x0c))

#define S3C2410_TCFG0	      S3C_TIMERREG(0x00)
#define S3C2410_TCFG1	      S3C_TIMERREG(0x04)
#define S3C2410_TCON	      S3C_TIMERREG(0x08)

#define S3C2410_TCNTB(tmr)    S3C_TIMERREG2(tmr, 0x00)
#define S3C2410_TCMPB(tmr)    S3C_TIMERREG2(tmr, 0x04)
#define S3C2410_TCNTO(tmr)    S3C_TIMERREG2(tmr, (((tmr) == 4) ? 0x04 : 0x08))

#define S3C2410_TCON_T4RELOAD	  (1<<22)
#define S3C2410_TCON_T4MANUALUPD  (1<<21)
#define S3C2410_TCON_T4START	  (1<<20)

#define S3C2410_TCON_T3RELOAD	  (1<<19)
#define S3C2410_TCON_T3INVERT	  (1<<18)
#define S3C2410_TCON_T3MANUALUPD  (1<<17)
#define S3C2410_TCON_T3START	  (1<<16)

#define S3C2410_TCON_T2RELOAD	  (1<<15)
#define S3C2410_TCON_T2INVERT	  (1<<14)
#define S3C2410_TCON_T2MANUALUPD  (1<<13)
#define S3C2410_TCON_T2START	  (1<<12)

#define S3C2410_TCON_T1RELOAD	  (1<<11)
#define S3C2410_TCON_T1INVERT	  (1<<10)
#define S3C2410_TCON_T1MANUALUPD  (1<<9)
#define S3C2410_TCON_T1START	  (1<<8)

#define S3C2410_TCON_T0DEADZONE	  (1<<4)
#define S3C2410_TCON_T0RELOAD	  (1<<3)
#define S3C2410_TCON_T0INVERT	  (1<<2)
#define S3C2410_TCON_T0MANUALUPD  (1<<1)
#define S3C2410_TCON_T0START	  (1<<0)

static void __iomem *base_addr_timer;
#define rTCFG0 (*(volatile unsigned long *)(base_addr_timer + S3C2410_TCFG0))
#define rTCFG1 (*(volatile unsigned long *)(base_addr_timer + S3C2410_TCFG1))
#define rTCON	(*(volatile unsigned long *)(base_addr_timer + S3C2410_TCON))
#define rTCNTB0	(*(volatile unsigned long *)(base_addr_timer + S3C2410_TCNTB(0)))
#define rTCMPB0	(*(volatile unsigned long *)(base_addr_timer + S3C2410_TCMPB(0)))
#define rTCNTO0 (*(volatile unsigned long *)(base_addr_timer + S3C2410_TCNTO(0)))
#define rTCNTB1	(*(volatile unsigned long *)(base_addr_timer + S3C2410_TCNTB(1)))
#define rTCMPB1	(*(volatile unsigned long *)(base_addr_timer + S3C2410_TCMPB(1)))
#define rTCNTO1 (*(volatile unsigned long *)(base_addr_timer + S3C2410_TCNTO(1)))
#define rTCNTB2	(*(volatile unsigned long *)(base_addr_timer + S3C2410_TCNTB(2)))
#define rTCMPB2	(*(volatile unsigned long *)(base_addr_timer + S3C2410_TCMPB(2)))
#define rTCNTO2 (*(volatile unsigned long *)(base_addr_timer + S3C2410_TCNTO(2)))


#define S3C2410_GPBCON	   (0x10)
#define S3C2410_GPBDAT	   (0x14)
#define S3C2410_GPBUP	   (0x18)
#define S3C2410_GPFCON	   (0x50)
#define S3C2410_GPFDAT	   (0x54)
#define S3C2410_GPFUP	   (0x58)

static void __iomem *base_addr_io;
#define rGPBCON    (*(volatile unsigned *)(base_addr_io + S3C2410_GPBCON))	//Port B control
#define rGPBDAT    (*(volatile unsigned *)(base_addr_io + S3C2410_GPBDAT))	//Port B data
#define rGPBUP     (*(volatile unsigned *)(base_addr_io + S3C2410_GPBUP))	//Pull-up control B
#define rGPFCON    (*(volatile unsigned *)(base_addr_io + S3C2410_GPFCON))	//Port B control
#define rGPFDAT    (*(volatile unsigned *)(base_addr_io + S3C2410_GPFDAT))	//Port B data
#define rGPFUP     (*(volatile unsigned *)(base_addr_io + S3C2410_GPFUP))	//Pull-up control B

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
/*
 * data structures
 */
struct PUMP_STATE
{
	float compress_ratio; 	//压缩比
	uint volume;		    //压缩总量
	uint heart_beat;
	uint direction;
	int step;
	uint pwm_rate[2];
	uint stat;
	uint pos;
	uint period;//定时器时间
	uint time_push_acc;
	uint time_push_avg;
	uint time_pull_acc;
	uint time_pull_avg;
	uint timer_reload;
};

struct pump_dev
{
	unsigned long freq;
	int ad_value;
	int globle_cnt; //opened instance number
	struct semaphore ADC_LOCK;
	spinlock_t lock;
	struct semaphore sem; /* mutual exclusion semaphore     */
	struct cdev cdev; /* Char device structure		*/
	struct PUMP_STATE pump_state;
};

/*
 * Split minors in two parts
 */
#define TYPE(minor)	(((minor) >> 4) & 0xf)	/* high nibble */
#define NUM(minor)	((minor) & 0xf)		/* low  nibble */


/*
 * Prototypes for shared functions
 */


ssize_t pump_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos);
ssize_t pump_write(struct file *filp, const char __user *buf, size_t count,
		loff_t *f_pos);
long pump_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);



#endif /* _PUMP_H_ */
