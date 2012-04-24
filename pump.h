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

#define def_freq 200
#define CHL	1
#define PRES 9
#define DEF_COMP_RATIO	1
#define DEF_VOL		5000.0
#define DEF_HEART_BEAT 70

struct PUMP_STATE
{
	float compress_ratio; 	//压缩比
	double volume;		    //压缩总量
	uint heart_beat;
	uint direction;
	int step;
	uint pwm_rate;
	uint stat;
	uint pos;
	uint period;//定时器时间
	uint time_push;
	uint time_pull;
};

struct pump_dev
{
//	struct scull_qset *data; /* Pointer to first quantum set */
//	int quantum; /* the current quantum size */
//	int qset; /* the current array size */
//	unsigned long size; /* amount of data stored here */
//	unsigned int access_key; /* used by sculluid and scullpriv */
	unsigned long freq;
	int ad_value;
	spinlock_t spin;
	spinlock_t ADC_LOCK;
	struct semaphore lock;
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

/*
 * Ioctl definitions
 */

/* Use 'k' as magic number */
#define SCULL_IOC_MAGIC  'k'
/* Please use a different 8-bit number in your code */

#define SCULL_IOCRESET    _IO(SCULL_IOC_MAGIC, 0)

/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": switch G and S atomically
 * H means "sHift": switch T and Q atomically
 */
#define SCULL_IOCSQUANTUM _IOW(SCULL_IOC_MAGIC,  1, int)
#define SCULL_IOCSQSET    _IOW(SCULL_IOC_MAGIC,  2, int)
#define SCULL_IOCTQUANTUM _IO(SCULL_IOC_MAGIC,   3)
#define SCULL_IOCTQSET    _IO(SCULL_IOC_MAGIC,   4)
#define SCULL_IOCGQUANTUM _IOR(SCULL_IOC_MAGIC,  5, int)
#define SCULL_IOCGQSET    _IOR(SCULL_IOC_MAGIC,  6, int)
#define SCULL_IOCQQUANTUM _IO(SCULL_IOC_MAGIC,   7)
#define SCULL_IOCQQSET    _IO(SCULL_IOC_MAGIC,   8)
#define SCULL_IOCXQUANTUM _IOWR(SCULL_IOC_MAGIC, 9, int)
#define SCULL_IOCXQSET    _IOWR(SCULL_IOC_MAGIC,10, int)
#define SCULL_IOCHQUANTUM _IO(SCULL_IOC_MAGIC,  11)
#define SCULL_IOCHQSET    _IO(SCULL_IOC_MAGIC,  12)


#endif /* _PUMP_H_ */
