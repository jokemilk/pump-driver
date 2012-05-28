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

#ifndef _PUMP_COMMAND_H_
#define _PUMP_COMMAND_H_

/*
 * Ioctl definitions
 */

/* Use 'k' as magic number */
#define PUMP_IOC_MAGIC  'j'
/* Please use a different 8-bit number in your code */

#define PUMP_RESET    _IO(PUMP_IOC_MAGIC, 0)



#define PUMP_START_T1 _IO(PUMP_IOC_MAGIC, 1)
#define PUMP_START_T2 _IO(PUMP_IOC_MAGIC, 2)
#define PUMP_STOP_T1 _IO(PUMP_IOC_MAGIC, 3)
#define PUMP_STOP_T2 _IO(PUMP_IOC_MAGIC, 4)
#define PUMP_SET_PUSH_ACC _IOW(PUMP_IOC_MAGIC,  5, uint)
#define PUMP_SET_PUSH_AVG _IOW(PUMP_IOC_MAGIC,  6, uint)
#define PUMP_SET_PULL_ACC _IOW(PUMP_IOC_MAGIC,  7, uint)
#define PUMP_SET_PULL_AVG _IOW(PUMP_IOC_MAGIC,  8, uint)
#define PUMP_SET_PWM0 _IOW(PUMP_IOC_MAGIC,  9, uint)
#define PUMP_SET_PWM1 _IOW(PUMP_IOC_MAGIC,  10, uint)
/*
#define PUMP_IOCTQUANTUM _IO(PUMP_IOC_MAGIC,   3)
#define PUMP_IOCTQSET    _IO(PUMP_IOC_MAGIC,   4)
#define PUMP_IOCGQUANTUM _IOR(PUMP_IOC_MAGIC,  5, int)
#define PUMP_IOCGQSET    _IOR(PUMP_IOC_MAGIC,  6, int)
#define PUMP_IOCQQUANTUM _IO(PUMP_IOC_MAGIC,   7)
#define PUMP_IOCQQSET    _IO(PUMP_IOC_MAGIC,   8)
#define PUMP_IOCXQUANTUM _IOWR(PUMP_IOC_MAGIC, 9, int)
#define PUMP_IOCXQSET    _IOWR(PUMP_IOC_MAGIC,10, int)
#define PUMP_IOCHQUANTUM _IO(PUMP_IOC_MAGIC,  11)
#define PUMP_IOCHQSET    _IO(PUMP_IOC_MAGIC,  12)
*/

#define PUMP_IOC_MAXNR 16

#endif /* _PUMP_H_ */
