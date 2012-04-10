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

#include <asm/system.h>		/* cli(), *_flags */
#include <asm/uaccess.h>	/* copy_*_user */

#include "pump.h"	/* local definitions */

//static __init int pump_init(void)
static  int pump_init(void)
{
	printk("start init driver pump!\n");
	printk("init driver pump success!\n ");
	return 0;
}

//static __exit void pump_exit(void)
static void pump_exit(void)
{
	printk("exit driver driver pump!\n");
}

module_init(pump_init);
module_exit(pump_exit);
MODULE_AUTHOR("jokemilk,jokemilk@yahoo.com.cn");
MODULE_LICENSE("Dual BSD/GPL");

