// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

#define GPIO_DEVICE_NAME "oplus-network-esim"

#define log_fmt(fmt) "[line:%d][module:%s][%s] " fmt

#define OPLUS_GPIO_ERR(a, arg...) \
do { \
	printk(KERN_NOTICE log_fmt(a), __LINE__, GPIO_DEVICE_NAME, __func__, ##arg); \
} while (0)

#define OPLUS_GPIO_MSG(a, arg...) \
do { \
	printk(KERN_INFO log_fmt(a), __LINE__, GPIO_DEVICE_NAME, __func__, ##arg); \
} while (0)



static int __init oplus_gpio_init(void)
{
	OPLUS_GPIO_MSG("enter\n");

	return 0;
}

static void __exit oplus_gpio_exit(void)
{
	OPLUS_GPIO_MSG("enter\n");
}


module_init(oplus_gpio_init);
module_exit(oplus_gpio_exit);


MODULE_DESCRIPTION("oplus gpio controller");
MODULE_LICENSE("GPL");
MODULE_ALIAS("gpio:oplus-gpio");
