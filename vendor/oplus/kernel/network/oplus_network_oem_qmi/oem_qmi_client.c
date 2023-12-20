// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/qrtr.h>
#include <linux/net.h>
#include <linux/completion.h>
#include <linux/idr.h>
#include <linux/string.h>
#include <net/sock.h>
#include <linux/soc/qcom/qmi.h>

#define OEM_QMI "oem_qmi"


#define log_fmt(fmt) "[line:%d][module:%s][%s] " fmt

#define OEM_QMI_ERR(a, arg...) \
do { \
	printk(KERN_NOTICE log_fmt(a), __LINE__, OEM_QMI, __func__, ##arg); \
} while (0)

#define OEM_QMI_MSG(a, arg...) \
do { \
	printk(KERN_INFO log_fmt(a), __LINE__, OEM_QMI, __func__, ##arg); \
} while (0)


static int oem_qmi_init(void)
{
	int ret;
	struct platform_device *pdev;

	OEM_QMI_MSG("enter\n");


	pdev = platform_device_alloc("oem_qmi_client", PLATFORM_DEVID_AUTO);

	if (!pdev) {
		ret = -ENOMEM;
		goto err_release_mem;
	}

	ret = platform_device_add(pdev);

	if (ret) {
		goto err_put_device;
	}

	OEM_QMI_MSG("leave\n");

	return 0;


err_put_device:
	platform_device_put(pdev);

err_release_mem:

	return ret;
}

static void oem_qmi_exit(void)
{
	OEM_QMI_MSG("enter\n");
}

module_init(oem_qmi_init);
module_exit(oem_qmi_exit);

MODULE_DESCRIPTION("OEM QMI client driver");
MODULE_LICENSE("GPL v2");
