// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>

#define SIM_DETECT_NAME "sim_detect"

MODULE_DESCRIPTION("sim_detect");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qicai.gu <qicai.gu>");
