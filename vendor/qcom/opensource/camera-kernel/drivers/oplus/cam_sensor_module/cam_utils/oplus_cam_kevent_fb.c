// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2022 Oplus. All rights reserved.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/kdev_t.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ktime.h>

#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) ||                                  \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
#include <soc/oplus/dft/kernel_fb.h>
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OLC)
#include <soc/oplus/dft/olc.h>
#endif
#include "oplus_cam_kevent_fb.h"
#include "cam_debug_util.h"

#define CAM_PROC_FB_NAME "cam_kern_fb"

static struct cam_fb_conf g_kernel_fb_conf[] = {
	{ EXCEP_PROBE, "CAM_EXCEP_PROBE", "EXP_HARDWARE", 0 },
	{ EXCEP_CLOCK, "CAM_EXCEP_CLOCK", "EXP_HARDWARE", 0 },
	{ EXCEP_VOLTAGE, "CAM_EXCEP_VOLTAGE", "EXP_HARDWARE", 0 },
	{ EXCEP_GPIO, "CAM_EXCEP_GPIO", "EXP_HARDWARE", 0 },
	{ EXCEP_I2C, "CAM_EXCEP_I2C", "EXP_HARDWARE", 0 },
	{ EXCEP_SOF_TIMEOUT, "CAM_EXCEP_SOF_TIMEOUT", "EXP_HARDWARE", 0 },
	{ EXCEP_CRC, "CAM_EXCEP_CRC", "EXP_HARDWARE", 0 },
	{ EXCEP_ACTUATOR, "CAM_EXCEP_ACTUATOR", "EXP_HARDWARE", 0 },
	{ EXCEP_EEPROM, "CAM_EXCEP_EEPROM", "EXP_HARDWARE", 0 }
};

static ssize_t kernel_fb_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *lo)
{
	char *r_buf;
	char *fb_event_id_pos = NULL;
	char event_id[MAX_ID] = { 0 };
	int idx = 0;
	int len;

	CAM_INFO(CAM_UTIL, "cam kernel_fb_write in");

	r_buf = (char *)kzalloc(MAX_BUF_LEN, GFP_KERNEL);
	if (!r_buf) {
		return count;
	}

	if (copy_from_user(r_buf, buf,
			   MAX_BUF_LEN > count ? count : MAX_BUF_LEN)) {
		goto exit;
	}

	r_buf[MAX_BUF_LEN - 1] = '\0'; /*make sure last bype is eof*/
	len = strlen(r_buf);
	CAM_INFO(CAM_UTIL, "cam kernel_fb_write r_buf=%s, len=%d", r_buf, len);

	if (idx == len) {
		CAM_ERR(CAM_UTIL, "not fount event id");
		goto exit;
	}

	fb_event_id_pos = strstr(r_buf, "FBEventId@@");
	if (fb_event_id_pos) {
		fb_event_id_pos += strlen("FBEventId@@");
		while (idx < len) {
			if (fb_event_id_pos[idx++] == '$') {
				break;
			}
		}
	} else {
		CAM_ERR(CAM_UTIL, "not fount event id");
		goto exit;
	}

	memcpy(event_id, fb_event_id_pos,
	       idx > MAX_ID ? MAX_ID : idx - 1); /* 1: more than one $ */
	CAM_INFO(CAM_UTIL, "kernel_fb_write:event_id = %s, payload=%s",
		 event_id, fb_event_id_pos + idx + 1);

#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) ||                                  \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
	if (oplus_kevent_fb(FB_CAMERA, event_id, fb_event_id_pos + idx + 1)) {
		/* camera excep feedback, 1:more than one $*/
		CAM_ERR(CAM_UTIL, "err: write kevent fb failed!");
	} else {
		CAM_INFO(CAM_UTIL, "write kevent fb event_id=%s", event_id);
	}
#endif
exit:
	kfree(r_buf);
	return count;
}

static ssize_t kernel_fb_read(struct file *file, char __user *buf, size_t count,
			      loff_t *ppos)
{
	return 0;
}

static const struct proc_ops cam_kern_fb_fops = {
	.proc_open = simple_open,
	.proc_read = kernel_fb_read,
	.proc_write = kernel_fb_write,
};

int cam_event_proc_init(void)
{
	struct proc_dir_entry *d_entry = NULL;

	d_entry = proc_create_data(CAM_PROC_FB_NAME, 0664, NULL,
				   &cam_kern_fb_fops, NULL);
	if (!d_entry) {
		CAM_ERR(CAM_UTIL, "failed to create kern_fb node");
		return -ENODEV;
	}
	return 0;
}

void cam_event_proc_exit(void)
{
	remove_proc_entry(CAM_PROC_FB_NAME, NULL);
}

static int find_event_id_index(int excep_id)
{
	int len = sizeof(g_kernel_fb_conf) / sizeof(g_kernel_fb_conf[0]);
	int ret = -1;
	int index = 0;

	for (index = 0; index < len; index++) {
		if (g_kernel_fb_conf[index].excepId == excep_id) {
			return index;
		}
	}
	return ret;
}

const unsigned char *acquire_event_field(int excepId)
{
	int len = sizeof(g_kernel_fb_conf) / sizeof(g_kernel_fb_conf[0]);
	int index = 0;
	for (index = 0; index < len; index++) {
		if (g_kernel_fb_conf[index].excepId == excepId) {
			return g_kernel_fb_conf[index].fb_field;
		}
	}
	return NULL;
}

int cam_olc_raise_exception(int excep_tpye, unsigned char *pay_load)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OLC)
	struct exception_info exp_info = { 0 };
#endif
	int ret = -1;
	struct timespec64 time = {};
	CAM_INFO(CAM_UTIL, " enter, type:%d", excep_tpye);

	if (excep_tpye >= MAX_EXCEP_TYPE) {
		CAM_ERR(CAM_UTIL, " excep_tpye:%d is beyond 0xf", excep_tpye);
		goto free_exp;
	}

	ktime_get_real_ts64(&time);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OLC)
	exp_info.time = time.tv_sec;
	exp_info.exceptionId = CAM_OLC_EXCEPTION_HEADER_ID |
			       QCOM_EXCEPTION_HARDWARE_MODULE | excep_tpye;
	exp_info.exceptionType = EXCEPTION_KERNEL;
	exp_info.level = EXP_LEVEL_CRITICAL;
	exp_info.atomicLogs = LOG_KERNEL | LOG_ANDROID;
	CAM_INFO(
		CAM_UTIL,
		"camera exception:id=0x%x,time=%ld,level=%d,atomicLogs=0x%lx,logParams=%s",
		exp_info.exceptionId, exp_info.time, exp_info.level,
		exp_info.atomicLogs, exp_info.logParams);
	ret = olc_raise_exception(&exp_info);
	if (ret) {
		CAM_ERR(CAM_UTIL, "err:raise fail, ret:%d", ret);
	} else {
		CAM_INFO(CAM_UTIL, "olc raise success");
	}
#endif
	if (pay_load) {
		int index = find_event_id_index(excep_tpye);
		if (index == -1 ||
		    (time.tv_sec - g_kernel_fb_conf[index].record_time <
		     RECORD_KB_EVENT_TIME)) { /* at least 30s between two times fb*/
			CAM_ERR(CAM_UTIL, "not find excep_tpye = %d",
				excep_tpye);
			goto free_exp;
		}

#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) ||                                  \
	defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
		if (oplus_kevent_fb(FB_CAMERA,
				    g_kernel_fb_conf[index].fb_event_id,
				    pay_load)) {
			/* camera excep feedback*/
			CAM_ERR(CAM_UTIL, "err: write kevent fb failed!");
		} else {
			CAM_INFO(CAM_UTIL,
				 "write kevent fb event_id=%s, payload=%s",
				 g_kernel_fb_conf[index].fb_event_id, pay_load);
			g_kernel_fb_conf[index].record_time = time.tv_sec;
		}
#endif
	}
free_exp:
	return ret;
}

