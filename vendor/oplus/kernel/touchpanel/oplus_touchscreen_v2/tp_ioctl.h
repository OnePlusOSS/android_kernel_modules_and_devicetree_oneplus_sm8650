// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */


#ifndef _TP_IOCTL_H_
#define _TP_IOCTL_H_

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/slab.h>

struct tp_ioc_en {
	u8 __user *buf;
	u32 size;
};

struct tp_ioc_buf {
	u8 __user *buf;
	u8 __user *msec; /* s64 ktime_to_ms */
	u8 __user *size;
};

#define HIDL_PROPERTY_READ_BOOL 1
#define HIDL_PROPERTY_READ_U32 2
#define HIDL_PROPERTY_COUNT_U32_ELEMS 3
#define HIDL_PROPERTY_READ_U32_ARRAY 4
#define HIDL_PROPERTY_READ_STRING_INDEX 5
#define HIDL_PROPERTY_COUNT_U8_ELEMS 6
#define HIDL_PROPERTY_READ_U8_ARRAY 7

struct tp_ioc_dts_str {
	u32 type;
	char __user *propname;
	void __user *out_values;
	int32_t __user *ret;
	u32 index;
	u32 name_size;
	u32 size;
};

struct pen_ioc_uplk_msg {
	unsigned int device_id; /* in */
	unsigned int timeout;   /* in */
	unsigned int cmd;       /* out */
	unsigned int len;       /* out */
	unsigned char __user *buf;  /* out */
};

struct pen_ioc_downlk_msg {
	unsigned int device_id; /* in */
	unsigned int cmd;       /* in */
	unsigned int len;       /* in */
	unsigned char __user *buf;   /* in */
};

#define TP_IOC_VERSION  0x01
#define TP_IOC_INIT	(_IOR(TP_IOC_VERSION, 0, struct tp_ioc_en))
#define TP_IOC_EN	(_IOW(TP_IOC_VERSION, 1, u8))
#define TP_IOC_INT	(_IOWR(TP_IOC_VERSION, 2, struct tp_ioc_buf))
#define TP_IOC_REF	(_IOWR(TP_IOC_VERSION, 3, struct tp_ioc_buf))
#define TP_IOC_DIFF	(_IOWR(TP_IOC_VERSION, 4, struct tp_ioc_buf))
#define TP_IOC_RAW	(_IOWR(TP_IOC_VERSION, 5, struct tp_ioc_buf))
#define TP_IOC_POINT	(_IOW(TP_IOC_VERSION, 6, struct tp_ioc_buf))
#define TP_IOC_DTS	(_IOR(TP_IOC_VERSION, 7, struct tp_ioc_dts_str))
#define TP_IOC_BUS	(_IOWR(TP_IOC_VERSION, 8, struct tp_ioc_buf))
#define TP_IOC_GPIO	(_IOWR(TP_IOC_VERSION, 9, struct tp_ioc_buf))

#define PEN_IOC_CMD_UPLK _IOWR(TP_IOC_VERSION, 10, struct pen_ioc_uplk_msg)
#define PEN_IOC_CMD_DOWNLK _IOWR(TP_IOC_VERSION, 11, struct pen_ioc_downlk_msg)


struct touch_point_report {
	uint8_t id;
	uint8_t report_bit;
	uint16_t x;
	uint16_t y;
	uint8_t press;
	uint8_t touch_major;
	uint8_t width_major;
	uint8_t reserved;
};

struct buf_head {
	uint8_t type;
	uint8_t extern_msg[7];
};

#define MSG_BUFF_SIZE	(256)
#define IO_BUF_SIZE_256	(8)


#define REPORT_X (0x01)
#define REPORT_Y (0x02)
#define REPORT_PRESS (0x04)
#define REPORT_TMAJOR (0x08)
#define REPORT_WMAJOR (0x10)
#define REPORT_KEYDOWN (0x20)
#define REPORT_KEYUP (0x40)
#define REPORT_UP (0x80)
#define REPORT_DOWN_ALL (0x7F)
enum BUF_TYPE
{
    BUF_TYPE_NONE,
    TYPE_POINT,
    TYPE_RAW,
    TYPE_DIFF,
    TYPE_SUSPEND,
    TYPE_RESUME,
    TYPE_RESET,
    TYPE_REPORT,
    TYPE_STATE,
};

enum IOC_STATE_TYPE
{
	IOC_STATE_NONE,
	IOC_STATE_DIR,
	IOC_STATE_CHARGER,
	IOC_STATE_WIRELESS_CHARGER,
	IOC_STATE_GAME,
	IOC_STATE_DEBUG_LEVEL,
	IOC_STATE_PREVENTION_PARA_CHANGE,
	IOC_STATE_RECLINING_MODE,
};

void touch_misc_state_change(void *p_device, enum IOC_STATE_TYPE type, int state);

void init_touch_misc_device(void *p_device);
void uninit_touch_misc_device(void *p_device);
#endif

