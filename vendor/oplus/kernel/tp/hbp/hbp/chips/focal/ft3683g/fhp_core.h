/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2020-2022, FocalTech Systems, Ltd., all rigfhps reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
* File Name: fhp_core.h
* Author: Focaltech Driver Team
* Created: 2020-06-17
*
* Abstract: include file of fhp driver for FocalTech
*
*****************************************************************************/

#ifndef __LINUX_FHP_CORE_H__
#define __LINUX_FHP_CORE_H__
/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
/*ID*/
#define FTS_CHIP_ID                         0x56

/*Please don't modify the value of following chip related macroes*/
#define FTS_REG_CHIP_ID                     0xA3
#define FTS_REG_FW_VER                      0xA6

#define FTS_CMD_START                       0x55
#define FTS_CMD_START_DELAY                 12
#define FTS_CMD_READ_ID                     0x90
#define FT3681_REG_FOD_ERROR_INFO           (0xE0)
#define FT3681_REG_FOD_ERROR_INFO_LEN       (14)
#define FT3681_REG_FOD_INFO                 (0xE1)
#define FT3681_REG_FOD_INFO_LEN             (10)
#define FT3681_REG_AOD_INFO                 (0xD3)
#define FT3681_REG_AOD_INFO_LEN             (6)

#define MAX_SPI_BUF_LENGTH                  4096
#define FTS_MAX_POINTS_SUPPORT              10
#define FTS_MAX_ID                          0x0A
#define FTS_MAX_POINTS_LENGTH               134  /* ((FTS_POINTS_ONE) + (FTS_POINTS_TWO))  */

#define FTS_TOUCH_E_NUM                     1
#define FTS_ONE_TCH_LEN_V2                  8

#define FTS_GESTURE_POINTS_MAX              6
#define FTS_GESTURE_DATA_LEN               (FTS_GESTURE_POINTS_MAX * 4 + 4)

#define FTS_MAX_COMMMAND_LENGTH             16

#define FRAME_SIZE_LEN                      4096
/*****************************************************************************
*  Alternative mode (When something goes wrong, the modules may be able to solve the problem.)
*****************************************************************************/
/*
 * For commnication error in PM(deep sleep) state
 */
#define FTS_PATCH_COMERR_PM                     0
#define FTS_TIMEOUT_COMERR_PM                   700

/*
 * Linux MultiTouch Protocol
 * 1: Protocol B(default), 0: Protocol A
 */
#define FTS_MT_PROTOCOL_B_EN                    1

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

struct ftxxxx_proc {
	struct proc_dir_entry *proc_entry;
	u8 opmode;
	u8 cmd_len;
	u8 cmd[FTS_MAX_COMMMAND_LENGTH];
};

struct fts_core {
	struct ftxxxx_proc proc;

	struct mutex bus_mutex;
	u8 *bus_tx_buf;
	u8 *bus_rx_buf;
	struct bus_operations *bus_ops;
};

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
enum gesture_id {
	GESTURE_FINGER_PRINT_ERROR = 0x16,
	GESTURE_RIGHT2LEFT_SWIP = 0x20,
	GESTURE_LEFT2RIGHT_SWIP = 0x21,
	GESTURE_DOWN2UP_SWIP = 0x22,
	GESTURE_UP2DOWN_SWIP = 0x23,
	GESTURE_DOUBLE_TAP = 0x24,
	GESTURE_DOUBLE_SWIP = 0x25,
	GESTURE_RIGHT_VEE = 0x51,
	GESTURE_LEFT_VEE = 0x52,
	GESTURE_DOWN_VEE = 0x53,
	GESTURE_UP_VEE = 0x54,
	GESTURE_O_CLOCKWISE = 0x57,
	GESTURE_O_ANTICLOCK = 0x30,
	GESTURE_W = 0x31,
	GESTURE_M = 0x32,
	GESTURE_FINGER_PRINT = 0x26,
	GESTURE_SINGLE_TAP = 0x27,
	GESTURE_HEART_ANTICLOCK = 0x55,
	GESTURE_HEART_CLOCKWISE = 0x59,
};

struct fod_info {
	int fp_id;
	int event_type;
	int fp_down;
	int fp_x;
	int fp_y;
	int fp_area_rate;
	int fp_time;
};

struct aod_info {
	u8 gesture_id;
	u8 point_num;
	u16 aod_x;
	u16 aod_y;
};

enum _FTS_TOUCH_ETYPE {
	TOUCH_DEFAULT = 0x00,
	TOUCH_PROTOCOL_v2 = 0x02,
	TOUCH_EXTRA_MSG = 0x08,
	TOUCH_PEN = 0x0B,
	TOUCH_GESTURE = 0x80,
	TOUCH_FW_INIT = 0x81,
	TOUCH_DEFAULT_HI_RES = 0x82,
	TOUCH_IGNORE = 0xFE,
	TOUCH_ERROR = 0xFF,
};

enum _FTS_TOUCH_OFFSET {
	FTS_TOUCH_OFFSET_E_XH = 0,
	FTS_TOUCH_OFFSET_XL,
	FTS_TOUCH_OFFSET_ID_YH,
	FTS_TOUCH_OFFSET_YL,
	FTS_TOUCH_OFFSET_PRE,
	FTS_TOUCH_OFFSET_AREA,
	FTS_TOUCH_OFFSET_MINOR,
};

enum _FTS_RST_REASON {
	FTS_RST_REASON_UNKNOWN  = 0,
	FTS_RST_REASON_FWUPDATE = 0x01,
	FTS_RST_REASON_WDT      = 0x02,
	FTS_RST_REASON_EXTERNAL = 0x04,
	FTS_RST_REASON_PWR      = 0x08,
	FTS_GESTURE_DIFF		= 0x80,
};


enum _FTS_FP_ERROR_TYPE {
	FTS_FINGERPRINT_DOWN_BEFORE_FP_ENABLE = 0,
	FTS_FINGERPRINT_X_Y_NOT_MATCH = 0x02,
	FTS_ANOTHER_FINGER_ON_NON_FP_ZONE = 0x04,
	FTS_FINGERPRINT_AREA_NOT_MATCH = 0x10,
	FTS_FINGERPRINT_OUT_MOVE_IN = 0x40,
};

/* communication interface */
int fhp_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen);
int fhp_read_reg(u8 addr, u8 *value);
int fhp_write(u8 *writebuf, u32 writelen);
int fhp_write_reg(u8 addr, u8 value);


#endif /* __LINUX_FHP_CORE_H__ */
