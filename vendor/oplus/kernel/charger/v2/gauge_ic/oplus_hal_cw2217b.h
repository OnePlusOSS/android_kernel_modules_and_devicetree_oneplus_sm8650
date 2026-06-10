// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2022 Oplus. All rights reserved.
 */

#ifndef __OPLUS_CW2217B_H__
#define __OPLUS_CW2217B_H__

#include <linux/regmap.h>
#include <oplus_chg_ic.h>
#include <oplus_mms_gauge.h>

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
#include <debug-kit.h>
#endif

#define DRIVER_VERSION			"1.1.0"

#define CWFG_ENABLE_LOG 1 /* CHANGE Customer need to change this for enable/disable log */

#define REG_CHIP_ID             0x00
#define REG_VCELL_H             0x02
#define REG_VCELL_L             0x03
#define REG_SOC_INT             0x04
#define REG_SOC_DECIMAL         0x05
#define REG_TEMP                0x06
#define REG_MODE_CONFIG         0x08
#define REG_GPIO_CONFIG         0x0A
#define REG_SOC_ALERT           0x0B
#define REG_TEMP_MAX            0x0C
#define REG_TEMP_MIN            0x0D
#define REG_CURRENT_H           0x0E
#define REG_CURRENT_L           0x0F
#define REG_T_HOST_H            0xA0
#define REG_T_HOST_L            0xA1
#define REG_USER_CONF           0xA2
#define REG_CYCLE_H             0xA4
#define REG_CYCLE_L             0xA5
#define REG_SOH                 0xA6
#define REG_IC_STATE            0xA7
#define REG_STB_CUR_H           0xA8
#define REG_STB_CUR_L           0xA9
#define REG_FW_CHECK            0xAA
#define REG_FW_VERSION          0xAB
#define REG_BAT_PROFILE         0x10

#define CONFIG_MODE_RESTART     0x30
#define CONFIG_MODE_ACTIVE      0x00
#define CONFIG_MODE_SLEEP       0xF0
#define CONFIG_UPDATE_FLG       0x80
#define IC_VCHIP_ID             0xA0
#define IC_READY_MARK           0x0C

#define GPIO_ENABLE_MIN_TEMP    0
#define GPIO_ENABLE_MAX_TEMP    0
#define GPIO_ENABLE_SOC_CHANGE  0
#define GPIO_SOC_IRQ_VALUE      0x0    /* 0x7F */
#define DEFINED_MAX_TEMP        45
#define DEFINED_MIN_TEMP        0

#define CWFG_NAME               "cw2217"
#define SIZE_OF_PROFILE         80
#define CW_INFO_LEN             255
#define CW_NAME_LEN             12

#define QUEUE_DELAYED_WORK_TIME  5000
#define QUEUE_START_WORK_TIME    50

#define CW_REG_WORD             2
#define CW_REG_BYTE             1
#define CW_REG_BYTE_BITS        8
#define CW_SLEEP_1MS            1
#define CW_SLEEP_20MS           20
#define CW_SLEEP_10MS           10
#define CW_VOL_MAGIC_PART1      5
#define CW_VOL_MAGIC_PART2      16
#define CW_SOC_MAGIC_BASE       256
#define CW_SOC_MAGIC_100        100
#define CW_TEMP_MAGIC_PART1     10
#define CW_TEMP_MAGIC_PART2     2
#define CW_TEMP_MAGIC_PART3     400
#define COMPLEMENT_CODE_U16     0x8000
#define CW_CUR_MAGIC_PART1      160000
#define CW_CUR_MAGIC_PART2      100
#define CW_STB_CUR_MAGIC_PART3  16
#define CW_SLEEP_100MS          100
#define CW_SLEEP_200MS          200
#define CW_SLEEP_COUNTS         50
#define CW_TRUE                 1
#define CW_RETRY_COUNT          3
#define CW_VOL_UNIT             1000
#define CW_CYCLE_MAGIC          16

#define NUM_0                   0
#define NUM_1                   1
#define ERR_NUM                 -1

#define BATTID_ARR_LEN          5
#define BATTID_ARR_WIDTH        2
#define BAT_TYPE_UNKNOWN        0

#define DEVICE_BQ27541          0
#define DEVICE_BQ27411          1
#define DEVICE_BQ28Z610         2
#define DEVICE_ZY0602           3
#define DEVICE_ZY0603           4
#define DEVICE_NFG8011B         5
#define DEVICE_BQ27Z561         6
#define DEVICE_CW2217B          7

#define CW2217_NOT_ACTIVE          1
#define CW2217_PROFILE_NOT_READY   2
#define CW2217_PROFILE_NEED_UPDATE 3
#define CW2217_CHECK_UPDATE        4

#define BATNUM 2
#define CHG_NTC_TEMP_COUNT 796
/*default batt-profile*/
static char *battery_name[BATNUM] = {"BLT004-ALT-7100MA"};
static unsigned char config_profile_info[SIZE_OF_PROFILE] = {
		0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
		0xAF, 0xBC, 0xBB, 0xBF, 0xA9, 0xA1, 0xDF, 0xCD,
		0xC3, 0xF0, 0xCE, 0x97, 0x7E, 0x64, 0x53, 0x46,
		0x3B, 0x32, 0x29, 0x86, 0x75, 0xE0, 0x39, 0xDE,
		0xCB, 0xCA, 0xD0, 0xD4, 0xD5, 0xD4, 0xD1, 0xCD,
		0xC8, 0xCA, 0xD9, 0xBE, 0xA2, 0x95, 0x8E, 0x84,
		0x81, 0x83, 0x8B, 0x95, 0xAA, 0x94, 0x6B, 0x6E,
		0x20, 0x00, 0xAB, 0x10, 0x00, 0x91, 0x83, 0x00,
		0x00, 0x00, 0x64, 0x14, 0xA0, 0xA0, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE9,
};

typedef enum {
	DEC_CV_FG_UNKNOWN,
	PACK_DOUBLE_SERIES,
	PACK_DOUBLE_PARALLE,
	PACK_SINGLE,
	PACK_SOH,
	QCOM_FG,
	MTK_FG,
	MB_TI,
	MB_CW,
	MB_SMI,
	DEC_CV_FG_MAX,
} DEC_CV_FG_TYPE;

struct cw_battery {
	struct i2c_client *client;
	struct device		*dev;
	struct workqueue_struct *cwfg_workqueue;
	struct delayed_work battery_delay_work;
        struct oplus_chg_ic_dev *ic_dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *ntc_switch2_low;
	struct pinctrl_state *ntc_switch2_high;
	bool ntc_switch_support;
	int  chip_id;
	int  voltage;
	int  ic_soc_h;
	int  ic_soc_l;
	int  ui_soc;
	int  temp;
	long cw_current;
	int  cycle;
	int  soh;
	int  fcc;
	int  design_capacity;
	int  rated_capacity;
	int  fw_version;
	struct iio_channel *batt_id_chan;
	struct iio_channel *sub_board_temp_chan;
        int batt_num;
	int cw_ui_full;
	bool cw_switch_config_profile;
	bool ignore_battery_authenticate;
	int cw_user_rsense;
	unsigned char cw_config_profile[SIZE_OF_PROFILE];
	u32 batid_voltage_range[BATTID_ARR_LEN][BATTID_ARR_WIDTH];

	struct mutex track_upload_lock;
	struct mutex track_cw_err_lock;
	u32 debug_force_cw_err;
	bool cw_err_uploading;
	struct delayed_work cw_track_update_work;
	char track_info[CW_INFO_LEN];
	u8 device_name[CW_NAME_LEN];
	int device_type;
	int chg_ntc_temp_table[CHG_NTC_TEMP_COUNT];
};

bool oplus_gauge_check_chip_is_null(void);
int oplus_vooc_get_fastchg_started(void);
int oplus_vooc_get_fastchg_ing(void);
#endif /* __OPLUS_CW2217B_H__ */
