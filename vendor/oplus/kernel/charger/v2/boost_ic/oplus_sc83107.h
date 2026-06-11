/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_sc83107.h
** Description: hybridboost ic
** Date: 2025-11-01
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#ifndef _OPLUS_SC83107_H_
#define _OPLUS_SC83107_H_

#include <oplus_chg_mutual.h>

/* reg 0x09*/
#define SC83107_POR_FLAG_BIT	BIT(1)
#define SC83107_VBAT_FALLING_FLAG_BIT	BIT(2)
#define SC83107_BST_OCP_FLAG_BIT	BIT(6)
#define SC83107_HOTDIE_FLAG_BIT	BIT(7)

/* reg 0x0A */
#define SC83107_TSD_FLAG_BIT		BIT(7)
#define SC83107_BPASS_VOUT_OVP_FLAG_BIT		BIT(6)
#define SC83107_BOOST_VOUT_OVP_FLAG_BIT		BIT(5)
#define SC83107_VOUT_UVP_FLAG_BIT		BIT(4)
#define SC83107_Q3_OR_Q6_OCP_FLAG_BIT 		BIT(2)
#define SC83107_PIN_DIAG_FAIL_FLAG_BIT		BIT(0)
#define ERR_MSG_BUF	PAGE_SIZE

#define TRACK_LOCAL_T_NS_TO_S_THD		1000000000
#define TRACK_UPLOAD_COUNT_MAX			10
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD	(24 * 3600)
#define REASON_LENGTH_MAX			1024
#define ERR_LENGTH_MAX				64
#define DUMP_LENGTH_MAX				512

enum sc83107_part_no {
	SC83107_DEVICE_ID = 0x01,
};

enum sc83107_fields {
	F_DEVICE_ID, /* reg0 */
	F_RESET, F_FSW_SET, F_OVP_TH_CT, F_OTG_MODE, F_FPWM_CFG, F_FORCE_BP, /* reg01 */
	F_VOUT_OVP_OFF, F_VREG, /* reg02 */
	F_THOT_DIE_CT, F_INT_DEGLITCH, F_ILIM_OFF, F_ILIM, /* reg03 */
	F_VTH_BP2CV_CT, F_VTH_BP2CV_DEG, F_SSFM, /* reg04 */
	F_VTH_CV2BP_CT_HYS, F_VTH_CV2BP_DEG, /* reg05 */
	F_TON_MAX_LIM, F_VBAT_SNS_DIS, F_WD_TIMEOUT, /* reg06 */
	F_LV2_COMP_CT, F_LV2_COMP_HYS_CT, F_LV0_COMP_CT, F_LV1_COMP_CT, /* reg07 */
	F_HOTDIE, F_BST_OCP, F_DCDCMODE, F_OPMODE_BOOST, F_OPMODE_BYP, F_VAC_OK, F_VBAT_OK, F_PGOOD, /* reg08 */
	F_HOTDIE_FLAG, F_BST_OCP_FLAG, F_INT_TIMEOUT_FLAG, F_VAC_FALLING_FLAG, F_VBAT_FALLING_FLAG, F_POR_FLAG, F_PGOOD_FLAG, /* reg09 */
	F_TSD_FLAG, F_BPASS_VOUT_OVP_FLAG, F_BOOST_VOUT_OVP_FLAG, F_VOUT_UVP_FLAG, F_WD_TIMEOUT_FLAG, F_Q3Q6_OCP_FLAG, F_PIN_DIAG_FAIL_FLAG, /* reg0A */
	F_HOTDIE_MASK, F_BST_OCP_MASK, F_INT_TIMEOUT_MASK, F_VAC_FALLING_MASK, F_VBAT_FALLING_MASK, F_PGOOD_MASK, /* reg0B */
	F_TSD_MASK, F_BPASS_VOUT_OVP_MASK, F_BOOST_VOUT_OVP_MASK, F_VOUT_UVP_MASK, F_WD_TIMEOUT_MASK, F_Q3Q6_OCP_MASK, F_PIN_DIAG_FAIL_MASK, /* reg0C */
	F_TON_MIN1, F_VOUT_UVP_DEG, F_DIS_Q3Q6_OCP, /* 0x0D */
	F_TON_MIN2, /* 0x0F */
	F_BCL_CLAMP_UP, /* reg81 */
};

struct sc83107_reg_field {
	uint8_t reg;
	uint8_t lsb;
	uint8_t msb;
	bool force_write;
};

#define REG_FIELD(_reg, _lsb, _msb) { \
	.reg = (_reg), \
	.lsb = (_lsb), \
	.msb = (_msb), \
}

#define SC_REG_FIELD_FORCE_WRITE(_reg, _lsb, _msb) {           \
				.reg = _reg,                \
				.lsb = _lsb,                \
				.msb = _msb,                \
				.force_write = true,        \
}
//REGISTER
static const struct sc83107_reg_field sc83107_reg_fields[] = {
	[F_DEVICE_ID] = REG_FIELD(0x00, 0, 7),
	[F_RESET] = REG_FIELD(0x01, 7, 7),
	[F_FSW_SET] = REG_FIELD(0x01, 6, 6),
	[F_OVP_TH_CT] = REG_FIELD(0x01, 3, 5),
	[F_OTG_MODE] = REG_FIELD(0x01, 2, 2),
	[F_FPWM_CFG] = REG_FIELD(0x01, 1, 1),
	[F_FORCE_BP] = REG_FIELD(0x01, 0, 0),
	[F_VOUT_OVP_OFF] = REG_FIELD(0x02, 7, 7),
	[F_VREG] = REG_FIELD(0x02, 0, 5),
	[F_THOT_DIE_CT] = REG_FIELD(0x03, 7, 7),
	[F_INT_DEGLITCH] = REG_FIELD(0x03, 5, 6),
	[F_ILIM_OFF] = REG_FIELD(0x03, 3, 3),
	[F_ILIM] = REG_FIELD(0x03, 0, 2),
	[F_VTH_BP2CV_CT] = REG_FIELD(0x04, 6, 7),
	[F_VTH_BP2CV_DEG] = REG_FIELD(0x04, 3, 5),
	[F_SSFM] = REG_FIELD(0x04, 0, 0),
	[F_VTH_CV2BP_CT_HYS] = REG_FIELD(0x05, 6, 7),
	[F_VTH_CV2BP_DEG] = REG_FIELD(0x05, 3, 5),
	[F_TON_MAX_LIM] = REG_FIELD(0x06, 4, 7),
	[F_VBAT_SNS_DIS] = REG_FIELD(0x06, 3, 3),
	[F_WD_TIMEOUT] = REG_FIELD(0x06, 0, 2),
	[F_LV2_COMP_CT] = REG_FIELD(0x07, 5, 7),
	[F_LV2_COMP_HYS_CT] = REG_FIELD(0x07, 4, 4),
	[F_LV0_COMP_CT] = REG_FIELD(0x07, 2, 3),
	[F_LV1_COMP_CT] = REG_FIELD(0x07, 0, 1),
	[F_HOTDIE] = REG_FIELD(0x08, 7, 7),
	[F_BST_OCP] = REG_FIELD(0x08, 6, 6),
	[F_DCDCMODE] = REG_FIELD(0x08, 5, 5),
	[F_OPMODE_BOOST] = REG_FIELD(0x08, 4, 4),
	[F_OPMODE_BYP] = REG_FIELD(0x08, 3, 3),
	[F_VAC_OK] = REG_FIELD(0x08, 2, 2),
	[F_VBAT_OK] = REG_FIELD(0x08, 1, 1),
	[F_PGOOD] = REG_FIELD(0x08, 0, 0),
	[F_HOTDIE_FLAG] = REG_FIELD(0x09, 7, 7),
	[F_BST_OCP_FLAG] = REG_FIELD(0x09, 6, 6),
	[F_INT_TIMEOUT_FLAG] = REG_FIELD(0x09, 4, 4),
	[F_VAC_FALLING_FLAG] = REG_FIELD(0x09, 3, 3),
	[F_VBAT_FALLING_FLAG] = REG_FIELD(0x09, 2, 2),
	[F_POR_FLAG] = REG_FIELD(0x09, 1, 1),
	[F_PGOOD_FLAG] = REG_FIELD(0x09, 0, 0),
	[F_TSD_FLAG] = REG_FIELD(0x0A, 7, 7),
	[F_BPASS_VOUT_OVP_FLAG] = REG_FIELD(0x0A, 6, 6),
	[F_BOOST_VOUT_OVP_FLAG] = REG_FIELD(0x0A, 5, 5),
	[F_VOUT_UVP_FLAG] = REG_FIELD(0x0A, 4, 4),
	[F_WD_TIMEOUT_FLAG] = REG_FIELD(0x0A, 3, 3),
	[F_Q3Q6_OCP_FLAG] = REG_FIELD(0x0A, 2, 2),
	[F_PIN_DIAG_FAIL_FLAG] = REG_FIELD(0x0A, 0, 0),
	[F_HOTDIE_MASK] = REG_FIELD(0x0B, 7, 7),
	[F_BST_OCP_MASK] = REG_FIELD(0x0B, 6, 6),
	[F_INT_TIMEOUT_MASK] = REG_FIELD(0x0B, 4, 4),
	[F_VAC_FALLING_MASK] = REG_FIELD(0x0B, 3, 3),
	[F_VBAT_FALLING_MASK] = REG_FIELD(0x0B, 2, 2),
	[F_PGOOD_MASK] = REG_FIELD(0x0B, 0, 0),
	[F_TSD_MASK] = REG_FIELD(0x0C, 7, 7),
	[F_BPASS_VOUT_OVP_MASK] = REG_FIELD(0x0C, 6, 6),
	[F_BOOST_VOUT_OVP_MASK] = REG_FIELD(0x0C, 5, 5),
	[F_VOUT_UVP_MASK] = REG_FIELD(0x0C, 4, 4),
	[F_WD_TIMEOUT_MASK] = REG_FIELD(0x0C, 3, 3),
	[F_Q3Q6_OCP_MASK] = REG_FIELD(0x0C, 2, 2),
	[F_PIN_DIAG_FAIL_MASK] = REG_FIELD(0x0C, 0, 0),
	[F_TON_MIN1] = REG_FIELD(0x0D, 5, 6),
	[F_VOUT_UVP_DEG] = REG_FIELD(0x0D, 4, 4),
	[F_DIS_Q3Q6_OCP] = REG_FIELD(0x0D, 3, 3),
	[F_TON_MIN2] = REG_FIELD(0x0F, 0, 1),
	[F_BCL_CLAMP_UP] = REG_FIELD(0x81, 3, 3),
};

struct sc83107_cfg_e {
	int operation_mode;
	int freq;
	int otg_mode;
	int wdt_timeout;
	int vout_ovp_dis;
	int current_limit_dis;
};

struct sc83107_chip {
	struct device *dev;
	struct i2c_client *client;

	struct oplus_chg_ic_dev *boost_ic;
	struct sc83107_cfg_e cfg;

	struct pinctrl *pinctrl;
	struct pinctrl_state *boost_inter_active;
	struct pinctrl_state *boost_inter_sleep;
	int irq_gpio;
	int irq;
	uint8_t dev_id;

	struct work_struct charger_plug_work;

	/* Force bypass retry mechanism */
	struct delayed_work force_bp_retry_work;
	bool force_bp_retry_enabled;

	/* Track upload mechanism */
	struct delayed_work track_upload_work;
	bool track_upload_pending;

	/* IRQ handler work - for processing interrupt in process context */
	struct delayed_work irq_handler_work;
	bool irq_handler_work_pending;

	/* GPIO state tracking */
	bool gpio_pulled_down;

	/* Suspend state tracking */
	atomic_t suspended;

	/* Wired topic subscription */
	struct oplus_mms *wired_topic;
	struct mms_subscribe *wired_subs;

	/* Comm topic subscription for boot completed */
	struct oplus_mms *comm_topic;
	struct mms_subscribe *comm_subs;
	int last_ui_soc; /* Track last UI SOC to detect 1% entry */

	/* Mutual notifier for reading error flag from partition */
	struct oplus_chg_mutual_notifier dischg_boost_err_flag_mutual;
	char dischg_boost_err_flag_data[128];
	struct delayed_work get_dischg_boost_err_flag_work;

	/* Work queue for uploading dischg boost err flag from mutual notifier (atomic context) */
	struct work_struct dischg_boost_err_flag_upload_work;
	unsigned int dischg_boost_err_flag; /* Error flag to be uploaded in process context */
	u8 dischg_boost_err_reg08_val; /* Register 0x08 value to be uploaded */
	u8 dischg_boost_err_reg09_val; /* Register 0x09 value to be uploaded */
	u8 dischg_boost_err_reg0a_val; /* Register 0x0A value to be uploaded */

	/* Wakelock for critical I2C operations */
	struct wakeup_source *i2c_wake_lock;

	/* Suspend/Resume CV configuration */
	int suspend_cv_mv;
	int resume_cv_mv;

	/* I2C bus reset feature control */
	bool i2c_bus_reset_enable;
};

enum sc83107_flag_type {
	SC83107_PGOOD_FLAG,
	SC83107_POR_FLAG,
	SC83107_VBAT_FALLING_FLAG,
	SC83107_VAC_FALLING_FLAG,
	SC83107_INT_TIMEOUT_FLAG,
	SC83107_RESERVED0_FLAG,
	SC83107_BST_OCP_FLAG,
	SC83107_HOTDIE_FLAG,
	SC83107_PIN_DIAG_FAIL_FLAG,
	SC83107_RESERVED1_FLAG,
	SC83107_Q3Q6_OCP_FLAG,
	SC83107_WD_TIMEOUT_FLAG,
	SC83107_VOUT_UVP_FLAG,
	SC83107_BOOST_VOUT_OVP_FLAG,
	SC83107_BPASS_VOUT_OVP_FLAG,
	SC83107_TSD_FLAG,
	SC83107_FLAG_MAX = SC83107_TSD_FLAG,
};

enum sc83107_mask_type {
	SC83107_MASK_PGOOD,
	SC83107_MASK_RESERVED0,
	SC83107_MASK_VBAT_FALLING,
	SC83107_MASK_VAC_FALLING,
	SC83107_MASK_INT_TIMEOUT,
	SC83107_MASK_RESERVED1,
	SC83107_MASK_BST_OCP,
	SC83107_MASK_HOTDIE,
	SC83107_MASK_PIN_DIAG_FAIL,
	SC83107_MASK_RESERVED2,
	SC83107_MASK_Q3Q6_OCP,
	SC83107_MASK_WD_TIMEOUT,
	SC83107_MASK_VOUT_UVP,
	SC83107_MASK_BOOST_VOUT_OVP,
	SC83107_MASK_BPASS_VOUT_OVP,
	SC83107_MASK_TSD,
	SC83107_MASK_MAX = SC83107_MASK_TSD,
};

#endif /*_OPLUS_SC83107_H_*/
