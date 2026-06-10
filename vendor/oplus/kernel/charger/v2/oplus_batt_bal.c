// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "[OPLUS_BATT_BAL]([%s][%d]): " fmt, __func__, __LINE__

#include "oplus_batt_bal.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/sched/clock.h>
#include <linux/of_platform.h>
#include <linux/jiffies.h>

#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_chg_monitor.h>
#include <oplus_mms_wired.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_wls.h>
#include <oplus_chg_pps.h>
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mtk_boot_common.h>
#endif
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <oplus_chg.h>
#include "monitor/oplus_chg_track.h"
#include <linux/sched.h>
#include <linux/thermal.h>

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "oplus_cfg.h"
#endif

#define BATT_BAL_CURVE_NAME_LEN 20
#define BATT_BAL_TIME_MS(x) x
#define BATT_BAL_CURR_MA(x) x
#define BATT_BAL_UPDATE_INTERVAL(time) msecs_to_jiffies(time)
#define BATT_BAL_ENTER_LPH_DISCHG_VBATT 3600
#define BATT_BAL_EXIT_LPH_DISCHG_VBATT 3700
#define BATT_BAL_LPH_DISCHG_SWITCH_COUNT 2
#define BATT_BAL_CURR_BELOW_COUNT 1
#define BATT_BAL_FAST_CURR_BELOW_COUNT 1

/* The num of temp region max suppoerted */
#define BATT_BAL_MAX_TEMP_COUNT_DFT 7

#define SERIES_FORM	1
#define BATT_BAL_FIX_CURR_MA(x) (x % 500 ? ((x /500 + 1) * 500) : x)
#define BATT_BAL_LIMIT_CURR_MA(x) ((x /500 - 1) * 500)
#define BATT_BAL_VOL_ERR_MV					1000
#define BATT_BAL_TEMP_ERR_DEC					(-400)
#define BATT_BAL_ABNORMAL_STATE_COUNT		2
#define BATT_BAL_STATUS_RECORD_LEN			500

#define OPLUS_CHG_GET_SUB_CURRENT          _IOWR('M', 1, char[256])
#define OPLUS_CHG_GET_SUB_VOLTAGE          _IOWR('M', 2, char[256])
#define OPLUS_CHG_GET_SUB_SOC              _IOWR('M', 3, char[256])
#define OPLUS_CHG_GET_SUB_TEMPERATURE      _IOWR('M', 4, char[256])
#define OPLUS_CHG_GET_PARALLEL_SUPPORT     _IOWR('M', 5, char[256])
#define OPLUS_CHG_GET_BAL_CURR             _IOWR('M', 6, char[256])

enum batt_temp_region {
	BATT_TEMP_REGION_T0 = 0,
	BATT_TEMP_REGION_T1,
	BATT_TEMP_REGION_T2,
	BATT_TEMP_REGION_T3,
	BATT_TEMP_REGION_T4,
	BATT_TEMP_REGION_T5,
	BATT_TEMP_REGION_T6,
	BATT_TEMP_REGION_T7,
	BATT_TEMP_REGION_T8,
	BATT_TEMP_REGION_T9,
	BATT_TEMP_REGION_T10,
	BATT_TEMP_REGION_MAX,
};

enum oplus_batt_bal_state {
	OPLUS_BATT_BAL_STATE_INIT = 0,
	OPLUS_BATT_BAL_STATE_DISCHG,
	OPLUS_BATT_BAL_STATE_NORMAL_CHG,
	OPLUS_BATT_BAL_STATE_FAST_START,
	OPLUS_BATT_BAL_STATE_FAST_BAL_START,
	OPLUS_BATT_BAL_STATE_FAST_CC_CHG,
	OPLUS_BATT_BAL_STATE_FFC_CHG,
	OPLUS_BATT_BAL_STATE_LPH_DISCHG,
	OPLUS_BATT_BAL_STATE_ABNORMAL,
};

enum oplus_batt_bal_sub_state {
	BAL_SUB_STATE_INIT,
	BAL_SUB_STATE_BAL,
	BAL_SUB_STATE_B1_TO_B2,
	BAL_SUB_STATE_B2_TO_B1,
};

enum bal_args_ctrl_region {
	CTRL_REGION_DISCHG = 0,
	CTRL_REGION_NORMAL_CHG ,
	CTRL_REGION_FAST_CHG,
	CTRL_REGION_FFC_CHG,
	CTRL_REGION_MAX,
};

enum fast_bal_ctrl_mode {
	FAST_CTRL_DEFUALT_MODE,
	FAST_CTRL_CURR_MODE,
	FAST_CTRL_VOLT_MODE,
};

struct batt_bal_spec {
	int volt;
	int curr;
};

struct batt_bal_curr_over {
	bool b1_over;
	bool b2_over;
	int curr_limit;
	int b1_limit_curr;
	int b2_limit_curr;
	unsigned long not_allow_limit_jiffies;
};

struct batt_bal_curr_below {
	bool b1_below;
	bool b2_below;
	int b1_below_count;
	int b2_below_count;
};

struct bal_bat_curve_table {
	struct batt_bal_spec * curve_table;
	int length;
};

struct oplus_batt_bal_cfg {
	int bal_iterm;
	int bal_max_iref_thr;

	int dischg_volt_diff_thr;
	int dischg_volt_diff_anti_thr;
	int dischg_curr_thr;
	int lph_dischg_volt_diff_thr;
	int lph_dischg_volt_diff_hold;

	int b1_design_cap;
	int b2_design_cap;

	int fastchg_b2_to_b1_max_curr_thr;
	int fastchg_b2_to_b1_max_curr_anti_thr;

	struct bal_bat_curve_table *b1;
	struct bal_bat_curve_table *b2;

	uint32_t bal_temp_region_count;
	int32_t chg_volt_diff_thr[BATT_TEMP_REGION_MAX];
	int32_t chg_curr_thr[BATT_TEMP_REGION_MAX];
	int32_t vout_thr[CTRL_REGION_MAX];
	int32_t curr_over_thr[CTRL_REGION_MAX];
	int32_t curr_below_thr[CTRL_REGION_MAX];

	uint8_t *strategy_name;
	uint8_t *b1_strategy_data;
	uint32_t b1_strategy_data_size;
	uint8_t *b2_strategy_data;
	uint32_t b2_strategy_data_size;
};

struct oplus_batt_bal_cutoff_limits {
	unsigned int cv_volt;
	unsigned int cv_curr;
	unsigned int cv_sub_curr;
	unsigned int ffc_curr;
	unsigned int ffc_sub_curr;
};

struct oplus_batt_bal_chip {
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;

	struct mutex state_lock;
	struct mutex cfg_lock;
	struct mutex update_batt_info_lock;
	enum oplus_batt_bal_state curr_bal_state;
	enum oplus_batt_bal_state pre_bal_state;
	enum oplus_batt_bal_sub_state sub_step_state;

	struct delayed_work batt_bal_init_work;
	struct delayed_work batt_bal_sm_work;
	struct delayed_work ic_trig_abnormal_clear_work;
	struct work_struct batt_bal_charging_init_work;
	struct work_struct batt_bal_online_init_work;
	struct work_struct batt_bal_disable_bal_work;
	struct work_struct err_handler_work;
	struct work_struct update_run_interval_work;
	struct work_struct update_bal_state_work;
	struct work_struct bal_track_work;

	struct oplus_mms *gauge_topic;
	struct oplus_mms *main_gauge_topic;
	struct oplus_mms *sub_gauge_topic;
	struct oplus_mms *batt_bal_topic;
	struct oplus_mms *comm_topic;
	struct oplus_mms *wired_topic;
	struct oplus_mms *vooc_topic;
	struct oplus_mms *err_topic;
	struct oplus_mms *ufcs_topic;
	struct oplus_mms *wls_topic;
	struct oplus_mms *pps_topic;
	struct mms_subscribe *comm_subs;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *gauge_subs;
	struct mms_subscribe *main_gauge_subs;
	struct mms_subscribe *sub_gauge_subs;
	struct mms_subscribe *vooc_subs;
	struct mms_subscribe *ufcs_subs;
	struct mms_subscribe *wls_subs;
	struct mms_subscribe *pps_subs;
	struct mms_subscribe *bal_subs;

	u8 status_record[BATT_BAL_STATUS_RECORD_LEN];
	int deep_support;
	atomic_t ic_trig_abnormal;

	bool wired_online;
	bool wls_online;
	struct votable *vooc_disable_votable;
	struct votable *pps_disable_votable;
	struct votable *ufcs_disable_votable;
	struct votable *wls_disable_votable;
	struct votable *run_interval_update_votable;
	struct votable *gauge_term_voltage_votable;
	int run_interval_update;

	struct oplus_batt_bal_cfg cfg;
	enum batt_bal_flow_dir flow_dir;
	int target_iref;

	enum batt_temp_region b1_temp_region;
	enum batt_temp_region b2_temp_region;

	int b1_volt;
	int b2_volt;
	int b1_temp;
	int b2_temp;
	int b1_curr;
	int b2_curr;
	int b1_soc;
	int b2_soc;

	struct batt_bal_curr_over curr_over;
	struct batt_bal_curr_below curr_below;
	struct oplus_batt_bal_cutoff_limits cutoff;

	bool wired_charging_enable;
	bool wls_charging_enable;

	bool vooc_fastchg_ing;
	bool vooc_by_normal_path;
	bool pps_fastchg_ing;
	bool ufcs_fastchg_ing;
	bool wls_fastchg_ing;
	int ffc_status;
	int ffc_step;

	int lph_dischg_switch_count;
	int eq_volt_diff_thr;
	int eq_curr_thr;

	enum fast_bal_ctrl_mode fast_ctrl_mode;
	int fast_curr_ref;
	int eq_fastchg_b2_to_b1_max_curr_thr;

	enum batt_bal_abnormal_state abnormal_state;
	struct oplus_chg_strategy *b1_inr_strategy;
	struct oplus_chg_strategy *b2_inr_strategy;
	int track_err_type;
	bool full_err_report;
	bool init_done;
	struct miscdevice misc_dev;
#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	struct oplus_cfg debug_cfg;
#endif
#ifdef CONFIG_THERMAL
	struct thermal_zone_device *sub_batt_temp_tzd;
#endif
	int alarm_curr_status;
};

static const char * const oplus_batt_bal_dir_flow_text[] = {
	[DEFAULT_DIR] = "default",
	[B1_TO_B2] = "b1_to_b2",
	[B2_TO_B1] = "b2_to_b1",
};

static const char * const batt_bal_temp_table[] = {
	[BATT_TEMP_REGION_T0] = "batt_temp_t0",
	[BATT_TEMP_REGION_T1] = "batt_temp_t1",
	[BATT_TEMP_REGION_T2] = "batt_temp_t2",
	[BATT_TEMP_REGION_T3] = "batt_temp_t3",
	[BATT_TEMP_REGION_T4] = "batt_temp_t4",
	[BATT_TEMP_REGION_T5] = "batt_temp_t5",
	[BATT_TEMP_REGION_T6] = "batt_temp_t6",
	[BATT_TEMP_REGION_T7] = "batt_temp_t7",
	[BATT_TEMP_REGION_T8] = "batt_temp_t8",
	[BATT_TEMP_REGION_T9] = "batt_temp_t9",
	[BATT_TEMP_REGION_T10] = "batt_temp_t10",
};

static const char * const oplus_batt_bal_state_text[] = {
	[OPLUS_BATT_BAL_STATE_INIT] = "init",
	[OPLUS_BATT_BAL_STATE_DISCHG] = "dischg",
	[OPLUS_BATT_BAL_STATE_NORMAL_CHG] = "normal_chg",
	[OPLUS_BATT_BAL_STATE_FAST_START] = "fast_start",
	[OPLUS_BATT_BAL_STATE_FAST_BAL_START] = "fast_bal_start",
	[OPLUS_BATT_BAL_STATE_FAST_CC_CHG] = "fast_cc_chg",
	[OPLUS_BATT_BAL_STATE_FFC_CHG] = "ffc_chg",
	[OPLUS_BATT_BAL_STATE_LPH_DISCHG] = "lph_dischg",
	[OPLUS_BATT_BAL_STATE_ABNORMAL] = "abnormal",
};

static int oplus_batt_bal_update_batt_info(struct oplus_batt_bal_chip *chip);

static const char *
oplus_batt_bal_get_bal_state_str(enum oplus_batt_bal_state state)
{
	return oplus_batt_bal_state_text[state];
}

__maybe_unused static struct oplus_batt_bal_chip *oplus_batt_bal_get_chip(void)
{
	struct oplus_batt_bal_chip *chip = NULL;
	static struct oplus_mms *batt_bal_topic;

	if (!batt_bal_topic)
		batt_bal_topic = oplus_mms_get_by_name("batt_bal");

	if (batt_bal_topic)
		chip = oplus_mms_get_drvdata(batt_bal_topic);

	return chip;
}

__maybe_unused static bool
is_err_topic_available(struct oplus_batt_bal_chip *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

__maybe_unused static bool
is_vooc_disable_votable_available(struct oplus_batt_bal_chip *chip)
{
	if (!chip->vooc_disable_votable)
		chip->vooc_disable_votable = find_votable("VOOC_DISABLE");
	return !!chip->vooc_disable_votable;
}

__maybe_unused static bool
is_pps_disable_votable_available(struct oplus_batt_bal_chip *chip)
{
	if (!chip->pps_disable_votable)
		chip->pps_disable_votable = find_votable("PPS_DISABLE");
	return !!chip->pps_disable_votable;
}

__maybe_unused static bool
is_ufcs_disable_votable_available(struct oplus_batt_bal_chip *chip)
{
	if (!chip->ufcs_disable_votable)
		chip->ufcs_disable_votable = find_votable("UFCS_DISABLE");
	return !!chip->ufcs_disable_votable;
}

__maybe_unused static bool
is_wls_disable_votable_available(struct oplus_batt_bal_chip *chip)
{
	if (!chip->wls_disable_votable)
		chip->wls_disable_votable = find_votable("WLS_FASTCHG_DISABLE");
	return !!chip->wls_disable_votable;
}

__maybe_unused static bool
is_gauge_term_voltage_votable_available(struct oplus_batt_bal_chip *chip)
{
	if (!chip->gauge_term_voltage_votable)
		chip->gauge_term_voltage_votable = find_votable("GAUGE_TERM_VOLTAGE");
	return !!chip->gauge_term_voltage_votable;
}

static ssize_t oplus_chg_read(struct file *fp, char __user *buff, size_t count, loff_t *off)
{
	char page[256] = { 0 };
	int len;
	struct oplus_batt_bal_chip *chip = container_of(fp->private_data,
		struct oplus_batt_bal_chip, misc_dev);

	mutex_lock(&chip->update_batt_info_lock);
	len = sprintf(page, "sub_current=%d\nsub_voltage=%d\nsub_soc=%d\nsub_temperature=%d\nmain_soc=%d\nbal_curr=%d\n",
		      -chip->b1_curr,  chip->b1_volt, chip->b1_soc, chip->b1_temp, chip->b2_soc, chip->target_iref);
	mutex_unlock(&chip->update_batt_info_lock);

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buff, page, (len < count ? len : count))) {
		chg_err("copy_to_user error\n");
		return -EFAULT;
	}
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

static ssize_t oplus_chg_write(struct file *fp, const char __user *buf, size_t count, loff_t *pos)
{
	return count;
}

static long oplus_chg_ioctl(struct file *fp, unsigned code, unsigned long value)
{
	char src[256] = { 0 };
	int ret;
	int b1_volt;
	int b1_temp;
	int b1_curr;
	int b1_soc;
	struct oplus_batt_bal_chip *chip = container_of(fp->private_data,
		struct oplus_batt_bal_chip, misc_dev);

	mutex_lock(&chip->update_batt_info_lock);
	b1_volt = chip->b1_volt;
	b1_temp = chip->b1_temp;
	b1_curr = -chip->b1_curr;
	b1_soc = chip->b1_soc;
	mutex_unlock(&chip->update_batt_info_lock);
	switch (code) {
	case OPLUS_CHG_GET_SUB_CURRENT:
		ret = sprintf(src, "sub_current=%d\n", b1_curr);
		break;
	case OPLUS_CHG_GET_SUB_VOLTAGE:
		ret = sprintf(src, "sub_voltage=%d\n", b1_volt);
		break;
	case OPLUS_CHG_GET_SUB_SOC:
		ret = sprintf(src, "sub_soc=%d\n", b1_soc);
		break;
	case OPLUS_CHG_GET_SUB_TEMPERATURE:
		ret = sprintf(src, "sub_temperature=%d\n", b1_temp);
		break;
	case OPLUS_CHG_GET_PARALLEL_SUPPORT:
		ret = sprintf(src, "support=%d\n", SERIES_FORM);
		break;
	case OPLUS_CHG_GET_BAL_CURR:
		ret = sprintf(src, "bal_curr=%d\n", chip->target_iref);
		break;
	default:
		return -EINVAL;
	}

	if (copy_to_user((void __user *)value, src, ret))
		ret = -EFAULT;

	return ret;
}

static int oplus_chg_open(struct inode *ip, struct file *fp)
{
	return 0;
}

static int oplus_chg_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static const struct file_operations oplus_chg_fops = {
	.owner = THIS_MODULE,
	.read = oplus_chg_read,
	.write = oplus_chg_write,
	.unlocked_ioctl = oplus_chg_ioctl,
	.open = oplus_chg_open,
	.release = oplus_chg_release,
};

static int oplus_batt_bal_parse_strategy_dt(struct oplus_batt_bal_chip *chip)
{
	int rc;
	struct device_node *node = NULL;
	struct oplus_batt_bal_cfg *cfg = &chip->cfg;

	node = chip->dev->of_node;
	rc = of_property_read_string(node, "oplus,bal_strategy_name",
							(const char **)&cfg->strategy_name);
	if (rc >= 0) {
		chg_info("strategy_name=%s\n", cfg->strategy_name);
		rc = oplus_chg_strategy_read_data(chip->dev,
			"oplus,bal_b1_strategy_data", &cfg->b1_strategy_data);
		if (rc < 0) {
			chg_err("read oplus,bal_b1_strategy_data failed, rc=%d\n", rc);
			cfg->b1_strategy_data = NULL;
			cfg->b1_strategy_data_size = 0;
		} else {
			chg_info("oplus,bal_b1_strategy_data size is %d\n", rc);
			cfg->b1_strategy_data_size = rc;
			/* bal_temp_region + temp_type + anti_thr */
			if (cfg->b1_strategy_data_size > ((chip->cfg.bal_temp_region_count + 2) * sizeof(u32))) {
				chg_err("oplus,bal_b1_strategy_data size err\n");
				if (cfg->b1_strategy_data)
					devm_kfree(chip->dev, cfg->b1_strategy_data);
				cfg->b1_strategy_data = NULL;
				cfg->b1_strategy_data_size = 0;
				return -EINVAL;
			}
		}

		rc = oplus_chg_strategy_read_data(chip->dev,
			"oplus,bal_b2_strategy_data", &cfg->b2_strategy_data);
		if (rc < 0) {
			chg_err("read oplus,bal_b2_strategy_data failed, rc=%d\n", rc);
			cfg->b2_strategy_data = NULL;
			cfg->b2_strategy_data_size = 0;
		} else {
			chg_info("oplus,bal_b2_strategy_data size is %d\n", rc);
			cfg->b2_strategy_data_size = rc;
			/* bal_temp_region + temp_type + anti_thr */
			if (cfg->b2_strategy_data_size > ((chip->cfg.bal_temp_region_count + 2) * sizeof(u32))) {
				chg_err("oplus,bal_b2_strategy_data size err\n");
				if (cfg->b2_strategy_data)
					devm_kfree(chip->dev, cfg->b2_strategy_data);
				cfg->b2_strategy_data = NULL;
				cfg->b2_strategy_data_size = 0;
				return -EINVAL;
			}
		}
	}

	return 0;
}

static void oplus_bal_track_err_report(struct oplus_batt_bal_chip *chip, int type)
{
	if (!chip)
		return;

	chip->track_err_type = type;
	schedule_work(&chip->bal_track_work);
}

static int oplus_batt_bal_parse_curve_dt(struct oplus_batt_bal_chip *chip)
{
	int rc;
	int i;
	int j;
	int k = 0;
	int length;
	struct device_node *node = NULL;
	struct device_node *temp_node = NULL;
	char name[BATT_BAL_CURVE_NAME_LEN] = {0};
	struct bal_bat_curve_table **batt;

	node = chip->dev->of_node;
	for (k =0; k < 2; k++) {
		memset(name, 0, sizeof(name));
		sprintf(name, "b%d_curve_table", k + 1);
		temp_node = of_get_child_by_name(node, name);
		if (!temp_node) {
			chg_err("%s node null\n", name);
			return -ENODEV;
		}
		if (!k)
			batt = &(chip->cfg.b1);
		else
			batt = &(chip->cfg.b2);
		*batt = devm_kzalloc(
				chip->dev, (chip->cfg.bal_temp_region_count + 1) * sizeof(struct bal_bat_curve_table),
				GFP_KERNEL);
		if (*batt == NULL) {
			chg_err("kzalloc error\n");
			return -ENOMEM;
		}

		if (chip->cfg.bal_temp_region_count > BATT_TEMP_REGION_MAX) {
			chg_err("Size not compatible ,bal_temp_region_count:%d, max_batt_bal_temp_table:%d",
				chip->cfg.bal_temp_region_count, BATT_TEMP_REGION_MAX);
			return -EINVAL;
		}

		for (i = 0; i < chip->cfg.bal_temp_region_count; i++) {
			rc = of_property_count_elems_of_size(temp_node, batt_bal_temp_table[i], sizeof(u32));
			if (rc > 0 && rc % (sizeof(struct batt_bal_spec)/sizeof(int)) == 0) {
				length = rc;
				(*batt)[i].length = length / (sizeof(struct batt_bal_spec) / sizeof(int));
				(*batt)[i].curve_table = devm_kzalloc(chip->dev, length * sizeof(struct batt_bal_spec), GFP_KERNEL);
				if ((*batt)[i].curve_table) {
					rc = of_property_read_u32_array(temp_node, batt_bal_temp_table[i],
						(u32 *)(*batt)[i].curve_table, length);
					if (rc < 0) {
						chg_err("parse %s failed, rc=%d\n", name, rc);
						(*batt)[i].length = 0;
						devm_kfree(chip->dev, (*batt)[i].curve_table);
					} else {
						chg_info("%s length =%d\n", batt_bal_temp_table[i], (*batt)[i].length);
						for (j = 0; j < (*batt)[i].length; j++)
							chg_info("b%d vbatt: %d curr: %d\n", k + 1,
								(*batt)[i].curve_table[j].volt, (*batt)[i].curve_table[j].curr);
					}
				}
			}
		}
	}

	return 0;
}

static int oplus_batt_bal_parse_dt(struct oplus_batt_bal_chip *chip)
{
	int i;
	int rc;
	struct device_node *node = NULL;
	int temp_region_count;

	if (!chip || !chip->dev) {
		chg_err("oplus_mos_dev null!\n");
		return -1;
	}

	node = chip->dev->of_node;

	rc = of_property_read_u32(node, "oplus,bal-temp-region-count", &chip->cfg.bal_temp_region_count);
	if (rc) {
		chip->cfg.bal_temp_region_count = BATT_BAL_MAX_TEMP_COUNT_DFT;
		chg_info("use default temp region count: %d\n", chip->cfg.bal_temp_region_count);
	} else {
		if (chip->cfg.bal_temp_region_count > BATT_TEMP_REGION_MAX) {
			chip->cfg.bal_temp_region_count = BATT_TEMP_REGION_MAX;
			chg_err("bal_temp_region_count %d > max limit %d",
				chip->cfg.bal_temp_region_count, BATT_TEMP_REGION_MAX);
		}
		chg_info("set temp region count as %d\n", chip->cfg.bal_temp_region_count);
	}

	temp_region_count = chip->cfg.bal_temp_region_count;
	rc = of_property_read_u32(node, "oplus,bal-dischg-volt-diff-thr", &chip->cfg.dischg_volt_diff_thr);
	if (rc)
		chip->cfg.dischg_volt_diff_thr = 2000;
	rc = of_property_read_u32(node, "oplus,bal-dischg-volt-diff-anti-thr", &chip->cfg.dischg_volt_diff_anti_thr);
	if (rc)
		chip->cfg.dischg_volt_diff_anti_thr = 50;

	rc = of_property_read_u32(node, "oplus,bal-dischg-curr-thr", &chip->cfg.dischg_curr_thr);
	if (rc)
		chip->cfg.dischg_curr_thr = 500;

	rc = of_property_read_u32(node, "oplus,bal-max-iref-thr", &chip->cfg.bal_max_iref_thr);
	if (rc)
		chip->cfg.bal_max_iref_thr = 1600;

	rc = of_property_read_u32(node, "oplus,bal-iterm", &chip->cfg.bal_iterm);
	if (rc)
		chip->cfg.bal_iterm = 50;

	rc = of_property_read_u32(node, "oplus,bal-b1-design-cap", &chip->cfg.b1_design_cap);
	if (rc)
		chip->cfg.b1_design_cap = 2000;
	rc = of_property_read_u32(node, "oplus,bal-b2-design-cap", &chip->cfg.b2_design_cap);
	if (rc)
		chip->cfg.b2_design_cap = 2800;

	rc = of_property_read_u32(node, "oplus,bal-lph-dischg-volt-diff-thr", &chip->cfg.lph_dischg_volt_diff_thr);
	if (rc)
		chip->cfg.lph_dischg_volt_diff_thr = 60;
	rc = of_property_read_u32(node, "oplus,bal-lph-dischg-volt-diff-hold", &chip->cfg.lph_dischg_volt_diff_hold);
	if (rc)
		chip->cfg.lph_dischg_volt_diff_hold = 30;

	rc = of_property_read_u32(node, "oplus,bal-fastchg-b2-to-b1-max-curr-thr",
		&chip->cfg.fastchg_b2_to_b1_max_curr_thr);
	if (rc)
		chip->cfg.fastchg_b2_to_b1_max_curr_thr = 4000;
	rc = of_property_read_u32(node, "oplus,bal-fastchg-b2-to-b1-max-curr-anti-thr",
		&chip->cfg.fastchg_b2_to_b1_max_curr_anti_thr);
	if (rc)
		chip->cfg.fastchg_b2_to_b1_max_curr_anti_thr = 500;

	rc = read_signed_data_from_node(node, "oplus,bal-volt-diff-thr",
					(s32 *)chip->cfg.chg_volt_diff_thr, temp_region_count);
	if (rc < 0) {
		chg_err("get oplus,bal-volt-diff-thr, rc=%d\n", rc);
		for (i = 0;i < temp_region_count; i++)
			chip->cfg.chg_volt_diff_thr[i] = 5;
	}

	rc = read_signed_data_from_node(node, "oplus,bal-curr-thr",
					(s32 *)chip->cfg.chg_curr_thr, temp_region_count);
	if (rc < 0) {
		chg_err("get oplus,bal-curr-thr, rc=%d\n", rc);
		for (i = 0; i < temp_region_count; i++)
			chip->cfg.chg_curr_thr[i] = 100;
	}

	rc = read_signed_data_from_node(node, "oplus,bal-vout-thr",
					(s32 *)chip->cfg.vout_thr, CTRL_REGION_MAX);
	if (rc < 0) {
		chg_err("get oplus,bal-vout-thr, rc=%d\n", rc);
		for (i = 0; i < CTRL_REGION_MAX; i++)
			chip->cfg.vout_thr[i] = 4488;
	}

	rc = read_signed_data_from_node(node, "oplus,bal-curr-over-thr",
					(s32 *)chip->cfg.curr_over_thr, CTRL_REGION_MAX);
	if (rc < 0) {
		chg_err("get oplus,bal-vout-thr, rc=%d\n", rc);
		for (i = 0; i < CTRL_REGION_MAX; i++)
			chip->cfg.curr_over_thr[i] = 50;
	}

	rc = read_signed_data_from_node(node, "oplus,bal-curr-below-thr",
					(s32 *)chip->cfg.curr_below_thr, CTRL_REGION_MAX);
	if (rc < 0) {
		chg_err("get oplus,bal-curr-below-thr, rc=%d\n", rc);
		for (i = 0; i < CTRL_REGION_MAX; i++)
			chip->cfg.curr_below_thr[i] = 50;
	}

	rc = oplus_batt_bal_parse_curve_dt(chip);
	rc |= oplus_batt_bal_parse_strategy_dt(chip);

	return rc;
}

static int oplus_batt_bal_run_interval_update_vote_callback(
	struct votable *votable, void *data, int time_ms, const char *client, bool step)
{
	int run_interval_update;
	struct oplus_batt_bal_chip *chip = data;

	if (time_ms < 0) {
		chg_err("time_ms=%d, restore default run update interval\n", time_ms);
		run_interval_update = BATT_BAL_TIME_MS(5000);
	} else {
		run_interval_update = time_ms;
		chg_info("set run update interval to %d\n", time_ms);
	}

	chip->run_interval_update = run_interval_update;
	return 0;
}

static void oplus_batt_bal_variables_reset(struct oplus_batt_bal_chip * chip)
{
	chip->curr_bal_state = OPLUS_BATT_BAL_STATE_DISCHG;
	chip->pre_bal_state = OPLUS_BATT_BAL_STATE_INIT;
	chip->abnormal_state = BATT_BAL_NO_ABNORMAL;
	chip->curr_over.not_allow_limit_jiffies = jiffies;
}

static int oplus_batt_bal_init(struct oplus_batt_bal_chip *chip)
{
	int rc;

	if (!chip) {
		chg_err("oplus_batt_bal_chip not specified!\n");
		return -EINVAL;
	}

	mutex_init(&chip->state_lock);
	mutex_init(&chip->update_batt_info_lock);
	mutex_init(&chip->cfg_lock);
	oplus_batt_bal_variables_reset(chip);
	rc = oplus_batt_bal_parse_dt(chip);
	if(rc < 0)
		return rc;

	chip->run_interval_update_votable =
		create_votable("BAL_RUN_TNTERVAL_UPDATE", VOTE_MIN,
			       oplus_batt_bal_run_interval_update_vote_callback, chip);
	if (IS_ERR(chip->run_interval_update_votable)) {
		rc = PTR_ERR(chip->run_interval_update_votable);
		chip->run_interval_update_votable = NULL;
		return rc;
	}

	return rc;
}

static void oplus_batt_bal_err_handler(struct oplus_chg_ic_dev *ic_dev,
					void *virq_data)
{
	struct oplus_batt_bal_chip *chip = virq_data;

	schedule_work(&chip->err_handler_work);
	return;
}

static int oplus_batt_bal_virq_register(struct oplus_batt_bal_chip *chip)
{
	int rc;

	rc = oplus_chg_ic_virq_register(chip->ic_dev, OPLUS_IC_VIRQ_ERR,
		oplus_batt_bal_err_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);

	return 0;
}

static int oplus_batt_bal_get_enable(struct oplus_batt_bal_chip *chip)
{
	bool en;
	int rc;

	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->ic_dev,
			       OPLUS_IC_FUNC_BAL_GET_ENABLE, &en);
	if (rc < 0) {
		chg_err("error get bal enable rc=%d\n", rc);
		return false;
	}

	return en;
}

static int oplus_batt_bal_get_pmos_enable(struct oplus_batt_bal_chip *chip)
{
	bool en;
	int rc;

	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->ic_dev,
			       OPLUS_IC_FUNC_BAL_GET_PMOS_ENABLE, &en);
	if (rc < 0) {
		chg_err("error get pmos enable rc=%d\n", rc);
		return false;
	}

	return en;
}

int oplus_batt_bal_pmos_disable(struct oplus_mms *topic)
{
	int rc;
	struct oplus_batt_bal_chip *chip;

	if (topic == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(topic);
	rc = oplus_chg_ic_func(chip->ic_dev,
			       OPLUS_IC_FUNC_BAL_SET_PMOS_ENABLE, false);
	if (rc < 0)
		chg_err("error set pmos disable rc=%d\n", rc);

	return rc;
}

static int oplus_batt_bal_set_pmos_enable(struct oplus_batt_bal_chip *chip, bool en)
{
	int rc;

	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->ic_dev,
			       OPLUS_IC_FUNC_BAL_SET_PMOS_ENABLE, en);
	if (rc < 0)
		chg_err("error set pmos enable rc=%d\n", rc);

	return rc;
}

static int oplus_batt_bal_set_hw_enable(struct oplus_batt_bal_chip *chip, bool en)
{
	int rc;

	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->ic_dev,
			       OPLUS_IC_FUNC_BAL_SET_HW_ENABLE, en);
	if (rc < 0)
		chg_err("error set hw enable rc=%d\n", rc);

	return rc;
}

static int oplus_batt_bal_set_conver_enable(struct oplus_batt_bal_chip *chip, bool en)
{
	int rc;

	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->ic_dev,
			       OPLUS_IC_FUNC_BAL_SET_CONVER_ENABLE, en);
	if (rc < 0)
		chg_err("error set conver enable rc=%d\n", rc);

	return rc;
}

static int oplus_batt_bal_set_vout(struct oplus_batt_bal_chip *chip, int vout)
{
	int rc;

	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->ic_dev,
			       OPLUS_IC_FUNC_BAL_SET_VOUT, vout);
	if (rc < 0)
		chg_err("error set vout rc=%d\n", rc);

	return rc;
}

static int oplus_batt_bal_set_iterm(struct oplus_batt_bal_chip *chip, int iterm)
{
	int rc;

	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->ic_dev,
			       OPLUS_IC_FUNC_BAL_SET_ITERM, iterm);
	if (rc < 0)
		chg_err("error set iterm rc=%d\n", rc);

	return rc;
}

static int oplus_batt_bal_set_iref(struct oplus_batt_bal_chip *chip, int iref)
{
	int rc;

	if (!chip)
		return -EINVAL;

	if (iref > chip->cfg.bal_max_iref_thr) {
		iref = chip->cfg.bal_max_iref_thr;
		chip->target_iref = iref;
	}

	rc = oplus_chg_ic_func(chip->ic_dev,
			       OPLUS_IC_FUNC_BAL_SET_IREF, iref);
	if (rc < 0)
		chg_err("error set iref rc=%d\n", rc);

	return rc;
}

static int oplus_batt_bal_set_flow_dir(struct oplus_batt_bal_chip *chip, int flow_dir)
{
	int rc;

	if (!chip)
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->ic_dev,
			       OPLUS_IC_FUNC_BAL_SET_FLOW_DIR, flow_dir);
	if (rc < 0)
		chg_err("error set iref rc=%d\n", rc);

	return rc;
}

static int oplus_batt_bal_reg_dump(struct oplus_batt_bal_chip *chip)
{
	int rc;

	if (!chip)
		return -EINVAL;

	if (!oplus_batt_bal_get_enable(chip))
		return -EINVAL;

	rc = oplus_chg_ic_func(chip->ic_dev, OPLUS_IC_FUNC_REG_DUMP);
	if (rc < 0)
		chg_err("error reg dump rc=%d\n", rc);

	return rc;
}

static int oplus_batt_bal_status_record(struct oplus_batt_bal_chip *chip)
{
	int index = 0;
	int pmos_status = oplus_batt_bal_get_pmos_enable(chip);
	int hw_status = oplus_batt_bal_get_enable(chip);

	index += snprintf(&(chip->status_record[index]), BATT_BAL_STATUS_RECORD_LEN - index,
		"pmos_status=%d;hw_status=%d;dir_flow=%s;target_iref=%d;",
		pmos_status, hw_status, oplus_batt_bal_dir_flow_text[chip->flow_dir], chip->target_iref);
	index += snprintf(&(chip->status_record[index]), BATT_BAL_STATUS_RECORD_LEN - index,
		"b1_vol=%d;b2_vol=%d;b1_curr=%d;b2_curr=%d;",
		chip->b1_volt, chip->b2_volt, chip->b1_curr, chip->b2_curr);
	index += snprintf(&(chip->status_record[index]), BATT_BAL_STATUS_RECORD_LEN - index,
		"vbatt_diff=%d;cbatt_diff=%d;state_machine=%d;abnormal_state=%d\n",
		chip->b2_volt - chip->b1_volt, (chip->b2_curr - chip->b1_curr) / 2,
		chip->curr_bal_state, chip->abnormal_state);

	chg_debug("index=%d\n", index);

	return index;
}

#define FULL_VOLT_GAP 50
#define IREF_CNT_MAX 3
#define IREF_ACC_ERR 300
static void oplus_batt_bal_check_track(struct oplus_batt_bal_chip *chip)
{
	union mms_msg_data data = { 0 };
	int temp_region;
	bool batt_full;
	static int last_iref;
	static unsigned long iref_jiffies;
	static int iref_cnt;
	static bool iref_err_report;
	int bal_curr;

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_TEMP_REGION, &data, false);
	temp_region = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_CHG_FULL, &data, false);
	batt_full = !!data.intval;
	if (batt_full && (temp_region > TEMP_REGION_COLD) && (temp_region < TEMP_REGION_WARM) &&
	    !chip->wired_charging_enable && !chip->wls_charging_enable &&
	    !chip->full_err_report && abs(chip->b2_volt - chip->b1_volt) > FULL_VOLT_GAP) {
		chip->full_err_report = true;
		oplus_bal_track_err_report(chip, VOL_GAP_WHEN_FULL);
	}
	if (chip->flow_dir == DEFAULT_DIR) {
		iref_cnt = 0;
		iref_err_report = false;
		return;
	}
	if (!chip->wired_online && !chip->wls_online)
		return;
	bal_curr = abs(chip->b1_curr - chip->b2_curr) / 2;
	if (last_iref != chip->target_iref) {
		last_iref = chip->target_iref;
		iref_jiffies = jiffies + (unsigned long)(5 * HZ);
		iref_cnt = 0;
	} else if (abs(chip->target_iref - bal_curr) > IREF_ACC_ERR) {
		if (iref_cnt < IREF_CNT_MAX) {
			iref_cnt++;
		} else if (time_is_before_jiffies(iref_jiffies) && !iref_err_report) {
			iref_err_report = true;
			oplus_bal_track_err_report(chip, BAL_CURR_ACC_ERR);
		}
	} else {
		iref_cnt = 0;
	}

	return;
}

static int oplus_batt_bal_other(struct oplus_batt_bal_chip *chip)
{
	int rc;

	if (!chip)
		return -EINVAL;

	rc = oplus_batt_bal_reg_dump(chip);
	oplus_batt_bal_status_record(chip);
	oplus_batt_bal_check_track(chip);

	return rc;
}

static int oplus_batt_bal_cfg(
	struct oplus_batt_bal_chip *chip, bool pmos_en,
	bool hw_en, int flow_dir, int target_iref, int vout)
{
	int rc = 0;

	if (!chip)
		return -EINVAL;
	if (chip->abnormal_state != BATT_BAL_NO_ABNORMAL &&
	    (pmos_en == true || hw_en == true))
		return rc;

	oplus_wired_set_supplementary_power_mos(chip->wired_topic, pmos_en);

	mutex_lock(&chip->cfg_lock);
	if (flow_dir == DEFAULT_DIR) {
		rc |= oplus_batt_bal_set_hw_enable(chip, hw_en);
		rc |= oplus_batt_bal_set_pmos_enable(chip, pmos_en);
	} else {
		rc |= oplus_batt_bal_set_hw_enable(chip, hw_en);
		rc |= oplus_batt_bal_set_pmos_enable(chip, pmos_en);

		if (vout)
			rc |= oplus_batt_bal_set_vout(chip, vout);

		rc |= oplus_batt_bal_set_flow_dir(chip, flow_dir);
		if (rc < 0) {
			mutex_unlock(&chip->cfg_lock);
			return rc;
		}
		rc |= oplus_batt_bal_set_iref(chip, target_iref);
		rc |= oplus_batt_bal_set_iterm(chip, chip->cfg.bal_iterm);

		if (target_iref) {
			if (!oplus_batt_bal_get_enable(chip))
				rc |= oplus_batt_bal_set_conver_enable(chip, hw_en);
		} else {
			rc |= oplus_batt_bal_set_conver_enable(chip, false);
		}
	}

	chip->flow_dir = flow_dir;
	mutex_unlock(&chip->cfg_lock);

	return rc;
}

static int oplus_batt_bal_update_enable(
	struct oplus_mms *topic, union mms_msg_data *data)
{
	struct oplus_batt_bal_chip *chip;
	int status;

	if (topic == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(topic);
	status = oplus_batt_bal_get_enable(chip);
	if (status < 0)
		data->intval = 0;
	else
		data->intval = !!status;

	return 0;
}

static int oplus_batt_bal_update_abnormal_state(
	struct oplus_mms *topic, union mms_msg_data *data)
{
	struct oplus_batt_bal_chip *chip;
	int abnormal_state = BATT_BAL_NO_ABNORMAL;

	if (topic == NULL) {
		chg_err("topic is NULL");
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(topic);

	abnormal_state = chip->abnormal_state;

end:
	data->intval = abnormal_state;
	return 0;
}

static int oplus_batt_bal_update_status(
			struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_batt_bal_chip *chip;
	int rc = 0;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}

	chip = oplus_mms_get_drvdata(mms);
	data->strval = chip->status_record;

	return rc;
}

static int oplus_batt_bal_update_curr_limit(
	struct oplus_mms *topic, union mms_msg_data *data)
{
	struct oplus_batt_bal_chip *chip;
	int curr_limit = 0;

	if (topic == NULL) {
		chg_err("topic is NULL");
		goto end;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(topic);

	curr_limit = chip->curr_over.curr_limit;

end:
	data->intval = curr_limit;
	return 0;
}


static void oplus_mms_batt_bal_update(struct oplus_mms *mms, bool publish)
{
	struct mms_msg *msg;
	int i, rc;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return;
	}

	for (i = 0; i < mms->desc->update_items_num; i++)
		oplus_mms_item_update(mms, mms->desc->update_items[i], true);
	if (publish) {
		msg = oplus_mms_alloc_msg(MSG_TYPE_TIMER, MSG_PRIO_MEDIUM, 0);
		if (msg == NULL) {
			chg_err("alloc msg buf error\n");
			return;
		}
		rc = oplus_mms_publish_msg(mms, msg);
		if (rc < 0) {
			chg_err("publish msg error, rc=%d\n", rc);
			kfree(msg);
			return;
		}
	}
}

static struct mms_item oplus_chg_batt_bal_item[] = {
	{
		.desc = {
			.item_id = BATT_BAL_ITEM_ENABLE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_batt_bal_update_enable,
		}
	},
	{
		.desc = {
			.item_id = BATT_BAL_ITEM_CURR_LIMIT,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_batt_bal_update_curr_limit,
		}
	},
	{
		.desc = {
			.item_id = BATT_BAL_ITEM_ABNORMAL_STATE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_batt_bal_update_abnormal_state,
		}
	},
	{
		.desc = {
			.item_id = BATT_BAL_ITEM_STATUS,
			.str_data = true,
			.update = oplus_batt_bal_update_status,
		}
	},
	{
		.desc = {
			.item_id = BATT_BAL_ITEM_LCF_ALARM,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	}
};

static const u32 oplus_batt_bal_update_item[] = {
	BATT_BAL_ITEM_ENABLE,
};

static const struct oplus_mms_desc oplus_mms_batt_bal_desc = {
	.name = "batt_bal",
	.type = OPLUS_MMS_TYPE_BATT_BAL,
	.item_table = oplus_chg_batt_bal_item,
	.item_num = ARRAY_SIZE(oplus_chg_batt_bal_item),
	.update_items = oplus_batt_bal_update_item,
	.update_items_num = ARRAY_SIZE(oplus_batt_bal_update_item),
	.update_interval = 0, /* ms */
	.update = oplus_mms_batt_bal_update,
};

static void oplus_batt_bal_check_batt_temp_region(
	struct oplus_batt_bal_chip *chip)
{
	int rc = 0;

	if (!chip)
		return;

	if (chip->b1_inr_strategy) {
		rc = oplus_chg_strategy_get_data(
			chip->b1_inr_strategy, &chip->b1_temp_region);
		if (rc < 0)
			chg_err("can't get b1_temp_region, rc=%d\n", rc);
	}

	if (chip->b2_inr_strategy) {
		rc = oplus_chg_strategy_get_data(
			chip->b2_inr_strategy, &chip->b2_temp_region);
		if (rc < 0)
			chg_err("can't get b2_temp_region, rc=%d\n", rc);
	}

	if (chip->b1_temp_region >= chip->cfg.bal_temp_region_count)
		chip->b1_temp_region = chip->cfg.bal_temp_region_count - 1;

	if (chip->b2_temp_region >= chip->cfg.bal_temp_region_count)
		chip->b2_temp_region = chip->cfg.bal_temp_region_count - 1;
}

static int oplus_batt_bal_update_batt_info(struct oplus_batt_bal_chip *chip)
{
	union mms_msg_data data = { 0 };

	if (IS_ERR_OR_NULL(chip->main_gauge_topic) ||
	    IS_ERR_OR_NULL(chip->sub_gauge_topic)) {
		chg_err("wait gauge toptic\n");
		return -EINVAL;
	}

	mutex_lock(&chip->update_batt_info_lock);
	oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	chip->b2_volt = data.intval;
	oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_CURR, &data, true);
	chip->b2_curr = -data.intval;
	oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_TEMP, &data, true);
	chip->b2_temp = data.intval;
	oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_SOC, &data, true);
	chip->b2_soc = data.intval;

	oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	chip->b1_volt = data.intval;
	oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_CURR, &data, true);
	chip->b1_curr = -data.intval;
	oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_TEMP, &data, true);
	chip->b1_temp = data.intval;
	oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_SOC, &data, true);
	chip->b1_soc = data.intval;
	oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_VOL_MIN, &data, true);
	mutex_unlock(&chip->update_batt_info_lock);

	return 0;
}

static void oplus_batt_bal_update_run_interval_work(struct work_struct *work)
{
	struct oplus_batt_bal_chip *chip =
		container_of(work, struct oplus_batt_bal_chip, update_run_interval_work);

	if ((chip->vooc_fastchg_ing && !chip->vooc_by_normal_path) ||
	     chip->pps_fastchg_ing || chip->ufcs_fastchg_ing || chip->wls_fastchg_ing)
		vote(chip->run_interval_update_votable, FASTCHG_VOTER, true, BATT_BAL_TIME_MS(500), false);
	else
		vote(chip->run_interval_update_votable, FASTCHG_VOTER, false, 0, false);

	if (chip->wired_online || chip->wls_online) {
		vote(chip->run_interval_update_votable, USER_VOTER, true, BATT_BAL_TIME_MS(2000), false);
	} else {
		vote(chip->run_interval_update_votable, USER_VOTER, false, 0, false);
	}
}

static void oplus_batt_bal_update_affect_state_update_args(struct oplus_batt_bal_chip *chip)
{
	union mms_msg_data data = { 0 };

	if (chip->wired_topic) {
		oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data, false);
		chip->wired_online = data.intval;
		oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHARGING_DISABLE, &data, false);
		chip->wired_charging_enable = !data.intval;
	}

	if (chip->wls_topic) {
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_ONLINE, &data, false);
		chip->wls_online = data.intval;
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_CHARGING_DISABLE, &data, false);
		chip->wls_charging_enable = !data.intval;
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_FASTCHG_STATUS, &data, false);
		chip->wls_fastchg_ing = !!data.intval;
	}

	if (chip->vooc_topic) {
		oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_STARTED, &data, false);
		chip->vooc_fastchg_ing = data.intval;
		oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_BY_NORMAL_PATH, &data, false);
		chip->vooc_by_normal_path = !!data.intval;
	}

	if (chip->pps_topic) {
		oplus_mms_get_item_data(chip->pps_topic, PPS_ITEM_CHARGING, &data, false);
		chip->pps_fastchg_ing = !!data.intval;
	}

	if (chip->ufcs_topic) {
		oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_CHARGING, &data, false);
		chip->ufcs_fastchg_ing = !!data.intval;
	}

	if (chip->comm_topic) {
		oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_FFC_STATUS, &data, false);
		chip->ffc_status = data.intval;
	}
}

static void oplus_batt_bal_state_daemon(struct oplus_batt_bal_chip *chip)
{
	enum oplus_batt_bal_state target_bal_state;

	mutex_lock(&chip->state_lock);
	oplus_batt_bal_update_affect_state_update_args(chip);
	if ((!chip->wired_online && !chip->wls_online) ||
		chip->ffc_status == FFC_WAIT || chip->ffc_status == FFC_IDLE)
		target_bal_state = OPLUS_BATT_BAL_STATE_DISCHG;
	else if ((chip->vooc_fastchg_ing && !chip->vooc_by_normal_path) ||
		chip->pps_fastchg_ing || chip->ufcs_fastchg_ing || chip->wls_fastchg_ing)
		target_bal_state = OPLUS_BATT_BAL_STATE_FAST_START;
	else if (chip->ffc_status == FFC_FAST)
		target_bal_state = OPLUS_BATT_BAL_STATE_FFC_CHG;
	else if ((chip->wired_online && chip->wired_charging_enable) ||
		(chip->wls_online && chip->wls_charging_enable))
		target_bal_state = OPLUS_BATT_BAL_STATE_NORMAL_CHG;
	else
		target_bal_state = OPLUS_BATT_BAL_STATE_DISCHG;

	if (chip->abnormal_state ||
	   (atomic_read(&chip->ic_trig_abnormal) && target_bal_state != OPLUS_BATT_BAL_STATE_DISCHG))
		target_bal_state = OPLUS_BATT_BAL_STATE_ABNORMAL;

	if ((chip->curr_bal_state == OPLUS_BATT_BAL_STATE_LPH_DISCHG &&
	    target_bal_state == OPLUS_BATT_BAL_STATE_DISCHG) ||
	    ((chip->curr_bal_state >= OPLUS_BATT_BAL_STATE_FAST_BAL_START &&
	    chip->curr_bal_state <= OPLUS_BATT_BAL_STATE_FAST_CC_CHG) &&
	    target_bal_state == OPLUS_BATT_BAL_STATE_FAST_START)) {
		chg_debug("continue to maintain current state\n");
	} else if (chip->curr_bal_state != target_bal_state) {
		chip->curr_bal_state = target_bal_state;
		chg_info("expect target_state=%s, current_state=%s\n",
			oplus_batt_bal_get_bal_state_str(target_bal_state),
			oplus_batt_bal_get_bal_state_str(chip->curr_bal_state));
	}
}

static void oplus_batt_bal_update_bal_state_work(struct work_struct *work)
{
	enum oplus_batt_bal_state target_bal_state;
	struct oplus_batt_bal_chip *chip =
		container_of(work, struct oplus_batt_bal_chip, update_bal_state_work);

	oplus_batt_bal_update_run_interval_work(&chip->update_run_interval_work);

	mutex_lock(&chip->state_lock);
	oplus_batt_bal_update_affect_state_update_args(chip);
	if ((!chip->wired_online && !chip->wls_online) ||
		chip->ffc_status == FFC_WAIT || chip->ffc_status == FFC_IDLE)
		target_bal_state = OPLUS_BATT_BAL_STATE_DISCHG;
	else if ((chip->vooc_fastchg_ing && !chip->vooc_by_normal_path) ||
		chip->pps_fastchg_ing || chip->ufcs_fastchg_ing || chip->wls_fastchg_ing)
		target_bal_state = OPLUS_BATT_BAL_STATE_FAST_START;
	else if (chip->ffc_status == FFC_FAST)
		target_bal_state = OPLUS_BATT_BAL_STATE_FFC_CHG;
	else if ((chip->wired_online && chip->wired_charging_enable) ||
		(chip->wls_online && chip->wls_charging_enable))
		target_bal_state = OPLUS_BATT_BAL_STATE_NORMAL_CHG;
	else
		target_bal_state = OPLUS_BATT_BAL_STATE_DISCHG;

	if (chip->abnormal_state ||
	   (atomic_read(&chip->ic_trig_abnormal) && target_bal_state != OPLUS_BATT_BAL_STATE_DISCHG))
		target_bal_state = OPLUS_BATT_BAL_STATE_ABNORMAL;

	chg_info("expect target_state=%s, current_state=%s\n",
		oplus_batt_bal_get_bal_state_str(target_bal_state),
		oplus_batt_bal_get_bal_state_str(chip->curr_bal_state));
	if ((chip->curr_bal_state == OPLUS_BATT_BAL_STATE_LPH_DISCHG &&
	    target_bal_state == OPLUS_BATT_BAL_STATE_DISCHG) ||
	    ((chip->curr_bal_state >= OPLUS_BATT_BAL_STATE_FAST_BAL_START &&
	    chip->curr_bal_state <= OPLUS_BATT_BAL_STATE_FAST_CC_CHG) &&
	    target_bal_state == OPLUS_BATT_BAL_STATE_FAST_START)) {
		chg_info("continue to maintain current state\n");
		mutex_unlock(&chip->state_lock);
	} else if (chip->curr_bal_state != target_bal_state) {
		chip->curr_bal_state = target_bal_state;
		mutex_unlock(&chip->state_lock);
		chg_info("state change\n");
		cancel_delayed_work_sync(&chip->batt_bal_sm_work);
		mod_delayed_work(system_highpri_wq, &chip->batt_bal_sm_work, 0);
	} else {
		mutex_unlock(&chip->state_lock);
	}
}

#define TRACK_UPLOAD_COUNT_MAX			10
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD	(24 * 3600)
#define REASON_LENGTH_MAX 256
static void oplus_batt_bal_track_work(struct work_struct *work)
{
	struct oplus_batt_bal_chip *chip =
		container_of(work, struct oplus_batt_bal_chip, bal_track_work);
	struct oplus_mms *err_topic;
	struct mms_msg *msg = NULL;
	int rc;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	int curr_time;
	char temp_str[REASON_LENGTH_MAX] = {0};
	int index = 0;

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return;
	}

	curr_time = oplus_chg_track_get_local_time_s();
	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;
	if (chip->track_err_type && upload_count < TRACK_UPLOAD_COUNT_MAX) {
		upload_count++;
		pre_upload_time = oplus_chg_track_get_local_time_s();
		chg_info("batt bal track trigger: %d\n", chip->track_err_type);
		switch(chip->track_err_type) {
		case VOL_GAP_BIG_IN_DISCHG:
			index += scnprintf(&(temp_str[index]), REASON_LENGTH_MAX - index,
					  "$$reason@@%s$$status@@", "vol_gap_big_in_dischg");
			break;
		case BAL_CURR_ACC_ERR:
			index += scnprintf(&(temp_str[index]), REASON_LENGTH_MAX - index,
					  "$$reason@@%s$$status@@", "bal_curr_accuracy_err");
			break;
		case VOL_GAP_WHEN_FULL:
			index += scnprintf(&(temp_str[index]), REASON_LENGTH_MAX - index,
					  "$$reason@@%s$$status@@", "vol_gap_when_full");
			break;
		default:
			break;
		}
		index += scnprintf(&(temp_str[index]), REASON_LENGTH_MAX - index,
				  "flow_dir:%s bal_curr:%d b1_vol:%d b1_curr:%d b1_soc:%d b1_temp:%d "
				  "b2_vol:%d b2_curr:%d b2_soc:%d b2_temp:%d PMOS:%d bal_en:%d",
				  oplus_batt_bal_dir_flow_text[chip->flow_dir], chip->target_iref,
				  chip->b1_volt, chip->b1_curr, chip->b1_soc, chip->b1_temp,
				  chip->b2_volt, chip->b2_curr, chip->b2_soc, chip->b2_temp,
				  oplus_batt_bal_get_pmos_enable(chip), oplus_batt_bal_get_enable(chip));
		msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_BAL_INFO,
					      temp_str);
		if (msg == NULL) {
			chg_err("alloc batt bal error msg error\n");
			return;
		}
		rc = oplus_mms_publish_msg_sync(err_topic, msg);
		if (rc < 0) {
			chg_err("publish batt bal error msg error, rc=%d\n", rc);
			kfree(msg);
		}
		chip->track_err_type = 0;
	}

	return;
}

static void oplus_batt_bal_set_curr_limit(
	struct oplus_batt_bal_chip *chip, int curr_limit)
{
	int rc;
	struct mms_msg *msg;

	if (chip->curr_over.curr_limit == curr_limit)
		return;

	chip->curr_over.curr_limit = curr_limit;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				BATT_BAL_ITEM_CURR_LIMIT);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->batt_bal_topic, msg);
	if (rc < 0) {
		chg_err("publish curr_limit msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("curr_limit = %d\n", chip->curr_over.curr_limit);
}

static int oplus_batt_bal_set_fcc(int fcc_curr, bool enable)
{
	struct votable *wired_fcc_votable;
	int rc;

	wired_fcc_votable = find_votable("WIRED_FCC");
	if (!wired_fcc_votable) {
		chg_err("WIRED_FCC votable not found\n");
		return -EINVAL;
	}

	rc = vote(wired_fcc_votable, CURR_LIMIT_VOTER, enable, fcc_curr, true);
	if (rc < 0)
		chg_err("vote fcc = %d rc = %d\n", fcc_curr, rc);

	return rc;
}

static void oplus_batt_bal_set_curr_over(
	struct oplus_batt_bal_chip *chip, bool trigger, int level_1, int level_2,
	enum batt_temp_region region_b1, enum batt_temp_region region_b2)
{
	int curr_limit;
	static int b1_limit_curr;
	static int b2_limit_curr;
	static int b1_level;
	static int b2_level;
	static enum batt_temp_region b1_region;
	static enum batt_temp_region b2_region;

	if (trigger) {
		if (time_is_before_jiffies(chip->curr_over.not_allow_limit_jiffies)) {
			chip->curr_over.not_allow_limit_jiffies = jiffies + (unsigned long)(5 * HZ);
			curr_limit = (chip->b1_curr + chip->b2_curr) / 2;
			curr_limit = BATT_BAL_LIMIT_CURR_MA(curr_limit);

			if (curr_limit <= BATT_BAL_CURR_MA(500))
				curr_limit = BATT_BAL_CURR_MA(500);
			if (chip->vooc_fastchg_ing || chip->pps_fastchg_ing ||
			    chip->ufcs_fastchg_ing || chip->wls_fastchg_ing)
				oplus_batt_bal_set_curr_limit(chip, curr_limit);
			else
				oplus_batt_bal_set_fcc(curr_limit * 2, true);
			b1_limit_curr = chip->curr_over.b1_limit_curr;
			b2_limit_curr = chip->curr_over.b2_limit_curr;
			b1_level = level_1;
			b2_level = level_2;
			b1_region = region_b1;
			b2_region = region_b2;
		}
	} else {
		/* temp region change or level up, need reset curr over spec check */
		if (b1_level < level_1 || b2_level < level_2 ||
		    b1_region != region_b1 || b2_region != region_b2) {
			oplus_batt_bal_set_curr_limit(chip, 0);
			oplus_batt_bal_set_fcc(0, false);
		}
		b1_limit_curr = chip->curr_over.b1_limit_curr;
		b2_limit_curr = chip->curr_over.b2_limit_curr;
		chip->curr_over.not_allow_limit_jiffies = jiffies;
	}
}

static void oplus_batt_bal_set_abnormal_state(
	struct oplus_batt_bal_chip *chip, int abnormal_state)
{
	struct mms_msg *msg;
	int rc;

	if (chip->abnormal_state == abnormal_state)
		return;

	chip->abnormal_state = abnormal_state;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  BATT_BAL_ITEM_ABNORMAL_STATE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->batt_bal_topic, msg);
	if (rc < 0) {
		chg_err("publish abnormal_state msg error, rc=%d\n", rc);
		kfree(msg);
	}

	chg_info("abnormal_state = %d\n", chip->abnormal_state);
}

static bool oplus_batt_bal_check_batt_curr_over(struct oplus_batt_bal_chip *chip)
{
	int i;
	bool curr_over;
	int b1_curr = chip->b1_curr;
	int b2_curr = chip->b2_curr;
	int level_1 = 0;
	int level_2 = 0;

	if (!chip->wired_online && !chip->wls_online) {
		chip->curr_over.b1_over = false;
		chip->curr_over.b2_over = false;
		return false;
	}

	if (!chip->cfg.b1 || !chip->cfg.b2 ||
	    !chip->cfg.b1[chip->b1_temp_region].curve_table ||
	    !chip->cfg.b2[chip->b2_temp_region].curve_table) {
		chip->curr_over.b1_over = false;
		chip->curr_over.b2_over = false;
		chg_err("batt spec not fond, limit to default curr\n");
		return false;
	}

	for (i = 0; i < chip->cfg.b1[chip->b1_temp_region].length; i++) {
		if (chip->b1_volt > chip->cfg.b1[chip->b1_temp_region].curve_table[i].volt)
			continue;

		if (b1_curr > chip->cfg.b1[chip->b1_temp_region].curve_table[i].curr)
			chip->curr_over.b1_over = true;
		else
			chip->curr_over.b1_over = false;
		chip->curr_over.b1_limit_curr = chip->cfg.b1[chip->b1_temp_region].curve_table[i].curr;
		level_1 = i;
		break;
	}
	if (i == chip->cfg.b1[chip->b1_temp_region].length) {
		chip->curr_over.b1_limit_curr = chip->cfg.b1[chip->b1_temp_region].curve_table[0].curr;
		level_1 = 0;
	}

	for (i = 0; i < chip->cfg.b2[chip->b2_temp_region].length; i++) {
		if (chip->b2_volt > chip->cfg.b2[chip->b2_temp_region].curve_table[i].volt)
			continue;

		if (b2_curr > chip->cfg.b2[chip->b2_temp_region].curve_table[i].curr)
			chip->curr_over.b2_over = true;
		else
			chip->curr_over.b2_over = false;
		chip->curr_over.b2_limit_curr = chip->cfg.b2[chip->b2_temp_region].curve_table[i].curr;
		level_2 = i;
		break;
	}
	if (i == chip->cfg.b2[chip->b2_temp_region].length) {
		chip->curr_over.b2_limit_curr = chip->cfg.b2[chip->b2_temp_region].curve_table[0].curr;
		level_2 = 0;
	}

	curr_over = (chip->curr_over.b1_over || chip->curr_over.b2_over);
	oplus_batt_bal_set_curr_over(chip, curr_over, level_1, level_2, chip->b1_temp_region, chip->b2_temp_region);

	return curr_over;
}

static int oplus_batt_bal_handle_state_abnormal(
	struct oplus_batt_bal_chip *chip)
{
	int rc = 0;

	rc = oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, 0, 0);

	return rc;
}

static int oplus_batt_bal_entry_state_dischg(struct oplus_batt_bal_chip *chip)
{
	int rc = 0;
	int volt_diff_thr = chip->cfg.dischg_volt_diff_thr;
	int vout = chip->cfg.vout_thr[CTRL_REGION_DISCHG];
	int curr_thr = chip->cfg.dischg_curr_thr;

	chip->target_iref = curr_thr;
	chip->lph_dischg_switch_count = 0;
	chip->eq_volt_diff_thr = volt_diff_thr;

	if (abs(chip->b2_volt - chip->b1_volt) <= volt_diff_thr)
		chip->sub_step_state = BAL_SUB_STATE_BAL;
	else if (chip->b1_volt - chip->b2_volt > volt_diff_thr)
		chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
	else
		chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;

	if (chip->sub_step_state == BAL_SUB_STATE_BAL)
		oplus_batt_bal_cfg(chip, true, false, DEFAULT_DIR, curr_thr, 0);
	else if (chip->sub_step_state == BAL_SUB_STATE_B1_TO_B2)
		oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, curr_thr, vout);
	else
		oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, curr_thr, vout);

	vote(chip->run_interval_update_votable, BAL_STATE_VOTER, false, 0, true);

	chg_info("sub_step_state=%d, volt_diff_thr=%d, target_iref=%d, vout=%d\n",
		chip->sub_step_state, volt_diff_thr, curr_thr, vout);

	return rc;
}

static bool oplus_batt_bal_switch_to_lph_dischg(
	struct oplus_batt_bal_chip *chip)
{
	bool rc = false;
	int term_voltage = 0;
	int vout = chip->cfg.vout_thr[CTRL_REGION_DISCHG];
	int volt_diff_thr = chip->cfg.lph_dischg_volt_diff_thr;
	int lph_dischg_vbatt_min = BATT_BAL_ENTER_LPH_DISCHG_VBATT;

	if (chip->deep_support && is_gauge_term_voltage_votable_available(chip)) {
		term_voltage = get_effective_result(chip->gauge_term_voltage_votable);
		if (term_voltage > 0)
			lph_dischg_vbatt_min = term_voltage + 300; /* todo switch volt set to term_voltage + 300mv */
	}

	chg_debug("lph_dischg_vbatt_min=%d\n", lph_dischg_vbatt_min);
	if (!atomic_read(&chip->ic_trig_abnormal) && (abs(chip->b2_volt - chip->b1_volt) > volt_diff_thr) &&
	   (chip->b2_volt < lph_dischg_vbatt_min || chip->b1_volt < lph_dischg_vbatt_min)) {
		chip->lph_dischg_switch_count++;
		if (chip->lph_dischg_switch_count > BATT_BAL_LPH_DISCHG_SWITCH_COUNT) {
			chip->lph_dischg_switch_count = 0;
			chip->curr_bal_state = OPLUS_BATT_BAL_STATE_LPH_DISCHG;
			vote(chip->run_interval_update_votable, BAL_STATE_VOTER,
				true, BATT_BAL_TIME_MS(10), true);
			chip->target_iref = BATT_BAL_CURR_MA(25);
			if (chip->b2_volt > chip->b1_volt) {
				chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
				oplus_batt_bal_cfg(chip, true, true, B2_TO_B1, chip->target_iref, vout);
			} else {
				chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
				oplus_batt_bal_cfg(chip, true, true, B1_TO_B2, chip->target_iref, vout);
			}
			chg_info("state switch to lph_dischg\n");
			rc = true;
		}
	} else {
		chip->lph_dischg_switch_count = 0;
	}

	return rc;
}

static bool oplus_batt_bal_lph_dischg_switch_to_dischg(
	struct oplus_batt_bal_chip *chip)
{
	int term_voltage = 0;
	int lph_dischg_exit_vbatt = BATT_BAL_EXIT_LPH_DISCHG_VBATT;

	if (chip->deep_support && is_gauge_term_voltage_votable_available(chip)) {
		term_voltage = get_effective_result(chip->gauge_term_voltage_votable);
		if (term_voltage > 0)
			lph_dischg_exit_vbatt = term_voltage + 400; /* todo switch volt set to term_voltage + 400mv */
	}

	chg_info("lph_dischg_exit_vbatt=%d\n", lph_dischg_exit_vbatt);
	if (chip->b2_volt > lph_dischg_exit_vbatt && chip->b1_volt > lph_dischg_exit_vbatt)
		return true;

	return false;
}

static int oplus_batt_bal_handle_state_dischg(
	struct oplus_batt_bal_chip *chip)
{
	int rc = 0;
	int vout = chip->cfg.vout_thr[CTRL_REGION_DISCHG];
	int curr_thr = chip->cfg.dischg_curr_thr;

	if (chip->pre_bal_state != chip->curr_bal_state) {
		oplus_batt_bal_entry_state_dischg(chip);
		return rc;
	}

	if (oplus_batt_bal_switch_to_lph_dischg(chip))
		return rc;

	chip->target_iref = curr_thr;
	chg_info("sub_step_state:%d, volt_diff_thr:%d, flow_dir:%d, \
		target_iref:%d, lph_dischg_switch_count:%d\n",
		chip->sub_step_state, chip->eq_volt_diff_thr, chip->flow_dir,
		chip->target_iref, chip->lph_dischg_switch_count);

	switch (chip->sub_step_state) {
	case BAL_SUB_STATE_BAL:
		if (abs(chip->b2_volt - chip->b1_volt) <= chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_BAL;
			oplus_batt_bal_cfg(chip, true, false, DEFAULT_DIR, curr_thr, 0);
		} else if (chip->b1_volt - chip->b2_volt > chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
		 	chip->eq_volt_diff_thr -= chip->cfg.dischg_volt_diff_anti_thr;
			oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, curr_thr, vout);
			oplus_bal_track_err_report(chip, VOL_GAP_BIG_IN_DISCHG);
		} else {
			chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
		 	chip->eq_volt_diff_thr -= chip->cfg.dischg_volt_diff_anti_thr;
			oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, curr_thr, vout);
			oplus_bal_track_err_report(chip, VOL_GAP_BIG_IN_DISCHG);
		}
		break;
	case BAL_SUB_STATE_B1_TO_B2:
		if (chip->b1_volt - chip->b2_volt > chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
			oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, curr_thr, vout);
		} else if (abs(chip->b2_volt - chip->b1_volt) <=  chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_BAL;
			chip->eq_volt_diff_thr += chip->cfg.dischg_volt_diff_anti_thr;
			oplus_batt_bal_cfg(chip, true, false, DEFAULT_DIR, curr_thr, 0);
		} else {
			chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
			chip->eq_volt_diff_thr = chip->cfg.dischg_volt_diff_thr;
			oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, curr_thr, vout);
			oplus_bal_track_err_report(chip, VOL_GAP_BIG_IN_DISCHG);
		}
		break;
	case BAL_SUB_STATE_B2_TO_B1:
		if (chip->b2_volt - chip->b1_volt >  chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
			oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, curr_thr, vout);
		} else if (abs(chip->b2_volt - chip->b1_volt) <=  chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_BAL;
			 chip->eq_volt_diff_thr += chip->cfg.dischg_volt_diff_anti_thr;
			oplus_batt_bal_cfg(chip, true, false, DEFAULT_DIR, curr_thr, 0);
		} else {
			chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
			chip->eq_volt_diff_thr = chip->cfg.dischg_volt_diff_thr;
			oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, curr_thr, vout);
			oplus_bal_track_err_report(chip, VOL_GAP_BIG_IN_DISCHG);
		}
		break;
	default:
		chg_err("!!!sub state error, not goto here\n");
		break;
	}

	return rc;
}

static int oplus_batt_bal_handle_state_lph_dischg(
	struct oplus_batt_bal_chip *chip)
{
	int rc = 0;
	int vout = chip->cfg.vout_thr[CTRL_REGION_DISCHG];

	if (is_client_vote_enabled(chip->run_interval_update_votable, BAL_STATE_VOTER))
		vote(chip->run_interval_update_votable, BAL_STATE_VOTER, false, 0, true);

	if (oplus_batt_bal_lph_dischg_switch_to_dischg(chip)) {
		oplus_batt_bal_entry_state_dischg(chip);
		chip->curr_bal_state = OPLUS_BATT_BAL_STATE_DISCHG;
		return rc;
	}

	switch (chip->sub_step_state) {
	case BAL_SUB_STATE_B2_TO_B1:
		if (chip->b2_volt - chip->b1_volt > chip->cfg.lph_dischg_volt_diff_hold) {
			chip->target_iref += BATT_BAL_CURR_MA(25);
			if (chip->target_iref > BATT_BAL_CURR_MA(500))
				chip->target_iref = BATT_BAL_CURR_MA(500);
			oplus_batt_bal_cfg(chip, true, true, B2_TO_B1, chip->target_iref, vout);
		} else if (chip->b2_volt >= chip->b1_volt) {
			oplus_batt_bal_cfg(chip, true, true, B2_TO_B1, chip->target_iref, vout);
		} else if (chip->target_iref > 0) {
			chip->target_iref -= BATT_BAL_CURR_MA(25);
			if (chip->target_iref < BATT_BAL_CURR_MA(0))
				chip->target_iref = BATT_BAL_CURR_MA(0);
			oplus_batt_bal_cfg(chip, true, true, B2_TO_B1, chip->target_iref, vout);
		} else {
			oplus_batt_bal_entry_state_dischg(chip);
			chip->curr_bal_state = OPLUS_BATT_BAL_STATE_DISCHG;
		}
		break;
	case BAL_SUB_STATE_B1_TO_B2:
		if (chip->b1_volt - chip->b2_volt > chip->cfg.lph_dischg_volt_diff_hold) {
			chip->target_iref += BATT_BAL_CURR_MA(25);
			if (chip->target_iref > BATT_BAL_CURR_MA(500))
				chip->target_iref = BATT_BAL_CURR_MA(500);
			oplus_batt_bal_cfg(chip, true, true, B1_TO_B2, chip->target_iref, vout);
		} else if (chip->b1_volt >= chip->b2_volt) {
			oplus_batt_bal_cfg(chip, true, true, B1_TO_B2, chip->target_iref, vout);
		} else if (chip->target_iref > 0) {
			chip->target_iref -= BATT_BAL_CURR_MA(25);
			if (chip->target_iref < BATT_BAL_CURR_MA(0))
				chip->target_iref = BATT_BAL_CURR_MA(0);
			oplus_batt_bal_cfg(chip, true, true, B1_TO_B2, chip->target_iref, vout);
		} else {
			oplus_batt_bal_entry_state_dischg(chip);
			chip->curr_bal_state = OPLUS_BATT_BAL_STATE_DISCHG;
		}
		break;
	default:
		chg_err("!!! should not goto here\n");
		break;
	}

	return rc;
}

static int oplus_batt_bal_handle_batt_curr_over(
	struct oplus_batt_bal_chip *chip, int vout)
{
	int rc = 0;
	int b1_eq_curr_thr = chip->cfg.chg_curr_thr[chip->b1_temp_region];
	int b2_eq_curr_thr = chip->cfg.chg_curr_thr[chip->b2_temp_region];

	chip->eq_curr_thr = b1_eq_curr_thr > b2_eq_curr_thr ? b1_eq_curr_thr : b2_eq_curr_thr;
	chg_info("sub_step_state=%d, flow_dir=%d, target_iref=%d, vout=%d\n",
		chip->sub_step_state, chip->flow_dir, chip->eq_curr_thr, vout);

	if (chip->curr_over.b1_over) {
		switch (chip->flow_dir) {
		case B1_TO_B2:
		case DEFAULT_DIR:
			chip->target_iref += BATT_BAL_CURR_MA(chip->eq_curr_thr);
			oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
			break;
		case B2_TO_B1:
			if (!chip->target_iref ||
			    chip->target_iref  > BATT_BAL_CURR_MA(chip->eq_curr_thr))
				chip->target_iref -= BATT_BAL_CURR_MA(chip->eq_curr_thr);
			else
				chip->target_iref = 0;
			if (chip->target_iref < 0) {
				chip->target_iref = BATT_BAL_CURR_MA(chip->eq_curr_thr);
				oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
				chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
			} else {
				oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
				chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
			}
			break;
		default:
			chg_err("should not goto here\n");
			break;
		}
	} else  {
		switch (chip->flow_dir) {
		case B2_TO_B1:
		case DEFAULT_DIR:
			chip->target_iref += BATT_BAL_CURR_MA(chip->eq_curr_thr);
			oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
			chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
			break;
		case B1_TO_B2:
			if (!chip->target_iref ||
			    chip->target_iref  > BATT_BAL_CURR_MA(chip->eq_curr_thr))
				chip->target_iref -= BATT_BAL_CURR_MA(chip->eq_curr_thr);
			else
				chip->target_iref = 0;
			if (chip->target_iref < 0) {
				chip->target_iref = BATT_BAL_CURR_MA(chip->eq_curr_thr);
				oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
				chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
			} else {
				oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
				chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
			}
		break;
			default:
			chg_err("should not goto here\n");
			break;
		}
	}

	return rc;
}

static int oplus_batt_bal_entry_state_init(
	struct oplus_batt_bal_chip *chip, int vout)
{
	int rc = 0;
	int b1_eq_curr_thr = chip->cfg.chg_curr_thr[chip->b1_temp_region];
	int b2_eq_curr_thr = chip->cfg.chg_curr_thr[chip->b2_temp_region];
	int b1_eq_volt_diff_thr = chip->cfg.chg_volt_diff_thr[chip->b1_temp_region];
	int b2_eq_volt_diff_thr = chip->cfg.chg_volt_diff_thr[chip->b2_temp_region];

	chip->eq_curr_thr = b1_eq_curr_thr > b2_eq_curr_thr ? b1_eq_curr_thr : b1_eq_curr_thr;
	chip->eq_volt_diff_thr =
		b1_eq_volt_diff_thr > b2_eq_volt_diff_thr ? b1_eq_volt_diff_thr : b2_eq_volt_diff_thr;
	chip->target_iref = chip->eq_curr_thr;

	memset(&(chip->curr_below), 0, sizeof(chip->curr_below));
	chip->alarm_curr_status = BAL_ALARM_NONE;
	chg_info("sub_step_state=%d, flow_dir=%d, target_iref=%d, volt_diff_thr=%d, vout=%d\n",
		chip->sub_step_state, chip->flow_dir, chip->eq_curr_thr, chip->eq_volt_diff_thr, vout);

	if (abs(chip->b2_volt - chip->b1_volt) <= chip->eq_volt_diff_thr)
		chip->sub_step_state = BAL_SUB_STATE_BAL;
	else if (chip->b1_volt - chip->b2_volt > chip->eq_volt_diff_thr)
		chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
	else
		chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;

	if (chip->sub_step_state == BAL_SUB_STATE_BAL) {
		chip->target_iref = BATT_BAL_CURR_MA(0);
		rc = oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, chip->target_iref, 0);
	} else if (chip->sub_step_state == BAL_SUB_STATE_B1_TO_B2) {
		rc = oplus_batt_bal_cfg(
			chip, false, true, B1_TO_B2, chip->target_iref, vout);
	} else {
		rc = oplus_batt_bal_cfg(
			chip, false, true, B2_TO_B1, chip->target_iref, vout);
	}

	return rc;
}

static int oplus_batt_bal_check_cv_curr_below(
	struct oplus_batt_bal_chip * chip)
{
	int rc = 0;

	if (chip->flow_dir == B2_TO_B1) {
		if (chip->b2_volt > chip->cutoff.cv_volt && chip->b2_curr > 0 &&
		    chip->b2_curr <= (chip->cutoff.cv_curr + chip->cfg.curr_below_thr[CTRL_REGION_NORMAL_CHG])) {
			chip->curr_below.b2_below_count++;
			if (chip->curr_below.b2_below_count > BATT_BAL_CURR_BELOW_COUNT)
				chip->curr_below.b2_below = true;
			else
				chip->curr_below.b2_below = false;
		} else {
			chip->curr_below.b2_below_count = 0;
		}
	}

	if (chip->flow_dir == B1_TO_B2) {
		if (chip->b1_volt > chip->cutoff.cv_volt && chip->b1_curr > 0 &&
		    chip->b1_curr <= (chip->cutoff.cv_sub_curr + chip->cfg.curr_below_thr[CTRL_REGION_NORMAL_CHG])) {
			chip->curr_below.b1_below_count++;
			if (chip->curr_below.b1_below_count > BATT_BAL_CURR_BELOW_COUNT)
				chip->curr_below.b1_below = true;
			else
				chip->curr_below.b1_below = false;
		} else {
			chip->curr_below.b1_below_count = 0;
		}
	}

	chg_info("cv_cutoff_volt=%d, cv_cutoff_curr=%d, cv_sub_curr=%d, below_status[%d, %d, %d, %d]\n",
		chip->cutoff.cv_volt, chip->cutoff.cv_curr, chip->cutoff.cv_sub_curr,
		chip->curr_below.b1_below, chip->curr_below.b2_below,
		chip->curr_below.b1_below_count, chip->curr_below.b2_below_count);

	return rc;
}

static int oplus_batt_bal_check_ffc_curr_below(
	struct oplus_batt_bal_chip * chip)
{
	int rc = 0;

	if (chip->flow_dir == B1_TO_B2) {
		if (chip->b1_curr > 0 && (chip->b1_curr <
		    chip->cutoff.ffc_sub_curr + chip->cfg.curr_below_thr[CTRL_REGION_FFC_CHG]))
			chip->curr_below.b1_below_count++;
		else
			chip->curr_below.b1_below_count = 0;

		if (chip->curr_below.b1_below_count > BATT_BAL_CURR_BELOW_COUNT)
			chip->curr_below.b1_below = true;
		else
			chip->curr_below.b1_below = false;
	}

	if (chip->flow_dir == B2_TO_B1) {
		if (chip->b2_curr > 0 && (chip->b2_curr <
		    chip->cutoff.ffc_curr + chip->cfg.curr_below_thr[CTRL_REGION_FFC_CHG]))
			chip->curr_below.b2_below_count++;
		else
			chip->curr_below.b2_below_count = 0;

		if (chip->curr_below.b2_below_count > BATT_BAL_CURR_BELOW_COUNT)
			chip->curr_below.b2_below = true;
		else
			chip->curr_below.b2_below = false;
	}

	chg_info("ffc_cutoff_curr=%d, ffc_cutoff_sub_curr=%d, below_status[%d, %d, %d, %d]\n",
		chip->cutoff.ffc_curr, chip->cutoff.ffc_sub_curr,
		chip->curr_below.b1_below, chip->curr_below.b2_below,
		chip->curr_below.b1_below_count, chip->curr_below.b2_below_count);

	return rc;
}

static int oplus_batt_bal_check_fast_curr_below(
	struct oplus_batt_bal_chip * chip)
{
	int rc = 0;

	if (chip->flow_dir == B1_TO_B2) {
		if (chip->alarm_curr_status == BAL_ALARM_B1 ||
		    chip->alarm_curr_status == BAL_ALARM_B1_AND_B2)
			chip->curr_below.b1_below_count++;
		else
			chip->curr_below.b1_below_count = 0;

		if (chip->curr_below.b1_below_count >= BATT_BAL_FAST_CURR_BELOW_COUNT)
			chip->curr_below.b1_below = true;
		else
			chip->curr_below.b1_below = false;
	}
	if (chip->flow_dir == B2_TO_B1) {
		if (chip->alarm_curr_status == BAL_ALARM_B2 ||
		    chip->alarm_curr_status == BAL_ALARM_B1_AND_B2)
			chip->curr_below.b2_below_count++;
		else
			chip->curr_below.b2_below_count = 0;

		if (chip->curr_below.b2_below_count >= BATT_BAL_FAST_CURR_BELOW_COUNT)
			chip->curr_below.b2_below = true;
		else
			chip->curr_below.b2_below = false;
	}

	return rc;
}

static int oplus_batt_bal_check_curr_below(
	struct oplus_batt_bal_chip * chip)
{
	int rc = 0;

	if (chip->curr_bal_state == OPLUS_BATT_BAL_STATE_NORMAL_CHG)
		oplus_batt_bal_check_cv_curr_below(chip);
	else if (chip->curr_bal_state == OPLUS_BATT_BAL_STATE_FAST_CC_CHG)
		oplus_batt_bal_check_fast_curr_below(chip);
	else
		oplus_batt_bal_check_ffc_curr_below(chip);

	return rc;
}


static int oplus_batt_bal_handle_state_nor_fast_chg(
	struct oplus_batt_bal_chip *chip)
{
	int rc = 0;
	int vout;
	int curr_over_thr;
	int b1_eq_curr_thr = chip->cfg.chg_curr_thr[chip->b1_temp_region];
	int b2_eq_curr_thr = chip->cfg.chg_curr_thr[chip->b2_temp_region];
	int b1_eq_volt_diff_thr = chip->cfg.chg_volt_diff_thr[chip->b1_temp_region];
	int b2_eq_volt_diff_thr = chip->cfg.chg_volt_diff_thr[chip->b2_temp_region];

	if (chip->curr_bal_state == OPLUS_BATT_BAL_STATE_NORMAL_CHG) {
		vout = chip->cfg.vout_thr[CTRL_REGION_NORMAL_CHG];
		curr_over_thr = chip->cfg.curr_over_thr[CTRL_REGION_NORMAL_CHG];
	} else {
		vout = chip->cfg.vout_thr[CTRL_REGION_FFC_CHG];
		curr_over_thr = chip->cfg.curr_over_thr[CTRL_REGION_FFC_CHG];
	}
	chip->eq_curr_thr = b1_eq_curr_thr > b2_eq_curr_thr ? b1_eq_curr_thr : b1_eq_curr_thr;
	chip->eq_volt_diff_thr =
		b1_eq_volt_diff_thr > b2_eq_volt_diff_thr ? b1_eq_volt_diff_thr : b2_eq_volt_diff_thr;

	if (chip->pre_bal_state != chip->curr_bal_state) {
		rc = oplus_batt_bal_entry_state_init(chip, vout);
		return rc;
	}

	if (chip->curr_over.b1_over || chip->curr_over.b2_over) {
		oplus_batt_bal_handle_batt_curr_over(chip, vout);
		return rc;
	}

	rc = oplus_batt_bal_check_curr_below(chip);

	chg_info("sub_step_state:%d, volt_diff_thr:%d, flow_dir:%d, \
		target_iref:%d, curr_over_thr:%d, vout:%d\n",
		chip->sub_step_state, chip->eq_volt_diff_thr, chip->flow_dir,
		chip->target_iref, curr_over_thr, vout);

	switch (chip->sub_step_state) {
	case BAL_SUB_STATE_BAL:
		if (abs(chip->b2_volt - chip->b1_volt) <= chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_BAL;
		} else if (chip->b1_volt - chip->b2_volt > chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
			rc = oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
		} else {
			chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
			rc = oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
		}
		break;
	case BAL_SUB_STATE_B1_TO_B2:
		chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
		if (chip->b1_volt - chip->b2_volt > chip->eq_volt_diff_thr) {
			if (!chip->curr_below.b1_below &&
			   (chip->b2_curr <= chip->curr_over.b2_limit_curr - curr_over_thr))
				chip->target_iref += BATT_BAL_CURR_MA(chip->eq_curr_thr);
			rc = oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
		} else if (abs(chip->b2_volt - chip->b1_volt) <= chip->eq_volt_diff_thr) {
			rc = oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
		} else {
			chip->target_iref -= BATT_BAL_CURR_MA(chip->eq_curr_thr);
			if (chip->target_iref < 0) {
				chip->target_iref = BATT_BAL_CURR_MA(chip->eq_curr_thr);
				chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
				rc = oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
			} else {
				rc = oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
			}
		}
		break;
	case BAL_SUB_STATE_B2_TO_B1:
		chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
		if (chip->b2_volt - chip->b1_volt > chip->eq_volt_diff_thr) {
			if (!chip->curr_below.b2_below &&
			   (chip->b1_curr <= chip->curr_over.b1_limit_curr - curr_over_thr))
				chip->target_iref += BATT_BAL_CURR_MA(chip->eq_curr_thr);
			rc = oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
		} else if (abs(chip->b2_volt - chip->b1_volt) <= chip->eq_volt_diff_thr) {
			rc = oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
		} else {
			chip->target_iref -= BATT_BAL_CURR_MA(chip->eq_curr_thr);
			if (chip->target_iref < 0) {
				chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
				chip->target_iref = BATT_BAL_CURR_MA(chip->eq_curr_thr);
				rc = oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
			} else {
				rc = oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
			}
		}
		break;
	default:
		chg_err("sub state error, not goto here\n");
		break;
	}

	return rc;
}

static int oplus_batt_bal_handle_state_fast_start(
	struct oplus_batt_bal_chip *chip)
{
	int rc = 0;

	chip->fast_ctrl_mode = FAST_CTRL_DEFUALT_MODE;
	chip->curr_bal_state = OPLUS_BATT_BAL_STATE_FAST_BAL_START;
	memset(&(chip->curr_below), 0, sizeof(chip->curr_below));
	rc = oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, chip->target_iref, 0);

	chg_info("sub_step_state:%d, flow_dir:%d, fast_bal_mode:%d\n",
		chip->sub_step_state, chip->flow_dir, chip->fast_ctrl_mode);

	return rc;
}

static int oplus_batt_bal_handle_state_fast_bal_start(
	struct oplus_batt_bal_chip *chip)
{
	int rc = 0;
	int vout = chip->cfg.vout_thr[CTRL_REGION_FAST_CHG];
	int b1_eq_curr_thr = chip->cfg.chg_curr_thr[chip->b1_temp_region];
	int b2_eq_curr_thr = chip->cfg.chg_curr_thr[chip->b2_temp_region];
	int b1_eq_volt_diff_thr = chip->cfg.chg_volt_diff_thr[chip->b1_temp_region];
	int b2_eq_volt_diff_thr = chip->cfg.chg_volt_diff_thr[chip->b2_temp_region];

	chip->eq_curr_thr = b1_eq_curr_thr > b2_eq_curr_thr ? b1_eq_curr_thr : b1_eq_curr_thr;
	chip->eq_volt_diff_thr =
		b1_eq_volt_diff_thr > b2_eq_volt_diff_thr ? b1_eq_volt_diff_thr : b2_eq_volt_diff_thr;

	chip->fast_curr_ref = (chip->b1_curr + chip->b2_curr) / 2;
	if (chip->fast_curr_ref < 0)
		chip->fast_curr_ref = BATT_BAL_CURR_MA(500);
	chip->fast_curr_ref = BATT_BAL_FIX_CURR_MA(chip->fast_curr_ref);
	chip->target_iref =
		chip->fast_curr_ref * abs(chip->cfg.b1_design_cap - chip->cfg.b2_design_cap) /
		(chip->cfg.b1_design_cap + chip->cfg.b2_design_cap);

	chip->curr_bal_state = OPLUS_BATT_BAL_STATE_FAST_CC_CHG;
	chip->eq_fastchg_b2_to_b1_max_curr_thr = chip->cfg.fastchg_b2_to_b1_max_curr_thr;
	if (chip->fast_curr_ref > chip->eq_fastchg_b2_to_b1_max_curr_thr) {
		chip->fast_ctrl_mode = FAST_CTRL_CURR_MODE;
		chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
		rc = oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
	} else {
		chip->fast_ctrl_mode = FAST_CTRL_VOLT_MODE;
		if (abs(chip->b2_volt - chip->b1_volt) <= chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_BAL;
			chip->target_iref = BATT_BAL_CURR_MA(0);
			oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, chip->target_iref, 0);
		} else if (chip->b1_volt - chip->b2_volt > chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
			chip->target_iref =  BATT_BAL_CURR_MA(chip->eq_curr_thr);
			oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
		} else {
			chip->target_iref = BATT_BAL_CURR_MA(chip->eq_curr_thr);
			chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
			oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
		}
	}

	return rc;
}

static int oplus_batt_bal_check_fast_bal_mode(
	struct oplus_batt_bal_chip *chip, int fast_curr_ref)
{
	bool mode_change = false;
	int vout = chip->cfg.vout_thr[CTRL_REGION_FAST_CHG];

	switch (chip->fast_ctrl_mode) {
	case FAST_CTRL_CURR_MODE :
		if (fast_curr_ref > chip->eq_fastchg_b2_to_b1_max_curr_thr) {
			chip->fast_ctrl_mode = FAST_CTRL_CURR_MODE;
	    		chip->target_iref =
				fast_curr_ref * abs(chip->cfg.b1_design_cap - chip->cfg.b2_design_cap) /
				(chip->cfg.b1_design_cap + chip->cfg.b2_design_cap);
			chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
			oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
		} else {
			mode_change = true;
			chip->fast_ctrl_mode = FAST_CTRL_VOLT_MODE;
			if (chip->fast_curr_ref)
	    			chip->target_iref = (chip->target_iref * fast_curr_ref / chip->fast_curr_ref);
			chip->eq_fastchg_b2_to_b1_max_curr_thr += chip->cfg.fastchg_b2_to_b1_max_curr_anti_thr;
		}
		break;
	case FAST_CTRL_VOLT_MODE :
		if (fast_curr_ref > chip->eq_fastchg_b2_to_b1_max_curr_thr) {
			chip->fast_ctrl_mode = FAST_CTRL_CURR_MODE;
	    		chip->target_iref =
				fast_curr_ref * abs(chip->cfg.b1_design_cap - chip->cfg.b2_design_cap) /
				(chip->cfg.b1_design_cap + chip->cfg.b2_design_cap);
			chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
			oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
			chip->eq_fastchg_b2_to_b1_max_curr_thr -= chip->cfg.fastchg_b2_to_b1_max_curr_anti_thr;
		} else {
			chip->fast_ctrl_mode = FAST_CTRL_VOLT_MODE;
			mode_change = false;
		}
		break;
	default:
		chg_err("!!!bal mode err, should not goto here\n");
		break;
	}

	chg_info("mode_change:%d, fast_bal_mode:%d, fast_current_ref:%d, target_iref=%d, vout=%d\n",
		mode_change, chip->fast_ctrl_mode, fast_curr_ref, chip->target_iref, vout);

	return mode_change;
}

static int oplus_batt_bal_handle_state_fast_cc_chg(
	struct oplus_batt_bal_chip *chip)
{
	int rc = 0;
	int fast_curr_ref;
	bool mode_change = false;
	int vout = chip->cfg.vout_thr[CTRL_REGION_FAST_CHG];
	int curr_over_thr = chip->cfg.curr_over_thr[CTRL_REGION_FAST_CHG];
	int b1_eq_curr_thr = chip->cfg.chg_curr_thr[chip->b1_temp_region];
	int b2_eq_curr_thr = chip->cfg.chg_curr_thr[chip->b2_temp_region];
	int b1_eq_volt_diff_thr = chip->cfg.chg_volt_diff_thr[chip->b1_temp_region];
	int b2_eq_volt_diff_thr = chip->cfg.chg_volt_diff_thr[chip->b2_temp_region];

	chip->eq_curr_thr = b1_eq_curr_thr > b2_eq_curr_thr ? b1_eq_curr_thr : b1_eq_curr_thr;
	chip->eq_volt_diff_thr =
		b1_eq_volt_diff_thr > b2_eq_volt_diff_thr ? b1_eq_volt_diff_thr : b2_eq_volt_diff_thr;

	fast_curr_ref = (chip->b1_curr + chip->b2_curr) / 2;
	if (fast_curr_ref < 0)
		fast_curr_ref = BATT_BAL_CURR_MA(500);
	fast_curr_ref = BATT_BAL_FIX_CURR_MA(fast_curr_ref);

	mode_change = oplus_batt_bal_check_fast_bal_mode(chip, fast_curr_ref);
	chip->fast_curr_ref = fast_curr_ref;
	if (mode_change || chip->fast_ctrl_mode == FAST_CTRL_CURR_MODE)
		return rc;

	rc = oplus_batt_bal_check_curr_below(chip);

	chg_info("sub_step_state:%d, volt_diff_thr:%d, curr_thr:%d, flow_dir:%d, \
		target_iref:%d, curr_over_thr:%d, vout:%d alarm_curr_bellow[%d %d]\n", chip->sub_step_state,
		chip->eq_volt_diff_thr, chip->eq_curr_thr, chip->flow_dir, chip->target_iref, curr_over_thr, vout,
		chip->curr_below.b1_below, chip->curr_below.b2_below);
	switch (chip->sub_step_state) {
	case BAL_SUB_STATE_B1_TO_B2:
		if (chip->b1_volt - chip->b2_volt > chip->eq_volt_diff_thr) {
			if (!chip->curr_below.b1_below &&
			   (chip->b2_curr <= chip->curr_over.b2_limit_curr - curr_over_thr))
				chip->target_iref += BATT_BAL_CURR_MA(chip->eq_curr_thr);
			oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
		} else if (abs(chip->b2_volt - chip->b1_volt) <= chip->eq_volt_diff_thr) {
			oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
		} else {
			if (!chip->target_iref || chip->target_iref  > BATT_BAL_CURR_MA(chip->eq_curr_thr))
				chip->target_iref -= BATT_BAL_CURR_MA(chip->eq_curr_thr);
			else
				chip->target_iref = 0;
			if (chip->target_iref < 0) {
				chip->target_iref = BATT_BAL_CURR_MA(chip->eq_curr_thr);
				oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
				chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
			} else {
				oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
			}
		}
		break;
	case BAL_SUB_STATE_B2_TO_B1:
		if (chip->b2_volt - chip->b1_volt > chip->eq_volt_diff_thr) {
			if (!chip->curr_below.b2_below &&
			   (chip->b1_curr <= chip->curr_over.b1_limit_curr - curr_over_thr))
				chip->target_iref += BATT_BAL_CURR_MA(chip->eq_curr_thr);
			oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
		} else if (abs(chip->b2_volt - chip->b1_volt) <= chip->eq_volt_diff_thr) {
				oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
		} else {
			if (!chip->target_iref || chip->target_iref  > BATT_BAL_CURR_MA(chip->eq_curr_thr))
				chip->target_iref -= BATT_BAL_CURR_MA(chip->eq_curr_thr);
			else
				chip->target_iref = 0;
			if (chip->target_iref < 0) {
				chip->target_iref = BATT_BAL_CURR_MA(chip->eq_curr_thr);
				oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
				chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
			} else {
				oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
			}
		}
		break;
	case BAL_SUB_STATE_BAL:
		if (abs(chip->b2_volt - chip->b1_volt) <= chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_BAL;
			chip->target_iref = BATT_BAL_CURR_MA(chip->eq_curr_thr);
			oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, chip->target_iref, 0);
		} else if (chip->b1_volt - chip->b2_volt > chip->eq_volt_diff_thr) {
			chip->sub_step_state = BAL_SUB_STATE_B1_TO_B2;
			chip->target_iref = BATT_BAL_CURR_MA(chip->eq_curr_thr);
			oplus_batt_bal_cfg(chip, false, true, B1_TO_B2, chip->target_iref, vout);
		} else {
			chip->target_iref = BATT_BAL_CURR_MA(chip->eq_curr_thr);
			chip->sub_step_state = BAL_SUB_STATE_B2_TO_B1;
			oplus_batt_bal_cfg(chip, false, true, B2_TO_B1, chip->target_iref, vout);
		}
		break;
	default:
		chg_err("!!!should not goto here\n");
		break;
	}

	return rc;
}

static void oplus_batt_bal_process(struct oplus_batt_bal_chip *chip)
{
	if (chip->curr_bal_state != chip->pre_bal_state)
		chg_info("pre_bal_state=%s, curr_bal_state=%s\n",
			oplus_batt_bal_get_bal_state_str(chip->pre_bal_state),
			oplus_batt_bal_get_bal_state_str(chip->curr_bal_state));

	switch (chip->curr_bal_state) {
	case OPLUS_BATT_BAL_STATE_ABNORMAL:
		oplus_batt_bal_handle_state_abnormal(chip);
		break;
	case OPLUS_BATT_BAL_STATE_DISCHG:
		oplus_batt_bal_handle_state_dischg(chip);
		break;
	case OPLUS_BATT_BAL_STATE_LPH_DISCHG:
		oplus_batt_bal_handle_state_lph_dischg(chip);
		break;
	case OPLUS_BATT_BAL_STATE_NORMAL_CHG:
	case OPLUS_BATT_BAL_STATE_FFC_CHG:
		oplus_batt_bal_handle_state_nor_fast_chg(chip);
		break;
	case OPLUS_BATT_BAL_STATE_FAST_START:
		oplus_batt_bal_handle_state_fast_start(chip);
		break;
	case OPLUS_BATT_BAL_STATE_FAST_BAL_START:
		oplus_batt_bal_handle_state_fast_bal_start(chip);
		break;
	case OPLUS_BATT_BAL_STATE_FAST_CC_CHG:
		oplus_batt_bal_handle_state_fast_cc_chg(chip);
		break;
	default:
		pr_err("!!!not goto here\n");
		break;
	}

	chip->pre_bal_state = chip->curr_bal_state;
	mutex_unlock(&chip->state_lock);
}

static int oplus_vooc_fastchg_disable(
	struct oplus_batt_bal_chip *chip, const char *client_str, bool disable)
{
	int rc;

	if (!chip->vooc_disable_votable)
		chip->vooc_disable_votable = find_votable("VOOC_DISABLE");
	if (!chip->vooc_disable_votable) {
		chg_err("VOOC_DISABLE votable not found\n");
		return -EINVAL;
	}

	rc = vote(chip->vooc_disable_votable, client_str, disable,
		  disable ? true : false, false);
	if (rc < 0)
		chg_err("%s vooc charger error, rc = %d\n",
			disable ? "disable" : "enable", rc);
	else
		chg_info("%s vooc charger\n", disable ? "disable" : "enable");

        return rc;
}

static int oplus_pps_fastchg_disable(
	struct oplus_batt_bal_chip *chip, const char *client_str, bool disable)
{
	int rc;

	if (!chip->pps_disable_votable)
		chip->pps_disable_votable = find_votable("PPS_DISABLE");
	if (!chip->pps_disable_votable) {
		chg_err("PPS_DISABLE votable not found\n");
		return -EINVAL;
	}

	rc = vote(chip->pps_disable_votable, client_str, disable,
		  disable ? true : false, false);
	if (rc < 0)
		chg_err("%s pps charger error, rc = %d\n",
			disable ? "disable" : "enable", rc);
	else
		chg_info("%s pps charger\n", disable ? "disable" : "enable");

        return rc;
}

static int oplus_ufcs_fastchg_disable(
	struct oplus_batt_bal_chip *chip, const char *client_str, bool disable)
{
	int rc;

	if (!chip->ufcs_disable_votable)
		chip->ufcs_disable_votable = find_votable("UFCS_DISABLE");
	if (!chip->ufcs_disable_votable) {
		chg_err("UFCS_DISABLE votable not found\n");
		return -EINVAL;
	}

	rc = vote(chip->ufcs_disable_votable, client_str, disable,
		  disable ? true : false, false);
	if (rc < 0)
		chg_err("%s ufcs charger error, rc = %d\n",
			disable ? "disable" : "enable", rc);
	else
		chg_info("%s ufcs charger\n", disable ? "disable" : "enable");

        return rc;
}

static int oplus_wls_fastchg_disable(
	struct oplus_batt_bal_chip *chip, const char *client_str, bool disable)
{
	int rc;

	if (!chip->wls_disable_votable)
		chip->wls_disable_votable = find_votable("WLS_FASTCHG_DISABLE");
	if (!chip->wls_disable_votable) {
		chg_err("WLS_DISABLE votable not found\n");
		return -EINVAL;
	}

	rc = vote(chip->wls_disable_votable, client_str, disable,
		  disable ? true : false, false);
	if (rc < 0)
		chg_err("%s wls charger error, rc = %d\n",
			disable ? "disable" : "enable", rc);
	else
		chg_info("%s wls charger\n", disable ? "disable" : "enable");

        return rc;
}


static int oplus_batt_bal_check_abnormal_state(
	struct oplus_batt_bal_chip *chip)
{
	int rc = 0;
	int abnormal_state = BATT_BAL_NO_ABNORMAL;
	static int abnormal_count = 0;

	if (IS_ERR_OR_NULL(chip->main_gauge_topic) ||
		IS_ERR_OR_NULL(chip->sub_gauge_topic)) {
		chg_err("wait gauge toptic\n");
		return -EINVAL;
	}

	if (chip->b2_volt < BATT_BAL_VOL_ERR_MV ||
		chip->b1_volt < BATT_BAL_VOL_ERR_MV) {
		abnormal_state = BATT_BAL_VOL_ABNORMAL;
	} else if (chip->b2_temp <= BATT_BAL_TEMP_ERR_DEC ||
		chip->b1_temp <= BATT_BAL_TEMP_ERR_DEC) {
		abnormal_state = BATT_BAL_TEMP_ABNORMAL;
	} else {
		abnormal_state = BATT_BAL_NO_ABNORMAL;
		abnormal_count = 0;
		if (is_vooc_disable_votable_available(chip) &&
		    is_client_vote_enabled(chip->vooc_disable_votable, BATT_BAL_VOTER))
			oplus_vooc_fastchg_disable(chip, BATT_BAL_VOTER, false);
		if (is_pps_disable_votable_available(chip) &&
		    is_client_vote_enabled(chip->pps_disable_votable, BATT_BAL_VOTER))
			oplus_pps_fastchg_disable(chip, BATT_BAL_VOTER, false);
		if (is_ufcs_disable_votable_available(chip) &&
		    is_client_vote_enabled(chip->ufcs_disable_votable, BATT_BAL_VOTER))
			oplus_ufcs_fastchg_disable(chip, BATT_BAL_VOTER, false);
		if (is_wls_disable_votable_available(chip) &&
		    is_client_vote_enabled(chip->wls_disable_votable, BATT_BAL_VOTER))
			oplus_wls_fastchg_disable(chip, BATT_BAL_VOTER, false);
		oplus_batt_bal_set_abnormal_state(chip, abnormal_state);
	}

	if (abnormal_state != BATT_BAL_NO_ABNORMAL &&
	    chip->abnormal_state != abnormal_state) {
		abnormal_count++;
		if (abnormal_count > BATT_BAL_ABNORMAL_STATE_COUNT) {
			oplus_vooc_fastchg_disable(chip, BATT_BAL_VOTER, true);
			oplus_pps_fastchg_disable(chip, BATT_BAL_VOTER, true);
			oplus_ufcs_fastchg_disable(chip, BATT_BAL_VOTER, true);
			oplus_wls_fastchg_disable(chip, BATT_BAL_VOTER, true);
			oplus_batt_bal_set_abnormal_state(chip, abnormal_state);
		}
	}

	return rc;
}

static void oplus_batt_bal_sm_work(struct work_struct *work)
{
	int rc;
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_batt_bal_chip *chip = container_of(dwork,
		struct oplus_batt_bal_chip, batt_bal_sm_work);

	rc = oplus_batt_bal_update_batt_info(chip);
	if (rc < 0 || !chip->init_done) {
		chg_err("update batt info fail or init not completed\n");
		goto try;
	}

	oplus_batt_bal_check_batt_temp_region(chip);
	oplus_batt_bal_check_batt_curr_over(chip);
	oplus_batt_bal_check_abnormal_state(chip);
	oplus_batt_bal_state_daemon(chip);
	oplus_batt_bal_process(chip);
	oplus_batt_bal_other(chip);

try:
	chg_info("run_interval_update=%d abnormal_state :%d "
		 "b2 info[%d, %d, %d, %d], b1 info[%d, %d, %d, %d] "
		 "b1 temp region = %s, b2 temp region = %s "
		 "curr_over=[%d, %d], curr_limit=[%d, %d] "
		 "pmos_enable=%d, bal_enable=%d "
		 "online[%d %d], ffc_status:%d, fastchg_ing[%d %d %d %d], vooc_by_normal_path:%d "
		 "charging_enable[%d %d], ic_trig_abnormal:%d\n",
		 chip->run_interval_update, chip->abnormal_state,
		 chip->b2_volt, -chip->b2_curr, chip->b2_temp, chip->b2_soc,
		 chip->b1_volt, -chip->b1_curr, chip->b1_temp, chip->b1_soc,
		 batt_bal_temp_table[chip->b1_temp_region],
		 batt_bal_temp_table[chip->b2_temp_region],
		 chip->curr_over.b1_over, chip->curr_over.b2_over,
		 chip->curr_over.b1_limit_curr, chip->curr_over.b2_limit_curr,
		 oplus_batt_bal_get_pmos_enable(chip), oplus_batt_bal_get_enable(chip),
		 chip->wired_online, chip->wls_online, chip->ffc_status,
		 chip->vooc_fastchg_ing, chip->pps_fastchg_ing, chip->ufcs_fastchg_ing, chip->wls_fastchg_ing,
		 chip->vooc_by_normal_path, chip->wired_charging_enable, chip->wls_charging_enable,
		 atomic_read(&chip->ic_trig_abnormal));
	if (get_effective_client(chip->run_interval_update_votable) &&
	    strcmp(get_effective_client(chip->run_interval_update_votable), DEF_VOTER))
		schedule_delayed_work(&chip->batt_bal_sm_work,
			BATT_BAL_UPDATE_INTERVAL(chip->run_interval_update));
}

static void oplus_batt_bal_ic_trig_abnormal_clear_work(
			struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_batt_bal_chip *chip = container_of(dwork,
		struct oplus_batt_bal_chip, ic_trig_abnormal_clear_work);

	atomic_set(&chip->ic_trig_abnormal, 0);
}

static void oplus_batt_bal_online_init_work(struct work_struct *work)
{
	struct oplus_batt_bal_chip *chip = container_of(work, struct oplus_batt_bal_chip,
		batt_bal_online_init_work);

	chip->alarm_curr_status = BAL_ALARM_NONE;
	oplus_batt_bal_pmos_disable(chip->batt_bal_topic);
	oplus_wired_set_supplementary_power_mos(chip->wired_topic, false);

	oplus_batt_bal_set_curr_limit(chip, 0);
	oplus_batt_bal_ic_trig_abnormal_clear_work(&(chip->ic_trig_abnormal_clear_work.work));
}

static void oplus_batt_bal_charging_init_work(struct work_struct *work)
{
	struct oplus_batt_bal_chip *chip = container_of(work, struct oplus_batt_bal_chip,
		batt_bal_charging_init_work);

	oplus_batt_bal_pmos_disable(chip->batt_bal_topic);
	oplus_wired_set_supplementary_power_mos(chip->wired_topic, false);
}

static void oplus_batt_bal_disable_bal_work(struct work_struct *work)
{
	struct oplus_batt_bal_chip *chip = container_of(work, struct oplus_batt_bal_chip,
		batt_bal_disable_bal_work);

	chip->curr_over.not_allow_limit_jiffies = jiffies;
	oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, chip->target_iref, 0);
}

static void oplus_batt_bal_comm_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_batt_bal_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_CV_CUTOFF_VOLT_CURR:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->cutoff.cv_volt = data.intval >> CUTOFF_DATA_SHIFT;
			chip->cutoff.cv_curr = (data.intval >> CUTOFF_ITERM_SHIFT) & CUTOFF_ITERM_MASK;
			chip->cutoff.cv_sub_curr = data.intval & CUTOFF_ITERM_MASK;
			chg_info("cutoff cv_volt=%d, cv_curr=%d, cv_sub_curr=%d\n",
				chip->cutoff.cv_volt, chip->cutoff.cv_curr, chip->cutoff.cv_sub_curr);
			break;
		case COMM_ITEM_FFC_CUTOFF_CURR:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->cutoff.ffc_curr = data.intval >> CUTOFF_DATA_SHIFT;
			chip->cutoff.ffc_sub_curr = data.intval & CUTOFF_DATA_MASK;
			chg_info("cutoff ffc_curr=%d, ffc_sub_curr=%d\n", chip->cutoff.ffc_curr, chip->cutoff.ffc_sub_curr);
			break;
		case COMM_ITEM_FFC_STATUS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			if (chip->ffc_status != data.intval) {
				chip->ffc_status = data.intval;
				schedule_work(&chip->update_bal_state_work);
			}
			chg_info("ffc_status=%d\n", chip->ffc_status);
			break;
		case COMM_ITEM_FFC_STEP:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->ffc_step = data.intval;
			if (chip->ffc_step != data.intval) {
				chip->ffc_step = data.intval;
				schedule_work(&chip->update_bal_state_work);
			}
			chg_info("ffc_step=%d\n", chip->ffc_step);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_batt_bal_subscribe_comm_topic(
	struct oplus_mms *topic, void *prv_data)
{
	union mms_msg_data data = { 0 };
	struct oplus_batt_bal_chip *chip = prv_data;

	chip->comm_topic = topic;
	chip->comm_subs =
		oplus_mms_subscribe(chip->comm_topic, chip,
				    oplus_batt_bal_comm_subs_callback,
				    "batt_bal");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe common topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_CV_CUTOFF_VOLT_CURR, &data, true);
	chip->cutoff.cv_volt = data.intval >> CUTOFF_DATA_SHIFT;
	chip->cutoff.cv_curr = (data.intval >> CUTOFF_ITERM_SHIFT) & CUTOFF_ITERM_MASK;
	chip->cutoff.cv_sub_curr = data.intval & CUTOFF_ITERM_MASK;
	chg_info("cutoff cv_volt=%d, cv_curr=%d, cv_sub_curr=%d\n",
		chip->cutoff.cv_volt, chip->cutoff.cv_curr, chip->cutoff.cv_sub_curr);

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_FFC_CUTOFF_CURR, &data, true);
	chip->cutoff.ffc_curr = data.intval >> CUTOFF_DATA_SHIFT;
	chip->cutoff.ffc_sub_curr = data.intval & CUTOFF_DATA_MASK;
	chg_info("cutoff ffc_curr=%d, ffc_sub_curr=%d\n", chip->cutoff.ffc_curr, chip->cutoff.ffc_sub_curr);

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_FFC_STATUS, &data, true);
	chip->ffc_status = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_FFC_STEP, &data, true);
	chip->ffc_step = data.intval;
	chg_info("ffc_status=%d, ffc_step=%d\n", chip->ffc_status, chip->ffc_step);

	if (chip->ffc_step || chip->ffc_status)
		schedule_work(&chip->update_bal_state_work);
}

static void oplus_batt_bal_wired_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_batt_bal_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			if (chip->wired_online != data.intval) {
				chip->wired_online = data.intval;
				if (chip->wired_online)
					queue_work(system_highpri_wq, &chip->batt_bal_online_init_work);
				else
					oplus_batt_bal_set_fcc(0, false);
				schedule_work(&chip->update_bal_state_work);
				chip->full_err_report = false;
			}
			chg_info("wired_online:%d\n", chip->wired_online);
			break;
		case WIRED_ITEM_CHARGING_DISABLE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			if (chip->wired_charging_enable != !data.intval) {
				chip->wired_charging_enable = !data.intval;
				if (chip->wired_charging_enable)
					queue_work(system_highpri_wq, &chip->batt_bal_charging_init_work);
				schedule_work(&chip->update_bal_state_work);
			}
			chg_info("charging_enable:%d\n", chip->wired_charging_enable);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_batt_bal_subscribe_wired_topic(struct oplus_mms *topic,
	void *prv_data)
{
	struct oplus_batt_bal_chip *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_batt_bal_wired_subs_callback,
				    "batt_bal");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
		PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHARGING_DISABLE, &data, true);
	chip->wired_charging_enable = !data.intval;
	chg_info("charging_enable:%d\n", chip->wired_charging_enable);

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data, true);
	chip->wired_online = !!data.intval;
	if (chip->wired_online) {
		oplus_batt_bal_set_pmos_enable(chip, false);
		oplus_wired_set_supplementary_power_mos(chip->wired_topic, false);
	}
	chg_info("wired_online:%d\n", chip->wired_online);

	if (chip->wired_online || chip->wired_charging_enable) {
		schedule_work(&chip->update_bal_state_work);
	}
}

static void oplus_batt_bal_vooc_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_batt_bal_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case VOOC_ITEM_VOOC_STARTED:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data, false);
			if (chip->vooc_fastchg_ing != data.intval) {
				chip->vooc_fastchg_ing = data.intval;
				if (chip->vooc_fastchg_ing)
					queue_work(system_highpri_wq, &chip->batt_bal_disable_bal_work);
				schedule_work(&chip->update_bal_state_work);
			}
			chg_info("vooc_fastchg_ing=%d\n", chip->vooc_fastchg_ing);
			break;
		case VOOC_ITEM_VOOC_BY_NORMAL_PATH:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data, false);
			if (chip->vooc_by_normal_path != data.intval) {
				chip->vooc_by_normal_path = !!data.intval;
				schedule_work(&chip->update_bal_state_work);
			}
			chg_info("vooc_by_normal_path=%d\n", chip->vooc_by_normal_path);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}
static void oplus_batt_bal_subscribe_vooc_topic(struct oplus_mms *topic,
	void *prv_data)
{
	union mms_msg_data data = { 0 };
	struct oplus_batt_bal_chip *chip = prv_data;

	chip->vooc_topic = topic;
	chip->vooc_subs =
	oplus_mms_subscribe(chip->vooc_topic, chip,
			    oplus_batt_bal_vooc_subs_callback,
			    "batt_bal");
	if (IS_ERR_OR_NULL(chip->vooc_subs)) {
		chg_err("subscribe vooc topic error, rc=%ld\n",
		PTR_ERR(chip->vooc_subs));
		return;
	}

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_STARTED, &data, true);
	chip->vooc_fastchg_ing = !!data.intval;
	chg_info("vooc_fastchg_ing=%d\n", chip->vooc_fastchg_ing);

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_BY_NORMAL_PATH, &data, true);
	chip->vooc_by_normal_path = !!data.intval;
	chg_info("vooc_by_normal_path=%d\n", chip->vooc_by_normal_path);

	if (chip->vooc_fastchg_ing || chip->vooc_by_normal_path) {
		if (chip->vooc_fastchg_ing)
			oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, chip->target_iref, 0);
		schedule_work(&chip->update_bal_state_work);
	}
}

static void oplus_batt_bal_gauge_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_batt_bal_chip *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_TIMER:
		if (chip && get_effective_client(chip->run_interval_update_votable) &&
		    (!strcmp(get_effective_client(chip->run_interval_update_votable), DEF_VOTER) ||
		    !(work_busy(&chip->batt_bal_sm_work.work))))
			schedule_delayed_work(&chip->batt_bal_sm_work, 0);
		break;
	default:
		break;
	}
}

static void oplus_batt_bal_subscribe_gauge_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	union mms_msg_data data = { 0 };
	struct oplus_batt_bal_chip *chip = prv_data;

	chip->gauge_topic = topic;
	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
				    oplus_batt_bal_gauge_subs_callback, "batt_bal");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}

	oplus_mms_get_item_data(topic, GAUGE_ITEM_DEEP_SUPPORT, &data, true);
	chip->deep_support = data.intval;
}

static void oplus_batt_bal_main_gauge_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	default:
		break;
	}
}

static void oplus_batt_bal_subscribe_main_gauge_topic(struct oplus_mms *topic,
	void *prv_data)
{
	struct oplus_batt_bal_chip *chip = prv_data;

	chip->main_gauge_topic = topic;
	chip->main_gauge_subs =
		oplus_mms_subscribe(chip->main_gauge_topic, chip,
				    oplus_batt_bal_main_gauge_subs_callback, "batt_bal");
	if (IS_ERR_OR_NULL(chip->main_gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->main_gauge_subs));
		return;
	}

	if (chip->b2_inr_strategy)
		oplus_chg_strategy_init(chip->b2_inr_strategy);

	vote(chip->run_interval_update_votable, DEF_VOTER, true, BATT_BAL_TIME_MS(10000), false);
	mod_delayed_work(system_highpri_wq, &chip->batt_bal_sm_work, 0);
}

static void oplus_batt_bal_sub_gauge_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	default:
		break;
	}
}

static void oplus_batt_bal_subscribe_sub_gauge_topic(struct oplus_mms *topic,
							void *prv_data)
{
	struct oplus_batt_bal_chip *chip = prv_data;

	chip->sub_gauge_topic = topic;
	chip->sub_gauge_subs =
		oplus_mms_subscribe(chip->sub_gauge_topic, chip,
				    oplus_batt_bal_sub_gauge_subs_callback, "batt_bal");
	if (IS_ERR_OR_NULL(chip->sub_gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->sub_gauge_subs));
		return;
	}

	if (chip->b1_inr_strategy)
		oplus_chg_strategy_init(chip->b1_inr_strategy);
	vote(chip->run_interval_update_votable, DEF_VOTER, true, BATT_BAL_TIME_MS(10000), false);
	mod_delayed_work(system_highpri_wq, &chip->batt_bal_sm_work, 0);
}

static void oplus_batt_bal_ufcs_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_batt_bal_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case UFCS_ITEM_CHARGING:
			oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (chip->ufcs_fastchg_ing != data.intval) {
				chip->ufcs_fastchg_ing = !!data.intval;
				if (chip->ufcs_fastchg_ing)
					queue_work(system_highpri_wq, &chip->batt_bal_disable_bal_work);
				schedule_work(&chip->update_bal_state_work);
			}
			chg_info("ufcs_fastchg_ing=%d\n", chip->ufcs_fastchg_ing);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_batt_bal_subscribe_ufcs_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_batt_bal_chip *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->ufcs_topic = topic;
	chip->ufcs_subs = oplus_mms_subscribe(topic, chip,
					     oplus_batt_bal_ufcs_subs_callback,
					     "batt_bal");
	if (IS_ERR_OR_NULL(chip->ufcs_subs)) {
		chg_err("subscribe ufcs topic error, rc=%ld\n",
			PTR_ERR(chip->ufcs_subs));
		return;
	}

	oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_CHARGING, &data, true);
	chip->ufcs_fastchg_ing = !!data.intval;
	if (chip->ufcs_fastchg_ing) {
		oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, chip->target_iref, 0);
		schedule_work(&chip->update_bal_state_work);
	}
	chg_info("ufcs_fastchg_ing=%d\n", chip->ufcs_fastchg_ing);
}

static void oplus_batt_bal_wls_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_batt_bal_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WLS_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->wls_topic, id, &data, false);
			if (chip->wls_online != data.intval) {
				chip->wls_online = data.intval;
				if (chip->wls_online)
					queue_work(system_highpri_wq, &chip->batt_bal_online_init_work);
				schedule_work(&chip->update_bal_state_work);
			}
			chg_info("wls_online=%d\n", chip->wls_online);
			break;
		case WLS_ITEM_CHARGING_DISABLE:
			oplus_mms_get_item_data(chip->wls_topic, id, &data, false);
			if (chip->wls_charging_enable != !data.intval) {
				chip->wls_charging_enable = !data.intval;
				if (chip->wls_charging_enable)
					queue_work(system_highpri_wq, &chip->batt_bal_charging_init_work);
				schedule_work(&chip->update_bal_state_work);
			}
			chg_info("charging_enable =%d\n", chip->wls_charging_enable);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_batt_bal_subscribe_wls_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_batt_bal_chip *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wls_topic = topic;
	chip->wls_subs = oplus_mms_subscribe(topic, chip,
					     oplus_batt_bal_wls_subs_callback,
					     "batt_bal");
	if (IS_ERR_OR_NULL(chip->wls_subs)) {
		chg_err("subscribe wls topic error, rc=%ld\n",
			PTR_ERR(chip->wls_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_ONLINE, &data, true);
	chip->wls_online = !!data.intval;
	if (chip->wls_online) {
		oplus_batt_bal_set_pmos_enable(chip, false);
		oplus_wired_set_supplementary_power_mos(chip->wired_topic, false);
	}
	chg_info("wls_online=%d\n", chip->wls_online);

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_CHARGING_DISABLE, &data, true);
	chip->wls_charging_enable = !data.intval;
	chg_info("charging_enable =%d\n", chip->wls_charging_enable);

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_FASTCHG_STATUS, &data, false);
	chip->wls_fastchg_ing = data.intval;
	chg_info("wls_fastchg_ing =%d\n", chip->wls_fastchg_ing);
	if (chip->wls_fastchg_ing)
		oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, chip->target_iref, 0);

	if (chip->wls_fastchg_ing || chip->wls_charging_enable || chip->wls_online) {
		schedule_work(&chip->update_bal_state_work);
	}
}

static void oplus_batt_bal_pps_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_batt_bal_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PPS_ITEM_CHARGING:
			oplus_mms_get_item_data(chip->pps_topic, id, &data, false);
			if (chip->pps_fastchg_ing != data.intval) {
				chip->pps_fastchg_ing = !!data.intval;
				if (chip->pps_fastchg_ing)
					queue_work(system_highpri_wq, &chip->batt_bal_disable_bal_work);
				schedule_work(&chip->update_bal_state_work);
			}
			chg_info("pps_fastchg_ing=%d\n", chip->pps_fastchg_ing);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_batt_bal_subscribe_pps_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_batt_bal_chip *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->pps_topic = topic;
	chip->pps_subs = oplus_mms_subscribe(topic, chip,
					     oplus_batt_bal_pps_subs_callback,
					     "batt_bal");
	if (IS_ERR_OR_NULL(chip->pps_subs)) {
		chg_err("subscribe pps topic error, rc=%ld\n",
			PTR_ERR(chip->pps_subs));
		return;
	}

	oplus_mms_get_item_data(chip->pps_topic, PPS_ITEM_CHARGING, &data, true);
	chip->pps_fastchg_ing = !!data.intval;
	if (chip->pps_fastchg_ing) {
		oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, chip->target_iref, 0);
		schedule_work(&chip->update_bal_state_work);
	}
	chg_info("pps_fastchg_ing=%d\n", chip->pps_fastchg_ing);
}

static void oplus_batt_bal_bal_subs_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_batt_bal_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case BATT_BAL_ITEM_LCF_ALARM:
			oplus_mms_get_item_data(chip->batt_bal_topic, id, &data, false);
			chip->alarm_curr_status = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_batt_bal_subscribe_bal_topic(struct oplus_batt_bal_chip *chip)
{
	chip->bal_subs =
		oplus_mms_subscribe(chip->batt_bal_topic, chip,
				    oplus_batt_bal_bal_subs_callback,
				    "batt_bal");
	if (IS_ERR_OR_NULL(chip->bal_subs)) {
		chg_err("subscribe bal topic error, rc=%ld\n",
			PTR_ERR(chip->bal_subs));
		return;
	}
}

static int oplus_chg_batt_bal_topic_init(struct oplus_batt_bal_chip *chip)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;

	mms_cfg.drv_data = chip;
	mms_cfg.of_node = chip->dev->of_node;

	chip->batt_bal_topic = devm_oplus_mms_register(chip->dev, &oplus_mms_batt_bal_desc, &mms_cfg);
	if (IS_ERR(chip->batt_bal_topic)) {
		chg_err("Couldn't register batt_bal_topic\n");
		rc = PTR_ERR(chip->batt_bal_topic);
		return rc;
	}
	oplus_batt_bal_subscribe_bal_topic(chip);

	oplus_mms_wait_topic("common", oplus_batt_bal_subscribe_comm_topic, chip);
	oplus_mms_wait_topic("wired", oplus_batt_bal_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("vooc", oplus_batt_bal_subscribe_vooc_topic, chip);
	oplus_mms_wait_topic("ufcs", oplus_batt_bal_subscribe_ufcs_topic, chip);
	oplus_mms_wait_topic("wireless", oplus_batt_bal_subscribe_wls_topic, chip);
	oplus_mms_wait_topic("pps", oplus_batt_bal_subscribe_pps_topic, chip);

	return 0;
}

static void oplus_batt_bal_set_ic_trig_abnormal(
			struct oplus_batt_bal_chip *chip, struct oplus_chg_ic_err_msg *msg)
{
	chg_info("ic_exit_reason=%s\n", batt_bal_ic_exit_reason_str(msg->sub_type));

	if (msg->sub_type == BATT_BAL_ERR_UNKNOW ||
	    msg->sub_type == BATT_BAL_ERR_L1_PEAK_CURR_LIMIT ||
	    msg->sub_type == BATT_BAL_ERR_L2_PEAK_CURR_LIMIT ||
	    msg->sub_type == BATT_BAL_ERR_WDG_TIMEOUT)
	    return;

	atomic_set(&chip->ic_trig_abnormal, 1);
	cancel_delayed_work(&chip->ic_trig_abnormal_clear_work);
	schedule_delayed_work(&chip->ic_trig_abnormal_clear_work, msecs_to_jiffies(120000));
	schedule_work(&chip->update_bal_state_work);
}

static void oplus_batt_bal_err_handler_work(struct work_struct *work)
{
	struct oplus_batt_bal_chip *chip = container_of(work, struct oplus_batt_bal_chip,
		err_handler_work);
	struct oplus_chg_ic_err_msg *msg, *tmp;
	struct list_head msg_list;

	INIT_LIST_HEAD(&msg_list);
	spin_lock(&chip->ic_dev->err_list_lock);
	if (!list_empty(&chip->ic_dev->err_list))
		list_replace_init(&chip->ic_dev->err_list, &msg_list);
	spin_unlock(&chip->ic_dev->err_list_lock);

	list_for_each_entry_safe(msg, tmp, &msg_list, list) {
		if (is_err_topic_available(chip))
			oplus_mms_publish_ic_err_msg(chip->err_topic,
						     ERR_ITEM_IC, msg);
		oplus_print_ic_err(msg);
		oplus_batt_bal_set_ic_trig_abnormal(chip, msg);
		list_del(&msg->list);
		kfree(msg);
	}
}

static void oplus_chg_batt_bal_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_batt_bal_chip *chip = container_of(dwork,
		struct oplus_batt_bal_chip, batt_bal_init_work);
	struct device_node *node = chip->dev->of_node;
	static int retry = OPLUS_CHG_IC_INIT_RETRY_MAX;
	int rc;

	chip->ic_dev = of_get_oplus_chg_ic(node, "oplus,batt_bal_ic", 0);
	if (chip->ic_dev == NULL) {
		chg_err("not find batt_bal ic\n");
		goto init_try_again;
	}

	rc = oplus_chg_ic_func(chip->ic_dev, OPLUS_IC_FUNC_INIT);
	if (rc == -EAGAIN) {
		chg_err("batt_bal_ic init timeout\n");
		goto init_try_again;
	} else if (rc < 0) {
		chg_err("batt_bal_ic init error, rc=%d\n", rc);
		retry = 0;
		goto init_error;
	}
	retry = 0;
	chip->init_done = true;
	oplus_batt_bal_cfg(chip, false, false, DEFAULT_DIR, 0, 0);
	oplus_batt_bal_get_pmos_enable(chip);
	oplus_batt_bal_reg_dump(chip);
	oplus_batt_bal_virq_register(chip);
	oplus_chg_batt_bal_topic_init(chip);

	return;

init_try_again:
	if (retry > 0) {
		retry--;
		schedule_delayed_work(&chip->batt_bal_init_work,
			msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
	} else {
		chg_err("oplus,batt_bal ic not found\n");
	}
init_error:
	chip->ic_dev = NULL;
	return;
}

static int oplus_batt_bal_strategy_init(struct oplus_batt_bal_chip *chip)
{
	struct oplus_batt_bal_cfg *cfg = &chip->cfg;

	chip->b1_inr_strategy =
		oplus_chg_strategy_alloc(cfg->strategy_name,
					 cfg->b1_strategy_data,
					 cfg->b1_strategy_data_size);
	devm_kfree(chip->dev, chip->cfg.b1_strategy_data);
	chip->cfg.b1_strategy_data = NULL;
	if (chip->b1_inr_strategy == NULL) {
		chg_err("%s strategy alloc error", cfg->strategy_name);
		return -EINVAL;
	}

	chip->b2_inr_strategy =
		oplus_chg_strategy_alloc(cfg->strategy_name,
					 cfg->b2_strategy_data,
					 cfg->b2_strategy_data_size);
	devm_kfree(chip->dev, chip->cfg.b2_strategy_data);
	chip->cfg.b2_strategy_data = NULL;
	if (chip->b2_inr_strategy == NULL) {
		oplus_chg_strategy_release(chip->b1_inr_strategy);
		chip->b1_inr_strategy = NULL;
		chg_err("%s strategy alloc error", cfg->strategy_name);
		return -EINVAL;
	}

	return 0;
}


#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "config/dynamic_cfg/oplus_batt_bal_cfg.h"
#endif

#if defined(CONFIG_THERMAL) && (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static int sub_batt_thermal_read_temp(struct thermal_zone_device *tzd,
		int *temp)
{
	struct oplus_batt_bal_chip *chip;

	if (!tzd) {
		chg_err("tzd is null\n");
		return 0;
	}

	chip = thermal_zone_device_priv(tzd);
	if (!chip) {
		chg_err("chip is null\n");
		return 0;
	}
	*temp = chip->b1_temp * 100; /* thermal zone need 0.001 */

	return 0;
}

static struct thermal_zone_device_ops sub_batt_temp_tzd_ops = {
	.get_temp = sub_batt_thermal_read_temp,
};

static int register_tz_thermal(struct oplus_batt_bal_chip *chip)
{
	int ret = 0;

	chip->sub_batt_temp_tzd = thermal_tripless_zone_device_register("sub_batt_temp",
					chip, &sub_batt_temp_tzd_ops, NULL);
	if (IS_ERR(chip->sub_batt_temp_tzd)) {
		chg_err("sub_batt_temp_tzd register fai\nl");
		return PTR_ERR(chip->sub_batt_temp_tzd);
	}
	ret = thermal_zone_device_enable(chip->sub_batt_temp_tzd);
	if (ret)
		thermal_zone_device_unregister(chip->sub_batt_temp_tzd);

	return ret;
}
#endif

static int oplus_bal_misc_dev_reg(struct oplus_batt_bal_chip *chip)
{
	int rc;

	chip->misc_dev.minor = MISC_DYNAMIC_MINOR;
	chip->misc_dev.name = "oplus_chg";
	chip->misc_dev.fops = &oplus_chg_fops;
	rc = misc_register(&chip->misc_dev);
	if (rc)
		chg_err("misc_register failed, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_batt_bal_probe(struct platform_device *pdev)
{
	int rc;
	struct oplus_batt_bal_chip *chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_batt_bal_chip),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	of_platform_populate(chip->dev->of_node, NULL, NULL, chip->dev);
	rc = oplus_batt_bal_init(chip);
	if (rc < 0)
		goto bal_init_err;

	rc = oplus_batt_bal_strategy_init(chip);
	if (rc < 0)
		goto strategy_init_err;

	chip->init_done = false;
	INIT_DELAYED_WORK(&chip->batt_bal_init_work, oplus_chg_batt_bal_init_work);
	INIT_DELAYED_WORK(&chip->batt_bal_sm_work, oplus_batt_bal_sm_work);
	INIT_DELAYED_WORK(&chip->ic_trig_abnormal_clear_work, oplus_batt_bal_ic_trig_abnormal_clear_work);
	INIT_WORK(&chip->batt_bal_charging_init_work, oplus_batt_bal_charging_init_work);
	INIT_WORK(&chip->batt_bal_online_init_work, oplus_batt_bal_online_init_work);
	INIT_WORK(&chip->batt_bal_disable_bal_work, oplus_batt_bal_disable_bal_work);
	INIT_WORK(&chip->err_handler_work, oplus_batt_bal_err_handler_work);
	INIT_WORK(&chip->update_run_interval_work, oplus_batt_bal_update_run_interval_work);
	INIT_WORK(&chip->update_bal_state_work, oplus_batt_bal_update_bal_state_work);
	INIT_WORK(&chip->bal_track_work, oplus_batt_bal_track_work);

	oplus_mms_wait_topic("gauge", oplus_batt_bal_subscribe_gauge_topic, chip);
	oplus_mms_wait_topic("gauge:0", oplus_batt_bal_subscribe_main_gauge_topic, chip);
	oplus_mms_wait_topic("gauge:1", oplus_batt_bal_subscribe_sub_gauge_topic, chip);
	oplus_bal_misc_dev_reg(chip);
	schedule_delayed_work(&chip->batt_bal_init_work, 0);

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	(void)oplus_batt_bal_reg_debug_config(chip);
#endif

#if defined(CONFIG_THERMAL) && (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	rc = register_tz_thermal(chip);
	if (rc)
		chg_err("register tz fail");
#endif

	chg_info("probe success\n");
	return 0;

strategy_init_err:
bal_init_err:
	chg_err("probe fail, rc=%d", rc);
	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);
	return rc;
}

static int oplus_chg_batt_bal_pm_resume(struct device *dev)
{
	return 0;
}

static int oplus_chg_batt_bal_pm_suspend(struct device *dev)
{
	struct oplus_batt_bal_chip *chip = dev_get_drvdata(dev);
	bool status = false;

	if(chip == NULL)
		return 0;

	status = oplus_batt_bal_get_pmos_enable(chip);
	if (!status && chip->abnormal_state == BATT_BAL_NO_ABNORMAL)
		oplus_batt_bal_set_pmos_enable(chip, true);

	oplus_wired_set_supplementary_power_mos(chip->wired_topic, true);

	return 0;
}

static const struct dev_pm_ops bal_pm_ops = {
	.resume = oplus_chg_batt_bal_pm_resume,
	.suspend = oplus_chg_batt_bal_pm_suspend,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_chg_batt_bal_remove(struct platform_device *pdev)
#else
static int oplus_chg_batt_bal_remove(struct platform_device *pdev)
#endif
{
	struct oplus_batt_bal_chip *chip = platform_get_drvdata(pdev);

	if (!IS_ERR_OR_NULL(chip->comm_subs))
		oplus_mms_unsubscribe(chip->comm_subs);
	if (!IS_ERR_OR_NULL(chip->wired_subs))
		oplus_mms_unsubscribe(chip->wired_subs);
	if (!IS_ERR_OR_NULL(chip->gauge_subs))
		oplus_mms_unsubscribe(chip->gauge_subs);
	if (!IS_ERR_OR_NULL(chip->vooc_subs))
		oplus_mms_unsubscribe(chip->vooc_subs);
	if (!IS_ERR_OR_NULL(chip->ufcs_subs))
		oplus_mms_unsubscribe(chip->ufcs_subs);

	if (chip->b1_inr_strategy)
		oplus_chg_strategy_release(chip->b1_inr_strategy);
	if (chip->b2_inr_strategy)
		oplus_chg_strategy_release(chip->b2_inr_strategy);

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	oplus_batt_bal_unreg_debug_config(chip);
#endif

	devm_kfree(&pdev->dev, chip);
	platform_set_drvdata(pdev, NULL);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_chg_batt_bal_match[] = {
	{ .compatible = "oplus,batt_bal" },
	{},
};

static struct platform_driver oplus_chg_batt_bal_driver = {
	.driver		= {
		.name = "oplus-batt_bal",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_chg_batt_bal_match),
		.pm = &bal_pm_ops,
	},
	.probe		= oplus_chg_batt_bal_probe,
	.remove		= oplus_chg_batt_bal_remove,
};

static __init int oplus_chg_batt_bal_init(void)
{
	return platform_driver_register(&oplus_chg_batt_bal_driver);
}

static __exit void oplus_chg_batt_bal_exit(void)
{
	platform_driver_unregister(&oplus_chg_batt_bal_driver);
}

oplus_chg_module_register(oplus_chg_batt_bal);
