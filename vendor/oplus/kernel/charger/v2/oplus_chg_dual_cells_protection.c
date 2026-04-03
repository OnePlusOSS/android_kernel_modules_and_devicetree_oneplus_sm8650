// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2024 . Oplus All rights reserved.
 */

#define pr_fmt(fmt) "[PROTECTION_CHG]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/gfp.h>
#include <linux/power_supply.h>
#include <linux/timer.h>

#include <oplus_chg.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_module.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_cpa.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_pps.h>
#include <oplus_chg_dual_cells_protection.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_wls.h>

#define SOC_JUMP_COUNT_MAX 3
#define FCC_SMALL_THR 3
#define FCC_RECOVER_THR 3
#define FCC_INTEGRAL_THR 20
#define INVALID_SOC (-1)
#define INVALID_FCC (-1)
#define INVALID_CC  (-1)
#define INT_FCC_UNIT 100
#define DUAL_CELLS_PROTECT_TRACK_INFO_LEN   200
#define BATT_FULL_IBAT_MA   10
#define DUAL_CELLS_TRACK_INTERVAL_MS 1000

struct oplus_dual_cells_protection_spec {
	int min_fcc;
	int soc_jump_thr;
	int min_oplus_fcc;
	int cur_recover_thr;
	int cur_limit;
};

struct oplus_dual_cells_protection {
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	struct oplus_mms *comm_topic;
	struct oplus_mms *wired_topic;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *ufcs_topic;
	struct oplus_mms *pps_topic;
	struct oplus_mms *vooc_topic;
	struct oplus_mms *wls_topic;
	struct oplus_mms *protection_topic;
	struct oplus_mms *err_topic;
	struct mms_subscribe *comm_subs;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *gauge_subs;

	struct votable *vooc_curr_votable;
	struct votable *ufcs_curr_votable;
	struct votable *pps_curr_votable;
	struct votable *wls_fcc_votable;

	bool wired_online;
	int batt_cc;
	int batt_fcc;
	int batt_integral_fcc;
	bool intfcc_valid;
	bool intfcc_checked;
	int pre_batt_soc;
	int batt_soc;
	int ibat_ma;
	int plugin_soc;
	bool batt_health;
	bool pre_batt_health;
	bool is_temp_normal;
	bool ufcs_charging;
	bool vooc_charging;
	bool pps_charging;
	bool wls_fastchg_charging;
	bool batt_full;
	bool fcc_check_support;
	bool intfcc_check_support;
	bool soc_check_support;
	bool fcc_track_support;
	bool intfcc_track_support;
	bool soc_track_support;
	int reason;
	int debug_reason;
	bool rechging;
	bool xvdd_occur;
	bool sn_change_occur;
	bool first_boot;

	struct oplus_dual_cells_protection_spec spec;
	struct delayed_work plugin_work;
	struct delayed_work batt_check_work;
	struct delayed_work dual_cells_protect_track_work;
};

const char *const protect_reason_str[] = {
	[PROTECT_UNKNOWN]	= "UNKNOWN",
	[FCC_SMALL]	        = "FCC SMALL",
	[INTFCC_SMALL]	        = "INTFCC SMALL",
	[SOC_JUMP]	        = "SOC JUMP",
	[FCC_RECOVER]	        = "FCC RECOVER",
	[FCC_RECOVERING]        = "FCC_RECOVERING",
	[INTFCC_RECOVER]	= "INTFCC RECOVER",
	[SN_CHANGE]	        = "SN CHANGE",
	[XVDD_OCCUR]	        = "XVDD OCCUR",
};

const char *get_protect_reason_str(enum dual_cells_protect_reason type)
{
	if (type < 0 || type >= REASON_MAX)
		return "Unknown";
	return protect_reason_str[type];
}

__maybe_unused static bool
is_err_topic_available(struct oplus_dual_cells_protection *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}

__maybe_unused static bool
is_gauge_topic_available(struct oplus_dual_cells_protection *chip)
{
	if (!chip->gauge_topic)
		chip->gauge_topic = oplus_mms_get_by_name("gauge");
	return !!chip->gauge_topic;
}

__maybe_unused static bool
is_vooc_curr_votable_available(struct oplus_dual_cells_protection *chip)
{
	if (!chip->vooc_curr_votable)
		chip->vooc_curr_votable = find_votable("VOOC_CURR");
	return !!chip->vooc_curr_votable;
}

__maybe_unused static bool
is_wls_fcc_votable_available(struct oplus_dual_cells_protection *chip)
{
	if (!chip->wls_fcc_votable)
		chip->wls_fcc_votable = find_votable("WLS_FCC");
	return !!chip->wls_fcc_votable;
}

__maybe_unused static bool
is_ufcs_curr_votable_available(struct oplus_dual_cells_protection *chip)
{
	if (!chip->ufcs_curr_votable)
		chip->ufcs_curr_votable = find_votable("UFCS_CURR");
	return !!chip->ufcs_curr_votable;
}

__maybe_unused static bool
is_pps_curr_votable_available(struct oplus_dual_cells_protection *chip)
{
	if (!chip->pps_curr_votable)
		chip->pps_curr_votable = find_votable("PPS_CURR");
	return !!chip->pps_curr_votable;
}

static void oplus_protection_wired_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_dual_cells_protection *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			schedule_delayed_work(&chip->plugin_work, 0);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_protection_check_temp_region(struct oplus_dual_cells_protection *chip, bool sync)
{
	int ret;
	enum oplus_temp_region bat_temp_region;
	union mms_msg_data data = { 0 };

	if (chip->comm_topic != NULL) {
		ret = oplus_mms_get_item_data(chip->comm_topic,
					COMM_ITEM_TEMP_REGION, &data, sync);
		if (ret < 0) {
			chip->is_temp_normal = true;
			chg_err("can't get COMM_ITEM_TEMP_REGION status, ret = %d", ret);
		} else {
			bat_temp_region = data.intval;
			if (bat_temp_region > TEMP_REGION_PRE_NORMAL && bat_temp_region < TEMP_REGION_WARM)
				chip->is_temp_normal = true;
			else
				chip->is_temp_normal = false;
		}
	} else {
		chip->is_temp_normal = true;
	}
}

static void oplus_protection_comm_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_dual_cells_protection *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_TEMP_REGION:
			oplus_protection_check_temp_region(chip, false);
			break;
		case COMM_ITEM_CHG_FULL:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->batt_full = !!data.intval;
			if (chip->batt_full) {
				oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data, false);
				chip->batt_cc = data.intval;
			}
			break;
		case COMM_ITEM_RECHGING:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->rechging = !!data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_check_charging_type(struct oplus_dual_cells_protection *chip)
{
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_CHARGING, &data, false);
	if (rc < 0)
		chip->ufcs_charging = false;
	else
		chip->ufcs_charging = !!data.intval;

	rc = oplus_mms_get_item_data(chip->pps_topic, PPS_ITEM_CHARGING, &data, false);
	if (rc < 0)
		chip->pps_charging = false;
	else
		chip->pps_charging = !!data.intval;

	rc = oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_CHARGING, &data, false);
	if (rc < 0)
		chip->vooc_charging = false;
	else
		chip->vooc_charging = !!data.intval;

	rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_FASTCHG_STATUS, &data, false);
	if (rc < 0)
		chip->wls_fastchg_charging = false;
	else
		chip->wls_fastchg_charging = !!data.intval;
}

static void oplus_protection_limit_curr(struct oplus_dual_cells_protection *chip)
{
	int curr = 0;
	if (!chip) {
		chg_err("chip is null");
		return;
	}

	curr = chip->spec.cur_limit;

	if (chip->batt_health) {
		if (is_vooc_curr_votable_available(chip))
			vote(chip->vooc_curr_votable, DUAL_CELLS_PROTECTION_VOTER,
				 false, 0, false);

		if (is_ufcs_curr_votable_available(chip))
			vote(chip->ufcs_curr_votable, DUAL_CELLS_PROTECTION_VOTER,
				 false, 0, false);

		if (is_pps_curr_votable_available(chip))
			vote(chip->pps_curr_votable, DUAL_CELLS_PROTECTION_VOTER,
				 false, 0, false);

		if (is_wls_fcc_votable_available(chip))
			vote(chip->pps_curr_votable, DUAL_CELLS_PROTECTION_VOTER,
				 false, 0, false);
	} else {
		oplus_check_charging_type(chip);
		if (chip->ufcs_charging && is_ufcs_curr_votable_available(chip))
			vote(chip->ufcs_curr_votable, DUAL_CELLS_PROTECTION_VOTER,
				 true, curr, false);

		if (chip->pps_charging && is_pps_curr_votable_available(chip))
			vote(chip->pps_curr_votable, DUAL_CELLS_PROTECTION_VOTER,
				 true, curr, false);

		if (chip->vooc_charging && is_vooc_curr_votable_available(chip))
			vote(chip->vooc_curr_votable, DUAL_CELLS_PROTECTION_VOTER,
				true, curr, false);

		if (chip->wls_fastchg_charging && is_wls_fcc_votable_available(chip))
			vote(chip->wls_fcc_votable, DUAL_CELLS_PROTECTION_VOTER,
				true, curr, false);
	}
}

static void oplus_protection_fcc_check(struct oplus_dual_cells_protection *chip)
{
	static int fcc_reduce_count = 0;
	static int batt_cc = INVALID_CC;
	static int pre_batt_cc = INVALID_CC;

	if (!chip->is_temp_normal)
		return;

	if (!chip->batt_full)
		return;

	batt_cc = chip->batt_cc;
	if (batt_cc == pre_batt_cc)
		return;

	if (chip->batt_fcc < chip->spec.min_fcc) {
		if (fcc_reduce_count < FCC_SMALL_THR)
			fcc_reduce_count++;
		if (chip->fcc_track_support || chip->fcc_check_support) {
			chip->reason = FCC_SMALL;
			schedule_delayed_work(&chip->dual_cells_protect_track_work, 0);
			msleep(DUAL_CELLS_TRACK_INTERVAL_MS);
		}
		chg_info("fcc is too small, count = %d, batt_fcc %d, batt_cc %d",
			fcc_reduce_count, chip->batt_fcc, chip->batt_cc);
	} else {
		fcc_reduce_count = 0;
		chg_info("fcc %d is more than min_fcc, cancel count", chip->batt_fcc);
	}

	pre_batt_cc = batt_cc;

	if (fcc_reduce_count >= FCC_SMALL_THR) {
		fcc_reduce_count = 0;
		chip->reason = FCC_SMALL;
		if (chip->fcc_check_support)
			chip->batt_health = false;
		chg_err("fcc is too small, batt is not health");
	}
}

#define SECOND_TO_HOUR 3600
#define CHECK_INTERVAL 5
#define INTER_FCC_LOW_THR 100

static void oplus_protection_integral_fcc_cal(struct oplus_dual_cells_protection *chip)
{
	int capacity = 0;
        int ibat = 0;
	if (!chip) {
		chg_err("chip is null");
		return;
	}
        ibat = -chip->ibat_ma;

	if (!chip->is_temp_normal) {
		chip->intfcc_valid = false;
		return;
	}

	if (!chip->wired_online)
		return;

	if (chip->intfcc_checked)
		return;

	if (chip->rechging)
		return;

	if (chip->plugin_soc != INVALID_SOC &&
		chip->plugin_soc > FCC_INTEGRAL_THR)
		return;

	capacity = (ibat * CHECK_INTERVAL * INT_FCC_UNIT) / SECOND_TO_HOUR;
	chip->batt_integral_fcc += capacity;

	if (chip->batt_full && chip->ibat_ma > -BATT_FULL_IBAT_MA &&
		chip->intfcc_valid && chip->batt_integral_fcc > INTER_FCC_LOW_THR)
		chip->intfcc_checked = true;
}

static void oplus_protection_integral_fcc_check(struct oplus_dual_cells_protection *chip)
{
	static int intfcc_reduce_count = 0;
	static int batt_cc = INVALID_CC;
	static int pre_batt_cc = INVALID_CC;
	int intfcc = 0;

	if (chip->plugin_soc > FCC_INTEGRAL_THR)
		return;

	if (!chip->intfcc_checked)
		return;

	if (chip->rechging || !chip->wired_online)
		return;

	batt_cc = chip->batt_cc;
	if (batt_cc == pre_batt_cc)
		return;

	intfcc = chip->batt_integral_fcc / (100 - chip->plugin_soc);

	if (intfcc < chip->spec.min_oplus_fcc) {
		if (intfcc_reduce_count < FCC_SMALL_THR)
			intfcc_reduce_count++;
		chg_info("integral fcc is too small, count = %d, intfcc %d, batt_cc %d",
			intfcc_reduce_count, intfcc, chip->batt_cc);
		if (chip->intfcc_check_support || chip->intfcc_track_support) {
			chip->reason = INTFCC_SMALL;
			schedule_delayed_work(&chip->dual_cells_protect_track_work, 0);
			msleep(DUAL_CELLS_TRACK_INTERVAL_MS);
		}
	} else {
		intfcc_reduce_count = 0;
		chg_info("intfcc %d is more than min_fcc, cancel count", intfcc);
	}

	pre_batt_cc = batt_cc;

	if (intfcc_reduce_count >= FCC_SMALL_THR) {
		if (chip->intfcc_check_support)
			chip->batt_health = false;
		intfcc_reduce_count = 0;
		chg_err("integral fcc is too small, batt is not health");
	}
}

static void oplus_protection_soc_check(struct oplus_dual_cells_protection *chip)
{
	static int soc_jump_count = 0;
	static int batt_cc = 0;
	static int jump_occur_cc = -1;
	int soc_delta = 0;

	if (!chip->is_temp_normal)
		goto cycle_out;

	batt_cc = chip->batt_cc;

	soc_delta = chip->batt_soc - chip->pre_batt_soc;
	soc_delta = soc_delta >= 0 ? soc_delta : -soc_delta;

	if (soc_delta > chip->spec.soc_jump_thr ||
		(chip->batt_full && chip->batt_soc < 100 - chip->spec.soc_jump_thr)) {
		if (batt_cc == jump_occur_cc)
			goto cycle_out;

		if (jump_occur_cc != -1 && (batt_cc - jump_occur_cc > 5))
			soc_jump_count = 0;

		jump_occur_cc = batt_cc;
		soc_jump_count++;

		chg_err("soc jump occur, soc_jump_count %d, jump_occur_cc %d, pre soc %d, soc %d",
			soc_jump_count, jump_occur_cc, chip->pre_batt_soc, chip->batt_soc);

		if (soc_jump_count >= SOC_JUMP_COUNT_MAX) {
			if (chip->soc_check_support)
				chip->batt_health = false;
			soc_jump_count = 0;
			jump_occur_cc = -1;
			chg_err("batt not health by soc jump");
		}
		if (chip->soc_check_support || chip->soc_track_support) {
			chip->reason = SOC_JUMP;
			schedule_delayed_work(&chip->dual_cells_protect_track_work, 0);
			msleep(DUAL_CELLS_TRACK_INTERVAL_MS);
		}
	}

cycle_out:
	chip->pre_batt_soc = chip->batt_soc;
}

static void oplus_protection_fcc_recover_check(struct oplus_dual_cells_protection *chip)
{
	static int fcc_recover_count = 0;
	static int batt_cc = INVALID_CC;
	static int pre_batt_cc = INVALID_CC;
	int recover_fcc = 0;

	if (!chip->batt_full)
		return;

	if (chip->reason != FCC_SMALL && chip->reason != FCC_RECOVERING)
		return;

	batt_cc = chip->batt_cc;
	if (batt_cc == pre_batt_cc)
		return;

	recover_fcc = chip->spec.cur_recover_thr + chip->spec.min_fcc;

	if (chip->batt_fcc > recover_fcc) {
		if (fcc_recover_count < FCC_RECOVER_THR) {
			fcc_recover_count++;
			chg_info("batt intfcc is recovering, fcc_recover_count = %d, cc = %d, fcc %d \n",
				fcc_recover_count, chip->batt_cc, chip->batt_fcc);
		}
		if (chip->fcc_check_support || chip->fcc_track_support) {
			chip->reason = FCC_RECOVERING;
			schedule_delayed_work(&chip->dual_cells_protect_track_work, 0);
			msleep(DUAL_CELLS_TRACK_INTERVAL_MS);
		}
	} else if (chip->batt_fcc <= recover_fcc) {
		fcc_recover_count = 0;
		chg_info("batt intfcc is not recovering, cc = %d, fcc %d \n",
				chip->batt_cc, chip->batt_fcc);
	}

	pre_batt_cc = batt_cc;

	if (fcc_recover_count >= FCC_RECOVER_THR) {
		if (chip->fcc_check_support)
			chip->batt_health = true;
		fcc_recover_count = 0;
		if (chip->fcc_check_support || chip->fcc_track_support) {
			chip->reason = FCC_RECOVER;
			schedule_delayed_work(&chip->dual_cells_protect_track_work, 0);
		}
		chg_info("batt fcc is recover\n");
	}
}

static void oplus_protection_int_fcc_recover_check(struct oplus_dual_cells_protection *chip)
{
	static int intfcc_recover_count = 0;
	static int batt_cc = INVALID_CC;
	static int pre_batt_cc = INVALID_CC;
	int recover_fcc = 0;
	int intfcc = 0;

	if (chip->plugin_soc != INVALID_SOC &&
		chip->plugin_soc > FCC_INTEGRAL_THR)
		return;

	if (!chip->intfcc_checked || !chip->wired_online)
		return;

	batt_cc = chip->batt_cc;
	if (batt_cc == pre_batt_cc)
		return;

	recover_fcc = chip->spec.cur_recover_thr + chip->spec.min_oplus_fcc;
	intfcc = chip->batt_integral_fcc / (100 - chip->plugin_soc);

	if (intfcc >= recover_fcc) {
		if (intfcc_recover_count < FCC_RECOVER_THR) {
			intfcc_recover_count++;
			chg_info("batt intfcc is recovering, intfcc = %d, intfcc_recover_count = %d, cc = %d \n",
				intfcc, intfcc_recover_count, chip->batt_cc);
		}
		if (chip->intfcc_check_support || chip->intfcc_track_support) {
			chip->reason = INTFCC_RECOVER;
			schedule_delayed_work(&chip->dual_cells_protect_track_work, 0);
		}
	} else {
		intfcc_recover_count = 0;
		chg_info("intfcc is not recover, intfcc = %d, cc = %d \n",
				intfcc, chip->batt_cc);
	}

	pre_batt_cc = batt_cc;

	if (intfcc_recover_count >= FCC_RECOVER_THR) {
		if (chip->intfcc_check_support)
			chip->batt_health = true;
		intfcc_recover_count = 0;
		chip->reason = INTFCC_RECOVER;
		chg_info("integral batt fcc is recover\n");
	}
}

#define GAUGE_ERROR_RETRY_MAX 3
static void oplus_protection_plugin_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_dual_cells_protection *chip =
		container_of(dwork, struct oplus_dual_cells_protection, plugin_work);
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data, true);
	chip->wired_online = !!data.intval;

	if (chip->first_boot) {
		chg_info("first boot, return\n");
		return;
	}

	if (chip->wired_online) {
		if (!is_gauge_topic_available(chip))
			return;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, true);
		chip->plugin_soc = data.intval;
		chip->pre_batt_soc = data.intval;
		chip->batt_soc = data.intval;
		chip->batt_integral_fcc = 0;
		chip->intfcc_checked = false;
		oplus_protection_check_temp_region(chip, true);
		chg_info("wired_online, plugin_soc %d, batt temp normal %d\n",
			chip->plugin_soc, chip->is_temp_normal);
		if (chip->is_temp_normal)
			chip->intfcc_valid = true;
		else
			chip->intfcc_valid = false;
	} else {
		/*wired offline, cancel all param*/
		chg_info("wired offline, cancel protection\n");
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, true);
		chip->pre_batt_soc = data.intval;
		chip->batt_soc = data.intval;
		chip->plugin_soc = INVALID_SOC;
		chip->batt_integral_fcc = 0;
		chip->intfcc_checked = false;
		if (is_vooc_curr_votable_available(chip))
			vote(chip->vooc_curr_votable, DUAL_CELLS_PROTECTION_VOTER,
				 false, 0, false);

		if (is_ufcs_curr_votable_available(chip))
			vote(chip->ufcs_curr_votable, DUAL_CELLS_PROTECTION_VOTER,
				 false, 0, false);

		if (is_pps_curr_votable_available(chip))
			vote(chip->pps_curr_votable, DUAL_CELLS_PROTECTION_VOTER,
				 false, 0, false);

		if (is_wls_fcc_votable_available(chip))
			vote(chip->wls_fcc_votable, DUAL_CELLS_PROTECTION_VOTER,
				 false, 0, false);
	}
	schedule_delayed_work(&chip->batt_check_work, 0);
}

static void oplus_dual_cells_protect_get_track_info(struct oplus_dual_cells_protection *chip,
	struct dual_cells_protect_track_info *info)
{
	if (chip == NULL || info == NULL) {
		chg_err("chip or info is null, return\n");
		return;
	}

	if (chip->debug_reason)
		info->reason = chip->debug_reason;
	else
		info->reason = chip->reason;
	info->batt_status = chip->batt_health;
	info->batt_cc = chip->batt_cc;
	info->batt_fcc = chip->batt_fcc;
	info->soc = chip->batt_soc;
	info->pre_soc = chip->pre_batt_soc;
	info->plugin_soc = chip->plugin_soc;
	info->batt_intfcc = chip->batt_integral_fcc / (100 - chip->plugin_soc);
	info->xvdd_occur = chip->xvdd_occur;
	info->sn_change = chip->sn_change_occur;
}

static int oplus_dual_cells_protect_pack_track_info(char *buf,
	struct dual_cells_protect_track_info *info)
{
	int index = 0;
	if (buf == NULL || info == NULL) {
		chg_err("buf or info is null, return\n");
		return -EINVAL;
	}

	index += scnprintf(buf + index, DUAL_CELLS_PROTECT_TRACK_INFO_LEN - index,
		"$$track_reason@@%s", get_protect_reason_str(info->reason));
	index += scnprintf(buf + index, DUAL_CELLS_PROTECT_TRACK_INFO_LEN - index,
		"$$batt_status@@%d", info->batt_status);
	index += scnprintf(buf + index, DUAL_CELLS_PROTECT_TRACK_INFO_LEN - index,
		"$$batt_cycle@@%d", info->batt_cc);
	index += scnprintf(buf + index, DUAL_CELLS_PROTECT_TRACK_INFO_LEN - index,
		"$$batt_fcc@@%d", info->batt_fcc);
	index += scnprintf(buf + index, DUAL_CELLS_PROTECT_TRACK_INFO_LEN - index,
		"$$batt_soc@@%d", info->soc);
	index += scnprintf(buf + index, DUAL_CELLS_PROTECT_TRACK_INFO_LEN - index,
		"$$batt_pre_soc@@%d", info->pre_soc);
	index += scnprintf(buf + index, DUAL_CELLS_PROTECT_TRACK_INFO_LEN - index,
		"$$plugin_soc@@%d", info->plugin_soc);
	index += scnprintf(buf + index, DUAL_CELLS_PROTECT_TRACK_INFO_LEN - index,
		"$$batt_intfcc@@%d", info->batt_intfcc);
	index += scnprintf(buf + index, DUAL_CELLS_PROTECT_TRACK_INFO_LEN - index,
		"$$xvdd_occur@@%d", info->xvdd_occur);
	index += scnprintf(buf + index, DUAL_CELLS_PROTECT_TRACK_INFO_LEN - index,
		"$$sn_change@@%d", info->sn_change);

	if (index > DUAL_CELLS_PROTECT_TRACK_INFO_LEN) {
		chg_err("track info exceeds length limit.");
		return -EINVAL;
	}

	return index;
}

static void oplus_dual_cells_protect_track_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_dual_cells_protection *chip =
		container_of(dwork, struct oplus_dual_cells_protection, dual_cells_protect_track_work);

	struct dual_cells_protect_track_info info;
	char *buf = NULL;
	int len = 0;
	struct mms_msg *topic_msg;
	int rc = 0;

	if (!chip) {
		chg_err("chip is null, return\n");
		return;
	}

	buf = kzalloc(DUAL_CELLS_PROTECT_TRACK_INFO_LEN * sizeof(char), GFP_KERNEL);
	if (buf == NULL) {
		chg_err("buf alloc error.\n");
		return;
	}

	/* get protection info */
	oplus_dual_cells_protect_get_track_info(chip, &info);
	/* pack info */
	len = oplus_dual_cells_protect_pack_track_info(buf, &info);
	/* creat err msg and trigger */
	if (is_err_topic_available(chip)) {
		topic_msg =
			oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, ERR_ITEM_IC,
						"[%s]-[%d]-[%d]:%s", "dual_cells_protect",
						OPLUS_IC_ERR_GAUGE, TRACK_GAGUE_ERR_BATT_CELLS_DAMAGE, buf);
		if (topic_msg == NULL) {
			chg_err("alloc topic msg error\n");
		} else {
			rc = oplus_mms_publish_msg_sync(chip->err_topic, topic_msg);
			if (rc < 0) {
				chg_err("publish topic msg error, rc=%d\n", rc);
				kfree(topic_msg);
			}
		}
	}
	chip->debug_reason = 0;
	kfree(buf);
}

static void oplus_protection_batt_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
        struct oplus_dual_cells_protection *chip = container_of(
			dwork, struct oplus_dual_cells_protection, batt_check_work);

	oplus_protection_integral_fcc_cal(chip);

	/* batt is health, Check for damage */
	if (chip->batt_health) {
		oplus_protection_soc_check(chip);
		oplus_protection_fcc_check(chip);
		oplus_protection_integral_fcc_check(chip);
	} else {
	/* batt is not health, Check whether it can be restored */
		oplus_protection_fcc_recover_check(chip);
		oplus_protection_int_fcc_recover_check(chip);
	}

	oplus_protection_limit_curr(chip);
	chg_info("batt_health %d, ibat %d, integral_fcc %d, batt fcc %d, batt_cc %d, batt_full %d, plug_soc %d, soc %d, reason %d\n",
			 chip->batt_health, chip->ibat_ma, chip->batt_integral_fcc, chip->batt_fcc,
			 chip->batt_cc, chip->batt_full, chip->plugin_soc, chip->batt_soc, chip->reason);
}

static void oplus_protection_gauge_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_dual_cells_protection *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data,
				false);
		chip->batt_cc = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data,
				false);
		chip->batt_fcc = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data,
				false);
		chip->batt_soc = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data,
				false);
		chip->ibat_ma = data.intval;
		if (chip->first_boot) {
			chip->first_boot = false;
			schedule_delayed_work(&chip->plugin_work, 0);
			chg_err("first boot\n");
		} else {
			schedule_delayed_work(&chip->batt_check_work, 0);
		}
		break;
	default:
		break;
	}
}

static void oplus_protection_subscribe_common_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_dual_cells_protection *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->comm_topic = topic;

	chip->comm_subs =
		oplus_mms_subscribe(chip->comm_topic, chip,
				    oplus_protection_comm_subs_callback,
				    "protection");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe comm topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_CHG_FULL,
				&data, true);
	chip->batt_full = !!data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_RECHGING,
				&data, true);
	chip->rechging = !!data.intval;

	oplus_protection_check_temp_region(chip, true);
}

static void oplus_protection_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_dual_cells_protection *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;

	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_protection_wired_subs_callback,
				    "protection");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
					true);
	chip->wired_online = !!data.intval;

	chg_err("subscribe wired topic, online %d\n", chip->wired_online);
}

static void oplus_protection_subscribe_ufcs_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_dual_cells_protection *chip = prv_data;
	chip->ufcs_topic = topic;
}

static void oplus_protection_subscribe_vooc_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_dual_cells_protection *chip = prv_data;
	chip->vooc_topic = topic;
}

static void oplus_protection_subscribe_pps_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_dual_cells_protection *chip = prv_data;
	chip->pps_topic = topic;
}

static void oplus_protection_subscribe_wls_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_dual_cells_protection *chip = prv_data;
	chip->wls_topic = topic;
}

static void oplus_protection_subscribe_gauge_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_dual_cells_protection *chip = prv_data;
	union mms_msg_data data = { 0 };
	chip->gauge_topic = topic;

	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
				    oplus_protection_gauge_subs_callback,
				    "protection");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data,
				true);
	chip->batt_cc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data,
				true);
	chip->batt_fcc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data,
				true);
	chip->batt_soc = data.intval;
	chip->pre_batt_soc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data,
				true);
	chip->ibat_ma = data.intval;
}

static int oplus_dual_cells_batt_status(struct oplus_mms *mms,
					union mms_msg_data *data)
{
	struct oplus_dual_cells_protection *chip;
	if (mms == NULL) {
		chg_err("topic is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);
	if (!chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	data->intval = chip->batt_health;
	return 0;
}

static void oplus_protection_topic_update(struct oplus_mms *mms, bool publish)
{
}

static struct mms_item oplus_protection_item[] = {
	{
		.desc = {
			.item_id = DUAL_CELLS_BATT_STATUS,
			.update = oplus_dual_cells_batt_status,
		}
	}
};

static const struct oplus_mms_desc oplus_protection_desc = {
	.name = "protection",
	.type = OPLUS_MMS_TYPE_PROTECTION,
	.item_table = oplus_protection_item,
	.item_num = ARRAY_SIZE(oplus_protection_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_protection_topic_update,
};

static int oplus_protection_topic_init(struct oplus_dual_cells_protection *chip)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;
	mms_cfg.drv_data = chip;
	mms_cfg.of_node = chip->dev->of_node;

	chip->protection_topic =
		devm_oplus_mms_register(chip->dev, &oplus_protection_desc, &mms_cfg);

	if (IS_ERR(chip->protection_topic)) {
		chg_err("Couldn't register protection topic\n");
		rc = PTR_ERR(chip->protection_topic);
		return rc;
	}

	oplus_mms_wait_topic("common", oplus_protection_subscribe_common_topic, chip);
	oplus_mms_wait_topic("gauge", oplus_protection_subscribe_gauge_topic, chip);
	oplus_mms_wait_topic("wired", oplus_protection_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("ufcs", oplus_protection_subscribe_ufcs_topic, chip);
	oplus_mms_wait_topic("pps", oplus_protection_subscribe_pps_topic, chip);
	oplus_mms_wait_topic("vooc", oplus_protection_subscribe_vooc_topic, chip);
	oplus_mms_wait_topic("wireless", oplus_protection_subscribe_wls_topic, chip);

	return 0;
}

static int oplus_dual_cells_protection_chip_init(struct oplus_dual_cells_protection *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct oplus_dual_cells_protection_spec *spec = &chip->spec;
	int rc = 0;

	rc = of_property_read_u32(node, "oplus_spec,cur_limit", &spec->cur_limit);
	if (rc < 0) {
		chg_err("read cur_limit failed, use default, rc %d\n", rc);
		spec->cur_limit = DUAL_CELLS_PROTECTION_CUR_LIMIT_DEFAULT;
	}

	chip->fcc_check_support = of_property_read_bool(node, "oplus,fcc_check_support");
	chip->fcc_track_support = of_property_read_bool(node, "oplus,fcc_track_support");
	if (chip->fcc_check_support || chip->fcc_track_support) {
		rc = of_property_read_u32(node, "oplus_spec,min-fcc", &spec->min_fcc);
		if (rc < 0)
			return -ENODEV;
	}

	chip->soc_check_support = of_property_read_bool(node, "oplus,soc_check_support");
	chip->soc_track_support = of_property_read_bool(node, "oplus,soc_track_support");
	if (chip->soc_check_support || chip->soc_track_support) {
		rc = of_property_read_u32(node, "oplus_spec,soc-jump-thr", &spec->soc_jump_thr);
		if (rc < 0)
			return -ENODEV;
	}

	chip->intfcc_check_support = of_property_read_bool(node, "oplus,intfcc_check_support");
	chip->intfcc_track_support = of_property_read_bool(node, "oplus,intfcc_track_support");
	if (chip->intfcc_check_support || chip->intfcc_track_support) {
		rc = of_property_read_u32(node, "oplus_spec,min-oplus-fcc", &spec->min_oplus_fcc);
		if (rc < 0)
			return -ENODEV;
	}

	rc = of_property_read_u32(node, "oplus_spec,cur-recover-thr", &spec->cur_recover_thr);
	if (rc < 0) {
		spec->cur_recover_thr = 200;
		chg_err("read cur-recover-thr failed, use default, rc=%d\n", rc);
	}

	return 0;
}

int oplus_chg_get_dual_cells_batt_health(struct oplus_mms *topic, int *status, int *reason)
{
	struct oplus_dual_cells_protection *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);

	*status = chip->batt_health;
	*reason = chip->reason;

	return 0;
}

void oplus_chg_set_dual_cells_batt_health(struct oplus_mms *topic, int val)
{
	struct oplus_dual_cells_protection *chip;
	int reason = 0;
	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);

	chip->batt_health = !!(val & 0x1);

	chip->xvdd_occur = !!((val & 0x4) >> 2);
	if (chip->xvdd_occur) {
		chip->reason = XVDD_OCCUR;
		schedule_delayed_work(&chip->dual_cells_protect_track_work, 0);
		chg_info("xvdd occur, track\n");
		msleep(DUAL_CELLS_TRACK_INTERVAL_MS);
	}
	chip->sn_change_occur = !!((val & 0x2) >> 1);
	if (chip->sn_change_occur) {
		chip->reason = SN_CHANGE;
		schedule_delayed_work(&chip->dual_cells_protect_track_work, 0);
		chg_info("sn change occur, track\n");
		msleep(DUAL_CELLS_TRACK_INTERVAL_MS);
	}

	reason = (val >> 3) & 0xf;
	if (reason >= PROTECT_UNKNOWN && reason < REASON_MAX)
		chip->reason = reason;

	chg_info("batt_health and reason from hidl xvdd %d, sn %d, batt_health %d, reason %d\n",
		chip->xvdd_occur, chip->sn_change_occur, chip->batt_health, chip->reason);
	chip->xvdd_occur = 0;
	chip->sn_change_occur = 0;
}

void oplus_chg_set_dual_cells_protect_track_debug(struct oplus_mms *topic, int val)
{
	struct oplus_dual_cells_protection *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);

	if (val) {
		chip->debug_reason = val;
		schedule_delayed_work(&chip->dual_cells_protect_track_work, 0);
		chg_info("debug track, reason %d\n", chip->debug_reason);
	}
}

int oplus_chg_get_dual_cells_protect_track_debug(struct oplus_mms *topic)
{
	struct oplus_dual_cells_protection *chip;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return -ENODEV;
	}

	chip = oplus_mms_get_drvdata(topic);

	return chip->debug_reason;
}

static int oplus_chg_dual_cells_protection_probe(struct platform_device *pdev)
{
	struct oplus_dual_cells_protection *chip;
	int rc = 0;

	if (pdev == NULL) {
		chg_err("oplus_chg_state_retention_probe input pdev error\n");
		return -ENODEV;
	}

	chg_info("start\n");

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_dual_cells_protection), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc dual_cells_protection buffer error\n");
		return 0;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	rc = oplus_dual_cells_protection_chip_init(chip);
	if (rc < 0) {
		chg_err("oplus dual cells protection init error, rc=%d\n", rc);
		goto parse_dt_err;
	}
	chip->batt_health = true;
	chip->pre_batt_health = true;
	chip->first_boot = true;

	rc = oplus_protection_topic_init(chip);
	if (rc < 0) {
		chg_err("oplus dual cells protection topic init error, rc=%d\n", rc);
		goto topic_reg_err;
	}

	INIT_DELAYED_WORK(&chip->plugin_work, oplus_protection_plugin_work);
	INIT_DELAYED_WORK(&chip->batt_check_work, oplus_protection_batt_check_work);
	INIT_DELAYED_WORK(&chip->dual_cells_protect_track_work, oplus_dual_cells_protect_track_work);

	chg_info("end\n");
	return 0;

topic_reg_err:
parse_dt_err:
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, chip);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_dual_cells_protection_remove(struct platform_device *pdev)
#else
static int oplus_dual_cells_protection_remove(struct platform_device *pdev)
#endif
{
	struct oplus_dual_cells_protection *chip = platform_get_drvdata(pdev);

	if (!IS_ERR_OR_NULL(chip->wired_subs))
		oplus_mms_unsubscribe(chip->wired_subs);
	if (!IS_ERR_OR_NULL(chip->gauge_subs))
		oplus_mms_unsubscribe(chip->gauge_subs);
	if (!IS_ERR_OR_NULL(chip->comm_subs))
		oplus_mms_unsubscribe(chip->comm_subs);
	devm_kfree(&pdev->dev, chip);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}


static const struct of_device_id oplus_dual_cells_protection_match[] = {
	{ .compatible = "oplus,dual_cells_protection" },
	{},
};

static struct platform_driver oplus_dual_cells_protection_driver = {
	.driver		= {
		.name = "oplus-dual_cells_protection",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_dual_cells_protection_match),
	},
	.probe		= oplus_chg_dual_cells_protection_probe,
	.remove		= oplus_dual_cells_protection_remove,
};

static __init int oplus_dual_cells_protection_init(void)
{
	return platform_driver_register(&oplus_dual_cells_protection_driver);
}

static __exit void oplus_dual_cells_protection_exit(void)
{
	platform_driver_unregister(&oplus_dual_cells_protection_driver);
}

oplus_chg_module_register(oplus_dual_cells_protection);
