/***********************************************************
** Copyright (C), 2008-2025 Oplus. All rights reserved.
** File: oplus_dischg_boost.c
** Description: dischg boost
** Date: 2025-11-01
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/

#define pr_fmt(fmt) "[CHG_DISCHG_BOOST]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/power_supply.h>
#include <oplus_parallel.h>
#include <linux/ktime.h>
#include <linux/sched/clock.h>
#include <linux/iio/consumer.h>
#include <uapi/linux/sched/types.h>
#include <linux/thermal.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/rtc.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/random.h>
#include <linux/pm_wakeup.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
#include <linux/msm_drm_notify.h>
#endif
#if IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY) || IS_ENABLED(CONFIG_OPLUS_CHG_DRM_PANEL_NOTIFY)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <drm/drm_panel.h>
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include <linux/mtk_panel_ext.h>
#include <linux/mtk_disp_notify.h>
#endif
#else
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG)
#include <mtk_panel_ext.h>
#include <mtk_disp_notify.h>
#endif
#endif

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mt-plat/mtk_boot_common.h>
#endif

#include <oplus_chg.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_module.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_wls.h>
#include <oplus_dischg_boost.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_pps.h>
#include <oplus_chg_plc.h>
#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#include "oplus_cfg.h"
#endif
#include <oplus_chg_state_retention.h>
#include <oplus_chg_mutual.h>
#include <oplus_chg_cpa.h>

#define REASON_LENGTH_MAX 256

struct oplus_boost_config {
	int ic_type;
	int init_cv_mv;
	int plugin_cv_mv;
	int plugout_cv_mv;
	int suspend_cv_mv;
	int resume_cv_mv;
	int normalchg_cv_mv;
	int fastchg_cv_mv;
	int plugin_bcl_rate;
	int plugout_bcl_rate;
	int auto_mode_soc_thr;
	int boost_ratio;
};

struct oplus_dischg_boost {
	struct device *dev;
	struct oplus_mms *wired_topic;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *comm_topic;
	struct oplus_mms *vooc_topic;
	struct oplus_mms *wls_topic;
	struct oplus_mms *cpa_topic;
	struct oplus_mms *dischg_boost_topic;
	struct mms_subscribe *gauge_subs;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *wls_subs;
	struct mms_subscribe *comm_subs;
	struct mms_subscribe *vooc_subs;
	struct mms_subscribe *cpa_subs;

	struct oplus_chg_ic_dev *boost_ic;

	struct delayed_work dischg_boost_init_work;
	struct delayed_work boost_cv_dynamic_curr_work;
	struct delayed_work pending_ops_work;

	struct oplus_boost_config config;
	struct votable *wired_curr_votable;
	struct votable *wls_curr_votable;
	struct votable *work_mode_votable;

	bool wired_online;
	bool wls_online;
	bool vooc_started;
	struct work_struct plugin_work;
	struct delayed_work set_automode_work;

	/* Pending operations to be executed on resume */
	bool otg_mode_pending;
	bool otg_mode_pending_value;
	bool fam_en_pending;
	bool fam_en_pending_value;

	/* Wakelock for critical I2C operations */
	struct wakeup_source *i2c_wake_lock;

	int vbat_mv;
	int bat_soc;
	int cv_now_mv;
	int ic_type;
	int dev_id;
	int cv_mv;
	int cv_mode_cnt;
};

/* Helper function to check if boost IC is suspended through IC function interface */
static bool oplus_boost_is_ic_suspended(struct oplus_dischg_boost *chip)
{
	bool suspended = false;
	int rc;

	if (!chip || !chip->boost_ic)
		return false;

	/* Check suspend state through IC function interface (works for sc83107, sc8527, etc.) */
	rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_IS_SUSPEND, &suspended);
	if (rc < 0) {
		/* If function is not supported, assume not suspended */
		if (rc != -ENOTSUPP)
			chg_err("Failed to get boost IC suspend state, rc=%d\n", rc);
		return false;
	}

	return suspended;
}

__maybe_unused static struct votable *
oplus_boost_get_curr_votable(struct oplus_dischg_boost *chip)
{
	struct votable *votable = NULL;

	if (chip->wls_online) {
		if (!chip->wls_curr_votable)
			chip->wls_curr_votable = find_votable("WLS_NOR_FCC");
		votable = chip->wls_curr_votable;
	} else if (chip->wired_online) {
		if (!chip->wired_curr_votable)
			chip->wired_curr_votable = find_votable("WIRED_FCC");
		votable = chip->wired_curr_votable;
	}

	return votable;
}

__maybe_unused static bool
is_work_mode_votable_available(struct oplus_dischg_boost *chip)
{
	if (!chip->work_mode_votable)
			chip->work_mode_votable = find_votable("BOOST_WORK_MODE");
		return !!chip->work_mode_votable;
}

static int oplus_boost_get_vbat(struct oplus_dischg_boost *chip, int *vbat_mv)
{
	union mms_msg_data data = { 0 };
	if (!chip || !chip->gauge_topic) {
		chg_err("chip or gauge_topic is null\n");
		return -ENODEV;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	*vbat_mv = data.intval;

	return 0;
}

__maybe_unused
static int oplus_boost_get_cv_mv(struct oplus_dischg_boost *chip, int *cv)
{
	int rc;

	if (chip->boost_ic == NULL) {
		chg_err("boost_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_GET_CV, cv);
	chg_info("boost get cv = %d\n", *cv);

	return rc;
}

__maybe_unused
static int oplus_boost_set_cv_mv(struct oplus_dischg_boost *chip, int mv)
{
	int rc;

	if (chip->boost_ic == NULL) {
		chg_err("boost_ic is NULL\n");
		return -ENODEV;
	}

	if (!mv) {
		chg_err("cv is not allowed to be 0\n");
		return -EINVAL;
	}

	chg_info("boost set cv=%d\n", mv);
	rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_SET_CV, mv);
	chip->cv_now_mv = mv;

	return rc;
}

__maybe_unused
static int oplus_boost_enable_otg_mode(struct oplus_dischg_boost *chip, bool en)
{
	int rc;

	if (chip->boost_ic == NULL) {
		chg_err("boost_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_ENABLE_OTG_MODE, en);

	return rc;
}

__maybe_unused
static int oplus_boost_enable_fam_en(struct oplus_dischg_boost *chip, bool en)
{
	int rc;

	if (chip->boost_ic == NULL) {
		chg_err("boost_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_SET_FAM_EN, en);

	return rc;
}

__maybe_unused
static int oplus_boost_set_work_mode(struct oplus_dischg_boost *chip, bool mode)
{
	int rc;

	if (chip->boost_ic == NULL) {
		chg_err("boost_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_SET_WORK_MODE, mode);

	return rc;
}

static int oplus_boost_work_mode_vote_callback(struct votable *votable,
					       void *data, int mode,
					       const char *client,
					       bool step)
{
	struct oplus_dischg_boost *chip = data;
	int rc;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	chg_info("work mode change to %d by %s\n", mode, client);
	rc = oplus_boost_set_work_mode(chip, !!mode);
	if (rc < 0)
		chg_err("set work mode error, rc=%d\n", rc);

	return rc;
}

__maybe_unused
static int oplus_boost_set_bcl_rate(struct oplus_dischg_boost *chip, int rate)
{
	int rc;

	if (chip->boost_ic == NULL) {
		chg_err("boost_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_SET_BCL_RATE, rate);

	return rc;
}

__maybe_unused
static int oplus_boost_set_bcl_vol(struct oplus_dischg_boost *chip, int vol0, int vol1, int vol2)
{
	int rc;

	if (chip->boost_ic == NULL) {
		chg_err("boost_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_SET_BCL_VOL, vol0, vol1, vol2);

	return rc;
}

__maybe_unused
static int oplus_boost_get_in_cv_mode(struct oplus_dischg_boost *chip, bool *cv_mode)
{
	int rc;

	if (chip->boost_ic == NULL) {
		chg_err("boost_ic is NULL\n");
		return -ENODEV;
	}

	rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_GET_IN_CV_MODE, cv_mode);

	return rc;
}

bool oplus_boost_get_cv_mode(struct oplus_mms *topic)
{
	struct oplus_dischg_boost *chip;
	int rc;
	bool cv_mode = false;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return false;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return false;

	rc = oplus_boost_get_in_cv_mode(chip, &cv_mode);
	if (rc < 0) {
		chg_err("get cv mode, err\n");
		return false;
	} else {
		chg_info("get cv mode is %d\n", cv_mode);
		return cv_mode;
	}
}

int oplus_boost_cv_mv_show(struct oplus_mms *topic)
{
	struct oplus_dischg_boost *chip;
	int rc;
	int cv_mv;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return 0;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return 0;

	rc = oplus_boost_get_cv_mv(chip, &cv_mv);
	if (cv_mv < 0) {
		chg_err("cv is < 0, err\n");
		return 0;
	} else {
		chg_info("cv is %d\n", cv_mv);
		return cv_mv;
	}
}

void oplus_boost_disable_auto_mode_store(struct oplus_mms *topic, int val)
{
	struct oplus_dischg_boost *chip;
	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	int rc;
	size_t index;
	union mms_msg_data data = { 0 };
	int vbat_mv = 0;
	int bat_soc = 0;
	struct oplus_mms *comm_topic = NULL;
	int ui_soc = -1;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return;

	/* 0: auto mode  1:force bypass mode */
	chg_info("set oplus_boost_disable_auto_mode = %d\n", val);
	if (is_work_mode_votable_available(chip)) {
		vote(chip->work_mode_votable, BOOST_RUS_VOTER, val, val, false);
	} else {
		chg_err("work_mode_votable not found\n");
	}

	/* Upload track info for RUS setting */
	err_topic = oplus_mms_get_by_name("error");
	if (err_topic) {
		char track_buf[REASON_LENGTH_MAX] = { 0 };

		/* Get battery info for track */
		if (chip->gauge_topic) {
			oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, false);
			vbat_mv = data.intval;
			oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, false);
			bat_soc = data.intval;
		}

		/* Get UI SOC from common topic */
		comm_topic = oplus_mms_get_by_name("common");
		if (comm_topic) {
			oplus_mms_get_item_data(comm_topic, COMM_ITEM_UI_SOC, &data, false);
			ui_soc = data.intval;
		}

		index = scnprintf(track_buf, REASON_LENGTH_MAX,
			"$$device_id@@%s", "dischg_boost");
		index += scnprintf(track_buf + index, REASON_LENGTH_MAX - index,
			"$$err_scene@@%s", "RUS_CONFIG");
		index += scnprintf(track_buf + index, REASON_LENGTH_MAX - index,
			"$$err_reason@@%s", "boost_disable_auto_mode");
		index += scnprintf(track_buf + index, REASON_LENGTH_MAX - index,
			"$$work_mode@@%d", val);
		index += scnprintf(track_buf + index, REASON_LENGTH_MAX - index,
			"$$vbat_mv@@%d", vbat_mv);
		index += scnprintf(track_buf + index, REASON_LENGTH_MAX - index,
			"$$bat_soc@@%d", bat_soc);
		index += scnprintf(track_buf + index, REASON_LENGTH_MAX - index,
			"$$ui_soc@@%d", ui_soc);
		index += scnprintf(track_buf + index, REASON_LENGTH_MAX - index,
			"$$wired_online@@%d", chip->wired_online ? 1 : 0);
		index += scnprintf(track_buf + index, REASON_LENGTH_MAX - index,
			"$$wls_online@@%d", chip->wls_online ? 1 : 0);

		msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
			ERR_ITEM_ERR_PHY_CP_INFO, track_buf);
		if (msg) {
			rc = oplus_mms_publish_msg_sync(err_topic, msg);
			if (rc < 0) {
				chg_err("publish boost RUS config track msg error, rc=%d\n", rc);
				kfree(msg);
			} else {
				chg_info("upload boost RUS config track: work_mode=%d, vbat=%dmV, bat_soc=%d%%, ui_soc=%d%%, wired_online=%d, wls_online=%d\n",
					val, vbat_mv, bat_soc, ui_soc, chip->wired_online ? 1 : 0, chip->wls_online ? 1 : 0);
			}
		} else {
			chg_err("alloc boost RUS config track msg error\n");
		}
	} else {
		chg_err("error topic not found, skip track upload\n");
	}

	return;
}

void oplus_boost_cv_mv_store(struct oplus_mms *topic, int val)
{
	struct oplus_dischg_boost *chip;
	int rc;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return;

	chg_info("set cv = %d\n", val);
	rc = oplus_boost_set_cv_mv(chip, val);
	if (rc < 0)
		chg_err("set cv err, rc = %d\n", rc);

	return;
}

void oplus_boost_set_otg_mode(struct oplus_mms *topic, bool en)
{
	struct oplus_dischg_boost *chip;
	int rc;

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return;

	/* If boost IC is suspended, cache the request and execute on resume */
	if (oplus_boost_is_ic_suspended(chip)) {
		chg_info("boost IC suspended, cache otg mode=%d, will set on resume\n", en);
		chip->otg_mode_pending = true;
		chip->otg_mode_pending_value = en;
		return;
	}

	chg_info("set otg mode = %d\n", en);
	rc = oplus_boost_enable_otg_mode(chip, en);
	if (rc < 0)
		chg_err("set otg mode error, rc = %d\n", rc);

	return;
}

void oplus_boost_set_fam_en(struct oplus_mms *topic, bool en)
{
	struct oplus_dischg_boost *chip;
	int rc;
	chg_info("set fam_en = %d\n", en);

	if (topic == NULL) {
		chg_err("topic is NULL\n");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip)
		return;

	/* If boost IC is suspended, cache the request and execute on resume */
	if (oplus_boost_is_ic_suspended(chip)) {
		chg_info("boost IC suspended, cache fam_en=%d, will set on resume\n", en);
		chip->fam_en_pending = true;
		chip->fam_en_pending_value = en;
		return;
	}

	chg_info("set fam_en = %d\n", en);
	rc = oplus_boost_enable_fam_en(chip, en);
	if (rc < 0)
		chg_err("set fam_en error, rc = %d\n", rc);

	return;
}

#define CV_MODE_CHECK_MS        5000
#define CV_CURRENT_SCALE_FACTOR 95  /* 95% scaling factor for CV mode */
#define PERCENTAGE_BASE         100 /* Base value for percentage calculation */

#define FCC_ACCURACY_MA           50
#define CV_MODE_STABLE_CNT        2
#define BOOST_RATIO_BASE          100
#define HYBRID_BOOST_RATIO_DEF    116
#define RG_CP_BOOST_RATIO_DEF     100
static void oplus_boost_cv_dynamic_curr_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_dischg_boost *chip = container_of(dwork,
		struct oplus_dischg_boost, boost_cv_dynamic_curr_work);
	int rc;
	bool cv_mode = false;
	int curr_default = 0;
	int vbat_mv = 0;
	int curr_cv = 0;
	struct votable *fcc_votable = NULL;
	s64 numerator, denominator, curr_cv_64;
	static int retry_count = 0;
	const int max_retry = 3;
	bool chg_online = chip->wls_online || chip->wired_online;

	/* Skip I2C operations if boost IC is suspended */
	if (oplus_boost_is_ic_suspended(chip)) {
		chg_info("boost IC suspended, skip cv dynamic curr work, reschedule later\n");
		if (chg_online)
			schedule_delayed_work(&chip->boost_cv_dynamic_curr_work, msecs_to_jiffies(CV_MODE_CHECK_MS));
		return;
	}

	fcc_votable = oplus_boost_get_curr_votable(chip);
	if (!fcc_votable) {
		chg_err("curr_votable not found!\n");
		return;
	}

	rc = oplus_boost_get_in_cv_mode(chip, &cv_mode);
	if (rc < 0) {
		chg_err("boost get cv mode error\n");
		if (retry_count++ < max_retry && chg_online)
			schedule_delayed_work(&chip->boost_cv_dynamic_curr_work, msecs_to_jiffies(CV_MODE_CHECK_MS));
		return;
	}
	retry_count = 0;

	if (cv_mode) {
		chip->cv_mode_cnt++;
		if (chip->cv_mode_cnt < CV_MODE_STABLE_CNT) {
			chg_info("cv_mode_cnt=%d < %d rerun\n", chip->cv_mode_cnt, CV_MODE_STABLE_CNT);
			goto rerun;
		}

		curr_default = get_effective_result_exclude_client(fcc_votable, BOOST_CV_VOTER);
		rc = oplus_boost_get_vbat(chip, &vbat_mv);
		if (rc < 0) {
			chg_err("can't get batt vol, rc=%d\n", rc);
			return;
		}
		if (!chip->cv_now_mv) {
			chg_info("cv_now_mv=0 invalid, return\n");
			return;
		}

		/* Formula:
		 *   curr_cv = vbat * ibat / efficiency / cv / boost_ratio
		 * where:
		 *   efficiency = CV_CURRENT_SCALE_FACTOR / PERCENTAGE_BASE
		 *   boost_ratio = chip->config.boost_ratio / BOOST_RATIO_BASE
		 *
		 * So:
		 *   curr_cv = vbat * ibat
		 *             * PERCENTAGE_BASE * BOOST_RATIO_BASE
		 *             / (CV_CURRENT_SCALE_FACTOR * cv_now_mv * boost_ratio)
		 */
		numerator = (s64)vbat_mv * curr_default *
			PERCENTAGE_BASE * BOOST_RATIO_BASE;
		denominator = (s64)CV_CURRENT_SCALE_FACTOR * chip->cv_now_mv *
			chip->config.boost_ratio;
		if (denominator <= 0) {
			chg_err("invalid denominator, cv_now_mv=%d, boost_ratio=%d\n",
				chip->cv_now_mv, chip->config.boost_ratio);
			goto rerun;
		}
		curr_cv_64 = div64_s64(numerator, denominator);
		curr_cv = (int)curr_cv_64;
		curr_cv = rounddown(curr_cv + FCC_ACCURACY_MA / 2, FCC_ACCURACY_MA);
		chg_info("cv mode current limit, curr_default= %d, vbat_mv= %d, "
			    "eff_rate = %d, boost_ratio = %d, cv_now_mv= %d, curr_cv= %d\n",
			    curr_default, vbat_mv, CV_CURRENT_SCALE_FACTOR,
			    chip->config.boost_ratio, chip->cv_now_mv, curr_cv);
		if (curr_cv < FCC_ACCURACY_MA)
			goto rerun;
		vote(fcc_votable, BOOST_CV_VOTER, true, curr_cv, true);
	} else {
		chip->cv_mode_cnt = 0;
		vote(fcc_votable, BOOST_CV_VOTER, false, 0, false);
	}

rerun:
	chg_online = chip->wls_online || chip->wired_online;
	if (chg_online)
		schedule_delayed_work(&chip->boost_cv_dynamic_curr_work, msecs_to_jiffies(CV_MODE_CHECK_MS));
}

#define AUTO_MODE_SOC_THR 40
static void oplus_boost_set_automode_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_dischg_boost *chip = container_of(dwork,
		struct oplus_dischg_boost, set_automode_work);
	struct oplus_boost_config *config = &chip->config;
	union mms_msg_data data = { 0 };
	static int last_automode_disable = -1;
	int automode_disable = 0;

	/* Work mode switching based on SOC is critical, use wakelock to prevent suspend during I2C operations */
	if (chip->i2c_wake_lock)
		__pm_wakeup_event(chip->i2c_wake_lock, 500);

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, false);
	chip->bat_soc = data.intval;
	if (chip->bat_soc >= config->auto_mode_soc_thr)
		automode_disable = 1;
	else
		automode_disable = 0;
	if (last_automode_disable != automode_disable) {
		/* Check if boost IC is suspended before executing vote */
		if (oplus_boost_is_ic_suspended(chip)) {
			/* If suspended, wait a few milliseconds for resume */
			chg_info("boost IC suspended, wait 5ms before setting work mode\n");
			msleep(5);
		}
		if (is_work_mode_votable_available(chip)) {
			vote(chip->work_mode_votable, BOOST_SOC_VOTER, automode_disable, automode_disable, false);
		} else {
			chg_err("work_mode_votable not found\n");
		}
		last_automode_disable = automode_disable;
		chg_info("set_automode, soc:%d work_mode=%d\n", chip->bat_soc, automode_disable);
	}

	/* Schedule next periodic check */
	schedule_delayed_work(&chip->set_automode_work, msecs_to_jiffies(10 * 1000));
}

static void oplus_boost_plugin_work(struct work_struct *work)
{
	struct oplus_dischg_boost *chip =
		container_of(work, struct oplus_dischg_boost, plugin_work);
	struct oplus_boost_config *config = &chip->config;

	/* Plugin work is critical, use wakelock to prevent suspend during I2C operations */
	if (chip->i2c_wake_lock)
		__pm_wakeup_event(chip->i2c_wake_lock, 500);

	if (chip->wls_online || chip->wired_online) {
		oplus_boost_set_cv_mv(chip, config->plugin_cv_mv);
		chg_err("plugin, set cv=%d\n", config->plugin_cv_mv);
#ifndef CONFIG_OPLUS_CHARGER_MTK
		oplus_boost_set_bcl_rate(chip, config->plugin_bcl_rate);
#endif
		cancel_delayed_work(&chip->boost_cv_dynamic_curr_work);
		schedule_delayed_work(&chip->boost_cv_dynamic_curr_work, 0);
	} else {
		oplus_boost_set_cv_mv(chip, config->plugout_cv_mv);
		chg_err("plugout, set cv=%d\n", config->plugout_cv_mv);
#ifndef CONFIG_OPLUS_CHARGER_MTK
		oplus_boost_set_bcl_rate(chip, config->plugout_bcl_rate);
#endif
		cancel_delayed_work(&chip->boost_cv_dynamic_curr_work);
		chip->cv_mode_cnt = 0;
		vote(chip->wired_curr_votable, BOOST_CV_VOTER, false, 0, false);
		vote(chip->wls_curr_votable, BOOST_CV_VOTER, false, 0, false);
	}
}

static int oplus_dischg_update_ic_type(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_dischg_boost *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->config.ic_type;

	return 0;
}

static int oplus_dischg_update_dev_id(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct oplus_dischg_boost *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = 1;

	return 0;
}

static void oplus_dishg_boost_parse_dt(struct oplus_dischg_boost *chip)
{
	struct oplus_boost_config *config = &chip->config;
	struct device_node *node = chip->dev->of_node;
	int rc;

	rc = of_property_read_u32(node, "oplus_boost,init-cv-mv", &config->init_cv_mv);
	if (rc)
		config->init_cv_mv = INIT_CV_MV;
	chg_info("init_cv_mv = %d\n", config->init_cv_mv);

	rc = of_property_read_u32(node, "oplus_boost,ic_type", &config->ic_type);
	if (rc)
		config->ic_type = HYBRID_BOOST;
	chg_info("ic_type = %d\n", config->ic_type);

	rc = of_property_read_u32(node, "oplus_boost,boost-ratio", &config->boost_ratio);
	if (rc) {
		if (config->ic_type == RG_CP)
			config->boost_ratio = RG_CP_BOOST_RATIO_DEF;
		else
			config->boost_ratio = HYBRID_BOOST_RATIO_DEF;
	}
	chg_info("boost_ratio = %d\n", config->boost_ratio);

	rc = of_property_read_u32(node, "oplus_boost,plugin-cv-mv", &config->plugin_cv_mv);
	if (rc)
		config->plugin_cv_mv = PLUGIN_CV_MV;
	chg_info("plugin_cv_mv = %d\n", config->plugin_cv_mv);

	rc = of_property_read_u32(node, "oplus_boost,plugout-cv-mv", &config->plugout_cv_mv);
	if (rc)
		config->plugout_cv_mv = PLUGOUT_CV_MV;
	chg_info("plugout_cv_mv = %d\n", config->plugout_cv_mv);

	rc = of_property_read_u32(node, "oplus_boost,suspend-cv-mv", &config->suspend_cv_mv);
	if (rc)
		config->suspend_cv_mv = SUSPEND_CV_MV;
	chg_info("suspend_cv_mv = %d\n", config->suspend_cv_mv);

	rc = of_property_read_u32(node, "oplus_boost,resume-cv-mv", &config->resume_cv_mv);
	if (rc)
		config->resume_cv_mv = RESUME_CV_MV;
	chg_info("resume_cv_mv = %d\n", config->resume_cv_mv);

	rc = of_property_read_u32(node, "oplus_boost,normalchg-cv-mv", &config->normalchg_cv_mv);
	if (rc)
		config->normalchg_cv_mv = NORMALCHG_CV_MV;
	chg_info("normalchg_cv_mv = %d\n", config->normalchg_cv_mv);

	rc = of_property_read_u32(node, "oplus_boost,fastchg-cv-mv", &config->fastchg_cv_mv);
	if (rc)
		config->fastchg_cv_mv = FASTCHG_CV_MV;
	chg_info("fastchg_cv_mv = %d\n", config->fastchg_cv_mv);

	rc = of_property_read_u32(node, "oplus_boost,plugin_bcl_rate", &config->plugin_bcl_rate);
	if (rc)
		config->plugin_bcl_rate = RATIO_500_THR;
	chg_info("plugin_bcl_rate = %d\n", config->plugin_bcl_rate);

	rc = of_property_read_u32(node, "oplus_boost,plugout_bcl_rate", &config->plugout_bcl_rate);
	if (rc)
		config->plugout_bcl_rate = RATIO_550_THR;
	chg_info("plugout_bcl_rate = %d\n", config->plugout_bcl_rate);

	rc = of_property_read_u32(node, "oplus_boost,auto_mode_soc_thr", &config->auto_mode_soc_thr);
	if (rc)
		config->auto_mode_soc_thr = AUTO_MODE_SOC_THR;
	chg_info("auto_mode_soc_thr = %d\n", config->auto_mode_soc_thr);
}

static void oplus_boost_gauge_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_dischg_boost *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case GAUGE_ITEM_VOL_MAX:
			oplus_mms_get_item_data(chip->gauge_topic, id, &data, false);
			chip->vbat_mv = data.intval;
			break;
		case GAUGE_ITEM_SOC:
			oplus_mms_get_item_data(chip->gauge_topic, id, &data, false);
			chip->bat_soc = data.intval;
			chg_info("bat_soc = %d\n", chip->bat_soc);
			schedule_delayed_work(&chip->set_automode_work, 0);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_boost_subscribe_gauge_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_dischg_boost *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->gauge_topic = topic;
	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
				    oplus_boost_gauge_subs_callback, "chg_boost");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data,
				true);
	chip->vbat_mv = data.intval;

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, false);
	chip->bat_soc = data.intval;
	schedule_delayed_work(&chip->set_automode_work, 0);
}

static void oplus_boost_wired_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_dischg_boost *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				false);
			chip->wired_online = data.intval;
			schedule_work(&chip->plugin_work);
			break;
		case WIRED_ITEM_CHG_TYPE:
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_boost_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_dischg_boost *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_boost_wired_subs_callback, "chg_boost");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				true);
	chip->wired_online = data.intval;

	schedule_work(&chip->plugin_work);
}

static void oplus_boost_wls_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_dischg_boost *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WLS_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wls_topic, id, &data, false);
			chip->wls_online = !!data.intval;
			schedule_work(&chip->plugin_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_boost_subscribe_wls_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_dischg_boost *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wls_topic = topic;
	chip->wls_subs =
		oplus_mms_subscribe(chip->wls_topic, chip,
				    oplus_boost_wls_subs_callback, "chg_boost");
	if (IS_ERR_OR_NULL(chip->wls_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wls_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_PRESENT, &data,
				true);
	chip->wls_online = data.intval;

	schedule_work(&chip->plugin_work);
}

static void oplus_boost_vooc_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_dischg_boost *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case VOOC_ITEM_VOOC_STARTED:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_started = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_boost_subscribe_vooc_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_dischg_boost *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->vooc_topic = topic;
	chip->vooc_subs = oplus_mms_subscribe(chip->vooc_topic, chip,
					      oplus_boost_vooc_subs_callback,
					      "chg_boost");
	if (IS_ERR_OR_NULL(chip->vooc_subs)) {
		chg_err("subscribe vooc topic error, rc=%ld\n",
			PTR_ERR(chip->vooc_subs));
		return;
	}

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_STARTED, &data,
				true);
	chip->vooc_started = data.intval;
}

static void oplus_boost_err_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	/* TODO */
}

static void oplus_boost_online_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	chg_info("%s online\n", ic_dev->manu_name);
}

static void oplus_boost_offline_handler(struct oplus_chg_ic_dev *ic_dev, void *virq_data)
{
	chg_err("%s offline\n", ic_dev->manu_name);
}

static int oplus_boost_virq_reg(struct oplus_dischg_boost *chip)
{
	int rc;

	rc = oplus_chg_ic_virq_register(chip->boost_ic, OPLUS_IC_VIRQ_ERR,
		oplus_boost_err_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ERR error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->boost_ic, OPLUS_IC_VIRQ_ONLINE,
		oplus_boost_online_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_ONLINE error, rc=%d", rc);
	rc = oplus_chg_ic_virq_register(chip->boost_ic, OPLUS_IC_VIRQ_OFFLINE,
		oplus_boost_offline_handler, chip);
	if (rc < 0)
		chg_err("register OPLUS_IC_VIRQ_OFFLINE error, rc=%d", rc);
	return 0;
}

static void oplus_dischg_boost_init_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_dischg_boost *chip = container_of(dwork,
		struct oplus_dischg_boost, dischg_boost_init_work);
	struct oplus_boost_config *config = &chip->config;
	struct device_node *node = chip->dev->of_node;
	static int retry = OPLUS_CHG_IC_INIT_RETRY_MAX;
	int rc;

	chip->boost_ic = of_get_oplus_chg_ic(node, "oplus,boost_ic", 0);
	if (chip->boost_ic == NULL) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->dischg_boost_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("oplus,boost_ic not found\n");
		}
		retry = 0;
		return;
	}

	/* Init work is critical, use wakelock to prevent suspend during I2C operations */
	if (chip->i2c_wake_lock)
		__pm_wakeup_event(chip->i2c_wake_lock, 500);

	rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_INIT);
	if (rc == -EAGAIN) {
		if (retry > 0) {
			retry--;
			schedule_delayed_work(&chip->dischg_boost_init_work,
				msecs_to_jiffies(OPLUS_CHG_IC_INIT_RETRY_DELAY));
			return;
		} else {
			chg_err("boost_ic init timeout\n");
		}
		retry = 0;
		return;
	} else if (rc < 0) {
		chg_err("boost_ic init error, rc=%d\n", rc);
		retry = 0;
		return;
	}
	retry = 0;
	oplus_boost_set_cv_mv(chip, config->init_cv_mv);

	/* Set suspend/resume CV configuration to sc83107 via IC interface */
	if (chip->boost_ic) {
		int rc = oplus_chg_ic_func(chip->boost_ic, OPLUS_IC_FUNC_BOOST_SET_SUSPEND_RESUME_CV,
			config->suspend_cv_mv, config->resume_cv_mv);
		if (rc < 0)
			chg_err("set suspend/resume cv config failed, rc=%d\n", rc);
		else
			chg_info("set suspend_cv=%d, resume_cv=%d\n", config->suspend_cv_mv, config->resume_cv_mv);
	}

	chg_info("dischg boost OPLUS_IC_FUNC_INIT success\n");
}

static void oplus_boost_ic_reg_callback(struct oplus_chg_ic_dev *ic, void *data, bool timeout)
{
	struct oplus_dischg_boost *chip;
	const char *name;
	int rc;

	if (data == NULL) {
		chg_err("ic(%s) data is NULL\n", ic->name);
		return;
	}
	chip = data;

	chip->boost_ic = ic;
	rc = oplus_boost_virq_reg(chip);
	if (rc < 0) {
		chg_err("cp virq register error, rc=%d\n", rc);
		return;
	}

	name = of_get_oplus_chg_ic_name(chip->dev->of_node, "oplus,boost_ic", 0);

	oplus_mms_wait_topic("wired", oplus_boost_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("gauge", oplus_boost_subscribe_gauge_topic, chip);
	oplus_mms_wait_topic("vooc", oplus_boost_subscribe_vooc_topic, chip);
	oplus_mms_wait_topic("wireless", oplus_boost_subscribe_wls_topic, chip);

	return;
}

static void oplus_mms_dischg_boost_update(struct oplus_mms *mms, bool publish)
{
	/*only for auto update*/
	return;
}

static struct mms_item oplus_dischg_boost_item[] = {
	{
		.desc = {
			.item_id = DISCHG_BOOST_ITEM_IC_TYPE,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_dischg_update_ic_type,
		}
	}, {
		.desc = {
			.item_id = DISCHG_BOOST_ITEM_DEV_ID,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_dischg_update_dev_id,
		}
	}
};

static const struct oplus_mms_desc oplus_mms_dischg_boost_desc = {
	.name = "dischg_boost",
	.type = OPLUS_MMS_TYPE_DISCHG_BOOST,
	.item_table = oplus_dischg_boost_item,
	.item_num = ARRAY_SIZE(oplus_dischg_boost_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_mms_dischg_boost_update,
};

static int oplus_dischg_boost_topic_init(struct oplus_dischg_boost *chip)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;

	mms_cfg.drv_data = chip;
	mms_cfg.of_node = chip->dev->of_node;

	chip->dischg_boost_topic = devm_oplus_mms_register(chip->dev, &oplus_mms_dischg_boost_desc, &mms_cfg);
	if (IS_ERR(chip->dischg_boost_topic)) {
		chg_err("Couldn't register dischg_boost_topic\n");
		rc = PTR_ERR(chip->dischg_boost_topic);
		return rc;
	}

	return 0;
}

static void oplus_boost_pending_ops_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_dischg_boost *chip = container_of(dwork,
		struct oplus_dischg_boost, pending_ops_work);
	int rc;

	/* Use wakelock to prevent suspend during I2C operations */
	if (chip->i2c_wake_lock)
		__pm_wakeup_event(chip->i2c_wake_lock, 500);

	/* Execute pending operations that were cached during suspend */
	if (chip->otg_mode_pending) {
		chg_info("executing pending otg mode=%d on resume\n", chip->otg_mode_pending_value);
		rc = oplus_boost_enable_otg_mode(chip, chip->otg_mode_pending_value);
		if (rc < 0)
			chg_err("set pending otg mode error, rc = %d\n", rc);
		chip->otg_mode_pending = false;
	}

	if (chip->fam_en_pending) {
		chg_info("executing pending fam_en=%d on resume\n", chip->fam_en_pending_value);
		rc = oplus_boost_enable_fam_en(chip, chip->fam_en_pending_value);
		if (rc < 0)
			chg_err("set pending fam_en error, rc = %d\n", rc);
		chip->fam_en_pending = false;
	}
}

static int oplus_boost_pm_resume(struct device *dev)
{
	struct oplus_dischg_boost *chip = dev_get_drvdata(dev);

	/* CV setting is now handled in sc83107_resume */
	/* Schedule pending operations to execute after 1s delay */
	if (chip->otg_mode_pending || chip->fam_en_pending)
		schedule_delayed_work(&chip->pending_ops_work, msecs_to_jiffies(1000));

	return 0;
}

static int oplus_boost_pm_suspend(struct device *dev)
{
	/* CV setting is now handled in sc83107_suspend */
	return 0;
}

static const struct dev_pm_ops oplus_boost_pm_ops = {
	.resume		= oplus_boost_pm_resume,
	.suspend	= oplus_boost_pm_suspend,
};

static int oplus_boost_probe(struct platform_device *pdev)
{
	struct oplus_dischg_boost *chip;
	const char *name;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_dischg_boost),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	oplus_dishg_boost_parse_dt(chip);

	INIT_WORK(&chip->plugin_work, oplus_boost_plugin_work);
	INIT_DELAYED_WORK(&chip->set_automode_work, oplus_boost_set_automode_work);
	INIT_DELAYED_WORK(&chip->dischg_boost_init_work, oplus_dischg_boost_init_work);
	INIT_DELAYED_WORK(&chip->boost_cv_dynamic_curr_work, oplus_boost_cv_dynamic_curr_work);
	INIT_DELAYED_WORK(&chip->pending_ops_work, oplus_boost_pending_ops_work);

	/* Initialize pending operation flags */
	chip->otg_mode_pending = false;
	chip->fam_en_pending = false;

	/* Initialize wakelock for critical I2C operations */
	chip->i2c_wake_lock = wakeup_source_register(chip->dev, "dischg_boost_i2c_wakeup");
	if (!chip->i2c_wake_lock)
		chg_err("Failed to register wakelock\n");

	schedule_delayed_work(&chip->dischg_boost_init_work, 0);

	name = of_get_oplus_chg_ic_name(pdev->dev.of_node, "oplus,boost_ic", 0);

	chip->work_mode_votable = create_votable("BOOST_WORK_MODE", VOTE_SET_ANY,
		oplus_boost_work_mode_vote_callback, chip);
	if (IS_ERR(chip->work_mode_votable)) {
		chg_err("create work_mode_votable error, rc=%ld\n", PTR_ERR(chip->work_mode_votable));
		chip->work_mode_votable = NULL;
	}
	oplus_chg_ic_wait_ic(name, oplus_boost_ic_reg_callback, chip);
	(void)oplus_dischg_boost_topic_init(chip);

	chg_info("probe success\n");
	return 0;
}

static const struct of_device_id oplus_boost_match[] = {
	{ .compatible = "oplus,dischg_boost" },
	{},
};

static struct platform_driver oplus_boost_driver = {
	.driver		= {
		.name = "oplus-dischg_boost",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_boost_match),
		.pm		= &oplus_boost_pm_ops,
	},
	.probe		= oplus_boost_probe,
	.shutdown   = NULL,
};

static __init int oplus_boost_init(void)
{
	return platform_driver_register(&oplus_boost_driver);
}

static __exit void oplus_boost_exit(void)
{
	platform_driver_unregister(&oplus_boost_driver);
}

oplus_chg_module_register(oplus_boost);
