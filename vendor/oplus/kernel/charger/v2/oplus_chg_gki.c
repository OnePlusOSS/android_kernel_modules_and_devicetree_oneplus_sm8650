// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2024 . Oplus All rights reserved.
 */

#define pr_fmt(fmt) "[GKI]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/power_supply.h>
#include <linux/sched/clock.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/oplus_project.h>
#endif
#include <oplus_chg.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_module.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_vooc.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_wls.h>
#include <oplus_chg_state_retention.h>
#include <oplus_chg_pps.h>
#include <oplus_msg_filter.h>
#include <recovery/state_keep.h>

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
struct state_keep_info {
	bool wired_online;
	int wired_type;
	int batt_status;
};
#endif

struct oplus_gki_device {
	struct device *dev;
	struct power_supply *usb_psy;
	struct power_supply *batt_psy;
	struct power_supply *wls_psy;
	struct mms_subscribe *gauge_subs;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *wls_subs;
	struct mms_subscribe *comm_subs;
	struct mms_subscribe *vooc_subs;
	struct mms_subscribe *ufcs_subs;
	struct mms_subscribe *retention_subs;
	struct mms_subscribe *pps_subs;
	struct mms_subscribe *keep_subs;
	struct oplus_mms *wired_topic;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *main_gauge_topic;
	struct oplus_mms *wls_topic;
	struct oplus_mms *comm_topic;
	struct oplus_mms *vooc_topic;
	struct oplus_mms *ufcs_topic;
	struct oplus_mms *retention_topic;
	struct oplus_mms *pps_topic;
	struct oplus_mms *keep_topic;

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	struct state_keep_info keep;
#endif
	struct oplus_msg_filter filter;
	struct work_struct gauge_update_work;
	struct work_struct wired_online_update_work;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	struct work_struct keep_wired_online_update_work;
#endif
	struct votable *chg_disable_votable;
	struct votable *wired_icl_votable;
	struct votable *wired_fcc_votable;
	struct votable *fv_votable;
	struct votable *vooc_curr_votable;
	struct votable *ufcs_curr_votable;
	struct votable *pps_curr_votable;
	struct votable *wired_suspend_votable;

	struct delayed_work status_keep_clean_work;
	struct delayed_work status_keep_delay_unlock_work;
	struct delayed_work retention_checkout_work;
	struct delayed_work usb_phy_suspend_recovery_work;
	struct wakeup_source *status_wake_lock;
	bool status_wake_lock_on;
	bool is_ui_keep;

	bool led_on;

	bool batt_exist;
	bool batt_auth;
	bool batt_hmac;
	bool ui_soc_ready;
	int vbat_mv;
	int vbat_min_mv;
	int temperature;
	int soc;
	int batt_fcc;
	int batt_rm;
	int pre_batt_status;
	int batt_status;
	int batt_status_keep;
	int batt_health;
	int batt_chg_type;
	int ui_soc;
	int batt_capacity_mah;
	int time_to_full;

	bool wired_online;
	int pre_wired_type;
	int usb_psy_desc_type;
	int cc_detect;
	int wired_type;
	int last_wired_type;
	int vbus_mv;
	int charger_cycle;
	bool vooc_charging;
	bool vooc_started;
	bool vooc_by_normal_path;
	bool vooc_online;

	bool wls_online;

	bool smart_charging_screenoff;
	bool retention_state;
	bool wired_present;
	bool retention_wired_plugout;
	bool pre_retention_state;
	bool retention_connect_state;
	enum oplus_temp_region temp_region;

	bool ufcs_online;
	bool pre_ufcs_online;
	bool pps_online;
	bool pre_pps_online;
};

#define RETRY_RETENTION_STATE	100

static struct oplus_gki_device *g_gki_dev;

__maybe_unused static bool
is_chg_disable_votable_available(struct oplus_gki_device *chip)
{
	if (!chip->chg_disable_votable)
		chip->chg_disable_votable = find_votable("CHG_DISABLE");

	return !!chip->chg_disable_votable;
}

__maybe_unused static bool
is_wired_icl_votable_available(struct oplus_gki_device *chip)
{
	if (!chip->wired_icl_votable)
		chip->wired_icl_votable = find_votable("WIRED_ICL");
	return !!chip->wired_icl_votable;
}

__maybe_unused static bool
is_wired_fcc_votable_available(struct oplus_gki_device *chip)
{
	if (!chip->wired_fcc_votable)
		chip->wired_fcc_votable = find_votable("WIRED_FCC");
	return !!chip->wired_fcc_votable;
}

__maybe_unused static bool
is_fv_votable_available(struct oplus_gki_device *chip)
{
	if (!chip->fv_votable)
		chip->fv_votable = find_votable("FV_MAX");
	return !!chip->fv_votable;
}

__maybe_unused static bool
is_vooc_curr_votable_available(struct oplus_gki_device *chip)
{
	if (!chip->vooc_curr_votable)
		chip->vooc_curr_votable = find_votable("VOOC_CURR");
	return !!chip->vooc_curr_votable;
}

__maybe_unused static bool
is_ufcs_curr_votable_available(struct oplus_gki_device *chip)
{
	if (!chip->ufcs_curr_votable)
		chip->ufcs_curr_votable = find_votable("UFCS_CURR");
	return !!chip->ufcs_curr_votable;
}

__maybe_unused static bool
is_pps_curr_votable_available(struct oplus_gki_device *chip)
{
	if (!chip->pps_curr_votable)
		chip->pps_curr_votable = find_votable("PPS_CURR");
	return !!chip->pps_curr_votable;
}

__maybe_unused static bool
is_wired_suspend_votable_available(struct oplus_gki_device *chip)
{
	if (!chip->wired_suspend_votable)
		chip->wired_suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	return !!chip->wired_suspend_votable;
}

static bool is_main_gauge_topic_available(struct oplus_gki_device *chip)
{
	if (!chip->main_gauge_topic)
		chip->main_gauge_topic = oplus_mms_get_by_name("gauge:0");

	return !!chip->main_gauge_topic;
}

#define KEEP_CLEAN_INTERVAL	2000
static int wls_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct oplus_gki_device *chip = power_supply_get_drvdata(psy);
	union mms_msg_data data = { 0 };
	int rc = 0;
	int bms_temp_compensation;
	static bool pre_wls_online;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		pval->intval = chip->wls_online;
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_STATUS_KEEP, &data, true);
		if (data.intval != WLS_SK_NULL) {
			pval->intval = 1;
			if (data.intval == WLS_SK_BY_FORCE_BPP) {
				if (!chip->status_wake_lock_on) {
					chg_info("force bpp acquire status_wake_lock\n");
					__pm_stay_awake(chip->status_wake_lock);
					chip->status_wake_lock_on = true;
				}
				schedule_delayed_work(&chip->status_keep_clean_work,
						      msecs_to_jiffies(KEEP_CLEAN_INTERVAL));
			}
			if (chip->wls_online)
				pre_wls_online = 1;
		} else {
			if (pre_wls_online && pval->intval == 0) {
				if (delayed_work_pending(&chip->status_keep_clean_work))
					cancel_delayed_work_sync(&chip->status_keep_clean_work);
				if (chip->status_wake_lock_on) {
					chg_info("release status_wake_lock\n");
					__pm_relax(chip->status_wake_lock);
					chip->status_wake_lock_on = false;
				}
				if (!chip->status_wake_lock_on) {
					chg_info("acquire status_wake_lock\n");
					__pm_stay_awake(chip->status_wake_lock);
					chip->status_wake_lock_on = true;
				}
				pre_wls_online = pval->intval;
				oplus_chg_wls_set_status_keep(chip->wls_topic, WLS_SK_BY_KERNEL);
				pval->intval = 1;
				schedule_delayed_work(&chip->status_keep_clean_work, msecs_to_jiffies(KEEP_CLEAN_INTERVAL));
				chip->is_ui_keep = true;
			} else {
				pre_wls_online = pval->intval;
				if (chip->is_ui_keep && pval->intval == 0) {
					pval->intval = 1;
					break;
				}
				if (chip->status_wake_lock_on) {
					cancel_delayed_work_sync(&chip->status_keep_clean_work);
					schedule_delayed_work(&chip->status_keep_clean_work, 0);
				}
			}
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_VOUT, &data, true);
		pval->intval = data.intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_VOUT, &data, true);
		pval->intval = data.intval * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_IOUT, &data, true);
		pval->intval = data.intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_IOUT, &data, true);
		pval->intval = data.intval * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (is_support_parallel_battery(chip->gauge_topic) &&
		    is_main_gauge_topic_available(chip)) {
			rc = oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_TEMP, &data, false);
			pval->intval = data.intval;
		} else {
			bms_temp_compensation = oplus_comm_get_bms_heat_temp_compensation(chip->comm_topic);
			if (bms_temp_compensation != 0 && chip->temperature < OPLUS_BMS_HEAT_THRE)
				pval->intval = chip->temperature - bms_temp_compensation;
			else
				pval->intval = chip->temperature;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = chip->wls_online;
		break;
	default:
		chg_err("get prop %d is not supported\n", prop);
		return -EINVAL;
	}
	if (rc < 0) {
		chg_err("Couldn't get prop %d rc = %d\n", prop, rc);
		return -ENODATA;
	}

	return 0;
}

static int wls_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	return 0;
}

static int wls_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	return 0;
}

static enum power_supply_property wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PRESENT,
};

static const struct power_supply_desc wls_psy_desc = {
	.name			= "wireless",
	.type			= POWER_SUPPLY_TYPE_WIRELESS,
	.properties		= wls_props,
	.num_properties		= ARRAY_SIZE(wls_props),
	.get_property		= wls_psy_get_prop,
	.set_property		= wls_psy_set_prop,
	.property_is_writeable	= wls_psy_prop_is_writeable,
};


static int oplus_to_psy_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_PD_PPS,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_TYPE_USB_DCP,
};

static int oplus_to_power_supply_type[] = {
	POWER_SUPPLY_TYPE_UNKNOWN,
	POWER_SUPPLY_TYPE_USB,
	POWER_SUPPLY_TYPE_USB_DCP,
	POWER_SUPPLY_TYPE_USB_CDP,
	POWER_SUPPLY_TYPE_USB_ACA,
	POWER_SUPPLY_TYPE_USB_TYPE_C,
	POWER_SUPPLY_TYPE_USB_PD,
	POWER_SUPPLY_TYPE_USB_PD_DRP,
	POWER_SUPPLY_TYPE_USB_PD,
	POWER_SUPPLY_TYPE_USB,
	POWER_SUPPLY_TYPE_USB_DCP,
	POWER_SUPPLY_TYPE_USB_DCP,
	POWER_SUPPLY_TYPE_USB_DCP,
	POWER_SUPPLY_TYPE_USB_DCP,
	POWER_SUPPLY_TYPE_USB_DCP,
	POWER_SUPPLY_TYPE_USB_DCP,
};

static int usb_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct oplus_gki_device *chip = power_supply_get_drvdata(psy);
	union mms_msg_data data = { 0 };
	int usb_temp_l, usb_temp_r;
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		pval->intval = chip->wired_online;
		if (!pval->intval) {
			oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data, false);
			pval->intval = data.intval;
		}
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
		if (chip->keep_topic && chip->keep.wired_online)
			pval->intval = chip->keep.wired_online;
#endif
		if (chip->retention_state)
			pval->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		pval->intval = oplus_wired_get_vbus() * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pval->intval =
			oplus_wired_get_voltage_max(chip->wired_topic) * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		pval->intval =  oplus_wired_get_ibus() * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (is_wired_icl_votable_available(chip))
			pval->intval = get_client_vote_locked(chip->wired_icl_votable,
						       SPEC_VOTER) *
				       1000;
		else
			rc = -ENOTSUPP;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (is_wired_icl_votable_available(chip))
			pval->intval =
				get_effective_result_locked(chip->wired_icl_votable) *
				1000;
		else
			rc = -ENOTSUPP;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		if (chip->wired_type == OPLUS_CHG_USB_TYPE_UNKNOWN) {
			oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE, &data, false);
			chip->wired_type = data.intval;
		}
		pval->intval = oplus_to_psy_usb_type[chip->wired_type];
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
		if (chip->keep_topic)
			pval->intval = oplus_to_psy_usb_type[chip->keep.wired_type];
#endif
		if (chip->retention_state)
			pval->intval = oplus_to_power_supply_type[chip->last_wired_type];
		break;
	case POWER_SUPPLY_PROP_TEMP:
		oplus_mms_get_item_data(chip->wired_topic,
					WIRED_ITEM_USB_TEMP_L, &data, true);
		usb_temp_l = data.intval;
		oplus_mms_get_item_data(chip->wired_topic,
					WIRED_ITEM_USB_TEMP_R, &data, true);
		usb_temp_r = data.intval;
		pval->intval = max(usb_temp_l, usb_temp_r);
		break;
	default:
		chg_err("get prop %d is not supported\n", prop);
		return -EINVAL;
	}
	if (rc < 0) {
		chg_err("Couldn't get prop %d rc = %d\n", prop, rc);
		return -ENODATA;
	}

	return 0;
}

#define CLEAN_SUSPEND_VOTE_DELAY_MS 2000
static void oplus_usb_phy_suspend_recovery_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_gki_device *chip = container_of(dwork, struct oplus_gki_device,
		usb_phy_suspend_recovery_work);

	if (is_wired_suspend_votable_available(chip))
		vote(chip->wired_suspend_votable, USB_PSY_VOTER, false, 0, false);
}

#define USB_PHY_SUSPEND_CURR 100
static void usb_psy_set_icl(struct oplus_gki_device *chip, int curr_ua)
{
	if (!chip) {
		chg_err("chip null\n");
		return;
	}

	if (!is_wired_suspend_votable_available(chip)) {
		chg_err("wired_suspend_votable not available\n");
		return;
	}

	if((chip->wired_type == OPLUS_CHG_USB_TYPE_SDP ||
		chip->wired_type == OPLUS_CHG_USB_TYPE_PD_SDP) &&
		((curr_ua / 1000) < USB_PHY_SUSPEND_CURR)) {
		cancel_delayed_work_sync(&chip->usb_phy_suspend_recovery_work);
		vote(chip->wired_suspend_votable, USB_PSY_VOTER, true, 1, false);
		schedule_delayed_work(&chip->usb_phy_suspend_recovery_work,
			msecs_to_jiffies(CLEAN_SUSPEND_VOTE_DELAY_MS));
	} else {
		vote(chip->wired_suspend_votable, USB_PSY_VOTER, false, 0, false);
	}
}

static int usb_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	int rc = 0;
	struct oplus_gki_device *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (oplus_chg_get_common_charge_icl_support_flags())
			usb_psy_set_icl(chip, pval->intval);
		break;
	default:
		chg_err("set prop %d is not supported\n", prop);
		return -EINVAL;
	}

	return rc;
}

static int usb_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_TEMP,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
static enum power_supply_usb_type usb_psy_supported_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_PD_PPS,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,
};
#endif

static struct power_supply_desc usb_psy_desc = {
	.name			= "usb",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= usb_props,
	.num_properties		= ARRAY_SIZE(usb_props),
	.get_property		= usb_psy_get_prop,
	.set_property		= usb_psy_set_prop,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	.usb_types		= usb_psy_supported_types,
	.num_usb_types		= ARRAY_SIZE(usb_psy_supported_types),
#else
	.usb_types		= BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
				  BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
				  BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
				  BIT(POWER_SUPPLY_USB_TYPE_C)       |
				  BIT(POWER_SUPPLY_USB_TYPE_PD)      |
				  BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
				  BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
				  BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
#endif
	.property_is_writeable	= usb_psy_prop_is_writeable,
};

static int oplus_gki_get_ui_soc(struct oplus_gki_device *chip)
{
	union mms_msg_data data = { 0 };
	int ui_soc = 50; /* default soc to use on error */
	int rc = 0;

	if (oplus_is_rf_ftm_mode()) {
		chg_info("rf_ftm mode, ui_soc is default = %d\n", ui_soc);
		return ui_soc;
	}

	if (chip->ui_soc_ready)
		return chip->ui_soc;

	chg_err("ui_soc not ready, use chip soc\n");
	if (chip->gauge_topic == NULL) {
		chg_err("gauge_topic is NULL, use default soc\n");
		return ui_soc;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data,
				     false);
	if (rc < 0) {
		chg_err("can't get battery soc, rc=%d\n", rc);
		return ui_soc;
	}
	ui_soc = data.intval;

	return ui_soc;
}

#define OPLUS_NS_TO_MS (1000 * 1000)
int oplus_chg_get_curr_time_ms(unsigned long *time_ms)
{
	u64 ts_nsec;

	ts_nsec = local_clock();
	*time_ms = ts_nsec / OPLUS_NS_TO_MS;

	return *time_ms;
}

static int oplus_gki_get_batt_status(struct oplus_gki_device *chip)
{
	int oplus_batt_status;
	static bool pre_is_wired = true;/*default:wired*/

	oplus_batt_status = chip->batt_status;
	if (chip->wls_online) {
		pre_is_wired = false;
		return oplus_batt_status;
	}
	if (chip->wired_present)
		pre_is_wired = true;
	if (is_chg_disable_votable_available(chip) &&
	    (get_client_vote(chip->chg_disable_votable, MMI_CHG_VOTER) > 0))
		return oplus_batt_status; /* mmi disable charge */
	if (chip->temp_region <= TEMP_REGION_COLD)
		return oplus_batt_status; /* temp cold disable charge */
	if (chip->batt_status != POWER_SUPPLY_STATUS_FULL &&
		chip->temp_region >= TEMP_REGION_HOT)
		return oplus_batt_status; /* temp hot disable charge */

	if (pre_is_wired && chip->retention_topic && chip->batt_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		if (!chip->retention_wired_plugout && !chip->wired_present)
			oplus_batt_status = chip->batt_status_keep;
		if (chip->retention_connect_state)
			oplus_batt_status = chip->batt_status_keep;
		chg_debug("retention_wired_plugout=%d, retention_connect_state=%d\n",
			chip->retention_wired_plugout, chip->retention_connect_state);
	}

	if (chip->batt_status == POWER_SUPPLY_STATUS_FULL &&
	    chip->temp_region == TEMP_REGION_WARM &&
	    oplus_gki_get_ui_soc(chip) != 100)
		oplus_batt_status = POWER_SUPPLY_STATUS_CHARGING;
	if (chip->batt_status == POWER_SUPPLY_STATUS_FULL &&
	    chip->temp_region == TEMP_REGION_HOT &&
	    oplus_gki_get_ui_soc(chip) != 100)
		oplus_batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	chg_debug("batt_status=%d, batt_status_keep=%d\n",
		  oplus_batt_status, chip->batt_status_keep);

	return oplus_batt_status;
}

#define KPOC_FORCE_VBUS_MV 5000
#define FORCE_VBUS_5V_TIME 10000
static int battery_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct oplus_gki_device *chip = power_supply_get_drvdata(psy);
	union mms_msg_data data = { 0 };
	struct power_supply *wlspsy = NULL;
	union power_supply_propval wlsval = { 0 };
	int rc = 0;
	int bms_temp_compensation;
	int batt_qmax_0 = 0;
	int batt_qmax_1 = 0;

	static int pre_batt_status = 0;
	unsigned long cur_chg_time = 0;

	if (chip->gauge_topic == NULL)
		return -ENODEV;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if  (!IS_ERR_OR_NULL(chip->wls_topic))
			(void)oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_STATUS_KEEP, &data, true);
		if (data.intval) {
			pval->intval = pre_batt_status;
		} else {
			pval->intval = oplus_gki_get_batt_status(chip);
			if (chip->wls_online) {
				pre_batt_status = pval->intval;
			} else if (pre_batt_status) {
				wlspsy = power_supply_get_by_name("wireless");
				if (wlspsy)
					power_supply_get_property(wlspsy, POWER_SUPPLY_PROP_ONLINE, &wlsval);
				if (wlsval.intval)
					pval->intval = pre_batt_status;
				else
					pre_batt_status = 0;
			}
			if (is_chg_disable_votable_available(chip) && chip->wired_online &&
				pval->intval == POWER_SUPPLY_STATUS_NOT_CHARGING) {
				if ((get_client_vote(chip->chg_disable_votable, EIS_VOTER) > 0 &&
				    get_effective_result_exclude_client(
						chip->chg_disable_votable, EIS_VOTER) == 0) ||
				    (get_client_vote(chip->chg_disable_votable, FLASH_MODE_VOTER) > 0 &&
				    get_effective_result_exclude_client(
						chip->chg_disable_votable, FLASH_MODE_VOTER) == 0)) {
					pval->intval = POWER_SUPPLY_STATUS_CHARGING;
					chip->batt_status = POWER_SUPPLY_STATUS_CHARGING;
					chg_info("EIS_VOTER or FLASH_MODE_VOTER: batt_status is %d\n", chip->batt_status);
				}
			}
		}
		chip->batt_status_keep = pval->intval;
		chg_debug("batt_status is %d, pval->intval =%d\n", chip->batt_status, pval->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		pval->intval = chip->batt_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = chip->batt_exist;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		pval->intval = chip->batt_chg_type;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		pval->intval = oplus_gki_get_ui_soc(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		if (is_fv_votable_available(chip))
			pval->intval =
				(get_client_vote(chip->fv_votable, SPEC_VOTER) +
				 50) *
				1000;
		else
			rc = -ENOTSUPP;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (is_support_parallel_battery(chip->gauge_topic) &&
		    is_main_gauge_topic_available(chip))
			rc = oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_VOL_MAX,
						     &data, false);
		else
			rc = oplus_mms_get_item_data(chip->gauge_topic,
						     GAUGE_ITEM_VOL_MAX, &data, false);
#ifdef CONFIG_OPLUS_CHARGER_MTK
		pval->intval = data.intval;
#else
		pval->intval = data.intval * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (is_support_parallel_battery(chip->gauge_topic) &&
		    is_main_gauge_topic_available(chip))
			rc = oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_VOL_MAX,
						     &data, false);
		else
			rc = oplus_mms_get_item_data(chip->gauge_topic,
						     GAUGE_ITEM_VOL_MAX, &data, false);
#ifdef CONFIG_OPLUS_CHARGER_MTK
		pval->intval = data.intval;
#else
		pval->intval = data.intval * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (is_support_parallel_battery(chip->gauge_topic) &&
		    is_main_gauge_topic_available(chip))
			rc = oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_CURR,
						     &data, (chip->wired_online || chip->wls_online));
		else
			rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR,
						     &data, (chip->wired_online || chip->wls_online));
		pval->intval = data.intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		pval->intval = 6500000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		/* not support CHARGE_CONTROL_LIMIT, just 1 level */
		pval->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (is_support_parallel_battery(chip->gauge_topic) &&
		    is_main_gauge_topic_available(chip)) {
			rc = oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_TEMP,
						     &data, false);
			pval->intval = data.intval;
		} else {
			bms_temp_compensation = oplus_comm_get_bms_heat_temp_compensation(chip->comm_topic);
			if (bms_temp_compensation != 0 && chip->temperature < OPLUS_BMS_HEAT_THRE)
				pval->intval = chip->temperature - bms_temp_compensation;
			else
				pval->intval = chip->temperature;
		}
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		pval->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		pval->intval = chip->batt_rm * 1000;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (chip->charger_cycle < 0)
			chip->charger_cycle = 0;
		pval->intval = chip->charger_cycle;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		pval->intval = chip->batt_capacity_mah * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		pval->intval = chip->batt_capacity_mah * 1000;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		oplus_gauge_get_qmax(chip->gauge_topic, 0, &batt_qmax_0);
		oplus_gauge_get_qmax(chip->gauge_topic, 1, &batt_qmax_1);
		pval->intval = batt_qmax_0 < batt_qmax_1 ? batt_qmax_0 : batt_qmax_1;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		if (!chip->batt_auth || !chip->batt_hmac)
			pval->strval = "uncertified battery";
		else
			pval->strval = "oplus battery";
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		if (chip->time_to_full > 0 && oplus_gki_get_ui_soc(chip) == 100)
			pval->intval = 0;
		else
			pval->intval = chip->time_to_full;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		pval->intval = 3600;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
	case POWER_SUPPLY_PROP_POWER_AVG:
		pval->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (chip->wired_online && chip->vooc_started == true &&
				chip->wired_type == OPLUS_CHG_USB_TYPE_SVOOC)
			pval->intval = 10000;
		else if (chip->wls_online)
			pval->intval = 0; /* TODO */
		else
			pval->intval = oplus_wired_get_vbus();
		if (oplus_is_power_off_charging() &&
		    oplus_chg_get_curr_time_ms(&cur_chg_time) < FORCE_VBUS_5V_TIME)
			pval->intval = KPOC_FORCE_VBUS_MV;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		rc = oplus_mms_get_item_data(chip->gauge_topic,
					     GAUGE_ITEM_VOL_MIN, &data, false);
#ifdef CONFIG_OPLUS_CHARGER_MTK
		pval->intval = data.intval;
#else
		pval->intval = data.intval * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		pval->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		if (oplus_gki_get_ui_soc(chip) == 0) {
			pval->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
			chg_err("POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL, should shutdown!!!\n");
			}
		break;
	default:
		chg_err("get prop %d is not supported\n", prop);
		return -EINVAL;
	}
	if (rc < 0) {
		chg_err("Couldn't get prop %d rc = %d\n", prop, rc);
		return -ENODATA;
	}

	return rc;
}

#define TTF_UPDATE_UEVENT_BIT		BIT(30)
#define TTF_VALUE_MASK			GENMASK(29, 0)
#define FASTCHG_ICL_MIN			1500

static int battery_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct oplus_gki_device *chip = power_supply_get_drvdata(psy);
	int val;
	int rc = 0;

	if (chip->gauge_topic == NULL)
		return -ENODEV;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!chip->smart_charging_screenoff)
			return -ENOTSUPP;
		val = pval->intval & 0XFFFF;
		if (chip->wired_online &&
		    is_wired_icl_votable_available(chip)) {
			if (chip->led_on) {
				vote(chip->wired_icl_votable, HIDL_VOTER, false, 0, true);
				if (is_vooc_curr_votable_available(chip))
					vote(chip->vooc_curr_votable, HIDL_VOTER, false, 0, false);
				if (is_ufcs_curr_votable_available(chip))
					vote(chip->ufcs_curr_votable, HIDL_VOTER, false, 0, false);
				if (is_pps_curr_votable_available(chip))
					vote(chip->pps_curr_votable, HIDL_VOTER, false, 0, false);
			} else {
				if ((chip->vooc_online && !chip->vooc_by_normal_path) ||
				    chip->ufcs_online || chip->pps_online) {
					vote(chip->wired_icl_votable, HIDL_VOTER, (val == 0) ? false : true,
					    (val < FASTCHG_ICL_MIN) ? FASTCHG_ICL_MIN : val, true);
				} else {
					vote(chip->wired_icl_votable, HIDL_VOTER, (val == 0) ? false : true,
					    val, true);
				}
				if (is_vooc_curr_votable_available(chip))
					vote(chip->vooc_curr_votable, HIDL_VOTER, (val == 0) ? false : true, val, false);
				if (is_ufcs_curr_votable_available(chip))
					vote(chip->ufcs_curr_votable, HIDL_VOTER, (val == 0) ? false : true, val, false);
				if (is_pps_curr_votable_available(chip))
					vote(chip->pps_curr_votable, HIDL_VOTER, (val == 0) ? false : true, val, false);
			}
		} else {
			rc = -ENOTSUPP;
		}
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		chip->time_to_full = (pval->intval & TTF_VALUE_MASK) > 0 ? (pval->intval & TTF_VALUE_MASK) : 0;
		if ((pval->intval & TTF_UPDATE_UEVENT_BIT) && (!IS_ERR_OR_NULL(chip->batt_psy)))
			oplus_power_supply_changed_gp(chip->batt_psy, 0);
		break;
	default:
		chg_err("set prop %d is not supported\n", prop);
		return -EINVAL;
	}

	return rc;
}

static int battery_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return 1;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return 1;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
#endif
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static const struct power_supply_desc batt_psy_desc = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= battery_props,
	.num_properties		= ARRAY_SIZE(battery_props),
	.get_property		= battery_psy_get_prop,
	.set_property		= battery_psy_set_prop,
	.property_is_writeable	= battery_psy_prop_is_writeable,
};

#define FORCE_UPDATE_TIME	60
#define HIGH_TEMP_UPDATE_THD	500
#define HIGH_TEMP_UPDATE_TIME	10
static void oplus_gki_gauge_update_work(struct work_struct *work)
{
	struct oplus_gki_device *chip =
		container_of(work, struct oplus_gki_device, gauge_update_work);
	union mms_msg_data data = { 0 };
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int eng_version;
#endif
	static unsigned long update_time = 0;

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data,
				false);
	chip->vbat_mv = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data,
				false);
	chip->vbat_min_mv = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data,
				false);
	chip->soc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data,
				false);
	chip->batt_fcc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RM, &data,
				false);
	chip->batt_rm = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data,
				false);
	chip->temperature = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data,
				false);
	chip->charger_cycle = data.intval;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	eng_version = get_eng_version();
	if (eng_version == AGING || eng_version == HIGH_TEMP_AGING ||
	    eng_version == FACTORY) {
		if (!IS_ERR_OR_NULL(chip->batt_psy))
			oplus_power_supply_changed(chip->batt_psy);
	} else {
#endif
		if ((chip->temperature >= HIGH_TEMP_UPDATE_THD
		    && time_is_before_eq_jiffies(update_time + (unsigned long)(HIGH_TEMP_UPDATE_TIME * HZ))) ||
		    (chip->temperature < HIGH_TEMP_UPDATE_THD
		    && time_is_before_eq_jiffies(update_time + (unsigned long)(FORCE_UPDATE_TIME * HZ)))) {
			if (!IS_ERR_OR_NULL(chip->batt_psy))
				oplus_power_supply_changed(chip->batt_psy);
			update_time = jiffies;
		}
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	}
#endif
}

static void oplus_gki_gauge_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_gki_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_TIMER:
		schedule_work(&chip->gauge_update_work);
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case GAUGE_ITEM_BATT_EXIST:
			rc = oplus_mms_get_item_data(chip->gauge_topic,
						     GAUGE_ITEM_BATT_EXIST,
						     &data, false);
			if (rc < 0)
				chip->batt_exist = false;
			else
				chip->batt_exist = data.intval;
			if (!IS_ERR_OR_NULL(chip->batt_psy))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			break;
		case GAUGE_ITEM_AUTH:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
						     &data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_AUTH data, rc=%d\n",
					rc);
				chip->batt_auth = false;
			} else {
				chip->batt_auth = !!data.intval;
			}
			break;
		case GAUGE_ITEM_HMAC:
			rc = oplus_mms_get_item_data(chip->gauge_topic, id,
						     &data, false);
			if (rc < 0) {
				chg_err("can't get GAUGE_ITEM_HMAC data, rc=%d\n",
					rc);
				chip->batt_hmac = false;
			} else {
				chip->batt_hmac = !!data.intval;
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_gki_subscribe_gauge_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_gki_device *chip = prv_data;
	struct power_supply_config psy_cfg = {};
	union mms_msg_data data = { 0 };
	struct power_supply *psy = NULL;
	int rc;

	chip->gauge_topic = topic;
	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
				    oplus_gki_gauge_subs_callback, "gki");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data,
				true);
	chip->vbat_mv = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data,
				true);
	chip->vbat_min_mv = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data,
				true);
	chip->soc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data,
				true);
	chip->batt_fcc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RM, &data,
				true);
	chip->batt_rm = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data,
				true);
	chip->temperature = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data,
				false);
	chip->charger_cycle = data.intval;
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_BATT_EXIST,
				     &data, true);
	if (rc < 0)
		chip->batt_exist = false;
	else
		chip->batt_exist = data.intval;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_AUTH, &data,
				     true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_AUTH data, rc=%d\n", rc);
		chip->batt_auth = false;
	} else {
		chip->batt_auth = !!data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_HMAC, &data,
				     true);
	if (rc < 0) {
		chg_err("can't get GAUGE_ITEM_HMAC data, rc=%d\n", rc);
		chip->batt_hmac = false;
	} else {
		chip->batt_hmac = !!data.intval;
	}

	chip->batt_capacity_mah = oplus_gauge_get_batt_capacity_mah(chip->gauge_topic);

	psy = power_supply_get_by_name("battery");
	if (psy != NULL) {
		chg_info("battery power_sypply already registered\n");
		return;
	}

	psy_cfg.drv_data = chip;
	chip->batt_psy = devm_power_supply_register(&topic->dev, &batt_psy_desc, &psy_cfg);
	if (IS_ERR_OR_NULL(chip->batt_psy)) {
		chg_err("Failed to register battery power supply, rc=%ld\n",
			PTR_ERR(chip->batt_psy));
		goto psy_err;
	}

	return;

psy_err:
	chip->batt_psy = NULL;
	oplus_mms_unsubscribe(chip->gauge_subs);
}

static bool oplus_gki_bc12_is_completed(struct oplus_gki_device *chip)
{
	union mms_msg_data data = { 0 };

	if (chip->wired_topic == NULL)
		return false;

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_BC12_COMPLETED,
				&data, true);
	return !!data.intval;
}

static void oplus_gki_wired_online_update_work(struct work_struct *work)
{
	struct oplus_gki_device *chip =
		container_of(work, struct oplus_gki_device, wired_online_update_work);
	union mms_msg_data data = { 0 };
	bool wired_online;
	bool changed = false;

	chg_debug("enter\n");
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data, false);
	wired_online = data.intval;
	if (wired_online)
		chip->retention_wired_plugout = false;

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE, &data, false);
	chip->wired_type = data.intval;
	if (!!chip->retention_topic) {
		chip->cc_detect = oplus_wired_get_hw_detect_recheck();
		if ((!wired_online || chip->wired_type ==
			POWER_SUPPLY_TYPE_UNKNOWN) && chip->cc_detect != CC_DETECT_NOTPLUG) {
			oplus_mms_get_item_data(chip->retention_topic, RETENTION_ITEM_CONNECT_STATUS,
						&data, false);
			chip->retention_state = data.intval;
			if (!chip->retention_state) {
				msleep(RETRY_RETENTION_STATE);
				oplus_mms_get_item_data(chip->retention_topic, RETENTION_ITEM_CONNECT_STATUS,
							&data, false);
				chip->retention_state = data.intval;
				chg_debug("retry_retention_state =%d\n", chip->retention_state);
			}
			chg_debug("gki_update_work_retention_state=%d\n", chip->retention_state);
		} else if (chip->cc_detect == CC_DETECT_NOTPLUG) {
			chip->retention_state = 0;
		} else if (chip->wired_type == OPLUS_CHG_USB_TYPE_PD_SDP || chip->wired_type ==
			  OPLUS_CHG_USB_TYPE_SDP || chip->wired_type == OPLUS_CHG_USB_TYPE_CDP) {
			chip->retention_state = 0;
		}
	}

	if (chip->wired_online != wired_online)
		changed = true;
	chip->wired_online = wired_online;
	chg_debug("wired_online=%d, chip->wired_type =%d, retention_state_real=%d, cc_detect =%d\n",
		chip->wired_online, chip->wired_type, chip->retention_state, chip->cc_detect);

	if (!chip->wired_online) {
		if (chip->retention_state) {
			if (chip->last_wired_type == POWER_SUPPLY_TYPE_UNKNOWN)
				usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			else
				usb_psy_desc.type = oplus_to_power_supply_type[chip->last_wired_type];
		} else {
			chip->pre_retention_state = 0;
			chip->retention_connect_state = 0;
			chip->last_wired_type = POWER_SUPPLY_TYPE_UNKNOWN;
			usb_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		}
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
		if (chip->keep_topic != NULL)
			usb_psy_desc.type = oplus_to_power_supply_type[chip->keep.wired_type];
#endif
		if (oplus_chg_get_common_charge_icl_support_flags() &&
			is_wired_suspend_votable_available(chip))
			vote(chip->wired_suspend_votable, USB_PSY_VOTER, false, 0, false);
	} else {
		if (!chip->retention_state && chip->wired_type)
			chip->last_wired_type = chip->wired_type;
		if (chip->retention_state) {
			if (chip->last_wired_type == POWER_SUPPLY_TYPE_UNKNOWN)
				usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			else
				usb_psy_desc.type = oplus_to_power_supply_type[chip->last_wired_type];
		} else {
			usb_psy_desc.type = oplus_to_power_supply_type[chip->wired_type];
		}
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
		if (chip->keep_topic != NULL)
			usb_psy_desc.type = oplus_to_power_supply_type[chip->keep.wired_type];
#endif
		if ((usb_psy_desc.type == POWER_SUPPLY_TYPE_UNKNOWN) &&
			oplus_gki_bc12_is_completed(chip))
			usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
	}
	chg_info("psy_type=%d, usb_psy_desc_type=%d, wired_type=%d, pre_wired_type=%d,"
		"retention_state=%d, wired_online=%d\n",
		usb_psy_desc.type, chip->usb_psy_desc_type, chip->wired_type, chip->pre_wired_type,
		chip->retention_state, chip->wired_online);
	if (chip->usb_psy_desc_type != usb_psy_desc.type ||
	    chip->pre_wired_type != chip->wired_type) {
		changed = true;
		chip->usb_psy_desc_type = usb_psy_desc.type;
		chip->pre_wired_type = chip->wired_type;
	}

	if (!IS_ERR_OR_NULL(chip->batt_psy) && changed) {
		chg_debug("charger info changed\n");
		oplus_power_supply_changed_gp(chip->batt_psy, 0);
	}
}

static void oplus_gki_wired_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_gki_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			if (chip->keep_topic != NULL)
				break;
#endif
			chg_debug("gki_chg_online enter\n");
			schedule_work(&chip->wired_online_update_work);
			break;
		case WIRED_ITEM_CHG_TYPE:
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			if (chip->keep_topic != NULL)
				break;
#endif
			chg_debug("gki_chg_type enter\n");
			schedule_work(&chip->wired_online_update_work);
			break;
		case WIRED_ITEM_BC12_COMPLETED:
			if (chip->wired_type == OPLUS_CHG_USB_TYPE_UNKNOWN)
				usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		case WIRED_ITEM_OTG_ENABLE:
			chg_info("otg enable power supply changed.\n");
			if (!IS_ERR_OR_NULL(chip->batt_psy))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			break;
		case WIRED_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chip->wired_present = !!data.intval;
			break;
		case WIRED_ITEM_CC_DETECT:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			if (!!chip->retention_topic && data.intval == CC_DETECT_NOTPLUG && !chip->wired_online) {
				chg_debug("gki_chg_ccdetect enter\n");
				schedule_work(&chip->wired_online_update_work);
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_gki_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_gki_device *chip = prv_data;
	struct power_supply_config psy_cfg = {};
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_gki_wired_subs_callback, "gki");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				true);
	chip->wired_online = data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE, &data,
				true);
	chip->wired_type = data.intval;
	if (chip->wired_online) {
		if (!chip->retention_state && chip->wired_type)
			chip->last_wired_type = chip->wired_type;
		if (chip->retention_state) {
			if (chip->last_wired_type == POWER_SUPPLY_TYPE_UNKNOWN)
				usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			else
				usb_psy_desc.type = oplus_to_power_supply_type[chip->last_wired_type];
		} else {
			usb_psy_desc.type = oplus_to_power_supply_type[chip->wired_type];
		}
		if ((usb_psy_desc.type == POWER_SUPPLY_TYPE_UNKNOWN) &&
		    oplus_gki_bc12_is_completed(chip))
			usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
	} else {
		if (chip->retention_state) {
			if (chip->last_wired_type == POWER_SUPPLY_TYPE_UNKNOWN)
				usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			else
				usb_psy_desc.type = oplus_to_power_supply_type[chip->last_wired_type];
		} else {
			chip->pre_retention_state = 0;
			chip->retention_connect_state = 0;
			chip->last_wired_type = POWER_SUPPLY_TYPE_UNKNOWN;
			usb_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		}
	}
	chg_info("psy_type=%d\n", usb_psy_desc.type);

	psy_cfg.drv_data = chip;
	chip->usb_psy = devm_power_supply_register(&topic->dev, &usb_psy_desc,
						   &psy_cfg);
	if (IS_ERR_OR_NULL(chip->usb_psy)) {
		chg_err("Failed to register usb power supply, rc=%ld\n",
			PTR_ERR(chip->usb_psy));
		goto psy_err;
	}

	return;

psy_err:
	chip->usb_psy = NULL;
	oplus_mms_unsubscribe(chip->wired_subs);
}

#define WLS_SK_TIME	5000
#define WLS_SK_DELAY_UNLOCK_TIME	100
static void oplus_chg_wls_status_keep_clean_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_gki_device *chip =
		container_of(dwork, struct oplus_gki_device, status_keep_clean_work);
	union mms_msg_data data = { 0 };

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_STATUS_KEEP, &data, true);
	if (data.intval == WLS_SK_BY_HAL || data.intval == WLS_SK_BY_FORCE_BPP) {
		oplus_chg_wls_set_status_keep(chip->wls_topic, WLS_SK_WAIT_TIMEOUT);
		schedule_delayed_work(&chip->status_keep_clean_work, msecs_to_jiffies(WLS_SK_TIME));
		return;
	}

	chip->is_ui_keep = false;
	oplus_chg_wls_set_status_keep(chip->wls_topic, WLS_SK_NULL);
	if (chip->batt_psy)
		power_supply_changed(chip->batt_psy);
	schedule_delayed_work(&chip->status_keep_delay_unlock_work, msecs_to_jiffies(WLS_SK_DELAY_UNLOCK_TIME));
}

static void oplus_chg_wls_status_keep_delay_unlock_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_gki_device *chip =
		container_of(dwork, struct oplus_gki_device, status_keep_delay_unlock_work);

	if (chip->status_wake_lock_on) {
		chg_info("release status_wake_lock\n");
		__pm_relax(chip->status_wake_lock);
		chip->status_wake_lock_on = false;
	}
}

static void oplus_gki_wls_subs_callback(struct mms_subscribe *subs,
					enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_gki_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WLS_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wls_topic, id, &data, false);
			chip->wls_online = !!data.intval;
			chg_info("wls_present=%d\n", chip->wls_online);
			if (!IS_ERR_OR_NULL(chip->batt_psy))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_gki_subscribe_wls_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_gki_device *chip = prv_data;
	struct power_supply_config psy_cfg = {};
	union mms_msg_data data = { 0 };

	chip->wls_topic = topic;
	chip->wls_subs = oplus_mms_subscribe(chip->wls_topic, chip, oplus_gki_wls_subs_callback, "gki");
	if (IS_ERR_OR_NULL(chip->wls_subs)) {
		chg_err("subscribe wls topic error, rc=%ld\n", PTR_ERR(chip->wls_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_PRESENT, &data, true);
	chip->wls_online = !!data.intval;

	psy_cfg.drv_data = chip;
	chip->wls_psy = devm_power_supply_register(&topic->dev, &wls_psy_desc, &psy_cfg);
	if (IS_ERR_OR_NULL(chip->wls_psy)) {
		chg_err("Failed to register wireless power supply, rc=%ld\n", PTR_ERR(chip->wls_psy));
		goto psy_err;
	}

	INIT_DELAYED_WORK(&chip->status_keep_clean_work, oplus_chg_wls_status_keep_clean_work);
	INIT_DELAYED_WORK(&chip->status_keep_delay_unlock_work, oplus_chg_wls_status_keep_delay_unlock_work);
	chip->status_wake_lock = wakeup_source_register(NULL, "status_wake_lock");
	chip->status_wake_lock_on = false;

	return;

psy_err:
	chip->wls_psy = NULL;
	oplus_mms_unsubscribe(chip->wls_subs);
}

static void oplus_gki_comm_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_gki_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_BATT_STATUS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->batt_status = data.intval;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
			if (chip->keep_topic != NULL)
				chip->batt_status = chip->keep.batt_status;
#endif

			chg_info("batt_status = %d, pre_batt_status = %d, wired_online = %d\n",
				  chip->batt_status, chip->pre_batt_status, chip->wired_online);
			if (!IS_ERR_OR_NULL(chip->batt_psy) && chip->pre_batt_status != chip->batt_status) {
				chip->pre_batt_status = chip->batt_status;
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			}
			break;
		case COMM_ITEM_BATT_HEALTH:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->batt_health = data.intval;
			if (!IS_ERR_OR_NULL(chip->batt_psy))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			break;
		case COMM_ITEM_BATT_CHG_TYPE:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->batt_chg_type = data.intval;
			break;
		case COMM_ITEM_UI_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->ui_soc = data.intval;
			if (chip->ui_soc < 0) {
				chip->ui_soc_ready = false;
				chip->ui_soc = 0;
			} else {
				chip->ui_soc_ready = true;
			}
			if (!IS_ERR_OR_NULL(chip->batt_psy))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			break;
		case COMM_ITEM_LED_ON:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->led_on = data.intval;
			if (chip->led_on) {
				if (is_vooc_curr_votable_available(chip))
					vote(chip->vooc_curr_votable, HIDL_VOTER, false, 0, false);

				if (is_ufcs_curr_votable_available(chip))
					vote(chip->ufcs_curr_votable, HIDL_VOTER, false, 0, false);

				if (is_pps_curr_votable_available(chip))
					vote(chip->pps_curr_votable, HIDL_VOTER, false, 0, false);
			}
			break;
		case COMM_ITEM_NOTIFY_CODE:
			if (!IS_ERR_OR_NULL(chip->batt_psy))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			break;
		case COMM_ITEM_TEMP_REGION:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->temp_region = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_gki_subscribe_comm_topic(struct oplus_mms *topic,
						void *prv_data)
{
	struct oplus_gki_device *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->comm_topic = topic;
	chip->comm_subs = oplus_mms_subscribe(chip->comm_topic, chip,
					      oplus_gki_comm_subs_callback,
					      "gki");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe common topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_BATT_STATUS, &data,
				true);
	chip->batt_status = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_BATT_HEALTH, &data,
				true);
	chip->batt_health = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_BATT_CHG_TYPE,
				&data, true);
	chip->batt_chg_type = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data,
				true);
	chip->ui_soc = data.intval;
	if (chip->ui_soc < 0) {
		chip->ui_soc_ready = false;
		chip->ui_soc = 0;
	} else {
		chip->ui_soc_ready = true;
	}
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_LED_ON, &data,
				true);
	chip->led_on = data.intval;

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_TEMP_REGION, &data,
				true);
	chip->temp_region = data.intval;
}

static void oplus_gki_vooc_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_gki_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case VOOC_ITEM_VOOC_CHARGING:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_charging = data.intval;
			if (!IS_ERR_OR_NULL(chip->batt_psy))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			break;
		case VOOC_ITEM_VOOC_STARTED:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_started = data.intval;
			if (!IS_ERR_OR_NULL(chip->batt_psy))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			break;
		case VOOC_ITEM_SID:
			if (!IS_ERR_OR_NULL(chip->batt_psy))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			break;
		case VOOC_ITEM_VOOC_BY_NORMAL_PATH:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
				false);
			chip->vooc_by_normal_path = !!data.intval;
			break;
		case VOOC_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
					false);
			chip->vooc_online = !!data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_gki_subscribe_vooc_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_gki_device *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->vooc_topic = topic;
	chip->vooc_subs = oplus_mms_subscribe(
		chip->vooc_topic, chip, oplus_gki_vooc_subs_callback, "gki");
	if (IS_ERR_OR_NULL(chip->vooc_subs)) {
		chg_err("subscribe vooc topic error, rc=%ld\n",
			PTR_ERR(chip->vooc_subs));
		return;
	}

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_CHARGING,
				&data, true);
	chip->vooc_charging = !!data.intval;

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_STARTED,
				&data, true);
	chip->vooc_started = !!data.intval;

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_BY_NORMAL_PATH,
				&data, true);
	chip->vooc_by_normal_path = !!data.intval;

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_ONLINE, &data, true);
	chip->vooc_online = !!data.intval;
}

static void oplus_gki_ufcs_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_gki_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	if (NULL == chip)
		return;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case UFCS_ITEM_ONLINE:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_online = !!data.intval;
			if ((!chip->pre_ufcs_online) && chip->ufcs_online &&
			    (!IS_ERR_OR_NULL(chip->batt_psy)))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			chip->pre_ufcs_online = chip->ufcs_online;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_gki_subscribe_ufcs_topic(struct oplus_mms *topic,
					       void *prv_data)
{
	struct oplus_gki_device *chip = prv_data;

	chip->ufcs_topic = topic;
	chip->ufcs_subs = oplus_mms_subscribe(chip->ufcs_topic, chip, oplus_gki_ufcs_subs_callback, "gki");
	if (IS_ERR_OR_NULL(chip->ufcs_subs)) {
		chg_err("subscribe ufcs topic error, rc=%ld\n",
			PTR_ERR(chip->ufcs_subs));
		return;
	}
}

static void oplus_gki_retention_checkout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_gki_device *chip =
		container_of(dwork, struct oplus_gki_device, retention_checkout_work);

	if (chip->retention_connect_state)
		return;
	if (chip->retention_connect_state == chip->pre_retention_state)
		return;
	if (chip->wired_online)
		return;
	chip->pre_retention_state = 0;
	chip->retention_connect_state = 0;
	chip->last_wired_type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->retention_state = 0;
	usb_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	if (!IS_ERR_OR_NULL(chip->batt_psy))
		oplus_power_supply_changed_gp(chip->batt_psy, 0);
	chg_debug("gki_retention_state, usb_psy_desc.type =%d\n", usb_psy_desc.type);
}

static void oplus_gki_retention_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_gki_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case RETENTION_ITEM_CONNECT_STATUS:
			oplus_mms_get_item_data(chip->retention_topic, id, &data,
					false);
			chip->retention_connect_state = !!data.intval;
			chg_debug("gki_retention_state, rc=%d\n", chip->retention_connect_state);
			if (!chip->retention_connect_state && chip->retention_connect_state != chip->pre_retention_state)
				schedule_delayed_work(&chip->retention_checkout_work, 0);
			if (chip->retention_connect_state)
				chip->pre_retention_state = chip->retention_connect_state;
			if (!chip->wired_present) {
				chip->retention_wired_plugout = true;
				if (!IS_ERR_OR_NULL(chip->batt_psy) &&
				    oplus_gki_get_batt_status(chip) != chip->batt_status)
					oplus_power_supply_changed_gp(chip->batt_psy, 0);
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_gki_subscribe_retention_topic(struct oplus_mms *topic,
					       void *prv_data)
{
	struct oplus_gki_device *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->retention_topic = topic;
	chip->retention_subs = oplus_mms_subscribe(chip->retention_topic, chip, oplus_gki_retention_subs_callback, "gki");
	if (IS_ERR_OR_NULL(chip->retention_subs)) {
		chg_err("subscribe retention topic error, rc=%ld\n",
			PTR_ERR(chip->retention_subs));
		return;
	}
	rc = oplus_mms_get_item_data(chip->retention_topic, RETENTION_ITEM_CONNECT_STATUS, &data, true);
	if (rc >= 0)
		chip->retention_state = !!data.intval;
}

static void oplus_gki_pps_subs_callback(struct mms_subscribe *subs,
					enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_gki_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PPS_ITEM_ONLINE:
			rc = oplus_mms_get_item_data(chip->pps_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->pps_online = !!data.intval;
			if ((!chip->pre_pps_online) && chip->pps_online &&
			    (!IS_ERR_OR_NULL(chip->batt_psy)))
				oplus_power_supply_changed_gp(chip->batt_psy, 0);
			chip->pre_pps_online = chip->pps_online;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_gki_subscribe_pps_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_gki_device *chip = prv_data;

	chip->pps_topic = topic;
	chip->pps_subs = oplus_mms_subscribe(chip->pps_topic, chip, oplus_gki_pps_subs_callback, "gki");
	if (IS_ERR_OR_NULL(chip->pps_subs)) {
		chg_err("subscribe pps topic error, rc=%ld\n", PTR_ERR(chip->pps_subs));
		return;
	}
}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
static void oplus_gki_keep_wired_online_update_work(struct work_struct *work)
{
	struct oplus_gki_device *chip =
		container_of(work, struct oplus_gki_device, keep_wired_online_update_work);

	chg_info("wired_online=%d, wired_type=%d\n",
		chip->keep.wired_online, chip->keep.wired_type);
	if (!chip->keep.wired_online) {
		usb_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		if (oplus_chg_get_common_charge_icl_support_flags() &&
		    is_wired_suspend_votable_available(chip))
			vote(chip->wired_suspend_votable, USB_PSY_VOTER, false, 0, false);
	} else {
		usb_psy_desc.type = oplus_to_power_supply_type[chip->keep.wired_type];
		if ((usb_psy_desc.type == POWER_SUPPLY_TYPE_UNKNOWN) &&
		    oplus_gki_bc12_is_completed(chip))
			usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
	}

	if (!IS_ERR_OR_NULL(chip->batt_psy))
		oplus_power_supply_changed_gp(chip->batt_psy, 0);
}

static void oplus_gki_keep_wired_type_check(struct oplus_gki_device *chip)
{
	if (chip->keep.wired_online) {
		usb_psy_desc.type = oplus_to_power_supply_type[chip->keep.wired_type];
		if ((usb_psy_desc.type == POWER_SUPPLY_TYPE_UNKNOWN) &&
		    oplus_gki_bc12_is_completed(chip))
			usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
	}
	chg_info("psy_type=%d, wired_type=%d\n", usb_psy_desc.type, chip->keep.wired_type);
	if (!IS_ERR_OR_NULL(chip->batt_psy))
		oplus_power_supply_changed_gp(chip->batt_psy, 0);
}
static void oplus_gki_keep_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_gki_device *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case STATE_KEEP_ITEM_WIRED_ONLINE:
			rc = oplus_mms_get_item_data(chip->keep_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->keep.wired_online = !!data.intval;
			chip->wired_online = chip->keep.wired_online;
			schedule_work(&chip->keep_wired_online_update_work);
			break;
		case STATE_KEEP_ITEM_WIRED_TYPE:
			rc = oplus_mms_get_item_data(chip->keep_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->keep.wired_type = data.intval;
			chip->wired_type = data.intval;
			oplus_gki_keep_wired_type_check(chip);
			break;
		case STATE_KEEP_ITEM_BATT_STATUS:
			rc = oplus_mms_get_item_data(chip->keep_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->keep.batt_status = data.intval;
			chip->batt_status = chip->keep.batt_status;
			chg_info("keep.batt_status = %d\n", chip->keep.batt_status);
			oplus_power_supply_changed_gp(chip->batt_psy, 0);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_gki_subscribe_keep_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_gki_device *chip = prv_data;

	chip->keep_topic = topic;
	chip->keep_subs = oplus_mms_subscribe(chip->keep_topic, chip, oplus_gki_keep_subs_callback, "gki");
	if (IS_ERR_OR_NULL(chip->keep_subs)) {
		chg_err("subscribe state_keep topic error, rc=%ld\n", PTR_ERR(chip->keep_subs));
		return;
	}
}
#endif

static void oplus_chg_gki_power_supply_update(void *data)
{
	struct power_supply *psy = data;
	if (psy != NULL)
		power_supply_changed(psy);
}

void oplus_power_supply_changed_gp(struct power_supply *psy, unsigned int grace_period_ms)
{
	struct oplus_gki_device *gki_dev = g_gki_dev;
	struct oplus_msg_queue *queue;
	int rc;

	if (psy == NULL) {
		chg_err("psy is NULL\n");
		return;
	}
	if (gki_dev == NULL) {
		chg_err("g_gki_dev is NULL\n");
		return;
	}

	queue = oplus_msg_filter_find_queue(&gki_dev->filter, psy->desc->name);
	if (queue == NULL) {
		rc = oplus_msg_filter_create_queue(&gki_dev->filter, psy->desc->name, psy);
		if (rc < 0)
			return;
		oplus_msg_filter_update_by_name_gp(&gki_dev->filter, psy->desc->name, grace_period_ms);
		return;
	}
	oplus_msg_filter_update_gp(queue, grace_period_ms);
}

void oplus_power_supply_changed(struct power_supply *psy)
{
	struct oplus_gki_device *gki_dev = g_gki_dev;
	struct oplus_msg_queue *queue;
	int rc;

	if (psy == NULL) {
		chg_err("psy is NULL\n");
		return;
	}
	if (gki_dev == NULL) {
		chg_err("g_gki_dev is NULL\n");
		return;
	}

	queue = oplus_msg_filter_find_queue(&gki_dev->filter, psy->desc->name);
	if (queue == NULL) {
		rc = oplus_msg_filter_create_queue(&gki_dev->filter, psy->desc->name, psy);
		if (rc < 0)
			return;
		oplus_msg_filter_update_by_name(&gki_dev->filter, psy->desc->name);
		return;
	}
	oplus_msg_filter_update(queue);
}

static int oplus_gki_driver_probe(struct platform_device *pdev)
{
	struct oplus_gki_device *gki_dev = g_gki_dev;
	struct power_supply *psy = NULL;
	struct power_supply_config psy_cfg = {};

	if (gki_dev == NULL) {
		chg_err("g_gki_dev is NULL\n");
		return -ENODEV;
	}

	gki_dev->dev = &pdev->dev;
	psy = power_supply_get_by_name("battery");
	if (psy != NULL) {
		chg_info("battery power_sypply already registered\n");
		return 0;
	}

	psy_cfg.drv_data = gki_dev;
	gki_dev->batt_psy = devm_power_supply_register(gki_dev->dev, &batt_psy_desc, &psy_cfg);
	if (IS_ERR_OR_NULL(gki_dev->batt_psy)) {
		chg_err("Failed to register battery power supply, rc=%ld\n",
			PTR_ERR(gki_dev->batt_psy));
		return PTR_ERR(gki_dev->batt_psy);
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_gki_driver_remove(struct platform_device *pdev)
#else
static int oplus_gki_driver_remove(struct platform_device *pdev)
#endif
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_chg_gki_match[] = {
	{ .compatible = "oplus,chg-gki" },
	{},
};

static struct platform_driver oplus_chg_gki_driver = {
	.driver = {
		.name = "oplus_chg_gki",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_chg_gki_match),
	},
	.probe = oplus_gki_driver_probe,
	.remove = oplus_gki_driver_remove,
};

static __init int oplus_chg_gki_init(void)
{
	struct oplus_gki_device *gki_dev;
	struct device_node *node;

	gki_dev = kzalloc(sizeof(struct oplus_gki_device), GFP_KERNEL);
	if (gki_dev == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	g_gki_dev = gki_dev;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		gki_dev->smart_charging_screenoff = false;
	else
		gki_dev->smart_charging_screenoff = of_property_read_bool(
			node, "oplus,smart_charging_screenoff");

	gki_dev->time_to_full = 0;
	INIT_WORK(&gki_dev->gauge_update_work, oplus_gki_gauge_update_work);
	INIT_WORK(&gki_dev->wired_online_update_work, oplus_gki_wired_online_update_work);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	INIT_WORK(&gki_dev->keep_wired_online_update_work, oplus_gki_keep_wired_online_update_work);
#endif
	INIT_DELAYED_WORK(&gki_dev->retention_checkout_work, oplus_gki_retention_checkout_work);
	INIT_DELAYED_WORK(&gki_dev->usb_phy_suspend_recovery_work,
		oplus_usb_phy_suspend_recovery_work);
	oplus_msg_filter_init(&gki_dev->filter, 500, 5000, oplus_chg_gki_power_supply_update);

	oplus_mms_wait_topic("gauge", oplus_gki_subscribe_gauge_topic, gki_dev);
	oplus_mms_wait_topic("wired", oplus_gki_subscribe_wired_topic, gki_dev);
	oplus_mms_wait_topic("wireless", oplus_gki_subscribe_wls_topic, gki_dev);
	oplus_mms_wait_topic("common", oplus_gki_subscribe_comm_topic, gki_dev);
	oplus_mms_wait_topic("vooc", oplus_gki_subscribe_vooc_topic, gki_dev);
	oplus_mms_wait_topic("ufcs", oplus_gki_subscribe_ufcs_topic, gki_dev);
	oplus_mms_wait_topic("retention", oplus_gki_subscribe_retention_topic, gki_dev);
	oplus_mms_wait_topic("pps", oplus_gki_subscribe_pps_topic, gki_dev);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	oplus_mms_wait_topic("state_keep", oplus_gki_subscribe_keep_topic, gki_dev);
#endif

	platform_driver_register(&oplus_chg_gki_driver);
	return 0;
}

static __exit void oplus_chg_gki_exit(void)
{
	if (g_gki_dev == NULL)
		return;

	if (g_gki_dev->batt_psy)
		power_supply_unregister(g_gki_dev->batt_psy);
	if (g_gki_dev->usb_psy)
		power_supply_unregister(g_gki_dev->usb_psy);
	if (g_gki_dev->wls_psy)
		power_supply_unregister(g_gki_dev->wls_psy);
	if (!IS_ERR_OR_NULL(g_gki_dev->wired_subs))
		oplus_mms_unsubscribe(g_gki_dev->wired_subs);
	if (!IS_ERR_OR_NULL(g_gki_dev->gauge_subs))
		oplus_mms_unsubscribe(g_gki_dev->gauge_subs);
	if (!IS_ERR_OR_NULL(g_gki_dev->wls_subs))
		oplus_mms_unsubscribe(g_gki_dev->wls_subs);
	if (!IS_ERR_OR_NULL(g_gki_dev->comm_subs))
		oplus_mms_unsubscribe(g_gki_dev->comm_subs);
	if (!IS_ERR_OR_NULL(g_gki_dev->vooc_subs))
		oplus_mms_unsubscribe(g_gki_dev->vooc_subs);
	if (!IS_ERR_OR_NULL(g_gki_dev->ufcs_subs))
		oplus_mms_unsubscribe(g_gki_dev->ufcs_subs);
	if (!IS_ERR_OR_NULL(g_gki_dev->retention_subs))
		oplus_mms_unsubscribe(g_gki_dev->retention_subs);
	if (!IS_ERR_OR_NULL(g_gki_dev->pps_subs))
		oplus_mms_unsubscribe(g_gki_dev->pps_subs);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	if (!IS_ERR_OR_NULL(g_gki_dev->keep_subs))
		oplus_mms_unsubscribe(g_gki_dev->keep_subs);
#endif
	oplus_msg_filter_release_queue(&g_gki_dev->filter);
	kfree(g_gki_dev);
	g_gki_dev = NULL;
	platform_driver_unregister(&oplus_chg_gki_driver);
}

oplus_chg_module_register(oplus_chg_gki);
