#define pr_fmt(fmt) "[MONITOR]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/power_supply.h>
#include <linux/sched/clock.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#include <oplus_chg_module.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_vooc.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_comm.h>
#include <oplus_battery_log.h>
#include <oplus_smart_chg.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_wls.h>
#include <oplus_chg_state_retention.h>

#include "oplus_monitor_internal.h"
#include <oplus_chg_dual_chan.h>
#if IS_ENABLED(CONFIG_OPLUS_FPGA_NOTIFY)
#include <soc/oplus/fpga_notify.h>
#include <oplus_i2c_rst_notify.h>
#endif
#include <oplus_chg_plc.h>

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
#include "track/oplus_track_state_keep.h"
#endif

__maybe_unused static bool is_fv_votable_available(struct oplus_monitor *chip)
{
	if (!chip->fv_votable)
		chip->fv_votable = find_votable("FV_MIN");
	return !!chip->fv_votable;
}

__maybe_unused static bool
is_wired_icl_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wired_icl_votable)
		chip->wired_icl_votable = find_votable("WIRED_ICL");
	return !!chip->wired_icl_votable;
}

__maybe_unused static bool
is_wired_fcc_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wired_fcc_votable)
		chip->wired_fcc_votable = find_votable("WIRED_FCC");
	return !!chip->wired_fcc_votable;
}

__maybe_unused static bool
is_wired_charge_suspend_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wired_charge_suspend_votable)
		chip->wired_charge_suspend_votable =
			find_votable("WIRED_CHARGE_SUSPEND");
	return !!chip->wired_charge_suspend_votable;
}

__maybe_unused static bool
is_wired_charging_disable_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wired_charging_disable_votable)
		chip->wired_charging_disable_votable =
			find_votable("WIRED_CHARGING_DISABLE");
	return !!chip->wired_charging_disable_votable;
}

__maybe_unused static bool
is_wls_icl_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wls_icl_votable)
		chip->wls_icl_votable = find_votable("WLS_NOR_ICL");
	return !!chip->wls_icl_votable;
}

__maybe_unused static bool
is_wls_fcc_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wls_fcc_votable)
		chip->wls_fcc_votable = find_votable("WLS_FCC");
	return !!chip->wls_fcc_votable;
}

__maybe_unused static bool
is_wls_charge_suspend_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wls_charge_suspend_votable)
		chip->wls_charge_suspend_votable =
			find_votable("WLS_CHARGE_SUSPEND");
	return !!chip->wls_charge_suspend_votable;
}

__maybe_unused static bool
is_wls_charging_disable_votable_available(struct oplus_monitor *chip)
{
	if (!chip->wls_charging_disable_votable)
		chip->wls_charging_disable_votable =
			find_votable("WLS_NOR_OUT_DISABLE");
	return !!chip->wls_charging_disable_votable;
}

__maybe_unused static bool
is_chg_disable_votable_available(struct oplus_monitor *chip)
{
	if (!chip->chg_disable_votable)
		chip->chg_disable_votable = find_votable("CHG_DISABLE");
	return !!chip->chg_disable_votable;
}

__maybe_unused static bool
is_vooc_curr_votable_available(struct oplus_monitor *chip)
{
	if (!chip->vooc_curr_votable)
		chip->vooc_curr_votable = find_votable("VOOC_CURR");
	return !!chip->vooc_curr_votable;
}

static bool is_main_gauge_topic_available(struct oplus_monitor *chip)
{
	if (!chip->main_gauge_topic)
		chip->main_gauge_topic = oplus_mms_get_by_name("gauge:0");

	return !!chip->main_gauge_topic;
}

static bool is_sub_gauge_topic_available(struct oplus_monitor *chip)
{
	if (!chip->sub_gauge_topic)
		chip->sub_gauge_topic = oplus_mms_get_by_name("gauge:1");

	return !!chip->sub_gauge_topic;
}

static void oplus_monitor_update_charge_info(struct oplus_monitor *chip)
{
	union mms_msg_data data = { 0 };

	if (is_wired_icl_votable_available(chip))
		chip->wired_icl_ma = get_effective_result(chip->wired_icl_votable);
	if (is_wired_charge_suspend_votable_available(chip))
		chip->wired_suspend = get_effective_result(chip->wired_charge_suspend_votable);
	if (is_wired_charge_suspend_votable_available(chip))
		chip->wired_user_suspend = get_client_vote(chip->wired_charge_suspend_votable, USER_VOTER);
	chip->wired_vbus_mv = oplus_wired_get_vbus();

	if (is_wls_icl_votable_available(chip))
		chip->wls_icl_ma = get_effective_result(chip->wls_icl_votable);
	if (is_wls_charge_suspend_votable_available(chip))
		chip->wls_suspend = get_effective_result(chip->wls_charge_suspend_votable);
	if (is_wls_charge_suspend_votable_available(chip))
		chip->wls_user_suspend = get_client_vote(chip->wls_charge_suspend_votable, USER_VOTER);

	if (is_chg_disable_votable_available(chip))
		chip->mmi_chg = !get_client_vote(chip->chg_disable_votable, MMI_CHG_VOTER);
	if (is_vooc_curr_votable_available(chip))
		chip->bcc_current = get_client_vote(chip->vooc_curr_votable, BCC_VOTER);

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_USB_STATUS, &data, true);
	chip->usb_status = data.intval;
	if (chip->wls_topic) {
		oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_NOTIFY, &data, true);
		chip->usb_status = chip->usb_status | data.intval;
	}

	chip->normal_cool_down = oplus_smart_chg_get_normal_cool_down();
	chip->otg_switch_status = oplus_wired_get_otg_switch_status();

	if (chip->wired_online) {
		if (is_wired_fcc_votable_available(chip))
			chip->fcc_ma = get_effective_result(chip->wired_fcc_votable);
		if (is_fv_votable_available(chip))
			chip->fv_mv = get_effective_result(chip->fv_votable);
		if (is_wired_charging_disable_votable_available(chip))
			chip->chg_disable = get_effective_result(chip->wired_charging_disable_votable);
		if (is_wired_charging_disable_votable_available(chip))
			chip->chg_user_disable = get_client_vote(chip->wired_charging_disable_votable, USER_VOTER);

		chip->wired_ibus_ma = oplus_wired_get_ibus();
	} else if (chip->wls_online) {
		if (is_wls_fcc_votable_available(chip))
			chip->fcc_ma = get_effective_result(chip->wls_fcc_votable);
		if (is_fv_votable_available(chip))
			chip->fv_mv = get_effective_result(chip->fv_votable);
		if (is_wls_charging_disable_votable_available(chip))
			chip->chg_disable = get_effective_result(chip->wls_charging_disable_votable);
		if (is_wls_charging_disable_votable_available(chip))
			chip->chg_user_disable = get_client_vote(chip->wls_charging_disable_votable, USER_VOTER);
		if (chip->wls_topic) {
			oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_IOUT, &data, true);
			chip->wls_iout_ma = data.intval;
			oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_VOUT, &data, true);
			chip->wls_vout_mv = data.intval;
			oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_WLS_TYPE, &data, true);
			chip->wls_charge_type = data.intval;
			if (chip->wls_charge_type)
				chip->wls_pre_type = chip->wls_charge_type;
			oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_MAGCVR, &data, true);
			chip->wls_magcvr_status = data.intval;
		}
	} else {
		chip->fcc_ma = 0;
		chip->fv_mv = 0;
		chip->chg_disable = true;
		chip->chg_user_disable = true;

		chip->wired_ibus_ma = 0;

		chip->wls_iout_ma = 0;
		chip->wls_vout_mv = 0;
		chip->wls_charge_type = 0;
		chip->wls_magcvr_status = MAGCVR_STATUS_FAR;
	}
	if (is_main_gauge_topic_available(chip)) {
		oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_VOL_MAX, &data, false);
		chip->main_vbat = data.intval;
		oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_CURR, &data, false);
		chip->main_ibat = data.intval;
		oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_TEMP, &data, false);
		chip->main_temp = data.intval;
		oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_SOC, &data, false);
		chip->main_soc = data.intval;
	}
	if (is_sub_gauge_topic_available(chip)) {
		oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_VOL_MAX, &data, false);
		chip->sub_vbat = data.intval;
		chip->vbat_min_mv = data.intval;
		oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_CURR, &data, false);
		chip->sub_ibat = data.intval;
		oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_TEMP, &data, false);
		chip->sub_temp = data.intval;
		oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_SOC, &data, false);
		chip->sub_soc = data.intval;
	}
}

static bool oplus_monitor_all_topic_is_ready(struct oplus_monitor *chip)
{
	if (!chip->wired_topic) {
		chg_err("wired topic not ready\n");
		return false;
	}
	/* TODO: wirelsee */
	if (!chip->gauge_topic) {
		chg_err("gauge topic not ready\n");
		return false;
	}
	if (!chip->vooc_topic) {
		chg_err("vooc topic not ready\n");
		return false;
	}
	if (!chip->comm_topic) {
		chg_err("common topic not ready\n");
		return false;
	}

	if (!chip->gauge_inited) {
		chg_err("gauge data not init\n");
		return false;
	}

	return true;
}

#define DUMP_REG_LOG_CNT_30S	3
#define MAX_SUB_BATT_INFO	32
static void oplus_monitor_charge_info_update_work(struct work_struct *work)
{
	struct oplus_monitor *chip = container_of(work, struct oplus_monitor,
						  charge_info_update_work);
	union mms_msg_data data = { 0 };
	static int dump_count = 0;
	static int update_reg_soc = -EINVAL;
	int rc;
	char sub_batt_info[MAX_SUB_BATT_INFO] = {0};
	int index = 0;

	if (chip->wired_online || chip->wls_online)
		oplus_mms_restore_publish(chip->err_topic);
	else
		oplus_mms_stop_publish(chip->err_topic);

	oplus_monitor_update_charge_info(chip);

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP,
				     &data, false);
	if (!rc)
		chip->shell_temp = data.intval;

	/*
	 * wait for all dependent topics to be prepared to ensure that the
	 * collected data is normal
	 */
	if (oplus_monitor_all_topic_is_ready(chip))
		oplus_chg_track_comm_monitor(chip);
	if (is_main_gauge_topic_available(chip) && is_sub_gauge_topic_available(chip))
		index += snprintf(sub_batt_info + index, MAX_SUB_BATT_INFO - index,
			"main_sub[%d %d %d %d]",
			chip->main_ibat, chip->main_soc, chip->sub_ibat, chip->sub_soc);

	printk(KERN_INFO "OPLUS_CHG[oplus_charge_info]: "
		"BATTERY[%d %d %d %d %d %d %d %d %d %d %d 0x%x %d], "
		"CHARGE[%d %d %d %d], "
		"WIRED[%d %d %d %d %d 0x%x %d %d %d %d %d], "
		"WIRELESS[%d %d %d %d %d 0x%x %d %d %d], "
		"VOOC[%d %d %d %d 0x%x], "
		"UFCS[%d %d %d 0x%x], "
		"COMMON[%d %d %d 0x%x %d %d %d %d %d %d] %s",
		chip->batt_temp, chip->shell_temp, chip->vbat_mv,
		chip->vbat_min_mv, chip->ibat_ma, chip->batt_soc, chip->ui_soc,
		chip->smooth_soc, chip->batt_rm, chip->batt_fcc, chip->batt_exist,
		chip->batt_err_code, chip->gauge_car_c,
		chip->fv_mv, chip->fcc_ma, chip->chg_disable, chip->chg_user_disable,
		chip->wired_online, chip->wired_ibus_ma, chip->wired_vbus_mv,
		chip->wired_icl_ma, chip->wired_charge_type, chip->wired_err_code,
		chip->wired_suspend, chip->wired_user_suspend, chip->cc_mode,
		chip->cc_detect, chip->otg_enable,
		chip->wls_online, chip->wls_iout_ma, chip->wls_vout_mv,
		chip->wls_icl_ma, chip->wls_charge_type, chip->wls_err_code,
		chip->wls_suspend, chip->wls_user_suspend, chip->wls_magcvr_status,
		chip->vooc_online, chip->vooc_started, chip->vooc_charging,
		chip->vooc_online_keep, chip->vooc_sid,
		chip->ufcs_online, chip->ufcs_charging, chip->ufcs_oplus_adapter,
		chip->ufcs_adapter_id,
		chip->temp_region, chip->ffc_status, chip->cool_down,
		chip->notify_code, chip->led_on, chip->deep_support, chip->delta_soc,
		chip->batt_fcc_comp, chip->batt_soh_comp, chip->uisoc_keep_2_err,
		sub_batt_info);

	if (chip->wired_online || chip->wls_online) {
		update_reg_soc = chip->batt_soc;
	} else {
		if (update_reg_soc != chip->batt_soc) {
			update_reg_soc = chip->batt_soc;
			rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_REG_INFO, &data, true);
			if (rc == 0 && data.strval && strlen(data.strval))
				printk(KERN_INFO "OPLUS_CHG [main_gauge_reg_info] %s\n", data.strval);
			if (is_sub_gauge_topic_available(chip)) {
				rc = oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_REG_INFO, &data, true);
				if (rc == 0 && data.strval && strlen(data.strval))
					printk(KERN_INFO "OPLUS_CHG [sub_gauge_reg_info] %s\n", data.strval);
			}
		}
	}

	if (!chip->wired_online)
		dump_count++;

	if ((chip->wired_online || dump_count == DUMP_REG_LOG_CNT_30S)
		&& (!chip->vooc_started || oplus_vooc_get_voocphy_support(chip->vooc_topic) == ADSP_VOOCPHY)) {
		dump_count = 0;
		oplus_wired_dump_regs();
	}
}

static int comm_info_dump_log_data(char *buffer, int size, void *dev_data)
{
	struct oplus_monitor *chip = dev_data;

	if (!buffer || !chip)
		return -ENOMEM;

	snprintf(buffer, size, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
		"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
		"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
		"%d,%d,%d,%d",
		chip->batt_temp, chip->shell_temp, chip->vbat_mv, chip->vbat_min_mv, chip->ibat_ma,
		chip->batt_soc, chip->ui_soc, chip->wired_online, chip->wired_charge_type, chip->notify_code,
		chip->wired_ibus_ma, chip->wired_vbus_mv, chip->smooth_soc, chip->led_on, chip->fv_mv,
		chip->fcc_ma, chip->wired_icl_ma, chip->otg_switch_status, chip->cool_down, chip->bcc_current,
		chip->normal_cool_down, chip->chg_cycle_status, chip->mmi_chg, chip->usb_status, chip->cc_detect,
		chip->batt_full, chip->rechging, chip->pd_svooc, chip->batt_status, chip->batt_qmax,
		chip->batt_soh, chip->gauge_car_c, chip->batt_rm, chip->batt_fcc);

	return 0;
}

static int comm_info_get_log_head(char *buffer, int size, void *dev_data)
{
	struct oplus_monitor *chip = dev_data;

	if (!buffer || !chip)
		return -ENOMEM;

	snprintf(buffer, size,
		",batt_temp,shell_temp,vbat_mv,vbat_min_mv,ibat_ma,"
		"batt_soc,ui_soc,wired_online,charge_type,notify_code,"
		"wired_ibus_ma,wired_vbus_mv,smooth_soc,led_on,fv_mv,"
		"fcc_ma,wired_icl_ma,otg_switch,cool_down,bcc_current,normal_cool_down,chg_cycle,"
		"mmi_chg,usb_status,cc_detect,batt_full,rechging,pd_svooc,prop_status,batt_qmax,"
		"batt_soh,gauge_car_c,batt_rm,batt_fcc");

	return 0;
}

static struct battery_log_ops battlog_comm_ops = {
	.dev_name = "comm_info",
	.dump_log_head = comm_info_get_log_head,
	.dump_log_content = comm_info_dump_log_data,
};

static void oplus_monitor_dual_chan_subs_callback(struct mms_subscribe *subs,
						enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_monitor *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case SWITCH_ITEM_DUAL_CHAN_STATUS:
			oplus_mms_get_item_data(chip->dual_chan_topic, id, &data,
						false);
			if (!!data.intval)
				oplus_chg_track_record_dual_chan_start(chip);
			else
				oplus_chg_track_record_dual_chan_end(chip);
			break;
		default:
			break;
		}
		    break;
	default:
		    break;
	}
}

static void oplus_monitor_subscribe_dual_chan_topic(struct oplus_mms *topic,
					void *prv_data)
{
	struct oplus_monitor *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->dual_chan_topic = topic;
	chip->dual_chan_subs =
		oplus_mms_subscribe(chip->dual_chan_topic, chip,
				    oplus_monitor_dual_chan_subs_callback,
				    "monitor");
	if (IS_ERR_OR_NULL(chip->dual_chan_subs)) {
		chg_err("subscribe dual_chan topic error, rc=%ld\n",
			PTR_ERR(chip->dual_chan_subs));
		return;
	}
	oplus_mms_get_item_data(chip->dual_chan_topic, SWITCH_ITEM_DUAL_CHAN_STATUS, &data,
				true);
	if (!!data.intval)
		oplus_chg_track_record_dual_chan_start(chip);
	else
		oplus_chg_track_record_dual_chan_end(chip);
}


static void oplus_monitor_gauge_subs_callback(struct mms_subscribe *subs,
					      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_monitor *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX,
					&data, false);
		chip->vbat_mv = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP,
					&data, false);
		chip->batt_temp = data.intval;
		if (is_sub_gauge_topic_available(chip)) {
			oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_VOL_MAX,
						&data, false);
			chip->vbat_min_mv = data.intval;
		} else {
			oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN,
						&data, false);
			chip->vbat_min_mv = data.intval;
		}
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC,
					&data, false);
		chip->batt_soc = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR,
					&data, false);
		chip->ibat_ma = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC,
					&data, false);
		chip->batt_fcc = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RM, &data,
					false);
		chip->batt_rm = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data,
					false);
		chip->batt_cc = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOH, &data,
					false);
		chip->batt_soh = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_DEEP_SUPPORT,
			&data, false);
		chip->deep_support = data.intval;

		chip->batt_fcc_comp = min(chip->batt_fcc + chip->batt_fcc_coeff * chip->batt_soh / 100,
			oplus_gauge_get_batt_capacity_mah(chip->gauge_topic));
		chip->batt_soh_comp = min(chip->batt_soh + chip->batt_soh_coeff * chip->batt_soh / 100, 100);
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_QMAX, &data, false);
		chip->batt_qmax = data.intval;
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CAR_C, &data, false);
		chip->gauge_car_c = data.intval;
		schedule_work(&chip->charge_info_update_work);
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case GAUGE_ITEM_BATT_EXIST:
			oplus_mms_get_item_data(chip->gauge_topic, id, &data,
						false);
			chip->batt_exist = !!data.intval;
			break;
		case GAUGE_ITEM_ERR_CODE:
			oplus_mms_get_item_data(chip->gauge_topic, id, &data,
						false);
			chip->batt_err_code = (unsigned int)data.intval;
			break;
		case GAUGE_ITEM_FCC_COEFF:
			oplus_mms_get_item_data(chip->gauge_topic, id, &data, false);
			chip->batt_fcc_coeff = data.intval;
			break;
		case GAUGE_ITEM_SOH_COEFF:
			oplus_mms_get_item_data(chip->gauge_topic, id, &data, false);
			chip->batt_soh_coeff = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_monitor_subscribe_gauge_topic(struct oplus_mms *topic,
					    void *prv_data)
{
	struct oplus_monitor *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->gauge_topic = topic;
	chip->gauge_subs =
		oplus_mms_subscribe(chip->gauge_topic, chip,
				    oplus_monitor_gauge_subs_callback,
				    "monitor");
	if (IS_ERR_OR_NULL(chip->gauge_subs)) {
		chg_err("subscribe gauge topic error, rc=%ld\n",
			PTR_ERR(chip->gauge_subs));
		return;
	}

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data,
				true);
	chip->vbat_mv = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data,
				true);
	chip->batt_temp = data.intval;
	if (is_sub_gauge_topic_available(chip)) {
		oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_VOL_MAX,
					&data, false);
		chip->vbat_min_mv = data.intval;
	} else {
		oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data,
					true);
		chip->vbat_min_mv = data.intval;
	}
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data,
				true);
	chip->batt_soc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data,
				true);
	chip->ibat_ma = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_FCC, &data,
				true);
	chip->batt_fcc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_RM, &data,
				true);
	chip->batt_rm = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CC, &data,
				true);
	chip->batt_cc = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOH, &data,
				true);
	chip->batt_soh = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_BATT_EXIST, &data,
				true);
	chip->batt_exist = !!data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_ERR_CODE, &data,
				true);
	chip->batt_err_code = (unsigned int)data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_DEEP_SUPPORT,
		&data, true);
	chip->deep_support = data.intval;

	chip->batt_fcc_comp = min(chip->batt_fcc + chip->batt_fcc_coeff * chip->batt_soh / 100,
		oplus_gauge_get_batt_capacity_mah(chip->gauge_topic));
	chip->batt_soh_comp = min(chip->batt_soh + chip->batt_soh_coeff * chip->batt_soh / 100, 100);
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_QMAX, &data, true);
	chip->batt_qmax = data.intval;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CAR_C, &data, true);
	chip->gauge_car_c = data.intval;
	chip->gauge_inited = true;
}

static void oplus_monitor_retention_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_monitor *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case RETENTION_ITEM_CONNECT_STATUS:
			oplus_mms_get_item_data(chip->retention_topic, id, &data, false);
			chip->retention_state = !!data.intval;
			if (chip->pre_retention_state != chip->retention_state && !chip->retention_state &&
				chip->total_disconnect_count > 0)
				oplus_chg_track_upload_wired_retention_online_info(chip);
			chip->pre_retention_state = chip->retention_state;
			oplus_chg_track_update_break_ui_online();
			break;
		case RETENTION_ITEM_TOTAL_DISCONNECT_COUNT:
			oplus_mms_get_item_data(chip->retention_topic, id, &data, false);
			if (data.intval > 0)
				chip->total_disconnect_count = data.intval - chip->vooc_normal_connect_count_level;
			if (!chip->retention_state)
				chip->vooc_normal_connect_count_level = 0;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_monitor_subscribe_retention_topic(struct oplus_mms *topic,
					       void *prv_data)
{
	struct oplus_monitor *chip = prv_data;

	chip->retention_topic = topic;
	chip->retention_subs =
		oplus_mms_subscribe(chip->retention_topic, chip,
				    oplus_monitor_retention_subs_callback,
				    "monitor");
	if (IS_ERR_OR_NULL(chip->retention_subs)) {
		chg_err("subscribe retention topic error, rc=%ld\n",
			PTR_ERR(chip->retention_subs));
		return;
	}
}

static void oplus_monitor_ufcs_subs_callback(struct mms_subscribe *subs,
					     enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_monitor *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case UFCS_ITEM_ONLINE:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_online = !!data.intval;
			break;
		case UFCS_ITEM_CHARGING:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_charging = !!data.intval;
			break;
		case UFCS_ITEM_ADAPTER_ID:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_adapter_id = (u32)data.intval;
			if (chip->ufcs_adapter_id > 0)
				chip->pre_ufcs_adapter_id = chip->ufcs_adapter_id;
			break;
		case UFCS_ITEM_OPLUS_ADAPTER:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_oplus_adapter = !!data.intval;
			if (chip->ufcs_oplus_adapter > 0)
				chip->pre_ufcs_oplus_adapter = chip->ufcs_oplus_adapter;
			break;
		case UFCS_ITEM_EMARK_POWER:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_emark_power = data.intval;
			if (chip->ufcs_emark_power > 0) {
				chip->pre_ufcs_emark_power = chip->ufcs_emark_power;
				oplus_chg_track_handle_wired_type_info(chip, TRACK_CHG_GET_THTS_TIME_TYPE);
			}
			break;
		case UFCS_ITEM_ADAPTER_POWER:
			rc = oplus_mms_get_item_data(chip->ufcs_topic, id, &data, false);
			if (rc < 0)
				break;
			chip->ufcs_adapter_power = data.intval;
			if (chip->ufcs_adapter_power > 0) {
				chip->pre_ufcs_adapter_power = chip->ufcs_adapter_power;
				oplus_chg_track_handle_wired_type_info(chip, TRACK_CHG_GET_THTS_TIME_TYPE);
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

static void oplus_monitor_subscribe_ufcs_topic(struct oplus_mms *topic,
					       void *prv_data)
{
	struct oplus_monitor *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->ufcs_topic = topic;
	chip->ufcs_subs =
		oplus_mms_subscribe(chip->ufcs_topic, chip,
				    oplus_monitor_ufcs_subs_callback,
				    "monitor");
	if (IS_ERR_OR_NULL(chip->ufcs_subs)) {
		chg_err("subscribe ufcs topic error, rc=%ld\n",
			PTR_ERR(chip->ufcs_subs));
		return;
	}

	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_ONLINE, &data, true);
	if (rc >= 0)
		chip->ufcs_online = !!data.intval;

	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_CHARGING, &data, true);
	if (rc >= 0)
		chip->ufcs_charging = !!data.intval;

	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_ADAPTER_ID, &data, true);
	if (rc >= 0) {
		chip->ufcs_adapter_id = (u32)data.intval;
		chip->pre_ufcs_adapter_id = chip->ufcs_adapter_id;
	}

	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_OPLUS_ADAPTER, &data, true);
	if (rc >= 0) {
		chip->ufcs_oplus_adapter = !!data.intval;
		chip->pre_ufcs_oplus_adapter = chip->ufcs_oplus_adapter;
	}

	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_EMARK_POWER, &data, true);
	if (rc >= 0) {
		chip->ufcs_emark_power = data.intval;
		chip->pre_ufcs_emark_power = chip->ufcs_emark_power;
	}

	rc = oplus_mms_get_item_data(chip->ufcs_topic, UFCS_ITEM_ADAPTER_POWER, &data, true);
	if (rc >= 0) {
		chip->ufcs_adapter_power = data.intval;
		chip->pre_ufcs_adapter_power = chip->ufcs_adapter_power;
	}

	if (chip->ufcs_emark_power > 0 || chip->ufcs_adapter_power > 0)
		oplus_chg_track_handle_wired_type_info(chip, TRACK_CHG_GET_THTS_TIME_TYPE);

}

static void oplus_monitor_plc_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_monitor *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PLC_ITEM_STATUS:
			oplus_mms_get_item_data(chip->plc_topic, id, &data,
						false);
			chip->plc_status = data.intval;
			break;
		case PLC_ITEM_ENABLE_CNTS:
			oplus_mms_get_item_data(chip->plc_topic, id, &data,
						false);
			chip->enable_count = data.intval;
			if (chip->enable_count == 1) {
				chip->plc_init_sm_soc = chip->ui_soc;
				chip->plc_init_ui_soc = chip->smooth_soc;
				chip->plc_init_temp = chip->shell_temp;
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

static void oplus_monitor_subscribe_plc_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_monitor *chip = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	chip->plc_topic = topic;
	chip->plc_subs = oplus_mms_subscribe(chip->plc_topic, chip,
					     oplus_monitor_plc_subs_callback,
					     "monitor");
	if (IS_ERR_OR_NULL(chip->plc_subs)) {
		chg_err("subscribe plc topic error, rc=%ld\n",
			PTR_ERR(chip->plc_subs));
		return;
	}
	chip->plc_support = true;

	rc = oplus_mms_get_item_data(chip->plc_topic, PLC_ITEM_STATUS, &data, true);
	if (rc >= 0)
		chip->plc_status = data.intval;

	rc = oplus_mms_get_item_data(chip->plc_topic, PLC_ITEM_ENABLE_CNTS, &data, true);
	if (rc >= 0)
		chip->enable_count = data.intval;

	if (chip->enable_count == 1) {
		chip->plc_init_sm_soc = chip->ui_soc;
		chip->plc_init_ui_soc = chip->smooth_soc;
		chip->plc_init_temp = chip->shell_temp;
	}
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
static void oplus_comm_publish_chg_err_otg_en_msg(struct oplus_monitor *chip, bool otg_en)
{
	struct votable *otg_disable_votable;

	otg_disable_votable = find_votable("OTG_DISABLE");
	if (otg_disable_votable == NULL)
		return;

	vote(otg_disable_votable, LIQ_ERR_VOTER, !otg_en, !otg_en, false);
}
#endif

#define OPLUS_CHG_IC_USB_PLUGOUT_DELAY 5000
static void oplus_monitor_wired_plugin_work(struct work_struct *work)
{
	struct oplus_monitor *chip =
		container_of(work, struct oplus_monitor, wired_plugin_work);
	oplus_chg_track_check_wired_charging_break(chip->wired_online);
	chip->oplus_liquid_intake_check_led_status = false;
}

static void oplus_monitor_wired_present_work(struct work_struct *work)
{
	bool wired_present = false;
	union mms_msg_data data = { 0 };
	struct oplus_monitor *chip =
		container_of(work, struct oplus_monitor, wired_present_work);

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_PRESENT, &data, false);
	wired_present = !!data.intval;
	oplus_chg_track_check_wired_mul_break_stat(wired_present);
}

#define TIMER_SIZE 10
#define DEBUG_WATER_INLET_PARA (3 * 60 * 60 * 1000) /* 3 hours */
#define TRACK_LOCAL_T_NS_TO_MS_THD 1000000
#define CHG_INTO_L_MAX 300
void oplus_chg_water_inlet_detect(struct oplus_monitor *chip, int reason, int plugin_count)
{
	static struct oplus_chg_into_l timer[TIMER_SIZE] = {0};
	static unsigned long long pre_timer = 0;
	unsigned long long plugin_timer_sum = 0;
	int i = 0;
	struct mms_msg *msg = NULL;
	struct oplus_mms *err_topic;
	char temp_str[CHG_INTO_L_MAX] = {0};
	int rc;
	int index = 0;
	union mms_msg_data data = { 0 };
	int usb_temp_l, usb_temp_r;

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return;
	}

	chg_info("in oplus_chg_water_inlet_detect, reason = %d, plugin_count = %d\n", reason, plugin_count);

	switch (reason) {
	case TRACK_CMD_LIQUID_INTAKE:
		if (!chip->oplus_liquid_intake_enable) {
			chg_info("water inlet buried point bypass.\n");
			break;
		}
		/* Check whether this is the first entry */
		if (pre_timer == 0) {
			pre_timer = local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD;
			break;
		}

		/* Make sure you can make a difference */
		if (plugin_count > 0) {
			timer[(plugin_count - 1) % TIMER_SIZE].time =
				local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD - pre_timer;
			timer[(plugin_count - 1) % TIMER_SIZE].cid_status =
				oplus_wired_get_hw_detect();

			chg_info("plugin_count:%d, timer:%llu, pre_timer:%llu, timer[%d][0]:%llu, hw_detect:%d\n",
				plugin_count, local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD, pre_timer, (plugin_count - 1) % TIMER_SIZE,
				timer[(plugin_count - 1) % TIMER_SIZE].time,
				timer[(plugin_count - 1) % TIMER_SIZE].cid_status);
		}
		/* Store the time of the current removal */
		pre_timer = local_clock() / TRACK_LOCAL_T_NS_TO_MS_THD;
		/* Time of calculation */
		for (i = 0; i < chip->chg_into_liquid_cc_disconnect; i++) {
			if((timer[i].time == 0 || timer[i].time > chip->chg_into_liquid_max_interval_time) ||
			   timer[i].cid_status == CC_DETECT_NOTPLUG)
				goto end;
			plugin_timer_sum += timer[i].time;
		}
		/* Trigger burial point */
		if( plugin_timer_sum <= chip->chg_into_liquid_total_time ) {
			chip->oplus_liquid_intake_enable = false;
			oplus_wired_set_typec_mode(TYPEC_PORT_ROLE_SNK);
#ifdef CONFIG_OPLUS_CHARGER_MTK
			oplus_comm_publish_chg_err_otg_en_msg(chip, false);
#endif
			chip->oplus_liquid_intake_led_status = chip->led_on;
			chip->oplus_liquid_intake_check_led_status = true;

			chg_info("plugin_timer_sum:%llu, ", plugin_timer_sum);
			index += snprintf(&(temp_str[index]), CHG_INTO_L_MAX - index,
				"plugin_timer_sum:%llu, ", plugin_timer_sum);
			for (i = 0; i < TIMER_SIZE; i++) {
				chg_info("timer%d:%llu, ", i, timer[i].time);
				index += snprintf(&(temp_str[index]), CHG_INTO_L_MAX - index,
				"timer%d:%llu,%d ", i, timer[i].time, timer[i].cid_status);
			}
			index += snprintf(&(temp_str[index]), CHG_INTO_L_MAX - index,
				"bat_temp:%d, ", chip->batt_temp);
			index += snprintf(&(temp_str[index]), CHG_INTO_L_MAX - index,
				"vbus:%d, ", chip->wired_vbus_mv);
			if (chip->wired_topic != NULL) {
				oplus_mms_get_item_data(chip->wired_topic,
						WIRED_ITEM_USB_TEMP_L, &data, true);
				usb_temp_l = data.intval;
				oplus_mms_get_item_data(chip->wired_topic,
						WIRED_ITEM_USB_TEMP_R, &data, true);
				usb_temp_r = data.intval;
				index += snprintf(&(temp_str[index]), CHG_INTO_L_MAX - index,
					"usb_temp_l:%d, usb_temp_r:%d, ", usb_temp_l, usb_temp_r);
			}
			index += snprintf(&(temp_str[index]), CHG_INTO_L_MAX - index,
				"led_on:%d", chip->oplus_liquid_intake_led_status);

			msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_CHG_INTO_LIQUID,
					      temp_str);
			if (msg == NULL) {
				chg_err("alloc chg into liquid error msg error\n");
				return;
			}
			rc = oplus_mms_publish_msg(err_topic, msg);
			if (rc < 0) {
				chg_err("publish chg into liquid error msg error, rc=%d\n", rc);
				kfree(msg);
			}
			schedule_delayed_work(&chip->chg_into_liquid_trigger_work_timeout,
				msecs_to_jiffies(DEBUG_WATER_INLET_PARA));
			pre_timer = 0;
			memset(timer, '\0', sizeof(timer));
		}
end:
		break;
	case TRACK_CMD_CLEAR_TIMER:
		pre_timer = 0;
		memset(timer, '\0', sizeof(timer));
		break;
	default:
		chg_info("!!!err cmd\n");
		break;
	}
}

static void oplus_mms_water_inlet_detect_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_monitor *chip =
		container_of(dwork, struct oplus_monitor, water_inlet_detect_work);
	static int last_cc_state = -1;

	if (chip->liquid_inlet_detection_switch) {
		if(((chip->cc_state > 0) && (last_cc_state == 0)) ||
		   last_cc_state == -1) { /* plugin */
			chg_info("cc plugin!\n");
			chip->water_inlet_plugin_count += 1;
			cancel_delayed_work_sync(&chip->water_inlet_clear_work);
		} else if((chip->cc_state == 0) && (last_cc_state > 0)) { /* plugout */
			chg_info("cc plugout!\n");
			oplus_chg_water_inlet_detect(chip, TRACK_CMD_LIQUID_INTAKE, chip->water_inlet_plugin_count);
			schedule_delayed_work(&chip->water_inlet_clear_work,
				msecs_to_jiffies(OPLUS_CHG_IC_USB_PLUGOUT_DELAY));
		}
	}

	last_cc_state = chip->cc_state;
}

static void oplus_mms_water_inlet_clear_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_monitor *chip =
		container_of(dwork, struct oplus_monitor, water_inlet_clear_work);

	chip->water_inlet_plugin_count = 0;
	oplus_chg_water_inlet_detect(chip, TRACK_CMD_CLEAR_TIMER, 0);
	chg_err("water_inlet_detect_work plugout timeout!\n");
}

static void oplus_chg_track_chg_into_liquid_trigger_timeout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_monitor *chip = container_of(
		dwork, struct oplus_monitor, chg_into_liquid_trigger_work_timeout);

	chip->oplus_liquid_intake_enable = true;
}

static void oplus_chg_dischg_profile_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_monitor *chip =
		container_of(dwork, struct oplus_monitor, dischg_profile_update_work);

	oplus_chg_track_update_dischg_profile(chip);
}

static void oplus_liquid_intake_otg_mode_track(struct oplus_monitor *chip)
{
	char temp_str[CHG_INTO_L_MAX] = {0};
	struct mms_msg *msg = NULL;
	int rc;
	int index = 0;
	struct oplus_mms *err_topic;

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return;
	}
	index += snprintf(&(temp_str[index]), CHG_INTO_L_MAX - index,
			  "otg_enable:%d, led_status:[%d %d], set drp", chip->otg_enable,
	chip->oplus_liquid_intake_led_status, chip->led_on);
	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_CHG_INTO_LIQUID,
		      temp_str);
	if (msg == NULL) {
		chg_err("alloc chg into liquid error msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(err_topic, msg);
	if (rc < 0) {
		chg_err("publish chg into liquid error msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

#define DISCHG_PROFILE_INIT_UI 20
static void oplus_chg_check_dischg_profile(struct oplus_monitor *chip)
{
	union mms_msg_data data = { 0 };
	bool charging;

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data, true);
	chip->ui_soc = data.intval;
	if (!chip->deep_support)
		return;
	charging = chip->wired_online || chip->wls_online;


	if (!charging && (chip->ui_soc >= DISCHG_PROFILE_INIT_UI)) {
		oplus_chg_track_init_dischg_profile(chip);
		schedule_delayed_work(&chip->dischg_profile_update_work, 0);
 	} else {
		cancel_delayed_work(&chip->dischg_profile_update_work);
		oplus_chg_track_upload_dischg_profile(chip);
	}
}

static void oplus_chg_dischg_profile_check_work(struct work_struct *work)
{

	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_monitor *chip =
		container_of(dwork, struct oplus_monitor, dischg_profile_check_work);

	if (!chip->ui_soc_ready) {
		schedule_delayed_work(&chip->dischg_profile_check_work, msecs_to_jiffies(5000));
		return;
	}
	oplus_chg_check_dischg_profile(chip);
}

static void oplus_monitor_wired_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_monitor *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->wired_online = !!data.intval;
			chip->notify_flag = 0;
			if (!chip->wired_online)
				oplus_chg_track_record_dual_chan_end(chip);
			oplus_chg_track_update_break_ui_online();
			schedule_work(&chip->charge_info_update_work);
			schedule_work(&chip->wired_plugin_work);
			schedule_delayed_work(&chip->dischg_profile_check_work, 0);
			break;
		case WIRED_ITEM_PRESENT:
			schedule_work(&chip->wired_present_work);
			break;
		case WIRED_ITEM_ERR_CODE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->wired_err_code = (unsigned int)data.intval;
			break;
		case WIRED_ITEM_CHG_TYPE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->wired_charge_type = data.intval;
			oplus_chg_track_handle_wired_type_info(chip, TRACK_CHG_GET_THTS_TIME_TYPE);
			break;
		case WIRED_ITEM_CC_MODE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->cc_mode = data.intval;
			break;
		case WIRED_ITEM_CC_DETECT:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->cc_detect = data.intval;
			break;
		case WIRED_ITEM_OTG_ENABLE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->otg_enable = !!data.intval;
			break;
		case WIRED_TIME_ABNORMAL_ADAPTER:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->pd_svooc = !!data.intval;
			break;
		case WIRED_TIME_TYPEC_STATE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->cc_state = !!data.intval;
			schedule_delayed_work(&chip->water_inlet_detect_work, 0);
			break;
		case WIRED_ITEM_ONLINE_STATUS_ERR:
			oplus_chg_track_upload_wired_online_err_info(chip);
			break;
		case WIRED_ITEM_VBUS_VOL_TYPE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data, false);
			chip->vbus_vol_type = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_monitor_subscribe_wired_topic(struct oplus_mms *topic,
					    void *prv_data)
{
	struct oplus_monitor *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_monitor_wired_subs_callback, "monitor");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				true);
	chip->wired_online = !!data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ERR_CODE, &data,
				true);
	chip->wired_err_code = (unsigned int)data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE, &data,
				true);
	chip->wired_charge_type = data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CC_MODE, &data,
				true);
	chip->cc_mode = data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CC_DETECT, &data,
				true);
	chip->cc_detect = data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_OTG_ENABLE, &data,
				true);
	chip->otg_enable = !!data.intval;

	oplus_mms_get_item_data(chip->wired_topic, WIRED_TIME_ABNORMAL_ADAPTER, &data,
				true);
	chip->pd_svooc = !!data.intval;
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_VBUS_VOL_TYPE, &data, true);
	chip->vbus_vol_type = data.intval;
}

static void oplus_monitor_wls_subs_callback(struct mms_subscribe *subs,
					enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_monitor *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WLS_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wls_topic, id, &data, false);
			chip->wls_online = !!data.intval;
			schedule_work(&chip->charge_info_update_work);
			oplus_chg_track_check_wls_charging_break(!!data.intval);
			oplus_chg_track_check_wls_mul_break_stat(!!data.intval);
			schedule_delayed_work(&chip->dischg_profile_check_work, 0);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_monitor_subscribe_wls_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_monitor *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wls_topic = topic;
	chip->wls_subs = oplus_mms_subscribe(chip->wls_topic, chip, oplus_monitor_wls_subs_callback, "monitor");
	if (IS_ERR_OR_NULL(chip->wls_subs)) {
		chg_err("subscribe wls topic error, rc=%ld\n", PTR_ERR(chip->wls_subs));
		return;
	}
	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_PRESENT, &data, true);
	chip->wls_online = !!data.intval;
	if (chip->wls_online)
		schedule_work(&chip->charge_info_update_work);
}

static void oplus_monitor_record_ffc_soc(struct oplus_monitor *chip, bool start)
{
	union mms_msg_data data = { 0 };
	int main_soc = -1, sub_soc = -1;

	if (is_main_gauge_topic_available(chip)) {
		oplus_mms_get_item_data(chip->main_gauge_topic, GAUGE_ITEM_SOC,
					&data, false);
		main_soc = data.intval;
	}
	if (is_sub_gauge_topic_available(chip)) {
		oplus_mms_get_item_data(chip->sub_gauge_topic, GAUGE_ITEM_SOC,
					&data, false);
		sub_soc = data.intval;
	}
	oplus_chg_track_record_ffc_soc(chip, main_soc, sub_soc, start);
}

static void oplus_monitor_ffc_step_change_work(struct work_struct *work)
{
	struct oplus_monitor *chip =
		container_of(work, struct oplus_monitor, ffc_step_change_work);
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_FFC_STEP, &data, false);
	if (rc < 0) {
		chg_err("can't get ffc step, rc=%d\n", rc);
		return;
	}

	if (chip->ffc_status == FFC_FAST) {
		oplus_chg_track_aging_ffc_check(chip, data.intval);
		oplus_monitor_record_ffc_soc(chip, true);
	}
}

static void oplus_monitor_ffc_end_work(struct work_struct *work)
{
	struct oplus_monitor *chip =
		container_of(work, struct oplus_monitor, ffc_end_work);

	if (chip->wired_online || chip->wls_online) {
		oplus_chg_track_record_ffc_end_time(chip);
		oplus_monitor_record_ffc_soc(chip, false);
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FPGA_NOTIFY)
static void oplus_monitor_fgpa_reset_start_work(struct work_struct *work)
{
	struct oplus_monitor *chip = container_of(
		work, struct oplus_monitor, fgpa_reset_start_work);
	struct mms_msg *msg;
	int rc;

	chip->fpga_reset_status = FPGA_RESET_START;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  ERR_ITEM_FPGA_RESET);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_monitor_fgpa_reset_end_work(struct work_struct *work)
{
	struct oplus_monitor *chip = container_of(
		work, struct oplus_monitor, fgpa_reset_end_work);
	struct mms_msg *msg;
	int rc;

	chg_info(" %s \n", __func__);
	chip->fpga_reset_status = FPGA_RESET_END;
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  ERR_ITEM_FPGA_RESET);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg_sync(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_monitor_gauge_i2c_rst_work(struct work_struct *work)
{
	struct oplus_monitor *chip = container_of(
		work, struct oplus_monitor, gauge_i2c_reset_work);
	struct mms_msg *msg;
	int rc;

	chg_info("i2c_reset_status:%d\n", chip->i2c_reset_status);
	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  ERR_ITEM_I2C_RESET);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg_sync(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static int oplus_monitor_fpga_state_change(struct notifier_block *nb, unsigned long ev, void *v)
{
	struct oplus_monitor *chip = container_of(nb, struct oplus_monitor, nb);

	chg_err(" %s call, event is %lu.\n", __func__, ev);

	if (!chip) {
		chg_err("chip null\n ");
		return 0;
	}
	if (ev == FPGA_RESET_END) {
		cancel_work_sync(&chip->fgpa_reset_start_work);
		schedule_work(&chip->fgpa_reset_end_work);
	} else if (ev == FPGA_RESET_START) {
		cancel_work_sync(&chip->fgpa_reset_end_work);
		schedule_work(&chip->fgpa_reset_start_work);
	}
	return 0;
}

static int oplus_monitor_gauge_i2c_rst(struct notifier_block *nb, unsigned long ev, void *v)
{
	struct oplus_monitor *chip = container_of(nb, struct oplus_monitor, i2c_nb);

	chg_err(" %s call, event is %lu.\n", __func__, ev);

	if (!chip) {
		chg_err("chip null\n ");
		return 0;
	}
	if (ev == I2C_RST_END) {
		chip->i2c_reset_status = GUAGE_I2C_RST_END;
		schedule_work(&chip->gauge_i2c_reset_work);
	} else if (ev == I2C_RST_START) {
		chip->i2c_reset_status = GUAGE_I2C_RST_START;
		schedule_work(&chip->gauge_i2c_reset_work);
	}

	return 0;
}
#endif

static void oplus_monitor_comm_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_monitor *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int pre_batt_status = POWER_SUPPLY_STATUS_UNKNOWN;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_TEMP_REGION:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->temp_region = data.intval;
			oplus_chg_track_cal_tbatt_status(chip);
			break;
		case COMM_ITEM_FFC_STATUS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			if (chip->ffc_status == FFC_FAST && data.intval == FFC_DEFAULT)
				schedule_work(&chip->ffc_end_work);
			chip->ffc_status = (unsigned int)data.intval;
			if (chip->ffc_status == FFC_FAST)
				schedule_work(&chip->ffc_step_change_work);
			break;
		case COMM_ITEM_COOL_DOWN:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->cool_down = data.intval;
			break;
		case COMM_ITEM_UI_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->ui_soc = data.intval;
			if (chip->ui_soc < 0) {
				chg_err("ui soc not ready, rc=%d\n",
					chip->ui_soc);
				chip->ui_soc = 0;
				chip->ui_soc_ready = false;
			} else {
				chip->ui_soc_ready = true;
				if (chip->ui_soc == 1)
					oplus_chg_track_set_uisoc_1_start(chip);
			}
			break;
		case COMM_ITEM_SHUTDOWN_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->soc_load = data.intval;
			break;
		case COMM_ITEM_SMOOTH_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->smooth_soc = data.intval;
			break;
		case COMM_ITEM_NOTIFY_CODE:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->notify_code = (unsigned int)data.intval;
			break;
		case COMM_ITEM_NOTIFY_FLAG:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->notify_flag = (unsigned int)data.intval;
			break;
		case COMM_ITEM_SHELL_TEMP:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->shell_temp = data.intval;
			break;
		case COMM_ITEM_LED_ON:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->led_on = !!data.intval;

			if (chip->oplus_liquid_intake_led_status != chip->led_on &&
			    chip->oplus_liquid_intake_check_led_status &&
			    oplus_wired_get_hw_detect() != CC_DETECT_NOTPLUG) {
				oplus_wired_set_typec_mode(TYPEC_PORT_ROLE_TRY_SNK);
#ifdef CONFIG_OPLUS_CHARGER_MTK
				oplus_comm_publish_chg_err_otg_en_msg(chip, true);
#endif
				oplus_liquid_intake_otg_mode_track(chip);
				chip->oplus_liquid_intake_check_led_status = false;
			} else if (chip->liquid_inlet_detection_switch) {
				chg_info("intake_led_status:%d, led_on:%d, check_led_status:%d, hw_detect:%d\n",
					 chip->oplus_liquid_intake_led_status, chip->led_on,
					 chip->oplus_liquid_intake_check_led_status, oplus_wired_get_hw_detect());
			}
			break;
		case COMM_ITEM_BATT_STATUS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->batt_status = data.intval;
			if (chip->batt_status != pre_batt_status &&
			    chip->batt_status == POWER_SUPPLY_STATUS_FULL)
				oplus_chg_track_charge_full(chip);
			pre_batt_status = chip->batt_status;
			break;
		case COMM_ITEM_CHG_FULL:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->batt_full = !!data.intval;
			break;
		case COMM_ITEM_RECHGING:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->rechging = !!data.intval;
			if (chip->rechging && chip->rechg_soc_en)
				oplus_chg_track_upload_rechg_info(chip);
			break;
		case COMM_ITEM_FFC_STEP:
			schedule_work(&chip->ffc_step_change_work);
			break;
		case COMM_ITEM_CHG_CYCLE_STATUS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->chg_cycle_status = data.intval;
			break;
		case COMM_ITEM_SLOW_CHG:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->slow_chg_pct = SLOW_CHG_TO_PCT(data.intval);
			chip->slow_chg_watt = SLOW_CHG_TO_WATT(data.intval);
			chip->slow_chg_enable = !!SLOW_CHG_TO_ENABLE(data.intval);
			break;
		case COMM_ITEM_DELTA_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->delta_soc = data.intval;
			break;
		case COMM_ITEM_SUPER_ENDURANCE_STATUS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->sem_info.status = !!data.intval;
			if (!chip->sem_info.uisoc_0)
				oplus_chg_track_super_endurance_mode_change(chip);
			break;
		case COMM_ITEM_SUPER_ENDURANCE_COUNT:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->sem_info.count = data.intval;
			break;
		case COMM_ITEM_UISOC_KEEP_2_ERROR:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->uisoc_keep_2_err = data.intval;
			if (chip->uisoc_keep_2_err)
				oplus_chg_track_upload_uisoc_keep_2_err_info(chip);
			break;
		case COMM_ITEM_RECHG_SOC_EN_STATUS:
			oplus_mms_get_item_data(chip->comm_topic, id, &data, false);
			chip->rechg_soc_en = RECHG_SOC_TO_ENABLE(data.intval);
			chip->rechg_soc_threshold =RECHG_SOC_TO_SOC(data.intval);
			oplus_chg_track_upload_rechg_info(chip);
			break;
		case COMM_ITEM_BOOT_COMPLETED:
			oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_BOOT_COMPLETED, &data, false);
			if (!!data.intval)
				oplus_chg_track_update_prechg_r_data(chip);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_monitor_subscribe_comm_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_monitor *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->comm_topic = topic;
	chip->comm_subs =
		oplus_mms_subscribe(chip->comm_topic, chip,
				    oplus_monitor_comm_subs_callback, "monitor");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe common topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_TEMP_REGION, &data,
				true);
	chip->temp_region = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_FFC_STATUS, &data,
				true);
	chip->ffc_status = (unsigned int)data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_COOL_DOWN, &data,
				true);
	chip->cool_down = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data,
				true);
	chip->ui_soc = data.intval;
	if (chip->ui_soc < 0) {
		chg_err("ui soc not ready, rc=%d\n", chip->ui_soc);
		chip->ui_soc = 0;
		chip->ui_soc_ready = false;
	} else {
		chip->ui_soc_ready = true;
		if (chip->ui_soc == 1)
			oplus_chg_track_set_uisoc_1_start(chip);
	}
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SMOOTH_SOC, &data,
				true);
	chip->smooth_soc = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_NOTIFY_CODE, &data,
				true);
	chip->notify_code = (unsigned int)data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_NOTIFY_FLAG, &data,
				true);
	chip->notify_flag = (unsigned int)data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SHELL_TEMP, &data,
				true);
	chip->shell_temp = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_LED_ON, &data,
				true);
	chip->led_on = !!data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_RECHGING, &data,
				true);
	chip->rechging = !!data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_CHG_CYCLE_STATUS, &data,
				true);
	chip->chg_cycle_status = data.intval;

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SLOW_CHG, &data, true);
	chip->slow_chg_pct = SLOW_CHG_TO_PCT(data.intval);
	chip->slow_chg_watt = SLOW_CHG_TO_WATT(data.intval);
	chip->slow_chg_enable = !!SLOW_CHG_TO_ENABLE(data.intval);

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_RECHG_SOC_EN_STATUS, &data, false);
	chip->rechg_soc_en = RECHG_SOC_TO_ENABLE(data.intval);
	chip->rechg_soc_threshold = RECHG_SOC_TO_SOC(data.intval);

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_BOOT_COMPLETED, &data, true);
	if (!!data.intval)
		oplus_chg_track_update_prechg_r_data(chip);

	schedule_delayed_work(&chip->dischg_profile_check_work, 0);
}

static void oplus_monitor_vooc_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_monitor *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case VOOC_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_online = !!data.intval;
			if (chip->vooc_online)
				chip->pre_vooc_sid = 0;
			break;
		case VOOC_ITEM_VOOC_STARTED:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_started = !!data.intval;
			break;
		case VOOC_ITEM_VOOC_CHARGING:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_charging = !!data.intval;
			break;
		case VOOC_ITEM_SID:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_sid = (unsigned int)data.intval;
			if (chip->vooc_sid != 0)
				chip->pre_vooc_sid = chip->vooc_sid;
			break;
		case VOOC_ITEM_ONLINE_KEEP:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_online_keep = !!data.intval;
			break;
		case VOOC_ITEM_BREAK_CODE:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			oplus_chg_track_set_fastchg_break_code(data.intval);
			break;
		case VOOC_ITEM_VOOC_BY_NORMAL_PATH:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			if (!!data.intval)
				chip->chg_ctrl_by_vooc = true;
			break;
		case VOOC_ITEM_NORMAL_CONNECT_COUNT_LEVEL:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			chip->vooc_normal_connect_count_level = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_monitor_subscribe_vooc_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct oplus_monitor *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->vooc_topic = topic;
	chip->vooc_subs =
		oplus_mms_subscribe(chip->vooc_topic, chip,
				    oplus_monitor_vooc_subs_callback, "monitor");
	if (IS_ERR_OR_NULL(chip->vooc_subs)) {
		chg_err("subscribe vooc topic error, rc=%ld\n",
			PTR_ERR(chip->vooc_subs));
		return;
	}

	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_ONLINE, &data,
				true);
	chip->vooc_online = !!data.intval;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_STARTED, &data,
				true);
	chip->vooc_started = !!data.intval;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_CHARGING,
				&data, true);
	chip->vooc_charging = !!data.intval;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_SID, &data, true);
	chip->vooc_sid = (unsigned int)data.intval;
	chip->pre_vooc_sid = chip->vooc_sid;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_ONLINE_KEEP, &data,
				true);
	chip->vooc_online_keep = !!data.intval;
	oplus_mms_get_item_data(chip->vooc_topic, VOOC_ITEM_VOOC_BY_NORMAL_PATH,
				&data, true);
	if (!!data.intval)
		chip->chg_ctrl_by_vooc = true;
}

static int oplus_monitor_update_fpga_reset(struct oplus_mms *mms,
				    union mms_msg_data *data)
{
	struct oplus_monitor *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->fpga_reset_status;

	return 0;
}

static int oplus_monitor_update_i2c_reset(struct oplus_mms *mms,
				union mms_msg_data *data)
{
	struct oplus_monitor *chip;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return -EINVAL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(mms);

	data->intval = chip->i2c_reset_status;

	return 0;
}

static void oplus_monitor_update(struct oplus_mms *mms, bool publish)
{
	/* TODO Active exception check */
}

static struct mms_item oplus_monitor_item[] = {
	{
		.desc = {
			.item_id = ERR_ITEM_IC,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_USBTEMP,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_VBAT_TOO_LOW,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = COMM_ITEM_UISOC_DROP_ERROR,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_VBAT_DIFF_OVER,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_UI_SOC_SHUTDOWN,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_DUAL_CHAN,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_MMI_CHG,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_SLOW_CHG,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_CHG_CYCLE,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_WLS_INFO,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_UFCS,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_DEEP_DISCHG_INFO,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_SUB_DEEP_DISCHG_INFO,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_CHG_INTO_LIQUID,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_DEEP_DISCHG_PROFILE,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_EIS_TIMEOUT,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_BAL_INFO,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_FPGA_RESET,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_monitor_update_fpga_reset,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_I2C_RESET,
			.str_data = false,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = oplus_monitor_update_i2c_reset,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_CLOSE_CP,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_BIDIRECT_CP_INFO,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_PLC_INFO,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_DEC_CV_INFO,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_LPD,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_DEVICE_ID,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_ERR_PHY_CP_INFO,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_PPS,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_GAUGE_R_INFO,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_SOCCP_CRASH,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_FCL_INFO,
			.str_data = true,
			.up_thr_enable = false,
			.down_thr_enable = false,
			.dead_thr_enable = false,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_BS_INFO,
			.str_data = true,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_STATE_KEEP_INFO,
			.str_data = true,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_STATE_KEEP_ABNORMAL,
			.str_data = true,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_USBIN_ABNORMAL,
		}
	},
	{
		.desc = {
			.item_id = ERR_ITEM_SEC_IC_MEM_INFO,
			.str_data = true,
		}
	}
};

static const struct oplus_mms_desc oplus_monitor_desc = {
	.name = "error",
	.type = OPLUS_MMS_TYPE_ERROR,
	.item_table = oplus_monitor_item,
	.item_num = ARRAY_SIZE(oplus_monitor_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 60000, /* 1 min */
	.update = oplus_monitor_update,
};

static int oplus_monitor_topic_init(struct oplus_monitor *chip)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;

	mms_cfg.drv_data = chip;
	mms_cfg.of_node = chip->dev->of_node;

	if (of_property_read_bool(mms_cfg.of_node,
				  "oplus,topic-update-interval")) {
		rc = of_property_read_u32(mms_cfg.of_node,
					  "oplus,topic-update-interval",
					  &mms_cfg.update_interval);
		if (rc < 0)
			mms_cfg.update_interval = 0;
	}

	chip->liquid_inlet_detection_switch = of_property_read_bool(chip->dev->of_node, "oplus,chg_into_liquid");

	rc = of_property_read_u32(mms_cfg.of_node, "oplus,chg_into_liquid_cc_disconnect", &chip->chg_into_liquid_cc_disconnect);
	if (rc < 0) {
		chip->chg_into_liquid_cc_disconnect = 10;
		chg_info("Not find oplus,chg_into_liquid_cc_disconnect err, init 10!\n");
	}

	rc = of_property_read_u32(mms_cfg.of_node, "oplus,chg_into_liquid_total_time", &chip->chg_into_liquid_total_time);
	if (rc < 0) {
		chip->chg_into_liquid_total_time = 5000;
		chg_info("Not fond oplus,chg_into_liquid_total_time err, init 5000ms!\n");
	}

	rc = of_property_read_u32(mms_cfg.of_node, "oplus,chg_into_liquid_max_interval_time", &chip->chg_into_liquid_max_interval_time);
	if (rc < 0) {
		chip->chg_into_liquid_max_interval_time = 700;
		chg_info("Not fond oplus,chg_into_liquid_max_interval_time err, init 700ms!\n");
	}
	chg_info("liquid_inlet_detection_switch = %d, cc_disconnect = %d, total_time = %d, max_interval_time = %d\n",
		 chip->liquid_inlet_detection_switch, chip->chg_into_liquid_cc_disconnect,
		 chip->chg_into_liquid_total_time, chip->chg_into_liquid_max_interval_time);

	chip->water_inlet_plugin_count = 0;
	chip->oplus_liquid_intake_enable = true;

	chip->err_topic = devm_oplus_mms_register(chip->dev, &oplus_monitor_desc, &mms_cfg);
	if (IS_ERR(chip->err_topic)) {
		chg_err("Couldn't register error topic\n");
		rc = PTR_ERR(chip->err_topic);
		return rc;
	}

	oplus_mms_stop_publish(chip->err_topic);

	return 0;
}

static int oplus_monitor_probe(struct platform_device *pdev)
{
	struct oplus_monitor *chip;
	int rc;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_monitor),
			    GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc memory error\n");
		return -ENOMEM;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	of_platform_populate(chip->dev->of_node, NULL, NULL, chip->dev);

	rc = oplus_monitor_topic_init(chip);
	if (rc < 0)
		goto topic_init_err;
	rc = oplus_chg_track_driver_init(chip);
	if (rc < 0) {
		chg_err("track driver init error, rc=%d\n", rc);
		goto track_init_err;
	}
	battlog_comm_ops.dev_data = (void *)chip;
	battery_log_ops_register(&battlog_comm_ops);

	INIT_WORK(&chip->charge_info_update_work,
		  oplus_monitor_charge_info_update_work);
	INIT_WORK(&chip->wired_plugin_work, oplus_monitor_wired_plugin_work);
	INIT_WORK(&chip->wired_present_work, oplus_monitor_wired_present_work);
	INIT_WORK(&chip->ffc_step_change_work, oplus_monitor_ffc_step_change_work);
	INIT_WORK(&chip->ffc_end_work, oplus_monitor_ffc_end_work);
	INIT_DELAYED_WORK(&chip->water_inlet_detect_work, oplus_mms_water_inlet_detect_work);
	INIT_DELAYED_WORK(&chip->water_inlet_clear_work, oplus_mms_water_inlet_clear_work);
	INIT_DELAYED_WORK(&chip->chg_into_liquid_trigger_work_timeout,
			  oplus_chg_track_chg_into_liquid_trigger_timeout_work);
	INIT_DELAYED_WORK(&chip->dischg_profile_update_work, oplus_chg_dischg_profile_update_work);
	INIT_DELAYED_WORK(&chip->dischg_profile_check_work, oplus_chg_dischg_profile_check_work);
#if IS_ENABLED(CONFIG_OPLUS_FPGA_NOTIFY)
	INIT_WORK(&chip->fgpa_reset_start_work, oplus_monitor_fgpa_reset_start_work);
	INIT_WORK(&chip->fgpa_reset_end_work, oplus_monitor_fgpa_reset_end_work);
	INIT_WORK(&chip->gauge_i2c_reset_work, oplus_monitor_gauge_i2c_rst_work);
	chip->nb.notifier_call = oplus_monitor_fpga_state_change;
	rc = fpga_register_notifier(&chip->nb);
	if (rc != 0)
		chg_err("%s : fpga_register_notifier failed!\n", __func__);
	chip->i2c_nb.notifier_call = oplus_monitor_gauge_i2c_rst;
	rc = i2c_rst_register_notifier(&chip->i2c_nb);
	if (rc != 0)
		chg_err("%s : i2c_rst_register_notifier failed!\n", __func__);
#endif
	oplus_mms_wait_topic("wired", oplus_monitor_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("wireless", oplus_monitor_subscribe_wls_topic, chip);
	oplus_mms_wait_topic("vooc", oplus_monitor_subscribe_vooc_topic, chip);
	oplus_mms_wait_topic("common", oplus_monitor_subscribe_comm_topic, chip);
	oplus_mms_wait_topic("dual_chan", oplus_monitor_subscribe_dual_chan_topic, chip);
	oplus_mms_wait_topic("gauge", oplus_monitor_subscribe_gauge_topic, chip);
	oplus_mms_wait_topic("ufcs", oplus_monitor_subscribe_ufcs_topic, chip);
	oplus_mms_wait_topic("retention", oplus_monitor_subscribe_retention_topic, chip);
	oplus_mms_wait_topic("plc", oplus_monitor_subscribe_plc_topic, chip);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	oplus_mms_wait_topic("state_keep", oplus_monitor_subscribe_keep_topic, chip);
#endif

	chg_info("probe success\n");
	return 0;

track_init_err:
topic_init_err:
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, chip);

	chg_info("probe error, rc=%d\n", rc);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_monitor_remove(struct platform_device *pdev)
#else
static int oplus_monitor_remove(struct platform_device *pdev)
#endif
{
	struct oplus_monitor *chip = platform_get_drvdata(pdev);

	if (!IS_ERR_OR_NULL(chip->gauge_subs))
		oplus_mms_unsubscribe(chip->gauge_subs);
	if (!IS_ERR_OR_NULL(chip->comm_subs))
		oplus_mms_unsubscribe(chip->comm_subs);
	if (!IS_ERR_OR_NULL(chip->vooc_subs))
		oplus_mms_unsubscribe(chip->vooc_subs);
	if (!IS_ERR_OR_NULL(chip->wls_subs))
		oplus_mms_unsubscribe(chip->wls_subs);
	if (!IS_ERR_OR_NULL(chip->wired_subs))
		oplus_mms_unsubscribe(chip->wired_subs);
	if (!IS_ERR_OR_NULL(chip->ufcs_subs))
		oplus_mms_unsubscribe(chip->ufcs_subs);
	if (!IS_ERR_OR_NULL(chip->plc_subs))
		oplus_mms_unsubscribe(chip->plc_subs);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	if (!IS_ERR_OR_NULL(chip->keep_subs))
		oplus_mms_unsubscribe(chip->keep_subs);
#endif
	oplus_chg_track_driver_exit(chip);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, chip);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static void oplus_monitor_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id oplus_monitor_match[] = {
	{ .compatible = "oplus,monitor" },
	{},
};

static struct platform_driver oplus_monitor_driver = {
	.driver		= {
		.name = "oplus-monitor",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_monitor_match),
	},
	.probe		= oplus_monitor_probe,
	.remove		= oplus_monitor_remove,
	.shutdown	= oplus_monitor_shutdown,
};

static __init int oplus_monitor_init(void)
{
	return platform_driver_register(&oplus_monitor_driver);
}

static __exit void oplus_monitor_exit(void)
{
	platform_driver_unregister(&oplus_monitor_driver);
}

oplus_chg_module_late_register(oplus_monitor);
