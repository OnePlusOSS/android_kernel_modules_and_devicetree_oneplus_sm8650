// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[MTK6895]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>
#include <linux/mfd/mt6397/core.h>/* PMIC MFD core header */
#include <linux/regmap.h>
#include <linux/of_platform.h>
#include <linux/rtc.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
#include <linux/vmalloc.h>
#endif

#include <asm/setup.h>

#include "oplus_hal_mtk6895S.h"

#ifdef OPLUS_FEATURE_CHG_BASIC
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/iio/consumer.h>
#include <mtk_battery.h>

#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_voter.h>
#include <tcpm.h>
#include <linux/thermal.h>
#ifdef OPLUS_FEATURE_CHG_BASIC
#include <linux/usb/typec.h>
#include <oplus_chg_pps.h>
#if IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
#include <tcpci.h>
#include <tcpci_typec.h>
#include "pd_policy_engine.h"
#else
#include "../../../misc/mediatek/typec/tcpc/inc/tcpci.h"
#include "../../../misc/mediatek/typec/tcpc/inc/tcpci_typec.h"
#include "../../../misc/mediatek/typec/tcpc/inc/pd_policy_engine.h"
#endif
#include <oplus_chg_wls.h>
#include <oplus_chg_monitor.h>
#include <oplus_chg_cpa.h>

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/system/oplus_project.h>
#endif

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
#include <soc/oplus/horae_flash_led_temp.h>
#endif

static int oplus_chg_set_pps_config(struct oplus_chg_ic_dev *ic_dev, int vbus_mv, int ibus_ma);
static int oplus_chg_set_fixed_pd_config(struct oplus_chg_ic_dev *ic_dev, int vol_mv, int curr_ma);
static int oplus_pps_get_authentiate(struct oplus_chg_ic_dev *ic_dev);
static int oplus_get_pps_info(struct oplus_chg_ic_dev *ic_dev, u32 *pdo, int num);
static int oplus_chg_get_pps_status(struct oplus_chg_ic_dev *ic_dev, u32 *src_info);
static int mtk_chg_get_charger_type(struct oplus_chg_ic_dev *ic_dev, int *type);
static int mtk_chg_input_present(struct oplus_chg_ic_dev *ic_dev, bool *present);
static int oplus_get_master_cp_temp(void);
static int oplus_chg_get_battery_btb_temp_cal(void);
static unsigned int adc_vin_reg;
#endif /* OPLUS_FEATURE_CHG_BASIC */

#define POWER_SUPPLY_TYPE_USB_HVDCP 13
#define OPLUS_CHG_PULL_UP_VOLT_THR  7500

#define AICL_POINT_VOL_9V           7600
#define AICL_POINT_SWITCH_THRE		7500
#define DUAL_BAT_AICL_POINT_VOL_9V  8500
#define AICL_POINT_VOL_5V           4140
#define HW_AICL_POINT_VOL_5V_PHASE1 4440
#define HW_AICL_POINT_VOL_6V_PHASE1 6000
#define HW_AICL_POINT_VOL_5V_PHASE2 4520
#define SW_AICL_POINT_VOL_5V_PHASE1 4500
#define SW_AICL_POINT_VOL_5V_PHASE2 4535
#define USB_HW_AICL_POINT           4600
#define USB_SW_AICL_POINT           4620

#ifdef OPLUS_FEATURE_CHG_BASIC
#define OPLUS_SVID 		    0x22D9
#define OPLUS_UVDM_AUTH_CMD	    0x1F
#define OPLUS_UVDM_POWER_CMD	    0x1
#define OPLUS_UVDM_EXAPDO_CMD	    0x2

#define PPS_RANDOM_NUMBER	    4
#define NO_OF_DATA_OBJECTS_MAX      7
#define PPS_KEY_COUNT		    4
#define LOW_BATT_SOC 1

static uint32_t pps_random[PPS_RANDOM_NUMBER] = { 1111, 2222, 3333, 4444 };
static uint32_t pps_adapter_result[NO_OF_DATA_OBJECTS_MAX] = { 0 };
static uint32_t pps_phone_result[NO_OF_DATA_OBJECTS_MAX] = { 0 };

static uint32_t tea_delta = 0;
static uint32_t publick_key[4] = { 0 };
static uint32_t private_key1[4] = { 0 };
static uint32_t private_key2[4] = { 0 };

#define VBUS_9V	9000
#define VBUS_5V	5000
#define IBUS_2A	2000
#define IBUS_3A	3000

#endif /* OPLUS_FEATURE_CHG_BASIC */

static struct mtk_charger *pinfo;

static int oplus_get_chargeric_temp(void);
static int oplus_get_subboard_temp(void);

extern int mt6375_get_hvdcp_type(void);
extern void mt6375_enable_hvdcp_detect(void);
extern int mt6375_set_hvdcp_to_5v(void);
extern int mt6375_set_hvdcp_to_9v(void);
extern void oplus_notify_hvdcp_detect_stat(void);
extern void oplus_set_hvdcp_flag_clear(void);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
extern int oplus_chg_set_shipping_mode(void);
#endif
extern bool mt6375_int_chrdet_attach(void);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
#undef PDE_DATA
#define PDE_DATA(inode) pde_data(inode)
#endif

static int oplus_chg_suspend_charger(bool suspend, const char *client_str)
{
	struct votable *suspend_votable;
	int rc;

	suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	if (!suspend_votable) {
		chg_err("WIRED_CHARGE_SUSPEND votable not found\n");
		return -EINVAL;
	}

	rc = vote(suspend_votable, client_str, suspend, 1, false);
	if (rc < 0)
		chg_err("%s charger error, rc = %d\n",
		        suspend ? "suspend" : "unsuspend", rc);
	else
		chg_info("%s charger\n", suspend ? "suspend" : "unsuspend");

	return rc;
}

static int oplus_chg_set_icl_by_vote(int icl, const char *client_str)
{
	struct votable *icl_votable;
	int rc;

	icl_votable = find_votable("WIRED_ICL");
	if (!icl_votable) {
		chg_err("WIRED_ICL votable not found\n");
		return -EINVAL;
	}

	rc = vote(icl_votable, client_str, true, icl, true);
	if (rc < 0)
		chg_err("set icl error: icl = %d, rc = %d\n", icl, rc);
	else
		chg_info("real icl = %d\n", icl);

	return rc;
}

static bool is_gauge_topic_available(struct mtk_charger *chip)
{
	if (!chip->gauge_topic)
		chip->gauge_topic = oplus_mms_get_by_name("gauge");

	return !!chip->gauge_topic;
}

/* TODO V2 */
int oplus_chg_wake_update_work(void)
{
	return 0;
}
EXPORT_SYMBOL(oplus_chg_wake_update_work);

bool oplus_chg_check_chip_is_null(void)
{
	return true;
}
EXPORT_SYMBOL(oplus_chg_check_chip_is_null);

int oplus_chg_check_ui_soc(void)
{
	return true;
}
EXPORT_SYMBOL(oplus_chg_check_ui_soc);

/* TODO V2 */
bool oplus_get_hmac(void)
{
	return true;
}
EXPORT_SYMBOL(oplus_get_hmac);

int oplus_is_vooc_project(void)
{
	int vooc_project = 0;
	struct oplus_mms *vooc_topic;

	vooc_topic = oplus_mms_get_by_name("vooc");
	if (!vooc_topic)
		return 0;

	vooc_project = oplus_vooc_get_project(vooc_topic);

	chr_info("get vooc project = %d\n", vooc_project);
	return vooc_project;
}
EXPORT_SYMBOL(oplus_is_vooc_project);

int oplus_chg_show_vooc_logo_ornot(void)
{
	int vooc_online_true = 0;
	int online = 0;
	int online_keep = 0;
	struct oplus_mms *vooc_topic;
	union mms_msg_data data = { 0 };
	int rc;

	vooc_topic = oplus_mms_get_by_name("vooc");
	if (!vooc_topic)
		return 0;

	rc = oplus_mms_get_item_data(vooc_topic, VOOC_ITEM_ONLINE, &data, false);
	if (!rc)
		online = data.intval;

	rc = oplus_mms_get_item_data(vooc_topic, VOOC_ITEM_ONLINE_KEEP, &data, false);
	if (!rc)
		online_keep = data.intval;

	vooc_online_true = online || online_keep;

	chr_info("get vooc online = %d\n", vooc_online_true);
	return vooc_online_true;
}
EXPORT_SYMBOL(oplus_chg_show_vooc_logo_ornot);

int oplus_get_prop_status(void)
{
	int prop_status = 0;
	struct oplus_mms *comm_topic;
	union mms_msg_data data = { 0 };
	int rc;

	comm_topic = oplus_mms_get_by_name("common");
	if (!comm_topic)
		return 0;

	rc = oplus_mms_get_item_data(comm_topic, COMM_ITEM_BATT_STATUS, &data, true);
	if (!rc)
		prop_status = data.intval;

	chr_info("get prop_status = %d\n", prop_status);

	return prop_status;
}
EXPORT_SYMBOL(oplus_get_prop_status);

bool oplus_mt_get_vbus_status(void)
{
	struct mtk_charger *info = pinfo;

	if (!info || !pinfo->tcpc) {
		pr_err("pinfo not found\n");
		return -ENODEV;
	}

	if (tcpm_inquire_typec_attach_state(pinfo->tcpc) == TYPEC_ATTACHED_SRC)
		return false;

	if (pinfo->chrdet_state)
		return true;
	else if (mt6375_int_chrdet_attach() == true)
		return true;
	else
		return false;
}
EXPORT_SYMBOL(oplus_mt_get_vbus_status);

int oplus_chg_get_notify_flag(void)
{
	int notify_flag = 0;
	struct oplus_mms *comm_topic;
	union mms_msg_data data = { 0 };
	int rc;

	comm_topic = oplus_mms_get_by_name("common");
	if (!comm_topic)
		return 0;

	rc = oplus_mms_get_item_data(comm_topic, COMM_ITEM_NOTIFY_FLAG, &data, false);
	if (!rc)
		notify_flag = data.intval;

	chr_info("get notify code is %d\n", notify_flag);
	return notify_flag;
}
EXPORT_SYMBOL(oplus_chg_get_notify_flag);

int oplus_chg_get_ui_soc(void)
{
	int ui_soc = 0;
	struct oplus_mms *comm_topic;
	union mms_msg_data data = { 0 };
	int rc;

	comm_topic = oplus_mms_get_by_name("common");
	if (!comm_topic)
		return 0;

	rc = oplus_mms_get_item_data(comm_topic, COMM_ITEM_UI_SOC, &data, false);
	if (!rc) {
		ui_soc = data.intval;
		if (ui_soc < 0) {
			chg_err("ui soc not ready, rc=%d\n", ui_soc);
			ui_soc = 0;
		}
	}

	chr_info("get ui soc = %d\n", ui_soc);
	return ui_soc;
}
EXPORT_SYMBOL(oplus_chg_get_ui_soc);

int oplus_chg_get_voocphy_support(void)
{
	struct oplus_mms *vooc_topic;

	vooc_topic = oplus_mms_get_by_name("vooc");
	if (!vooc_topic) {
		chg_err("vooc topic not found");
		return 0;
	}

	return oplus_vooc_get_voocphy_support(vooc_topic);
}
EXPORT_SYMBOL(oplus_chg_get_voocphy_support);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
struct oplus_gauge_chip {
	int gauge_unused;
};

struct oplus_gauge_chip *g_gauge_chip = NULL;

void oplus_gauge_init(struct oplus_gauge_chip *chip)
{
	g_gauge_chip = chip;

	return;
}
EXPORT_SYMBOL(oplus_gauge_init);
#else
extern struct oplus_gauge_chip *g_gauge_chip;

void oplus_gauge_init(struct oplus_gauge_chip *chip)
{
	g_gauge_chip = chip;

	return;
}
EXPORT_SYMBOL(oplus_gauge_init);
#endif
/*end TODO V2*/

int get_uisoc(struct mtk_charger *info)
{
	union power_supply_propval prop;
	struct power_supply *bat_psy = NULL;
	int ret;

	bat_psy = info->bat_psy;

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s retry to get bat_psy\n", __func__);
		bat_psy = devm_power_supply_get_by_phandle(&info->pdev->dev, "gauge");
		info->bat_psy = bat_psy;
	}

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s Couldn't get bat_psy\n", __func__);
		ret = 50;
	} else {
		ret = power_supply_get_property(bat_psy,
			POWER_SUPPLY_PROP_CAPACITY, &prop);
		ret = prop.intval;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int oplus_chg_get_batt_soc(void)
{
	int batt_soc = 50; /* default batt_soc set 50 */
	struct oplus_mms *gauge_topic;
	union mms_msg_data data = { 0 };
	int rc;

	gauge_topic = oplus_mms_get_by_name("gauge");
	if (!gauge_topic)
		return batt_soc;

	rc = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_SOC, &data, true);
	if (!rc) {
		batt_soc = data.intval;
		if (batt_soc < 0) {
			chg_err("batt soc not ready, batt_soc=%d\n", batt_soc);
			batt_soc = 50;
		}
	}

	chr_info("get batt soc = %d\n", batt_soc);
	return batt_soc;
}

int get_battery_voltage(struct mtk_charger *info)
{
	union power_supply_propval prop;
	struct power_supply *bat_psy = NULL;
	int ret;

	bat_psy = info->bat_psy;

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s retry to get bat_psy\n", __func__);
		bat_psy = devm_power_supply_get_by_phandle(&info->pdev->dev, "gauge");
		info->bat_psy = bat_psy;
	}

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s Couldn't get bat_psy\n", __func__);
		ret = 3999;
	} else {
		ret = power_supply_get_property(bat_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		ret = prop.intval / 1000;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_battery_temperature(struct mtk_charger *info)
{
	union power_supply_propval prop;
	struct power_supply *bat_psy = NULL;
	int ret = 0;
	int tmp_ret = 0;

	bat_psy = info->bat_psy;

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s retry to get bat_psy\n", __func__);
		bat_psy = devm_power_supply_get_by_phandle(&info->pdev->dev, "gauge");
		info->bat_psy = bat_psy;
	}

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s Couldn't get bat_psy\n", __func__);
		ret = 27;
	} else {
		tmp_ret = power_supply_get_property(bat_psy,
			POWER_SUPPLY_PROP_TEMP, &prop);
		ret = prop.intval / 10;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_battery_current(struct mtk_charger *info)
{
	union power_supply_propval prop;
	struct power_supply *bat_psy = NULL;
	int ret = 0;
	int tmp_ret = 0;

	bat_psy = info->bat_psy;

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s retry to get bat_psy\n", __func__);
		bat_psy = devm_power_supply_get_by_phandle(&info->pdev->dev, "gauge");
		info->bat_psy = bat_psy;
	}

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s Couldn't get bat_psy\n", __func__);
		ret = 0;
	} else {
		tmp_ret = power_supply_get_property(bat_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
		ret = prop.intval / 1000;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

static int get_pmic_vbus(struct mtk_charger *info, int *vchr)
{
	union power_supply_propval prop = { 0 };
	static struct power_supply *chg_psy;
	int ret;

	if (chg_psy == NULL)
		chg_psy = power_supply_get_by_name("mtk_charger_type");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		ret = -1;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	}
	*vchr = prop.intval;

	chr_debug("%s vbus:%d\n", __func__,
		prop.intval);
	return ret;
}

int get_vbus(struct mtk_charger *info)
{
	int ret = 0;
	int vchr = 0;

	if (info == NULL)
		return 0;
	ret = charger_dev_get_vbus(info->chg1_dev, &vchr);
	if (ret < 0) {
		ret = get_pmic_vbus(info, &vchr);
		if (ret < 0)
			chr_err("%s: get vbus failed: %d\n", __func__, ret);
	} else
		vchr /= 1000;

	return vchr;
}

#define R_CHARGER_1 330
#define R_CHARGER_2 39

int battery_meter_get_charger_voltage(void)
{
	int vbus_mv = 0;
	int ret;

	if (!pinfo)
		return 0;

	if (!IS_ERR_OR_NULL(pinfo->vbus_chan)) {
		ret = iio_read_channel_processed(pinfo->vbus_chan, &vbus_mv);
		if (ret < 0) {
			chg_err("failed to read vbus ,ret=%d\n", ret);
			return vbus_mv;
		}

		vbus_mv = (((R_CHARGER_1 + R_CHARGER_2) * 100 * (vbus_mv)) / R_CHARGER_2) / 100;
		return vbus_mv;
	} else {
		return get_vbus(pinfo);
	}
}

int get_ibat(struct mtk_charger *info)
{
	int ret = 0;
	int ibat = 0;

	if (info == NULL)
		return -EINVAL;
	ret = charger_dev_get_ibat(info->chg1_dev, &ibat);
	if (ret < 0)
		chr_err("%s: get ibat failed: %d\n", __func__, ret);

	return ibat / 1000;
}

int get_ibus(struct mtk_charger *info)
{
	int ret = 0;
	int ibus = 0;

	if (info == NULL)
		return -EINVAL;
	ret = charger_dev_get_ibus(info->chg1_dev, &ibus);
	if (ret < 0)
		chr_err("%s: get ibus failed: %d\n", __func__, ret);

	return ibus / 1000;
}

bool is_battery_exist(struct mtk_charger *info)
{
	union power_supply_propval prop;
	struct power_supply *bat_psy = NULL;
	int ret = 0;
	int tmp_ret = 0;

	bat_psy = info->bat_psy;

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s retry to get bat_psy\n", __func__);
		bat_psy = devm_power_supply_get_by_phandle(&info->pdev->dev, "gauge");
		info->bat_psy = bat_psy;
	}

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s Couldn't get bat_psy\n", __func__);
		ret = 1;
	} else {
		tmp_ret = power_supply_get_property(bat_psy,
			POWER_SUPPLY_PROP_PRESENT, &prop);
		ret = prop.intval;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

bool is_charger_exist(struct mtk_charger *info)
{
	union power_supply_propval prop;
	static struct power_supply *chg_psy;
	int ret;
	int tmp_ret = 0;

	chg_psy = info->chg_psy;

	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s retry to get chg_psy\n", __func__);
		chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev, "charger");
		info->chg_psy = chg_psy;
	}

	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chg_err("%s Couldn't get chg_psy\n", __func__);
		ret = -1;
	} else {
		tmp_ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		ret = prop.intval;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_charger_type(struct mtk_charger *info)
{
	union power_supply_propval prop = { 0 };
	union power_supply_propval prop2 = { 0 };
	union power_supply_propval prop3 = { 0 };
	static struct power_supply *chg_psy;
	int ret;

	chg_psy = info->chg_psy;

	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s retry to get chg_psy\n", __func__);
		chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev, "charger");
		info->chg_psy = chg_psy;
	}

	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);

		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_TYPE, &prop2);

		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop3);

		if (prop.intval == 0 ||
		    (prop2.intval == POWER_SUPPLY_TYPE_USB &&
		    prop3.intval == POWER_SUPPLY_USB_TYPE_UNKNOWN))
			prop2.intval = POWER_SUPPLY_TYPE_UNKNOWN;
	}

	chr_debug("%s online:%d type:%d usb_type:%d\n", __func__,
		prop.intval,
		prop2.intval,
		prop3.intval);

	return prop2.intval;
}

int get_usb_type(struct mtk_charger *info)
{
	union power_supply_propval prop = { 0 };
	union power_supply_propval prop2 = { 0 };
	static struct power_supply *chg_psy;
	int ret;

	chg_psy = info->chg_psy;

	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s retry to get chg_psy\n", __func__);
		chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev,
						       "charger");
		info->chg_psy = chg_psy;
	}
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop2);
	}
	chr_debug("%s online:%d usb_type:%d\n", __func__,
		prop.intval,
		prop2.intval);
	return prop2.intval;
}

int get_charger_temperature(struct mtk_charger *info,
	struct charger_device *chg)
{
	int ret = 0;
	int tchg_min = 0, tchg_max = 0;

	if (info == NULL)
		return 0;

	ret = charger_dev_get_temperature(chg, &tchg_min, &tchg_max);
	if (ret < 0)
		chr_err("%s: get temperature failed: %d\n", __func__, ret);
	else
		ret = (tchg_max + tchg_min) / 2;

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_charger_charging_current(struct mtk_charger *info,
	struct charger_device *chg)
{
	int ret = 0;
	int oldua = 0;

	if (info == NULL)
		return 0;
	ret = charger_dev_get_charging_current(chg, &oldua);
	if (ret < 0)
		chr_err("%s: get charging current failed: %d\n", __func__, ret);
	else
		ret = oldua;

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_charger_input_current(struct mtk_charger *info,
	struct charger_device *chg)
{
	int ret = 0;
	int oldua = 0;

	if (info == NULL)
		return 0;
	ret = charger_dev_get_input_current(chg, &oldua);
	if (ret < 0)
		chr_err("%s: get input current failed: %d\n", __func__, ret);
	else
		ret = oldua;

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_charger_zcv(struct mtk_charger *info,
	struct charger_device *chg)
{
	int ret = 0;
	int zcv = 0;

	if (info == NULL)
		return 0;

	ret = charger_dev_get_zcv(chg, &zcv);
	if (ret < 0)
		chr_err("%s: get charger zcv failed: %d\n", __func__, ret);
	else
		ret = zcv;
	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

#define PMIC_RG_VCDT_HV_EN_ADDR		0xb88
#define PMIC_RG_VCDT_HV_EN_MASK		0x1
#define PMIC_RG_VCDT_HV_EN_SHIFT	11

static void pmic_set_register_value(struct regmap *map,
	unsigned int addr,
	unsigned int mask,
	unsigned int shift,
	unsigned int val)
{
	regmap_update_bits(map,
		addr,
		mask << shift,
		val << shift);
}

unsigned int pmic_get_register_value(struct regmap *map,
	unsigned int addr,
	unsigned int mask,
	unsigned int shift)
{
	unsigned int value = 0;

	regmap_read(map, addr, &value);
	value =
		(value &
		(mask << shift))
		>> shift;
	return value;
}

#define DEFAULT_PMIC_RG_MOS_CON1_ADDR	0x989
#define DEFAULT_PMIC_RG_MOS_CON1_EN_MASK	0x7
#define DEFAULT_PMIC_RG_MOS_CON1_ENABLE	0x2
#define DEFAULT_PMIC_RG_MOS_CON1_DISABLE	0x0

static struct regmap *pmic_get_regmap(const char *name)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_find_node_by_name(NULL, name);
	if (!np)
		return NULL;

	pdev = of_find_device_by_node(np->child);
	if (!pdev)
		return NULL;

	return dev_get_regmap(pdev->dev.parent, NULL);
}

static int oplus_chg_common_set_dischg_enable(struct mtk_charger *info, bool en)
{
	int ret;
	struct regmap *regmap;

	regmap = pmic_get_regmap("second_pmic");

	if (!regmap) {
		pr_err("%s, oplus_regmap is null!!!\n", __func__);
		return -1;
	}

	if (en) {
		ret = regmap_update_bits(regmap,
		                         info->usbtemp_dischg_reg_addr,
		                         info->usbtemp_dischg_reg_mask, info->usbtemp_dischg_enable);
		if (ret < 0) {
			pr_err("%s, enable dischg failed.\n", __func__);
			return ret;
		}
	} else {
		ret = regmap_update_bits(regmap,
		                         info->usbtemp_dischg_reg_addr,
		                         info->usbtemp_dischg_reg_mask, info->usbtemp_dischg_disable);

		if (ret < 0) {
			pr_err("%s, disable dischg failed.\n", __func__);
			return ret;
		}
	}

	return 0;
}

int disable_hw_ovp(struct mtk_charger *info, int en)
{
	struct device_node *pmic_node;
	struct platform_device *pmic_pdev;
	struct mt6397_chip *chip;
	struct regmap *regmap;

	pmic_node = of_parse_phandle(info->pdev->dev.of_node, "pmic", 0);
	if (!pmic_node) {
		chr_err("get pmic_node fail\n");
		return -1;
	}

	pmic_pdev = of_find_device_by_node(pmic_node);
	if (!pmic_pdev) {
		chr_err("get pmic_pdev fail\n");
		return -1;
	}
	chip = dev_get_drvdata(&(pmic_pdev->dev));

	if (!chip) {
		chr_err("get chip fail\n");
		return -1;
	}

	regmap = chip->regmap;

	pmic_set_register_value(regmap,
		PMIC_RG_VCDT_HV_EN_ADDR,
		PMIC_RG_VCDT_HV_EN_SHIFT,
		PMIC_RG_VCDT_HV_EN_MASK,
		en);

	return 0;
}

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

#ifdef MODULE
static char __chg_cmdline[COMMAND_LINE_SIZE];
static char *chg_cmdline = __chg_cmdline;

const char *chg_get_cmd(void)
{
	struct device_node * of_chosen = NULL;
	char *bootargs = NULL;

	if (__chg_cmdline[0] != 0)
		return chg_cmdline;

	of_chosen = of_find_node_by_path("/chosen");
	if (of_chosen) {
		bootargs = (char *)of_get_property(
					of_chosen, "bootargs", NULL);
		if (!bootargs)
			chr_err("%s: failed to get bootargs\n", __func__);
		else {
			strcpy(__chg_cmdline, bootargs);
			chr_err("%s: bootargs: %s\n", __func__, bootargs);
		}
	} else
		chr_err("%s: failed to get /chosen \n", __func__);

	return chg_cmdline;
}

#else
const char *chg_get_cmd(void)
{
	return saved_command_line;
}
#endif

int chr_get_debug_level(void)
{
	struct power_supply *psy;
	static struct mtk_charger *info;
	int ret;

	if (info == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (psy == NULL)
			ret = CHRLOG_DEBUG_LEVEL;
		else {
			info =
			(struct mtk_charger *)power_supply_get_drvdata(psy);
			if (info == NULL)
				ret = CHRLOG_DEBUG_LEVEL;
			else
				ret = info->log_level;
		}
	} else
		ret = info->log_level;

	return ret;
}
EXPORT_SYMBOL(chr_get_debug_level);

void _wake_up_charger(struct mtk_charger *info)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
/* do not use mtk charger thread */
	return;
#else
	unsigned long flags;

	info->timer_cb_duration[2] = ktime_get_boottime();
	if (info == NULL)
		return;

	spin_lock_irqsave(&info->slock, flags);
	info->timer_cb_duration[3] = ktime_get_boottime();
	if (!info->charger_wakelock->active)
		__pm_stay_awake(info->charger_wakelock);
	info->timer_cb_duration[4] = ktime_get_boottime();
	spin_unlock_irqrestore(&info->slock, flags);
	info->charger_thread_timeout = true;
	info->timer_cb_duration[5] = ktime_get_boottime();
	wake_up_interruptible(&info->wait_que);
#endif
}

bool is_disable_charger(struct mtk_charger *info)
{
	if (info == NULL)
		return true;

	if (info->disable_charger == true || IS_ENABLED(CONFIG_POWER_EXT))
		return true;
	else
		return false;
}

int _mtk_enable_charging(struct mtk_charger *info,
	bool en)
{
	chr_debug("%s en:%d\n", __func__, en);
	if (info->algo.enable_charging != NULL)
		return info->algo.enable_charging(info, en);
	return false;
}

int mtk_charger_notifier(struct mtk_charger *info, int event)
{
	return srcu_notifier_call_chain(&info->evt_nh, event, NULL);
}

static void mtk_charger_parse_dt(struct mtk_charger *info,
				struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 val = 0;
	struct device_node *boot_node = NULL;
	struct tag_bootmode *tag = NULL;
	int ret = 0;
	int i = 0;
	unsigned int support_subboard_parameter[SUPPORT_PARAMETER_SIZE] = {0};
	struct temp_param subboard_table[SUPPORT_TABLE_SIZE / 2] = {{0, 0}, };

	boot_node = of_parse_phandle(dev->of_node, "bootmode", 0);
	if (!boot_node)
		chr_err("%s: failed to get boot mode phandle\n", __func__);
	else {
		tag = (struct tag_bootmode *)of_get_property(boot_node,
							"atag,boot", NULL);
		if (!tag)
			chr_err("%s: failed to get atag,boot\n", __func__);
		else {
			chr_err("%s: size:0x%x tag:0x%x bootmode:0x%x boottype:0x%x\n",
				__func__, tag->size, tag->tag,
				tag->bootmode, tag->boottype);
			info->bootmode = tag->bootmode;
			info->boottype = tag->boottype;
		}
	}

	if (of_property_read_string(np, "algorithm_name",
		&info->algorithm_name) < 0) {
		chr_err("%s: no algorithm_name name\n", __func__);
		info->algorithm_name = "Basic";
	}

#ifndef OPLUS_FEATURE_CHG_BASIC
	if (strcmp(info->algorithm_name, "Basic") == 0) {
		chr_err("found Basic\n");
		mtk_basic_charger_init(info);
	} else if (strcmp(info->algorithm_name, "Pulse") == 0) {
		chr_err("found Pulse\n");
		mtk_pulse_charger_init(info);
	}
#endif

	info->disable_charger = of_property_read_bool(np, "disable_charger");
	info->atm_enabled = of_property_read_bool(np, "atm_is_enabled");
	info->enable_sw_safety_timer =
			of_property_read_bool(np, "enable_sw_safety_timer");
	info->sw_safety_timer_setting = info->enable_sw_safety_timer;

	/* common */

	if (of_property_read_u32(np, "charger_configuration", &val) >= 0)
		info->config = val;
	else {
		chr_err("use default charger_configuration:%d\n",
			SINGLE_CHARGER);
		info->config = SINGLE_CHARGER;
	}

	if (of_property_read_u32(np, "battery_cv", &val) >= 0)
		info->data.battery_cv = val;
	else {
		chr_err("use default BATTERY_CV:%d\n", BATTERY_CV);
		info->data.battery_cv = BATTERY_CV;
	}

	if (of_property_read_u32(np, "max_charger_voltage", &val) >= 0)
		info->data.max_charger_voltage = val;
	else {
		chr_err("use default V_CHARGER_MAX:%d\n", V_CHARGER_MAX);
		info->data.max_charger_voltage = V_CHARGER_MAX;
	}
	info->data.max_charger_voltage_setting = info->data.max_charger_voltage;

	if (of_property_read_u32(np, "min_charger_voltage", &val) >= 0)
		info->data.min_charger_voltage = val;
	else {
		chr_err("use default V_CHARGER_MIN:%d\n", V_CHARGER_MIN);
		info->data.min_charger_voltage = V_CHARGER_MIN;
	}
	info->enable_vbat_mon = of_property_read_bool(np, "enable_vbat_mon");
	if (info->enable_vbat_mon == true)
		info->setting.vbat_mon_en = true;
	chr_err("use 6pin bat, enable_vbat_mon:%d\n", info->enable_vbat_mon);

	/* sw jeita */
	info->enable_sw_jeita = of_property_read_bool(np, "enable_sw_jeita");
	if (of_property_read_u32(np, "jeita_temp_above_t4_cv", &val) >= 0)
		info->data.jeita_temp_above_t4_cv = val;
	else {
		chr_err("use default JEITA_TEMP_ABOVE_T4_CV:%d\n",
			JEITA_TEMP_ABOVE_T4_CV);
		info->data.jeita_temp_above_t4_cv = JEITA_TEMP_ABOVE_T4_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t3_to_t4_cv", &val) >= 0)
		info->data.jeita_temp_t3_to_t4_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T3_TO_T4_CV:%d\n",
			JEITA_TEMP_T3_TO_T4_CV);
		info->data.jeita_temp_t3_to_t4_cv = JEITA_TEMP_T3_TO_T4_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t2_to_t3_cv", &val) >= 0)
		info->data.jeita_temp_t2_to_t3_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T2_TO_T3_CV:%d\n",
			JEITA_TEMP_T2_TO_T3_CV);
		info->data.jeita_temp_t2_to_t3_cv = JEITA_TEMP_T2_TO_T3_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t1_to_t2_cv", &val) >= 0)
		info->data.jeita_temp_t1_to_t2_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T1_TO_T2_CV:%d\n",
			JEITA_TEMP_T1_TO_T2_CV);
		info->data.jeita_temp_t1_to_t2_cv = JEITA_TEMP_T1_TO_T2_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t0_to_t1_cv", &val) >= 0)
		info->data.jeita_temp_t0_to_t1_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T0_TO_T1_CV:%d\n",
			JEITA_TEMP_T0_TO_T1_CV);
		info->data.jeita_temp_t0_to_t1_cv = JEITA_TEMP_T0_TO_T1_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_below_t0_cv", &val) >= 0)
		info->data.jeita_temp_below_t0_cv = val;
	else {
		chr_err("use default JEITA_TEMP_BELOW_T0_CV:%d\n",
			JEITA_TEMP_BELOW_T0_CV);
		info->data.jeita_temp_below_t0_cv = JEITA_TEMP_BELOW_T0_CV;
	}

	if (of_property_read_u32(np, "temp_t4_thres", &val) >= 0)
		info->data.temp_t4_thres = val;
	else {
		chr_err("use default TEMP_T4_THRES:%d\n",
			TEMP_T4_THRES);
		info->data.temp_t4_thres = TEMP_T4_THRES;
	}

	if (of_property_read_u32(np, "temp_t4_thres_minus_x_degree", &val) >= 0)
		info->data.temp_t4_thres_minus_x_degree = val;
	else {
		chr_err("use default TEMP_T4_THRES_MINUS_X_DEGREE:%d\n",
			TEMP_T4_THRES_MINUS_X_DEGREE);
		info->data.temp_t4_thres_minus_x_degree =
					TEMP_T4_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t3_thres", &val) >= 0)
		info->data.temp_t3_thres = val;
	else {
		chr_err("use default TEMP_T3_THRES:%d\n",
			TEMP_T3_THRES);
		info->data.temp_t3_thres = TEMP_T3_THRES;
	}

	if (of_property_read_u32(np, "temp_t3_thres_minus_x_degree", &val) >= 0)
		info->data.temp_t3_thres_minus_x_degree = val;
	else {
		chr_err("use default TEMP_T3_THRES_MINUS_X_DEGREE:%d\n",
			TEMP_T3_THRES_MINUS_X_DEGREE);
		info->data.temp_t3_thres_minus_x_degree =
					TEMP_T3_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t2_thres", &val) >= 0)
		info->data.temp_t2_thres = val;
	else {
		chr_err("use default TEMP_T2_THRES:%d\n",
			TEMP_T2_THRES);
		info->data.temp_t2_thres = TEMP_T2_THRES;
	}

	if (of_property_read_u32(np, "temp_t2_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t2_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T2_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T2_THRES_PLUS_X_DEGREE);
		info->data.temp_t2_thres_plus_x_degree =
					TEMP_T2_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t1_thres", &val) >= 0)
		info->data.temp_t1_thres = val;
	else {
		chr_err("use default TEMP_T1_THRES:%d\n",
			TEMP_T1_THRES);
		info->data.temp_t1_thres = TEMP_T1_THRES;
	}

	if (of_property_read_u32(np, "temp_t1_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t1_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T1_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T1_THRES_PLUS_X_DEGREE);
		info->data.temp_t1_thres_plus_x_degree =
					TEMP_T1_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t0_thres", &val) >= 0)
		info->data.temp_t0_thres = val;
	else {
		chr_err("use default TEMP_T0_THRES:%d\n",
			TEMP_T0_THRES);
		info->data.temp_t0_thres = TEMP_T0_THRES;
	}

	if (of_property_read_u32(np, "temp_t0_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t0_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T0_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T0_THRES_PLUS_X_DEGREE);
		info->data.temp_t0_thres_plus_x_degree =
					TEMP_T0_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_neg_10_thres", &val) >= 0)
		info->data.temp_neg_10_thres = val;
	else {
		chr_err("use default TEMP_NEG_10_THRES:%d\n",
			TEMP_NEG_10_THRES);
		info->data.temp_neg_10_thres = TEMP_NEG_10_THRES;
	}

	/* battery temperature protection */
	info->thermal.sm = BAT_TEMP_NORMAL;
	info->thermal.enable_min_charge_temp =
		of_property_read_bool(np, "enable_min_charge_temp");

	if (of_property_read_u32(np, "min_charge_temp", &val) >= 0)
		info->thermal.min_charge_temp = val;
	else {
		chr_err("use default MIN_CHARGE_TEMP:%d\n",
			MIN_CHARGE_TEMP);
		info->thermal.min_charge_temp = MIN_CHARGE_TEMP;
	}

	if (of_property_read_u32(np, "min_charge_temp_plus_x_degree", &val)
		>= 0) {
		info->thermal.min_charge_temp_plus_x_degree = val;
	} else {
		chr_err("use default MIN_CHARGE_TEMP_PLUS_X_DEGREE:%d\n",
			MIN_CHARGE_TEMP_PLUS_X_DEGREE);
		info->thermal.min_charge_temp_plus_x_degree =
					MIN_CHARGE_TEMP_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "max_charge_temp", &val) >= 0)
		info->thermal.max_charge_temp = val;
	else {
		chr_err("use default MAX_CHARGE_TEMP:%d\n",
			MAX_CHARGE_TEMP);
		info->thermal.max_charge_temp = MAX_CHARGE_TEMP;
	}

	if (of_property_read_u32(np, "max_charge_temp_minus_x_degree", &val)
		>= 0) {
		info->thermal.max_charge_temp_minus_x_degree = val;
	} else {
		chr_err("use default MAX_CHARGE_TEMP_MINUS_X_DEGREE:%d\n",
			MAX_CHARGE_TEMP_MINUS_X_DEGREE);
		info->thermal.max_charge_temp_minus_x_degree =
					MAX_CHARGE_TEMP_MINUS_X_DEGREE;
	}

	/* charging current */
	if (of_property_read_u32(np, "usb_charger_current", &val) >= 0) {
		info->data.usb_charger_current = val;
	} else {
		chr_err("use default USB_CHARGER_CURRENT:%d\n",
			USB_CHARGER_CURRENT);
		info->data.usb_charger_current = USB_CHARGER_CURRENT;
	}

	if (of_property_read_u32(np, "ac_charger_current", &val) >= 0) {
		info->data.ac_charger_current = val;
	} else {
		chr_err("use default AC_CHARGER_CURRENT:%d\n",
			AC_CHARGER_CURRENT);
		info->data.ac_charger_current = AC_CHARGER_CURRENT;
	}

	if (of_property_read_u32(np, "ac_charger_input_current", &val) >= 0)
		info->data.ac_charger_input_current = val;
	else {
		chr_err("use default AC_CHARGER_INPUT_CURRENT:%d\n",
			AC_CHARGER_INPUT_CURRENT);
		info->data.ac_charger_input_current = AC_CHARGER_INPUT_CURRENT;
	}

	if (of_property_read_u32(np, "charging_host_charger_current", &val)
		>= 0) {
		info->data.charging_host_charger_current = val;
	} else {
		chr_err("use default CHARGING_HOST_CHARGER_CURRENT:%d\n",
			CHARGING_HOST_CHARGER_CURRENT);
		info->data.charging_host_charger_current =
					CHARGING_HOST_CHARGER_CURRENT;
	}

	/* dynamic mivr */
	info->enable_dynamic_mivr =
			of_property_read_bool(np, "enable_dynamic_mivr");

	if (of_property_read_u32(np, "min_charger_voltage_1", &val) >= 0)
		info->data.min_charger_voltage_1 = val;
	else {
		chr_err("use default V_CHARGER_MIN_1: %d\n", V_CHARGER_MIN_1);
		info->data.min_charger_voltage_1 = V_CHARGER_MIN_1;
	}

	if (of_property_read_u32(np, "min_charger_voltage_2", &val) >= 0)
		info->data.min_charger_voltage_2 = val;
	else {
		chr_err("use default V_CHARGER_MIN_2: %d\n", V_CHARGER_MIN_2);
		info->data.min_charger_voltage_2 = V_CHARGER_MIN_2;
	}

	if (of_property_read_u32(np, "max_dmivr_charger_current", &val) >= 0)
		info->data.max_dmivr_charger_current = val;
	else {
		chr_err("use default MAX_DMIVR_CHARGER_CURRENT: %d\n",
			MAX_DMIVR_CHARGER_CURRENT);
		info->data.max_dmivr_charger_current =
					MAX_DMIVR_CHARGER_CURRENT;
	}
	/* fast charging algo support indicator */
	info->enable_fast_charging_indicator =
			of_property_read_bool(np, "enable_fast_charging_indicator");

	info->support_ntc_01c_precision = of_property_read_bool(np, "oplus,support_ntc_01c_precision");
	info->support_subboard_ntc = of_property_read_bool(np, "oplus,support_subboard_ntc");
	/* ntc_resistance:100k internal_pull_up:100k voltage:1.84v */
	info->ntc_temp_volt_1840mv = of_property_read_bool(np, "oplus,ntc_temp_volt_1840mv");
	chr_debug("%s: support_ntc_01c_precision: %d, support_subboard_ntc = %d\n",
		__func__, info->support_ntc_01c_precision, info->support_subboard_ntc);

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
	info->ntc_switch_gpio.ntc_switch_charge_ic = of_property_read_bool(np, "oplus,ntc_switch_charge_ic");
#endif

	ret = read_signed_data_from_node(np, "oplus,support_subboard_parameter",
			(s32 *)(support_subboard_parameter), SUPPORT_PARAMETER_SIZE);
	if (ret < 0) {
		info->subboard_parameters.parameters_support = 0;
		info->subboard_parameters.table_support = 0;
	} else {
		info->subboard_parameters.parameters_support = support_subboard_parameter[0];
		info->subboard_parameters.table_support = support_subboard_parameter[1];
		info->subboard_parameters.i_tap_over_critical_low = support_subboard_parameter[2];
		info->subboard_parameters.i_rap_pull_up_r = support_subboard_parameter[3];
		info->subboard_parameters.i_rap_pull_up_voltage = support_subboard_parameter[4];
		info->subboard_parameters.i_tap_min = support_subboard_parameter[5];
		info->subboard_parameters.i_tap_max = support_subboard_parameter[6];
		info->subboard_parameters.i_25c_volt = support_subboard_parameter[7];
		ret = read_signed_data_from_node(np, "oplus,support_subboard_table",
			(s32 *)subboard_table, SUPPORT_TABLE_SIZE);
		if (ret < 0) {
			info->subboard_parameters.table_support = 0;
		} else {
			info->support_subboard_table = vmalloc(sizeof(struct temp_param *) * SUPPORT_TABLE_SIZE / 2);
			if (info->support_subboard_table) {
				for (i = 0; i < SUPPORT_TABLE_SIZE / 2; i++) {
					info->support_subboard_table[i].temperature_r = subboard_table[i].temperature_r;
					info->support_subboard_table[i].bts_temp = subboard_table[i].bts_temp;
				}
			}
		}
		chr_debug("%s:%d, %d, %d, %d,%d, %d, %d, %d\n", __func__, info->subboard_parameters.parameters_support,
				info->subboard_parameters.table_support, info->subboard_parameters.i_tap_over_critical_low,
				info->subboard_parameters.i_rap_pull_up_r, info->subboard_parameters.i_rap_pull_up_voltage,
				info->subboard_parameters.i_tap_min, info->subboard_parameters.i_tap_max,
				info->subboard_parameters.i_25c_volt);
	}

	if (of_property_read_u32(np, "oplus,sub_board_pull_up_r", &val) >= 0)
		info->sub_board_pull_up_r = val;
	else
		info->sub_board_pull_up_r = 100000;

	info->usbtemp_dischg_reg_configurable =
		of_property_read_bool(np, "oplus,support_usbtemp_dischg_reg_configurable");

	info->usb_aicl_enhance = of_property_read_bool(np, "oplus,usb_aicl_enhance");

	if (of_property_read_u32(np, "oplus,usbtemp_dischg_reg_addr", &val) >= 0)
		info->usbtemp_dischg_reg_addr = val;
	else {
		chr_err("use default usbtemp_dischg_reg_addr: %d\n", DEFAULT_PMIC_RG_MOS_CON1_ADDR);
		info->usbtemp_dischg_reg_addr = DEFAULT_PMIC_RG_MOS_CON1_ADDR;
	}

	if (of_property_read_u32(np, "oplus,usbtemp_dischg_reg_mask", &val) >= 0)
		info->usbtemp_dischg_reg_mask = val;
	else {
		chr_err("use default usbtemp_dischg_reg_mask: %d\n", DEFAULT_PMIC_RG_MOS_CON1_EN_MASK);
		info->usbtemp_dischg_reg_mask = DEFAULT_PMIC_RG_MOS_CON1_EN_MASK;
	}

	if (of_property_read_u32(np, "oplus,usbtemp_dischg_enable", &val) >= 0)
		info->usbtemp_dischg_enable = val;
	else {
		chr_err("use default usbtemp_dischg_enable: %d\n", DEFAULT_PMIC_RG_MOS_CON1_ENABLE);
		info->usbtemp_dischg_enable = DEFAULT_PMIC_RG_MOS_CON1_ENABLE;
	}

	if (of_property_read_u32(np, "oplus,usbtemp_dischg_disable", &val) >= 0)
		info->usbtemp_dischg_disable = val;
	else {
		chr_err("use default usbtemp_dischg_disable: %d\n", DEFAULT_PMIC_RG_MOS_CON1_DISABLE);
		info->usbtemp_dischg_disable = DEFAULT_PMIC_RG_MOS_CON1_DISABLE;
	}

	if (of_property_read_u32(np, "oplus,wls_boost_vol_start_mv", &val) >= 0)
		info->wls_boost_vol_start_mv = val;
	else
		info->wls_boost_vol_start_mv = 0;

	if (of_property_read_u32(np, "oplus,wls_boost_vol_max_mv", &val) >= 0)
		info->wls_boost_vol_max_mv = val;
	else
		info->wls_boost_vol_max_mv = 5500;

	if (of_property_read_u32(np, "oplus,low_batt_otg_boost_curr_ua", &val) >= 0) {
		info->low_batt_otg_boost_curr_ua = val;
	} else {
		info->low_batt_otg_boost_curr_ua = 0;
	}
	info->cp_sub_btb_temp_share = of_property_read_bool(np, "oplus,cp_sub_btb_temp_share");
	info->support_adc_vin = of_property_read_bool(np, "oplus,support_adc_vin");
	info->cp_btb_temp_share = of_property_read_bool(np, "oplus,cp_btb_temp_share");
	chr_info("cp_sub_btb_temp_share: %d, support_adc_vin: %d, cp_btb_temp_share: %d\n",
		info->cp_sub_btb_temp_share, info->support_adc_vin, info->cp_btb_temp_share);
	if (info->support_adc_vin)
		of_property_read_u32(np, "adc_vin_reg", &adc_vin_reg);

	chr_debug("%s:usbtemp dischg (configurable, reg, mask, enable, disable):(%d, %d, %d, %d, %d)\n",
	          __func__, info->usbtemp_dischg_reg_configurable, info->usbtemp_dischg_reg_addr,
	          info->usbtemp_dischg_reg_mask, info->usbtemp_dischg_enable,
	          info->usbtemp_dischg_disable);
}

static void mtk_charger_start_timer(struct mtk_charger *info)
{
	struct timespec64 end_time, time_now;
	ktime_t ktime, ktime_now;
	int ret = 0;

	/* If the timer was already set, cancel it */
	ret = alarm_try_to_cancel(&info->charger_timer);
	if (ret < 0) {
		chr_err("%s: callback was running, skip timer\n", __func__);
		return;
	}

	ktime_now = ktime_get_boottime();
	time_now = ktime_to_timespec64(ktime_now);
	end_time.tv_sec = time_now.tv_sec + info->polling_interval;
	end_time.tv_nsec = time_now.tv_nsec + 0;
	info->endtime = end_time;
	ktime = ktime_set(info->endtime.tv_sec, info->endtime.tv_nsec);

	chr_err("alarm timer start:%d, %lld %ld\n", ret,
		info->endtime.tv_sec, info->endtime.tv_nsec);
	alarm_start(&info->charger_timer, ktime);
}

static void check_battery_exist(struct mtk_charger *info)
{
	unsigned int i = 0;
	int count = 0;
	/*int boot_mode = get_boot_mode();*/

	if (is_disable_charger(info))
		return;

	for (i = 0; i < 3; i++) {
		if (is_battery_exist(info) == false)
			count++;
	}

#ifdef FIXME
	if (count >= 3) {
		if (boot_mode == META_BOOT || boot_mode == ADVMETA_BOOT ||
		    boot_mode == ATE_FACTORY_BOOT)
			chr_info("boot_mode = %d, bypass battery check\n",
				boot_mode);
		else {
			chr_err("battery doesn't exist, shutdown\n");
			orderly_poweroff(true);
		}
	}
#endif
}

static void check_dynamic_mivr(struct mtk_charger *info)
{
	int i = 0, ret = 0;
	int vbat = 0;
	bool is_fast_charge = false;
	struct chg_alg_device *alg = NULL;

	if (!info->enable_dynamic_mivr)
		return;

	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		if (alg == NULL)
			continue;

		ret = chg_alg_is_algo_ready(alg);
		if (ret == ALG_RUNNING) {
			is_fast_charge = true;
			break;
		}
	}

	if (!is_fast_charge) {
		vbat = get_battery_voltage(info);
		if (vbat < info->data.min_charger_voltage_2 / 1000 - 200)
			charger_dev_set_mivr(info->chg1_dev,
				info->data.min_charger_voltage_2);
		else if (vbat < info->data.min_charger_voltage_1 / 1000 - 200)
			charger_dev_set_mivr(info->chg1_dev,
				info->data.min_charger_voltage_1);
		else
			charger_dev_set_mivr(info->chg1_dev,
				info->data.min_charger_voltage);
	}
}

/* sw jeita */
void do_sw_jeita_state_machine(struct mtk_charger *info)
{
	struct sw_jeita_data *sw_jeita;

	sw_jeita = &info->sw_jeita;
	sw_jeita->pre_sm = sw_jeita->sm;
	sw_jeita->charging = true;

	/* JEITA battery temp Standard */
	if (info->battery_temp >= info->data.temp_t4_thres) {
		chr_err("[SW_JEITA] Battery Over high Temperature(%d) !!\n",
			info->data.temp_t4_thres);

		sw_jeita->sm = TEMP_ABOVE_T4;
		sw_jeita->charging = false;
	} else if (info->battery_temp > info->data.temp_t3_thres) {
		/* control 45 degree to normal behavior */
		if ((sw_jeita->sm == TEMP_ABOVE_T4)
		    && (info->battery_temp
			>= info->data.temp_t4_thres_minus_x_degree)) {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d,not allow charging yet!!\n",
				info->data.temp_t4_thres_minus_x_degree,
				info->data.temp_t4_thres);

			sw_jeita->charging = false;
		} else {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
				info->data.temp_t3_thres,
				info->data.temp_t4_thres);

			sw_jeita->sm = TEMP_T3_TO_T4;
		}
	} else if (info->battery_temp >= info->data.temp_t2_thres) {
		if (((sw_jeita->sm == TEMP_T3_TO_T4)
		     && (info->battery_temp
			 >= info->data.temp_t3_thres_minus_x_degree))
		    || ((sw_jeita->sm == TEMP_T1_TO_T2)
			&& (info->battery_temp
			    <= info->data.temp_t2_thres_plus_x_degree))) {
			chr_err("[SW_JEITA] Battery Temperature not recovery to normal temperature charging mode yet!!\n");
		} else {
			chr_err("[SW_JEITA] Battery Normal Temperature between %d and %d !!\n",
				info->data.temp_t2_thres,
				info->data.temp_t3_thres);
			sw_jeita->sm = TEMP_T2_TO_T3;
		}
	} else if (info->battery_temp >= info->data.temp_t1_thres) {
		if ((sw_jeita->sm == TEMP_T0_TO_T1
		     || sw_jeita->sm == TEMP_BELOW_T0)
		    && (info->battery_temp
			<= info->data.temp_t1_thres_plus_x_degree)) {
			if (sw_jeita->sm == TEMP_T0_TO_T1) {
				chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
					info->data.temp_t1_thres_plus_x_degree,
					info->data.temp_t2_thres);
			}
			if (sw_jeita->sm == TEMP_BELOW_T0) {
				chr_err("[SW_JEITA] Battery Temperature between %d and %d,not allow charging yet!!\n",
					info->data.temp_t1_thres,
					info->data.temp_t1_thres_plus_x_degree);
				sw_jeita->charging = false;
			}
		} else {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
				info->data.temp_t1_thres,
				info->data.temp_t2_thres);

			sw_jeita->sm = TEMP_T1_TO_T2;
		}
	} else if (info->battery_temp >= info->data.temp_t0_thres) {
		if ((sw_jeita->sm == TEMP_BELOW_T0)
		    && (info->battery_temp
			<= info->data.temp_t0_thres_plus_x_degree)) {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d,not allow charging yet!!\n",
				info->data.temp_t0_thres,
				info->data.temp_t0_thres_plus_x_degree);

			sw_jeita->charging = false;
		} else {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
				info->data.temp_t0_thres,
				info->data.temp_t1_thres);

			sw_jeita->sm = TEMP_T0_TO_T1;
		}
	} else {
		chr_err("[SW_JEITA] Battery below low Temperature(%d) !!\n",
			info->data.temp_t0_thres);
		sw_jeita->sm = TEMP_BELOW_T0;
		sw_jeita->charging = false;
	}

	/* set CV after temperature changed */
	/* In normal range, we adjust CV dynamically */
	if (sw_jeita->sm != TEMP_T2_TO_T3) {
		if (sw_jeita->sm == TEMP_ABOVE_T4)
			sw_jeita->cv = info->data.jeita_temp_above_t4_cv;
		else if (sw_jeita->sm == TEMP_T3_TO_T4)
			sw_jeita->cv = info->data.jeita_temp_t3_to_t4_cv;
		else if (sw_jeita->sm == TEMP_T2_TO_T3)
			sw_jeita->cv = 0;
		else if (sw_jeita->sm == TEMP_T1_TO_T2)
			sw_jeita->cv = info->data.jeita_temp_t1_to_t2_cv;
		else if (sw_jeita->sm == TEMP_T0_TO_T1)
			sw_jeita->cv = info->data.jeita_temp_t0_to_t1_cv;
		else if (sw_jeita->sm == TEMP_BELOW_T0)
			sw_jeita->cv = info->data.jeita_temp_below_t0_cv;
		else
			sw_jeita->cv = info->data.battery_cv;
	} else {
		sw_jeita->cv = 0;
	}

	chr_err("[SW_JEITA]preState:%d newState:%d tmp:%d cv:%d\n",
		sw_jeita->pre_sm, sw_jeita->sm, info->battery_temp,
		sw_jeita->cv);
}

static int mtk_chgstat_notify(struct mtk_charger *info)
{
	int ret = 0;
	char *env[2] = { "CHGSTAT=1", NULL };

	chr_err("%s: 0x%x\n", __func__, info->notify_code);
	ret = kobject_uevent_env(&info->pdev->dev.kobj, KOBJ_CHANGE, env);
	if (ret)
		chr_err("%s: kobject_uevent_fail, ret=%d", __func__, ret);

	return ret;
}

static void mtk_charger_set_algo_log_level(struct mtk_charger *info, int level)
{
	struct chg_alg_device *alg;
	int i = 0, ret = 0;

	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		if (alg == NULL)
			continue;

		ret = chg_alg_set_prop(alg, ALG_LOG_LEVEL, level);
		if (ret < 0)
			chr_err("%s: set ALG_LOG_LEVEL fail, ret =%d", __func__, ret);
	}
}

static ssize_t sw_jeita_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->enable_sw_jeita);
	return sprintf(buf, "%d\n", pinfo->enable_sw_jeita);
}

static ssize_t sw_jeita_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp == 0)
			pinfo->enable_sw_jeita = false;
		else
			pinfo->enable_sw_jeita = true;

	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}

static DEVICE_ATTR_RW(sw_jeita);
/* sw jeita end*/

static ssize_t chr_type_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->chr_type);
	return sprintf(buf, "%d\n", pinfo->chr_type);
}

static ssize_t chr_type_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0)
		pinfo->chr_type = temp;
	else
		chr_err("%s: format error!\n", __func__);

	return size;
}

static DEVICE_ATTR_RW(chr_type);

static ssize_t Pump_Express_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret = 0, i = 0;
	bool is_ta_detected = false;
	struct mtk_charger *pinfo = dev->driver_data;
	struct chg_alg_device *alg = NULL;

	if (!pinfo) {
		chr_err("%s: pinfo is null\n", __func__);
		return sprintf(buf, "%d\n", is_ta_detected);
	}

	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = pinfo->alg[i];
		if (alg == NULL)
			continue;
		ret = chg_alg_is_algo_ready(alg);
		if (ret == ALG_RUNNING) {
			is_ta_detected = true;
			break;
		}
	}
	chr_err("%s: idx = %d, detect = %d\n", __func__, i, is_ta_detected);
	return sprintf(buf, "%d\n", is_ta_detected);
}

static DEVICE_ATTR_RO(Pump_Express);

static ssize_t fast_chg_indicator_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_debug("%s: %d\n", __func__, pinfo->fast_charging_indicator);
	return sprintf(buf, "%d\n", pinfo->fast_charging_indicator);
}

static ssize_t fast_chg_indicator_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int temp;

	if (kstrtouint(buf, 10, &temp) == 0)
		pinfo->fast_charging_indicator = temp;
	else
		chr_err("%s: format error!\n", __func__);

	if (pinfo->fast_charging_indicator > 0) {
		pinfo->log_level = CHRLOG_DEBUG_LEVEL;
		mtk_charger_set_algo_log_level(pinfo, pinfo->log_level);
	}

	_wake_up_charger(pinfo);
	return size;
}

static DEVICE_ATTR_RW(fast_chg_indicator);

static ssize_t enable_meta_current_limit_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_debug("%s: %d\n", __func__, pinfo->enable_meta_current_limit);
	return sprintf(buf, "%d\n", pinfo->enable_meta_current_limit);
}

static ssize_t enable_meta_current_limit_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int temp;

	if (kstrtouint(buf, 10, &temp) == 0)
		pinfo->enable_meta_current_limit = temp;
	else
		chr_err("%s: format error!\n", __func__);

	if (pinfo->enable_meta_current_limit > 0) {
		pinfo->log_level = CHRLOG_DEBUG_LEVEL;
		mtk_charger_set_algo_log_level(pinfo, pinfo->log_level);
	}

	_wake_up_charger(pinfo);
	return size;
}

static DEVICE_ATTR_RW(enable_meta_current_limit);

static ssize_t vbat_mon_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_debug("%s: %d\n", __func__, pinfo->enable_vbat_mon);
	return sprintf(buf, "%d\n", pinfo->enable_vbat_mon);
}

static ssize_t vbat_mon_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int temp;

	if (kstrtouint(buf, 10, &temp) == 0) {
		if (temp == 0)
			pinfo->enable_vbat_mon = false;
		else
			pinfo->enable_vbat_mon = true;
	} else {
		chr_err("%s: format error!\n", __func__);
	}

	_wake_up_charger(pinfo);
	return size;
}

static DEVICE_ATTR_RW(vbat_mon);

static ssize_t ADC_Charger_Voltage_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int vbus = get_vbus(pinfo); /* mV */

	chr_err("%s: %d\n", __func__, vbus);
	return sprintf(buf, "%d\n", vbus);
}

static DEVICE_ATTR_RO(ADC_Charger_Voltage);

static ssize_t ADC_Charging_Current_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int ibat = get_battery_current(pinfo); /* mA */

	chr_err("%s: %d\n", __func__, ibat);
	return sprintf(buf, "%d\n", ibat);
}

static DEVICE_ATTR_RO(ADC_Charging_Current);

static ssize_t input_current_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int aicr = 0;

	aicr = pinfo->chg_data[CHG1_SETTING].thermal_input_current_limit;
	chr_err("%s: %d\n", __func__, aicr);
	return sprintf(buf, "%d\n", aicr);
}

static ssize_t input_current_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	struct charger_data *chg_data;
	signed int temp;

	chg_data = &pinfo->chg_data[CHG1_SETTING];
	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp < 0)
			chg_data->thermal_input_current_limit = 0;
		else
			chg_data->thermal_input_current_limit = temp;
	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}

static DEVICE_ATTR_RW(input_current);

static ssize_t charger_ic_ntc_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	int temp = 0;

	if (pinfo->chargeric_temp_chan)
		temp = oplus_get_chargeric_temp();
	else if (pinfo->cp_btb_temp_share)
		temp = oplus_chg_get_battery_btb_temp_cal();
	else
		temp = oplus_get_master_cp_temp();
	return sprintf(buf, "%d\n", temp);
}

static DEVICE_ATTR_RO(charger_ic_ntc);

static ssize_t charger_subboard_ntc_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	int temp = 0;

	temp = oplus_get_subboard_temp();
	return sprintf(buf, "%d\n", temp);
}

static DEVICE_ATTR_RO(charger_subboard_ntc);

static ssize_t charger_log_level_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->log_level);
	return sprintf(buf, "%d\n", pinfo->log_level);
}

static ssize_t charger_log_level_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp < 0) {
			chr_err("val is invalid: %d\n", temp);
			temp = 0;
		}
		pinfo->log_level = temp;
		chr_err("log_level=%d\n", pinfo->log_level);

	} else {
		chr_err("format error!\n");
	}
	return size;
}

static DEVICE_ATTR_RW(charger_log_level);

static ssize_t BatteryNotify_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_info("%s: 0x%x\n", __func__, pinfo->notify_code);

	return sprintf(buf, "%u\n", pinfo->notify_code);
}

static ssize_t BatteryNotify_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int reg = 0;
	int ret = 0;

	if (buf != NULL && size != 0) {
		ret = kstrtouint(buf, 16, &reg);
		if (ret < 0) {
			chr_err("%s: failed, ret = %d\n", __func__, ret);
			return ret;
		}
		pinfo->notify_code = reg;
		chr_info("%s: store code=0x%x\n", __func__, pinfo->notify_code);
		mtk_chgstat_notify(pinfo);
	}
	return size;
}

static DEVICE_ATTR_RW(BatteryNotify);

/* procfs */
static int mtk_chg_set_cv_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;

	seq_printf(m, "%d\n", pinfo->data.battery_cv);
	return 0;
}

static int mtk_chg_set_cv_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_set_cv_show, PDE_DATA(node));
}

static ssize_t mtk_chg_set_cv_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int cv = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));
	struct power_supply *psy = NULL;
	union  power_supply_propval dynamic_cv;

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	ret = kstrtou32(desc, 10, &cv);
	if (ret == 0) {
		if (cv >= BATTERY_CV) {
			info->data.battery_cv = BATTERY_CV;
			chr_info("%s: adjust charge voltage %dV too high, use default cv\n",
				  __func__, cv);
		} else {
			info->data.battery_cv = cv;
			chr_info("%s: adjust charge voltage = %dV\n", __func__, cv);
		}
		psy = power_supply_get_by_name("battery");
		if (!IS_ERR_OR_NULL(psy)) {
			dynamic_cv.intval = info->data.battery_cv;
			ret = power_supply_set_property(psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &dynamic_cv);
			if (ret < 0)
				chr_err("set gauge cv fail\n");
		}
		return count;
	}

	chr_err("%s: bad argument\n", __func__);
	return count;
}

static const struct proc_ops mtk_chg_set_cv_fops = {
	.proc_open = mtk_chg_set_cv_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = mtk_chg_set_cv_write,
};

static int mtk_chg_current_cmd_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;

	seq_printf(m, "%d %d\n", pinfo->usb_unlimited, pinfo->cmd_discharging);
	return 0;
}

static int mtk_chg_current_cmd_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_current_cmd_show, PDE_DATA(node));
}

static ssize_t mtk_chg_current_cmd_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0;
	char desc[32] = {0};
	int current_unlimited = 0;
	int cmd_discharging = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	if (sscanf(desc, "%d %d", &current_unlimited, &cmd_discharging) == 2) {
		info->usb_unlimited = current_unlimited;
		if (cmd_discharging == 1) {
			info->cmd_discharging = true;
			charger_dev_enable(info->chg1_dev, false);
			charger_dev_do_event(info->chg1_dev,
					EVENT_DISCHARGE, 0);
		} else if (cmd_discharging == 0) {
			info->cmd_discharging = false;
			charger_dev_enable(info->chg1_dev, true);
			charger_dev_do_event(info->chg1_dev,
					EVENT_RECHARGE, 0);
		}

		chr_info("%s: current_unlimited=%d, cmd_discharging=%d\n",
			__func__, current_unlimited, cmd_discharging);
		return count;
	}

	chr_err("bad argument, echo [usb_unlimited] [disable] > current_cmd\n");
	return count;
}

static const struct proc_ops mtk_chg_current_cmd_fops = {
	.proc_open = mtk_chg_current_cmd_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = mtk_chg_current_cmd_write,
};

static int mtk_chg_en_power_path_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;
	bool power_path_en = true;

	charger_dev_is_powerpath_enabled(pinfo->chg1_dev, &power_path_en);
	seq_printf(m, "%d\n", power_path_en);

	return 0;
}

static int mtk_chg_en_power_path_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_en_power_path_show, PDE_DATA(node));
}

static ssize_t mtk_chg_en_power_path_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int enable = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	ret = kstrtou32(desc, 10, &enable);
	if (ret == 0) {
		charger_dev_enable_powerpath(info->chg1_dev, enable);
		chr_info("%s: enable power path = %d\n", __func__, enable);
		return count;
	}

	chr_err("bad argument, echo [enable] > en_power_path\n");
	return count;
}

static const struct proc_ops mtk_chg_en_power_path_fops = {
	.proc_open = mtk_chg_en_power_path_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = mtk_chg_en_power_path_write,
};

static int mtk_chg_en_safety_timer_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;
	bool safety_timer_en = false;

	charger_dev_is_safety_timer_enabled(pinfo->chg1_dev, &safety_timer_en);
	seq_printf(m, "%d\n", safety_timer_en);

	return 0;
}

static int mtk_chg_en_safety_timer_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_en_safety_timer_show, PDE_DATA(node));
}

static ssize_t mtk_chg_en_safety_timer_write(struct file *file,
	const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int enable = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	ret = kstrtou32(desc, 10, &enable);
	if (ret == 0) {
		charger_dev_enable_safety_timer(info->chg1_dev, enable);
		info->safety_timer_cmd = (int)enable;
		chr_info("%s: enable safety timer = %d\n", __func__, enable);

		/* SW safety timer */
		if (info->sw_safety_timer_setting == true) {
			if (enable)
				info->enable_sw_safety_timer = true;
			else
				info->enable_sw_safety_timer = false;
		}

		return count;
	}

	chr_err("bad argument, echo [enable] > en_safety_timer\n");
	return count;
}

static const struct proc_ops mtk_chg_en_safety_timer_fops = {
	.proc_open = mtk_chg_en_safety_timer_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = mtk_chg_en_safety_timer_write,
};

void scd_ctrl_cmd_from_user(void *nl_data, struct sc_nl_msg_t *ret_msg)
{
	struct sc_nl_msg_t *msg;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return;

	msg = nl_data;
	ret_msg->sc_cmd = msg->sc_cmd;

	switch (msg->sc_cmd) {
	case SC_DAEMON_CMD_PRINT_LOG: {
		chr_err("[%s] %s", SC_TAG, &msg->sc_data[0]);
		}
	break;
	case SC_DAEMON_CMD_SET_DAEMON_PID: {
		memcpy(&info->sc.g_scd_pid, &msg->sc_data[0],
			sizeof(info->sc.g_scd_pid));
		chr_err("[%s][fr] SC_DAEMON_CMD_SET_DAEMON_PID = %d(first launch)\n",
			SC_TAG, info->sc.g_scd_pid);
		}
	break;
	case SC_DAEMON_CMD_SETTING: {
		struct scd_cmd_param_t_1 data;
			memcpy(&data, &msg->sc_data[0],
				sizeof(struct scd_cmd_param_t_1));

		chr_debug("[%s] rcv data:%d %d %d %d %d %d %d %d %d %d %d %d %d %d Ans:%d\n",
			SC_TAG,
			data.data[0],
			data.data[1],
			data.data[2],
			data.data[3],
			data.data[4],
			data.data[5],
			data.data[6],
			data.data[7],
			data.data[8],
			data.data[9],
			data.data[10],
			data.data[11],
			data.data[12],
			data.data[13],
			data.data[14]);

		info->sc.solution = data.data[SC_SOLUTION];
		if (data.data[SC_SOLUTION] == SC_DISABLE)
			info->sc.disable_charger = true;
		else if (data.data[SC_SOLUTION] == SC_REDUCE)
			info->sc.disable_charger = false;
		else
			info->sc.disable_charger = false;
	}
	break;
	default:
		chr_err("[%s] bad sc_DAEMON_CTRL_CMD_FROM_USER 0x%x\n", SC_TAG, msg->sc_cmd);
		break;
	}
}

static void sc_nl_send_to_user(u32 pid, int seq, struct sc_nl_msg_t *reply_msg)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	/* int size=sizeof(struct fgd_nl_msg_t); */
	int size = reply_msg->sc_data_len + SCD_NL_MSG_T_HDR_LEN;
	int len = NLMSG_SPACE(size);
	void *data;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return;

	reply_msg->identity = SCD_NL_MAGIC;

	if (in_interrupt())
		skb = alloc_skb(len, GFP_ATOMIC);
	else
		skb = alloc_skb(len, GFP_KERNEL);

	if (!skb)
		return;

	nlh = nlmsg_put(skb, pid, seq, 0, size, 0);
	data = NLMSG_DATA(nlh);
	memcpy(data, reply_msg, size);
	NETLINK_CB(skb).portid = 0;	/* from kernel */
	NETLINK_CB(skb).dst_group = 0;	/* unicast */

	ret = netlink_unicast(info->sc.daemo_nl_sk, skb, pid, MSG_DONTWAIT);
	if (ret < 0) {
		chr_err("[Netlink] sc send failed %d\n", ret);
		return;
	}
}

static void chg_nl_data_handler(struct sk_buff *skb)
{
	u32 pid;
	kuid_t uid;
	int seq;
	void *data;
	struct nlmsghdr *nlh;
	struct sc_nl_msg_t *sc_msg, *sc_ret_msg;
	int size = 0;

	nlh = (struct nlmsghdr *)skb->data;
	pid = NETLINK_CREDS(skb)->pid;
	uid = NETLINK_CREDS(skb)->uid;
	seq = nlh->nlmsg_seq;

	data = NLMSG_DATA(nlh);

	sc_msg = (struct sc_nl_msg_t *)data;

	size = sc_msg->sc_ret_data_len + SCD_NL_MSG_T_HDR_LEN;

	if (size > (PAGE_SIZE << 1))
		sc_ret_msg = vmalloc(size);
	else {
		if (in_interrupt())
			sc_ret_msg = kmalloc(size, GFP_ATOMIC);
	else
		sc_ret_msg = kmalloc(size, GFP_KERNEL);
	}

	if (sc_ret_msg == NULL) {
		if (size > PAGE_SIZE)
			sc_ret_msg = vmalloc(size);

		if (sc_ret_msg == NULL)
			return;
	}

	memset(sc_ret_msg, 0, size);

	scd_ctrl_cmd_from_user(data, sc_ret_msg);
	sc_nl_send_to_user(pid, seq, sc_ret_msg);

	kvfree(sc_ret_msg);
}

void sc_select_charging_current(struct mtk_charger *info, struct charger_data *pdata)
{
	if (info->sc.g_scd_pid != 0 && info->sc.disable_in_this_plug == false) {
		chr_debug("sck: %d %d %d %d %d\n",
			info->sc.pre_ibat,
			info->sc.sc_ibat,
			pdata->charging_current_limit,
			pdata->thermal_charging_current_limit,
			info->sc.solution);
		if (info->sc.pre_ibat == -1 || info->sc.solution == SC_IGNORE
			|| info->sc.solution == SC_DISABLE) {
			info->sc.sc_ibat = -1;
		} else {
			if (info->sc.pre_ibat == pdata->charging_current_limit
				&& info->sc.solution == SC_REDUCE
				&& ((pdata->charging_current_limit - 100000) >= 500000)) {
				if (info->sc.sc_ibat == -1)
					info->sc.sc_ibat = pdata->charging_current_limit - 100000;
				else if (info->sc.sc_ibat - 100000 >= 500000)
					info->sc.sc_ibat = info->sc.sc_ibat - 100000;
			}
		}
	}
	info->sc.pre_ibat =  pdata->charging_current_limit;

	if (pdata->thermal_charging_current_limit != -1) {
		if (pdata->thermal_charging_current_limit <
		    pdata->charging_current_limit)
			pdata->charging_current_limit =
					pdata->thermal_charging_current_limit;
		info->sc.disable_in_this_plug = true;
	} else if ((info->sc.solution == SC_REDUCE || info->sc.solution == SC_KEEP)
		&& info->sc.sc_ibat <
		pdata->charging_current_limit && info->sc.g_scd_pid != 0 &&
		info->sc.disable_in_this_plug == false) {
		pdata->charging_current_limit = info->sc.sc_ibat;
	}
}

void sc_init(struct smartcharging *sc)
{
	sc->enable = false;
	sc->battery_size = 3000;
	sc->start_time = 0;
	sc->end_time = 80000;
	sc->current_limit = 2000;
	sc->target_percentage = 80;
	sc->left_time_for_cv = 3600;
	sc->pre_ibat = -1;
}

void sc_update(struct mtk_charger *info)
{
	memset(&info->sc.data, 0, sizeof(struct scd_cmd_param_t_1));
	info->sc.data.data[SC_VBAT] = get_battery_voltage(info);
	info->sc.data.data[SC_BAT_TMP] = get_battery_temperature(info);
	info->sc.data.data[SC_UISOC] = get_uisoc(info);
	/* info->sc.data.data[SC_SOC] = battery_get_soc(); */

	info->sc.data.data[SC_ENABLE] = info->sc.enable;
	info->sc.data.data[SC_BAT_SIZE] = info->sc.battery_size;
	info->sc.data.data[SC_START_TIME] = info->sc.start_time;
	info->sc.data.data[SC_END_TIME] = info->sc.end_time;
	info->sc.data.data[SC_IBAT_LIMIT] = info->sc.current_limit;
	info->sc.data.data[SC_TARGET_PERCENTAGE] = info->sc.target_percentage;
	info->sc.data.data[SC_LEFT_TIME_FOR_CV] = info->sc.left_time_for_cv;

	charger_dev_get_charging_current(info->chg1_dev, &info->sc.data.data[SC_IBAT_SETTING]);
	info->sc.data.data[SC_IBAT_SETTING] = info->sc.data.data[SC_IBAT_SETTING] / 1000;
	info->sc.data.data[SC_IBAT] = get_battery_current(info);
}

int wakeup_sc_algo_cmd(struct scd_cmd_param_t_1 *data, int subcmd, int para1)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (info->sc.g_scd_pid != 0) {
		struct sc_nl_msg_t *sc_msg;
		int size = SCD_NL_MSG_T_HDR_LEN + sizeof(struct scd_cmd_param_t_1);

		if (size > (PAGE_SIZE << 1))
			sc_msg = (struct sc_nl_msg_t*)vmalloc(size);
		else {
			if (in_interrupt())
				sc_msg = kmalloc(size, GFP_ATOMIC);
			else
				sc_msg = kmalloc(size, GFP_KERNEL);
		}

		if (sc_msg == NULL) {
			if (size > PAGE_SIZE)
				sc_msg = (struct sc_nl_msg_t*)vmalloc(size);

			if (sc_msg == NULL)
				return -1;
		}
		chr_debug(
			"[wakeup_fg_algo] malloc size=%d pid=%d\n",
			size, info->sc.g_scd_pid);
		memset(sc_msg, 0, size);
		sc_msg->sc_cmd = SC_DAEMON_CMD_NOTIFY_DAEMON;
		sc_msg->sc_subcmd = subcmd;
		sc_msg->sc_subcmd_para1 = para1;
		memcpy(sc_msg->sc_data, data, sizeof(struct scd_cmd_param_t_1));
		sc_msg->sc_data_len += sizeof(struct scd_cmd_param_t_1);
		sc_nl_send_to_user(info->sc.g_scd_pid, 0, sc_msg);

		kvfree(sc_msg);

		return 0;
	}
	chr_debug("pid is NULL\n");
	return -1;
}

static ssize_t enable_sc_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	chr_err(
	"[enable smartcharging] : %d\n",
	info->sc.enable);

	return sprintf(buf, "%d\n", info->sc.enable);
}

static ssize_t enable_sc_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (buf != NULL && size != 0) {
		chr_err("[enable smartcharging] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		if (ret < 0) {
			chr_err("[enable smartcharging error] ret is %d ??\n", ret);
			val = 0;
		}

		if (val == 0)
			info->sc.enable = false;
		else
			info->sc.enable = true;

		chr_err(
			"[enable smartcharging]enable smartcharging=%d\n",
			info->sc.enable);
	}
	return size;
}
static DEVICE_ATTR_RW(enable_sc);

static ssize_t sc_stime_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	chr_err(
	"[smartcharging stime] : %d\n",
	info->sc.start_time);

	return sprintf(buf, "%d\n", info->sc.start_time);
}

static ssize_t sc_stime_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}

	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (buf != NULL && size != 0) {
		chr_err("[smartcharging stime] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		info->sc.start_time = val;

		chr_info("[smartcharging stime]enable smartcharging=%d\n",
			  info->sc.start_time);
	}
	return size;
}
static DEVICE_ATTR_RW(sc_stime);

static ssize_t sc_etime_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	chr_err(
	"[smartcharging etime] : %d\n",
	info->sc.end_time);

	return sprintf(buf, "%d\n", info->sc.end_time);
}

static ssize_t sc_etime_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (buf != NULL && size != 0) {
		chr_err("[smartcharging etime] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		if (ret < 0) {
			chr_err(
				"[smartcharging etime] val is %d ??\n",
				(int)val);
			val = 0;
		}

		info->sc.end_time = val;

		chr_err(
			"[smartcharging stime]enable smartcharging=%d\n",
			info->sc.end_time);
	}
	return size;
}
static DEVICE_ATTR_RW(sc_etime);

static ssize_t sc_tuisoc_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	chr_err(
	"[smartcharging target uisoc] : %d\n",
	info->sc.target_percentage);

	return sprintf(buf, "%d\n", info->sc.target_percentage);
}

static ssize_t sc_tuisoc_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (buf != NULL && size != 0) {
		chr_err("[smartcharging tuisoc] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		if (ret < 0) {
			chr_err("[smartcharging tuisoc] ret=%d\n", ret);
			val = 0;
		} else {
			info->sc.target_percentage = val;
		}

		chr_err(
			"[smartcharging stime]tuisoc=%d\n",
			info->sc.target_percentage);
	}
	return size;
}
static DEVICE_ATTR_RW(sc_tuisoc);

static ssize_t sc_ibat_limit_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	chr_err(
	"[smartcharging ibat limit] : %d\n",
	info->sc.current_limit);

	return sprintf(buf, "%d\n", info->sc.current_limit);
}

static ssize_t sc_ibat_limit_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (buf != NULL && size != 0) {
		chr_err("[smartcharging ibat limit] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		if (ret < 0) {
			chr_err("[smartcharging ibat limit] ret is %d ??\n", ret);
			val = 0;
		}
		info->sc.current_limit = val;

		chr_err(
			"[smartcharging ibat limit]=%d\n",
			info->sc.current_limit);
	}
	return size;
}
static DEVICE_ATTR_RW(sc_ibat_limit);

int mtk_chg_enable_vbus_ovp(bool enable)
{
	static struct mtk_charger *pinfo;
	int ret = 0;
	u32 sw_ovp = 0;
	struct power_supply *psy;

	if (pinfo == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (psy == NULL) {
			chr_err("[%s]psy is not rdy\n", __func__);
			return -1;
		}

		pinfo = (struct mtk_charger *)power_supply_get_drvdata(psy);
		if (pinfo == NULL) {
			chr_err("[%s]mtk_gauge is not rdy\n", __func__);
			return -1;
		}
	}

	if (enable)
		sw_ovp = pinfo->data.max_charger_voltage_setting;
	else
		sw_ovp = 15000000;

	/* Enable/Disable SW OVP status */
	pinfo->data.max_charger_voltage = sw_ovp;

	disable_hw_ovp(pinfo, enable);

	chr_err("[%s] en:%d ovp:%d\n",
			    __func__, enable, sw_ovp);
	return ret;
}
EXPORT_SYMBOL(mtk_chg_enable_vbus_ovp);

/* return false if vbus is over max_charger_voltage */
static bool mtk_chg_check_vbus(struct mtk_charger *info)
{
	int vchr = 0;

	vchr = get_vbus(info) * 1000; /* uV */
	if (vchr > info->data.max_charger_voltage) {
		chr_err("%s: vbus(%d mV) > %d mV\n", __func__, vchr / 1000,
			info->data.max_charger_voltage / 1000);
		return false;
	}
	return true;
}

static void mtk_battery_notify_VCharger_check(struct mtk_charger *info)
{
#if defined(BATTERY_NOTIFY_CASE_0001_VCHARGER)
	int vchr = 0;

	vchr = get_vbus(info) * 1000; /* uV */
	if (vchr < info->data.max_charger_voltage)
		info->notify_code &= ~CHG_VBUS_OV_STATUS;
	else {
		info->notify_code |= CHG_VBUS_OV_STATUS;
		chr_err("[BATTERY] charger_vol(%d mV) > %d mV\n",
			vchr / 1000, info->data.max_charger_voltage / 1000);
		mtk_chgstat_notify(info);
	}
#endif
}

static void mtk_battery_notify_VBatTemp_check(struct mtk_charger *info)
{
#if defined(BATTERY_NOTIFY_CASE_0002_VBATTEMP)
	if (info->battery_temp >= info->thermal.max_charge_temp) {
		info->notify_code |= CHG_BAT_OT_STATUS;
		chr_err("[BATTERY] bat_temp(%d) out of range(too high)\n",
			info->battery_temp);
		mtk_chgstat_notify(info);
	} else {
		info->notify_code &= ~CHG_BAT_OT_STATUS;
	}

	if (info->enable_sw_jeita == true) {
		if (info->battery_temp < info->data.temp_neg_10_thres) {
			info->notify_code |= CHG_BAT_LT_STATUS;
			chr_err("bat_temp(%d) out of range(too low)\n",
				info->battery_temp);
			mtk_chgstat_notify(info);
		} else {
			info->notify_code &= ~CHG_BAT_LT_STATUS;
		}
	} else {
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
		if (info->battery_temp < info->thermal.min_charge_temp) {
			info->notify_code |= CHG_BAT_LT_STATUS;
			chr_err("bat_temp(%d) out of range(too low)\n",
				info->battery_temp);
			mtk_chgstat_notify(info);
		} else {
			info->notify_code &= ~CHG_BAT_LT_STATUS;
		}
#endif
	}
#endif
}

static void mtk_battery_notify_UI_test(struct mtk_charger *info)
{
	switch (info->notify_test_mode) {
	case 1:
		info->notify_code = CHG_VBUS_OV_STATUS;
		chr_debug("[%s] CASE_0001_VCHARGER\n", __func__);
		break;
	case 2:
		info->notify_code = CHG_BAT_OT_STATUS;
		chr_debug("[%s] CASE_0002_VBATTEMP\n", __func__);
		break;
	case 3:
		info->notify_code = CHG_OC_STATUS;
		chr_debug("[%s] CASE_0003_ICHARGING\n", __func__);
		break;
	case 4:
		info->notify_code = CHG_BAT_OV_STATUS;
		chr_debug("[%s] CASE_0004_VBAT\n", __func__);
		break;
	case 5:
		info->notify_code = CHG_ST_TMO_STATUS;
		chr_debug("[%s] CASE_0005_TOTAL_CHARGINGTIME\n", __func__);
		break;
	case 6:
		info->notify_code = CHG_BAT_LT_STATUS;
		chr_debug("[%s] CASE6: VBATTEMP_LOW\n", __func__);
		break;
	case 7:
		info->notify_code = CHG_TYPEC_WD_STATUS;
		chr_debug("[%s] CASE7: Moisture Detection\n", __func__);
		break;
	default:
		chr_debug("[%s] Unknown BN_TestMode Code: %x\n",
			__func__, info->notify_test_mode);
	}
	mtk_chgstat_notify(info);
}

static void mtk_battery_notify_check(struct mtk_charger *info)
{
	if (info->notify_test_mode == 0x0000) {
		mtk_battery_notify_VCharger_check(info);
		mtk_battery_notify_VBatTemp_check(info);
	} else {
		mtk_battery_notify_UI_test(info);
	}
}

static void mtk_chg_get_tchg(struct mtk_charger *info)
{
	int ret;
	int tchg_min = -127, tchg_max = -127;
	struct charger_data *pdata;
	bool en = false;

	pdata = &info->chg_data[CHG1_SETTING];
	ret = charger_dev_get_temperature(info->chg1_dev, &tchg_min, &tchg_max);
	if (ret < 0) {
		pdata->junction_temp_min = -127;
		pdata->junction_temp_max = -127;
	} else {
		pdata->junction_temp_min = tchg_min;
		pdata->junction_temp_max = tchg_max;
	}

	if (info->chg2_dev) {
		pdata = &info->chg_data[CHG2_SETTING];
		ret = charger_dev_get_temperature(info->chg2_dev,
			&tchg_min, &tchg_max);

		if (ret < 0) {
			pdata->junction_temp_min = -127;
			pdata->junction_temp_max = -127;
		} else {
			pdata->junction_temp_min = tchg_min;
			pdata->junction_temp_max = tchg_max;
		}
	}

	if (info->dvchg1_dev) {
		pdata = &info->chg_data[DVCHG1_SETTING];
		pdata->junction_temp_min = -127;
		pdata->junction_temp_max = -127;
		ret = charger_dev_is_enabled(info->dvchg1_dev, &en);
		if (ret >= 0 && en) {
			ret = charger_dev_get_adc(info->dvchg1_dev,
						  ADC_CHANNEL_TEMP_JC,
						  &tchg_min, &tchg_max);
			if (ret >= 0) {
				pdata->junction_temp_min = tchg_min;
				pdata->junction_temp_max = tchg_max;
			}
		}
	}

	if (info->dvchg2_dev) {
		pdata = &info->chg_data[DVCHG2_SETTING];
		pdata->junction_temp_min = -127;
		pdata->junction_temp_max = -127;
		ret = charger_dev_is_enabled(info->dvchg2_dev, &en);
		if (ret >= 0 && en) {
			ret = charger_dev_get_adc(info->dvchg2_dev,
						  ADC_CHANNEL_TEMP_JC,
						  &tchg_min, &tchg_max);
			if (ret >= 0) {
				pdata->junction_temp_min = tchg_min;
				pdata->junction_temp_max = tchg_max;
			}
		}
	}
}

static void charger_check_status(struct mtk_charger *info)
{
	bool charging = true;
	bool chg_dev_chgen = true;
	int temperature;
	struct battery_thermal_protection_data *thermal;
	int uisoc = 0;

	if (get_charger_type(info) == POWER_SUPPLY_TYPE_UNKNOWN)
		return;

	temperature = info->battery_temp;
	thermal = &info->thermal;
	uisoc = get_uisoc(info);

	info->setting.vbat_mon_en = true;
	if (info->enable_sw_jeita == true || info->enable_vbat_mon != true ||
	    info->batpro_done == true)
		info->setting.vbat_mon_en = false;

	if (info->enable_sw_jeita == true) {
		do_sw_jeita_state_machine(info);
		if (info->sw_jeita.charging == false) {
			charging = false;
			goto stop_charging;
		}
	} else {
		if (thermal->enable_min_charge_temp) {
			if (temperature < thermal->min_charge_temp) {
				chr_err("Battery Under Temperature or NTC fail %d %d\n",
					temperature, thermal->min_charge_temp);
				thermal->sm = BAT_TEMP_LOW;
				charging = false;
				goto stop_charging;
			} else if (thermal->sm == BAT_TEMP_LOW) {
				if (temperature >=
				    thermal->min_charge_temp_plus_x_degree) {
					chr_err("Battery Temperature raise from %d to %d(%d), allow charging!!\n",
					thermal->min_charge_temp,
					temperature,
					thermal->min_charge_temp_plus_x_degree);
					thermal->sm = BAT_TEMP_NORMAL;
				} else {
					charging = false;
					goto stop_charging;
				}
			}
		}

		if (temperature >= thermal->max_charge_temp) {
			chr_err("Battery over Temperature or NTC fail %d %d\n",
				temperature, thermal->max_charge_temp);
			thermal->sm = BAT_TEMP_HIGH;
			charging = false;
			goto stop_charging;
		} else if (thermal->sm == BAT_TEMP_HIGH) {
			if (temperature
			    < thermal->max_charge_temp_minus_x_degree) {
				chr_err("Battery Temperature raise from %d to %d(%d), allow charging!!\n",
				thermal->max_charge_temp,
				temperature,
				thermal->max_charge_temp_minus_x_degree);
				thermal->sm = BAT_TEMP_NORMAL;
			} else {
				charging = false;
				goto stop_charging;
			}
		}
	}

	mtk_chg_get_tchg(info);

	if (!mtk_chg_check_vbus(info)) {
		charging = false;
		goto stop_charging;
	}

	if (info->cmd_discharging)
		charging = false;
	if (info->safety_timeout)
		charging = false;
	if (info->vbusov_stat)
		charging = false;
	if (info->sc.disable_charger == true)
		charging = false;
stop_charging:
	mtk_battery_notify_check(info);

	if (charging && uisoc < 80 && info->batpro_done == true) {
		info->setting.vbat_mon_en = true;
		info->batpro_done = false;
		info->stop_6pin_re_en = false;
	}

	chr_err("tmp:%d (jeita:%d sm:%d cv:%d en:%d) (sm:%d) en:%d c:%d s:%d ov:%d sc:%d %d %d saf_cmd:%d bat_mon:%d %d\n",
		temperature, info->enable_sw_jeita, info->sw_jeita.sm,
		info->sw_jeita.cv, info->sw_jeita.charging, thermal->sm,
		charging, info->cmd_discharging, info->safety_timeout,
		info->vbusov_stat, info->sc.disable_charger,
		info->can_charging, charging, info->safety_timer_cmd,
		info->enable_vbat_mon, info->batpro_done);

	charger_dev_is_enabled(info->chg1_dev, &chg_dev_chgen);

	if (charging != info->can_charging)
		_mtk_enable_charging(info, charging);
	else if (charging == false && chg_dev_chgen == true)
		_mtk_enable_charging(info, charging);

	info->can_charging = charging;
}

static bool charger_init_algo(struct mtk_charger *info)
{
	struct chg_alg_device *alg;
	int idx = 0;

	alg = get_chg_alg_by_name("pe5");
	info->alg[idx] = alg;
	if (alg == NULL)
		chr_err("get pe5 fail\n");
	else {
		chr_err("get pe5 success\n");
		alg->config = info->config;
		alg->alg_id = PE5_ID;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}
	idx++;

	alg = get_chg_alg_by_name("pe4");
	info->alg[idx] = alg;
	if (alg == NULL)
		chr_err("get pe4 fail\n");
	else {
		chr_err("get pe4 success\n");
		alg->config = info->config;
		alg->alg_id = PE4_ID;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}
	idx++;

	alg = get_chg_alg_by_name("pd");
	info->alg[idx] = alg;
	if (alg == NULL)
		chr_err("get pd fail\n");
	else {
		chr_err("get pd success\n");
		alg->config = info->config;
		alg->alg_id = PDC_ID;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}
	idx++;

	alg = get_chg_alg_by_name("pe2");
	info->alg[idx] = alg;
	if (alg == NULL)
		chr_err("get pe2 fail\n");
	else {
		chr_err("get pe2 success\n");
		alg->config = info->config;
		alg->alg_id = PE2_ID;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}
	idx++;

	alg = get_chg_alg_by_name("pe");
	info->alg[idx] = alg;
	if (alg == NULL)
		chr_err("get pe fail\n");
	else {
		chr_err("get pe success\n");
		alg->config = info->config;
		alg->alg_id = PE_ID;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}

	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev)
		chr_err("Found primary charger\n");
	else {
		chr_err("*** Error : can't find primary charger ***\n");
		return false;
	}

	chr_err("config is %d\n", info->config);
	if (info->config == DUAL_CHARGERS_IN_SERIES) {
		info->chg2_dev = get_charger_by_name("secondary_chg");
		if (info->chg2_dev)
			chr_err("Found secondary charger\n");
		else {
			chr_err("*** Error : can't find secondary charger ***\n");
			return false;
		}
	} else if (info->config == DIVIDER_CHARGER ||
		   info->config == DUAL_DIVIDER_CHARGERS) {
		info->dvchg1_dev = get_charger_by_name("primary_dvchg");
		if (info->dvchg1_dev)
			chr_err("Found primary divider charger\n");
		else {
			chr_err("*** Error : can't find primary divider charger ***\n");
			return false;
		}
		if (info->config == DUAL_DIVIDER_CHARGERS) {
			info->dvchg2_dev =
				get_charger_by_name("secondary_dvchg");
			if (info->dvchg2_dev)
				chr_err("Found secondary divider charger\n");
			else {
				chr_err("*** Error : can't find secondary divider charger ***\n");
				return false;
			}
		}
	}

	chr_err("register chg1 notifier %d %d\n",
		info->chg1_dev != NULL, info->algo.do_event != NULL);
	if (info->chg1_dev != NULL && info->algo.do_event != NULL) {
		chr_err("register chg1 notifier done\n");
		info->chg1_nb.notifier_call = info->algo.do_event;
		register_charger_device_notifier(info->chg1_dev,
						&info->chg1_nb);
		charger_dev_set_drvdata(info->chg1_dev, info);
	}

	chr_err("register dvchg chg1 notifier %d %d\n",
		info->dvchg1_dev != NULL, info->algo.do_dvchg1_event != NULL);
	if (info->dvchg1_dev != NULL && info->algo.do_dvchg1_event != NULL) {
		chr_err("register dvchg chg1 notifier done\n");
		info->dvchg1_nb.notifier_call = info->algo.do_dvchg1_event;
		register_charger_device_notifier(info->dvchg1_dev,
						&info->dvchg1_nb);
		charger_dev_set_drvdata(info->dvchg1_dev, info);
	}

	chr_err("register dvchg chg2 notifier %d %d\n",
		info->dvchg2_dev != NULL, info->algo.do_dvchg2_event != NULL);
	if (info->dvchg2_dev != NULL && info->algo.do_dvchg2_event != NULL) {
		chr_err("register dvchg chg2 notifier done\n");
		info->dvchg2_nb.notifier_call = info->algo.do_dvchg2_event;
		register_charger_device_notifier(info->dvchg2_dev,
						 &info->dvchg2_nb);
		charger_dev_set_drvdata(info->dvchg2_dev, info);
	}

	return true;
}

static int mtk_charger_plug_out(struct mtk_charger *info)
{
	struct charger_data *pdata1 = &info->chg_data[CHG1_SETTING];
	struct charger_data *pdata2 = &info->chg_data[CHG2_SETTING];
	struct chg_alg_device *alg;
	struct chg_alg_notify notify;
	int i;

	chr_err("%s\n", __func__);
	info->chr_type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->charger_thread_polling = false;
	info->pd_reset = false;
	info->hvdcp_disable = false;

	pdata1->disable_charging_count = 0;
	pdata1->input_current_limit_by_aicl = -1;
	pdata2->disable_charging_count = 0;

	notify.evt = EVT_PLUG_OUT;
	notify.value = 0;
	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		chg_alg_notifier_call(alg, &notify);
	}
	memset(&info->sc.data, 0, sizeof(struct scd_cmd_param_t_1));
	wakeup_sc_algo_cmd(&info->sc.data, SC_EVENT_PLUG_OUT, 0);
	charger_dev_set_input_current(info->chg1_dev, 100000);
	charger_dev_set_mivr(info->chg1_dev, info->data.min_charger_voltage);
	charger_dev_plug_out(info->chg1_dev);

	return 0;
}

static int mtk_charger_plug_in(struct mtk_charger *info,
				int chr_type)
{
	struct chg_alg_device *alg;
	struct chg_alg_notify notify;
	int i, vbat;

	chr_debug("%s\n",
		__func__);

	info->chr_type = chr_type;
	info->usb_type = get_usb_type(info);
	info->charger_thread_polling = true;

	info->can_charging = true;
	/* info->enable_dynamic_cv = true; */
	info->safety_timeout = false;
	info->vbusov_stat = false;
	info->old_cv = 0;
	info->stop_6pin_re_en = false;
	info->batpro_done = false;

	chr_err("mtk_is_charger_on plug in, type:%d\n", chr_type);

	vbat = get_battery_voltage(info);

	notify.evt = EVT_PLUG_IN;
	notify.value = 0;
	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		chg_alg_notifier_call(alg, &notify);
		chg_alg_set_prop(alg, ALG_REF_VBAT, vbat);
	}

	memset(&info->sc.data, 0, sizeof(struct scd_cmd_param_t_1));
	info->sc.disable_in_this_plug = false;
	wakeup_sc_algo_cmd(&info->sc.data, SC_EVENT_PLUG_IN, 0);
	charger_dev_set_input_current(info->chg1_dev,
				info->data.usb_charger_current);
	charger_dev_plug_in(info->chg1_dev);

	return 0;
}


static bool mtk_is_charger_on(struct mtk_charger *info)
{
	int chr_type;

	chr_type = get_charger_type(info);
	if (chr_type == POWER_SUPPLY_TYPE_UNKNOWN) {
		if (info->chr_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			mtk_charger_plug_out(info);
			mutex_lock(&info->cable_out_lock);
			info->cable_out_cnt = 0;
			mutex_unlock(&info->cable_out_lock);
		}
	} else {
		if (info->chr_type == POWER_SUPPLY_TYPE_UNKNOWN)
			mtk_charger_plug_in(info, chr_type);
		else {
			info->chr_type = chr_type;
			info->usb_type = get_usb_type(info);
		}

		if (info->cable_out_cnt > 0) {
			mtk_charger_plug_out(info);
			mtk_charger_plug_in(info, chr_type);
			mutex_lock(&info->cable_out_lock);
			info->cable_out_cnt = 0;
			mutex_unlock(&info->cable_out_lock);
		}
	}

	if (chr_type == POWER_SUPPLY_TYPE_UNKNOWN)
		return false;

	return true;
}

#ifndef OPLUS_FEATURE_CHG_BASIC
static void charger_send_kpoc_uevent(struct mtk_charger *info)
{
	static bool first_time = true;
	ktime_t ktime_now;

	if (first_time) {
		info->uevent_time_check = ktime_get();
		first_time = false;
	} else {
		ktime_now = ktime_get();
		if ((ktime_ms_delta(ktime_now, info->uevent_time_check) / 1000) >= 60) {
			mtk_chgstat_notify(info);
			info->uevent_time_check = ktime_now;
		}
	}
}

static void kpoc_power_off_check(struct mtk_charger *info)
{
	unsigned int boot_mode = info->bootmode;
	int vbus = 0;

	/* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
	/* 9 = LOW_POWER_OFF_CHARGING_BOOT */
	if (boot_mode == 8 || boot_mode == 9) {
		vbus = get_vbus(info);
		if (vbus >= 0 && vbus < 2500 && !mtk_is_charger_on(info) && !info->pd_reset) {
			chr_err("Unplug Charger/USB in KPOC mode, vbus=%d, shutdown\n", vbus);
			while (1) {
				if (info->is_suspend == false) {
					chr_err("%s, not in suspend, shutdown\n", __func__);
					kernel_power_off();
				} else {
					chr_err("%s, suspend! cannot shutdown\n", __func__);
					msleep(20);
				}
			}
		}
		charger_send_kpoc_uevent(info);
	}
}
#endif

static void charger_status_check(struct mtk_charger *info)
{
	union power_supply_propval online, status;
	struct power_supply *chg_psy = NULL;
	int ret;
	bool charging = true;

	chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev,
						       "charger");
	if (IS_ERR_OR_NULL(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &online);

		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_STATUS, &status);

		if (!online.intval)
			charging = false;
		else {
			if (status.intval == POWER_SUPPLY_STATUS_NOT_CHARGING)
				charging = false;
		}
	}
	if (charging != info->is_charging)
		power_supply_changed(info->psy1);
	info->is_charging = charging;
}


static char *dump_charger_type(int chg_type, int usb_type)
{
	switch (chg_type) {
	case POWER_SUPPLY_TYPE_UNKNOWN:
		return "none";
	case POWER_SUPPLY_TYPE_USB:
		if (usb_type == POWER_SUPPLY_USB_TYPE_SDP)
			return "usb";
		else
			return "nonstd";
	case POWER_SUPPLY_TYPE_USB_CDP:
		return "usb-h";
	case POWER_SUPPLY_TYPE_USB_DCP:
		return "std";
	/* case POWER_SUPPLY_TYPE_USB_FLOAT:
		return "nonstd"; */
	default:
		return "unknown";
	}
}

void oplus_usbtemp_set_cc_open(void)
{
	if (pinfo == NULL || pinfo->tcpc == NULL)
		return;

	tcpm_typec_disable_function(pinfo->tcpc, true);
}

void oplus_usbtemp_recover_cc_open(void)
{
	if (pinfo == NULL || pinfo->tcpc == NULL)
		return;

	tcpm_typec_disable_function(pinfo->tcpc, false);
}

static int charger_routine_thread(void *arg)
{
	struct mtk_charger *info = arg;
	unsigned long flags;
	static bool is_module_init_done;
	bool is_charger_on;
	int ret;
	int vbat_min, vbat_max;
	u32 chg_cv = 0;

	while (1) {
		ret = wait_event_interruptible(info->wait_que,
			(info->charger_thread_timeout == true));
		if (ret < 0) {
			chr_err("%s: wait event been interrupted(%d)\n", __func__, ret);
			continue;
		}

		while (is_module_init_done == false) {
			if (charger_init_algo(info) == true)
				is_module_init_done = true;
			else {
				chr_err("charger_init fail\n");
				msleep(5000);
			}
		}

		mutex_lock(&info->charger_lock);
		spin_lock_irqsave(&info->slock, flags);
		if (!info->charger_wakelock->active)
			__pm_stay_awake(info->charger_wakelock);
		spin_unlock_irqrestore(&info->slock, flags);
		info->charger_thread_timeout = false;

		info->battery_temp = get_battery_temperature(info);
		ret = charger_dev_get_adc(info->chg1_dev,
			ADC_CHANNEL_VBAT, &vbat_min, &vbat_max);
		ret = charger_dev_get_constant_voltage(info->chg1_dev, &chg_cv);

		if (vbat_min != 0)
			vbat_min = vbat_min / 1000;

		chr_err("Vbat=%d vbats=%d vbus:%d ibus:%d I=%d T=%d uisoc:%d type:%s>%s pd:%d swchg_ibat:%d cv:%d\n",
			get_battery_voltage(info),
			vbat_min,
			get_vbus(info),
			get_ibus(info),
			get_battery_current(info),
			info->battery_temp,
			get_uisoc(info),
			dump_charger_type(info->chr_type, info->usb_type),
			dump_charger_type(get_charger_type(info), get_usb_type(info)),
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
			info->ta_status[info->select_adapter_idx], get_ibat(info), chg_cv);
#else
			info->pd_type, get_ibat(info), chg_cv);
#endif

		is_charger_on = mtk_is_charger_on(info);

		if (info->charger_thread_polling == true)
			mtk_charger_start_timer(info);

		check_battery_exist(info);
		check_dynamic_mivr(info);
		charger_check_status(info);
#ifndef OPLUS_FEATURE_CHG_BASIC
		kpoc_power_off_check(info);
#endif

		if (is_disable_charger(info) == false &&
			is_charger_on == true &&
			info->can_charging == true) {
			if (info->algo.do_algorithm)
				info->algo.do_algorithm(info);
			sc_update(info);
			wakeup_sc_algo_cmd(&info->sc.data, SC_EVENT_CHARGING, 0);
			charger_status_check(info);
		} else {
			chr_debug("disable charging %d %d %d\n",
			    is_disable_charger(info), is_charger_on, info->can_charging);
			sc_update(info);
			wakeup_sc_algo_cmd(&info->sc.data, SC_EVENT_STOP_CHARGING, 0);
		}

		spin_lock_irqsave(&info->slock, flags);
		__pm_relax(info->charger_wakelock);
		spin_unlock_irqrestore(&info->slock, flags);
		chr_debug("%s end , %d\n",
			__func__, info->charger_thread_timeout);
		mutex_unlock(&info->charger_lock);
	}

	return 0;
}


#ifdef CONFIG_PM
static int charger_pm_event(struct notifier_block *notifier,
			unsigned long pm_event, void *unused)
{
	ktime_t ktime_now;
	struct timespec64 now;
	struct mtk_charger *info;

	info = container_of(notifier,
		struct mtk_charger, pm_notifier);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		info->is_suspend = true;
		chr_debug("%s: enter PM_SUSPEND_PREPARE\n", __func__);
		break;
	case PM_POST_SUSPEND:
		info->is_suspend = false;
		chr_debug("%s: enter PM_POST_SUSPEND\n", __func__);
		ktime_now = ktime_get_boottime();
		now = ktime_to_timespec64(ktime_now);

		if (timespec64_compare(&now, &info->endtime) >= 0 &&
			info->endtime.tv_sec != 0 &&
			info->endtime.tv_nsec != 0) {
			chr_err("%s: alarm timeout, wake up charger\n",
				__func__);
			__pm_relax(info->charger_wakelock);
			info->endtime.tv_sec = 0;
			info->endtime.tv_nsec = 0;
			_wake_up_charger(info);
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}
#endif /* CONFIG_PM */

static enum alarmtimer_restart
	mtk_charger_alarm_timer_func(struct alarm *alarm, ktime_t now)
{
	struct mtk_charger *info =
	container_of(alarm, struct mtk_charger, charger_timer);
	ktime_t *time_p = info->timer_cb_duration;

	info->timer_cb_duration[0] = ktime_get_boottime();
	if (info->is_suspend == false) {
		chr_debug("%s: not suspend, wake up charger\n", __func__);
		info->timer_cb_duration[1] = ktime_get_boottime();
		_wake_up_charger(info);
		info->timer_cb_duration[6] = ktime_get_boottime();
	} else {
		chr_debug("%s: alarm timer timeout\n", __func__);
		__pm_stay_awake(info->charger_wakelock);
	}

	info->timer_cb_duration[7] = ktime_get_boottime();

	if (ktime_us_delta(time_p[7], time_p[0]) > 5000)
		chr_err("delta_t: %lld %lld %lld %lld %lld %lld %lld (%lld)\n",
			ktime_us_delta(time_p[1], time_p[0]),
			ktime_us_delta(time_p[2], time_p[1]),
			ktime_us_delta(time_p[3], time_p[2]),
			ktime_us_delta(time_p[4], time_p[3]),
			ktime_us_delta(time_p[5], time_p[4]),
			ktime_us_delta(time_p[6], time_p[5]),
			ktime_us_delta(time_p[7], time_p[6]),
			ktime_us_delta(time_p[7], time_p[0]));

	return ALARMTIMER_NORESTART;
}

static void mtk_charger_init_timer(struct mtk_charger *info)
{
	alarm_init(&info->charger_timer, ALARM_BOOTTIME,
			mtk_charger_alarm_timer_func);
	mtk_charger_start_timer(info);

#ifdef CONFIG_PM
	if (register_pm_notifier(&info->pm_notifier))
		chr_err("%s: register pm failed\n", __func__);
#endif /* CONFIG_PM */
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void mtk_charger_setup_files(struct platform_device *pdev)
#else
static int mtk_charger_setup_files(struct platform_device *pdev)
#endif
{
	int ret = 0;
	struct proc_dir_entry *battery_dir = NULL, *entry = NULL;
	struct mtk_charger *info = platform_get_drvdata(pdev);

	ret = device_create_file(&(pdev->dev), &dev_attr_sw_jeita);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_chr_type);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_enable_meta_current_limit);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_fast_chg_indicator);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_vbat_mon);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_Pump_Express);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_ADC_Charger_Voltage);
	if (ret)
		goto _out;
	ret = device_create_file(&(pdev->dev), &dev_attr_ADC_Charging_Current);
	if (ret)
		goto _out;
	ret = device_create_file(&(pdev->dev), &dev_attr_input_current);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_charger_log_level);
	if (ret)
		goto _out;

	/* Battery warning */
	ret = device_create_file(&(pdev->dev), &dev_attr_BatteryNotify);
	if (ret)
		goto _out;

	/* sysfs node */
	ret = device_create_file(&(pdev->dev), &dev_attr_enable_sc);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_sc_stime);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_sc_etime);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_sc_tuisoc);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_sc_ibat_limit);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_charger_subboard_ntc);
	if (ret)
		goto _out;
	ret = device_create_file(&(pdev->dev), &dev_attr_charger_ic_ntc);
	if (ret)
		goto _out;

	battery_dir = proc_mkdir("mtk_battery_cmd", NULL);
	if (!battery_dir) {
		chr_err("%s: mkdir /proc/mtk_battery_cmd failed\n", __func__);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
		return -ENOMEM;
#else
		return;
#endif
	}

	entry = proc_create_data("current_cmd", 0644, battery_dir,
			&mtk_chg_current_cmd_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}
	entry = proc_create_data("en_power_path", 0644, battery_dir,
			&mtk_chg_en_power_path_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}
	entry = proc_create_data("en_safety_timer", 0644, battery_dir,
			&mtk_chg_en_safety_timer_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}
	entry = proc_create_data("set_cv", 0644, battery_dir,
			&mtk_chg_set_cv_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#else
        return;
#endif

fail_procfs:
	remove_proc_subtree("mtk_battery_cmd", NULL);
_out:

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return ret;
#else
	return;
#endif
}

void mtk_charger_get_atm_mode(struct mtk_charger *info)
{
	char atm_str[64] = {0};
	char *ptr = NULL, *ptr_e = NULL;
	char keyword[] = "androidboot.atm=";
	int size = 0;

	ptr = strstr(chg_get_cmd(), keyword);
	if (ptr != 0) {
		ptr_e = strstr(ptr, " ");
		if (ptr_e == 0)
			goto end;

		size = ptr_e - (ptr + strlen(keyword));
		if (size <= 0)
			goto end;
		strncpy(atm_str, ptr + strlen(keyword), size);
		atm_str[size] = '\0';
		chr_err("%s: atm_str: %s\n", __func__, atm_str);

		if (!strncmp(atm_str, "enable", strlen("enable")))
			info->atm_enabled = true;
	}
end:
	chr_err("%s: atm_enabled = %d\n", __func__, info->atm_enabled);
}

static int psy_charger_property_is_writeable(struct power_supply *psy,
					       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		return 1;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return 1;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		return 0;
	}
}
#define SUSPEND_RECOVERY_DELAY_MS 2000
static void oplus_charger_suspend_recovery_work(struct work_struct *work)
{
	chg_info("voted suspend recovery, unsuspend\n");
	oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
	oplus_chg_suspend_charger(false, USB_IBUS_DRAW_VOTER);
	oplus_chg_suspend_charger(false, TCPC_IBUS_DRAW_VOTER);
}

#ifdef OPLUS_FEATURE_CHG_BASIC
#if IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
static bool is_wired_topic_available(struct mtk_charger *chip)
{
	if (!chip->wired_topic)
		chip->wired_topic = oplus_mms_get_by_name("wired");
	return !!chip->wired_topic;
}
#endif

static bool is_err_topic_available(struct mtk_charger *chip)
{
	if (!chip->err_topic)
		chip->err_topic = oplus_mms_get_by_name("error");
	return !!chip->err_topic;
}
static void oplus_publish_close_cp_item_work(struct work_struct *work)
{
	struct mtk_charger *chip = container_of(work, struct mtk_charger, publish_close_cp_item_work.work);
	struct mms_msg *msg;
	int rc;
	if (!is_err_topic_available(chip)) {
		chg_err("error topic not found\n");
		return;
	}
	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_CLOSE_CP, 1);
	if (msg == NULL) {
		chg_err("alloc close cp msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->err_topic, msg);
	if (rc < 0) {
		chg_err("publish close cp msg error, rc=%d\n", rc);
		kfree(msg);
	}
}
#endif
static enum power_supply_property charger_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
};

static int psy_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_charger *info;
	struct charger_device *chg;
#ifndef OPLUS_FEATURE_CHG_BASIC
	struct charger_data *pdata;
#endif
	int ret = 0;
	struct chg_alg_device *alg = NULL;

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);

	chr_err("%s psp:%d\n",
		__func__, psp);


	if (info->psy1 != NULL &&
		info->psy1 == psy)
		chg = info->chg1_dev;
	else if (info->psy2 != NULL &&
		info->psy2 == psy)
		chg = info->chg2_dev;
	else if (info->psy_dvchg1 != NULL && info->psy_dvchg1 == psy)
		chg = info->dvchg1_dev;
	else if (info->psy_dvchg2 != NULL && info->psy_dvchg2 == psy)
		chg = info->dvchg2_dev;
	else {
		chr_err("%s fail\n", __func__);
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (chg == info->dvchg1_dev) {
			val->intval = false;
			alg = get_chg_alg_by_name("pe5");
			if (alg == NULL)
				chr_err("get pe5 fail\n");
			else {
				ret = chg_alg_is_algo_ready(alg);
				if (ret == ALG_RUNNING)
					val->intval = true;
			}
			break;
		}

		val->intval = is_charger_exist(info);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (chg != NULL)
			val->intval = true;
		else
			val->intval = false;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = info->enable_hv_charging;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_vbus(info);
		break;
	case POWER_SUPPLY_PROP_TEMP:
#ifndef OPLUS_FEATURE_CHG_BASIC
		if (chg == info->chg1_dev)
			val->intval =
				info->chg_data[CHG1_SETTING].junction_temp_max * 10;
		else if (chg == info->chg2_dev)
			val->intval =
				info->chg_data[CHG2_SETTING].junction_temp_max * 10;
		else if (chg == info->dvchg1_dev) {
			pdata = &info->chg_data[DVCHG1_SETTING];
			val->intval = pdata->junction_temp_max;
		} else if (chg == info->dvchg2_dev) {
			pdata = &info->chg_data[DVCHG2_SETTING];
			val->intval = pdata->junction_temp_max;
		} else
			val->intval = -127;
#else
		if (pinfo->chargeric_temp_chan)
			val->intval = oplus_get_chargeric_temp();
		else if (pinfo->cp_btb_temp_share)
			val->intval = oplus_chg_get_battery_btb_temp_cal();
		else
			val->intval = oplus_get_master_cp_temp();
#endif
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = get_charger_charging_current(info, chg);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = get_charger_input_current(info, chg);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = info->chr_type;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_BOOT:
		val->intval = get_charger_zcv(info, chg);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int psy_charger_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	struct mtk_charger *info;
	int idx;

	chr_err("%s: prop:%d %d\n", __func__, psp, val->intval);

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);

	if (info->psy1 != NULL &&
		info->psy1 == psy)
		idx = CHG1_SETTING;
	else if (info->psy2 != NULL &&
		info->psy2 == psy)
		idx = CHG2_SETTING;
	else if (info->psy_dvchg1 != NULL && info->psy_dvchg1 == psy)
		idx = DVCHG1_SETTING;
	else if (info->psy_dvchg2 != NULL && info->psy_dvchg2 == psy)
		idx = DVCHG2_SETTING;
	else {
		chr_err("%s fail\n", __func__);
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (val->intval > 0)
			info->enable_hv_charging = true;
		else
			info->enable_hv_charging = false;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		info->chg_data[idx].thermal_charging_current_limit =
			val->intval;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		info->chg_data[idx].thermal_input_current_limit =
			val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		if (oplus_chg_get_common_charge_icl_support_flags()) {
			if (val->intval > 0) {
				cancel_delayed_work_sync(&info->charger_suspend_recovery_work);
				oplus_chg_suspend_charger(true, USB_IBUS_DRAW_VOTER);
				schedule_delayed_work(&info->charger_suspend_recovery_work,
				                      msecs_to_jiffies(SUSPEND_RECOVERY_DELAY_MS));
			} else {
				oplus_chg_suspend_charger(false, USB_IBUS_DRAW_VOTER);
			}
		}
		break;
	default:
		return -EINVAL;
	}
	_wake_up_charger(info);

	return 0;
}

static void mtk_charger_external_power_changed(struct power_supply *psy)
{
	struct mtk_charger *info;
	union power_supply_propval prop = { 0 };
	union power_supply_propval prop2 = { 0 };
	struct power_supply *chg_psy = NULL;
	int ret;

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);
	chg_psy = info->chg_psy;

	if (IS_ERR_OR_NULL(chg_psy)) {
		chg_err("%s Couldn't get chg_psy\n", __func__);
	chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev,
						       "charger");
		info->chg_psy = chg_psy;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop2);
	}

	chg_info("%s event, name:%s online:%d type:%d vbus:%d\n", __func__,
		psy->desc->name, prop.intval, prop2.intval,
		get_vbus(info));

#ifdef OPLUS_FEATURE_CHG_BASIC
	pinfo = info;
	oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
#endif

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	oplus_set_hvdcp_flag_clear();
#endif

	_wake_up_charger(info);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
#if !IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
static void hvdcp_detect_dn_check(struct mtk_charger *info)
{
	union power_supply_propval prop = {0}, prop2 = {0};
	struct power_supply *chg_psy = NULL;
	int ret;

	chg_psy = info->chg_psy;

	if (IS_ERR_OR_NULL(chg_psy)) {
		chg_err("%s Couldn't get chg_psy\n", __func__);
		chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev, "charger");
		info->chg_psy = chg_psy;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop2);
	}

	chg_info("%s event online:%d type:%d vbus:%d\n", __func__,
		prop.intval, prop2.intval,
		get_vbus(info));

	oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	oplus_set_hvdcp_flag_clear();
#endif

	_wake_up_charger(info);
}
#endif
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
int notify_adapter_event(struct notifier_block *notifier,
			unsigned long evt, void *val)
{
	struct mtk_charger *pinfo = NULL;
	u32 boot_mode = 0;
	bool report_psy = true;
	int index = 0;
	struct info_notifier_block *ta_nb;

	ta_nb = container_of(notifier, struct info_notifier_block, nb);
	pinfo = ta_nb->info;
	index = ta_nb - pinfo->ta_nb;
	chr_err("%s %lu, %d\n", __func__, evt, index);
	boot_mode = pinfo->bootmode;

	switch (evt) {
	case TA_DETACH:
		mutex_lock(&pinfo->ta_lock);
		chr_err("TA Notify Detach\n");
		pinfo->ta_status[index] = TA_DETACH;
		mutex_unlock(&pinfo->ta_lock);
		mtk_chg_alg_notify_call(pinfo, EVT_DETACH, 0);
		_wake_up_charger(pinfo);
		/* reset PE40 */
		break;

	case TA_ATTACH:
		mutex_lock(&pinfo->ta_lock);
		chr_err("TA Notify Attach\n");
		pinfo->ta_status[index] = TA_ATTACH;
		mutex_unlock(&pinfo->ta_lock);
		_wake_up_charger(pinfo);
		/* reset PE40 */
		break;

	case TA_DETECT_FAIL:
		mutex_lock(&pinfo->ta_lock);
		chr_err("TA Notify Detect Fail\n");
		pinfo->ta_status[index] = TA_DETECT_FAIL;
		mutex_unlock(&pinfo->ta_lock);
		_wake_up_charger(pinfo);
		/* reset PE50 */
		break;

	case TA_HARD_RESET:
		mutex_lock(&pinfo->ta_lock);
		chr_err("TA Notify Hard Reset\n");
		pinfo->ta_status[index] = TA_HARD_RESET;
		mutex_unlock(&pinfo->ta_lock);
		_wake_up_charger(pinfo);
		/* PD is ready */
		break;

	case TA_SOFT_RESET:
		mutex_lock(&pinfo->ta_lock);
		chr_err("TA Notify Soft Reset\n");
		pinfo->ta_status[index] = TA_SOFT_RESET;
		mutex_unlock(&pinfo->ta_lock);
		_wake_up_charger(pinfo);
		/* PD30 is ready */
		break;
	case MTK_TYPEC_WD_STATUS:
		chr_err("wd status = %d\n", *(bool *)val);
		pinfo->water_detected = *(bool *)val;
		if (pinfo->water_detected == true) {
			pinfo->notify_code |= CHG_TYPEC_WD_STATUS;
			pinfo->record_water_detected = true;
			if (boot_mode == 8 || boot_mode == 9)
				pinfo->enable_hv_charging = false;
		} else {
			pinfo->notify_code &= ~CHG_TYPEC_WD_STATUS;
			if (boot_mode == 8 || boot_mode == 9)
				pinfo->enable_hv_charging = true;
		}
		mtk_chgstat_notify(pinfo);
		report_psy = boot_mode == 8 || boot_mode == 9;
		break;
	}
	chr_debug("%s: evt: pd:%d, ufcs:%d\n", __func__,
	pinfo->ta_status[PD], pinfo->ta_status[UFCS]);
		/* adapter_control */
	if (report_psy)
		power_supply_changed(pinfo->psy1);
	return NOTIFY_DONE;
}
#else
int notify_adapter_event(struct notifier_block *notifier,
			unsigned long evt, void *val)
{
	struct mtk_charger *pinfo = NULL;

	chr_err("%s %lu\n", __func__, evt);

	pinfo = container_of(notifier,
		struct mtk_charger, pd_nb);

	switch (evt) {
	case  MTK_PD_CONNECT_NONE:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify Detach\n");
		pinfo->pd_type = MTK_PD_CONNECT_NONE;
		pinfo->pd_reset = false;
		mutex_unlock(&pinfo->pd_lock);
		mtk_chg_alg_notify_call(pinfo, EVT_DETACH, 0);
		/* reset PE40 */
		break;

	case MTK_PD_CONNECT_HARD_RESET:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify HardReset\n");
		pinfo->pd_type = MTK_PD_CONNECT_NONE;
		pinfo->pd_reset = true;
		mutex_unlock(&pinfo->pd_lock);
		mtk_chg_alg_notify_call(pinfo, EVT_HARDRESET, 0);
		_wake_up_charger(pinfo);
		/* reset PE40 */
		break;

	case MTK_PD_CONNECT_PE_READY_SNK:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify fixe voltage ready\n");
		pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK;
		pinfo->pd_reset = false;
		mutex_unlock(&pinfo->pd_lock);
		/* PD is ready */
		break;

	case MTK_PD_CONNECT_PE_READY_SNK_PD30:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify PD30 ready\r\n");
		pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK_PD30;
		pinfo->pd_reset = false;
		mutex_unlock(&pinfo->pd_lock);
		/* PD30 is ready */
		break;

	case MTK_PD_CONNECT_PE_READY_SNK_APDO:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify APDO Ready\n");
		pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK_APDO;
		pinfo->pd_reset = false;
		mutex_unlock(&pinfo->pd_lock);
		/* PE40 is ready */
		_wake_up_charger(pinfo);
		break;

	case MTK_PD_CONNECT_TYPEC_ONLY_SNK:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify Type-C Ready\n");
		pinfo->pd_type = MTK_PD_CONNECT_TYPEC_ONLY_SNK;
		pinfo->pd_reset = false;
		mutex_unlock(&pinfo->pd_lock);
		/* type C is ready */
		_wake_up_charger(pinfo);
		break;
	case MTK_TYPEC_WD_STATUS:
		chr_err("wd status = %d\n", *(bool *)val);
		pinfo->water_detected = *(bool *)val;
		if (pinfo->water_detected == true)
			pinfo->notify_code |= CHG_TYPEC_WD_STATUS;
		else
			pinfo->notify_code &= ~CHG_TYPEC_WD_STATUS;
		mtk_chgstat_notify(pinfo);
		break;
#ifdef OPLUS_FEATURE_CHG_BASIC
	default:
		break;
#endif
	}

	return NOTIFY_DONE;
}
#endif

int chg_alg_event(struct notifier_block *notifier,
			unsigned long event, void *data)
{
	chr_err("evt:%ld\n", event);

	return NOTIFY_DONE;
}

#if !IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
static void mtk_tcpc_set_otg_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct mtk_charger *chip;
	if (!ic_dev) {
		chr_err("ic_dev NULL\n");
		return;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip) {
		chg_err("chip is NULL\n");
		return;
	}

	if (chip->otg_enable == en)
		return;

	chip->otg_enable = en;
	chip->reverse_enable = en;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_OTG_ENABLE);
	if (chip->reverse_chg_ic_dev)
		oplus_chg_ic_virq_trigger(chip->reverse_chg_ic_dev, OPLUS_IC_VIRQ_REVERSE_ENABLE);
}

static void mtk_tcpc_set_high_reverse_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct mtk_charger *chip;
	if (!ic_dev) {
		chg_err("ic_dev is NULL\n");
		return;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip) {
		chg_err("chip is NULL\n");
		return;
	}
	if (chip->high_reverse_enable == en)
		return;

	chip->high_reverse_enable = en;
	if (chip->reverse_chg_ic_dev)
		oplus_chg_ic_virq_trigger(chip->reverse_chg_ic_dev, OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE);
}
#endif

#define OPLUS_SVID 0x22D9
#define TCPM_SUCCESS 0
#define DISCOVER_SVID_MAX_RETRIES 3
#define DISCOVER_SVID_RETRY_DELAY 50
#define DISCOVER_SVID_INTERNAL_CMD_DELAY 10

static int oplus_get_pd_partner_svids(struct tcpc_device *tcpc_dev)
{
	int i = 0;
	int ret = 0;
	struct tcpm_svid_list svid_list = {0, {0}};
	int chg_type = oplus_wired_get_chg_type();

	ret = tcpm_inquire_pd_partner_svids(tcpc_dev, &svid_list);
	if (ret == TCPM_SUCCESS) {
		for (i = 0; i < svid_list.cnt; i++) {
			chg_info("svid[%d] = 0x%x\n", i, svid_list.svids[i]);
			if (svid_list.svids[i] == OPLUS_SVID) {
				if (chg_type == OPLUS_CHG_USB_TYPE_PD_SDP) {
					pinfo->reverse_chg_svid = OPLUS_SVID;
					pinfo->oplus_pd_sdp_svid = true;
					chg_info("get svid is oplus src device\n");
				} else {
					pinfo->pd_svooc = true;
					chg_info("get match svid and this is oplus adapter\n");
				}
				break;
			}
		}
	}
	return ret;
}

static int oplus_get_pd_partner_inform(struct tcpc_device *tcpc_dev)
{
	int ret = 0;
	uint32_t data[VDO_MAX_NR] = {0};
	int chg_type = oplus_wired_get_chg_type();

	ret = tcpm_inquire_pd_partner_inform(tcpc_dev, data);
	if (ret == TCPM_SUCCESS && (data[0] & 0xFFFF) == OPLUS_SVID) {
		if (chg_type == OPLUS_CHG_USB_TYPE_PD_SDP) {
			pinfo->reverse_chg_svid = OPLUS_SVID;
			pinfo->oplus_pd_sdp_svid = true;
			chg_info("get svid is oplus src device\n");
		} else {
			pinfo->pd_svooc = true;
			chg_info("get match svid and this is oplus adapter\n");
		}
	} else {
		chg_err("get the partner infom failed, ret = %d\n", ret);
	}
	return ret;
}

static int oplus_discover_id(struct tcpc_device *tcpc_dev)
{
	int ret = 0;

	ret = tcpm_dpm_vdm_discover_id(tcpc_dev, NULL);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))  && !IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
	if (ret == TCP_DPM_RET_NOT_SUPPORT ||
	    ret == TCP_DPM_RET_DENIED_WRONG_ROLE ||
	    ret == TCPM_ERROR_PUT_EVENT) {
#else
	if (ret == TCP_DPM_RET_NOT_SUPPORT ||
	    ret == TCPM_ERROR_PUT_EVENT) {
#endif
		chg_err(" failed, ret = %d.\n", ret);
		return -EFAULT;
	} else if (ret != TCPM_SUCCESS) {
		chg_err("failed to discover id, ret = %d\n", ret);
	}

	return ret;
}

static int oplus_discover_svid(struct tcpc_device *tcpc_dev)
{
	int ret = 0;

	/* Note: the WC065A11JCH not support the cmd of discover_svid. */
	ret = tcpm_dpm_vdm_discover_svid(tcpc_dev, NULL);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) && !IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
	if (ret == TCP_DPM_RET_DENIED_WRONG_ROLE ||
	    ret == TCPM_ERROR_PUT_EVENT) {
#else
	if (ret == TCPM_ERROR_PUT_EVENT) {
#endif
		chg_err(" failed, ret = %d.\n", ret);
		return -EFAULT;
	} else if (ret != TCPM_SUCCESS) {
		chg_err("failed to discover svid,ret = %d\n", ret);
	}

	return ret;
}

static int oplus_get_adapter_svid(void)
{
	int ret = 0;
	int discover_svid_ret;
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	int disc_svid_retries = 0;

	if (tcpc_dev == NULL) {
		chg_err("tcpc_dev is null return\n");
		return -1;
	}

	if (pinfo->pd_svooc)
		goto trigger_irq;

	do {
		disc_svid_retries++;
		if (disc_svid_retries > DISCOVER_SVID_MAX_RETRIES)
			break;

		ret = oplus_discover_id(tcpc_dev);
		if (ret == -EFAULT) {
			chg_err("vdm_discover_id failed, ret = %d, retries: %d, not try again.\n",
				ret, disc_svid_retries);
			break;
		} else if (ret != TCPM_SUCCESS) {
			chg_err("failed to discover id,  disc_svid_retries: %d, ret = %d\n",
				disc_svid_retries, ret);
			mdelay(DISCOVER_SVID_RETRY_DELAY);
			continue;
		}

		mdelay(DISCOVER_SVID_INTERNAL_CMD_DELAY);
		discover_svid_ret = oplus_discover_svid(tcpc_dev);
		if (discover_svid_ret == -EFAULT) {
			chg_err("get the svid failed, ret = %d, retries: %d, not try again.\n",
				discover_svid_ret, disc_svid_retries);
			break;
		} else if (ret != TCPM_SUCCESS) {
			chg_err("Failed to discover svid. ret %d retries: %d\n",
				discover_svid_ret, disc_svid_retries);
		}
		mdelay(DISCOVER_SVID_INTERNAL_CMD_DELAY);

		ret = oplus_get_pd_partner_svids(tcpc_dev);
		if (ret == TCPM_SUCCESS) {
			goto trigger_irq;
		} else {
			chg_err("get the pd partner svids failed, ret = %d, retries = %d\n",
				ret, disc_svid_retries);

			/* Note: the WC065A11JCH maybe not support the get_pd_partner_svids */
			if (discover_svid_ret == TCP_DPM_RET_NOT_SUPPORT)
				chg_info("not support to get_pd_partner_svids, ret = %d, discover_svid_ret = %d\n",
					 ret, discover_svid_ret);
			mdelay(DISCOVER_SVID_RETRY_DELAY);
		}

		/* retry to get the SVID by PD partner inform. */
		mdelay(DISCOVER_SVID_INTERNAL_CMD_DELAY);
		ret = oplus_get_pd_partner_inform(tcpc_dev);
		if (ret == TCPM_SUCCESS) {
			goto trigger_irq;
		} else {
			chg_err("get the partner infom failed, ret = %d, retries = %d.\n",
				ret, disc_svid_retries);

			/* discover svid return not support and get pd_partner_inform failed, not try again.*/
			if (discover_svid_ret == TCP_DPM_RET_NOT_SUPPORT) {
				chg_info("not support to get the svid, ret = %d, discover_svid_ret = %d\n",
					  ret, discover_svid_ret);
				goto trigger_irq;
			}
			mdelay(DISCOVER_SVID_RETRY_DELAY);
		}
	} while (ret != TCPM_SUCCESS && disc_svid_retries < DISCOVER_SVID_MAX_RETRIES);

trigger_irq:
	oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_SVID);

	return 0;
}

static void oplus_svid_check_work(struct work_struct *work)
{
	oplus_get_adapter_svid();
}

static void oplus_reverse_chg_svid_check_work(struct work_struct *work)
{
	int ret = 0;
	struct mtk_charger *chip = container_of(work,
		struct mtk_charger, reverse_chg_svid_check_work.work);
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	int disc_svid_retries = 0;
	uint32_t data[VDO_MAX_NR] = {0};

	if (tcpc_dev == NULL) {
		chg_err("tcpc_dev is null, return\n");
		return;
	}

	do {
		disc_svid_retries++;
		if (disc_svid_retries > DISCOVER_SVID_MAX_RETRIES)
			break;

		ret = oplus_discover_id(tcpc_dev);
		if (ret == -EFAULT) {
			chg_err("oplus_discover_id failed, ret = %d, retries: %d, not try again.\n",
				ret, disc_svid_retries);
			break;
		} else if (ret != TCPM_SUCCESS) {
			chg_err("oplus_discover_id not success,  disc_svid_retries: %d, ret = %d\n",
				disc_svid_retries, ret);
			mdelay(DISCOVER_SVID_RETRY_DELAY);
			continue;
		}

		/* retry to get the SVID by PD partner inform. */
		mdelay(DISCOVER_SVID_INTERNAL_CMD_DELAY);

		ret = tcpm_inquire_pd_partner_inform(tcpc_dev, data);
		if (ret == TCPM_SUCCESS) {
			chip->reverse_chg_svid = data[0] & 0xFFFF;
			chg_info("reverse chg get the svid = 0x%04X", chip->reverse_chg_svid);
			break;
		} else if (ret != TCPM_SUCCESS) {
			chg_err("tcpm_inquire_pd_partner_inform failed, ret = %d, retries = %d.\n",
				ret, disc_svid_retries);
			mdelay(DISCOVER_SVID_RETRY_DELAY);
		}
	} while (ret != TCPM_SUCCESS && disc_svid_retries <= DISCOVER_SVID_MAX_RETRIES);
}

#define DEFAULT_PDO_CURR 500 /*500ma*/
#if !IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
static int oplus_get_max_current_from_fixed_pdo(struct mtk_charger *chip, int volt)
{
	int i = 0;
	int chg_type = oplus_wired_get_chg_type();

	if (!oplus_chg_get_common_charge_icl_support_flags())
		return -EINVAL;

	if (chip->pdo[0].pdo_data == 0) {
		if (chg_type == OPLUS_CHG_USB_TYPE_PD ||
			chg_type == OPLUS_CHG_USB_TYPE_PD_DRP ||
			chg_type == OPLUS_CHG_USB_TYPE_PD_PPS ||
			chg_type == OPLUS_CHG_USB_TYPE_PD_SDP) {
			chg_err("get pdo info error. return %dma\n", DEFAULT_PDO_CURR);
			return DEFAULT_PDO_CURR;
		}
		chg_err("get pdo info error\n");
		return -EINVAL;
	}

	for (i = 0; i < (PPS_PDO_MAX - 1); i++) {
		if (chip->pdo[i].pdo_type != USBPD_PDMSG_PDOTYPE_FIXED_SUPPLY)
			continue;

		if (volt <= PD_PDO_VOL(chip->pdo[i].voltage_50mv)) {
			chg_info("SourceCap[%d]: %08X, FixedSupply PDO V=%d mV, I=%d mA,"
				"UsbCommCapable=%d, USBSuspendSupported:%d\n", i,
				chip->pdo[i].pdo_data, PD_PDO_VOL(chip->pdo[i].voltage_50mv),
				PD_PDO_CURR_MAX(chip->pdo[i].max_current_10ma),
				chip->pdo[i].usb_comm_capable, chip->pdo[i].usb_suspend_supported);
			return PD_PDO_CURR_MAX(chip->pdo[i].max_current_10ma);
		}
	}
	return -EINVAL;
}
#else
static int oplus_get_max_current_from_fixed_pdo(struct mtk_charger *chip, int volt)
{
	union mms_msg_data data = { 0 };
	int rc = 0;
	int chg_type = oplus_wired_get_chg_type();

	if (!chip)
		return	-EINVAL;

	if (is_wired_topic_available(chip) &&
		(chg_type == OPLUS_CHG_USB_TYPE_PD ||
		chg_type == OPLUS_CHG_USB_TYPE_PD_DRP ||
		chg_type == OPLUS_CHG_USB_TYPE_PD_PPS ||
		chg_type == OPLUS_CHG_USB_TYPE_PD_SDP)) {
		rc = oplus_mms_get_item_data(chip->wired_topic,
					WIRED_ITEM_CHARGER_CURR_MAX, &data, false);
		if (rc >= 0)
			return data.intval;
		else
			return DEFAULT_PDO_CURR;
	}
	return -EINVAL;
}
#endif

static int mtk_chg_get_source_pdo(struct oplus_chg_ic_dev *ic_dev, u32 *data, int *num)
{
	struct mtk_charger *chip;
	int pdo_index;

	if (!ic_dev) {
		chg_err("ic_dev is NULL\n");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	*num = chip->cap_nr;
	for (pdo_index = 0; pdo_index < chip->cap_nr; pdo_index++) {
		data[pdo_index] = chip->pdo[pdo_index].pdo_data;
	}

	return 0;
}

static void oplus_sourcecap_done_work(struct work_struct *work)
{
	struct mtk_charger *chip = container_of(work,
		struct mtk_charger, sourcecap_done_work.work);
	int max_pdo_current = 0;

	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_POWER_CHANGED);
	/*set default input current from pdo*/
	max_pdo_current = oplus_get_max_current_from_fixed_pdo(chip, VBUS_9V);
	if (max_pdo_current < 0)
		max_pdo_current = oplus_get_max_current_from_fixed_pdo(chip, VBUS_5V);
	if (max_pdo_current >= 0)
		oplus_chg_set_icl_by_vote(max_pdo_current, PD_PDO_ICL_VOTER);
}

#define DEFAULT_CURR_BY_CC 100
#define SINK_SUSPEND_CURRENT 5
#define WAIT_SOURCECPA_DOWN 1000
#if !IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
static int pd_tcp_notifier_call(struct notifier_block *pnb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct mtk_charger *pinfo = NULL;
	int ret = 0;
	int i;

	pinfo = container_of(pnb, struct mtk_charger, pd_nb);

	chg_info("PD charger event:%d %d\n", (int)event,
		(int)noti->pd_state.connected);

	switch (event) {
	case TCP_NOTIFY_SOURCE_VBUS:
		chg_info("pd type:%d. source vbus %dmV %dmA type(0x%02X)\n",
		         pinfo->pd_type, noti->vbus_state.mv, noti->vbus_state.ma, noti->vbus_state.type);
		if (noti->vbus_state.mv == VBUS_5V) {
			mtk_tcpc_set_otg_enable(pinfo->ic_dev, true);
			mtk_tcpc_set_high_reverse_enable(pinfo->ic_dev, false);
			pinfo->power_role = POWER_ROLE_SOURCE;
		} else if (noti->vbus_state.mv == VBUS_9V) {
			mtk_tcpc_set_high_reverse_enable(pinfo->ic_dev, true);
			pinfo->power_role = POWER_ROLE_SOURCE;
		} else {
			mtk_tcpc_set_high_reverse_enable(pinfo->ic_dev, false);
			mtk_tcpc_set_otg_enable(pinfo->ic_dev, false);
		}
		if (pinfo->pre_source_vbus == 0 && noti->vbus_state.mv == VBUS_5V)
			schedule_delayed_work(&pinfo->reverse_chg_svid_check_work,
				msecs_to_jiffies(WAIT_SOURCECPA_DOWN));
		pinfo->pre_source_vbus = noti->vbus_state.mv;
		oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_POWER_ROLE_STATUS);
		break;

	case TCP_NOTIFY_SINK_VBUS:
		chg_info("pd type:%d. sink vbus %dmV %dmA type(0x%02X)\n",
		         pinfo->pd_type, noti->vbus_state.mv, noti->vbus_state.ma, noti->vbus_state.type);
		pinfo->power_role = POWER_ROLE_SINK;
		if (oplus_chg_get_common_charge_icl_support_flags() &&
		    pinfo->pd_type != MTK_PD_CONNECT_NONE &&
		    noti->vbus_state.ma <= SINK_SUSPEND_CURRENT) {
			cancel_delayed_work_sync(&pinfo->charger_suspend_recovery_work);
			oplus_chg_suspend_charger(true, TCPC_IBUS_DRAW_VOTER);
			schedule_delayed_work(&pinfo->charger_suspend_recovery_work,
			                      msecs_to_jiffies(SUSPEND_RECOVERY_DELAY_MS));
		}
		if (noti->vbus_state.ma > SINK_SUSPEND_CURRENT) {
			oplus_chg_suspend_charger(false, TCPC_IBUS_DRAW_VOTER);
		}
		oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_POWER_ROLE_STATUS);
		break;

	case TCP_NOTIFY_PD_STATE:
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			pinfo->pd_type = PD_CONNECT_NONE;
			oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
			pinfo->pd_chg_volt = VBUS_5V;
			chr_err("PD Notify Detach\n");
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			break;

		case PD_CONNECT_HARD_RESET:
			pinfo->pd_type = PD_CONNECT_NONE;
			chr_err("PD Notify HardReset\n");
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			if (pinfo->reverse_chg_ic_dev)
				oplus_chg_ic_virq_trigger(pinfo->reverse_chg_ic_dev, OPLUS_IC_VIRQ_HARD_RESET);
			break;

		case PD_CONNECT_PE_READY_SNK:
			pinfo->pd_type = PD_CONNECT_PE_READY_SNK;
			chr_err("PD Notify fixe voltage ready\n");
			cancel_delayed_work_sync(&pinfo->svid_check_work);
			schedule_delayed_work(&pinfo->svid_check_work, 0);
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_PD_COMPLETED);
			break;

		case PD_CONNECT_PE_READY_SNK_PD30:
			pinfo->pd_type = PD_CONNECT_PE_READY_SNK_PD30;
			chr_err("PD Notify PD30 ready\r\n");
			cancel_delayed_work_sync(&pinfo->svid_check_work);
			schedule_delayed_work(&pinfo->svid_check_work, 0);
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_PD_COMPLETED);
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_POWER_CHANGED);
			break;

		case PD_CONNECT_PE_READY_SNK_APDO:
			pinfo->pd_type = PD_CONNECT_PE_READY_SNK_APDO;
			chr_err("PD Notify APDO Ready\n");
			cancel_delayed_work_sync(&pinfo->svid_check_work);
			schedule_delayed_work(&pinfo->svid_check_work, 0);
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_PD_COMPLETED);
			break;

		case PD_CONNECT_TYPEC_ONLY_SNK_DFT:
			/* fall-through */
		case PD_CONNECT_TYPEC_ONLY_SNK:
			pinfo->pd_type = PD_CONNECT_TYPEC_ONLY_SNK;
			chr_err("PD Notify Type-C Ready\n");
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_SVID);/* not support svid */
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_PD_COMPLETED);
			break;
		}
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		pinfo->source_plug_out = false;
		chr_err("old_state=%d, new_state=%d\n",
				noti->typec_state.old_state,
				noti->typec_state.new_state);
		if (noti->typec_state.new_state == TYPEC_ATTACHED_DBGACC_SNK ||
		    noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC ||
		    noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC) {
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_PD_COMPLETED);
			chr_err("TYPEC_STATE = DBGACC_SNK || CUSTOM_SRC || NORP_SRC\n");
		}
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			power_supply_changed(pinfo->psy1);
			chr_err("adb she-C SRC plug in\n");
		} else if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SNK) {
			chr_err("Type-C SINK plug in\n");
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SRC) &&
			noti->typec_state.new_state == TYPEC_UNATTACHED) {
			pinfo->source_plug_out = true;
			chr_err("Type-C SRC plug out\n");
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SNK) &&
			noti->typec_state.new_state == TYPEC_UNATTACHED) {
			chr_err("Type-C SNK plug out\n");
			schedule_delayed_work(&pinfo->publish_close_cp_item_work, 0);
		}
		pinfo->typec_state = noti->typec_state.new_state;
		oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_TYPEC_STATE);
		break;
	case TCP_NOTIFY_WD_STATUS:
		chr_err("wd status = %d\n", (bool)noti->wd_status.water_detected);
		pinfo->water_detected = (bool)noti->wd_status.water_detected;
		if (pinfo->water_detected == true)
			pinfo->notify_code |= CHG_TYPEC_WD_STATUS;
		else
			pinfo->notify_code &= ~CHG_TYPEC_WD_STATUS;
		mtk_chgstat_notify(pinfo);
		break;
	case TCP_NOTIFY_WD0_STATE:
		pinfo->wd0_detect = noti->wd0_state.wd0;
		pr_err("%s wd0 = %d\n", __func__, noti->wd0_state.wd0);
		oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CC_DETECT);
		break;
	case TCP_NOTIFY_CHRDET_STATE:
		pinfo->chrdet_state = noti->chrdet_state.chrdet;
		pr_err("%s chrdet = %d\n", __func__, noti->chrdet_state.chrdet);

		cancel_delayed_work_sync(&pinfo->detach_clean_work);
		if (noti->chrdet_state.chrdet == 0) {
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_SVID);

			for (i = 0; i < pinfo->cap_nr; i++)
				pinfo->pdo[i].pdo_data = 0;
			oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
			/*fix the one plus charger break bug: delay to set the pd_svooc state*/
			schedule_delayed_work(&pinfo->detach_clean_work, msecs_to_jiffies(1500));
		}
		oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_PLUGIN);
		break;
	case TCP_NOTIFY_CABLE_TYPE:
		if (noti->cable_type.type == TCPC_CABLE_TYPE_A2C) {
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_PD_COMPLETED);
			chr_info("CABLE_TYPE = A to C\n");
		}
		break;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	case TCP_NOTIFY_HVDCP_DETECT_DN:
		chr_err("HVDCP_DETECT_DN = %d\n", (bool)noti->hvdcp_detect.hvdcp_detect_dn);
		hvdcp_detect_dn_check(pinfo);
		break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_PD_SOURCECAP_UPDATE_SUPPORT)
	case TCP_NOTIFY_PD_SOURCECAP_DONE:
		chg_info("PD_SOURCECAP_DONE\n");
		pinfo->cap_nr = (int)noti->caps_msg.caps->nr;
		for (i = 0; i < pinfo->cap_nr; i++) {
			pinfo->pdo[i].pdo_data = (u32)noti->caps_msg.caps->pdos[i];
			chg_info("SourceCap[%d]: %08X\n", i + 1, pinfo->pdo[i].pdo_data);
		}
		if (oplus_chg_get_common_charge_icl_support_flags())
			schedule_delayed_work(&pinfo->sourcecap_done_work, 0);
		break;
#endif
	default:
		break;
	}
	return ret;
}
#endif

#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
static int rtc_reset_check(void)
{
	int rc = 0;
	struct rtc_time tm;
	struct rtc_device *rtc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return 0;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	if ((tm.tm_year == 70) && (tm.tm_mon == 0) && (tm.tm_mday <= 1)) {
		chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  @@@ wday: %d, yday: %d, isdst: %d\n",
			tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
			tm.tm_wday, tm.tm_yday, tm.tm_isdst);
		rtc_class_close(rtc);
		return 1;
	}

	chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  ###  wday: %d, yday: %d, isdst: %d\n",
		tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
		tm.tm_wday, tm.tm_yday, tm.tm_isdst);

close_time:
	rtc_class_close(rtc);
	return 0;
}
#endif /* CONFIG_OPLUS_RTC_DET_SUPPORT */

int mt_power_supply_type_check(void)
{
	int chr_type;
	union power_supply_propval prop;
	struct power_supply *chg_psy = NULL;

	chg_psy = devm_power_supply_get_by_phandle(&pinfo->pdev->dev, "charger");
	if (IS_ERR_OR_NULL(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
	} else {
		power_supply_get_property(chg_psy, POWER_SUPPLY_PROP_ONLINE, &prop);
	}

	chr_type = get_charger_type(pinfo);

	chr_err("charger_type[%d]\n", chr_type);

	return chr_type;
}
EXPORT_SYMBOL(mt_power_supply_type_check);

void mt_usb_connect(void)
{
	return;
}
EXPORT_SYMBOL(mt_usb_connect);

void mt_usb_disconnect(void)
{
	return;
}
EXPORT_SYMBOL(mt_usb_disconnect);

int get_rtc_spare_oplus_fg_value(void)
{
	return 0;
}
EXPORT_SYMBOL(get_rtc_spare_oplus_fg_value);

int set_rtc_spare_oplus_fg_value(int soc)
{
        return 0;
}
EXPORT_SYMBOL(set_rtc_spare_oplus_fg_value);

bool is_meta_mode(void)
{
	if (!pinfo) {
		pr_err("%s: pinfo is null\n", __func__);
		return false;
	}

	pr_err("%s: bootmode = %d\n", __func__, pinfo->bootmode);

	if (pinfo->bootmode == 1)
		return true;
	else
		return false;
}
EXPORT_SYMBOL(is_meta_mode);

int oplus_chg_check_ui_soc_is_ready(void)
{
	if (!pinfo)
		return false;
	else
		return is_gauge_topic_available(pinfo);
}
EXPORT_SYMBOL(oplus_chg_check_ui_soc_is_ready);

int set_chr_enable_otg(unsigned int enable)
{
	return 0;
}
EXPORT_SYMBOL(set_chr_enable_otg);

#ifdef OPLUS_FEATURE_CHG_BASIC

static int oplus_suspend_charger(bool suspend)
{
	struct votable *suspend_votable;
	int rc;

	suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	if (!suspend_votable) {
		chg_err("WIRED_CHARGE_SUSPEND votable not found\n");
		return -EINVAL;
	}

	rc = vote(suspend_votable, PDQC_VOTER, suspend, 0, true);
	if (rc < 0)
		chg_err("%s charger error, rc=%d\n",
			suspend ? "suspend" : "unsuspend", rc);
	else
		chg_info("%s charger\n", suspend ? "suspend" : "unsuspend");

	return rc;
}

static int mtk_chg_gauge_init(struct oplus_chg_ic_dev *ic_dev)
{
	ic_dev->online = true;
	return 0;
}

static int mtk_chg_gauge_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	return 0;
}

static int oplus_mt6375_suspend_charger(struct mtk_charger *info)
{
	charger_dev_enable_powerpath(info->chg1_dev, false);

	return 0;
}

static int oplus_mt6375_unsuspend_charger(struct mtk_charger *info)
{
	charger_dev_enable_powerpath(info->chg1_dev, true);

	return 0;
}

static int oplus_mt6375_enable_charging(struct mtk_charger *info)
{
	int rc = 0;

	rc = charger_dev_enable(info->chg1_dev, true);
	if (rc < 0)
		chg_info("enable charging fail\n");

	return rc;
}

static int oplus_mt6375_disable_charging(struct mtk_charger *info)
{
	int rc = 0;

	rc = charger_dev_enable(info->chg1_dev, false);
	if (rc < 0)
		chg_info("enable charging fail\n");

	return rc;
}

static int oplus_mt6375_check_charging_enable(struct mtk_charger *info)
{
	int rc = 0;
	bool enable = false;
	bool power_path_en = false;

	rc = charger_dev_is_enabled(info->chg1_dev, &enable);
	if (rc < 0) {
		chg_info("charger_dev_is_enabled fail\n");
		return rc;
	}

	rc = charger_dev_is_powerpath_enabled(info->chg1_dev, &power_path_en);
	if (rc < 0) {
		chg_info("charger_dev_is_powerpath_enabled fail\n");
		return rc;
        }

	chg_info("enable = %d, power_path_en = %d\n", enable, power_path_en);

	return (power_path_en && enable);
}

static int oplus_mt6375_set_aicl_point(struct oplus_chg_ic_dev *ic_dev, int vbatt)
{
	int rc = 0;
	static int hw_aicl_point = HW_AICL_POINT_VOL_5V_PHASE1;
	struct charger_device *chg = NULL;
	struct mtk_charger *info;
	int chg_vol = 0;
	int aicl_point = AICL_POINT_VOL_9V;
	int charger_type;
	bool present = false;

	if (!ic_dev) {
		chg_err("set aicl_point fail, ic_dev is null\n");
		return -ENODEV;
	}

	info = oplus_chg_ic_get_drvdata(ic_dev);
	if (!info) {
		chg_err("set aicl_point fail, info is null\n");
		return -ENODEV;
	}

	chg = info->chg1_dev;
	if (!chg) {
		chg_err("set aicl_point fail, chg is null\n");
		return -ENODEV;
	}

	if (oplus_gauge_get_batt_num() == 1)
		aicl_point = AICL_POINT_VOL_9V;
	else
		aicl_point = DUAL_BAT_AICL_POINT_VOL_9V;

	chg_vol = battery_meter_get_charger_voltage();
	rc = mtk_chg_get_charger_type(info->ic_dev, &charger_type);
	if (chg_vol > AICL_POINT_SWITCH_THRE) {
		hw_aicl_point = aicl_point;
		if (rc >= 0 && charger_type == OPLUS_CHG_USB_TYPE_PD_SDP
			&& info->oplus_pd_sdp_svid)
			hw_aicl_point = HW_AICL_POINT_VOL_6V_PHASE1;
	} else {
		if (hw_aicl_point == AICL_POINT_VOL_9V || hw_aicl_point == DUAL_BAT_AICL_POINT_VOL_9V ||
		    hw_aicl_point == USB_HW_AICL_POINT || hw_aicl_point == HW_AICL_POINT_VOL_6V_PHASE1)
			hw_aicl_point = HW_AICL_POINT_VOL_5V_PHASE1;
		if (hw_aicl_point == HW_AICL_POINT_VOL_5V_PHASE1 && vbatt > AICL_POINT_VOL_5V)
			hw_aicl_point = HW_AICL_POINT_VOL_5V_PHASE2;
		else if (hw_aicl_point == HW_AICL_POINT_VOL_5V_PHASE2 && vbatt <= AICL_POINT_VOL_5V)
			hw_aicl_point = HW_AICL_POINT_VOL_5V_PHASE1;

		mtk_chg_input_present(info->ic_dev, &present);
		rc = mtk_chg_get_charger_type(info->ic_dev, &charger_type);
		if (rc >= 0 && info->usb_aicl_enhance &&
		    charger_type == OPLUS_CHG_USB_TYPE_SDP && present)
			hw_aicl_point = USB_HW_AICL_POINT;
	}
	chg_info("set aicl point:%d charger_type:%d\n", hw_aicl_point, charger_type);
	rc = charger_dev_set_mivr(chg, hw_aicl_point * 1000);
	if (rc < 0)
		chg_err("set aicl point:%d fail\n", hw_aicl_point);

	return 0;
}

static int oplus_chg_wls_set_vindpm(struct oplus_chg_ic_dev *ic_dev, int vindpm_mv)
{
	int rc = 0;
	int vol = 0;

	if (!ic_dev) {
		chg_err("wls set vindpm fail, ic_dev is null\n");
		return -ENODEV;
	}

	switch (vindpm_mv) {
	case WLS_VINDPM_BPP:
	case WLS_VINDPM_EPP:
	case WLS_VINDPM_AIRVOOC:
	case WLS_VINDPM_AIRSVOOC:
		vol = HW_AICL_POINT_VOL_5V_PHASE1;
		break;
	case WLS_VINDPM_DEFAULT:
		return rc;
	default:
		vol = HW_AICL_POINT_VOL_5V_PHASE1;
		break;
	}

	rc = oplus_mt6375_set_aicl_point(ic_dev, vol);
	chg_info("wls set vindpm vol: %d\n", vol);
	return rc;
}

static int usb_icl[] = {
	300, 500, 900, 1200, 1300, 1350, 1500, 1800, 2000, 2400, 3000,
};

static int oplus_mt6375_input_current_limit_write_without_aicl(struct mtk_charger *info,
			int value)
{
	struct charger_device *chg = NULL;
	int rc = 0;

	if (!info || !info->chg1_dev) {
		chg_debug("info or chg1_dev is NULL, return!\n");
		return rc;
	}

	chg = info->chg1_dev;
	chg_debug("set aicr without aicl = %d", value);
	rc = charger_dev_set_input_current(chg, value * 1000);

	return rc;
}

static int oplus_chg_usb_set_input_current(struct mtk_charger *info, int current_ma,
	int aicl_point)
{
	int rc = 0, i = 0;
	int chg_vol = 0;
	struct charger_device *chg = info->chg1_dev;
	bool pre_step = false;

	for (i = 1; i <= current_ma / 100; i++) {
		rc = charger_dev_set_input_current(chg, i * 1000 * 100);
		if (rc) {
			chg_err("set icl to %d uA fail, rc=%d\n", i * 100000, rc);
			return rc;
		} else {
			chg_err("set icl to %d uA\n", i * 100000);
		}
		msleep(90);
		chg_vol = battery_meter_get_charger_voltage();
		if (chg_vol < aicl_point) {
			chg_err("chg_vol < aicl_point break here\n");
			i = i - 1;
			pre_step = true;
			break;
		}
		if (i == current_ma / 100) {
			chg_err("current_ma / 100 break here\n");
			break;
		}
	}
	if (i <= 0)
		i = 1;
	if (pre_step) {
		rc = charger_dev_set_input_current(chg, i * 1000 * 100);
		if (rc) {
			chg_err("set icl 2 to %d uA fail, rc=%d\n", i * 100000, rc);
			return rc;
		} else {
			chg_err("set icl 2 to %d uA\n", i * 100000);
		}
	}
	chg_info("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n",
		 chg_vol, i, i * 100000, aicl_point);

	return rc;
}

#define UNKONW_CURR 500
static int oplus_mt6375_input_current_limit_write(struct mtk_charger *info, int value)
{
	int rc = 0;
	int i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	int aicl_point_temp = AICL_POINT_VOL_9V;
	int chg_type = oplus_wired_get_chg_type();
	struct charger_device *chg = info->chg1_dev;
	int batt_volt;
	union mms_msg_data data = {0};
	int charger_type;
	bool present = false;
	int max_pdo_current;

	for (i = ARRAY_SIZE(usb_icl) - 1; i >= 0; i--) {
		if (usb_icl[i] <= value)
			break;
		else if (i == 0)
			break;
	}

	if (oplus_gauge_get_batt_num() == 1)
		aicl_point_temp = AICL_POINT_VOL_9V;
	else
		aicl_point_temp = DUAL_BAT_AICL_POINT_VOL_9V;

	chg_info("usb input max current limit= %d setting %02x\n", value, i);
	chg_vol = battery_meter_get_charger_voltage();
	if (is_gauge_topic_available(info)) {
		oplus_mms_get_item_data(info->gauge_topic,
					GAUGE_ITEM_VOL_MAX, &data, false);
		batt_volt = data.intval;

		if (chg_vol > aicl_point_temp &&
		    (chg_type == OPLUS_CHG_USB_TYPE_PD || chg_type == OPLUS_CHG_USB_TYPE_QC2)) {
			aicl_point = aicl_point_temp;
		} else {
			if (batt_volt > AICL_POINT_VOL_5V)
				aicl_point = SW_AICL_POINT_VOL_5V_PHASE2;
			else
				aicl_point = SW_AICL_POINT_VOL_5V_PHASE1;
		}
	} else {
		chg_info("gauge_topic is null, use default aicl_point 4500\n");
		aicl_point = 4500;
	}

	if (oplus_chg_get_common_charge_icl_support_flags()) {
		max_pdo_current = oplus_get_max_current_from_fixed_pdo(info, info->pd_chg_volt);
		chg_info("max_pdo_current:%d ma\n", max_pdo_current);

		if (max_pdo_current >= 0)
			value = min(value, max_pdo_current);
		if (value < DEFAULT_CURR_BY_CC) {
			cancel_delayed_work_sync(&info->charger_suspend_recovery_work);
			oplus_chg_suspend_charger(true, PD_PDO_ICL_VOTER);
			schedule_delayed_work(&info->charger_suspend_recovery_work,
			                      msecs_to_jiffies(SUSPEND_RECOVERY_DELAY_MS));

			goto aicl_rerun;
		} else if (value < usb_icl[0]) {
			oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
			goto common_charge_aicl_end;
		} else {
			oplus_chg_suspend_charger(false, PD_PDO_ICL_VOTER);
		}
	}

	if (info->usb_aicl_enhance) {
		mtk_chg_input_present(info->ic_dev, &present);
		rc = mtk_chg_get_charger_type(info->ic_dev, &charger_type);
		if (rc >= 0  && (charger_type == OPLUS_CHG_USB_TYPE_SDP ||
		    charger_type == OPLUS_CHG_USB_TYPE_CDP ||
		    (charger_type == OPLUS_CHG_USB_TYPE_UNKNOWN && value == UNKONW_CURR)) &&
		    present) {
			if (charger_type == OPLUS_CHG_USB_TYPE_SDP) {
				aicl_point = USB_SW_AICL_POINT;
				oplus_mt6375_set_aicl_point(info->ic_dev, batt_volt);
			}
			rc = oplus_chg_usb_set_input_current(info, value, aicl_point);
			goto aicl_rerun;
		}
	}

	if (value < 500) {
		i = 0;
		goto aicl_end;
	}

	i = 1; /* 500 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		chg_debug("use 500 here\n");
		goto aicl_end;
	} else if (value < 900) {
		goto aicl_end;
	}

	i = 2; /* 900 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value < 1200) {
		goto aicl_end;
	}

	i = 3; /* 1200 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1300 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 2;
		goto aicl_pre_step;
	}

	i = 5; /* 1350 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 3;
		goto aicl_pre_step;
	}

	i = 6; /* 1500 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 4;
		goto aicl_pre_step;
	} else if (value < 1300) {
		i = i - 3;
		goto aicl_end;
	} else if (value <= 1350) {
		i = i - 2;
		goto aicl_end;
	} else if (value < 1500) {
		i = i - 1;
		goto aicl_end;
	} else if (value < 1800) {
		goto aicl_end;
	}

	i = 7; /* 1800 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value < 2000) {
		goto aicl_end;
	}

	i = 8; /* 2000 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value < 2400) {
		goto aicl_end;
	}

	i = 9; /* 2400 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value < 3000) {
		goto aicl_end;
	}

	i = 10; /* 3000 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value >= 3000) {
		goto aicl_end;
	}

aicl_pre_step:
	chg_info("usb input max current limit aicl chg_vol=%d i[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, i, usb_icl[i], aicl_point);
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	goto aicl_rerun;
aicl_end:
	chg_info("usb input max current limit aicl chg_vol=%d i[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, i, usb_icl[i], aicl_point);
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	goto aicl_rerun;
common_charge_aicl_end:
	rc = charger_dev_set_input_current(chg, DEFAULT_CURR_BY_CC * 1000);
	chg_info("common_charge_aicl_end set icl:%d mA, rc=%d\n", DEFAULT_CURR_BY_CC, rc);
aicl_rerun:
	return rc;
}

static int oplus_mt6375_charging_current_write_fast(struct mtk_charger *info, int chg_curr)
{
	int rc = 0;
	u32 ret_chg_curr = 0;

	rc = charger_dev_set_charging_current(info->chg1_dev, chg_curr * 1000);
	if (rc < 0) {
		chg_info("set fast charge current:%d fail\n", chg_curr);
	} else {
		charger_dev_get_charging_current(info->chg1_dev, &ret_chg_curr);
		chg_info("set fast charge current:%d ret_chg_curr = %d\n", chg_curr, ret_chg_curr);
	}

	return rc;
}

static int oplus_mt6375_float_voltage_write(struct mtk_charger *info, int vfloat_mv)
{
	int rc = 0;

	rc = charger_dev_set_constant_voltage(info->chg1_dev, vfloat_mv * 1000);
	if (rc < 0)
		chg_info("set float voltage:%d fail\n", vfloat_mv);

	return rc;
}

static int oplus_mt6375_float_voltage_read(struct mtk_charger *info)
{
	int rc = 0;
	int vfloat_mv = 0;

	if (!info) {
		chg_err("charger_data not ready!\n");
		return 0;
	}

	rc = charger_dev_get_constant_voltage(info->chg1_dev, &vfloat_mv);
	if (rc < 0) {
		chg_err("get float voltage fail\n");
		return rc;
	}

	vfloat_mv = vfloat_mv / 1000;

	return vfloat_mv;
}

static int oplus_mt6375_set_termchg_current(struct mtk_charger *info, int term_curr)
{
	int rc = 0;

	rc = charger_dev_set_eoc_current(info->chg1_dev, term_curr * 1000);
	if (rc < 0)
		chg_info("set termchg_current fail\n");

	return rc;
}

static int oplus_mt6375_get_chg_ibus(struct mtk_charger *info)
{
	u32 ibus = 0;
	int ret = 0;

	ret = charger_dev_get_ibus(info->chg1_dev, &ibus);
	if (!ret)
		return ibus;

	return -1;
}

static int oplus_mt6375_otg_enable(struct mtk_charger *info)
{
	return charger_dev_enable_otg(info->chg1_dev, true);
}

static int oplus_mt6375_otg_disable(struct mtk_charger *info)
{
	return charger_dev_enable_otg(info->chg1_dev, false);
}

static int oplus_mt6375_set_chging_term_disable(struct mtk_charger *info)
{
	int rc = 0;

	if (!info) {
		chg_err("[OPLUS_CHG]: oplus_chip not ready!\n");
		return -ENODEV;
	}

	rc = charger_dev_enable_termination(info->chg1_dev, false);
	if (rc < 0) {
		chg_debug("disable chging_term fail\n");
		return -ENODEV;
	}

	return 0;
}

static int mtk_chg_init(struct oplus_chg_ic_dev *ic_dev)
{
	ic_dev->online = true;
	return 0;
}

static int mtk_chg_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	return 0;
}

static int mtk_chg_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int  mtk_chg_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int mtk_chg_input_present(struct oplus_chg_ic_dev *ic_dev, bool *present)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	if (oplus_mt_get_vbus_status() == true)
		*present = true;
	else
		*present = false;

	return 0;
}

static int mtk_chg_input_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	int rc;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);
	bool power_path_en;

	charger_dev_is_powerpath_enabled(info->chg1_dev, &power_path_en);
	if (!power_path_en == suspend) {
		chg_info("charger input already %s\n",
			 suspend ? "suspend" : "unsuspend");
		return 0;
	}

	if (suspend)
		rc = oplus_mt6375_suspend_charger(info);
	else
		rc = oplus_mt6375_unsuspend_charger(info);

	chg_info("charger input %s, rc = %d\n",
		 suspend ? "suspend" : "unsuspend", rc);

	return rc;
}

static int mtk_chg_input_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	bool is_enable = 0;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	charger_dev_is_powerpath_enabled(info->chg1_dev, &is_enable);
	*suspend = !is_enable;

	return 0;
}

static int mtk_chg_output_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	int rc = 0;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	if (suspend)
		rc = oplus_mt6375_disable_charging(info);
	else
		rc = oplus_mt6375_enable_charging(info);

	chg_info("%s: charger out %s, rc = %d",
		 __func__, suspend ? "suspend" : "unsuspend", rc);

	return rc;
}

static int mtk_chg_output_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	*suspend = oplus_mt6375_check_charging_enable(info);

	return 0;
}

static int mtk_chg_set_icl(struct oplus_chg_ic_dev *ic_dev, bool vooc_mode,
			   bool step, int icl_ma)
{
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	if (step)
		return oplus_mt6375_input_current_limit_write(info, icl_ma);
	else
		return oplus_mt6375_input_current_limit_write_without_aicl(info, icl_ma);
}

static int mtk_chg_get_icl(struct oplus_chg_ic_dev *ic_dev, int *icl_ma)
{
	/* TODO */
	return 0;
}

static int mtk_chg_set_fcc(struct oplus_chg_ic_dev *ic_dev, int fcc_ma)
{
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	return oplus_mt6375_charging_current_write_fast(info, fcc_ma);
}

static int mtk_chg_set_fv(struct oplus_chg_ic_dev *ic_dev, int fv_mv)
{
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	return oplus_mt6375_float_voltage_write(info, fv_mv);
}

static int mtk_chg_get_fv(struct oplus_chg_ic_dev *ic_dev, int *fv_mv)
{
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	*fv_mv = oplus_mt6375_float_voltage_read(info);

	return 0;
}

static int mtk_chg_set_iterm(struct oplus_chg_ic_dev *ic_dev, int iterm_ma)
{
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	return oplus_mt6375_set_termchg_current(info, iterm_ma);
}

static int mtk_chg_set_rechg_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	/* TODO */
	return 0;
}

static int mtk_chg_get_input_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	*curr_ma = oplus_mt6375_get_chg_ibus(info) / 1000;

	return 0;
}

static int mtk_chg_get_input_vol(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	*vol_mv = battery_meter_get_charger_voltage();

	return 0;
}

static int mtk_chg_otg_boost_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	if (en) {
		rc = oplus_mt6375_otg_enable(info);
	} else {
		info->reverse_chg_svid = 0;
		rc = oplus_mt6375_otg_disable(info);
	}

	if (rc < 0)
		chg_err("can't %s otg boost, rc=%d\n",
			en ? "enable" : "disable", rc);
	return 0;
}

static int mtk_chg_set_otg_boost_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	int rc;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	if (info == NULL) {
		chg_err("info is NULL");
		return -ENODEV;
	}

	rc =  charger_dev_set_boost_voltage_limit(info->chg1_dev, vol_mv * 1000);
	if (rc < 0)
		chg_err("set otg vol err, rc=%d\n", rc);

	return rc;
}

static int mtk_chg_set_otg_boost_curr_limit(struct oplus_chg_ic_dev *ic_dev, int curr_uA)
{
	int rc;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);
	struct charger_device *chg;
	int batt_soc;

	if (info == NULL) {
		chg_err("info is NULL");
		return -ENODEV;
	}

	chg = info->chg1_dev;
	batt_soc = oplus_chg_get_batt_soc();
	chg_info("mtk_chg_set_otg_boost_curr_limit get batt_soc is %d\n", batt_soc);
	if ((batt_soc <= LOW_BATT_SOC) && (info->low_batt_otg_boost_curr_ua != 0) &&
		(info->low_batt_otg_boost_curr_ua <= curr_uA)) {
		curr_uA = info->low_batt_otg_boost_curr_ua;
		chg_info("batt_soc is low, set curr %d ua\n", curr_uA);
	}
	rc = charger_dev_set_boost_current_limit(chg, curr_uA);
	if (rc < 0)
		chg_err("set otg cc err, rc=%d\n", rc);
	return rc;
}

static int mtk_chg_set_otg_boost_ocp_limit(struct oplus_chg_ic_dev *ic_dev, int curr_mA)
{
	int rc;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);
	struct charger_device *chg;

	if (info == NULL) {
		chg_err("info is NULL");
		return -ENODEV;
	}
	chg = info->chg1_dev;
	if (chg == NULL) {
		chg_err("chg is NULL");
		return -ENODEV;
	}

	rc = charger_dev_set_boost_current_limit(chg, curr_mA * 1000);
	if (rc < 0) {
		chg_err("set otg cc err, rc=%d\n", rc);
	} else {
		chg_info("set otg_cc=%dmA ok!\n", curr_mA);
	}
	return rc;
}

static void oplus_sink_request_check_work(struct work_struct *work)
{
	struct mtk_charger *chip = container_of(work,
		struct mtk_charger, sink_request_check_work.work);

	chip->msg_type = REVERSE_CHG_MSG_TYPE_SINK_REQ_PDO;
	if (chip->reverse_chg_ic_dev)
		oplus_chg_ic_virq_trigger(chip->reverse_chg_ic_dev, OPLUS_IC_VIRQ_SINK_REQ_MSG);
}

#define SRC_PE_READY_DELAY 100
#define SRC_WAIT_PE_RDY_TH 10
#define NORMAL_VBUS 5000
#define NORMAL_IBUS 1000
static int mtk_chg_set_reverse_boost_pdo
	(struct oplus_chg_ic_dev *ic_dev, int pdo0_volt, int pdo0_curr, int pdo_voltage, int pdo_current)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) && !IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
	struct tcpc_device *tcpc = tcpc_dev_get_by_name("type_c_port0");
	struct tcpm_power_cap cap = { 1, {0x26019064} };
	struct mtk_charger *chip;

	uint32_t flags;
	struct tcpm_power_cap user_cap;
	int i = 0;
	int rc;

	if (ic_dev  == NULL) {
		chg_err("ic_dev is NULL, return\n");
		return -EINVAL;
	}
	if (!tcpc) {
		chg_info("tcpc_dev is null, return\n");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->reverse_enable) {
		chg_err("reverse_enable = 0, return\n");
		return -EFAULT;
	}

	rc = tcpm_inquire_pd_local_source_cap(tcpc, &user_cap);
	if (rc < 0) {
		chg_err("tcpm_inquire_pd_local_source_cap error: rc=%d\n", rc);
		return rc;
	}

	flags = user_cap.pdos[0] & (0x1FF << 23);

	if (pdo_voltage == NORMAL_VBUS) {
		cap.pdos[0] = PDO_FIXED(pdo_voltage, pdo_current, flags);
		cap.cnt = 1;
	} else {
		cap.pdos[0] = PDO_FIXED(pdo0_volt, pdo0_curr, flags);
		cap.pdos[1] = PDO_FIXED(pdo_voltage, pdo_current, 0);
		cap.cnt = 2;
	}

	if (tcpc->pd_port.pe_pd_state != PE_SRC_READY) {
		for (i = 0; i <= SRC_WAIT_PE_RDY_TH; i++) {
			msleep(SRC_PE_READY_DELAY);
			chg_info("!!!pe_pd_state : %d, src_pe_not_rdy_cnt: %d\n", tcpc->pd_port.pe_pd_state, i);
			if (tcpc->pd_port.pe_pd_state != PE_SRC_READY)
				continue;
			else
				break;
		}
	}

	chg_info("set pdo voltage = %d, current=%d \n", pdo_voltage, pdo_current);
	chg_info("!!!pe_pd_state: %d(%s), REVERSE_PDO: update sourcecap cnt[%d] pdo: 0x%08X, 0x%08X\n",
		tcpc->pd_port.pe_pd_state, (tcpc->pd_port.pe_pd_state == PE_SRC_READY) ? "PE_SRC_READY" : "PE_SRC_NOT_READY",
		cap.cnt, cap.pdos[0], cap.pdos[1]);

	rc = tcpm_set_pd_local_source_cap(tcpc, &cap);
	if (rc < 0) {
		chg_err("tcpm_set_pd_local_source_cap error: rc=%d\n", rc);
		return rc;
	}

	rc = tcpm_dpm_pd_source_cap(tcpc, NULL);
	if (rc < 0) {
		chg_err("tcpm_dpm_pd_source_cap error: rc=%d\n", rc);
		return rc;
	}

	schedule_delayed_work(&chip->sink_request_check_work,
			msecs_to_jiffies(WAIT_SOURCECPA_DOWN));
#endif
	return 0;
}

static int mtk_chg_set_rvs_high_mode_en(struct oplus_chg_ic_dev *ic_dev, int enable)
{
#if !IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	if (enable) {
		mtk_chg_set_reverse_boost_pdo(ic_dev, NORMAL_VBUS, NORMAL_IBUS, VBUS_9V, NORMAL_IBUS);
	} else {
		mtk_tcpc_set_high_reverse_enable(ic_dev, false);
		mtk_tcpc_set_otg_enable(ic_dev, false);
		chg_err("close high reverse and otg");
	}
#endif
	return 0;
}

static int mtk_chg_aicl_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	/* TODO */

	return rc;
}

static int mtk_chg_aicl_rerun(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	/* TODO */

	return rc;
}

static int mtk_chg_aicl_reset(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;
	struct charger_device *chg = NULL;
	struct mtk_charger *info;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	info = oplus_chg_ic_get_drvdata(ic_dev);
	if (!info) {
		chg_err("reset aicl_point fail, info is null\n");
		return -ENODEV;
	}

	chg = info->chg1_dev;
	if (!chg) {
		chg_err("reset aicl_point fail, chg is null\n");
		return -ENODEV;
	}

	rc = charger_dev_set_mivr(chg, HW_AICL_POINT_VOL_5V_PHASE1 * 1000);
	if (rc < 0)
		chg_err("reset aicl point fail\n");

	return rc;
}

static int mtk_chg_get_cc_orientation(struct oplus_chg_ic_dev *ic_dev, int *orientation)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->tcpc) {
		chg_err("tcpc is NULL");
		return -ENODEV;
	}

	if (tcpm_inquire_typec_attach_state(chip->tcpc) != TYPEC_UNATTACHED)
		*orientation = (int)tcpm_inquire_cc_polarity(chip->tcpc) + 1;
	else
		*orientation = 0;

	if (*orientation != 0)
		chg_info("cc[%d]\n", *orientation);

	return 0;
}

static int mtk_chg_get_hw_detect(struct oplus_chg_ic_dev *ic_dev, int *detected, bool recheck)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*detected = chip->wd0_detect;
	return 0;
}

static int oplus_chg_get_power_role(struct oplus_chg_ic_dev *ic_dev, int *power_role)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*power_role = chip->power_role;
	return 0;
}

static int mtk_chg_get_typec_state(struct oplus_chg_ic_dev *ic_dev, int *state)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*state = chip->typec_state;

	return 0;
}

static bool oplus_pd_without_usb(void)
{
	struct tcpc_device *tcpc;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc) {
		chg_err("get type_c_port0 fail\n");
		return -ENODEV;
	}

	if (!tcpm_inquire_pd_connected(tcpc))
		return true;
	return (tcpm_inquire_dpm_flags(tcpc) &
			DPM_FLAGS_PARTNER_USB_COMM) ? false : true;
}

static int mtk_chg_get_charger_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct mtk_charger *chip;
	union power_supply_propval prop;
	static struct power_supply *chg_psy;
	bool online;
	int usb_type, charger_type;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chg_psy = chip->chg_psy;
	if (!chg_psy || IS_ERR(chg_psy)) {
		chr_err("retry to get chg_psy\n");
		chg_psy = devm_power_supply_get_by_phandle(&chip->pdev->dev, "charger");
		chip->chg_psy = chg_psy;
	}

	if (!chg_psy || IS_ERR(chg_psy)) {
		chr_err("Couldn't get chg_psy\n");
		return -ENODEV;
	}

	rc = power_supply_get_property(chg_psy, POWER_SUPPLY_PROP_ONLINE,
				       &prop);
	if (rc < 0) {
		chg_err("can't get usb online status, rc=%d", rc);
		online = false;
	} else {
		online = !!prop.intval;
	}

	rc = power_supply_get_property(chg_psy, POWER_SUPPLY_PROP_TYPE, &prop);
	if (rc < 0) {
		chg_err("can't get charger type, rc=%d", rc);
		return rc;
	}
	charger_type = prop.intval;

	rc = power_supply_get_property(chg_psy, POWER_SUPPLY_PROP_USB_TYPE,
				       &prop);
	if (rc < 0) {
		chg_err("can't get usb type, rc=%d", rc);
		return rc;
	}
	usb_type = prop.intval;

	if (!online || (charger_type == POWER_SUPPLY_TYPE_USB &&
			usb_type == POWER_SUPPLY_USB_TYPE_UNKNOWN))
		charger_type = POWER_SUPPLY_TYPE_UNKNOWN;

	chr_debug("online:%d type:%d usb_type:%d\n", online, charger_type,
		  usb_type);

	switch (charger_type) {
	case POWER_SUPPLY_TYPE_UNKNOWN:
		*type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		break;
	case POWER_SUPPLY_TYPE_USB:
		*type = OPLUS_CHG_USB_TYPE_SDP;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		*type = OPLUS_CHG_USB_TYPE_DCP;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		*type = OPLUS_CHG_USB_TYPE_CDP;
		break;
	case POWER_SUPPLY_TYPE_USB_ACA:
		*type = OPLUS_CHG_USB_TYPE_ACA;
		break;
	case POWER_SUPPLY_TYPE_USB_TYPE_C:
		*type = OPLUS_CHG_USB_TYPE_C;
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		*type = OPLUS_CHG_USB_TYPE_PD;
		break;
	case POWER_SUPPLY_TYPE_USB_PD_DRP:
		*type = OPLUS_CHG_USB_TYPE_PD_DRP;
		break;
	case POWER_SUPPLY_TYPE_APPLE_BRICK_ID:
		*type = OPLUS_CHG_USB_TYPE_APPLE_BRICK_ID;
		break;
	default:
		*type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		break;
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (chip->pd_type == PD_CONNECT_PE_READY_SNK ||
	    chip->pd_type == PD_CONNECT_PE_READY_SNK_PD30) {
		*type = OPLUS_CHG_USB_TYPE_PD;
	} else if (chip->pd_type == PD_CONNECT_PE_READY_SNK_APDO) {
		*type = OPLUS_CHG_USB_TYPE_PD_PPS;
	}
#else
	if (chip->pd_type == PD_CONNECT_PE_READY_SNK ||
			chip->pd_type == PD_CONNECT_PE_READY_SNK_PD30||
			chip->pd_type == PD_CONNECT_PE_READY_SNK_APDO)
		*type = OPLUS_CHG_USB_TYPE_PD;
#endif /* OPLUS_FEATURE_CHG_BASIC */

	if (*type != OPLUS_CHG_USB_TYPE_UNKNOWN && !oplus_pd_without_usb())
		*type = OPLUS_CHG_USB_TYPE_PD_SDP;

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	if (mt6375_get_hvdcp_type() == POWER_SUPPLY_TYPE_USB_HVDCP)
		*type = OPLUS_CHG_USB_TYPE_QC2;
#endif
	if (!chip->tcpc)
		chip->tcpc = tcpc_dev_get_by_name("type_c_port0");
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (charger_type == POWER_SUPPLY_TYPE_USB_DCP && chip->tcpc &&
	    tcpm_inquire_pd_connected(chip->tcpc)) {
		if (*type != OPLUS_CHG_USB_TYPE_PD_PPS && *type != OPLUS_CHG_USB_TYPE_PD)
			*type = OPLUS_CHG_USB_TYPE_PD;
	}
#else
	if (charger_type == POWER_SUPPLY_TYPE_USB_DCP && chip->tcpc &&
	    tcpm_inquire_pd_connected(chip->tcpc))
		*type = OPLUS_CHG_USB_TYPE_PD;
#endif /* OPLUS_FEATURE_CHG_BASIC */

	return 0;
}

static int mtk_chg_rerun_bc12(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int mtk_chg_qc_detect_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	if (en) {
		chg_err("enter chg enable qc!\n");
		mt6375_enable_hvdcp_detect();
	}
#endif
	return 0;
}

static int mtk_chg_shipmode_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	if (info == NULL)
		return -EINVAL;
	if (en) {
		chg_info(" enable ship mode \n");
		charger_dev_enable_ship_mode(info->chg1_dev);
	}
#else
	if (en) {
		chg_info(" enable ship mode \n");
		oplus_chg_set_shipping_mode();
	}
#endif

	return 0;
}

static void oplus_chg_get_soc_and_temp(int *soc, int *temp)
{
	struct oplus_mms *common_topic;
	union mms_msg_data data = {0};

	common_topic = oplus_mms_get_by_name("common");
	if (common_topic) {
		oplus_mms_get_item_data(common_topic,
					COMM_ITEM_UI_SOC, &data, false);
		*soc = data.intval;
		oplus_mms_get_item_data(common_topic,
					COMM_ITEM_SHELL_TEMP, &data, false);
		*temp = data.intval;
	} else {
		chg_err("common topic not found\n");
	}
}

static void oplus_hvdcp_detect_work(struct work_struct *work)
{
	int chg_volt = 0;
	struct mtk_charger *chip = container_of(work,
		struct mtk_charger, hvdcp_detect_work.work);

	if (chip->hvdcp_disable == false) {
		chg_volt = get_vbus(chip);
		if (OPLUS_CHG_PULL_UP_VOLT_THR > chg_volt) {
			chip->hvdcp_disable = true;
		}
		chg_info("chg_volt = [%d] hvdcp_disable = %d \n",
			 chg_volt, chip->hvdcp_disable);
	}
}

static void oplus_detach_clean_work(struct work_struct *work)
{
	struct mtk_charger *chip = container_of(work,
			struct mtk_charger, detach_clean_work.work);

	if (chip) {
		chg_info("clean the pd_svooc \n");
		chip->pd_svooc = false;
		chip->oplus_pd_sdp_svid = false;
	}
}

#define PDQC_BOOST_DELAY_MS	300
#define QC_BUCK_DELAY_MS	400

static void oplus_chg_set_qc_config_forvoocphy(struct mtk_charger *chip, int vol_mv)
{
	int chg_vol = battery_meter_get_charger_voltage();
	int batt_volt = AICL_POINT_VOL_5V;
	union mms_msg_data data = {0};

	chg_info("set qc to %d mv, chg_vol = %d\n", vol_mv, chg_vol);
	if (vol_mv == 9000) {
		if (chg_vol > OPLUS_CHG_PULL_UP_VOLT_THR)
			return;

		mt6375_set_hvdcp_to_9v();
		oplus_notify_hvdcp_detect_stat();
		msleep(PDQC_BOOST_DELAY_MS);
		if (is_gauge_topic_available(chip)) {
			oplus_mms_get_item_data(chip->gauge_topic,
					GAUGE_ITEM_VOL_MAX, &data, false);
			batt_volt = data.intval;
		}
		oplus_mt6375_set_aicl_point(chip->ic_dev, batt_volt);
	} else {
		if (chg_vol > OPLUS_CHG_PULL_UP_VOLT_THR) {
			mtk_chg_aicl_reset(chip->ic_dev);
			mt6375_set_hvdcp_to_5v();
			msleep(PDQC_BOOST_DELAY_MS);
			if (is_gauge_topic_available(chip)) {
				oplus_mms_get_item_data(chip->gauge_topic,
						GAUGE_ITEM_VOL_MAX, &data, false);
				batt_volt = data.intval;
			}
			oplus_mt6375_set_aicl_point(chip->ic_dev, batt_volt);
		}
	}
}

static void oplus_chg_set_qc_config_forsvooc(struct mtk_charger *chip, int vol_mv)
{
	int chg_vol = battery_meter_get_charger_voltage();

	chg_info("set qc to %d mv, chg_vol = %d\n", vol_mv, chg_vol);

	if (vol_mv == 9000) {
		if (chg_vol > OPLUS_CHG_PULL_UP_VOLT_THR)
			return;

		mt6375_set_hvdcp_to_5v();
		msleep(PDQC_BOOST_DELAY_MS);
		oplus_suspend_charger(true);
		mt6375_set_hvdcp_to_9v();
		oplus_notify_hvdcp_detect_stat();
		msleep(PDQC_BOOST_DELAY_MS);
		oplus_suspend_charger(false);
	} else {
		if (chg_vol > OPLUS_CHG_PULL_UP_VOLT_THR) {
			oplus_suspend_charger(true);
			mt6375_set_hvdcp_to_5v();
			msleep(QC_BUCK_DELAY_MS);
			oplus_suspend_charger(false);
		}
	}
}

static int mtk_chg_set_qc_config(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_qc_version version, int vol_mv)
{
	struct mtk_charger *chip;
	int soc, temp;
	int chg_vol = battery_meter_get_charger_voltage();

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (version) {
	case OPLUS_CHG_QC_2_0:
		if (vol_mv != 5000 && vol_mv != 9000) {
			chg_err("Unsupported qc voltage(=%d)\n", vol_mv);
			return -EINVAL;
		} else if (vol_mv == 9000) {
			oplus_chg_get_soc_and_temp(&soc, &temp);
			if (soc >= 90 || temp > 530 || chg_vol > OPLUS_CHG_PULL_UP_VOLT_THR) {
				chg_info("no allow set 9V, soc = %d, temp = %d, chg_vol = %d\n",
					  soc, temp, chg_vol);
				return 0;
			}
		}

		if (oplus_chg_get_voocphy_support()) {
			oplus_chg_set_qc_config_forvoocphy(chip, vol_mv);
		} else if (oplus_gauge_get_batt_num() != 1) {
			oplus_chg_set_qc_config_forsvooc(chip, vol_mv);
		} else {
			if (vol_mv == 9000) {
				mt6375_set_hvdcp_to_9v();
			} else {
				if (chg_vol > OPLUS_CHG_PULL_UP_VOLT_THR)
					mt6375_set_hvdcp_to_5v();
			}
		}
		break;
	case OPLUS_CHG_QC_3_0:
	default:
		chg_err("Unsupported qc version(=%d)\n", version);
		return -EINVAL;
	}

	return 0;
}

#define PD_SWITCH_POLICY_DELAY_MS	100
static int oplus_pdc_setup(int *vbus_mv, int *ibus_ma) {
	int ret = 0;
	int vbus_mv_t = 0;
	int ibus_ma_t = 0;
	struct tcpc_device *tcpc = NULL;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}

	if (*vbus_mv == VBUS_5V)
		ret = tcpm_set_pd_charging_policy(tcpc, DPM_CHARGING_POLICY_VSAFE5V, NULL);
	else
		ret = tcpm_set_pd_charging_policy(tcpc, DPM_CHARGING_POLICY_MAX_POWER_LVIC, NULL);

	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_set_apdo_charging_policy fail\n");
		return -EINVAL;
	}
	msleep(PD_SWITCH_POLICY_DELAY_MS);

	ret = tcpm_dpm_pd_request(tcpc, *vbus_mv, *ibus_ma, NULL);
	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_dpm_pd_request fail\n");
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(tcpc, &vbus_mv_t, &ibus_ma_t);
	if (ret != TCPM_SUCCESS) {
		chg_err("inquire current vbus_mv and ibus_ma fail\n");
		return -EINVAL;
	}
	chg_info("request vbus_mv[%d], ibus_ma[%d]\n", vbus_mv_t, ibus_ma_t);

	return 0;
}

static int oplus_chg_set_pd_config_voocphy(struct mtk_charger *chip, int vol_mv)
{
	int vbus_mv = VBUS_9V;
	int ibus_ma = IBUS_2A;
	int ret = 0;
	struct adapter_power_cap cap;
	int i;
	int batt_volt = AICL_POINT_VOL_5V;
	union mms_msg_data data = {0};

	cap.nr = 0;
	cap.pdp = 0;
	for (i = 0; i < ADAPTER_CAP_MAX_NR; i++) {
		cap.max_mv[i] = 0;
		cap.min_mv[i] = 0;
		cap.ma[i] = 0;
		cap.type[i] = 0;
		cap.pwr_limit[i] = 0;
	}

	if (vol_mv == 9000) {
		if (chip->pd_type == PD_CONNECT_PE_READY_SNK_APDO) {
			adapter_dev_get_cap(chip->pd_adapter, MTK_PD_APDO, &cap);
			for (i = 0; i < cap.nr; i++) {
				chg_info("PD APDO cap %d: mV:%d,%d mA:%d type:%d pwr_limit:%d pdp:%d\n",
					 i, cap.max_mv[i], cap.min_mv[i], cap.ma[i], cap.type[i],
					 cap.pwr_limit[i], cap.pdp);
			}

			for (i = 0; i < cap.nr; i++) {
				if (cap.min_mv[i] <= VBUS_9V && VBUS_9V <= cap.max_mv[i]) {
					vbus_mv = VBUS_9V;
					ibus_ma = cap.ma[i];
					if (ibus_ma > IBUS_2A)
						ibus_ma = IBUS_2A;
					break;
				}
			}
		} else if (chip->pd_type == PD_CONNECT_PE_READY_SNK ||
			   chip->pd_type == PD_CONNECT_PE_READY_SNK_PD30) {
			adapter_dev_get_cap(chip->pd_adapter, MTK_PD, &cap);
			for (i = 0; i < cap.nr; i++) {
				chg_info("PD cap %d: mV:%d,%d mA:%d type:%d\n",
					 i, cap.max_mv[i], cap.min_mv[i], cap.ma[i], cap.type[i]);
			}

			for (i = 0; i < cap.nr; i++) {
				if (VBUS_9V <= cap.max_mv[i]) {
					vbus_mv = cap.max_mv[i];
					ibus_ma = cap.ma[i];
					if (ibus_ma > IBUS_2A)
						ibus_ma = IBUS_2A;

					/* for nexode mini 30w cd305 adapter. almid 7058213 */
					if (vbus_mv > VBUS_9V)
						vbus_mv = VBUS_5V;

					break;
				}
			}
		} else {
			vbus_mv = VBUS_9V;
			ibus_ma = IBUS_2A;
		}

		chg_info("PD request: %dmV, %dmA\n", vbus_mv, ibus_ma);
		ret = oplus_pdc_setup(&vbus_mv, &ibus_ma);
	} else {
		vbus_mv = VBUS_5V;
		ibus_ma = IBUS_3A;
		chg_info("PD request: %dmV, %dmA\n", vbus_mv, ibus_ma);
		mtk_chg_aicl_reset(chip->ic_dev);
		ret = oplus_pdc_setup(&vbus_mv, &ibus_ma);
	}
	msleep(PDQC_BOOST_DELAY_MS);
	if (is_gauge_topic_available(chip)) {
		oplus_mms_get_item_data(chip->gauge_topic,
				GAUGE_ITEM_VOL_MAX, &data, false);
		batt_volt = data.intval;
	}
	oplus_mt6375_set_aicl_point(chip->ic_dev, batt_volt);

	return ret;
}

static int oplus_chg_set_pd_config(int vol_mv, int curr_ma)
{
	int rc = 0;

	oplus_suspend_charger(true);
	rc = oplus_pdc_setup(&vol_mv, &curr_ma);
	if (rc != TCPM_SUCCESS) {
		chg_err("oplus_pdc_setup fail, rc=%d\n", rc);
		rc = -EINVAL;
		goto out;
	}
	msleep(PDQC_BOOST_DELAY_MS);
	chg_info("PD Default vbus_mv[%d], ibus_ma[%d]\n", vol_mv, curr_ma);
out:
	oplus_suspend_charger(false);

	return rc;
}

static bool mtk_chg_vooc_protocol_is_disabled(struct mtk_charger *chip)
{
	static struct oplus_mms *cpa_topic;

	if (IS_ERR_OR_NULL(cpa_topic)) {
		cpa_topic = oplus_mms_get_by_name("cpa");
		if (IS_ERR_OR_NULL(cpa_topic)) {
			chg_err("cpa topic not found\n");
			return false;
		}
	}

	return !oplus_cpa_protocol_check_enable(cpa_topic, CHG_PROTOCOL_VOOC);
}

static int mtk_chg_should_disable_pd(struct oplus_chg_ic_dev *ic_dev)
{
	struct mtk_charger *chip;
	int disable_pd = 0;
	struct votable *vooc_disable_votable;
	int vooc_disable = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	vooc_disable_votable = find_votable("VOOC_DISABLE");
	if (vooc_disable_votable)
		vooc_disable = get_effective_result(vooc_disable_votable);
	else
		chg_err("VOOC_DISABLE votable not found\n");

	if (mtk_chg_vooc_protocol_is_disabled(chip))
		vooc_disable = true;

	disable_pd = chip->pd_svooc;
	if (chip->pd_svooc && vooc_disable) {
		disable_pd = false;
	}
	chg_info("pd_svooc = %d, vooc_disable = %d\n", chip->pd_svooc, vooc_disable);

	return disable_pd;
}

static void oplus_source_pdo_check_work(struct work_struct *work)
{
	struct mtk_charger *chip = container_of(work,
		struct mtk_charger, source_pdo_check_work.work);
	int max_pdo_current = 0;

	max_pdo_current = oplus_get_max_current_from_fixed_pdo(chip, VBUS_9V);
	if (max_pdo_current < 0)
		max_pdo_current = oplus_get_max_current_from_fixed_pdo(chip, VBUS_5V);
	if (max_pdo_current >= 0)
		oplus_chg_set_icl_by_vote(max_pdo_current, PD_PDO_ICL_VOTER);
}

static int mtk_chg_set_pd_config(struct oplus_chg_ic_dev *ic_dev, u32 pdo)
{
	struct mtk_charger *chip;
	int vol_mv, curr_ma;
	int soc, temp;
	int rc = 0;
	int chg_vol = battery_meter_get_charger_voltage();

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (mtk_chg_should_disable_pd(ic_dev)) {
		chg_info("disable pd\n");
		return 0;
	}

	switch (PD_SRC_PDO_TYPE(pdo)) {
	case PD_SRC_PDO_TYPE_FIXED:
		vol_mv = PD_SRC_PDO_FIXED_VOLTAGE(pdo) * 50;
		curr_ma = PD_SRC_PDO_FIXED_MAX_CURR(pdo) * 10;
		if (curr_ma >= 2000)
			curr_ma = 2000;
		chip->pd_chg_volt = vol_mv;
		break;
	case PD_SRC_PDO_TYPE_BATTERY:
	case PD_SRC_PDO_TYPE_VARIABLE:
	case PD_SRC_PDO_TYPE_AUGMENTED:
	default:
		chg_err("Unsupported pdo type(=%d)\n", PD_SRC_PDO_TYPE(pdo));
		return -EINVAL;
	}

	if (vol_mv == 9000) {
		oplus_chg_get_soc_and_temp(&soc, &temp);
		if (soc >= 90 || temp > 530 || chg_vol > OPLUS_CHG_PULL_UP_VOLT_THR) {
			chg_info("no allow set 9V, soc = %d, temp = %d, chg_vol = %d\n",
				  soc, temp, chg_vol);
			return 0;
		}
	}

	if (oplus_chg_get_voocphy_support()) {
		rc = oplus_chg_set_pd_config_voocphy(chip, vol_mv);
	} else if (oplus_gauge_get_batt_num() != 1) {
		rc = oplus_chg_set_pd_config(vol_mv, curr_ma);
	}

	schedule_delayed_work(&chip->source_pdo_check_work,
				msecs_to_jiffies(WAIT_SOURCECPA_DOWN));

	return rc;
}

static int mtk_chg_get_shutdown_soc(struct oplus_chg_ic_dev *ic_dev, int *soc)
{
	/* TODO */
	return -ENOTSUPP;
}

static int mtk_chg_backup_soc(struct oplus_chg_ic_dev *ic_dev, int soc)
{
	/* TODO */
	return -ENOTSUPP;
}

static int mtk_chg_get_vbus_collapse_status(struct oplus_chg_ic_dev *ic_dev, bool *collapse)
{
	/* TODO */
	return -ENOTSUPP;
}

static int mtk_chg_get_typec_role(struct oplus_chg_ic_dev *ic_dev,
					enum oplus_chg_typec_port_role_type *mode)
{
	struct mtk_charger *chip;
	uint8_t typec_role;

	if (!ic_dev || !pinfo || !pinfo->tcpc) {
		chg_err("[%s]: g_oplus_chip not ready!\n", __func__);
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	typec_role = tcpm_inquire_typec_role(pinfo->tcpc);

	switch(typec_role) {
	case TYPEC_ROLE_SNK:
		*mode = TYPEC_PORT_ROLE_SNK;
		break;
	case TYPEC_ROLE_SRC:
		*mode = TYPEC_PORT_ROLE_SRC;
		break;
	case TYPEC_ROLE_DRP:
		*mode = TYPEC_PORT_ROLE_DRP;
		break;
	case TYPEC_ROLE_TRY_SRC:
		*mode = TYPEC_PORT_ROLE_TRY_SRC;
		break;
	case TYPEC_ROLE_TRY_SNK:
		*mode = TYPEC_PORT_ROLE_TRY_SNK;
		break;
	case TYPEC_ROLE_UNKNOWN:
	case TYPEC_ROLE_NR:
	default:
		*mode = TYPEC_PORT_ROLE_INVALID;
		break;
	}
	return 0;
}


static int mtk_chg_get_typec_mode(struct oplus_chg_ic_dev *ic_dev,
					enum oplus_chg_typec_port_role_type *mode)
{
	struct mtk_charger *chip;
	uint8_t typec_state;

	if (!ic_dev || !pinfo || !pinfo->tcpc) {
		chg_err("[%s]: g_oplus_chip not ready!\n", __func__);
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	typec_state = tcpm_inquire_typec_attach_state(pinfo->tcpc);

	switch(typec_state) {
	case TYPEC_ATTACHED_SNK:
	case TYPEC_ATTACHED_DBGACC_SNK:
		*mode = TYPEC_PORT_ROLE_SNK;
		break;
	case TYPEC_ATTACHED_SRC:
	case TYPEC_ATTACHED_CUSTOM_SRC:
	case TYPEC_ATTACHED_NORP_SRC:
		*mode = TYPEC_PORT_ROLE_SRC;
		break;
	default:
		*mode = TYPEC_PORT_ROLE_INVALID;
		break;
	}
	return 0;
}

static int mtk_chg_set_typec_mode(struct oplus_chg_ic_dev *ic_dev,
					enum oplus_chg_typec_port_role_type mode)
{
	struct mtk_charger *chip;
	uint8_t typec_role;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip->tcpc)
		chip->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!chip->tcpc) {
		chg_err("get type_c_port0 fail\n");
		return -ENODEV;
	}

	switch(mode) {
	case TYPEC_PORT_ROLE_DRP:
		typec_role = TYPEC_ROLE_DRP;
		break;
	case TYPEC_PORT_ROLE_SNK:
		typec_role = TYPEC_ROLE_SNK;
		break;
	case TYPEC_PORT_ROLE_SRC:
		typec_role = TYPEC_ROLE_SRC;
		break;
	case TYPEC_PORT_ROLE_TRY_SNK:
		typec_role = TYPEC_ROLE_TRY_SNK;
		break;
	case TYPEC_PORT_ROLE_TRY_SRC:
		typec_role = TYPEC_ROLE_TRY_SRC;
		break;
	case TYPEC_PORT_ROLE_INVALID:
		typec_role = TYPEC_ROLE_UNKNOWN;
		break;
	case TYPEC_PORT_ROLE_DISABLE:
		oplus_usbtemp_set_cc_open();
		goto out;
	case TYPEC_PORT_ROLE_ENABLE:
		oplus_usbtemp_recover_cc_open();
		goto out;
	}

	rc = tcpm_typec_change_role_postpone(pinfo->tcpc, typec_role, true);
	if (rc < 0)
		chg_err("can't set typec_role=%d, rc=%d\n", mode, rc);
out:
	return rc;
}

static int mtk_chg_set_otg_switch_status(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	/* TODO */
	struct mtk_charger *chip;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if(pinfo->wd0_detect) {
		chg_err("[OPLUS_CHG][%s]: wd0 is 1, should not set\n", __func__);
		return 0;
	}

	chg_err("[OPLUS_CHG][%s]: otg switch[%d]\n", __func__, en);
	tcpm_typec_change_role_postpone(pinfo->tcpc, en ? TYPEC_ROLE_DRP : TYPEC_ROLE_SNK, true);

	return 0;
}

static int mtk_chg_get_otg_switch_status(struct oplus_chg_ic_dev *ic_dev, bool *en)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	chg_err("[OPLUS_CHG][%s]: chip->otg_switch return\n", __func__);

	return 0;
}

static int mtk_chg_get_otg_enbale(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct mtk_charger *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*enable = chip->otg_enable;
	return 0;
}

static int mtk_chg_get_reverse_enbale(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct mtk_charger *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*enable = chip->reverse_enable;
	return 0;
}

static int mtk_chg_get_high_reverse_enbale(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct mtk_charger *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*enable = chip->high_reverse_enable;
	return 0;
}

static int mtk_chg_get_reverse_chg_svid(struct oplus_chg_ic_dev *ic_dev, uint32_t *svid)
{
	struct mtk_charger *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*svid = chip->reverse_chg_svid;
	return 0;
}

static int oplus_chg_get_rvs_chg_msg_type(struct oplus_chg_ic_dev *ic_dev, int *msg_type)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*msg_type = chip->msg_type;

	return 0;
}

static int oplus_chg_get_sink_req_pdo(struct oplus_chg_ic_dev *ic_dev, int *vbus_mv_t, int *ibus_ma_t)
{
	int ret = 0;
	struct tcpc_device *tcpc = NULL;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(tcpc, vbus_mv_t, ibus_ma_t);

	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_inquire_pd_contract fail, ret=%d\n", ret);
		return -EINVAL;
	}
	chg_info("request vbus_mv[%d], ibus_ma[%d]\n", *vbus_mv_t, *ibus_ma_t);

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) && !IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
static int tcpm_inquire_pd_contract_rdo_position(struct tcpc_device *tcpc,
	uint8_t *rdo_pos)
{
	int ret;
	struct pd_port *pd_port = &tcpc->pd_port;
	struct pe_data *pe_data = &pd_port->pe_data;

	ret = tcpm_check_pd_attached(tcpc);
	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_check_pd_attached fail, ret=%d\n", ret);
		return ret;
	}

	mutex_lock(&pd_port->pd_lock);
	if (pd_port->pe_data.explicit_contract)
		*rdo_pos = pe_data->selected_cap;
	else
		ret = TCPM_ERROR_NO_EXPLICIT_CONTRACT;
	mutex_unlock(&pd_port->pd_lock);

	return ret;
}
#endif

static int mtk_chg_get_reverse_chg_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) && !IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
	int ret = 0;
	struct tcpc_device *tcpc = NULL;
	uint8_t rdo_pos;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract_rdo_position(tcpc, &rdo_pos);
	if (ret != TCPM_SUCCESS) {
		*type = REVERSE_CHG_TYPE_UNKNOWN;
		chg_err("tcpm_inquire_pd_contract_rdo_position fail, ret=%d\n", ret);
		return -EINVAL;
	}
	chg_info("request rdo_pos[0x%X]\n", rdo_pos);
	if (((rdo_pos - 1) & 0x03) == 0x03)
		*type = REVERSE_CHG_TYPE_PPS;
	else if (rdo_pos)
		*type = REVERSE_CHG_TYPE_PD;
	else
		*type = REVERSE_CHG_TYPE_NORMAL;
	chg_info("request type[%d]\n", *type);
#endif
	return 0;
}

static int mtk_chg_get_source_plug_out(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct mtk_charger *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*enable = chip->source_plug_out;
	return 0;
}

static int mtk_chg_is_oplus_svid(struct oplus_chg_ic_dev *ic_dev, bool *oplus_svid)
{
	struct mtk_charger *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*oplus_svid = chip->pd_svooc;
	return 0;
}

static int mtk_chg_set_usbtemp_dischg_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct mtk_charger *chip;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip->usbtemp_dischg_reg_configurable) {
		oplus_chg_set_dischg_enable(en);
		chg_err("set_usbtemp_dischg_enable=%d\n", en);
	} else {
		oplus_chg_common_set_dischg_enable(chip, en);
		chg_err("common set_usbtemp_dischg_enable=%d\n", en);
	}
	return 0;
}

static int mtk_chg_get_usbtemp_dischg_status(struct oplus_chg_ic_dev *ic_dev, bool *en)
{
	struct mtk_charger *chip;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	/*
	 * The method controlled by pmic pin can not read status,
	 * the level is always one
	 */
	*en = true;

	return 0;
}

static int oplus_chg_get_vbat_pwr_vol(void);
static int mtk_chg_get_vbat_pwr(struct oplus_chg_ic_dev *ic_dev, int *vbat_pwr)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*vbat_pwr = oplus_chg_get_vbat_pwr_vol();

	return 0;
}

static int oplus_chg_get_usb_btb_temp_cal(void);
static int mtk_chg_get_usb_btb_temp(struct oplus_chg_ic_dev *ic_dev, int *usb_btb_temp)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*usb_btb_temp = oplus_chg_get_usb_btb_temp_cal();

	return 0;
}

static int oplus_chg_get_sub_battery_btb_temp_cal(void);
static int mtk_chg_get_batt_btb_temp(struct oplus_chg_ic_dev *ic_dev, int *batt_btb_temp)
{
	int batt_temp, sub_batt_temp;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	batt_temp = oplus_chg_get_battery_btb_temp_cal() / 10;

	if (pinfo->sub_batt_btb_temp_chan) {
		sub_batt_temp = oplus_chg_get_sub_battery_btb_temp_cal() / 10;
		*batt_btb_temp = batt_temp >= sub_batt_temp ? batt_temp : sub_batt_temp;
	} else {
		*batt_btb_temp = batt_temp;
	}

	return 0;
}
static int oplus_chg_get_subboard_temp(struct oplus_chg_ic_dev *ic_dev, int *temp)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*temp = oplus_get_subboard_temp();
	return 0;
}

static struct mtk_gauge* oplus_gauge_get_mtk_gauge(void)
{
	static struct mtk_gauge *gauge = NULL;
	struct power_supply *psy;

	if (gauge == NULL) {
		psy = power_supply_get_by_name("mtk-gauge");
		if (psy == NULL) {
			chg_err("psy is not rdy\n");
			return NULL;
		}

		gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
		if (gauge == NULL) {
			chg_err("mtk_gauge is not rdy\n");
			return NULL;
		}
		power_supply_put(psy);
	}
	return gauge;
}

static int oplus_gauge_get_battery_temp_adc_voltage(int *val)
{
	int ret = 0;
	static struct mtk_gauge *gauge = NULL;

	gauge = oplus_gauge_get_mtk_gauge();
	if (NULL == gauge) {
		chg_err("gauge is NULL.");
		return -EOPNOTSUPP;
	}

	if (IS_ERR(gauge->chan_bat_temp)) {
		chg_err("auxadc chan: baton(bat_temp) error\n");
		return -EOPNOTSUPP;
	}

	ret = iio_read_channel_processed(gauge->chan_bat_temp, val);
	if (ret < 0)
		chg_err("ret = %d", ret);

	chg_debug("bat_temp_volt = %d, ret = %d", *val, ret);

	return ret;
}

static int oplus_get_sub_btb_state(struct mtk_charger *info)
{
	int adc_voltage = 0;
	int state = BATT_BTB_STATE_NOT_SUPPORT;
	int rc = 0;

	if (NULL == info) {
		chg_err("info is NULL");
		return -ENODEV;
	}

	/* check if the btb_state can be get by battery temp adc which is by ADC channel. */
	if (info->sub_btb_valid_adc[0] || info->sub_btb_valid_adc[1]) {
		/* get the real batt temp adc voltage */
		rc = oplus_gauge_get_battery_temp_adc_voltage(&adc_voltage);
		if (rc < 0) {
			chg_err("get the battery temp adc failed, rc = %d", rc);
			return rc;
		}

		/* check the silicon sub_btb state */
		if (info->sub_btb_valid_adc[0] && !info->sub_btb_valid_adc[1]) {
			/*
			 * for silicon battery, when the valid temp_adc is less than 0.5V, the sub_btb is connected,
			 * otherwise, the sub_btb is disconnected.
			 */
			if (adc_voltage >= info->sub_btb_valid_adc[0])
				state = BATT_BTB_STATE_NOT_CONNECT;
			else
				state = BATT_BTB_STATE_CONNECT;
		}
	}
	chg_debug("adc_voltage = %d, adc_valid[%d - %d], state = %d",
		  adc_voltage, info->sub_btb_valid_adc[0],
		  info->sub_btb_valid_adc[1], state);

	return state;
}

static int oplus_gauge_get_sub_btb_state(struct oplus_chg_ic_dev *ic_dev,
					 enum oplus_sub_btb_state *state)
{
	struct mtk_charger *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	rc = oplus_get_sub_btb_state(chip);
	if (rc < 0) {
		chg_err("get sub_btb_state fail, rc = %d", rc);
		return rc;
	} else {
		*state = rc;
	}

	return 0;
}

static int oplus_chg_wls_boost_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	if (info == NULL) {
		chg_err("info is NULL");
		return -ENODEV;
	}

	rc = charger_dev_enable_otg(info->chg1_dev, en);
	if (rc < 0)
		chg_err("set boost enable fail, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_set_wls_boost_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	int rc;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	if (info == NULL) {
		chg_err("info is NULL");
		return -ENODEV;
	}

	if (vol_mv == WLS_TRX_MODE_VOL_MV && info->wls_boost_vol_start_mv > 0)
		vol_mv = info->wls_boost_vol_start_mv;
	if (vol_mv > info->wls_boost_vol_max_mv)
		vol_mv = info->wls_boost_vol_max_mv;
	rc =  charger_dev_set_boost_voltage_limit(info->chg1_dev, vol_mv * 1000);
	if (rc < 0)
		chg_err("set boost vol fail, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_set_wls_boost_curr_limit(struct oplus_chg_ic_dev *ic_dev, int curr_ma)
{
	int rc;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	if (info == NULL) {
		chg_err("info is NULL");
		return -ENODEV;
	}

	rc =  charger_dev_set_boost_current_limit(info->chg1_dev, curr_ma * 1000);
	if (rc < 0)
		chg_err("set curr limit fail, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_wls_input_suspend(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static int oplus_chg_set_wls_icl(struct oplus_chg_ic_dev *ic_dev, int icl_ma)
{
	int rc;
	struct mtk_charger *info = oplus_chg_ic_get_drvdata(ic_dev);

	if (info == NULL) {
		chg_err("info is NULL");
		return -ENODEV;
	}

	rc = charger_dev_set_input_current(info->chg1_dev, icl_ma * 1000);
	if (rc < 0)
		chg_err("set input current fail, rc=%d\n", rc);

	return rc;
}

static int oplus_chg_get_wls_icl(struct oplus_chg_ic_dev *ic_dev, int *icl_ma)
{
	return 0;
}

static int oplus_chg_get_wls_input_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	/*TODO*/
	return -1;
}

static int oplus_chg_get_wls_input_vol(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	return 0;
}

static int oplus_chg_wls_aicl_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return mtk_chg_aicl_enable(ic_dev, en);
}

static int oplus_chg_wls_aicl_rerun(struct oplus_chg_ic_dev *ic_dev)
{
	return mtk_chg_aicl_rerun(ic_dev);
}

static int oplus_chg_wls_set_usb_drv(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static void *oplus_chg_mt6895_gauge_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
		(func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, mtk_chg_gauge_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, mtk_chg_gauge_exit);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SUBBOARD_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SUBBOARD_TEMP,
				oplus_chg_get_subboard_temp);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SUB_BTB_CONNECT_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SUB_BTB_CONNECT_STATE,
					       oplus_gauge_get_sub_btb_state);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}
	return func;
}

static int mtk_chg_get_usb_aicl_enhance(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct mtk_charger *info;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	info = oplus_chg_ic_get_drvdata(ic_dev);
	*enable = info->usb_aicl_enhance;

	return 0;
}

static int mtk_chg_get_lpd_info(struct oplus_chg_ic_dev *ic_dev, u32 *buf, u32 flag)
{
#if IS_ENABLED(CONFIG_OPLUS_LIQUID_DETECTION)
	struct tcpc_device *tcpc = NULL;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}
	if (tcpc->ops && tcpc->ops->get_lpd_info)
		return tcpc->ops->get_lpd_info(tcpc, buf, flag);
	chg_err("get_lpd_info fail");
	return -ENOTSUPP;
#endif
	return -ENOTSUPP;
}

static void *oplus_chg_get_func(struct oplus_chg_ic_dev *ic_dev,
				enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
					       mtk_chg_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					       mtk_chg_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
					       mtk_chg_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
					       mtk_chg_smt_test);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_PRESENT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_PRESENT,
					       mtk_chg_input_present);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND,
					       mtk_chg_input_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND,
							mtk_chg_input_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND,
							mtk_chg_output_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND,
							mtk_chg_output_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ICL,
					       mtk_chg_set_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_ICL,
					       mtk_chg_get_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FCC,
					       mtk_chg_set_fcc);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FV,
					       mtk_chg_set_fv);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_FV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_FV,
					       mtk_chg_get_fv);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ITERM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ITERM,
					       mtk_chg_set_iterm);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL,
					       mtk_chg_set_rechg_vol);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR,
							mtk_chg_get_input_curr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL,
					       mtk_chg_get_input_vol);
		break;
	case OPLUS_IC_FUNC_OTG_BOOST_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_OTG_BOOST_ENABLE,
					       mtk_chg_otg_boost_enable);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_VOL,
					       mtk_chg_set_otg_boost_vol);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT,
							mtk_chg_set_otg_boost_curr_limit);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_ENABLE,
					       mtk_chg_aicl_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RERUN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RERUN,
					       mtk_chg_aicl_rerun);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RESET,
					       mtk_chg_aicl_reset);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION,
							mtk_chg_get_cc_orientation);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_HW_DETECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_HW_DETECT,
					       mtk_chg_get_hw_detect);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE,
							oplus_chg_get_power_role);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_TYPEC_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_TYPEC_STATE,
					       mtk_chg_get_typec_state);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_USB_AICL_ENHANCE,
					       mtk_chg_get_usb_aicl_enhance);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE,
							mtk_chg_get_charger_type);
		break;
	case OPLUS_IC_FUNC_BUCK_RERUN_BC12:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_RERUN_BC12,
					       mtk_chg_rerun_bc12);
		break;
	case OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE,
							mtk_chg_qc_detect_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE,
							mtk_chg_shipmode_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG,
					       mtk_chg_set_qc_config);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG,
					       mtk_chg_set_pd_config);
		break;
	case OPLUS_IC_FUNC_GET_SHUTDOWN_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SHUTDOWN_SOC,
					       mtk_chg_get_shutdown_soc);
		break;
	case OPLUS_IC_FUNC_BACKUP_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BACKUP_SOC,
					       mtk_chg_backup_soc);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS,
			mtk_chg_get_vbus_collapse_status);
		break;
	case OPLUS_IC_FUNC_GET_TYPEC_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_TYPEC_MODE,
					       mtk_chg_get_typec_mode);
		break;
	case OPLUS_IC_FUNC_SET_TYPEC_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_TYPEC_MODE,
					       mtk_chg_set_typec_mode);
		break;
	case OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS,
					       mtk_chg_set_otg_switch_status);
		break;
	case OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS,
					       mtk_chg_get_otg_switch_status);
		break;
	case OPLUS_IC_FUNC_GET_OTG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_ENABLE,
					       mtk_chg_get_otg_enbale);
		break;
	case OPLUS_IC_FUNC_GET_SOURCE_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SOURCE_PDO,
					       mtk_chg_get_source_pdo);
		break;
	case OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE,
					       mtk_chg_set_usbtemp_dischg_enable);
		break;
	case OPLUS_IC_FUNC_GET_USB_DISCHG_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_USB_DISCHG_STATUS,
					       mtk_chg_get_usbtemp_dischg_status);
		break;
	case OPLUS_IC_FUNC_IS_OPLUS_SVID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_IS_OPLUS_SVID,
					       mtk_chg_is_oplus_svid);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_USB_BTB_TEMP,
					       mtk_chg_get_usb_btb_temp);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_BATT_BTB_TEMP,
					       mtk_chg_get_batt_btb_temp);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_VBAT_PWR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_VBAT_PWR,
					       mtk_chg_get_vbat_pwr);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_AICL_POINT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_AICL_POINT,
					       oplus_mt6375_set_aicl_point);
		break;
	case OPLUS_IC_FUNC_GET_TYPEC_ROLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_TYPEC_ROLE,
					       mtk_chg_get_typec_role);
		break;
	case OPLUS_IC_FUNC_WLS_BOOST_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_WLS_BOOST_ENABLE, oplus_chg_wls_boost_enable);
		break;
	case OPLUS_IC_FUNC_SET_WLS_BOOST_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_WLS_BOOST_VOL, oplus_chg_set_wls_boost_vol);
		break;
	case OPLUS_IC_FUNC_SET_WLS_BOOST_CURR_LIMIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_WLS_BOOST_CURR_LIMIT, oplus_chg_set_wls_boost_curr_limit);
		break;
	case OPLUS_IC_FUNC_BUCK_WLS_INPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WLS_INPUT_SUSPEND, oplus_chg_wls_input_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_WLS_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_WLS_ICL, oplus_chg_set_wls_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_WLS_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_WLS_ICL, oplus_chg_get_wls_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_CURR, oplus_chg_get_wls_input_curr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_WLS_INPUT_VOL, oplus_chg_get_wls_input_vol);
		break;
	case OPLUS_IC_FUNC_BUCK_WLS_AICL_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WLS_AICL_ENABLE, oplus_chg_wls_aicl_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_WLS_AICL_RERUN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WLS_AICL_RERUN, oplus_chg_wls_aicl_rerun);
		break;
	case OPLUS_IC_FUNC_BUCK_DIS_INSERT_DETECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_DIS_INSERT_DETECT, oplus_chg_wls_set_usb_drv);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_LPD_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_LPD_INFO, mtk_chg_get_lpd_info);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_VINDPM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_VINDPM,
					       oplus_chg_wls_set_vindpm);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq mtk_chg_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_CC_DETECT },
	{ .virq_id = OPLUS_IC_VIRQ_PLUGIN },
	{ .virq_id = OPLUS_IC_VIRQ_CC_CHANGED },
	{ .virq_id = OPLUS_IC_VIRQ_CHG_TYPE_CHANGE },
	{ .virq_id = OPLUS_IC_VIRQ_OTG_ENABLE },
	{ .virq_id = OPLUS_IC_VIRQ_SVID },
	{ .virq_id = OPLUS_IC_VIRQ_TYPEC_STATE },
	{ .virq_id = OPLUS_IC_VIRQ_PD_COMPLETED },
	{.virq_id = OPLUS_IC_VIRQ_POWER_CHANGED },
	{.virq_id = OPLUS_IC_VIRQ_POWER_ROLE_STATUS },
};

struct oplus_chg_ic_virq oplus_chg_mt6895_gauge_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
};
#endif /* OPLUS_FEATURE_CHG_BASIC */

#ifdef OPLUS_FEATURE_CHG_BASIC
#define CHARGER_25C_VOLT	900
#define TEMP_25C		250

static struct temp_param sub_board_temp_table[] = {
	{-40, 4397119}, {-39, 4092874}, {-38, 3811717}, {-37, 3551749}, {-36, 3311236}, {-35, 3088599}, {-34, 2882396}, {-33, 2691310},
	{-32, 2514137}, {-31, 2349778}, {-30, 2197225}, {-29, 2055558}, {-28, 1923932}, {-27, 1801573}, {-26, 1687773}, {-25, 1581881},
	{-24, 1483100}, {-23, 1391113}, {-22, 1305413}, {-21, 1225531}, {-20, 1151037}, {-19, 1081535}, {-18, 1016661}, {-17,  956080},
	{-16,  899481}, {-15,  846579}, {-14,  797111}, {-13,  750834}, {-12,  707524}, {-11,  666972}, {-10,  628988}, {-9,   593342},
	{-8,   559931}, {-7,   528602}, {-6,   499212}, {-5,   471632}, {-4,   445772}, {-3,   421480}, {-2,   398652}, {-1,   377193},
	{0,    357012}, {1,    338006}, {2,    320122}, {3,    303287}, {4,    287434}, {5,    272500}, {6,    258426}, {7,    245160},
	{8,    232649}, {9,    220847}, {10,   209710}, {11,   199196}, {12,   189268}, {13,   179890}, {14,   171028}, {15,   162651},
	{16,   154726}, {17,   147232}, {18,   140142}, {19,   133432}, {20,   127080}, {21,   121066}, {22,   115368}, {23,   109970},
	{24,   104852}, {25,   100000}, {26,   95398 }, {27,   91032 }, {28,   86889 }, {29,   82956 }, {30,   79222 }, {31,   75675 },
	{32,   72306 }, {33,   69104 }, {34,   66061 }, {35,   63167 }, {36,   60415 }, {37,   57797 }, {38,   55306 }, {39,   52934 },
	{40,   50677 }, {41,   48528 }, {42,   46482 }, {43,   44533 }, {44,   42675 }, {45,   40904 }, {46,   39213 }, {47,   37601 },
	{48,   36063 }, {49,   34595 }, {50,   33195 }, {51,   31859 }, {52,   30584 }, {53,   29366 }, {54,   28203 }, {55,   27091 },
	{56,   26028 }, {57,   25013 }, {58,   24042 }, {59,   23113 }, {60,   22224 }, {61,   21374 }, {62,   20561 }, {63,   19782 },
	{64,   19036 }, {65,   18323 }, {66,   17640 }, {67,   16986 }, {68,   16360 }, {69,   15760 }, {70,   15184 }, {71,   14631 },
	{72,   14101 }, {73,   13592 }, {74,   13104 }, {75,   12635 }, {76,   12187 }, {77,   11757 }, {78,   11344 }, {79,   10947 },
	{80,   10566 }, {81,   10200 }, {82,     9848}, {83,     9510}, {84,     9185}, {85,     8873}, {86,     8572}, {87,     8283},
	{88,     8006}, {89,     7738}, {90,     7481}, {91,     7234}, {92,     6997}, {93,     6769}, {94,     6548}, {95,     6337},
	{96,     6132}, {97,     5934}, {98,     5744}, {99,     5561}, {100,    5384}, {101,    5214}, {102,    5051}, {103,    4893},
	{104,    4741}, {105,    4594}, {106,    4453}, {107,    4316}, {108,    4184}, {109,    4057}, {110,    3934}, {111,    3816},
	{112,    3701}, {113,    3591}, {114,    3484}, {115,    3380}, {116,    3281}, {117,    3185}, {118,    3093}, {119,    3003},
	{120,    2916}, {121,    2832}, {122,    2751}, {123,    2672}, {124,    2596}, {125,    2522}, {125,    2522}, {125,    2522},
	{125,    2522}, {125,    2522}, {125,    2522}, {125,    2522}, {125,    2522}, {125,    2522}, {125,    2522}, {125,    2522},
	{125,    2522}, {125,    2522}, {125,    2522}, {125,    2522},
};

static struct temp_param charger_ic_temp_table[] = {
	{-40, 4251000}, {-39, 3962000}, {-38, 3695000}, {-37, 3447000}, {-36, 3218000}, {-35, 3005000},
	{-34, 2807000}, {-33, 2624000}, {-32, 2454000}, {-31, 2296000}, {-30, 2149000}, {-29, 2012000},
	{-28, 1885000}, {-27, 1767000}, {-26, 1656000}, {-25, 1554000}, {-24, 1458000}, {-23, 1369000},
	{-22, 1286000}, {-21, 1208000}, {-20, 1135000}, {-19, 1068000}, {-18, 1004000}, {-17, 945000 },
	{-16, 889600 }, {-15, 837800 }, {-14, 789300 }, {-13, 743900 }, {-12, 701300 }, {-11, 661500 },
	{-10, 624100 }, {-9, 589000  }, {-8, 556200  }, {-7, 525300  }, {-6, 496300  }, {-5, 469100  },
	{-4, 443500  }, {-3, 419500  }, {-2, 396900  }, {-1, 375600  }, {0, 355600   }, {1, 336800   },
	{2, 319100   }, {3, 302400   }, {4, 286700   }, {5, 271800   }, {6, 257800   }, {7, 244700   },
	{8, 232200   }, {9, 220500   }, {10, 209400  }, {11, 198900  }, {12, 189000  }, {13, 179700  },
	{14, 170900  }, {15, 162500  }, {16, 154600  }, {17, 147200  }, {18, 140100  }, {19, 133400  },
	{20, 127000  }, {21, 121000  }, {22, 115400  }, {23, 110000  }, {24, 104800  }, {25, 100000  },
	{26, 95400   }, {27, 91040   }, {28, 86900   }, {29, 82970   }, {30, 79230   }, {31, 75690   },
	{32, 72320   }, {33, 69120   }, {34, 66070   }, {35, 63180   }, {36, 60420   }, {37, 57810   },
	{38, 55310   }, {39, 52940   }, {40, 50680   }, {41, 48530   }, {42, 46490   }, {43, 44530   },
	{44, 42670   }, {45, 40900   }, {46, 39210   }, {47, 37600   }, {48, 36060   }, {49, 34600   },
	{50, 33190   }, {51, 31860   }, {52, 30580   }, {53, 29360   }, {54, 28200   }, {55, 27090   },
	{56, 26030   }, {57, 25010   }, {58, 24040   }, {59, 23110   }, {60, 22220   }, {61, 21370   },
	{62, 20560   }, {63, 19780   }, {64, 19040   }, {65, 18320   }, {66, 17640   }, {67, 16990   },
	{68, 16360   }, {69, 15760   }, {70, 15180   }, {71, 14630   }, {72, 14100   }, {73, 13600   },
	{74, 13110   }, {75, 12640   }, {76, 12190   }, {77, 11760   }, {78, 11350   }, {79, 10960   },
	{80, 10580   }, {81, 10210   }, {82, 9859    }, {83, 9522    }, {84, 9198    }, {85, 8887    },
	{86, 8587    }, {87, 8299    }, {88, 8022    }, {89, 7756    }, {90, 7500    }, {91, 7254    },
	{92, 7016    }, {93, 6788    }, {94, 6568    }, {95, 6357    }, {96, 6153    }, {97, 5957    },
	{98, 5768    }, {99, 5586    }, {100, 5410   }, {101, 5241   }, {102, 5078   }, {103, 4921   },
	{104, 4769   }, {105, 4623   }, {106, 4482   }, {107, 4346   }, {108, 4215   }, {109, 4088   },
	{110, 3965   }, {111, 3847   }, {112, 3733   }, {113, 3623   }, {114, 3517   }, {115, 3415   },
	{116, 3315   }, {117, 3220   }, {118, 3127   }, {119, 3038   }, {120, 2951   }, {121, 2868   },
	{122, 2787   }, {123, 2709   }, {124, 2633   }, {125, 2560   }
};

static __s16 oplus_ch_thermistor_conver_temp(__s32 res, struct ntc_temp *ntc_param)
{
	int i = 0;
	int asize = 0;
	__s32 res1 = 0, res2 = 0;
	__s32 tap_value = -2000, tmp1 = 0, tmp2 = 0;

	if (!ntc_param)
		return tap_value;

	asize = ntc_param->i_table_size;

	if (res >= ntc_param->pst_temp_table[0].temperature_r) {
		tap_value = ntc_param->i_tap_min;	/* min */
	} else if (res <= ntc_param->pst_temp_table[asize - 1].temperature_r) {
		tap_value = ntc_param->i_tap_max;	/* max */
	} else {
		res1 = ntc_param->pst_temp_table[0].temperature_r;
		tmp1 = ntc_param->pst_temp_table[0].bts_temp;

		for (i = 0; i < asize; i++) {
			if (res >= ntc_param->pst_temp_table[i].temperature_r) {
				res2 = ntc_param->pst_temp_table[i].temperature_r;
				tmp2 = ntc_param->pst_temp_table[i].bts_temp;
				break;
			}
			res1 = ntc_param->pst_temp_table[i].temperature_r;
			tmp1 = ntc_param->pst_temp_table[i].bts_temp;
		}

		tap_value = (((res - res2) * tmp1) + ((res1 - res) * tmp2)) * 10 / (res1 - res2);
	}

	return tap_value;
}


static __s16 oplus_res_to_temp(struct ntc_temp *ntc_param)
{
	__s32 tres;
	__u64 dwvcrich = 0;
	__s32 chg_tmp = -100;
	__u64 dwvcrich2 = 0;

	if (!ntc_param) {
		return TEMP_25C;
	}
	dwvcrich = ((__u64)ntc_param->i_tap_over_critical_low * (__u64)ntc_param->i_rap_pull_up_voltage);
	dwvcrich2 = (ntc_param->i_tap_over_critical_low + ntc_param->i_rap_pull_up_r);
	do_div(dwvcrich, dwvcrich2);

	if (ntc_param->ui_dwvolt > ((__u32)dwvcrich)) {
		tres = ntc_param->i_tap_over_critical_low;
	} else {
		tres = (ntc_param->i_rap_pull_up_r * ntc_param->ui_dwvolt) / (ntc_param->i_rap_pull_up_voltage - ntc_param->ui_dwvolt);
	}

	/* convert register to temperature */
	chg_tmp = oplus_ch_thermistor_conver_temp(tres, ntc_param);
	chg_info("tres :%d, chg_tmp :%d\n", tres, chg_tmp);
	return chg_tmp;
}

static int oplus_get_temp_volt(struct ntc_temp *ntc_param)
{
	int rc = 0;
	int ntc_temp_volt = 0;
	struct iio_channel *temp_chan = NULL;
	if (!ntc_param || !pinfo) {
		return -1;
	}
	switch (ntc_param->e_ntc_type) {
	case NTC_SUB_BOARD:
		if (!pinfo->subboard_temp_chan) {
			chg_err("subboard_temp_chan NULL\n");
			return -1;
		}
		temp_chan = pinfo->subboard_temp_chan;
		break;
	case NTC_CHARGER_IC:
		if (!pinfo->chargeric_temp_chan) {
			chg_err("chargeric_temp_chan NULL\n");
			return -1;
		}
		temp_chan = pinfo->chargeric_temp_chan;
#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
		if (pinfo->ntc_switch_gpio.ntc_switch_charge_ic)
			rc = oplus_set_ntc_switch_lock(CHARGE_IC, true);
#endif
		break;
	case NTC_BATTERY_BTB:
		if (!pinfo->batt_btb_temp_chan) {
			chg_err("batt_btb_temp_chan NULL\n");
			return -1;
		}
		temp_chan = pinfo->batt_btb_temp_chan;
		break;
	case NTC_SUB_BATTERY_BTB:
		if (!pinfo->sub_batt_btb_temp_chan) {
			chg_err("batt_btb_temp_chan NULL\n");
			return -1;
		}
		temp_chan = pinfo->sub_batt_btb_temp_chan;
		break;
	case NTC_USB_BTB:
		if (!pinfo->usb_btb_temp_chan) {
			chg_err("usb_btb_temp_chan NULL\n");
			return -1;
		}
		temp_chan = pinfo->usb_btb_temp_chan;
		break;
	case NTC_MASTER_CP_BTB:
		if (!pinfo->master_cp_chan) {
			chg_err("master_cp_chan NULL\n");
			return -1;
		}
		temp_chan = pinfo->master_cp_chan;
		break;
	case NTC_SLAVE_CP_BTB:
		if (!pinfo->slave_cp_chan) {
			chg_err("slave_cp_chan NULL\n");
			return -1;
		}
		temp_chan = pinfo->slave_cp_chan;
		break;
	case NTC_VBAT_PWR_BTB:
		if (!pinfo->vbat_pwr_chan) {
			chg_err("vbat_pwr_chan NULL\n");
			return -1;
		}
		temp_chan = pinfo->vbat_pwr_chan;
		break;
	default:
		break;
	}

	if (!temp_chan) {
		chg_err("unsupported ntc %d\n", ntc_param->e_ntc_type);
		return -1;
	}
	rc = iio_read_channel_processed(temp_chan, &ntc_temp_volt);
#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
	switch (ntc_param->e_ntc_type) {
	case NTC_CHARGER_IC:
		if (pinfo->ntc_switch_gpio.ntc_switch_charge_ic)
			oplus_set_ntc_switch_lock(CHARGE_IC, false);
		break;
	default:
		break;
	}
#endif

	if (rc < 0) {
		chg_err("read ntc_temp_chan volt failed, rc=%d\n", rc);
	}

	if (ntc_temp_volt < 0) {
		ntc_temp_volt = ntc_param->i_25c_volt;
	}
	chg_info("e_ntc_type = %d,ntc_temp_volt = %d\n", ntc_param->e_ntc_type, ntc_temp_volt);
	return ntc_temp_volt;
}

static int oplus_adc_vin_get_volt(void)
{
	unsigned int tia_data_register = 0;
	int vin_volt = 0;
	void __iomem *tia_addr = NULL;

	tia_addr = ioremap(adc_vin_reg, 0x4);
	if (!tia_addr)
		chg_err("tia_addr ioremap failed\n");

	if (tia_addr) {
		tia_data_register = readl(tia_addr);
		tia_data_register &=  ~(1 << 15); /* mtk provide need mask msb */
		vin_volt = ((unsigned long long)tia_data_register * 1840) >> 15; /* mtk provide tia_data_register/32768*1.84 */
		chg_info("read after mask MSB tia_data_register = 0x%x, read TIA VIN Volt = %d\n",
			tia_data_register, vin_volt);
	} else {
		tia_data_register = 0;
		chg_err("tia_addr is NULL\n");
	}

	if (!tia_addr)
		iounmap(tia_addr);

	return vin_volt;
}

static int oplus_chg_get_vbat_pwr_vol(void)
{
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};
	int vol_temp = 0;

	if (!pinfo) {
		chg_err("null pinfo\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_VBAT_PWR_BTB;
		ntc_param.i_tap_over_critical_low = 4397119;
		ntc_param.i_rap_pull_up_r = 100000;
		ntc_param.i_rap_pull_up_voltage = 1800;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		ntc_param.pst_temp_table = sub_board_temp_table;
		ntc_param.i_table_size = (sizeof(sub_board_temp_table) / sizeof(struct temp_param));
		is_param_init = true;

		chg_debug("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n", \
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r, \
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}
	vol_temp = oplus_get_temp_volt(&ntc_param);
	if (vol_temp < 0) {
		chg_err("get vbus pwr volt failed, use default 25c volt\n");
		pinfo->vbat_pwr_vol = ntc_param.i_25c_volt;
	} else {
		pinfo->vbat_pwr_vol = (unsigned int)vol_temp;
	}
	ntc_param.ui_dwvolt = pinfo->vbat_pwr_vol;
	/* To maintain unit consistency with the Qcom platform, returns uV.*/

	chg_info("vbat power volt:%d\n", pinfo->vbat_pwr_vol);
	return pinfo->vbat_pwr_vol;
}

static int oplus_chg_get_battery_btb_temp_cal(void)
{
	int subboard_temp = 0;
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};

	if (!pinfo) {
		chg_err("null pinfo\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_BATTERY_BTB;
		ntc_param.i_tap_over_critical_low = 4397119;
		ntc_param.i_rap_pull_up_r = 100000;
		ntc_param.i_rap_pull_up_voltage = 1800;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		ntc_param.pst_temp_table = sub_board_temp_table;
		ntc_param.i_table_size = (sizeof(sub_board_temp_table) / sizeof(struct temp_param));
		if (pinfo->ntc_temp_volt_1840mv) {
			ntc_param.i_tap_over_critical_low = 4251000;
			ntc_param.i_rap_pull_up_voltage = 1840;
			ntc_param.pst_temp_table = charger_ic_temp_table;
			ntc_param.i_table_size = (sizeof(charger_ic_temp_table) / sizeof(struct temp_param));
		}
		is_param_init = true;

		chg_err("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n", \
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r, \
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}
	ntc_param.ui_dwvolt = oplus_get_temp_volt(&ntc_param);

	if ((ntc_param.ui_dwvolt == -1) && (pinfo->support_adc_vin == true))
		ntc_param.ui_dwvolt = oplus_adc_vin_get_volt();

	subboard_temp = oplus_res_to_temp(&ntc_param);

	if (pinfo->support_ntc_01c_precision) {
		pinfo->i_sub_board_temp = subboard_temp;
	} else {
		pinfo->i_sub_board_temp = subboard_temp / 10;
	}
	if (pinfo->cp_btb_temp_share)
		pinfo->chargeric_temp = pinfo->i_sub_board_temp;
	chg_info("subboard_temp: %d,volt:%d\n", pinfo->i_sub_board_temp, ntc_param.ui_dwvolt);
	return pinfo->i_sub_board_temp;
}

static int oplus_chg_get_sub_battery_btb_temp_cal(void)
{
	int subboard_temp = 0;
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};

	if (!pinfo || !pinfo->sub_batt_btb_temp_chan) {
		chg_err("null pinfo\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_SUB_BATTERY_BTB;
		ntc_param.i_tap_over_critical_low = 4251000;
		ntc_param.i_rap_pull_up_r = 100000;
		ntc_param.i_rap_pull_up_voltage = 1840;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		ntc_param.pst_temp_table = charger_ic_temp_table;
		ntc_param.i_table_size = (sizeof(charger_ic_temp_table) / sizeof(struct temp_param));
		is_param_init = true;

		chg_err("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n", \
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r, \
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}
	ntc_param.ui_dwvolt = oplus_get_temp_volt(&ntc_param);
	subboard_temp = oplus_res_to_temp(&ntc_param);

	if (pinfo->support_ntc_01c_precision) {
		pinfo->i_sub_board_temp = subboard_temp;
	} else {
		pinfo->i_sub_board_temp = subboard_temp / 10;
	}
	chg_info("sub_bat_btb_temp: %d,volt:%d\n", pinfo->i_sub_board_temp, ntc_param.ui_dwvolt);
	return pinfo->i_sub_board_temp;
}


static int oplus_chg_get_usb_btb_temp_cal(void)
{
	int subboard_temp = 0;
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};

	if (!pinfo) {
		chg_err("null pinfo\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_USB_BTB;
		ntc_param.i_tap_over_critical_low = 4397119;
		ntc_param.i_rap_pull_up_r = 100000;
		ntc_param.i_rap_pull_up_voltage = 1800;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		ntc_param.pst_temp_table = sub_board_temp_table;
		ntc_param.i_table_size = (sizeof(sub_board_temp_table) / sizeof(struct temp_param));
		if (pinfo->ntc_temp_volt_1840mv) {
			ntc_param.i_tap_over_critical_low = 4251000;
			ntc_param.i_rap_pull_up_voltage = 1840;
			ntc_param.pst_temp_table = charger_ic_temp_table;
			ntc_param.i_table_size = (sizeof(charger_ic_temp_table) / sizeof(struct temp_param));
		}
		is_param_init = true;

		chg_err("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n", \
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r, \
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}
	ntc_param.ui_dwvolt = oplus_get_temp_volt(&ntc_param);
	subboard_temp = oplus_res_to_temp(&ntc_param);

	if (pinfo->support_ntc_01c_precision) {
		pinfo->i_sub_board_temp = subboard_temp;
	} else {
		pinfo->i_sub_board_temp = subboard_temp / 10;
	}
	pinfo->i_sub_board_temp = subboard_temp / 10;/*bringup 6895*/
	chg_info("subboard_temp:%d,volt:%d\n", pinfo->i_sub_board_temp, ntc_param.ui_dwvolt);
	return pinfo->i_sub_board_temp;
}


static int oplus_get_subboard_temp(void)
{
	int temp = 0;
	int sub_board_temp = 0;
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};
	int i = 0;

	if (!pinfo) {
		chg_err("null pinfo\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_SUB_BOARD;
		ntc_param.i_tap_over_critical_low = 4397119;
		ntc_param.i_rap_pull_up_r = 100000;
		ntc_param.i_rap_pull_up_voltage = 1840;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		ntc_param.pst_temp_table = sub_board_temp_table;
		ntc_param.i_table_size = (sizeof(sub_board_temp_table) / sizeof(struct temp_param));
		is_param_init = true;

		if (pinfo->subboard_parameters.parameters_support) {
			ntc_param.i_tap_over_critical_low = pinfo->subboard_parameters.i_tap_over_critical_low;
			ntc_param.i_rap_pull_up_r = pinfo->subboard_parameters.i_rap_pull_up_r;
			ntc_param.i_rap_pull_up_voltage = pinfo->subboard_parameters.i_rap_pull_up_voltage;
			ntc_param.i_tap_min = pinfo->subboard_parameters.i_tap_min;
			ntc_param.i_tap_max = pinfo->subboard_parameters.i_tap_max;
			ntc_param.i_25c_volt = pinfo->subboard_parameters.i_25c_volt;
			if (pinfo->subboard_parameters.table_support && pinfo->support_subboard_table) {
				for (i = 0; i < SUPPORT_TABLE_SIZE / 2; i++) {
					ntc_param.pst_temp_table[i].temperature_r = pinfo->support_subboard_table[i].temperature_r;
					ntc_param.pst_temp_table[i].bts_temp = pinfo->support_subboard_table[i].bts_temp;
				}
				ntc_param.i_table_size = (SUPPORT_TABLE_SIZE / 2);
				kvfree(pinfo->support_subboard_table);
				pinfo->support_subboard_table = NULL;
			}
		} else if (pinfo->sub_board_pull_up_r)
			ntc_param.i_rap_pull_up_r = pinfo->sub_board_pull_up_r;
		chg_info("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n", \
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r, \
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}
	ntc_param.ui_dwvolt = oplus_get_temp_volt(&ntc_param);
	temp = oplus_res_to_temp(&ntc_param);

	if (pinfo->support_ntc_01c_precision)
		sub_board_temp = temp;
	else
		sub_board_temp = temp / 10;

	chg_info("temp:%d, volt:%d\n", sub_board_temp, ntc_param.ui_dwvolt);

	return sub_board_temp;
}

int oplus_force_get_subboard_temp(void)
{
	return oplus_get_subboard_temp();
}
EXPORT_SYMBOL(oplus_force_get_subboard_temp);

static int oplus_get_master_cp_temp(void)
{
	int temp = 0;
	int master_cp_temp = 0;
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};

	if (!pinfo) {
		chg_err("null pinfo\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_MASTER_CP_BTB;
		ntc_param.i_tap_over_critical_low = 4397119;
		ntc_param.i_rap_pull_up_r = 100000;
		ntc_param.i_rap_pull_up_voltage = 1840;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		ntc_param.pst_temp_table = sub_board_temp_table;
		ntc_param.i_table_size = (sizeof(sub_board_temp_table) / sizeof(struct temp_param));
		if (pinfo->ntc_temp_volt_1840mv) {
			ntc_param.i_tap_over_critical_low = 4251000;
			ntc_param.i_rap_pull_up_voltage = 1840;
			ntc_param.pst_temp_table = charger_ic_temp_table;
			ntc_param.i_table_size = (sizeof(charger_ic_temp_table) / sizeof(struct temp_param));
		}
		is_param_init = true;

		chg_info("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n", \
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r, \
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}
	ntc_param.ui_dwvolt = oplus_get_temp_volt(&ntc_param);
	temp = oplus_res_to_temp(&ntc_param);

	if (pinfo->support_ntc_01c_precision) {
		master_cp_temp = temp;
		pinfo->chargeric_temp = temp;
	} else {
		master_cp_temp = temp / 10;
		pinfo->chargeric_temp = temp / 10;
	}
	chg_info("temp:%d, volt:%d\n", master_cp_temp, ntc_param.ui_dwvolt);

	return master_cp_temp;
}

static int oplus_get_slave_cp_temp(void)
{
	int temp = 0;
	int slave_cp_temp = 0;
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};

	if (!pinfo) {
		chg_err("null pinfo\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_SLAVE_CP_BTB;
		ntc_param.i_tap_over_critical_low = 4397119;
		ntc_param.i_rap_pull_up_r = 100000;
		ntc_param.i_rap_pull_up_voltage = 1840;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		ntc_param.pst_temp_table = sub_board_temp_table;
		ntc_param.i_table_size = (sizeof(sub_board_temp_table) / sizeof(struct temp_param));
		if (pinfo->ntc_temp_volt_1840mv) {
			ntc_param.i_tap_over_critical_low = 4251000;
			ntc_param.i_rap_pull_up_voltage = 1840;
			ntc_param.pst_temp_table = charger_ic_temp_table;
			ntc_param.i_table_size = (sizeof(charger_ic_temp_table) / sizeof(struct temp_param));
		}
		is_param_init = true;

		chg_info("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n", \
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r, \
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}
	ntc_param.ui_dwvolt = oplus_get_temp_volt(&ntc_param);
	temp = oplus_res_to_temp(&ntc_param);

	if (pinfo->support_ntc_01c_precision)
		slave_cp_temp = temp;
	else
		slave_cp_temp = temp / 10;

	chg_info("temp:%d, volt:%d\n", slave_cp_temp, ntc_param.ui_dwvolt);

	return slave_cp_temp;
}

static int oplus_get_chargeric_temp(void)
{
	int chargeric_temp = 0;
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};

	if (!pinfo) {
		chg_err("null pinfo\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_CHARGER_IC;
		ntc_param.i_tap_over_critical_low = 4251000;
		ntc_param.i_rap_pull_up_r = 100000;
		ntc_param.i_rap_pull_up_voltage = 1840;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		ntc_param.pst_temp_table = charger_ic_temp_table;
		ntc_param.i_table_size = (sizeof(charger_ic_temp_table) / sizeof(struct temp_param));
		is_param_init = true;
		chg_err("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n", \
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r, \
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}

	ntc_param.ui_dwvolt = oplus_get_temp_volt(&ntc_param);
	chargeric_temp = oplus_res_to_temp(&ntc_param);

	if (pinfo->support_ntc_01c_precision) {
		pinfo->chargeric_temp = chargeric_temp;
	} else {
		pinfo->chargeric_temp = chargeric_temp / 10;
	}
	chg_debug("chargeric_temp:%d,volt:%d\n", pinfo->chargeric_temp, ntc_param.ui_dwvolt);

	return pinfo->chargeric_temp;
}

int oplus_mt6375_get_tchg(int *tchg_min,	int *tchg_max)
{
	if (pinfo != NULL) {
		if (pinfo->chargeric_temp_chan)
			oplus_get_chargeric_temp();
		else if (pinfo->cp_btb_temp_share)
			oplus_chg_get_battery_btb_temp_cal();
		else
			oplus_get_master_cp_temp();

		*tchg_min = pinfo->chargeric_temp;
		*tchg_max = pinfo->chargeric_temp;
		return pinfo->chargeric_temp;
	}

	return -EBUSY;
}

bool oplus_tchg_01c_precision(void)
{
	if (!pinfo) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: charger_data not ready!\n", __func__);
		return false;
	}
	return pinfo->support_ntc_01c_precision;
}
EXPORT_SYMBOL(oplus_tchg_01c_precision);

static int oplus_chg_cp_pps_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int oplus_chg_cp_pps_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static void *oplus_chg_cp_pps_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_cp_pps_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_cp_pps_exit);
		break;
	case OPLUS_IC_FUNC_PPS_PDO_SET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_PPS_PDO_SET, oplus_chg_set_pps_config);
		break;
	case OPLUS_IC_FUNC_FIXED_PDO_SET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_FIXED_PDO_SET, oplus_chg_set_fixed_pd_config);
		break;
	case OPLUS_IC_FUNC_PPS_VERIFY_ADAPTER:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_PPS_VERIFY_ADAPTER, oplus_pps_get_authentiate);
		break;
	case OPLUS_IC_FUNC_PPS_GET_PDO_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_PPS_GET_PDO_INFO, oplus_get_pps_info);
		break;
	case OPLUS_IC_FUNC_GET_PPS_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_PPS_STATUS, oplus_chg_get_pps_status);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_chg_cp_pps_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
};

static int reverse_chg_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);
	return 0;
}

static int reverse_chg_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);
	return 0;
}

static int reverse_chg_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int  reverse_chg_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int  mtk_chg_set_wdt_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static int  mtk_chg_set_kick_wdt(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static void *oplus_chg_reverse_chg_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
					       reverse_chg_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					       reverse_chg_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
					       reverse_chg_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
					       reverse_chg_smt_test);
		break;
	case OPLUS_IC_FUNC_GET_REVERSE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_ENABLE,
					       mtk_chg_get_reverse_enbale);
		break;
	case OPLUS_IC_FUNC_GET_HIGH_REVERSE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_HIGH_REVERSE_ENABLE,
					       mtk_chg_get_high_reverse_enbale);
		break;
	case OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN,
							mtk_chg_set_rvs_high_mode_en);
		break;
	case OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO,
							mtk_chg_set_reverse_boost_pdo);
		break;
	case OPLUS_IC_FUNC_GET_REVERSE_CHG_SVID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_CHG_SVID,
					       mtk_chg_get_reverse_chg_svid);
		break;
	case OPLUS_IC_FUNC_REVERSE_CHG_WDT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REVERSE_CHG_WDT_ENABLE,
					       mtk_chg_set_wdt_enable);
		break;
	case OPLUS_IC_FUNC_REVERSE_CHG_KICK_WDT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REVERSE_CHG_KICK_WDT,
					       mtk_chg_set_kick_wdt);
		break;
	case OPLUS_IC_FUNC_GET_RVS_CHG_MSG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_RVS_CHG_MSG,
					       oplus_chg_get_rvs_chg_msg_type);
		break;
	case OPLUS_IC_FUNC_GET_SINK_REQ_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SINK_REQ_PDO,
					       oplus_chg_get_sink_req_pdo);
		break;
	case OPLUS_IC_FUNC_GET_REVERSE_CHG_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_CHG_TYPE,
					       mtk_chg_get_reverse_chg_type);
		break;
	case OPLUS_IC_FUNC_GET_SOURCE_PLUG_OUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SOURCE_PLUG_OUT,
					       mtk_chg_get_source_plug_out);
		break;
	case OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_NORMAL_REVERSE_HW_OCP,
					       mtk_chg_set_otg_boost_ocp_limit);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq oplus_chg_reverse_chg_virq_table[] = {
	{.virq_id = OPLUS_IC_VIRQ_ERR },
	{.virq_id = OPLUS_IC_VIRQ_ONLINE },
	{.virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{.virq_id = OPLUS_IC_VIRQ_REVERSE_ENABLE },
	{.virq_id = OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE },
	{.virq_id = OPLUS_IC_VIRQ_HARD_RESET },
	{.virq_id = OPLUS_IC_VIRQ_SINK_REQ_MSG },
};

static int oplus_chg_dpdm_switch_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	if (ic_dev->online)
		return 0;
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);

	return 0;
}

static int oplus_chg_dpdm_switch_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;
	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);

	return 0;
}

static int oplus_chg_dpdm_switch_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int oplus_chg_dpdm_switch_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int oplus_chg_dpdm_switch_set_switch_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_dpdm_switch_mode mode)
{
	struct mtk_charger *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	switch (mode) {
	case DPDM_SWITCH_TO_AP:
		chg_info("dpdm switch to ap\n");
		break;
	case DPDM_SWITCH_TO_VOOC:
		chg_info("dpdm switch to vooc\n");
		break;
	case DPDM_SWITCH_TO_UFCS:
		chg_info("dpdm switch to ufcs\n");
		break;
	default:
		chg_err("not supported mode, mode=%d\n", mode);
		return -EINVAL;
	}
	chip->dpdm_switch_mode = mode;

	return rc;
}

static int oplus_chg_dpdm_switch_get_switch_mode(struct oplus_chg_ic_dev *ic_dev, enum oplus_dpdm_switch_mode *mode)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*mode = chip->dpdm_switch_mode;

	return 0;
}

static void *oplus_chg_dpdm_switch_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}
	if (!oplus_chg_ic_func_is_support(ic_dev, func_id)) {
		chg_info("%s: this func(=%d) is not supported\n",  ic_dev->name, func_id);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, oplus_chg_dpdm_switch_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, oplus_chg_dpdm_switch_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, oplus_chg_dpdm_switch_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, oplus_chg_dpdm_switch_smt_test);
		break;
	case OPLUS_IC_FUNC_SET_DPDM_SWITCH_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_DPDM_SWITCH_MODE,
			oplus_chg_dpdm_switch_set_switch_mode);
		break;
	case OPLUS_IC_FUNC_GET_DPDM_SWITCH_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_DPDM_SWITCH_MODE,
			oplus_chg_dpdm_switch_get_switch_mode);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static struct oplus_chg_ic_virq oplus_chg_dpdm_switch_virq_table[] = {
	{.virq_id = OPLUS_IC_VIRQ_ERR},
	{.virq_id = OPLUS_IC_VIRQ_ONLINE},
	{.virq_id = OPLUS_IC_VIRQ_OFFLINE},
};

#endif /* OPLUS_FEATURE_CHG_BASIC */

static int oplus_mtk_ic_register(struct device *dev, struct mtk_charger *info)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct device_node *node = NULL;
	struct device_node *child;
	struct oplus_chg_ic_dev *ic_dev = NULL;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	int rc;

	if (NULL == dev || NULL == dev->of_node || NULL == info) {
		chg_err("input param is NULL\n");
		return -ENODEV;
	}
	node = dev->of_node;

	rc = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		return -ENODEV;
	}
	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		return -ENODEV;
	}

	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "buck-MTK6895");
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = mtk_chg_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(mtk_chg_virq_table);
	ic_cfg.of_node = node;
	pinfo->ic_dev = devm_oplus_chg_ic_register(&pinfo->pdev->dev, &ic_cfg);
	if (!pinfo->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		return rc;
	}
	chg_info("register %s\n", node->name);

	for_each_child_of_node(dev->of_node, child) {
		rc = of_property_read_u32(child, "oplus,ic_type", &ic_type);
		if (rc < 0) {
			chg_err("can't get %s ic type, rc=%d\n", child->name, rc);
			continue;
		}
		rc = of_property_read_u32(child, "oplus,ic_index", &ic_index);
		if (rc < 0) {
			chg_err("can't get %s ic index, rc=%d\n", child->name, rc);
			continue;
		}
		chg_info("node %s, ic_type = %d, ic_index = %d\n", child->name, ic_type, ic_index);

		ic_cfg.name = child->name;
		ic_cfg.index = ic_index;
		ic_cfg.type = ic_type;
		ic_cfg.of_node = child;

		switch (ic_type) {
		case OPLUS_CHG_IC_GAUGE:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "gauge-MTK6895");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_mt6895_gauge_get_func;
			ic_cfg.virq_data = oplus_chg_mt6895_gauge_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_mt6895_gauge_virq_table);
			break;
		case OPLUS_CHG_IC_PPS:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "pps-MTK6895");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_cp_pps_get_func;
			ic_cfg.virq_data = oplus_chg_cp_pps_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_cp_pps_virq_table);
			break;
		case OPLUS_CHG_IC_REVERSE:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "reverse-MTK6895");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_reverse_chg_get_func;
			ic_cfg.virq_data = oplus_chg_reverse_chg_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_reverse_chg_virq_table);
			break;
		case OPLUS_CHG_IC_MISC:
			snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "misc-dpdm-switch");
			snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
			ic_cfg.get_func = oplus_chg_dpdm_switch_get_func;
			ic_cfg.virq_data = oplus_chg_dpdm_switch_virq_table;
			ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_dpdm_switch_virq_table);
			break;
		default:
			chg_err("not support ic_type(=%d)\n", ic_type);
			continue;
		}

		ic_dev = devm_oplus_chg_ic_register(&pinfo->pdev->dev, &ic_cfg);
		if (!ic_dev) {
			rc = -ENODEV;
			chg_err("register %s error\n", child->name);
			continue;
		}
		chg_info("register %s success.\n", child->name);

		switch (ic_type) {
		case OPLUS_CHG_IC_GAUGE:
			info->gauge_ic_dev = ic_dev;
			break;
		case OPLUS_CHG_IC_PPS:
			info->pps_ic = ic_dev;
			oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_INIT);
			break;
		case OPLUS_CHG_IC_REVERSE:
			info->reverse_chg_ic_dev = ic_dev;
			oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_INIT);
			break;
		case OPLUS_CHG_IC_MISC:
			break;
		default:
			chg_err("not support ic_type(=%d) ...\n", ic_type);
			continue;
		}
		of_platform_populate(child, NULL, NULL, dev);
	}

	return 0;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static int oplus_chg_set_pps_config(struct oplus_chg_ic_dev *ic_dev, int vbus_mv, int ibus_ma)
{
	int ret = 0;
	int vbus_mv_t = 0;
	int ibus_ma_t = 0;
	struct tcpc_device *tcpc = NULL;

	chg_info("request vbus_mv[%d], ibus_ma[%d]\n", vbus_mv, ibus_ma);

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}

	if (tcpc->pd_port.dpm_charging_policy != DPM_CHARGING_POLICY_PPS) {
		ret = tcpm_set_apdo_charging_policy(tcpc, DPM_CHARGING_POLICY_PPS, vbus_mv, ibus_ma, NULL);
		if (ret == TCP_DPM_RET_REJECT) {
			chg_err("set_apdo_charging_policy reject\n");
			return 0;
		} else if (ret != 0) {
			chg_err("set_apdo_charging_policy error\n");
			return MTK_ADAPTER_ERROR;
		}
	}

	ret = tcpm_dpm_pd_request(tcpc, vbus_mv, ibus_ma, NULL);
	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_dpm_pd_request fail\n");
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(tcpc, &vbus_mv_t, &ibus_ma_t);
	if (ret != TCPM_SUCCESS) {
		chg_err("inquire current vbus_mv and ibus_ma fail\n");
		return -EINVAL;
	}

	chg_info("request vbus_mv[%d], ibus_ma[%d]\n", vbus_mv_t, ibus_ma_t);

	return 0;
}

static int oplus_chg_set_fixed_pd_config(struct oplus_chg_ic_dev *ic_dev, int vol_mv, int curr_ma)
{
	int rc = 0;

	if (curr_ma >= 2000)
		curr_ma = 2000;
	rc = oplus_chg_set_pd_config(vol_mv, curr_ma);

	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) && !IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
static int oplus_pps_get_authen_data(void)
{
	int i;
	int ret;
	/* Note: the structure is changed after kernel 6.1 */
	struct tcp_dpm_custom_vdm_data vdm_data;
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	for (i = 0; i < PPS_RANDOM_NUMBER; i++)
		get_random_bytes(&pps_random[i], 4);

	vdm_data.cnt = 5;
	vdm_data.wait_resp = true;
	vdm_data.data[0] = PD_SVDM_HDR(OPLUS_SVID, OPLUS_UVDM_AUTH_CMD);
	vdm_data.data[1] = pps_random[0];
	vdm_data.data[2] = pps_random[1];
	vdm_data.data[3] = pps_random[2];
	vdm_data.data[4] = pps_random[3];
	ret = tcpm_dpm_send_custom_vdm(tcpc_dev, &vdm_data, NULL);
	if (ret == TCPM_SUCCESS) {
		for (i = 0; i < vdm_data.cnt; i++) {
			if (i >= 1)
				pps_adapter_result[i-1] = vdm_data.data[i];
		}
	} else {
		chg_err("tcpm_dpm_send_custom_vdm fail(%d)\n", ret);
		return 0;
	}

	return ret;
}
#else
static int oplus_pps_get_authen_data(void)
{
	int i;
	int ret = 0;
	struct tcp_dpm_custom_vdm_data vdm_data;
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	for (i = 0; i < PPS_RANDOM_NUMBER; i++)
		get_random_bytes(&pps_random[i], 4);

	vdm_data.cnt = 5;
	vdm_data.wait_resp = true;

	vdm_data.vdos[0] = PD_SVDM_HDR(OPLUS_SVID, OPLUS_UVDM_AUTH_CMD);
	vdm_data.vdos[1] = pps_random[0];
	vdm_data.vdos[2] = pps_random[1];
	vdm_data.vdos[3] = pps_random[2];
	vdm_data.vdos[4] = pps_random[3];
	ret = tcpm_dpm_send_custom_vdm(tcpc_dev, &vdm_data, NULL);
	if (ret == TCPM_SUCCESS) {
		for (i = 0; i < vdm_data.cnt; i++) {
			if (i >= 1)
				pps_adapter_result[i-1] = vdm_data.vdos[i];
		}
	} else {
		chg_err("tcpm_dpm_send_custom_vdm fail(%d)\n", ret);
		return 0;
	}
	return ret;
}
#endif /* LINUX_VERSION_CODE >= 6.1 */

static void tea_encrypt(uint32_t *v, uint32_t *k)
{
	uint32_t i, temp_l, temp_h; /* set up */
	uint32_t sum = 0;

	for (i = 0; i < 32; i++) {
		sum += tea_delta;
		temp_l = ((v[1] << 4) + k[0]) ^ (v[1] + sum) ^ ((v[1] >> 5) + k[1]);
		temp_h = ((v[0] << 4) + k[2]) ^ (v[0] + sum) ^ ((v[0] >> 5) + k[3]);
		v[0] += temp_l;
		v[1] += temp_h;
	}
}

static void tea_encrypt2(uint32_t *v, uint32_t *k)
{
	uint32_t i, temp_l, temp_h; /* set up */
	uint32_t sum = 0;

	for (i = 0; i < 32; i++) {
		sum += tea_delta;
		temp_l = ((v[1] << 4) + k[0]) ^ (v[1] + sum) ^ ((v[1] >> 5) + k[1]);
		temp_h = ((v[0] << 4) + k[2]) ^ (v[0] + sum) ^ ((v[0] >> 5) + k[3]);
		v[0] += temp_l;
		v[1] += temp_h;
	}
	v[0] = ~v[0];
	v[1] = ~v[1];
}

void tea_encrypt_process(uint32_t *key)
{
	uint32_t  i;
	uint32_t  *k = (uint32_t *) key;
	uint32_t  loop = ((key[3] >> 29) & 0x07) + 1;

	pps_phone_result[0] = pps_random[0];
	pps_phone_result[1] = pps_random[1];
	pps_phone_result[2] = pps_random[2];
	pps_phone_result[3] = pps_random[3];

	pps_phone_result[2] = ~pps_phone_result[2];
	pps_phone_result[3] = ~pps_phone_result[3];
	for (i = 0; i < loop; i++) {
		tea_encrypt(pps_phone_result, k);
		tea_encrypt2(pps_phone_result + 2, k);
	}
	pps_phone_result[2] = ~pps_phone_result[2];
	pps_phone_result[3] = ~pps_phone_result[3];
}

void setup_new_key(uint32_t *new_key, uint32_t *key)
{
	uint32_t richtek_key1[4] = { 0 };
	uint32_t richtek_key2[4] = { 0 };

	memcpy(richtek_key1, private_key1, sizeof(uint32_t) * 4);
	memcpy(richtek_key2, private_key2, sizeof(uint32_t) * 4);
	memcpy(new_key, key, sizeof(uint32_t) * 4);
	tea_encrypt(new_key, richtek_key1);
	tea_encrypt(new_key + 2, richtek_key2);
}

bool oplus_pps_check_authen_data(void)
{
	uint32_t  key[4] = { 0 };
	uint32_t  new_key[4] = { 0 };
	int i;

	memcpy(key, publick_key, sizeof(uint32_t) * 4);
	setup_new_key(new_key, key);
	tea_encrypt_process(new_key);
	for (i = 0; i < 4; i++) {
		if (pps_adapter_result[i] != pps_phone_result[i])
			return false;
	}

	return true;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) && !IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
static bool oplus_pps_check_adapter_maxi(void)
{
	int ret;
	int i;
	struct tcp_dpm_custom_vdm_data vdm_data;
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	vdm_data.cnt = 2;
	vdm_data.wait_resp = true;
	vdm_data.data[0] = PD_UVDM_HDR(OPLUS_SVID, OPLUS_UVDM_POWER_CMD);
	vdm_data.data[1] = 0;
	for (i = 0; i < vdm_data.cnt; i++)
		chg_info("send uvdm_data1[%d] = 0x%08x\n", i, vdm_data.data[i]);
	ret = tcpm_dpm_send_custom_vdm(tcpc_dev, &vdm_data, NULL);
	if (ret != TCPM_SUCCESS) {
		chg_info("tcpm_dpm_send_custom_vdm fail(%d)\n", ret);
		return false;
	}

	return true;
}
#else
static bool oplus_pps_check_adapter_maxi(void)
{
	int ret;
	int i;
	struct tcp_dpm_custom_vdm_data vdm_data;
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	vdm_data.cnt = 2;
	vdm_data.wait_resp = true;

	vdm_data.vdos[0] = PD_UVDM_HDR(OPLUS_SVID, OPLUS_UVDM_POWER_CMD);
	vdm_data.vdos[1] = 0;
	for (i = 0; i < vdm_data.cnt; i++)
		chg_info("send uvdm_data1[%d] = 0x%08x\n", i, vdm_data.vdos[i]);
	ret = tcpm_dpm_send_custom_vdm(tcpc_dev, &vdm_data, NULL);
	if (ret != TCPM_SUCCESS) {
		chg_info("tcpm_dpm_send_custom_vdm fail(%d)\n", ret);
		return false;
	}

	return true;
}
#endif /* LINUX_VERSION_CODE >= 6.1 */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) && !IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
static int oplus_pps_enable_extended_maxi(void)
{
	int i;
	int ret;
	/* Note: the tcp_dpm_custom_vdm_data is changed after 6.1.0 */
	struct tcp_dpm_custom_vdm_data vdm_data;
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	vdm_data.cnt = 2;
	vdm_data.wait_resp = true;
	vdm_data.data[0] = PD_UVDM_HDR(OPLUS_SVID, OPLUS_UVDM_EXAPDO_CMD);
	vdm_data.data[1] = 0;
	for (i = 0; i < vdm_data.cnt; i++)
		chg_info("send uvdm_data2[%d] = 0x%08x\n", i, vdm_data.data[i]);

	ret = tcpm_dpm_send_custom_vdm(tcpc_dev, &vdm_data, NULL);
	if (ret == TCPM_SUCCESS) {
		for (i = 0; i < vdm_data.cnt; i++)
			chg_info("receive uvdm_data2[%d] = 0x%08x\n", i, vdm_data.data[i]);
	} else {
		chg_info("tcpm_dpm_send_custom_vdm fail(%d)\n", ret);
		return ret;
	}

	return ret;
}
#else
static int oplus_pps_enable_extended_maxi(void)
{
	int i;
	int ret;
	struct tcp_dpm_custom_vdm_data vdm_data;
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	vdm_data.cnt = 2;
	vdm_data.wait_resp = true;

	vdm_data.vdos[0] = PD_UVDM_HDR(OPLUS_SVID, OPLUS_UVDM_EXAPDO_CMD);
	vdm_data.vdos[1] = 0;
	for (i = 0; i < vdm_data.cnt; i++)
		chg_info("send uvdm_data2[%d] = 0x%08x\n", i, vdm_data.vdos[i]);

	ret = tcpm_dpm_send_custom_vdm(tcpc_dev, &vdm_data, NULL);
	if (ret == TCPM_SUCCESS) {
		for (i = 0; i < vdm_data.cnt; i++)
			chg_info("receive uvdm_data2[%d] = 0x%08x\n", i, vdm_data.vdos[i]);
	} else {
		chg_info("tcpm_dpm_send_custom_vdm fail(%d)\n", ret);
		return ret;
	}
	return ret;
}
#endif /* LINUX_VERSION_CODE >= 6.1 */

static int oplus_pps_get_authentiate(struct oplus_chg_ic_dev *ic_dev)
{
	struct tcpc_device *tcpc = NULL;
	bool auth;

#define AUTHENTIATE_PASS	1
#define AUTHENTIATE_FAILED	(-1)

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}

	if (tcpm_inquire_pd_data_role(tcpc) != TYPEC_HOST)
		tcpm_dpm_pd_data_swap(tcpc, TYPEC_HOST, NULL);

	oplus_pps_get_authen_data();
	auth = oplus_pps_check_authen_data();
	if (auth == false) {
		chg_err("pps check authen failed\n");
		if (tcpm_inquire_pd_data_role(tcpc) != TYPEC_DEVICE)
			tcpm_dpm_pd_data_swap(tcpc, TYPEC_DEVICE, NULL);
		return AUTHENTIATE_FAILED;
	}

	if (oplus_pps_check_adapter_maxi())
		oplus_pps_enable_extended_maxi();

	if (tcpm_inquire_pd_data_role(tcpc) != TYPEC_DEVICE)
		tcpm_dpm_pd_data_swap(tcpc, TYPEC_DEVICE, NULL);

	return AUTHENTIATE_PASS;
}

static int oplus_get_pps_info(struct oplus_chg_ic_dev *ic_dev, u32 *pdo, int num)
{
	struct tcpc_device *tcpc = NULL;
	struct tcpm_power_cap cap;
	int pdo_max = num > PPS_PDO_MAX ? PPS_PDO_MAX : num;
	int ret, pdo_index;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}
	ret = tcpm_dpm_pd_get_source_cap(tcpc, NULL);
	if (ret == TCPM_SUCCESS)
		chg_err("tcpm_dpm_pd_get_source_cap_ext ok(%d)\n", ret);
	else
		chg_err("tcpm_dpm_pd_get_source_cap_ext failed(%d)\n", ret);

	ret = tcpm_inquire_pd_source_cap(tcpc, &cap);
	if (ret != TCPM_SUCCESS)
		return ret;

	for (pdo_index = 0; pdo_index < pdo_max; pdo_index++)
		pdo[pdo_index] = cap.pdos[pdo_index];

	return 0;
}

static int oplus_chg_get_pps_status(struct oplus_chg_ic_dev *ic_dev, u32 *src_info)
{
	int tcpm_ret = TCPM_SUCCESS;
	struct pd_pps_status pps_status;
	struct tcpc_device *tcpc = NULL;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}

	tcpm_ret = tcpm_dpm_pd_get_pps_status(tcpc, NULL, &pps_status);
	if (tcpm_ret != 0)
		return tcpm_ret;

	*src_info = (PD_PPS_SET_OUTPUT_MV(pps_status.output_mv) | PD_PPS_SET_OUTPUT_MA(pps_status.output_ma) << 16);

	return 0;
}

static void init_pps_key(void)
{
	int ret = 0;
	int i = 0;
	char oplus_pps_offset[64] = { 0 };
	struct device_node *np = NULL;

	np = of_find_node_by_name(NULL, "oplus_pps");
	if (np) {
		for (i = 0; i < PPS_KEY_COUNT; i++) {
			sprintf(oplus_pps_offset, "Publick_key_%d", i);
			ret = of_property_read_u32(np, oplus_pps_offset, &(publick_key[i]));
			if (ret)
				chg_err(" publick_key[%d]: %s %d fail", i, oplus_pps_offset, publick_key[i]);
			chg_err(" publick_key[%d]: %s %d", i, oplus_pps_offset, publick_key[i]);
		}

		for (i = 0; i < PPS_KEY_COUNT; i++) {
			sprintf(oplus_pps_offset, "private_key1_%d", i);
			ret = of_property_read_u32(np, oplus_pps_offset, &(private_key1[i]));
			if (ret)
				chg_err(" private_key1[%d]: %s %d fail", i, oplus_pps_offset, private_key1[i]);
			chg_err(" private_key1[%d]: %s %d", i, oplus_pps_offset, private_key1[i]);
		}

		for (i = 0; i < PPS_KEY_COUNT; i++) {
			sprintf(oplus_pps_offset, "private_key2_%d", i);
			ret = of_property_read_u32(np, oplus_pps_offset, &(private_key2[i]));
			if (ret)
				chg_err(" private_key2[%d]: %s %d fail", i, oplus_pps_offset, private_key2[i]);
			chg_err(" private_key2[%d]: %s %d", i, oplus_pps_offset, private_key2[i]);
		}

		ret = of_property_read_u32(np, "tea_delta", &(tea_delta));
		if (ret) {
			chg_err(" tea_delta fail");
			return;
		}
		chg_err(" tea_delta = %d OK!\n", tea_delta);
	} else {
		chg_err(" np not found!\n");
	}
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

#ifdef CONFIG_THERMAL
static int master_cp_thermal_read_temp(struct thermal_zone_device *tzd,
		int *temp)
{
	int cp_temp;

	if (pinfo->cp_btb_temp_share)
		cp_temp = oplus_chg_get_battery_btb_temp_cal() * 100;
	else
		cp_temp = oplus_get_master_cp_temp() * 100; /* thermal zone need 0.001 */

	*temp = cp_temp;

	return 0;
}

static struct thermal_zone_device_ops master_cp_temp_tzd_ops = {
	.get_temp = master_cp_thermal_read_temp,
};

static int slave_cp_thermal_read_temp(struct thermal_zone_device *tzd,
		int *temp)
{
	int sub_cp_temp;

	if (pinfo->cp_sub_btb_temp_share)
		sub_cp_temp = oplus_chg_get_sub_battery_btb_temp_cal() *100; /* thermal zone need 0.001 */
	else
		sub_cp_temp = oplus_get_slave_cp_temp() * 100; /* thermal zone need 0.001 */

	*temp = sub_cp_temp;

	return 0;
}

static struct thermal_zone_device_ops slave_cp_temp_tzd_ops = {
	.get_temp = slave_cp_thermal_read_temp,
};

static int cp_thermal_read_temp(struct thermal_zone_device *tzd,
		int *temp)
{
	int cp_temp;

	cp_temp = oplus_chg_get_battery_btb_temp_cal() * 100; /* thermal zone need 0.001 */
	*temp = cp_temp;

	return 0;
}

static struct thermal_zone_device_ops cp_temp_tzd_ops = {
	.get_temp = cp_thermal_read_temp,
};

static int subborad_thermal_read_temp(struct thermal_zone_device *tzd, int *temp)
{
	if (temp != NULL)
		*temp = oplus_get_subboard_temp() * 100; /* thermal zone need 0.001 */

	return 0;
}

static struct thermal_zone_device_ops subboard_temp_tzd_ops = {
	.get_temp = subborad_thermal_read_temp,
};

#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
static int charger_ic_thermal_read_temp(struct thermal_zone_device *tzd, int *temp)
{
	if (temp != NULL)
		*temp = oplus_get_chargeric_temp() * 100; /* thermal zone need 0.001 */

	return 0;
}

static struct thermal_zone_device_ops charger_ic_temp_tzd_ops = {
	.get_temp = charger_ic_thermal_read_temp,
};
#endif

static int bat_sub_btb_thermal_read_temp(struct thermal_zone_device *tzd, int *temp)
{
	if (temp != NULL)
		*temp = oplus_chg_get_sub_battery_btb_temp_cal() * 100; /* thermal zone need 0.001 */

	return 0;
}

static struct thermal_zone_device_ops bat_sub_btb_temp_tzd_ops = {
	.get_temp = bat_sub_btb_thermal_read_temp,
};

static int bat_btb_thermal_read_temp(struct thermal_zone_device *tzd, int *temp)
{
	if (temp != NULL)
		*temp = oplus_chg_get_battery_btb_temp_cal() * 100; /* thermal zone need 0.001 */

	return 0;
}

static struct thermal_zone_device_ops bat_btb_temp_tzd_ops = {
	.get_temp = bat_btb_thermal_read_temp,
};

static struct thermal_zone_device *register_tz_device(const char *type,
	struct thermal_zone_device_ops *ops)
{
	struct thermal_zone_device* tzd = NULL;
	int ret;

#if(LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	tzd = thermal_tripless_zone_device_register(type, NULL, ops, NULL);
#else
	tzd = thermal_zone_device_register(type, 0, 0, NULL, ops, NULL, 0, 0);
#endif
	if (IS_ERR(tzd)) {
		chg_err("%s register fail", type);
		return NULL;
	}
	ret = thermal_zone_device_enable(tzd);
	if (ret) {
		chg_err("%s enable fail", type);
		thermal_zone_device_unregister(tzd);
	}

	return tzd;
}

static int register_tz_thermal(struct mtk_charger *info)
{
	if (info == NULL) {
		chg_err("info is null");
		return -ENODEV;
	}

	info->master_cp_temp_tzd = register_tz_device("master_cp_temp", &master_cp_temp_tzd_ops);
	info->slave_cp_temp_tzd = register_tz_device("slave_cp_temp", &slave_cp_temp_tzd_ops);

	info->cp_temp_tzd = register_tz_device("charge_pump_temp", &cp_temp_tzd_ops);
	info->bat_btb_temp_tzd = register_tz_device("bat_btb_temp", &bat_btb_temp_tzd_ops);
	info->bat_sub_btb_temp_tzd = register_tz_device("bat_sub_btb_temp", &bat_sub_btb_temp_tzd_ops);
	info->subboard_temp_tzd = register_tz_device("subboard_temp", &subboard_temp_tzd_ops);
#if IS_ENABLED(CONFIG_HORAE_FLASH_LED_THERMAL)
	if (info->ntc_switch_gpio.ntc_switch_charge_ic)
		info->charger_ic_temp_tzd = register_tz_device("charge_ic_temp", &charger_ic_temp_tzd_ops);
#endif
	return 0;
}
#endif /* CONFIG_THERMAL */

static int oplus_gauge_sub_btb_parse_dt(struct mtk_charger *chip, struct device *dev)
{
	int rc = 0;
	int i = 0;

	if (chip == NULL)
		return -ENODEV;

	rc = read_signed_data_from_node(dev->of_node, "oplus,sub_btb_valid_temp",
					(s32 *)(chip->sub_btb_valid_adc),
					OPLUS_SUB_BTB_MAX);
	if (rc < 0) {
		chg_err("get sub_btb_valid_temp, rc = %d\n", rc);
		for (i = 0; i < OPLUS_SUB_BTB_MAX - 1; i++)
			chip->sub_btb_valid_adc[i] = 0;
	} else {
		for (i = 0; i < OPLUS_SUB_BTB_MAX - 1; i++)
			chg_info(" valid_temp[%d] = %d", i, chip->sub_btb_valid_adc[i]);
	}

	return rc;
}

static int mtk_chg_set_vooc_status(int status)
{
#if IS_ENABLED(CONFIG_OPLUS_CANCEL_USB_SWITCH)
	struct tcpc_device *tcpc = NULL;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}
	chg_info("set_vooc_status:%d", status);
	if (tcpc->ops && tcpc->ops->set_vooc_status)
		return tcpc->ops->set_vooc_status(tcpc, !!status);

	chg_err("set_vooc_status fail");
#endif
	return -ENOTSUPP;
}

static void oplus_mt6375_vooc_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct mtk_charger *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case VOOC_ITEM_OLD_ADAPTER_STATUS:
			oplus_mms_get_item_data(chip->vooc_topic, id, &data,
						false);
			mtk_chg_set_vooc_status(data.intval);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_mt6375_subscribe_vooc_topic(struct oplus_mms *topic,
					   void *prv_data)
{
	struct mtk_charger *chip = prv_data;

	chip->vooc_topic = topic;
	chip->vooc_subs =
		oplus_mms_subscribe(chip->vooc_topic, chip,
				    oplus_mt6375_vooc_subs_callback, "mt6375");
	if (IS_ERR_OR_NULL(chip->vooc_subs)) {
		chg_err("subscribe vooc topic error, rc=%ld\n",
			PTR_ERR(chip->vooc_subs));
	}
}

static int mtk_charger_probe(struct platform_device *pdev)
{
	struct mtk_charger *info = NULL;
	int i;
	char *name = NULL;
	struct netlink_kernel_cfg cfg = {
		.input = chg_nl_data_handler,
	};
#ifdef OPLUS_FEATURE_CHG_BASIC
	int rc = 0;
#endif

	chr_info("starts\n");

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (!tcpc_dev_get_by_name("type_c_port0")) {
		chr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		return -EPROBE_DEFER;
	}

#endif /* OPLUS_FEATURE_CHG_BASIC */

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

	mtk_charger_parse_dt(info, &pdev->dev);

#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_gauge_sub_btb_parse_dt(info, &pdev->dev);
	pinfo = info;

	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev) {
		chg_err("found primary charger [%s]\n",
			info->chg1_dev->props.alias_name);
	} else {
		chg_err("can't find primary charger!\n");
	}
#endif
	if (info && info->chg1_dev) {
		charger_dev_enable_safety_timer(info->chg1_dev, false);
		chg_err("charger_dev_enable_safety_timer disable\n");
	}

	mutex_init(&info->cable_out_lock);
	mutex_init(&info->charger_lock);
	mutex_init(&info->pd_lock);

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s",
		"charger suspend wakelock");
	info->charger_wakelock =
		wakeup_source_register(NULL, name);
	spin_lock_init(&info->slock);

	init_waitqueue_head(&info->wait_que);
	info->polling_interval = CHARGING_INTERVAL;
	mtk_charger_init_timer(info);
#ifdef CONFIG_PM
	info->pm_notifier.notifier_call = charger_pm_event;
#endif /* CONFIG_PM */
	srcu_init_notifier_head(&info->evt_nh);
	mtk_charger_setup_files(pdev);
	mtk_charger_get_atm_mode(info);

	for (i = 0; i < CHGS_SETTING_MAX; i++) {
		info->chg_data[i].thermal_charging_current_limit = -1;
		info->chg_data[i].thermal_input_current_limit = -1;
		info->chg_data[i].input_current_limit_by_aicl = -1;
	}
	info->enable_hv_charging = true;

	info->psy_desc1.name = "mtk-master-charger";
	info->psy_desc1.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_desc1.properties = charger_psy_properties;
	info->psy_desc1.num_properties = ARRAY_SIZE(charger_psy_properties);
	info->psy_desc1.get_property = psy_charger_get_property;
	info->psy_desc1.set_property = psy_charger_set_property;
	info->psy_desc1.property_is_writeable =
			psy_charger_property_is_writeable;
	info->psy_desc1.external_power_changed =
		mtk_charger_external_power_changed;
	info->psy_cfg1.drv_data = info;
	info->psy1 = power_supply_register(&pdev->dev, &info->psy_desc1,
			&info->psy_cfg1);

	info->chg_psy = devm_power_supply_get_by_phandle(&pdev->dev,
		"charger");
	if (IS_ERR_OR_NULL(info->chg_psy))
		chr_err("%s: devm power fail to get chg_psy\n", __func__);

	info->bat_psy = devm_power_supply_get_by_phandle(&pdev->dev,
		"gauge");
	if (IS_ERR_OR_NULL(info->bat_psy))
		chr_err("%s: devm power fail to get bat_psy\n", __func__);

	if (IS_ERR(info->psy1))
		chr_err("register psy1 fail:%ld\n",
			PTR_ERR(info->psy1));

	info->psy_desc2.name = "mtk-slave-charger";
	info->psy_desc2.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_desc2.properties = charger_psy_properties;
	info->psy_desc2.num_properties = ARRAY_SIZE(charger_psy_properties);
	info->psy_desc2.get_property = psy_charger_get_property;
	info->psy_desc2.set_property = psy_charger_set_property;
	info->psy_desc2.property_is_writeable =
			psy_charger_property_is_writeable;
	info->psy_cfg2.drv_data = info;
	info->psy2 = power_supply_register(&pdev->dev, &info->psy_desc2,
			&info->psy_cfg2);

	if (IS_ERR(info->psy2))
		chr_err("register psy2 fail:%ld\n",
			PTR_ERR(info->psy2));

	info->psy_dvchg_desc1.name = "mtk-mst-div-charger";
	info->psy_dvchg_desc1.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_dvchg_desc1.properties = charger_psy_properties;
	info->psy_dvchg_desc1.num_properties =
		ARRAY_SIZE(charger_psy_properties);
	info->psy_dvchg_desc1.get_property = psy_charger_get_property;
	info->psy_dvchg_desc1.set_property = psy_charger_set_property;
	info->psy_dvchg_desc1.property_is_writeable =
		psy_charger_property_is_writeable;
	info->psy_dvchg_cfg1.drv_data = info;
	info->psy_dvchg1 = power_supply_register(&pdev->dev,
						 &info->psy_dvchg_desc1,
						 &info->psy_dvchg_cfg1);
	if (IS_ERR(info->psy_dvchg1))
		chr_err("register psy dvchg1 fail:%ld\n",
			PTR_ERR(info->psy_dvchg1));

	info->psy_dvchg_desc2.name = "mtk-slv-div-charger";
	info->psy_dvchg_desc2.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_dvchg_desc2.properties = charger_psy_properties;
	info->psy_dvchg_desc2.num_properties =
		ARRAY_SIZE(charger_psy_properties);
	info->psy_dvchg_desc2.get_property = psy_charger_get_property;
	info->psy_dvchg_desc2.set_property = psy_charger_set_property;
	info->psy_dvchg_desc2.property_is_writeable =
		psy_charger_property_is_writeable;
	info->psy_dvchg_cfg2.drv_data = info;
	info->psy_dvchg2 = power_supply_register(&pdev->dev,
						 &info->psy_dvchg_desc2,
						 &info->psy_dvchg_cfg2);
	if (IS_ERR(info->psy_dvchg2))
		chr_err("register psy dvchg2 fail:%ld\n",
			PTR_ERR(info->psy_dvchg2));

	info->log_level = CHRLOG_ERROR_LEVEL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	for (i = 0; i < MAX_TA_IDX;i++) {
		info->adapter_dev[i] =
			get_adapter_by_name(adapter_type_names[i]);
		if (!info->adapter_dev[i])
			chr_err("%s: No %s found\n", __func__, adapter_type_names[i]);
		else {
			info->ta_nb[i].nb.notifier_call = notify_adapter_event;
			info->ta_nb[i].info = info;
			register_adapter_device_notifier(info->adapter_dev[i],
					&(info->ta_nb[i].nb));
		}
	}
#else
	info->pd_adapter = get_adapter_by_name("pd_adapter");
	if (!info->pd_adapter)
		chr_err("No pd adapter found\n");
	else {
		info->pd_nb.notifier_call = notify_adapter_event;
		register_adapter_device_notifier(info->pd_adapter,
						 &info->pd_nb);
	}
#endif

	info->sc.daemo_nl_sk = netlink_kernel_create(&init_net, NETLINK_CHG, &cfg);

	if (info->sc.daemo_nl_sk == NULL)
		chr_err("sc netlink_kernel_create error id:%d\n", NETLINK_CHG);
	else
		chr_err("sc_netlink_kernel_create success id:%d\n", NETLINK_CHG);
	sc_init(&info->sc);

	info->chg_alg_nb.notifier_call = chg_alg_event;

	info->fast_charging_indicator = 0;
	info->is_charging = false;
	info->safety_timer_cmd = -1;

	kthread_run(charger_routine_thread, info, "charger_thread");

#ifdef OPLUS_FEATURE_CHG_BASIC
#if !IS_ENABLED(CONFIG_OPLUS_PD_MANAGER_CHARGER)
	pinfo->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!pinfo->tcpc) {
		chr_err("%s get tcpc device type_c_port0 fail\n", __func__);
	} else {
		pinfo->pd_nb.notifier_call = pd_tcp_notifier_call;
		rc = register_tcp_dev_notifier(pinfo->tcpc, &pinfo->pd_nb,
					TCP_NOTIFY_TYPE_ALL);
		if (rc < 0) {
			pr_err("%s: register tcpc notifer fail\n", __func__);
			return -EINVAL;
		}
	}
#endif
	pinfo->batt_btb_temp_chan = devm_iio_channel_get(&pdev->dev, "auxadc5_batt_btb_temp");
	if (IS_ERR(pinfo->batt_btb_temp_chan)) {
		chg_err("Couldn't get batt_btb_temp_chan...\n");
		pinfo->batt_btb_temp_chan = NULL;
	}

	pinfo->sub_batt_btb_temp_chan = devm_iio_channel_get(&pdev->dev, "sub_bat_btb_temp");
	if (IS_ERR(pinfo->sub_batt_btb_temp_chan)) {
		chg_err("Couldn't get sub_batt_btb_temp_chan...\n");
		pinfo->sub_batt_btb_temp_chan = NULL;
	}

	pinfo->usb_btb_temp_chan = devm_iio_channel_get(&pdev->dev, "auxadc6_usb_btb_temp");
	if (IS_ERR(pinfo->usb_btb_temp_chan)) {
		chg_err("Couldn't get usb_btb_temp_chan...\n");
		pinfo->usb_btb_temp_chan = NULL;
	}

	pinfo->subboard_temp_chan = devm_iio_channel_get(&pdev->dev, "auxadc1-ntc_sub_bat_tem");
	if (IS_ERR(pinfo->subboard_temp_chan)) {
		chg_err("Couldn't get subboard_temp_chan...\n");
		pinfo->subboard_temp_chan = NULL;
	}

	pinfo->chargeric_temp_chan = devm_iio_channel_get(&pdev->dev, "auxadc5-chargeric_temp");
	if (IS_ERR(pinfo->chargeric_temp_chan)) {
		chg_err("Couldn't get chargeric_temp_chan...\n");
		pinfo->chargeric_temp_chan = NULL;
	}

	pinfo->master_cp_chan = devm_iio_channel_get(&pdev->dev, "master_cp_temp");
	if (IS_ERR(pinfo->master_cp_chan)) {
		chg_err("Couldn't get master_cp_chan...\n");
		pinfo->master_cp_chan = NULL;
	}

	pinfo->slave_cp_chan = devm_iio_channel_get(&pdev->dev, "slave_cp_temp");
	if (IS_ERR(pinfo->slave_cp_chan)) {
		chg_err("Couldn't get slave_cp_chan...\n");
		pinfo->slave_cp_chan = NULL;
	}

	pinfo->vbus_chan = devm_iio_channel_get(&pdev->dev, "vbus");
	if (IS_ERR(pinfo->vbus_chan)) {
		chg_err("Couldn't get vbus...\n");
		pinfo->vbus_chan = NULL;
	}

	pinfo->vbat_pwr_chan = devm_iio_channel_get(&pdev->dev, "vbat_pwr_vol");
	if (IS_ERR(pinfo->vbat_pwr_chan)) {
		chg_err("Couldn't get vbat_pwr_chan...\n");
		pinfo->vbat_pwr_chan = NULL;
	}

	pinfo->hvdcp_disable = false;
	INIT_DELAYED_WORK(&pinfo->hvdcp_detect_work, oplus_hvdcp_detect_work);
	pinfo->pd_svooc = false;
	pinfo->oplus_pd_sdp_svid = false;
	INIT_DELAYED_WORK(&pinfo->detach_clean_work, oplus_detach_clean_work);

	pinfo->pd_chg_volt = VBUS_5V;
	INIT_DELAYED_WORK(&pinfo->sourcecap_done_work, oplus_sourcecap_done_work);
	INIT_DELAYED_WORK(&pinfo->charger_suspend_recovery_work, oplus_charger_suspend_recovery_work);
	INIT_DELAYED_WORK(&pinfo->publish_close_cp_item_work, oplus_publish_close_cp_item_work);
	INIT_DELAYED_WORK(&pinfo->svid_check_work, oplus_svid_check_work);
	INIT_DELAYED_WORK(&pinfo->reverse_chg_svid_check_work, oplus_reverse_chg_svid_check_work);
	INIT_DELAYED_WORK(&pinfo->sink_request_check_work, oplus_sink_request_check_work);
	INIT_DELAYED_WORK(&pinfo->source_pdo_check_work, oplus_source_pdo_check_work);

	if (oplus_mtk_ic_register(&pdev->dev, pinfo) != 0)
		goto reg_ic_err;

	init_pps_key();
#ifdef CONFIG_THERMAL
	rc = register_tz_thermal(pinfo);
	if (rc)
		chg_err("register tz fail");
#endif
	oplus_mt6375_set_chging_term_disable(info);
	oplus_mms_wait_topic("vooc", oplus_mt6375_subscribe_vooc_topic, pinfo);
reg_ic_err:
#endif

	chg_info("probe complete\n");
	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
static int mtk_charger_remove(struct platform_device *dev)
#else
static void mtk_charger_remove(struct platform_device *dev)
#endif
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#else
	return;
#endif
}

static void mtk_charger_shutdown(struct platform_device *dev)
{
	struct mtk_charger *info = platform_get_drvdata(dev);
	int i;

	for (i = 0; i < MAX_ALG_NO; i++) {
		if (info->alg[i] == NULL)
			continue;
		chg_alg_stop_algo(info->alg[i]);
	}
}

static const struct of_device_id mtk_charger_of_match[] = {
	{.compatible = "mediatek,charger", },
	{},
};

MODULE_DEVICE_TABLE(of, mtk_charger_of_match);

struct platform_device mtk_charger_device = {
	.name = "charger",
	.id = -1,
};

static struct platform_driver mtk_charger_driver = {
	.probe = mtk_charger_probe,
	.remove = mtk_charger_remove,
	.shutdown = mtk_charger_shutdown,
	.driver = {
		   .name = "charger",
		   .of_match_table = mtk_charger_of_match,
	},
};

static int __init mtk_charger_init(void)
{
	return platform_driver_register(&mtk_charger_driver);
}
#ifndef OPLUS_FEATURE_CHG_BASIC
module_init(mtk_charger_init);
#endif

static void __exit mtk_charger_exit(void)
{
	platform_driver_unregister(&mtk_charger_driver);
}
#ifndef OPLUS_FEATURE_CHG_BASIC
module_exit(mtk_charger_exit);
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
oplus_chg_module_register(mtk_charger);
#endif

MODULE_AUTHOR("wy.chuang <wy.chuang@mediatek.com>");
MODULE_DESCRIPTION("MTK Charger Driver");
MODULE_LICENSE("GPL");
