/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/kdev_t.h>

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>

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
#include <linux/alarmtimer.h>
#include <linux/version.h>
#include <mt-plat/mtk_boot_common.h>
#include <asm/setup.h>
#include "../oplus_chg_module.h"
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/mfd/mt6397/core.h>/* PMIC MFD core header */
#include <soc/oplus/system/oplus_project.h>
#include "../oplus_chg_track.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"
#include "../oplus_configfs.h"
#include "op_charge.h"
#ifdef CONFIG_OPLUS_PD_EXT_SUPPORT
#include "../pd_ext/inc/tcpci.h"
#include "../pd_ext/inc/tcpm.h"
#include "../pd_ext/inc/tcpci_typec.h"
#else
#include <tcpci.h>
#endif
#include <linux/iio/consumer.h>
#include "../oplus_pps.h"

bool is_mtksvooc_project = false;

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

/* Structure to define dependencies of MTK battery module code on OPLUS charger framework */
struct mtk_oplus_batt_interface
{
	bool(*set_charge_power_sel)(int select);
};
extern void register_mtk_oplus_batt_interfaces(struct mtk_oplus_batt_interface *intf);

extern int mtk_basic_charger_init(struct mtk_charger *info);
extern int mtk_pulse_charger_init(struct mtk_charger *info);
extern int get_uisoc(struct mtk_charger *info);
extern int get_battery_current(struct mtk_charger *info);
extern void register_mtk_oplus_chg_interfaces(struct mtk_oplus_chg_interface *intf);
bool is_disable_charger(struct mtk_charger *info);
bool is_typec_adapter(struct mtk_charger *info);
void mt_usb_connect_v1(void);
void mt_usb_disconnect_v1(void);
static int get_chargeric_temp(void);
extern int oplus_get_prop_status(void);

/* Structure to define dependencies of MTK charger module code on OPLUS charger framework */
struct mtk_oplus_chg_interface mtk_oplus_chg_intf = {
        .track_record_chg_type_info = oplus_chg_track_record_chg_type_info,
        .wake_update_work = oplus_chg_wake_update_work,
        .is_single_batt_svooc = is_vooc_support_single_batt_svooc,
        .set_otg_online = oplus_chg_set_otg_online,
        /* VOOC related */
        .set_charger_type_unknown = oplus_chg_set_charger_type_unknown,
        .clear_chargerid_info = oplus_chg_clear_chargerid_info,
        .set_chargerid_switch_val = oplus_chg_set_chargerid_switch_val,
        .get_fastchg_started = oplus_vooc_get_fastchg_started,
        .reset_fastchg_after_usbout = oplus_vooc_reset_fastchg_after_usbout,
        /* VOOCPHY related */
        .get_voocphy_support = oplus_chg_get_voocphy_support,
};


struct mtk_oplus_batt_interface *g_oplus_batt_intf;

extern int charger_ic_flag;
extern bool oplus_chg_check_chip_is_null(void);
extern bool oplus_tchg_01c_precision(void);

/************ kpoc_charger *******************/
extern int oplus_chg_get_ui_soc(void);
extern int oplus_chg_get_notify_flag(void);
extern int oplus_chg_show_vooc_logo_ornot(void);
extern bool pmic_chrdet_status(void);
extern int oplus_get_prop_status(void);
extern void sgm7220_set_typec_sinkonly(void);
extern void sgm7220_set_typec_cc_open(void);

struct oplus_chg_chip *g_oplus_chip = NULL;
int oplus_usb_switch_gpio_gpio_init(void);
static bool oplus_ship_check_is_gpio(struct oplus_chg_chip *chip);
int oplus_ship_gpio_init(struct oplus_chg_chip *chip);
void smbchg_enter_shipmode(struct oplus_chg_chip *chip);
bool oplus_shortc_check_is_gpio(struct oplus_chg_chip *chip);
int oplus_shortc_gpio_init(struct oplus_chg_chip *chip);
extern struct oplus_chg_operations  bq24190_chg_ops;
extern struct oplus_chg_operations  bq25890h_chg_ops;
extern struct oplus_chg_operations  bq25601d_chg_ops;
extern struct oplus_chg_operations  oplus_chg_rt9471_ops;
extern struct oplus_chg_operations  oplus_chg_rt9467_ops;
extern struct oplus_chg_operations  oplus_chg_bq2589x_ops;
extern struct oplus_chg_operations  oplus_chg_bq2591x_ops;
extern struct oplus_chg_operations  oplus_chg_sy6974_ops;
#ifdef CONFIG_CHARGER_SC6607
extern struct oplus_chg_operations  oplus_chg_sc6607_ops;
extern bool check_ntc_suport_1000k(void);
#endif
extern struct oplus_chg_operations  oplus_chg_sgm41512_ops;
struct iio_channel *usb_chan1; /*usb_temp_auxadc_channel 1*/
struct iio_channel *usb_chan2; /*usb_temp_auxadc_channel 2*/
static int usbtemp_log_control = 0;
static int usb_debug_temp = 65535;
static int charger_ic__det_flag = 0;
int ap_temp_debug = 65535;
module_param(usb_debug_temp, int, 0644);
module_param(ap_temp_debug, int, 0644);
module_param(usbtemp_log_control, int, 0644);
#define USBTEMP_LOG_PRINT(fmt, args...)					\
do {								\
	if (usbtemp_log_control) {	\
		pr_debug(fmt, ##args);				\
	}							\
} while (0)

#define OPLUS_MIN_PDO_VOL 5000
#define OPLUS_MIN_PDO_CUR 3000

static struct charger_manager_drvdata *pinfo_drvdata;
static struct list_head consumer_head = LIST_HEAD_INIT(consumer_head);
static DEFINE_MUTEX(consumer_mutex);

extern struct oplus_chg_operations * oplus_get_chg_ops(void);
extern int oplus_usbtemp_monitor_common(void *data);
extern int oplus_usbtemp_monitor_common_new_method(void *data);
extern void oplus_usbtemp_recover_func(struct oplus_chg_chip *chip);

extern enum alarmtimer_restart usbtemp_alarm_timer_func(struct alarm *alarm, ktime_t now);

extern bool oplus_gauge_ic_chip_is_null(void);
extern bool oplus_vooc_check_chip_is_null(void);
extern bool oplus_adapter_check_chip_is_null(void);
int oplus_tbatt_power_off_task_init(struct oplus_chg_chip *chip);
extern bool is_kthread_get_adc(void);
extern int get_charger_ntc_volt(void);
extern int get_bat_con_ntc_volt(void);

#define CHG_BQ25601D 2
#define CHG_BQ25890H 1
#define CHG_BQ24190  0
#define CHG_SMB1351  3
#define CHG_MT6370   4
#define DEFAULT_BATTERY_TMP_WHEN_ERROR	-400

extern int oplus_get_rtc_ui_soc(void);
extern int oplus_set_rtc_ui_soc(int value);

static struct mtk_charger *pinfo;

static struct task_struct *oplus_usbtemp_kthread;
static DECLARE_WAIT_QUEUE_HEAD(oplus_usbtemp_wq);
void oplus_set_otg_switch_status(bool value);
void oplus_wake_up_usbtemp_thread(void);
static void oplus_ccdetect_disable(void);
static void oplus_ccdetect_enable(void);
bool oplus_usbtemp_condition(void);
extern void oplus_chg_turn_off_charging(struct oplus_chg_chip *chip);

extern bool set_charge_power_sel(int);

static enum mtk_pd_connect_type pd_connect_tbl[] = {
	MTK_PD_CONNECT_NONE,
	MTK_PD_CONNECT_TYPEC_ONLY_SNK,
	MTK_PD_CONNECT_TYPEC_ONLY_SNK,
	MTK_PD_CONNECT_NONE,
	MTK_PD_CONNECT_PE_READY_SNK,
	MTK_PD_CONNECT_NONE,
	MTK_PD_CONNECT_PE_READY_SNK_PD30,
	MTK_PD_CONNECT_NONE,
	MTK_PD_CONNECT_PE_READY_SNK_APDO,
	MTK_PD_CONNECT_HARD_RESET,
	MTK_PD_CONNECT_SOFT_RESET,
	MTK_PD_CONNECT_TYPEC_ONLY_SNK,
	MTK_PD_CONNECT_TYPEC_ONLY_SNK,
};
/*====================================================================*/

#define USB_TEMP_HIGH		0x01 /*bit0*/
#define USB_WATER_DETECT	0x02 /*bit1*/
#define USB_RESERVE2		0x04 /*bit2*/
#define USB_RESERVE3		0x08 /*bit3*/
#define USB_RESERVE4		0x10 /*bit4*/
#define USB_DONOT_USE		0x80000000 /*bit31*/
#define DEFAULT_PULL_UP_R0      100000 /*pull up 100K*/
#define PULL_UP_R0_390K         390000 /*Pull up 390K*/

/**************************************************************
 * bit[0]=0: NO standard typec device/cable connected(ccdetect gpio in high level)
 * bit[0]=1: standard typec device/cable connected(ccdetect gpio in low level)
 * bit[1]=0: NO OTG typec device/cable connected
 * bit[1]=1: OTG typec device/cable connected
 **************************************************************/
#define DISCONNECT 0
#define STANDARD_TYPEC_DEV_CONNECT BIT(0)
#define OTG_DEV_CONNECT BIT(1)

#define QC_CHARGER_VOLTAGE_HIGH 7500
#define QC_SOC_HIGH 90
#define QC_TEMP_HIGH 420

static int usb_status = 0;


#ifdef CONFIG_OPLUS_CHARGER_MTK
int __attribute__((weak)) get_boot_reason(void)
{
	return 0;
}
#endif

int __attribute__((weak)) get_charger_ntc_volt(void)
{
	return 0;
}

int __attribute__((weak)) get_bat_con_ntc_volt(void)
{
	return 0;
}

bool __attribute__((weak)) mtk_pdc_init(struct mtk_charger *info)
{
	return true;
}

bool __attribute__((weak)) pmic_chrdet_status(void)
{
	return false;
}

int __attribute__((weak)) oplus_get_rtc_ui_soc(void)
{
	return 50; /* to-do */
}

int __attribute__((weak)) oplus_set_rtc_ui_soc(int value)
{
	return value; /* to-do */
}

bool oplus_otgctl_by_buckboost(void)
{
	if (!g_oplus_chip)
		return false;

	return g_oplus_chip->vbatt_num == 2;
}

int battery_get_bat_temperature(void);
int battery_get_bat_temperature(void)
{
	return get_battery_temperature(pinfo);
}

int battery_get_bat_voltage(void);
int battery_get_bat_voltage(void)
{
	return get_battery_voltage(pinfo);
}

int battery_get_bat_current(void);
int battery_get_bat_current(void)
{
	return get_battery_current(pinfo);
}

int battery_get_vbus(void);
int battery_get_vbus(void)
{
	return 0;
}

int battery_get_soc(void);
int battery_get_soc(void)
{
	return 0;
}

int battery_get_uisoc(void);
int battery_get_uisoc(void)
{
	return get_uisoc(pinfo);
}

int charger_get_vbus(void);
signed int battery_meter_get_charger_voltage(void)
{
	return get_vbus(pinfo);
}

int get_uisoc(struct mtk_charger *info)
{
	union power_supply_propval prop = {0};
	struct power_supply *bat_psy = NULL;
	int ret = 0;

	bat_psy = info->bat_manager_psy;

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s retry to get bat_psy\n", __func__);
		bat_psy = power_supply_get_by_name("battery");
		info->bat_manager_psy = bat_psy;
	}

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s Couldn't get bat_psy\n", __func__);
		ret = 50;
	} else {
		ret = power_supply_get_property(bat_psy,
			POWER_SUPPLY_PROP_CAPACITY, &prop);
		if (ret < 0) {
			chr_err("%s Couldn't get soc\n", __func__);
			ret = 50;
			return ret;
		}
		ret = prop.intval;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_battery_voltage(struct mtk_charger *info)
{
	union power_supply_propval prop = {0};
	struct power_supply *bat_psy = NULL;
	int ret = 0;

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
		if (ret < 0) {
			chr_err("%s Couldn't get vbat\n", __func__);
			ret = 3999;
			return ret;
		}
		ret = prop.intval / 1000;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_cs_side_battery_voltage(struct mtk_charger *info, int *vbat)
{
	union power_supply_propval prop = {0};
	struct power_supply *bat_psy = NULL;
	int ret = 0;
	int tmp_ret = 0;

	bat_psy = info->bat2_psy;

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_debug("%s retry to get bat_psy\n", __func__);
		bat_psy = devm_power_supply_get_by_phandle(&info->pdev->dev, "gauge2");
		info->bat2_psy = bat_psy;
	}

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_debug("%s Couldn't get bat_psy\n", __func__);
		ret = charger_dev_get_vbat(info->cschg1_dev, vbat);
		if (ret < 0)
			*vbat = 3999;
		else
			ret = FROM_CS_ADC;
	} else {
		tmp_ret = power_supply_get_property(bat_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		if (tmp_ret < 0)
			chr_debug("%s: %d\n", __func__, tmp_ret);
		*vbat = prop.intval / 1000;
		ret = FROM_CHG_IC;
	}

	chr_debug("%s:%d %d\n", __func__,
		ret, *vbat);
	return ret;
}

int get_battery_temperature(struct mtk_charger *info)
{
	union power_supply_propval prop = {0};
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
		if (tmp_ret < 0)
			chr_debug("%s: %d\n", __func__, tmp_ret);
		ret = prop.intval / 10;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_battery_current(struct mtk_charger *info)
{
	union power_supply_propval prop = {0};
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
		if (tmp_ret < 0)
			chr_debug("%s: %d\n", __func__, tmp_ret);
		ret = prop.intval / 1000;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_cs_side_battery_current(struct mtk_charger *info, int *ibat)
{
	union power_supply_propval prop = {0};
	struct power_supply *bat_psy = NULL;
	int ret = 0;
	int tmp_ret = 0;
	/* return 1: MTK ADC, return 2: SC ADC*/
	bat_psy = info->bat2_psy;

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_debug("%s retry to get bat_psy\n", __func__);
		bat_psy = devm_power_supply_get_by_phandle(&info->pdev->dev, "gauge2");
		info->bat2_psy = bat_psy;
	}

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_debug("%s Couldn't get bat_psy\n", __func__);
		ret = charger_cs_get_ibat(info->cschg1_dev, ibat);
		if (ret < 0)
			*ibat = 0;
		else
			ret = FROM_CS_ADC;
	} else {
		tmp_ret = power_supply_get_property(bat_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
		if (tmp_ret < 0)
			chr_debug("%s: %d\n", __func__, tmp_ret);
		*ibat = prop.intval / 1000;
		ret = FROM_CHG_IC;
	}

	chr_debug("%s:%d %d\n", __func__,
		ret, *ibat);
	return ret;
}

int get_pmic_vbus(struct mtk_charger *info, int *vchr)
{
	union power_supply_propval prop = {0};
	static struct power_supply *chg_psy;
	int ret;

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
	union power_supply_propval prop = {0};
	struct power_supply *bat_psy = NULL;
	int ret = 0;
	int tmp_ret = 0;

	bat_psy = info->bat_psy;

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s retry to get bat_psy\n", __func__);
		bat_psy = power_supply_get_by_name("battery");
		info->bat_psy = bat_psy;
	}

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		chr_err("%s Couldn't get bat_psy\n", __func__);
		ret = 1;
	} else {
		tmp_ret = power_supply_get_property(bat_psy,
			POWER_SUPPLY_PROP_PRESENT, &prop);
		if (tmp_ret < 0)
			chr_debug("%s: %d\n", __func__, tmp_ret);
		ret = prop.intval;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

bool is_charger_exist(struct mtk_charger *info)
{
	union power_supply_propval prop = {0};
	static struct power_supply *chg_psy;
	int ret = 0;
	int tmp_ret = 0;

	chg_psy = info->chg_psy;

	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s retry to get chg_psy\n", __func__);


		chg_psy = power_supply_get_by_name("primary_chg");

		info->chg_psy = chg_psy;
	}

	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		pr_notice("%s Couldn't get chg_psy\n", __func__);
		ret = -1;
	} else {
		tmp_ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		if (tmp_ret < 0)
			chr_debug("%s: %d\n", __func__, tmp_ret);
		ret = prop.intval;
	}

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_charger_type(struct mtk_charger *info)
{
	union power_supply_propval prop = {0};
	union power_supply_propval prop2 = {0};
	union power_supply_propval prop3 = {0};
	static struct power_supply *bc12_psy;
	int ret;

	bc12_psy = info->bc12_psy;

	if (bc12_psy == NULL || IS_ERR(bc12_psy)) {
		chr_err("%s retry to get bc12_psy\n", __func__);

		bc12_psy = power_supply_get_by_name("primary_chg");

		info->bc12_psy = bc12_psy;
	}

	if (bc12_psy == NULL || IS_ERR(bc12_psy)) {
		chr_err("%s Couldn't get bc12_psy\n", __func__);
	} else {
		ret = power_supply_get_property(bc12_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		if (ret < 0)
			chr_debug("%s: %d\n", __func__, ret);
		ret = power_supply_get_property(bc12_psy,
			POWER_SUPPLY_PROP_TYPE, &prop2);
		if (ret < 0)
			chr_debug("%s: %d\n", __func__, ret);
		ret = power_supply_get_property(bc12_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop3);
		if (ret < 0)
			chr_debug("%s: %d\n", __func__, ret);

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
	union power_supply_propval prop = {0};
	union power_supply_propval prop2 = {0};
	static struct power_supply *bc12_psy;

	int ret = 0;

	bc12_psy = info->bc12_psy;

	if (bc12_psy == NULL || IS_ERR(bc12_psy)) {
		chr_err("%s retry to get bc12_psy\n", __func__);

		bc12_psy = power_supply_get_by_name("primary_chg");

		info->bc12_psy = bc12_psy;
	}
	if (bc12_psy == NULL || IS_ERR(bc12_psy)) {
		chr_err("%s Couldn't get bc12_psy\n", __func__);
	} else {
		ret = power_supply_get_property(bc12_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		if (ret < 0)
			chr_debug("%s Couldn't get cablestat.\n", __func__);
		ret = power_supply_get_property(bc12_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop2);
		if (ret < 0)
			chr_debug("%s Couldn't get usbtype.\n", __func__);
	}
	chr_debug("%s online:%d usb_type:%d\n", __func__,
		prop.intval,
		prop2.intval);
	return prop2.intval;
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

/*for p922x compile*/
void __attribute__((weak)) oplus_set_wrx_otg_value(void)
{
	return;
}
int __attribute__((weak)) oplus_get_idt_en_val(void)
{
	return -1;
}
int __attribute__((weak)) oplus_get_wrx_en_val(void)
{
	return -1;
}
int __attribute__((weak)) oplus_get_wrx_otg_val(void)
{
	return 0;
}
void __attribute__((weak)) oplus_wireless_set_otg_en_val(void)
{
	return;
}
void __attribute__((weak)) oplus_dcin_irq_enable(void)
{
	return;
}

bool __attribute__((weak)) oplus_get_wired_otg_online(void)
{
	return false;
}

bool __attribute__((weak)) oplus_get_wired_chg_present(void)
{
	return false;
}

void oplus_otg_enable_by_buckboost(void)
{
	if (!g_oplus_chip || !(g_oplus_chip->chg_ops->charging_disable) || !(g_oplus_chip->chg_ops->charging_disable))
		return;

	g_oplus_chip->chg_ops->charging_disable();
	g_oplus_chip->chg_ops->otg_enable();
}

void oplus_otg_disable_by_buckboost(void)
{
	if (!g_oplus_chip || !(g_oplus_chip->chg_ops->otg_disable))
		return;

	g_oplus_chip->chg_ops->otg_disable();
}

void oplus_gauge_set_event(int event)
{
	if (NULL != pinfo) {
		charger_manager_notifier(pinfo, event);
		chr_err("[%s] notify mtkfuelgauge event = %d\n", __func__, event);
	}
}

int oplus_get_usb_status(void)
{
	if (g_oplus_chip->usb_status == USB_TEMP_HIGH) {
		return g_oplus_chip->usb_status;
	} else {
		return usb_status;
	}
}

void charger_ic_enable_ship_mode(struct oplus_chg_chip *chip)
{
	if (!chip->is_double_charger_support) {
		if (chip->chg_ops->enable_shipmode) {
			chip->chg_ops->enable_shipmode(true);
		}
		} else {
		if (chip->sub_chg_ops->enable_shipmode) {
			chip->sub_chg_ops->enable_shipmode(true);
		}
		if (chip->chg_ops->enable_shipmode) {
			chip->chg_ops->enable_shipmode(true);
		}
	}
}

#define OPLUS_SVID 0x22d9
uint32_t pd_svooc_abnormal_adapter[] = {
	0x20002,
	0x10002,
	0x10001,
	0x40001,
};

int oplus_get_adapter_svid(void)
{
	int i = 0, j = 0;
	uint32_t vdos[VDO_MAX_NR] = {0};
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	struct tcpm_svid_list svid_list = {0, {0}};

	if (tcpc_dev == NULL || !g_oplus_chip) {
		chg_err("tcpc_dev is null return\n");
		return -1;
	}

	if (!oplus_is_vooc_project()) {
		chg_err("device don't support vooc\n");
		return -1;
	}

	tcpm_inquire_pd_partner_svids(tcpc_dev, &svid_list);
	for (i = 0; i < svid_list.cnt; i++) {
		chg_err("svid[%d] = %d\n", i, svid_list.svids[i]);
		if (svid_list.svids[i] == OPLUS_SVID) {
			g_oplus_chip->pd_svooc = true;
			chg_err("match svid and this is oplus adapter\n");
			break;
		}
	}

	tcpm_inquire_pd_partner_inform(tcpc_dev, vdos);
	if ((vdos[0] & 0xFFFF) == OPLUS_SVID) {
		g_oplus_chip->pd_svooc = true;
		chg_err("match svid and this is oplus adapter 11\n");
		for (j = 0; j < ARRAY_SIZE(pd_svooc_abnormal_adapter); j++) {
			if (pd_svooc_abnormal_adapter[j] == vdos[2]) {
				chg_err("This is oplus gnd abnormal adapter %x %x \n", vdos[1], vdos[2]);
				g_oplus_chip->is_abnormal_adapter = true;
				break;
			}
		}
	}

	return 0;
}

/* ==================================================================== */


int oplus_battery_meter_get_battery_voltage(void)
{
	return 4000;
}

bool is_vooc_project(void)
{
	return false;
}

static bool is_even_c_prj = false;
static bool is_even_c_project(void)
{
	return is_even_c_prj;
}

static int oplus_even_c_prj_parse_dt(struct oplus_chg_chip *chip)
{
	struct device_node *node = NULL;

	if (chip != NULL)
		node = chip->dev->of_node;

	if (chip == NULL || node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}

	is_even_c_prj = of_property_read_bool(node, "qcom,is_even_c_prj");
	if (is_even_c_prj == false) {
		chg_err("not even-c project!\n");
	}

	return 0;
}

bool meter_fg_30_get_battery_authenticate(void);
int charger_ic_flag = 1;
int oplus_which_charger_ic(void)
{
	return charger_ic_flag;
}

int get_charger_ic_det(struct oplus_chg_chip *chip)
{
	int count = 0;
	int n = charger_ic__det_flag;

	if (!chip) {
		return charger_ic__det_flag;
	}

	while (n) {
		++count;
		n = (n - 1) & n;
	}

	chip->is_double_charger_support = count > 1 ? true:false;
	chg_err("charger_ic__det_flag:%d\n", charger_ic__det_flag);
	return charger_ic__det_flag;
}

void set_charger_ic(int sel)
{
	charger_ic__det_flag |= 1 << sel;
}
EXPORT_SYMBOL(set_charger_ic);

extern enum charger_type mt_charger_type_detection(void);
extern enum charger_type mt_charger_type_detection_bq25890h(void);
extern enum charger_type mt_charger_type_detection_bq25601d(void);
extern bool upmu_is_chr_det(void);
extern enum charger_type g_chr_type;

static bool oplus_ship_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (gpio_is_valid(chip->normalchg_gpio.ship_gpio))
		return true;

	return false;
}

int oplus_ship_gpio_init(struct oplus_chg_chip *chip)
{
	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	chip->normalchg_gpio.ship_active =
		pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
			"ship_active");

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.ship_active)) {
		chg_err("get ship_active fail\n");
		return -EINVAL;
	}
	chip->normalchg_gpio.ship_sleep =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
				"ship_sleep");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.ship_sleep)) {
		chg_err("get ship_sleep fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl,
		chip->normalchg_gpio.ship_sleep);
	return 0;
}

#define SHIP_MODE_CONFIG		0x40
#define SHIP_MODE_MASK			BIT(0)
#define SHIP_MODE_ENABLE		0
#define PWM_COUNT				5
void smbchg_enter_shipmode(struct oplus_chg_chip *chip)
{
	int i = 0;
	chg_err("enter smbchg_enter_shipmode\n");

	if (oplus_ship_check_is_gpio(chip) == true) {
		chg_err("select gpio control\n");
		if (!IS_ERR_OR_NULL(chip->normalchg_gpio.ship_active) && !IS_ERR_OR_NULL(chip->normalchg_gpio.ship_sleep)) {
			pinctrl_select_state(chip->normalchg_gpio.pinctrl,
				chip->normalchg_gpio.ship_sleep);
			for (i = 0; i < PWM_COUNT; i++) {
				/*gpio_direction_output(chip->normalchg_gpio.ship_gpio, 1);*/
				pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_active);
				mdelay(3);
				/*gpio_direction_output(chip->normalchg_gpio.ship_gpio, 0);*/
				pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_sleep);
				mdelay(3);
			}
		}
		chg_err("power off after 15s\n");
	}
}
void enter_ship_mode_function(struct oplus_chg_chip *chip)
{
	if (chip != NULL) {
		if (chip->enable_shipmode && !chip->disable_ship_mode) {
			printk("enter_ship_mode_function\n");
			smbchg_enter_shipmode(chip);
			charger_ic_enable_ship_mode(chip);
		} else {
			chg_err("The program does not support ship_mode.\n");
		}
	}
}

/*bool is_disable_charger(void)
{
	if (pinfo == NULL)
		return true;

	if (pinfo->disable_charger == true || IS_ENABLED(CONFIG_POWER_EXT))
		return true;
	else
		return false;
}*/

bool is_disable_charger(struct mtk_charger *info)
{
	if (info == NULL)
		return true;

	if (info->disable_charger == true || IS_ENABLED(CONFIG_POWER_EXT))
		return true;
	else
		return false;
}

void BATTERY_SetUSBState(int usb_state_value)
{
	if (is_disable_charger(pinfo)) {
		chr_err("[%s] in FPGA/EVB, no service\n", __func__);
	} else {
		if ((usb_state_value < USB_SUSPEND) ||
			((usb_state_value > USB_CONFIGURED))) {
			chr_err("%s Fail! Restore to default value\n",
				__func__);
			usb_state_value = USB_UNCONFIGURED;
		} else {
			chr_err("%s Success! Set %d\n", __func__,
				usb_state_value);
			if (pinfo)
				pinfo->usb_state = usb_state_value;
		}
	}
}

unsigned int set_chr_input_current_limit(int current_limit)
{
	return 500;
}

int get_chr_temperature(int *min_temp, int *max_temp)
{
	*min_temp = 25;
	*max_temp = 30;

	return 0;
}

int set_chr_boost_current_limit(unsigned int current_limit)
{
	return 0;
}

int set_chr_enable_otg(unsigned int enable)
{
	if (!g_oplus_chip) {
		printk(KERN_ERR "[oplus_CHG][%s]: otg [%s] failed.\n", __func__, enable?"Enable":"Disable");
		return -1;
}

	if (enable) {
			g_oplus_chip->chg_ops->otg_enable();
	} else {
			g_oplus_chip->chg_ops->otg_disable();
	}
	return 0;
}
EXPORT_SYMBOL(set_chr_enable_otg);

enum charger_type __attribute__((weak)) mt_get_charger_type(void)
{
	return CHARGER_UNKNOWN;
}

int mtk_chr_is_charger_exist(unsigned char *exist)
{
	if (mt_get_charger_type() == CHARGER_UNKNOWN)
		*exist = 0;
	else
		*exist = 1;
	return 0;
}

void oplus_set_otg_switch_status(bool value)
{
	if (pinfo != NULL && pinfo->tcpc != NULL) {
		printk(KERN_ERR "[oplus_CHG][%s]: otg switch[%d]\n", __func__, value);
		tcpm_typec_change_role_postpone(pinfo->tcpc, value ? TYPEC_ROLE_TRY_SNK : TYPEC_ROLE_SNK, true);
	} else {
		printk(KERN_ERR "[oplus_CHG][%s]: tcpc device fail\n", __func__);
	}
}
EXPORT_SYMBOL(oplus_set_otg_switch_status);


/*=============== fix me==================*/
int chargerlog_level = CHRLOG_ERROR_LEVEL;
EXPORT_SYMBOL(chargerlog_level);

int chr_get_debug_level(void)
{
	return chargerlog_level;
}
EXPORT_SYMBOL(chr_get_debug_level);


static void _wake_up_charger(struct mtk_charger *info)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
	return;
#else
	unsigned long flags;

	if (info == NULL)
		return;

	spin_lock_irqsave(&info->slock, flags);
	if (!info->charger_wakelock->active)
			__pm_stay_awake(info->charger_wakelock);
	spin_unlock_irqrestore(&info->slock, flags);
	info->charger_thread_timeout = true;
	wake_up(&info->wait_que);
#endif /*OPLUS_FEATURE_CHG_BASIC*/
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


/* charger_manager ops  */
bool is_meta_mode(void)
{
	if (!pinfo) {
		pr_err("%s: info is null\n", __func__);
		return false;
	}

	pr_err("%s: bootmode = %d\n", __func__, pinfo->bootmode);

	if (pinfo->bootmode == 1)
		return true;
	else
		return false;
}
EXPORT_SYMBOL(is_meta_mode);

static int _mtk_charger_change_current_setting(struct mtk_charger *info)
{
	if (info != NULL && info->algo.change_current_setting)
		return info->algo.change_current_setting(info);

	return 0;
}

static int _mtk_charger_do_charging(struct mtk_charger *info, bool en);
static int _mtk_charger_do_charging(struct mtk_charger *info, bool en)
{
	if (info != NULL && info->algo.enable_charging)
		info->algo.enable_charging(info, en);
	return 0;
}
/* charger_manager ops end */

/* user interface */
struct charger_consumer *charger_manager_get_by_name(struct device *dev,
	const char *name)
{
	struct charger_consumer *puser;

	puser = kzalloc(sizeof(struct charger_consumer), GFP_KERNEL);
	if (puser == NULL)
		return NULL;

	mutex_lock(&consumer_mutex);
	puser->dev = dev;

	list_add(&puser->list, &consumer_head);
	if (pinfo != NULL)
		puser->cm = pinfo;

	mutex_unlock(&consumer_mutex);

	return puser;
}
EXPORT_SYMBOL(charger_manager_get_by_name);


int charger_manager_enable_power_path(struct charger_consumer *consumer,
	int idx, bool en)
{
	int ret = 0;
	bool is_en = true;
	struct mtk_charger *info = consumer->cm;
	struct charger_device *chg_dev = NULL;


	if (!info)
		return -EINVAL;

	switch (idx) {
	case MAIN_CHARGER:
		chg_dev = info->chg1_dev;
		break;
	case SLAVE_CHARGER:
		chg_dev = info->chg2_dev;
		break;
	default:
		return -EINVAL;
	}

	ret = charger_dev_is_powerpath_enabled(chg_dev, &is_en);
	if (ret < 0) {
		chr_err("%s: get is power path enabled failed\n", __func__);
		return ret;
	}
	if (is_en == en) {
		chr_err("%s: power path is already en = %d\n", __func__, is_en);
		return 0;
	}

	pr_info("%s: enable power path = %d\n", __func__, en);
	return charger_dev_enable_powerpath(chg_dev, en);
}

static int _charger_manager_enable_charging(struct charger_consumer *consumer,
	int idx, bool en)
{
	struct mtk_charger *info = consumer->cm;

	chr_err("%s: dev:%s idx:%d en:%d\n", __func__, dev_name(consumer->dev),
		idx, en);

	if (info != NULL) {
		struct charger_data *pdata = NULL;

		if (idx == CHG1_SETTING)
			pdata = &info->chg_data[CHG1_SETTING];
		else if (idx == CHG2_SETTING)
			pdata = &info->chg_data[CHG2_SETTING];
		else
			return -ENOTSUPP;

		if (en == false) {
			_mtk_charger_do_charging(info, en);
			pdata->disable_charging_count++;
		} else {
			if (pdata->disable_charging_count == 1) {
				_mtk_charger_do_charging(info, en);
				pdata->disable_charging_count = 0;
			} else if (pdata->disable_charging_count > 1)
				pdata->disable_charging_count--;
		}
		chr_err("%s: dev:%s idx:%d en:%d cnt:%d\n", __func__,
			dev_name(consumer->dev), idx, en,
			pdata->disable_charging_count);

		return 0;
	}
	return -EBUSY;
}

int charger_manager_enable_charging(struct charger_consumer *consumer,
	int idx, bool en)
{
	struct mtk_charger *info = consumer->cm;
	int ret = 0;
#ifdef OPLUS_FEATURE_CHG_BASIC
	return -EBUSY;
#endif

	mutex_lock(&info->charger_lock);
	ret = _charger_manager_enable_charging(consumer, idx, en);
	mutex_unlock(&info->charger_lock);
	return ret;
}

int charger_manager_set_input_current_limit(struct charger_consumer *consumer,
	int idx, int input_current)
{
	struct mtk_charger *info = consumer->cm;

#ifdef OPLUS_FEATURE_CHG_BASIC
	return 0;
#endif

	if (info != NULL) {
		struct charger_data *pdata;

		if (idx == CHG1_SETTING)
			pdata = &info->chg_data[CHG1_SETTING];
		else if (idx == CHG2_SETTING)
			pdata = &info->chg_data[CHG2_SETTING];
		else if (idx == DVCHG1_SETTING)
			pdata = &info->chg_data[DVCHG1_SETTING];
		else if (idx == HVDVCHG2_SETTING)
			pdata = &info->chg_data[HVDVCHG2_SETTING];
		else
			return -ENOTSUPP;
		pdata->thermal_input_current_limit = input_current;

		chr_err("%s: dev:%s idx:%d en:%d\n", __func__,
			dev_name(consumer->dev), idx, input_current);
		_mtk_charger_change_current_setting(info);
		_wake_up_charger(info);
		return 0;
	}
	return -EBUSY;
}

int charger_manager_set_charging_current_limit(
	struct charger_consumer *consumer, int idx, int charging_current)
{
	struct mtk_charger *info = consumer->cm;
#ifdef OPLUS_FEATURE_CHG_BASIC
	return 0;
#endif

	if (info != NULL) {
		struct charger_data *pdata;

		if (idx == CHG1_SETTING)
			pdata = &info->chg_data[CHG1_SETTING];
		else if (idx == CHG2_SETTING)
			pdata = &info->chg_data[CHG2_SETTING];
		else
			return -ENOTSUPP;

		pdata->thermal_charging_current_limit = charging_current;
		chr_err("%s: dev:%s idx:%d en:%d\n", __func__,
			dev_name(consumer->dev), idx, charging_current);
		_mtk_charger_change_current_setting(info);
		_wake_up_charger(info);
		return 0;
	}
	return -EBUSY;
}

int charger_manager_get_charger_temperature(struct charger_consumer *consumer,
	int idx, int *tchg_min,	int *tchg_max)
{
	struct mtk_charger *info = consumer->cm;

	if (info != NULL) {
		struct charger_data *pdata = NULL;

#ifndef OPLUS_FEATURE_CHG_BASIC
		if (!upmu_get_rgs_chrdet()) {
			pr_debug("[%s] No cable in, skip it\n", __func__);
			*tchg_min = -127;
			*tchg_max = -127;
			return -EINVAL;
		}
#endif

		if (idx == CHG1_SETTING)
			pdata = &info->chg_data[CHG1_SETTING];
		else if (idx == CHG2_SETTING)
			pdata = &info->chg_data[CHG2_SETTING];
		else if (idx == DVCHG1_SETTING)
			pdata = &info->chg_data[DVCHG1_SETTING];
		else if (idx == HVDVCHG2_SETTING)
			pdata = &info->chg_data[HVDVCHG2_SETTING];
		else
			return -ENOTSUPP;

		*tchg_min = pdata->junction_temp_min;
		*tchg_max = pdata->junction_temp_max;

		return 0;
	}
	return -EBUSY;
}

int charger_manager_force_charging_current(struct charger_consumer *consumer,
	int idx, int charging_current)
{
	struct mtk_charger *info = consumer->cm;

	if (info != NULL) {
		struct charger_data *pdata = NULL;

		if (idx == CHG1_SETTING)
			pdata = &info->chg_data[CHG1_SETTING];
		else if (idx == CHG2_SETTING)
			pdata = &info->chg_data[CHG2_SETTING];
		else
			return -ENOTSUPP;

		pdata->force_charging_current = charging_current;
		_mtk_charger_change_current_setting(info);
		_wake_up_charger(info);
		return 0;
	}
	return -EBUSY;
}

int charger_manager_get_zcv(struct charger_consumer *consumer, int idx, u32 *uV)
{
	struct mtk_charger *info = consumer->cm;

	int ret = 0;
	struct charger_device *pchg = NULL;

	if (info != NULL) {
		if (idx == CHG1_SETTING) {
			pchg = info->chg1_dev;
			ret = charger_dev_get_zcv(pchg, uV);
		} else if (idx == SLAVE_CHARGER) {
			pchg = info->chg2_dev;
			ret = charger_dev_get_zcv(pchg, uV);
		} else
			ret = -1;

	} else {
		chr_err("%s info is null\n", __func__);
	}
	chr_err("%s zcv:%d ret:%d\n", __func__, *uV, ret);

	return 0;
}

int register_charger_manager_notifier(struct charger_consumer *consumer,
	struct notifier_block *nb)
{
	int ret = 0;
	struct mtk_charger *info = consumer->cm;

	mutex_lock(&consumer_mutex);
	if (info != NULL)
		ret = srcu_notifier_chain_register(&info->evt_nh, nb);
	else
		consumer->pnb = nb;
	mutex_unlock(&consumer_mutex);

	return ret;
}

int unregister_charger_manager_notifier(struct charger_consumer *consumer,
				struct notifier_block *nb)
{
	int ret = 0;
	struct mtk_charger *info = consumer->cm;

	mutex_lock(&consumer_mutex);
	if (info != NULL)
		ret = srcu_notifier_chain_unregister(&info->evt_nh, nb);
	else
		consumer->pnb = NULL;
	mutex_unlock(&consumer_mutex);

	return ret;
}

bool oplus_shortc_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (gpio_is_valid(chip->normalchg_gpio.shortc_gpio)) {
		return true;
	}
	return false;
}

int oplus_shortc_gpio_init(struct oplus_chg_chip *chip)
{
	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	chip->normalchg_gpio.shortc_active =
		pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
			"shortc_active");

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.shortc_active)) {
		chg_err("get shortc_active fail\n");
		return -EINVAL;
        }

	pinctrl_select_state(chip->normalchg_gpio.pinctrl,
		chip->normalchg_gpio.shortc_active);
	return 0;
}
#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
bool oplus_chg_get_shortc_hw_gpio_status(void)
{
	bool shortc_hw_status = 1;

	if(oplus_shortc_check_is_gpio(g_oplus_chip) == true) {
		shortc_hw_status = !!(gpio_get_value(g_oplus_chip->normalchg_gpio.shortc_gpio));
	}
	return shortc_hw_status;
}
#else
bool oplus_chg_get_shortc_hw_gpio_status(void)
{
	bool shortc_hw_status = 1;

	return shortc_hw_status;
}
#endif

int usbtemp_channel_init(struct device *dev)
{
	int ret = 0;

	usb_chan1 = iio_channel_get(dev, "usbtemp-ch3");
	if (IS_ERR_OR_NULL(usb_chan1)) {
		chg_err("usb_chan1 init fial,err = %ld", PTR_ERR(usb_chan1));
		ret = -1;
	}

	usb_chan2 = iio_channel_get(dev, "usbtemp-ch4");
	if(IS_ERR_OR_NULL(usb_chan2)) {
		chg_err("usb_chan2 init fial,err = %ld", PTR_ERR(usb_chan2));
		ret = -1;
	}

	return ret;
}

static void set_usbswitch_to_rxtx(struct oplus_chg_chip *chip)
{
	int ret = 0;

	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return;
	}
	mutex_lock(&chip->normalchg_gpio.pinctrl_mutex);
	gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 1);
	ret = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.charger_gpio_as_output2);
	mutex_unlock(&chip->normalchg_gpio.pinctrl_mutex);
	if (ret < 0) {
		chg_err("failed to set pinctrl int\n");
		return;
	}
	chg_err("set_usbswitch_to_rxtx \n");
}

static void set_usbswitch_to_dpdm(struct oplus_chg_chip *chip)
{
	int ret = 0;

	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return;
	}
	mutex_lock(&chip->normalchg_gpio.pinctrl_mutex);
	gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 0);
	ret = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.charger_gpio_as_output1);
	mutex_unlock(&chip->normalchg_gpio.pinctrl_mutex);
	if (ret < 0) {
		chg_err("failed to set pinctrl int\n");
		return;
	}
	chg_err("set_usbswitch_to_dpdm \n");
}

static bool is_support_chargerid_check(void)
{
#ifdef CONFIG_OPLUS_CHECK_CHARGERID_VOLT
	return true;
#else
	return false;
#endif
}

int mt_get_chargerid_volt(void)
{
	int chargerid_volt = 0;

	if (is_support_chargerid_check() == true) {
		chg_debug("chargerid_volt = %d \n",
					   chargerid_volt);
	} else {
		chg_debug("is_support_chargerid_check = false !\n");
		return 0;
	}

	return chargerid_volt;
}


void mt_set_chargerid_switch_val(int value)
{
	chg_debug("set_value= %d\n", value);
	if (NULL == g_oplus_chip)
		return;
	if (is_support_chargerid_check() == false)
		return;
	if (g_oplus_chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return;
	}
	if (IS_ERR_OR_NULL(g_oplus_chip->normalchg_gpio.pinctrl)
		|| IS_ERR_OR_NULL(g_oplus_chip->normalchg_gpio.charger_gpio_as_output1)
		|| IS_ERR_OR_NULL(g_oplus_chip->normalchg_gpio.charger_gpio_as_output2)) {
		chg_err("pinctrl null, return\n");
		return;
	}
	if (1 == value) {
			set_usbswitch_to_rxtx(g_oplus_chip);
	} else if (0 == value) {
		set_usbswitch_to_dpdm(g_oplus_chip);
	} else {
		/* do nothing */
	}
	chg_debug("get_val:%d\n", gpio_get_value(g_oplus_chip->normalchg_gpio.chargerid_switch_gpio));
}

int mt_get_chargerid_switch_val(void)
{
	int gpio_status = 0;

	if (NULL == g_oplus_chip)
		return 0;
	if (is_support_chargerid_check() == false)
		return 0;
	gpio_status = gpio_get_value(g_oplus_chip->normalchg_gpio.chargerid_switch_gpio);

	chg_debug("mt_get_chargerid_switch_val:%d\n", gpio_status);
	return gpio_status;
}

int oplus_usb_switch_gpio_gpio_init(void)
{
	g_oplus_chip->normalchg_gpio.pinctrl = devm_pinctrl_get(g_oplus_chip->dev);
	if (IS_ERR_OR_NULL(g_oplus_chip->normalchg_gpio.pinctrl)) {
		chg_err("get normalchg_gpio.chargerid_switch_gpio pinctrl falil\n");
		return -EINVAL;
	}
	g_oplus_chip->normalchg_gpio.charger_gpio_as_output1 = pinctrl_lookup_state(g_oplus_chip->normalchg_gpio.pinctrl,
								"charger_gpio_as_output_low");
	if (IS_ERR_OR_NULL(g_oplus_chip->normalchg_gpio.charger_gpio_as_output1)) {
		chg_err("get charger_gpio_as_output_low fail\n");
		return -EINVAL;
	}
	g_oplus_chip->normalchg_gpio.charger_gpio_as_output2 = pinctrl_lookup_state(g_oplus_chip->normalchg_gpio.pinctrl,
								"charger_gpio_as_output_high");
	if (IS_ERR_OR_NULL(g_oplus_chip->normalchg_gpio.charger_gpio_as_output2)) {
		chg_err("get charger_gpio_as_output_high fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(g_oplus_chip->normalchg_gpio.pinctrl, g_oplus_chip->normalchg_gpio.charger_gpio_as_output1);
	return 0;
}


int charger_pretype_get(void)
{
	int chg_type = STANDARD_HOST;
	/* chg_type = hw_charging_get_charger_type(); */
	return chg_type;
}


bool oplus_pmic_check_chip_is_null(void)
{
	if (!is_vooc_project()) {
		return true;
	} else {
		return false;
	}
}

/*====================================================================*/
static bool oplus_usbtemp_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.dischg_gpio))
		return true;

	return false;
}

static bool oplus_usbtemp_check_is_support(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	if (oplus_usbtemp_check_is_gpio(chip) == true)
		return true;

	chg_err("not support, return false\n");

	return false;
}


static int oplus_dischg_gpio_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
		chg_err("get dischg_pinctrl fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.dischg_enable = pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "dischg_enable");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_enable)) {
		chg_err("get dischg_enable fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.dischg_disable = pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "dischg_disable");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_disable)) {
		chg_err("get dischg_disable fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_disable);

	return 0;
}

#define USB_20C		20 /*degreeC*/
#define USB_30C		30 /*degreeC*/
#define USB_50C		50
#define USB_57C		57
#define USB_100C	100
#define USB_25C_VOLT	1192	/*1192mv*/
#define USB_50C_VOLT	450
#define USB_55C_VOLT	448
#define USB_60C_VOLT	327
#define VBUS_VOLT_THRESHOLD	3000 /*3V*/
#define RETRY_CNT_DELAY		5 /*ms*/
#define MIN_MONITOR_INTERVAL	50 /*50ms*/
#define VBUS_MONITOR_INTERVAL	3000 /*3s*/

#define USB_PORT_PULL_UP_R      390000 /*390K*/
#define USB_PORT_PULL_UP_R_100     100000 /*100k even-c*/
#define USB_PORT_PULL_UP_VOLT   1800  /*1.8V*/
#define USB_NTC_TABLE_SIZE 74
#define USB_NTC_PULLUP_NOTFROM_DTS 0


void oplus_get_usbtemp_volt(struct oplus_chg_chip *chip)
{
	int usbtemp_volt = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}
	iio_read_channel_processed(usb_chan1, &usbtemp_volt);
	if (usbtemp_volt <= 0) {
		usbtemp_volt = USB_25C_VOLT;
	}
	chip->usbtemp_volt_r = usbtemp_volt;
	iio_read_channel_processed(usb_chan2, &usbtemp_volt);
	if (usbtemp_volt <= 0) {
		usbtemp_volt = USB_25C_VOLT;
	}
	chip->usbtemp_volt_l = usbtemp_volt;

	return;
}

static bool oplus_chg_get_vbus_status(struct oplus_chg_chip *chip)
{
	bool vbus_status = false;

	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return vbus_status;
	}

	vbus_status = chip->chg_ops->check_chrdet_status();
	return vbus_status;
}

#ifdef OPLUS_FEATURE_CHG_BASIC

#define TEMP_25C 25
#define OPLUS_TCHG_01C_PRECISION	10
static struct temp_param chargeric_temp_table[] = {
	{-40, 4251000}, {-39, 3962000}, {-38, 3695000}, {-37, 3447000}, {-36, 3218000}, {-35, 3005000}, {-34, 2807000}, {-33, 2624000},
	{-32, 2454000}, {-31, 2296000}, {-30, 2149000}, {-29, 2012000}, {-28, 1885000}, {-27, 1767000}, {-26, 1656000}, {-25, 1554000},
	{-24, 1458000}, {-23, 1369000}, {-22, 1286000}, {-21, 1208000}, {-20, 1135000}, {-19, 1068000}, {-18, 1004000}, {-17,  945000},
	{-16,  889600}, {-15,  837800}, {-14,  789300}, {-13,  743900}, {-12,  701300}, {-11,  661500}, {-10,  624100}, {-9,   589000},
	{-8,   556200}, {-7,   525300}, {-6,   496300}, {-5,   469100}, {-4,   443500}, {-3,   419500}, {-2,   396900}, {-1,   375600},
	{0,    355600}, {1,    336800}, {2,    319100}, {3,    302400}, {4,    286700}, {5,    271800}, {6,    257800}, {7,    244700},
	{8,    232200}, {9,    220500}, {10,   209400}, {11,   198900}, {12,   189000}, {13,   179700}, {14,   170900}, {15,   162500},
	{16,   154600}, {17,   147200}, {18,   140100}, {19,   133400}, {20,   127000}, {21,   121000}, {22,   115400}, {23,   110000},
	{24,   104800}, {25,   100000}, {26,   95400 }, {27,   91040 }, {28,   86900 }, {29,   82970 }, {30,   79230 }, {31,   75690 },
	{32,   72320 }, {33,   69120 }, {34,   66070 }, {35,   63180 }, {36,   60420 }, {37,   57810 }, {38,   55310 }, {39,   52940 },
	{40,   50680 }, {41,   48530 }, {42,   46490 }, {43,   44530 }, {44,   42670 }, {45,   40900 }, {46,   39210 }, {47,   37600 },
	{48,   36060 }, {49,   34595 }, {50,   33195 }, {51,   31859 }, {52,   30584 }, {53,   29366 }, {54,   28203 }, {55,   27091 },
	{56,   26028 }, {57,   25013 }, {58,   24042 }, {59,   23113 }, {60,   22224 }, {61,   21374 }, {62,   20561 }, {63,   19782 },
	{64,   19036 }, {65,   18323 }, {66,   17640 }, {67,   16986 }, {68,   16360 }, {69,   15760 }, {70,   15184 }, {71,   14631 },
	{72,   14101 }, {73,   13592 }, {74,   13104 }, {75,   12635 }, {76,   12187 }, {77,   11757 }, {78,   11344 }, {79,   10947 },
	{80,   10566 }, {81,   10200 }, {82,     9848}, {83,     9510}, {84,     9185}, {85,     8873}, {86,     8572}, {87,     8283},
	{88,     8006}, {89,     7738}, {90,     7481}, {91,     7234}, {92,     6997}, {93,     6769}, {94,     6548}, {95,     6337},
	{96,     6132}, {97,     5934}, {98,     5744}, {99,     5561}, {100,    5384}, {101,    5214}, {102,    5051}, {103,    4893},
	{104,    4769}, {105,    4623}, {106,    4482}, {107,    4346}, {108,    4215}, {109,    4088}, {110,    3966}, {111,    3848},
	{112,    3734}, {113,    3624}, {114,    3518}, {115,    3415}, {116,    3316}, {117,    3220}, {118,    3128}, {119,    3038},
	{120,    2952}, {121,    2868}, {122,    2787}, {123,    2709}, {124,    2634}, {125,    2561}
};

static struct temp_param chargeric_temp_table_390k[] = {
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

struct NTC_TEMPERATURE {
	int ntc_temp;
	int temperature_r;
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

		tap_value = (((res - res2) * tmp1) + ((res1 - res) * tmp2)) * OPLUS_TCHG_01C_PRECISION / (res1 - res2);
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

	return chg_tmp;
}

static int get_chargeric_temp(void)
{
	int chargeric_temp = 0;
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};

	if (!pinfo) {
		chg_err("null pinfo or pdata\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_CHARGER_IC;
		ntc_param.i_tap_over_critical_low = 425100;
		ntc_param.i_rap_pull_up_r = PULL_UP_R0_390K;
		ntc_param.i_rap_pull_up_voltage = 1800;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		if(ntc_param.i_rap_pull_up_r == PULL_UP_R0_390K) {
			ntc_param.pst_temp_table = chargeric_temp_table_390k;
			ntc_param.i_table_size = (sizeof(chargeric_temp_table_390k) / sizeof(struct temp_param));
		} else {
			ntc_param.pst_temp_table = chargeric_temp_table;
			ntc_param.i_table_size = (sizeof(chargeric_temp_table) / sizeof(struct temp_param));
		}
		is_param_init = true;
		chg_err("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n",
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r,
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}

	ntc_param.ui_dwvolt =  get_charger_ntc_volt();
	chargeric_temp = oplus_res_to_temp(&ntc_param);

	chg_err("chargeric_temp:%d\n", chargeric_temp);
	return chargeric_temp;
}

int oplus_chg_get_chargeric_temp_cal(void)
{
	int chargeric_temp_cal = 0;

	if (!pinfo) {
		chg_err("null pinfo or pdata\n");
		return TEMP_25C;
	}

	chargeric_temp_cal = get_chargeric_temp();
	if (pinfo->support_ntc_01c_precision) {
		return chargeric_temp_cal;
	} else {
		return chargeric_temp_cal / OPLUS_TCHG_01C_PRECISION;
	}
}
EXPORT_SYMBOL(oplus_chg_get_chargeric_temp_cal);

static int get_batcon_temp(void)
{
	int batt_btb_temp = 0;
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};

	if (!pinfo) {
		chg_err("null pinfo or pdata\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_BATTERY;
		ntc_param.i_tap_over_critical_low = 425100;
		ntc_param.i_rap_pull_up_r = DEFAULT_PULL_UP_R0;
		ntc_param.i_rap_pull_up_voltage = 1800;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		if(ntc_param.i_rap_pull_up_r == PULL_UP_R0_390K) {
			ntc_param.pst_temp_table = chargeric_temp_table_390k;
			ntc_param.i_table_size = (sizeof(chargeric_temp_table_390k) / sizeof(struct temp_param));
		} else {
			ntc_param.pst_temp_table = chargeric_temp_table;
			ntc_param.i_table_size = (sizeof(chargeric_temp_table) / sizeof(struct temp_param));
		}
		is_param_init = true;
		chg_err("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n",
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r,
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}

	ntc_param.ui_dwvolt =  get_bat_con_ntc_volt();
	batt_btb_temp = oplus_res_to_temp(&ntc_param);

	chg_err("batt_btb_temp:%d\n", batt_btb_temp);
	return batt_btb_temp;
}

int oplus_chg_get_battery_btb_temp_cal(void)
{
	int battery_btb_temp_cal = 0;

	if (!pinfo) {
		chg_err("null pinfo or pdata\n");
		return TEMP_25C;
	}

#ifdef CONFIG_CHARGER_SC6607
	if (g_oplus_chip && g_oplus_chip->chg_ops->get_cp_tsbus && check_ntc_suport_1000k()) {
		return g_oplus_chip->chg_ops->get_cp_tsbus() / UNIT_TRANS;
	}
#endif
	if (g_oplus_chip && g_oplus_chip->chg_ops->get_cp_tsbat) {
		return g_oplus_chip->chg_ops->get_cp_tsbat();
	}
	battery_btb_temp_cal = get_batcon_temp();
	if (pinfo->support_ntc_01c_precision) {
		return battery_btb_temp_cal;
	} else {
		return battery_btb_temp_cal / OPLUS_TCHG_01C_PRECISION;
	}
}
EXPORT_SYMBOL(oplus_chg_get_battery_btb_temp_cal);
#endif

/* factory mode start*/
#define CHARGER_DEVNAME "charger_ftm"
#define GET_IS_SLAVE_CHARGER_EXIST _IOW('k', 13, int)

static struct class *charger_class;
static struct cdev *charger_cdev;
static int charger_major;
static dev_t charger_devno;

static int is_slave_charger_exist(void)
{
	if (get_charger_by_name("secondary_chg") == NULL)
		return 0;
	return 1;
}

static long charger_ftm_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	int ret = 0;
	int out_data = 0;
	void __user *user_data = (void __user *)arg;

	switch (cmd) {
	case GET_IS_SLAVE_CHARGER_EXIST:
		out_data = is_slave_charger_exist();
		ret = copy_to_user(user_data, &out_data, sizeof(out_data));
		chr_err("[%s] SLAVE_CHARGER_EXIST: %d\n", __func__, out_data);
		break;
	default:
		chr_err("[%s] Error ID\n", __func__);
		break;
	}

	return ret;
}
#ifdef CONFIG_COMPAT
static long charger_ftm_compat_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case GET_IS_SLAVE_CHARGER_EXIST:
		ret = file->f_op->unlocked_ioctl(file, cmd, arg);
		break;
	default:
		chr_err("[%s] Error ID\n", __func__);
		break;
	}

	return ret;
}
#endif
static int charger_ftm_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int charger_ftm_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations charger_ftm_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = charger_ftm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = charger_ftm_compat_ioctl,
#endif
	.open = charger_ftm_open,
	.release = charger_ftm_release,
};

void charger_ftm_init(void)
{
	struct class_device *class_dev = NULL;
	int ret = 0;

	ret = alloc_chrdev_region(&charger_devno, 0, 1, CHARGER_DEVNAME);
	if (ret < 0) {
		chr_err("[%s]Can't get major num for charger_ftm\n", __func__);
		return;
	}

	charger_cdev = cdev_alloc();
	if (!charger_cdev) {
		chr_err("[%s]cdev_alloc fail\n", __func__);
		goto unregister;
	}
	charger_cdev->owner = THIS_MODULE;
	charger_cdev->ops = &charger_ftm_fops;

	ret = cdev_add(charger_cdev, charger_devno, 1);
	if (ret < 0) {
		chr_err("[%s] cdev_add failed\n", __func__);
		goto free_cdev;
	}

	charger_major = MAJOR(charger_devno);
	charger_class = class_create(CHARGER_DEVNAME);
	if (IS_ERR(charger_class)) {
		chr_err("[%s] class_create failed\n", __func__);
		goto free_cdev;
	}

	class_dev = (struct class_device *)device_create(charger_class,
				NULL, charger_devno, NULL, CHARGER_DEVNAME);
	if (IS_ERR(class_dev)) {
		chr_err("[%s] device_create failed\n", __func__);
		goto free_class;
	}

	pr_debug("%s done\n", __func__);
	return;

free_class:
	class_destroy(charger_class);
free_cdev:
	cdev_del(charger_cdev);
unregister:
	unregister_chrdev_region(charger_devno, 1);
}
/* factory mode end */

static char __chg_cmdline[COMMAND_LINE_SIZE];
static char *chg_cmdline = __chg_cmdline;

const char *chg_get_cmd(void)
{
	struct device_node *of_chosen = NULL;
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
		chr_err("%s: failed to get /chosen\n", __func__);

	return chg_cmdline;
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

/* internal algorithm common function */
bool is_dual_charger_supported(struct mtk_charger *info)
{
	if (info->chg2_dev == NULL)
		return false;
	return true;
}

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
		sw_ovp = pinfo->data.vbus_sw_ovp_voltage;

	/* Enable/Disable SW OVP status */
	pinfo->data.max_charger_voltage = sw_ovp;

	disable_hw_ovp(pinfo, enable);

	chr_err("[%s] en:%d ovp:%d\n",
			    __func__, enable, sw_ovp);
	return ret;
}
EXPORT_SYMBOL(mtk_chg_enable_vbus_ovp);

int charger_enable_vbus_ovp(struct mtk_charger *pinfo, bool enable)
{
	int ret = 0;
	u32 sw_ovp = 0;

	if (enable)
		sw_ovp = pinfo->data.max_charger_voltage_setting;
	else
		sw_ovp = 15000000;

	/* Enable/Disable SW OVP status */
	pinfo->data.max_charger_voltage = sw_ovp;

	chr_err("[%s] en:%d ovp:%d\n",
			    __func__, enable, sw_ovp);
	return ret;
}

int charger_get_vbus(void)
{
	int ret = 0;
	int vchr = 0;

	if (pinfo == NULL)
		return 0;
	ret = charger_dev_get_vbus(pinfo->chg1_dev, &vchr);
	if (ret < 0) {
		chr_err("%s: get vbus failed: %d\n", __func__, ret);
		return ret;
	}

	vchr = vchr / 1000;
	return vchr;
}

/* internal algorithm common function end */

/* sw jeita */
void sw_jeita_state_machine_init(struct mtk_charger *info)
{
	struct sw_jeita_data *sw_jeita;

	if (info->enable_sw_jeita == true) {
		sw_jeita = &info->sw_jeita;
		info->battery_temp = battery_get_bat_temperature();

		if (info->battery_temp >= info->data.temp_t4_thres)
			sw_jeita->sm = TEMP_ABOVE_T4;
		else if (info->battery_temp > info->data.temp_t3_thres)
			sw_jeita->sm = TEMP_T3_TO_T4;
		else if (info->battery_temp >= info->data.temp_t2_thres)
			sw_jeita->sm = TEMP_T2_TO_T3;
		else if (info->battery_temp >= info->data.temp_t1_thres)
			sw_jeita->sm = TEMP_T1_TO_T2;
		else if (info->battery_temp >= info->data.temp_t0_thres)
			sw_jeita->sm = TEMP_T0_TO_T1;
		else
			sw_jeita->sm = TEMP_BELOW_T0;

		chr_err("[SW_JEITA] tmp:%d sm:%d\n",
			info->battery_temp, sw_jeita->sm);
	}
}

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
/* sw jeita end*/

static ssize_t show_charger_log_level(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	chr_err("%s: %d\n", __func__, chargerlog_level);
	return sprintf(buf, "%d\n", chargerlog_level);
}

static ssize_t store_charger_log_level(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret = 0;

	chr_err("%s\n", __func__);

	if (buf != NULL && size != 0) {
		chr_err("%s: buf is %s\n", __func__, buf);
		ret = kstrtoul(buf, 10, &val);
		if (ret < 0) {
			chr_err("%s: kstrtoul fail, ret = %d\n", __func__, ret);
			return ret;
		}
		if (val < 0) {
			chr_err("%s: val is inavlid: %ld\n", __func__, val);
			val = 0;
		}
		chargerlog_level = val;
		chr_err("%s: log_level=%d\n", __func__, chargerlog_level);
	}
	return size;
}
static DEVICE_ATTR(charger_log_level, 0664, show_charger_log_level,
		store_charger_log_level);

static __maybe_unused ssize_t show_pdc_max_watt_level(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct mtk_charger  *pinfo = dev->driver_data;

	return sprintf(buf, "%d\n", mtk_pdc_get_max_watt(pinfo));
}

static __maybe_unused ssize_t store_pdc_max_watt_level(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mtk_charger  *pinfo = dev->driver_data;
	int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		mtk_pdc_set_max_watt(pinfo, temp);
		chr_err("[store_pdc_max_watt]:%d\n", temp);
	} else
		chr_err("[store_pdc_max_watt]: format error!\n");

	return size;
}
static __maybe_unused DEVICE_ATTR(pdc_max_watt, 0644, show_pdc_max_watt_level,
		store_pdc_max_watt_level);


int charger_manager_notifier(struct mtk_charger *info, int event)
{
	return srcu_notifier_call_chain(&info->evt_nh, event, NULL);
}

int notify_battery_full(void)
{
	printk("notify_battery_full_is_ok\n");

	if (charger_manager_notifier(pinfo, CHARGER_NOTIFY_EOC)) {
		printk("notifier fail\n");
		return 1;
	} else {
		return 0;
	}
}


void mtk_charger_int_handler(void)
{
	if (pinfo == NULL) {
		chr_err("charger is not rdy ,skip1\n");
		return;
	}

	if (mt_get_charger_type() == CHARGER_UNKNOWN) {
		mutex_lock(&pinfo->cable_out_lock);
		pinfo->cable_out_cnt++;
		chr_err("cable_out_cnt=%d\n", pinfo->cable_out_cnt);
		mutex_unlock(&pinfo->cable_out_lock);
		charger_manager_notifier(pinfo, CHARGER_NOTIFY_STOP_CHARGING);
	} else {
#ifdef OPLUS_FEATURE_CHG_BASIC
		oplus_wake_up_usbtemp_thread();
#endif /*OPLUS_FEATURE_CHG_BASIC*/
		charger_manager_notifier(pinfo, CHARGER_NOTIFY_START_CHARGING);
	}
	chr_err("wake_up_charger\n");
	_wake_up_charger(pinfo);
}

/* procfs */
static int mtk_chg_set_cv_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;

	seq_printf(m, "%d\n", pinfo->data.battery_cv);
	return 0;
}

static int mtk_chg_set_cv_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_set_cv_show, pde_data(node));
}

static ssize_t mtk_chg_set_cv_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int cv = 0;
	struct mtk_charger *info = pde_data(file_inode(file));
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
	.proc_lseek = noop_llseek,
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
	return single_open(file, mtk_chg_current_cmd_show, pde_data(node));
}

static ssize_t mtk_chg_current_cmd_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0;
	char desc[32] = {0};
	int current_unlimited = 0;
	int cmd_discharging = 0;
	struct mtk_charger *info = pde_data(file_inode(file));

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
	.proc_lseek = noop_llseek,
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
	return single_open(file, mtk_chg_en_power_path_show, pde_data(node));
}

static ssize_t mtk_chg_en_power_path_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int enable = 0;
	struct mtk_charger *info = pde_data(file_inode(file));

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
	.proc_lseek = noop_llseek,
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
	return single_open(file, mtk_chg_en_safety_timer_show, pde_data(node));
}

static ssize_t mtk_chg_en_safety_timer_write(struct file *file,
	const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int enable = 0;
	struct mtk_charger *info = pde_data(file_inode(file));

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
	.proc_lseek = noop_llseek,
	.proc_release = single_release,
	.proc_write = mtk_chg_en_safety_timer_write,
};


int sc_get_sys_time(void);
int sc_get_sys_time(void)
{
	struct rtc_time tm_android = {0};
	struct timespec64 tv_android = {0};
	int timep = 0;

	ktime_get_real_ts64(&tv_android);
	rtc_time64_to_tm(tv_android.tv_sec, &tm_android);
	tv_android.tv_sec -= (uint64_t)sys_tz.tz_minuteswest * 60;
	rtc_time64_to_tm(tv_android.tv_sec, &tm_android);
	timep = tm_android.tm_sec + tm_android.tm_min * 60 + tm_android.tm_hour * 3600;

	return timep;
}

int sc_get_left_time(int s, int e, int now);
int sc_get_left_time(int s, int e, int now)
{
	if (e >= s) {
		if (now >= s && now < e)
			return e-now;
	} else {
		if (now >= s)
			return 86400 - now + e;
		else if (now < e)
			return e-now;
	}
	return 0;
}

char *sc_solToStr(int s);
char *sc_solToStr(int s)
{
	switch (s) {
	case SC_IGNORE:
		return "ignore";
	case SC_KEEP:
		return "keep";
	case SC_DISABLE:
		return "disable";
	case SC_REDUCE:
		return "reduce";
	default:
		return "none";
	}
}

int smart_charging(struct mtk_charger *info)
{
	int time_to_target = 0;
	int time_to_full_default_current = -1;
	int time_to_full_default_current_limit = -1;
	int ret_value = SC_KEEP;
	int sc_real_time = sc_get_sys_time();
	int sc_left_time = sc_get_left_time(info->sc.start_time, info->sc.end_time, sc_real_time);
	int sc_battery_percentage = get_uisoc(info) * 100;
	int sc_charger_current = get_battery_current(info);

	time_to_target = sc_left_time - info->sc.left_time_for_cv;

	if (info->sc.enable == false || sc_left_time <= 0
		|| sc_left_time < info->sc.left_time_for_cv
		|| (sc_charger_current <= 0 && info->sc.last_solution != SC_DISABLE))
		ret_value = SC_IGNORE;
	else {
		if (sc_battery_percentage > info->sc.target_percentage * 100) {
			if (time_to_target > 0)
				ret_value = SC_DISABLE;
		} else {
			if (sc_charger_current != 0)
				time_to_full_default_current =
					info->sc.battery_size * 3600 / 10000 *
					(10000 - sc_battery_percentage)
						/ sc_charger_current;
			else
				time_to_full_default_current =
					info->sc.battery_size * 3600 / 10000 *
					(10000 - sc_battery_percentage);
			chr_err("sc1: %d %d %d %d %d\n",
				time_to_full_default_current,
				info->sc.battery_size,
				sc_battery_percentage,
				sc_charger_current,
				info->sc.current_limit);

			if (time_to_full_default_current < time_to_target &&
				info->sc.current_limit != -1 &&
				sc_charger_current > info->sc.current_limit) {
				time_to_full_default_current_limit =
					info->sc.battery_size / 10000 *
					(10000 - sc_battery_percentage)
					/ info->sc.current_limit;

				chr_err("sc2: %d %d %d %d\n",
					time_to_full_default_current_limit,
					info->sc.battery_size,
					sc_battery_percentage,
					info->sc.current_limit);

				if (time_to_full_default_current_limit < time_to_target &&
					sc_charger_current > info->sc.current_limit)
					ret_value = SC_REDUCE;
			}
		}
	}
	info->sc.last_solution = ret_value;
	if (info->sc.last_solution == SC_DISABLE)
		info->sc.disable_charger = true;
	else
		info->sc.disable_charger = false;
	chr_err("[sc]disable_charger: %d\n", info->sc.disable_charger);
	chr_err("[sc1]en:%d t:%d,%d,%d,%d t:%d,%d,%d,%d c:%d,%d ibus:%d uisoc: %d,%d s:%d ans:%s\n",
		info->sc.enable, info->sc.start_time, info->sc.end_time,
		sc_real_time, sc_left_time, info->sc.left_time_for_cv,
		time_to_target, time_to_full_default_current, time_to_full_default_current_limit,
		sc_charger_current, info->sc.current_limit,
		get_ibus(info), get_uisoc(info), info->sc.target_percentage,
		info->sc.battery_size, sc_solToStr(info->sc.last_solution));

	return ret_value;
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

int _mtk_enable_charging(struct mtk_charger *info,
	bool en)
{
	chr_debug("%s en:%d\n", __func__, en);
	if (info->algo.enable_charging != NULL)
		return info->algo.enable_charging(info, en);
	return false;
}

#ifdef CONFIG_PM
static int charger_pm_event(struct notifier_block *notifier,
			unsigned long pm_event, void *unused)
{
	ktime_t ktime_now;
	struct timespec64 now;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		pinfo->is_suspend = true;
		chr_debug("%s: enter PM_SUSPEND_PREPARE\n", __func__);
		break;
	case PM_POST_SUSPEND:
		pinfo->is_suspend = false;
		chr_debug("%s: enter PM_POST_SUSPEND\n", __func__);
		ktime_now = ktime_get_boottime();
		now = ktime_to_timespec64(ktime_now);

		if (timespec64_compare(&now, &pinfo->endtime) >= 0 &&
			pinfo->endtime.tv_sec != 0 &&
			pinfo->endtime.tv_nsec != 0) {
			chr_err("%s: alarm timeout, wake up charger\n",
				__func__);
			__pm_relax(pinfo->charger_wakelock);
			pinfo->endtime.tv_sec = 0;
			pinfo->endtime.tv_nsec = 0;
			_wake_up_charger(pinfo);
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block charger_pm_notifier_func = {
	.notifier_call = charger_pm_event,
	.priority = 0,
};
#endif /* CONFIG_PM */

static enum alarmtimer_restart
	mtk_charger_alarm_timer_func(struct alarm *alarm, ktime_t now)
{
	if (pinfo->is_suspend == false) {
		chr_err("%s: not suspend, wake up charger\n", __func__);
		_wake_up_charger(pinfo);
	} else {
		chr_err("%s: alarm timer timeout\n", __func__);
		__pm_stay_awake(pinfo->charger_wakelock);
	}

	return ALARMTIMER_NORESTART;
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

	chr_err("%s: alarm timer start:%d, %lld %ld\n", __func__, ret,
		info->endtime.tv_sec, info->endtime.tv_nsec);
	alarm_start(&info->charger_timer, ktime);
}

static void mtk_charger_init_timer(struct mtk_charger *info)
{
	alarm_init(&info->charger_timer, ALARM_BOOTTIME,
			mtk_charger_alarm_timer_func);
	mtk_charger_start_timer(info);

#ifdef CONFIG_PM
	if (register_pm_notifier(&charger_pm_notifier_func))
		chr_err("%s: register pm failed\n", __func__);
#endif /* CONFIG_PM */
}

static void mtk_charger_parse_dt(struct mtk_charger *info,
				struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 val = 0;
	struct device_node *boot_node = NULL;
	struct tag_bootmode *tag = NULL;

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

	if (of_property_read_string(np, "algorithm-name",
		&info->algorithm_name) < 0) {
		if (of_property_read_string(np, "algorithm_name",
			&info->algorithm_name) < 0) {
			chr_err("%s: no algorithm_name, use Basic\n", __func__);
			info->algorithm_name = "Basic";
		}
	}

	info->disable_charger = of_property_read_bool(np, "disable_charger")
		|| of_property_read_bool(np, "disable-charger");
	info->charger_unlimited = of_property_read_bool(np, "charger_unlimited")
		|| of_property_read_bool(np, "charger-unlimited");
	info->atm_enabled = of_property_read_bool(np, "atm_is_enabled")
		|| of_property_read_bool(np, "atm-is-enabled");
	info->enable_sw_safety_timer =
			of_property_read_bool(np, "enable_sw_safety_timer")
			|| of_property_read_bool(np, "enable-sw-safety-timer");
	info->sw_safety_timer_setting = info->enable_sw_safety_timer;
	info->disable_aicl = of_property_read_bool(np, "disable_aicl")
		|| of_property_read_bool(np, "disable-aicl");
	info->alg_new_arbitration = of_property_read_bool(np, "alg_new_arbitration")
		|| of_property_read_bool(np, "alg-new-arbitration");
	info->alg_unchangeable = of_property_read_bool(np, "alg_unchangeable")
		|| of_property_read_bool(np, "alg-unchangeable");

	/* common */

	if (of_property_read_u32(np, "charger_configuration", &val) >= 0)
		info->config = val;
	else if (of_property_read_u32(np, "charger-configuration", &val) >= 0)
		info->config = val;
	else {
		chr_err("use default charger_configuration:%d\n",
			SINGLE_CHARGER);
		info->config = SINGLE_CHARGER;
	}

	if (of_property_read_u32(np, "battery_cv", &val) >= 0)
		info->data.battery_cv = val;
	else if (of_property_read_u32(np, "battery-cv", &val) >= 0)
		info->data.battery_cv = val;
	else {
		chr_err("use default BATTERY_CV:%d\n", BATTERY_CV);
		info->data.battery_cv = BATTERY_CV;
	}


	info->enable_boot_volt =
		of_property_read_bool(np, "enable_boot_volt")
		|| of_property_read_bool(np, "enable-boot-volt");

	if (of_property_read_u32(np, "max_charger_voltage", &val) >= 0)
		info->data.max_charger_voltage = val;
	else if (of_property_read_u32(np, "max-charger-voltage", &val) >= 0)
		info->data.max_charger_voltage = val;
	else {
		chr_err("use default V_CHARGER_MAX:%d\n", V_CHARGER_MAX);
		info->data.max_charger_voltage = V_CHARGER_MAX;
	}
	info->data.max_charger_voltage_setting = info->data.max_charger_voltage;

	if (of_property_read_u32(np, "vbus_sw_ovp_voltage", &val) >= 0)
		info->data.vbus_sw_ovp_voltage = val;
	else if (of_property_read_u32(np, "vbus-sw-ovp-voltage", &val) >= 0)
		info->data.vbus_sw_ovp_voltage = val;
	else {
		chr_err("use default V_CHARGER_MAX:%d\n", V_CHARGER_MAX);
		info->data.vbus_sw_ovp_voltage = VBUS_OVP_VOLTAGE;
	}

	if (of_property_read_u32(np, "min_charger_voltage", &val) >= 0)
		info->data.min_charger_voltage = val;
	else if (of_property_read_u32(np, "min-charger-voltage", &val) >= 0)
		info->data.min_charger_voltage = val;
	else {
		chr_err("use default V_CHARGER_MIN:%d\n", V_CHARGER_MIN);
		info->data.min_charger_voltage = V_CHARGER_MIN;
	}

	if (of_property_read_u32(np, "enable_vbat_mon", &val) >= 0) {
		info->enable_vbat_mon = val;
		info->enable_vbat_mon_bak = val;
	} else if (of_property_read_u32(np, "enable-vbat-mon", &val) >= 0) {
		info->enable_vbat_mon = val;
		info->enable_vbat_mon_bak = val;
	} else {
		chr_err("use default enable 6pin\n");
		info->enable_vbat_mon = 0;
		info->enable_vbat_mon_bak = 0;
	}
	chr_err("enable_vbat_mon:%d\n", info->enable_vbat_mon);

	/* sw jeita */
	info->enable_sw_jeita = of_property_read_bool(np, "enable_sw_jeita")
		|| of_property_read_bool(np, "enable-sw-jeita");

	if (of_property_read_u32(np, "jeita_temp_above_t4_cv", &val) >= 0)
		info->data.jeita_temp_above_t4_cv = val;
	else if (of_property_read_u32(np, "jeita-temp-above-t4-cv", &val) >= 0)
		info->data.jeita_temp_above_t4_cv = val;
	else {
		chr_err("use default JEITA_TEMP_ABOVE_T4_CV:%d\n",
			JEITA_TEMP_ABOVE_T4_CV);
		info->data.jeita_temp_above_t4_cv = JEITA_TEMP_ABOVE_T4_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t3_to_t4_cv", &val) >= 0)
		info->data.jeita_temp_t3_to_t4_cv = val;
	else if (of_property_read_u32(np, "jeita-temp-t3-to-t4-cv", &val) >= 0)
		info->data.jeita_temp_t3_to_t4_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T3_TO_T4_CV:%d\n",
			JEITA_TEMP_T3_TO_T4_CV);
		info->data.jeita_temp_t3_to_t4_cv = JEITA_TEMP_T3_TO_T4_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t2_to_t3_cv", &val) >= 0)
		info->data.jeita_temp_t2_to_t3_cv = val;
	else if (of_property_read_u32(np, "jeita-temp-t2-to-t3-cv", &val) >= 0)
		info->data.jeita_temp_t2_to_t3_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T2_TO_T3_CV:%d\n",
			JEITA_TEMP_T2_TO_T3_CV);
		info->data.jeita_temp_t2_to_t3_cv = JEITA_TEMP_T2_TO_T3_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t1_to_t2_cv", &val) >= 0)
		info->data.jeita_temp_t1_to_t2_cv = val;
	else if (of_property_read_u32(np, "jeita-temp-t1-to-t2-cv", &val) >= 0)
		info->data.jeita_temp_t1_to_t2_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T1_TO_T2_CV:%d\n",
			JEITA_TEMP_T1_TO_T2_CV);
		info->data.jeita_temp_t1_to_t2_cv = JEITA_TEMP_T1_TO_T2_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t0_to_t1_cv", &val) >= 0)
		info->data.jeita_temp_t0_to_t1_cv = val;
	else if (of_property_read_u32(np, "jeita-temp-t0-to-t1-cv", &val) >= 0)
		info->data.jeita_temp_t0_to_t1_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T0_TO_T1_CV:%d\n",
			JEITA_TEMP_T0_TO_T1_CV);
		info->data.jeita_temp_t0_to_t1_cv = JEITA_TEMP_T0_TO_T1_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_below_t0_cv", &val) >= 0)
		info->data.jeita_temp_below_t0_cv = val;
	if (of_property_read_u32(np, "jeita-temp-below-t0-cv", &val) >= 0)
		info->data.jeita_temp_below_t0_cv = val;
	else {
		chr_err("use default JEITA_TEMP_BELOW_T0_CV:%d\n",
			JEITA_TEMP_BELOW_T0_CV);
		info->data.jeita_temp_below_t0_cv = JEITA_TEMP_BELOW_T0_CV;
	}

	if (of_property_read_u32(np, "temp_t4_thres", &val) >= 0)
		info->data.temp_t4_thres = val;
	else if (of_property_read_u32(np, "temp-t4-thres", &val) >= 0)
		info->data.temp_t4_thres = val;
	else {
		chr_err("use default TEMP_T4_THRES:%d\n",
			TEMP_T4_THRES);
		info->data.temp_t4_thres = TEMP_T4_THRES;
	}

	if (of_property_read_u32(np, "temp_t4_thres_minus_x_degree", &val) >= 0)
		info->data.temp_t4_thres_minus_x_degree = val;
	else if (of_property_read_u32(np, "temp-t4-thres-minus-x-degree", &val) >= 0)
		info->data.temp_t4_thres_minus_x_degree = val;
	else {
		chr_err("use default TEMP_T4_THRES_MINUS_X_DEGREE:%d\n",
			TEMP_T4_THRES_MINUS_X_DEGREE);
		info->data.temp_t4_thres_minus_x_degree =
					TEMP_T4_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t3_thres", &val) >= 0)
		info->data.temp_t3_thres = val;
	else if (of_property_read_u32(np, "temp-t3-thres", &val) >= 0)
		info->data.temp_t3_thres = val;
	else {
		chr_err("use default TEMP_T3_THRES:%d\n",
			TEMP_T3_THRES);
		info->data.temp_t3_thres = TEMP_T3_THRES;
	}

	if (of_property_read_u32(np, "temp_t3_thres_minus_x_degree", &val) >= 0)
		info->data.temp_t3_thres_minus_x_degree = val;
	else if (of_property_read_u32(np, "temp-t3-thres-minus-x-degree", &val) >= 0)
		info->data.temp_t3_thres_minus_x_degree = val;
	else {
		chr_err("use default TEMP_T3_THRES_MINUS_X_DEGREE:%d\n",
			TEMP_T3_THRES_MINUS_X_DEGREE);
		info->data.temp_t3_thres_minus_x_degree =
					TEMP_T3_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t2_thres", &val) >= 0)
		info->data.temp_t2_thres = val;
	else if (of_property_read_u32(np, "temp-t2-thres", &val) >= 0)
		info->data.temp_t2_thres = val;
	else {
		chr_err("use default TEMP_T2_THRES:%d\n",
			TEMP_T2_THRES);
		info->data.temp_t2_thres = TEMP_T2_THRES;
	}

	if (of_property_read_u32(np, "temp_t2_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t2_thres_plus_x_degree = val;
	else if (of_property_read_u32(np, "temp-t2-thres-plus-x-degree", &val) >= 0)
		info->data.temp_t2_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T2_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T2_THRES_PLUS_X_DEGREE);
		info->data.temp_t2_thres_plus_x_degree =
					TEMP_T2_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t1_thres", &val) >= 0)
		info->data.temp_t1_thres = val;
	else if (of_property_read_u32(np, "temp-t1-thres", &val) >= 0)
		info->data.temp_t1_thres = val;
	else {
		chr_err("use default TEMP_T1_THRES:%d\n",
			TEMP_T1_THRES);
		info->data.temp_t1_thres = TEMP_T1_THRES;
	}

	if (of_property_read_u32(np, "temp_t1_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t1_thres_plus_x_degree = val;
	else if (of_property_read_u32(np, "temp-t1-thres-plus-x-degree", &val) >= 0)
		info->data.temp_t1_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T1_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T1_THRES_PLUS_X_DEGREE);
		info->data.temp_t1_thres_plus_x_degree =
					TEMP_T1_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t0_thres", &val) >= 0)
		info->data.temp_t0_thres = val;
	else if (of_property_read_u32(np, "temp-t0-thres", &val) >= 0)
		info->data.temp_t0_thres = val;
	else {
		chr_err("use default TEMP_T0_THRES:%d\n",
			TEMP_T0_THRES);
		info->data.temp_t0_thres = TEMP_T0_THRES;
	}

	if (of_property_read_u32(np, "temp_t0_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t0_thres_plus_x_degree = val;
	else if (of_property_read_u32(np, "temp-t0-thres-plus-x-degree", &val) >= 0)
		info->data.temp_t0_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T0_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T0_THRES_PLUS_X_DEGREE);
		info->data.temp_t0_thres_plus_x_degree =
					TEMP_T0_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_neg_10_thres", &val) >= 0)
		info->data.temp_neg_10_thres = val;
	else if (of_property_read_u32(np, "temp-neg-10-thres", &val) >= 0)
		info->data.temp_neg_10_thres = val;
	else {
		chr_err("use default TEMP_NEG_10_THRES:%d\n",
			TEMP_NEG_10_THRES);
		info->data.temp_neg_10_thres = TEMP_NEG_10_THRES;
	}

	/* battery temperature protection */
	info->thermal.sm = BAT_TEMP_NORMAL;
	info->thermal.enable_min_charge_temp =
		of_property_read_bool(np, "enable_min_charge_temp")
		|| of_property_read_bool(np, "enable-min-charge-temp");

	if (of_property_read_u32(np, "min_charge_temp", &val) >= 0)
		info->thermal.min_charge_temp = val;
	else if (of_property_read_u32(np, "min-charge-temp", &val) >= 0)
		info->thermal.min_charge_temp = val;
	else {
		chr_err("use default MIN_CHARGE_TEMP:%d\n",
			MIN_CHARGE_TEMP);
		info->thermal.min_charge_temp = MIN_CHARGE_TEMP;
	}

	if (of_property_read_u32(np, "min_charge_temp_plus_x_degree", &val)
		>= 0) {
		info->thermal.min_charge_temp_plus_x_degree = val;
	} else if (of_property_read_u32(np, "min-charge-temp-plus-x-degree", &val)
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
	else if (of_property_read_u32(np, "max-charge-temp", &val) >= 0)
		info->thermal.max_charge_temp = val;
	else {
		chr_err("use default MAX_CHARGE_TEMP:%d\n",
			MAX_CHARGE_TEMP);
		info->thermal.max_charge_temp = MAX_CHARGE_TEMP;
	}

	if (of_property_read_u32(np, "max_charge_temp_minus_x_degree", &val)
		>= 0) {
		info->thermal.max_charge_temp_minus_x_degree = val;
	} else if (of_property_read_u32(np, "max-charge-temp-minus-x-degree", &val)
		>= 0) {
		info->thermal.max_charge_temp_minus_x_degree = val;
	} else {
		chr_err("use default MAX_CHARGE_TEMP_MINUS_X_DEGREE:%d\n",
			MAX_CHARGE_TEMP_MINUS_X_DEGREE);
		info->thermal.max_charge_temp_minus_x_degree =
					MAX_CHARGE_TEMP_MINUS_X_DEGREE;
	}

	/* charging current */
	if (of_property_read_u32(np, "usb_charger_current", &val) >= 0)
		info->data.usb_charger_current = val;
	else if (of_property_read_u32(np, "usb-charger-current", &val) >= 0)
		info->data.usb_charger_current = val;
	else {
		chr_err("use default USB_CHARGER_CURRENT:%d\n",
			USB_CHARGER_CURRENT);
		info->data.usb_charger_current = USB_CHARGER_CURRENT;
	}

	if (of_property_read_u32(np, "ac_charger_current", &val) >= 0)
		info->data.ac_charger_current = val;
	if (of_property_read_u32(np, "ac-charger-current", &val) >= 0)
		info->data.ac_charger_current = val;
	else {
		chr_err("use default AC_CHARGER_CURRENT:%d\n",
			AC_CHARGER_CURRENT);
		info->data.ac_charger_current = AC_CHARGER_CURRENT;
	}

	if (of_property_read_u32(np, "ac_charger_input_current", &val) >= 0)
		info->data.ac_charger_input_current = val;
	else if (of_property_read_u32(np, "ac-charger-input-current", &val) >= 0)
		info->data.ac_charger_input_current = val;
	else {
		chr_err("use default AC_CHARGER_INPUT_CURRENT:%d\n",
			AC_CHARGER_INPUT_CURRENT);
		info->data.ac_charger_input_current = AC_CHARGER_INPUT_CURRENT;
	}

	if (of_property_read_u32(np, "charging_host_charger_current", &val)
		>= 0) {
		info->data.charging_host_charger_current = val;
	} else if (of_property_read_u32(np, "charging-host-charger-current", &val)
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
			of_property_read_bool(np, "enable_dynamic_mivr")
			|| of_property_read_bool(np, "enable-dynamic-mivr");

	if (of_property_read_u32(np, "min_charger_voltage_1", &val) >= 0)
		info->data.min_charger_voltage_1 = val;
	else if (of_property_read_u32(np, "min-charger-voltage-1", &val) >= 0)
		info->data.min_charger_voltage_1 = val;
	else {
		chr_err("use default V_CHARGER_MIN_1: %d\n", V_CHARGER_MIN_1);
		info->data.min_charger_voltage_1 = V_CHARGER_MIN_1;
	}

	if (of_property_read_u32(np, "min_charger_voltage_2", &val) >= 0)
		info->data.min_charger_voltage_2 = val;
	else if (of_property_read_u32(np, "min-charger-voltage-2", &val) >= 0)
		info->data.min_charger_voltage_2 = val;
	else {
		chr_err("use default V_CHARGER_MIN_2: %d\n", V_CHARGER_MIN_2);
		info->data.min_charger_voltage_2 = V_CHARGER_MIN_2;
	}

	if (of_property_read_u32(np, "max_dmivr_charger_current", &val) >= 0)
		info->data.max_dmivr_charger_current = val;
	else if (of_property_read_u32(np, "max-dmivr-charger-current", &val) >= 0)
		info->data.max_dmivr_charger_current = val;
	else {
		chr_err("use default MAX_DMIVR_CHARGER_CURRENT: %d\n",
			MAX_DMIVR_CHARGER_CURRENT);
		info->data.max_dmivr_charger_current =
					MAX_DMIVR_CHARGER_CURRENT;
	}
	/* fast charging algo support indicator */
	info->enable_fast_charging_indicator =
			of_property_read_bool(np, "enable_fast_charging_indicator")
			|| of_property_read_bool(np, "enable-fast-charging-indicator");

	/*	adapter priority */
	if (of_property_read_u32(np, "adapter-priority", &val)>= 0)
		info->setting.adapter_priority = val;

	/*	dual parallel battery*/
	np = of_parse_phandle(dev->of_node, "current-selector", 0);
	if (np) {
		info->cs_gpio_index = of_get_named_gpio(dev->of_node, "cs-gpios", 0);
		if (of_property_read_string(np, "cs-name",
			&info->curr_select_name) < 0) {
			chr_err("%s: no cs-name\n", __func__);
			info->curr_select_name = "NULL";
		}
		info->cs_with_gauge =
			of_property_read_bool(np, "cs-gauge");
		chr_err("%s: %d\n", __func__, info->cs_with_gauge);
		if (of_property_read_u32(np, "comp-resist", &val) >= 0)
			info->comp_resist = val;
		else
			info->comp_resist = 25;
	} else {
		chr_err("%s: failed to get current_selector\n", __func__);
		info->cs_hw_disable = true;
		info->curr_select_name = "NULL";
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	info->support_ntc_01c_precision = of_property_read_bool(np, "qcom,support_ntc_01c_precision");
	chr_debug("%s: support_ntc_01c_precision: %d\n",
		__func__, info->support_ntc_01c_precision);
#endif
}

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

static ssize_t alg_new_arbitration_show(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_debug("%s: %d\n", __func__, pinfo->alg_new_arbitration);
	return sprintf(buf, "%d\n", pinfo->alg_new_arbitration);
}

static ssize_t alg_new_arbitration_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int temp;

	if (kstrtouint(buf, 10, &temp) == 0)
		pinfo->alg_new_arbitration = temp;
	else
		chr_err("%s: format error!\n", __func__);

	_wake_up_charger(pinfo);
	return size;
}
static DEVICE_ATTR_RW(alg_new_arbitration);

static ssize_t alg_unchangeable_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_debug("%s: %d\n", __func__, pinfo->alg_unchangeable);
	return sprintf(buf, "%d\n", pinfo->alg_unchangeable);
}

static ssize_t alg_unchangeable_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int temp;

	if (kstrtouint(buf, 10, &temp) == 0)
		pinfo->alg_unchangeable = temp;
	else
		chr_err("%s: format error!\n", __func__);

	_wake_up_charger(pinfo);
	return size;
}
static DEVICE_ATTR_RW(alg_unchangeable);

static ssize_t High_voltage_chg_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: hv_charging = %d\n", __func__, pinfo->enable_hv_charging);
	return sprintf(buf, "%d\n", pinfo->enable_hv_charging);
}
static DEVICE_ATTR_RO(High_voltage_chg_enable);

static ssize_t Rust_detect_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: Rust detect = %d\n", __func__, pinfo->record_water_detected);
	return sprintf(buf, "%d\n", pinfo->record_water_detected);
}
static DEVICE_ATTR_RO(Rust_detect);

static ssize_t Thermal_throttle_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	struct charger_data *chg_data = &(pinfo->chg_data[CHG1_SETTING]);

	return sprintf(buf, "%d\n", chg_data->thermal_throttle_record);
}
static DEVICE_ATTR_RO(Thermal_throttle);

static ssize_t ta_type_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	char *ta_type_name = "None";
	int ta_type = MTK_CAP_TYPE_UNKNOWN;

	ta_type = adapter_dev_get_property(pinfo->select_adapter, CAP_TYPE);
	switch (ta_type) {
	case MTK_CAP_TYPE_UNKNOWN:
		ta_type_name = "None";
		break;
	case MTK_PD:
		ta_type_name = "PD";
		break;
	case MTK_UFCS:
		ta_type_name = "UFCS";
		break;
	case MTK_PD_APDO:
		ta_type_name = "PD with PPS";
		break;
	}
	chr_err("%s: %d\n", __func__, ta_type);
	return sprintf(buf, "%s\n", ta_type_name);
}
static DEVICE_ATTR_RO(ta_type);

static ssize_t Charging_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret = 0, i = 0;
	char *alg_name = "normal";
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
	if (alg == NULL)
		return sprintf(buf, "%s\n", alg_name);

	switch (alg->alg_id) {
	case PE_ID:
		alg_name = "PE";
		break;
	case PE2_ID:
		alg_name = "PE2";
		break;
	case PDC_ID:
		alg_name = "PDC";
		break;
	case PE4_ID:
		alg_name = "PE4";
		break;
	case PE5_ID:
		alg_name = "P5";
		break;
	case PE5P_ID:
		alg_name = "P5P";
		break;
	}
	chr_err("%s: charging_mode: %s\n", __func__, alg_name);
	return sprintf(buf, "%s\n", alg_name);
}
static DEVICE_ATTR_RO(Charging_mode);

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

	if ((pinfo->fast_charging_indicator > 0) &&
	    (pinfo->bootmode == 8 || pinfo->bootmode == 9)) {
		pinfo->log_level = CHRLOG_DEBUG_LEVEL;
		mtk_charger_set_algo_log_level(pinfo, pinfo->log_level);
	}

	_wake_up_charger(pinfo);
	return size;
}
static DEVICE_ATTR_RW(fast_chg_indicator);

static ssize_t cs_para_mode_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_debug("%s: %d\n", __func__, pinfo->cs_para_mode);
	return sprintf(buf, "%d\n", pinfo->cs_para_mode);
}

static ssize_t cs_para_mode_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int temp;

	if (kstrtouint(buf, 10, &temp) == 0)
		pinfo->cs_para_mode = temp;
	else
		chr_err("%s: format error!\n", __func__);

	if (pinfo->cs_para_mode > 0) {
		pinfo->log_level = CHRLOG_DEBUG_LEVEL;
		mtk_charger_set_algo_log_level(pinfo, pinfo->log_level);
	}

	_wake_up_charger(pinfo);
	return size;
}
static DEVICE_ATTR_RW(cs_para_mode);

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
		if (ret == -ERANGE || ret == -EINVAL)
			return -EINVAL;
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
	long val = 0;
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
		ret = kstrtol(buf, 10, &val);
		if (ret == -ERANGE || ret == -EINVAL)
			return -EINVAL;
		if (val < 0) {
			chr_err(
				"[smartcharging stime] val is %ld ??\n",
				val);
			val = 0;
		}

		if (val >= 0)
			info->sc.start_time = (int)val;

		chr_err(
			"[smartcharging stime]enable smartcharging=%d\n",
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
	long val = 0;
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
		ret = kstrtol(buf, 10, &val);
		if (ret == -ERANGE || ret == -EINVAL)
			return -EINVAL;
		if (val < 0) {
			chr_err(
				"[smartcharging etime] val is %ld ??\n",
				val);
			val = 0;
		}

		if (val >= 0)
			info->sc.end_time = (int)val;

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
	long val = 0;
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
		ret = kstrtol(buf, 10, &val);
		if (ret == -ERANGE || ret == -EINVAL)
			return -EINVAL;
		if (val < 0) {
			chr_err(
				"[smartcharging tuisoc] val is %ld ??\n",
				val);
			val = 0;
		}

		if (val >= 0)
			info->sc.target_percentage = (int)val;

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
	long val = 0;
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
		ret = kstrtol(buf, 10, &val);
		if (ret == -ERANGE || ret == -EINVAL)
			return -EINVAL;
		if (val < 0) {
			chr_err(
				"[smartcharging ibat limit] val is %ld ??\n",
				val);
			val = 0;
		}

		if (val >= 0)
			info->sc.current_limit = (int)val;

		chr_err(
			"[smartcharging ibat limit]=%d\n",
			info->sc.current_limit);
	}
	return size;
}
static DEVICE_ATTR_RW(sc_ibat_limit);

static ssize_t enable_power_path_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;
	bool power_path_en = true;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	charger_dev_is_powerpath_enabled(info->chg1_dev, &power_path_en);
	return sprintf(buf, "%d\n", power_path_en);
}

static ssize_t enable_power_path_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	long val = 0;
	int ret;
	bool enable = true;
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
		ret = kstrtoul(buf, 10, &val);
		if (ret == -ERANGE || ret == -EINVAL)
			return -EINVAL;
		if (val == 0)
			enable = false;
		else
			enable = true;

		charger_dev_enable_powerpath(info->chg1_dev, enable);
		info->cmd_pp = enable;
		chr_err("%s: enable power path = %d\n", __func__, enable);
	}

	return size;
}
static DEVICE_ATTR_RW(enable_power_path);

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

static ssize_t cs_heatlim_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_debug("%s: %d\n", __func__, pinfo->cs_heatlim);
	return sprintf(buf, "%d\n", pinfo->cs_heatlim);
}

static ssize_t cs_heatlim_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int temp;

	if (kstrtouint(buf, 10, &temp) == 0)
		pinfo->cs_heatlim = temp;
	else
		chr_err("%s: format error!\n", __func__);

	if (pinfo->cs_heatlim > 0) {
		pinfo->log_level = CHRLOG_DEBUG_LEVEL;
		mtk_charger_set_algo_log_level(pinfo, pinfo->log_level);
	}

	_wake_up_charger(pinfo);
	return size;
}
static DEVICE_ATTR_RW(cs_heatlim);

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

static ssize_t sw_ovp_threshold_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->data.max_charger_voltage);
	return sprintf(buf, "%d\n", pinfo->data.max_charger_voltage);
}

static ssize_t sw_ovp_threshold_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp < 0)
			pinfo->data.max_charger_voltage = pinfo->data.vbus_sw_ovp_voltage;
		else
			pinfo->data.max_charger_voltage = temp;
		chr_err("%s: %d\n", __func__, pinfo->data.max_charger_voltage);

	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}
static DEVICE_ATTR_RW(sw_ovp_threshold);

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

static ssize_t show_input_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	pr_debug("[Battery] %s : %x\n",
		__func__, pinfo->chg_data[CHG1_SETTING].thermal_input_current_limit);
	return sprintf(buf, "%u\n",
			pinfo->chg_data[CHG1_SETTING].thermal_input_current_limit);
}

static ssize_t store_input_current(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int reg = 0;
	int ret;

	pr_debug("[Battery] %s\n", __func__);
	if (buf != NULL && size != 0) {
		pr_debug("[Battery] buf is %s and size is %zu\n", buf, size);
		ret = kstrtouint(buf, 16, &reg);
		pinfo->chg_data[CHG1_SETTING].thermal_input_current_limit = reg;
		pr_debug("[Battery] %s: %x\n",
			__func__, pinfo->chg_data[CHG1_SETTING].thermal_input_current_limit);
	}
	return size;
}
static DEVICE_ATTR(input_current, 0644, show_input_current,
		store_input_current);

static  __maybe_unused ssize_t show_chg1_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	pr_debug("[Battery] %s : %x\n",
		__func__, pinfo->chg_data[CHG1_SETTING].thermal_charging_current_limit);
	return sprintf(buf, "%u\n",
			pinfo->chg_data[CHG1_SETTING].thermal_charging_current_limit);
}

static __maybe_unused ssize_t store_chg1_current(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int reg = 0;
	int ret;

	pr_debug("[Battery] %s\n", __func__);
	if (buf != NULL && size != 0) {
		pr_debug("[Battery] buf is %s and size is %zu\n", buf, size);
		ret = kstrtouint(buf, 16, &reg);
		pinfo->chg_data[CHG1_SETTING].thermal_charging_current_limit = reg;
		pr_debug("[Battery] %s: %x\n", __func__,
			pinfo->chg_data[CHG1_SETTING].thermal_charging_current_limit);
	}
	return size;
}
static  __maybe_unused DEVICE_ATTR(chg1_current, 0644, show_chg1_current, store_chg1_current);

static  __maybe_unused ssize_t show_chg2_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	pr_debug("[Battery] %s : %x\n",
		__func__, pinfo->chg_data[CHG2_SETTING].thermal_charging_current_limit);
	return sprintf(buf, "%u\n",
			pinfo->chg_data[CHG2_SETTING].thermal_charging_current_limit);
}

static  __maybe_unused ssize_t store_chg2_current(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int reg = 0;
	int ret;

	pr_debug("[Battery] %s\n", __func__);
	if (buf != NULL && size != 0) {
		pr_debug("[Battery] buf is %s and size is %zu\n", buf, size);
		ret = kstrtouint(buf, 16, &reg);
		pinfo->chg_data[CHG2_SETTING].thermal_charging_current_limit = reg;
		pr_debug("[Battery] %s: %x\n", __func__,
			pinfo->chg_data[CHG2_SETTING].thermal_charging_current_limit);
	}
	return size;
}
static  __maybe_unused DEVICE_ATTR(chg2_current, 0644, show_chg2_current, store_chg2_current);

static ssize_t show_BatNotify(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	pr_debug("[Battery] show_BatteryNotify: 0x%x\n", pinfo->notify_code);

	return sprintf(buf, "%u\n", pinfo->notify_code);
}

static ssize_t store_BatNotify(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int reg = 0;
	int ret;

	pr_debug("[Battery] store_BatteryNotify\n");
	if (buf != NULL && size != 0) {
		pr_debug("[Battery] buf is %s and size is %zu\n", buf, size);
		ret = kstrtouint(buf, 16, &reg);
		pinfo->notify_code = reg;
		pr_debug("[Battery] store code: 0x%x\n", pinfo->notify_code);

		mtk_chgstat_notify(pinfo);
	}
	return size;
}

static DEVICE_ATTR(BatteryNotify, 0664, show_BatNotify, store_BatNotify);

static  __maybe_unused ssize_t show_BN_TestMode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	pr_debug("[Battery] %s : %x\n", __func__, pinfo->notify_test_mode);
	return sprintf(buf, "%u\n", pinfo->notify_test_mode);
}

static  __maybe_unused ssize_t store_BN_TestMode(struct device *dev,
		struct device_attribute *attr, const char *buf,  size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int reg = 0;
	int ret;

	pr_debug("[Battery] %s\n", __func__);
	if (buf != NULL && size != 0) {
		pr_debug("[Battery] buf is %s and size is %zu\n", buf, size);
		ret = kstrtouint(buf, 16, &reg);
		pinfo->notify_test_mode = reg;
		pr_debug("[Battery] store mode: %x\n", pinfo->notify_test_mode);
	}
	return size;
}
static  __maybe_unused DEVICE_ATTR(BN_TestMode, 0644, show_BN_TestMode, store_BN_TestMode);

/* Create sysfs and procfs attributes */
static int mtk_charger_setup_files(struct platform_device *pdev)
{
	int ret = 0;
	struct proc_dir_entry *battery_dir = NULL, *entry = NULL;
	struct mtk_charger *info = platform_get_drvdata(pdev);

	ret = device_create_file(&(pdev->dev), &dev_attr_sw_jeita);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_sw_ovp_threshold);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_chr_type);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_enable_meta_current_limit);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_cs_heatlim);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_cs_para_mode);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_fast_chg_indicator);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_Charging_mode);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_ta_type);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_High_voltage_chg_enable);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_Rust_detect);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_Thermal_throttle);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_alg_new_arbitration);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_alg_unchangeable);
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

	ret = device_create_file(&(pdev->dev), &dev_attr_enable_power_path);
	if (ret)
		goto _out;

	battery_dir = proc_mkdir("mtk_battery_cmd", NULL);
	if (!battery_dir) {
		chr_err("%s: mkdir /proc/mtk_battery_cmd failed\n", __func__);
		return -ENOMEM;
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

	return 0;

fail_procfs:
	remove_proc_subtree("mtk_battery_cmd", NULL);
_out:
	return ret;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static void oplus_chg_pps_get_source_cap(struct mtk_charger *info);
#endif

int notify_adapter_event(struct notifier_block *notifier,
			unsigned long evt, void *val)
{
	struct mtk_charger *pinfo;
	u32 boot_mode = 0;
	bool report_psy = true;
	int index = 0;
	struct info_notifier_block *ta_nb;

	ta_nb = container_of(notifier, struct info_notifier_block, nb);
	pinfo = ta_nb->info;
	pinfo->ta_hardreset = false;
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
		pinfo->ta_hardreset = true;
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
			oplus_vooc_set_disable_adapter_output(true);
			if (g_oplus_chip && g_oplus_chip->usb_psy)
				power_supply_changed(g_oplus_chip->usb_psy);
		} else {
			pinfo->notify_code &= ~CHG_TYPEC_WD_STATUS;
			if (boot_mode == 8 || boot_mode == 9)
				pinfo->enable_hv_charging = true;
			oplus_vooc_set_disable_adapter_output(false);
			if (g_oplus_chip && g_oplus_chip->usb_psy)
				power_supply_changed(g_oplus_chip->usb_psy);
		}
		mtk_chgstat_notify(pinfo);
		report_psy = boot_mode == 8 || boot_mode == 9;
		break;
	}
	chr_debug("%s: evt: pd:%d, ufcs:%d\n", __func__,
	pinfo->ta_status[PD], pinfo->ta_status[UFCS]);

	if (report_psy)
		power_supply_changed(pinfo->psy1);
	return NOTIFY_DONE;
}

void mt_usb_connect_v1(void)
{
	return;
}

void mt_usb_disconnect_v1(void)
{
	return;
}

static int proc_dump_log_show(struct seq_file *m, void *v)
{
	struct adapter_power_cap cap;
	int i;

	cap.nr = 0;
	cap.pdp = 0;
	for (i = 0; i < ADAPTER_CAP_MAX_NR; i++) {
		cap.max_mv[i] = 0;
		cap.min_mv[i] = 0;
		cap.ma[i] = 0;
		cap.type[i] = 0;
		cap.pwr_limit[i] = 0;
	}

	if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
		seq_puts(m, "********** PD APDO cap Dump **********\n");

		adapter_dev_get_cap(pinfo->pd_adapter, MTK_PD_APDO, &cap);
		for (i = 0; i < cap.nr; i++) {
			seq_printf(m,
			"%d: mV:%d,%d mA:%d type:%d pwr_limit:%d pdp:%d\n", i,
			cap.max_mv[i], cap.min_mv[i], cap.ma[i],
			cap.type[i], cap.pwr_limit[i], cap.pdp);
		}
	} else if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK
		|| pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30) {
		seq_puts(m, "********** PD cap Dump **********\n");

		adapter_dev_get_cap(pinfo->pd_adapter, MTK_PD, &cap);
		for (i = 0; i < cap.nr; i++) {
			seq_printf(m, "%d: mV:%d,%d mA:%d type:%d\n", i,
			cap.max_mv[i], cap.min_mv[i], cap.ma[i], cap.type[i]);
		}
	}

	return 0;
}

static ssize_t proc_write(
	struct file *file, const char __user *buffer,
	size_t count, loff_t *f_pos)
{
	return count;
}


static int proc_dump_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_dump_log_show, NULL);
}

const struct proc_ops charger_dump_log_proc_fops = {
	.proc_open = proc_dump_log_open,
	.proc_read = seq_read,
	.proc_lseek	= seq_lseek,
	.proc_write = proc_write,
};

void charger_debug_init(void)
{
	struct proc_dir_entry *charger_dir;

	charger_dir = proc_mkdir("mtk_charger", NULL);
	if (!charger_dir) {
		chr_err("fail to mkdir /proc/charger\n");
		return;
	}

	proc_create("dump_log", 0644,
		charger_dir, &charger_dump_log_proc_fops);
}

static bool oplus_ccdetect_check_is_wd0(struct oplus_chg_chip *chip)
{
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		chg_err("device tree info missing\n");
		return false;
	}

	if (of_property_read_bool(node, "qcom,ccdetect_by_wd0"))
		return true;

	return false;
}

bool oplus_chg_get_wd0_status(void)
{
	if (!pinfo) {
		chg_err("pinfo null!\n");
		return false;
	}

	return pinfo->wd0_detect;
}

bool oplus_ccdetect_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return false;
	}

	if (gpio_is_valid(chip->chgic_mtk.oplus_info->ccdetect_gpio))
		return true;

	return false;
}

bool oplus_get_otg_online_status_default(void)
{
	if (!g_oplus_chip || !pinfo || !pinfo->tcpc) {
		chg_err("fail to init oplus_chip\n");
		return false;
	}

	if (tcpm_inquire_typec_attach_state(pinfo->tcpc) == TYPEC_ATTACHED_SRC)
		g_oplus_chip->otg_online = true;
	else
		g_oplus_chip->otg_online = false;

	return g_oplus_chip->otg_online;
}

int oplus_get_otg_online_status(void)
{
	int online = 0;
	int level = 0;
	int typec_otg = 0;
	static int pre_level = 1;
	static int pre_typec_otg = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip || !pinfo || !pinfo->tcpc) {
		chg_err("g_oplus_chip not ready!\n");
		return false;
	}

	if (oplus_ccdetect_check_is_gpio(chip) == true) {
		level = gpio_get_value(chip->chgic_mtk.oplus_info->ccdetect_gpio);
		if (level != gpio_get_value(chip->chgic_mtk.oplus_info->ccdetect_gpio)) {
			chg_err("ccdetect_gpio is unstable, try again...\n");
			usleep_range(5000, 5100);
			level = gpio_get_value(chip->chgic_mtk.oplus_info->ccdetect_gpio);
		}
	} else
		return oplus_get_otg_online_status_default();

	online = (level == 1) ? DISCONNECT : STANDARD_TYPEC_DEV_CONNECT;

	if (tcpm_inquire_typec_attach_state(pinfo->tcpc) == TYPEC_ATTACHED_SRC)
		typec_otg = 1;
	else
		typec_otg = 0;

	online = online | ((typec_otg == 1) ? OTG_DEV_CONNECT : DISCONNECT);

	if ((pre_level ^ level) || (pre_typec_otg ^ typec_otg)) {
		pre_level = level;
		pre_typec_otg = typec_otg;
		chg_debug("gpio[%s], c-otg[%d], otg_online[%d]\n",
			 level ? "H" : "L", typec_otg, online);
	}

	chip->otg_online = typec_otg;
	return online;
}

bool oplus_get_otg_switch_status(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return false;
	}

	return chip->otg_switch;
}

int oplus_get_typec_cc_orientation(void)
{
	int typec_cc_orientation = 0;
	static struct tcpc_device *tcpc_dev = NULL;

	if (tcpc_dev == NULL)
		tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	if (tcpc_dev != NULL) {
		if (tcpm_inquire_typec_attach_state(tcpc_dev) != TYPEC_UNATTACHED) {
			typec_cc_orientation = (int)tcpm_inquire_cc_polarity(tcpc_dev) + 1;
		} else {
			typec_cc_orientation = 0;
		}
		if (typec_cc_orientation != 0)
			chg_err(" cc[%d]\n", typec_cc_orientation);
	} else {
		typec_cc_orientation = 0;
	}
	return typec_cc_orientation;
}

int oplus_check_pd_usb_type(void)
{
	struct tcpc_device *tcpc;
	int ret = 0;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc) {
		chg_err("get type_c_port0 fail\n");
		return PORT_ERROR;
	}

	if (!tcpm_inquire_pd_connected(tcpc))
		return PORT_A;

	ret = tcpm_inquire_dpm_flags(tcpc);
	if (ret & DPM_FLAGS_PARTNER_USB_COMM)
		return PORT_PD_WITH_USB;

	return PORT_PD_WITHOUT_USB;
}

bool oplus_usbtemp_condition(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip)
		return false;

	return oplus_chg_get_vbus_status(chip);
}

static void oplus_usbtemp_thread_init(void)
{
	if (g_oplus_chip == NULL)
		return;

	if (g_oplus_chip->support_usbtemp_protect_v2)
		oplus_usbtemp_kthread = kthread_run(oplus_usbtemp_monitor_common_new_method,
				g_oplus_chip, "usbtemp_kthread");
	else
		oplus_usbtemp_kthread = kthread_run(oplus_usbtemp_monitor_common,
				g_oplus_chip, "usbtemp_kthread");

	if (IS_ERR(oplus_usbtemp_kthread))
		chg_err("failed to cread oplus_usbtemp_kthread\n");
}

void oplus_wake_up_usbtemp_thread(void)
{
	if (g_oplus_chip == NULL) {
		chg_err("g_oplus_chip not ready return\n");
		return;
	}

	if (oplus_usbtemp_check_is_support() == true) {
		g_oplus_chip->usbtemp_check = oplus_usbtemp_condition();
		wake_up_interruptible(&oplus_usbtemp_wq);
		chg_debug("wake_up_usbtemp_thread, vbus_status:%d\n", oplus_usbtemp_condition());
	}
}
EXPORT_SYMBOL(oplus_wake_up_usbtemp_thread);

static int oplus_chg_chargerid_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;

	if (chip != NULL)
		node = chip->dev->of_node;

	if (chip == NULL || node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_gpio =
			of_get_named_gpio(node, "qcom,chargerid_switch-gpio", 0);
	if (chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("Couldn't read chargerid_switch-gpio rc=%d, chargerid_switch-gpio:%d\n",
				rc, chip->normalchg_gpio.chargerid_switch_gpio);
	} else {
		if (gpio_is_valid(chip->normalchg_gpio.chargerid_switch_gpio)) {
			rc = gpio_request(chip->normalchg_gpio.chargerid_switch_gpio, "charging_switch1-gpio");
			if (rc) {
				chg_err("unable to request chargerid_switch-gpio:%d\n",
						chip->normalchg_gpio.chargerid_switch_gpio);
			} else {
				rc = oplus_usb_switch_gpio_gpio_init();
				if (rc)
					chg_err("unable to init chargerid_switch-gpio:%d\n",
							chip->normalchg_gpio.chargerid_switch_gpio);
			}
		}
		chg_err("chargerid_switch-gpio:%d\n", chip->normalchg_gpio.chargerid_switch_gpio);
	}

	return 0;
}

static int oplus_chg_shipmode_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;

        if (chip != NULL)
        	node = chip->dev->of_node;

	if (chip == NULL || node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}

	chip->disable_ship_mode = of_property_read_bool(node, "qcom,disable-ship-mode");
	chg_err("disable_ship_mode:%d!!!\n", chip->disable_ship_mode);

	chip->normalchg_gpio.ship_gpio =
			of_get_named_gpio(node, "qcom,ship-gpio", 0);
	if (chip->normalchg_gpio.ship_gpio <= 0) {
		chg_err("Couldn't read qcom,ship-gpio rc = %d, qcom,ship-gpio:%d\n",
				rc, chip->normalchg_gpio.ship_gpio);
	} else {
		if (oplus_ship_check_is_gpio(chip) == true) {
			rc = gpio_request(chip->normalchg_gpio.ship_gpio, "ship-gpio");
			if (rc) {
				chg_err("unable to request ship-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
			} else {
				rc = oplus_ship_gpio_init(chip);
				if (rc)
					chg_err("unable to init ship-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
			}
		}
		chg_err("ship-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
	}

	return rc;
}

static int oplus_chg_shortc_hw_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;

        if (chip != NULL)
		node = chip->dev->of_node;

	if (chip == NULL || node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.shortc_gpio = of_get_named_gpio(node, "qcom,shortc-gpio", 0);
	if (chip->normalchg_gpio.shortc_gpio <= 0) {
		chg_err("Couldn't read qcom,shortc-gpio rc=%d, qcom,shortc-gpio:%d\n",
				rc, chip->normalchg_gpio.shortc_gpio);
	} else {
		if (oplus_shortc_check_is_gpio(chip) == true) {
			rc = gpio_request(chip->normalchg_gpio.shortc_gpio, "shortc-gpio");
			if (rc) {
				chg_err("unable to request shortc-gpio:%d\n",
						chip->normalchg_gpio.shortc_gpio);
			} else {
				rc = oplus_shortc_gpio_init(chip);
				if (rc)
					chg_err("unable to init shortc-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
			}
		}
		chg_err("shortc-gpio:%d\n", chip->normalchg_gpio.shortc_gpio);
	}

	return rc;
}

static int oplus_chg_usbtemp_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	u32 val = 0;
	struct device_node *node = NULL;

	if (chip)
		node = chip->dev->of_node;
	if (node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}

	if (of_property_read_u32(node, "qcom,usb_ntc_pullup", &val) >= 0)
		chip->usb_port_ntc_pullup = val;
	else {
		chip->usb_port_ntc_pullup = USB_NTC_PULLUP_NOTFROM_DTS;
		chg_err("do not set usb prot ntc pullup by dts\n");
	}

	chip->normalchg_gpio.dischg_gpio = of_get_named_gpio(node, "qcom,dischg-gpio", 0);
	if (chip->normalchg_gpio.dischg_gpio <= 0) {
		chg_err("Couldn't read qcom,dischg-gpio rc=%d, qcom,dischg-gpio:%d\n",
				rc, chip->normalchg_gpio.dischg_gpio);
	} else {
		if (oplus_usbtemp_check_is_support() == true) {
			rc = gpio_request(chip->normalchg_gpio.dischg_gpio, "dischg-gpio");
			if (rc) {
				chg_err("unable to request dischg-gpio:%d\n",
						chip->normalchg_gpio.dischg_gpio);
			} else {
				rc = oplus_dischg_gpio_init(chip);
				if (rc)
					chg_err("unable to init dischg-gpio:%d\n",
							chip->normalchg_gpio.dischg_gpio);

				rc = usbtemp_channel_init(chip->dev);
				if (rc)
					chg_err("unable to init usbtemp_channel\n");
			}
		}
		chg_err("dischg-gpio:%d\n", chip->normalchg_gpio.dischg_gpio);
	}

	return rc;
}

static void oplus_wd0_detect_work(struct work_struct *work)
{
	int level;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("g_oplus_chip not ready!\n");
		return;
	}

	level = !oplus_chg_get_wd0_status();

	if (level != 1) {
		oplus_wake_up_usbtemp_thread();
	} else {
		chip->usbtemp_check = oplus_usbtemp_condition();

		if (chip->usb_status == USB_TEMP_HIGH) {
			schedule_delayed_work(&pinfo->usbtemp_recover_work, 0);
		}
	}
}

static void oplus_usbtemp_recover_work(struct work_struct *work)
{
	oplus_usbtemp_recover_func(g_oplus_chip);
}

void oplus_ccdetect_irq_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return;
	}
	chip->chgic_mtk.oplus_info->ccdetect_irq = gpio_to_irq(chip->chgic_mtk.oplus_info->ccdetect_gpio);
	chg_err("chip->chgic_mtk.oplus_info->ccdetect_gpio[%d]!\n",
		chip->chgic_mtk.oplus_info->ccdetect_gpio);
}

irqreturn_t oplus_ccdetect_change_handler(int irq, void *data)
{
	cancel_delayed_work_sync(&pinfo->ccdetect_work);
	chg_err("Scheduling ccdetect work!\n");
	schedule_delayed_work(&pinfo->ccdetect_work,
			msecs_to_jiffies(CCDETECT_DELAY_MS));
	return IRQ_HANDLED;
}

int oplus_ccdetect_gpio_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return -EINVAL;
	}

	chip->chgic_mtk.oplus_info->pinctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chip->chgic_mtk.oplus_info->pinctrl)) {
		chg_err("get ccdetect_pinctrl fail\n");
		return -EINVAL;
	}

	chip->chgic_mtk.oplus_info->ccdetect_active = pinctrl_lookup_state(chip->chgic_mtk.oplus_info->pinctrl, "ccdetect_active");
	if (IS_ERR_OR_NULL(chip->chgic_mtk.oplus_info->ccdetect_active)) {
		chg_err("get ccdetect_active fail\n");
		return -EINVAL;
	}

	chip->chgic_mtk.oplus_info->ccdetect_sleep = pinctrl_lookup_state(chip->chgic_mtk.oplus_info->pinctrl, "ccdetect_sleep");
	if (IS_ERR_OR_NULL(chip->chgic_mtk.oplus_info->ccdetect_sleep)) {
		chg_err("get ccdetect_sleep fail\n");
		return -EINVAL;
	}
	if (chip->chgic_mtk.oplus_info->ccdetect_gpio > 0) {
		gpio_direction_input(chip->chgic_mtk.oplus_info->ccdetect_gpio);
	}

	pinctrl_select_state(chip->chgic_mtk.oplus_info->pinctrl,  chip->chgic_mtk.oplus_info->ccdetect_active);
	return 0;
}

static void oplus_ccdetect_work(struct work_struct *work)
{
	int level;
	struct oplus_chg_chip *chip = g_oplus_chip;

	level = gpio_get_value(chip->chgic_mtk.oplus_info->ccdetect_gpio);
	if (level != 1) {
		oplus_ccdetect_enable();
	        oplus_wake_up_usbtemp_thread();
	} else {
		if (g_oplus_chip)
			g_oplus_chip->usbtemp_check = oplus_usbtemp_condition();

		if (oplus_get_otg_switch_status() == false)
			oplus_ccdetect_disable();
		if(g_oplus_chip->usb_status == USB_TEMP_HIGH) {
			schedule_delayed_work(&pinfo->usbtemp_recover_work, 0);
		}
	}
}

bool oplus_ccdetect_support_check(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	if (oplus_ccdetect_check_is_gpio(chip) == true)
		return true;

	return false;
}

static void oplus_ccdetect_irq_register(struct oplus_chg_chip *chip)
{
	int ret = 0;

	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return;
	}

	ret = devm_request_threaded_irq(chip->dev,  chip->chgic_mtk.oplus_info->ccdetect_irq,
			NULL, oplus_ccdetect_change_handler, IRQF_TRIGGER_FALLING
			| IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ccdetect-change", chip);
	if (ret < 0) {
		chg_err("Unable to request ccdetect-change irq: %d\n", ret);
	}
	chg_err("!!!!! irq register\n");

	ret = enable_irq_wake(chip->chgic_mtk.oplus_info->ccdetect_irq);
	if (ret != 0) {
		chg_err("enable_irq_wake: ccdetect_irq failed %d\n", ret);
	}
}

static void oplus_ccdetect_enable(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return;
	}

	if (oplus_ccdetect_check_is_gpio(chip) != true
			&& oplus_ccdetect_check_is_wd0(chip) != true)
		return;

	/* set try SNK */
	if (pinfo != NULL && pinfo->tcpc != NULL) {
		tcpm_typec_change_role_postpone(pinfo->tcpc, TYPEC_ROLE_TRY_SNK, true);
		chg_err("set try snk!\n");
	}
}

static void oplus_ccdetect_disable(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return;
	}

	if (oplus_ccdetect_check_is_gpio(chip) != true
			&& oplus_ccdetect_check_is_wd0(chip) != true)
		return;

	/* set SINK mode */
	if (pinfo != NULL && pinfo->tcpc != NULL) {
		tcpm_typec_change_role_postpone(pinfo->tcpc, TYPEC_ROLE_SNK, true);
		chg_err("set sink!\n");
	}
}

int oplus_chg_ccdetect_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;

	if (chip)
		node = chip->dev->of_node;
	if (node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}
	chip->chgic_mtk.oplus_info->ccdetect_gpio = of_get_named_gpio(node, "qcom,ccdetect-gpio", 0);
	if (chip->chgic_mtk.oplus_info->ccdetect_gpio <= 0) {
		chg_err("Couldn't read qcom,ccdetect-gpio rc=%d, qcom,ccdetect-gpio:%d\n",
				rc, chip->chgic_mtk.oplus_info->ccdetect_gpio);
	} else {
		if (oplus_ccdetect_support_check() == true) {
			rc = gpio_request(chip->chgic_mtk.oplus_info->ccdetect_gpio, "ccdetect-gpio");
			if (rc) {
				chg_err("unable to request ccdetect_gpio:%d\n",
						chip->chgic_mtk.oplus_info->ccdetect_gpio);
			} else {
				rc = oplus_ccdetect_gpio_init(chip);
				if (rc) {
					chg_err("unable to init ccdetect_gpio:%d\n",
							chip->chgic_mtk.oplus_info->ccdetect_gpio);
				} else {
					oplus_ccdetect_irq_init(chip);
				}
			}
		}
		chg_err("ccdetect-gpio:%d\n", chip->chgic_mtk.oplus_info->ccdetect_gpio);
	}

	return rc;
}
static int oplus_chg_cclogic_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;
	struct charger_manager_drvdata *info_drvdata = pinfo_drvdata;

	if (chip)
		node = chip->dev->of_node;
	if (node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}

	if(info_drvdata) {
		info_drvdata->external_cclogic = of_property_read_bool(node, "oplus,use_external_cclogic");
		if (info_drvdata->external_cclogic == false) {
			chg_err("not support qcom,use_external_cclogic is false\n");
		}
	}
	return rc;
}


static int oplus_chg_parse_custom_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	if (chip == NULL) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}

	rc = oplus_chg_chargerid_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg_chargerid_parse_dt fail!\n", __func__);
		return -EINVAL;
	}

	rc = oplus_even_c_prj_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_even_c_prj_parse_dt fail!\n", __func__);
		return -EINVAL;
	}

	rc = oplus_chg_shipmode_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg_shipmode_parse_dt fail!\n", __func__);
		return -EINVAL;
	}

	rc = oplus_chg_shortc_hw_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg_shortc_hw_parse_dt fail!\n", __func__);
		return -EINVAL;
	}

	rc = oplus_chg_usbtemp_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg_usbtemp_parse_dt fail!\n", __func__);
		return -EINVAL;
	}

	rc = oplus_chg_ccdetect_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg_ccdetect_parse_dt fail!\n", __func__);
	}

	rc = oplus_chg_cclogic_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg_cclogic_parse_dt fail!\n", __func__);
		return -EINVAL;
	}
	return rc;
}

/*====================================================================*/

/************************************************/
/* Power Supply Functions
*************************************************/
static int mt_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int rc = 0;
	rc = oplus_ac_get_property(psy, psp, val);
	if (rc < 0) {
		val->intval = 0;
	}

	return 0;
}

static int mt_usb_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret = 0;
	switch (psp) {
	default:
		pr_err("writeable prop %d is not supported in usb\n", psp);
		ret = -EINVAL;
		break;
	}
	return 0;
}

static int mt_usb_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int rc = 0;
	switch (psp) {
	default:
		rc = oplus_usb_get_property(psy, psp, val);
		if (rc < 0) {
			val->intval = 0;
		}
	}
	return rc;
}

static int mt_usb_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	int ret = 0;
	switch (psp) {
	default:
		pr_err("set prop %d is not supported in usb\n", psp);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int battery_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc = 0;
	rc = oplus_battery_property_is_writeable(psy, psp);
	return rc;
}

static int battery_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	return oplus_battery_set_property(psy, psp, val);
}

static int battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		if (g_oplus_chip && (g_oplus_chip->ui_soc == 0)) {
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
			pr_err("bat pro POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL, should shutdown!!!\n");
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (g_oplus_chip) {
			val->intval = g_oplus_chip->batt_fcc * 1000;
		}
		break;
	default:
		rc = oplus_battery_get_property(psy, psp, val);
		if (rc < 0) {
			if (psp == POWER_SUPPLY_PROP_TEMP)
				val->intval = DEFAULT_BATTERY_TMP_WHEN_ERROR;
			else
				val->intval = 0;
		}
		break;
	}
	return 0;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static int mt_mastercharger_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret = 0;
	switch (psp) {
	default:
		pr_err("writeable prop %d is not supported in usb\n", psp);
		ret = -EINVAL;
		break;
	}
	return 0;
}

static int mastercharger_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = oplus_chg_get_chargeric_temp_cal();
		pr_err("chargeric_temp_cal : %d", val->intval);
		break;
	default:
		break;
	}
	return 0;
}

static enum power_supply_property mastercharger_properties[] = {
	POWER_SUPPLY_PROP_TEMP,
};
#endif

static enum power_supply_property mt_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mt_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};
static enum power_supply_property battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};


static int oplus_power_supply_init(struct oplus_chg_chip *chip)
{
	int ret = 0;

	struct oplus_chg_chip *mt_chg = NULL;
	mt_chg = chip;
	mt_chg->ac_psd.name = "ac";
	mt_chg->ac_psd.type = POWER_SUPPLY_TYPE_MAINS;
	mt_chg->ac_psd.properties = mt_ac_properties;
	mt_chg->ac_psd.num_properties = ARRAY_SIZE(mt_ac_properties);
	mt_chg->ac_psd.get_property = mt_ac_get_property;
	mt_chg->ac_cfg.drv_data = mt_chg;

	mt_chg->usb_psd.name = "usb";
	mt_chg->usb_psd.type = POWER_SUPPLY_TYPE_USB;
	mt_chg->usb_psd.properties = mt_usb_properties;
	mt_chg->usb_psd.num_properties = ARRAY_SIZE(mt_usb_properties);
	mt_chg->usb_psd.get_property = mt_usb_get_property;
	mt_chg->usb_psd.set_property = mt_usb_set_property;
	mt_chg->usb_psd.property_is_writeable = mt_usb_prop_is_writeable;
	mt_chg->usb_cfg.drv_data = mt_chg;

	mt_chg->battery_psd.name = "battery";
	mt_chg->battery_psd.type = POWER_SUPPLY_TYPE_BATTERY;
	mt_chg->battery_psd.properties = battery_properties;
	mt_chg->battery_psd.num_properties = ARRAY_SIZE(battery_properties);
	mt_chg->battery_psd.get_property = battery_get_property;
	mt_chg->battery_psd.set_property = battery_set_property;
	mt_chg->battery_psd.property_is_writeable = battery_prop_is_writeable,


		mt_chg->ac_psy = power_supply_register(mt_chg->dev, &mt_chg->ac_psd,
				&mt_chg->ac_cfg);
	if (IS_ERR(mt_chg->ac_psy)) {
		dev_err(mt_chg->dev, "Failed to register power supply ac: %ld\n",
				PTR_ERR(mt_chg->ac_psy));
		ret = PTR_ERR(mt_chg->ac_psy);
		goto err_ac_psy;
	}
	mt_chg->usb_psy = power_supply_register(mt_chg->dev, &mt_chg->usb_psd,
			&mt_chg->usb_cfg);
	if (IS_ERR(mt_chg->usb_psy)) {
		dev_err(mt_chg->dev, "Failed to register power supply usb: %ld\n",
				PTR_ERR(mt_chg->usb_psy));
		ret = PTR_ERR(mt_chg->usb_psy);
		goto err_usb_psy;
	}
	mt_chg->batt_psy = power_supply_register(mt_chg->dev, &mt_chg->battery_psd,
			NULL);
	if (IS_ERR(mt_chg->batt_psy)) {
		dev_err(mt_chg->dev, "Failed to register power supply battery: %ld\n",
				PTR_ERR(mt_chg->batt_psy));
		ret = PTR_ERR(mt_chg->batt_psy);
		goto err_battery_psy;
	}

	chg_err("%s OK\n", __func__);
	return 0;

err_usb_psy:
	power_supply_unregister(mt_chg->ac_psy);
err_ac_psy:
	power_supply_unregister(mt_chg->usb_psy);
err_battery_psy:
	power_supply_unregister(mt_chg->batt_psy);

	return ret;
}

#define VBUS_9V	9000
#define VBUS_5V	5000
#define IBUS_2A	2000
#define IBUS_3A	3000
int oplus_pdc_setup(int *vbus_mv, int *ibus_ma)
{
	int ret = 0;
	int vbus_mv_t = 0;
	int ibus_ma_t = 0;
	struct tcpc_device *tcpc = NULL;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		printk(KERN_ERR "%s:get type_c_port0 fail\n", __func__);
		return -EINVAL;
	}

	if (*vbus_mv == VBUS_5V)
		ret = tcpm_set_pd_charging_policy(tcpc, DPM_CHARGING_POLICY_VSAFE5V, NULL);
	else
		ret = tcpm_set_pd_charging_policy(tcpc, DPM_CHARGING_POLICY_MAX_POWER_LVIC, NULL);

	ret = tcpm_dpm_pd_request(tcpc, *vbus_mv, *ibus_ma, NULL);
	if (ret != TCPM_SUCCESS) {
		printk(KERN_ERR "%s: tcpm_dpm_pd_request fail\n", __func__);
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(tcpc, &vbus_mv_t, &ibus_ma_t);
	if (ret != TCPM_SUCCESS) {
		printk(KERN_ERR "%s: inquire current vbus_mv and ibus_ma fail\n", __func__);
		return -EINVAL;
	}

	printk(KERN_ERR "%s: request vbus_mv[%d], ibus_ma[%d]\n", __func__, vbus_mv_t, ibus_ma_t);

	return 0;
}


static int masterchg_power_supply_init(struct master_chg_psy *master_chg)
{
	struct master_chg_psy *mastercharger_psy = NULL;
	int ret = 0;

	mastercharger_psy = master_chg;
	if (!mastercharger_psy) {
		chr_err("register mtk-master-charger error\n");
		return -EINVAL;
	}
	chr_err("register mtk-master-charger\n");
	mastercharger_psy->mastercharger_psy_desc.name = "mtk-master-charger";
	mastercharger_psy->mastercharger_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	mastercharger_psy->mastercharger_psy_desc.properties = mastercharger_properties;
	mastercharger_psy->mastercharger_psy_desc.num_properties = ARRAY_SIZE(mastercharger_properties);
	mastercharger_psy->mastercharger_psy_desc.get_property = mastercharger_get_property;
	mastercharger_psy->mastercharger_psy_desc.property_is_writeable =
		mt_mastercharger_prop_is_writeable;
	mastercharger_psy->mastercharger_psy_cfg.drv_data = mastercharger_psy;
	mastercharger_psy->mastercharger_psy = power_supply_register(mastercharger_psy->dev, &mastercharger_psy->mastercharger_psy_desc,
	&mastercharger_psy->mastercharger_psy_cfg);
	if (IS_ERR(mastercharger_psy->mastercharger_psy)) {
		chr_err("register mastercharger_psy fail\n");
		ret = PTR_ERR(mastercharger_psy->mastercharger_psy);
		goto err_mastercharger_psy;
	}
	return 0;
err_mastercharger_psy:
	power_supply_unregister(mastercharger_psy->mastercharger_psy);

	return ret;
}
static int oplus_chg_parse_charger_dt_2nd_override(struct oplus_chg_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,iterm_ma_2nd", &chip->limits.iterm_ma);
	if (rc < 0) {
		chip->limits.iterm_ma = 300;
	}

	rc = of_property_read_u32(node, "qcom,recharge-mv_2nd", &chip->limits.recharge_mv);
	if (rc < 0) {
		chip->limits.recharge_mv = 121;
	}

	rc = of_property_read_u32(node, "qcom,temp_little_cold_vfloat_mv_2nd",
			&chip->limits.temp_little_cold_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_little_cold_vfloat_mv = 4391;
	}

	rc = of_property_read_u32(node, "qcom,temp_cool_vfloat_mv_2nd",
			&chip->limits.temp_cool_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_cool_vfloat_mv = 4391;
	}

	rc = of_property_read_u32(node, "qcom,temp_little_cool_vfloat_mv_2nd",
			&chip->limits.temp_little_cool_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_little_cool_vfloat_mv = 4391;
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_vfloat_mv_2nd",
			&chip->limits.temp_normal_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_normal_vfloat_mv = 4391;
	}

	rc = of_property_read_u32(node, "qcom,little_cold_vfloat_over_sw_limit_2nd",
			&chip->limits.little_cold_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.little_cold_vfloat_over_sw_limit = 4395;
	}

	rc = of_property_read_u32(node, "qcom,cool_vfloat_over_sw_limit_2nd",
			&chip->limits.cool_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.cool_vfloat_over_sw_limit = 4395;
	}

	rc = of_property_read_u32(node, "qcom,little_cool_vfloat_over_sw_limit_2nd",
			&chip->limits.little_cool_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.little_cool_vfloat_over_sw_limit = 4395;
	}

	rc = of_property_read_u32(node, "qcom,normal_vfloat_over_sw_limit_2nd",
			&chip->limits.normal_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.normal_vfloat_over_sw_limit = 4395;
	}

	rc = of_property_read_u32(node, "qcom,default_iterm_ma_2nd",
			&chip->limits.default_iterm_ma);
	if (rc < 0) {
		chip->limits.default_iterm_ma = 300;
	}

	rc = of_property_read_u32(node, "qcom,default_temp_normal_vfloat_mv_2nd",
			&chip->limits.default_temp_normal_vfloat_mv);
	if (rc < 0) {
		chip->limits.default_temp_normal_vfloat_mv = 4391;
	}

	rc = of_property_read_u32(node, "qcom,default_normal_vfloat_over_sw_limit_2nd",
			&chip->limits.default_normal_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.default_normal_vfloat_over_sw_limit = 4395;
	}

	chg_err("iterm_ma = %d, recharge_mv = %d, temp_little_cold_vfloat_mv = %d, \
		temp_cool_vfloat_mv = %d, temp_little_cool_vfloat_mv = %d, \
		temp_normal_vfloat_mv = %d, little_cold_vfloat_over_sw_limit = %d, \
		cool_vfloat_over_sw_limit = %d, little_cool_vfloat_over_sw_limit = %d, \
		normal_vfloat_over_sw_limit = %d, default_iterm_ma = %d, \
		default_temp_normal_vfloat_mv = %d, default_normal_vfloat_over_sw_limit = %d\n",
		chip->limits.iterm_ma, chip->limits.recharge_mv, chip->limits.temp_little_cold_vfloat_mv,
		chip->limits.temp_cool_vfloat_mv, chip->limits.temp_little_cool_vfloat_mv,
		chip->limits.temp_normal_vfloat_mv, chip->limits.little_cold_vfloat_over_sw_limit,
		chip->limits.cool_vfloat_over_sw_limit, chip->limits.little_cool_vfloat_over_sw_limit,
		chip->limits.normal_vfloat_over_sw_limit, chip->limits.default_iterm_ma,
		chip->limits.default_temp_normal_vfloat_mv, chip->limits.default_normal_vfloat_over_sw_limit);

	return rc;
}

/*====================================================================*/
static ssize_t show_StopCharging_Test(struct device *dev, struct device_attribute *attr, char *buf)
{
	g_oplus_chip->stop_chg = false;
	oplus_chg_turn_off_charging(g_oplus_chip);
	printk("StopCharging_Test\n");
	return sprintf(buf, "chr=%d\n", g_oplus_chip->stop_chg);
}

static ssize_t store_StopCharging_Test(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return -1;
}
static DEVICE_ATTR(StopCharging_Test, 0664, show_StopCharging_Test, store_StopCharging_Test);

static ssize_t show_StartCharging_Test(struct device *dev, struct device_attribute *attr, char *buf)
{
	g_oplus_chip->stop_chg = true;
	oplus_chg_turn_on_charging(g_oplus_chip);
	printk("StartCharging_Test\n");
	return sprintf(buf, "chr=%d\n", g_oplus_chip->stop_chg);
}
static ssize_t store_StartCharging_Test(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return -1;
}
static DEVICE_ATTR(StartCharging_Test, 0664, show_StartCharging_Test, store_StartCharging_Test);

void oplus_chg_default_method0(void)
{
	pr_err("charger ic default %d\n", charger_ic__det_flag);
}

int oplus_chg_default_method1(void)
{
	return 0;
}

int oplus_chg_default_method2(int n)
{
	return 0;
}

void oplus_chg_default_method3(int n)
{
	return;
}

bool oplus_chg_default_method4(void)
{
	return false;
}

/* To do to call right function call in the caller */
unsigned int mt6358_upmu_set_rg_vs1_fpwm(unsigned val)
{
	return 0;
}

void oplus_chg_set_fix_mode(bool en)
{
	int ret = -1;

	ret = mt6358_upmu_set_rg_vs1_fpwm(en);
	if (ret < 0)
		pr_err("DCDC to fix mode failed, ret:%d", ret);
	else
		pr_err("DCDC to %s mode successful", (en ? "Fix": "Auto"));
}

#ifdef OPLUS_FEATURE_CHG_BASIC
int oplus_chg_set_pps_config(int vbus_mv, int ibus_ma)
{
	int ret = 0;
	int vbus_mv_t = 0;
	int ibus_ma_t = 0;
	struct tcpc_device *tcpc = NULL;

	chg_err("request vbus_mv[%d], ibus_ma[%d]", vbus_mv, ibus_ma);

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail");
		return -EINVAL;
	}

	if (tcpc->pd_port.dpm_charging_policy != DPM_CHARGING_POLICY_PPS) {
		ret = tcpm_set_apdo_charging_policy(tcpc, DPM_CHARGING_POLICY_PPS, vbus_mv, ibus_ma, NULL);
		if (ret == TCP_DPM_RET_REJECT) {
			chg_err("set_apdo_charging_policy reject");
			return 0;
		} else if (ret != 0) {
			chg_err("set_apdo_charging_policy error %d", ret);
			return MTK_ADAPTER_ERROR;
		}
	}

	ret = tcpm_dpm_pd_request(tcpc, vbus_mv, ibus_ma, NULL);
	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_dpm_pd_request fail");
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(tcpc, &vbus_mv_t, &ibus_ma_t);
	if (ret != TCPM_SUCCESS) {
		chg_err("inquire current vbus_mv and ibus_ma fail");
		return -EINVAL;
	}

	chg_err("inquire_pd_contract vbus_mv_t[%d], ibus_ma_t[%d]", vbus_mv_t, ibus_ma_t);

	return ret;
}

#define OPLUS_GET_PPS_STATUS_ERR 0xFFFFFF
u32 oplus_chg_get_pps_status(void)
{
	int tcpm_ret = TCPM_SUCCESS;
	struct pd_pps_status pps_status;
	struct tcpc_device *tcpc = NULL;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail");
		return OPLUS_GET_PPS_STATUS_ERR;
	}

	tcpm_ret = tcpm_dpm_pd_get_pps_status(tcpc, NULL, &pps_status);
	if (tcpm_ret == TCP_DPM_RET_NOT_SUPPORT)
		return OPLUS_GET_PPS_STATUS_ERR;
	else if (tcpm_ret != 0)
		return OPLUS_GET_PPS_STATUS_ERR;

	return (PD_PPS_SET_OUTPUT_MV(pps_status.output_mv) | PD_PPS_SET_OUTPUT_MA(pps_status.output_ma) << 16);
}

__attribute__((unused)) static void oplus_chg_pps_get_source_cap(struct mtk_charger *info)
{
	struct tcpm_power_cap_val apdo_cap;
	struct pd_source_cap_ext cap_ext;
	struct oplus_chg_chip *chip = g_oplus_chip;
	uint8_t cap_i = 0;
	int ret = 0;
	int idx = 0;
	unsigned int i = 0;

	if (!chip) {
		pr_err("oplus_chip is null!\n");
		return;
	}

	if (info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
		while (1) {
			ret = tcpm_inquire_pd_source_apdo(info->tcpc,
					TCPM_POWER_CAP_APDO_TYPE_PPS,
					&cap_i, &apdo_cap);
			if (ret == TCPM_ERROR_NOT_FOUND) {
				break;
			} else if (ret != TCPM_SUCCESS) {
				chg_err("tcpm_inquire_pd_source_apdo failed(%d)\n", ret);
				break;
			}
			if (get_charger_ic_det(chip) != (1 << SC6607)) {
				ret = tcpm_dpm_pd_get_source_cap_ext(info->tcpc,
					NULL, &cap_ext);
				if (ret == TCPM_SUCCESS)
					info->srccap.pdp = cap_ext.source_pdp;
				else {
					info->srccap.pdp = 0;
					chg_err("tcpm_dpm_pd_get_source_cap_ext failed(%d)\n", ret);
				}
			}

			info->srccap.pwr_limit[idx] = apdo_cap.pwr_limit;
			/* If TA has PDP, we set pwr_limit as true */
			if (info->srccap.pdp > 0 && !info->srccap.pwr_limit[idx])
				info->srccap.pwr_limit[idx] = 1;
			info->srccap.ma[idx] = apdo_cap.ma;
			info->srccap.max_mv[idx] = apdo_cap.max_mv;
			info->srccap.min_mv[idx] = apdo_cap.min_mv;
			info->srccap.maxwatt[idx] = apdo_cap.max_mv * apdo_cap.ma;
			info->srccap.minwatt[idx] = apdo_cap.min_mv * apdo_cap.ma;
			info->srccap.type[idx] = MTK_PD_APDO;

			idx++;

			chg_info("pps_boundary[%d], %d mv ~ %d mv, %d ma pl:%d\n",
				cap_i,
				apdo_cap.min_mv, apdo_cap.max_mv,
				apdo_cap.ma, apdo_cap.pwr_limit);
			if (idx >= ADAPTER_CAP_MAX_NR) {
				chg_info("CAP NR > %d\n", ADAPTER_CAP_MAX_NR);
				break;
			}
		}

		info->srccap.nr = idx;

		for (i = 0; i < info->srccap.nr; i++) {
			chg_info("pps_cap[%d:%d], %d mv ~ %d mv, %d ma pl:%d pdp:%d\n",
				i, (int)info->srccap.nr, info->srccap.min_mv[i],
				info->srccap.max_mv[i], info->srccap.ma[i],
				info->srccap.pwr_limit[i], info->srccap.pdp);
		}

		if (cap_i == 0 || info->srccap.nr == 0)
			chg_info("no APDO for pps\n");
		else
			oplus_pps_set_power(OPLUS_PPS_POWER_THIRD,
				info->srccap.ma[info->srccap.nr - 1],
				info->srccap.max_mv[info->srccap.nr - 1]);
	}

	return;
}

int oplus_chg_pps_get_max_cur(int vbus_mv)
{
	unsigned int i = 0;
	int ibus_ma = 0;

	if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
		for (i = 0; i < pinfo->srccap.nr; i++) {
			if (pinfo->srccap.min_mv[i] <= vbus_mv && vbus_mv <= pinfo->srccap.max_mv[i]) {
				if (ibus_ma < pinfo->srccap.ma[i]) {
					ibus_ma = pinfo->srccap.ma[i];
				}
			}
		}
	}

	chg_info("oplus_chg_pps_get_max_cur ibus_ma: %d\n", ibus_ma);

	if (ibus_ma > 0)
		return ibus_ma;
	else
		return -EINVAL;
}

int oplus_chg_pps_get_max_volt(void)
{
	unsigned int i = 0;
	int vbus_mv = 0;

	if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
		for (i = 0; i < pinfo->srccap.nr; i++) {
			if (vbus_mv < pinfo->srccap.max_mv[i]) {
				vbus_mv = pinfo->srccap.max_mv[i];
			}
		}
	}

	chg_info("oplus_chg_pps_get_max_volt vbus_mv: %d\n", vbus_mv);

	if (vbus_mv > 0)
		return vbus_mv;
	else
		return -EINVAL;
}

bool oplus_mt_get_vbus_status(void)
{
	return false;
}
EXPORT_SYMBOL(oplus_mt_get_vbus_status);

int oplus_force_get_subboard_temp(void)
{
	return 0;
}
EXPORT_SYMBOL(oplus_force_get_subboard_temp);

int oplus_chg_check_ui_soc_is_ready(void)
{
	return 0;
}
EXPORT_SYMBOL(oplus_chg_check_ui_soc_is_ready);

int oplus_pps_pd_exit(void)
{
	int ret = -1;
	int vbus_mv_t = OPLUS_MIN_PDO_VOL;
	int ibus_ma_t = OPLUS_MIN_PDO_CUR;

	struct oplus_chg_chip *chip = g_oplus_chip;
	struct tcpc_device *tcpc = NULL;

	if (chip == NULL) {
		chg_err("no oplus_chg_chip");
		return -ENODEV;
	}

	/*
	if (oplus_mt_get_vbus_status() == false)
		return ret;
	*/
	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail");
		return -EINVAL;
	}

	ret = tcpm_set_pd_charging_policy(tcpc, DPM_CHARGING_POLICY_VSAFE5V, NULL);

	ret = tcpm_dpm_pd_request(tcpc, vbus_mv_t, ibus_ma_t, NULL);
	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_dpm_pd_request fail");
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(tcpc, &vbus_mv_t, &ibus_ma_t);
	if (ret != TCPM_SUCCESS) {
		chg_err("inquire current vbus_mv and ibus_ma fail");
		return -EINVAL;
	}

	msleep(100);
	chg_info("PD Default vbus_mv[%d], ibus_ma[%d]\n", vbus_mv_t, ibus_ma_t);

	return ret;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */


void register_mtk_oplus_batt_interfaces(struct mtk_oplus_batt_interface *intf)
{
	g_oplus_batt_intf = intf;
}
EXPORT_SYMBOL(register_mtk_oplus_batt_interfaces);

bool oplus_chg_set_charge_power_sel(int index)
{
	if (g_oplus_batt_intf && g_oplus_batt_intf->set_charge_power_sel) {
		return g_oplus_batt_intf->set_charge_power_sel(index);
	} else {
		pr_err("%s - g_oplus_batt_intf or set_charge_power_sel is NULL", __func__);
	}

	return 0;
}

void oplus_chg_choose_gauge_curve(int index_curve)
{
	static int last_curve_index = -1;
	int target_index_curve = -1;

	if (!pinfo)
		target_index_curve = CHARGER_NORMAL_CHG_CURVE;

	if (index_curve == CHARGER_SUBTYPE_QC
			|| index_curve == CHARGER_SUBTYPE_PD
			|| index_curve == CHARGER_SUBTYPE_FASTCHG_VOOC) {
		target_index_curve = CHARGER_FASTCHG_VOOC_AND_QCPD_CURVE;
	} else if (index_curve == 0) {
		target_index_curve = CHARGER_NORMAL_CHG_CURVE;
	} else {
		target_index_curve = CHARGER_FASTCHG_SVOOC_CURVE;
	}

	printk(KERN_ERR "%s: index_curve() =%d  target_index_curve =%d last_curve_index =%d",
		__func__, index_curve, target_index_curve, last_curve_index);

	if (target_index_curve != last_curve_index) {
		oplus_chg_set_charge_power_sel(target_index_curve);
		last_curve_index = target_index_curve;
	}
	return;
}

void oplus_usbtemp_recover_cc_open(void)
{
	if (pinfo == NULL || pinfo->tcpc == NULL)
		return;

	tcpm_typec_disable_function(pinfo->tcpc, false);
}

bool oplus_chg_check_qchv_condition(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		pr_err("oplus_chip is null\n");
		return false;
	}

	chip->charger_volt = chip->chg_ops->get_charger_volt();
	if (chip->dual_charger_support && chip->soc < QC_SOC_HIGH &&
	    chip->charger_volt < QC_CHARGER_VOLTAGE_HIGH &&
	    chip->temperature <= QC_TEMP_HIGH &&
	    !chip->cool_down_force_5v)
		return true;

	return false;
}


void oplus_set_typec_sinkonly(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct charger_manager_drvdata *info_drvdata = pinfo_drvdata;

	if (!chip || !pinfo || !pinfo->tcpc) {
		return;
	}
	if (info_drvdata != NULL && info_drvdata->external_cclogic) {
		/*sgm7220_set_typec_sinkonly();*/
	} else {
		tcpm_typec_disable_function(pinfo->tcpc, false);
		tcpm_typec_change_role(pinfo->tcpc, TYPEC_ROLE_SNK);
	}
}


void oplus_set_typec_cc_open(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	struct charger_manager_drvdata *info_drvdata = pinfo_drvdata;

	if (!chip || !pinfo || !pinfo->tcpc) {
		return;
	}
	if (info_drvdata != NULL && info_drvdata->external_cclogic) {
		/*sgm7220_set_typec_cc_open();*/
	} else {
		tcpm_typec_disable_function(pinfo->tcpc, true);
	}
}

void oplus_mt_power_off(void) /* to-do */
{
	struct tcpc_device *tcpc_dev = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (g_oplus_chip->ac_online != true) {
		if (tcpc_dev == NULL) {
			tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
		}
		if (tcpc_dev) {
			if (!(tcpc_dev->pd_wait_hard_reset_complete)) {
					/*&& !oplus_mt_get_vbus_status()) {*/  /* to-do */
				kernel_power_off();
			}
		}
	} else {
		printk(KERN_ERR "[OPLUS_CHG][%s]: ac_online is true, return!\n", __func__);
	}
}

struct oplus_chg_operations  oplus_chg_default_ops = {
	.dump_registers = oplus_chg_default_method0,
	.kick_wdt = oplus_chg_default_method1,
	.hardware_init = oplus_chg_default_method1,
	.charging_current_write_fast = oplus_chg_default_method2,
	.set_aicl_point = oplus_chg_default_method3,
	.input_current_write = oplus_chg_default_method2,
	.float_voltage_write = oplus_chg_default_method2,
	.term_current_set = oplus_chg_default_method2,
	.charging_enable = oplus_chg_default_method1,
	.charging_disable = oplus_chg_default_method1,
	.get_charging_enable = oplus_chg_default_method1,
	.charger_suspend = oplus_chg_default_method1,
	.charger_unsuspend = oplus_chg_default_method1,
	.set_rechg_vol = oplus_chg_default_method2,
	.reset_charger = oplus_chg_default_method1,
	.read_full = oplus_chg_default_method1,
	.otg_enable = oplus_chg_default_method1,
	.otg_disable = oplus_chg_default_method1,
	.set_charging_term_disable = oplus_chg_default_method1,
	.check_charger_resume = oplus_chg_default_method4,

	.get_charger_type = oplus_chg_default_method1,
	.get_charger_volt = oplus_chg_default_method1,
	/*int (*get_charger_current)(void);*/
	.get_chargerid_volt = NULL,
	.set_chargerid_switch_val = oplus_chg_default_method3,
	.get_chargerid_switch_val = oplus_chg_default_method1,
	.check_chrdet_status = (bool(*) (void))pmic_chrdet_status,

	.get_boot_mode = (int (*)(void))get_boot_mode,
	.get_boot_reason = (int (*)(void))get_boot_reason,
	.get_instant_vbatt = oplus_battery_meter_get_battery_voltage,
	.get_rtc_soc = oplus_get_rtc_ui_soc,
	.set_rtc_soc = oplus_set_rtc_ui_soc,
	.set_power_off = oplus_mt_power_off,
	.usb_connect = mt_usb_connect_v1,
	.usb_disconnect = mt_usb_disconnect_v1,
	.get_chg_current_step = oplus_chg_default_method1,
	.need_to_check_ibatt = oplus_chg_default_method4,
	.get_dyna_aicl_result = oplus_chg_default_method1,
	.get_shortc_hw_gpio_status = oplus_chg_default_method4,
	/*void (*check_is_iindpm_mode) (void);*/
	.oplus_chg_get_pd_type = NULL,
	.oplus_chg_pd_setup = NULL,
	.get_charger_subtype = oplus_chg_default_method1,
	.set_qc_config = NULL,
	.enable_qc_detect = NULL,
	.oplus_chg_set_high_vbus = NULL,
	.get_platform_gauge_curve = oplus_chg_choose_gauge_curve,
};



int chg_alg_event(struct notifier_block *notifier,
			unsigned long event, void *data)
{
	chr_err("%s: evt:%lu\n", __func__, event);

	return NOTIFY_DONE;
}

static int pd_tcp_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	bool vbus_on;
	struct oplus_pps_chip *pps_chip;

	pr_err("%s: event:%lu", __func__, event);

	switch (event) {
	case TCP_NOTIFY_SOURCE_VBUS:
		pr_info("source vbus = %dmv\n",
				noti->vbus_state.mv);
		vbus_on = (noti->vbus_state.mv) ? true : false;
		set_chr_enable_otg(vbus_on);
		break;

	case TCP_NOTIFY_PD_STATE:
		chr_err("%s: PD state received:%d\n", __func__, noti->pd_state.connected);
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			mutex_lock(&pinfo->pd_lock);
			chr_err("PD Notify Detach\n");
			pinfo->pd_type = pd_connect_tbl[PD_CONNECT_NONE];
			mutex_unlock(&pinfo->pd_lock);
			pinfo->in_good_connect = false;
			pps_chip = oplus_pps_get_pps_chip();
			if (pps_chip) {
				pps_chip->adapter_info.svid = 0;
				pps_chip->adapter_info.pid = 0;
				pps_chip->adapter_info.bcd = 0;
				pps_chip->adapter_info.nr = 0;
			}
			chr_err("PD_CONNECT_NONE in_good_connect false\n");
			/* reset PE40 */
			break;

		case PD_CONNECT_HARD_RESET:
			mutex_lock(&pinfo->pd_lock);
			chr_err("PD Notify HardReset\n");
			pinfo->pd_type = pd_connect_tbl[PD_CONNECT_NONE];
			pinfo->pd_reset = true;
			mutex_unlock(&pinfo->pd_lock);
			_wake_up_charger(pinfo);
			pinfo->in_good_connect = false;
			chr_err("PD_CONNECT_HARD_RESET in_good_connect false\n");
			/* reset PE40 */
			break;

		case PD_CONNECT_PE_READY_SNK:
			mutex_lock(&pinfo->pd_lock);
			chr_err("PD Notify fixe voltage ready\n");
			pinfo->pd_type = pd_connect_tbl[PD_CONNECT_PE_READY_SNK];
			mutex_unlock(&pinfo->pd_lock);
			oplus_chg_track_record_chg_type_info();
			pinfo->in_good_connect = true;
			oplus_get_adapter_svid();
			chr_err("PD_CONNECT_PE_READY_SNK_PD30 in_good_connect true\n");
			/* PD is ready */
			break;

		case PD_CONNECT_PE_READY_SNK_PD30:
			mutex_lock(&pinfo->pd_lock);
			chr_err("PD Notify PD30 ready\r\n");
			pinfo->pd_type = pd_connect_tbl[PD_CONNECT_PE_READY_SNK_PD30];
			mutex_unlock(&pinfo->pd_lock);
			oplus_chg_track_record_chg_type_info();
			pinfo->in_good_connect = true;
			oplus_get_adapter_svid();
			chr_err("PD_CONNECT_PE_READY_SNK_PD30 in_good_connect true\n");
			/* PD30 is ready */
			break;

		case PD_CONNECT_PE_READY_SNK_APDO:
			mutex_lock(&pinfo->pd_lock);
			chr_err("PD Notify APDO Ready\n");
			pinfo->pd_type = pd_connect_tbl[PD_CONNECT_PE_READY_SNK_APDO];
			mutex_unlock(&pinfo->pd_lock);
			/* PE40 is ready */
			_wake_up_charger(pinfo);
			oplus_chg_track_record_chg_type_info();
			pinfo->in_good_connect = true;
			oplus_get_adapter_svid();
			chr_err("PD_CONNECT_PE_READY_SNK_APDO in_good_connect true\n");
			oplus_chg_pps_get_source_cap(pinfo);
			oplus_chg_wake_update_work();
			break;

		case PD_CONNECT_TYPEC_ONLY_SNK:
			mutex_lock(&pinfo->pd_lock);
			chr_err("PD Notify Type-C Ready\n");
			pinfo->pd_type = pd_connect_tbl[PD_CONNECT_TYPEC_ONLY_SNK];
			mutex_unlock(&pinfo->pd_lock);
			/* type C is ready */
			_wake_up_charger(pinfo);
			break;
		}
		break;
	case TCP_NOTIFY_WD_STATUS:
		chg_err("wd status = %d\n", (bool)noti->wd_status.water_detected);
		pinfo->water_detected = (bool)noti->wd_status.water_detected;
		if (pinfo->water_detected == true)
			pinfo->notify_code |= CHG_TYPEC_WD_STATUS;
		else
			pinfo->notify_code &= ~CHG_TYPEC_WD_STATUS;
		mtk_chgstat_notify(pinfo);
		break;
	case TCP_NOTIFY_WD0_STATE:
		pinfo->wd0_detect = noti->wd0_state.wd0;
		chg_err("wd0 = %d\n", noti->wd0_state.wd0);

		if (pinfo->wd0_detect) {
			oplus_ccdetect_enable();
		} else {
			if (oplus_get_otg_switch_status() == false)
				oplus_ccdetect_disable();
		}

		if (pinfo->wd0_detect_work.work.func) {
			schedule_delayed_work(&pinfo->wd0_detect_work, msecs_to_jiffies(CCDETECT_DELAY_MS));
		} else {
			chg_err("&pinfo->wd0_detect_work.work.func is NULL\n");
		}

		if (g_oplus_chip && g_oplus_chip->usb_psy) {
			power_supply_changed(g_oplus_chip->usb_psy);
			if (pinfo->wd0_detect == 0) {
				oplus_usbtemp_recover_cc_open();
			}
		}
		break;
	case TCP_NOTIFY_CHRDET_STATE:
		pinfo->chrdet_state = noti->chrdet_state.chrdet;
		chg_err("chrdet = %d\n", noti->chrdet_state.chrdet);

		if (pinfo->chrdet_state) {
			oplus_chg_wake_update_work();
			oplus_wake_up_usbtemp_thread();
			oplus_chg_track_check_wired_charging_break(OPLUS_TRACK_CHG_PLUGIN);
		} else {
			oplus_chg_set_charger_type_unknown();
			oplus_chg_track_check_wired_charging_break(OPLUS_TRACK_CHG_PLUGOUT);
		}
		break;
	case TCP_NOTIFY_HARD_RESET_STATE:
		switch (noti->hreset_state.state) {
		case TCP_HRESET_SIGNAL_SEND:
		case TCP_HRESET_SIGNAL_RECV:
			pinfo->wait_hard_reset_complete = true;
			break;
		default:
			pinfo->wait_hard_reset_complete = false;
			break;
		}
		chg_err("pd_wait_hard_reset_complete: %d\n", pinfo->wait_hard_reset_complete);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static int oplus_charger_probe(struct platform_device *pdev)
{
	struct mtk_charger *info = NULL;
	struct charger_manager_drvdata *info_drvdata = NULL;
	struct list_head *pos;
	struct list_head *phead = &consumer_head;
	struct charger_consumer *ptr;
	int ret = 0;
	struct oplus_chg_chip *oplus_chip = NULL;
	struct master_chg_psy *masterchg_psy = NULL;
	int rc = 0;
	int level = 0;

#ifdef OPLUS_FEATURE_CHG_BASIC
/* add for charger_wakelock */
	char *name = NULL;
#endif
	chr_err("%s: starts\n", __func__);
	oplus_chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_chg_chip), GFP_KERNEL);
	if (!oplus_chip) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}

	masterchg_psy = devm_kzalloc(&pdev->dev, sizeof(struct master_chg_psy), GFP_KERNEL);
	if (!masterchg_psy) {
		chg_err(" masterchg_psy kzalloc() failed\n");
		return -ENOMEM;
	}
	oplus_chip->dev = &pdev->dev;
	masterchg_psy->dev = &pdev->dev;
	ret = oplus_chg_parse_svooc_dt(oplus_chip);
	if (oplus_chip->vbatt_num == 1) {
		if (oplus_gauge_check_chip_is_null()) {
			chg_err("gauge chip null, will do after bettery init.\n");
			return -EPROBE_DEFER;
		}
		oplus_chip->is_double_charger_support = 0;

		charger_ic__det_flag = get_charger_ic_det(oplus_chip);
		if (charger_ic__det_flag == 0) {
			chg_err("charger IC is null, will do after bettery init.\n");
			return -EPROBE_DEFER;
		}

		if(oplus_chip->is_double_charger_support) {
			switch(charger_ic__det_flag) {
			case ((1 << BQ2591X) | (1 << BQ2589X)):
				/*oplus_chip->chg_ops = &oplus_chg_bq2589x_ops;
				oplus_chip->sub_chg_ops = &oplus_chg_bq2591x_ops; */
				chg_err("BQ2591X ,BQ2589X charger IC currently not supported\n");
				break;
			case ((1 << BQ2589X) | (1 << RT9471D)):
				/*oplus_chip->chg_ops = &oplus_chg_bq2589x_ops;
				oplus_chip->sub_chg_ops = &oplus_chg_rt9471_ops;*/
				chg_err("BQ2589X ,RT9471D charger IC currently not supported\n");
				break;
			case ((1 << SY6974) | (1 << BQ2589X)):
				/*oplus_chip->chg_ops = &oplus_chg_bq2589x_ops;
				oplus_chip->sub_chg_ops = &oplus_chg_sy6974_ops;*/
				chg_err("SY6974 ,BQ2589X charger IC currently not supported\n");
				break;
			default:
				oplus_chip->chg_ops = &oplus_chg_default_ops;
				oplus_chip->sub_chg_ops = &oplus_chg_default_ops;
			}
		} else {
			switch(charger_ic__det_flag) {
			case (1 << RT9471D):
				/*oplus_chip->chg_ops = &oplus_chg_rt9471_ops;*/
				chg_err("RT9471D charger IC currently not supported\n");
				break;
			case (1 << SGM41512):
				oplus_chip->chg_ops = &oplus_chg_sgm41512_ops;
				chg_err("SGM41512 charger IC currently not supported\n");
				break;
#ifdef CONFIG_CHARGER_SC6607
			case (1 << SC6607):
				oplus_chip->chg_ops = &oplus_chg_sc6607_ops;
				chg_err("charger IC sc6607 successful\n");
				break;
#endif /* CONFIG_CHARGER_SC6607 */
			default:
				oplus_chip->chg_ops = &oplus_chg_default_ops;
			}
		}
	} else {
		if (oplus_gauge_ic_chip_is_null() || oplus_vooc_check_chip_is_null()
					|| oplus_adapter_check_chip_is_null()) {
			chg_err("[oplus_chg_init] vooc || gauge || chg not ready, will do after bettery init.\n");
			return -EPROBE_DEFER;
		}
	}

	/*move from oplus_mtk_charger.c begin*/
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info_drvdata = devm_kzalloc(&pdev->dev, sizeof(*info_drvdata), GFP_KERNEL);
	if (!info_drvdata)
		return -ENOMEM;
	pinfo = info;
	info_drvdata->pinfo = info;
	pinfo_drvdata = info_drvdata;
	oplus_chip->chgic_mtk.oplus_info = info;
	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

	info->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!info->tcpc) {
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

	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev) {
		chr_err("found primary charger [%s]\n",
			info->chg1_dev->props.alias_name);
	} else {
		chr_err("can't find primary charger!\n");
	}

	mtk_charger_parse_dt(info, &pdev->dev);

	mutex_init(&info->charger_lock);
	mutex_init(&info->pd_lock);
	mutex_init(&info->cable_out_lock);
	/*atomic_set(&info->enable_kpoc_shdn, 1);*/
	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s",
		"charger suspend wakelock");
	info->charger_wakelock = wakeup_source_register(NULL, name);

	spin_lock_init(&info->slock);

	/* init thread */
	init_waitqueue_head(&info->wait_que);
	info->polling_interval = CHARGING_INTERVAL;

	info->chg_data[CHG1_SETTING].thermal_charging_current_limit = -1;
	info->chg_data[CHG1_SETTING].thermal_input_current_limit = -1;
	info->chg_data[CHG1_SETTING].input_current_limit_by_aicl = -1;
	info->chg_data[CHG2_SETTING].thermal_charging_current_limit = -1;
	info->chg_data[CHG2_SETTING].thermal_input_current_limit = -1;

	info->sw_jeita.error_recovery_flag = true;

	mtk_charger_init_timer(info);

	if (info->chg1_dev != NULL && info->algo.do_event != NULL) {
		info->chg1_nb.notifier_call = info->algo.do_event;
		register_charger_device_notifier(info->chg1_dev,
						&info->chg1_nb);
		charger_dev_set_drvdata(info->chg1_dev, info);
	}

	srcu_init_notifier_head(&info->evt_nh);
	ret = mtk_charger_setup_files(pdev);
	if (ret)
		chr_err("Error creating sysfs interface\n");

	info->pd_adapter = get_adapter_by_name("pd_adapter");
	if (info->pd_adapter)
		chr_err("Found PD adapter [%s]\n",
			info->pd_adapter->props.alias_name);
	else
		chr_err("*** Error : can't find PD adapter ***\n");

	oplus_chg_configfs_init(oplus_chip);

	mtk_pdc_init(info);

	charger_ftm_init();
	mtk_charger_get_atm_mode(info);

#ifdef CONFIG_MTK_CHARGER_UNLIMITED
	info->usb_unlimited = true;
	info->enable_sw_safety_timer = false;
	charger_dev_enable_safety_timer(info->chg1_dev, false);
#endif

	charger_debug_init();

	mutex_lock(&consumer_mutex);
	list_for_each(pos, phead) {
		ptr = container_of(pos, struct charger_consumer, list);
		ptr->cm = info;
		if (ptr->pnb != NULL) {
			srcu_notifier_chain_register(&info->evt_nh, ptr->pnb);
			ptr->pnb = NULL;
		}
	}
	mutex_unlock(&consumer_mutex);
	/*info->chg1_consumer =
		charger_manager_get_by_name(&pdev->dev, "charger_port1");
	info->init_done = true;*/
	_wake_up_charger(info);
	/*move from oplus_mtk_charger.c end*/

	ret = oplus_power_supply_init(oplus_chip);
	if (of_property_read_bool(pdev->dev.of_node, "qcom,mastercharger_psy_set")) {
		ret = masterchg_power_supply_init(masterchg_psy);
	}

	printk("oplus_charger_probe end %p, prev %p, next %p\n",
			&oplus_chip->batt_psy->dev.power.wakeup->entry,
			oplus_chip->batt_psy->dev.power.wakeup->entry.prev,
			oplus_chip->batt_psy->dev.power.wakeup->entry.next);

	g_oplus_chip = oplus_chip;
	oplus_chip->chg_ops->hardware_init();
	/*keep "Ibus < 200mA" for 1s, so vooc/svooc adapter go into idle and release D+*/
	if (oplus_which_charger_ic() == 1) {/*for bq25890h*/
		oplus_chip->chg_ops->input_current_write(100);
	}
	oplus_chip->authenticate = oplus_gauge_get_batt_authenticate();
	oplus_chg_parse_custom_dt(oplus_chip);
	oplus_chg_parse_charger_dt(oplus_chip);
	if (is_even_c_project()) {
		gpio_direction_output(oplus_chip->normalchg_gpio.chargerid_switch_gpio, 1);/* even-c */
	}
	if (oplus_which_charger_ic() == 2) {/*for bq25601d*/
		oplus_chg_parse_charger_dt_2nd_override(oplus_chip);
	}
	oplus_chg_init(oplus_chip);
	device_init_wakeup(&pdev->dev, 1);
	oplus_chg_wake_update_work();
	oplus_tbatt_power_off_task_init(oplus_chip);

	if (oplus_ccdetect_support_check() == true) {
		INIT_DELAYED_WORK(&pinfo->ccdetect_work, oplus_ccdetect_work);
		INIT_DELAYED_WORK(&pinfo->usbtemp_recover_work, oplus_usbtemp_recover_work);
		oplus_ccdetect_irq_register(oplus_chip);
		level = gpio_get_value(oplus_chip->chgic_mtk.oplus_info->ccdetect_gpio);
		usleep_range(2000, 2100);
		if (level != gpio_get_value(oplus_chip->chgic_mtk.oplus_info->ccdetect_gpio)) {
			chg_err("ccdetect_gpio is unstable, try again...\n");
			usleep_range(10000, 11000);
			level = gpio_get_value(oplus_chip->chgic_mtk.oplus_info->ccdetect_gpio);
		}

		if (level <= 0)
			schedule_delayed_work(&pinfo->ccdetect_work, msecs_to_jiffies(6000));

		chg_err("ccdetect_gpio ..level[%d]\n", level);
	}

	if (oplus_ccdetect_check_is_wd0(oplus_chip) == true) {
		INIT_DELAYED_WORK(&pinfo->wd0_detect_work, oplus_wd0_detect_work);
		if (oplus_ccdetect_support_check() == false) {
			INIT_DELAYED_WORK(&pinfo->usbtemp_recover_work, oplus_usbtemp_recover_work);
		}
	}

	if (oplus_usbtemp_check_is_support() == true) {
		if (g_oplus_chip->usb_port_ntc_pullup == USB_PORT_PULL_UP_R_100) {
			oplus_chip->con_volt = con_volt_20131;
			oplus_chip->con_temp = con_temp_20131;
			oplus_chip->len_array = ARRAY_SIZE(con_temp_20131);
		} else {
			oplus_chip->con_volt = con_volt_20682;
			oplus_chip->con_temp = con_temp_20682;
			oplus_chip->len_array = ARRAY_SIZE(con_temp_20682);
		}
		if (alarmtimer_get_rtcdev()) {
			alarm_init(&oplus_chip->usbtemp_alarm_timer, ALARM_BOOTTIME, usbtemp_alarm_timer_func);
		} else {
			chg_err("Failed to get soc alarm-timer");
		}
		oplus_usbtemp_thread_init();
		oplus_wake_up_usbtemp_thread();
	}
	if (IS_ERR(oplus_chip->batt_psy) == 0) {
		ret = device_create_file(&oplus_chip->batt_psy->dev, &dev_attr_StopCharging_Test);/*stop charging*/
		ret = device_create_file(&oplus_chip->batt_psy->dev, &dev_attr_StartCharging_Test);
	}

	return 0;
}


static int oplus_charger_remove(struct platform_device *dev)
{
	oplus_chg_configfs_exit();
	return 0;
}

static void oplus_charger_shutdown(struct platform_device *dev)
{
	if (g_oplus_chip != NULL)
		enter_ship_mode_function(g_oplus_chip);
}

static const struct of_device_id oplus_charger_of_match[] = {
	{.compatible = "mediatek,charger", },
	{},
};

MODULE_DEVICE_TABLE(of, oplus_charger_of_match);

struct platform_device oplus_charger_device = {
	.name = "charger",
	.id = -1,
};

static struct platform_driver oplus_charger_driver = {
	.probe = oplus_charger_probe,
	.remove = oplus_charger_remove,
	.shutdown = oplus_charger_shutdown,
	.driver = {
		   .name = "charger",
		   .of_match_table = oplus_charger_of_match,
		   },
};

static int __init oplus_charger_init(void)
{
	return platform_driver_register(&oplus_charger_driver);
}

static void __exit oplus_charger_exit(void)
{
	platform_driver_unregister(&oplus_charger_driver);
}
oplus_chg_module_register(oplus_charger);

MODULE_DESCRIPTION("MTK Charger Driver");
MODULE_LICENSE("GPL");
