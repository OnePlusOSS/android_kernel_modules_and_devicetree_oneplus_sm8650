// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2021 Oplus. All rights reserved.
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
#include <linux/rtc.h>
#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"
#include "../oplus_adapter.h"
#include "../oplus_short.h"
#include "../oplus_configfs.h"
#include "../oplus_ufcs.h"
#include "op_charge.h"
#include "../../../misc/mediatek/typec/tcpc/inc/tcpci.h"
#include <linux/iio/consumer.h>
#include "oplus_charge_pump.h"
#include "oplus_mp2650.h"
#include "../voocphy/oplus_voocphy.h"
#include "../oplus_pps.h"
#include <tcpm.h>
#include "charger_class.h"

#define DEFAULT_BATTERY_TMP_WHEN_ERROR	-400
#define MT6360_PMU_DPDM_CTRL           0x328
#define USB_TEMP_HIGH		0x01 /*bit0*/
#define USB_WATER_DETECT	0x02 /*bit1*/
#define USB_RESERVE2		0x04 /*bit2*/
#define USB_RESERVE3		0x08 /*bit3*/
#define USB_RESERVE4		0x10 /*bit4*/
#define USB_DONOT_USE		0x80000000

#define OPLUS_MIN_PDO_VOL	5000
#define OPLUS_MIN_PDO_CUR	3000
#define TEMP_25C 250
#define OTG_VBUS_CURRENT_LIMIT_DEFAULT	1100000

int oplus_mtk_hv_flashled_plug(int plug);
int oplus_chg_get_mmi_status(void);

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
        .get_mmi_status = oplus_chg_get_mmi_status,
        .chg_check_break = oplus_chg_check_break,
        .track_check_wired_charging_break = oplus_chg_track_check_wired_charging_break,
        .get_adapter_update_status = oplus_vooc_get_adapter_update_status,
        .get_fastchg_to_normal = oplus_vooc_get_fastchg_to_normal,
        .get_fastchg_to_warm = oplus_vooc_get_fastchg_to_warm,
        .get_support_type = oplus_pps_get_support_type,
        .hv_flashled_plug = oplus_mtk_hv_flashled_plug,
};

static bool em_mode = false;
static int usb_status = 0;
int is_vooc_cfg = false;
bool is_mtksvooc_project = false;
struct mtk_oplus_batt_interface *g_oplus_batt_intf;
struct oplus_chg_chip *g_oplus_chip = NULL;
static struct charger_device *primary_charger;

static int mtk_charger_force_disable_power_path(struct mtk_charger *info,
	int idx, bool disable);
extern int battery_meter_get_charger_voltage(void);
static int oplus_mt6360_reset_charger(void);
static int oplus_mt6360_enable_charging(void);
static int oplus_mt6360_disable_charging(void);
static int oplus_mt6360_float_voltage_write(int vflaot_mv);
static int oplus_mt6360_suspend_charger(void);
static int oplus_mt6360_unsuspend_charger(void);
static int oplus_mt6360_charging_current_write_fast(int chg_curr);
static int oplus_mt6360_set_termchg_current(int term_curr);
static int oplus_mt6360_set_rechg_voltage(int rechg_mv);
static struct mtk_charger *pinfo;
static struct list_head consumer_head = LIST_HEAD_INIT(consumer_head);
static DEFINE_MUTEX(consumer_mutex);
static void mtk_chg_get_tchg(struct mtk_charger *info);
static void oplus_get_chargeric_temp_volt(struct charger_data *pdata);
static void get_chargeric_temp(struct charger_data *pdata);
static bool oplus_chg_is_support_qcpd(void);
static struct task_struct *oplus_usbtemp_kthread;
static bool is_vooc_project(void);
static DECLARE_WAIT_QUEUE_HEAD(oplus_usbtemp_wq);

int oplus_tbatt_power_off_task_init(struct oplus_chg_chip *chip);
void oplus_set_otg_switch_status(bool value);
void oplus_wake_up_usbtemp_thread(void);
void oplus_usbtemp_recover_cc_open(void);
bool oplus_check_pdphy_ready(void);
bool oplus_check_pd_state_ready(void);
bool oplus_usbtemp_condition(void);
bool is_disable_charger(struct mtk_charger *info);
void mt_usb_connect_v1(void);
void mt_usb_disconnect_v1(void);
bool oplus_ccdetect_check_is_gpio(struct oplus_chg_chip *chip);
int oplus_ccdetect_gpio_init(struct oplus_chg_chip *chip);
void oplus_ccdetect_irq_init(struct oplus_chg_chip *chip);
void oplus_ccdetect_disable(void);
void oplus_ccdetect_enable(void);
int oplus_ccdetect_get_power_role(void);
bool oplus_get_otg_switch_status(void);
bool oplus_ccdetect_support_check(void);
int oplus_chg_ccdetect_parse_dt(struct oplus_chg_chip *chip);
int oplus_get_otg_online_status(void);
bool oplus_get_otg_online_status_default(void);

extern struct oplus_chg_operations * oplus_get_chg_ops(void);
extern int oplus_usbtemp_monitor_common(void *data);
extern int oplus_usbtemp_monitor_common_new_method(void *data);
extern void oplus_usbtemp_recover_func(struct oplus_chg_chip *chip);
extern bool set_charge_power_sel(int);
extern bool oplus_chg_get_dischg_flag(void);
extern void register_mtk_oplus_batt_interfaces(struct mtk_oplus_batt_interface *intf);
extern void register_mtk_oplus_chg_interfaces(struct mtk_oplus_chg_interface *intf);
extern int oplus_get_prop_status(void);

/* Add for OTG */
extern int is_vooc_support_single_batt_svooc(void);
extern void vooc_enable_cp_for_otg(int en);

bool oplus_otgctl_by_buckboost(void);
void oplus_otg_enable_by_buckboost(void);
void oplus_otg_disable_by_buckboost(void);


#ifdef CONFIG_OPLUS_CHARGER_MTK
struct mtk_hv_flashled_pinctrl {
	int chgvin_gpio;
	int pmic_chgfunc_gpio;
	int bc1_2_done;
	bool hv_flashled_support;
	struct mutex chgvin_mutex;

	struct pinctrl *pinctrl;
	struct pinctrl_state *chgvin_enable;
	struct pinctrl_state *chgvin_disable;
	struct pinctrl_state *pmic_chgfunc_enable;
	struct pinctrl_state *pmic_chgfunc_disable;
};

struct mtk_hv_flashled_pinctrl mtkhv_flashled_pinctrl;

#define OPLUS_DIVIDER_WORK_MODE_AUTO			1
#define OPLUS_DIVIDER_WORK_MODE_FIXED		0
static int charge_pump_mode = OPLUS_DIVIDER_WORK_MODE_AUTO;
extern int oplus_set_divider_work_mode(int work_mode);
#endif

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
	{120,    2916}, {121,    2832}, {122,    2751}, {123,    2672}, {124,    2596}, {125,    2522}
};

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

__attribute__((unused)) static void oplus_set_usb_status(int status)
{
	usb_status = usb_status | status;
}

__attribute__((unused)) static void oplus_clear_usb_status(int status)
{
	if (g_oplus_chip->usb_status == USB_TEMP_HIGH) {
		g_oplus_chip->usb_status = g_oplus_chip->usb_status & (~USB_TEMP_HIGH);
	}
	usb_status = usb_status & (~status);
}

int oplus_get_usb_status(void)
{
	if (g_oplus_chip->usb_status == USB_TEMP_HIGH) {
		return g_oplus_chip->usb_status;
	} else {
		return usb_status;
	}
}

static bool is_vooc_project(void)
{
	return is_vooc_cfg;
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
	return 0;
}

int mtk_chr_is_charger_exist(unsigned char *exist)
{
	if (mt_get_charger_type() == CHARGER_UNKNOWN)
		*exist = 0;
	else
		*exist = 1;
	return 0;
}

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
	if (!info->charger_wakelock.active)
		__pm_stay_awake(&info->charger_wakelock);
	spin_unlock_irqrestore(&info->slock, flags);
	info->charger_thread_timeout = true;
	wake_up(&info->wait_que);
#endif
}

/* charger_manager ops  */
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

#ifdef OPLUS_FEATURE_CHG_BASIC
	return 0;
#endif

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

#ifdef OPLUS_FEATURE_CHG_BASIC
		if (!pdata) {
			pr_err("%s, pdata null\n", __func__);
			return -EINVAL;
		}

		if(idx == CHG1_SETTING) {
			if (pinfo->chargeric_temp_chan) {
				oplus_get_chargeric_temp_volt(pdata);
				get_chargeric_temp(pdata);
				pdata->junction_temp_min = pdata->chargeric_temp;
				pdata->junction_temp_max = pdata->chargeric_temp;
				consumer->support_ntc_01c_precision = info->support_ntc_01c_precision;
			}
		}
#endif

		*tchg_min = pdata->junction_temp_min;
		*tchg_max = pdata->junction_temp_max;

		return 0;
	}
	return -EBUSY;
}
EXPORT_SYMBOL(charger_manager_get_charger_temperature);

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

/* user interface end*/

/* factory mode */
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
static DEVICE_ATTR(charger_log_level, 0644, show_charger_log_level,
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

static void oplus_check_charger_out_func(struct work_struct *work)
{
	if ((oplus_vooc_get_fastchg_started() == true) ||
	    (oplus_vooc_get_fastchg_to_normal() == true) ||
	    (oplus_vooc_get_fastchg_to_warm() == true) ||
	    (oplus_vooc_get_fastchg_dummy_started() == true))
		oplus_voocphy_chg_out_check_event_handle(true);
	return;
}

void mtk_charger_int_handler(void)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
	chr_err("%s\n", __func__);
	if (!g_oplus_chip) {
		chg_err("g_oplus_chip is null\n");
		return;
	}
	if (is_vooc_project() == false) {
		if (mt_get_charger_type() != CHARGER_UNKNOWN) {
			oplus_wake_up_usbtemp_thread();

			pr_err("%s, Charger Plug In\n", __func__);
			if (mtkhv_flashled_pinctrl.hv_flashled_support) {
				mtkhv_flashled_pinctrl.bc1_2_done = true;
				if (g_oplus_chip->camera_on) {
					mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_disable);
					mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					chr_err("[%s] camera_on %d, chg_vin:%d \n", __func__, g_oplus_chip->camera_on, gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
				}
			}
			charger_manager_notifier(pinfo, CHARGER_NOTIFY_START_CHARGING);
			pinfo->step_status = STEP_CHG_STATUS_STEP1;
			pinfo->step_status_pre = STEP_CHG_STATUS_INVALID;
			pinfo->step_cnt = 0;
			pinfo->step_chg_current = pinfo->data.step1_current_ma;
			charger_dev_set_input_current(g_oplus_chip->chgic_mtk.oplus_info->chg1_dev, 500000);
			schedule_delayed_work(&pinfo->step_charging_work, msecs_to_jiffies(5000));
		} else {
			g_oplus_chip->charger_current_pre = -1;
			pr_err("%s, Charger Plug Out\n", __func__);
#ifdef OPLUS_FEATURE_CHG_BASIC
			if (g_oplus_chip)
				g_oplus_chip->usbtemp_check = oplus_usbtemp_condition();
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
			if (oplus_chg_get_voocphy_support() == AP_SINGLE_CP_VOOCPHY ||
			    oplus_chg_get_voocphy_support() == AP_DUAL_CP_VOOCPHY) {
				if (oplus_vooc_get_fastchg_started() == true &&
				    g_oplus_chip->is_abnormal_adapter != true) {
					chg_err("!!!charger out but fastchg still true, need check charger out\n");
					schedule_delayed_work(&pinfo->check_charger_out_work,
					                      round_jiffies_relative(msecs_to_jiffies(3000)));
				}
			}
#endif

			if (mtkhv_flashled_pinctrl.hv_flashled_support) {
				mtkhv_flashled_pinctrl.bc1_2_done = false;
				if(mt6360_get_vbus_rising() != true) {
					mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_enable);
					mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					pr_err("[OPLUS_CHG]   %d", gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
				}
			}
			charger_dev_set_input_current(g_oplus_chip->chgic_mtk.oplus_info->chg1_dev, 500000);
			charger_manager_notifier(pinfo, CHARGER_NOTIFY_STOP_CHARGING);
			cancel_delayed_work(&pinfo->step_charging_work);
		}
	} else if (oplus_voocphy_get_bidirect_cp_support()) {
			oplus_voocphy_set_chg_auto_mode(false);
			g_oplus_chip->bidirect_abnormal_adapter = false;
			if (mt_get_charger_type() != CHARGER_UNKNOWN) {
				oplus_wake_up_usbtemp_thread();
				if (mtkhv_flashled_pinctrl.hv_flashled_support) {
					mtkhv_flashled_pinctrl.bc1_2_done = true;
				}
				chr_err("bidirect Charger Plug In\n");
			} else {
				g_oplus_chip->charger_current_pre = -1;
				if (g_oplus_chip)
					g_oplus_chip->usbtemp_check = oplus_usbtemp_condition();
				if (mtkhv_flashled_pinctrl.hv_flashled_support) {
					mtkhv_flashled_pinctrl.bc1_2_done = false;
					if(mt6360_get_vbus_rising() != true) {
						mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
						pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_enable);
						mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
						pr_err("[OPLUS_CHG] chgvin_gpio = %d", gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
					}
				}
				chr_err("bidirect Charger Plug Out,charger_current_pre = %d\n", g_oplus_chip->charger_current_pre);
			}
			charger_dev_set_input_current(g_oplus_chip->chgic_mtk.oplus_info->chg1_dev, 500000);
	} else {
		if (mt_get_charger_type() != CHARGER_UNKNOWN) {
			oplus_wake_up_usbtemp_thread();
			oplus_set_divider_work_mode(OPLUS_DIVIDER_WORK_MODE_FIXED);
			chr_err("charge_pump_mode = %d\n", charge_pump_mode);

			if (mtkhv_flashled_pinctrl.hv_flashled_support) {
				mtkhv_flashled_pinctrl.bc1_2_done = true;
				if (g_oplus_chip->camera_on) {
					mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_disable);
					mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					chr_err("[%s] camera_on %d\n", __func__, g_oplus_chip->camera_on);
					pr_err("[OPLUS_CHG]   %d", gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
				}
			}
			chr_err("Charger Plug In\n");
		} else {
			oplus_set_divider_work_mode(charge_pump_mode);
			chr_err("charge_pump_mode = %d\n", charge_pump_mode);
#ifdef OPLUS_FEATURE_CHG_BASIC
			if (g_oplus_chip)
				g_oplus_chip->usbtemp_check = oplus_usbtemp_condition();
#endif
			if (mtkhv_flashled_pinctrl.hv_flashled_support) {
				mtkhv_flashled_pinctrl.bc1_2_done = false;
				if(mt6360_get_vbus_rising() != true) {
					mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_enable);
					mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					pr_err("[OPLUS_CHG]   %d", gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
				}
			}
			g_oplus_chip->charger_current_pre = -1;
			chr_err("Charger Plug Out\n");
		}

		charger_dev_set_input_current(g_oplus_chip->chgic_mtk.oplus_info->chg1_dev, 500000);
		if (g_oplus_chip && g_oplus_chip->vbatt_num == 2) {
			oplus_mt6360_suspend_charger();
		}
	}

#else
	chr_err("%s\n", __func__);

	if (pinfo == NULL) {
		chr_err("charger is not rdy ,skip1\n");
		return;
	}

	if (pinfo->init_done != true) {
		chr_err("charger is not rdy ,skip2\n");
		return;
	}

	if (mt_get_charger_type() == CHARGER_UNKNOWN) {
		mutex_lock(&pinfo->cable_out_lock);
		pinfo->cable_out_cnt++;
		chr_err("cable_out_cnt=%d\n", pinfo->cable_out_cnt);
		mutex_unlock(&pinfo->cable_out_lock);
		charger_manager_notifier(pinfo, CHARGER_NOTIFY_STOP_CHARGING);
	} else
		charger_manager_notifier(pinfo, CHARGER_NOTIFY_START_CHARGING);

	chr_err("wake_up_charger\n");
	_wake_up_charger(pinfo);
#endif /* OPLUS_FEATURE_CHG_BASIC */
}

int oplus_mtk_hv_flashled_plug(int plug)
{
	chr_err("oplus_mtk_hv_flashled_plug %d\n", plug);

	if (!mtkhv_flashled_pinctrl.hv_flashled_support) {
		return -1;
	}

	if(plug == 1) { /*plug_in*/
		if (g_oplus_chip->camera_on) {
			mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
			pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_disable);
			mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
			/*gpio_direction_output(mtkhv_flashled_pinctrl.chgvin_gpio, 1);*/
			chr_err("[%s] camera_on %d\n", __func__, g_oplus_chip->camera_on);
			pr_err("[OPLUS_CHG]   %d", gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
		}
	} else if (plug == 0) {
		mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_enable);
		mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		/*gpio_direction_output(mtkhv_flashled_pinctrl.chgvin_gpio, 0);*/
		pr_err("[OPLUS_CHG]   %d", gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
	}

	return 0;
}
EXPORT_SYMBOL(oplus_mtk_hv_flashled_plug);

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
	info->safety_timeout = false;
	info->vbusov_stat = false;
	info->old_cv = 0;
	info->stop_6pin_re_en = false;
	info->batpro_done = false;
	smart_charging(info);
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

	charger_dev_plug_in(info->chg1_dev);
	mtk_charger_force_disable_power_path(info, CHG1_SETTING, false);

	return 0;
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
	info->dpdmov_stat = false;
	info->lst_dpdmov_stat = false;
	info->pd_reset = false;

	pdata1->disable_charging_count = 0;
	pdata1->input_current_limit_by_aicl = -1;
	pdata2->disable_charging_count = 0;

	notify.evt = EVT_PLUG_OUT;
	notify.value = 0;
	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		chg_alg_notifier_call(alg, &notify);
		chg_alg_plugout_reset(alg);
	}
	memset(&info->sc.data, 0, sizeof(struct scd_cmd_param_t_1));
	charger_dev_set_input_current(info->chg1_dev, 100000);
	charger_dev_set_mivr(info->chg1_dev, info->data.min_charger_voltage);
	charger_dev_plug_out(info->chg1_dev);
	mtk_charger_force_disable_power_path(info, CHG1_SETTING, true);

	if (info->enable_vbat_mon)
		charger_dev_enable_6pin_battery_charging(info->chg1_dev, false);

	/*mtk_adapter_protocol_init(info);*/
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
		if (info->chr_type != chr_type)
			mtk_charger_plug_in(info, chr_type);

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

/* return false if vbus is over max_charger_voltage */
static bool mtk_chg_check_vbus(struct mtk_charger *info)
{
	int vchr = 0;

	vchr = battery_get_vbus() * 1000; /* uV */
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

	vchr = battery_get_vbus() * 1000; /* uV */
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
		pr_debug("[%s] CASE_0001_VCHARGER\n", __func__);
		break;
	case 2:
		info->notify_code = CHG_BAT_OT_STATUS;
		pr_debug("[%s] CASE_0002_VBATTEMP\n", __func__);
		break;
	case 3:
		info->notify_code = CHG_OC_STATUS;
		pr_debug("[%s] CASE_0003_ICHARGING\n", __func__);
		break;
	case 4:
		info->notify_code = CHG_BAT_OV_STATUS;
		pr_debug("[%s] CASE_0004_VBAT\n", __func__);
		break;
	case 5:
		info->notify_code = CHG_ST_TMO_STATUS;
		pr_debug("[%s] CASE_0005_TOTAL_CHARGINGTIME\n", __func__);
		break;
	case 6:
		info->notify_code = CHG_BAT_LT_STATUS;
		pr_debug("[%s] CASE6: VBATTEMP_LOW\n", __func__);
		break;
	case 7:
		info->notify_code = CHG_TYPEC_WD_STATUS;
		pr_debug("[%s] CASE7: Moisture Detection\n", __func__);
		break;
	default:
		pr_debug("[%s] Unknown BN_TestMode Code: %x\n",
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

static void check_battery_exist(struct mtk_charger *info)
{
	unsigned int i = 0;
	int count = 0;

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

static void mtk_chg_get_tchg(struct mtk_charger *info)
{
	int ret;
	int tchg_min = -127, tchg_max = -127;
	struct charger_data *pdata;

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
		ret = charger_dev_get_adc(info->dvchg1_dev,
					  ADC_CHANNEL_TEMP_JC,
					  &tchg_min, &tchg_max);
		if (ret < 0) {
			pdata->junction_temp_min = -127;
			pdata->junction_temp_max = -127;
		} else {
			pdata->junction_temp_min = tchg_min;
			pdata->junction_temp_max = tchg_max;
		}
	}

	if (info->dvchg2_dev) {
		pdata = &info->chg_data[DVCHG2_SETTING];
		ret = charger_dev_get_adc(info->dvchg2_dev,
					  ADC_CHANNEL_TEMP_JC,
					  &tchg_min, &tchg_max);
		if (ret < 0) {
			pdata->junction_temp_min = -127;
			pdata->junction_temp_max = -127;
		} else {
			pdata->junction_temp_min = tchg_min;
			pdata->junction_temp_max = tchg_max;
		}
	}

	if (info->hvdvchg1_dev) {
		pdata = &info->chg_data[HVDVCHG1_SETTING];
		ret = charger_dev_get_adc(info->hvdvchg1_dev,
					  ADC_CHANNEL_TEMP_JC,
					  &tchg_min, &tchg_max);
		if (ret < 0) {
			pdata->junction_temp_min = -127;
			pdata->junction_temp_max = -127;
		} else {
			pdata->junction_temp_min = tchg_min;
			pdata->junction_temp_max = tchg_max;
		}
	}

	if (info->hvdvchg2_dev) {
		pdata = &info->chg_data[HVDVCHG2_SETTING];
		ret = charger_dev_get_adc(info->hvdvchg2_dev,
					  ADC_CHANNEL_TEMP_JC,
					  &tchg_min, &tchg_max);
		if (ret < 0) {
			pdata->junction_temp_min = -127;
			pdata->junction_temp_max = -127;
		} else {
			pdata->junction_temp_min = tchg_min;
			pdata->junction_temp_max = tchg_max;
		}
	}
}

int _mtk_enable_charging(struct mtk_charger *info,
	bool en)
{
	chr_debug("%s en:%d\n", __func__, en);
	if (info->algo.enable_charging != NULL)
		return info->algo.enable_charging(info, en);
	return false;
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
	if (info->dpdmov_stat)
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

	chr_err("tmp:%d (jeita:%d sm:%d cv:%d en:%d) (sm:%d) en:%d c:%d s:%d ov:%d %d sc:%d %d %d saf_cmd:%d bat_mon:%d %d\n",
		temperature, info->enable_sw_jeita, info->sw_jeita.sm,
		info->sw_jeita.cv, info->sw_jeita.charging, thermal->sm,
		charging, info->cmd_discharging, info->safety_timeout,
		info->vbusov_stat, info->dpdmov_stat, info->sc.disable_charger,
		info->can_charging, charging, info->safety_timer_cmd,
		info->enable_vbat_mon, info->batpro_done);

	charger_dev_is_enabled(info->chg1_dev, &chg_dev_chgen);

	if (charging != info->can_charging)
		_mtk_enable_charging(info, charging);
	else if (charging == false && chg_dev_chgen == true)
		_mtk_enable_charging(info, charging);

	info->can_charging = charging;
}

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
	if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT ||
	    boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
		vbus = get_vbus(info);
		if (vbus >= 0 && vbus < 2500 && !mtk_is_charger_on(info) &&
			!info->pd_reset) {
			chr_err("Unplug Charger/USB in KPOC mode, vbus=%d, shutdown\n", vbus);
			while (1) {
				if (info->is_suspend == false) {
					chr_err("%s, not in suspend, shutdown\n", __func__);
					kernel_power_off();
					break;
				} else {
					chr_err("%s, suspend! cannot shutdown\n", __func__);
					msleep(20);
				}
			}
		}
		charger_send_kpoc_uevent(info);
	}
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
	unsigned long flags;

	if (pinfo->is_suspend == false) {
		chr_err("%s: not suspend, wake up charger\n", __func__);
		_wake_up_charger(pinfo);
	} else {
		chr_err("%s: alarm timer timeout\n", __func__);
		spin_lock_irqsave(&pinfo->slock, flags);
		if (!pinfo->charger_wakelock->active)
			__pm_stay_awake(pinfo->charger_wakelock);
		spin_unlock_irqrestore(&pinfo->slock, flags);
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

static void charger_status_check(struct mtk_charger *info)
{
	union power_supply_propval online = {0}, status = {0};
	struct power_supply *chg_psy = NULL;
	int ret;
	bool charging = true;

#if IS_ENABLED(CONFIG_MTK_PLAT_POWER_6893)
	chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev,
						       "charger");
#else
	chg_psy = power_supply_get_by_name("primary_chg");
#endif
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

static bool charger_init_algo(struct mtk_charger *info)
{
	struct chg_alg_device *alg;
	int idx = 0;
	int ret = 0;

	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev)
		chr_err("%s, Found primary charger\n", __func__);
	else {
		chr_err("%s, *** Error : can't find primary charger ***\n"
			, __func__);
		return false;
	}

	chr_err("%s, start current_selector init flow: %s\n", __func__, info->curr_select_name);
	if (strcmp(info->curr_select_name, "current_selector_master") == 0)
		info->cschg1_dev = get_charger_by_name("current_selector_master");

	if (info->cschg1_dev) {
		chr_err("%s, Found main current selector charger\n", __func__);
		ret = charger_cs_init_setting(info->cschg1_dev);
		if (ret < 0) {
			chr_err("%s, failed to init cs, close cs function\n", __func__);
			info->cs_hw_disable = true;
		} else {
			ret = charger_dev_set_constant_voltage(info->cschg1_dev, 4350);
		if (ret < 0)
			chr_err("%s: failed to set cs1 cv to: 4350mV.\n", __func__);
		ret = charger_dev_set_charging_current(info->cschg1_dev, AC_CS_NORMAL_CC);
		if (ret < 0)
			chr_err("%s: failed to set cs1 cc to: %d mA.\n", __func__, AC_CS_NORMAL_CC);
		}
		info->cs_cc_now = AC_CS_NORMAL_CC;
	} else {
		chr_err("%s, *** Warning : can't find main current selector charger ***\n"
			, __func__);
	}

	alg = get_chg_alg_by_name("pe5p");
	info->alg[idx] = alg;
	if (alg == NULL)
		chr_err("get pe5p fail\n");
	else {
		chr_err("get pe5p success\n");
		alg->config = info->config;
		alg->alg_id = PE5P_ID;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}
	idx++;

	alg = get_chg_alg_by_name("hvbp");
	info->alg[idx] = alg;
	if (alg == NULL)
		chr_err("get hvbp fail\n");
	else {
		chr_err("get hvbp success\n");
		alg->config = info->config;
		alg->alg_id = HVBP_ID;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}
	idx++;

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

	alg = get_chg_alg_by_name("pe45");
	if (alg == NULL) {
		chr_err("cannot get pe45\n");
		alg = get_chg_alg_by_name("pe4");
		info->alg[idx] = alg;
		if (alg == NULL)
			chr_err("cannot get pe4\n");
		else {
			chr_err("get pe4 success\n");
			alg->config = info->config;
			alg->alg_id = PE4_ID;
			chg_alg_init_algo(alg);
			register_chg_alg_notifier(alg, &info->chg_alg_nb);
		}
	} else {
		info->alg[idx] = alg;
		chr_err("get pe45 success\n");
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
	} else if (info->config == HVDIVIDER_CHARGER ||
		   info->config == DUAL_HVDIVIDER_CHARGERS) {
		info->hvdvchg1_dev = get_charger_by_name("hvdiv2_chg1");
		if (info->hvdvchg1_dev)
			chr_err("Found primary hvdivider charger\n");
		else {
			chr_err("*** Error : can't find primary hvdivider charger ***\n");
			return false;
		}
		if (info->config == DUAL_HVDIVIDER_CHARGERS) {
			info->hvdvchg2_dev = get_charger_by_name("hvdiv2_chg2");
			if (info->hvdvchg2_dev)
				chr_err("Found secondary hvdivider charger\n");
			else {
				chr_err("*** Error : can't find secondary hvdivider charger ***\n");
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

	chr_err("register hvdvchg chg1 notifier %d %d\n",
		info->hvdvchg1_dev != NULL,
		info->algo.do_hvdvchg1_event != NULL);
	if (info->hvdvchg1_dev != NULL &&
	    info->algo.do_hvdvchg1_event != NULL) {
		chr_err("register hvdvchg chg1 notifier done\n");
		info->hvdvchg1_nb.notifier_call = info->algo.do_hvdvchg1_event;
		register_charger_device_notifier(info->hvdvchg1_dev,
						 &info->hvdvchg1_nb);
		charger_dev_set_drvdata(info->hvdvchg1_dev, info);
	}

	chr_err("register hvdvchg chg2 notifier %d %d\n",
		info->hvdvchg2_dev != NULL,
		info->algo.do_hvdvchg2_event != NULL);
	if (info->hvdvchg2_dev != NULL &&
	    info->algo.do_hvdvchg2_event != NULL) {
		chr_err("register hvdvchg chg2 notifier done\n");
		info->hvdvchg2_nb.notifier_call = info->algo.do_hvdvchg2_event;
		register_charger_device_notifier(info->hvdvchg2_dev,
						 &info->hvdvchg2_nb);
		charger_dev_set_drvdata(info->hvdvchg2_dev, info);
	}

	return true;
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
	default:
		return "unknown";
	}
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

static int charger_routine_thread(void *arg)
{
	struct mtk_charger *info = arg;
	unsigned long flags;
	unsigned int init_times = 3;
	static bool is_module_init_done;
	bool is_charger_on;
	int ret;
	int vbat_min = 0;
	int vbat_max = 0;
	int cs_vbat, cs_ibat;
	u32 chg_cv = 0;

	while (1) {
		ret = wait_event_interruptible(info->wait_que,
			(info->charger_thread_timeout == true));
		if (ret < 0) {
			chr_err("%s: wait event been interrupted(%d)\n", __func__, ret);
			continue;
		}

		while (is_module_init_done == false) {
			if (charger_init_algo(info) == true) {
				is_module_init_done = true;
				if (info->charger_unlimited) {
					info->enable_sw_safety_timer = false;
					charger_dev_enable_safety_timer(info->chg1_dev, false);
				}
			}
			else {
				if (init_times > 0) {
					chr_err("retry to init charger\n");
					init_times = init_times - 1;
					msleep(10000);
				} else {
					chr_err("holding to init charger\n");
					msleep(60000);
				}
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

		/* get data from chgIC first, cs adc is backup */
		get_cs_side_battery_voltage(info, &cs_vbat);
		get_cs_side_battery_current(info, &cs_ibat);

		is_charger_on = mtk_is_charger_on(info);

		if (info->charger_thread_polling == true)
			mtk_charger_start_timer(info);

		check_battery_exist(info);
		check_dynamic_mivr(info);
		charger_check_status(info);
		/*mtk_check_ta_status(info);*/
		kpoc_power_off_check(info);

		if (!info->cs_hw_disable)
			chr_err("Vbat=%d vbat2=%d vbats=%d vbus:%d ibus:%d I=%d I2=%d T=%d uisoc:%d type:%s>%s idx:%d ta_stat:%d swchg_ibat:%d cv:%d cmd_pp:%d, pd_reset:%d\n",
				get_battery_voltage(info),
				cs_vbat,
				vbat_min,
				get_vbus(info),
				get_ibus(info),
				get_battery_current(info),
				cs_ibat,
				info->battery_temp,
				get_uisoc(info),
				dump_charger_type(info->chr_type, info->usb_type),
				dump_charger_type(get_charger_type(info), get_usb_type(info)), info->select_adapter_idx,
				info->ta_status[info->select_adapter_idx], get_ibat(info), chg_cv, info->cmd_pp, info->pd_reset);
		else
			chr_err("Vbat=%d vbats=%d vbus:%d ibus:%d I=%d T=%d uisoc:%d type:%s>%s idx:%d ta_stat:%d swchg_ibat:%d cv:%d cmd_pp:%d pd_reset:%d\n",
				get_battery_voltage(info),
				vbat_min,
				get_vbus(info),
				get_ibus(info),
				get_battery_current(info),
				info->battery_temp,
				get_uisoc(info),
				dump_charger_type(info->chr_type, info->usb_type),
				dump_charger_type(get_charger_type(info), get_usb_type(info)), info->select_adapter_idx,
				info->ta_status[info->select_adapter_idx], get_ibat(info), chg_cv, info->cmd_pp, info->pd_reset);

		if (is_disable_charger(info) == false &&
			is_charger_on == true &&
			info->can_charging == true) {
			if (info->algo.do_algorithm)
				info->algo.do_algorithm(info);
			charger_status_check(info);
		} else {
			chr_debug("disable charging %d %d %d\n",
			    is_disable_charger(info), is_charger_on, info->can_charging);
		}
		if (info->bootmode != 1 && info->bootmode != 2 && info->bootmode != 4
			&& info->bootmode != 8 && info->bootmode != 9)
			smart_charging(info);
		spin_lock_irqsave(&info->slock, flags);
		__pm_relax(info->charger_wakelock);
		spin_unlock_irqrestore(&info->slock, flags);
		chr_debug("%s end , %d\n",
			__func__, info->charger_thread_timeout);
		mutex_unlock(&info->charger_lock);

		if (info->enable_boot_volt &&
			ktime_get_seconds() > RESET_BOOT_VOLT_TIME &&
			!info->reset_boot_volt_times) {
			ret = charger_dev_set_boot_volt_times(info->chg1_dev, 0);
			if (ret < 0)
				chr_err("reset boot_battery_voltage times fails %d\n", ret);
			else {
				info->reset_boot_volt_times = 1;
				chr_err("reset boot_battery_voltage times\n");
			}
		}
	}

	return 0;
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

#ifndef OPLUS_FEATURE_CHG_BASIC
	if (strcmp(info->algorithm_name, "Basic") == 0) {
		chr_err("found Basic\n");
		mtk_basic_charger_init(info);
	} else if (strcmp(info->algorithm_name, "Pulse") == 0) {
		chr_err("found Pulse\n");
		mtk_pulse_charger_init(info);
	}
#endif

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

	if (of_property_read_u32(np, "qcom,sub_board_pull_up_r", &val) >= 0) {
		g_oplus_chip->chgic_mtk.sub_board_pull_up_r = val;
	} else {
		chr_err("use default sub_board_pull_up_r:%d\n",
			SUB_BOARD_PULL_UP_R);
		g_oplus_chip->chgic_mtk.sub_board_pull_up_r = SUB_BOARD_PULL_UP_R;
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

	if (of_property_read_u32(np, "oplus,otg_current_limit", &val)>= 0)
		info->otg_current_limit = val;
	else
		info->otg_current_limit = OTG_VBUS_CURRENT_LIMIT_DEFAULT;

#endif
}

static int mtk_charger_enable_power_path(struct mtk_charger *info,
	int idx, bool en)
{
	int ret = 0;
	bool is_en = true;
	struct charger_device *chg_dev = NULL;

	if (!info)
		return -EINVAL;

	switch (idx) {
	case CHG1_SETTING:
		chg_dev = get_charger_by_name("primary_chg");
		break;
	case CHG2_SETTING:
		chg_dev = get_charger_by_name("secondary_chg");
		break;
	default:
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(chg_dev)) {
		chr_err("%s: chg_dev not found\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&info->pp_lock[idx]);
	info->enable_pp[idx] = en;

	if (info->force_disable_pp[idx])
		goto out;

	ret = charger_dev_is_powerpath_enabled(chg_dev, &is_en);
	if (ret < 0) {
		chr_err("%s: get is power path enabled failed\n", __func__);
		goto out;
	}
	if (is_en == en) {
		chr_err("%s: power path is already en = %d\n", __func__, is_en);
		goto out;
	}

	pr_info("%s: enable power path = %d\n", __func__, en);
	ret = charger_dev_enable_powerpath(chg_dev, en);
out:
	mutex_unlock(&info->pp_lock[idx]);
	return ret;
}

static int mtk_charger_force_disable_power_path(struct mtk_charger *info,
	int idx, bool disable)
{
	int ret = 0;
	struct charger_device *chg_dev = NULL;

	if (!info)
		return -EINVAL;

	switch (idx) {
	case CHG1_SETTING:
		chg_dev = get_charger_by_name("primary_chg");
		break;
	case CHG2_SETTING:
		chg_dev = get_charger_by_name("secondary_chg");
		break;
	default:
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(chg_dev)) {
		chr_err("%s: chg_dev not found\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&info->pp_lock[idx]);

	if (disable == info->force_disable_pp[idx])
		goto out;

	info->force_disable_pp[idx] = disable;
	ret = charger_dev_enable_powerpath(chg_dev,
		info->force_disable_pp[idx] ? false : info->enable_pp[idx]);
out:
	mutex_unlock(&info->pp_lock[idx]);
	return ret;
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

static int psy_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_charger *info;
	struct charger_device *chg;
	int ret = 0, chg_vbat = 0, vbat_max = 0, idx = 0;
	struct chg_alg_device *alg = NULL;

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);
	if (info == NULL) {
		chr_err("%s: get info failed\n", __func__);
		return -EINVAL;
	}
	chr_debug("%s psp:%d\n", __func__, psp);

	if (info->psy1 == psy) {
		chg = info->chg1_dev;
		idx = CHG1_SETTING;
	} else if (info->psy2 == psy) {
		chg = info->chg2_dev;
		idx = CHG2_SETTING;
	} else if (info->psy_dvchg1 == psy) {
		chg = info->dvchg1_dev;
		idx = DVCHG1_SETTING;
	} else if (info->psy_dvchg2 == psy) {
		chg = info->dvchg2_dev;
		idx = DVCHG2_SETTING;
	} else if (info->psy_hvdvchg1 == psy) {
		chg = info->hvdvchg1_dev;
		idx = HVDVCHG1_SETTING;
	} else if (info->psy_hvdvchg2 == psy) {
		chg = info->hvdvchg2_dev;
		idx = HVDVCHG2_SETTING;
	} else {
		chr_err("%s fail\n", __func__);
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (idx == DVCHG1_SETTING || idx == DVCHG2_SETTING ||
		    idx == HVDVCHG1_SETTING || idx == HVDVCHG2_SETTING) {
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
		val->intval = info->chg_data[idx].junction_temp_max * 10;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval =
			info->chg_data[idx].thermal_charging_current_limit;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval =
			info->chg_data[idx].thermal_input_current_limit;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_BOOT:
		val->intval = get_charger_zcv(info, chg);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		chr_debug("not yet\n");
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = charger_dev_get_adc(info->chg1_dev,
			ADC_CHANNEL_VBAT, &chg_vbat, &vbat_max);
		val->intval = chg_vbat;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int psy_charger_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	struct mtk_charger *info;
	int idx;

	chr_err("%s: prop:%d %d\n", __func__, psp, val->intval);

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);
	if (info == NULL) {
		chr_err("%s: failed to get info\n", __func__);
		return -EINVAL;
	}

	if (info->psy1 == psy)
		idx = CHG1_SETTING;
	else if (info->psy2 == psy)
		idx = CHG2_SETTING;
	else if (info->psy_dvchg1 == psy)
		idx = DVCHG1_SETTING;
	else if (info->psy_dvchg2 == psy)
		idx = DVCHG2_SETTING;
	else if (info->psy_hvdvchg1 == psy)
		idx = HVDVCHG1_SETTING;
	else if (info->psy_hvdvchg2 == psy)
		idx = HVDVCHG2_SETTING;
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
		if (val->intval > 0)
			mtk_charger_enable_power_path(info, idx, false);
		else
			mtk_charger_enable_power_path(info, idx, true);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		if (val->intval > 0)
			mtk_charger_force_disable_power_path(info, idx, true);
		else
			mtk_charger_force_disable_power_path(info, idx, false);
		break;
	default:
		return -EINVAL;
	}
	_wake_up_charger(info);

	return 0;
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

static DEVICE_ATTR(BatteryNotify, 0644, show_BatNotify, store_BatNotify);

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

#define OPLUS_SVID 0x22D9
uint32_t pd_svooc_abnormal_adapter[] = {
	0x20002,
	0x10002,
};

int oplus_get_adapter_svid(void)
{
	int i = 0;
	int j = 0;
	int ret = 0;
	uint32_t vdos[VDO_MAX_NR] = {0};
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	struct tcpm_svid_list svid_list = {0, {0}};

	if (tcpc_dev == NULL || !g_oplus_chip) {
		chg_err("tcpc_dev is null return\n");
		return -1;
	}

	if (!g_oplus_chip->pd_svooc) {
		ret = tcpm_dpm_vdm_discover_id(tcpc_dev, NULL);
		if (ret != TCPM_SUCCESS) {
			chg_err("Failed to discover id\n");
		}

		ret = tcpm_dpm_vdm_discover_svid(tcpc_dev, NULL);
		if (ret != TCPM_SUCCESS) {
			chg_err("Failed to discover svid\n");
		}
	}

	tcpm_inquire_pd_partner_svids(tcpc_dev, &svid_list);
	for (i = 0; i < svid_list.cnt; i++) {
		chg_info("svid[%d] = 0x%x\n", i, svid_list.svids[i]);
		if (svid_list.svids[i] == OPLUS_SVID) {
			g_oplus_chip->pd_svooc = true;
			chg_info("match svid and this is oplus adapter\n");
			break;
		}
	}

	tcpm_inquire_pd_partner_inform(tcpc_dev, vdos);
	if ((vdos[0] & 0xFFFF) == OPLUS_SVID) {
		g_oplus_chip->pd_svooc = true;
		chg_info("match svid and this is oplus adapter\n");
		for (j = 0; j < ARRAY_SIZE(pd_svooc_abnormal_adapter); j++) {
			if (pd_svooc_abnormal_adapter[j] == vdos[2]) {
				chg_info("This is oplus gnd abnormal adapter %x %x\n", vdos[1], vdos[2]);
				g_oplus_chip->is_abnormal_adapter = true;
				break;
			}
		}
	}
	chg_info("svid[0x%x],pid&bcd [0x%x 0x%x]\n", vdos[0], vdos[1], vdos[2]);

	return 0;
}

bool oplus_chg_check_pd_svooc_adapater(void)
{
	if (!g_oplus_chip) {
		chr_err("g_oplus_chip is null return \n");
		return false;
	}

	chg_err("pd_svooc = %d\n", g_oplus_chip->pd_svooc);

	return (g_oplus_chip->pd_svooc);
}
EXPORT_SYMBOL(oplus_chg_check_pd_svooc_adapater);

bool oplus_check_pd_state_ready(void)
{
	if (!pinfo) {
		chr_err("pinfo is null return \n");
		return false;
	}
	return (pinfo->in_good_connect);
}
EXPORT_SYMBOL(oplus_check_pd_state_ready);

bool oplus_check_pdphy_ready(void)
{
	return (pinfo != NULL && pinfo->tcpc != NULL && pinfo->tcpc->pd_inited_flag);
}

static void oplus_chg_pps_get_source_cap(struct mtk_charger *info);
static void mtk_charger_external_power_changed(struct power_supply *psy);
static void mtk_charger_external_power_changed(struct power_supply *psy)
{
	struct mtk_charger *info;
	union power_supply_propval prop = {0};
	union power_supply_propval prop2 = {0};
	union power_supply_propval vbat0 = {0};
	struct power_supply *chg_psy = NULL;
	int ret;

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);
	if (info == NULL) {
		pr_notice("%s: failed to get info\n", __func__);
		return;
	}
	chg_psy = info->chg_psy;

	if (IS_ERR_OR_NULL(chg_psy)) {
		pr_notice("%s Couldn't get chg_psy\n", __func__);
#if IS_ENABLED(CONFIG_MTK_PLAT_POWER_6893)
		chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev,
						       "charger");
#else
		chg_psy = power_supply_get_by_name("primary_chg");
#endif
		info->chg_psy = chg_psy;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop2);
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ENERGY_EMPTY, &vbat0);
	}

	if (info->vbat0_flag != vbat0.intval) {
		if (vbat0.intval) {
			info->enable_vbat_mon = false;
			charger_dev_enable_6pin_battery_charging(info->chg1_dev, false);
		} else
			info->enable_vbat_mon = info->enable_vbat_mon_bak;

		info->vbat0_flag = vbat0.intval;
	}

	pr_notice("%s event, name:%s online:%d type:%d vbus:%d\n", __func__,
		psy->desc->name, prop.intval, prop2.intval,
		get_vbus(info));

	_wake_up_charger(info);
}

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
			oplus_set_usb_status(USB_WATER_DETECT);
			oplus_vooc_set_disable_adapter_output(true);
			if (g_oplus_chip && g_oplus_chip->usb_psy)
				power_supply_changed(g_oplus_chip->usb_psy);
		} else {
			pinfo->notify_code &= ~CHG_TYPEC_WD_STATUS;
			if (boot_mode == 8 || boot_mode == 9)
				pinfo->enable_hv_charging = true;
			oplus_clear_usb_status(USB_WATER_DETECT);
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

static void oplus_mt6360_dump_registers(void)
{
	struct charger_device *chg = NULL;
	static bool musb_hdrc_release = false;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (musb_hdrc_release == false &&
			g_oplus_chip->unwakelock_chg == 1 &&
			mt_get_charger_type() == NONSTANDARD_CHARGER) {
		musb_hdrc_release = true;
		mt_usb_disconnect_v1();
	} else {
		if (musb_hdrc_release == true &&
				g_oplus_chip->unwakelock_chg == 0 &&
				mt_get_charger_type() == NONSTANDARD_CHARGER) {
			musb_hdrc_release = false;
			mt_usb_connect_v1();
		}
	}

	/*This function runs for more than 400ms, so return when no charger for saving power */
	if (g_oplus_chip->charger_type == POWER_SUPPLY_TYPE_UNKNOWN
			|| oplus_get_chg_powersave() == true) {
		return;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	if (pinfo->data.dual_charger_support) {
		charger_dev_dump_registers(chg);
	} else {
		charger_dev_dump_registers(chg);
	}

	return;
}

static int oplus_mt6360_kick_wdt(void)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_kick_wdt(chg);
	if (rc < 0) {
		chg_debug("charger_dev_kick_wdt fail\n");
	}
	return 0;
}

static int oplus_mt6360_hardware_init(void)
{
	int hw_aicl_point = 4400;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	oplus_mt6360_reset_charger();

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		oplus_mt6360_disable_charging();
		oplus_mt6360_float_voltage_write(4400);
		msleep(100);
	}

	oplus_mt6360_float_voltage_write(4385);

	mt6360_set_register(0x1C, 0xFF, 0xF9);

	mt6360_set_register(0x18, 0xF, 0x4);

	oplus_mt6360_charging_current_write_fast(512);

	oplus_mt6360_set_termchg_current(150);

	oplus_mt6360_set_rechg_voltage(100);

	charger_dev_set_mivr(chg, hw_aicl_point * 1000);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (oplus_is_rf_ftm_mode()) {
		oplus_mt6360_suspend_charger();
		oplus_mt6360_disable_charging();
	} else {
		oplus_mt6360_unsuspend_charger();
		oplus_mt6360_enable_charging();
	}
#else /* CONFIG_OPLUS_CHARGER_MTK */
	oplus_mt6360_unsuspend_charger();
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	mt6360_set_register(0x1D, 0x30, 0x10);

	return 0;
}

static int oplus_mt6360_charging_current_write_fast(int chg_curr)
{
	int rc = 0;
	u32 ret_chg_curr = 0;
	struct charger_device *chg = NULL;
	int main_cur = 0;
	int slave_cur = 0;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}

	if (!g_oplus_chip->authenticate) {
		g_oplus_chip->chg_ops->charging_disable();
		printk(KERN_ERR "[OPLUS_CHG][%s]:!g_oplus_chip->authenticate , charging_disable\n", __func__);
		return 0;
	}

	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	if (pinfo->data.dual_charger_support) {
		if (g_oplus_chip->tbatt_status == BATTERY_STATUS__NORMAL) {
			if (chg_curr > pinfo->step_chg_current)
				chg_curr = pinfo->step_chg_current;
		}

		if (g_oplus_chip->dual_charger_support &&
				(g_oplus_chip->slave_charger_enable || em_mode)) {
			main_cur = (chg_curr * (100 - g_oplus_chip->slave_pct))/100;
			main_cur -= main_cur % 100;
			slave_cur = chg_curr - main_cur;

			rc = charger_dev_set_charging_current(chg, main_cur * 1000);
			if (rc < 0) {
				chg_debug("set fast charge current:%d fail\n", main_cur);
			}

		} else {
			rc = charger_dev_set_charging_current(chg, chg_curr * 1000);
			if (rc < 0) {
				chg_debug("set fast charge current:%d fail\n", chg_curr);
			}
		}
	} else {
		rc = charger_dev_set_charging_current(chg, chg_curr * 1000);
		if (rc < 0) {
			chg_debug("set fast charge current:%d fail\n", chg_curr);
		} else {
			charger_dev_get_charging_current(chg, &ret_chg_curr);
			chg_debug("set fast charge current:%d ret_chg_curr = %d\n", chg_curr, ret_chg_curr);
		}
	}
	return 0;
}

#define HW_AICL_POINT_PDQC_SINGLE_BAT	7600
#define HW_AICL_POINT_PDQC_DUAL_BAT		8400
#define PDQC_9V_AICL_SET_THD			7000
static void oplus_mt6360_set_aicl_point(int vbatt)
{
	int rc = 0;
	static int hw_aicl_point = 4400;
	static int pdqc_hw_aicl_point = 7600;
	struct charger_device *chg = NULL;
	int chg_vol = 0;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (!g_oplus_chip->authenticate) {
		printk(KERN_ERR "[OPLUS_CHG][%s]:!g_oplus_chip->authenticate \n", __func__);
		return;
	}

	if (g_oplus_chip->vbatt_num == 1) {
		pdqc_hw_aicl_point = HW_AICL_POINT_PDQC_SINGLE_BAT;
	} else {
		pdqc_hw_aicl_point = HW_AICL_POINT_PDQC_DUAL_BAT;
	}
	chg_debug("pdqc_hw_aicl_point:%d\n", pdqc_hw_aicl_point);
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	chg_vol = battery_meter_get_charger_voltage();

	if (g_oplus_chip->pdqc_9v_voltage_adaptive) {
		if ((g_oplus_chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_QC ||
			g_oplus_chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_PD) &&
			(oplus_vooc_get_fastchg_started() == false) && (chg_vol > PDQC_9V_AICL_SET_THD)) {
				hw_aicl_point = pdqc_hw_aicl_point;
		} else {
			if (((hw_aicl_point == 4400) || (hw_aicl_point == pdqc_hw_aicl_point)) && vbatt > 4100) {
				hw_aicl_point = 4500;
			} else if (((hw_aicl_point == 4400) || (hw_aicl_point == pdqc_hw_aicl_point)) && vbatt <= 4100) {
				hw_aicl_point = 4400;
			}
		}
	} else {
		if (hw_aicl_point == 4400 && vbatt > 4100) {
			hw_aicl_point = 4500;
		} else if (hw_aicl_point == 4500 && vbatt <= 4100) {
			hw_aicl_point = 4400;
		}
	}
	rc = charger_dev_set_mivr(chg, hw_aicl_point * 1000);
	if (rc < 0) {
		chg_debug("set aicl point:%d fail\n", hw_aicl_point);
	}
	chg_debug("set aicl point:%d,chg_vol:%d\n", hw_aicl_point, chg_vol);
}

static int usb_icl[] = {
	300, 500, 900, 1200, 1350, 1500, 2000, 2400, 3000,
};

#define BATT_VOLT_FLAG		4100
#define SW_AICL_POINT_LOW	4500
#define SW_AICL_POINT_HIGH	4550
#define SW_AICL_POINT_PDQC	7600

static int oplus_mt6360_input_current_limit_write(int value)
{
	int rc = 0;
	int i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	int vbus_mv = 0;
	int ibus_ma = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg_debug("usb input max current limit=%d setting %02x\n", value, i);

	if (!g_oplus_chip->authenticate) {
		g_oplus_chip->chg_ops->charging_disable();
		printk(KERN_ERR "[OPLUS_CHG][%s]: !g_oplus_chip->authenticate , charging_disable\n", __func__);
		return 0;
	}

	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	if (g_oplus_chip->chg_ops->oplus_chg_get_pd_type) {
		if (g_oplus_chip->chg_ops->oplus_chg_get_pd_type() != PD_INACTIVE) {
			rc = oplus_pdc_get(&vbus_mv, &ibus_ma);
			if (rc >= 0 && ibus_ma >= 500 && ibus_ma < 3000 && value > ibus_ma) {
				value = ibus_ma;
				chg_debug("usb input max current limit=%d(pd)\n", value);
			}
		}
	}
	if (g_oplus_chip->charger_current_pre == value) {
		chg_err("charger_current_pre == value =%d.\n", value);
		return rc;
	} else {
		g_oplus_chip->charger_current_pre = value;
	}

	if (g_oplus_chip->dual_charger_support ||
		g_oplus_chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_QC ||
		g_oplus_chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_PD) {
		chg_vol = battery_meter_get_charger_voltage();
		if (chg_vol > SW_AICL_POINT_PDQC) {
			aicl_point = SW_AICL_POINT_PDQC;
		} else {
			if (g_oplus_chip->batt_volt > BATT_VOLT_FLAG)
				aicl_point = SW_AICL_POINT_HIGH;
			else
				aicl_point = SW_AICL_POINT_LOW;
		}
	} else {
		if (g_oplus_chip->batt_volt > BATT_VOLT_FLAG)
			aicl_point = SW_AICL_POINT_HIGH;
		else
			aicl_point = SW_AICL_POINT_LOW;
	}

	if (value < 500) {
		i = 0;
		goto aicl_end;
	}

	mt6360_aicl_enable(false);

	i = 1; /* 500 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		chg_debug("use 500 here\n");
		goto aicl_end;
	} else if (value < 900)
		goto aicl_end;

	i = 2; /* 900 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value < 1200)
		goto aicl_end;

	i = 3; /* 1200 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1350 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 2;
		goto aicl_pre_step;
	}

	i = 5; /* 1500 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 3; /*We DO NOT use 1.2A here*/
		goto aicl_pre_step;
	} else if (value < 1500) {
		i = i - 2; /*We use 1.2A here*/
		goto aicl_end;
	} else if (value < 2000)
		goto aicl_end;

	i = 6; /* 2000 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value < 2400)
		goto aicl_end;

	i = 7; /* 2400 */
        rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
        msleep(90);
        chg_vol = battery_meter_get_charger_voltage();
        if (chg_vol < aicl_point) {
                i = i - 1;
                goto aicl_pre_step;
        } else if (value < 3000)
                goto aicl_end;

	i = 8; /* 3000 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value >= 3000)
		goto aicl_end;

aicl_pre_step:
	chg_debug("usb input max current limit aicl chg_vol=%d i[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, i, usb_icl[i], aicl_point);
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	goto aicl_rerun;
aicl_end:
	chg_debug("usb input max current limit aicl chg_vol=%d i[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, i, usb_icl[i], aicl_point);
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	goto aicl_rerun;
aicl_rerun:
	mt6360_aicl_enable(true);
	g_oplus_chip->charger_current_pre = usb_icl[i];
	return rc;
}

#define DELTA_MV        32
static int oplus_mt6360_float_voltage_write(int vfloat_mv)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	if (pinfo->data.dual_charger_support) {
		rc = charger_dev_set_constant_voltage(chg, vfloat_mv * 1000);
		if (rc < 0) {
			chg_debug("set float voltage:%d fail\n", vfloat_mv);
		}
	} else {
		rc = charger_dev_set_constant_voltage(chg, vfloat_mv * 1000);
		if (rc < 0) {
			chg_debug("set float voltage:%d fail\n", vfloat_mv);
		}
	}

	return 0;
}

static int oplus_mt6360_set_termchg_current(int term_curr)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_set_eoc_current(chg, term_curr * 1000);
	if (rc < 0) {
		/*chg_debug("set termchg_current fail\n");*/
	}
	return 0;
}

static int oplus_mt6360_enable_charging(void)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	rc = charger_dev_enable(chg, true);
	if (rc < 0) {
		chg_debug("enable charging fail\n");
	}

	return 0;
}

static int oplus_mt6360_disable_charging(void)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	g_oplus_chip->charger_current_pre = -1;
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	if (pinfo->data.dual_charger_support) {
		rc = charger_dev_enable(chg, false);
		if (rc < 0) {
			chg_debug("disable charging fail\n");
		}
	} else {
		rc = charger_dev_enable(chg, false);
		if (rc < 0) {
			chg_debug("disable charging fail\n");
		}
	}

	return 0;
}

static int oplus_mt6360_check_charging_enable(void)
{
	return mt6360_check_charging_enable();
}

static int oplus_mt6360_suspend_charger(void)
{
	int rc = 0;

	rc = mt6360_suspend_charger(true);
	if (rc < 0) {
		chg_debug("suspend charger fail\n");
	}
	if (oplus_voocphy_get_bidirect_cp_support() == true && oplus_vooc_get_fastchg_started() == true) {
		mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_disable);
		mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		chr_err("[%s] chgvin_disable\n", __func__);
	}

	return 0;
}

static int oplus_mt6360_unsuspend_charger(void)
{
	int rc = 0;

	rc = mt6360_suspend_charger(false);
	if (rc < 0) {
		chg_debug("unsuspend charger fail\n");
	}
	if (oplus_voocphy_get_bidirect_cp_support() == true) {
		mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_enable);
		mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		chr_err("[%s] chgvin_enable\n", __func__);
	}

	return 0;
}

static int oplus_mt6360_set_rechg_voltage(int rechg_mv)
{
	int rc = 0;

	rc = mt6360_set_rechg_voltage(rechg_mv);
	if (rc < 0) {
		chg_debug("set rechg voltage fail:%d\n", rechg_mv);
	}
	return 0;
}

static int oplus_mt6360_reset_charger(void)
{
	int rc = 0;

	rc = mt6360_reset_charger();
	if (rc < 0) {
		chg_debug("reset charger fail\n");
	}
	return 0;
}

static int oplus_mt6360_registers_read_full(void)
{
	bool full = false;
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_is_charging_done(chg, &full);
	if (rc < 0) {
		chg_debug("registers read full  fail\n");
		full = false;
	} else {
		/*chg_debug("registers read full\n");*/
	}

	return full;
}

static int oplus_mt6360_otg_enable(void)
{
	return 0;
}

static int oplus_mt6360_otg_disable(void)
{
	return 0;
}

static int oplus_mt6360_set_chging_term_disable(void)
{
	int rc = 0;

	rc = mt6360_set_chging_term_disable(true);
	if (rc < 0) {
		chg_debug("disable chging_term fail\n");
	}
	return 0;
}

static bool oplus_mt6360_check_charger_resume(void)
{
	return true;
}

static int oplus_mt6360_get_chg_current_step(void)
{
	return 100;
}

static bool oplus_pd_without_usb(void)
{
	struct tcpc_device *tcpc;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc) {
		chg_err("get type_c_port0 fail\n");
		return true;
	}

	if (!tcpm_inquire_pd_connected(tcpc))
		return true;
	return (tcpm_inquire_dpm_flags(tcpc) &
			DPM_FLAGS_PARTNER_USB_COMM) ? false : true;
}

int mt_power_supply_type_check(void)
{
	int charger_type = POWER_SUPPLY_TYPE_UNKNOWN;

	switch(mt_get_charger_type()) {
	case CHARGER_UNKNOWN:
		break;
	case STANDARD_HOST:
		if (!oplus_pd_without_usb())
			charger_type = POWER_SUPPLY_TYPE_USB_PD_SDP;
		else
			charger_type = POWER_SUPPLY_TYPE_USB;
		break;
	case CHARGING_HOST:
		charger_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case NONSTANDARD_CHARGER:
	case STANDARD_CHARGER:
	case APPLE_2_1A_CHARGER:
	case APPLE_1_0A_CHARGER:
	case APPLE_0_5A_CHARGER:
		charger_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		break;
	}
	chg_debug("charger_type[%d]\n", charger_type);
	if (g_oplus_chip) {
		if ((g_oplus_chip->charger_type != charger_type) && g_oplus_chip->usb_psy) {
			if (g_oplus_chip->charger_type == POWER_SUPPLY_TYPE_USB &&
			    charger_type == POWER_SUPPLY_TYPE_USB_PD_SDP) {
				g_oplus_chip->charger_type = charger_type;
				oplus_chg_turn_on_charging(g_oplus_chip);
			}
			power_supply_changed(g_oplus_chip->usb_psy);
		}
	}

	return charger_type;
}

void mt_usb_connect_v1(void)
{
	return;
}

void mt_usb_disconnect_v1(void)
{
	return;
}

bool oplus_mt_get_vbus_status(void)
{
	bool vbus_status = false;
	static bool pre_vbus_status = false;
	int ret = 0;

	if (!g_oplus_chip || !pinfo || !pinfo->tcpc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: g_oplus_chip not ready!\n", __func__);
		return false;
	}

	if(g_oplus_chip->vbatt_num == 2
		&& tcpm_inquire_typec_attach_state(pinfo->tcpc) == TYPEC_ATTACHED_SRC) {
		return false;
	}

	ret = mt6360_get_vbus_rising();
	if (ret < 0) {
		if (g_oplus_chip && g_oplus_chip->unwakelock_chg == 1
				&& g_oplus_chip->charger_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			printk(KERN_ERR "[OPLUS_CHG][%s]: unwakelock_chg=1, use pre status\n", __func__);
			return pre_vbus_status;
		} else {
			return false;
		}
	}
	if (ret == 0)
		vbus_status = false;
	else
		vbus_status = true;
	pre_vbus_status = vbus_status;
	return vbus_status;
}

int oplus_battery_meter_get_battery_voltage(void)
{
	return 4000;
}

int get_rtc_spare_oplus_fg_value(void)
{
	return 0;
}

int set_rtc_spare_oplus_fg_value(int soc)
{
	return 0;
}

void oplus_mt_power_off(void)
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
			if (!(tcpc_dev->pd_wait_hard_reset_complete)
					&& !oplus_mt_get_vbus_status()) {
				kernel_power_off();
			}
		}
	} else {
		printk(KERN_ERR "[OPLUS_CHG][%s]: ac_online is true, return!\n", __func__);
	}
}

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int oplus_mt6360_chg_get_dyna_aicl_result(void)
{
	int rc = 0;
	int aicl_ma = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_get_input_current(chg, &aicl_ma);
	if (rc < 0) {
		chg_debug("get dyna aicl fail\n");
		return 500;
	}
	return aicl_ma / 1000;
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
/*====================================================================*/

int oplus_chg_get_main_ibat(void)
{
	int ibat = 0;
	int ret = 0;
	struct charger_device *chg = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		return -1;
	}

	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	ret = charger_dev_get_ibat(chg, &ibat);
	if (ret < 0) {
		pr_err("[%s] get ibat fail\n", __func__);
		return -1;
	}

	return ibat / 1000;
}

static void set_usbswitch_to_rxtx(struct oplus_chg_chip *chip)
{
	int ret = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	mutex_lock(&chip->normalchg_gpio.pinctrl_mutex);
	gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 1);
	ret = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.charger_gpio_as_output2);
	mutex_unlock(&chip->normalchg_gpio.pinctrl_mutex);
	if (ret < 0) {
		chg_err("failed to set pinctrl int\n");
	}
	chg_err("set_usbswitch_to_rxtx\n");
	return;
}

static void set_usbswitch_to_dpdm(struct oplus_chg_chip *chip)
{
	int ret = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
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
	chg_err("set_usbswitch_to_dpdm\n");
}
static bool chargerid_support = false;
static bool is_support_chargerid_check(void)
{
#ifdef CONFIG_OPLUS_CHECK_CHARGERID_VOLT
	return chargerid_support;
#else
	return false;
#endif
}

int mt_get_chargerid_volt(void)
{
	int chargerid_volt = 0;
	int rc = 0;

	if (!pinfo->charger_id_chan) {
                chg_err("charger_id_chan NULL\n");
                return 0;
        }

	if (is_support_chargerid_check() == true) {
		rc = iio_read_channel_processed(pinfo->charger_id_chan, &chargerid_volt);
		if (rc < 0) {
			chg_err("read charger_id_chan fail, rc=%d\n", rc);
			return 0;
		}
		chg_debug("chargerid_volt=%d\n", chargerid_volt);
	} else {
		chg_debug("is_support_chargerid_check=false!\n");
		return 0;
	}

	return chargerid_volt;
}

void mt_set_chargerid_switch_val(int value)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (is_support_chargerid_check() == false)
		return;

	if (chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return;
	}
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)
			|| IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output1)
			|| IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output2)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value == 1) {
		set_usbswitch_to_rxtx(chip);
	} else if (value == 0) {
		set_usbswitch_to_dpdm(chip);
	} else {
		/*do nothing*/
	}
	chg_debug("get_val=%d\n", gpio_get_value(chip->normalchg_gpio.chargerid_switch_gpio));

	return;
}

int mt_get_chargerid_switch_val(void)
{
	int gpio_status = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	if (is_support_chargerid_check() == false)
		return 0;

	if (chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return 0;
	}

	gpio_status = gpio_get_value(chip->normalchg_gpio.chargerid_switch_gpio);

	chg_debug("mt_get_chargerid_switch_val=%d\n", gpio_status);

	return gpio_status;
}

static int oplus_usb_switch_gpio_gpio_init(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
		chg_err("get normalchg_gpio.chargerid_switch_gpio pinctrl fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.charger_gpio_as_output1 =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
			"charger_gpio_as_output_low");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output1)) {
		chg_err("get charger_gpio_as_output_low fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.charger_gpio_as_output2 =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
			"charger_gpio_as_output_high");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output2)) {
		chg_err("get charger_gpio_as_output_high fail\n");
		return -EINVAL;
	}

	mutex_lock(&chip->normalchg_gpio.pinctrl_mutex);
	pinctrl_select_state(chip->normalchg_gpio.pinctrl,
			chip->normalchg_gpio.charger_gpio_as_output1);
	mutex_unlock(&chip->normalchg_gpio.pinctrl_mutex);

	return 0;
}

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

	chargerid_support = of_property_read_bool(node, "qcom,chargerid_support");
	if (chargerid_support == false) {
		chg_err("not support chargerid\n");
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

	return rc;
}

static bool oplus_shortc_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.shortc_gpio)) {
		return true;
	}

	return false;
}

static int oplus_shortc_gpio_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);

	chip->normalchg_gpio.shortc_active =
		pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "shortc_active");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.shortc_active)) {
		chg_err("get shortc_active fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.shortc_active);

	return 0;
}

#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
static bool oplus_chg_get_shortc_hw_gpio_status(void)
{
	bool shortc_hw_status = 1;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return shortc_hw_status;
	}

	if (oplus_shortc_check_is_gpio(chip) == true) {
		shortc_hw_status = !!(gpio_get_value(chip->normalchg_gpio.shortc_gpio));
	}
	return shortc_hw_status;
}
#else /* CONFIG_OPLUS_SHORT_HW_CHECK */
static bool oplus_chg_get_shortc_hw_gpio_status(void)
{
	bool shortc_hw_status = 1;

	return shortc_hw_status;
}
#endif /* CONFIG_OPLUS_SHORT_HW_CHECK */

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

static bool oplus_ship_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.ship_gpio))
		return true;

	return false;
}

static int oplus_ship_gpio_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}

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
static void smbchg_enter_shipmode(struct oplus_chg_chip *chip)
{
	int i = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (oplus_ship_check_is_gpio(chip) == true) {
		chg_err("select gpio control\n");
		if (!IS_ERR_OR_NULL(chip->normalchg_gpio.ship_active) && !IS_ERR_OR_NULL(chip->normalchg_gpio.ship_sleep)) {
			pinctrl_select_state(chip->normalchg_gpio.pinctrl,
				chip->normalchg_gpio.ship_sleep);
			for (i = 0; i < PWM_COUNT; i++) {
				pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_active);
				mdelay(3);
				pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_sleep);
				mdelay(3);
			}
		}
		chg_err("power off after 15s\n");
	} else {
		if (!chip->disable_ship_mode) {
			mt6360_enter_shipmode();
			chg_err("power off after 18s\n");
		}
	}
}
static void enter_ship_mode_function(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (chip->enable_shipmode) {
		smbchg_enter_shipmode(chip);
		printk(KERN_ERR "[OPLUS_CHG][%s]: enter_ship_mode_function\n", __func__);
	}
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

#define CHARGER_25C_VOLT	2457/*mv*/
static int oplus_get_ntc_temp(struct iio_channel *ntc_temp_chan, int *ntc_temp)
{
	int rc = 0;
	int ntc_temp_volt = 0;
	int i = 0;

	if (!ntc_temp_chan || !ntc_temp) {
		chg_err("ntc_temp_chan or ntc_temp NULL\n");
		return -EINVAL;
	}

	rc = iio_read_channel_processed(ntc_temp_chan, &ntc_temp_volt);
	if ((rc < 0) || (ntc_temp_volt < 0)) {
		chg_err("read ntc_temp_chan volt failed, rc=%d\n", rc);
		ntc_temp_volt = CHARGER_25C_VOLT;
	}
	for (i = ARRAY_SIZE(con_volt_20131) - 1; i >= 0; i--) {
		if (con_volt_20131[i] >= ntc_temp_volt)
			break;
		else if (i == 0)
			break;
	}

	*ntc_temp = con_temp_20131[i];

	return 0;
}

int oplus_chg_get_battery_btb_temp_cal(void)
{
	int rc = 0;
	int temp = 25;
	if (!pinfo || !pinfo->batcon_temp_chan) {
		chg_err("pinfo or batcon_temp_chan is NULL\n");
		return 25;
	}

	rc = oplus_get_ntc_temp(pinfo->batcon_temp_chan, &temp);
	if (rc < 0)
		return 25;

	return temp;
}
EXPORT_SYMBOL(oplus_chg_get_battery_btb_temp_cal);

int oplus_chg_get_usb_btb_temp_cal(void)
{
	int rc = 0;
	int temp = 25;
	if (!pinfo || !pinfo->usbcon_temp_chan) {
		chg_err("pinfo or usbcon_temp_chan is NULL\n");
		return 25;
	}

	rc = oplus_get_ntc_temp(pinfo->usbcon_temp_chan, &temp);
	if (rc < 0)
		return 25;

	return temp;
}
EXPORT_SYMBOL(oplus_chg_get_usb_btb_temp_cal);


static void oplus_get_chargeric_temp_volt(struct charger_data *pdata)
{
	int rc = 0;
	int chargeric_temp_volt = 0;

	if (!pdata) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: charger_data not ready!\n", __func__);
		return;
	}

	if (!pinfo->chargeric_temp_chan) {
		chg_err("chargeric_temp_chan NULL\n");
		return;
	}

	rc = iio_read_channel_processed(pinfo->chargeric_temp_chan, &chargeric_temp_volt);
	if ((rc < 0) || (chargeric_temp_volt < 0)) {
		chg_err("read chargeric_temp_chan volt failed, rc=%d\n", rc);
		chargeric_temp_volt = CHARGER_25C_VOLT;
	}
	pdata->chargeric_temp_volt = chargeric_temp_volt;

	return;
}

static void get_chargeric_temp(struct charger_data *pdata)
{
	int i = 0;
	int step_vol = 0;
	int temp_offset = 0;

	if (!pdata) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: charger_data not ready!\n", __func__);
		return;
	}

	for (i = ARRAY_SIZE(con_volt_20131) - 1; i >= 0; i--) {
		if (con_volt_20131[i] >= pdata->chargeric_temp_volt)
			break;
		else if (i == 0)
			break;
	}
	pdata->chargeric_temp = con_temp_20131[i];
	if (pinfo && pinfo->support_ntc_01c_precision) {
		if((0 == i) || (ARRAY_SIZE(con_volt_20131) - 1 == i)) { /* avoid array out of bounds */
			pdata->chargeric_temp = pdata->chargeric_temp * 10;
		} else {
			pdata->chargeric_temp = pdata->chargeric_temp * 10;
			step_vol = ((con_volt_20131[i] - con_volt_20131[i + 1]) * 1000)/ 10;
			temp_offset = ((con_volt_20131[i] - pdata->chargeric_temp_volt) * 1000) / step_vol;
			pdata->chargeric_temp += temp_offset;
		}
	}
	/*chg_debug("chargeric_temp:%d con_volt_20131[%d] = %d con_temp_20131[%d] = %d chargeric_temp_volt = %d step_vol = %d temp_offset = %d\n",
		pdata->chargeric_temp, i, con_volt_20131[i], i, con_temp_20131[i], pdata->chargeric_temp_volt, step_vol, temp_offset);*/

	return;
}

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

bool oplus_ccdetect_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->chgic_mtk.oplus_info->ccdetect_gpio)) {
		return true;
	}

	return false;
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

	chg_err("not support, return false\n");

	return false;
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
struct delayed_work usbtemp_recover_work;
void oplus_ccdetect_work(struct work_struct *work)
{
	int level;
	struct oplus_chg_chip *chip = g_oplus_chip;
	level = gpio_get_value(chip->chgic_mtk.oplus_info->ccdetect_gpio);
	pr_err("%s: level [%d]", __func__, level);
	if (level != 1) {
		oplus_ccdetect_enable();
        oplus_wake_up_usbtemp_thread();
	} else {
		oplus_chg_clear_abnormal_adapter_var();
		if (g_oplus_chip)
			g_oplus_chip->usbtemp_check = oplus_usbtemp_condition();

		if(g_oplus_chip->usb_status == USB_TEMP_HIGH) {
			oplus_usbtemp_recover_cc_open();
		}
		if (oplus_get_otg_switch_status() == false) {
			oplus_ccdetect_disable();
		}
		if(g_oplus_chip->usb_status == USB_TEMP_HIGH) {
			schedule_delayed_work(&usbtemp_recover_work, 0);
		}
	}
}

void oplus_usbtemp_recover_work(struct work_struct *work)
{
	oplus_usbtemp_recover_func(g_oplus_chip);
}

void oplus_ccdetect_irq_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}
	chip->chgic_mtk.oplus_info->ccdetect_irq = gpio_to_irq(chip->chgic_mtk.oplus_info->ccdetect_gpio);
	printk(KERN_ERR "[OPLUS_CHG][%s]: chip->chgic_mtk.oplus_info->ccdetect_gpio[%d]!\n", __func__, chip->chgic_mtk.oplus_info->ccdetect_gpio);
}

#define CCDETECT_DELAY_MS	50
struct delayed_work ccdetect_work;
irqreturn_t oplus_ccdetect_change_handler(int irq, void *data)
{
	cancel_delayed_work_sync(&ccdetect_work);
	printk(KERN_ERR "[OPLUS_CHG][%s]: Scheduling ccdetect work!\n", __func__);
	schedule_delayed_work(&ccdetect_work,
			msecs_to_jiffies(CCDETECT_DELAY_MS));
	return IRQ_HANDLED;
}

/**************************************************************
 * bit[0]=0: NO standard typec device/cable connected(ccdetect gpio in high level)
 * bit[0]=1: standard typec device/cable connected(ccdetect gpio in low level)
 * bit[1]=0: NO OTG typec device/cable connected
 * bit[1]=1: OTG typec device/cable connected
 **************************************************************/
#define DISCONNECT						0
#define STANDARD_TYPEC_DEV_CONNECT	BIT(0)
#define OTG_DEV_CONNECT				BIT(1)

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
		printk(KERN_ERR "[OPLUS_CHG][%s]: g_oplus_chip not ready!\n", __func__);
		return false;
	}

	if (oplus_ccdetect_check_is_gpio(chip) == true) {
		level = gpio_get_value(chip->chgic_mtk.oplus_info->ccdetect_gpio);
		if (level != gpio_get_value(chip->chgic_mtk.oplus_info->ccdetect_gpio)) {
			printk(KERN_ERR "[OPLUS_CHG][%s]: ccdetect_gpio is unstable, try again...\n", __func__);
			usleep_range(5000, 5100);
			level = gpio_get_value(chip->chgic_mtk.oplus_info->ccdetect_gpio);
		}
	} else {
		return oplus_get_otg_online_status_default();
	}
	online = (level == 1) ? DISCONNECT : STANDARD_TYPEC_DEV_CONNECT;

	if (tcpm_inquire_typec_attach_state(pinfo->tcpc) == TYPEC_ATTACHED_SRC) {
		typec_otg = 1;
	} else {
		typec_otg = 0;
	}
	online = online | ((typec_otg == 1) ? OTG_DEV_CONNECT : DISCONNECT);

	if ((pre_level ^ level) || (pre_typec_otg ^ typec_otg)) {
		pre_level = level;
		pre_typec_otg = typec_otg;
		printk(KERN_ERR "[OPLUS_CHG][%s]: gpio[%s], c-otg[%d], otg_online[%d]\n",
				__func__, level ? "H" : "L", typec_otg, online);
	}

	chip->otg_online = typec_otg;
	return online;
}


bool oplus_get_otg_switch_status(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	return chip->otg_switch;
}

int oplus_get_typec_cc_orientation(void)
{
	int val = 0;

	if (pinfo != NULL && pinfo->tcpc != NULL) {
		if (tcpm_inquire_typec_attach_state(pinfo->tcpc) != TYPEC_UNATTACHED) {
			val = (int)tcpm_inquire_cc_polarity(pinfo->tcpc) + 1;
		} else {
			val = 0;
		}
		if (val != 0)
			printk(KERN_ERR "[OPLUS_CHG][%s]: cc[%d]\n", __func__, val);
	} else {
		val = 0;
	}

	return val;
}

void oplus_ccdetect_irq_register(struct oplus_chg_chip *chip)
{
	int ret = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	ret = devm_request_threaded_irq(chip->dev,  chip->chgic_mtk.oplus_info->ccdetect_irq,
			NULL, oplus_ccdetect_change_handler, IRQF_TRIGGER_FALLING
			| IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ccdetect-change", chip);
	if (ret < 0) {
		chg_err("Unable to request ccdetect-change irq: %d\n", ret);
	}
	printk(KERN_ERR "%s: !!!!! irq register\n", __FUNCTION__);

	ret = enable_irq_wake(chip->chgic_mtk.oplus_info->ccdetect_irq);
	if (ret != 0) {
		chg_err("enable_irq_wake: ccdetect_irq failed %d\n", ret);
	}
}

void oplus_ccdetect_enable(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (oplus_ccdetect_check_is_gpio(chip) != true)
		return;

	if (oplus_chg_get_dischg_flag()) {
		return;
	}
	/* set DRP mode */
	if (pinfo != NULL && pinfo->tcpc != NULL) {
		tcpm_typec_change_role(pinfo->tcpc, TYPEC_ROLE_TRY_SNK);
		pr_err("%s: set drp", __func__);
	}
}

void oplus_ccdetect_disable(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (oplus_ccdetect_check_is_gpio(chip) != true)
		return;

	/* set SINK mode */
	if (pinfo != NULL && pinfo->tcpc != NULL) {
		tcpm_typec_change_role(pinfo->tcpc, TYPEC_ROLE_SNK);
		pr_err("%s: set sink", __func__);
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

#define USBTEMP_DEFAULT_VOLT_VALUE_MV 950
void oplus_get_usbtemp_volt(struct oplus_chg_chip *chip)
{
	int rc, usbtemp_volt = 0;
	static int usbtemp_volt_l_pre = USBTEMP_DEFAULT_VOLT_VALUE_MV;
	static int usbtemp_volt_r_pre = USBTEMP_DEFAULT_VOLT_VALUE_MV;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb5_chg not ready!\n", __func__);
		return;
	}

	if (IS_ERR_OR_NULL(pinfo->usb_temp_v_l_chan)) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: pinfo->usb_temp_v_l_chan  is  NULL !\n", __func__);
		chip->usbtemp_volt_l = usbtemp_volt_l_pre;
		goto usbtemp_next;
	}

	rc = iio_read_channel_processed(pinfo->usb_temp_v_l_chan, &usbtemp_volt);
	if (rc < 0) {
		chg_err("[OPLUS_CHG][%s]: iio_read_channel_processed  get error\n", __func__);
		chip->usbtemp_volt_l = usbtemp_volt_l_pre;
		goto usbtemp_next;
	}
	chip->usbtemp_volt_l = usbtemp_volt;
	usbtemp_volt_l_pre = chip->usbtemp_volt_l;
usbtemp_next:
	usbtemp_volt = 0;
	if (IS_ERR_OR_NULL(pinfo->usb_temp_v_r_chan)) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: pinfo->usb_temp_v_r_chan  is  NULL !\n", __func__);
		chip->usbtemp_volt_r = usbtemp_volt_r_pre;
		return;
	}

	rc = iio_read_channel_processed(pinfo->usb_temp_v_r_chan, &usbtemp_volt);
	if (rc < 0) {
		chg_err("[OPLUS_CHG][%s]: iio_read_channel_processed  get error\n", __func__);
		chip->usbtemp_volt_r = usbtemp_volt_r_pre;
		return;
	}
	chip->usbtemp_volt_r = usbtemp_volt;
	usbtemp_volt_r_pre = chip->usbtemp_volt_r;

	return;
}

#define SUBBOARD_DEFAULT_VOLT_VALUE_MV 600
static int oplus_get_temp_volt(struct ntc_temp *ntc_param)
{
	int rc = 0;
	int sub_temp_volt = 0;
	struct iio_channel		*temp_chan = NULL;
	static int sub_temp_volt_pre = SUBBOARD_DEFAULT_VOLT_VALUE_MV;

	if (!ntc_param || !pinfo) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: ntc_param || pinfo not ready!\n", __func__);
		return -1;
	}

	switch (ntc_param->e_ntc_type) {
	case NTC_SUB_BOARD:
		if (!pinfo->subboard_temp_chan) {
			printk(KERN_ERR "[OPLUS_CHG][%s]: pinfo->subboard_temp_chan  is  NULL !\n", __func__);
			sub_temp_volt = sub_temp_volt_pre;
			goto subboardtemp_next;
		}
		temp_chan = pinfo->subboard_temp_chan;
		break;
	default:
	break;
	}

	if (!temp_chan) {
		chg_err("unsupported ntc %d\n", ntc_param->e_ntc_type);
		return -1;
	}
	rc = iio_read_channel_processed(temp_chan, &sub_temp_volt);
	if (rc < 0) {
		chg_err("[OPLUS_CHG][%s]: iio_read_channel_processed  get error\n", __func__);
		sub_temp_volt = sub_temp_volt_pre;
		goto subboardtemp_next;
	}

	if (sub_temp_volt > ntc_param->i_rap_pull_up_voltage) {
		sub_temp_volt = sub_temp_volt / 1000;
	}
	sub_temp_volt_pre = sub_temp_volt;
subboardtemp_next:
	/*chg_err("subboard_temp_chan:%d\n", sub_temp_volt);*/
	return sub_temp_volt;
}

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

	return chg_tmp;
}

int oplus_force_get_subboard_temp(void)
{
	int subboard_temp = 0;
	static bool is_param_init = false;
	static struct ntc_temp ntc_param = {0};

	if (!pinfo || !pinfo->subboard_temp_chan) {
		chg_err("null pinfo\n");
		return TEMP_25C;
	}

	if (!is_param_init) {
		ntc_param.e_ntc_type = NTC_SUB_BOARD;
		ntc_param.i_tap_over_critical_low = 4397119;
		ntc_param.i_rap_pull_up_r = g_oplus_chip->chgic_mtk.sub_board_pull_up_r;
		ntc_param.i_rap_pull_up_voltage = 1800;
		ntc_param.i_tap_min = -400;
		ntc_param.i_tap_max = 1250;
		ntc_param.i_25c_volt = 2457;
		ntc_param.pst_temp_table = sub_board_temp_table;
		ntc_param.i_table_size = (sizeof(sub_board_temp_table) / sizeof(struct temp_param));
		is_param_init = true;

		/* chg_err("get_PCB_Version() = %d\n",get_PCB_Version()); */
		chg_err("ntc_type:%d,critical_low:%d,pull_up_r=%d,pull_up_voltage=%d,tap_min=%d,tap_max=%d,table_size=%d\n",
			ntc_param.e_ntc_type, ntc_param.i_tap_over_critical_low, ntc_param.i_rap_pull_up_r,
			ntc_param.i_rap_pull_up_voltage, ntc_param.i_tap_min, ntc_param.i_tap_max, ntc_param.i_table_size);
	}
	ntc_param.ui_dwvolt = oplus_get_temp_volt(&ntc_param);
	subboard_temp = oplus_res_to_temp(&ntc_param);

	pinfo->i_sub_board_temp = subboard_temp;
	/*chg_err("subboard_temp:%d\n", subboard_temp);*/
	return pinfo->i_sub_board_temp;
}

int oplus_chg_get_subboard_temp_cal(void)
{
	return oplus_force_get_subboard_temp();
}
EXPORT_SYMBOL(oplus_chg_get_subboard_temp_cal);

static bool oplus_chg_get_vbus_status(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	return chip->charger_exist;
}

static void oplus_usbtemp_thread_init(void)
{
	if (g_oplus_chip->support_usbtemp_protect_v2)
		oplus_usbtemp_kthread =
			kthread_run(oplus_usbtemp_monitor_common_new_method, g_oplus_chip, "usbtemp_kthread");
	else
		oplus_usbtemp_kthread =
			kthread_run(oplus_usbtemp_monitor_common, g_oplus_chip, "usbtemp_kthread");
	if (IS_ERR(oplus_usbtemp_kthread)) {
		chg_err("failed to cread oplus_usbtemp_kthread\n");
	}
}

void oplus_wake_up_usbtemp_thread(void)
{
	if (oplus_usbtemp_check_is_support() == true) {
		g_oplus_chip->usbtemp_check = oplus_usbtemp_condition();
		if (g_oplus_chip->usbtemp_check) {
			if (g_oplus_chip->support_usbtemp_protect_v2)
				wake_up_interruptible(&g_oplus_chip->oplus_usbtemp_wq_new_method);
			else
				wake_up_interruptible(&g_oplus_chip->oplus_usbtemp_wq);
		}
	}
}

static int oplus_chg_usbtemp_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;

	if (chip)
		node = chip->dev->of_node;
	if (node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
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
			}
		}
		chg_err("dischg-gpio:%d\n", chip->normalchg_gpio.dischg_gpio);
	}

	return rc;
}

static bool oplus_mtk_hv_flashled_check_is_gpio(void)
{
	if (gpio_is_valid(mtkhv_flashled_pinctrl.chgvin_gpio) && gpio_is_valid(mtkhv_flashled_pinctrl.pmic_chgfunc_gpio)) {
		return true;
	} else {
		chg_err("[%s] fail\n", __func__);
		return false;
	}
}
static int oplus_mtk_hv_flashled_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;

	if (chip != NULL)
		node = chip->dev->of_node;

	if (chip == NULL || node == NULL) {
		chg_err("[%s] oplus_chip or device tree info. missing\n", __func__);
		return -EINVAL;
	}

	mtkhv_flashled_pinctrl.hv_flashled_support = of_property_read_bool(node, "qcom,hv_flashled_support");
	if (mtkhv_flashled_pinctrl.hv_flashled_support == false) {
		chg_err("[%s] hv_flashled_support not support\n", __func__);
		return -EINVAL;
	}
	mutex_init(&mtkhv_flashled_pinctrl.chgvin_mutex);

	mtkhv_flashled_pinctrl.chgvin_gpio = of_get_named_gpio(node, "qcom,chgvin", 0);
	mtkhv_flashled_pinctrl.pmic_chgfunc_gpio = of_get_named_gpio(node, "qcom,pmic_chgfunc", 0);
	if (mtkhv_flashled_pinctrl.chgvin_gpio <= 0 || mtkhv_flashled_pinctrl.pmic_chgfunc_gpio <= 0) {
		chg_err("read dts fail %d %d\n", mtkhv_flashled_pinctrl.chgvin_gpio, mtkhv_flashled_pinctrl.pmic_chgfunc_gpio);
	} else {
		if (oplus_mtk_hv_flashled_check_is_gpio() == true) {
			rc = gpio_request(mtkhv_flashled_pinctrl.chgvin_gpio, "chgvin");
			if (rc) {
				chg_err("unable to request chgvin:%d\n",
						mtkhv_flashled_pinctrl.chgvin_gpio);
			} else {
				mtkhv_flashled_pinctrl.pinctrl = devm_pinctrl_get(chip->dev);
				/*chgvin*/
				mtkhv_flashled_pinctrl.chgvin_enable =
					pinctrl_lookup_state(mtkhv_flashled_pinctrl.pinctrl, "chgvin_enable");
				if (IS_ERR_OR_NULL(mtkhv_flashled_pinctrl.chgvin_enable)) {
					chg_err("get chgvin_enable fail\n");
					return -EINVAL;
				}

				mtkhv_flashled_pinctrl.chgvin_disable =
					pinctrl_lookup_state(mtkhv_flashled_pinctrl.pinctrl, "chgvin_disable");
				if (IS_ERR_OR_NULL(mtkhv_flashled_pinctrl.chgvin_disable)) {
					chg_err("get chgvin_disable fail\n");
					return -EINVAL;
				}
				pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_enable);


				rc = gpio_request(mtkhv_flashled_pinctrl.pmic_chgfunc_gpio, "pmic_chgfunc");
				if (rc) {
					chg_err("unable to request pmic_chgfunc:%d\n",
							mtkhv_flashled_pinctrl.pmic_chgfunc_gpio);
				} else {
					/*pmic_chgfunc*/
					mtkhv_flashled_pinctrl.pmic_chgfunc_enable =
						pinctrl_lookup_state(mtkhv_flashled_pinctrl.pinctrl, "pmic_chgfunc_enable");
					if (IS_ERR_OR_NULL(mtkhv_flashled_pinctrl.pmic_chgfunc_enable)) {
						chg_err("get pmic_chgfunc_enable fail\n");
						return -EINVAL;
					}

					mtkhv_flashled_pinctrl.pmic_chgfunc_disable =
						pinctrl_lookup_state(mtkhv_flashled_pinctrl.pinctrl, "pmic_chgfunc_disable");
					if (IS_ERR_OR_NULL(mtkhv_flashled_pinctrl.pmic_chgfunc_disable)) {
						chg_err("get pmic_chgfunc_disable fail\n");
						return -EINVAL;
					}
					pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.pmic_chgfunc_disable);
				}
			}
		}
	}

	chg_err("mtk_hv_flash_led:%d\n", rc);
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

	rc = oplus_mtk_hv_flashled_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_mtk_hv_flashled_dt fail!\n", __func__);
		return -EINVAL;
	}
	return rc;
}

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

	return rc;
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

static int battery_prop_is_writeable(struct power_supply *psy,
	enum power_supply_property psp)
{
	return oplus_battery_property_is_writeable(psy, psp);
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
				chg_err("bat pro POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL, should shutdown!!!\n");
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

static const enum power_supply_property mt_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const enum power_supply_property mt_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static const enum power_supply_property battery_properties[] = {
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

	if (chip == NULL) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}
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
	mt_chg->usb_cfg.drv_data = mt_chg;

	mt_chg->battery_psd.name = "battery";
	mt_chg->battery_psd.type = POWER_SUPPLY_TYPE_BATTERY;
	mt_chg->battery_psd.properties = battery_properties;
	mt_chg->battery_psd.num_properties = ARRAY_SIZE(battery_properties);
	mt_chg->battery_psd.get_property = battery_get_property;
	mt_chg->battery_psd.set_property = battery_set_property;
	mt_chg->battery_psd.property_is_writeable = battery_prop_is_writeable;

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

err_battery_psy:
	power_supply_unregister(mt_chg->usb_psy);
err_usb_psy:
	power_supply_unregister(mt_chg->ac_psy);
err_ac_psy:

	return ret;
}

void oplus_set_otg_switch_status(bool value)
{
	if (pinfo != NULL && pinfo->tcpc != NULL) {
		if(oplus_ccdetect_check_is_gpio(g_oplus_chip) == true) {
			if(gpio_get_value(g_oplus_chip->chgic_mtk.oplus_info->ccdetect_gpio) == 0) {
				printk(KERN_ERR "[OPLUS_CHG][oplus_set_otg_switch_status]: gpio[L], should set, return\n");
				return;
			}
		}
		printk(KERN_ERR "[OPLUS_CHG][%s]: otg switch[%d]\n", __func__, value);
		tcpm_typec_change_role(pinfo->tcpc, value ? TYPEC_ROLE_TRY_SNK : TYPEC_ROLE_SNK);
	}
}
EXPORT_SYMBOL(oplus_set_otg_switch_status);

int oplus_chg_get_mmi_status(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 1;
	}
	if (chip->mmi_chg == 0)
		printk(KERN_ERR "[OPLUS_CHG][%s]: mmi_chg[%d]\n", __func__, chip->mmi_chg);
	return chip->mmi_chg;
}
EXPORT_SYMBOL(oplus_chg_get_mmi_status);

#define VBUS_9V	9000
#define VBUS_5V	5000
#define IBUS_2A	2000
#define IBUS_3A	3000
int oplus_chg_get_pd_type(void)
{
	if (pinfo != NULL) {
		chg_err("pd_type: %d\n", pinfo->pd_type);
		if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK ||
				pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30) {
			return PD_ACTIVE;
		} else if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
			if (oplus_pps_get_chg_status() != PPS_NOT_SUPPORT) {
				return PD_PPS_ACTIVE;
			} else {
				return PD_ACTIVE;
			}
		} else {
			return PD_INACTIVE;
		}
	}

	return PD_INACTIVE;
}
EXPORT_SYMBOL(oplus_chg_get_pd_type);

int oplus_check_cc_mode(void) {
	const char *tcpc_name = "type_c_port0";
	struct tcpc_device *tcpc_dev;

	tcpc_dev = tcpc_dev_get_by_name(tcpc_name);
	if (IS_ERR_OR_NULL(tcpc_dev)) {
		chg_err("tcpc info error\n");
		return -EINVAL;
	}

	return tcpm_inquire_typec_role(tcpc_dev);
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

int oplus_pdc_get(int *vbus_mv, int *ibus_ma)
{
	int ret = 0;
	struct tcpc_device *tcpc = NULL;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		printk(KERN_ERR "%s:get type_c_port0 fail\n", __func__);
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(tcpc, vbus_mv, ibus_ma);
	if (ret != TCPM_SUCCESS) {
		printk(KERN_ERR "%s: inquire current vbus_mv and ibus_ma fail\n", __func__);
		return -EINVAL;
	}
	printk(KERN_ERR "%s: default vbus_mv[%d], ibus_ma[%d]\n", __func__, *vbus_mv, *ibus_ma);
	return 0;
}

int oplus_mt6360_pd_setup_forsvooc(void)
{
	int vbus_mv = VBUS_5V;
	int ibus_ma = IBUS_2A;
	int ret = -1;
	struct adapter_power_cap cap;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int i;

	if(chip->pd_svooc) {
		pr_err("%s pd_svooc support\n", __func__);
		return 0;
	}

	cap.nr = 0;
	cap.pdp = 0;
	for (i = 0; i < ADAPTER_CAP_MAX_NR; i++) {
		cap.max_mv[i] = 0;
		cap.min_mv[i] = 0;
		cap.ma[i] = 0;
		cap.type[i] = 0;
		cap.pwr_limit[i] = 0;
	}

	printk(KERN_ERR "%s: pd_type %d pd9v svooc [%d %d %d]", __func__, pinfo->pd_type, chip->limits.vbatt_pdqc_to_9v_thr,
		chip->limits.vbatt_pdqc_to_5v_thr, chip->batt_volt);
	if (oplus_chg_get_voocphy_support() == AP_SINGLE_CP_VOOCPHY
		|| oplus_chg_get_voocphy_support() == AP_DUAL_CP_VOOCPHY) {
		chip->charger_volt = chip->chg_ops->get_charger_volt();
		if (!chip->calling_on && !chip->camera_on && chip->charger_volt < 6500 && chip->soc < 90
				&& chip->temperature <= 420 && chip->cool_down_force_5v == false) {
			oplus_voocphy_set_pdqc_config();
			if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
				adapter_dev_get_cap(pinfo->adapter_dev[PD], MTK_PD_APDO, &cap);
				for (i = 0; i < cap.nr; i++) {
					printk(KERN_ERR "PD APDO cap %d: mV:%d,%d mA:%d type:%d pwr_limit:%d pdp:%d\n", i,
						cap.max_mv[i], cap.min_mv[i], cap.ma[i],
						cap.type[i], cap.pwr_limit[i], cap.pdp);
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
			} else if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK
				|| pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30) {
				adapter_dev_get_cap(pinfo->adapter_dev[PD], MTK_PD, &cap);
				for (i = 0; i < cap.nr; i++) {
					printk(KERN_ERR "PD cap %d: mV:%d,%d mA:%d type:%d\n", i,
						cap.max_mv[i], cap.min_mv[i], cap.ma[i], cap.type[i]);
				}

				for (i = 0; i < cap.nr; i++) {
					if (VBUS_9V <= cap.max_mv[i]) {
						vbus_mv = cap.max_mv[i];
						ibus_ma = cap.ma[i];
						if (ibus_ma > IBUS_2A)
							ibus_ma = IBUS_2A;
						break;
					}
				}
			} else {
				vbus_mv = VBUS_5V;
				ibus_ma = IBUS_2A;
			}

			printk(KERN_ERR "PD request: %dmV, %dmA, pdqc_9v_voltage_adaptive:%d\n", vbus_mv, ibus_ma, chip->pdqc_9v_voltage_adaptive);
			ret = oplus_pdc_setup(&vbus_mv, &ibus_ma);
			if (chip->pdqc_9v_voltage_adaptive) {
				msleep(300);
				chip->charger_volt = chip->chg_ops->get_charger_volt();
				if ((vbus_mv >= VBUS_9V) && (chip->charger_volt > 7500)) {
					oplus_chg_pdqc9v_vindpm_vol_switch(OPLUS_PDQC_5VTO9V);
				} else {
					oplus_chg_pdqc9v_vindpm_vol_switch(OPLUS_PDQC_9VTO5V);
				}
			}
		} else {
			if (chip->charger_volt > 7500 &&
					(chip->calling_on || chip->camera_on || chip->soc >= 90 || chip->batt_volt >= 4450
					|| chip->temperature > 420 || chip->cool_down_force_5v == true)) {
				vbus_mv = VBUS_5V;
				ibus_ma = IBUS_3A;

				printk(KERN_ERR "PD request: %dmV, %dmA\n", vbus_mv, ibus_ma);
				ret = oplus_pdc_setup(&vbus_mv, &ibus_ma);
				if (chip->pdqc_9v_voltage_adaptive) {
					oplus_chg_pdqc9v_vindpm_vol_switch(OPLUS_PDQC_9VTO5V);
				}
			}

			printk(KERN_ERR "%s: pd9v svooc  default[%d %d]", __func__, em_mode, chip->batt_volt);
		}
	} else {
		if (!chip->calling_on && chip->charger_volt < 6500 && chip->soc < 90
				&& chip->temperature <= 530 && chip->cool_down_force_5v == false
				&& (chip->batt_volt < chip->limits.vbatt_pdqc_to_9v_thr)) {
			if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
				adapter_dev_get_cap(pinfo->adapter_dev[PD], MTK_PD_APDO, &cap);
				for (i = 0; i < cap.nr; i++) {
					printk(KERN_ERR "PD APDO cap %d: mV:%d,%d mA:%d type:%d pwr_limit:%d pdp:%d\n", i,
							cap.max_mv[i], cap.min_mv[i], cap.ma[i],
							cap.type[i], cap.pwr_limit[i], cap.pdp);
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
			} else if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK
				|| pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30) {
				adapter_dev_get_cap(pinfo->adapter_dev[PD], MTK_PD, &cap);
				for (i = 0; i < cap.nr; i++) {
					printk(KERN_ERR "PD cap %d: mV:%d,%d mA:%d type:%d\n", i,
						cap.max_mv[i], cap.min_mv[i], cap.ma[i], cap.type[i]);
				}

				for (i = 0; i < cap.nr; i++) {
					if (VBUS_9V <= cap.max_mv[i]) {
						vbus_mv = cap.max_mv[i];
						ibus_ma = cap.ma[i];
						if (ibus_ma > IBUS_2A)
							ibus_ma = IBUS_2A;
						break;
					}
				}
			} else {
				vbus_mv = VBUS_5V;
				ibus_ma = IBUS_2A;
			}
			oplus_chg_suspend_charger();
			oplus_chg_config_charger_vsys_threshold(0x02);/*set Vsys Skip threshold 104%*/
			oplus_chg_enable_burst_mode(false);
			ret = oplus_pdc_setup(&vbus_mv, &ibus_ma);
			printk(KERN_ERR "%s: PD request: vbus[%d], ibus[%d], ret[%d]\n", __func__, vbus_mv, ibus_ma, ret);
			msleep(300);
			oplus_chg_unsuspend_charger();
		} else {
			if (chip->charger_volt > 7500 &&
				(chip->calling_on || chip->soc >= 90 || chip->batt_volt >= chip->limits.vbatt_pdqc_to_5v_thr
				|| chip->temperature > 530 || chip->cool_down_force_5v == true)) {
				vbus_mv = VBUS_5V;
				ibus_ma = IBUS_3A;

				chip->chg_ops->input_current_write(500);
				oplus_chg_suspend_charger();
				oplus_chg_config_charger_vsys_threshold(0x03);/*set Vsys Skip threshold 101%*/
				ret = oplus_pdc_setup(&vbus_mv, &ibus_ma);
				printk(KERN_ERR "%s: PD request:vbus[%d], ibus[%d], ret[%d]\n", __func__, vbus_mv, ibus_ma, ret);
				msleep(300);
				printk(KERN_ERR "%s: charger voltage=%d", __func__, chip->charger_volt);
				oplus_chg_unsuspend_charger();
			}

			printk(KERN_ERR "%s: pd9v svooc  default[%d %d]", __func__, em_mode, chip->batt_volt);
		}
	}

	return ret;
}

int oplus_mt6360_pd_setup(void)
{
	int vbus_mv = VBUS_5V;
	int ibus_ma = IBUS_2A;
	int ret = -1;
	struct adapter_power_cap cap;
	struct oplus_chg_chip *chip = g_oplus_chip;
	int i;

	if (is_mtksvooc_project) {
		ret = oplus_mt6360_pd_setup_forsvooc();
	} else {
		cap.nr = 0;
		cap.pdp = 0;
		for (i = 0; i < ADAPTER_CAP_MAX_NR; i++) {
			cap.max_mv[i] = 0;
			cap.min_mv[i] = 0;
			cap.ma[i] = 0;
			cap.type[i] = 0;
			cap.pwr_limit[i] = 0;
		}

		printk(KERN_ERR "pd_type: %d\n", pinfo->pd_type);

		if (!chip->calling_on && !chip->camera_on && chip->charger_volt < 6500 && chip->soc < 90
			&& chip->temperature <= 420 && chip->cool_down_force_5v == false) {
			if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
				adapter_dev_get_cap(pinfo->adapter_dev[PD], MTK_PD_APDO, &cap);
				for (i = 0; i < cap.nr; i++) {
					printk(KERN_ERR "PD APDO cap %d: mV:%d,%d mA:%d type:%d pwr_limit:%d pdp:%d\n", i,
						cap.max_mv[i], cap.min_mv[i], cap.ma[i],
						cap.type[i], cap.pwr_limit[i], cap.pdp);
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
			} else if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK
				|| pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30) {
				adapter_dev_get_cap(pinfo->adapter_dev[PD], MTK_PD, &cap);
				for (i = 0; i < cap.nr; i++) {
					printk(KERN_ERR "PD cap %d: mV:%d,%d mA:%d type:%d\n", i,
						cap.max_mv[i], cap.min_mv[i], cap.ma[i], cap.type[i]);
				}

				for (i = 0; i < cap.nr; i++) {
					if (VBUS_9V <= cap.max_mv[i]) {
						vbus_mv = cap.max_mv[i];
						ibus_ma = cap.ma[i];
						if (ibus_ma > IBUS_2A)
							ibus_ma = IBUS_2A;
						break;
					}
				}
			} else {
				vbus_mv = VBUS_5V;
				ibus_ma = IBUS_2A;
			}

			printk(KERN_ERR "PD request: %dmV, %dmA\n", vbus_mv, ibus_ma);
			ret = oplus_pdc_setup(&vbus_mv, &ibus_ma);
		} else {
			if (chip->charger_volt > 7500 &&
				(chip->calling_on || chip->camera_on || chip->soc >= 90 || chip->batt_volt >= 4450
				|| chip->temperature > 420 || chip->cool_down_force_5v == true)) {
				vbus_mv = VBUS_5V;
				ibus_ma = IBUS_3A;

				printk(KERN_ERR "PD request: %dmV, %dmA\n", vbus_mv, ibus_ma);
				ret = oplus_pdc_setup(&vbus_mv, &ibus_ma);
			}
		}
	}

	return ret;
}


int oplus_chg_enable_hvdcp_detect(void);
int oplus_chg_set_pd_config(void)
{
	return oplus_mt6360_pd_setup();
}

int oplus_chg_enable_qc_detect(void)
{
	return oplus_chg_enable_hvdcp_detect();
}

int oplus_chg_set_pps_config(int vbus_mv, int ibus_ma)
{
	int ret = 0;
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
			chg_err("set_apdo_charging_policy error %d\n", ret);
			return MTK_ADAPTER_ERROR;
		}
	}

	ret = tcpm_dpm_pd_request(tcpc, vbus_mv, ibus_ma, NULL);
	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_dpm_pd_request fail\n");
		return -EINVAL;
	}
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
		chg_err("get type_c_port0 fail\n");
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
	uint8_t cap_i = 0;
	int ret = 0;
	int idx = 0;
	unsigned int i = 0;

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

			ret = tcpm_dpm_pd_get_source_cap_ext(info->tcpc,
				NULL, &cap_ext);
			if (ret == TCPM_SUCCESS)
				info->srccap.pdp = cap_ext.source_pdp;
			else {
				info->srccap.pdp = 0;
				chg_err("tcpm_dpm_pd_get_source_cap_ext failed(%d)\n", ret);
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

	if (oplus_mt_get_vbus_status() == false)
		return ret;
	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}

	ret = tcpm_set_pd_charging_policy(tcpc, tcpc->pd_port.dpm_charging_policy_default, NULL);

	ret = tcpm_dpm_pd_request(tcpc, vbus_mv_t, ibus_ma_t, NULL);
	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_dpm_pd_request fail\n");
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(tcpc, &vbus_mv_t, &ibus_ma_t);
	if (ret != TCPM_SUCCESS) {
		chg_err("inquire current vbus_mv and ibus_ma fail\n");
		return -EINVAL;
	}

	msleep(100);
	chg_info("PD Default vbus_mv[%d], ibus_ma[%d]\n", vbus_mv_t, ibus_ma_t);

	return ret;
}

int oplus_chg_get_charger_subtype(void)
{
	int charg_subtype = CHARGER_SUBTYPE_DEFAULT;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!pinfo || !chip)
		return CHARGER_SUBTYPE_DEFAULT;

	if (!oplus_chg_is_support_qcpd())
		return CHARGER_SUBTYPE_DEFAULT;

	charg_subtype = oplus_ufcs_get_fastchg_type();
	if (charg_subtype != CHARGER_SUBTYPE_DEFAULT) {
		return CHARGER_SUBTYPE_UFCS;
	}

	if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK ||
		pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30) {
		if (chip->pd_svooc || mt_power_supply_type_check() == POWER_SUPPLY_TYPE_USB_PD_SDP)
			return CHARGER_SUBTYPE_DEFAULT;
		else
			return CHARGER_SUBTYPE_PD;
	} else if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
		if (oplus_pps_check_third_pps_support())
			return CHARGER_SUBTYPE_PPS;
		else {
			if (chip->pd_svooc || mt_power_supply_type_check() == POWER_SUPPLY_TYPE_USB_PD_SDP)
				return CHARGER_SUBTYPE_DEFAULT;
			else
				return CHARGER_SUBTYPE_PD;
		}
	}

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	if (mt6360_get_hvdcp_type() == POWER_SUPPLY_TYPE_USB_HVDCP) {
		return CHARGER_SUBTYPE_QC;
	}
#endif

	return CHARGER_SUBTYPE_DEFAULT;
}

void register_mtk_oplus_batt_interfaces(struct mtk_oplus_batt_interface *intf)
{
	g_oplus_batt_intf = intf;
}
EXPORT_SYMBOL(register_mtk_oplus_batt_interfaces);

bool oplus_chg_set_charge_power_sel(int index)
{
	if (g_oplus_batt_intf->set_charge_power_sel)
		return g_oplus_batt_intf->set_charge_power_sel(index);

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
	} else if (index_curve == CHARGER_SUBTYPE_PPS || index_curve == CHARGER_SUBTYPE_UFCS) {
		target_index_curve = CHARGER_FASTCHG_PPS_AND_UFCS_CURVE;
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

#define QC_CHARGER_VOLTAGE_HIGH 6500
#define QC_SOC_HIGH 90
#define QC_TEMP_HIGH 420
bool oplus_chg_check_qchv_condition(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		pr_err("oplus_chip is null\n");
		return false;
	}

	if (oplus_chg_get_voocphy_support() == AP_SINGLE_CP_VOOCPHY
		|| oplus_chg_get_voocphy_support() == AP_DUAL_CP_VOOCPHY) {
		chip->charger_volt = chip->chg_ops->get_charger_volt();
		if (!chip->calling_on && !chip->camera_on && chip->charger_volt < QC_CHARGER_VOLTAGE_HIGH
			&& chip->soc < QC_SOC_HIGH && chip->temperature <= QC_TEMP_HIGH && !chip->cool_down_force_5v) {
			return true;
		}
	}

	return false;
}

int oplus_chg_set_qc_config_forsvooc(void)
{
	int ret = -1;
	struct oplus_chg_chip *chip = g_oplus_chip;
	if (!chip) {
		pr_err("oplus_chip is null\n");
		return -1;
	}

	printk(KERN_ERR "%s: qc9v svooc [%d %d %d]", __func__, chip->limits.vbatt_pdqc_to_9v_thr, chip->limits.vbatt_pdqc_to_5v_thr, chip->batt_volt);
	if (oplus_chg_get_voocphy_support() == AP_SINGLE_CP_VOOCPHY
		|| oplus_chg_get_voocphy_support() == AP_DUAL_CP_VOOCPHY) {
		chip->charger_volt = chip->chg_ops->get_charger_volt();
		if (!chip->calling_on && !chip->camera_on && chip->charger_volt < 6500 && chip->soc < 90 && chip->temperature <= 420 && chip->cool_down_force_5v == false) {
			printk(KERN_ERR "%s: set qc to 9V", __func__);
			oplus_voocphy_set_pdqc_config();
			mt6360_set_register(MT6360_PMU_DPDM_CTRL, 0x1F, 0x18);
#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
			oplus_notify_hvdcp_detect_stat();
#endif
			if (chip->pdqc_9v_voltage_adaptive) {
				msleep(300);
				chip->charger_volt = chip->chg_ops->get_charger_volt();
				if (chip->charger_volt > 7500) {
					oplus_chg_pdqc9v_vindpm_vol_switch(OPLUS_PDQC_5VTO9V);
				}
			}
			ret = 0;
		} else {
			if (chip->charger_volt > 7500 &&
					(chip->calling_on || chip->camera_on || chip->soc >= 90 || chip->batt_volt >= 4450
					|| chip->temperature > 420 || chip->cool_down_force_5v == true)) {
				printk(KERN_ERR "%s: set qc to 5V", __func__);
				mt6360_set_register(MT6360_PMU_DPDM_CTRL, 0x1F, 0x15);
				if (chip->pdqc_9v_voltage_adaptive) {
					oplus_chg_pdqc9v_vindpm_vol_switch(OPLUS_PDQC_9VTO5V);
				}
				ret = 0;
			}
			printk(KERN_ERR "%s: qc9v svooc  default[%d %d]", __func__, em_mode, chip->batt_volt);
		}
	} else {
		if (!chip->calling_on && chip->charger_volt < 6500 && chip->soc < 90
			&& chip->temperature <= 530 && chip->cool_down_force_5v == false
			&& (chip->batt_volt < chip->limits.vbatt_pdqc_to_9v_thr)) {
			printk(KERN_ERR "%s: set qc to 9V", __func__);
			mt6360_set_register(MT6360_PMU_DPDM_CTRL, 0x1F, 0x15);	/*Before request 9V, need to force 5V first.*/
			msleep(300);
			oplus_chg_suspend_charger();
			oplus_chg_config_charger_vsys_threshold(0x02);/*set Vsys Skip threshold 104%*/
			oplus_chg_enable_burst_mode(false);
			mt6360_set_register(MT6360_PMU_DPDM_CTRL, 0x1F, 0x18);
#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
			oplus_notify_hvdcp_detect_stat();
#endif
			msleep(300);
			oplus_chg_unsuspend_charger();
			ret = 0;
		} else {
			if (chip->charger_volt > 7500 &&
				(chip->calling_on || chip->soc >= 90
				|| chip->batt_volt >= chip->limits.vbatt_pdqc_to_5v_thr || chip->temperature > 530 || chip->cool_down_force_5v == true)) {
				printk(KERN_ERR "%s: set qc to 5V", __func__);
				chip->chg_ops->input_current_write(500);
				oplus_chg_suspend_charger();
				oplus_chg_config_charger_vsys_threshold(0x03);/*set Vsys Skip threshold 101%*/
				mt6360_set_register(MT6360_PMU_DPDM_CTRL, 0x1F, 0x15);
				msleep(400);
				printk(KERN_ERR "%s: charger voltage=%d", __func__, chip->charger_volt);
				oplus_chg_unsuspend_charger();
				ret = 0;
			}
			printk(KERN_ERR "%s: qc9v svooc  default[%d %d]", __func__, em_mode, chip->batt_volt);
		}
	}

	return ret;
}

int oplus_chg_set_qc_config(void)
{
	int ret = -1;
	struct oplus_chg_chip *chip = g_oplus_chip;

	pr_err("oplus_chg_set_qc_config\n");

	if (!chip) {
		pr_err("oplus_chip is null\n");
		return -1;
	}

	if (is_mtksvooc_project) {
		ret = oplus_chg_set_qc_config_forsvooc();
	} else {
		if (!chip->calling_on && !chip->camera_on && chip->charger_volt < 6500 && chip->soc < 90
			&& chip->temperature <= 420 && chip->cool_down_force_5v == false) {
			printk(KERN_ERR "%s: set qc to 9V", __func__);
			mt6360_set_register(MT6360_PMU_DPDM_CTRL, 0x1F, 0x18);
			ret = 0;
		} else {
			if (chip->charger_volt > 7500 &&
				(chip->calling_on || chip->camera_on || chip->soc >= 90 || chip->batt_volt >= 4450
				|| chip->temperature > 420 || chip->cool_down_force_5v == true)) {
				printk(KERN_ERR "%s: set qc to 5V", __func__);
				mt6360_set_register(MT6360_PMU_DPDM_CTRL, 0x1F, 0x15);
				ret = 0;
			}
		}
	}

	return ret;
}

int oplus_chg_enable_hvdcp_detect(void)
{
#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	mt6360_enable_hvdcp_detect();
#endif
	return 0;
}

static void mt6360_step_charging_work(struct work_struct *work)
{
	int tbat_normal_current = 0;
	int step_chg_current = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		pr_err("%s, oplus_chip null\n", __func__);
		return;
	}

	if (!pinfo) {
		pr_err("%s, pinfo null\n", __func__);
		return;
	}
	if (pinfo->data.dual_charger_support) {
		if (chip->tbatt_status == BATTERY_STATUS__NORMAL) {
			tbat_normal_current = oplus_chg_get_tbatt_normal_charging_current(chip);


				if (pinfo->step_status == STEP_CHG_STATUS_STEP1) {
					pinfo->step_cnt += 5;
					if (pinfo->step_cnt >= pinfo->data.step1_time) {
						pinfo->step_status = STEP_CHG_STATUS_STEP2;
						pinfo->step_cnt = 0;
					}
				} else if (pinfo->step_status == STEP_CHG_STATUS_STEP2) {
					pinfo->step_cnt += 5;
					if (pinfo->step_cnt >= pinfo->data.step2_time) {
						pinfo->step_status = STEP_CHG_STATUS_STEP3;
						pinfo->step_cnt = 0;
					}
				} else {
					 if (pinfo->step_status == STEP_CHG_STATUS_STEP3) {
						pinfo->step_cnt = 0;
					}
			}

			if (pinfo->step_status == STEP_CHG_STATUS_STEP1)
				step_chg_current = pinfo->data.step1_current_ma;
			else if (pinfo->step_status == STEP_CHG_STATUS_STEP2)
				step_chg_current = pinfo->data.step2_current_ma;
			else if (pinfo->step_status == STEP_CHG_STATUS_STEP3)
				step_chg_current = pinfo->data.step3_current_ma;
			else
				step_chg_current = 0;

			if (step_chg_current != 0) {
				if (tbat_normal_current >= step_chg_current) {
					pinfo->step_chg_current = step_chg_current;
				} else {
					pinfo->step_chg_current = tbat_normal_current;
				}
			} else {
				pinfo->step_chg_current = tbat_normal_current;
			}

			if (pinfo->step_status != pinfo->step_status_pre) {
				pr_err("%s, step status: %d, step charging current: %d\n", __func__, pinfo->step_status, pinfo->step_chg_current);
				oplus_mt6360_charging_current_write_fast(pinfo->step_chg_current);
				pinfo->step_status_pre = pinfo->step_status;
			}
		}

		schedule_delayed_work(&pinfo->step_charging_work, msecs_to_jiffies(5000));
	}

	return;
}

void oplus_chg_set_camera_on(bool val)
{
	if (!g_oplus_chip) {
		return;
	} else {
		g_oplus_chip->camera_on = val;
		if (oplus_voocphy_get_bidirect_cp_support()) {
			if (g_oplus_chip->charger_exist) {
				if (g_oplus_chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_QC) {
					oplus_chg_set_qc_config();
				} else if (g_oplus_chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_PD) {
					oplus_mt6360_pd_setup();
				} else {
					oplus_chg_set_flash_led_status(g_oplus_chip->camera_on);
				}
			} else {
				oplus_chg_set_flash_led_status(g_oplus_chip->camera_on);
			}
			return;
		}
		if (g_oplus_chip->dual_charger_support
			|| oplus_chg_get_voocphy_support() == AP_SINGLE_CP_VOOCPHY
			|| oplus_chg_get_voocphy_support() == AP_DUAL_CP_VOOCPHY) {
			if (g_oplus_chip->charger_exist) {
				if (g_oplus_chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_QC) {
					oplus_chg_set_qc_config();
				} else if (g_oplus_chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_PD
					&& !(is_mtksvooc_project == true && g_oplus_chip->pd_svooc == true)) {
					oplus_mt6360_pd_setup();
				} else {
					if (!mtkhv_flashled_pinctrl.hv_flashled_support)
						oplus_chg_set_flash_led_status(g_oplus_chip->camera_on);
				}
			} else if (!g_oplus_chip->charger_exist) {
				if (!mtkhv_flashled_pinctrl.hv_flashled_support)
					oplus_chg_set_flash_led_status(g_oplus_chip->camera_on);
			}
		}

		if (mtkhv_flashled_pinctrl.hv_flashled_support) {
			chg_err("bc1.2_done = %d camera_on %d \n", mtkhv_flashled_pinctrl.bc1_2_done, val);
			if (mtkhv_flashled_pinctrl.bc1_2_done && (g_oplus_chip->chg_ops->get_charger_subtype() != CHARGER_SUBTYPE_QC)) {
				if (val) {
					mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_disable);
					mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					chg_err("chgvin_gpio : %d", gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
				} else {
					mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_enable);
					mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
					chg_err("chgvin_gpio : %d", gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
					if (g_oplus_chip->charger_exist && POWER_SUPPLY_TYPE_USB_DCP == g_oplus_chip->charger_type) {
						if (g_oplus_chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_QC) {
							chr_err("oplus_chg_enable_qc_detect\n");
							oplus_chg_enable_qc_detect();
							oplus_chg_set_qc_config();
						}
						if (g_oplus_chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_PD) {
							chr_err("oplus_mt6360_pd_setup\n");
							oplus_mt6360_pd_setup();
						}
					}
				}
			}
		}
	}
}
EXPORT_SYMBOL(oplus_chg_set_camera_on);

void oplus_set_typec_sinkonly(void);
void oplus_set_typec_sinkonly(void)
{
	if (pinfo != NULL && pinfo->tcpc != NULL) {
		tcpm_typec_disable_function(pinfo->tcpc, false);
		printk(KERN_ERR "[OPLUS_CHG][%s]: usbtemp occur otg switch[0]\n", __func__);
		tcpm_typec_change_role(pinfo->tcpc, TYPEC_ROLE_SNK);
	}
};

void oplus_set_typec_cc_open(void);
void oplus_set_typec_cc_open(void)
{
	if (pinfo == NULL || pinfo->tcpc == NULL)
		return;

	tcpm_typec_disable_function(pinfo->tcpc, true);
	chg_err(" !\n");
}

void oplus_usbtemp_recover_cc_open(void)
{
	if (pinfo == NULL || pinfo->tcpc == NULL)
		return;

	tcpm_typec_disable_function(pinfo->tcpc, false);
	chg_err(" !\n");
}

bool oplus_usbtemp_condition(void)
{
	int level = -1;
	int chg_volt = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if(!chip || !pinfo || !pinfo->tcpc) {
		return false;
	}
	if(oplus_ccdetect_check_is_gpio(g_oplus_chip)) {
		level = gpio_get_value(chip->chgic_mtk.oplus_info->ccdetect_gpio);
		if(level == 1
			|| tcpm_inquire_typec_attach_state(pinfo->tcpc) == TYPEC_ATTACHED_AUDIO
			|| tcpm_inquire_typec_attach_state(pinfo->tcpc) == TYPEC_ATTACHED_SRC) {
			return false;
		}
		if (oplus_vooc_get_fastchg_ing() != true) {
			chg_volt = chip->chg_ops->get_charger_volt();
			if(chg_volt < 3000) {
				return false;
			}
		}
		return true;
	}
	return oplus_chg_get_vbus_status(chip);
}

#ifdef CONFIG_OPLUS_CHARGER_MTK

/*modify for cfi*/
static int oplus_get_boot_mode(void);
static int oplus_get_boot_mode(void)
{
	return (int)get_boot_mode();
}

int __attribute__((weak)) get_boot_reason(void)
{
	return 0;
}

static int oplus_get_boot_reason(void);
static int oplus_get_boot_reason(void)
{
	return (int)get_boot_reason();
}
#endif

static bool oplus_chg_is_support_qcpd(void)
{
	if (!pinfo)
		return false;
	if (pinfo->data.pd_not_support && pinfo->data.qc_not_support)
		return false;
	return true;
}

struct oplus_chg_operations  mtk6360_chg_ops = {
	.dump_registers = oplus_mt6360_dump_registers,
	.kick_wdt = oplus_mt6360_kick_wdt,
	.hardware_init = oplus_mt6360_hardware_init,
	.charging_current_write_fast = oplus_mt6360_charging_current_write_fast,
	.set_aicl_point = oplus_mt6360_set_aicl_point,
	.input_current_write = oplus_mt6360_input_current_limit_write,
	.float_voltage_write = oplus_mt6360_float_voltage_write,
	.term_current_set = oplus_mt6360_set_termchg_current,
	.charging_enable = oplus_mt6360_enable_charging,
	.charging_disable = oplus_mt6360_disable_charging,
	.get_charging_enable = oplus_mt6360_check_charging_enable,
	.charger_suspend = oplus_mt6360_suspend_charger,
	.charger_unsuspend = oplus_mt6360_unsuspend_charger,
	.set_rechg_vol = oplus_mt6360_set_rechg_voltage,
	.reset_charger = oplus_mt6360_reset_charger,
	.read_full = oplus_mt6360_registers_read_full,
	.otg_enable = oplus_mt6360_otg_enable,
	.otg_disable = oplus_mt6360_otg_disable,
	.set_charging_term_disable = oplus_mt6360_set_chging_term_disable,
	.check_charger_resume = oplus_mt6360_check_charger_resume,
	.get_chg_current_step = oplus_mt6360_get_chg_current_step,
#ifdef CONFIG_OPLUS_CHARGER_MTK
	.get_charger_type = mt_power_supply_type_check,
	.get_charger_volt = battery_meter_get_charger_voltage,
	.check_chrdet_status = oplus_mt_get_vbus_status,
	.get_instant_vbatt = oplus_battery_meter_get_battery_voltage,
	.get_boot_mode = oplus_get_boot_mode,
	.get_boot_reason = oplus_get_boot_reason,
	.get_chargerid_volt = mt_get_chargerid_volt,
	.set_chargerid_switch_val = mt_set_chargerid_switch_val ,
	.get_chargerid_switch_val  = mt_get_chargerid_switch_val,
	.get_rtc_soc = get_rtc_spare_oplus_fg_value,
	.set_rtc_soc = set_rtc_spare_oplus_fg_value,
	.set_power_off = oplus_mt_power_off,
	.get_charger_subtype = oplus_chg_get_charger_subtype,
	.usb_connect = mt_usb_connect_v1,
	.usb_disconnect = mt_usb_disconnect_v1,
	.get_platform_gauge_curve = oplus_chg_choose_gauge_curve,
#else /* CONFIG_OPLUS_CHARGER_MTK */
	.get_charger_type = qpnp_charger_type_get,
	.get_charger_volt = qpnp_get_prop_charger_voltage_now,
	.check_chrdet_status = qpnp_lbc_is_usb_chg_plugged_in,
	.get_instant_vbatt = qpnp_get_prop_battery_voltage_now,
	.get_boot_mode = get_boot_mode,
	.get_rtc_soc = qpnp_get_pmic_soc_memory,
	.set_rtc_soc = qpnp_set_pmic_soc_memory,
#endif /* CONFIG_OPLUS_CHARGER_MTK */

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = oplus_mt6360_chg_get_dyna_aicl_result,
#endif
	.get_shortc_hw_gpio_status = oplus_chg_get_shortc_hw_gpio_status,
#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
#endif
	.oplus_chg_get_pd_type = oplus_chg_get_pd_type,
	.oplus_chg_pd_setup = oplus_mt6360_pd_setup,
	.set_qc_config = oplus_chg_set_qc_config,
	.enable_qc_detect = oplus_chg_enable_hvdcp_detect,	/*for qc 9v2a*/
	.check_pdphy_ready = oplus_check_pdphy_ready,
	.get_usbtemp_volt = oplus_get_usbtemp_volt,
	.set_typec_sinkonly = oplus_set_typec_sinkonly,
	.set_typec_cc_open = oplus_set_typec_cc_open,
	.oplus_usbtemp_monitor_condition = oplus_usbtemp_condition,
	.check_qchv_condition = oplus_chg_check_qchv_condition,
	.get_subboard_temp = oplus_force_get_subboard_temp,
	.is_support_qcpd = oplus_chg_is_support_qcpd,
	.check_cc_mode = oplus_check_cc_mode,
};
/*====================================================================*/
EXPORT_SYMBOL(oplus_set_typec_sinkonly);
EXPORT_SYMBOL(oplus_set_typec_cc_open);
EXPORT_SYMBOL(oplus_get_usbtemp_volt);
EXPORT_SYMBOL(oplus_usbtemp_condition);

bool oplus_otgctl_by_buckboost(void)
{
	if (!g_oplus_chip)
		return false;
	if (oplus_voocphy_get_bidirect_cp_support())
		return false;

	return g_oplus_chip->vbatt_num == 2;
}

void oplus_otg_enable_by_buckboost(void)
{
	if (!g_oplus_chip || !(g_oplus_chip->chg_ops->charging_disable) || !(g_oplus_chip->chg_ops->otg_enable))
		return;

	g_oplus_chip->chg_ops->charging_disable();
	g_oplus_chip->chg_ops->otg_enable();

	if (mtkhv_flashled_pinctrl.pinctrl) {
		mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_disable);
		mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		chg_err("chgvin_gpio : %d", gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
	}
}

void oplus_otg_disable_by_buckboost(void)
{
	if (!g_oplus_chip || !(g_oplus_chip->chg_ops->otg_disable))
		return;

	g_oplus_chip->chg_ops->otg_disable();

	if (mtkhv_flashled_pinctrl.pinctrl) {
		mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_enable);
		mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		chg_err("chgvin_gpio : %d", gpio_get_value(mtkhv_flashled_pinctrl.chgvin_gpio));
	}
}

void oplus_gauge_set_event(int event)
{
	if (NULL != pinfo) {
		charger_manager_notifier(pinfo, event);
		chr_err("[%s] notify mtkfuelgauge event = %d\n", __func__, event);
	}
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

		adapter_dev_get_cap(pinfo->adapter_dev[PD], MTK_PD_APDO, &cap);
		for (i = 0; i < cap.nr; i++) {
			seq_printf(m,
			"%d: mV:%d,%d mA:%d type:%d pwr_limit:%d pdp:%d\n", i,
			cap.max_mv[i], cap.min_mv[i], cap.ma[i],
			cap.type[i], cap.pwr_limit[i], cap.pdp);
		}
	} else if (pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK
		|| pinfo->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30) {
		seq_puts(m, "********** PD cap Dump **********\n");

		adapter_dev_get_cap(pinfo->adapter_dev[PD], MTK_PD, &cap);
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

#ifdef OPLUS_FEATURE_CHG_BASIC
	charger_dir = proc_mkdir("charger_mtk", NULL);
#else
	charger_dir = proc_mkdir("charger", NULL);
#endif
	if (!charger_dir) {
		chr_err("fail to mkdir /proc/charger\n");
		return;
	}

	proc_create("dump_log", 0640,
		charger_dir, &charger_dump_log_proc_fops);
}

void scd_ctrl_cmd_from_user(void *nl_data, struct sc_nl_msg_t *ret_msg)
{
	struct sc_nl_msg_t *msg;

	msg = nl_data;
	ret_msg->sc_cmd = msg->sc_cmd;

	switch (msg->sc_cmd) {
	case SC_DAEMON_CMD_PRINT_LOG: {
		chr_err("%s", &msg->sc_data[0]);
	}
	break;

	case SC_DAEMON_CMD_SET_DAEMON_PID: {
		memcpy(&pinfo->sc.g_scd_pid, &msg->sc_data[0],
				sizeof(pinfo->sc.g_scd_pid));
		chr_err("[fr] SC_DAEMON_CMD_SET_DAEMON_PID = %d(first launch)\n",
				pinfo->sc.g_scd_pid);
	}
	break;

	case SC_DAEMON_CMD_SETTING: {
		struct scd_cmd_param_t_1 data;

		memcpy(&data, &msg->sc_data[0],
				sizeof(struct scd_cmd_param_t_1));
			chr_debug("rcv data:%d %d %d %d %d %d %d %d %d %d %d %d %d %d Ans:%d\n",
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

		pinfo->sc.solution = data.data[SC_SOLUTION];
		if (data.data[SC_SOLUTION] == SC_DISABLE)
			pinfo->sc.disable_charger = true;
		else if (data.data[SC_SOLUTION] == SC_REDUCE)
			pinfo->sc.disable_charger = false;
		else
			pinfo->sc.disable_charger = false;
	}
	break;

	default:
		chr_err("bad sc_DAEMON_CTRL_CMD_FROM_USER 0x%x\n", msg->sc_cmd);
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

	ret = netlink_unicast(pinfo->sc.daemo_nl_sk, skb, pid, MSG_DONTWAIT);
	if (ret < 0) {
		chr_err("[Netlink] sc send failed %d\n", ret);
		return;
	}
}

static __maybe_unused void chg_nl_data_handler(struct sk_buff *skb)
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

void sc_update(struct mtk_charger *pinfo)
{
	memset(&pinfo->sc.data, 0, sizeof(struct scd_cmd_param_t_1));
	pinfo->sc.data.data[SC_VBAT] = battery_get_bat_voltage();
	pinfo->sc.data.data[SC_BAT_TMP] = battery_get_bat_temperature();
	pinfo->sc.data.data[SC_UISOC] = battery_get_uisoc();
	pinfo->sc.data.data[SC_SOC] = battery_get_soc();

	pinfo->sc.data.data[SC_ENABLE] = pinfo->sc.enable;
	pinfo->sc.data.data[SC_BAT_SIZE] = pinfo->sc.battery_size;
	pinfo->sc.data.data[SC_START_TIME] = pinfo->sc.start_time;
	pinfo->sc.data.data[SC_END_TIME] = pinfo->sc.end_time;
	pinfo->sc.data.data[SC_IBAT_LIMIT] = pinfo->sc.current_limit;
	pinfo->sc.data.data[SC_TARGET_PERCENTAGE] = pinfo->sc.target_percentage;
	pinfo->sc.data.data[SC_LEFT_TIME_FOR_CV] = pinfo->sc.left_time_for_cv;

	charger_dev_get_charging_current(pinfo->chg1_dev, &pinfo->sc.data.data[SC_IBAT_SETTING]);
	pinfo->sc.data.data[SC_IBAT_SETTING] = pinfo->sc.data.data[SC_IBAT_SETTING] / 1000;
	pinfo->sc.data.data[SC_IBAT] = battery_get_bat_current() / 10;
	charger_dev_get_ibus(pinfo->chg1_dev, &pinfo->sc.data.data[SC_IBUS]);
	if (chargerlog_level == 1)
		pinfo->sc.data.data[SC_DBGLV] = 3;
	else
		pinfo->sc.data.data[SC_DBGLV] = 7;
}

int wakeup_sc_algo_cmd(struct scd_cmd_param_t_1 *data, int subcmd, int para1)
{
	if (pinfo->sc.g_scd_pid != 0) {
		struct sc_nl_msg_t *sc_msg;
		int size = SCD_NL_MSG_T_HDR_LEN + sizeof(struct scd_cmd_param_t_1);

		if (size > (PAGE_SIZE << 1))
			sc_msg = vmalloc(size);
		else {
			if (in_interrupt())
				sc_msg = kmalloc(size, GFP_ATOMIC);
		else
			sc_msg = kmalloc(size, GFP_KERNEL);
		}

		if (sc_msg == NULL) {
			if (size > PAGE_SIZE)
				sc_msg = vmalloc(size);

			if (sc_msg == NULL)
				return -1;
		}
		sc_update(pinfo);
		chr_debug(
			"[wakeup_fg_algo] malloc size=%d pid=%d\n",
			size, pinfo->sc.g_scd_pid);
		memset(sc_msg, 0, size);
		sc_msg->sc_cmd = SC_DAEMON_CMD_NOTIFY_DAEMON;
		sc_msg->sc_subcmd = subcmd;
		sc_msg->sc_subcmd_para1 = para1;
		memcpy(sc_msg->sc_data, data, sizeof(struct scd_cmd_param_t_1));
		sc_msg->sc_data_len += sizeof(struct scd_cmd_param_t_1);
		sc_nl_send_to_user(pinfo->sc.g_scd_pid, 0, sc_msg);

		kvfree(sc_msg);

		return 0;
	}
	chr_debug("pid is NULL\n");
	return -1;
}

static  __maybe_unused ssize_t show_typec_sbu_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	chg_err("get oplus_get_typec_sbu_voltage = [%d].\n",  oplus_get_typec_sbu_voltage());

	return sprintf(buf, "%d\n",  oplus_get_typec_sbu_voltage());
}
static  __maybe_unused ssize_t store_typec_sbu_voltage(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return -1;
}
static __maybe_unused DEVICE_ATTR(typec_sbu_voltage, 0664, show_typec_sbu_voltage, store_typec_sbu_voltage);

static bool oplus_ccdetect_check_is_wd0(struct oplus_chg_chip *chip)
{
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		printk(KERN_ERR "%s:device tree info missing\n", __func__);
		return false;
	}

	if (chip->support_wd0)
		return true;
	if (of_property_read_bool(node, "qcom,ccdetect_by_wd0")) {
		chip->support_wd0 = true;
		return true;
	}

	return false;
}

bool oplus_chg_get_wd0_status(void)
{
	if (!pinfo) {
		pr_err("%s, pinfo null!\n", __func__);
		return false;
	}

	return pinfo->wd0_detect;
}

void oplus_wd0_detect_work(struct work_struct *work)
{
	int level;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		pr_err("%s: g_oplus_chip not ready!\n", __func__);
		return;
	}

	level = !oplus_chg_get_wd0_status();
	pr_err("%s: level [%d]", __func__, level);

	if (level != 1) {
		oplus_wake_up_usbtemp_thread();
	} else {
		chip->usbtemp_check = oplus_usbtemp_condition();
		schedule_delayed_work(&usbtemp_recover_work, 0);
	}

	/*schedule_delayed_work(&wd0_detect_work, msecs_to_jiffies(CCDETECT_DELAY_MS));*/
}

struct delayed_work wd0_detect_work;
static int pd_tcp_notifier_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	bool vbus_on;
	struct oplus_pps_chip *pps_chip;

	pr_err("%s: event:%lu", __func__, event);
	switch (event) {
	case TCP_NOTIFY_WD0_STATE:
		pinfo->wd0_detect = noti->wd0_state.wd0;
		pr_err("%s wd0 = %d\n", __func__, noti->wd0_state.wd0);
		schedule_delayed_work(&wd0_detect_work, msecs_to_jiffies(CCDETECT_DELAY_MS));
		break;

	case TCP_NOTIFY_SOURCE_VBUS:
		pr_info("source vbus = %dmv\n",
				 noti->vbus_state.mv);
		vbus_on = (noti->vbus_state.mv) ? true : false;
		if (!primary_charger) {
			primary_charger = get_charger_by_name("primary_chg");
			if (!primary_charger) {
				pr_info("%s: get primary charger device failed\n", __func__);
				return -ENODEV;
			}
		}
		if (vbus_on) {
			/* Modify for OTG */
			printk("typec vbus_on\n");
			if (is_vooc_support_single_batt_svooc() == true) {
				vooc_enable_cp_for_otg(1);
			}

			if (oplus_otgctl_by_buckboost()) {
				oplus_otg_enable_by_buckboost();
			} else {
				charger_dev_enable_otg(primary_charger, true);
				charger_dev_set_boost_current_limit(primary_charger,
						pinfo->otg_current_limit);
			}
		} else {
			/* Modify for OTG */
			if (oplus_otgctl_by_buckboost()) {
				oplus_otg_disable_by_buckboost();
			} else {
				charger_dev_enable_otg(primary_charger, false);
			}

			if (is_vooc_support_single_batt_svooc() == true) {
				vooc_enable_cp_for_otg(0);
			}
		}
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

	default:
		break;
	}

	return NOTIFY_OK;
}

int chg_alg_event(struct notifier_block *notifier,
			unsigned long event, void *data)
{
	chr_err("%s: evt:%lu\n", __func__, event);

	return NOTIFY_DONE;
}

static char *mtk_charger_supplied_to[] = {
	"battery"
};

static int mtk_charger_probe(struct platform_device *pdev)
{
	struct mtk_charger *info = NULL;
	int i;
	char *name = NULL;
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct oplus_chg_chip *oplus_chip;
	int level = 0;
	int rc = 0;
#endif

	chr_err("%s: starts\n", __func__);
#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_chip = devm_kzalloc(&pdev->dev, sizeof(*oplus_chip), GFP_KERNEL);
	if (!oplus_chip)
		return -ENOMEM;

	oplus_chip->dev = &pdev->dev;
	g_oplus_chip = oplus_chip;
	oplus_chg_parse_svooc_dt(oplus_chip);
	if (oplus_chip->vbatt_num == 1) {
		if (oplus_gauge_check_chip_is_null()) {
			chg_err("[oplus_chg_init] gauge null, will do after bettery init.\n");
			return -EPROBE_DEFER;
		}
		oplus_chip->chg_ops = &mtk6360_chg_ops;
	} else {
		chg_err("[oplus_chg_init] gauge[%d]vooc[%d]adapter[%d]\n",
				oplus_gauge_check_chip_is_null(),
				oplus_vooc_check_chip_is_null(),
				oplus_adapter_check_chip_is_null());
		if (oplus_voocphy_get_bidirect_cp_support()) {
			if (oplus_gauge_check_chip_is_null() || oplus_adapter_check_chip_is_null()) {
				chg_err("[oplus_chg_init] gauge || vooc || adapter null, will do after bettery init.\n");
				return -EPROBE_DEFER;
			}
			oplus_chip->chg_ops = &mtk6360_chg_ops;
		} else {
			if (oplus_gauge_check_chip_is_null() || oplus_vooc_check_chip_is_null() || oplus_adapter_check_chip_is_null()) {
				chg_err("[oplus_chg_init] gauge || vooc || adapter null, will do after bettery init.\n");
				return -EPROBE_DEFER;
			}
			oplus_chip->chg_ops = oplus_get_chg_ops();
		}
		is_vooc_cfg = true;
		is_mtksvooc_project = true;
		chg_err("%s is_vooc_cfg = %d\n", __func__, is_vooc_cfg);
	}
#endif /* OPLUS_FEATURE_CHG_BASIC */

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

	pinfo = info;
	mtk_charger_parse_dt(info, &pdev->dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_chip->chgic_mtk.oplus_info = info;

	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev) {
		chg_err("found primary charger [%s]\n",
			info->chg1_dev->props.alias_name);
	} else {
		chg_err("can't find primary charger!\n");
		return -EPROBE_DEFER;
	}
#endif

	mutex_init(&info->cable_out_lock);
	mutex_init(&info->charger_lock);
	mutex_init(&info->pd_lock);
	mutex_init(&info->ta_lock);
	for (i = 0; i < CHG2_SETTING + 1; i++) {
		mutex_init(&info->pp_lock[i]);
		info->force_disable_pp[i] = false;
		info->enable_pp[i] = true;
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	register_mtk_oplus_chg_interfaces(&mtk_oplus_chg_intf);
	oplus_power_supply_init(oplus_chip);
	oplus_chg_parse_custom_dt(oplus_chip);
	oplus_chg_parse_charger_dt(oplus_chip);
	oplus_chip->chg_ops->hardware_init();
	oplus_chip->authenticate = oplus_gauge_get_batt_authenticate();
	chg_err("oplus_chg_init!\n");
	oplus_chg_init(oplus_chip);

	if(oplus_chg_get_voocphy_support() == AP_SINGLE_CP_VOOCPHY
		|| oplus_chg_get_voocphy_support() == AP_DUAL_CP_VOOCPHY) {
		is_mtksvooc_project = true;
		chg_err("support voocphy, is_mtksvooc_project is true!\n");
	}

	if (get_boot_mode() != KERNEL_POWER_OFF_CHARGING_BOOT) {
		oplus_tbatt_power_off_task_init(oplus_chip);
	}
#endif

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s",
		"charger suspend wakelock");
	info->charger_wakelock =
		wakeup_source_register(NULL, name);
	spin_lock_init(&info->slock);

	init_waitqueue_head(&info->wait_que);
	info->polling_interval = CHARGING_INTERVAL;
	mtk_charger_init_timer(info);
#ifdef CONFIG_PM
	if (register_pm_notifier(&info->pm_notifier)) {
		chr_err("%s: register pm failed\n", __func__);
		return -ENODEV;
	}
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
	info->psy_desc1.usb_types = charger_psy_usb_types;
	info->psy_desc1.num_usb_types = ARRAY_SIZE(charger_psy_usb_types);
	info->psy_desc1.properties = charger_psy_properties;
	info->psy_desc1.num_properties = ARRAY_SIZE(charger_psy_properties);
	info->psy_desc1.get_property = psy_charger_get_property;
	info->psy_desc1.set_property = psy_charger_set_property;
	info->psy_desc1.property_is_writeable =
			psy_charger_property_is_writeable;
	info->psy_desc1.external_power_changed =
		mtk_charger_external_power_changed;
	info->psy_cfg1.drv_data = info;
	info->psy_cfg1.supplied_to = mtk_charger_supplied_to;
	info->psy_cfg1.num_supplicants = ARRAY_SIZE(mtk_charger_supplied_to);
	info->psy1 = power_supply_register(&pdev->dev, &info->psy_desc1,
			&info->psy_cfg1);
#if IS_ENABLED(CONFIG_MTK_PLAT_POWER_6893)
	info->chg_psy = devm_power_supply_get_by_phandle(&pdev->dev,
		"charger");
#else
	info->chg_psy = power_supply_get_by_name("primary_chg");
#endif
	if (IS_ERR_OR_NULL(info->chg_psy))
		chr_err("%s: devm power fail to get chg_psy\n", __func__);

#if !IS_ENABLED(CONFIG_MTK_PLAT_POWER_6893)
	info->bc12_psy = power_supply_get_by_name("primary_chg");
	if (IS_ERR_OR_NULL(info->bc12_psy))
		chr_err("%s: devm power fail to get bc12_psy\n", __func__);
#endif

	info->bat_psy = devm_power_supply_get_by_phandle(&pdev->dev,
		"gauge");
	if (IS_ERR_OR_NULL(info->bat_psy))
		chr_err("%s: devm power fail to get bat_psy\n", __func__);

	if (IS_ERR(info->psy1))
		chr_err("register psy1 fail:%ld\n",
			PTR_ERR(info->psy1));

	info->psy_desc2.name = "mtk-slave-charger";
	info->psy_desc2.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_desc2.usb_types = charger_psy_usb_types;
	info->psy_desc2.num_usb_types = ARRAY_SIZE(charger_psy_usb_types);
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

	info->psy_dvchg_desc1.name = "mtk-mst-div-chg";
	info->psy_dvchg_desc1.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_dvchg_desc1.usb_types = charger_psy_usb_types;
	info->psy_dvchg_desc1.num_usb_types = ARRAY_SIZE(charger_psy_usb_types);
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

	info->psy_dvchg_desc2.name = "mtk-slv-div-chg";
	info->psy_dvchg_desc2.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_dvchg_desc2.usb_types = charger_psy_usb_types;
	info->psy_dvchg_desc2.num_usb_types = ARRAY_SIZE(charger_psy_usb_types);
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

	info->psy_hvdvchg_desc1.name = "mtk-mst-hvdiv-chg";
	info->psy_hvdvchg_desc1.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_hvdvchg_desc1.usb_types = charger_psy_usb_types;
	info->psy_hvdvchg_desc1.num_usb_types = ARRAY_SIZE(charger_psy_usb_types);
	info->psy_hvdvchg_desc1.properties = charger_psy_properties;
	info->psy_hvdvchg_desc1.num_properties =
					     ARRAY_SIZE(charger_psy_properties);
	info->psy_hvdvchg_desc1.get_property = psy_charger_get_property;
	info->psy_hvdvchg_desc1.set_property = psy_charger_set_property;
	info->psy_hvdvchg_desc1.property_is_writeable =
					      psy_charger_property_is_writeable;
	info->psy_hvdvchg_cfg1.drv_data = info;
	info->psy_hvdvchg1 = power_supply_register(&pdev->dev,
						   &info->psy_hvdvchg_desc1,
						   &info->psy_hvdvchg_cfg1);
	if (IS_ERR(info->psy_hvdvchg1))
		chr_err("register psy hvdvchg1 fail:%ld\n",
					PTR_ERR(info->psy_hvdvchg1));

	info->psy_hvdvchg_desc2.name = "mtk-slv-hvdiv-chg";
	info->psy_hvdvchg_desc2.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_hvdvchg_desc2.usb_types = charger_psy_usb_types;
	info->psy_hvdvchg_desc2.num_usb_types = ARRAY_SIZE(charger_psy_usb_types);
	info->psy_hvdvchg_desc2.properties = charger_psy_properties;
	info->psy_hvdvchg_desc2.num_properties =
					     ARRAY_SIZE(charger_psy_properties);
	info->psy_hvdvchg_desc2.get_property = psy_charger_get_property;
	info->psy_hvdvchg_desc2.set_property = psy_charger_set_property;
	info->psy_hvdvchg_desc2.property_is_writeable =
					      psy_charger_property_is_writeable;
	info->psy_hvdvchg_cfg2.drv_data = info;
	info->psy_hvdvchg2 = power_supply_register(&pdev->dev,
						   &info->psy_hvdvchg_desc2,
						   &info->psy_hvdvchg_cfg2);
	if (IS_ERR(info->psy_hvdvchg2))
		chr_err("register psy hvdvchg2 fail:%ld\n",
					PTR_ERR(info->psy_hvdvchg2));

	info->log_level = CHRLOG_DEBUG_LEVEL;
	/*mtk_adapter_protocol_init(info);*/

	for (i = 0;i < MAX_TA_IDX;i++) {
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

#ifdef OPLUS_FEATURE_CHG_BASIC
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

	pinfo->chargeric_temp_chan = iio_channel_get(oplus_chip->dev, "auxadc3-chargeric_temp");
	if (IS_ERR(pinfo->chargeric_temp_chan)) {
		chg_err("Couldn't get chargeric_temp_chan...\n");
		pinfo->chargeric_temp_chan = NULL;
	}

	pinfo->charger_id_chan = iio_channel_get(oplus_chip->dev, "auxadc3-charger_id");
	if (IS_ERR(pinfo->charger_id_chan)) {
		chg_err("Couldn't get charger_id_chan...\n");
		pinfo->charger_id_chan = NULL;
	}

	pinfo->usb_temp_v_l_chan = iio_channel_get(oplus_chip->dev, "auxadc4-usb_temp_v_l");
	if (IS_ERR(pinfo->usb_temp_v_l_chan)) {
		chg_err("Couldn't get usb_temp_v_l_chan...\n");
		pinfo->usb_temp_v_l_chan = NULL;
	}

	pinfo->usb_temp_v_r_chan = iio_channel_get(oplus_chip->dev, "auxadc5-usb_temp_v_r");
	if (IS_ERR(pinfo->usb_temp_v_r_chan)) {
		chg_err("Couldn't get usb_temp_v_r_chan...\n");
		pinfo->usb_temp_v_r_chan = NULL;
	}

	pinfo->subboard_temp_chan = iio_channel_get(oplus_chip->dev, "auxadc6-subboard_temp");
	if (IS_ERR(pinfo->subboard_temp_chan)) {
		chg_err("Couldn't get subboard_temp_chan...\n");
		pinfo->subboard_temp_chan = NULL;
	}

	pinfo->usbcon_temp_chan = iio_channel_get(oplus_chip->dev, "usbcon_temp");
	if (IS_ERR(pinfo->usbcon_temp_chan)) {
		chg_err("Couldn't get usbcon_temp_chan...\n");
		pinfo->usbcon_temp_chan = NULL;
	}

	pinfo->batcon_temp_chan = iio_channel_get(oplus_chip->dev, "batcon_temp");
	if (IS_ERR(pinfo->batcon_temp_chan)) {
		chg_err("Couldn't get batcon_temp_chan...\n");
		pinfo->batcon_temp_chan = NULL;
	}

	if (oplus_ccdetect_support_check() == true) {
		INIT_DELAYED_WORK(&ccdetect_work, oplus_ccdetect_work);
		INIT_DELAYED_WORK(&usbtemp_recover_work, oplus_usbtemp_recover_work);
		oplus_ccdetect_irq_register(oplus_chip);
		level = gpio_get_value(oplus_chip->chgic_mtk.oplus_info->ccdetect_gpio);
		usleep_range(2000, 2100);
		if (level != gpio_get_value(oplus_chip->chgic_mtk.oplus_info->ccdetect_gpio)) {
			printk(KERN_ERR "[OPLUS_CHG][%s]: ccdetect_gpio is unstable, try again...\n", __func__);
			usleep_range(10000, 11000);
			level = gpio_get_value(oplus_chip->chgic_mtk.oplus_info->ccdetect_gpio);
		}
		if ((level <= 0) && (is_meta_mode() == false)) {
			schedule_delayed_work(&ccdetect_work, msecs_to_jiffies(6000));
		}
		printk(KERN_ERR "[OPLUS_CHG][%s]: ccdetect_gpio ..level[%d]  \n", __func__, level);
	} else if (oplus_ccdetect_check_is_wd0(oplus_chip) == true) {
		INIT_DELAYED_WORK(&wd0_detect_work, oplus_wd0_detect_work);
		INIT_DELAYED_WORK(&usbtemp_recover_work, oplus_usbtemp_recover_work);
	}

	oplus_chip->con_volt = con_volt_20131;
	oplus_chip->con_temp = con_temp_20131;
	oplus_chip->len_array = ARRAY_SIZE(con_temp_20131);

	if (oplus_usbtemp_check_is_support() == true)
		oplus_usbtemp_thread_init();

	if (is_vooc_project() == true)
		oplus_mt6360_suspend_charger();

	oplus_chg_configfs_init(oplus_chip);
#endif

	sc_init(&info->sc);
	info->chg_alg_nb.notifier_call = chg_alg_event;

	info->fast_charging_indicator = 0;
	info->enable_meta_current_limit = 1;

	if (strcmp(info->curr_select_name, "NULL")) {
		info->cs_para_mode = 0;
		info->cs_heatlim = 5;
		info->dual_chg_stat = STILL_CHG;
	}

	info->is_charging = false;
	info->safety_timer_cmd = -1;
	info->cmd_pp = -1;

	/* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
	/* 9 = LOW_POWER_OFF_CHARGING_BOOT */
	if (info != NULL && info->bootmode != 8 && info->bootmode != 9)
		mtk_charger_force_disable_power_path(info, CHG1_SETTING, true);

	pr_err("%s charger thread started,,!!", __func__);
	kthread_run(charger_routine_thread, info, "charger_thread");
#ifdef OPLUS_FEATURE_CHG_BASIC
	INIT_DELAYED_WORK(&pinfo->step_charging_work, mt6360_step_charging_work);
	INIT_DELAYED_WORK(&pinfo->check_charger_out_work, oplus_check_charger_out_func);
	chg_err("oplus_chg_wake_update_work!\n");
	oplus_chg_wake_update_work();
#endif
	pr_err("%s done successfully,,!!", __func__);
	return 0;
}

static int mtk_charger_remove(struct platform_device *dev)
{
	return 0;
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
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (mtkhv_flashled_pinctrl.hv_flashled_support) {
		mtkhv_flashled_pinctrl.bc1_2_done = false;
		mutex_lock(&mtkhv_flashled_pinctrl.chgvin_mutex);
		pinctrl_select_state(mtkhv_flashled_pinctrl.pinctrl, mtkhv_flashled_pinctrl.chgvin_enable);
		mutex_unlock(&mtkhv_flashled_pinctrl.chgvin_mutex);
	}

	if (g_oplus_chip) {
		enter_ship_mode_function(g_oplus_chip);
	}
#endif
}

static unsigned long suspend_tm_sec = 0;
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc = NULL;
	int rc = 0;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		chg_err("%s: unable to open rtc device (%s)\n",
				__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		chg_err("Error reading rtc device (%s) : %d\n",
				CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		chg_err("Invalid RTC time (%s): %d\n",
				CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static int mtk_charger_pm_resume(struct device *dev)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;

	if (!g_oplus_chip) {
		return 0;
	}
	if (g_oplus_chip->chg_ops != &mtk6360_chg_ops) {
		return 0;
	}
	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	if (sleep_time > 60) {
		oplus_chg_soc_update_when_resume(sleep_time);
	}
	return 0;
}

static int mtk_charger_pm_suspend(struct device *dev)
{
	if (!g_oplus_chip) {
		return 0;
	}
	if (g_oplus_chip->chg_ops != &mtk6360_chg_ops) {
		return 0;
	}

	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	return 0;
}

static const struct dev_pm_ops mtk_charger_pm_ops = {
	.resume		= mtk_charger_pm_resume,
	.suspend		= mtk_charger_pm_suspend,
};

static const struct of_device_id mtk_charger_of_match[] = {
	{.compatible = "mediatek,charger", },
	{},
};

MODULE_DEVICE_TABLE(of, mtk_charger_of_match);

struct platform_device charger_device = {
	.name = "charger",
	.id = -1,
};

static struct platform_driver charger_driver = {
	.probe = mtk_charger_probe,
	.remove = mtk_charger_remove,
	.shutdown = mtk_charger_shutdown,
	.driver = {
		   .name = "charger",
		   .of_match_table = mtk_charger_of_match,
		   .pm = &mtk_charger_pm_ops,
	},
};


extern int oplus_optiga_driver_init(void);
extern void oplus_optiga_driver_exit(void);

extern int bq27541_driver_init(void);
extern void bq27541_driver_exit(void);

static int __init mtk_charger_init(void)
{
#ifdef CONFIG_OPLUS_CHARGER_OPTIGA
	oplus_optiga_driver_init();
#endif
	bq27541_driver_init();
	return platform_driver_register(&charger_driver);
}

static void __exit mtk_charger_exit(void)
{
#ifdef CONFIG_OPLUS_CHARGER_OPTIGA
	oplus_optiga_driver_exit();
#endif
	bq27541_driver_exit();
	platform_driver_unregister(&charger_driver);
}

/*Modify for charging*/
oplus_chg_module_register(mtk_charger);

MODULE_DESCRIPTION("MTK Charger Driver");
MODULE_LICENSE("GPL");
