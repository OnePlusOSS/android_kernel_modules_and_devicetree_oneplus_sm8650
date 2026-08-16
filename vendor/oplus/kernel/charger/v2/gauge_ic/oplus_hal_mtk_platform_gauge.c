// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[MT6375_GAUGE]([%s][%d]): " fmt, __func__, __LINE__

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>

/* #include <mt-plat/battery_common.h> */
#include <soc/oplus/device_info.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#else
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <soc/oplus/device_info.h>
#include <linux/proc_fs.h>
#include <linux/soc/qcom/smem.h>
#endif
#include<linux/gfp.h>

#ifdef OPLUS_SHA1_HMAC
#include <linux/random.h>
#endif
#include <oplus_chg_module.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_cpa.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_monitor.h>
#include <mtk_battery.h>
#include <linux/build_bug.h>

#include "oplus_hal_mtk_platform_gauge.h"
#include <oplus_chg_wls.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
#include "../../oplus_gauge.h"
extern struct oplus_gauge_chip *g_gauge_chip;
#else
#include "oplus_gauge.h"
struct oplus_gauge_chip *g_gauge_chip = NULL;
#endif
static struct chip_mt6375_gauge *g_mt6375_chip;

enum oplus_track_item_idx {
	TRACK_ITEM_START = 0,
	TRACK_PRE_VBAT = TRACK_ITEM_START,
	TRACK_CUR_VBAT,
	TRACK_PRE_TBAT,
	TRACK_CUR_TBAT,
	TRACK_PRE_CAR_C,
	TRACK_CUR_CAR_C,
	TRACK_PRE_TOTAL_CAR,
	TRACK_CUR_TOTAL_CAR,
	TRACK_PRE_C_SOC,
	TRACK_CUR_C_SOC,
	TRACK_PRE_V_SOC,
	TRACK_CUR_V_SOC,
	TRACK_PRE_SOC,
	TRACK_CUR_SOC,
	TRACK_PRE_UI_SOC,
	TRACK_CUR_UI_SOC,
	TRACK_PRE_QMAX,
	TRACK_CUR_QMAX,
	TRACK_PRE_QUSE,
	TRACK_CUR_QUSE,
	TRACK_PRE_ZCV,
	TRACK_CUR_ZCV,
	TRACK_PRE_AGING,
	TRACK_CUR_AGING,
	TRACK_BATT_CC,
	TRACK_PRE_SHOW_AG,
	TRACK_CUR_SHOW_AG,
	TRACK_ITEM_END
};

const static unsigned int oplus_chg_track_pattern[] = {
	/*plugout - report all data items*/
	[GAUGE_TRACK_CALI_FLAG_PLUGOUT] =
		BIT(TRACK_PRE_VBAT)   | BIT(TRACK_CUR_VBAT)   | BIT(TRACK_PRE_TBAT)      | BIT(TRACK_CUR_TBAT) |
		BIT(TRACK_PRE_CAR_C)  | BIT(TRACK_CUR_CAR_C)  | BIT(TRACK_PRE_TOTAL_CAR) | BIT(TRACK_CUR_TOTAL_CAR) |
		BIT(TRACK_PRE_C_SOC)  | BIT(TRACK_CUR_C_SOC)  | BIT(TRACK_PRE_V_SOC)     | BIT(TRACK_CUR_V_SOC) |
		BIT(TRACK_PRE_SOC)    | BIT(TRACK_CUR_SOC)    | BIT(TRACK_PRE_UI_SOC)    | BIT(TRACK_CUR_UI_SOC) |
		BIT(TRACK_PRE_QMAX)   | BIT(TRACK_CUR_QMAX)   | BIT(TRACK_PRE_QUSE)      | BIT(TRACK_CUR_QUSE) |
		BIT(TRACK_PRE_ZCV)    | BIT(TRACK_CUR_ZCV)    | BIT(TRACK_PRE_AGING)     | BIT(TRACK_CUR_AGING) |
		BIT(TRACK_PRE_SHOW_AG)| BIT(TRACK_CUR_SHOW_AG)| BIT(TRACK_BATT_CC),

	/*full - report all data items*/
	[GAUGE_TRACK_CALI_FLAG_CHG_FULL] =
		BIT(TRACK_PRE_VBAT)   | BIT(TRACK_CUR_VBAT)   | BIT(TRACK_PRE_TBAT)      | BIT(TRACK_CUR_TBAT) |
		BIT(TRACK_PRE_CAR_C)  | BIT(TRACK_CUR_CAR_C)  | BIT(TRACK_PRE_TOTAL_CAR) | BIT(TRACK_CUR_TOTAL_CAR) |
		BIT(TRACK_PRE_C_SOC)  | BIT(TRACK_CUR_C_SOC)  | BIT(TRACK_PRE_V_SOC)     | BIT(TRACK_CUR_V_SOC) |
		BIT(TRACK_PRE_SOC)    | BIT(TRACK_CUR_SOC)    | BIT(TRACK_PRE_UI_SOC)    | BIT(TRACK_CUR_UI_SOC) |
		BIT(TRACK_PRE_QMAX)   | BIT(TRACK_CUR_QMAX)   | BIT(TRACK_PRE_QUSE)      | BIT(TRACK_CUR_QUSE) |
		BIT(TRACK_PRE_ZCV)    | BIT(TRACK_CUR_ZCV)    | BIT(TRACK_PRE_AGING)     | BIT(TRACK_CUR_AGING) |
		BIT(TRACK_PRE_SHOW_AG)| BIT(TRACK_CUR_SHOW_AG)| BIT(TRACK_BATT_CC),

	/*zcv - report all data items*/
	[GAUGE_TRACK_CALI_FLAG_ZCV] =
		BIT(TRACK_PRE_VBAT)   | BIT(TRACK_CUR_VBAT)   | BIT(TRACK_PRE_TBAT)      | BIT(TRACK_CUR_TBAT) |
		BIT(TRACK_PRE_CAR_C)  | BIT(TRACK_CUR_CAR_C)  | BIT(TRACK_PRE_TOTAL_CAR) | BIT(TRACK_CUR_TOTAL_CAR) |
		BIT(TRACK_PRE_C_SOC)  | BIT(TRACK_CUR_C_SOC)  | BIT(TRACK_PRE_V_SOC)     | BIT(TRACK_CUR_V_SOC) |
		BIT(TRACK_PRE_SOC)    | BIT(TRACK_CUR_SOC)    | BIT(TRACK_PRE_UI_SOC)    | BIT(TRACK_CUR_UI_SOC) |
		BIT(TRACK_PRE_QMAX)   | BIT(TRACK_CUR_QMAX)   | BIT(TRACK_PRE_QUSE)      | BIT(TRACK_CUR_QUSE) |
		BIT(TRACK_PRE_ZCV)    | BIT(TRACK_CUR_ZCV)    | BIT(TRACK_PRE_AGING)     | BIT(TRACK_CUR_AGING) |
		BIT(TRACK_PRE_SHOW_AG)| BIT(TRACK_CUR_SHOW_AG)| BIT(TRACK_BATT_CC),

	/*aging - report all data items*/
	[GAUGE_TRACK_CALI_FLAG_AGING] =
		BIT(TRACK_PRE_VBAT)   | BIT(TRACK_CUR_VBAT)   | BIT(TRACK_PRE_TBAT)      | BIT(TRACK_CUR_TBAT) |
		BIT(TRACK_PRE_CAR_C)  | BIT(TRACK_CUR_CAR_C)  | BIT(TRACK_PRE_TOTAL_CAR) | BIT(TRACK_CUR_TOTAL_CAR) |
		BIT(TRACK_PRE_C_SOC)  | BIT(TRACK_CUR_C_SOC)  | BIT(TRACK_PRE_V_SOC)     | BIT(TRACK_CUR_V_SOC) |
		BIT(TRACK_PRE_SOC)    | BIT(TRACK_CUR_SOC)    | BIT(TRACK_PRE_UI_SOC)    | BIT(TRACK_CUR_UI_SOC) |
		BIT(TRACK_PRE_QMAX)   | BIT(TRACK_CUR_QMAX)   | BIT(TRACK_PRE_QUSE)      | BIT(TRACK_CUR_QUSE) |
		BIT(TRACK_PRE_ZCV)    | BIT(TRACK_CUR_ZCV)    | BIT(TRACK_PRE_AGING)     | BIT(TRACK_CUR_AGING) |
		BIT(TRACK_PRE_SHOW_AG)| BIT(TRACK_CUR_SHOW_AG)| BIT(TRACK_BATT_CC)
};

static int oplus_mt6375_cali_info_item_to_val(struct gauge_track_cali_info_s *info,
	enum oplus_track_item_idx idx)
{
#ifdef OPLUS_FEATURE_GAUGE_CALI_TRACK
	switch (idx) {
	case TRACK_PRE_VBAT:
	case TRACK_CUR_VBAT:
		return info->vbat;
	case TRACK_PRE_TBAT:
	case TRACK_CUR_TBAT:
		return info->tbat;
	case TRACK_PRE_CAR_C:
	case TRACK_CUR_CAR_C:
		return info->car_c;
	case TRACK_PRE_TOTAL_CAR:
	case TRACK_CUR_TOTAL_CAR:
		return info->total_car;
	case TRACK_PRE_C_SOC:
	case TRACK_CUR_C_SOC:
		return info->c_soc;
	case TRACK_PRE_V_SOC:
	case TRACK_CUR_V_SOC:
		return info->v_soc;
	case TRACK_PRE_SOC:
	case TRACK_CUR_SOC:
		return info->soc;
	case TRACK_PRE_UI_SOC:
	case TRACK_CUR_UI_SOC:
		return info->ui_soc;
	case TRACK_PRE_QMAX:
	case TRACK_CUR_QMAX:
		return info->qmax;
	case TRACK_PRE_QUSE:
	case TRACK_CUR_QUSE:
		return info->quse;
	case TRACK_PRE_ZCV:
	case TRACK_CUR_ZCV:
		return info->zcv;
	case TRACK_PRE_AGING:
	case TRACK_CUR_AGING:
		return info->aging_factor;
	case TRACK_PRE_SHOW_AG:
	case TRACK_CUR_SHOW_AG:
		return info->show_ag;
	case TRACK_BATT_CC:
		return info->batt_cc;
	default:
		return 0;
	}
#endif
	return 0;
}

static int oplus_mt6375_pack_cali_info(struct gauge_track_cali_info_s *pre,
	struct gauge_track_cali_info_s *cur, int reason, char *buf)
{
	int i;
	int index = 0;
	int offset = 0;
	unsigned int pattern;

	pattern = oplus_chg_track_pattern[reason];
	index = scnprintf(buf, OPLUS_CHG_TRACK_MTK_CALI_INFO_LEN,
			"$$track_reason@@%d$$err_scene@@%s$$info@@(", reason, "gauge_cali");
	for (i = TRACK_ITEM_START; i < TRACK_ITEM_END; i++) {
		if (i != TRACK_ITEM_START)
			index += scnprintf(buf + index, OPLUS_CHG_TRACK_MTK_CALI_INFO_LEN - index, ",");
		if((pattern & BIT(i)) == 0)
			continue;

		if (i == TRACK_BATT_CC) {
			offset++;
			index += scnprintf(buf + index, OPLUS_CHG_TRACK_MTK_CALI_INFO_LEN - index,
				"%d", oplus_mt6375_cali_info_item_to_val(cur, i));
			continue;
		}
		if ((offset + i) % 2 == 0)
			index += scnprintf(buf + index, OPLUS_CHG_TRACK_MTK_CALI_INFO_LEN - index,
				"%d", oplus_mt6375_cali_info_item_to_val(pre, i));
		else
			index += scnprintf(buf + index, OPLUS_CHG_TRACK_MTK_CALI_INFO_LEN - index,
				"%d", oplus_mt6375_cali_info_item_to_val(cur, i));
	}
	index += scnprintf(buf + index, OPLUS_CHG_TRACK_MTK_CALI_INFO_LEN - index, ")");

	if (index > OPLUS_CHG_TRACK_MTK_CALI_INFO_LEN) {
		chg_err("track info exceeds length limit.");
		return -EINVAL;
	}

	return index;
}

static struct mtk_battery* oplus_gauge_get_mtk_battery(void)
{
	struct mtk_gauge *gauge;
	struct power_supply *psy;
	static struct mtk_battery *gm;

	if (gm == NULL) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
		psy = power_supply_get_by_name("mt6375-gauge");
		if (psy == NULL) {
			chg_err("6375 psy is not rdy\n");
			psy = power_supply_get_by_name("mt6379-gauge1");
			if (psy == NULL) {
				chg_err("6379 psy is not rdy\n");
				psy = power_supply_get_by_name("mtk-gauge");
				if (psy == NULL) {
					chg_err("psy is not rdy\n");
					return NULL;
				}
			}
		}
#else
		psy = power_supply_get_by_name("mtk-gauge");
		if (psy == NULL) {
			chg_err("psy is not rdy\n");
			return NULL;
		}
#endif
		gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
		if (gauge == NULL) {
			chg_err("mtk_gauge is not rdy\n");
			return NULL;
		}
		gm = gauge->gm;
	}
	return gm;
}

static void oplus_chg_update_gauge_cali_track_info_internal(struct mtk_battery *gm,
	struct gauge_track_cali_info_s *info)
{
#ifdef OPLUS_FEATURE_GAUGE_CALI_TRACK
	if (gm == NULL || info == NULL) {
		chg_err("input is null\n");
		return;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	info->tbat = gm->bm->bs_data.bat_batt_temp;
#else
	info->tbat = gm->bs_data.bat_batt_temp;
#endif
	info->vbat = gm->batt_volt;
	info->ui_soc = gm->fg_cust_data.ui_old_soc;
	info->soc = gm->soc;
	info->c_soc = gm->fg_cust_data.c_soc;
	info->v_soc = gm->fg_cust_data.v_soc;
	info->car_c = gm->car_c;
	info->total_car = gm->total_car;
	info->aging_factor = gm->aging_factor;
	info->qmax = gm->algo_qmax;
	info->quse = gm->prev_batt_fcc;
	info->zcv = gm->zcv;
	info->batt_cc = gm->bat_cycle;
	info->show_ag = gm->soh;
#endif /*OPLUS_FEATURE_GAUGE_CALI_TRACK*/
}

static int oplus_mt6375_trigger_gauge_cali_track(struct gauge_track_cali_info_s *pre_info,
	struct gauge_track_cali_info_s *cur_info, int reason)
{
	char *buf = NULL;
	int len = 0;
	struct chip_mt6375_gauge *chip;

	chg_info("trigger reason:%d\n", reason);

	if (g_mt6375_chip == NULL) {
		chg_err("chip is null\n");
		return -EINVAL;
	}
	chip = g_mt6375_chip;

	buf = kzalloc(OPLUS_CHG_TRACK_MTK_CALI_INFO_LEN, GFP_KERNEL);
	if (buf == NULL) {
		chg_err("buf alloc error.\n");
		return -ENOMEM;
	}

	len = oplus_mt6375_pack_cali_info(pre_info, cur_info, reason, buf);

	if (len > 0) {
		oplus_chg_ic_creat_err_msg(chip->ic_dev, OPLUS_IC_ERR_GAUGE, TRACK_GAGUE_MTK_CALI_INFO, buf);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_ERR);
	}
	kfree(buf);
	return 0;
}

#ifdef OPLUS_FEATURE_GAUGE_CALI_TRACK /*define in mtk_battery.h*/
static struct gauge_track_ops mtk_plat_gauge_track_ops =
{
	.mtk_gauge_cali_track = oplus_mt6375_trigger_gauge_cali_track,
};
#endif /*OPLUS_FEATURE_GAUGE_CALI_TRACK*/

#define OPLUS_GAUGE_CALI_TRACK_PLUG_TIME_THD_MS (2 * 60 * 1000)
static void oplus_mt6375_gauge_cali_track_by_plug_work(struct work_struct *work)
{
	static struct gauge_track_cali_info_s pre_info;
	struct gauge_track_cali_info_s cur_info;
	static ktime_t online_time;
	struct chip_mt6375_gauge *chip;
	struct mtk_battery *gm;

	gm = oplus_gauge_get_mtk_battery();

	if (gm == NULL) {
		chg_err("gm is null\n");
		return;
	}

	chip = container_of(work, struct chip_mt6375_gauge, gauge_cali_track_by_plug_work);

	if (chip->wired_online) {
		online_time = ktime_get();
		oplus_chg_update_gauge_cali_track_info_internal(gm, &pre_info);
	} else {
		if(ktime_ms_delta(ktime_get(), online_time) < OPLUS_GAUGE_CALI_TRACK_PLUG_TIME_THD_MS)
			return;

		oplus_chg_update_gauge_cali_track_info_internal(gm, &cur_info);
		oplus_mt6375_trigger_gauge_cali_track(&pre_info, &cur_info, GAUGE_TRACK_CALI_FLAG_PLUGOUT);
	}
}

static void oplus_mt6375_gauge_cali_track_by_full_work(struct work_struct *work)
{
#ifdef OPLUS_FEATURE_GAUGE_CALI_TRACK
	struct gauge_track_cali_info_s info;
	struct mtk_battery *gm;

	gm = oplus_gauge_get_mtk_battery();

	if (gm == NULL) {
		chg_err("gm is null\n");
		return;
	}

	oplus_chg_update_gauge_cali_track_info_internal(gm, &info);
	oplus_mt6375_trigger_gauge_cali_track(&gm->pre_info, &info, GAUGE_TRACK_CALI_FLAG_CHG_FULL);
#endif
}

static void oplus_mt6375_gauge_online_handler(struct chip_mt6375_gauge *chip)
{
	union mms_msg_data data = {0};

	if (chip == NULL) {
		chg_err("chip is null\n");
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data, false);
	chip->wired_online = !!data.intval;

	if (chip->mtk_gauge_cali_track_support)
		schedule_work(&chip->gauge_cali_track_by_plug_work);
}

static void oplus_mt6375_gauge_chg_full_handler(struct chip_mt6375_gauge *chip)
{
	bool chg_full;
	union mms_msg_data data = {0};

	if (chip == NULL) {
		chg_err("chip is null\n");
		return;
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_CHG_FULL, &data, false);
	chg_full = !!data.intval;

	if(chg_full && chip->mtk_gauge_cali_track_support)
		schedule_work(&chip->gauge_cali_track_by_full_work);
}

static void oplus_mt6375_gauge_subs_wired_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct chip_mt6375_gauge *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			oplus_mt6375_gauge_online_handler(chip);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_mt6375_gauge_subs_comm_callback(struct mms_subscribe *subs,
	enum mms_msg_type type, u32 id, bool sync)
{
	struct chip_mt6375_gauge *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_CHG_FULL:
			oplus_mt6375_gauge_chg_full_handler(chip);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_mt6375_gauge_wls_subs_callback(struct mms_subscribe *subs,
						 enum mms_msg_type type, u32 id, bool sync)
{
	struct chip_mt6375_gauge *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WLS_ITEM_PRESENT:
			oplus_mms_get_item_data(chip->wls_topic, id, &data, false);
			chip->wls_online = !!data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_mt6375_gauge_subscribe_wls_topic(struct oplus_mms *topic,
						   void *prv_data)
{
	struct chip_mt6375_gauge *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wls_topic = topic;
	chip->wls_subs = oplus_mms_subscribe(chip->wls_topic, chip,
					     oplus_mt6375_gauge_wls_subs_callback,
					     chip->ic_dev->manu_name);
	if (IS_ERR_OR_NULL(chip->wls_subs)) {
		chg_err("subscribe wls topic error, rc=%ld\n", PTR_ERR(chip->wls_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_PRESENT, &data, true);
	chip->wls_online = !!data.intval;

	return;
}

static void oplus_mt6375_gauge_subscribe_wired_topic(struct oplus_mms *topic, void *data)
{
	struct chip_mt6375_gauge *chip = data;

	chip->wired_topic = topic;
	chip->wired_subs = oplus_mms_subscribe(chip->wired_topic,
		chip, oplus_mt6375_gauge_subs_wired_callback, chip->ic_dev->manu_name);
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}
}

static void oplus_mt6375_gauge_subscribe_comm_topic(struct oplus_mms *topic, void *data)
{
	struct chip_mt6375_gauge *chip = data;

	chip->comm_topic = topic;
	chip->comm_subs = oplus_mms_subscribe(chip->comm_topic,
		chip, oplus_mt6375_gauge_subs_comm_callback, chip->ic_dev->manu_name);
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe comm topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}
}

static int oplus_mt6375_guage_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct mtk_battery *gm = NULL;
	struct chip_mt6375_gauge *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev  is NULL");
		return -EAGAIN;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -EAGAIN;
	}

	ic_dev->online = false;

	if (g_gauge_chip == NULL) {
		ic_dev->online = false;
		chg_err("ic_dev->online = %d\n", ic_dev->online);
		return -EAGAIN;
	}

	if(g_gauge_chip->gauge_ops->get_battery_soc() < 0) {
		chg_err("soc abnormal");
		return -EAGAIN;
	}

	ic_dev->online = true;

	if (chip->mtk_gauge_cali_track_support) {
		gm = oplus_gauge_get_mtk_battery();
#ifdef OPLUS_FEATURE_GAUGE_CALI_TRACK
		BUILD_BUG_ON_MSG(TRACK_ITEM_END > 32, "oplus_chg_track_pattern only has 32 bits");
		if (gm != NULL)
			gm->oplus_track_ops = &mtk_plat_gauge_track_ops;
#endif /*OPLUS_FEATURE_GAUGE_CALI_TRACK*/
		INIT_WORK(&chip->gauge_cali_track_by_plug_work, oplus_mt6375_gauge_cali_track_by_plug_work);
		INIT_WORK(&chip->gauge_cali_track_by_full_work, oplus_mt6375_gauge_cali_track_by_full_work);
	}

	oplus_mms_wait_topic("wired", oplus_mt6375_gauge_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("common", oplus_mt6375_gauge_subscribe_comm_topic, chip);
	oplus_mms_wait_topic("wireless", oplus_mt6375_gauge_subscribe_wls_topic, chip);

	chg_info("oplus_mt6375_guage_init, ic_dev->online = %d\n", ic_dev->online);
	return 0;
}

static int oplus_mt6375_guage_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	chg_info("oplus_mt6375_guage_exit, ic_dev->online = %d\n", ic_dev->online);

	return 0;
}

static int oplus_mt6375_guage_get_batt_vol(struct oplus_chg_ic_dev *ic_dev, int index, int *vol_mv)
{
	*vol_mv = g_gauge_chip->gauge_ops->get_battery_mvolts();

	return 0;
}

static int oplus_mt6375_guage_get_batt_max(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	*vol_mv = g_gauge_chip->gauge_ops->get_battery_mvolts();

	return 0;
}
static int oplus_mt6375_guage_get_batt_min(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	*vol_mv = g_gauge_chip->gauge_ops->get_battery_mvolts();

	return 0;
}

static int oplus_mt6375_guage_get_batt_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	*curr_ma = g_gauge_chip->gauge_ops->get_average_current();

	return 0;
}

static int oplus_mt6375_guage_get_batt_temp(struct oplus_chg_ic_dev *ic_dev, int *temp)
{
	int soc;
	soc = g_gauge_chip->gauge_ops->get_battery_soc();
	if (soc < 0) {
		return -1;
	}
	*temp = g_gauge_chip->gauge_ops->get_battery_temperature();

	return 0;
}

static int oplus_mt6375_guage_get_batt_soc(struct oplus_chg_ic_dev *ic_dev, int *soc)
{
	*soc = g_gauge_chip->gauge_ops->get_battery_soc();
	if (*soc < 0) {
		return -1;
	}

	return 0;
}

static int oplus_mt6375_guage_get_batt_fcc(struct oplus_chg_ic_dev *ic_dev, int *fcc)
{
	*fcc = g_gauge_chip->gauge_ops->get_battery_fcc();

	return 0;
}

static int oplus_mt6375_guage_get_batt_cc(struct oplus_chg_ic_dev *ic_dev, int *cc)
{
	*cc = g_gauge_chip->gauge_ops->get_battery_cc();

	return 0;
}

static int oplus_mt6375_guage_get_batt_rm(struct oplus_chg_ic_dev *ic_dev, int *rm)
{
	*rm = g_gauge_chip->gauge_ops->get_batt_remaining_capacity();

	return 0;
}

static int oplus_mt6375_guage_get_batt_soh(struct oplus_chg_ic_dev *ic_dev, int *soh)
{
	*soh = g_gauge_chip->gauge_ops->get_battery_soh();

	return 0;
}

static int oplus_mt6375_guage_get_batt_auth(struct oplus_chg_ic_dev *ic_dev, bool *pass)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	if(g_gauge_chip == NULL) {
		chg_err("g_gauge_chip == NULL\n");
		return -ENODEV;
	}

	*pass = g_gauge_chip->gauge_ops->get_battery_authenticate();
	chg_info("*pass = %d\n", *pass);

	return 0;
}

static int oplus_mt6375_guage_get_afi_update_done(struct oplus_chg_ic_dev *ic_dev, bool *status)
{
	*status = true;
	return 0;
}

static int oplus_mt6375_guage_get_batt_hmac(struct oplus_chg_ic_dev *ic_dev, bool *pass)
{
	if (g_gauge_chip &&
	    g_gauge_chip->gauge_ops &&
	    g_gauge_chip->gauge_ops->get_battery_hmac)
		*pass = g_gauge_chip->gauge_ops->get_battery_hmac();
	else
		*pass = true;

	chg_info("*pass = %d\n", *pass);
	return 0;
}

#define RESET_GAUGE_RETRY_TIMES 2
static int oplus_mt6375_guage_set_reset_gauge(struct oplus_chg_ic_dev *ic_dev, const int *buf)
{
	int enable = 0;
	int reset_soh = 0;
	int reset_cc = 0;
	int rc = 0;
	int get_soh;
	int get_cc;
	int retry_count = 0;

	if (buf == NULL) {
		chg_err("oplus_mt6375_guage_set_reset_gauge buf= NULL\n");
		return -EINVAL;
	}

	enable = buf[0];
	reset_soh = buf[1];
	reset_cc = buf[2];
	chg_info("oplus_mt6375_guage_set_reset_gauge = %d %d %d\n", enable, reset_soh, reset_cc);

	if (enable != 1) {
		chg_debug("%s: Reset not enabled\n", __func__);
		goto out;
	}

	do {
		retry_count++;
		if (!g_gauge_chip) {
			chg_err("%s: Gauge chip not initialized\n", __func__);
			rc = -ENODEV;
			break;
		}

		if (!g_gauge_chip->gauge_ops) {
			chg_err("%s: Gauge ops undefined\n", __func__);
			rc = -ENOTSUPP;
			break;
		}

		if (!g_gauge_chip->gauge_ops->set_gauge_aging) {
			chg_err("%s: set_gauge_aging not supported\n", __func__);
			rc = -ENOTSUPP;
			break;
		}

		if (!g_gauge_chip->gauge_ops->set_gauge_cycles) {
			chg_err("%s: set_gauge_cycles not supported\n", __func__);
			rc = -ENOTSUPP;
			break;
		}

		get_soh = g_gauge_chip->gauge_ops->get_battery_soh();
		if (get_soh != reset_soh) {
			g_gauge_chip->gauge_ops->set_gauge_aging(reset_soh);
			continue;
		}

		get_cc = g_gauge_chip->gauge_ops->get_battery_cc();
		chg_info("%s: Reset retry_count = %d %d %d %d\n", __func__, retry_count, enable, get_soh, get_cc);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
		if (get_cc != reset_cc) {
#else
		if (get_cc != 0) {
#endif
			g_gauge_chip->gauge_ops->set_gauge_cycles(reset_cc);
			continue;
		}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
		if (get_soh == reset_soh && get_cc == reset_cc) {
#else
		if (get_soh == reset_soh && get_cc == 0) {
#endif
			chg_info("%s: Reset successful\n", __func__);
			break;
		}
	} while (retry_count < RESET_GAUGE_RETRY_TIMES);

out:
	return rc;
}

static int oplus_mt6375_guage_set_batt_full(struct oplus_chg_ic_dev *ic_dev, bool full)
{
	g_gauge_chip->gauge_ops->set_battery_full(full);
	chg_info("full = %d\n", full);

	return 0;
}

static int oplus_mt6375_guage_update_dod0(struct oplus_chg_ic_dev *ic_dev)
{
	return g_gauge_chip->gauge_ops->update_battery_dod0();
}

static int oplus_mt6375_guage_update_soc_smooth_parameter(struct oplus_chg_ic_dev *ic_dev)
{
	return g_gauge_chip->gauge_ops->update_soc_smooth_parameter();
}

static int oplus_mt6375_guage_get_batt_num(struct oplus_chg_ic_dev *ic_dev, int *num)
{
	struct chip_mt6375_gauge *chip;

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	*num = chip->batt_num;

	return 0;
}

static int oplus_mt6375_guage_get_gauge_type(struct oplus_chg_ic_dev *ic_dev, int *gauge_type)
{
	struct chip_mt6375_gauge *chip;
	int rc;
	int temp;

	if (ic_dev == NULL || gauge_type == NULL) {
		chg_err("oplus_chg_ic_dev or gauge_type is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	rc = oplus_mt6375_guage_get_batt_temp(ic_dev, &temp);
	if (rc < 0 || temp == -400) {
		*gauge_type = GAUGE_TYPE_UNKNOW;
		return 0;
	}

	*gauge_type = GAUGE_TYPE_PLATFORM;

	return 0;
}

static int oplus_mt6375_guage_get_batt_exist(struct oplus_chg_ic_dev *ic_dev, bool *exist)
{
	*exist = true;

	return 0;
}

static int oplus_mt6375_gauge_get_gauge_car_c(struct oplus_chg_ic_dev *ic_dev, int *car_c)
{
	int rc;

	if (car_c == NULL) {
		chg_err("car_c is NULL\n");
		return -EINVAL;
	}
	if (g_gauge_chip == NULL || g_gauge_chip->gauge_ops == NULL) {
		chg_err("g_gauge_chip is null.\n");
		return -EINVAL;
	}

	if (g_gauge_chip->gauge_ops->get_gauge_car_c == NULL)
		return -EINVAL;

	rc = g_gauge_chip->gauge_ops->get_gauge_car_c(car_c);
	/* Change unit from 0.1mAh to 1mAh, rounding to the nearest whole number.*/
	*car_c = (*car_c + 5) / 10;
	if (rc < 0) {
		chg_err("failed to get car_c from mtk\n");
		*car_c = 0;
		return -EINVAL;
	}
	return 0;
}

static int oplus_mt6375_gauge_get_qmax(struct oplus_chg_ic_dev *ic_dev, int batt_id, int *qmax)
{
	int rc;

	if (qmax == NULL) {
		chg_err("qmax is null.\n");
		return -EINVAL;
	}

	if (g_gauge_chip == NULL || g_gauge_chip->gauge_ops == NULL) {
		chg_err("g_gauge_chip is null.\n");
		return -EINVAL;
	}

	if (g_gauge_chip->gauge_ops->get_batt_qmax == NULL)
		return -EINVAL;

	/*single cell battery, qmax1 == qmax2*/
	rc = g_gauge_chip->gauge_ops->get_batt_qmax(qmax, qmax);
	if (rc < 0) {
		chg_err("failed to get qmax from mtk\n");
		*qmax = 0;
		return -EINVAL;
	}
	return 0;
}

static bool is_cpa_topic_available(struct chip_mt6375_gauge *chip)
{
	if (!chip->cpa_topic)
		chip->cpa_topic = oplus_mms_get_by_name("cpa");

	return !!chip->cpa_topic;
}

#define POWER_50W 50000
#define POWER_20W 20000
#define POWER_10W 10000
static int oplus_mt6375_gauge_set_power_sel(struct oplus_chg_ic_dev *ic_dev, int type,
					   int adapter_id, bool pd_svooc)
{
	static int last_curve_index = -1;
	int target_index_curve = -1;
	struct chip_mt6375_gauge *chip;
	int power = 0;
	int rc = 0;
	union mms_msg_data data = { 0 };

	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip->mtk_gauge_power_sel_support)
		return -1;

	if (!chip->mtk_gauge_power_sel_by_power_support) {
		if (type == CHARGER_SUBTYPE_DEFAULT)
			target_index_curve = CHARGER_NORMAL_CHG_CURVE;
		else if (type == CHARGER_SUBTYPE_FASTCHG_SVOOC)
			target_index_curve = CHARGER_FASTCHG_SVOOC_CURVE;
		else if (type == CHARGER_SUBTYPE_FASTCHG_VOOC ||
			 type == CHARGER_SUBTYPE_PD ||
			 type == CHARGER_SUBTYPE_QC)
			target_index_curve = CHARGER_FASTCHG_VOOC_AND_QCPD_CURVE;
		else if (type == CHARGER_SUBTYPE_UFCS ||
			 type == CHARGER_SUBTYPE_PPS)
			target_index_curve = CHARGER_FASTCHG_PPS_AND_UFCS_CURVE;
		else
			target_index_curve = CHARGER_FASTCHG_SVOOC_CURVE;
	} else {
		if (chip->wls_online) {
			rc = oplus_mms_get_item_data(chip->wls_topic, WLS_ITEM_MAX_POWER, &data, true);
			if (rc == 0) {
				power = data.intval;
				chg_info("power=%d, wls_online=%d\n", power, chip->wls_online);
			}
		} else {
			if (is_cpa_topic_available(chip))
				power = oplus_cpa_get_actual_used_power(chip->cpa_topic);
		}

		if (power <= POWER_10W)
			target_index_curve = CHARGER_NORMAL_CHG_ZCV_R0;
		else if (power >= POWER_50W)
			target_index_curve = CHARGER_FASTCHG_ZCV_R1;
		else if (power >= POWER_20W && power < POWER_50W)
			target_index_curve = CHARGER_FASTCHG_ZCV_R2;
		else if (power > POWER_10W && power < POWER_20W)
			target_index_curve = CHARGER_FASTCHG_ZCV_R3;
	}

	if (target_index_curve != last_curve_index) {
		chg_err("target_index_curve=%d, last_curve_index=%d, type[%d], adapter_id[%d]\n",
			target_index_curve, last_curve_index, type, adapter_id);
		g_gauge_chip->gauge_ops->set_gauge_power_sel(target_index_curve);
		last_curve_index = target_index_curve;
	}

	return 0;
}

static int oplus_get_battery_temperature(void)
{
	int temp = 0;

	if (NULL == g_gauge_chip ||
	    NULL == g_gauge_chip->gauge_ops ||
	    NULL == g_gauge_chip->gauge_ops->get_battery_temperature)
		return 25;

	temp = g_gauge_chip->gauge_ops->get_battery_temperature();
	return temp;
}

static int oplus_get_sub_btb_state(struct chip_mt6375_gauge *info)
{
	int temp = 0;
	int state = BATT_BTB_STATE_NOT_SUPPORT;

	if (NULL == info) {
		chg_err("info is NULL");
		return -ENODEV;
	}

	/* check if the btb_state can be get by battery temp which is by ADC channel. */
	if (info->sub_btb_valid_temp[0] || info->sub_btb_valid_temp[1]) {
		/* get the real batt temp */
		temp = oplus_get_battery_temperature();

		/* valid temp range: min < ~ < max graphite  battery of sub_btb is online. */
		if (info->sub_btb_valid_temp[0] && info->sub_btb_valid_temp[1]) {
			if (temp > info->sub_btb_valid_temp[1] || temp < info->sub_btb_valid_temp[0])
				state = BATT_BTB_STATE_NOT_CONNECT;
			else
				state = BATT_BTB_STATE_CONNECT;
		} else if (info->sub_btb_valid_temp[0] && !info->sub_btb_valid_temp[1]) {
			/*
			 * valid temp range: < max, for [1000, 0], more than 1000,
			 * silicon battery of sub_btb is Onlne.
			 */
			if (temp < info->sub_btb_valid_temp[0])
				state = BATT_BTB_STATE_NOT_CONNECT;
			else
				state = BATT_BTB_STATE_CONNECT;
		}
	}
	chg_debug("temp = %d, temp_valid[%d]-[%d], state = %d",
		 temp, info->sub_btb_valid_temp[0],
		 info->sub_btb_valid_temp[1], state);

	return state;
}

static int oplus_gauge_get_sub_btb_state(struct oplus_chg_ic_dev *ic_dev,
					 enum oplus_sub_btb_state *state)
{
	struct chip_mt6375_gauge *chip;
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

static int oplus_mtk_get_dec_fg_type(struct oplus_chg_ic_dev *ic_dev, int*fg_type)
{
	if (!ic_dev || !fg_type) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*fg_type = DEC_CV_MTK_FG;

	return 0;
}

static int oplus_mtk_get_dec_cv_soh(struct oplus_chg_ic_dev *ic_dev, int*dec_soh)
{
	if (!ic_dev || !dec_soh) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*dec_soh = g_gauge_chip->gauge_ops->get_battery_cc();

	return 0;
}

static int oplus_mtk_sync_plugin_state(struct oplus_chg_ic_dev *ic_dev)
{
	if (g_gauge_chip == NULL || g_gauge_chip->gauge_ops == NULL) {
		chg_err("g_gauge_chip is null.\n");
		return -EINVAL;
	}

	if (g_gauge_chip->gauge_ops->sync_plugin_state == NULL)
		return -EINVAL;

	g_gauge_chip->gauge_ops->sync_plugin_state();

	return 0;
}

static int oplus_get_term_vol_index(struct chip_mt6375_gauge *chip)
{
	int temp = 0;
	int index = TERM_VOL_COUNT - 1;
	int i = 0;
	temp = oplus_get_battery_temperature();

	for (i = 0; i < TERM_VOL_COUNT - 1; i++) {
		if (temp >= chip->term_vol_temp_range[i]) {
			index = i;
			break;
		}
	}

	chg_info("temp = %d, index = %d\n", temp, index);
	return index;
}

#define TERM_VOL_MULTIPLIER  10
static int oplus_get_term_vol_delta(int volt, struct chip_mt6375_gauge *chip)
{
	int index = 0;
	int term_vol_delta = 0;

	index = oplus_get_term_vol_index(chip);
	term_vol_delta = volt*TERM_VOL_MULTIPLIER - chip->term_vol_base[index];
	chg_info("term_vol_delta = %d\n", term_vol_delta);

	return term_vol_delta;
}

static int oplus_mtk_set_sili_ic_deep_term_volt(struct oplus_chg_ic_dev *ic_dev, int volt)
{
	struct chip_mt6375_gauge *chip;
	int term_vol_delta = 0;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (g_gauge_chip == NULL || g_gauge_chip->gauge_ops == NULL || chip == NULL) {
		chg_err("g_gauge_chip is null.\n");
		return -EINVAL;
	}

	if (g_gauge_chip->gauge_ops->set_sili_ic_deep_term_volt == NULL || g_mt6375_chip->term_vol_support == false)
		return -EINVAL;

	term_vol_delta = oplus_get_term_vol_delta(volt, chip);

	g_gauge_chip->gauge_ops->set_sili_ic_deep_term_volt(term_vol_delta);
	return 0;
}

static int oplus_mtk_get_sili_ic_deep_term_volt(struct oplus_chg_ic_dev *ic_dev, int *volt)
{
	if (g_gauge_chip == NULL || g_gauge_chip->gauge_ops == NULL) {
		chg_err("g_gauge_chip is null.\n");
		return -EINVAL;
	}

	if (g_gauge_chip->gauge_ops->get_sili_ic_deep_term_volt == NULL)
		return -EINVAL;

	*volt = g_gauge_chip->gauge_ops->get_sili_ic_deep_term_volt();
	return 0;
}

static int oplus_mtk_set_vct(struct oplus_chg_ic_dev *ic_dev, int value)
{
	int vbatt_full = 0;
	struct chip_mt6375_gauge *chip;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (g_gauge_chip == NULL || g_gauge_chip->gauge_ops == NULL || chip == NULL) {
		chg_err("g_gauge_chip is null.\n");
		return -EINVAL;
	}

	if (g_gauge_chip->gauge_ops->set_fg_vct == NULL)
		return -EINVAL;

	vbatt_full = chip->vbatt_full_cv - value * TERM_VOL_MULTIPLIER;
	g_gauge_chip->gauge_ops->set_fg_vct(vbatt_full);

	return 0;
}

static int oplus_mtk_get_vct(struct oplus_chg_ic_dev *ic_dev, int *value)
{
	if (g_gauge_chip == NULL || g_gauge_chip->gauge_ops == NULL) {
		chg_err("g_gauge_chip is null.\n");
		return -EINVAL;
	}

	if (g_gauge_chip->gauge_ops->get_fg_vct == NULL)
		return -EINVAL;

	*value = g_gauge_chip->gauge_ops->get_fg_vct();

	return 0;
}

#define GAUGE_INDEX_MAIN 0
#define GAUGE_INDEX_SUB 1

static int oplus_mtk_get_battery_dod0(struct oplus_chg_ic_dev *ic_dev, int index, int *dod0)
{
	if (g_gauge_chip == NULL || g_gauge_chip->gauge_ops == NULL) {
		chg_err("g_gauge_chip is null.\n");
		return -EINVAL;
	}

	if (g_gauge_chip->gauge_ops->get_battery_dod == NULL)
		return -EINVAL;

	switch (index) {
	case GAUGE_INDEX_MAIN:
	case GAUGE_INDEX_SUB:
		*dod0 = g_gauge_chip->gauge_ops->get_battery_dod(index);
		break;
	default:
		chg_info("index(=%d), over size\n", index);
		return -EINVAL;
	}

	return 0;
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
					       oplus_mt6375_guage_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					       oplus_mt6375_guage_exit);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL,
					       oplus_mt6375_guage_get_batt_vol);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX,
					       oplus_mt6375_guage_get_batt_max);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN,
					       oplus_mt6375_guage_get_batt_min);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR,
			oplus_mt6375_guage_get_batt_curr);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP,
			oplus_mt6375_guage_get_batt_temp);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC,
					       oplus_mt6375_guage_get_batt_soc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC,
					       oplus_mt6375_guage_get_batt_fcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_CC,
					       oplus_mt6375_guage_get_batt_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_RM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_RM,
					       oplus_mt6375_guage_get_batt_rm);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH,
					       oplus_mt6375_guage_get_batt_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_AUTH,
			oplus_mt6375_guage_get_batt_auth);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_GET_BATT_HMAC,
			oplus_mt6375_guage_get_batt_hmac);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_SET_BATT_FULL,
			oplus_mt6375_guage_set_batt_full);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_RESET_GAUGE_DATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_SET_RESET_GAUGE_DATE,
			oplus_mt6375_guage_set_reset_gauge);
		break;
	case OPLUS_IC_FUNC_GAUGE_UPDATE_DOD0:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_UPDATE_DOD0,
					       oplus_mt6375_guage_update_dod0);
		break;
	case OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH:
		func = OPLUS_CHG_IC_FUNC_CHECK(
			OPLUS_IC_FUNC_GAUGE_UPDATE_SOC_SMOOTH,
			oplus_mt6375_guage_update_soc_smooth_parameter);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM,
					       oplus_mt6375_guage_get_batt_num);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_GAUGE_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_GAUGE_TYPE,
					       oplus_mt6375_guage_get_gauge_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_AFI_UPDATE_DONE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_AFI_UPDATE_DONE,
					       oplus_mt6375_guage_get_afi_update_done);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST,
					       oplus_mt6375_guage_get_batt_exist);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_BATTERY_CURVE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_BATTERY_CURVE,
					       oplus_mt6375_gauge_set_power_sel);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_QMAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_QMAX,
					       oplus_mt6375_gauge_get_qmax);
        break;
	case OPLUS_IC_FUNC_GAUGE_GET_GAUGE_CAR_C:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_GAUGE_CAR_C,
					       oplus_mt6375_gauge_get_gauge_car_c);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_SUB_BTB_CONNECT_STATE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_SUB_BTB_CONNECT_STATE,
					       oplus_gauge_get_sub_btb_state);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEC_FG_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEC_FG_TYPE,
						   oplus_mtk_get_dec_fg_type);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEC_CV_SOH,
						   oplus_mtk_get_dec_cv_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_SYNC_PLUGIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SYNC_PLUGIN,
							oplus_mtk_sync_plugin_state);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DEEP_TERM_VOLT,
							oplus_mtk_get_sili_ic_deep_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_DEEP_TERM_VOLT,
							oplus_mtk_set_sili_ic_deep_term_volt);
		break;
	case OPLUS_IC_FUNC_GAUGE_SET_VCT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_SET_VCT, oplus_mtk_set_vct);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_VCT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_VCT, oplus_mtk_get_vct);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_DOD0:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_DOD0,
						oplus_mtk_get_battery_dod0);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq mt6375_guage_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_ONLINE },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{ .virq_id = OPLUS_IC_VIRQ_RESUME },
};

static void oplus_mt6375_guage_parse_dt(struct chip_mt6375_gauge *chip)
{
	int rc = 0;
	int i = 0;
	atomic_set(&chip->locked, 0);
	atomic_set(&chip->suspended, 0);
	rc = of_property_read_u32(chip->dev->of_node, "oplus,batt_num",
				  &chip->batt_num);
	if (rc < 0) {
		chg_err("can't get oplus,batt_num, rc = %d\n", rc);
		chip->batt_num = 1;
	}
	chip->mtk_gauge_power_sel_support =
		of_property_read_bool(chip->dev->of_node, "oplus,mtk_gauge_power_sel_support");

	chip->mtk_gauge_power_sel_by_power_support =
		of_property_read_bool(chip->dev->of_node, "oplus,mtk_gauge_power_sel_by_power_support");

	chg_info("batt_num = %d, mtk_gauge_power_sel_support = %d, mtk_gauge_power_sel_by_power_support:%d\n",
		 chip->batt_num, chip->mtk_gauge_power_sel_support, chip->mtk_gauge_power_sel_by_power_support);

	chip->mtk_gauge_cali_track_support =
		of_property_read_bool(chip->dev->of_node, "oplus,mtk_gauge_cali_track_support");
	chg_info("mtk_gauge_cali_track_support = %d\n", chip->mtk_gauge_cali_track_support);

	rc = of_property_read_u32(chip->dev->of_node, "oplus,vbatt_full_cv", &chip->vbatt_full_cv);
	if (rc < 0) {
		chg_err("can't get oplus,vbatt_full_cv, rc = %d\n", rc);
		chip->vbatt_full_cv = 44500;
	}
	chg_info("vbatt_full_cv = %d\n", chip->vbatt_full_cv);

	rc = of_property_count_elems_of_size(chip->dev->of_node, "oplus,term_vol_temp_range", sizeof(u32));
	chg_info("term_vol_temp_range count = %d\n", rc);
	if (rc == TERM_VOL_COUNT - 1) {
		rc = read_signed_data_from_node(chip->dev->of_node, "oplus,term_vol_temp_range",
			(s32 *)(chip->term_vol_temp_range),
			TERM_VOL_COUNT - 1);
		for (i = 0; i < TERM_VOL_COUNT - 1; i++) {
			chg_info("term_vol_temp_range[%d] = %d\n", i, chip->term_vol_temp_range[i]);
		}
	}

	rc = of_property_count_elems_of_size(chip->dev->of_node, "oplus,term_vol_base", sizeof(u32));
	if (rc ==  TERM_VOL_COUNT) {
		rc = of_property_read_u32_array(chip->dev->of_node, "oplus,term_vol_base",
			(u32 *)(chip->term_vol_base), rc);
	}
	if (rc < 0) {
		chip->term_vol_support = false;
	} else {
		chip->term_vol_support = true;
	}
	chg_info("term_vol_support = %d\n", chip->term_vol_support);
}

static int oplus_gauge_sub_btb_parse_dt(struct chip_mt6375_gauge *chip)
{
	int rc = 0;
	int i = 0;

	if (chip == NULL)
		return -ENODEV;

	rc = read_signed_data_from_node(chip->dev->of_node, "oplus,sub_btb_valid_temp",
					(s32 *)(chip->sub_btb_valid_temp),
					OPLUS_SUB_BTB_MAX);
	if (rc < 0) {
		chg_err("get sub_btb_valid_temp, rc = %d\n", rc);
		for (i = 0; i < OPLUS_SUB_BTB_MAX - 1; i++)
			chip->sub_btb_valid_temp[i] = 0;
	} else {
		for (i = 0; i < OPLUS_SUB_BTB_MAX - 1; i++)
			chg_info(" valid_temp[%d] = %d", i, chip->sub_btb_valid_temp[i]);
	}

	return rc;
}


static int mt6375_guage_driver_probe(struct platform_device *pdev)
{
	struct chip_mt6375_gauge *chip;
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	int rc = 0;
	struct device_node *node = oplus_get_node_by_child_gauge(pdev->dev.of_node);

	if (node && of_property_read_bool(node, "skip_gauge_probe")) {
		dev_err(&pdev->dev, "non-external gauge, skip pmic gauge driver probe\n");
		return -ENODEV;
	}

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
	g_gauge_chip = oplus_mtk_gauge_init();
#endif
	g_mt6375_chip = chip;
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	atomic_set(&chip->suspended, 0);
	mutex_init(&chip->chip_mutex);
	oplus_mt6375_guage_parse_dt(chip);
	oplus_gauge_sub_btb_parse_dt(chip);

	chip->soc_pre = 50;
	chip->batt_vol_pre = 3800;
	chip->fc_pre = 0;
	chip->qm_pre = 0;
	chip->pd_pre = 0;
	chip->rcu_pre = 0;
	chip->rcf_pre = 0;
	chip->fcu_pre = 0;
	chip->fcf_pre = 0;
	chip->sou_pre = 0;
	chip->do0_pre = 0;
	chip->doe_pre = 0;
	chip->trm_pre = 0;
	chip->pc_pre = 0;
	chip->qs_pre = 0;
	chip->max_vol_pre = 3800;
	chip->min_vol_pre = 3800;
	chip->current_pre = 999;
	chip->protect_check_done = false;
	chip->afi_update_done = true;
	chip->disabled = false;
	chip->error_occured = false;
	chip->need_check = true;
	chip->protect_check_done = true;

	atomic_set(&chip->locked, 0);
	rc = of_property_read_u32(chip->dev->of_node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		goto error;
	}
	rc = of_property_read_u32(chip->dev->of_node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto error;
	}
	ic_cfg.name = chip->dev->of_node->name;
	ic_cfg.index = ic_index;
	chip->device_type = 0;

	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "gauge-mt6375");

	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = mt6375_guage_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(mt6375_guage_virq_table);
	ic_cfg.of_node = chip->dev->of_node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", chip->dev->of_node->name);
		goto error;
	}
	chg_info("register %s\n", chip->dev->of_node->name);

#ifndef CONFIG_OPLUS_CHARGER_MTK
	oplus_vooc_get_fastchg_started_pfunc(&oplus_vooc_get_fastchg_started);
	oplus_vooc_get_fastchg_ing_pfunc(&oplus_vooc_get_fastchg_ing);
#endif

	oplus_mt6375_guage_init(chip->ic_dev);
	chg_info("mt6375_guage_driver_probe success\n");
	return 0;

error:
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void mt6375_guage_driver_remove(struct platform_device *pdev)
#else
static int mt6375_guage_driver_remove(struct platform_device *pdev)
#endif
{
	struct mt6375_device *chip = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, chip);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}
/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/

static const struct of_device_id mt6375_gauge_match[] = {
	{ .compatible = "oplus,hal_mt6375_gauge" },
	{ .compatible = "oplus,hal_mt6379_gauge" },
	{},
};

static struct platform_driver mt6375_gauge_driver = {
	.driver = {
		.name = "oplus_mt6375_gauge",
		.of_match_table = mt6375_gauge_match,
	},
	.probe = mt6375_guage_driver_probe,
	.remove = mt6375_guage_driver_remove,
};

static __init int oplus_mt6375_gauge_driver_init(void)
{
	int rc;

	rc = platform_driver_register(&mt6375_gauge_driver);
	if (rc < 0)
		chg_err("failed to register mt6375 debug driver, rc = %d\n", rc);

	return rc;
}

static __exit void oplus_mt6375_gauge_driver_exit(void)
{
	platform_driver_unregister(&mt6375_gauge_driver);
}

oplus_chg_module_register(oplus_mt6375_gauge_driver);

MODULE_DESCRIPTION("Driver for mt6375 gauge");
MODULE_LICENSE("GPL v2");

