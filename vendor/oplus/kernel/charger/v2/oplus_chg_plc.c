// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2024 Oplus. All rights reserved.
 */


#define pr_fmt(fmt) "[PLC]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/gfp.h>
#include <linux/sched/clock.h>
#include <linux/proc_fs.h>

#include <oplus_chg.h>
#include <oplus_chg_module.h>
#include <oplus_chg_monitor.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_cpa.h>
#include <oplus_chg_ufcs.h>
#include <oplus_chg_plc.h>
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
#include <recovery/state_keep.h>
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
#define pde_data(inode) PDE_DATA(inode)
#endif

#define PROC_DATA_BUF_SIZE	256
#define PLC_IBAT_AVG_NUM	10
#define PLC_INFO_LEN		1023
struct plc_data {
	int ibat_index;
	int ibat_cnts;
	int ibus_index;
	int ibus_cnts;
	int init_soc;
	int init_sm_soc;
	int init_ui_soc;
	int sm_soc;
	int avg_ibat;
	int avg_curr;
	int avg_ibus;
	int ibat_low;
	int ibus_over;
	bool plc_check;
	bool init_status;
	int ibat_column[PLC_IBAT_AVG_NUM];
	int ibus_column[PLC_IBAT_AVG_NUM];
};

enum plc_track_type {
	PLC_TRACK_SOC_EXIT,
	PLC_TRACK_IBAT_EXIT,
	PLC_TRACK_IBUS_ENTER,
	PLC_TRACK_SOC_ADD,
};

struct plc_track_info {
	unsigned char msg[PLC_INFO_LEN + 1];
	int index;
};

struct ibat_delta {
	int ibat;
	int ibus;
};

static struct ibat_delta plc_ibus_table[] = {
	{ 0, 0 },
	{ 100, 50 },
	{ 200, 100 },
	{ 400, 200 },
	{ 600, 300 },
	{ 800, 400 },
	{ 10000, 500 },
};

struct oplus_chg_plc {
	struct device *dev;
	struct proc_dir_entry *plc_entry;
	struct oplus_mms *plc_topic;
	struct oplus_mms *gauge_topic;
	struct oplus_mms *comm_topic;
	struct oplus_mms *wired_topic;
	struct oplus_mms *cpa_topic;
	struct oplus_mms *keep_topic;

	struct mms_subscribe *comm_subs;
	struct mms_subscribe *wired_subs;
	struct mms_subscribe *plc_subs;
	struct mms_subscribe *gauge_subs;
	struct mms_subscribe *cpa_subs;
	struct mms_subscribe *keep_subs;

	struct votable *force_buck_votable;
	struct votable *output_suspend_votable;
	struct votable *wired_suspend_votable;

	struct delayed_work plc_disable_wait_work;
	struct delayed_work plc_enable_timeout_work;
	struct delayed_work charger_disable_work;
	struct work_struct protocol_change_work;
	struct work_struct chg_mode_change_work;
	struct work_struct wired_online_work;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	struct work_struct state_keep_ready_work;
#endif
	unsigned long protocol_change_jiffies;

	struct list_head protocol_list;
	spinlock_t protocol_list_lock;
	struct mutex status_control_lock;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	struct mutex restore_lock;
#endif
	struct oplus_plc_protocol *opp;
	struct oplus_plc_protocol *buck_opp;

	bool force_buck;
	bool wired_online;
	bool user_enabled;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	bool restore_plc;
	bool keep_wired_online;
#endif

	enum oplus_chg_protocol_type cpa_current_type;
	enum oplus_plc_chg_mode chg_mode;
	int ui_soc;
	int sm_soc;
	int plc_status;
	int plc_buck;
	int plc_soc;
	bool plc_support_timeout;
	int enable_cnts;

	struct plc_track_info track_info;
	int track_count;
};

static bool is_output_suspend_votable_available(struct oplus_chg_plc *chip)
{
	if (!chip->output_suspend_votable)
		chip->output_suspend_votable = find_votable("WIRED_CHARGING_DISABLE");
	return !!chip->output_suspend_votable;
}

static bool is_wired_suspend_votable_available(struct oplus_chg_plc *chip)
{
	if (!chip->wired_suspend_votable)
		chip->wired_suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	return !!chip->wired_suspend_votable;
}

static int oplus_chg_plc_enable_action(struct oplus_chg_plc *chip, bool enable);

struct oplus_plc_strategy;
struct oplus_plc_strategy_desc {
	enum oplus_plc_strategy_type type;
	struct oplus_plc_strategy *(*strategy_alloc)(
		struct oplus_plc_protocol *opp, struct device_node *node,
		struct proc_dir_entry *entry);
	int (*strategy_release)(struct oplus_plc_strategy *strategy);
	int (*strategy_init)(struct oplus_plc_strategy *strategy);
	int (*strategy_start)(struct oplus_plc_strategy *strategy);
	int (*strategy_exit)(struct oplus_plc_strategy *strategy);
};

struct oplus_plc_strategy {
	struct oplus_plc_protocol *opp;
	struct oplus_plc_strategy_desc *desc;
	struct proc_dir_entry *entry;
	struct device_node *node;

	bool initialized;
	bool started;
};

struct oplus_plc_strategy_group {
	const char *name;
	struct oplus_plc_strategy *strategy;
};

struct oplus_plc_protocol_record {
	unsigned long start_jiffies;
	enum oplus_chg_protocol_type cp_type;
	int start_soc;
	int start_sm_soc;
	int start_ui_soc;
	int start_temp;
	int exit_soc;
	int exit_sm_soc;
	int exit_ui_soc;
	int exit_temp;
};

struct oplus_plc_protocol {
	struct list_head list;
	void *priv_data;
	const struct oplus_plc_protocol_desc *desc;
	struct oplus_plc_strategy *strategy;
	struct oplus_chg_plc *plc;
	struct proc_dir_entry *entry;
	struct oplus_plc_protocol_record record;
	bool enable;
	int strategy_num;
	struct oplus_plc_strategy_group strategy_groups[];
};

static struct oplus_plc_protocol *oplus_plc_get_protocol(struct oplus_chg_plc *chip)
{
	struct oplus_plc_protocol *opp;
	bool find = false;

	if (chip->cpa_current_type <= CHG_PROTOCOL_INVALID ||
	    chip->cpa_current_type >= CHG_PROTOCOL_MAX)
		return NULL;

	spin_lock(&chip->protocol_list_lock);
	list_for_each_entry(opp, &chip->protocol_list, list) {
		if (BIT(chip->cpa_current_type) & opp->desc->protocol) {
			find = true;
			break;
		}
	}
	spin_unlock(&chip->protocol_list_lock);

	if (!find)
		return NULL;
	return opp;
}

static int oplus_plc_suspend_charger(struct oplus_chg_plc *chip, bool suspend)
{
	if (!is_wired_suspend_votable_available(chip))
		return -ENOTSUPP;
	return vote(chip->wired_suspend_votable, PLC_VOTER, suspend, suspend, false);
}

static void oplus_plc_charger_disable_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_plc *chip =
		container_of(dwork, struct oplus_chg_plc, charger_disable_work);

	if (chip->plc_status != PLC_STATUS_ENABLE)
		return;

	vote(chip->output_suspend_votable, PLC_VOTER, true, 1, false);
}

static bool oplus_plc_charger_is_disabled(struct oplus_chg_plc *chip)
{
	if (!is_output_suspend_votable_available(chip))
		return false;
	if (get_client_vote(chip->output_suspend_votable, PLC_VOTER) <= 0)
		return false;
	return true;
}

static int oplus_plc_disable_charger(struct oplus_chg_plc *chip, bool disable)
{
#define PLC_DISABLE_DELAY_MS	6000
	unsigned long target_jiffies = jiffies;

	if (!is_output_suspend_votable_available(chip))
		return -ENOTSUPP;

	if (!disable) {
		cancel_delayed_work_sync(&chip->charger_disable_work);
		return vote(chip->output_suspend_votable, PLC_VOTER, false, 0, false);
	}
	if (oplus_plc_charger_is_disabled(chip))
		return 0;
	if (work_busy(&chip->charger_disable_work.work))
		return 0;

	if (chip->opp && chip->opp->desc->current_active)
		target_jiffies = chip->protocol_change_jiffies + msecs_to_jiffies(PLC_DISABLE_DELAY_MS);
	if (time_before_eq(target_jiffies, jiffies))
		return vote(chip->output_suspend_votable, PLC_VOTER, true, 1, false);
	chg_info("charger disable after %dms\n", jiffies_to_msecs(target_jiffies - jiffies));
	schedule_delayed_work(&chip->charger_disable_work, target_jiffies - jiffies);

	return 0;
}

static int oplus_plc_strategy_init(struct oplus_plc_strategy *strategy);
static int oplus_plc_strategy_start(struct oplus_plc_strategy *strategy);
static int oplus_plc_strategy_exit(struct oplus_plc_strategy *strategy);

static void oplus_plc_protocol_record_start(struct oplus_plc_protocol *opp)
{
	struct oplus_chg_plc *chip = opp->plc;
	int rc;
	union mms_msg_data data = { 0 };

	opp->record.start_jiffies = jiffies;
	opp->record.cp_type = chip->cpa_current_type;
	opp->record.start_sm_soc = chip->sm_soc;
	opp->record.start_ui_soc = chip->ui_soc;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		opp->record.start_soc = 0;
	} else {
		opp->record.start_soc = data.intval;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		opp->record.start_temp = 0;
	} else {
		opp->record.start_temp = data.intval;
	}
}

static int oplus_plc_protocol_enable(struct oplus_chg_plc *chip, enum oplus_plc_chg_mode mode)
{
	struct oplus_plc_protocol *opp;
	int rc;

	opp = chip->opp;
	if (opp == NULL)
		return -EINVAL;

	if (mode != PLC_CHG_MODE_BUCK && chip->force_buck) {
		mode = PLC_CHG_MODE_BUCK;
		chg_info("mode chang to %s by force vote\n",
			 oplus_plc_chg_mode_str(mode));
	}

	chg_info("%s: enable %s mode\n", opp->desc->name, oplus_plc_chg_mode_str(mode));

	if (opp->strategy != NULL) {
		rc = oplus_plc_strategy_init(opp->strategy);
		if (rc < 0) {
			chg_err("%s: strategy init error, rc=%d\n",
				opp->desc->name, rc);
			return rc;
		}
	}
	oplus_plc_protocol_record_start(opp);
	rc = opp->desc->ops.enable(opp, mode);
	if (rc < 0) {
		chg_err("%s: enable %s mode plc error, rc=%d\n",
			opp->desc->name, oplus_plc_chg_mode_str(mode), rc);
		return rc;
	}
	if (opp->desc->ops.get_chg_mode != NULL) {
		rc = opp->desc->ops.get_chg_mode(opp);
		if (rc < 0) {
			chg_err("%s: get_chg_mode error, rc=%d\n",
				opp->desc->name, rc);
			goto get_chg_mode_error;
		}
		if (rc == PLC_CHG_MODE_CP) {
			rc = oplus_plc_disable_charger(chip, false);
			if (rc < 0) {
				chg_err("enable charger error, rc=%d\n", rc);
				goto get_chg_mode_error;
			}
			rc = oplus_plc_suspend_charger(chip, true);
			if (rc < 0) {
				chg_err("suspend charger error, rc=%d\n", rc);
				goto suspend_error;
			}
			goto strategy_start;
		}
	}

	/* buck mode */
	rc = oplus_plc_disable_charger(chip, true);
	if (rc < 0) {
		chg_err("disable charger error, rc=%d\n", rc);
		goto get_chg_mode_error;
	}
	rc = oplus_plc_suspend_charger(chip, false);
	if (rc < 0) {
		chg_err("unsuspend charger error, rc=%d\n", rc);
		goto suspend_error;
	}

strategy_start:
	if (opp->strategy != NULL) {
		rc = oplus_plc_strategy_start(opp->strategy);
		if (rc < 0) {
			chg_err("%s: strategy start error, rc=%d\n",
				opp->desc->name, rc);
			goto strategy_start_error;
		}
	} else {
		chg_info("%s: strategy is NULL\n", opp->desc->name);
	}
	opp->enable = true;

	return 0;

strategy_start_error:
	oplus_plc_suspend_charger(chip, false);
suspend_error:
	oplus_plc_disable_charger(chip, false);
get_chg_mode_error:
	opp->desc->ops.disable(opp);
	return rc;
}

static void oplus_plc_protocol_record_exit(struct oplus_plc_protocol *opp)
{
#define PROTOCOL_TRACK_TIME_DEAD_ZONE_S		5
	struct oplus_chg_plc *chip = opp->plc;
	int rc;
	union mms_msg_data data = { 0 };
	unsigned long time;

	time = (jiffies - opp->record.start_jiffies) / HZ;
	if (time < PROTOCOL_TRACK_TIME_DEAD_ZONE_S)
		return;

	opp->record.start_jiffies = jiffies;
	opp->record.exit_sm_soc = chip->sm_soc;
	opp->record.exit_ui_soc = chip->ui_soc;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		opp->record.exit_soc = 0;
	} else {
		opp->record.exit_soc = data.intval;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		opp->record.exit_temp = 0;
	} else {
		opp->record.exit_temp = data.intval;
	}

	if (chip->track_info.index < PLC_INFO_LEN) {
		chip->track_info.index += scnprintf(
			&(chip->track_info.msg[chip->track_info.index]),
			PLC_INFO_LEN - chip->track_info.index,
			"$$protocol_%d@@%s$$time_%d@@%lu"
			"$$start_soc_%d@@%d,$$exit_soc_%d@@%d"
			"$$start_sm_soc_%d@@%d,$$exit_sm_soc_%d@@%d"
			"$$start_ui_soc_%d@@%d,$$exit_ui_soc_%d@@%d"
			"$$start_temp_%d@@%d,$$exit_temp_%d@@%d",
			chip->track_count, get_protocol_name_str(opp->record.cp_type), chip->track_count, time,
			chip->track_count, opp->record.start_soc, chip->track_count, opp->record.exit_soc,
			chip->track_count, opp->record.start_sm_soc, chip->track_count, opp->record.exit_sm_soc,
			chip->track_count, opp->record.start_ui_soc, chip->track_count, opp->record.exit_ui_soc,
			chip->track_count, opp->record.start_temp, chip->track_count, opp->record.exit_temp);
	}
	chip->track_count++;
}

static void oplus_plc_protocol_disable(struct oplus_chg_plc *chip)
{
	struct oplus_plc_protocol *opp;
	int rc;

	opp = chip->opp;
	if (!opp)
		return;
	chg_info("%s: disable\n", opp->desc->name);
	rc = oplus_plc_strategy_exit(opp->strategy);
	if (rc < 0)
		chg_err("%s: strategy exit error, rc=%d\n",
			opp->desc->name, rc);
	opp->enable = false;
	opp->desc->ops.disable(opp);
	oplus_plc_protocol_record_exit(opp);
}

static int oplus_plc_protocol_reset_protocol(struct oplus_chg_plc *chip)
{
	struct oplus_plc_protocol *opp;
	int rc;

	opp = chip->opp;
	if (opp == NULL)
		return 0;
	if (opp->desc->ops.reset_protocol == NULL)
		return 0;

	rc = opp->desc->ops.reset_protocol(opp);
	if (rc < 0)
		chg_err("%s: reset protocol error, rc=%d\n",
			opp->desc->name, rc);
	return rc;
}

static int oplus_plc_protocol_set_ibus(struct oplus_chg_plc *chip, int curr_ma)
{
	struct oplus_plc_protocol *opp;
	int rc;

	opp = chip->opp;
	if (opp == NULL)
		return 0;
	if (opp->desc->ops.set_ibus == NULL)
		return 0;

	rc = opp->desc->ops.set_ibus(opp, curr_ma);
	if (rc < 0)
		chg_err("%s: set ibus to %dmA error, rc=%d\n",
			opp->desc->name, curr_ma, rc);
	return rc;
}

static int oplus_plc_protocol_get_ibus(struct oplus_chg_plc *chip)
{
	struct oplus_plc_protocol *opp;
	int rc;

	opp = chip->opp;
	if (opp == NULL)
		return -ENOTSUPP;
	if (opp->desc->ops.get_ibus == NULL)
		return -ENOTSUPP;

	rc = opp->desc->ops.get_ibus(opp);
	if (rc < 0)
		chg_err("%s: get ibus error, rc=%d\n",
			opp->desc->name, rc);
	return rc;
}

static int oplus_plc_protocol_get_chg_mode(struct oplus_chg_plc *chip)
{
	struct oplus_plc_protocol *opp;
	int rc;

	opp = chip->opp;
	if (opp == NULL)
		return PLC_CHG_MODE_BUCK;
	if (opp->desc->ops.get_chg_mode == NULL)
		return 0;

	rc = opp->desc->ops.get_chg_mode(opp);
	if (rc < 0)
		chg_err("%s: get chg mode error, rc=%d\n",
			opp->desc->name, rc);
	return rc;
}

static int oplus_plc_publish_enable_cnts(struct oplus_chg_plc *chip)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  PLC_ITEM_ENABLE_CNTS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(chip->plc_topic, msg);
	if (rc < 0) {
		chg_err("publish enable enable cnts msg error, rc=%d\n", rc);
		kfree(msg);
		return rc;
	}

	return 0;
}

static bool oplus_plc_protocol_support(struct oplus_chg_plc *chip)
{
	if ((chip->cpa_current_type != CHG_PROTOCOL_INVALID && chip->opp == NULL) ||
	    (chip->cpa_current_type == CHG_PROTOCOL_INVALID && chip->buck_opp == NULL))
		return false;
	return true;
}

static int plc_info_debug_track = 0;
module_param(plc_info_debug_track, int, 0644);
MODULE_PARM_DESC(plc_info_debug_track, "debug track");
#define TRACK_UPLOAD_COUNT_MAX 3
#define TRACK_LOCAL_T_NS_TO_S_THD 1000000000
#define TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD (24 * 3600)
static int oplus_plc_upload_plc_info(struct oplus_chg_plc *chip, char *info)
{
	struct oplus_mms *err_topic;
	struct mms_msg *msg;
	int rc;
	static int upload_count = 0;
	static int pre_upload_time = 0;
	int curr_time;

	curr_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;

	if (curr_time - pre_upload_time > TRACK_DEVICE_ABNORMAL_UPLOAD_PERIOD)
		upload_count = 0;

	if (upload_count >= TRACK_UPLOAD_COUNT_MAX)
		return -ENODEV;

	pre_upload_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;

	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic) {
		chg_err("error topic not found\n");
		return -ENODEV;
	}

	msg = oplus_mms_alloc_str_msg(
		MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, ERR_ITEM_PLC_INFO, info);
	if (msg == NULL) {
		chg_err("alloc plc error msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish plc error msg error, rc=%d\n", rc);
		kfree(msg);
	}

	upload_count++;

	return rc;
}

static int __oplus_chg_plc_enable(struct oplus_chg_plc *chip)
{
	int rc;

	if (!oplus_plc_protocol_support(chip) || !chip->wired_online) {
		chg_info("%s: not allow plc\n",
			 get_protocol_name_str(chip->cpa_current_type));
		return -ENOTSUPP;
	}

	chg_info("plc enable\n");
	chip->enable_cnts++;
	oplus_plc_publish_enable_cnts(chip);
	memset(&chip->track_info, 0, sizeof(chip->track_info));
	chip->track_count = 0;

	/* if the CPA type is not obtained, the buck_opp is used by default */
	if (chip->opp == NULL)
		chip->opp = chip->buck_opp;
	rc = oplus_plc_protocol_enable(chip, PLC_CHG_MODE_AUTO);
	if (rc < 0) {
		chg_err("%s: enable error\n", chip->opp->desc->name);
		return rc;
	}

	return 0;
}

static void __oplus_chg_plc_disable(struct oplus_chg_plc *chip)
{
	chg_info("plc disable\n");
	oplus_plc_protocol_disable(chip);
	oplus_plc_disable_charger(chip, false);
	oplus_plc_suspend_charger(chip, false);
	if (chip->track_count > 0) {
		oplus_plc_upload_plc_info(chip, chip->track_info.msg);
		memset(&chip->track_info, 0, sizeof(chip->track_info));
		chip->track_count = 0;
	}
}

static const char *oplus_plc_strategy_type_str(enum oplus_plc_strategy_type type)
{
	switch(type) {
	case PLC_STRATEGY_STEP:
		return "step";
	case PLC_STRATEGY_SIMPLE:
		return "simple";
	case PLC_STRATEGY_PID:
		return "pid";
	default:
		return "invalid";
	}
}

struct oplus_plc_strategy_step {
	struct oplus_plc_strategy strategy;
	struct plc_data data;
	struct plc_track_info info;

	struct delayed_work current_work;
	struct delayed_work track_work;
};

static void step_strategy_track_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_plc_strategy_step *step =
		container_of(dwork, struct oplus_plc_strategy_step, track_work);
	struct oplus_chg_plc *chip;
	int vbat_min_mv, batt_temp, ibat_ma, soc_now;
	int rc;
	union mms_msg_data data = { 0 };

	chip = step->strategy.opp->plc;
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		soc_now = 0;
	} else {
		soc_now = data.intval;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		batt_temp = 0;
	} else {
		batt_temp = data.intval;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data, false);
	if (rc < 0) {
		chg_err("can't get vbat_min, rc=%d\n", rc);
		vbat_min_mv = 0;
	} else {
		vbat_min_mv = data.intval;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data,
				     false);
	if (rc < 0) {
		chg_err("can't get ibat_ma, rc=%d\n", rc);
		ibat_ma = 0;
	} else {
		ibat_ma = data.intval;
	}

	if (step->info.index < PLC_INFO_LEN) {
		step->info.index += scnprintf(&(step->info.msg[step->info.index]),
			PLC_INFO_LEN - step->info.index, "$$enable_cnts@@%d$$exit_soc@@%d"
			"$$exit_sm_soc@@%d$$exit_ui_soc@@%d$$exit_temp@@%d$$exit_vbat@@%d$$exit_ibat@@%d",
			chip->enable_cnts, soc_now, chip->sm_soc, chip->ui_soc, batt_temp, vbat_min_mv, ibat_ma);
	}
	chip->enable_cnts = 0;

	oplus_plc_upload_plc_info(chip, step->info.msg);
	memset(&(step->info), 0, sizeof(step->info));
}

static void step_strategy_get_deleta_track_msg(struct oplus_plc_strategy_step *step, int type)
{
	struct oplus_chg_plc *chip;
	union mms_msg_data data = { 0 };
	int vbat_min_mv, batt_temp, ibat_ma, soc_now, curr_vote;
	int rc;

	chip = step->strategy.opp->plc;
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		soc_now = 0;
	} else {
		soc_now = data.intval;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		batt_temp = 0;
	} else {
		batt_temp = data.intval;
	}

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data, false);
	if (rc < 0) {
		chg_err("can't get vbat_min, rc=%d\n", rc);
		vbat_min_mv = 0;
	} else {
		vbat_min_mv = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, false);
	if (rc < 0) {
		chg_err("can't get ibat_ma, rc=%d\n", rc);
		ibat_ma = 0;
	} else {
		ibat_ma = data.intval;
	}

	curr_vote = oplus_plc_protocol_get_ibus(chip);
	if (curr_vote < 0)
		chg_err("can't get protocol ibus, rc=%d\n", curr_vote);

	chg_info("[%d, %d, %d][%d, %d, %d, %d, %d, %d, %d, %d]\n", type, step->data.init_sm_soc, step->data.init_ui_soc,
		chip->sm_soc, soc_now, vbat_min_mv, batt_temp, ibat_ma, step->data.avg_ibus, step->data.avg_ibat, curr_vote);
	if (step->info.index < PLC_INFO_LEN)
		step->info.index += scnprintf(&(step->info.msg[step->info.index]),
		PLC_INFO_LEN - step->info.index, "$$exit_type@@%d$$smooth_soc_%d@@%d$$soc_now_%d@@%d$$vbat_%d@@%d"
		"$$tbat_%d@@%d$$ibat_%d@@%d$$avg_ibus_%d@@%d$$avg_ibat_%d@@%d$$curr_vote_%d@@%d",
		type, chip->sm_soc, type, soc_now, type, vbat_min_mv, type, batt_temp, type, ibat_ma, type,
		step->data.avg_ibus, type, step->data.avg_ibat, type, curr_vote, type);
}

static void step_strategy_init_status(struct oplus_plc_strategy_step *step)
{
	struct oplus_chg_plc *chip;
	union mms_msg_data data = { 0 };
	int vbat_min_mv, batt_temp, ibat_ma;
	int rc;

	if (step->data.init_status)
		return;

	chip = step->strategy.opp->plc;
	step->data.init_status = true;
	step->data.init_ui_soc = chip->ui_soc;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_SOC, &data, false);
	step->data.init_soc = data.intval;
	step->data.init_sm_soc = chip->sm_soc;

	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_TEMP, &data, false);
	if (rc < 0) {
		chg_err("can't get batt_temp, rc=%d\n", rc);
		batt_temp = 0;
	} else {
		batt_temp = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MIN, &data, false);
	if (rc < 0) {
		chg_err("can't get vbat_min, rc=%d\n", rc);
		vbat_min_mv = 0;
	} else {
		vbat_min_mv = data.intval;
	}
	rc = oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, false);
	if (rc < 0) {
		chg_err("can't get ibat_ma, rc=%d\n", rc);
		ibat_ma = 0;
	} else {
		ibat_ma = data.intval;
	}

	oplus_plc_protocol_set_ibus(chip, PLC_IBUS_DEFAULT);
	chg_info("[%d, %d]\n", step->data.init_sm_soc, step->data.init_ui_soc);

	if (step->info.index < PLC_INFO_LEN)
		step->info.index += scnprintf(&(step->info.msg[step->info.index]),
			PLC_INFO_LEN - step->info.index, "$$plc_buck@@%d$$init_sm_soc@@%d"
			"$$init_ui_soc@@%d$$init_soc@@%d$$vbat_min@@%d$$tbat@@%d$$ibat_ma@@%d",
			chip->plc_buck, step->data.init_sm_soc, step->data.init_ui_soc,
			step->data.init_soc, batt_temp, vbat_min_mv, ibat_ma);
}

static void step_strategy_read_ibatt(struct oplus_plc_strategy_step *step)
{
	struct oplus_chg_plc *chip;
	union mms_msg_data data = { 0 };
	int ibus_pmic = 0;

	chip = step->strategy.opp->plc;
	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, true);
	if (step->data.ibus_index >= PLC_IBAT_AVG_NUM)
		step->data.ibus_index = step->data.ibus_index % PLC_IBAT_AVG_NUM;
	step->data.ibat_column[step->data.ibat_index] = data.intval;
	step->data.ibat_index = (step->data.ibat_index + 1) % PLC_IBAT_AVG_NUM;
	step->data.ibat_cnts++;
	if (!step->data.ibat_index)
		step->data.plc_check = true;
	else
		step->data.plc_check = false;

	ibus_pmic = oplus_wired_get_ibus();
	step->data.ibus_column[step->data.ibus_index] = ibus_pmic;
	step->data.ibus_index = (step->data.ibus_index + 1) % PLC_IBAT_AVG_NUM;
	step->data.ibus_cnts++;
}

static int step_strategy_get_avg_ibat(struct oplus_plc_strategy_step *step)
{
	int sum = 0, i;

	for (i = 0; i < PLC_IBAT_AVG_NUM; i++)
		sum += step->data.ibat_column[i];

	step->data.avg_ibat = sum / PLC_IBAT_AVG_NUM;
	return step->data.avg_ibat;
}

static int step_strategy_get_avg_ibus(struct oplus_plc_strategy_step *step)
{
	int sum = 0, i;

	for (i = 0; i < PLC_IBAT_AVG_NUM; i++)
		sum += step->data.ibus_column[i];

	step->data.avg_ibus = sum / PLC_IBAT_AVG_NUM;
	return sum / PLC_IBAT_AVG_NUM;
}

static int step_strategy_get_delta_ibat(struct oplus_plc_strategy_step *step)
{
	int avg_ibat = 0, delta_ibat = 0, batt_num = 2;
	int asize = 0, i, ibat1 = 0, ibat2 = 0, ibus1 = 0, ibus2 = 0;

#define PLC_DELTA_ISTEP 50

	avg_ibat = step_strategy_get_avg_ibat(step);
	batt_num = oplus_gauge_get_batt_num();
	asize = sizeof(plc_ibus_table) / sizeof(struct ibat_delta);

	if (abs(avg_ibat) <= plc_ibus_table[0].ibat) {
		delta_ibat = plc_ibus_table[0].ibus;
	} else if (abs(avg_ibat) >= plc_ibus_table[asize - 1].ibat) {
		delta_ibat = plc_ibus_table[asize - 1].ibus;
	} else {
		ibat1 = plc_ibus_table[0].ibat;
		ibus1 = plc_ibus_table[0].ibus;

		for (i = 1; i < asize; i++) {
			if (abs(avg_ibat) < plc_ibus_table[i].ibat) {
				ibat2 = plc_ibus_table[i].ibat;
				ibus2 = plc_ibus_table[i].ibus;
				break;
			}
			ibat1 = plc_ibus_table[i].ibat;
			ibus1 = plc_ibus_table[i].ibus;
		}
		delta_ibat = ((abs(avg_ibat) - ibat1) * ibus2 + (ibat2 - abs(avg_ibat)) * ibus1) /
			(ibat2 - ibat1);
	}
	delta_ibat *= batt_num;
	delta_ibat = (delta_ibat / PLC_DELTA_ISTEP) * PLC_DELTA_ISTEP;

	return delta_ibat;
}

static int step_strategy_check_plc_ibus(struct oplus_plc_strategy_step *step)
{
	struct oplus_chg_plc *chip;
	int soc_now = 0, delta_soc = 0;
	int delta_ibat = 0, curr_vote = 0;
	int ibus_plc = PLC_IBUS_DEFAULT;
	int chg_mode;

	chip = step->strategy.opp->plc;
	soc_now = chip->sm_soc;
	delta_soc = soc_now - step->data.init_ui_soc;
	curr_vote = oplus_plc_protocol_get_ibus(chip);
	if (curr_vote < 0) {
		chg_err("can't get protocol ibus, rc=%d\n", curr_vote);
		return PLC_IBUS_DEFAULT;
	}
	delta_ibat = step_strategy_get_delta_ibat(step);
	chg_mode = oplus_plc_protocol_get_chg_mode(chip);

	if (step->data.init_ui_soc >= chip->plc_soc) {
		if (chg_mode == PLC_CHG_MODE_CP) {
			step_strategy_get_deleta_track_msg(step, PLC_TRACK_SOC_EXIT);
			oplus_plc_protocol_enable(chip, PLC_CHG_MODE_BUCK);
		}
		ibus_plc = PLC_IBUS_DEFAULT;
	} else if ((step->data.init_ui_soc > step->data.init_sm_soc) && (step->data.init_ui_soc > soc_now)) {
		ibus_plc = PLC_IBUS_MAX;
	} else if ((step->data.init_ui_soc < step->data.init_sm_soc) && (step->data.init_ui_soc < soc_now)) {
		ibus_plc = PLC_IBUS_MIN;
	} else {
		if (delta_soc < 0) {
			ibus_plc = PLC_IBUS_MAX;
		} else if (delta_soc == 0) {
			if (step->data.avg_ibat > 0)
				ibus_plc = curr_vote + delta_ibat;
			else
				ibus_plc = curr_vote - delta_ibat;
		} else {
			ibus_plc = PLC_IBUS_MIN;
		}
	}

	if (ibus_plc < PLC_IBUS_MIN)
		ibus_plc = PLC_IBUS_MIN;
	if (ibus_plc > PLC_IBUS_MAX)
		ibus_plc = PLC_IBUS_MAX;

	return ibus_plc;
}

static void step_strategy_ibat_check(struct oplus_plc_strategy_step *step)
{
	struct oplus_chg_plc *chip;
	int ibus_pmic = 0, ibus_plc = 0;
	int chg_mode;

#define PLC_IBAT_LOW_CNTS 4
#define PLC_IBUS_HIGH_CNTS 4
#define PLC_IBUS_HIGH_MAX 600
#define PLC_SUSPEND_DELAY 1000

	chip = step->strategy.opp->plc;
	ibus_pmic = step_strategy_get_avg_ibus(step);
	ibus_plc = step_strategy_check_plc_ibus(step);
	chg_mode = oplus_plc_protocol_get_chg_mode(chip);

	if (chg_mode == PLC_CHG_MODE_CP &&
	    ibus_plc <= PLC_IBUS_MIN &&
	    step->data.avg_ibat < 0)
		step->data.ibat_low++;
	else
		step->data.ibat_low = 0;

	if (chg_mode == PLC_CHG_MODE_CP &&
	    (step->data.ibat_low >= PLC_IBAT_LOW_CNTS)) {
		oplus_plc_protocol_enable(chip, PLC_CHG_MODE_BUCK);
		step_strategy_get_deleta_track_msg(step, PLC_TRACK_IBAT_EXIT);
		return;
	}

	if (chg_mode == PLC_CHG_MODE_BUCK && ibus_pmic > PLC_IBUS_HIGH_MAX && !chip->force_buck)
		step->data.ibus_over++;
	else
		step->data.ibus_over = 0;

	if (step->data.ibus_over >= PLC_IBUS_HIGH_CNTS) {
		oplus_plc_suspend_charger(chip, true);
		msleep(PLC_SUSPEND_DELAY);
		oplus_plc_suspend_charger(chip, false);
		/* TODO */
		oplus_plc_protocol_enable(chip, PLC_CHG_MODE_CP);
		step_strategy_get_deleta_track_msg(step, PLC_TRACK_IBUS_ENTER);
	}

	chg_mode = oplus_plc_protocol_get_chg_mode(chip);
	if (chg_mode == PLC_CHG_MODE_CP)
		oplus_plc_protocol_set_ibus(chip, ibus_plc);
}

#define STEP_STRATEGY_MONITOR_CURRENT_DELAY 1000
static void step_strategy_monitor_current_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_plc_strategy_step *step =
		container_of(dwork, struct oplus_plc_strategy_step, current_work);
	struct oplus_chg_plc *chip;

	chip = step->strategy.opp->plc;
	step_strategy_init_status(step);
	step_strategy_read_ibatt(step);
	if (step->data.plc_check)
		step_strategy_ibat_check(step);

	if (step->data.sm_soc != chip->sm_soc) {
		step->data.sm_soc = chip->sm_soc;
		step_strategy_get_deleta_track_msg(step, PLC_TRACK_SOC_ADD);
	}

	schedule_delayed_work(&step->current_work,
		msecs_to_jiffies(STEP_STRATEGY_MONITOR_CURRENT_DELAY));
}

static struct oplus_plc_strategy *step_strategy_alloc(
	struct oplus_plc_protocol *opp, struct device_node *node, struct proc_dir_entry *entry)
{
	struct oplus_plc_strategy_step *strategy;

	strategy = kzalloc(sizeof(struct oplus_plc_strategy_step), GFP_KERNEL);
	if (strategy == NULL) {
		chg_err("alloc strategy buf error\n");
		return NULL;
	}

	INIT_DELAYED_WORK(&strategy->current_work, step_strategy_monitor_current_work);
	INIT_DELAYED_WORK(&strategy->track_work, step_strategy_track_work);

	return &strategy->strategy;
}

static int step_strategy_release(struct oplus_plc_strategy *strategy)
{
	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}

	kfree(strategy);
	return 0;
}

static int step_strategy_init(struct oplus_plc_strategy *strategy)
{
	struct oplus_plc_strategy_step *step =
		(struct oplus_plc_strategy_step *)strategy;
	struct oplus_chg_plc *chip;

	chip = step->strategy.opp->plc;
	step->data.plc_check = false;
	step->data.init_status = false;
	step->data.ibat_index = 0;
	step->data.ibat_cnts = 0;
	step->data.ibus_index = 0;
	step->data.ibus_cnts = 0;
	step->data.ibat_low = 0;
	step->data.ibus_over = 0;
	step->data.sm_soc = chip->sm_soc;

	memset(step->data.ibat_column, 0, PLC_IBAT_AVG_NUM);
	memset(step->data.ibus_column, 0, PLC_IBAT_AVG_NUM);

	return 0;
}

static int step_strategy_start(struct oplus_plc_strategy *strategy)
{
	struct oplus_plc_strategy_step *step =
		(struct oplus_plc_strategy_step *)strategy;
	struct oplus_chg_plc *chip;

	chip = step->strategy.opp->plc;
	if (chip->ui_soc >= chip->plc_soc)
		vote(chip->force_buck_votable, PLC_SOC_VOTER, true, 1, false);

	schedule_delayed_work(&step->current_work,
		msecs_to_jiffies(STEP_STRATEGY_MONITOR_CURRENT_DELAY));
	return 0;
}

static int step_strategy_exit(struct oplus_plc_strategy *strategy)
{
	struct oplus_plc_strategy_step *step =
		(struct oplus_plc_strategy_step *)strategy;
	struct oplus_chg_plc *chip;

	chip = step->strategy.opp->plc;
	cancel_delayed_work_sync(&step->current_work);
	if (chip->enable_cnts > 0 || plc_info_debug_track)
		schedule_delayed_work(&step->track_work, 0);
	vote(chip->force_buck_votable, PLC_SOC_VOTER, false, 0, false);

	return 0;
}

enum simple_strategy_curr_type {
	STRATEGY_CURR_DEFAULT = 0,
	STRATEGY_CURR_HIGH,
	STRATEGY_CURR_MAX
};

struct oplus_plc_strategy_simple {
	struct oplus_plc_strategy strategy;
	int last_sm_soc;
	enum simple_strategy_curr_type curr_type;
	enum oplus_plc_chg_mode chg_mode;
	u32 curr_ma[STRATEGY_CURR_MAX];
	bool no_curr_data;
	struct delayed_work monitor_work;
};

static ssize_t simple_strategy_ibus_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_plc_strategy_simple *simple = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE] = { 0 };
	int len = 0;

	len += scnprintf(buf, sizeof(buf) - 1, "%d,%d\n",
			simple->curr_ma[STRATEGY_CURR_DEFAULT],
			simple->curr_ma[STRATEGY_CURR_HIGH]);
	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

static ssize_t simple_strategy_ibus_proc_write(struct file *file, const char __user *buf, size_t len, loff_t *data)
{
	struct oplus_plc_strategy_simple *simple = pde_data(file_inode(file));
	char tmp_buf[PROC_DATA_BUF_SIZE] = { 0 };
	int rc;

	if (len > PROC_DATA_BUF_SIZE)
		return -EFAULT;
	if (copy_from_user(tmp_buf, buf, len))
		return -EFAULT;

	rc = sscanf(tmp_buf, "%d,%d", &simple->curr_ma[STRATEGY_CURR_DEFAULT],
		&simple->curr_ma[STRATEGY_CURR_HIGH]);
	if (rc != 2) {
		chg_err("data buf error\n");
		return -EFAULT;
	}
	chg_info("%s[%s]: curr_ma=%d,%d\n", simple->strategy.opp->desc->name,
		 simple->strategy.node->name,
		 simple->curr_ma[STRATEGY_CURR_DEFAULT],
		 simple->curr_ma[STRATEGY_CURR_HIGH]);

	return len;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations simple_strategy_ibus_ops =
{
	.read = simple_strategy_ibus_proc_read,
	.write = simple_strategy_ibus_proc_write,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops simple_strategy_ibus_ops =
{
	.proc_read  = simple_strategy_ibus_proc_read,
	.proc_write = simple_strategy_ibus_proc_write,
	.proc_lseek = noop_llseek,
};
#endif

#define SIMPLE_STRATEGY_MONITOR_DELAY_MS 1000
static void simple_strategy_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_plc_strategy_simple *simple =
		container_of(dwork, struct oplus_plc_strategy_simple, monitor_work);
	struct oplus_chg_plc *chip;
	int chg_mode;

	chip = simple->strategy.opp->plc;
	chg_mode = oplus_plc_protocol_get_chg_mode(chip);
	if (chg_mode < 0) {
		chg_err("cannot get chg mode, rc=%d\n", chg_mode);
		goto out;
	}

	if (chip->sm_soc < simple->last_sm_soc) {
		if (simple->chg_mode == chg_mode &&
		    simple->curr_type == STRATEGY_CURR_HIGH)
			goto out;
		simple->curr_type = STRATEGY_CURR_HIGH;
		if (!simple->no_curr_data)
			oplus_plc_protocol_set_ibus(chip, simple->curr_ma[STRATEGY_CURR_HIGH]);
		if (chg_mode != PLC_CHG_MODE_CP &&
		    oplus_plc_charger_is_disabled(chip))
			oplus_plc_disable_charger(chip, false);
	} else if (chip->sm_soc > simple->last_sm_soc) {
		if (simple->chg_mode == chg_mode &&
		    simple->curr_type == STRATEGY_CURR_DEFAULT)
			goto out;
		simple->curr_type = STRATEGY_CURR_DEFAULT;
		if (!simple->no_curr_data)
			oplus_plc_protocol_set_ibus(chip, simple->curr_ma[STRATEGY_CURR_DEFAULT]);
		if (!oplus_plc_charger_is_disabled(chip))
			oplus_plc_disable_charger(chip, true);
	} else {
		if (simple->chg_mode == chg_mode)
			goto out;
		if (!simple->no_curr_data)
			oplus_plc_protocol_set_ibus(chip, simple->curr_ma[simple->curr_type]);
		if (chg_mode != PLC_CHG_MODE_CP &&
		    simple->curr_type == STRATEGY_CURR_HIGH &&
		    oplus_plc_charger_is_disabled(chip))
			oplus_plc_disable_charger(chip, false);
	}

out:
	simple->chg_mode = chg_mode;
	simple->last_sm_soc = chip->sm_soc;
	schedule_delayed_work(&simple->monitor_work,
		msecs_to_jiffies(SIMPLE_STRATEGY_MONITOR_DELAY_MS));
}

static struct oplus_plc_strategy *simple_strategy_alloc(
	struct oplus_plc_protocol *opp, struct device_node *node, struct proc_dir_entry *entry)
{
	struct oplus_plc_strategy_simple *simple;
	struct proc_dir_entry *pr_entry_tmp;
	int rc;

	simple = kzalloc(sizeof(struct oplus_plc_strategy_simple), GFP_KERNEL);
	if (simple == NULL) {
		chg_err("alloc %s strategy buf error\n",
			oplus_plc_strategy_type_str(PLC_STRATEGY_SIMPLE));
		return NULL;
	}
	INIT_DELAYED_WORK(&simple->monitor_work, simple_strategy_monitor_work);

	if (node == NULL) {
		simple->no_curr_data = true;
	} else {
		simple->no_curr_data = false;
		rc = of_property_count_elems_of_size(
			node, "strategy_curr_ma", sizeof(u32));
		if (rc < 0) {
			chg_err("cannot parse \"strategy_curr_ma\", rc=%d", rc);
			goto err;
		}
		if (rc != STRATEGY_CURR_MAX) {
			chg_err("\"strategy_curr_ma\" can only contain %d data, rc=%d",
				STRATEGY_CURR_MAX, rc);
			goto err;
		}
		rc = of_property_read_u32_array(node, "strategy_curr_ma",
						simple->curr_ma, STRATEGY_CURR_MAX);
		if (rc < 0) {
			chg_err("cannot parse \"strategy_curr_ma\", rc=%d", rc);
			goto err;
		}

		if (entry == NULL)
			goto done;

		pr_entry_tmp = proc_create_data("ibus", 0644, entry,
			&simple_strategy_ibus_ops, simple);
		if (pr_entry_tmp == NULL)
			chg_err("Couldn't create ibus proc entry\n");
	}

done:
	return &simple->strategy;

err:
	kfree(simple);
	return NULL;
}

static int simple_strategy_release(struct oplus_plc_strategy *strategy)
{
	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}

	kfree(strategy);
	return 0;
}

static int simple_strategy_init(struct oplus_plc_strategy *strategy)
{
	struct oplus_plc_strategy_simple *simple =
		(struct oplus_plc_strategy_simple *)strategy;
	struct oplus_chg_plc *chip;

	chip = simple->strategy.opp->plc;
	simple->last_sm_soc = chip->sm_soc;
	simple->curr_type = STRATEGY_CURR_DEFAULT;
	simple->chg_mode = oplus_plc_protocol_get_chg_mode(chip);

	return 0;
}

static int simple_strategy_start(struct oplus_plc_strategy *strategy)
{
	struct oplus_plc_strategy_simple *simple =
		(struct oplus_plc_strategy_simple *)strategy;
	struct oplus_chg_plc *chip;

	chip = simple->strategy.opp->plc;
	if (!simple->no_curr_data)
		oplus_plc_protocol_set_ibus(chip, simple->curr_ma[STRATEGY_CURR_DEFAULT]);
	simple->curr_type = STRATEGY_CURR_DEFAULT;
	schedule_delayed_work(&simple->monitor_work,
		msecs_to_jiffies(SIMPLE_STRATEGY_MONITOR_DELAY_MS));

	return 0;
}

static int simple_strategy_exit(struct oplus_plc_strategy *strategy)
{
	struct oplus_plc_strategy_simple *simple =
		(struct oplus_plc_strategy_simple *)strategy;

	cancel_delayed_work_sync(&simple->monitor_work);

	return 0;
}

static struct oplus_plc_strategy_desc g_strategy_desc[] = {
	{
		.type = PLC_STRATEGY_STEP,
		.strategy_alloc = step_strategy_alloc,
		.strategy_release = step_strategy_release,
		.strategy_init = step_strategy_init,
		.strategy_start = step_strategy_start,
		.strategy_exit = step_strategy_exit,
	}, {
		.type = PLC_STRATEGY_SIMPLE,
		.strategy_alloc = simple_strategy_alloc,
		.strategy_release = simple_strategy_release,
		.strategy_init = simple_strategy_init,
		.strategy_start = simple_strategy_start,
		.strategy_exit = simple_strategy_exit,
	}
};

static ssize_t strategy_type_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_plc_strategy *strategy = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE] = { 0 };
	int len = 0;

	len += scnprintf(buf, sizeof(buf) - 1, "%s\n",
			oplus_plc_strategy_type_str(strategy->desc->type));
	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations strategy_type_ops =
{
	.read = strategy_type_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops strategy_type_ops =
{
	.proc_read  = strategy_type_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static ssize_t strategy_name_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_plc_strategy *strategy = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE] = { 0 };
	int i;
	const char *name = NULL;
	int len = 0;

	for (i = 0; i < strategy->opp->strategy_num; i++) {
		if (strategy->opp->strategy_groups[i].strategy == strategy) {
			name = strategy->opp->strategy_groups[i].name;
			break;
		}
	}
	len += scnprintf(buf, sizeof(buf) - 1, "%s\n", name);
	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations strategy_name_ops =
{
	.read = strategy_name_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops strategy_name_ops =
{
	.proc_read  = strategy_name_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static struct oplus_plc_strategy *oplus_plc_strategy_alloc(
	struct oplus_plc_protocol *opp,
	struct device_node *strategy_node)
{
	struct device_node *data_node;
	enum oplus_plc_strategy_type type;
	struct oplus_plc_strategy_desc *desc = NULL;
	struct oplus_plc_strategy *strategy;
	struct proc_dir_entry *entry = NULL;
	int i;
	int rc;

	rc = of_property_read_u32(strategy_node, "oplus,strategy_type", &type);
	if (rc < 0) {
		chg_err("can't get plc strategy type, rc=%d\n", rc);
		return NULL;
	}

	for (i = 0; i < ARRAY_SIZE(g_strategy_desc); i++) {
		desc = &g_strategy_desc[i];
		if (desc->type == type)
			break;
	}
	if (desc->type != type) {
		chg_err("strategy[%d] not found\n", type);
		return NULL;
	}

	if (desc->strategy_alloc == NULL) {
		chg_err("%s: strategy_alloc func is NULL\n",
			oplus_plc_strategy_type_str(desc->type));
		return NULL;
	}

	if (opp->entry != NULL) {
		entry = proc_mkdir(strategy_node->name, (opp->entry));
		if (entry == NULL)
			chg_err("Couldn't create charger/plc/%s/%s proc entry\n",
				opp->desc->name, strategy_node->name);
	}

	data_node = of_find_node_by_name(strategy_node, "oplus,strategy_data");
	strategy = desc->strategy_alloc(opp, data_node, entry);
	if (strategy == NULL) {
		chg_err("%s: strategy alloc error\n",
			oplus_plc_strategy_type_str(desc->type));
		if (entry != NULL)
			proc_remove(entry);
		return NULL;
	}
	strategy->node = strategy_node;
	strategy->entry = entry;
	strategy->opp = opp;
	strategy->desc = desc;

	if (strategy->entry == NULL)
		goto done;

	entry = proc_create_data("type", 0444, strategy->entry,
		&strategy_type_ops, strategy);
	if (entry == NULL)
		chg_err("%s: Couldn't create type proc entry\n", strategy_node->name);
	entry = proc_create_data("name", 0444, strategy->entry,
		&strategy_name_ops, strategy);
	if (entry == NULL)
		chg_err("%s: Couldn't create name proc entry\n", strategy_node->name);

done:
	return strategy;
}

static int oplus_plc_strategy_release(struct oplus_plc_strategy *strategy)
{
	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (strategy->entry != NULL)
		proc_remove(strategy->entry);
	if (strategy->desc->strategy_release == NULL)
		return 0;

	return strategy->desc->strategy_release(strategy);
}

static int oplus_plc_strategy_init(struct oplus_plc_strategy *strategy)
{
	int rc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (strategy->desc->strategy_init == NULL) {
		chg_err("%s: strategy_init func is NULL\n",
			oplus_plc_strategy_type_str(strategy->desc->type));
		return -EFAULT;
	}
	if (strategy->started) {
		chg_debug("strategy already running and cannot be initialized\n");
		return 0;
	}

	rc = strategy->desc->strategy_init(strategy);
	if (rc < 0) {
		chg_err("%s: strategy init error, rc=%d\n",
			oplus_plc_strategy_type_str(strategy->desc->type), rc);
		return rc;
	}
	strategy->initialized = true;

	return 0;
}

static int oplus_plc_strategy_start(struct oplus_plc_strategy *strategy)
{
	int rc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (strategy->desc->strategy_start == NULL) {
		chg_err("%s: strategy_start func is NULL\n",
			oplus_plc_strategy_type_str(strategy->desc->type));
		return -EFAULT;
	}
	if (!strategy->initialized) {
		chg_err("%s: strategy not initialized\n",
			oplus_plc_strategy_type_str(strategy->desc->type));
		return -EFAULT;
	}
	if (strategy->started) {
		chg_debug("strategy already running\n");
		return 0;
	}

	rc = strategy->desc->strategy_start(strategy);
	if (rc < 0) {
		chg_err("%s: strategy start error, rc=%d\n",
			oplus_plc_strategy_type_str(strategy->desc->type), rc);
		return rc;
	}
	strategy->started = true;
	chg_info("%s: strategy start\n",
		 oplus_plc_strategy_type_str(strategy->desc->type));

	return 0;
}

static int oplus_plc_strategy_exit(struct oplus_plc_strategy *strategy)
{
	int rc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (strategy->desc->strategy_exit == NULL) {
		chg_err("%s: strategy_exit func is NULL\n",
			oplus_plc_strategy_type_str(strategy->desc->type));
		return -EFAULT;
	}
	if (!strategy->initialized) {
		chg_info("%s: strategy not initialized\n",
			oplus_plc_strategy_type_str(strategy->desc->type));
		return 0;
	}

	rc = strategy->desc->strategy_exit(strategy);
	if (rc < 0)
		chg_err("%s: strategy exit error, rc=%d\n",
			oplus_plc_strategy_type_str(strategy->desc->type), rc);
	strategy->initialized = false;
	strategy->started = false;
	chg_info("%s: strategy exit\n",
		 oplus_plc_strategy_type_str(strategy->desc->type));

	return rc;
}

static void oplus_plc_subscribe_gauge_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_plc *chip = prv_data;

	chip->gauge_topic = topic;
}

static void oplus_plc_set_status(struct oplus_chg_plc *chip, enum plc_enable_status status)
{
	struct mms_msg *msg;
	int rc;

	if (chip->plc_status == status)
		return;
	chip->plc_status = status;
	chg_info("plc_status=%s\n", plc_enable_status_str(status));

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  PLC_ITEM_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(chip->plc_topic, msg);
	if (rc < 0) {
		chg_err("publish plc status msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void oplus_plc_disable_wait_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_plc *chip =
		container_of(dwork, struct oplus_chg_plc, plc_disable_wait_work);
	int rc;

	chg_info("start disable\n");
	mutex_lock(&chip->status_control_lock);
	rc = oplus_plc_protocol_reset_protocol(chip);
	if (rc < 0)
		chg_err("plc reset protocol error, rc=%d\n", rc);
	__oplus_chg_plc_disable(chip);
	if (!oplus_plc_protocol_support(chip) || !chip->wired_online)
		oplus_plc_set_status(chip, PLC_STATUS_NOT_ALLOW);
	else
		oplus_plc_set_status(chip, PLC_STATUS_DISABLE);
	mutex_unlock(&chip->status_control_lock);
}

static void oplus_plc_enable_timeout_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_plc *chip = container_of(dwork, struct oplus_chg_plc, plc_enable_timeout_work);
	int rc;

	chg_info("timeout\n");
	mutex_lock(&chip->status_control_lock);
	if (chip->plc_status != PLC_STATUS_ENABLE)
		goto done;
	rc = oplus_plc_protocol_reset_protocol(chip);
	if (rc < 0)
		chg_err("plc reset protocol error, rc=%d\n", rc);
	__oplus_chg_plc_disable(chip);
	if (!oplus_plc_protocol_support(chip) || !chip->wired_online)
		oplus_plc_set_status(chip, PLC_STATUS_NOT_ALLOW);
	else
		oplus_plc_set_status(chip, PLC_STATUS_TIMEOUT);
done:
	mutex_unlock(&chip->status_control_lock);
}

static void oplus_plc_chg_mode_change_work(struct work_struct *work)
{
	struct oplus_chg_plc *chip =
		container_of(work, struct oplus_chg_plc, chg_mode_change_work);
	union mms_msg_data data = { 0 };
	enum oplus_plc_chg_mode chg_mode;
	int rc;

	rc = oplus_mms_get_item_data(chip->plc_topic, PLC_ITEM_CHG_MODE,
		&data, false);
	if (rc < 0) {
		chg_err("cannot get PLC_ITEM_CHG_MODE data, rc=%d\n", rc);
		rc = oplus_plc_protocol_get_chg_mode(chip);
		if (rc < 0) {
			chg_err("cannot get chg mode, rc=%d\n", rc);
			return;
		}
		chg_mode = rc;
	} else {
		chg_mode = data.intval;
	}
	chg_info("chg_mode change to %s\n", oplus_plc_chg_mode_str(chg_mode));

	if (chg_mode == PLC_CHG_MODE_CP) {
		rc = oplus_plc_disable_charger(chip, false);
		if (rc < 0)
			chg_err("enable charger error, rc=%d\n", rc);
		rc = oplus_plc_suspend_charger(chip, true);
		if (rc < 0)
			chg_err("suspend charger error, rc=%d\n", rc);
	} else {
		rc = oplus_plc_disable_charger(chip, true);
		if (rc < 0)
			chg_err("disable charger error, rc=%d\n", rc);
		rc = oplus_plc_suspend_charger(chip, false);
		if (rc < 0)
			chg_err("unsuspend charger error, rc=%d\n", rc);
	}
}

static void oplus_plc_plc_subs_callback(struct mms_subscribe *subs,
					enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_plc *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PLC_ITEM_CHG_MODE:
			if (chip->plc_status == PLC_STATUS_ENABLE)
				schedule_work(&chip->chg_mode_change_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static int oplus_plc_subscribe_plc_topic(struct oplus_chg_plc *chip)
{
	union mms_msg_data data = { 0 };
	int rc;

	chip->plc_subs =
		oplus_mms_subscribe(chip->plc_topic, chip,
				    oplus_plc_plc_subs_callback,
				    "plc");
	if (IS_ERR_OR_NULL(chip->plc_subs)) {
		chg_err("subscribe plc topic error, rc=%ld\n",
			PTR_ERR(chip->plc_subs));
		return PTR_ERR(chip->plc_subs);
	}
	rc = oplus_mms_get_item_data(chip->plc_topic, PLC_ITEM_CHG_MODE,
		&data, true);
	if (rc < 0) {
		chg_err("get chg mode error, rc=%d\n", rc);
		chip->chg_mode = PLC_CHG_MODE_BUCK;
	} else {
		chip->chg_mode = data.intval;
	}

	return 0;
}

static void oplus_plc_comm_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_plc *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_UI_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->ui_soc = data.intval;
			break;
		case COMM_ITEM_SMOOTH_SOC:
			oplus_mms_get_item_data(chip->comm_topic, id, &data,
						false);
			chip->sm_soc = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}


static void oplus_plc_subscribe_comm_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_chg_plc *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->comm_topic = topic;
	chip->comm_subs =
		oplus_mms_subscribe(topic, chip,
				    oplus_plc_comm_subs_callback, "plc");
	if (IS_ERR_OR_NULL(chip->comm_subs)) {
		chg_err("subscribe comm topic error, rc=%ld\n",
			PTR_ERR(chip->comm_subs));
		return;
	}

	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_UI_SOC, &data, false);
	chip->ui_soc = data.intval;
	oplus_mms_get_item_data(chip->comm_topic, COMM_ITEM_SMOOTH_SOC, &data, false);
	chip->sm_soc = data.intval;
}

static void oplus_plc_wired_online_work(struct work_struct *work)
{
	struct oplus_chg_plc *chip =
		container_of(work, struct oplus_chg_plc, wired_online_work);
	bool wired_online = chip->wired_online;
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	int rc;

	mutex_lock(&chip->restore_lock);
	if (wired_online && chip->restore_plc) {
		chip->restore_plc = false;
		rc = oplus_chg_plc_enable_action(chip, true);
		chg_info("try restore plc, rc=%d\n", rc);
		mutex_unlock(&chip->restore_lock);
		return;
	}
	mutex_unlock(&chip->restore_lock);
#endif

	if (wired_online && chip->plc_status != PLC_STATUS_ENABLE) {
		mutex_lock(&chip->status_control_lock);
		vote(chip->output_suspend_votable, PLC_VOTER, false, 0, false);
		vote(chip->wired_suspend_votable, PLC_VOTER, false, 0, false);
		if (!oplus_plc_protocol_support(chip))
			oplus_plc_set_status(chip, PLC_STATUS_NOT_ALLOW);
		else
			oplus_plc_set_status(chip, PLC_STATUS_DISABLE);
		mutex_unlock(&chip->status_control_lock);
	} else if (!wired_online) {
		if (chip->plc_status == PLC_STATUS_ENABLE) {
			cancel_delayed_work(&chip->plc_disable_wait_work);
			schedule_delayed_work(&chip->plc_disable_wait_work, 0);
		} else {
			mutex_lock(&chip->status_control_lock);
			oplus_plc_set_status(chip, PLC_STATUS_NOT_ALLOW);
			vote(chip->output_suspend_votable, PLC_VOTER, false, 0, false);
			vote(chip->wired_suspend_votable, PLC_VOTER, false, 0, false);
			mutex_unlock(&chip->status_control_lock);
		}
	}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	if (wired_online != READ_ONCE(wired_online)) {
		chg_info("wired_online changed: %d -> %d\n", wired_online, READ_ONCE(wired_online));
		schedule_work(&chip->wired_online_work);
	}
#endif
}

static void oplus_plc_wired_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_plc *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->wired_online = !!data.intval;
			schedule_work(&chip->wired_online_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_plc_subscribe_wired_topic(struct oplus_mms *topic,
					     void *prv_data)
{
	struct oplus_chg_plc *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    oplus_plc_wired_subs_callback, "plc");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_ONLINE, &data,
				true);
	chip->wired_online = !!data.intval;
	schedule_work(&chip->wired_online_work);
}

static void oplus_plc_protocol_change_work(struct work_struct *work)
{
	struct oplus_chg_plc *chip =
		container_of(work, struct oplus_chg_plc, protocol_change_work);
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW,
		&data, false);
	if (rc < 0) {
		chg_err("cannot get CPA_ITEM_ALLOW data, rc=%d\n", rc);
		return;
	}
	chg_info("cpa_current_type=%s\n",
		get_protocol_name_str(data.intval));

	mutex_lock(&chip->status_control_lock);
	if (chip->plc_status == PLC_STATUS_ENABLE) {
		oplus_plc_protocol_disable(chip);
		chip->cpa_current_type = data.intval;
		chip->opp = oplus_plc_get_protocol(chip);
		if (chip->cpa_current_type != CHG_PROTOCOL_INVALID &&
		    chip->opp == NULL) {
			chg_info("%s: not allow plc\n",
				 get_protocol_name_str(chip->cpa_current_type));
			__oplus_chg_plc_disable(chip);
			oplus_plc_set_status(chip, PLC_STATUS_NOT_ALLOW);
		} else {
			if (chip->opp && chip->opp->desc->current_active &&
			    (chip->chg_mode == PLC_CHG_MODE_BUCK)) {
				/*
				* For protocols that require current activation,
				* need to wait for a while before disable charger
				*/
				oplus_plc_disable_charger(chip, false);
				oplus_plc_disable_charger(chip, true);
			}
			oplus_plc_protocol_enable(chip, PLC_CHG_MODE_AUTO);
		}
	} else {
		chip->cpa_current_type = data.intval;
		chip->opp = oplus_plc_get_protocol(chip);
		if (!oplus_plc_protocol_support(chip) || !chip->wired_online)
			oplus_plc_set_status(chip, PLC_STATUS_NOT_ALLOW);
		else
			oplus_plc_set_status(chip, PLC_STATUS_DISABLE);
	}
	mutex_unlock(&chip->status_control_lock);
}

static void oplus_plc_cpa_subs_callback(struct mms_subscribe *subs,
					enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_plc *chip = subs->priv_data;

	switch (type) {
	case MSG_TYPE_TIMER:
		break;
	case MSG_TYPE_ITEM:
		switch (id) {
		case CPA_ITEM_ALLOW:
			chip->protocol_change_jiffies = jiffies;
			schedule_work(&chip->protocol_change_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_plc_subscribe_cpa_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_plc *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->cpa_topic = topic;
	chip->cpa_subs =
		oplus_mms_subscribe(chip->cpa_topic, chip,
				    oplus_plc_cpa_subs_callback, "plc");
	if (IS_ERR_OR_NULL(chip->cpa_subs)) {
		chg_err("subscribe cpa topic error, rc=%ld\n",
			PTR_ERR(chip->cpa_subs));
		return;
	}

	oplus_mms_get_item_data(chip->cpa_topic, CPA_ITEM_ALLOW, &data, true);
	chip->cpa_current_type = data.intval;
	chip->opp = oplus_plc_get_protocol(chip);
	mutex_lock(&chip->status_control_lock);
	if (!oplus_plc_protocol_support(chip) || !chip->wired_online)
		oplus_plc_set_status(chip, PLC_STATUS_NOT_ALLOW);
	else
		oplus_plc_set_status(chip, PLC_STATUS_DISABLE);
	mutex_unlock(&chip->status_control_lock);
}

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
static void oplus_plc_state_keep_ready_work(struct work_struct *work)
{
	struct oplus_chg_plc *chip =
		container_of(work, struct oplus_chg_plc, state_keep_ready_work);
	enum plc_enable_status plc_status;
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(chip->keep_topic, STATE_KEEP_ITEM_READY, &data, false);
	if (rc < 0) {
		chg_err("cannot get state_keep ready status, rc=%d\n", rc);
		return;
	}
	if (data.intval == 0)
		return;
	if (!chip->user_enabled)
		return;

	rc = oplus_mms_get_item_data(chip->keep_topic, STATE_KEEP_ITEM_PLC_STATUS, &data, false);
	if (rc < 0) {
		chg_err("cannot get state_keep plc status, rc=%d\n", rc);
		return;
	}
	plc_status = data.intval;

	rc = oplus_mms_get_item_data(chip->keep_topic, STATE_KEEP_ITEM_WIRED_ONLINE, &data, false);
	if (rc < 0) {
		chg_err("cannot get state_keep wired online, rc=%d\n", rc);
		chip->keep_wired_online = false;
	} else {
		chip->keep_wired_online = !!data.intval;
	}

	chg_info("keep_plc_status=%s, keep_wired_online=%d\n",
		plc_enable_status_str(plc_status), chip->keep_wired_online);
	if (plc_status != PLC_STATUS_ENABLE || !chip->keep_wired_online)
		return;

	mutex_lock(&chip->restore_lock);
	if (chip->wired_online) {
		chip->restore_plc = false;
		rc = oplus_chg_plc_enable_action(chip, true);
		chg_info("try restore plc, rc=%d\n", rc);
	} else {
		chip->restore_plc = true;
	}
	mutex_unlock(&chip->restore_lock);
}

static void oplus_plc_keep_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_chg_plc *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case STATE_KEEP_ITEM_READY:
			schedule_work(&chip->state_keep_ready_work);
			break;
		case STATE_KEEP_ITEM_WIRED_ONLINE:
			rc = oplus_mms_get_item_data(chip->keep_topic, id, &data, false);
			if (rc < 0) {
				chg_err("cannot get state_keep wired online, rc=%d\n", rc);
				chip->keep_wired_online = false;
			} else {
				chip->keep_wired_online = !!data.intval;
			}
			if (!chip->keep_wired_online)
				chip->user_enabled = false;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void oplus_plc_subscribe_keep_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_chg_plc *chip = prv_data;

	chip->keep_topic = topic;
	chip->keep_subs =
		oplus_mms_subscribe(chip->keep_topic, chip,
				    oplus_plc_keep_subs_callback, "plc");
	if (IS_ERR_OR_NULL(chip->keep_subs)) {
		chg_err("subscribe state_keep topic error, rc=%ld\n",
			PTR_ERR(chip->keep_subs));
	}
}
#endif /* CONFIG_OPLUS_CHG_STATE_KEEP */

static int oplus_plc_update_enable_status(struct oplus_mms *mms,
					    union mms_msg_data *data)
{
	struct oplus_chg_plc *chip;

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

	data->intval = chip->plc_status;

	return 0;
}

static int oplus_plc_update_enable_cnts(struct oplus_mms *mms,
					    union mms_msg_data *data)
{
	struct oplus_chg_plc *chip;

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

	data->intval = chip->enable_cnts;
	return 0;
}

static int oplus_plc_update_chg_mode(struct oplus_mms *mms,
				     union mms_msg_data *data)
{
	struct oplus_chg_plc *chip;
	int chg_mode;

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

	chg_mode = oplus_plc_protocol_get_chg_mode(chip);
	if (chg_mode < 0) {
		chg_err("cannot get chg mode, rc=%d\n", chg_mode);
		return chg_mode;
	}

	data->intval = chg_mode;
	return 0;
}

static void oplus_plc_topic_update(struct oplus_mms *mms, bool publish)
{
}

static struct mms_item oplus_plc_item[] = {
	{
		.desc = {
			.item_id = PLC_ITEM_STATUS,
			.update = oplus_plc_update_enable_status,
		}
	}, {
		.desc = {
			.item_id = PLC_ITEM_ENABLE_CNTS,
			.update = oplus_plc_update_enable_cnts,
		}
	}, {
		.desc = {
			.item_id = PLC_ITEM_CHG_MODE,
			.update = oplus_plc_update_chg_mode,
		}
	}
};

static const struct oplus_mms_desc oplus_plc_desc = {
	.name = "plc",
	.type = OPLUS_MMS_TYPE_PLC,
	.item_table = oplus_plc_item,
	.item_num = ARRAY_SIZE(oplus_plc_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = oplus_plc_topic_update,
};

static int oplus_plc_topic_init(struct oplus_chg_plc *chip)
{
	struct oplus_mms_config mms_cfg = {};
	int rc;

	mms_cfg.drv_data = chip;
	mms_cfg.of_node = chip->dev->of_node;

	chip->plc_status = PLC_STATUS_NOT_ALLOW;
	chip->plc_topic =
		devm_oplus_mms_register(chip->dev, &oplus_plc_desc, &mms_cfg);
	if (IS_ERR(chip->plc_topic)) {
		chg_err("Couldn't register plc topic\n");
		rc = PTR_ERR(chip->plc_topic);
		return rc;
	}
	oplus_plc_subscribe_plc_topic(chip);
	oplus_mms_wait_topic("common", oplus_plc_subscribe_comm_topic, chip);
	oplus_mms_wait_topic("wired", oplus_plc_subscribe_wired_topic, chip);
	oplus_mms_wait_topic("gauge", oplus_plc_subscribe_gauge_topic, chip);
	oplus_mms_wait_topic("cpa", oplus_plc_subscribe_cpa_topic, chip);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	oplus_mms_wait_topic("state_keep", oplus_plc_subscribe_keep_topic, chip);
#endif

	return 0;
}

static int oplus_plc_parse_dt(struct oplus_chg_plc *chip)
{
	struct device_node *node = oplus_get_node_by_type(chip->dev->of_node);
	int rc;

	rc = of_property_read_u32(node, "oplus,plc_buck",
				  &chip->plc_buck);
	if (rc < 0) {
		chg_err("get oplus,plc_buck property error, rc=%d\n",
			rc);
		chip->plc_buck = 0;
	}
	rc = of_property_read_u32(node, "oplus,plc_soc",
				  &chip->plc_soc);
	if (rc < 0) {
		chg_err("get oplus,plc_soc property error, rc=%d\n",
			rc);
		chip->plc_soc = 90;
	}

	chip->plc_support_timeout = of_property_read_bool(node, "oplus,plc_support_timeout");
	return 0;
}

static int oplus_plc_force_buck_vote_callback(
	struct votable *votable, void *data, int val, const char *client, bool step)
{
	struct oplus_chg_plc *chip = data;

	if (val < 0)
		chip->force_buck = false;
	else
		chip->force_buck = !!val;
	chg_info("force_buck set to %s by %s\n",
		 chip->force_buck ? "true" : "false", client);

	return 0;
}

static int oplus_plc_vote_init(struct oplus_chg_plc *chip)
{
	int rc;

	chip->force_buck_votable = create_votable("PLC_FORCE_BUCK", VOTE_SET_ANY,
		oplus_plc_force_buck_vote_callback, chip);
	if (IS_ERR(chip->force_buck_votable)) {
		rc = PTR_ERR(chip->force_buck_votable);
		chip->force_buck_votable = NULL;
		chg_err("PLC_FORCE_BUCK votable create error, rc=%d\n", rc);
		return rc;
	}
	vote(chip->force_buck_votable, DEF_VOTER, chip->plc_buck, chip->plc_buck, false);

	return 0;
}

static int oplus_buck_plc_enable(struct oplus_plc_protocol *opp, enum oplus_plc_chg_mode mode)
{
	return 0;
}

static int oplus_buck_plc_disable(struct oplus_plc_protocol *opp)
{
	return 0;
}

static int oplus_buck_plc_get_chg_mode(struct oplus_plc_protocol *opp)
{
	return PLC_CHG_MODE_BUCK;
}

static struct oplus_plc_protocol_desc g_plc_protocol_desc = {
	.name = "buck",
	.protocol = BIT(CHG_PROTOCOL_BC12) |
		    BIT(CHG_PROTOCOL_PD) |
		    BIT(CHG_PROTOCOL_QC),
	.current_active = false,
	.ops = {
		.enable = oplus_buck_plc_enable,
		.disable = oplus_buck_plc_disable,
		.reset_protocol = NULL,
		.set_ibus = NULL,
		.get_ibus = NULL,
		.get_chg_mode = oplus_buck_plc_get_chg_mode,
	}
};

static ssize_t current_protocol_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_chg_plc *chip = pde_data(file_inode(file));
	struct oplus_plc_protocol *opp;
	char buf[PROC_DATA_BUF_SIZE] = { 0 };
	int len = 0;

	if (chip->opp != NULL)
		opp = chip->opp;
	else
		opp = chip->buck_opp;

	if (opp != NULL)
		len += scnprintf(buf, sizeof(buf) - 1, "%s[%s]\n",
				get_protocol_name_str(chip->cpa_current_type),
				opp->desc->name);
	else
		len += scnprintf(buf, sizeof(buf) - 1, "NULL\n");

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations current_protocol_ops =
{
	.read = current_protocol_proc_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops current_protocol_ops =
{
	.proc_read  = current_protocol_proc_read,
	.proc_lseek = noop_llseek,
};
#endif

static int oplus_chg_plc_probe(struct platform_device *pdev)
{
	struct oplus_chg_plc *chip;
	struct proc_dir_entry *pr_entry_tmp;
	int rc;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_chg_plc), GFP_KERNEL);
	if (chip == NULL) {
		chg_err("alloc oplus_chg_plc struct buffer error\n");
		return -ENOMEM;
	}
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	INIT_LIST_HEAD(&chip->protocol_list);
	oplus_plc_parse_dt(chip);
	rc = oplus_plc_vote_init(chip);
	if (rc < 0)
		goto vote_init_err;

	chip->plc_entry = proc_mkdir("charger/plc", NULL);
	if (chip->plc_entry == NULL) {
		chg_err("Couldn't create charger/plc proc entry\n");
	} else {
		pr_entry_tmp = proc_create_data("current_protocol", 0444,
			chip->plc_entry, &current_protocol_ops, chip);
		if (pr_entry_tmp == NULL)
			chg_err("Couldn't create current_protocol proc entry\n");
	}

	rc = oplus_plc_topic_init(chip);
	if (rc < 0)
		goto topic_reg_err;
	spin_lock_init(&chip->protocol_list_lock);
	chip->buck_opp = oplus_plc_register_protocol(chip->plc_topic,
		&g_plc_protocol_desc, chip->dev->of_node, chip);
	if (chip->buck_opp == NULL)
		chg_err("register buck plc protocol error");

	mutex_init(&chip->status_control_lock);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	mutex_init(&chip->restore_lock);
#endif
	INIT_DELAYED_WORK(&chip->plc_disable_wait_work, oplus_plc_disable_wait_work);
	INIT_DELAYED_WORK(&chip->plc_enable_timeout_work, oplus_plc_enable_timeout_work);
	INIT_DELAYED_WORK(&chip->charger_disable_work, oplus_plc_charger_disable_work);
	INIT_WORK(&chip->protocol_change_work, oplus_plc_protocol_change_work);
	INIT_WORK(&chip->chg_mode_change_work, oplus_plc_chg_mode_change_work);
	INIT_WORK(&chip->wired_online_work, oplus_plc_wired_online_work);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	INIT_WORK(&chip->state_keep_ready_work, oplus_plc_state_keep_ready_work);
#endif

	return 0;

topic_reg_err:
	if (chip->plc_entry != NULL)
		proc_remove(chip->plc_entry);
vote_init_err:
	devm_kfree(&pdev->dev, chip);
	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_chg_plc_remove(struct platform_device *pdev)
#else
static int oplus_chg_plc_remove(struct platform_device *pdev)
#endif
{
	struct oplus_chg_plc *chip = platform_get_drvdata(pdev);

	oplus_plc_release_protocol(chip->plc_topic, chip->buck_opp);
	if (!IS_ERR_OR_NULL(chip->comm_subs))
		oplus_mms_unsubscribe(chip->comm_subs);
	if (!IS_ERR_OR_NULL(chip->gauge_subs))
		oplus_mms_unsubscribe(chip->gauge_subs);
	if (!IS_ERR_OR_NULL(chip->wired_subs))
		oplus_mms_unsubscribe(chip->wired_subs);
	if (!IS_ERR_OR_NULL(chip->cpa_subs))
		oplus_mms_unsubscribe(chip->cpa_subs);
#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)
	if (!IS_ERR_OR_NULL(chip->keep_subs))
		oplus_mms_unsubscribe(chip->keep_subs);
#endif

	if (chip->plc_entry != NULL)
		proc_remove(chip->plc_entry);
	if (chip->force_buck_votable != NULL)
		destroy_votable(chip->force_buck_votable);

	devm_kfree(&pdev->dev, chip);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return 0;
#endif
}

static const struct of_device_id oplus_chg_plc_match[] = {
	{ .compatible = "oplus,plc_charge" },
	{},
};

static struct platform_driver oplus_chg_plc_driver = {
	.driver = {
		.name = "oplus-plc_charge",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(oplus_chg_plc_match),
	},
	.probe = oplus_chg_plc_probe,
	.remove = oplus_chg_plc_remove,
};

static __init int oplus_chg_plc_init(void)
{
	return platform_driver_register(&oplus_chg_plc_driver);
}

static __exit void oplus_chg_plc_exit(void)
{
	platform_driver_unregister(&oplus_chg_plc_driver);
}

oplus_chg_module_register(oplus_chg_plc);

/* PLC API */

const char *oplus_plc_chg_mode_str(enum oplus_plc_chg_mode mode)
{
	switch(mode) {
	case PLC_CHG_MODE_BUCK:
		return "buck";
	case PLC_CHG_MODE_CP:
		return "cp";
	case PLC_CHG_MODE_AUTO:
		return "auto";
	default:
		return "invalid";
	}
}

void *oplus_plc_protocol_get_priv_data(struct oplus_plc_protocol *opp)
{
	if (opp == NULL)
		return NULL;
	return opp->priv_data;
}

static ssize_t opp_current_strategy_proc_read(struct file *file, char __user *buff, size_t count, loff_t *off)
{
	struct oplus_plc_protocol *opp = pde_data(file_inode(file));
	char buf[PROC_DATA_BUF_SIZE] = { 0 };
	const char *name = NULL;
	int i;
	int len = 0;

	if (opp->strategy == NULL) {
		len += scnprintf(buf, sizeof(buf) - 1, "NULL[NULL]\n");
	} else {
		for (i = 0; i < opp->strategy_num; i++) {
			if (opp->strategy_groups[i].strategy == opp->strategy)
				name = opp->strategy_groups[i].name;
		}
		if (opp->strategy->node != NULL)
			len += scnprintf(buf, sizeof(buf) - 1, "%s[%s]\n",
					name, opp->strategy->node->name);
		else
			len += scnprintf(buf, sizeof(buf) - 1, "%s[NULL]\n", name);
	}
	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buff, buf, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

static ssize_t opp_current_strategy_proc_write(struct file *file, const char __user *buf, size_t len, loff_t *data)
{
	struct oplus_plc_protocol *opp = pde_data(file_inode(file));
	char tmp_buf[PROC_DATA_BUF_SIZE] = { 0 };
	int rc;

	if (len > PROC_DATA_BUF_SIZE - 1)
		return -EFAULT;
	if (copy_from_user(tmp_buf, buf, len))
		return -EFAULT;

	rc = oplus_plc_protocol_set_strategy(opp, tmp_buf);
	if (rc < 0)
		return rc;

	return len;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations opp_current_strategy_ops =
{
	.read = opp_current_strategy_proc_read,
	.write = opp_current_strategy_proc_write,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops opp_current_strategy_ops =
{
	.proc_read  = opp_current_strategy_proc_read,
	.proc_write = opp_current_strategy_proc_write,
	.proc_lseek = noop_llseek,
};
#endif

static int oplus_plc_protocol_proc_init(struct oplus_plc_protocol *opp)
{
	struct oplus_chg_plc *chip = opp->plc;
	struct proc_dir_entry *entry;

	if (chip->plc_entry == NULL)
		return 0;

	opp->entry = proc_mkdir(opp->desc->name, chip->plc_entry);
	if (opp->entry == NULL) {
		chg_err("Couldn't create charger/plc/%s proc entry\n",
			opp->desc->name);
		return -EFAULT;
	}

	entry = proc_create_data("current_strategy", 0644, opp->entry,
		&opp_current_strategy_ops, opp);
	if (entry == NULL)
		chg_err("%s: Couldn't create current_strategy proc entry\n",
			opp->desc->name);

	return 0;
}

static struct oplus_plc_protocol *oplus_plc_register_v1_ufcs_protocol(
	struct oplus_chg_plc *chip,
	struct oplus_plc_protocol_desc *desc,
	void *data)
{
	struct oplus_plc_protocol *opp;

	opp = devm_kzalloc(chip->dev,
		sizeof(struct oplus_plc_protocol) +
			sizeof(struct oplus_plc_strategy_group),
		GFP_KERNEL);
	if (opp == NULL) {
		chg_err("alloc opp buf error\n");
		return NULL;
	}
	opp->priv_data = data;
	opp->desc = desc;
	opp->plc = chip;
	opp->strategy_num = 1;
	oplus_plc_protocol_proc_init(opp);

	opp->strategy_groups[0].name = "default";
	opp->strategy_groups[0].strategy = step_strategy_alloc(opp, NULL, NULL);
	if (opp->strategy_groups[0].strategy == NULL) {
		chg_err("%s: strategy alloc error\n", opp->strategy_groups[0].name);
		goto strategy_alloc_err;
	}
	opp->strategy = opp->strategy_groups[0].strategy;

	opp->strategy->node = NULL;
	opp->strategy->entry = NULL;
	opp->strategy->opp = opp;
	opp->strategy->desc = &g_strategy_desc[0];

	spin_lock(&chip->protocol_list_lock);
	list_add(&opp->list, &chip->protocol_list);
	spin_unlock(&chip->protocol_list_lock);

	return opp;

strategy_alloc_err:
	if (opp->entry != NULL)
		proc_remove(opp->entry);
	devm_kfree(chip->dev, opp);
	return NULL;
}

struct oplus_plc_protocol *oplus_plc_register_protocol(
	struct oplus_mms *topic,
	struct oplus_plc_protocol_desc *desc,
	struct device_node *node,
	void *data)
{
	struct oplus_chg_plc *chip;
	struct oplus_plc_protocol *opp;
	struct device_node *strategy_node;
	int strategy_num;
	int i;
	int rc;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return NULL;
	}
	if (desc == NULL) {
		chg_err("desc is NULL");
		return NULL;
	}
	if (node == NULL) {
		chg_err("node is NULL");
		return NULL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return NULL;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip) {
		chg_err("chip is NULL");
		return NULL;
	}

	/* check ops */
	if (desc->ops.enable == NULL) {
		chg_err("enable func is NULL");
		return NULL;
	}
	if (desc->ops.disable == NULL) {
		chg_err("disable func is NULL");
		return NULL;
	}

	rc = of_property_count_elems_of_size(node, "oplus,plc_strategy-data",
					     sizeof(u32));
	if (rc < 0) {
		if (strcmp(desc->name, "ufcs") == 0)
			return oplus_plc_register_v1_ufcs_protocol(chip, desc, data);
		chg_err("can't get \"oplus,plc_strategy-data\" number, rc=%d\n", rc);
		return NULL;
	}
	strategy_num = rc;
	rc = of_property_count_strings(node, "oplus,plc_strategy-names");
	if (rc < 0) {
		chg_err("can't get \"oplus,plc_strategy-names\" number, rc=%d\n", rc);
		return NULL;
	}
	if (strategy_num != rc) {
		chg_err("\"oplus,plc_strategy-data\" and"
			"\"oplus,plc_strategy-names\" data do not match\n");
		return NULL;
	}

	opp = devm_kzalloc(chip->dev,
		sizeof(struct oplus_plc_protocol) +
			sizeof(struct oplus_plc_strategy_group) * strategy_num,
		GFP_KERNEL);
	if (opp == NULL) {
		chg_err("alloc opp buf error\n");
		return NULL;
	}
	opp->priv_data = data;
	opp->desc = desc;
	opp->plc = chip;
	opp->strategy_num = strategy_num;
	oplus_plc_protocol_proc_init(opp);

	for (i = 0; i < strategy_num; i++) {
		rc = of_property_read_string_index(node,
			"oplus,plc_strategy-names", i,
			&opp->strategy_groups[i].name);
		if (rc < 0) {
			chg_err("cannot parse \"oplus,plc_strategy-data\", rc=%d", rc);
			strategy_num = i;
			goto strategy_alloc_err;
		}
		strategy_node = of_parse_phandle(node, "oplus,plc_strategy-data", i);
		if (strategy_node == NULL) {
			chg_err("cannot parse \"oplus,plc_strategy-names\", rc=%d", rc);
			strategy_num = i;
			goto strategy_alloc_err;
		}
		opp->strategy_groups[i].strategy = oplus_plc_strategy_alloc(opp, strategy_node);
		if (opp->strategy_groups[i].strategy == NULL) {
			chg_err("%s: strategy alloc error\n", opp->strategy_groups[i].name);
			strategy_num = i;
			goto strategy_alloc_err;
		}
		if (strcmp(opp->strategy_groups[i].name, "default") == 0) {
			opp->strategy = opp->strategy_groups[i].strategy;
			chg_info("%s: use %s[%s] strategy\n", desc->name,
				 opp->strategy_groups[i].name,
				 oplus_plc_strategy_type_str(opp->strategy->desc->type));
		}
	}

	spin_lock(&chip->protocol_list_lock);
	list_add(&opp->list, &chip->protocol_list);
	spin_unlock(&chip->protocol_list_lock);

	return opp;

strategy_alloc_err:
	if (strategy_num > 0) {
		for (i = strategy_num - 1; i >= 0; i--)
			oplus_plc_strategy_release(opp->strategy_groups[i].strategy);
	}
	if (opp->entry != NULL)
		proc_remove(opp->entry);
	devm_kfree(chip->dev, opp);
	return NULL;
}

void oplus_plc_release_protocol(struct oplus_mms *topic, struct oplus_plc_protocol *opp)
{
	struct oplus_chg_plc *chip;
	int i;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return;
	}
	if (opp == NULL) {
		chg_err("opp is NULL");
		return;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip) {
		chg_err("chip is NULL");
		return;
	}

	for (i = 0; i < opp->strategy_num; i++)
		oplus_plc_strategy_release(opp->strategy_groups[i].strategy);
	if (opp->entry != NULL)
		proc_remove(opp->entry);
	spin_lock(&chip->protocol_list_lock);
	list_del(&opp->list);
	spin_unlock(&chip->protocol_list_lock);
	devm_kfree(chip->dev, opp);
}

static struct oplus_plc_strategy *oplus_plc_protocol_find_strategy(
	struct oplus_plc_protocol *opp, const char *name)
{
	int i;

	for (i = 0; i < opp->strategy_num; i++) {
		if (strcmp(opp->strategy_groups[i].name, name) == 0)
			return opp->strategy_groups[i].strategy;
	}
	return NULL;
}

int oplus_plc_protocol_set_strategy(struct oplus_plc_protocol *opp, const char *name)
{
	struct oplus_plc_strategy *strategy;
	struct oplus_chg_plc *chip;
	int rc;

	if (opp == NULL) {
		chg_err("opp is NULL\n");
		return -EINVAL;
	}
	if (name == NULL) {
		chg_err("name is NULL\n");
		return -EINVAL;
	}
	chip = opp->plc;

	strategy = oplus_plc_protocol_find_strategy(opp, name);
	if (strategy == NULL) {
		chg_err("%s: %s strategy not found\n", opp->desc->name, name);
		return -ENOTSUPP;
	}
	if (opp->strategy != NULL && opp->strategy->started)
		oplus_plc_strategy_exit(opp->strategy);
	opp->strategy = strategy;
	chg_info("%s: use %s[%s] strategy\n", opp->desc->name,
		 name, oplus_plc_strategy_type_str(strategy->desc->type));
	if (chip->opp == opp && opp->enable) {
		rc = oplus_plc_strategy_init(opp->strategy);
		if (rc < 0) {
			chg_err("%s: strategy init error, rc=%d\n",
				opp->desc->name, rc);
			return rc;
		}
		rc = oplus_plc_strategy_start(opp->strategy);
		if (rc < 0) {
			chg_err("%s: strategy start error, rc=%d\n",
				opp->desc->name, rc);
			return rc;
		}
	}

	return 0;
}

#define PLC_ENABLE_TIMEOUT_DELAY	300000
static int oplus_chg_plc_schedule_timeout_work(struct oplus_chg_plc *chip)
{
	if (!chip->plc_support_timeout)
		return 0;

	schedule_delayed_work(&chip->plc_enable_timeout_work,
			msecs_to_jiffies(PLC_ENABLE_TIMEOUT_DELAY));

	return 0;
}

static int oplus_chg_plc_enable_action(struct oplus_chg_plc *chip, bool enable)
{
	int rc = 0;

#define PLC_DISABLE_WAIT_DELAY		1000
	chg_info("plc enable: %d\n", enable);

	cancel_delayed_work_sync(&chip->plc_enable_timeout_work);

	mutex_lock(&chip->status_control_lock);
	if (!enable) {
		if (chip->plc_status != PLC_STATUS_ENABLE)
			goto out;
		/* disable work */
		schedule_delayed_work(&chip->plc_disable_wait_work,
			msecs_to_jiffies(PLC_DISABLE_WAIT_DELAY));
		goto out;
	}

	if (work_busy(&chip->plc_disable_wait_work.work)) {
		mutex_unlock(&chip->status_control_lock);
		cancel_delayed_work_sync(&chip->plc_disable_wait_work);
		mutex_lock(&chip->status_control_lock);
	}
	/*
	 * Check the plc_status again to determine
	 * if it needs to be reopened
	 */
	if (chip->plc_status == PLC_STATUS_ENABLE) {
		oplus_chg_plc_schedule_timeout_work(chip);
		goto out;
	}
	if (!chip->wired_online) {
		chg_err("wired_online is false\n");
		rc = -EFAULT;
		goto out;
	}
	if (chip->plc_status == PLC_STATUS_NOT_ALLOW) {
		chg_err("plc_status is not_allow\n");
		rc = -EFAULT;
		goto out;
	}

	rc = __oplus_chg_plc_enable(chip);
	if (rc == -ENOTSUPP) {
		oplus_plc_set_status(chip, PLC_STATUS_NOT_ALLOW);
	} else if (rc < 0) {
		chg_err("plc enbale error, rc=%d\n", rc);
	} else {
		oplus_plc_set_status(chip, PLC_STATUS_ENABLE);
		oplus_chg_plc_schedule_timeout_work(chip);
	}
out:
	mutex_unlock(&chip->status_control_lock);
	return rc;
}

int oplus_chg_plc_enable(struct oplus_mms *topic, bool enable)
{
	struct oplus_chg_plc *chip;
	int rc;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -EINVAL;
	}
	chip = oplus_mms_get_drvdata(topic);
	if (!chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}

	rc = oplus_chg_plc_enable_action(chip, enable);
	if (rc < 0) {
		chg_err("plc action error, rc=%d\n", rc);
		if (!enable)
			chip->user_enabled = enable;
	} else {
		chip->user_enabled = enable;
	}

	return rc;
}
