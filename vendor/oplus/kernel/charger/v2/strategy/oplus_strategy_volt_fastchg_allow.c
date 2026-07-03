// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[STRATEGY_VFA]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_strategy.h>
#include <oplus_chg_voter.h>

#define VFA_PAUSE_TIME_MS	2000
#define VFA_PAUSE_STEP_MV	200
#define VFA_MARGIN_MAX_MV	1000

struct vfa_strategy {
	struct oplus_chg_strategy strategy;
	struct oplus_mms *gauge_topic;
	struct votable *wired_charging_disable_votable;

	int min_allow_vbat_mv;
	int first_allow_vbat_mv;
	int over_margin_timeout_ms;

	int current_margin_mv;
	unsigned long over_margin_jiffies;
	int stop_step_idx;
	bool hold_margin_update;
	int vbat_mv;

	bool first_checked;
	bool last_allow;
	struct delayed_work resume_charge_work;
};

static bool is_gauge_topic_available(struct vfa_strategy *vfa)
{
	if (!vfa->gauge_topic)
		vfa->gauge_topic = oplus_mms_get_by_name("gauge");

	return !!vfa->gauge_topic;
}

static bool is_wired_charging_disable_votable_available(struct vfa_strategy *vfa)
{
	if (!vfa->wired_charging_disable_votable)
		vfa->wired_charging_disable_votable = find_votable("WIRED_CHARGING_DISABLE");
	return !!vfa->wired_charging_disable_votable;
}

static int vfa_check_resources(struct oplus_chg_strategy *strategy, struct vfa_strategy **out_vfa)
{
	struct vfa_strategy *vfa;

	if (strategy == NULL || out_vfa == NULL) {
		chg_err("strategy or out_vfa is NULL\n");
		return -EINVAL;
	}

	vfa = (struct vfa_strategy *)strategy;
	*out_vfa = vfa;

	if (!is_gauge_topic_available(vfa)) {
		chg_err("gauge topic not found\n");
		return -ENODEV;
	}

	if (!is_wired_charging_disable_votable_available(vfa)) {
		chg_err("WIRED_CHARGING_DISABLE votable not found\n");
		return -ENODEV;
	}

	return 0;
}

static void vfa_resume_charge_work(struct work_struct *work)
{
	union mms_msg_data data = { 0 };
	struct vfa_strategy *vfa =
		container_of(work, struct vfa_strategy, resume_charge_work.work);

	if (!is_wired_charging_disable_votable_available(vfa))
		return;

	oplus_mms_get_item_data(vfa->gauge_topic, GAUGE_ITEM_VOL_MIN, &data, true);
	vfa->vbat_mv = data.intval;
	vote(vfa->wired_charging_disable_votable, VFA_VOTER, false, 0, false);

	if (vfa->vbat_mv >= vfa->min_allow_vbat_mv) {
		vfa->last_allow = true;
	} else if (!vfa->hold_margin_update) {
		vfa->current_margin_mv += vfa->min_allow_vbat_mv - vfa->vbat_mv;
		vfa->current_margin_mv = min(vfa->current_margin_mv, VFA_MARGIN_MAX_MV);
	}
	chg_info("resume charging, vbat=%d, allow=%d, margin_mv=%d, hold_margin=%d\n",
		vfa->vbat_mv, vfa->last_allow, vfa->current_margin_mv, vfa->hold_margin_update);
	vfa->hold_margin_update = false;
}

static struct oplus_chg_strategy *vfa_strategy_alloc(unsigned char *buf, size_t size)
{
	return ERR_PTR(-ENOTSUPP);
}

static struct oplus_chg_strategy *vfa_strategy_alloc_by_node(struct device_node *node)
{
	struct vfa_strategy *vfa;
	u32 data;
	int rc;

	if (node == NULL) {
		chg_err("node is NULL\n");
		return ERR_PTR(-EINVAL);
	}

	vfa = kzalloc(sizeof(*vfa), GFP_KERNEL);
	if (vfa == NULL) {
		chg_err("alloc strategy memory error\n");
		return ERR_PTR(-ENOMEM);
	}

	rc = of_property_read_u32(node, "oplus,fastchg_allow_min_vbat_mv", &data);
	if (rc < 0) {
		chg_err("read fastchg_allow_min_vbat_mv failed, rc=%d\n", rc);
		kfree(vfa);
		return ERR_PTR(rc);
	}
	vfa->min_allow_vbat_mv = data;

	rc = of_property_read_u32(node, "oplus,fastchg_allow_first_vbat_mv", &data);
	if (rc < 0) {
		chg_err("read oplus,fastchg_allow_first_vbat_mv failed, rc=%d\n", rc);
		kfree(vfa);
		return ERR_PTR(rc);
	}
	vfa->first_allow_vbat_mv = data;

	rc = of_property_read_u32(node, "oplus,fastchg_allow_over_margin_timeout_ms",
		&data);
	if (rc < 0) {
		chg_err("read oplus,fastchg_allow_over_margin_timeout_ms failed, rc=%d\n",
			rc);
		kfree(vfa);
		return ERR_PTR(rc);
	}
	vfa->over_margin_timeout_ms = data;
	vfa->hold_margin_update = false;
	INIT_DELAYED_WORK(&vfa->resume_charge_work, vfa_resume_charge_work);

	chg_info("vfa_node=%s min_allow_vbat_mv=%d first_allow=%d timeout_ms=%d\n",
		kbasename(node->full_name),
		vfa->min_allow_vbat_mv, vfa->first_allow_vbat_mv,
		vfa->over_margin_timeout_ms);

	return &vfa->strategy;
}

static int vfa_strategy_release(struct oplus_chg_strategy *strategy)
{
	struct vfa_strategy *vfa;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	vfa = (struct vfa_strategy *)strategy;

	cancel_delayed_work_sync(&vfa->resume_charge_work);
	kfree(vfa);

	return 0;
}

static int vfa_strategy_init(struct oplus_chg_strategy *strategy)
{
	struct vfa_strategy *vfa;
	int rc;

	rc = vfa_check_resources(strategy, &vfa);
	if (rc)
		return rc;

	vfa->current_margin_mv = 0;
	vfa->over_margin_jiffies = 0;
	vfa->stop_step_idx = 1;
	vfa->hold_margin_update = false;
	cancel_delayed_work_sync(&vfa->resume_charge_work);
	vote(vfa->wired_charging_disable_votable, VFA_VOTER, false, 0, false);

	return 0;
}

static bool vfa_try_first_stage(struct vfa_strategy *vfa, bool *allow_fastchg)
{
	if (vfa->last_allow) {
		*allow_fastchg = true;
		return true;
	}

	if (!vfa->first_checked) {
		vfa->first_checked = true;
		if (vfa->vbat_mv >= vfa->first_allow_vbat_mv) {
			*allow_fastchg = true;
		} else {
			cancel_delayed_work_sync(&vfa->resume_charge_work);
			vote(vfa->wired_charging_disable_votable, VFA_VOTER, true, 1, false);
			vfa->hold_margin_update = true;
			schedule_delayed_work(&vfa->resume_charge_work, msecs_to_jiffies(VFA_PAUSE_TIME_MS));
		}
		return true;
	}

	return false;
}

static bool vfa_try_stop_step(struct vfa_strategy *vfa)
{
	if (vfa->stop_step_idx > VFA_MARGIN_MAX_MV / VFA_PAUSE_STEP_MV)
		return false;

	if (vfa->current_margin_mv < VFA_PAUSE_STEP_MV * vfa->stop_step_idx)
		return false;

	if (vfa->vbat_mv >= vfa->min_allow_vbat_mv + VFA_PAUSE_STEP_MV * vfa->stop_step_idx) {
		cancel_delayed_work_sync(&vfa->resume_charge_work);
		vote(vfa->wired_charging_disable_votable, VFA_VOTER, true, 1, false);
		vfa->hold_margin_update = true;
		schedule_delayed_work(&vfa->resume_charge_work, msecs_to_jiffies(VFA_PAUSE_TIME_MS));
		vfa->stop_step_idx++;
		chg_info("step pause idx=%d, vbat=%d\n", vfa->stop_step_idx, vfa->vbat_mv);
		return true;
	}

	return false;
}

static bool vfa_try_over_margin(struct vfa_strategy *vfa, bool *allow_fastchg)
{
	if (vfa->stop_step_idx > VFA_MARGIN_MAX_MV / VFA_PAUSE_STEP_MV &&
	    vfa->vbat_mv >= vfa->min_allow_vbat_mv + VFA_MARGIN_MAX_MV) {
		if (!vfa->over_margin_jiffies)
			vfa->over_margin_jiffies = jiffies;
		if (time_after(jiffies, vfa->over_margin_jiffies +
			       msecs_to_jiffies(vfa->over_margin_timeout_ms))) {
			*allow_fastchg = true;
			return true;
		}
		return false;
	}

	vfa->over_margin_jiffies = 0;
	return false;
}

static bool vfa_try_margin_pause(struct vfa_strategy *vfa)
{
	if (vfa->vbat_mv >= vfa->min_allow_vbat_mv + vfa->current_margin_mv) {
		cancel_delayed_work_sync(&vfa->resume_charge_work);
		vote(vfa->wired_charging_disable_votable, VFA_VOTER, true, 1, false);
		schedule_delayed_work(&vfa->resume_charge_work, msecs_to_jiffies(VFA_PAUSE_TIME_MS));
		return true;
	}

	return false;
}

static int vfa_strategy_get_data(struct oplus_chg_strategy *strategy, void *ret)
{
	struct vfa_strategy *vfa;
	union mms_msg_data data = { 0 };
	int rc;
	bool allow_fastchg = false;

	if (ret == NULL) {
		chg_err("ret is NULL\n");
		return -EINVAL;
	}

	rc = vfa_check_resources(strategy, &vfa);
	if (rc)
		return rc;

	oplus_mms_get_item_data(vfa->gauge_topic, GAUGE_ITEM_VOL_MIN, &data, true);
	vfa->vbat_mv = data.intval;

	if (vfa_try_first_stage(vfa, &allow_fastchg))
		goto done;

	if (get_client_vote(vfa->wired_charging_disable_votable, VFA_VOTER) > 0) {
		chg_info("stop charging, vbat=%d\n", vfa->vbat_mv);
		goto done;
	}

	if (vfa_try_stop_step(vfa))
		goto done;

	if (vfa_try_over_margin(vfa, &allow_fastchg))
		goto done;

	if (vfa_try_margin_pause(vfa))
		goto done;

done:
	chg_info("vbat=%d, min_allow=%d, margin_cur=%d, stop_idx=%d, last_allow=%d, allow=%d, over_margin_ms=%u\n",
		vfa->vbat_mv, vfa->min_allow_vbat_mv, vfa->current_margin_mv,
		vfa->stop_step_idx, vfa->last_allow, allow_fastchg,
		vfa->over_margin_jiffies ? jiffies_to_msecs(vfa->over_margin_jiffies - INITIAL_JIFFIES) : 0);
	vfa->last_allow = allow_fastchg;

	*((int *)ret) = allow_fastchg ? 1 : 0;

	return 0;
}

static int vfa_strategy_set_process_data(struct oplus_chg_strategy *strategy,
	const char *type, unsigned long arg)
{
	struct vfa_strategy *vfa;

	if (strategy == NULL || type == NULL)
		return -EINVAL;

	vfa = (struct vfa_strategy *)strategy;

	if (strncmp(type, "reset", strlen("reset")) == 0) {
		vfa->first_checked = false;
		vfa->last_allow = false;
		vfa->over_margin_jiffies = 0;
		vfa->current_margin_mv = 0;
		vfa->stop_step_idx = 1;
		vfa->hold_margin_update = false;
		cancel_delayed_work_sync(&vfa->resume_charge_work);
		vote(vfa->wired_charging_disable_votable, VFA_VOTER, false, 0, false);
		return 0;
	}

	return -ENOTSUPP;
}

static struct oplus_chg_strategy_desc vfa_strategy_desc = {
	.name = "vfa_strategy",
	.strategy_init = vfa_strategy_init,
	.strategy_release = vfa_strategy_release,
	.strategy_alloc = vfa_strategy_alloc,
	.strategy_alloc_by_node = vfa_strategy_alloc_by_node,
	.strategy_get_data = vfa_strategy_get_data,
	.strategy_set_process_data = vfa_strategy_set_process_data,
};

int vfa_strategy_register(void)
{
	return oplus_chg_strategy_register(&vfa_strategy_desc);
}

