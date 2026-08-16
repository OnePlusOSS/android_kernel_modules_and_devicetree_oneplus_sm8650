// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[REC/SK/VDD]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_chg_voter.h>
#include <oplus_mms_wired.h>
#include <recovery/state_keep.h>

struct vooc_disconnect_detection {
	struct device_node *node;
	struct state_keep_client *client;

	struct oplus_mms *wired_topic;
	struct mms_subscribe *wired_subs;
	struct oplus_mms *keep_topic;
	struct mms_subscribe *keep_subs;
	struct oplus_mms *vooc_topic;
	struct mms_subscribe *vooc_subs;

	struct votable *vooc_curr_votable;

	struct work_struct wired_online_check_work;
	struct work_struct hw_detect_update_work;
	struct delayed_work hw_detect_check_work;
	struct delayed_work final_check_work;
	struct completion check_complete;

	bool check_start;
	bool keep_online;
	int hw_detect;
	unsigned int hw_detect_check_count;
	unsigned long disable_protocol_mask;
	unsigned long wired_disconnect_time;
	bool wired_present;

	bool wired_online;
};

static struct vooc_disconnect_detection *g_vdd;

static int g_vooc_current_table[] = {
	   0,    0, /* 0 means no current limit */
	7000, 7000,
	6000, 6000,
	5000, 5000,
	4000, 4000,
	3000, 3000,
	2000, 2000
};

#define HW_DETECT_CHECK_TIME_MS		50
#define HW_DETECT_CHECK_COUNT		3
#define DISCONNECT_TIME_THR_MS		3000
#define NOT_ABNORMAL_DIS_COUNT_THR	3
#define CHECK_MAX_TIME_MS		((DISCONNECT_TIME_THR_MS) + 500)

static bool is_vooc_curr_votable_available(struct vooc_disconnect_detection *vdd)
{
	if (!vdd->vooc_curr_votable)
		vdd->vooc_curr_votable = find_votable("VOOC_CURR");
	return !!vdd->vooc_curr_votable;
}

static void vdd_check_done(struct vooc_disconnect_detection *vdd, bool keep)
{
	chg_info("keep=%d\n", keep);
	vdd->check_start = false;
	vdd->keep_online = keep;
	vdd->hw_detect_check_count = 0;
	complete_all(&vdd->check_complete);
	if (!keep) {
		if (is_vooc_curr_votable_available(vdd))
			vote(vdd->vooc_curr_votable, REC_VDD_VOTER, false, 0, false);
	}
}

static void vdd_state_reset(struct vooc_disconnect_detection *vdd)
{
	chg_info("reset\n");
	vdd->check_start = false;
	vdd->keep_online = false;
	vdd->hw_detect_check_count = 0;
	if (is_vooc_curr_votable_available(vdd))
		vote(vdd->vooc_curr_votable, REC_VDD_VOTER, false, 0, false);
}

static int vdd_client_reset(struct state_keep_client *client)
{
	struct vooc_disconnect_detection *vdd = client->priv_data;

	if (vdd == NULL) {
		chg_err("vdd is null\n");
		return -EINVAL;
	}
	vdd_state_reset(vdd);

	return 0;
}

static int vdd_client_enable(struct state_keep_client *client)
{
	struct vooc_disconnect_detection *vdd = client->priv_data;

	if (vdd == NULL) {
		chg_err("vdd is null\n");
		return -EINVAL;
	}
	vdd_state_reset(vdd);
	client->enabled = true;

	return 0;
}

static int vdd_client_disable(struct state_keep_client *client)
{
	struct vooc_disconnect_detection *vdd = client->priv_data;

	if (vdd == NULL) {
		chg_err("vdd is null\n");
		return -EINVAL;
	}
	client->enabled = false;
	if (is_vooc_curr_votable_available(vdd))
		vote(vdd->vooc_curr_votable, REC_VDD_VOTER, false, 0, false);

	return 0;
}

static int vdd_client_start_check(struct state_keep_client *client, enum oplus_chg_protocol_type protocol)
{
	struct vooc_disconnect_detection *vdd = client->priv_data;

	if (vdd == NULL) {
		chg_err("vdd is null\n");
		return -EINVAL;
	}
	if (protocol != CHG_PROTOCOL_VOOC) {
		vdd_state_reset(vdd);
		return 0;
	}

	vdd->wired_online = false;
	cancel_delayed_work_sync(&vdd->hw_detect_check_work);
	vdd->check_start = true;
	vdd->keep_online = false;
	vdd->hw_detect_check_count = 0;
	reinit_completion(&vdd->check_complete);
	schedule_delayed_work(&vdd->hw_detect_check_work, 0);

	return 0;
}

static bool vdd_client_need_keep(struct state_keep_client *client, enum oplus_chg_protocol_type protocol)
{
	struct vooc_disconnect_detection *vdd = client->priv_data;
	unsigned long left;

	if (vdd == NULL) {
		chg_err("vdd is null\n");
		return false;
	}

	if (protocol != CHG_PROTOCOL_VOOC) {
		vdd_state_reset(vdd);
		return false;
	}

	if (!vdd->check_start)
		return vdd->keep_online;
	left = wait_for_completion_timeout(&vdd->check_complete, msecs_to_jiffies(CHECK_MAX_TIME_MS));
	if (left == 0) {
		chg_err("check timeout\n");
		return false;
	}
	return vdd->keep_online;
}

static unsigned int vdd_check_current_table(struct vooc_disconnect_detection *vdd,
	const int32_t *current_table, int32_t current_table_num, unsigned int count)
{
	int curr;

	if (count >= current_table_num) {
		chg_info("switch count is greater than %d\n", current_table_num);
		vdd_state_reset(vdd);
		return (SK_SWITCH_SWITCH_PROTOCOL | SK_SWITCH_DISABLE_PROTOCOL);
	}
	curr = current_table[count];
	chg_info("count=%u, curr=%d\n", count, curr);
	if (curr > 0 && is_vooc_curr_votable_available(vdd))
		vote(vdd->vooc_curr_votable, REC_VDD_VOTER, true, curr, false);

	return 0;
}

static unsigned int vdd_client_switch_protocol(struct state_keep_client *client,
	unsigned int total_count, unsigned int protocol_count,
	enum oplus_chg_protocol_type protocol, int power)
{
	struct vooc_disconnect_detection *vdd = client->priv_data;

	if (vdd == NULL) {
		chg_err("vdd is null\n");
		return (SK_SWITCH_SWITCH_PROTOCOL | SK_SWITCH_DISABLE_PROTOCOL);
	}
	if (protocol != CHG_PROTOCOL_VOOC) {
		vdd_state_reset(vdd);
		return 0;
	}

	return vdd_check_current_table(vdd, g_vooc_current_table,
			ARRAY_SIZE(g_vooc_current_table), protocol_count);
}

static void vdd_final_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct vooc_disconnect_detection *vdd =
		container_of(dwork, struct vooc_disconnect_detection, final_check_work);

	vdd_check_done(vdd, vdd->wired_online);
}

static void vdd_hw_detect_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct vooc_disconnect_detection *vdd =
		container_of(dwork, struct vooc_disconnect_detection, hw_detect_check_work);
	int hw_detect;
	unsigned int time;

	hw_detect = oplus_wired_get_hw_detect();
	chg_info("hw_detect=%d, check_count=%u\n", hw_detect, vdd->hw_detect_check_count);
	if (hw_detect != CC_DETECT_PLUGIN) {
		vdd_check_done(vdd, false);
		return;
	}

	vdd->hw_detect_check_count++;
	if (vdd->hw_detect_check_count < HW_DETECT_CHECK_COUNT && !vdd->wired_online)
		goto next;

	cancel_delayed_work_sync(&vdd->final_check_work);
	if (vdd->wired_online) {
		vdd_check_done(vdd, true);
	} else {
		time = DISCONNECT_TIME_THR_MS - (vdd->hw_detect_check_count - 1) * HW_DETECT_CHECK_TIME_MS;
		schedule_delayed_work(&vdd->final_check_work, msecs_to_jiffies(time));
	}

	return;

next:
	schedule_delayed_work(&vdd->hw_detect_check_work, msecs_to_jiffies(HW_DETECT_CHECK_TIME_MS));
}

static void vdd_wired_online_check_work(struct work_struct *work)
{
	struct vooc_disconnect_detection *vdd =
		container_of(work, struct vooc_disconnect_detection, wired_online_check_work);
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(vdd->wired_topic, WIRED_ITEM_ONLINE, &data, false);
	if (rc < 0) {
		chg_err("failed to get wired online, rc=%d\n", rc);
		return;
	}
	vdd->wired_online = !!data.intval;
	chg_info("wired online: %d\n", vdd->wired_online);

	if (vdd->wired_online) {
		if (delayed_work_pending(&vdd->hw_detect_check_work)) {
			cancel_delayed_work_sync(&vdd->hw_detect_check_work);
			vdd_check_done(vdd, true);
		} else if (delayed_work_pending(&vdd->final_check_work)) {
			cancel_delayed_work_sync(&vdd->final_check_work);
			vdd_check_done(vdd, true);
		}
	}
}

static void vdd_hw_detect_update_work(struct work_struct *work)
{
	struct vooc_disconnect_detection *vdd =
		container_of(work, struct vooc_disconnect_detection, hw_detect_update_work);
	union mms_msg_data data = { 0 };
	int hw_detect;
	int rc;

	rc = oplus_mms_get_item_data(vdd->wired_topic, WIRED_ITEM_CC_DETECT, &data, false);
	if (rc < 0) {
		chg_err("failed to get wired hw_detect, rc=%d\n", rc);
		return;
	}
	hw_detect = data.intval;
	chg_info("hw_detect: %d\n", hw_detect);

	if (hw_detect == CC_DETECT_PLUGIN)
		return;

	if (delayed_work_pending(&vdd->final_check_work)) {
		cancel_delayed_work_sync(&vdd->final_check_work);
		vdd_check_done(vdd, false);
	}
}

static void vdd_wired_subs_callback(struct mms_subscribe *subs,
				    enum mms_msg_type type, u32 id, bool sync)
{
	struct vooc_disconnect_detection *vdd = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			schedule_work(&vdd->wired_online_check_work);
			break;
		case WIRED_ITEM_CC_DETECT:
			schedule_work(&vdd->hw_detect_update_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void vdd_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct vooc_disconnect_detection *vdd = prv_data;

	vdd->wired_topic = topic;
	vdd->wired_subs =
		oplus_mms_subscribe(vdd->wired_topic, vdd,
				    vdd_wired_subs_callback,
				    "vooc_disconnect_detection");
	if (IS_ERR_OR_NULL(vdd->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(vdd->wired_subs));
		return;
	}
}

static struct state_keep_client_desc g_state_keep_client_desc = {
	.name = "vooc_disconnect_detection",
	.priority = STATE_KEEP_CLIENT_VOOC_DISCONNECT_DETECTION,
	.ops = {
		.reset = vdd_client_reset,
		.enable = vdd_client_enable,
		.disable = vdd_client_disable,
		.start_check = vdd_client_start_check,
		.need_keep = vdd_client_need_keep,
		.switch_protocol = vdd_client_switch_protocol,
	}
};

static int vdd_client_register(struct vooc_disconnect_detection *vdd)
{
	struct state_keep_client *client;

	client = state_keep_client_register(vdd->keep_topic, &g_state_keep_client_desc, vdd);
	if (IS_ERR_OR_NULL(client)) {
		chg_err("register vooc_disconnect_detection client failed\n");
		return -EINVAL;
	}
	vdd->client = client;
	state_keep_client_enable(client);

	return 0;
}

static void vdd_subscribe_state_keep_topic(struct oplus_mms *topic, void *prv_data)
{
	struct vooc_disconnect_detection *vdd = prv_data;

	vdd->keep_topic = topic;
	vdd_client_register(vdd);
}

int vooc_disconnect_detection_init(struct device_node *node)
{
	struct vooc_disconnect_detection *vdd;

	vdd = kzalloc(sizeof(struct vooc_disconnect_detection), GFP_KERNEL);
	if (vdd == NULL) {
		chg_err("failed to alloc memory for vdd");
		return -ENOMEM;
	}
	g_vdd = vdd;
	vdd->node = node;

	init_completion(&vdd->check_complete);
	vdd_state_reset(vdd);

	INIT_WORK(&vdd->wired_online_check_work, vdd_wired_online_check_work);
	INIT_WORK(&vdd->hw_detect_update_work, vdd_hw_detect_update_work);
	INIT_DELAYED_WORK(&vdd->hw_detect_check_work, vdd_hw_detect_check_work);
	INIT_DELAYED_WORK(&vdd->final_check_work, vdd_final_check_work);

	oplus_mms_wait_topic("state_keep", vdd_subscribe_state_keep_topic, vdd);
	oplus_mms_wait_topic("wired", vdd_subscribe_wired_topic, vdd);

	return 0;
}

void vooc_disconnect_monitor_exit(void)
{
	if (!g_vdd)
		return;

	cancel_delayed_work_sync(&g_vdd->final_check_work);
	cancel_delayed_work_sync(&g_vdd->hw_detect_check_work);

	if (!IS_ERR_OR_NULL(g_vdd->keep_subs))
		oplus_mms_unsubscribe(g_vdd->keep_subs);
	if (!IS_ERR_OR_NULL(g_vdd->wired_subs))
		oplus_mms_unsubscribe(g_vdd->wired_subs);
	if (g_vdd->client)
		state_keep_client_unregister(g_vdd->client);

	kfree(g_vdd);
	g_vdd = NULL;
}
