// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[REC/SK]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/sched/clock.h>

#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_wls.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_plc.h>
#include <oplus_chg_ufcs.h>
#include <recovery/oplus_chg_recovery.h>
#include <recovery/state_keep.h>
#include <oplus_chg_monitor.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/system/oplus_project.h>
#endif

#include "detection/wired_disconnect_detection.h"
#include "detection/vooc_disconnect_detection.h"

struct state_keep_data {
	bool wired_online;
	bool wls_online;
	int wired_type;
	int batt_status;
	int fast_chg_type;
	int cpa_power;
	int ui_power;
	enum plc_enable_status plc_status;

	bool first_check;
	bool recording;
	bool wired_keep;
	bool wls_keep;
	bool ready;
	bool disabled;
	unsigned int keep_count;

	enum oplus_chg_protocol_type current_protocol;
	enum oplus_chg_protocol_type pre_protocol;
	unsigned int current_protocol_count;
	int protocol_power_mw;
	int disconnect_time;

	bool batt_status_delay_update;
};

struct state_keep_status_info {
	void *priv_data;
	state_keep_get_status_t get_status;
	bool initialized;
};

struct abnormal_info {
	unsigned int wired_abnormal_count;
};

enum state_keep_track_type {
	STATE_KEEP_TRACK_GENERAL = 0,
	STATE_KEEP_TRACK_ABNORMAL,
	STATE_KEEP_TRACK_MAX,
};

struct state_keep_track_info {
	int upload_count[STATE_KEEP_TRACK_MAX];
	int upload_time[STATE_KEEP_TRACK_MAX];
};

struct state_keep {
	struct device_node *node;
	struct list_head client_list;
	struct mutex client_list_lock;
	struct dentry *debug_root;

	struct state_keep_data data;
	struct state_keep_status_info status_info[STATE_KEEP_STATUS_MAX];
	struct abnormal_info abnormal;
	struct state_keep_track_info track;

	struct oplus_mms *keep_topic;
	struct oplus_mms *comm_topic;
	struct mms_subscribe *comm_subs;
	struct oplus_mms *wired_topic;
	struct mms_subscribe *wired_subs;
	struct oplus_mms *wls_topic;
	struct mms_subscribe *wls_subs;
	struct oplus_mms *cpa_topic;
	struct mms_subscribe *cpa_subs;
	struct oplus_mms *plc_topic;
	struct mms_subscribe *plc_subs;
	struct oplus_mms *ufcs_topic;
	struct mms_subscribe *ufcs_subs;

	struct work_struct wired_online_update_work;
	struct work_struct wired_type_update_work;
	struct work_struct batt_status_update_work;
	struct work_struct protocol_type_update_work;
	struct work_struct power_change_work;
	struct work_struct plc_status_update_work;

	struct work_struct hw_detect_check_work;
	struct work_struct client_check_work;

	struct delayed_work abnormal_check_work;
	struct delayed_work batt_status_delay_update_work;

	struct wakeup_source *awake_lock;
	bool wakeup_flag;
};

#define USER_BUFFER_SIZE			128
#define TRACK_BUFFER_SIZE			4096
#define ABNORMAL_CHECK_INTERVAL_MS		1000
#define ABNORMAL_CHECK_COUNT_MAX		10
#define TRACK_UPLOAD_COUNT_MAX			3
#define TRACK_LOCAL_T_NS_TO_S_THD		1000000000
#define TRACK_UPLOAD_PERIOD			(24 * 3600)
#define PROTOCOL_COUNT_RESET_TIME_S		(6 * 60)
#define BATT_STATUS_DELAY_UPDATE_TIME_MS	1500
#define RESTORE_RECORDING_INTERVAL_MS		1000

struct item_show_info {
	enum state_keep_topic_item id;
	const char *name;
};

static struct item_show_info g_item_show_infos[] = {
	{ STATE_KEEP_ITEM_READY, "ready" },
	{ STATE_KEEP_ITEM_WIRED_KEEP, "wired_keep" },
	{ STATE_KEEP_ITEM_WLS_KEEP, "wls_keep" },
	{ STATE_KEEP_ITEM_RESET, "reset" },
	{ STATE_KEEP_ITEM_SWITCH_PROTOCOL, "switch_protocol" },
	{ STATE_KEEP_ITEM_WIRED_ONLINE, "wired_online" },
	{ STATE_KEEP_ITEM_WIRED_TYPE, "wired_type" },
	{ STATE_KEEP_ITEM_BATT_STATUS, "batt_status" },
	{ STATE_KEEP_ITEM_FAST_CHG_TYPE, "fast_chg_type" },
	{ STATE_KEEP_ITEM_CPA_POWER, "cpa_power" },
	{ STATE_KEEP_ITEM_UI_POWER, "ui_power" },
	{ STATE_KEEP_ITEM_PLC_STATUS, "plc_status" },
};

static void state_keep_status_info_reset(struct state_keep *sk)
{
	int i;

	for (i = 0; i < STATE_KEEP_STATUS_MAX; i++)
		sk->status_info[i].initialized = false;
}

static void state_keep_set_awake(struct state_keep *sk, bool awake)
{
	if (!sk->awake_lock)
		return;

	if (awake && !sk->wakeup_flag) {
		sk->wakeup_flag = true;
		__pm_stay_awake(sk->awake_lock);
	} else if (!awake && sk->wakeup_flag) {
		__pm_relax(sk->awake_lock);
		sk->wakeup_flag = false;
	}
}

static void state_keep_set_status_update(struct state_keep *sk)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  STATE_KEEP_ITEM_WIRED_ONLINE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
	} else {
		rc = oplus_mms_publish_msg(sk->keep_topic, msg);
		if (rc < 0) {
			chg_err("publish wired online msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  STATE_KEEP_ITEM_WIRED_TYPE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
	} else {
		rc = oplus_mms_publish_msg(sk->keep_topic, msg);
		if (rc < 0) {
			chg_err("publish wired type msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  STATE_KEEP_ITEM_BATT_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
	} else {
		rc = oplus_mms_publish_msg(sk->keep_topic, msg);
		if (rc < 0) {
			chg_err("publish battery status msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM,
				  STATE_KEEP_ITEM_PLC_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
	} else {
		rc = oplus_mms_publish_msg(sk->keep_topic, msg);
		if (rc < 0) {
			chg_err("publish plc status msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}
}

static void state_keep_set_ready(struct state_keep *sk, bool ready)
{
	struct mms_msg *msg;
	int rc;

	if (sk->data.ready == ready)
		return;
	sk->data.ready = ready;
	chg_info("ready=%d\n", ready);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  STATE_KEEP_ITEM_READY);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
	} else {
		rc = oplus_mms_publish_msg_sync(sk->keep_topic, msg);
		if (rc < 0) {
			chg_err("publish ready msg error, rc=%d\n", rc);
			kfree(msg);
		}
	}

	if (ready)
		state_keep_set_awake(sk, false);
}

static void state_keep_set_wired_keep(struct state_keep *sk, bool keep)
{
	struct mms_msg *msg;
	int rc;

	if (sk->data.wired_keep == keep)
		return;
	sk->data.wired_keep = keep;
	chg_info("wired_keep=%d\n", keep);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  STATE_KEEP_ITEM_WIRED_KEEP);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg_sync(sk->keep_topic, msg);
	if (rc < 0) {
		chg_err("publish wired keep msg error, rc=%d\n", rc);
		kfree(msg);
	}

	state_keep_set_status_update(sk);
}

static void state_keep_set_wls_keep(struct state_keep *sk, bool keep)
{
	struct mms_msg *msg;
	int rc;

	if (sk->data.wls_keep == keep)
		return;
	sk->data.wls_keep = keep;
	chg_info("wls_keep=%d\n", keep);

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				  STATE_KEEP_ITEM_WLS_KEEP);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg_sync(sk->keep_topic, msg);
	if (rc < 0) {
		chg_err("publish wls keep msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void state_keep_set_reset(struct state_keep *sk)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				      STATE_KEEP_ITEM_RESET, 1);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg_sync(sk->keep_topic, msg);
	if (rc < 0) {
		chg_err("publish reset msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void state_keep_set_switch_protocol(struct state_keep *sk)
{
	struct mms_msg *msg;
	int rc;

	msg = oplus_mms_alloc_int_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH,
				      STATE_KEEP_ITEM_SWITCH_PROTOCOL, 1);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg_sync(sk->keep_topic, msg);
	if (rc < 0) {
		chg_err("publish switch protocol msg error, rc=%d\n", rc);
		kfree(msg);
	} else {
		chg_info("publish switch protocol msg\n");
	}
}

static void state_keep_state_init(struct state_keep *sk)
{
	sk->data.keep_count = 0;
	sk->data.first_check = true;
	sk->data.recording = true;
	state_keep_status_info_reset(sk);
	sk->data.current_protocol = CHG_PROTOCOL_INVALID;
	sk->data.current_protocol_count = 0;
	sk->data.pre_protocol = CHG_PROTOCOL_INVALID;
	sk->data.protocol_power_mw = 0;
	sk->data.disconnect_time = 0;
	state_keep_set_wls_keep(sk, false);
	state_keep_set_wired_keep(sk, false);
	state_keep_set_ready(sk, true);
}

static void state_keep_reset(struct state_keep *sk)
{
	struct state_keep_client *client;

	chg_info("state_keep reset\n");
	state_keep_state_init(sk);
	state_keep_set_reset(sk);

	mutex_lock(&sk->client_list_lock);
	list_for_each_entry(client, &sk->client_list, list) {
		if (client->desc->ops.reset == NULL)
			continue;
		client->desc->ops.reset(client);
	}
	mutex_unlock(&sk->client_list_lock);

	sk->abnormal.wired_abnormal_count = 0;
}

static void state_keep_enable(struct state_keep *sk)
{
	if (!sk->data.disabled)
		return;

	if (sk->data.wired_online) {
		if (sk->data.first_check) {
			sk->data.recording = true;
			state_keep_status_info_reset(sk);
		}
		state_keep_set_ready(sk, false);
		state_keep_set_wls_keep(sk, false);
		state_keep_set_wired_keep(sk, true);
	}
	sk->data.disabled = false;
}

static void state_keep_disable(struct state_keep *sk)
{
	if (sk->data.disabled)
		return;
	sk->data.disabled = true;
	state_keep_state_init(sk);
}

static void state_keep_execution_switch_protocol(struct state_keep *sk,
	bool need_disable_protocol, bool need_switch_protocol)
{
	if (!need_switch_protocol) {
		oplus_cpa_request(sk->cpa_topic, sk->data.current_protocol);
		return;
	}

	if (sk->data.current_protocol != CHG_PROTOCOL_BC12) {
		if (need_disable_protocol) {
			if (sk->data.current_protocol == CHG_PROTOCOL_VOOC)
				oplus_cpa_request(sk->cpa_topic, CHG_PROTOCOL_QC);
			oplus_cpa_protocol_disable(sk->cpa_topic, sk->data.current_protocol);
			chg_info("disable %s protocol\n", get_protocol_name_str(sk->data.current_protocol));
		}
	} else {
		chg_info("restart state keep\n");
		oplus_cpa_protocol_enable_all(sk->cpa_topic);
		oplus_cpa_protocol_clear_power_all(sk->cpa_topic);
		oplus_cpa_protocol_restore_max_power_all(sk->cpa_topic);
	}
	sk->data.current_protocol_count = 0;
	sk->data.protocol_power_mw = 0;
	state_keep_set_switch_protocol(sk);
}

static void state_keep_check_protocol_switch(struct state_keep *sk, struct state_keep_client *client)
{
	int curr_time;
	int power_mw;
	unsigned int switch_info = 0;

	power_mw = sk->data.protocol_power_mw;
	curr_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	if (sk->data.pre_protocol != sk->data.current_protocol) {
		sk->data.current_protocol_count = 0;
		sk->data.protocol_power_mw = 0;
		sk->data.pre_protocol = sk->data.current_protocol;
	} else {
		if (sk->data.disconnect_time != 0 &&
		    (curr_time - sk->data.disconnect_time) > PROTOCOL_COUNT_RESET_TIME_S) {
			chg_info("%s: reset protocol count\n",
				 get_protocol_name_str(sk->data.current_protocol));
			sk->data.current_protocol_count = 0;
		}
	}
	sk->data.disconnect_time = curr_time;
	sk->data.keep_count++;
	sk->data.current_protocol_count++;

	if (client->desc->ops.switch_protocol != NULL) {
		switch_info = client->desc->ops.switch_protocol(
			client, sk->data.keep_count,
			sk->data.current_protocol_count,
			sk->data.current_protocol, power_mw);
	}
	chg_info("%s: keep_count=%u, protocol_count=%u, protocol=%s, power=%d, need_switch=%d, need_disable=%d\n",
		 client->desc->name, sk->data.keep_count, sk->data.current_protocol_count,
		 get_protocol_name_str(sk->data.current_protocol), power_mw,
		 !!(switch_info & SK_SWITCH_SWITCH_PROTOCOL),
		 !!(switch_info & SK_SWITCH_DISABLE_PROTOCOL));

	state_keep_execution_switch_protocol(sk,
		switch_info & SK_SWITCH_DISABLE_PROTOCOL,
		switch_info & SK_SWITCH_SWITCH_PROTOCOL);
}

static bool state_keep_track_check_limit(struct state_keep *sk, enum state_keep_track_type type)
{
	int curr_time;

	curr_time = local_clock() / TRACK_LOCAL_T_NS_TO_S_THD;
	if (curr_time - sk->track.upload_time[type] > TRACK_UPLOAD_PERIOD)
		sk->track.upload_count[type] = 0;
	if (sk->track.upload_count[type] >= TRACK_UPLOAD_COUNT_MAX)
		return true;

	sk->track.upload_time[type] = curr_time;
	sk->track.upload_count[type]++;
	return false;
}

const static int g_track_type_upload_map[STATE_KEEP_TRACK_MAX] = {
	[STATE_KEEP_TRACK_GENERAL] = ERR_ITEM_STATE_KEEP_INFO,
	[STATE_KEEP_TRACK_ABNORMAL] = ERR_ITEM_STATE_KEEP_ABNORMAL,
};

static void state_keep_track_upload(struct state_keep *sk,
				    enum state_keep_track_type type,
				    char *buf, int index)
{
	struct mms_msg *msg;
	struct oplus_mms *err_topic;
	struct item_show_info *info;
	union mms_msg_data data = { 0 };
	int i;
	int rc;

	if (index >= TRACK_BUFFER_SIZE - 1)
		return;
	if (state_keep_track_check_limit(sk, type))
		return;
	err_topic = oplus_mms_get_by_name("error");
	if (!err_topic)
		return;

	for (i = 0; i < ARRAY_SIZE(g_item_show_infos); i++) {
		info = &g_item_show_infos[i];
		oplus_mms_get_item_data(sk->keep_topic, info->id, &data, false);
		index += scnprintf(buf + index, TRACK_BUFFER_SIZE - index - 1, "$$%s@@%d",
			info->name, data.intval);
	}

	msg = oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_LOW,
				      g_track_type_upload_map[type], buf);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg_sync(err_topic, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void state_keep_upload_general_info(struct state_keep *sk)
{
	char *buf;
	int index;

	if (sk->data.keep_count == 0)
		return;

	buf = kzalloc(TRACK_BUFFER_SIZE, GFP_KERNEL);
	if (!buf) {
		chg_err("alloc buf error\n");
		return;
	}

	index = scnprintf(buf, TRACK_BUFFER_SIZE - 1, "$$count@@%d", sk->data.keep_count);
	state_keep_track_upload(sk, STATE_KEEP_TRACK_GENERAL, buf, index);
	kfree(buf);
}

static void state_keep_hw_detect_check_work(struct work_struct *work)
{
	struct state_keep *sk =
		container_of(work, struct state_keep, hw_detect_check_work);
	union mms_msg_data data = { 0 };
	int hw_detect;
	int rc;

	rc = oplus_mms_get_item_data(sk->wired_topic, WIRED_ITEM_CC_DETECT, &data, false);
	if (rc < 0) {
		chg_err("failed to get wired hw_detect, rc=%d\n", rc);
		return;
	}
	hw_detect = data.intval;
	chg_info("hw_detect: %d\n", hw_detect);
	if (hw_detect == CC_DETECT_PLUGIN)
		return;

	state_keep_set_awake(sk, false);
}

static void state_keep_client_check_work(struct work_struct *work)
{
	struct state_keep *sk =
		container_of(work, struct state_keep, client_check_work);
	struct state_keep_client *client;
	bool need_keep = false;

	mutex_lock(&sk->client_list_lock);
	list_for_each_entry(client, &sk->client_list, list) {
		if (!client->enabled)
			continue;
		if (client->start_error)
			continue;
		need_keep = client->desc->ops.need_keep(client, sk->data.current_protocol);
		chg_info("%s: %d\n", client->desc->name, need_keep);
		if (need_keep)
			break;
	}
	mutex_unlock(&sk->client_list_lock);
	if (!need_keep) {
		state_keep_upload_general_info(sk);
		state_keep_state_init(sk);
		return;
	}

	state_keep_check_protocol_switch(sk, client);
	state_keep_set_ready(sk, true);
}

static bool check_client_ops(struct state_keep_client_desc *desc)
{
	if (desc->ops.need_keep == NULL) {
		chg_err("%s: need_keep func is NULL\n", desc->name);
		return false;
	}
	if (desc->ops.start_check == NULL) {
		chg_err("%s: start_check func is NULL\n", desc->name);
		return false;
	}
	if (desc->ops.reset == NULL) {
		chg_err("%s: reset func is NULL\n", desc->name);
		return false;
	}
	if (desc->ops.enable == NULL) {
		chg_err("%s: enable func is NULL\n", desc->name);
		return false;
	}
	if (desc->ops.disable == NULL) {
		chg_err("%s: disable func is NULL\n", desc->name);
		return false;
	}

	return true;
}

static void state_keep_client_debugfs_init(struct state_keep *sk, struct state_keep_client *client)
{
	if (sk->debug_root == NULL)
		return;
	client->debug_root = debugfs_create_dir(client->desc->name, sk->debug_root);
	if (client->debug_root == NULL) {
		chg_err("%s: debugfs create dir failed\n", client->desc->name);
		return;
	}

	debugfs_create_bool("enabled", S_IFREG | 0444, client->debug_root, &client->enabled);
}

struct state_keep_client *state_keep_client_register(
	struct oplus_mms *topic,
	struct state_keep_client_desc *desc,
	void *data)
{
	struct state_keep *sk;
	struct state_keep_client *client, *tmp;
	bool found = false;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return NULL;
	}
	if (desc == NULL) {
		chg_err("desc is NULL");
		return NULL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return NULL;
	}
	sk = oplus_mms_get_drvdata(topic);

	client = kzalloc(sizeof(struct state_keep_client), GFP_KERNEL);
	if (client == NULL) {
		chg_err("alloc state_keep_client struct buffer error\n");
		return NULL;
	}

	if (!check_client_ops(desc))
		goto desc_check_err;
	client->desc = desc;
	client->priv_data = data;
	client->sk = sk;
	state_keep_client_debugfs_init(sk, client);

	mutex_lock(&sk->client_list_lock);
	list_for_each_entry(tmp, &sk->client_list, list) {
		if (client->desc->priority < tmp->desc->priority) {
			list_add_tail(&client->list, &tmp->list);
			found = true;
			break;
		}
	}
	if (!found)
		list_add_tail(&client->list, &sk->client_list);
	mutex_unlock(&sk->client_list_lock);

	return client;

desc_check_err:
	kfree(client);
	return NULL;
}

void state_keep_client_unregister(struct state_keep_client *client)
{
	struct state_keep *sk;

	if (client == NULL) {
		chg_err("client is NULL");
		return;
	}
	sk = client->sk;

	mutex_lock(&sk->client_list_lock);
	list_del(&client->list);
	mutex_unlock(&sk->client_list_lock);
	if (client->debug_root != NULL)
		debugfs_remove_recursive(client->debug_root);
	kfree(client);
}

int state_keep_client_enable(struct state_keep_client *client)
{
	if (client == NULL) {
		chg_err("client is NULL");
		return -EINVAL;
	}
	return client->desc->ops.enable(client);
}

int state_keep_client_disable(struct state_keep_client *client)
{
	if (client == NULL) {
		chg_err("client is NULL");
		return -EINVAL;
	}
	return client->desc->ops.disable(client);
}

bool state_keep_client_is_enabled(struct state_keep_client *client)
{
	if (client == NULL) {
		chg_err("client is NULL");
		return false;
	}
	return client->enabled;
}

int state_keep_client_reset(struct state_keep_client *client)
{
	if (client == NULL) {
		chg_err("client is NULL");
		return false;
	}
	return client->desc->ops.reset(client);
}

int state_keep_status_info_register(
	struct oplus_mms *topic,
	enum state_keep_status_type type,
	state_keep_get_status_t func,
	void *priv_data)
{
	struct state_keep *sk;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return -EINVAL;
	}
	if (func == NULL) {
		chg_err("func is NULL");
		return -EINVAL;
	}
	sk = oplus_mms_get_drvdata(topic);

	sk->status_info[type].initialized = false;
	sk->status_info[type].get_status = func;
	sk->status_info[type].priv_data = priv_data;

	return 0;
}

void state_keep_status_info_unregister(
	struct oplus_mms *topic,
	enum state_keep_status_type type)
{
	struct state_keep *sk;

	if (topic == NULL) {
		chg_err("topic is NULL");
		return;
	}
	sk = oplus_mms_get_drvdata(topic);

	sk->status_info[type].get_status = NULL;
	sk->status_info[type].priv_data = NULL;
}

static struct state_keep *state_keep_check_item_update_parameter(
	struct oplus_mms *mms, union mms_msg_data *data)
{
	if (mms == NULL) {
		chg_err("mms is NULL");
		return NULL;
	}
	if (data == NULL) {
		chg_err("data is NULL");
		return NULL;
	}

	return oplus_mms_get_drvdata(mms);
}

static int state_keep_update_ready(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct state_keep *sk;

	sk = state_keep_check_item_update_parameter(mms, data);
	if (sk == NULL) {
		chg_err("can't get state_keep struct info\n");
		return -EINVAL;
	}

	data->intval = sk->data.ready;

	return 0;
}

static int state_keep_update_wired_keep(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct state_keep *sk;

	sk = state_keep_check_item_update_parameter(mms, data);
	if (sk == NULL) {
		chg_err("can't get state_keep struct info\n");
		return -EINVAL;
	}

	data->intval = sk->data.wired_keep;

	return 0;
}

static int state_keep_update_wls_keep(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct state_keep *sk;

	sk = state_keep_check_item_update_parameter(mms, data);
	if (sk == NULL) {
		chg_err("can't get state_keep struct info\n");
		return -EINVAL;
	}

	data->intval = sk->data.wls_keep;

	return 0;
}

static int state_keep_update_wired_online(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct state_keep *sk;

	sk = state_keep_check_item_update_parameter(mms, data);
	if (sk == NULL) {
		chg_err("can't get state_keep struct info\n");
		return -EINVAL;
	}

	data->intval = sk->data.wired_online || sk->data.wired_keep;

	return 0;
}

static int state_keep_get_wired_type(struct state_keep *sk, bool update, int *wired_type)
{
	union mms_msg_data data = { 0 };
	bool recording;
	int rc;

	rc = oplus_mms_get_item_data(sk->wired_topic, WIRED_ITEM_CHG_TYPE, &data, update);
	if (rc < 0) {
		chg_err("get wired type error, rc=%d\n", rc);
		return rc;
	}
	chg_debug("recording=%d, wired_type=%d\n",
		  sk->data.recording, data.intval);

	recording = READ_ONCE(sk->data.recording);
	recording = recording || (sk->data.wired_type == OPLUS_CHG_USB_TYPE_UNKNOWN);

	if (!sk->data.disabled && sk->data.wired_keep) {
		if (recording) {
			if (data.intval == OPLUS_CHG_USB_TYPE_UNKNOWN)
				*wired_type = sk->data.wired_type;
			else
				*wired_type = data.intval;
		} else {
			*wired_type = sk->data.wired_type;
		}
	} else {
		*wired_type = data.intval;
	}
	sk->data.wired_type = *wired_type;

	return 0;
}

static int state_keep_update_wired_type(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct state_keep *sk;
	int wired_type;
	int rc;

	sk = state_keep_check_item_update_parameter(mms, data);
	if (sk == NULL) {
		chg_err("can't get state_keep struct info\n");
		return -EINVAL;
	}

	rc = state_keep_get_wired_type(sk, true, &wired_type);
	if (rc < 0)
		return rc;
	data->intval = wired_type;

	return 0;
}

static void state_keep_batt_status_delay_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct state_keep *sk =
		container_of(dwork, struct state_keep, batt_status_delay_update_work);

	sk->data.batt_status_delay_update = false;
	chg_info("batt_status_delay_update=false\n");
	schedule_work(&sk->batt_status_update_work);
}

static int state_keep_get_batt_status(struct state_keep *sk, bool update, int *batt_status)
{
	union mms_msg_data data = { 0 };
	int rc;
	bool batt_status_keep = false;

	rc = oplus_mms_get_item_data(sk->comm_topic, COMM_ITEM_BATT_STATUS, &data, update);
	if (rc < 0) {
		chg_err("get battery status error, rc=%d\n", rc);
		return rc;
	}
	chg_debug("recording=%d, batt_status=%d\n", sk->data.recording, data.intval);

	batt_status_keep = !sk->data.wired_online;
	batt_status_keep |= sk->data.batt_status_delay_update;

	if (!sk->data.disabled && sk->data.wired_keep && batt_status_keep) {
		if (READ_ONCE(sk->data.recording))
			*batt_status = data.intval;
		else
			*batt_status = sk->data.batt_status;
	} else {
		*batt_status = data.intval;
	}
	sk->data.batt_status = *batt_status;

	return 0;
}

static int state_keep_update_batt_status(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct state_keep *sk;
	int batt_status;
	int rc;

	sk = state_keep_check_item_update_parameter(mms, data);
	if (sk == NULL) {
		chg_err("can't get state_keep struct info\n");
		return -EINVAL;
	}

	rc = state_keep_get_batt_status(sk, true, &batt_status);
	if (rc < 0)
		return rc;
	data->intval = batt_status;

	return 0;
}

static bool state_keep_is_keep_status(struct state_keep *sk)
{
	bool is_keep_status = false;

	is_keep_status = sk->data.wired_keep || sk->data.wls_keep;
	is_keep_status &= !sk->data.disabled;
	return is_keep_status;
}

static bool state_keep_skip_recording(struct state_keep *sk, enum state_keep_status_type type)
{
	bool skip_recording = false;

	skip_recording = state_keep_is_keep_status(sk);
	skip_recording &= sk->status_info[type].initialized;
	skip_recording &= !sk->data.recording;
	return skip_recording;
}

static int state_keep_update_fast_chg_type(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct state_keep *sk;
	int fast_chg_type = CHARGER_SUBTYPE_DEFAULT;

	sk = state_keep_check_item_update_parameter(mms, data);
	if (sk == NULL) {
		chg_err("can't get state_keep struct info\n");
		return -EINVAL;
	}

	if (state_keep_skip_recording(sk, STATE_KEEP_STATUS_FAST_CHG_TYPE) &&
	    sk->data.fast_chg_type != CHARGER_SUBTYPE_DEFAULT)
		goto done;
	if (sk->status_info[STATE_KEEP_STATUS_FAST_CHG_TYPE].get_status == NULL)
		return -ENOTSUPP;
	fast_chg_type = sk->status_info[STATE_KEEP_STATUS_FAST_CHG_TYPE].get_status(
		sk->status_info[STATE_KEEP_STATUS_FAST_CHG_TYPE].priv_data);
	chg_info("recording=%d, fast_chg_type=%d\n", sk->data.recording, fast_chg_type);
	if (!state_keep_is_keep_status(sk) || fast_chg_type != CHARGER_SUBTYPE_DEFAULT ||
	    !sk->status_info[STATE_KEEP_STATUS_FAST_CHG_TYPE].initialized){
		sk->data.fast_chg_type = fast_chg_type;
		sk->status_info[STATE_KEEP_STATUS_FAST_CHG_TYPE].initialized = true;
	}
done:
	data->intval = sk->data.fast_chg_type;
	return 0;
}

static int state_keep_update_cpa_power(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct state_keep *sk;
	int cpa_power;

	sk = state_keep_check_item_update_parameter(mms, data);
	if (sk == NULL) {
		chg_err("can't get state_keep struct info\n");
		return -EINVAL;
	}
	if (state_keep_skip_recording(sk, STATE_KEEP_STATUS_CPA_POWER) &&
	    sk->data.cpa_power != 0)
		goto done;
	if (sk->status_info[STATE_KEEP_STATUS_CPA_POWER].get_status == NULL)
		return -ENOTSUPP;
	cpa_power = sk->status_info[STATE_KEEP_STATUS_CPA_POWER].get_status(
		sk->status_info[STATE_KEEP_STATUS_CPA_POWER].priv_data);
	chg_info("recording=%d, cpa_power=%d\n", sk->data.recording, sk->data.cpa_power);
	if (!state_keep_is_keep_status(sk) || cpa_power != 0 ||
	    !sk->status_info[STATE_KEEP_STATUS_CPA_POWER].initialized){
		sk->data.cpa_power = cpa_power;
		sk->status_info[STATE_KEEP_STATUS_CPA_POWER].initialized = true;
	}

done:
	data->intval = sk->data.cpa_power;
	return 0;
}

static int state_keep_update_ui_power(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct state_keep *sk;
	int ui_power;

	sk = state_keep_check_item_update_parameter(mms, data);
	if (sk == NULL) {
		chg_err("can't get state_keep struct info\n");
		return -EINVAL;
	}

	if (state_keep_skip_recording(sk, STATE_KEEP_STATUS_UI_POWER) &&
	    sk->data.ui_power != 0)
		goto done;

	if (sk->status_info[STATE_KEEP_STATUS_UI_POWER].get_status == NULL)
		return -ENOTSUPP;
	ui_power = sk->status_info[STATE_KEEP_STATUS_UI_POWER].get_status(
		sk->status_info[STATE_KEEP_STATUS_UI_POWER].priv_data);
	chg_debug("recording=%d, ui_power=%d\n", sk->data.recording, sk->data.ui_power);
	if (!state_keep_is_keep_status(sk) || ui_power != 0 ||
	    !sk->status_info[STATE_KEEP_STATUS_UI_POWER].initialized){
		sk->data.ui_power = ui_power;
		sk->status_info[STATE_KEEP_STATUS_UI_POWER].initialized = true;
	}

done:
	data->intval = sk->data.ui_power;
	return 0;
}

static int state_keep_get_plc_status(struct state_keep *sk, bool update, enum plc_enable_status *plc_status)
{
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(sk->plc_topic, PLC_ITEM_STATUS, &data, update);
	if (rc < 0) {
		chg_err("get plc status error, rc=%d\n", rc);
		return rc;
	}
	chg_debug("recording=%d, plc_status=%s\n", sk->data.recording,
		  plc_enable_status_str(data.intval));

	if (!sk->data.disabled && sk->data.wired_keep) {
		if (READ_ONCE(sk->data.recording))
			*plc_status = data.intval;
		else
			*plc_status = sk->data.plc_status;
	} else {
		*plc_status = data.intval;
	}

	return 0;
}

static int state_keep_update_plc_status(struct oplus_mms *mms, union mms_msg_data *data)
{
	struct state_keep *sk;
	enum plc_enable_status plc_status;
	int rc;

	sk = state_keep_check_item_update_parameter(mms, data);
	if (sk == NULL) {
		chg_err("can't get state_keep struct info\n");
		return -EINVAL;
	}

	rc = state_keep_get_plc_status(sk, true, &plc_status);
	if (rc < 0)
		return rc;
	data->intval = plc_status;

	return 0;
}

static void state_keep_topic_update(struct oplus_mms *mms, bool publish)
{
}

static struct mms_item state_keep_topic_item[] = {
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_READY,
			.update = state_keep_update_ready,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_WIRED_KEEP,
			.update = state_keep_update_wired_keep,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_WLS_KEEP,
			.update = state_keep_update_wls_keep,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_RESET,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_SWITCH_PROTOCOL,
			.update = NULL,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_WIRED_ONLINE,
			.update = state_keep_update_wired_online,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_WIRED_TYPE,
			.update = state_keep_update_wired_type,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_BATT_STATUS,
			.update = state_keep_update_batt_status,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_FAST_CHG_TYPE,
			.update = state_keep_update_fast_chg_type,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_CPA_POWER,
			.update = state_keep_update_cpa_power,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_UI_POWER,
			.update = state_keep_update_ui_power,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
	{
		.desc = {
			.item_id = STATE_KEEP_ITEM_PLC_STATUS,
			.update = state_keep_update_plc_status,
			.dead_thr_enable = true,
			.dead_zone_thr = 1,
		}
	},
};

static const struct oplus_mms_desc state_keep_topic_desc = {
	.name = "state_keep",
	.type = OPLUS_MMS_TYPE_STATE_KEEP,
	.item_table = state_keep_topic_item,
	.item_num = ARRAY_SIZE(state_keep_topic_item),
	.update_items = NULL,
	.update_items_num = 0,
	.update_interval = 0, /* ms */
	.update = state_keep_topic_update,
};

enum wired_abnormal_type {
	WIRED_ABNORMAL_NONE = 0,
	WIRED_ABNORMAL_ONLINE = 1,
	WIRED_ABNORMAL_TYPE,
	WIRED_ABNORMAL_BATT_STATUS,
	WIRED_ABNORMAL_FAST_CHG_TYPE,
	WIRED_ABNORMAL_CPA_POWER,
	WIRED_ABNORMAL_UI_POWER,
	WIRED_ABNORMAL_PLC_STATUS,
};

static const char *const wired_abnormal_type_str[] = {
	[WIRED_ABNORMAL_NONE] = "none",
	[WIRED_ABNORMAL_ONLINE] = "wired_online",
	[WIRED_ABNORMAL_TYPE] = "wired_type",
	[WIRED_ABNORMAL_BATT_STATUS] = "batt_status",
	[WIRED_ABNORMAL_FAST_CHG_TYPE] = "fast_chg_type",
	[WIRED_ABNORMAL_CPA_POWER] = "cpa_power",
	[WIRED_ABNORMAL_UI_POWER] = "ui_power",
	[WIRED_ABNORMAL_PLC_STATUS] = "plc_status",
};

static const char *show_wired_abnormal_type_str(unsigned int type)
{
	if (type >= ARRAY_SIZE(wired_abnormal_type_str))
		return "unknown";
	return wired_abnormal_type_str[type];
}

static unsigned int state_keep_wired_abnormal_check(struct state_keep *sk)
{
	int rc;
	union mms_msg_data data = { 0 };

	if (sk->data.wired_online)
		return WIRED_ABNORMAL_NONE;

	rc = oplus_mms_get_item_data(sk->keep_topic, STATE_KEEP_ITEM_WIRED_ONLINE, &data, false);
	if (rc >= 0 && !!data.intval)
		return WIRED_ABNORMAL_ONLINE;
	rc = oplus_mms_get_item_data(sk->keep_topic, STATE_KEEP_ITEM_WIRED_TYPE, &data, false);
	if (rc >= 0 && data.intval != OPLUS_CHG_USB_TYPE_UNKNOWN)
		return WIRED_ABNORMAL_TYPE;
	rc = oplus_mms_get_item_data(sk->keep_topic, STATE_KEEP_ITEM_BATT_STATUS, &data, false);
	if (rc >= 0 && data.intval == POWER_SUPPLY_STATUS_CHARGING)
		return WIRED_ABNORMAL_BATT_STATUS;
	rc = oplus_mms_get_item_data(sk->keep_topic, STATE_KEEP_ITEM_FAST_CHG_TYPE, &data, false);
	if (rc >= 0 && data.intval != 0)
		return WIRED_ABNORMAL_FAST_CHG_TYPE;
	rc = oplus_mms_get_item_data(sk->keep_topic, STATE_KEEP_ITEM_CPA_POWER, &data, false);
	if (rc >= 0 && data.intval != 0)
		return WIRED_ABNORMAL_CPA_POWER;
	rc = oplus_mms_get_item_data(sk->keep_topic, STATE_KEEP_ITEM_UI_POWER, &data, false);
	if (rc >= 0 && data.intval != 0)
		return WIRED_ABNORMAL_UI_POWER;
	rc = oplus_mms_get_item_data(sk->keep_topic, STATE_KEEP_ITEM_PLC_STATUS, &data, false);
	if (rc >= 0 && data.intval == PLC_STATUS_ENABLE)
		return WIRED_ABNORMAL_PLC_STATUS;

	return WIRED_ABNORMAL_NONE;
}

static void state_keep_upload_abnormal_info(struct state_keep *sk, enum wired_abnormal_type type)
{
	char *buf;
	int index;

	buf = kzalloc(TRACK_BUFFER_SIZE, GFP_KERNEL);
	if (!buf) {
		chg_err("alloc buf error\n");
		return;
	}

	index = scnprintf(buf, TRACK_BUFFER_SIZE - 1, "$$type@@%d", type);
	state_keep_track_upload(sk, STATE_KEEP_TRACK_ABNORMAL, buf, index);
	kfree(buf);
}

static void state_keep_abnormal_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct state_keep *sk =
		container_of(dwork, struct state_keep, abnormal_check_work);
	bool abnormal = false;
	unsigned int wired_abnormal_type = WIRED_ABNORMAL_NONE;

	wired_abnormal_type = state_keep_wired_abnormal_check(sk);
	if (wired_abnormal_type != WIRED_ABNORMAL_NONE) {
		abnormal = true;
		sk->abnormal.wired_abnormal_count++;
	}
	if (!abnormal) {
		sk->abnormal.wired_abnormal_count = 0;
		return;
	}

	if (sk->abnormal.wired_abnormal_count < ABNORMAL_CHECK_COUNT_MAX) {
		schedule_delayed_work(&sk->abnormal_check_work,
			msecs_to_jiffies(ABNORMAL_CHECK_INTERVAL_MS));
		return;
	}

	chg_err("wired abnormal: %s\n", show_wired_abnormal_type_str(wired_abnormal_type));
	state_keep_upload_abnormal_info(sk, wired_abnormal_type);
	state_keep_reset(sk);
}

static void state_keep_client_start_check(struct state_keep *sk)
{
	struct state_keep_client *client;
	int rc;

	mutex_lock(&sk->client_list_lock);
	list_for_each_entry(client, &sk->client_list, list) {
		if (!client->enabled)
			continue;
		chg_info("%s: start check\n", client->desc->name);
		rc = client->desc->ops.start_check(client, sk->data.current_protocol);
		if (rc < 0) {
			chg_err("%s: start check failed, rc=%d\n", client->desc->name, rc);
			client->start_error = true;
		} else {
			client->start_error = false;
		}
	}
	mutex_unlock(&sk->client_list_lock);
}

static void state_keep_wired_online_update_work(struct work_struct *work)
{
	struct state_keep *sk =
		container_of(work, struct state_keep, wired_online_update_work);
	struct mms_msg *msg;
	bool wired_online;
	int rc;

	wired_online = sk->data.wired_online;
	cancel_work_sync(&sk->client_check_work);
	cancel_delayed_work_sync(&sk->batt_status_delay_update_work);
	if (!sk->data.disabled) {
		if (sk->data.wired_online) {
			state_keep_set_ready(sk, false);
			state_keep_set_wls_keep(sk, false);
			state_keep_set_wired_keep(sk, true);
			sk->data.batt_status_delay_update = true;
			schedule_delayed_work(&sk->batt_status_delay_update_work,
				msecs_to_jiffies(BATT_STATUS_DELAY_UPDATE_TIME_MS));
		} else {
			state_keep_set_awake(sk, true);
			state_keep_client_start_check(sk);
			schedule_work(&sk->client_check_work);
			sk->data.batt_status_delay_update = false;
		}
	}

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, STATE_KEEP_ITEM_WIRED_ONLINE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(sk->keep_topic, msg);
	if (rc < 0) {
		chg_err("publish wired online msg error, rc=%d\n", rc);
		kfree(msg);
	}

	if (READ_ONCE(sk->data.wired_online) != wired_online) {
		chg_info("wired online change to %d\n", !wired_online);
		schedule_work(&sk->wired_online_update_work);
	}
}

static void state_keep_wired_type_update_work(struct work_struct *work)
{
	struct state_keep *sk =
		container_of(work, struct state_keep, wired_type_update_work);
	int wired_type;
	struct mms_msg *msg;
	int rc;

	rc = state_keep_get_wired_type(sk, false, &wired_type);
	if (rc < 0)
		return;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, STATE_KEEP_ITEM_WIRED_TYPE);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(sk->keep_topic, msg);
	if (rc < 0) {
		chg_err("publish wired type msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void state_keep_batt_status_update_work(struct work_struct *work)
{
	struct state_keep *sk =
		container_of(work, struct state_keep, batt_status_update_work);
	int batt_status;
	struct mms_msg *msg;
	int rc;

	rc = state_keep_get_batt_status(sk, false, &batt_status);
	if (rc < 0)
		return;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, STATE_KEEP_ITEM_BATT_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(sk->keep_topic, msg);
	if (rc < 0) {
		chg_err("publish battery status msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void state_keep_common_subs_callback(struct mms_subscribe *subs,
					    enum mms_msg_type type, u32 id, bool sync)
{
	struct state_keep *sk = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case COMM_ITEM_BATT_STATUS:
			schedule_work(&sk->batt_status_update_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void state_keep_subscribe_common_topic(struct oplus_mms *topic, void *prv_data)
{
	struct state_keep *sk = prv_data;
	union mms_msg_data data = { 0 };

	sk->comm_topic = topic;
	sk->comm_subs =
		oplus_mms_subscribe(sk->comm_topic, sk,
				    state_keep_common_subs_callback,
				    "state_keep");
	if (IS_ERR_OR_NULL(sk->comm_subs)) {
		chg_err("subscribe common topic error, rc=%ld\n",
			PTR_ERR(sk->wired_subs));
		return;
	}

	(void)oplus_mms_get_item_data(sk->comm_topic, COMM_ITEM_BATT_STATUS, &data, true);
	schedule_work(&sk->batt_status_update_work);
}

static void state_keep_wired_subs_callback(struct mms_subscribe *subs,
					   enum mms_msg_type type, u32 id, bool sync)
{
	struct state_keep *sk = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_ONLINE:
			rc = oplus_mms_get_item_data(sk->wired_topic, id, &data, false);
			if (rc < 0) {
				chg_err("get wired online failed, rc=%d\n", rc);
				break;
			}
			sk->data.wired_online = !!data.intval;
			chg_info("wired online: %d\n", sk->data.wired_online);
			if (!sk->data.disabled) {
				if (sk->data.wired_online && sk->data.first_check) {
					sk->data.recording = true;
					state_keep_status_info_reset(sk);
				} else if (!sk->data.wired_online) {
					sk->data.recording = false;
					sk->data.first_check = false;
					if (sk->data.wls_online)
						state_keep_status_info_reset(sk);
					schedule_delayed_work(&sk->abnormal_check_work,
						msecs_to_jiffies(ABNORMAL_CHECK_INTERVAL_MS));
				}
			}
			schedule_work(&sk->wired_online_update_work);
			break;
		case WIRED_ITEM_CHG_TYPE:
			schedule_work(&sk->wired_type_update_work);
			break;
		case WIRED_ITEM_CC_DETECT:
			schedule_work(&sk->hw_detect_check_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void state_keep_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct state_keep *sk = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	sk->wired_topic = topic;
	sk->wired_subs =
		oplus_mms_subscribe(sk->wired_topic, sk,
				    state_keep_wired_subs_callback,
				    "state_keep");
	if (IS_ERR_OR_NULL(sk->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(sk->wired_subs));
		return;
	}
	/*
	 * This must be the first to receive the wired_online notification to
	 * ensure that the status record is correct
	 */
	(void)oplus_mms_subs_move_to_top(sk->wired_subs);

	rc = oplus_mms_get_item_data(sk->wired_topic, WIRED_ITEM_ONLINE, &data, true);
	if (rc < 0)
		chg_err("get wired online error, rc=%d\n", rc);
	else
		sk->data.wired_online = !!data.intval;
	if (sk->data.wired_online) {
		sk->data.recording = true;
		state_keep_status_info_reset(sk);
		schedule_work(&sk->wired_online_update_work);
	}
	(void)oplus_mms_get_item_data(sk->wired_topic, WIRED_ITEM_CHG_TYPE, &data, true);
	schedule_work(&sk->wired_type_update_work);
}

static void state_keep_wls_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct state_keep *sk = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WLS_ITEM_ONLINE:
			oplus_mms_get_item_data(sk->wls_topic, id, &data, false);
			sk->data.wls_online = !!data.intval;
			chg_info("wireless online: %d\n", sk->data.wls_online);
			// TODO: Support for wireless intermittent detection needs to be fixed.
			state_keep_status_info_reset(sk);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void state_keep_subscribe_wls_topic(struct oplus_mms *topic, void *prv_data)
{
	struct state_keep *sk = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	sk->wls_topic = topic;
	sk->wls_subs =
		oplus_mms_subscribe(sk->wls_topic, sk,
				    state_keep_wls_subs_callback,
				    "state_keep");
	if (IS_ERR_OR_NULL(sk->wls_subs)) {
		chg_err("subscribe wireless topic error, rc=%ld\n",
			PTR_ERR(sk->wls_subs));
		return;
	}

	rc = oplus_mms_get_item_data(sk->wls_topic, WLS_ITEM_ONLINE, &data, true);
	if (rc < 0)
		chg_err("get wireless online error, rc=%d\n", rc);
	else
		sk->data.wls_online = !!data.intval;
}

static void state_keep_protocol_type_update_work(struct work_struct *work)
{
	struct state_keep *sk =
		container_of(work, struct state_keep, protocol_type_update_work);
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(sk->cpa_topic, CPA_ITEM_ALLOW, &data, false);
	if (rc < 0) {
		chg_err("get cpa item error, rc=%d\n", rc);
		return;
	}

	if (data.intval == CHG_PROTOCOL_INVALID) {
		if (sk->data.current_protocol == CHG_PROTOCOL_INVALID) {
			sk->data.current_protocol_count = 0;
			sk->data.protocol_power_mw = 0;
		} else if (sk->data.current_protocol == CHG_PROTOCOL_VOOC) {
			if (sk->data.wired_online)
				oplus_cpa_request(sk->cpa_topic, CHG_PROTOCOL_QC);
		}
	} else {
		sk->data.current_protocol = data.intval;
	}

	chg_debug("protocol: %s\n", get_protocol_name_str(sk->data.current_protocol));
}

static void state_keep_power_change_work(struct work_struct *work)
{
	struct state_keep *sk =
		container_of(work, struct state_keep, power_change_work);
	union mms_msg_data data = { 0 };
	int rc;

	rc = oplus_mms_get_item_data(sk->cpa_topic, CPA_ITEM_POWER, &data, false);
	if (rc < 0) {
		chg_err("get cpa item error, rc=%d\n", rc);
		return;
	}

	if (data.intval > sk->data.protocol_power_mw)
		sk->data.protocol_power_mw = data.intval;
	chg_debug("%s: %dmW\n", get_protocol_name_str(sk->data.current_protocol),
		  sk->data.protocol_power_mw);

	/*
	 * Actively update the state to prevent the HAL layer from not
	 * obtaining the state in a timely manner.
	 */
	(void)oplus_mms_get_item_data(sk->keep_topic, STATE_KEEP_ITEM_FAST_CHG_TYPE, &data, true);
	(void)oplus_mms_get_item_data(sk->keep_topic, STATE_KEEP_ITEM_CPA_POWER, &data, true);
	(void)oplus_mms_get_item_data(sk->keep_topic, STATE_KEEP_ITEM_UI_POWER, &data, true);
}

static void state_keep_cpa_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct state_keep *sk = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case CPA_ITEM_ALLOW:
			schedule_work(&sk->protocol_type_update_work);
			break;
		case CPA_ITEM_POWER:
			schedule_work(&sk->power_change_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void state_keep_subscribe_cpa_topic(struct oplus_mms *topic, void *prv_data)
{
	struct state_keep *sk = prv_data;

	sk->cpa_topic = topic;
	sk->cpa_subs =
		oplus_mms_subscribe(sk->cpa_topic, sk,
				    state_keep_cpa_subs_callback,
				    "state_keep");
	if (IS_ERR_OR_NULL(sk->cpa_subs)) {
		chg_err("subscribe cpa topic error, rc=%ld\n",
			PTR_ERR(sk->cpa_subs));
		return;
	}

	schedule_work(&sk->protocol_type_update_work);
	schedule_work(&sk->power_change_work);
}

static void state_keep_plc_status_update_work(struct work_struct *work)
{
	struct state_keep *sk =
		container_of(work, struct state_keep, plc_status_update_work);
	enum plc_enable_status plc_status;
	struct mms_msg *msg;
	int rc;

	rc = state_keep_get_plc_status(sk, false, &plc_status);
	if (rc < 0)
		return;
	sk->data.plc_status = plc_status;

	msg = oplus_mms_alloc_msg(MSG_TYPE_ITEM, MSG_PRIO_MEDIUM, STATE_KEEP_ITEM_PLC_STATUS);
	if (msg == NULL) {
		chg_err("alloc msg error\n");
		return;
	}
	rc = oplus_mms_publish_msg(sk->keep_topic, msg);
	if (rc < 0) {
		chg_err("publish plc status msg error, rc=%d\n", rc);
		kfree(msg);
	}
}

static void state_keep_plc_subs_callback(struct mms_subscribe *subs,
					 enum mms_msg_type type, u32 id, bool sync)
{
	struct state_keep *sk = subs->priv_data;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PLC_ITEM_STATUS:
			schedule_work(&sk->plc_status_update_work);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void state_keep_subscribe_plc_topic(struct oplus_mms *topic, void *prv_data)
{
	struct state_keep *sk = prv_data;

	sk->plc_topic = topic;
	sk->plc_subs =
		oplus_mms_subscribe(sk->plc_topic, sk,
				    state_keep_plc_subs_callback,
				    "state_keep");
	if (IS_ERR_OR_NULL(sk->plc_subs)) {
		chg_err("subscribe plc topic error, rc=%ld\n",
			PTR_ERR(sk->plc_subs));
		return;
	}

	schedule_work(&sk->plc_status_update_work);
}

static void state_keep_ufcs_subs_callback(struct mms_subscribe *subs,
					  enum mms_msg_type type, u32 id, bool sync)
{
	struct state_keep *sk = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case PLC_ITEM_STATUS:
			if (!sync)
				break;
			rc = oplus_mms_get_item_data(sk->ufcs_topic, UFCS_ITEM_TEST_MODE, &data, false);
			if (rc < 0) {
				chg_err("get ufcs test mode error, rc=%d\n", rc);
				break;
			}
			if (!!data.intval)
				state_keep_disable(sk);
			else
				state_keep_enable(sk);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void state_keep_subscribe_ufcs_topic(struct oplus_mms *topic, void *prv_data)
{
	struct state_keep *sk = prv_data;
	union mms_msg_data data = { 0 };
	int rc;

	sk->ufcs_topic = topic;
	sk->ufcs_subs =
		oplus_mms_subscribe(sk->ufcs_topic, sk,
				    state_keep_ufcs_subs_callback,
				    "state_keep");
	if (IS_ERR_OR_NULL(sk->ufcs_subs)) {
		chg_err("subscribe plc topic error, rc=%ld\n",
			PTR_ERR(sk->ufcs_subs));
		return;
	}

	rc = oplus_mms_get_item_data(sk->ufcs_topic, UFCS_ITEM_TEST_MODE, &data, true);
	if (rc < 0) {
		chg_err("get ufcs test mode error, rc=%d\n", rc);
		return;
	}
	if (!!data.intval)
		state_keep_disable(sk);
}

static int state_keep_topic_init(struct device *dev, struct state_keep *sk)
{
	struct oplus_mms_config mms_cfg = {};

	mms_cfg.drv_data = sk;
	mms_cfg.of_node = NULL;

	sk->keep_topic = devm_oplus_mms_register(dev, &state_keep_topic_desc, &mms_cfg);
	if (IS_ERR(sk->keep_topic)) {
		chg_err("can't register state_keep topic\n");
		return PTR_ERR(sk->keep_topic);
	}

	return 0;
}

static void state_keep_init_detection_algorithm(struct state_keep *sk)
{
	struct device_node *node;
	int rc;

	node = of_find_node_by_name(sk->node, "wired_disconnect_detection");
	if (node != NULL) {
		rc = wired_disconnect_detection_init(node);
		if (rc < 0)
			chg_err("wired_disconnect_detection init failed, rc=%d\n", rc);
	}

	node = of_find_node_by_name(sk->node, "vooc_disconnect_detection");
	if (node != NULL) {
		rc = vooc_disconnect_detection_init(node);
		if (rc < 0)
			chg_err("vooc_disconnect_detection init failed, rc=%d\n", rc);
	}
}

static void state_keep_exit_detection_algorithm(struct state_keep *sk)
{
	if (of_find_node_by_name(sk->node, "wired_disconnect_detection") != NULL)
		wired_disconnect_monitor_exit();
	if (of_find_node_by_name(sk->node, "vooc_disconnect_detection") != NULL)
		vooc_disconnect_monitor_exit();
}

static int state_keep_status_show(struct seq_file *m, void *data)
{
	struct state_keep *sk = m->private;
	struct state_keep_client *client;
	struct item_show_info *info;
	union mms_msg_data item_data = { 0 };
	int i;

	seq_printf(m, "Clients:\n");
	mutex_lock(&sk->client_list_lock);
	list_for_each_entry(client, &sk->client_list, list) {
		seq_printf(m, "    %s: ", client->desc->name);
		if (client->enabled)
			seq_printf(m, "enabled\n");
		else
			seq_printf(m, "disabled\n");
	}
	mutex_unlock(&sk->client_list_lock);

	seq_printf(m, "Status:\n");
	seq_printf(m, "    first_check: %d\n", sk->data.first_check);
	seq_printf(m, "    recording: %d\n", sk->data.recording);
	seq_printf(m, "    keep_count: %d\n", sk->data.keep_count);
	seq_printf(m, "    current_protocol: %s\n", get_protocol_name_str(sk->data.current_protocol));
	seq_printf(m, "    pre_protocol: %s\n", get_protocol_name_str(sk->data.pre_protocol));
	seq_printf(m, "    current_protocol_count: %d\n", sk->data.current_protocol_count);
	seq_printf(m, "    protocol_power_mw: %d\n", sk->data.protocol_power_mw);
	seq_printf(m, "    disabled: %d\n", sk->data.disabled);

	seq_printf(m, "Item:\n");
	for (i = 0; i < ARRAY_SIZE(g_item_show_infos); i++) {
		info = &g_item_show_infos[i];
		oplus_mms_get_item_data(sk->keep_topic, info->id, &item_data, false);
		seq_printf(m, "    %s: %d\n", info->name, item_data.intval);
	}

	return 0;
}

static int state_keep_status_open(struct inode *inode, struct file *file)
{
	struct state_keep *sk = inode->i_private;

	return single_open(file, state_keep_status_show, sk);
}

static const struct file_operations state_keep_status_fops = {
	.owner		= THIS_MODULE,
	.open		= state_keep_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int state_keep_client_enable_by_name(struct state_keep *sk, const char *name, bool enable)
{
	struct state_keep_client *client;
	bool found = false;

	if (strncmp(name, "all", strlen("all")) == 0) {
		chg_info("%s all clients\n", enable ? "enable" : "disable");
		mutex_lock(&sk->client_list_lock);
		list_for_each_entry(client, &sk->client_list, list) {
			if (enable)
				client->desc->ops.enable(client);
			else
				client->desc->ops.disable(client);
		}
		mutex_unlock(&sk->client_list_lock);
		return 0;
	}

	mutex_lock(&sk->client_list_lock);
	list_for_each_entry(client, &sk->client_list, list) {
		if (strncmp(name, client->desc->name, strlen(client->desc->name)) == 0) {
			chg_info("%s %s client\n", client->desc->name, enable ? "enable" : "disable");
			if (enable)
				client->desc->ops.enable(client);
			else
				client->desc->ops.disable(client);
			found = true;
			break;
		}
	}
	mutex_unlock(&sk->client_list_lock);
	if (!found) {
		chg_err("client %s not found\n", name);
		return -EINVAL;
	}

	return 0;
}

static ssize_t state_keep_enable_write(struct file *file,
	const char __user *buf, size_t count, loff_t *lo)
{
	struct state_keep *sk = file->private_data;
	char buffer[USER_BUFFER_SIZE] = { 0 };
	int rc;

	if (count >= USER_BUFFER_SIZE) {
		chg_err("data is too big\n");
		return -EINVAL;
	}
	rc = copy_from_user(buffer, buf, count);
	if (rc < 0) {
		chg_err("copy data error\n");
		return rc;
	}

	rc = state_keep_client_enable_by_name(sk, buffer, true);
	if (rc < 0)
		return rc;

	return count;
}

static const struct file_operations state_keep_enable_fops =
{
	.owner = THIS_MODULE,
	.open  = simple_open,
	.write  = state_keep_enable_write,
	.llseek = noop_llseek,
};

static ssize_t state_keep_disable_write(struct file *file,
	const char __user *buf, size_t count, loff_t *lo)
{
	struct state_keep *sk = file->private_data;
	char buffer[USER_BUFFER_SIZE] = { 0 };
	int rc;

	if (count >= USER_BUFFER_SIZE) {
		chg_err("data is too big\n");
		return -EINVAL;
	}
	rc = copy_from_user(buffer, buf, count);
	if (rc < 0) {
		chg_err("copy data error\n");
		return rc;
	}

	rc = state_keep_client_enable_by_name(sk, buffer, false);
	if (rc < 0)
		return rc;

	return count;
}

static const struct file_operations state_keep_disable_fops =
{
	.owner = THIS_MODULE,
	.open  = simple_open,
	.write  = state_keep_disable_write,
	.llseek = noop_llseek,
};

static void state_keep_debugfs_init(struct state_keep *sk)
{
	struct dentry *root;
	struct dentry *entry;

	root = oplus_recovery_get_debug_root();
	if (root == NULL)
		return;
	sk->debug_root = debugfs_create_dir("state_keep", root);
	if (sk->debug_root == NULL) {
		chg_err("debugfs create state_keep dir failed\n");
		return;
	}

	entry = debugfs_create_file("status", S_IFREG | 0444,
		sk->debug_root, sk, &state_keep_status_fops);
	if (entry == NULL)
		chg_err("debugfs create status file failed\n");

	entry = debugfs_create_file("enable", S_IFREG | 0200,
		sk->debug_root, sk, &state_keep_enable_fops);
	if (entry == NULL)
		chg_err("debugfs create enable file failed\n");

	entry = debugfs_create_file("disable", S_IFREG | 0200,
		sk->debug_root, sk, &state_keep_disable_fops);
	if (entry == NULL)
		chg_err("debugfs create disable file failed\n");
}

int state_keep_init(struct device *dev, struct device_node *node)
{
	struct state_keep *sk;
	int rc;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == FACTORY) {
		chg_info("factory mode not support state_keep\n");
		return 0;
	}
#endif

	sk = devm_kzalloc(dev, sizeof(struct state_keep), GFP_KERNEL);
	if (sk == NULL) {
		chg_err("alloc state_keep buffer error\n");
		return -ENOMEM;
	}
	sk->node = node;

	rc = state_keep_topic_init(dev, sk);
	if (rc) {
		chg_err("state_keep_topic_init failed, rc=%d\n", rc);
		goto topic_init_err;
	}

	sk->awake_lock = wakeup_source_register(NULL, "charger state keep");
	sk->wakeup_flag = false;

	state_keep_state_init(sk);
	INIT_LIST_HEAD(&sk->client_list);
	mutex_init(&sk->client_list_lock);

	INIT_WORK(&sk->hw_detect_check_work, state_keep_hw_detect_check_work);
	INIT_WORK(&sk->client_check_work, state_keep_client_check_work);
	INIT_WORK(&sk->wired_online_update_work, state_keep_wired_online_update_work);
	INIT_WORK(&sk->wired_type_update_work, state_keep_wired_type_update_work);
	INIT_WORK(&sk->batt_status_update_work, state_keep_batt_status_update_work);
	INIT_WORK(&sk->protocol_type_update_work, state_keep_protocol_type_update_work);
	INIT_WORK(&sk->power_change_work, state_keep_power_change_work);
	INIT_WORK(&sk->plc_status_update_work, state_keep_plc_status_update_work);

	INIT_DELAYED_WORK(&sk->abnormal_check_work, state_keep_abnormal_check_work);
	INIT_DELAYED_WORK(&sk->batt_status_delay_update_work, state_keep_batt_status_delay_update_work);

	state_keep_debugfs_init(sk);
	state_keep_init_detection_algorithm(sk);

	oplus_mms_wait_topic("common", state_keep_subscribe_common_topic, sk);
	oplus_mms_wait_topic("wired", state_keep_subscribe_wired_topic, sk);
	oplus_mms_wait_topic("wireless", state_keep_subscribe_wls_topic, sk);
	oplus_mms_wait_topic("cpa", state_keep_subscribe_cpa_topic, sk);
	oplus_mms_wait_topic("plc", state_keep_subscribe_plc_topic, sk);
	oplus_mms_wait_topic("ufcs", state_keep_subscribe_ufcs_topic, sk);

	return 0;

topic_init_err:
	devm_kfree(dev, sk);
	return rc;
}

void state_keep_exit(struct device *dev)
{
	struct state_keep *sk;
	struct oplus_mms *topic;

#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	if (get_eng_version() == FACTORY) {
		chg_info("factory mode not support state_keep\n");
		return;
	}
#endif

	topic = oplus_mms_get_by_name("state_keep");
	if (!topic)
		return;
	sk = oplus_mms_get_drvdata(topic);
	if (!sk)
		return;

	cancel_delayed_work_sync(&sk->abnormal_check_work);
	cancel_delayed_work_sync(&sk->batt_status_delay_update_work);

	if (!IS_ERR_OR_NULL(sk->wired_subs))
		oplus_mms_unsubscribe(sk->wired_subs);
	if (!IS_ERR_OR_NULL(sk->wls_subs))
		oplus_mms_unsubscribe(sk->wls_subs);
	if (!IS_ERR_OR_NULL(sk->comm_subs))
		oplus_mms_unsubscribe(sk->comm_subs);
	if (!IS_ERR_OR_NULL(sk->cpa_subs))
		oplus_mms_unsubscribe(sk->cpa_subs);
	if (!IS_ERR_OR_NULL(sk->plc_subs))
		oplus_mms_unsubscribe(sk->plc_subs);
	if (!IS_ERR_OR_NULL(sk->ufcs_subs))
		oplus_mms_unsubscribe(sk->ufcs_subs);

	if (sk->debug_root != NULL)
		debugfs_remove_recursive(sk->debug_root);
	wakeup_source_unregister(sk->awake_lock);

	state_keep_exit_detection_algorithm(sk);

	devm_kfree(dev, sk);
}
