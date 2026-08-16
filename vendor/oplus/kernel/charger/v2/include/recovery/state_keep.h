/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#ifndef __OPLUS_STATUS_KEEP_H__
#define __OPLUS_STATUS_KEEP_H__

#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <oplus_mms.h>
#include <oplus_chg_cpa.h>

enum state_keep_topic_item {
	STATE_KEEP_ITEM_READY,
	STATE_KEEP_ITEM_WIRED_KEEP,
	STATE_KEEP_ITEM_WLS_KEEP,
	STATE_KEEP_ITEM_RESET,
	STATE_KEEP_ITEM_SWITCH_PROTOCOL,
	STATE_KEEP_ITEM_WIRED_ONLINE,
	STATE_KEEP_ITEM_WIRED_TYPE,
	STATE_KEEP_ITEM_BATT_STATUS,
	STATE_KEEP_ITEM_FAST_CHG_TYPE,
	STATE_KEEP_ITEM_CPA_POWER,
	STATE_KEEP_ITEM_UI_POWER,
	STATE_KEEP_ITEM_PLC_STATUS,
};

enum state_keep_status_type {
	STATE_KEEP_STATUS_FAST_CHG_TYPE = 0,
	STATE_KEEP_STATUS_CPA_POWER,
	STATE_KEEP_STATUS_UI_POWER,
	STATE_KEEP_STATUS_MAX,
};

/* priority list */
enum state_keep_client_priority {
	STATE_KEEP_CLIENT_VOOC_DISCONNECT_DETECTION = 0,
	STATE_KEEP_CLIENT_WIRED_DISCONNECT_DETECTION,
};

enum state_keep_switch_info {
	SK_SWITCH_SWITCH_PROTOCOL = BIT(0),
	SK_SWITCH_DISABLE_PROTOCOL = BIT(1),
};

typedef int (*state_keep_get_status_t) (void *priv_data);

struct state_keep_client {
	void *priv_data;
	const struct state_keep_client_desc *desc;
	struct state_keep *sk;
	struct dentry *debug_root;
	struct list_head list;
	bool enabled;
	bool start_error;
};

struct state_keep_client_ops {
	int (*reset)(struct state_keep_client *client);
	int (*enable)(struct state_keep_client *client);
	int (*disable)(struct state_keep_client *client);
	int (*start_check)(struct state_keep_client *client,
			   enum oplus_chg_protocol_type protocol);
	bool (*need_keep)(struct state_keep_client *client,
			  enum oplus_chg_protocol_type protocol);
	unsigned int (*switch_protocol)(struct state_keep_client *client,
					unsigned int total_count,
					unsigned int protocol_count,
					enum oplus_chg_protocol_type protocol,
					int power);
};

struct state_keep_client_desc {
	const char *name;
	unsigned int priority;
	struct state_keep_client_ops ops;
};

#if IS_ENABLED(CONFIG_OPLUS_CHG_STATE_KEEP)

struct state_keep_client *state_keep_client_register(
	struct oplus_mms *topic,
	struct state_keep_client_desc *desc,
	void *data);
void state_keep_client_unregister(struct state_keep_client *client);
int state_keep_client_enable(struct state_keep_client *client);
int state_keep_client_disable(struct state_keep_client *client);
bool state_keep_client_is_enabled(struct state_keep_client *client);
int state_keep_client_reset(struct state_keep_client *client);
int state_keep_status_info_register(
	struct oplus_mms *topic,
	enum state_keep_status_type type,
	state_keep_get_status_t func,
	void *priv_data);
void state_keep_status_info_unregister(
	struct oplus_mms *topic,
	enum state_keep_status_type type);

int state_keep_init(struct device *dev, struct device_node *node);
void state_keep_exit(struct device *dev);

#else

static inline struct state_keep_client *state_keep_client_register(
	struct oplus_mms *topic,
	struct state_keep_client_desc *desc,
	void *data)
{
	return NULL;
}

static inline void state_keep_client_unregister(struct state_keep_client *client)
{}

static inline int state_keep_client_enable(struct state_keep_client *client)
{
	return -EINVAL;
}

static inline int state_keep_client_disable(struct state_keep_client *client)
{
	return -EINVAL;
}

static inline bool state_keep_client_is_enabled(struct state_keep_client *client)
{
	return false;
}

static inline int state_keep_client_reset(struct state_keep_client *client)
{
	return -EINVAL;
}

static inline int state_keep_status_info_register(
	struct oplus_mms *topic,
	enum state_keep_status_type type,
	state_keep_get_status_t func,
	void *priv_data)
{
	return -EINVAL;
}

static inline void state_keep_status_info_unregister(
	struct oplus_mms *topic,
	enum state_keep_status_type type)
{}

static inline int state_keep_init(struct device *dev, struct device_node *node)
{
	return -EINVAL;
}

static inline void state_keep_exit(struct device *dev)
{}

#endif /* CONFIG_OPLUS_CHG_STATE_KEEP */

#endif /* __OPLUS_STATUS_KEEP_H__ */
