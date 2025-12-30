// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[TRACK/SK]([%s][%d]): " fmt, __func__, __LINE__

#include "oplus_track_state_keep.h"
#include <recovery/state_keep.h>
#include "../oplus_monitor_internal.h"

void oplus_monitor_keep_subs_callback(struct mms_subscribe *subs,
				      enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_monitor *chip = subs->priv_data;
	union mms_msg_data data = { 0 };
	int rc;

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case STATE_KEEP_ITEM_READY:
			rc = oplus_mms_get_item_data(chip->keep_topic, id, &data, false);
			if (rc < 0) {
				chg_err("get item data failed, rc=%d\n", rc);
				break;
			}
			if (!!data.intval)
				oplus_chg_track_update_break_ui_online();
			break;
		case STATE_KEEP_ITEM_SWITCH_PROTOCOL:
			chip->switch_protocol = true;
			break;
		case STATE_KEEP_ITEM_WIRED_ONLINE:
			rc = oplus_mms_get_item_data(chip->keep_topic, id, &data, false);
			if (rc < 0) {
				chg_err("get item data failed, rc=%d\n", rc);
				break;
			}
			chip->keep_wired_online = !!data.intval;
			if (!chip->keep_wired_online)
				chip->switch_protocol = false;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

void oplus_monitor_subscribe_keep_topic(struct oplus_mms *topic, void *prv_data)
{
	struct oplus_monitor *chip = prv_data;

	chip->keep_topic = topic;
	chip->keep_subs = oplus_mms_subscribe(chip->keep_topic, chip,
					     oplus_monitor_keep_subs_callback,
					     "monitor");
	if (IS_ERR_OR_NULL(chip->keep_subs)) {
		chg_err("subscribe state_keep topic error, rc=%ld\n",
			PTR_ERR(chip->keep_subs));
	}
}
