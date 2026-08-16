// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#ifndef __OPLUS_TRACK_STATE_KEEP_H__
#define __OPLUS_TRACK_STATE_KEEP_H__

#include <oplus_mms.h>

void oplus_monitor_keep_subs_callback(struct mms_subscribe *subs,
				      enum mms_msg_type type, u32 id, bool sync);
void oplus_monitor_subscribe_keep_topic(struct oplus_mms *topic, void *prv_data);

#endif /* __OPLUS_TRACK_STATE_KEEP_H__ */
