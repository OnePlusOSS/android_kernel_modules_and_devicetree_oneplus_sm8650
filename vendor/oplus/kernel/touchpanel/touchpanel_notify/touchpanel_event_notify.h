// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _TOUCHPANEL_EVENTNOTIFY_H
#define _TOUCHPANEL_EVENTNOTIFY_H

#define EVENT_ACTION_FOR_FINGPRINT 0x01
#define EVENT_ACTION_FOR_FILM      0x02
#define EVENT_ACTION_FOR_FP_GIRP   0x03
#define EVENT_ACTION_UNDER_WATER   0x04

struct touchpanel_event {
	int touchpanel_id;
	int x;
	int y;
	int fid;       /* Finger ID */
	char type;     /* 'D' - Down, 'M' - Move, 'U' - Up, */
	int touch_state;
	int area_rate;
	int touch_early_down_flag;
	long is_touch_fp_area_cnt;
	ktime_t touch_fp_area_time;
	ktime_t fp_down_time;
	int tp_firmware_time;
};

struct touch_film_info {
	bool filmed;
	int level;
	bool trusty;
};

#define EVENT_TYPE_DOWN    'D'
#define EVENT_TYPE_MOVE    'M'
#define EVENT_TYPE_UP      'U'

/* caller API */
int touchpanel_event_register_notifier(struct notifier_block *nb);
int touchpanel_event_unregister_notifier(struct notifier_block *nb);

/* callee API */
void touchpanel_event_call_notifier(unsigned long action, void *data);

#endif /* _TOUCHPANEL_EVENTNOTIFY_H */
