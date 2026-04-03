/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _TOUCHPANEL_HEALTHONFO_
#define _TOUCHPANEL_HEALTHONFO_

#include <linux/i2c.h>
#include <linux/firmware.h>

#define DEFAULT_CHILD_STR_LEN       8
#define DEFAULT_REPORT_STR_LEN      100
#define DEFAULT_BUF_MATRIX_LINEBREAK    8
#define PREFIX_HEALTH_REPORT        "health_report-"
#define MAX_HEALTH_REPORT_LEN       50

#define SIG_SCREEN_ON_NO_ACK_TIMEOUT_CNT             "sig_screen_on_no_ack_timeout_cnt"
#define SIG_SCREEN_OFF_NO_ACK_TIMEOUT_CNT            "sig_screen_off_no_ack_timeout_cnt"
#define SIG_SCREEN_ON_NO_ACK_CNT                     "sig_screen_on_no_ack_cnt"
#define SIG_SCREEN_OFF_NO_ACK_CNT                    "sig_screen_off_no_ack_cnt"
#define FP_GRIP_SMALL_AREA_CNT                       "fp_grip_small_area_cnt"
#define FP_GRIP_BIG_AREA_CNT                         "fp_grip_big_area_cnt"
#define FP_GRIP_RELEASE_CNT                          "fp_grip_release_cnt"
#define MAX_NO_ACK_CNT                               4

#define FIRST_SCREEN_OFF_NOTIFY_TIME                 "first_screen_off_notify_time"
#define FIRST_SCREEN_OFF_NOTIFY_TIME_BELOW           "first_screen_off_notify_time_below_10"
#define MAX_SCREEN_OFF_NOTIFY_TIME                   10000
#define MAX_NOTIFY_LIST                              6

struct health_value_count {
	struct list_head head;
	void *value;
	int count;
};

struct notify_data {
	long screen_on_no_ack_cnt;
	long screen_off_no_ack_cnt;
	bool is_first_screen_off_notify;
	int notif_type_list[MAX_NOTIFY_LIST];
	int early_trigger_list[MAX_NOTIFY_LIST];
	signed long notify_cnt;
};

struct monitor_data {
	struct list_head	health_report_list;
	struct notify_data notify;
};

int hbp_healthinfo_report(struct monitor_data *monitor_data, char *report);

int hbp_healthinfo_read(char *buf, size_t size, struct monitor_data *monitor_data);

int hbp_healthinfo_clear(struct monitor_data *monitor_data);

int hbp_healthinfo_init(struct monitor_data *monitor_data);

#endif /* _TOUCHPANEL_HEALTHONFO_ */
