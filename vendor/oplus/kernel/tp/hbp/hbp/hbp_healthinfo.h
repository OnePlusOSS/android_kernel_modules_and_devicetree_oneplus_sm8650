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

struct health_value_count {
	struct list_head head;
	void *value;
	int count;
};

struct monitor_data {
	struct list_head	health_report_list;
};

int hbp_healthinfo_report(struct monitor_data *monitor_data, char *report);

int hbp_healthinfo_read(char *buf, size_t size, struct monitor_data *monitor_data);

int hbp_healthinfo_clear(struct monitor_data *monitor_data);

int hbp_healthinfo_init(struct monitor_data *monitor_data);

#endif /* _TOUCHPANEL_HEALTHONFO_ */
