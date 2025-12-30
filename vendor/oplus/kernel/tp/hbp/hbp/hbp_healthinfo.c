// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/err.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <asm/stack_pointer.h>
#include <asm/current.h>
#include <linux/version.h>

#include "hbp_healthinfo.h"
#include "utils/debug.h"

int update_value_count_list(struct list_head *list, void *value)
{
	struct list_head *pos = NULL;
	struct health_value_count *vc = NULL;
	char *value_str = (char *)value;

	list_for_each(pos, list) {
		vc = (struct health_value_count *)pos;

		if (!strncmp((char *)vc->value, value_str, strlen(value_str))) {
			vc->count++;
			hbp_debug("%s str=%s, count=%d\n", __func__, (char *)vc->value, vc->count);
			return vc->count;
		}
	}

	vc = kzalloc(sizeof(struct health_value_count), GFP_KERNEL);

	if (vc) {
		vc->value = kzalloc(sizeof(char) * (strlen(value_str) + 1), GFP_KERNEL);

		if (vc->value) {
			strncpy((char *)vc->value, value_str, strlen(value_str) + 1);
			vc->count = 1;
			list_add_tail(&vc->head, list);
			hbp_debug("%s str=%s, count=%d\n", __func__, (char *)vc->value, vc->count);
			return vc->count;

		} else {
			hbp_info("vc->value kzalloc failed.\n");
			kfree(vc);
			return -1;
		}
	} else {
		hbp_info("kzalloc failed.\n");
		return -1;
	}

	return 0;
}

int clear_value_count_list(struct list_head *list)
{
	struct list_head *pos = NULL;
	struct health_value_count *vc = NULL;

	while (!list_empty(list)) {
		pos = list->next;
		list_del(pos);
		vc = list_entry(pos, struct health_value_count, head);
		kfree(vc->value);
		kfree(vc);
	}
	if (list_empty(list)) {
		hbp_info("list is cleared success.\n");
	} else {
		hbp_info("list is cleared fail.\n");
	}

	return 0;
}

int print_value_count_list(char *buf, size_t size, struct list_head *list, char *prefix)
{
	int cnt = 0;
	struct list_head *pos = NULL;
	struct health_value_count *vc = NULL;

	list_for_each(pos, list) {
		vc = (struct health_value_count *)pos;

		cnt += scnprintf(buf + cnt, size - cnt, "%s%s:%d\n", prefix ? prefix : "", (char *)vc->value, vc->count);

		hbp_debug("%s%s:%d\n", prefix ? prefix : "", (char *)vc->value, vc->count);
	}

	return cnt;
}

int hbp_healthinfo_report(struct monitor_data *monitor_data, char *report)
{
	int ret = 0;

	if (!monitor_data) {
		return 0;
	}

	ret = update_value_count_list(&monitor_data->health_report_list, report);

	return ret;
}

int hbp_healthinfo_read(char __user *buf, size_t size, struct monitor_data *monitor_data)
{
	char *info = NULL;

	if (!monitor_data) {
		return 0;
	}

	if (!buf) {
		hbp_err("health buf is Null.\n");
		return -1;
	}

	if (size <= 0) {
		hbp_err("health buf size invalid.\n");
		return -1;
	}

	info = kzalloc(size, GFP_KERNEL);
	if (!info) {
		hbp_err("health info alloc failed.\n");
		return -1;
	}

	/*debug info*/
	print_value_count_list(info, size, &monitor_data->health_report_list, PREFIX_HEALTH_REPORT);

	if (copy_to_user(buf, info, size)) {
		hbp_err("copy to user error");
	}
	kfree(info);

	return 0;
}

int hbp_healthinfo_clear(struct monitor_data *monitor_data)
{
	if (!monitor_data) {
		return 0;
	}

	hbp_info("Clear health info Now!\n");

	/*debug info*/
	clear_value_count_list(&monitor_data->health_report_list);

	hbp_info("Clear health info Finish!\n");

	return 0;
}

int hbp_healthinfo_init(struct monitor_data *monitor_data)
{
	if (!monitor_data) {
		hbp_info("monitor_data is NULL.\n");
		return -1;
	}

	INIT_LIST_HEAD(&monitor_data->health_report_list);
	return 0;
}
