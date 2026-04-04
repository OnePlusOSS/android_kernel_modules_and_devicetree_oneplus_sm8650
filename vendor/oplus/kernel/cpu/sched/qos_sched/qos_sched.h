/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2025 Oplus. All rights reserved.
 */

#ifndef _OPLUS_QOS_SCHED_H
#define _OPLUS_QOS_SCHED_H

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "qos_sched: " fmt

#include <linux/ioctl.h>

#define QOS_SCHED_MAGIC 'q'

#define qs_err(fmt, ...) \
		printk_deferred(KERN_ERR "[sched qos][%s]"fmt, __func__, ##__VA_ARGS__)

enum {
	SET_TID_LEVEL = 1,
	SET_PID_LEVEL,
	SET_TID_ARRAY_LEVEL,
	SET_LEVEL_MAX,
};

#define QOS_TASK_PID_FLAG_BITS 1
#define QOS_TASK_PID_FLAG_MASK    ((1ul << QOS_TASK_PID_FLAG_BITS) - 1)
#define QOS_TASK_PID_MASK (~QOS_TASK_PID_FLAG_MASK)

#define QOS_SCHED_UCLAMP_DEFAULT (-2)
#define QOS_SCHED_UCLAMP_RESET (-1)
#define QOS_SCHED_PRIO_DEFAULT (-2)
#define QOS_SCHED_PRIO_RESET (-1)

#define MAX_OUTPUT	(512)
#define EXTRA_SIZE (50)
#define MAX_GUARD_SIZE (MAX_OUTPUT - EXTRA_SIZE)

#define IOCTL_SET_TID_LEVEL \
	_IOW(QOS_SCHED_MAGIC, SET_TID_LEVEL, struct qos_sched_ioctl_data)
#define IOCTL_SET_PID_LEVEL \
	_IOW(QOS_SCHED_MAGIC, SET_PID_LEVEL, struct qos_sched_ioctl_data)
#define IOCTL_SET_TID_ARRAY_LEVEL \
	_IOW(QOS_SCHED_MAGIC, SET_TID_ARRAY_LEVEL, struct qos_sched_ioctl_data)

#define MAX_TIDS 64
struct tid_array {
	int count;
	int tids[MAX_TIDS];
};

struct qos_sched_ioctl_data {
	int level;
	union {
		int tid;
		int pid;
		struct tid_array tarray;
	} info;
};

struct oplus_task_group {
	int qos_level;
	int qos_latency;
};

#endif /* _OPLUS_QOS_SCHED_H */

