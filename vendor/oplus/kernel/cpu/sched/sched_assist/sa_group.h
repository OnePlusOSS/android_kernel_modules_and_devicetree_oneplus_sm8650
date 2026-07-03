/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2025 Oplus. All rights reserved.
 */

#ifndef _OPLUS_SA_GROUP_H_
#define _OPLUS_SA_GROUP_H_

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "sa_group: " fmt

#include <linux/cgroup-defs.h>

#define OPLUS_SG_IDX (3)
#define NR_TG_GRP (40)
#define SHARE_DEFAULT (100)
#define BG_SHARE_DEFAULT (50)
#define MAX_OUTPUT	(1024)
#define EXTRA_SIZE (100)
#define MAX_GUARD_SIZE (MAX_OUTPUT - EXTRA_SIZE)
#define SG_DDL_RTHRES_DEFAULT (60)

enum oplus_cgrp {
	ROOTGROUP,
	FOREGROUND,
	BACKGROUND,
	TOP_APP,
	SYSTEM_BG,
	FOREGROUND_WINDOW,
	CAMERA_DAEMON,
	SERVICE_FG,
	NORMAL_FG,
	MEM,
	SSTOP,
	SSFG,
	BG,
	OPLUS_CGRP_DEFAULT,
	OPLUS_CGRP_MAX,
};

struct oplus_sg_info {
	const char *tg_name;
	int id;
	u64 ddl;
	u64 ddl_rthres;
	bool dynamic_share;
	unsigned long calc_shares;
};

struct css_tg_map {
	struct list_head map_list;
	struct oplus_sg_info *sg_info;
};

extern struct task_group root_task_group;

static inline bool same_cgrp(const char *s1, const char *s2)
{
	if (strlen(s1) != strlen(s2))
		return false;

	if (!strncmp(s1, s2, strlen(s1)))
		return true;

	return false;
}

u64 get_sg_ddl_rthres(struct task_group *tg);
struct css_tg_map *get_oplus_tg_map(struct task_group *tg);
void oplus_sg_wake_up_new_task(struct task_struct *tsk);
void oplus_sched_group_init(struct proc_dir_entry *pde);
void oplus_update_tg_map(struct cgroup_subsys_state *css, bool initial);
bool fg_task(struct task_struct *p);
bool bg_task(struct task_struct *p);
bool ta_task(struct task_struct *p);
bool rootcg_task(struct task_struct *p);

#endif