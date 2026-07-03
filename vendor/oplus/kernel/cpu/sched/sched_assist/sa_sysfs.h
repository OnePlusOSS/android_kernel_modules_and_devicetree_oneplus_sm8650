/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SA_SYSFS_H_
#define _OPLUS_SA_SYSFS_H_

enum {
	OPT_STR_TYPE = 0,
	OPT_STR_PID,
	OPT_STR_VAL,
	OPT_STR_MAX = 3,
};

extern int global_sched_assist_enabled;
extern int global_sched_assist_scene;
extern char global_ux_task[];
extern pid_t save_audio_tgid;

int oplus_sched_assist_proc_init(void);
void oplus_sched_assist_proc_deinit(void);

extern struct task_struct *find_task_by_vpid(pid_t vnr);
long write_task_ux(pid_t pid, pid_t tid, int ux_value, bool fromSysOrApp);
#endif /* _OPLUS_SA_SYSFS_H_ */
