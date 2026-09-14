// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */
#include <linux/slab.h>
#include <linux/sched/task.h>
#include <linux/minmax.h>
#include <linux/align.h>
#include <asm/cache.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>
#include <asm/barrier.h>
#include <uapi/linux/sched/types.h>
#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>
#include "sa_oemdata.h"
#include "sched_assist.h"
#include "sa_common.h"

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
#include "sa_balance.h"
#endif


static struct kmem_cache *oplus_task_struct_cachep;

static inline void init_oplus_task_struct(struct oplus_task_struct *ots, struct task_struct *task);

static inline struct oplus_task_struct *alloc_oplus_task_struct(struct task_struct *task)
{
	struct oplus_task_struct *ots = kmem_cache_alloc(oplus_task_struct_cachep, GFP_ATOMIC);
	if (IS_ERR_OR_NULL(ots))
		return NULL;

	init_oplus_task_struct(ots, task);
	ots->task = task;
	return ots;
}

static inline void free_oplus_task_struct(struct oplus_task_struct *ots)
{
	if (!ots)
		return;

	kmem_cache_free(oplus_task_struct_cachep, ots);
}

void android_vh_dup_task_struct_handler(void *unused,
		struct task_struct *tsk, struct task_struct *orig)
{
	struct oplus_task_struct *ots;
	struct oplus_task_struct *orig_ots;

	if (!tsk || !orig)
		return;
	/* The required space has been allocated */
	if (!IS_ERR_OR_NULL((void *)tsk->android_oem_data1[OTS_IDX]))
		return;

	ots = alloc_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots)) {
		ux_err("failed %s oplus_task_struc common:%s, pid:%d, tgid=%d\n", __func__, tsk->comm, tsk->pid, tsk->tgid);
		return;
	}

	/* if thread fork from RenderThread, inherit its IM_FLAG_RENDER_THREAD */
	orig_ots = get_oplus_task_struct(orig);
	if (!IS_ERR_OR_NULL(orig_ots)) {
		if (test_bit(IM_FLAG_RENDER_THREAD, &orig_ots->im_flag) && !strcmp(orig->comm, "RenderThread")) {
			set_bit(IM_FLAG_RENDER_THREAD, &ots->im_flag);
		}
	}
	smp_mb();

	WRITE_ONCE(tsk->android_oem_data1[OTS_IDX], (u64) ots);
}

void android_vh_free_task_handler(void *unused, struct task_struct *tsk)
{
	struct oplus_task_struct *ots = NULL;

	if (!tsk)
		return;

	ots = (struct oplus_task_struct *) READ_ONCE(tsk->android_oem_data1[OTS_IDX]);
	if (IS_ERR_OR_NULL(ots))
		return;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	/*
	 * NOTE:
	 * When the task is destroyed, the task needs to be removed from the
	 * rt_boost linked list, otherwise it may cause a crash due to access
	 * to an illegal address.
	 */
	remove_rt_boost_task(tsk);
#endif

	WRITE_ONCE(tsk->android_oem_data1[OTS_IDX], 0);

	ots->im_flag = 0;
	smp_mb();

	free_oplus_task_struct(ots);
}

static int register_oemdata_hooks(void)
{
	int ret = 0;

	REGISTER_TRACE_VH(android_vh_dup_task_struct, android_vh_dup_task_struct_handler);
	REGISTER_TRACE_VH(android_vh_free_task, android_vh_free_task_handler);

	return ret;
}

static void unregister_oemdata_hooks(void)
{
	UNREGISTER_TRACE_VH(android_vh_dup_task_struct, android_vh_dup_task_struct_handler);
	UNREGISTER_TRACE_VH(android_vh_free_task, android_vh_free_task_handler);
}

/*
 * NOTE:
 * Initialize the oplus_task_struct here.
 */
 static void init_oplus_task_struct(struct oplus_task_struct *ots, struct task_struct *task)
{
	memset(ots, 0, sizeof(struct oplus_task_struct));
	/* CONFIG_OPLUS_FEATURE_SCHED_ASSIST */
	RB_CLEAR_NODE(&ots->ux_entry);
	RB_CLEAR_NODE(&ots->exec_time_node);
	atomic64_set(&ots->inherit_ux, 0);
	ots->enqueue_time = 0;
	ots->inherit_ux_start = 0;
	/* u64 sum_exec_baseline; */
	ots->total_exec = 0;
	ots->vruntime = 0;
	ots->preset_vruntime = 0;
	ots->cfs_delta = -1;
	/* contains ux state
	1. if static and inherited ux both exist, static ux stores in ux_state, inherited ux in sub_ux_state.
	2. if only static ux exists, static ux stores in ux_state.
	2. if only inherited ux exists, inherited ux stores in ux_state */
	ots->ux_state = 0;
	ots->sub_ux_state = 0;
	ots->ux_depth = 0;
	ots->ux_priority = -1;
	ots->ux_nice = -1;
	ots->im_flag = 0;
	ots->affinity_pid = -1;
	ots->affinity_tgid = -1;
	memset(&ots->state, 0, sizeof(unsigned long));
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_QOS_SCHED)
	ots->qos_level = -1;
	ots->qos_recover_prio = -2;
	mutex_init(&ots->qs_mutex);
#endif
	atomic_set(&ots->is_vip_mvp, 0);
/* #if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_DDL) */
	ots->ddl = 0;
	ots->ddl_active_ts = 0;
	ots->runnable_ts = 0;
	RB_CLEAR_NODE(&ots->ddl_node);
/* #endif */
/*#if IS_ENABLED(CONFIG_OPLUS_FEATURE_ABNORMAL_FLAG)*/
	ots->abnormal_flag = 0;
/*#endif*/
	/* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
	ots->lb_state = 0;
	ots->ld_flag = 0;
	/* CONFIG_OPLUS_FEATURE_TASK_LOAD */
	ots->is_update_runtime = 0;
	ots->target_process = -1;
	ots->wake_tid = 0;
	ots->running_start_time = 0;
	ots->update_running_start_time = false;
	ots->exec_calc_runtime = 0;
	cpumask_clear(&ots->cpus_requested);
/*#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)*/
	ots->block_start_time = 0;
/*#endif*/
	/* CONFIG_OPLUS_FEATURE_FRAME_BOOST */
	INIT_LIST_HEAD(&ots->fbg_list);
	raw_spin_lock_init(&ots->fbg_list_entry_lock);
	ots->fbg_running = false; /* task belongs to a group, and in running */
	ots->fbg_state = 0;
	ots->preferred_cluster_id = -1;
	ots->fbg_depth = -1;
	ots->last_wake_ts = 0;
	ots->fbg_cur_group = 0;
/*#ifdef CONFIG_LOCKING_PROTECT*/
	ots->locking_start_time = 0;
	INIT_LIST_HEAD(&ots->locking_entry);
	ots->locking_depth = 0;
	ots->lk_tick_hit = 0;
/*#endif*/

/*#if IS_ENABLED(CONFIG_OPLUS_LOCKING_STRATEGY)*/
	memset(&ots->lkinfo, 0, sizeof(struct locking_info));
	INIT_LIST_HEAD(&ots->lkinfo.node);
/*#endif*/
/*#if IS_ENABLED(CONFIG_SCX_SCHED_ENABLE)*/
	ots->tick_hit_count = 0;
	ots->start_jiffies = 0;
/*#endif*/
/*#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FDLEAK_CHECK)*/
	ots->fdleak_flag = 0;
/*#endif*/
/*#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)*/
	/* for loadbalance */
	plist_node_init(&ots->rtb, MAX_IM_FLAG_PRIO);

	/*
	* The following variables are used to calculate the time
	* a task spends in the running/runnable state.
	*/
	ots->snap_run_delay = 0;
	ots->snap_pcount = 0;
/*#endif*/

#if IS_ENABLED(CONFIG_OPLUS_SCHED_TUNE)
ots->stune_idx = -1;
#endif
/*#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)*/
	atomic_set(&ots->pipeline_cpu, -1);
/*#endif*/
/* for oplus secure guard */
	ots->sg_flag = 0;
	ots->sg_scno = 0;
	ots->sg_uid = 0;
	ots->sg_euid = 0;
	ots->sg_gid = 0;
	ots->sg_egid = 0;
/*#if IS_ENABLED(CONFIG_ARM64_AMU_EXTN) && IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)*/
	ots->uid_struct = NULL;
	ots->amu_instruct = 0;
	ots->amu_cycle = 0;
/*#endif*/
	/* for binder ux */
	ots->binder_async_ux_enable = 0;
	ots->binder_async_ux_sts = false;
	ots->binder_thread_mode = 0;
	ots->binder_thread_node = NULL;
	/*
	* for vip binder:
	* vip_thread_policy_max_threads:for the max number of vip threads except binder_max_threads.
	* vip_save_threads:the vip thread saved in binder_max_threads
	*/
	ots->vip_thread_policy_max_threads = 0;
	ots->vip_save_threads = 0;
#ifdef CONFIG_OPLUS_CAMERA_UX
	if (CAMERA_UID == task_uid(task).val) {
		if (!strncmp(task->comm, CAMERA_PROVIDER_NAME, 15)) {
			ots->ux_state = SA_TYPE_HEAVY;
		}
	}
#endif
}
static void alloc_ots_mem_for_all_threads(void)
{
	struct task_struct *p, *g;
	u32 iter_cpu;

	read_lock(&tasklist_lock);

	for_each_process_thread(g, p) {
		struct oplus_task_struct *ots = NULL;

		ots = (struct oplus_task_struct *) READ_ONCE(p->android_oem_data1[OTS_IDX]);
		if (IS_ERR_OR_NULL(ots)) {
			ots = alloc_oplus_task_struct(p);
			if (!IS_ERR_OR_NULL(ots)) {
				smp_mb();
				WRITE_ONCE(p->android_oem_data1[OTS_IDX], (u64) ots);
			}
		}
	}
	for_each_possible_cpu(iter_cpu) {
		struct oplus_task_struct *ots = NULL;

		p = cpu_rq(iter_cpu)->idle;
		ots = (struct oplus_task_struct *) READ_ONCE(p->android_oem_data1[OTS_IDX]);
		if (IS_ERR_OR_NULL(ots)) {
			ots = alloc_oplus_task_struct(p);
			if (!IS_ERR_OR_NULL(ots)) {
				smp_mb();
				WRITE_ONCE(p->android_oem_data1[OTS_IDX], (u64) ots);
			}
		}
	}
	read_unlock(&tasklist_lock);
}

int sa_oemdata_init(void)
{
	oplus_task_struct_cachep = kmem_cache_create("oplus_task_struct",
			sizeof(struct oplus_task_struct), 0,
			SLAB_PANIC | SLAB_ACCOUNT, NULL);

	if (!oplus_task_struct_cachep)
		return -ENOMEM;

	alloc_ots_mem_for_all_threads();

	register_oemdata_hooks();

	return 0;
}

void __maybe_unused sa_oemdata_deinit(void)
{
	unregister_oemdata_hooks();
	kmem_cache_destroy(oplus_task_struct_cachep);
}

