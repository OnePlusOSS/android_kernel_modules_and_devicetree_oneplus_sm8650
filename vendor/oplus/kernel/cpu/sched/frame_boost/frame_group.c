// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 */
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/sched/cpufreq.h>
#include <linux/syscore_ops.h>
#include <drivers/android/binder_internal.h>
#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>
#include <linux/reciprocal_div.h>

#include <trace/hooks/binder.h>
#include <trace/hooks/sched.h>
#include <trace/events/sched.h>
#include <trace/events/task.h>
#include <trace/events/power.h>

#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#include "frame_boost.h"
#include "cluster_boost.h"
#include "frame_debug.h"
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
#include "../eas_opt/oplus_cap.h"
#endif
#define CREATE_TRACE_POINTS
#include "frame_boost_trace.h"

#define NONE_FRAME_TASK      (0)
#define STATIC_FRAME_TASK    (1 << 0)
#define BINDER_FRAME_TASK    (1 << 1)
#define PENDING_FRAME_TASK   (1 << 14)

#define FRAME_DEFAULT        (1 << 2)
#define FRAME_COMPOSITION    (1 << 3)
#define FRAME_GAME           (1 << 4)
#define FRAME_INPUTMETHOD    (1 << 5)
#define FRAME_PENDING        (1 << 15)

#define GROUP_BIT_MASK(grp_id)  (1 << (grp_id + 1))
#define MULTI_FRAME_GROUP_MASK (((1 << MULTI_FBG_NUM) - 1) << (MULTI_FBG_ID + 1))
#define FRAME_GROUP_MASK \
		(FRAME_DEFAULT |\
		 FRAME_COMPOSITION |\
		 FRAME_GAME |\
		 FRAME_INPUTMETHOD |\
		 MULTI_FRAME_GROUP_MASK)

#define DEFAULT_BINDER_DEPTH (2)
#define MAX_BINDER_THREADS   (6)
#define INVALID_FBG_DEPTH    (-1)

#define DEFAULT_FRAME_RATE   (60)

#define DEFAULT_FREQ_UPDATE_MIN_INTERVAL    (2 * NSEC_PER_MSEC)
#define DEFAULT_UTIL_INVALID_INTERVAL       (32 * NSEC_PER_MSEC)

static int use_vutil_threshold = 100;
module_param_named(use_vutil_threshold, use_vutil_threshold, uint, 0644);

struct frame_group *frame_boost_groups[MAX_NUM_FBG_ID];

static DEFINE_RAW_SPINLOCK(freq_protect_lock);

static atomic_t fbg_initialized = ATOMIC_INIT(0);

__read_mostly int num_sched_clusters;

unsigned int ed_task_boost_type;
EXPORT_SYMBOL_GPL(ed_task_boost_type);

#define SIG_ED_TO_GPA 44
#define MIN_SEND_SIG_INTERVAL (50 * NSEC_PER_MSEC) /* 50ms */
static u64 last_send_sig_time;
static int game_ed_duration = 9500000; /* 9.5ms */
static int game_ed_user_pid = -1;
static struct task_struct *game_ed_user_task = NULL;
static DEFINE_RAW_SPINLOCK(game_ed_lock); /* protect game_ed_user_task */
static struct irq_work game_ed_irq_work;

struct list_head cluster_head;
#define for_each_sched_cluster(cluster) \
	list_for_each_entry_rcu(cluster, &cluster_head, list)

/*********************************
 * frame group common function
 *********************************/
static inline void move_list(struct list_head *dst, struct list_head *src)
{
	struct list_head *first, *last;

	first = src->next;
	last = src->prev;

	first->prev = dst;
	dst->prev = last;
	last->next = dst;

	/* Ensure list sanity before making the head visible to all CPUs. */
	smp_mb();
	dst->next = first;
}

static void get_possible_siblings(int cpuid, struct cpumask *cluster_cpus)
{
	int cpu;
	struct cpu_topology *cpu_topo, *cpuid_topo = &cpu_topology[cpuid];

	if (cpuid_topo->cluster_id == -1)
		return;

	for_each_possible_cpu(cpu) {
		cpu_topo = &cpu_topology[cpu];

		if (cpuid_topo->cluster_id != cpu_topo->cluster_id)
			continue;
		cpumask_set_cpu(cpu, cluster_cpus);
	}
}

static void insert_cluster(struct oplus_sched_cluster *cluster, struct list_head *head)
{
	struct oplus_sched_cluster *tmp;
	struct list_head *iter = head;

	list_for_each_entry(tmp, head, list) {
		if (arch_scale_cpu_capacity(cpumask_first(&cluster->cpus))
			< arch_scale_cpu_capacity(cpumask_first(&tmp->cpus)))
			break;
		iter = &tmp->list;
	}

	list_add(&cluster->list, iter);
}

static void cleanup_clusters(struct list_head *head)
{
	struct oplus_sched_cluster *cluster, *tmp;

	list_for_each_entry_safe(cluster, tmp, head, list) {
		list_del(&cluster->list);
		num_sched_clusters--;
		kfree(cluster);
	}
}

static struct oplus_sched_cluster *alloc_new_cluster(const struct cpumask *cpus)
{
	struct oplus_sched_cluster *cluster = NULL;

	cluster = kzalloc(sizeof(struct oplus_sched_cluster), GFP_ATOMIC);
	BUG_ON(!cluster);

	INIT_LIST_HEAD(&cluster->list);
	cluster->cpus = *cpus;

	return cluster;
}

struct oplus_sched_cluster *fb_cluster[MAX_CLS_NUM];

static inline void add_cluster(const struct cpumask *cpus, struct list_head *head)
{
	unsigned long capacity = 0, insert_capacity = 0;
	struct oplus_sched_cluster *cluster = NULL;

	capacity = arch_scale_cpu_capacity(cpumask_first(cpus));
	/* If arch_capacity is no different between mid cluster and max cluster,
	 * just combind them
	 */
	list_for_each_entry_rcu(cluster, head, list) {
		insert_capacity = arch_scale_cpu_capacity(cpumask_first(&cluster->cpus));
		if (capacity < insert_capacity) {
			ofb_debug("insert cluster=%*pbl is same as exist cluster=%*pbl\n",
				cpumask_pr_args(cpus), cpumask_pr_args(&cluster->cpus));
			break;
		}
	}

	cluster = alloc_new_cluster(cpus);
	insert_cluster(cluster, head);

	fb_cluster[num_sched_clusters] = cluster;

	num_sched_clusters++;
}

static inline void assign_cluster_ids(struct list_head *head)
{
	struct oplus_sched_cluster *cluster;
	int pos = 0;

	list_for_each_entry(cluster, head, list)
		cluster->id = pos++;
}

static bool build_clusters(void)
{
	struct cpumask cpus = *cpu_possible_mask;
	struct cpumask cluster_cpus;
	struct list_head new_head;
	int i;

	INIT_LIST_HEAD(&cluster_head);
	INIT_LIST_HEAD(&new_head);

	/* If this work failed, our cluster_head can still used with only one cluster struct */
	for_each_cpu(i, &cpus) {
		cpumask_clear(&cluster_cpus);
		get_possible_siblings(i, &cluster_cpus);
		if (cpumask_empty(&cluster_cpus)) {
			cleanup_clusters(&new_head);
			return false;
		}
		cpumask_andnot(&cpus, &cpus, &cluster_cpus);
		add_cluster(&cluster_cpus, &new_head);
	}

	assign_cluster_ids(&new_head);
	move_list(&cluster_head, &new_head);
	return true;
}

/* We use these flag(FRAME_COMPOSITION, FRAME_GAME, FRAME_INPUTMETHOD) to check which group
 * @task is in instead of traversing the whole group list
 */
inline struct frame_group *task_get_frame_group(struct oplus_task_struct *ots)
{
	int i;

	if (!(ots->fbg_state & FRAME_GROUP_MASK)) {
		return NULL;
	}

	if (ots->fbg_state & FRAME_COMPOSITION)
		return frame_boost_groups[SF_FRAME_GROUP_ID];
	else if (ots->fbg_state & FRAME_GAME)
		return frame_boost_groups[GAME_FRAME_GROUP_ID];
	else if (ots->fbg_state & FRAME_INPUTMETHOD)
		return frame_boost_groups[INPUTMETHOD_FRAME_GROUP_ID];
	else {
		for (i = MULTI_FBG_ID; i < MULTI_FBG_ID + MULTI_FBG_NUM; i++) {
			if (ots->fbg_state & GROUP_BIT_MASK(i))
				return frame_boost_groups[i];
		}
	}

	return NULL;
}

int task_get_frame_group_id(int pid)
{
	struct task_struct *tsk;
	struct oplus_task_struct *ots;
	struct frame_group *grp = NULL;
	int ret = 0;

	tsk = find_task_by_vpid(pid);
	if (!tsk) {
		ret = -ERR_TASK;
		goto out;
	}

	ots = get_oplus_task_struct(tsk);
	if (!ots) {
		ret = -ERR_OTS;
		goto out;
	}

	grp = task_get_frame_group(ots);
	if (grp == NULL) {
		ret = -ERR_GROUP;
		goto out;
	}

	ret = grp->id;

out:
	/*
	 * if (unlikely(sysctl_frame_boost_debug & DEBUG_KMSG)) {
	 *	if (ret <= 0)
	 *	ofb_debug("get frame group failed for task[%d][%s], fbg_state[%x], err[%d]\n",
	 *				pid, tsk ? tsk->comm : "NULL", ots ? ots->fbg_state : 0, ret);
	 *}
	 */
	return ret;
}
EXPORT_SYMBOL_GPL(task_get_frame_group_id);

static inline raw_spinlock_t *task_get_frame_group_lock(struct oplus_task_struct *ots)
{
	struct frame_group *grp;

	grp = task_get_frame_group(ots);
	if (grp == NULL)
		return NULL;
	return &grp->lock;
}

static inline bool __frame_boost_enabled(void)
{
	return likely(sysctl_frame_boost_enable);
}

bool frame_boost_enabled(void)
{
	return __frame_boost_enabled();
}
EXPORT_SYMBOL_GPL(frame_boost_enabled);

bool is_fbg_task(struct task_struct *p)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(p);

	if (IS_ERR_OR_NULL(ots))
		return false;

	return ots->fbg_state && !(ots->fbg_state & PENDING_FRAME_TASK);
}
EXPORT_SYMBOL_GPL(is_fbg_task);

/*********************************
 * frame group clock
 *********************************/
static ktime_t ktime_last;
static bool fbg_ktime_suspended;

u64 fbg_ktime_get_ns(void)
{
	if (unlikely(fbg_ktime_suspended))
		return ktime_to_ns(ktime_last);

	return ktime_get_ns();
}
EXPORT_SYMBOL_GPL(fbg_ktime_get_ns);

static void fbg_resume(void)
{
	fbg_ktime_suspended = false;
}

static int fbg_suspend(void)
{
	ktime_last = ktime_get();
	fbg_ktime_suspended = true;
	return 0;
}

static struct syscore_ops fbg_syscore_ops = {
	.resume		= fbg_resume,
	.suspend	= fbg_suspend
};

/***************************************************
 * add/remove static frame task to/from frame group
 ***************************************************/
static struct task_struct* remove_task_from_frame_group(struct task_struct *tsk)
{
	struct task_struct *task_removed = NULL;
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);
	struct frame_group *grp = NULL;

	if (IS_ERR_OR_NULL(ots))
		return task_removed;

	raw_spin_lock(&ots->fbg_list_entry_lock);
	grp = task_get_frame_group(ots);
	if (grp == NULL) {
		raw_spin_unlock(&ots->fbg_list_entry_lock);
		return task_removed;
	}
	/* Prevent deletion of tasks that are not in the current group */
	if (ots->fbg_cur_group != grp->id) {
		raw_spin_unlock(&ots->fbg_list_entry_lock);
		return task_removed;
	}
	lockdep_assert_held(&grp->lock);

	if (ots->fbg_state & STATIC_FRAME_TASK) {
		list_del_init(&ots->fbg_list);
		ots->fbg_state = NONE_FRAME_TASK;
		ots->fbg_depth = INVALID_FBG_DEPTH;
		ots->fbg_cur_group = 0;

		if (tsk == grp->ui) {
			grp->ui = NULL;
			grp->ui_pid = 0;
		} else if (tsk == grp->render) {
			grp->render = NULL;
			grp->render_pid = 0;
		}

		if (ots->fbg_running) {
			ots->fbg_running = false;
			grp->nr_running--;
			if (unlikely(grp->nr_running < 0))
				grp->nr_running = 0;
		}
		/*BUG: 7976822*/
		/*put_task_struct(tsk);*/
		task_removed = tsk;
	}
	raw_spin_unlock(&ots->fbg_list_entry_lock);

	if (list_empty(&grp->tasks)) {
		grp->preferred_cluster = NULL;
		grp->available_cluster = NULL;
		atomic64_set(&grp->policy_util, 0);
		atomic64_set(&grp->curr_util, 0);
		grp->nr_running = 0;
	}

	return task_removed;
}

static void clear_all_frame_task(struct frame_group *grp, struct list_head *task_list)
{
	struct oplus_task_struct *ots;
	struct oplus_task_struct *tmp;

	list_for_each_entry_safe(ots, tmp, &grp->tasks, fbg_list) {
		struct task_struct *p = ots_to_ts(ots);

		/*raw_spin_lock(&ots->fbg_list_entry_lock);*/

		if (ots->fbg_state & STATIC_FRAME_TASK) {
			if (p == grp->ui) {
				grp->ui = NULL;
				grp->ui_pid = 0;
			} else if (p == grp->render) {
				grp->render = NULL;
				grp->render_pid = 0;
			}

			if (ots->fbg_running) {
				ots->fbg_running = false;
				grp->nr_running--;
				if (unlikely(grp->nr_running < 0))
					grp->nr_running = 0;
			}
		}

		/*BUG: 7976822*/
		/*list_del_init(&ots->fbg_list);
		ots->fbg_state = NONE_FRAME_TASK;
		ots->fbg_depth = INVALID_FBG_DEPTH;
		ots->fbg_cur_group = 0;

		if (unlikely(sysctl_frame_boost_debug & DEBUG_KMSG))
			ofb_debug("remove task[%d][%s] from grp_id[%d] succeed\n", p->pid, p->comm, grp->id);
		put_task_struct(p);
		raw_spin_unlock(&ots->fbg_list_entry_lock);*/

		/* set task to a pending state temporarily*/
		ots->fbg_state = FRAME_PENDING | PENDING_FRAME_TASK;
	}

	/*BUG: 7976822*/
	list_add(task_list, &grp->tasks);
	list_del_init(&grp->tasks);

	if (list_empty(&grp->tasks)) {
		grp->preferred_cluster = NULL;
		grp->available_cluster = NULL;
		atomic64_set(&grp->policy_util, 0);
		atomic64_set(&grp->curr_util, 0);
		grp->nr_running = 0;
		grp->binder_thread_num = 0;
	} else if (unlikely(sysctl_frame_boost_debug & DEBUG_KMSG))
		ofb_debug("task still be left in grp_id[%d]\n", grp->id);
}

static void release_frame_task_list(struct list_head *task_list)
{
	struct oplus_task_struct *ots;
	struct oplus_task_struct *tmp;
	unsigned long flags;

	local_irq_save(flags);
	list_for_each_entry_safe(ots, tmp, task_list, fbg_list) {
		struct task_struct *p = ots_to_ts(ots);

		raw_spin_lock(&ots->fbg_list_entry_lock);

		list_del_init(&ots->fbg_list);
		ots->fbg_state = NONE_FRAME_TASK;
		ots->fbg_depth = INVALID_FBG_DEPTH;
		ots->fbg_cur_group = 0;
		raw_spin_unlock(&ots->fbg_list_entry_lock);
		if (unlikely(sysctl_frame_boost_debug & DEBUG_KMSG))
			ofb_debug("remove task[%d][%s] succeed\n", p->pid, p->comm);
		put_task_struct(p);
	}
	local_irq_restore(flags);

	if (unlikely(!list_empty(task_list)) && unlikely(sysctl_frame_boost_debug & DEBUG_KMSG))
		ofb_debug("task leak!\n");
}

void clear_all_static_frame_task_lock(int grp_id)
{
	unsigned long flags;
	LIST_HEAD(task_list);
	struct frame_group *grp = NULL;

	grp = frame_boost_groups[grp_id];
	raw_spin_lock_irqsave(&grp->lock, flags);
	clear_all_frame_task(grp, &task_list);
	raw_spin_unlock_irqrestore(&grp->lock, flags);
	release_frame_task_list(&task_list);
}
EXPORT_SYMBOL_GPL(clear_all_static_frame_task_lock);

/*
 * WARNING: This function must be called with rcu_read_lock held.
 */
static bool add_task_to_frame_group(struct frame_group *grp, struct task_struct *task)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);
	int cur_fbg_state = 0, is_single_bit = 0;

	WARN_ON_ONCE(!rcu_read_lock_held());
	if (IS_ERR_OR_NULL(ots))
		return false;

	raw_spin_lock(&ots->fbg_list_entry_lock);

	/* ots->fbg_state != 0, avoid adding task to duplicate groups.
	 * also need to checkout ots->fbg_list, if ots->If fbg_state
	 * has been modified accidentally.
	 */
	if (ots->fbg_state || (!list_empty(&ots->fbg_list)) || task->flags & PF_EXITING) {
		raw_spin_unlock(&ots->fbg_list_entry_lock);
		return false;
	}
	get_task_struct(task);
	list_add(&ots->fbg_list, &grp->tasks);
	ots->fbg_state = STATIC_FRAME_TASK;
	ots->fbg_cur_group = grp->id;

	if (grp == frame_boost_groups[SF_FRAME_GROUP_ID])
		ots->fbg_state |= FRAME_COMPOSITION;
	else if (grp == frame_boost_groups[GAME_FRAME_GROUP_ID])
		ots->fbg_state |= FRAME_GAME;
	else if (grp == frame_boost_groups[INPUTMETHOD_FRAME_GROUP_ID])
		ots->fbg_state |= FRAME_INPUTMETHOD;
	else if (is_multi_frame_fbg(grp->id))
		ots->fbg_state |= GROUP_BIT_MASK(grp->id);

	/* check if fbg_state contains more than one group, means: fbg_state - 1 != 2^n
	 * it's unlikely to happen here.
	 */
	cur_fbg_state = ots->fbg_state;
	is_single_bit = is_power_of_2(cur_fbg_state - STATIC_FRAME_TASK);
	if (!is_single_bit) {
		ofb_err("line:[%d]: task=%s, pid=%d, tgid=%d, prio=%d, fbg_state=%d, group_id:%d, cpu=%d\n",
				__LINE__, task->comm, task->pid, task->tgid, task->prio, cur_fbg_state, ots->fbg_cur_group, task_cpu(task));
		BUG_ON(!is_single_bit);
	}

	/* Static frame task's depth is zero */
	ots->fbg_depth = 0;

	raw_spin_unlock(&ots->fbg_list_entry_lock);
	if (unlikely(sysctl_frame_boost_debug & DEBUG_KMSG))
		ofb_debug("add task[%d][%s] to grp_id[%d], fbg_state[0x%x]\n", task->pid, task->comm, grp->id, ots->fbg_state);
	return true;
}

void set_ui_thread(int grp_id, int pid, int tid)
{
	unsigned long flags;
	LIST_HEAD(task_list);
	struct task_struct *ui;
	struct oplus_task_struct *ots = NULL;
	struct frame_group *grp = NULL;

	grp = frame_boost_groups[grp_id];

	if (is_multi_frame_fbg(grp_id) && !is_active_multi_frame_fbg(grp_id)) {
		ofb_err("set ui_pid[%d] to inactive grp_id[%d]\n", pid, grp_id);
		return;
	}
	raw_spin_lock_irqsave(&grp->lock, flags);
	if (pid <= 0 || pid == grp->ui_pid) {
		ofb_err("set invalid or same ui_pid[%d] to grp_id[%d]->ui_pid[%d]\n", pid, grp_id, grp->ui_pid);
		goto done;
	}
	/* There is a conflict with add_task_to_frame_group() */
	rcu_read_lock();
	ui = find_task_by_vpid(pid);
	if (ui) {
		ots = get_oplus_task_struct(ui);
		if (check_group_condition(ots->fbg_cur_group, grp_id)) {
			rcu_read_unlock();
			goto done;
		}
	}

	if (grp->ui) {
		clear_all_frame_task(grp, &task_list);
	}

	if (ui && add_task_to_frame_group(grp, ui)) {
		grp->ui = ui;
		grp->ui_pid = pid;

		if (unlikely(sysctl_frame_boost_debug & DEBUG_KMSG))
			ofb_debug("set ui_pid[%s][%d] to grp_id[%d]\n", ui->comm, ui->pid, grp_id);
	}
	rcu_read_unlock();
done:
	raw_spin_unlock_irqrestore(&grp->lock, flags);
	release_frame_task_list(&task_list);
}
EXPORT_SYMBOL_GPL(set_ui_thread);

void set_render_thread(int grp_id, int pid, int tid)
{
	unsigned long flags;
	struct task_struct *render;
	struct task_struct *task_removed = NULL;
	struct oplus_task_struct *ots = NULL;
	struct frame_group *grp;

	if (!is_fbg(grp_id)) {
		ofb_err("set render_tid[%d] to invalid grp_id[%d]\n", pid, grp_id);
		return;
	}

	grp = frame_boost_groups[grp_id];

	if (is_multi_frame_fbg(grp_id) && !is_active_multi_frame_fbg(grp_id)) {
		ofb_err("set render_tid[%d] to inactive grp_id[%d]\n", pid, grp_id);
		return;
	}
	raw_spin_lock_irqsave(&grp->lock, flags);
	if (tid <= 0 || pid != grp->ui_pid || tid == grp->render_pid) {
		goto done;
	}

	/* There is a conflict with add_task_to_frame_group() */
	rcu_read_lock();
	render = find_task_by_vpid(tid);
	if (render) {
		ots = get_oplus_task_struct(render);
		if (check_group_condition(ots->fbg_cur_group, grp_id)) {
			rcu_read_unlock();
			goto done;
		}
	}

	if (grp->render)
		task_removed = remove_task_from_frame_group(grp->render);

	if (render && add_task_to_frame_group(grp, render)) {
		grp->render = render;
		grp->render_pid = tid;
		if (unlikely(sysctl_frame_boost_debug & DEBUG_KMSG))
			ofb_debug("set render_tid[%s][%d] to grp_id[%d]\n", render->comm, render->pid, grp_id);
	}
	rcu_read_unlock();
done:
	raw_spin_unlock_irqrestore(&grp->lock, flags);
	if (task_removed) {
		put_task_struct(task_removed);
	}
}
EXPORT_SYMBOL_GPL(set_render_thread);

void set_hwui_thread(int grp_id, int pid, int hwtid1, int hwtid2)
{
	unsigned long flags;
	struct task_struct *task_to_remove1 = NULL;
	struct task_struct *task_to_remove2 = NULL;
	struct task_struct *hwtask1, *hwtask2;
	struct oplus_task_struct *hwots1 = NULL, *hwots2 = NULL;
	struct frame_group *grp;

	grp = frame_boost_groups[grp_id];

	raw_spin_lock_irqsave(&grp->lock, flags);
	if (hwtid1 <= 0 || hwtid2 <= 0 || hwtid1 == grp->hwtid1 || hwtid2 == grp->hwtid2 || pid != grp->ui_pid) {
		goto done;
	}

	/* There is a conflict with add_task_to_frame_group() */
	rcu_read_lock();
	hwtask1 = find_task_by_vpid(hwtid1);
	hwtask2 = find_task_by_vpid(hwtid2);
	if (hwtask1 && hwtask2) {
		hwots1 = get_oplus_task_struct(hwtask1);
		hwots2 = get_oplus_task_struct(hwtask2);
		if (check_group_condition(hwots1->fbg_cur_group, grp_id) ||
			check_group_condition(hwots2->fbg_cur_group, grp_id)) {
			rcu_read_unlock();
			goto done;
		}
	}

	if (grp->hwtask1)
		task_to_remove1 = remove_task_from_frame_group(grp->hwtask1);
	if (grp->hwtask2)
		task_to_remove2 = remove_task_from_frame_group(grp->hwtask2);

	if (hwtask1 && add_task_to_frame_group(grp, hwtask1)) {
		grp->hwtask1 = hwtask1;
		grp->hwtid1 = hwtid1;
	}

	if (hwtask2 && add_task_to_frame_group(grp, hwtask2)) {
		grp->hwtask2 = hwtask2;
		grp->hwtid2 = hwtid2;
	}
	rcu_read_unlock();
done:
	raw_spin_unlock_irqrestore(&grp->lock, flags);
	if (task_to_remove1) {
		put_task_struct(task_to_remove1);
	}
	if (task_to_remove2) {
		put_task_struct(task_to_remove2);
	}
}
EXPORT_SYMBOL_GPL(set_hwui_thread);

int get_frame_group_ui(int grp_id)
{
	return frame_boost_groups[grp_id]->ui_pid;
}
EXPORT_SYMBOL_GPL(get_frame_group_ui);

void set_sf_thread(int pid, int tid)
{
	unsigned long flags;
	LIST_HEAD(task_list);
	struct task_struct *ui;
	struct oplus_task_struct *ots = NULL;
	struct frame_group *grp = frame_boost_groups[SF_FRAME_GROUP_ID];

	raw_spin_lock_irqsave(&grp->lock, flags);
	if (pid <= 0 || pid == grp->ui_pid)
		goto done;

	/* There is a conflict with add_task_to_frame_group() */
	rcu_read_lock();
	ui = find_task_by_vpid(pid);
	if (ui) {
		ots = get_oplus_task_struct(ui);
		if (check_group_condition(ots->fbg_cur_group, SF_FRAME_GROUP_ID)) {
			rcu_read_unlock();
			goto done;
		}
	}

	if (grp->ui) {
		clear_all_frame_task(grp, &task_list);
	}

	if (ui && add_task_to_frame_group(grp, ui)) {
		grp->ui = ui;
		grp->ui_pid = pid;
	}
	rcu_read_unlock();
done:
	raw_spin_unlock_irqrestore(&grp->lock, flags);
	release_frame_task_list(&task_list);
}
EXPORT_SYMBOL_GPL(set_sf_thread);

void set_renderengine_thread(int pid, int tid)
{
	unsigned long flags;
	struct task_struct *task_removed = NULL;
	struct task_struct *render;
	struct oplus_task_struct *ots = NULL;
	struct frame_group *grp = frame_boost_groups[SF_FRAME_GROUP_ID];

	raw_spin_lock_irqsave(&grp->lock, flags);
	if (tid <= 0 || pid != grp->ui_pid || tid == grp->render_pid)
		goto done;
	/* There is a conflict with add_task_to_frame_group() */
	rcu_read_lock();
	render = find_task_by_vpid(tid);
	if (render) {
		ots = get_oplus_task_struct(render);
		if (check_group_condition(ots->fbg_cur_group, SF_FRAME_GROUP_ID)) {
			rcu_read_unlock();
			goto done;
		}
	}

	if (grp->render)
		task_removed = remove_task_from_frame_group(grp->render);

	if (render && add_task_to_frame_group(grp, render)) {
		grp->render = render;
		grp->render_pid = tid;
	}
	rcu_read_unlock();
done:
	raw_spin_unlock_irqrestore(&grp->lock, flags);
	if (task_removed) {
		put_task_struct(task_removed);
	}
}
EXPORT_SYMBOL_GPL(set_renderengine_thread);

static inline bool is_same_uid(struct task_struct *p, struct task_struct *grp_ui)
{
	int p_uid, ui_uid;

	if (p == NULL || grp_ui == NULL)
		return false;

	p_uid = task_uid(p).val;
	ui_uid = task_uid(grp_ui).val;

	return (p_uid == ui_uid);
}

bool add_rm_related_frame_task(int grp_id, int pid, int tid, int add, int r_depth, int r_width)
{
	unsigned long flags;
	struct task_struct *task_removed = NULL;
	struct task_struct *tsk = NULL;
	struct frame_group *grp = NULL;
	bool success = false;

	rcu_read_lock();
	tsk = find_task_by_vpid(tid);
	if (!tsk)
		goto out;

	grp = frame_boost_groups[grp_id];
	raw_spin_lock_irqsave(&grp->lock, flags);
	if (add && is_same_uid(tsk, grp->ui)) {
		add_task_to_frame_group(grp, tsk);
	} else if (!add) {
		task_removed = remove_task_from_frame_group(tsk);
	}
	raw_spin_unlock_irqrestore(&grp->lock, flags);

	/* TODO: find related threads and set them as frame task
	 * if (r_depth > 0 && r_width > 0) {
	 * }
	 */

	success = true;
out:
	rcu_read_unlock();
	if (task_removed) {
		put_task_struct(task_removed);
	}
	return success;
}
EXPORT_SYMBOL_GPL(add_rm_related_frame_task);

bool add_task_to_game_frame_group(int tid, int add)
{
	unsigned long flags;
	struct task_struct *task_removed = NULL;
	struct task_struct *tsk = NULL;
	struct frame_group *grp = NULL;
	struct oplus_task_struct *ots = NULL;
	bool success = false;

	rcu_read_lock();
	tsk = find_task_by_vpid(tid);
	/* game_frame_boost_group not add binder task */
	if (!tsk || strstr(tsk->comm, "binder:") || strstr(tsk->comm, "HwBinder:"))
		goto out;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots))
		goto out;

	raw_spin_lock(&ots->fbg_list_entry_lock);
	/*
	 * only the task which not belonged to any group can be added to game_frame_boost_group in this func,
	 * only the task which belonged to game_frame_boost_group can be removed in this func.
	 */
	if ((add && ots->fbg_state)
		|| (!add && !(ots->fbg_state & FRAME_GAME))) {
		raw_spin_unlock(&ots->fbg_list_entry_lock);
		goto out;
	}
	raw_spin_unlock(&ots->fbg_list_entry_lock);

	grp = frame_boost_groups[GAME_FRAME_GROUP_ID];
	raw_spin_lock_irqsave(&grp->lock, flags);
	if (add) {
		add_task_to_frame_group(grp, tsk);
	} else if (!add) {
		task_removed = remove_task_from_frame_group(tsk);
	}
	raw_spin_unlock_irqrestore(&grp->lock, flags);

	success = true;
out:
	rcu_read_unlock();
	if (task_removed) {
		put_task_struct(task_removed);
	}
	return success;
}
EXPORT_SYMBOL_GPL(add_task_to_game_frame_group);

/**********************************************************
 * add/remove dynamic binder frame task to/from frame group
 **********************************************************/
static struct task_struct* remove_binder_from_frame_group(struct task_struct *binder)
{
	struct oplus_task_struct *ots_binder = get_oplus_task_struct(binder);
	struct frame_group *grp = NULL;

	if (IS_ERR_OR_NULL(ots_binder) || !(ots_binder->fbg_state & BINDER_FRAME_TASK))
		return NULL;

	grp = task_get_frame_group(ots_binder);
	if (grp == NULL)
		return NULL;
	lockdep_assert_held(&grp->lock);

	raw_spin_lock(&ots_binder->fbg_list_entry_lock);

	/* judge two times for hot path performance */
	if (!(ots_binder->fbg_state & BINDER_FRAME_TASK)) {
		raw_spin_unlock(&ots_binder->fbg_list_entry_lock);
		return NULL;
	}

	list_del_init(&ots_binder->fbg_list);
	ots_binder->fbg_state = NONE_FRAME_TASK;
	ots_binder->fbg_depth = INVALID_FBG_DEPTH;

	raw_spin_unlock(&ots_binder->fbg_list_entry_lock);

	grp->binder_thread_num--;

	if (grp->binder_thread_num < 0)
		ofb_err("group binder num is less than 0, binder_num=%d, grp->id=%d, prio=%d",
			grp->binder_thread_num, grp->id, binder->prio);

	/*BUG: 7976822*/
	/*put_task_struct(binder);*/
	return binder;
}

static void add_binder_to_frame_group(struct task_struct *binder, struct task_struct *from)
{
	struct oplus_task_struct *ots_binder, *ots_from;
	unsigned long flags;
	struct frame_group *grp = NULL;

	if (binder == NULL || from == NULL)
		return;

	ots_binder = get_oplus_task_struct(binder);
	ots_from = get_oplus_task_struct(from);

	if (IS_ERR_OR_NULL(ots_binder) || IS_ERR_OR_NULL(ots_from))
		return;

	/* game_frame_boost_group and inputmethod_frame_boost_group not add binder task */
	if (ots_from->fbg_state & (FRAME_GAME | FRAME_INPUTMETHOD | FRAME_PENDING))
		return;

	grp = task_get_frame_group(ots_from);
	if (grp == NULL)
		return;

	raw_spin_lock_irqsave(&grp->lock, flags);

	if (ots_from->fbg_state == NONE_FRAME_TASK || ots_binder->fbg_state)
		goto unlock;

	if (grp->binder_thread_num > MAX_BINDER_THREADS)
		goto unlock;

	if ((ots_from->fbg_state & BINDER_FRAME_TASK) &&
		ots_from->fbg_depth >= DEFAULT_BINDER_DEPTH) {
		goto unlock;
	}

	raw_spin_lock(&ots_binder->fbg_list_entry_lock);

	/* judge two times for hot path performance */
	if (ots_binder->fbg_state) {
		raw_spin_unlock(&ots_binder->fbg_list_entry_lock);
		goto unlock;
	}

	if (list_empty(&ots_binder->fbg_list) && ots_binder->fbg_cur_group == 0) {
		list_add(&ots_binder->fbg_list, &grp->tasks);
		get_task_struct(binder);
		ots_binder->fbg_state = BINDER_FRAME_TASK;
		ots_binder->fbg_state |= (ots_from->fbg_state & FRAME_GROUP_MASK);
		ots_binder->fbg_depth = ots_from->fbg_depth + 1;
		grp->binder_thread_num++;
	}

	raw_spin_unlock(&ots_binder->fbg_list_entry_lock);
unlock:
	raw_spin_unlock_irqrestore(&grp->lock, flags);
}

/*
 * task_rename_hook - check if the binder thread should be add to
 *                            frame group
 * @task: binder thread that to be wokeup.
 * @sync: whether to do a synchronous wake-up.
 *       the other paramenter is unused
 */
void task_rename_hook(void *unused, struct task_struct *p, const char *buf)
{
}

#ifdef TODO_DELME
/*
 * fbg_binder_wakeup_hook - check if the binder thread should be add to
 *                            frame group
 * @task: binder thread that to be wokeup.
 * @sync: whether to do a synchronous wake-up.
 *       the other paramenter is unused
 */
static void fbg_binder_wakeup_hook(void *unused, struct task_struct *caller_task,
	struct task_struct *binder_proc_task, struct task_struct *binder_th_task,
	bool pending_async, bool sync)
{
	if (sync)
		add_binder_to_frame_group(binder_th_task, current);
}
#endif
/*
 * fbg_binder_restore_priority_hook - check if the binder thread should be remove from
 *             frame group after finishing their work
 * @task: binder thread that finished binder request and restore to saved priority.
 * @t: binder transaction that to be finished
 *       the other paramenter is unused
 */
static void fbg_binder_restore_priority_hook(void *unused, struct binder_transaction *t,
	struct task_struct *task)
{
	struct task_struct *task_removed = NULL;
	struct oplus_task_struct *ots = get_oplus_task_struct(task);
	unsigned long flags;
	raw_spinlock_t *lock = NULL;

	if (IS_ERR_OR_NULL(ots))
		return;

	lock = task_get_frame_group_lock(ots);
	if (lock == NULL)
		return;

	if (task != NULL) {
		raw_spin_lock_irqsave(lock, flags);
		task_removed = remove_binder_from_frame_group(task);
		raw_spin_unlock_irqrestore(lock, flags);
	}
	if (task_removed) {
		put_task_struct(task_removed);
	}
}

/*
 * fbg_binder_wait_for_work_hook - check if the binder thread should be remove from
 *             frame group before insert to idle binder list
 * @task: binder thread
 * @do_proc_work: whether the binder thread is waiting for new request
 *       the other paramenter is unused
 */
static void fbg_binder_wait_for_work_hook(void *unused, bool do_proc_work,
	struct binder_thread *tsk, struct binder_proc *proc)
{
	struct task_struct *task_removed = NULL;
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk->task);
	unsigned long flags;
	raw_spinlock_t *lock = NULL;

	if (IS_ERR_OR_NULL(ots))
		return;

	if (do_proc_work) {
		lock = task_get_frame_group_lock(ots);
		if (lock == NULL)
			return;

		raw_spin_lock_irqsave(lock, flags);
		task_removed = remove_binder_from_frame_group(tsk->task);
		raw_spin_unlock_irqrestore(lock, flags);
	}

	if (task_removed) {
		put_task_struct(task_removed);
	}
}

static void fbg_sync_txn_recvd_hook(void *unused, struct task_struct *tsk, struct task_struct *from)
{
	add_binder_to_frame_group(tsk, from);
}


/*********************************
 * load tracking for frame group
 *********************************/
static inline unsigned int get_cur_freq(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);

	return (policy == NULL) ? 0 : policy->cur;
}

static inline unsigned int get_max_freq(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);

	return (policy == NULL) ? 0 : policy->cpuinfo.max_freq;
}

void set_frame_group_window_size(int grp_id, unsigned int window_size)
{
	struct frame_group *grp = NULL;
	unsigned long flags;

	grp = frame_boost_groups[grp_id];
	raw_spin_lock_irqsave(&grp->lock, flags);
	grp->window_size = window_size;
	raw_spin_unlock_irqrestore(&grp->lock, flags);

}
EXPORT_SYMBOL_GPL(set_frame_group_window_size);

#define DIV64_U64_ROUNDUP(X, Y) div64_u64((X) + (Y - 1), Y)
static inline u64 scale_exec_time(u64 delta, struct rq *rq)
{
	u64 task_exec_scale;
	unsigned int cur_freq, max_freq;
	int cpu = cpu_of(rq);

	/* TODO:
	 * Use freq_avg instead of freq_cur, because freq may trans when task running.
	 * Can we use this hook trace_android_rvh_cpufreq_transition?
	 */
	cur_freq = get_cur_freq(cpu);
	max_freq = get_max_freq(cpu);

	if (unlikely(cur_freq <= 0) || unlikely(max_freq <= 0) || unlikely(cur_freq > max_freq)) {
		ofb_err("cpu=%d cur_freq=%u max_freq=%u\n", cpu, cur_freq, max_freq);
		return delta;
	}

	task_exec_scale = DIV64_U64_ROUNDUP(cur_freq *
				arch_scale_cpu_capacity(cpu),
				max_freq);

	return (delta * task_exec_scale) >> 10;
}

static s64 update_window_start(u64 wallclock, struct frame_group *grp)
{
	s64 delta;
	int grp_id = grp->id;

	lockdep_assert_held(&grp->lock);

	delta = wallclock - grp->window_start;

	if (delta <= 0) {
		ofb_debug("wallclock=%llu is lesser than window_start=%llu, group_id=%d",
			wallclock, grp->window_start, grp_id);
		return delta;
	}

	grp->window_start = wallclock;
	grp->prev_window_size = grp->window_size;
	grp->window_busy = (grp->curr_window_exec * 100) / delta;


	if (unlikely(sysctl_frame_boost_debug & DEBUG_FTRACE))
		trace_printk("ui_pid=%d window_start=%llu window_size=%llu delta=%lld group_id=%d\n",
			grp->ui_pid, grp->window_start, grp->window_size, delta, grp_id);

	return delta;
}

static void update_group_exectime(struct frame_group *grp, int group_id)
{
	lockdep_assert_held(&grp->lock);

	grp->prev_window_scale = grp->curr_window_scale;
	grp->curr_window_scale = 0;
	grp->prev_window_exec = grp->curr_window_exec;
	grp->curr_window_exec = 0;
	grp->curr_end_exec = 0;
	grp->curr_end_time = 0;
	grp->handler_busy = false;

	if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE)) {
		val_systrace_c(grp->id, grp->prev_window_exec, "prev_window_exec", prev_window_exec);
		val_systrace_c(grp->id, get_frame_putil(group_id, grp->prev_window_scale, grp->frame_zone),
			"prev_window_util", prev_window_util);
		val_systrace_c(grp->id, grp->curr_window_exec, "curr_window_exec", curr_window_exec);
		val_systrace_c(grp->id, get_frame_putil(group_id, grp->curr_window_scale, grp->frame_zone),
			"curr_window_util", curr_window_util);
	}
}

static void update_util_before_window_rollover(int grp_id);
int rollover_frame_group_window(int grp_id)
{
	u64 wallclock;
	unsigned long flags;
	struct frame_group *grp;

	update_util_before_window_rollover(grp_id);

	grp = frame_boost_groups[grp_id];
	raw_spin_lock_irqsave(&grp->lock, flags);

	wallclock = fbg_ktime_get_ns();
	update_window_start(wallclock, grp);
	if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE)) {
		val_systrace_c(grp_id, get_frame_rate(grp_id), "framerate", framerate);
		val_systrace_c(grp_id, grp->ui_pid, "00_ui", ui_id);
	}

	/* We set curr_window_* as prev_window_* and clear curr_window_*,
	 * but prev_window_* now may belong to old_frame_app, and curr_window_*
	 * belong to new_frame_app, when called from ioctl(BOOST_MOVE_FG).
	 */
	update_group_exectime(grp, grp_id);

	raw_spin_unlock_irqrestore(&grp->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(rollover_frame_group_window);

/**************************************
 * cpu frequence adjust for frame group
 ***************************************/
static void update_frame_zone(struct frame_group *grp, u64 wallclock)
{
	s64 delta;
	struct frame_info *frame_info;

	frame_info = fbg_frame_info(grp->id);
	if (frame_info == NULL)
		return;

	grp->frame_zone = 0;

	delta = wallclock - grp->window_start;
	if (delta <= (2 * grp->window_size)) {
		if (is_multi_frame_fbg(grp->id) && frame_info->next_vsync)
			grp->frame_zone |= FRAME_ZONE;
		if (grp->id == SF_FRAME_GROUP_ID)
			grp->frame_zone |= FRAME_ZONE;
	}
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	if (sysctl_slide_boost_enabled || sysctl_input_boost_enabled)
		grp->frame_zone |= USER_ZONE;
#endif

	if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE)) {
		val_systrace_c(grp->id, grp->frame_zone, "01_frame_zone", frame_zone);
	}
}

void update_frame_group_buffer_count(void)
{
	int i, buffer_count;
	struct frame_info *frame_info;

	for (i = MULTI_FBG_ID; i <  MULTI_FBG_ID + MULTI_FBG_NUM; i++) {
		if (is_active_multi_frame_fbg(i)) {
			frame_info = fbg_frame_info(i);
			atomic_dec_if_positive(&frame_info->buffer_count);
			buffer_count = atomic_read(&frame_info->buffer_count);
			if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE))
				val_systrace_c(i, buffer_count, "buffer_count", buffer_count_id);
		}
	}
}
EXPORT_SYMBOL_GPL(update_frame_group_buffer_count);

extern struct reciprocal_value reciprocal_value(u32 d);
struct reciprocal_value schedtune_spc_rdiv;
static long schedtune_margin(unsigned long util, long boost)
{
	long long margin = 0;

	/*
	 * Signal proportional compensation (SPC)
	 *
	 * The Boost (B) value is used to compute a Margin (M) which is
	 * proportional to the complement of the original Signal (S):
	 *   M = B * (SCHED_CAPACITY_SCALE - S)
	 * The obtained M could be used by the caller to "boost" S.
	 */
	if (boost >= 0) {
		margin = SCHED_CAPACITY_SCALE - util;
		margin *= boost;
	} else
		margin = -util * boost;

	margin = reciprocal_divide(margin, schedtune_spc_rdiv);

	if (boost < 0)
		margin *= -1;

	return margin;
}

static long schedtune_grp_margin(unsigned long util, int stune_boost_pct)
{
	if (stune_boost_pct == 0 || util == 0)
		return 0;

	return schedtune_margin(util, stune_boost_pct);
}

static struct oplus_sched_cluster *best_cluster(struct frame_group *grp)
{
	int cpu;
	unsigned long max_cap = 0, cap = 0, best_cap = 0;
	struct oplus_sched_cluster *cluster = NULL, *max_cluster = NULL, *best_cluster = NULL;
	unsigned long util = atomic64_read(&grp->policy_util);
	unsigned long boosted_grp_util = util;
	long boosted_margin_util = 0;

	if (is_multi_frame_fbg(grp->id)) {
		if (util >  grp->stune_boost[BOOST_UTIL_MIN_THRESHOLD])
			boosted_margin_util = schedtune_grp_margin(util, grp->stune_boost[BOOST_DEF_MIGR]);
		boosted_grp_util += boosted_margin_util;

		if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE))
			val_systrace_c(grp->id, boosted_margin_util ? grp->stune_boost[BOOST_DEF_MIGR] : 0, "cfs_boost_migr", cfs_boost_migr);
	}

	for_each_sched_cluster(cluster) {
		cpu = cpumask_first(&cluster->cpus);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
		cap = real_cpu_cap[cpu];
#else
		cap = capacity_orig_of(cpu);
#endif
		/* We sort cluster list by using arch_scale_cpu_capacity() when
		 * build_clusters(). But here we consider freqlimit case and use
		 * capacity_orig_of() to find the max cluster
		 */
		if (cap > max_cap) {
			max_cap = cap;
			max_cluster = cluster;
		}

		if (boosted_grp_util <= cap) {
			best_cap = cap;
			best_cluster = cluster;
			break;
		}
	}

	if (!best_cluster) {
		best_cap = max_cap;
		best_cluster = max_cluster;
	}

	/* We hope to spread frame group task, if preferred_cluster has only
	 * one core and platform has 3 clusters, try to find available_cluster
	 */
	if (num_sched_clusters <= 2) {
		grp->available_cluster = NULL;
	} else {
		if (fb_cluster[num_sched_clusters-1]->id == best_cluster->id) {
			/* if best_cluster is cpu7, then available_cluster is cpu4-6 */
			grp->available_cluster = fb_cluster[num_sched_clusters-2];

		} else if (fb_cluster[num_sched_clusters-2]->id == best_cluster->id) {
			/* if best_cluster is cpu4-6, then available_cluster is cpu7 */
			grp->available_cluster = fb_cluster[num_sched_clusters-1];

		} else if (fb_cluster[num_sched_clusters-3]->id == best_cluster->id) {
			/* if best_cluster is cpu0-3, then available_cluster is cpu4~6 */
			grp->available_cluster = fb_cluster[num_sched_clusters-2];
		}
	}

	if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE)) {
		pref_cpus_systrace_c(grp->id, cpumask_first(&best_cluster->cpus));
		if (grp->available_cluster)
			avai_cpus_systrace_c(grp->id, cpumask_first(&grp->available_cluster->cpus));
		else
			avai_cpus_systrace_c(grp->id, 0);
	}

	/* Now we get preferred_cluster */
	return best_cluster;
}

static unsigned long update_freq_policy_util(struct frame_group *grp, u64 wallclock,
	unsigned int flags)
{
	unsigned long prev_putil = 0, curr_putil = 0, vutil = 0, frame_util = 0;
	u64 timeline;
	bool use_vutil = true;
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
#else
	u64 check_timeline = 0;
#endif
	int avai_buffer_count = 0;

	lockdep_assert_held(&grp->lock);
	update_frame_zone(grp, wallclock);

	if (!(grp->frame_zone & FRAME_ZONE))
		return 0;

	prev_putil = get_frame_putil(grp->id, grp->prev_window_scale, grp->frame_zone);
	curr_putil = get_frame_putil(grp->id, grp->curr_window_scale, grp->frame_zone);
	atomic64_set(&grp->curr_util, curr_putil);
	frame_util = max_t(unsigned long, prev_putil, curr_putil);

	/* We allow vendor governor's freq-query using vutil, but we only updating
	 * last_util_update_time when called from new hook update_curr()
	 */
	/* if (flags & SCHED_CPUFREQ_DEF_FRAMEBOOST || flags & SCHED_CPUFREQ_SF_FRAMEBOOST)
	 * atomic64_set(&grp->last_util_update_time, wallclock);
	 */
	/* For rt_uinon_group, vutil is useless */
	if (grp == frame_boost_groups[SF_FRAME_GROUP_ID]) {
		use_vutil = false;
		goto unused_vutil;
	}

	timeline = wallclock - grp->window_start;

	if (grp->curr_end_time > 0 && grp->curr_end_exec > 0) {
		if ((grp->curr_window_exec - grp->curr_end_exec) > ((wallclock - grp->curr_end_time) >> 2))
			grp->handler_busy = true;
		else
			grp->handler_busy = false;
	}

	if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE)) {
		val_systrace_c(grp->id, grp->handler_busy, "handler_busy", handler_busy);
		val_systrace_c(grp->id, (grp->curr_end_exec > 0) ? (grp->curr_window_exec - grp->curr_end_exec) : 0,
				"delta_end_exec", delta_end_exec);
		val_systrace_c(grp->id, (grp->curr_end_time > 0) ? (wallclock - grp->curr_end_time) : 0,
				"delta_end_time", delta_end_time);
	}
	vutil = get_frame_vutil(grp->id, timeline, grp->handler_busy, &avai_buffer_count);

#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	if ((frame_util >= vutil) ||
		(avai_buffer_count >= 3) ||
		((avai_buffer_count == 2) && (curr_putil < prev_putil || prev_putil < use_vutil_threshold) && (curr_putil < (vutil >> 1))) ||
		((avai_buffer_count == 1 && curr_putil < prev_putil) || prev_putil < use_vutil_threshold))
		use_vutil = false;
#else
	/* Be carefully using vtuil */
	if (grp->frame_zone & FRAME_ZONE && grp->frame_zone & USER_ZONE) {
		if (is_high_frame_rate(grp->id))
			check_timeline = grp->window_size - (grp->window_size >> 3);
		else
			check_timeline = grp->window_size - (grp->window_size >> 2);
	}

	if (timeline > check_timeline && curr_putil < (vutil >> 1)) {
		use_vutil = false;
	}
#endif /* CONFIG_OPLUS_SYSTEM_KERNEL_QCOM */

unused_vutil:
	if (use_vutil)
		frame_util = vutil;

	if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE) && (grp != frame_boost_groups[SF_FRAME_GROUP_ID])) {
			val_systrace_c(grp->id, vutil, "vutil", vutil_id);
			val_systrace_c(grp->id, use_vutil, "use_vutil", use_vutil_id);
	}

	if (unlikely(sysctl_frame_boost_debug & DEBUG_FTRACE)) {
		trace_printk("flags=%u wallclock=%llu window_start=%llu timeline=%llu prev_putil=%lu curr_util=%lu(curr_exec_util=%lu) vutil=%lu use_vutil=%d grp=%d\n",
			flags, wallclock, grp->window_start, timeline, prev_putil, curr_putil,
			get_frame_putil(grp->id, grp->curr_window_exec, grp->frame_zone), vutil, use_vutil, grp->id);
	}

	return frame_uclamp(grp->id, frame_util);
}

void fbg_get_frame_scale(unsigned long *frame_scale)
{
	struct frame_group *grp = frame_boost_groups[GAME_FRAME_GROUP_ID];

	*frame_scale = grp->prev_window_scale;
}
EXPORT_SYMBOL_GPL(fbg_get_frame_scale);

void fbg_get_frame_busy(unsigned int *frame_busy)
{
	struct frame_group *grp = frame_boost_groups[GAME_FRAME_GROUP_ID];

	*frame_busy = grp->window_busy;
}
EXPORT_SYMBOL_GPL(fbg_get_frame_busy);

/* TODO for each group */
void fbg_get_prev_util(unsigned long *prev_util)
{
	struct frame_group *grp = frame_boost_groups[DEFAULT_FRAME_GROUP_ID];

	if (!grp->frame_zone)
		return;

	*prev_util = get_frame_putil(grp->id, grp->prev_window_scale, grp->frame_zone);
}
EXPORT_SYMBOL_GPL(fbg_get_prev_util);

void fbg_get_curr_util(unsigned long *curr_util)
{
	struct frame_group *grp = frame_boost_groups[DEFAULT_FRAME_GROUP_ID];

	if (!grp->frame_zone)
		return;

	*curr_util = get_frame_putil(grp->id, grp->curr_window_scale, grp->frame_zone);
}
EXPORT_SYMBOL_GPL(fbg_get_curr_util);

void fbg_set_end_exec(int grp_id)
{
	unsigned long flags;
	struct frame_group *grp = frame_boost_groups[grp_id];

	raw_spin_lock_irqsave(&grp->lock, flags);
	grp->curr_end_exec = grp->curr_window_exec;
	grp->curr_end_time = atomic64_read(&grp->last_util_update_time);
	raw_spin_unlock_irqrestore(&grp->lock, flags);
}
EXPORT_SYMBOL_GPL(fbg_set_end_exec);

bool check_putil_over_thresh(int grp_id, unsigned long thresh)
{
	struct frame_group *grp = frame_boost_groups[grp_id];
	unsigned long putil = 0;

	putil = get_frame_putil(grp->id, grp->curr_window_scale, FRAME_ZONE);
	return putil >= thresh;
}
EXPORT_SYMBOL_GPL(check_putil_over_thresh);

static bool valid_freq_querys(const struct cpumask *query_cpus, struct frame_group *grp)
{
	int count = 0;
	struct task_struct *p = NULL;
#if (0)
	cpumask_t grp_cpus = CPU_MASK_NONE;
#endif
	cpumask_t on_cpus = CPU_MASK_NONE;
	struct rq *rq;
	struct oplus_task_struct *ots = NULL;
	u64 now = fbg_ktime_get_ns();
	int cpu;

	lockdep_assert_held(&grp->lock);

	if (list_empty(&grp->tasks))
		return false;

	if ((now - atomic64_read(&grp->last_util_update_time)) >= (2 * grp->window_size))
		return false;

#if (0)
	cpumask_copy(&grp_cpus, &grp->preferred_cluster->cpus);
	if (grp->available_cluster)
		cpumask_or(&grp_cpus, &grp_cpus, &grp->available_cluster->cpus);

	if (!cpumask_intersects(query_cpus, &grp_cpus))
		return false;
#endif

	/* Make sure our group task is running on query_cpus now,
	 * otherwise we don't need to update freq.
	 */
	list_for_each_entry(ots, &grp->tasks, fbg_list) {
		p = ots_to_ts(ots);

		cpu = task_cpu(p);
		rq = cpu_rq(cpu);
		if (task_on_cpu(rq, p))
			cpumask_set_cpu(task_cpu(p), &on_cpus);

		if (count > 900) {
			ofb_err("line:[%d]: p=%s, pid=%d, prio=%d, fbg_state=%d, cpu=%d\n",
				__LINE__, p->comm, p->pid, p->prio, ots->fbg_state, cpu);
		}
		/* detect infinite loop */
		BUG_ON(++count > 1000);
	}

	return cpumask_intersects(&on_cpus, query_cpus);
}

bool fbg_freq_policy_util(unsigned int policy_flags, const struct cpumask *query_cpus,
	unsigned long *util)
{
	unsigned long flags;
	unsigned long policy_util = 0, raw_util = *util;
	struct frame_group *grp = NULL;
	unsigned long boosted_policy_util = 0;
	u64 wallclock = fbg_ktime_get_ns();
	unsigned int max_grp_id = 0, first_cpu = cpumask_first(query_cpus);
	unsigned int i;
	long boosted_margin_util;

	if (!__frame_boost_enabled())
		return false;

	/* Adjust governor util with multi frame boost group's policy util */
	for (i = MULTI_FBG_ID; i <  MULTI_FBG_ID + MULTI_FBG_NUM; i++) {
		policy_util = 0;
		boosted_policy_util = 0;
		boosted_margin_util = 0;

		grp = frame_boost_groups[i];
		raw_spin_lock_irqsave(&grp->lock, flags);
		if (list_empty(&grp->tasks))
			goto unlock_fbg;
		if (!grp->preferred_cluster || (get_frame_state(i) == FRAME_END && grp->handler_busy == false))
			goto unlock_fbg;

		if (valid_freq_querys(query_cpus, grp)) {
		/* We allow cfs group used vutil, so we should always update vtuil no matter
		 * it's vendor governor's query or frame boost group's query.
		 */
			if (!(policy_flags & SCHED_CPUFREQ_DEF_FRAMEBOOST) &&
				!(policy_flags & SCHED_CPUFREQ_SF_FRAMEBOOST) &&
				!(policy_flags & SCHED_CPUFREQ_IMS_FRAMEBOOST))
				atomic64_set(&grp->policy_util, update_freq_policy_util(grp,
					wallclock, policy_flags));

			policy_util = atomic64_read(&grp->policy_util);

			if (policy_util >  grp->stune_boost[BOOST_UTIL_MIN_THRESHOLD])
				boosted_margin_util = schedtune_grp_margin(policy_util, grp->stune_boost[BOOST_DEF_FREQ]);
			boosted_policy_util = policy_util + boosted_margin_util;
		}
		*util = max(*util, boosted_policy_util);
		if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE)) {
			unsigned long curr_util = policy_util ? atomic64_read(&grp->curr_util) : 0;

			val_systrace_c(grp->id, boosted_policy_util * 10000 + curr_util, "cfs_policy/curr_util", cfs_policy_curr_util);
			val_systrace_c(grp->id, grp->stune_boost[BOOST_DEF_FREQ], "cfs_boost_freq", cfs_boost_freq);
			if (boosted_policy_util == *util)
				max_grp_id = grp->id;
		}

unlock_fbg:
		raw_spin_unlock_irqrestore(&grp->lock, flags);
	}

	/* Adjust governor util with sf_composition_group's policy util */
	grp = frame_boost_groups[SF_FRAME_GROUP_ID];
	raw_spin_lock_irqsave(&grp->lock, flags);

	policy_util = 0;
	boosted_policy_util = 0;
	if (valid_freq_querys(query_cpus, grp)) {
		policy_util = atomic64_read(&grp->policy_util);
		boosted_policy_util = policy_util +
			schedtune_grp_margin(policy_util, grp->stune_boost[BOOST_SF_IN_GPU] ?
				grp->stune_boost[BOOST_SF_FREQ_GPU] : grp->stune_boost[BOOST_SF_FREQ_NONGPU]);
	}

	raw_spin_unlock_irqrestore(&grp->lock, flags);
	*util = max(*util, boosted_policy_util);

	if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE)) {
		int boost_sf;
		unsigned long curr_util = policy_util ? atomic64_read(&grp->curr_util) : 0;
		val_systrace_c(grp->id, boosted_policy_util * 10000 + curr_util, "rt_policy/curr_util", rt_policy_curr_util);
		if (grp->stune_boost[BOOST_SF_IN_GPU])
			boost_sf = grp->stune_boost[BOOST_SF_FREQ_GPU] * 1000 + grp->stune_boost[BOOST_SF_MIGR_GPU];
		else
			boost_sf = grp->stune_boost[BOOST_SF_FREQ_NONGPU] * 1000 + grp->stune_boost[BOOST_SF_MIGR_NONGPU];
		val_systrace_c(grp->id, boost_sf, "rt_boost_freq/migr", rt_boost_freq_migr);
		if (boosted_policy_util == *util)
			max_grp_id = grp->id;
	}

	grp = frame_boost_groups[INPUTMETHOD_FRAME_GROUP_ID];
	raw_spin_lock_irqsave(&grp->lock, flags);

	policy_util = 0;
	if (valid_freq_querys(query_cpus, grp))
		policy_util = atomic64_read(&grp->policy_util);

	raw_spin_unlock_irqrestore(&grp->lock, flags);
	*util = max(*util, policy_util);

	if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE)) {
		val_systrace_c(grp->id, policy_util,  "IMS_policy_util", IMS_policy_util);
		if (policy_util == *util)
			max_grp_id = grp->id;

		if (raw_util == *util)
			max_grp_id = 0;
		max_grp_systrace_c(first_cpu, max_grp_id);
	}

	return (raw_util != *util);
}
EXPORT_SYMBOL_GPL(fbg_freq_policy_util);

static inline bool should_update_cpufreq(u64 wallclock, struct frame_group *grp,
	raw_spinlock_t *lock)
{
	s64 delta = 0;

	lockdep_assert_held(lock);

	if (list_empty(&grp->tasks))
		return false;

	delta = wallclock - atomic64_read(&grp->last_freq_update_time);
	if (delta < DEFAULT_FREQ_UPDATE_MIN_INTERVAL)
		return false;

	return true;
}

static inline void cpufreq_update_util_wrap(struct rq *rq, unsigned int flags)
{
	unsigned long lock_flags;

	raw_spin_lock_irqsave(&freq_protect_lock, lock_flags);
	cpufreq_update_util(rq, flags);
	raw_spin_unlock_irqrestore(&freq_protect_lock, lock_flags);
}

bool default_group_update_cpufreq(int grp_id)
{
	struct frame_group *grp = frame_boost_groups[grp_id];
	unsigned long flags;
	bool ret = false;
	bool need_update_prev_freq = false;
	bool need_update_next_freq = false;
	int prev_cpu, next_cpu;
	struct oplus_sched_cluster *preferred_cluster = NULL;
	struct rq *rq = NULL;
	u64 wallclock = fbg_ktime_get_ns();

	raw_spin_lock_irqsave(&grp->lock, flags);

	if (list_empty(&grp->tasks))
		goto unlock;

	atomic64_set(&grp->policy_util, update_freq_policy_util(grp, wallclock, SCHED_CPUFREQ_DEF_FRAMEBOOST));
	/*
	 *Update frame group preferred cluster before updating cpufreq,
	 * so we can make decision target cluster.
	 */
	preferred_cluster = best_cluster(grp);
	if (!grp->preferred_cluster)
		grp->preferred_cluster = preferred_cluster;
	else if (grp->preferred_cluster != preferred_cluster) {
		prev_cpu = cpumask_first(&grp->preferred_cluster->cpus);
		grp->preferred_cluster = preferred_cluster;
		/*
		 * Once preferred_cluster changed, update prev_cluster's cpufreq without any limit.
		 * And then get_freq_policy_util() will return 0 in this update call.
		 */
		need_update_prev_freq = true;
		ret = true;
	}
	next_cpu = cpumask_first(&grp->preferred_cluster->cpus);

	if (should_update_cpufreq(wallclock, grp, &grp->lock)) {
		atomic64_set(&grp->last_freq_update_time, wallclock);
		need_update_next_freq = true;
	}

unlock:
	raw_spin_unlock_irqrestore(&grp->lock, flags);

	rcu_read_lock();
	if (need_update_prev_freq) {
		rq = cpu_rq(prev_cpu);
		if (fbg_hook.update_freq)
			fbg_hook.update_freq(rq, SCHED_CPUFREQ_DEF_FRAMEBOOST);
		else
			cpufreq_update_util_wrap(rq, SCHED_CPUFREQ_DEF_FRAMEBOOST);
	}

	if (need_update_next_freq) {
		rq = cpu_rq(next_cpu);
		if (fbg_hook.update_freq)
			fbg_hook.update_freq(rq, SCHED_CPUFREQ_DEF_FRAMEBOOST);
		else
			cpufreq_update_util_wrap(rq, SCHED_CPUFREQ_DEF_FRAMEBOOST);
	}
	rcu_read_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(default_group_update_cpufreq);

bool sf_composition_update_cpufreq(struct task_struct *tsk)
{
	struct frame_group *grp = frame_boost_groups[SF_FRAME_GROUP_ID];

	unsigned long flags;
	bool ret = false;
	bool need_update = false;
	struct rq *rq = NULL;
	u64 wallclock = fbg_ktime_get_ns();

	raw_spin_lock_irqsave(&grp->lock, flags);

	if (list_empty(&grp->tasks))
		goto unlock;

	atomic64_set(&grp->policy_util, update_freq_policy_util(grp, wallclock, SCHED_CPUFREQ_SF_FRAMEBOOST));

	if (should_update_cpufreq(wallclock, grp, &grp->lock)) {
		atomic64_set(&grp->last_freq_update_time, wallclock);
		need_update = true;
	}

unlock:
	raw_spin_unlock_irqrestore(&grp->lock, flags);

	if (need_update) {
		rq = task_rq(tsk);
		if (fbg_hook.update_freq)
			fbg_hook.update_freq(rq, SCHED_CPUFREQ_SF_FRAMEBOOST);
		else
			cpufreq_update_util_wrap(rq, SCHED_CPUFREQ_SF_FRAMEBOOST);
	}

	return ret;
}

void input_update_cpufreq(int grp_id, struct task_struct *tsk)
{
	struct rq *rq = NULL;

	struct frame_group *grp = frame_boost_groups[grp_id];
	unsigned long flags;
	u64 wallclock = fbg_ktime_get_ns();

	raw_spin_lock_irqsave(&grp->lock, flags);

	if (list_empty(&grp->tasks)) {
		raw_spin_unlock_irqrestore(&grp->lock, flags);
		return;
	}

	if (grp_id == INPUTMETHOD_FRAME_GROUP_ID) {
		if ((wallclock - atomic64_read(&grp->last_util_update_time)) >= (2 * grp->window_size)) {
			atomic64_set(&grp->policy_util, 0);
			atomic64_set(&grp->last_util_update_time, wallclock);
		}
	}
	grp->preferred_cluster = best_cluster(grp);

	raw_spin_unlock_irqrestore(&grp->lock, flags);

	rq = task_rq(tsk);
	if (fbg_hook.update_freq)
		fbg_hook.update_freq(rq, SCHED_CPUFREQ_IMS_FRAMEBOOST);
	else
		cpufreq_update_util_wrap(rq, SCHED_CPUFREQ_IMS_FRAMEBOOST);
}

void input_set_boost_start(int grp_id)
{
	struct frame_group *grp = frame_boost_groups[grp_id];
	struct task_struct *ui;
	unsigned long flags;
	u64 wallclock = fbg_ktime_get_ns();

	raw_spin_lock_irqsave(&grp->lock, flags);
	if (list_empty(&grp->tasks)) {
		raw_spin_unlock_irqrestore(&grp->lock, flags);
		return;
	}

	if (is_multi_frame_fbg(grp_id)) {
		atomic64_set(&grp->policy_util, frame_uclamp(grp_id, 0));
	} else if (grp_id == INPUTMETHOD_FRAME_GROUP_ID) {
		grp->policy_util = grp->curr_util;
		atomic64_set(&grp->last_util_update_time, wallclock);
	} else {
		raw_spin_unlock_irqrestore(&grp->lock, flags);
		return;
	}

	raw_spin_unlock_irqrestore(&grp->lock, flags);

	rcu_read_lock();
	ui = rcu_dereference(grp->ui);
	if (ui)
		get_task_struct(ui);
	rcu_read_unlock();

	if (ui) {
		input_update_cpufreq(grp_id, ui);
		put_task_struct(ui);
	}
}
EXPORT_SYMBOL_GPL(input_set_boost_start);

/*
 * update_frame_group_util - update frame group utility if the task is drawing frame
 * @task: task that is updating running time.
 * @delta: running time is nano second
 *       the other paramenter is unused
 */
static void update_frame_group_util(struct task_struct *p, u64 running,
	u64 wallclock, struct frame_group *grp)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(p);
	u64 adjusted_running;
	u64 window_start;
	u64 delta_wc_ws;
	u64 prev_exec, exec_scale;
	struct rq *rq = task_rq(p);

	if (IS_ERR_OR_NULL(ots))
		return;

	lockdep_assert_held(&grp->lock);

	window_start = grp->window_start;
	if (unlikely(wallclock < window_start)) {
		ofb_debug("failed to update util with wc=%llu ws=%llu\n",
			wallclock,
			window_start);
		return;
	}

	delta_wc_ws = wallclock - window_start;

	/*
	 * adjust the running time, for serial load track.
	 * only adjust STATIC_FRAME_TASK tasks, not BINDER_FRAME_TASK tasks,
	 * matched with the logic of update_group_nr_running().
	 */
	if (ots->fbg_state & STATIC_FRAME_TASK) {
		if (grp->mark_start <= 0)
			return;

		adjusted_running = wallclock - grp->mark_start;
		if (unlikely(adjusted_running <= 0)) {
			ofb_debug("adjusted_running <= 0 with wc=%llu ms=%llu\n",
				wallclock, grp->mark_start);
			return;
		}

		if (unlikely(sysctl_frame_boost_debug & DEBUG_FTRACE)) {
			trace_printk("raw_running=%llu, adjusted_running=%llu,"
				" old_mark_start=%llu, new_mark_start=%llu\n",
				running, adjusted_running, grp->mark_start, wallclock);
		}

		grp->mark_start = wallclock;
		running = adjusted_running;
	}

	if (running <= 0)
		return;

	/* Per group load tracking in FBG */
	if (likely(delta_wc_ws >= running)) {
		grp->curr_window_exec += running;

		exec_scale = scale_exec_time(running, rq);
		grp->curr_window_scale += exec_scale;
		if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE)) {
			val_systrace_c(grp->id, grp->curr_window_exec, "curr_window_exec", curr_window_exec);
			val_systrace_c(grp->id, get_frame_putil(grp->id, grp->curr_window_scale, grp->frame_zone),
				"curr_window_util", curr_window_util);
		}
	} else {
		/* Prev window group statistic */
		prev_exec = running - delta_wc_ws;
		grp->prev_window_exec += prev_exec;

		exec_scale = scale_exec_time(prev_exec, rq);
		grp->prev_window_scale += exec_scale;

		/* Curr window group statistic */
		grp->curr_window_exec += delta_wc_ws;

		exec_scale = scale_exec_time(delta_wc_ws, rq);
		grp->curr_window_scale += exec_scale;

		if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE)) {
			val_systrace_c(grp->id, grp->prev_window_exec, "prev_window_exec", prev_window_exec);
			val_systrace_c(grp->id, get_frame_putil(grp->id, grp->prev_window_scale, grp->frame_zone), "prev_window_util", prev_window_util);
			val_systrace_c(grp->id, grp->curr_window_exec, "curr_window_exec", curr_window_exec);
			val_systrace_c(grp->id, get_frame_putil(grp->id, grp->curr_window_scale, grp->frame_zone), "curr_window_util", curr_window_util);
		}
	}

	atomic64_set(&grp->last_util_update_time, wallclock);
}

static inline void fbg_update_task_util(struct task_struct *tsk, u64 runtime,
	bool need_freq_update)
{
	struct frame_group *grp = NULL;
	struct oplus_task_struct *ots = NULL;
	unsigned long flags;
	u64 wallclock;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots) || (ots->fbg_state == NONE_FRAME_TASK) || (ots->fbg_state & PENDING_FRAME_TASK))
		return;

	grp = task_get_frame_group(ots);
	if (grp == NULL)
		return;
	if (ots->fbg_state & FRAME_INPUTMETHOD)
		goto skip;

	raw_spin_lock_irqsave(&grp->lock, flags);
	/* When task update running time, doing following works:
	 * 1) update frame group util;
	 * 2) update frame group's frame zone;
	 * 3) try to update cpufreq.
	 */
	wallclock = fbg_ktime_get_ns();
	update_frame_group_util(tsk, runtime, wallclock, grp);

	raw_spin_unlock_irqrestore(&grp->lock, flags);

skip:
	if (need_freq_update) {
		if (grp->id == SF_FRAME_GROUP_ID)
			sf_composition_update_cpufreq(tsk);
		else if (grp->id == INPUTMETHOD_FRAME_GROUP_ID)
			input_update_cpufreq(grp->id, tsk);
		else if (is_multi_frame_fbg(grp->id))
			default_group_update_cpufreq(grp->id);
	}
}

static void fbg_update_task_util_hook(void *unused, struct task_struct *tsk,
	u64 runtime, u64 vruntime)
{
	fbg_update_task_util(tsk, runtime, true);
}

enum task_event {
	PUT_PREV_TASK	= 0,
	PICK_NEXT_TASK	= 1,
};

/*
 * Update the number of running threads in the group.
 *
 * If thread belonging to a group start running, nr_running of group +1.
 * If thread belonging to a group stop running, nr_running of group -1.
 *
 * We only consider STATIC_FRAME_TASK tasks, not BINDER_FRAME_TASK tasks,
 * because same BINDER_FRAME_TASK tasks bouncing between different group.
 *
 * When nr_running form 0 to 1, the mark_start set to the current time.
 */
static void update_group_nr_running(struct task_struct *p, int event)
{
	struct oplus_task_struct *ots = NULL;
	struct frame_group *grp = NULL;
	unsigned long flags;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	grp = task_get_frame_group(ots);
	if (grp == NULL)
		return;

	raw_spin_lock_irqsave(&grp->lock, flags);
	if (event == PICK_NEXT_TASK && (ots->fbg_state & STATIC_FRAME_TASK)) {
		ots->fbg_running = true;
		grp->nr_running++;
		if (grp->nr_running == 1)
			grp->mark_start = max(grp->mark_start, fbg_ktime_get_ns());
		if (unlikely(sysctl_frame_boost_debug & DEBUG_FTRACE)) {
			trace_printk("grp->id=%d, next->comm=%s, next->tid=%d, nr_running=%d, mark_start=%llu\n",
				grp->id, p->comm, p->pid, grp->nr_running, grp->mark_start);
		}
	} else if (event == PUT_PREV_TASK && ots->fbg_running) {
		ots->fbg_running = false;
		grp->nr_running--;
		if (unlikely(grp->nr_running < 0))
			grp->nr_running = 0;
		if (unlikely(sysctl_frame_boost_debug & DEBUG_FTRACE)) {
			trace_printk("grp->id=%d, prev->comm=%s, prev->tid=%d, nr_running=%d\n",
				grp->id, p->comm, p->pid, grp->nr_running);
		}
	}
	raw_spin_unlock_irqrestore(&grp->lock, flags);
}

void fbg_set_group_policy_util(int grp_id, int min_util)
{
	unsigned long flags;
	struct frame_group *grp;

	if (min_util < 0)
		min_util = 0;

	grp = frame_boost_groups[grp_id];

	raw_spin_lock_irqsave(&grp->lock, flags);
	/* Store group util from userspace setting to curr_util*/
	atomic64_set(&grp->curr_util, min_util);
	raw_spin_unlock_irqrestore(&grp->lock, flags);
}
EXPORT_SYMBOL_GPL(fbg_set_group_policy_util);

void fbg_android_rvh_schedule_handler(struct task_struct *prev,
	struct task_struct *next, struct rq *rq)
{
	struct oplus_task_struct *ots;

	if (unlikely(sysctl_frame_boost_debug & DEBUG_SYSTRACE) && likely(prev != next)) {
		ots = get_oplus_task_struct(next);
		if (ots)
			cpu_val_systrace_c(ots->fbg_state, cpu_of(rq), "fbg_state", fbg_state);
	}

	if (atomic_read(&fbg_initialized) == 0)
		return;

	if (unlikely(prev == next))
		return;

	/* prev task */
	update_group_nr_running(prev, PUT_PREV_TASK);
	/* next task */
	update_group_nr_running(next, PICK_NEXT_TASK);
}
EXPORT_SYMBOL_GPL(fbg_android_rvh_schedule_handler);

void fbg_android_rvh_cpufreq_transition(struct cpufreq_policy *policy)
{
	struct task_struct *curr_task;
	struct rq *rq;
	int cpu;

	if (atomic_read(&fbg_initialized) == 0)
		return;

	for_each_cpu(cpu, policy->cpus) {
		rq = cpu_rq(cpu);

		rcu_read_lock();
		curr_task = rcu_dereference(rq->curr);
		if (curr_task)
			get_task_struct(curr_task);
		rcu_read_unlock();

		if (curr_task) {
			fbg_update_task_util(curr_task, 0, false);
			put_task_struct(curr_task);
		}
	}
}
EXPORT_SYMBOL_GPL(fbg_android_rvh_cpufreq_transition);

static void update_util_before_window_rollover(int group_id)
{
	struct oplus_sched_cluster *cluster;
	struct task_struct *curr_task;
	struct rq *rq;
	int cpu;

	if (group_id != GAME_FRAME_GROUP_ID)
		return;

	for_each_sched_cluster(cluster) {
		for_each_cpu(cpu, &cluster->cpus) {
			rq = cpu_rq(cpu);

			rcu_read_lock();
			curr_task = rcu_dereference(rq->curr);
			if (curr_task)
				get_task_struct(curr_task);
			rcu_read_unlock();

			if (curr_task) {
				fbg_update_task_util(curr_task, 0, false);
				put_task_struct(curr_task);
			}
		}
	}
}

/*********************************
 * task placement for frame group
 *********************************/
/* If task util arrive (max * 80%), it's misfit */
#define fits_capacity(util, max)	((util) * 1280 < (max) * 1024)

/*
 * group_task_fits_cluster_cpus - check if frame group preferred cluster is suitable for
 *             frame task
 *
 * We should know that our preferred cluster comes from util-tracing with frame window,
 * which may not fit original load-tracing with larger window size.
 */
static bool group_task_fits_cluster_cpus(struct task_struct *tsk,
	struct oplus_sched_cluster *cluster)
{
	/* If group task prefer silver core, just let it go */
	if (!cluster || !cluster->id)
		return false;

	return true;
}
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
#ifdef CONFIG_HAVE_SCHED_AVG_IRQ
static inline unsigned long real_cpu_util_irq(struct rq *rq)
{
	return rq->avg_irq.util_avg;
}

	static inline
unsigned long real_scale_irq_capacity(unsigned long util, unsigned long irq, unsigned long max)
{
	util *= (max - irq);
	util /= max;

	return util;
}
#else
static inline unsigned long real_cpu_util_irq(struct rq *rq)
{
	return 0;
}

	static inline
unsigned long real_scale_irq_capacity(unsigned long util, unsigned long irq, unsigned long max)
{
	return util;
}
#endif

#ifdef CONFIG_SCHED_THERMAL_PRESSURE
static inline u64 real_thermal_load_avg(struct rq *rq)
{
		return READ_ONCE(rq->avg_thermal.load_avg);
}
#else
static inline u64 real_thermal_load_avg(struct rq *rq)
{
		return 0;
}
#endif
static unsigned long real_scale_rt_capacity(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long max = arch_scale_cpu_capacity(cpu);
	unsigned long used, free;
	unsigned long irq;

	irq = real_cpu_util_irq(rq);

	if (unlikely(irq >= max))
		return 1;

	/*
	 * avg_rt.util_avg and avg_dl.util_avg track binary signals
	 * (running and not running) with weights 0 and 1024 respectively.
	 * avg_thermal.load_avg tracks thermal pressure and the weighted
	 * average uses the actual delta max capacity(load).
	 */
	used = READ_ONCE(rq->avg_rt.util_avg);
	used += READ_ONCE(rq->avg_dl.util_avg);
	used += real_thermal_load_avg(rq);

	if (unlikely(used >= max))
		return 1;

	free = max - used;

	return real_scale_irq_capacity(free, irq, max);
}
#endif
inline unsigned long capacity_of(int cpu)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
	if (eas_opt_enable && !force_apply_ocap_enable)
		return real_scale_rt_capacity(cpu);
	else
		return cpu_rq(cpu)->cpu_capacity;
#else
	return cpu_rq(cpu)->cpu_capacity;
#endif
}

#define lsub_positive(_ptr, _val) do {				\
	typeof(_ptr) ptr = (_ptr);				\
	*ptr -= min_t(typeof(*ptr), *ptr, _val);		\
} while (0)

static inline unsigned long cpu_util(int cpu)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	if (sched_feat(UTIL_EST))
		util = max(util, READ_ONCE(cfs_rq->avg.util_est.enqueued));

	return min_t(unsigned long, util, capacity_orig_of(cpu));
}

static inline unsigned long task_util(struct task_struct *p)
{
	return READ_ONCE(p->se.avg.util_avg);
}

static inline unsigned long _task_util_est(struct task_struct *p)
{
	struct util_est ue = READ_ONCE(p->se.avg.util_est);

	return max(ue.ewma, (ue.enqueued & ~UTIL_AVG_UNCHANGED));
}

unsigned long cpu_util_without(int cpu, struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	/* Task has no contribution or is new */
	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		return cpu_util(cpu);

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	/* Discount task's util from CPU's util */
	lsub_positive(&util, task_util(p));

	/*
	 * Covered cases:
	 *
	 * a) if *p is the only task sleeping on this CPU, then:
	 *      cpu_util (== task_util) > util_est (== 0)
	 *    and thus we return:
	 *      cpu_util_without = (cpu_util - task_util) = 0
	 *
	 * b) if other tasks are SLEEPING on this CPU, which is now exiting
	 *    IDLE, then:
	 *      cpu_util >= task_util
	 *      cpu_util > util_est (== 0)
	 *    and thus we discount *p's blocked utilization to return:
	 *      cpu_util_without = (cpu_util - task_util) >= 0
	 *
	 * c) if other tasks are RUNNABLE on that CPU and
	 *      util_est > cpu_util
	 *    then we use util_est since it returns a more restrictive
	 *    estimation of the spare capacity on that CPU, by just
	 *    considering the expected utilization of tasks already
	 *    runnable on that CPU.
	 *
	 * Cases a) and b) are covered by the above code, while case c) is
	 * covered by the following code when estimated utilization is
	 * enabled.
	 */
	if (sched_feat(UTIL_EST)) {
		unsigned int estimated =
			READ_ONCE(cfs_rq->avg.util_est.enqueued);

		/*
		 * Despite the following checks we still have a small window
		 * for a possible race, when an execl's select_task_rq_fair()
		 * races with LB's detach_task():
		 *
		 *   detach_task()
		 *     p->on_rq = TASK_ON_RQ_MIGRATING;
		 *     ---------------------------------- A
		 *     deactivate_task()                   \
		 *       dequeue_task()                     + RaceTime
		 *         util_est_dequeue()              /
		 *     ---------------------------------- B
		 *
		 * The additional check on "current == p" it's required to
		 * properly fix the execl regression and it helps in further
		 * reducing the chances for the above race.
		 */
		if (unlikely(task_on_rq_queued(p) || current == p))
			lsub_positive(&estimated, _task_util_est(p));

		util = max(util, estimated);
	}

	/*
	 * Utilization (estimated) can exceed the CPU capacity, thus let's
	 * clamp to the maximum CPU capacity to ensure consistency with
	 * the cpu_util call.
	 */
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
	return min_t(unsigned long, util, real_cpu_cap[cpu]);
#else
	return min_t(unsigned long, util, capacity_orig_of(cpu));
#endif
}

static bool task_is_rt(struct task_struct *task)
{
	BUG_ON(!task);

	/* valid RT priority is 0..MAX_RT_PRIO-1 */
	if ((task->prio <= MAX_RT_PRIO-1) && (task->prio >= 0))
		return true;

	return false;
}

bool set_frame_group_task_to_perfer_cpu(struct task_struct *p, int *target_cpu)
{
	int iter_cpu = 0;
	bool walk_next_cls = false;
	struct oplus_sched_cluster *cluster = NULL;
	cpumask_t search_cpus = CPU_MASK_NONE;
	long spare_cap = 0;
	long max_spare_cap = -1;
	int max_spare_cap_cpu = -1, backup_cpu = -1;
	struct frame_group *grp = NULL;
	struct oplus_task_struct *ots = get_oplus_task_struct(p);
	cpumask_t available_cpus = CPU_MASK_NONE;

	if (IS_ERR_OR_NULL(ots))
		return false;

	if (!__frame_boost_enabled())
		return false;

	/* The interface is currently offered for use in games. */
	cluster = fbg_get_task_preferred_cluster(p);
	if (cluster == NULL) {
		/* Some threads created before moduler working, just init them here. */
		if (ots->fbg_list.prev == 0 && ots->fbg_list.next == 0) {
			ots->fbg_state = NONE_FRAME_TASK;
			ots->fbg_depth = INVALID_FBG_DEPTH;
			INIT_LIST_HEAD(&ots->fbg_list);
		}

		if (!ots->fbg_state || (ots->fbg_state & PENDING_FRAME_TASK))
			return false;

		grp = task_get_frame_group(ots);
		if (grp == NULL)
			return false;

		if ((grp->id != INPUTMETHOD_FRAME_GROUP_ID) && !is_multi_frame_fbg(grp->id))
			return false;

		if (!group_task_fits_cluster_cpus(p, grp->preferred_cluster))
			return false;

		cluster = grp->preferred_cluster;
	}

retry:
	cpumask_and(&search_cpus, p->cpus_ptr, cpu_active_mask);
#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
	if (fbg_cpu_halt_mask)
		cpumask_andnot(&search_cpus, &search_cpus, fbg_cpu_halt_mask);
#endif /* CONFIG_OPLUS_ADD_CORE_CTRL_MASK */
	cpumask_and(&search_cpus, &search_cpus, &cluster->cpus);

	/* In case preferred_cluster->cpus are inactive, give it a try to walk_next_cls */
	if ((grp != NULL) && (cluster == grp->preferred_cluster))
		walk_next_cls = true;

	for_each_cpu(iter_cpu, &search_cpus) {
		struct rq *rq = NULL;
		struct oplus_rq *orq = NULL;
		struct task_struct *curr = NULL;

		rq = cpu_rq(iter_cpu);
		curr = rq->curr;
		orq = get_oplus_rq(rq);

		if (curr) {
			struct oplus_task_struct *ots_curr =
				get_oplus_task_struct(curr);

			/* Avoid puting group task on the same cpu */
			if (!IS_ERR_OR_NULL(ots_curr) && ots_curr->fbg_state && !(ots_curr->fbg_state & PENDING_FRAME_TASK)) {
				if ((backup_cpu == -1) && task_is_rt(curr)) {
					backup_cpu = iter_cpu;
					walk_next_cls = false;
				}
				continue;
			}

			/* If an ux amd rt thread running on this CPU, drop it! */
			if (oplus_get_ux_state(rq->curr) & SCHED_ASSIST_UX_MASK)
				continue;

			if (rq->curr->prio < MAX_RT_PRIO)
				continue;

			/* If there are ux and rt threads in runnable state on this CPU, drop it! */
			if (orq_has_ux_tasks(orq))
				continue;

			if (rt_rq_is_runnable(&rq->rt))
				continue;
		}


		backup_cpu = iter_cpu;
		walk_next_cls = false;

		if (available_idle_cpu(iter_cpu)
			|| (iter_cpu == task_cpu(p) && p->__state == TASK_RUNNING)) {
			if (grp != NULL && grp->available_cluster)
				available_cpus = grp->available_cluster->cpus;

			trace_find_frame_boost_cpu(p, &search_cpus,
				&cluster->cpus, &available_cpus,
				"idle_backup", *target_cpu, iter_cpu);
			*target_cpu = iter_cpu;

			return true;
		}
		spare_cap =
		    max_t(long,
			  capacity_of(iter_cpu) - cpu_util_without(iter_cpu, p),
			  0);
		if (spare_cap > max_spare_cap) {
			max_spare_cap = spare_cap;
			max_spare_cap_cpu = iter_cpu;
		}
	}

	if (max_spare_cap_cpu != -1) {
		if (grp != NULL && grp->available_cluster)
			available_cpus = grp->available_cluster->cpus;

		trace_find_frame_boost_cpu(p, &search_cpus,
			&cluster->cpus, &available_cpus,
			"max_spare", *target_cpu,
			max_spare_cap_cpu);
		*target_cpu = max_spare_cap_cpu;

		return true;
	} else if (!walk_next_cls && backup_cpu != -1) {
		if (grp != NULL && grp->available_cluster)
			available_cpus = grp->available_cluster->cpus;

		trace_find_frame_boost_cpu(p, &search_cpus,
			&cluster->cpus, &available_cpus,
			"backup_cpu", *target_cpu,
			backup_cpu);
		*target_cpu = backup_cpu;

		return true;
	}

	if (walk_next_cls && grp != NULL && grp->available_cluster) {
		cluster = grp->available_cluster;
		cpumask_clear(&search_cpus);
		walk_next_cls = false;
		goto retry;
	}

	return false;
}

EXPORT_SYMBOL_GPL(set_frame_group_task_to_perfer_cpu);

/*
 * fbg_need_up_migration - check if frame group task @p fits this cpu @rq
 *
 * This function is only used for default_frame_boost_group.
 */
bool fbg_need_up_migration(struct task_struct *p, struct rq *rq)
{
	unsigned long cpu_capacity = capacity_orig_of(cpu_of(rq));
	struct frame_group *grp = NULL;
	struct oplus_sched_cluster *cluster = NULL;
	struct oplus_task_struct *ots = NULL;
	unsigned long flags;

	if (!__frame_boost_enabled())
		return false;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots) || !ots->fbg_state || ots->fbg_state & (FRAME_COMPOSITION | FRAME_GAME | FRAME_INPUTMETHOD | FRAME_PENDING))
		return false;

	grp = task_get_frame_group(ots);
	if (grp == NULL)
		return false;

	raw_spin_lock_irqsave(&grp->lock, flags);
	cluster = grp->preferred_cluster;
	raw_spin_unlock_irqrestore(&grp->lock, flags);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
	cpu_capacity = real_cpu_cap[cpu_of(rq)];
	return group_task_fits_cluster_cpus(p, cluster) &&
			(cpu_capacity < real_cpu_cap[cpumask_first(&cluster->cpus)]);
#else
	return group_task_fits_cluster_cpus(p, cluster) &&
		(cpu_capacity < capacity_orig_of(cpumask_first(&cluster->cpus)));
#endif
}
EXPORT_SYMBOL_GPL(fbg_need_up_migration);

/*
 * fbg_skip_migration - check if frame group task @p can be migrated from src_cpu
 *             to dst_cpu
 *
 * This function is only used for default_frame_boost_group.
 */
bool fbg_skip_migration(struct task_struct *tsk, int src_cpu, int dst_cpu)
{
	struct oplus_sched_cluster *cluster = NULL;
	struct frame_group *grp = NULL;
	struct oplus_task_struct *ots = NULL, *dst_ots = NULL;
	struct rq *dst_rq = cpu_rq(dst_cpu);
	unsigned long flags;

	if (!__frame_boost_enabled())
		return false;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots) || !ots->fbg_state || ots->fbg_state & (FRAME_COMPOSITION | FRAME_GAME | FRAME_INPUTMETHOD | FRAME_PENDING))
		return false;

	dst_ots = get_oplus_task_struct(dst_rq->curr);
	if (!IS_ERR_OR_NULL(dst_ots) && dst_ots->fbg_state && !(dst_ots->fbg_state & PENDING_FRAME_TASK))
		return true;

	grp = task_get_frame_group(ots);
	if (grp == NULL)
		return false;

	raw_spin_lock_irqsave(&grp->lock, flags);
	cluster = grp->preferred_cluster;
	raw_spin_unlock_irqrestore(&grp->lock, flags);

	if (!group_task_fits_cluster_cpus(tsk, cluster))
		return false;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
	return real_cpu_cap[dst_cpu] < real_cpu_cap[cpumask_first(&cluster->cpus)];
#else
	return capacity_orig_of(dst_cpu) < capacity_orig_of(cpumask_first(&cluster->cpus));
#endif
}
EXPORT_SYMBOL_GPL(fbg_skip_migration);

bool fbg_rt_task_fits_capacity(struct task_struct *tsk, int cpu)
{
	struct oplus_task_struct *ots = NULL;
	struct frame_group *grp = NULL;
	unsigned long grp_util = 0, raw_util = 0;
	bool fits = true;
	u64 now = fbg_ktime_get_ns();

	if (!__frame_boost_enabled())
		return true;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots))
		return true;

	/* Some threads created before moduler working, just init them here. */
	if (ots->fbg_list.prev == 0 && ots->fbg_list.next == 0) {
		ots->fbg_state = NONE_FRAME_TASK;
		ots->fbg_depth = INVALID_FBG_DEPTH;
		INIT_LIST_HEAD(&ots->fbg_list);
	}

	if (!(ots->fbg_state & FRAME_COMPOSITION))
		return true;

	grp = frame_boost_groups[SF_FRAME_GROUP_ID];
	if (!grp->frame_zone || (now - atomic64_read(&grp->last_util_update_time)) >= (2 * grp->window_size))
		return true;

	raw_util = atomic64_read(&grp->policy_util);
	grp_util = raw_util + schedtune_grp_margin(raw_util, grp->stune_boost[BOOST_SF_IN_GPU] ?
			grp->stune_boost[BOOST_SF_MIGR_GPU] : grp->stune_boost[BOOST_SF_MIGR_NONGPU]);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
	fits = real_cpu_cap[cpu] >= grp_util;
#else
	fits = capacity_orig_of(cpu) >= grp_util;
#endif

	if (unlikely(sysctl_frame_boost_debug & DEBUG_FTRACE))
		trace_printk("comm=%-12s pid=%d tgid=%d cpu=%d grp_util=%lu raw_util=%lu cpu_cap=%lu fits=%d\n",
			tsk->comm, tsk->pid, tsk->tgid, cpu, grp_util, raw_util,
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
			real_cpu_cap[cpu], fits);
#else
			capacity_orig_of(cpu), fits);
#endif

	return fits;
}
EXPORT_SYMBOL_GPL(fbg_rt_task_fits_capacity);

bool fbg_skip_rt_sync(struct rq *rq, struct task_struct *p, bool *sync)
{
	int cpu = cpu_of(rq);

	if (*sync && !fbg_rt_task_fits_capacity(p, cpu)) {
		*sync = false;
		return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(fbg_skip_rt_sync);

/*********************************
 * frame group initialize
 *********************************/
static void fbg_flush_task_hook(void *unused, struct task_struct *tsk)
{
	struct task_struct *task_removed;
	struct task_struct *binder_removed;
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);
	unsigned long flags;
	raw_spinlock_t *lock = NULL;

	if (IS_ERR_OR_NULL(ots))
		return;

	lock = task_get_frame_group_lock(ots);
	if (lock == NULL)
		return;

	raw_spin_lock_irqsave(lock, flags);
	/* game group task also removed here */
	task_removed = remove_task_from_frame_group(tsk);
	binder_removed = remove_binder_from_frame_group(tsk);
	raw_spin_unlock_irqrestore(lock, flags);
	if (task_removed) {
		put_task_struct(task_removed);
	}
	if (binder_removed) {
		put_task_struct(binder_removed);
	}
}

static void fbg_sched_fork_hook(void *unused, struct task_struct *tsk)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);

	if (IS_ERR_OR_NULL(ots))
		return;

	ots->fbg_state = NONE_FRAME_TASK;
	ots->fbg_depth = INVALID_FBG_DEPTH;
	ots->fbg_running = false;
	ots->preferred_cluster_id = -1;
	ots->fbg_cur_group = 0;
	INIT_LIST_HEAD(&ots->fbg_list);
	raw_spin_lock_init(&ots->fbg_list_entry_lock);
}

void fbg_add_update_freq_hook(void (*func)(struct rq *rq, unsigned int flags))
{
	if (fbg_hook.update_freq == NULL)
		fbg_hook.update_freq = func;
}
EXPORT_SYMBOL_GPL(fbg_add_update_freq_hook);

void fbg_game_set_ed_info(int ed_duration, int ed_user_pid)
{
	struct task_struct *ed_user_task;

	game_ed_duration = ed_duration;

	if (game_ed_user_pid != ed_user_pid) {
		raw_spin_lock(&game_ed_lock);
		if (game_ed_user_task) {
			put_task_struct(game_ed_user_task);
			game_ed_user_task = NULL;
		}

		rcu_read_lock();
		ed_user_task = find_task_by_vpid(ed_user_pid);
		if (ed_user_task) {
			get_task_struct(ed_user_task);
			game_ed_user_task = ed_user_task;
			game_ed_user_pid = ed_user_pid;
		}
		rcu_read_unlock();

		if (!game_ed_user_task) {
			game_ed_duration = 9500000; /* 9.5ms */
			game_ed_user_pid = -1;
		}
		raw_spin_unlock(&game_ed_lock);
	}
}
EXPORT_SYMBOL_GPL(fbg_game_set_ed_info);

void fbg_game_get_ed_info(int *ed_duration, int *ed_user_pid)
{
	*ed_duration = game_ed_duration;
	*ed_user_pid = game_ed_user_pid;
}
EXPORT_SYMBOL_GPL(fbg_game_get_ed_info);

void fbg_game_ed(struct rq *rq)
{
	unsigned long flags;
	struct frame_group *grp;
	bool is_game_group_empty;
	struct task_struct *p;
	struct oplus_task_struct *ots;
	int loop_max = 5;
	u64 now, window_start;

	if (!__frame_boost_enabled())
		return;

	if (!rq->cfs.h_nr_running)
		return;

	grp = frame_boost_groups[GAME_FRAME_GROUP_ID];
	raw_spin_lock_irqsave(&grp->lock, flags);
	is_game_group_empty = list_empty(&grp->tasks);
	window_start = grp->window_start;
	raw_spin_unlock_irqrestore(&grp->lock, flags);

	if (is_game_group_empty)
		return;

	now = fbg_ktime_get_ns();
	if (now - last_send_sig_time < MIN_SEND_SIG_INTERVAL)
		return;

	list_for_each_entry(p, &rq->cfs_tasks, se.group_node) {
		if (!loop_max)
			break;

		ots = get_oplus_task_struct(p);
		if (IS_ERR_OR_NULL(ots) || !ots->fbg_state || !(ots->fbg_state & FRAME_GAME))
			continue;

		if (now - max(ots->last_wake_ts, window_start) > game_ed_duration) {
			if (unlikely(sysctl_frame_boost_debug & DEBUG_FTRACE))
				trace_printk("comm=%s, pid=%d, tgid=%d, runnable time=%llu --- send ed signal=%d to gpa\n",
					p->comm, p->pid, p->tgid, now - ots->last_wake_ts, SIG_ED_TO_GPA);

			if (likely(cpu_online(raw_smp_processor_id())))
				irq_work_queue(&game_ed_irq_work);
			else
				irq_work_queue_on(&game_ed_irq_work, cpumask_any(cpu_online_mask));

			last_send_sig_time = now;

			break;
		}

		loop_max--;
	}
}
EXPORT_SYMBOL_GPL(fbg_game_ed);

static void fbg_game_ed_irq_work(struct irq_work *irq_work)
{
	raw_spin_lock(&game_ed_lock);
	if (game_ed_user_task)
		send_sig_info(SIG_ED_TO_GPA, SEND_SIG_PRIV, game_ed_user_task);
	raw_spin_unlock(&game_ed_lock);
}

int get_effect_stune_boost(struct task_struct *tsk, unsigned int type)
{
	struct frame_group *grp = NULL;
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);

	if (IS_ERR_OR_NULL(ots))
		return 0;

	if ((type < 0) || (type >= BOOST_MAX_TYPE))
		return 0;
	grp = task_get_frame_group(ots);
	if (grp == NULL)
		return 0;

	return grp->stune_boost[type];
}
EXPORT_SYMBOL_GPL(get_effect_stune_boost);


bool fbg_is_ed_task(struct task_struct *tsk, u64 wall_clock)
{
	struct oplus_task_struct *ots = NULL;
	struct frame_group *grp = NULL;
	unsigned long ed_task_boost_max_duration;
	unsigned long ed_task_boost_mid_duration;
	unsigned long ed_task_boost_timeout_duration;
	u64 exec_time = 0;

	if (!__frame_boost_enabled() || (!(sysctl_slide_boost_enabled || sysctl_input_boost_enabled)))
		return false;

	ots = get_oplus_task_struct(tsk);
	if (IS_ERR_OR_NULL(ots) || !ots->fbg_state || ots->fbg_state & (FRAME_COMPOSITION | FRAME_GAME | FRAME_INPUTMETHOD | FRAME_PENDING))
		return false;

	if (ots->last_wake_ts && wall_clock > ots->last_wake_ts)
		exec_time = wall_clock - ots->last_wake_ts;
	grp = task_get_frame_group(ots);
	if (!grp)
		return false;
	ed_task_boost_max_duration = mult_frac(grp->window_size, grp->stune_boost[BOOST_ED_TASK_MAX_DURATION], 100);
	ed_task_boost_mid_duration = mult_frac(grp->window_size, grp->stune_boost[BOOST_ED_TASK_MID_DURATION], 100);
	ed_task_boost_timeout_duration = mult_frac(grp->window_size, grp->stune_boost[BOOST_ED_TASK_TIME_OUT_DURATION], 100);
	if (exec_time >= ed_task_boost_max_duration && exec_time < ed_task_boost_timeout_duration) {
		ed_task_boost_type = ED_TASK_BOOST_MAX;
		return true;
	} else if (exec_time >= ed_task_boost_mid_duration && exec_time < ed_task_boost_max_duration) {
		ed_task_boost_type = ED_TASK_BOOST_MID;
		return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(fbg_is_ed_task);

void update_wake_up(struct task_struct *p)
{
	struct oplus_task_struct *ots = NULL;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;
	ots->last_wake_ts = fbg_ktime_get_ns();
}
EXPORT_SYMBOL_GPL(update_wake_up);

static void fbg_new_task_stats(void *unused, struct task_struct *p)
{
	if (!__frame_boost_enabled())
		return;

	update_wake_up(p);
}

static void fbg_try_to_wake_up(void *unused, struct task_struct *p)
{
	if (!__frame_boost_enabled())
		return;

	update_wake_up(p);
}

static void fbg_update_freq_hook(void *unused, bool preempt, struct task_struct *prev,
	struct task_struct *next, unsigned int prev_state)
{
	u64 fbg_wall_clock = fbg_ktime_get_ns();
	struct rq *rq = NULL;

	if (unlikely(fbg_hook.update_freq == NULL))
		return;

	if (fbg_is_ed_task(next, fbg_wall_clock)) {
		rq = cpu_rq(next->wake_cpu);
		fbg_hook.update_freq(rq, SCHED_CPUFREQ_EARLY_DET);
	}
}

static void fbg_update_suspend_resume(void *unused, const char *action, int val, bool start)
{
	if (!__frame_boost_enabled())
		return;

	if (!strncmp(action, "timekeeping_freeze", 18)) {
		if (start)
			fbg_suspend();
		else
			fbg_resume();
	}
}

void register_frame_group_vendor_hooks(void)
{
	/* Register vender hook in driver/android/binder.c */
#ifdef TODO_DELME
	register_android_vh_binder_proc_transaction_finish(fbg_binder_wakeup_hook, NULL);
#endif
	register_trace_android_vh_binder_restore_priority(fbg_binder_restore_priority_hook, NULL);
	register_trace_android_vh_binder_wait_for_work(fbg_binder_wait_for_work_hook, NULL);
	register_trace_android_vh_sync_txn_recvd(fbg_sync_txn_recvd_hook, NULL);

	/* Register vendor hook in fs/exec.c */
	register_trace_task_rename(task_rename_hook, NULL);

	/* Register vender hook in kernel/sched/fair.c{rt.c|deadline.c} */
	register_trace_sched_stat_runtime(fbg_update_task_util_hook, NULL);

	/* Register vender hook in kernel/sched/core.c */
	register_trace_android_rvh_flush_task(fbg_flush_task_hook, NULL);
	register_trace_android_rvh_sched_fork(fbg_sched_fork_hook, NULL);

	register_trace_android_rvh_try_to_wake_up(fbg_try_to_wake_up, NULL);
	register_trace_android_rvh_new_task_stats(fbg_new_task_stats, NULL);

	/* Register vender hook in kernel/sched/core.c */
	register_trace_sched_switch(fbg_update_freq_hook, NULL);

	/* Register vender hook in kernel/time/tick-common.c */
	register_trace_suspend_resume(fbg_update_suspend_resume, NULL);
}

int info_show(struct seq_file *m, void *v)
{
	unsigned long flags;
	struct frame_group *grp = NULL;
	struct task_struct *tsk = NULL;
	struct oplus_task_struct *ots = NULL;
	int i;

	for (i = MULTI_FBG_ID; i < MULTI_FBG_ID + MULTI_FBG_NUM; i++) {
		seq_puts(m, "\n---- MULTI FRAME GROUP[");
		seq_printf(m, "%d", i);
		seq_puts(m, "]----\n");
		grp = frame_boost_groups[i];
		raw_spin_lock_irqsave(&grp->lock, flags);
		list_for_each_entry(ots, &grp->tasks, fbg_list) {
			tsk = ots_to_ts(ots);
			seq_printf(m, "comm=%-16s  pid=%-6d  tgid=%-6d  state=0x%x  depth=%d\n",
				tsk->comm, tsk->pid, tsk->tgid, ots->fbg_state, ots->fbg_depth);
		}
		raw_spin_unlock_irqrestore(&grp->lock, flags);
	}
	seq_puts(m, "\n---- SF COMPOSITION GROUP ----\n");
	grp = frame_boost_groups[SF_FRAME_GROUP_ID];
	raw_spin_lock_irqsave(&grp->lock, flags);
	list_for_each_entry(ots, &grp->tasks, fbg_list) {
		tsk = ots_to_ts(ots);
		seq_printf(m, "comm=%-16s  pid=%-6d  tgid=%-6d  state=0x%x  depth=%d\n",
			tsk->comm, tsk->pid, tsk->tgid, ots->fbg_state, ots->fbg_depth);
	}
	raw_spin_unlock_irqrestore(&grp->lock, flags);

	seq_puts(m, "\n---- GAME FRAME GROUP ----\n");
	grp = frame_boost_groups[GAME_FRAME_GROUP_ID];
	raw_spin_lock_irqsave(&grp->lock, flags);
	list_for_each_entry(ots, &grp->tasks, fbg_list) {
		tsk = ots_to_ts(ots);
		seq_printf(m, "comm=%-16s  pid=%-6d  tgid=%-6d  state=0x%x  depth=%d\n",
			tsk->comm, tsk->pid, tsk->tgid, ots->fbg_state, ots->fbg_depth);
	}
	raw_spin_unlock_irqrestore(&grp->lock, flags);

	seq_puts(m, "\n---- INPUTMETHOD FRAME GROUP ----\n");
	grp = frame_boost_groups[INPUTMETHOD_FRAME_GROUP_ID];
	raw_spin_lock_irqsave(&grp->lock, flags);
	list_for_each_entry(ots, &grp->tasks, fbg_list) {
		tsk = ots_to_ts(ots);
		seq_printf(m, "comm=%-16s  pid=%-6d  tgid=%-6d  state=0x%x  depth=%d\n",
			tsk->comm, tsk->pid, tsk->tgid, ots->fbg_state, ots->fbg_depth);
	}
	raw_spin_unlock_irqrestore(&grp->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(info_show);

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
struct cpumask *fbg_cpu_halt_mask;
void init_fbg_halt_mask(struct cpumask *halt_mask)
{
	fbg_cpu_halt_mask = halt_mask;
}
EXPORT_SYMBOL_GPL(init_fbg_halt_mask);
#endif

static void set_default_stunedata(struct frame_group *grp)
{
	grp->stune_boost[BOOST_ED_TASK_MID_DURATION] = 60; /* default 0.6*window */
	grp->stune_boost[BOOST_ED_TASK_MAX_DURATION] = 80; /* default 0.8*window */
	grp->stune_boost[BOOST_ED_TASK_TIME_OUT_DURATION] = 200; /* default 2*window */
	grp->stune_boost[BOOST_ED_TASK_MID_UTIL] = 600; /* default mid util */
	grp->stune_boost[BOOST_ED_TASK_MAX_UTIL] = 900; /* default max util */
}

int frame_group_init(void)
{
	struct frame_group *grp = NULL;
	int i, ret = 0;
	struct oplus_sched_cluster *cluster = NULL;

	for (i = 1; i < MAX_NUM_FBG_ID; i++) {
		grp = kzalloc(sizeof(*grp), GFP_NOWAIT);
		if (!grp) {
			ret = -ENOMEM;
			goto out;
		}

		INIT_LIST_HEAD(&grp->tasks);
		grp->window_size = NSEC_PER_SEC / DEFAULT_FRAME_RATE;
		grp->window_start = 0;
		grp->nr_running = 0;
		grp->mark_start = 0;
		grp->preferred_cluster = NULL;
		grp->available_cluster = NULL;
		raw_spin_lock_init(&grp->lock);
		frame_boost_groups[i] = grp;
		grp->id = i;
		if (is_multi_frame_fbg(i))
			set_default_stunedata(grp);
	}

	grp = frame_boost_groups[SF_FRAME_GROUP_ID];
#ifdef CONFIG_ARCH_MEDIATEK
	grp->stune_boost[BOOST_SF_FREQ_GPU] = 60;
	grp->stune_boost[BOOST_SF_MIGR_GPU] = 60;
#else
	grp->stune_boost[BOOST_SF_FREQ_GPU] = 30;
	grp->stune_boost[BOOST_SF_MIGR_GPU] = 30;
#endif /* CONFIG_ARCH_MEDIATEK */
	ed_task_boost_type = 0;

	schedtune_spc_rdiv = reciprocal_value(100);

	if (!build_clusters()) {
		ret = -1;
		ofb_err("failed to build sched cluster\n");
		goto out;
	}

	for_each_sched_cluster(cluster)
		ofb_debug("num_cluster=%d id=%d cpumask=%*pbl capacity=%lu num_cpus=%d\n",
			num_sched_clusters, cluster->id, cpumask_pr_args(&cluster->cpus),
			arch_scale_cpu_capacity(cpumask_first(&cluster->cpus)),
			num_possible_cpus());

	register_syscore_ops(&fbg_syscore_ops);

	init_irq_work(&game_ed_irq_work, fbg_game_ed_irq_work);

	atomic_set(&fbg_initialized, 1);

out:
	return ret;
}
