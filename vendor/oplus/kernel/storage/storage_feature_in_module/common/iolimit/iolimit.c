// SPDX-License-Identifier: GPL-2.0-only
/*
 * limit task's buffer write in cgroup.
 *
 * Copyright 2023 Unisoc(Shanghai) Technologies Co.Ltd
 *
 * Licensed under the Unisoc General Software License, version 1.0 (the
 * License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * https://www.unisoc.com/en_us/license/UNISOC_GENERAL_LICENSE_V1.0-EN_US
 * Software distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 * See the Unisoc General Software License, version 1.0 for more details.
 */

//This file has been modified by Unisoc (Tianjin) Technologies Co., Ltd in 2023.
#include <linux/version.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/blkdev.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
#include "linux/blk-cgroup.h"
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
#include "block/blk-cgroup.h"
#endif
#include <linux/cgroup.h>
#include <linux/jiffies.h>
#include <linux/sched/signal.h>
#include <linux/kthread.h>
#include <trace/hooks/mm.h>

#define CREATE_TRACE_POINTS


#undef TRACE_SYSTEM
#define TRACE_SYSTEM unisoc_io

#include <linux/tracepoint.h>

TRACE_EVENT(iolimit_write_control,
	TP_PROTO(unsigned long delta),

	TP_ARGS(delta),

	TP_STRUCT__entry(
		__field(pid_t, tgid)
		__field(pid_t, pid)
		__array(char, comm, TASK_COMM_LEN)
		__field(unsigned long, delta)
	),

	TP_fast_assign(
		__entry->tgid = current->tgid;
		__entry->pid  = current->pid;
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
		__entry->delta = delta * 1000 / HZ;
	),

	TP_printk("tgid:%d pid:%d comm=%s delta=%lu\n",
		__entry->tgid,
		__entry->pid,
		__entry->comm,
		__entry->delta
	)
);

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trace

#include <trace/define_trace.h>


//IO control's window is selected as (1/8)s.
#define WAIT_PARTS_NUM		(8)
#define WAIT_INTERNAL_JIF	(HZ/WAIT_PARTS_NUM)

static struct blkcg_policy iolimit_policy;

struct iolimit_grp {
	struct blkcg_policy_data cpd;

	struct timer_list	write_clear_timer;
	atomic64_t		write_max;	//max write bytes per seconds
	s64			nr_written;
	s64			nr_written_pause; //max write bytes in a window
	spinlock_t		write_lock;
	wait_queue_head_t	write_wq;
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
/*blkcg_css & kthread_blkcg not export api from kernel, copy from kthread.c*/
struct kthread {
	unsigned long flags;
	unsigned int cpu;
	int result;
	int (*threadfn)(void *);
	void *data;
	struct completion parked;
	struct completion exited;
#ifdef CONFIG_BLK_CGROUP
	struct cgroup_subsys_state *blkcg_css;
#endif
	/* To store the full name if task comm is truncated. */
	char *full_name;
};

static inline struct kthread *to_kthread(struct task_struct *k)
{
	WARN_ON(!(k->flags & PF_KTHREAD));
	return k->worker_private;
}

/**
 * kthread_blkcg - get associated blkcg css of current kthread
 *
 * Current thread must be a kthread.
 */
struct cgroup_subsys_state *kthread_blkcg(void)
{
	struct kthread *kthread;

	if (current->flags & PF_KTHREAD) {
		kthread = to_kthread(current);
		if (kthread)
			return kthread->blkcg_css;
	}
	return NULL;
}

/**
 * blkcg_css - find the current css
 *
 * Find the css associat KERed with either the kthread or the current task.
 * This may return a dying css, so it is up to the caller to use tryget logic
 * to confirm it is alive and well.
 */
static struct cgroup_subsys_state *blkcg_css(void)
{
        struct cgroup_subsys_state *css;

        css = kthread_blkcg();
        if (css)
                return css;
        return task_css(current, io_cgrp_id);
}
#endif

static inline struct iolimit_grp *cpd_to_iolimitcg(struct blkcg_policy_data *cpd)
{
	return cpd ? container_of(cpd, struct iolimit_grp, cpd) : NULL;
}

static inline struct iolimit_grp *css_to_iolimitcg(struct cgroup_subsys_state *css)
{
	struct blkcg *blkcg = css_to_blkcg(css);

	if (!blkcg)
		return NULL;

	return cpd_to_iolimitcg(blkcg_to_cpd(blkcg, &iolimit_policy));
}

static inline struct iolimit_grp *current_to_iolimitcg(void)
{
	return css_to_iolimitcg(blkcg_css());
}

static void write_clear_timer_fn(struct timer_list *t)
{
	struct iolimit_grp *iolimit_blkcg = from_timer(iolimit_blkcg, t, write_clear_timer);

	if (!iolimit_blkcg)
		return;

	spin_lock_bh(&iolimit_blkcg->write_lock);
	iolimit_blkcg->nr_written = 0;
	spin_unlock_bh(&iolimit_blkcg->write_lock);
	wake_up_all(&iolimit_blkcg->write_wq);
}

static bool is_write_need_wakeup(struct iolimit_grp *iolimit_blkcg, size_t count)
{
	bool ret = false;

	if (atomic64_read(&iolimit_blkcg->write_max) == 0)
		return true;

	if (iolimit_blkcg->nr_written_pause > (iolimit_blkcg->nr_written + count))
		return true;

	rcu_read_lock();
	if (iolimit_blkcg != current_to_iolimitcg())
		ret = true;

	rcu_read_unlock();
	return ret;
}

static bool is_need_iolimit(struct iolimit_grp *iolimit_blkcg, bool write)
{
	s64 setlimit = write ? atomic64_read(&iolimit_blkcg->write_max) : 0;

	if (setlimit == 0)
		return false;

	if (fatal_signal_pending(current))
		return false;

	return true;
}

void do_io_write_bandwidth_control(int count)
{
	struct iolimit_grp *iolimit_blkcg;
	struct cgroup_subsys_state *css;
	int io_space_cnt;
	int ret;
	unsigned long start_time = jiffies;
	unsigned long delta;

repeat:
	rcu_read_lock();
	iolimit_blkcg = current_to_iolimitcg();

	if (!iolimit_blkcg)
		goto out_rcu;
	if (!is_need_iolimit(iolimit_blkcg, true))
		goto out_rcu;

	spin_lock_bh(&iolimit_blkcg->write_lock);
	io_space_cnt = iolimit_blkcg->nr_written_pause -
			iolimit_blkcg->nr_written;
	if (io_space_cnt < count) {
		spin_unlock_bh(&iolimit_blkcg->write_lock);
		css = blkcg_css();
		if (!css)
			goto out_rcu;
		if (css_tryget_online(css)) {
			rcu_read_unlock();
			ret = wait_event_interruptible(iolimit_blkcg->write_wq,
				is_write_need_wakeup(iolimit_blkcg, count));

			css_put(css);

			// here wake up by signal
			if (ret < 0)
				goto out;
		} else {
			rcu_read_unlock();
		}

		goto repeat;
	} else {
		if (iolimit_blkcg->nr_written == 0) {
			mod_timer(&iolimit_blkcg->write_clear_timer,
					jiffies + WAIT_INTERNAL_JIF);
		}

		iolimit_blkcg->nr_written += count;
		spin_unlock_bh(&iolimit_blkcg->write_lock);
	}

out_rcu:
	rcu_read_unlock();
out:
	delta = jiffies - start_time;
	if (delta > 0)
		trace_iolimit_write_control(delta);
}

#if (LINUX_VERSION_CODE > KERNEL_VERSION(6, 1, 0))
static void io_write_bandwidth_control(void *unuse, struct inode *inode)
{
	struct cgroup_subsys_state *css;

	rcu_read_lock();
	css = blkcg_css();

	if (!css || !cgroup_parent(css->cgroup)) {
		rcu_read_unlock();
		return;
	}
	rcu_read_unlock();

	do_io_write_bandwidth_control(PAGE_SIZE);
}

#elif (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 1, 0))

static void io_write_bandwidth_control(void *data, void *unuse)
{
	struct cgroup_subsys_state *css;

	rcu_read_lock();
	css = blkcg_css();

	if (!css || !cgroup_parent(css->cgroup)) {
		rcu_read_unlock();
		return;
	}
	rcu_read_unlock();

	do_io_write_bandwidth_control(PAGE_SIZE);
}
#endif

static struct blkcg_policy_data *iolimit_alloc_cpd(gfp_t gfp)
{
	struct iolimit_grp *iolimit_blkcg;

	iolimit_blkcg = kzalloc(sizeof(*iolimit_blkcg), gfp);
	if (!iolimit_blkcg)
		return NULL;

	atomic64_set(&iolimit_blkcg->write_max, 0);
	iolimit_blkcg->nr_written = 0;
	iolimit_blkcg->nr_written_pause = 0;

	timer_setup(&iolimit_blkcg->write_clear_timer, write_clear_timer_fn, 0);

	spin_lock_init(&iolimit_blkcg->write_lock);
	init_waitqueue_head(&iolimit_blkcg->write_wq);

	return &iolimit_blkcg->cpd;
}

static void iolimit_free_cpd(struct blkcg_policy_data *cpd)
{
	struct iolimit_grp *iolimit_blkcg;

	iolimit_blkcg = cpd_to_iolimitcg(cpd);
	if (!iolimit_blkcg)
		return;

	wake_up_all(&iolimit_blkcg->write_wq);
	del_timer_sync(&iolimit_blkcg->write_clear_timer);

	kfree(iolimit_blkcg);
}

static int write_limit_store(struct cgroup_subsys_state *css,
		struct cftype *cft, s64 limit)
{
	struct iolimit_grp *iolimit_blkcg;

	if (limit < 0 || !css)
		return -EINVAL;

	iolimit_blkcg = css_to_iolimitcg(css);
	if (!iolimit_blkcg)
		return -EAGAIN;
	atomic64_set(&iolimit_blkcg->write_max, limit);

	spin_lock_bh(&iolimit_blkcg->write_lock);
	iolimit_blkcg->nr_written_pause = limit / WAIT_PARTS_NUM;
	spin_unlock_bh(&iolimit_blkcg->write_lock);

	return 0;
}

static s64 write_limit_show(struct cgroup_subsys_state *css,
		struct cftype *cft)
{
	struct iolimit_grp *iolimit_blkcg;

	if (!css)
		return -EINVAL;

	iolimit_blkcg = css_to_iolimitcg(css);
	if (!iolimit_blkcg)
		return -EAGAIN;
	return atomic64_read(&iolimit_blkcg->write_max);
}

static struct cftype iolimit_files[] = {
	{
		.name		= "iolimit.write_max",
		.flags		= CFTYPE_NOT_ON_ROOT,
		.write_s64	= write_limit_store,
		.read_s64	= write_limit_show,
	},
	{ } /* terminate */
};

static struct cftype iolimit_legacy_files[] = {
	{
		.name		= "iolimit.write_max",
		.flags		= CFTYPE_NOT_ON_ROOT,
		.write_s64	= write_limit_store,
		.read_s64	= write_limit_show,
	},
	{ } /* terminate */
};

static struct blkcg_policy iolimit_policy = {
	.dfl_cftypes	= iolimit_files,
	.legacy_cftypes	= iolimit_legacy_files,

	.cpd_alloc_fn	= iolimit_alloc_cpd,
	.cpd_free_fn	= iolimit_free_cpd,
};

static int __init iolimit_init(void)
{
	int ret;
	ret = blkcg_policy_register(&iolimit_policy);

	if (ret < 0) {
		pr_err("blkcg policy register failed: %d\n", ret);
		return ret;
	}

	register_trace_android_rvh_ctl_dirty_rate(io_write_bandwidth_control, NULL);
	pr_info("iolimit module init done\n");

	return ret;
}

/*
 * because iolimit registers rvh, and rvh cannot be unregisterred.
 * So, iolimit cannot support rmmod, or task calling vendor hook
 * will panic as mem abort.
 */
/*
 * static void __exit iolimit_exit(void)
 *{
 *	blkcg_policy_unregister(&iolimit_policy);
 *}
 */

module_init(iolimit_init)
//module_exit(iolimit_exit)
MODULE_AUTHOR("Jing Xia <jing.xia@unisoc.com>");
MODULE_LICENSE("GPL");
