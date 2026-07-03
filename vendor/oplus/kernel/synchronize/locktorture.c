// SPDX-License-Identifier: GPL-2.0+
/*
 * Module-based torture test facility for locking
 *
 * Copyright (C) IBM Corporation, 2014
 *
 * Authors: Paul E. McKenney <paulmck@linux.ibm.com>
 *          Davidlohr Bueso <dave@stgolabs.net>
 *	Based on kernel/rcu/torture.c.
 */

#define pr_fmt(fmt) fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/sched/rt.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <linux/rtmutex.h>
#include <linux/atomic.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/torture.h>
#include <linux/reboot.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched/clock.h>
#include <linux/cgroup.h>

#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#include <../kernel/oplus_cpu/sched/sched_assist/sa_group.h>
#include "locking_main.h"

#define RMMOD_EXIT	(2)

#define	FULL_TEST	(1)
#define	SINGLE_TEST	(2)

static struct task_struct **writer_tasks;
static struct task_struct **reader_tasks;

static bool lock_is_write_held;
static atomic_t lock_is_read_held;

static char *lock_str[] = {
	"spinlock", "rwlock", "mutex",
	"rtmutex", "rwsem", "pcp_rwsem"
};

static char *group_str[] = {
	"GRP_OT", "GRP_BG", "GRP_FG",
	"GRP_TA", "GRP_UX", "GRP_RT"
};

/* acquire latency statistic */
enum lock_types {
	SPIN_LOCK,
	RW_LOCK,
	MUTEX,
	RTMUTEX,
	RWSEM,
	PCP_RWSEM,
	LOCK_TYPES
};

enum thread_attr {
    GRP_OT,         /* normal thread */
    GRP_BG,
    GRP_FG,
    GRP_TA,
    GRP_UX,
    GRP_RT,
    NR_GRP_TYPES,
};

struct lock_stress_stats {
	long n_lock_fail;
	long n_lock_acquired;
	enum thread_attr attr;
};

/*
 * Operations vector for selecting different types of tests.
 */
struct lock_torture_ops {
	void (*init)(void);
	void (*exit)(void);
	int (*writelock)(void);
	void (*write_delay)(unsigned long delay_us);
	void (*writeunlock)(void);
	int (*readlock)(void);
	void (*read_delay)(unsigned long delay_us);
	void (*readunlock)(void);
	unsigned long flags; /* for irq spinlocks */
	const char *name;
};

struct lock_torture_cfg {
	/********************WARNNING*******************/
	/* Don't change the order of any param below.*/
	/* If you want to add sth, add it to tail.*/
	/********************WARNNING*******************/
	int normal_writers;
    int bg_writers;
    int fg_writers;
    int top_writers;
    int ux_writers;
    int rt_writers;

    int normal_readers;
    int bg_readers;
    int fg_readers;
    int top_readers;
    int ux_readers;
    int rt_readers;
	/* The interval between attempts to hold the lock(jiffies) */
	int stutter;
	/* critical section duration(us)*/
	int cs_us;
	/* test duration(s).*/
	int duration_s;
	/* lock type.*/
	int lock_type;

	struct lock_torture_ops *cur_ops;
};

struct lock_torture_cxt {
	struct lock_torture_cfg cfg;
	int mode;
	bool init_called;
	atomic_t n_lock_torture_errors;
	struct lock_stress_stats *lwsa; /* writer statistics */
	struct lock_stress_stats *lrsa; /* reader statistics */
};
static struct lock_torture_cxt cxt = {.mode = FULL_TEST};


/**************************origin locktorture*********************************/
#define FULLSTOP_NORMAL	0    /* Normal operation. */
#define FULLSTOP_STOP	1    /* Normal rmmod of torture. */

static atomic_t fullstop;

static void o_torture_cleanup_begin(void)
{
	atomic_set_release(&fullstop, FULLSTOP_STOP);
}

static void o_torture_cleanup_end(void)
{
	atomic_set_release(&fullstop, FULLSTOP_NORMAL);
}

static bool o_torture_must_stop_irq(void)
{
    return atomic_read_acquire(&fullstop) != FULLSTOP_NORMAL;
}

static bool o_torture_must_stop(void)
{
    return o_torture_must_stop_irq() || kthread_should_stop();
}

static void o_torture_kthread_stopping(void)
{
    while (!kthread_should_stop()) {
        schedule_timeout_uninterruptible(1);
    }
}

static int torture_create_thread(int (*fn)(void *arg), void *arg, char *s, struct task_struct **tp)
{
    int ret = 0;

    *tp = kthread_run(fn, arg, "%s", s);
    if (IS_ERR(*tp)) {
            ret = PTR_ERR(*tp);
            *tp = NULL;
    }
    return ret;
}

static void torture_stop_thread(struct task_struct **tp)
{
    if (*tp == NULL)
            return;
    kthread_stop(*tp);
    *tp = NULL;
}


/*****************************************************************************/


static void torture_fixed_delay(unsigned long delay_us)
{
	udelay(delay_us);
	// usleep_range(delay_us, delay_us + 10);
}


static int set_current_ux(int rw)
{
    oplus_set_ux_state_lock(current, SA_TYPE_LISTPICK, -1, true);
    return 0;
}

static int set_current_rt(int rw)
{
    struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

    return sched_setscheduler(current, SCHED_FIFO, &param);
}

#ifdef CONFIG_OPLUS_LOCKING_CGROUP_SUPPORT
int attach_cgroup_from_name(struct task_struct *tsk, const char *path);
#endif
/* compatible with enum thread_attr, first item is OT. */
/* BG/FG/TA. */
//static char *cgroup_files[] = {" ", "background", "foreground", "top-app"};
static int write_cgroup_file(enum thread_attr attr, int rw)
{
#ifdef CONFIG_OPLUS_LOCKING_CGROUP_SUPPORT
	return attach_cgroup_from_name(current, cgroup_files[attr]);
#else
	pr_err("not support BG/FG/TA");
	return -1;
#endif
}

static int set_thread_attr(enum thread_attr attr, int rw)
{
    switch(attr) {
		case GRP_OT:
			/* Nothing to do.*/
			break;
		case GRP_BG:
		case GRP_FG:
		case GRP_TA:
			return write_cgroup_file(attr, rw);
		case GRP_UX:
			return set_current_ux(rw);
			break;
		case GRP_RT:
			return set_current_rt(rw);
			break;
		default:
			return -1;
    }
    return 0;
}

/*************************kernel lock ops def*******************************/
static DEFINE_SPINLOCK(torture_spinlock);

static int torture_spin_lock_write_lock(void)
__acquires(torture_spinlock)
{
	spin_lock(&torture_spinlock);
	return 0;
}

static void torture_spin_lock_write_unlock(void)
__releases(torture_spinlock)
{
	spin_unlock(&torture_spinlock);
}

static struct lock_torture_ops spin_lock_ops = {
	.writelock	= torture_spin_lock_write_lock,
	.write_delay	= torture_fixed_delay,
	.writeunlock	= torture_spin_lock_write_unlock,
	.readlock       = NULL,
	.read_delay     = NULL,
	.readunlock     = NULL,
	.name		= "spinlock"
};


static DEFINE_RWLOCK(torture_rwlock);

static int torture_rwlock_write_lock(void)
__acquires(torture_rwlock)
{
	write_lock(&torture_rwlock);
	return 0;
}

static void torture_rwlock_write_unlock(void)
__releases(torture_rwlock)
{
	write_unlock(&torture_rwlock);
}

static int torture_rwlock_read_lock(void)
__acquires(torture_rwlock)
{
	read_lock(&torture_rwlock);
	return 0;
}

static void torture_rwlock_read_unlock(void)
__releases(torture_rwlock)
{
	read_unlock(&torture_rwlock);
}

static struct lock_torture_ops rw_lock_ops = {
	.writelock	= torture_rwlock_write_lock,
	.write_delay	= torture_fixed_delay,
	.writeunlock	= torture_rwlock_write_unlock,
	.readlock       = torture_rwlock_read_lock,
	.read_delay     = torture_fixed_delay,
	.readunlock     = torture_rwlock_read_unlock,
	.name		= "rwlock"
};


static DEFINE_MUTEX(torture_mutex);

static int torture_mutex_lock(void)
__acquires(torture_mutex)
{
	mutex_lock(&torture_mutex);
	return 0;
}

static void torture_mutex_unlock(void)
__releases(torture_mutex)
{
	mutex_unlock(&torture_mutex);
}

static struct lock_torture_ops mutex_lock_ops = {
	.writelock	= torture_mutex_lock,
	.write_delay	= torture_fixed_delay,
	.writeunlock	= torture_mutex_unlock,
	.readlock       = NULL,
	.read_delay     = NULL,
	.readunlock     = NULL,
	.name		= "mutex"
};


#ifdef CONFIG_RT_MUTEXES
static DEFINE_RT_MUTEX(torture_rtmutex);

static int torture_rtmutex_lock(void)
__acquires(torture_rtmutex)
{
	rt_mutex_lock(&torture_rtmutex);
	return 0;
}

static void torture_rtmutex_unlock(void)
__releases(torture_rtmutex)
{
	rt_mutex_unlock(&torture_rtmutex);
}

static struct lock_torture_ops rtmutex_lock_ops = {
	.writelock	= torture_rtmutex_lock,
	.write_delay	= torture_fixed_delay,
	.writeunlock	= torture_rtmutex_unlock,
	.readlock       = NULL,
	.read_delay     = NULL,
	.readunlock     = NULL,
	.name		= "rtmutex"
};
#endif


static DECLARE_RWSEM(torture_rwsem);
static int torture_rwsem_down_write(void)
__acquires(torture_rwsem)
{
	down_write(&torture_rwsem);
	return 0;
}

static void torture_rwsem_up_write(void)
__releases(torture_rwsem)
{
	up_write(&torture_rwsem);
}

static int torture_rwsem_down_read(void)
__acquires(torture_rwsem)
{
	down_read(&torture_rwsem);
	return 0;
}

static void torture_rwsem_up_read(void)
__releases(torture_rwsem)
{
	up_read(&torture_rwsem);
}

static struct lock_torture_ops rwsem_lock_ops = {
	.writelock	= torture_rwsem_down_write,
	.write_delay	= torture_fixed_delay,
	.writeunlock	= torture_rwsem_up_write,
	.readlock       = torture_rwsem_down_read,
	.read_delay     = torture_fixed_delay,
	.readunlock     = torture_rwsem_up_read,
	.name		= "rwsem"
};


#include <linux/percpu-rwsem.h>
static struct percpu_rw_semaphore pcpu_rwsem;

static void torture_percpu_rwsem_init(void)
{
	BUG_ON(percpu_init_rwsem(&pcpu_rwsem));
}

static void torture_percpu_rwsem_exit(void)
{
	percpu_free_rwsem(&pcpu_rwsem);
}

static int torture_percpu_rwsem_down_write(void)
__acquires(pcpu_rwsem)
{
	percpu_down_write(&pcpu_rwsem);
	return 0;
}

static void torture_percpu_rwsem_up_write(void)
__releases(pcpu_rwsem)
{
	percpu_up_write(&pcpu_rwsem);
}

static int torture_percpu_rwsem_down_read(void)
__acquires(pcpu_rwsem)
{
	percpu_down_read(&pcpu_rwsem);
	return 0;
}

static void torture_percpu_rwsem_up_read(void)
__releases(pcpu_rwsem)
{
	percpu_up_read(&pcpu_rwsem);
}

static struct lock_torture_ops percpu_rwsem_lock_ops = {
	.init		= torture_percpu_rwsem_init,
	.exit		= torture_percpu_rwsem_exit,
	.writelock	= torture_percpu_rwsem_down_write,
	.write_delay	= torture_fixed_delay,
	.writeunlock	= torture_percpu_rwsem_up_write,
	.readlock       = torture_percpu_rwsem_down_read,
	.read_delay     = torture_fixed_delay,
	.readunlock     = torture_percpu_rwsem_up_read,
	.name		= "pcp_rwsem"
};

/*************************kernel lock ops def*******************************/


/*****************************statistics***********************************/

#define GRP_TYPES	3
#define LATENCY_LVLS	6

#define READER	0
#define WRITER	1
#define RWTYPE	2

#define TOT_TIME	0
#define TOT_CNTS	1
#define REC_ITEM	2

static unsigned long scopes[LATENCY_LVLS][2] = {
	{0, 500 * NSEC_PER_USEC},
	{500 * NSEC_PER_USEC, 2 * NSEC_PER_MSEC},
	{2 * NSEC_PER_MSEC, 5 * NSEC_PER_MSEC},
	{5 * NSEC_PER_MSEC, 20 * NSEC_PER_MSEC},
	{20 * NSEC_PER_MSEC, 50 * NSEC_PER_MSEC},
	{50 * NSEC_PER_MSEC, ULONG_MAX}
};

/* Acquire latency statistic.
** LOCK_TYPES:0-spinlock,1-rwlock,2-mutex,3-rtmutex,4-rwsem,5-percpu_rwsem
** NR_GRP_TYPES:0-OT,1-BG,2-FG,3-TA,4-UX,5-RT
** RWTYPE:0-reader,1-writer
** LATENCY_LVLS:see definition of scopes[] above.
*/
static atomic_t lat_stats[LOCK_TYPES][NR_GRP_TYPES][RWTYPE][LATENCY_LVLS];

/* Acquire latency statistic.
** LOCK_TYPES:0-spinlock,1-rwlock,2-mutex,3-rtmutex,4-rwsem,5-percpu_rwsem
** NR_GRP_TYPES:0-OT,1-BG,2-FG,3-TA,4-UX,5-RT
** RWTYPE:0-reader,1-writer
** REC_ITEM:0-total time,1-total cnt.
*/
static atomic64_t lat_total_time[LOCK_TYPES][NR_GRP_TYPES][RWTYPE][REC_ITEM];

static void acquire_start(void)
{
	struct oplus_task_struct *ots;

	/* Current, no rculock here.*/
	ots = get_oplus_task_struct(current);
	if (unlikely(IS_ERR_OR_NULL(ots))) {
		trace_printk("locktorture : ots == NULL !!!!!!!\n");
		return;
	}

	/* start must always pair with finish. */
	if (ots->lkinfo.acquire_stamp)
		trace_printk("locktorture : why torture stamp not 0 !!!!!!!\n");

	ots->lkinfo.acquire_stamp = sched_clock();
}

static void acquire_finish(int lock_type, int rw, enum thread_attr grp)
{
	struct oplus_task_struct *ots;
	unsigned long duration;
	int i;

	ots = get_oplus_task_struct(current);
	if (unlikely(IS_ERR_OR_NULL(ots))) {
		trace_printk("locktorture : ots == NULL !!!!!!!\n");
		return;
	}
	if (0 == ots->lkinfo.acquire_stamp)
		trace_printk("locktorture : why torture stamp is 0 !!!!!!!\n");

	duration = sched_clock() - ots->lkinfo.acquire_stamp;

	ots->lkinfo.acquire_stamp = 0;

	atomic64_add(duration, &lat_total_time[lock_type][grp][rw][0]);
	atomic64_inc(&lat_total_time[lock_type][grp][rw][1]);

	for (i = 0; i < LATENCY_LVLS; i++) {
		if (duration < scopes[i][1] && duration >= scopes[i][0]) {
			atomic_inc(&lat_stats[lock_type][grp][rw][i]);
			return;
		}
	}
}
/**************************************************************************/
/*****************************statistics***********************************/
/**************************************************************************/



/*************************************************************************/
/*****************************proc files**********************************/
/*************************************************************************/

#define TORTURE_PROC_DIR        "torture"
#define TORTURE_PROC_CTRL       "torture_ctrl"
#define TORTURE_PROC_CONF       "torture_configs"
#define TORTURE_PROC_RESULTS    "torture_results"
extern struct proc_dir_entry *d_oplus_locking;
static int torture_ctrl;
struct proc_dir_entry *d_torture;

#define OUT_BUF_SIZE	(512 * 1024)
static char out_buf[OUT_BUF_SIZE];
static int out_buf_idx = 0;

static DECLARE_WAIT_QUEUE_HEAD(torture_wq);
static atomic_t thread_run = ATOMIC_INIT(0);

static DECLARE_COMPLETION(torture_done);

#define CONFIG_ITEMS_NR         17
static char *matchs[CONFIG_ITEMS_NR] = {
    "mode=", "normal_w=", "bg_w=",
    "fg_w=", "top_w=", "ux_w=",
    "rt_w=", "normal_r=", "bg_r=",
    "fg_r=", "top_r=", "ux_r=",
    "rt_r=", "stutter=", "cs_us=",
    "duration_s=", "lock_type="
};

static int limits[CONFIG_ITEMS_NR][2] = {
    {FULL_TEST, SINGLE_TEST},
    {0, 1000},
    {0, 1000},
    {0, 1000},
    {0, 1000},
    {0, 1000},
    {0, 1000},
    {0, 1000},
    {0, 1000},
    {0, 1000},
    {0, 1000},
    {0, 1000},
    {0, 1000},
    {0, INT_MAX},
    {0, INT_MAX},
    {0, INT_MAX},
    {0, INT_MAX}
};

#define FULL_CONFIGS_NR (sizeof(full_configs) / sizeof(full_configs[0]))

struct lock_torture_cfg full_configs[] = {
    {
        .cur_ops = &spin_lock_ops,
        .lock_type = SPIN_LOCK,
        .normal_writers = 32,
        .normal_readers = 0,
        .stutter = 10,
        .cs_us = 10,
        .duration_s = 20
    },
    {
        .cur_ops = &rw_lock_ops,
        .lock_type = RW_LOCK,
        .normal_writers = 8,
        .normal_readers = 8,
        .stutter = 10,
        .cs_us = 10,
        .duration_s = 20
    },
    {
        .cur_ops = &mutex_lock_ops,
        .lock_type = MUTEX,
        .normal_writers = 40,
		.ux_writers = 16,
		.rt_writers = 8,
        .stutter = 10,
        .cs_us = 500,
        .duration_s = 20
    },
    {
        .cur_ops = &rtmutex_lock_ops,
        .lock_type = RTMUTEX,
        .normal_writers = 40,
        .ux_writers = 16,
        .rt_writers = 8,
        .stutter = 10,
        .cs_us = 500,
        .duration_s = 20
    },
	{
        .cur_ops = &rwsem_lock_ops,
        .lock_type = RWSEM,
        .normal_writers = 4,
		.normal_readers = 60,
		.ux_readers = 16,
		.rt_readers = 8,
        .stutter = 10,
        .cs_us = 500,
        .duration_s = 20
    },
    {
        .cur_ops = &percpu_rwsem_lock_ops,
        .lock_type = PCP_RWSEM,
        .normal_writers = 4,
		.normal_readers = 60,
		.ux_readers = 16,
		.rt_readers = 8,
        .stutter = 10,
        .cs_us = 500,
        .duration_s = 20
    }
};

/*
 * Create an lock-torture-statistics message in the specified buffer.
 */
static int __torture_print_stats(char *page,
				  struct lock_stress_stats *statp, bool write)
{
	long cur;
	bool fail = false;
	int i, n_stress;
	long max = 0, min = statp ? data_race(statp[0].n_lock_acquired) : 0;
	long long sum = 0;
	int idx = 0;

	int *p;
    int total_writers = 0;
    int total_readers = 0;

    p = &cxt.cfg.normal_writers;
    for (i = 0; i < LOCK_TYPES; i++) {
        total_writers += *p;
        p++;
    }

	if (cxt.cfg.cur_ops->readlock) {
        p = &cxt.cfg.normal_readers;
        for (i = 0; i < LOCK_TYPES; i++) {
            total_readers += *p;
            p++;
        }
    }

	n_stress = write ? total_writers : total_readers;

	for (i = 0; i < n_stress; i++) {
		if (data_race(statp[i].n_lock_fail))
			fail = true;
		cur = data_race(statp[i].n_lock_acquired);
		sum += cur;
		if (max < cur)
			max = cur;
		if (min > cur)
			min = cur;
	}
	idx += sprintf(&page[idx],
		"%s:  Total: %lld  Max/Min: %ld/%ld  Fail: %d %s\n",
		write ? "Writes" : "Reads ",
		sum, max, min,
		fail, fail ? "!!!" : "");
	if (fail)
		atomic_inc(&cxt.n_lock_torture_errors);
	return idx;
}

#define STAMP_TO_UNIT_NUM(x)	(x / 1000000 >= 1 ? (x / 1000000) : (x / 1000))
#define STAMP_TO_NUM_UNIT(x)	(x / 1000000 >= 1 ? "ms" : "us")

static void __torture_latency_print(void)
{
	int i, j, k, m;
	char tmp[50];

	out_buf_idx += sprintf(&out_buf[out_buf_idx], "\n");
	out_buf_idx += sprintf(&out_buf[out_buf_idx], "%-16s","scopes");
	for (i = 0; i < LATENCY_LVLS; i++) {
		if (i < LATENCY_LVLS - 1) {
			sprintf(tmp, "%lu%s~%lu%s",
				STAMP_TO_UNIT_NUM(scopes[i][0]),
				STAMP_TO_NUM_UNIT(scopes[i][0]),
				STAMP_TO_UNIT_NUM(scopes[i][1]),
				STAMP_TO_NUM_UNIT(scopes[i][1]));
		}
		else {
			sprintf(tmp, "%lu%s~%s",
				STAMP_TO_UNIT_NUM(scopes[i][0]),
				STAMP_TO_NUM_UNIT(scopes[i][0]),
				"UL_MAX");
		}
		out_buf_idx += sprintf(&out_buf[out_buf_idx], "%-12s", tmp);
	}
	out_buf_idx += sprintf(&out_buf[out_buf_idx], "%-12s", "avg_time");
	out_buf_idx += sprintf(&out_buf[out_buf_idx], "\n");

	if (SINGLE_TEST == cxt.mode) {
		for (m = 0; m < NR_GRP_TYPES; m++) {
			for (j = 0; j < RWTYPE; j++) {
				if (!cxt.cfg.cur_ops->readlock && (j == 0))
					continue;
				sprintf(tmp, "%s-%s%s", cxt.cfg.cur_ops->name,
					&group_str[m][4],
					(j == 0) ? "-r" : "-w");
				out_buf_idx += sprintf(&out_buf[out_buf_idx],
					"%-16s", tmp);

				for (i = 0; i < LATENCY_LVLS; i++) {
					out_buf_idx += sprintf(
						&out_buf[out_buf_idx],
						"%-12d",
						atomic_read(&lat_stats[cxt.cfg.lock_type][m][j][i]));
				}
				out_buf_idx += sprintf(&out_buf[out_buf_idx], "%lu",
					(unsigned long)(atomic64_read(&lat_total_time[cxt.cfg.lock_type][m][j][0])
					/ atomic64_read(&lat_total_time[cxt.cfg.lock_type][m][j][1])));
				out_buf_idx += sprintf(&out_buf[out_buf_idx], "\n");
			}
		}
	} else {
		for (k = 0; k < LOCK_TYPES; k++) {
			for (m = 0; m < NR_GRP_TYPES; m++) {
				for (j = 0; j < 2; j++) {
					if (!full_configs[k].cur_ops->readlock
								&& (j == 0))
						continue;
					sprintf(tmp, "%s-%s%s", lock_str[k],
						&group_str[m][4],
						(j == 0) ? "-r" : "-w");
					out_buf_idx += sprintf(
						&out_buf[out_buf_idx],
						"%-16s", tmp);

					for (i = 0; i < LATENCY_LVLS; i++) {
						out_buf_idx += sprintf(
							&out_buf[out_buf_idx],
							"%-12d",
							atomic_read(&lat_stats[k][m][j][i]));
					}
					out_buf_idx += sprintf(
						&out_buf[out_buf_idx],
						"%lu",
						(unsigned long)(atomic64_read(&lat_total_time[k][m][j][0])
						/ atomic64_read(&lat_total_time[k][m][j][1])));
					out_buf_idx += sprintf(
						&out_buf[out_buf_idx], "\n");
				}
			}
		}
		out_buf_idx += sprintf(&out_buf[out_buf_idx], "\n");
	}
}

/*
 * Print torture statistics.  Caller must ensure that there is only one
 * call to this function at a given time!!!  This is normally accomplished
 * by relying on the module system to only have one copy of the module
 * loaded, and then by giving the lock_torture_stats kthread full control
 * (or the init/cleanup functions when lock_torture_stats thread is not
 * running).
 */
static void lock_torture_stats_print(void)
{

	out_buf_idx += sprintf(&out_buf[out_buf_idx],
		"%s-{\n", cxt.cfg.cur_ops->name);
	out_buf_idx += __torture_print_stats(&out_buf[out_buf_idx],
		cxt.lwsa, true);

	if (cxt.cfg.cur_ops->readlock) {
		out_buf_idx += __torture_print_stats(
			&out_buf[out_buf_idx], cxt.lrsa, false);
	}
	out_buf_idx += sprintf(&out_buf[out_buf_idx], "}\n");
}

static ssize_t torture_ctrl_write(struct file *file, const char __user *buf,
                               size_t count, loff_t *ppos)
{
        char kbuf[5] = {0};
        int err, onoff;

        if (count >= 5)
                return -EFAULT;

	if (torture_ctrl)
		return -EBUSY;

        if (copy_from_user(kbuf, buf, count)) {
            pr_err("ERROR : Failed to copy_from_user"
				"to set torture_ctrl flag \n");
            return -EFAULT;
        }
        err = kstrtoint(strstrip(kbuf), 0, &onoff);
        if (err < 0) {
            pr_err("ERROR : Failed to kstrtoint"
				"to set torture_ctrl flag \n");
            return -EFAULT;
        }

        torture_ctrl = !!onoff;
        if (torture_ctrl) {
			atomic_set(&thread_run, 1);
			wake_up(&torture_wq);
        } else {
			complete(&torture_done);
        }
        return count;
}

static int torture_ctrl_show(struct seq_file *m, void *v)
{
        seq_printf(m, "%d\n",torture_ctrl);
        return 0;
}

static int torture_ctrl_open(struct inode *inode, struct file *file)
{
        return single_open(file, torture_ctrl_show, inode);
}

static const struct proc_ops torture_ctrl_ops = {
        .proc_open              = torture_ctrl_open,
        .proc_write             = torture_ctrl_write,
        .proc_read              = seq_read,
        .proc_lseek             = seq_lseek,
        .proc_release           = single_release,
};

static ssize_t torture_configs_write(struct file *file, const char __user *buf,
                               size_t count, loff_t *ppos)
{
	int i;
	char *start;
	long val[CONFIG_ITEMS_NR];
	int *p;

	char kbuf[500] = {0};

	if (torture_ctrl)
		return -EBUSY;

	if (copy_from_user(kbuf, buf, count)) {
		pr_err("ERROR : Failed to copy_from_user"
			"to set thread_ctrl flag \n");
		return -EFAULT;
	}

	for (i = 0; i < CONFIG_ITEMS_NR; i++) {
		val[i] = -1;
        start = strstr(kbuf, matchs[i]);
        if (NULL == start)
            continue;
        val[i] = simple_strtol(start + strlen(matchs[i]), NULL, 10);
    }

	/* mode setting. */
	if (val[0] != FULL_TEST && val[0] != SINGLE_TEST)
		return -EINVAL;
	/* full test mode noneed to set other params. */
	else if (val[0] == FULL_TEST) {
		cxt.mode = FULL_TEST;
		return count;
	} else {
		cxt.mode = SINGLE_TEST;
	}

	/* If set SINGLE_TEST mode, you have to set all params,
	 * otherwise it will fail.
	 */
	for (i = 0; i < CONFIG_ITEMS_NR; i++) {
		if (val[i] == -1) {
			printk("Some config item not given \n");
			return -EINVAL;
		}
	}

	p = (int*)&cxt.cfg;
	for (i = 1; i < CONFIG_ITEMS_NR; i++) {
		if (val[i] < limits[i][0] || val[i] > limits[i][1]) {
			return -EINVAL;
		} else {
			p[i - 1] = val[i];
		}
	}
	if (val[i - 1] <= -1 || val[i - 1] >= FULL_CONFIGS_NR)
		return -EINVAL;
	cxt.cfg.cur_ops = full_configs[val[i - 1]].cur_ops;
	return count;
}

static int torture_configs_show(struct seq_file *m, void *v)
{
	char kbuf[2000];
	int idx = 0;
	int *p;
	int i, j;

	if (torture_ctrl)
        return -EBUSY;

    idx += sprintf(&kbuf[idx],
        "***************************************************\n"
        "NOTE:\nmode:       1-FULL_TEST,2-SINGLE_TEST\n"
        "lock_type:  0-spinlock,1-rwlock,2-mutex,"
		"3-rtmutex,4-rwsem,5-percpu_rwsem\n"
        "nwrites:    range-(0~1000)\n"
        "nreads:     range-(0-1000)\n"
        "stutter:    range-(>0),unit-ms\n"
        "cs_us:      range-(>0),unit-us\n"
        "duration-s: range-(>0),unit-s\n"
        "e.x:\n"
        "    echo \"mode=1\" > ${THIS_FILE}\n"
        "    echo \"mode=2 normal_w=30 bg_w=0"
		" fg_w=0 top_w=0 ux_w=0 rt_w=0 normal_r=0"
		" bg_r=0 fg_r=0 top_r=0 ux_r=0 rt_r=0 stutter=10"
		" cs_us=100 duration_s=30 lock_type=2\" > ${THIS_FILE}\n"
        "    echo \"mode=2 normal_w=1 bg_w=2 "
		"fg_w=3 top_w=4 ux_w=5 rt_w=6 normal_r=1"
		" bg_r=2 fg_r=3 top_r=4 ux_r=5 rt_r=6"
		" stutter=10 cs_us=100 duration_s=30"
		" lock_type=1\" > ${THIS_FILE}\n"
		"Warning:BG/FG/TA will not support default!\n"
        "*************************************************\n");

	idx += sprintf(&kbuf[idx], "Current config:\n");
    idx += sprintf(&kbuf[idx], "Mode = %d\n", cxt.mode);
    if (cxt.mode == FULL_TEST) {
        for (i = 0; i < ARRAY_SIZE(full_configs); i++) {
            /* Exclude mode */
            p = (int*)&full_configs[i];
            for (j = 0; j < CONFIG_ITEMS_NR - 1; j++) {
                    idx += sprintf(&kbuf[idx], "%s%d ",matchs[j + 1], p[j]);
            }
            idx += sprintf(&kbuf[idx], "\n");
        }
    } else {
        p = (int*)&cxt.cfg;
        for (i = 0; i < CONFIG_ITEMS_NR - 1; i++) {
            idx += sprintf(&kbuf[idx], "%s%d ",matchs[i + 1], p[i]);
        }
    }
    seq_printf(m, "%s\n",kbuf);
    return 0;

}

static int torture_configs_open(struct inode *inode, struct file *file)
{
    return single_open(file, torture_configs_show, inode);
}

static const struct proc_ops torture_configs_ops = {
    .proc_open              = torture_configs_open,
    .proc_write             = torture_configs_write,
    .proc_read              = seq_read,
    .proc_lseek             = seq_lseek,
    .proc_release           = single_release,
};


static int torture_results_show(struct seq_file *m, void *v)
{
	if (torture_ctrl)
		return -EBUSY;

    seq_printf(m, "%s\n", out_buf);
    return 0;
}

static int torture_results_open(struct inode *inode, struct file *file)
{
    return single_open(file, torture_results_show, inode);
}

static const struct proc_ops torture_results_ops = {
    .proc_open              = torture_results_open,
    .proc_read              = seq_read,
    .proc_lseek             = seq_lseek,
    .proc_release           = single_release,
};



static int create_proc_nodes(void)
{
    d_torture = proc_mkdir(TORTURE_PROC_DIR, d_oplus_locking);
    if (d_torture) {
        proc_create(TORTURE_PROC_CTRL, S_IRUGO | S_IWUGO, d_torture,
                &torture_ctrl_ops);
        proc_create(TORTURE_PROC_CONF, S_IRUGO | S_IWUGO, d_torture,
                &torture_configs_ops);
        proc_create(TORTURE_PROC_RESULTS, S_IRUGO | S_IWUGO, d_torture,
                &torture_results_ops);
        return 0;
    }
    return -EFAULT;
}

static void remove_proc_nodes(void)
{
    if (d_torture) {
        remove_proc_entry(TORTURE_PROC_CTRL, d_torture);
        remove_proc_entry(TORTURE_PROC_CONF, d_torture);
        remove_proc_entry(TORTURE_PROC_RESULTS, d_torture);
        remove_proc_entry(TORTURE_PROC_DIR, d_oplus_locking);
    }
}

/*************************************************************************/
/*****************************proc files**********************************/
/*************************************************************************/


static atomic_t threads_cnts[NR_GRP_TYPES];
static void print_thread_attr(void)
{
	struct oplus_task_struct *ots;
	struct task_struct *p;

	if (current->prio < MAX_RT_PRIO) {
		atomic_inc(&threads_cnts[GRP_RT]);
		trace_printk("locktorture : ---Create RT thread, num : %d---\n",
			atomic_read(&threads_cnts[GRP_RT]));
		return;
	}
	ots = get_oplus_task_struct(current);
	if (!IS_ERR_OR_NULL(ots) && (ots->ux_state & SCHED_ASSIST_UX_MASK)) {
		atomic_inc(&threads_cnts[GRP_UX]);
		trace_printk("locktorture : ---Create UX thread, num : %d---\n",
			atomic_read(&threads_cnts[GRP_UX]));
		return;
	}
	p = current;
	if (ta_task(p)) {
		atomic_inc(&threads_cnts[GRP_TA]);
		trace_printk("locktorture : ---Create TA thread, num : %d---\n",
			atomic_read(&threads_cnts[GRP_TA]));
	}
	else if (fg_task(p)) {
		atomic_inc(&threads_cnts[GRP_FG]);
		trace_printk("locktorture : ---Create FG thread, num : %d---\n",
			atomic_read(&threads_cnts[GRP_FG]));
	}
	else if (bg_task(p)) {
		atomic_inc(&threads_cnts[GRP_BG]);
		trace_printk("locktorture : ---Create BG thread, num : %d---\n",
			atomic_read(&threads_cnts[GRP_BG]));
	}
	else {
		atomic_inc(&threads_cnts[GRP_OT]);
		trace_printk("locktorture : ---Create OT thread, num : %d---\n",
			atomic_read(&threads_cnts[GRP_OT]));
	}

}

/*
 * Lock torture writer kthread.  Repeatedly acquires and releases
 * the lock, checking for duplicate acquisitions.
 */
static int lock_torture_writer(void *arg)
{
	struct lock_stress_stats *lwsp = arg;

	if (set_thread_attr(lwsp->attr, 1) < 0)
		trace_printk("locktorture : Failed to call set_thread_attr \n");
	msleep(1);
	print_thread_attr();
	do {
		acquire_start();
		cxt.cfg.cur_ops->writelock();
		acquire_finish(cxt.cfg.lock_type, 1, lwsp->attr);
		if (WARN_ON_ONCE(lock_is_write_held))
			lwsp->n_lock_fail++;
		lock_is_write_held = true;
		if (WARN_ON_ONCE(atomic_read(&lock_is_read_held)))
			lwsp->n_lock_fail++; /* rare, but... */

		lwsp->n_lock_acquired++;
		cxt.cfg.cur_ops->write_delay(cxt.cfg.cs_us);
		lock_is_write_held = false;
		cxt.cfg.cur_ops->writeunlock();

		schedule_timeout_interruptible(msecs_to_jiffies(cxt.cfg.stutter));
	} while (!o_torture_must_stop());


	trace_printk("locktorture : ---Exit %s thread, num : %d---\n",
		&group_str[lwsp->attr][4],
		atomic_read(&threads_cnts[lwsp->attr]));
	atomic_dec(&threads_cnts[lwsp->attr]);

	o_torture_kthread_stopping();
	return 0;
}

/*
 * Lock torture reader kthread.  Repeatedly acquires and releases
 * the reader lock.
 */
static int lock_torture_reader(void *arg)
{
	struct lock_stress_stats *lrsp = arg;

	if (set_thread_attr(lrsp->attr, 0) < 0)
		trace_printk("locktorture : Failed to call set_thread_attr \n");
	msleep(1);
	print_thread_attr();
	do {
		acquire_start();
		cxt.cfg.cur_ops->readlock();
		acquire_finish(cxt.cfg.lock_type, 0, lrsp->attr);
		atomic_inc(&lock_is_read_held);
		if (WARN_ON_ONCE(lock_is_write_held))
			lrsp->n_lock_fail++; /* rare, but... */

		lrsp->n_lock_acquired++;
		cxt.cfg.cur_ops->read_delay(cxt.cfg.cs_us);
		atomic_dec(&lock_is_read_held);
		cxt.cfg.cur_ops->readunlock();

		schedule_timeout_interruptible(msecs_to_jiffies(cxt.cfg.stutter));
	} while (!o_torture_must_stop());

	trace_printk("locktorture : ---Exit %s thread, num : %d---\n",
		&group_str[lrsp->attr][4],
		atomic_read(&threads_cnts[lrsp->attr]));
	atomic_dec(&threads_cnts[lrsp->attr]);

	o_torture_kthread_stopping();
	return 0;
}

static void lock_torture_cleanup(void)
{
	int i;
	int *p;
    int total_writers = 0;
    int total_readers = 0;

    p = &cxt.cfg.normal_writers;
    for (i = 0; i < LOCK_TYPES; i++) {
        total_writers += *p;
        p++;
    }

    if (cxt.cfg.cur_ops->readlock) {
        p = &cxt.cfg.normal_readers;
        for (i = 0; i < LOCK_TYPES; i++) {
            total_readers += *p;
            p++;
        }
    }


	o_torture_cleanup_begin();

	/*
	 * Indicates early cleanup, meaning that the test has not run,
	 * such as when passing bogus args when loading the module.
	 * However cxt->cur_ops.init() may have been invoked, so beside
	 * perform the underlying torture-specific cleanups, cur_ops.exit()
	 * will be invoked if needed.
	 */
	if (!cxt.lwsa && !cxt.lrsa)
		goto end;

	if (writer_tasks) {
		for (i = 0; i < total_writers; i++)
			torture_stop_thread(&writer_tasks[i]);
		kfree(writer_tasks);
		writer_tasks = NULL;
		printk("stop %d writers thread \n", i);
	}

	if (reader_tasks) {
		for (i = 0; i < total_readers; i++)
			torture_stop_thread(&reader_tasks[i]);
		kfree(reader_tasks);
		reader_tasks = NULL;
		printk("stop %d readers thread \n", i);
	}

	lock_torture_stats_print();


	kfree(cxt.lwsa);
	cxt.lwsa = NULL;
	kfree(cxt.lrsa);
	cxt.lrsa = NULL;

end:
	if (cxt.init_called) {
		if (cxt.cfg.cur_ops->exit)
			cxt.cfg.cur_ops->exit();
		cxt.init_called = false;
	}
	o_torture_cleanup_end();
}

static int lock_torture_start(void)
{
	int i, j;
	int *p;
	int firsterr = 0;
	int total_writers = 0;
	int total_readers = 0;
	char name[128];

	p = &cxt.cfg.normal_writers;
    for (i = 0; i < LOCK_TYPES; i++) {
        total_writers += *p;
        p++;
    }
    if (cxt.cfg.cur_ops->readlock) {
        p = &cxt.cfg.normal_readers;
        for (i = 0; i < LOCK_TYPES; i++) {
            total_readers += *p;
            p++;
        }
    }
	if (cxt.cfg.cur_ops->init) {
		cxt.cfg.cur_ops->init();
		cxt.init_called = true;
	}

	/* Initialize the statistics so that each run gets its own numbers. */
	if (total_writers) {
		lock_is_write_held = false;
		cxt.lwsa = kmalloc_array(total_writers,
					sizeof(*cxt.lwsa),
					GFP_KERNEL);
		if (cxt.lwsa == NULL) {
			printk("cxt.lwsa: Out of memory");
			firsterr = -ENOMEM;
			goto unwind;
		}

		for (i = 0; i < total_writers; i++) {
			cxt.lwsa[i].n_lock_fail = 0;
			cxt.lwsa[i].n_lock_acquired = 0;
		}
	}
	if (cxt.cfg.cur_ops->readlock) {
		if (total_readers) {
			cxt.lrsa = kmalloc_array(total_readers,
						 sizeof(*cxt.lrsa),
						 GFP_KERNEL);
			if (cxt.lrsa == NULL) {
				printk("cxt.lrsa: Out of memory");
				firsterr = -ENOMEM;
				kfree(cxt.lwsa);
				cxt.lwsa = NULL;
				goto unwind;
			}

			for (i = 0; i < total_readers; i++) {
				cxt.lrsa[i].n_lock_fail = 0;
				cxt.lrsa[i].n_lock_acquired = 0;
			}
		}
	}
	if (total_writers) {
        writer_tasks = kcalloc(total_writers,
                               sizeof(writer_tasks[0]),
                               GFP_KERNEL);
        if (writer_tasks == NULL) {
                pr_err("writer_tasks: Out of memory");
                firsterr = -ENOMEM;
                goto unwind;
        }
    }

    if (cxt.cfg.cur_ops->readlock) {
        reader_tasks = kcalloc(total_readers,
                               sizeof(reader_tasks[0]),
                               GFP_KERNEL);
        if (reader_tasks == NULL) {
            trace_printk("locktorture : reader_tasks: Out of memory");
            kfree(writer_tasks);
            writer_tasks = NULL;
            firsterr = -ENOMEM;
            goto unwind;
        }
    }
	j = 0;
    p = &cxt.cfg.normal_writers;
    for (i = 0; i < total_writers; i++) {
        while (j >= *p) {
                j = 0;
                p++;
        }
        cxt.lwsa[i].attr = (p - &cxt.cfg.normal_writers);
		sprintf(name, "%s-%s-w", lock_str[cxt.cfg.lock_type], &group_str[cxt.lwsa[i].attr][4]);
        firsterr = torture_create_thread(lock_torture_writer, &cxt.lwsa[i], name, &writer_tasks[i]);
        if (firsterr) {
            goto unwind;
		}
        j++;
    }
    j = 0;
    p = &cxt.cfg.normal_readers;
    for (i = 0; i < total_readers; i++) {
        while (j >= *p) {
                j = 0;
                p++;
        }
        cxt.lrsa[i].attr = (p - &cxt.cfg.normal_readers);
		sprintf(name, "%s-%s-r", lock_str[cxt.cfg.lock_type], &group_str[cxt.lrsa[i].attr][4]);
        firsterr = torture_create_thread(lock_torture_reader, &cxt.lrsa[i], name, &reader_tasks[i]);
        if (firsterr) {
                goto unwind;
		}
        j++;
    }
	return 0;

unwind:
	lock_torture_cleanup();
	return firsterr;
}


static struct task_struct *torture_task;
static int torture_run(void *unused)
{
	int i;
	char *p;
	int stop;

	while (!kthread_should_stop()) {
		wait_event_interruptible(torture_wq, atomic_read(&thread_run));
		/* rmmod exit. */
		if (RMMOD_EXIT == atomic_read_acquire(&thread_run))
			continue;
		/* clear latency data before rerun.*/
		p = (char *)lat_stats;
		memset(p, 0, sizeof(lat_stats));
		p = (char*)lat_total_time;
		memset(p, 0, sizeof(lat_total_time));

		if (cxt.mode == SINGLE_TEST) {
			lock_torture_start();
			wait_for_completion_timeout(&torture_done, cxt.cfg.duration_s * HZ);
			lock_torture_cleanup();
			__torture_latency_print();
		} else {
			for (i = 0; i < ARRAY_SIZE(full_configs); i++) {
				memcpy(&cxt.cfg, &full_configs[i], sizeof(struct lock_torture_cfg));
				lock_torture_start();
				stop = wait_for_completion_timeout(&torture_done, cxt.cfg.duration_s * HZ);
				lock_torture_cleanup();
				if (stop)
					break;
			}
			__torture_latency_print();
		}

		/* Reset wakeup condition. */
		atomic_set(&thread_run, 0);
		/* Reset ctrl flag*/
		torture_ctrl = 0;
		/* Reset buf idx.*/
		out_buf_idx = 0;

	}
	return 0;
}


static int __init lock_torture_init(void)
{
	int ret = 0;
	struct cgroup * cgrp;
	char path[100];

	torture_task = kthread_run(torture_run, NULL, "torture_task");
	if (IS_ERR(torture_task)) {
		printk("locktorture : Failed to create torture task thread! \n");
		return PTR_ERR(torture_task);
	}
	ret = create_proc_nodes();
	if (ret)
		printk("locktorture : Failed to create torture node! \n");

	cgrp = task_cgroup(current, cpu_cgrp_id);
	if (cgrp) {
		cgroup_path(cgrp, path, 100);
		printk("locktorture : path = %s\n", path);
	} else {
		printk("locktorture : cgrp == NULL\n");
	}
	return ret;
}

static void __exit lock_torture_exit(void)
{
	atomic_set_release(&thread_run, RMMOD_EXIT);
	wake_up(&torture_wq);
	kthread_stop(torture_task);
	remove_proc_nodes();
}

module_init(lock_torture_init);
module_exit(lock_torture_exit);
MODULE_LICENSE("GPL");
