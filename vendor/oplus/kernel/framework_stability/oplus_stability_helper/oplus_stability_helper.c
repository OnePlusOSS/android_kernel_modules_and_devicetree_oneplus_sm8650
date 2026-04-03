// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2030 Oplus. All rights reserved.
 */

#include <trace/hooks/signal.h>
#include <trace/hooks/binder.h>
#include <trace/hooks/sched.h>
#include <trace/events/vmscan.h>
#include <drivers/android/binder_alloc.h>
#include <drivers/android/binder_internal.h>

#include "frk_netlink.h"
#include "frk_stability.h"
#include "binder_watcher.h"
#include "thread_watcher.h"
#include "lowmem_watcher.h"

#define ROOT_UID	0
#define SYSTEM_UID	1000

static bool is_root_process(struct task_struct *t)
{
	if (task_uid(t).val == ROOT_UID) {
		if ((!strcmp(t->comm, "main")
			&& (t->parent != NULL && !strcmp(t->parent->comm, "init"))) /*zygote*/
			|| !strcmp(t->comm, "Binder:netd")) {
			return true;
		}
	}
	return false;
}

static bool is_system_process(struct task_struct *t)
{
	if (task_uid(t).val == SYSTEM_UID) {
		if (!strcmp(t->comm, "system_server")
			|| !strcmp(t->comm, "surfaceflinger")
			|| !strcmp(t->comm, "servicemanager")
			|| !strcmp(t->comm, "hwservicemanage")
			|| !strncmp(t->comm, "composer", 8)) {
			return true;
		}
	}
	return false;
}

static bool is_top_thirdparty_process(struct task_struct *t)
{
	if (!strcmp(t->comm, "hinamworld.main")
		|| !strcmp(t->comm, "m.taobao.taobao")
		|| !strcmp(t->comm, "droid.ugc.aweme")) {
		return true;
	}
	return false;
}

static bool is_key_process(struct task_struct *t)
{
	struct pid *pgrp;
	struct task_struct *taskp;

	if (t->pid == t->tgid) {
		if (is_system_process(t) || is_root_process(t)
			|| is_top_thirdparty_process(t)) {
			return true;
		}
	} else {
		pgrp = get_task_pid(t->group_leader, PIDTYPE_PID);
		if (pgrp != NULL) {
			taskp = pid_task(pgrp, PIDTYPE_PID);
			if (taskp != NULL && (is_system_process(taskp)
				|| is_root_process(taskp)|| is_top_thirdparty_process(taskp))) {
				return true;
			}
		}
	}
	return false;
}

void send_signal_catcher(void *ignore, int sig, struct task_struct *src, struct task_struct *dst)
{
	if (sig == 33 && src->tgid == dst->tgid) {
		return; /*exclude system_server's "Signal Catcher" sending to itself*/
	} else if ((sig == 33 || sig == SIGQUIT || sig == SIGKILL || sig == SIGABRT || sig == SIGHUP
		|| sig == SIGSTOP || sig == SIGTERM || sig == SIGPIPE || sig == SIGCONT || sig == SIGSEGV)
		&& is_key_process(dst)) {
		printk("<critical>Some other process %d:%d:%s (uid:%d ppid %d:%s) wants to send sig:%d to process %d:%d:%s\n",
			src->tgid, src->pid, src->comm, task_uid(src).val, src->parent != NULL ? src->parent->pid : -1,
			src->parent != NULL ? src->parent->comm : "null", sig, dst->tgid, dst->pid, dst->comm);
	}
}


struct workqueue_struct *thread_watcher_wq;
struct thread_watcher_args  *work_args;

struct kmem_cache *thread_watcher_struct_cachep;


static int __init helper_init(void)
{
	int ret = 0;

	REGISTER_TRACE_VH(do_send_sig_info, send_signal_catcher);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))  && (LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0))
	REGISTER_TRACE_VH(binder_detect_low_async_space, binder_buffer_watcher);
	REGISTER_TRACE_VH(copy_process, thread_watcher);

	thread_watcher_wq =
		create_workqueue("stability_thread_watcher");
	if (!thread_watcher_wq) {
		ret = -ENOMEM;
		pr_err("create thread watcher workqueue failed , ret=%d\n", ret);
		return ret;
	}

	thread_watcher_struct_cachep = kmem_cache_create(
        	"thread_watcher_struct_cachep",         /* Name */
		sizeof(struct thread_watcher_args),   /* Object Size */
		0,                              /* Alignment */
		SLAB_HWCACHE_ALIGN,             /* Flags */
		NULL);


	if (NULL == thread_watcher_struct_cachep) {
		ret = -ENOMEM;
		pr_err("create thread watcher struct cachep failed , ret=%d\n", ret);
		return ret;
	}
	create_frk_netlink(33);

	/* register trace_mm_vmscan_direct_reclaim_begin */
	ret = register_trace_mm_vmscan_direct_reclaim_begin(lowmem_report, NULL);
	if (ret) {
		pr_err("failed to register_trace_mm_vmscan_direct_reclaim_begin, ret=%d\n", ret);
		return ret;
	}
	init_mem_confg();
#endif
	return 0;
}

static void __exit helper_exit(void)
{
	UNREGISTER_TRACE_VH(do_send_sig_info, send_signal_catcher);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))  && (LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0))
	UNREGISTER_TRACE_VH(binder_detect_low_async_space, binder_buffer_watcher);
	UNREGISTER_TRACE_VH(copy_process, thread_watcher);
	if (thread_watcher_wq) {
		destroy_workqueue(thread_watcher_wq);
		thread_watcher_wq = NULL;
	}
	if (thread_watcher_struct_cachep) {
		kmem_cache_destroy(thread_watcher_struct_cachep);
	}
	destroy_frk_netlink();

	/* unregister trace_mm_vmscan_direct_reclaim_begin */
	unregister_trace_mm_vmscan_direct_reclaim_begin(lowmem_report, NULL);
#endif
}

module_init(helper_init);
module_exit(helper_exit);

MODULE_DESCRIPTION("oplus stability helper");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(MINIDUMP);
MODULE_IMPORT_NS(DMA_BUF);
