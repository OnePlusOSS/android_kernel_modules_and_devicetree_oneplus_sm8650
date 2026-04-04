// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/cgroup-defs.h>
#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>
#include <linux/cgroup.h>
#include <trace/hooks/cgroup.h>
#include <trace/hooks/signal.h>
#include <trace/events/sched.h>
#include <linux/kprobes.h>

#include "sched_assist.h"
#include "sa_common.h"
#include "sa_sysfs.h"
#include "sa_exec.h"
#include "sa_fair.h"
#include "sa_oemdata.h"
#include "sa_priority.h"
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
#include "sa_balance.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_SCHED_GROUP_OPT)
#include "sa_group.h"
#endif

#define HI_MASK		0xFF00000000000000UL
#define HI_FLAG		0xAB00000000000000UL

static wake_up_new_task_handler_t wunt_handler;

static void set_ux_to_task(struct task_struct *new) {
	struct pt_regs *childregs = task_pt_regs(new);
	unsigned long newsp;
	unsigned long fn_addr;
	void __user *user_sp_ptr;
	int ux_state;

	if (is_compat_thread(task_thread_info(new)))
		newsp = childregs->compat_sp;
	else
		newsp = childregs->sp;

	if ((void *)newsp == NULL)
		return;

	user_sp_ptr = (void __user *)(uintptr_t)(newsp);
	if (0 != copy_from_user(&fn_addr, user_sp_ptr, sizeof(unsigned long)))
		return;

	if ((fn_addr & HI_MASK) != HI_FLAG)
		return;

	ux_state = oplus_get_static_ux_state(current->group_leader);
	if (ux_state) {
		oplus_set_ux_state_lock(new, ux_state, -1, true);
		return;
	}

	ux_state = oplus_get_inherited_ux_state(current->group_leader);
	if (ux_state) {
		oplus_set_ux_state_lock(new, ux_state, INHERIT_UX_MAX, true);
	}
}

static void android_rvh_wake_up_new_task_handler(void *unused, struct task_struct *new) {
	set_ux_to_task(new);

#if IS_ENABLED(CONFIG_OPLUS_SCHED_GROUP_OPT)
	oplus_sg_wake_up_new_task(new);
#endif
	/*
	 * Due to function vendor hook limitation,
	 * used ext handler pointer to invoke to other module
	 */
	if (wunt_handler) {
		wunt_handler(new);
	}
}

void register_wake_up_new_task_ext_handler(wake_up_new_task_handler_t ext_handler)
{
	wunt_handler = ext_handler;
}
EXPORT_SYMBOL(register_wake_up_new_task_ext_handler);

static int register_scheduler_vendor_hooks(void)
{
	int ret;

	/* register vender hook in kernel/sched/fair.c */
	REGISTER_TRACE_RVH(android_rvh_place_entity, android_rvh_place_entity_handler);
	REGISTER_TRACE_RVH(android_rvh_check_preempt_tick, android_rvh_check_preempt_tick_handler);
	REGISTER_TRACE_RVH(android_rvh_can_migrate_task, android_rvh_can_migrate_task_handler);
#ifndef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	/* REGISTER_TRACE_RVH(android_rvh_post_init_entity_util_avg, android_rvh_post_init_entity_util_avg_handler); */
#endif
	/* REGISTER_TRACE_RVH(android_rvh_select_task_rq_fair, android_rvh_select_task_rq_fair_handler); */
	/* REGISTER_TRACE_RVH(android_rvh_find_energy_efficient_cpu, android_rvh_find_energy_efficient_cpu_handler); */

	REGISTER_TRACE_RVH(android_rvh_enqueue_entity, android_rvh_enqueue_entity_handler);
	REGISTER_TRACE_RVH(android_rvh_dequeue_entity, android_rvh_dequeue_entity_handler);

#ifndef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	REGISTER_TRACE_RVH(android_rvh_check_preempt_wakeup, android_rvh_check_preempt_wakeup_handler);
#ifndef CONFIG_MTK_SCHED_VIP_TASK
	REGISTER_TRACE_RVH(android_rvh_replace_next_task_fair, android_rvh_replace_next_task_fair_handler);
#endif
#endif

	/* register vender hook in kernel/sched/topology.c */
	REGISTER_TRACE_VH(android_vh_build_sched_domains, android_vh_build_sched_domains_handler);

	/* register vender hook in  kernel/sched/rt.c */
#ifndef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	/* REGISTER_TRACE_RVH(android_rvh_select_task_rq_rt, android_rvh_select_task_rq_rt_handler); */
	REGISTER_TRACE_RVH(android_rvh_find_lowest_rq, android_rvh_find_lowest_rq_handler);
#endif

	/* register vender hook in kernel/sched/core.c */
	REGISTER_TRACE_RVH(android_rvh_sched_fork, android_rvh_sched_fork_handler);
#ifndef CONFIG_MTK_SCHED_FAST_LOAD_TRACKING
	REGISTER_TRACE_RVH(android_rvh_schedule, android_rvh_schedule_handler);
#endif
	REGISTER_TRACE_RVH(android_vh_scheduler_tick, android_vh_scheduler_tick_handler);
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	REGISTER_TRACE_RVH(android_rvh_after_enqueue_task, android_rvh_after_enqueue_task_handler);
	REGISTER_TRACE_RVH(android_rvh_dequeue_task, android_rvh_dequeue_task_handler);
#endif
	REGISTER_TRACE_RVH(android_rvh_set_cpus_allowed_by_task, android_rvh_set_cpus_allowed_by_task_handler);
	REGISTER_TRACE_RVH(android_rvh_set_cpus_allowed_comm, android_rvh_set_cpus_allowed_comm_handler);
#if IS_ENABLED(CONFIG_OPLUS_SCHED_GROUP_OPT)
	REGISTER_TRACE_VH(android_vh_reweight_entity, android_vh_reweight_entity_handler);
#endif
	/* register vender hook in fs/exec.c */
	REGISTER_TRACE_VH(task_rename, task_rename_handler);

	/* register vendor hook in kernel/cgroup/cgroup-v1.c */
	/* REGISTER_TRACE_VH(android_vh_cgroup_set_task, android_vh_cgroup_set_task_handler); */

	/* register vendor hook in kernel/signal.c  */
	REGISTER_TRACE_VH(android_vh_exit_signal, android_vh_exit_signal_handler);

	REGISTER_TRACE_VH(sched_stat_runtime, android_vh_sched_stat_runtime_handler);

	REGISTER_TRACE_VH(android_rvh_wake_up_new_task, android_rvh_wake_up_new_task_handler);

#ifdef CONFIG_BLOCKIO_UX_OPT
	sa_blockio_init();
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_BAN_APP_SET_AFFINITY)
	/* register vendor hook in kernel/core.c */
	REGISTER_TRACE_VH(android_vh_sched_setaffinity_early, android_vh_sched_setaffinity_early_handler);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	oplus_loadbalance_init();
#endif

#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
	REGISTER_TRACE_VH(android_vh_account_process_tick_gran, android_vh_account_process_tick_gran_handler);
	REGISTER_TRACE_VH(sched_switch, sa_sched_switch_handler);
#endif
	return 0;
}

typedef int (*profile_event_register_t)(enum profile_type type,
		struct notifier_block *n);
profile_event_register_t  _profile_event_register;

int detect_symbol(void)
{
	int ret;
	static struct kprobe kp = {
		.symbol_name = "profile_event_register"
	};

	ret = register_kprobe(&kp);
	if (ret < 0) {
		pr_warn("register  failed\n");
		return ret;
	}
	_profile_event_register = (profile_event_register_t)kp.addr;
	pr_info("_profile_event_register:%ps\n", _profile_event_register);
	unregister_kprobe(&kp);

	return 0;
}

void enable_sched_assist(int step) {
	static int ux_hooks = 0;
	ux_hooks |= step;

	if (OPLUS_UX_HOOK_MASK == ux_hooks) {
		global_sched_assist_enabled |= FEATURE_COMMON;
	#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
		global_sched_assist_enabled |= FEATURE_SPREAD;
	#endif
	}
}
EXPORT_SYMBOL_GPL(enable_sched_assist);

static int __init oplus_sched_assist_init(void)
{
	int ret;

	ret = sa_oemdata_init();
	if (ret != 0)
		return ret;

	sched_assist_init_oplus_rq();
	update_ux_sched_cputopo();
#ifdef CONFIG_OPLUS_FEATURE_TICK_GRAN
	resched_timer_init();
#endif

	ret = register_scheduler_vendor_hooks();
	if (ret != 0)
		return ret;

	ret = oplus_sched_assist_proc_init();
	if (ret != 0)
		return ret;
	detect_symbol();
	if (_profile_event_register)
		/* register a notifier to monitor task exit */
		(*_profile_event_register)(PROFILE_TASK_EXIT, &process_exit_notifier_block);

#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	enable_sched_assist(OPLUS_UX_HOOK_MASK);
#endif

	ux_debug("sched assist init succeed!\n");
	return 0;
}

module_init(oplus_sched_assist_init);

MODULE_DESCRIPTION("Oplus Sched Assist Vender Hooks Driver");
MODULE_LICENSE("GPL v2");
