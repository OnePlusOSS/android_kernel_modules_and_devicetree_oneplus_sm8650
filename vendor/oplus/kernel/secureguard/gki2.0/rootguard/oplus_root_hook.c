// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 *
 * root check: check whether caller's privilege(xxid) changed after syscall
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cred.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <trace/events/syscalls.h>
#include <linux/sched/signal.h>
#include <linux/unistd.h>
#include "oplus_root_hook.h"
#include "oplus_guard_general.h"
#include "oplus_kevent.h"
/* for use oplus_task_struct */
#if defined(QCOM_PLATFORM)
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#elif defined(MTK_PLATFORM)
#include <../drivers/soc/oplus/cpu/sched/sched_assist/sa_common.h>
#endif

#if defined(WHITE_LIST_SUPPORT)
#include <linux/string.h>
#include <linux/sched/task.h>
#endif /* WHITE_LIST_SUPPORT */

#define KERNEL_ADDR_LIMIT 0x0000008000000000

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
static int selinux_enabled = 1;
#else
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
unsigned long get_fs(void)
{
	return 0;
}
#endif

/* secureguard sub-module string for print */
#define SG_ROOT_HOOK    "[secureguard][root_hook]"

extern int kevent_send_to_user(struct kernel_packet_info *userinfo);

void oplus_root_check_succ(uid_t uid, uid_t euid, gid_t egid, int callnum)
{
	struct kernel_packet_info *dcs_event;
	char dcs_stack[sizeof(struct kernel_packet_info) + 256];
	const char *dcs_event_tag = "kernel_event";
	const char *dcs_event_id = "root_check";
	char *dcs_event_payload = NULL;

	int ret = -1;
	char comm[TASK_COMM_LEN], nameofppid[TASK_COMM_LEN];
	struct task_struct *parent_task = NULL;
	int ppid = -1;

	memset(comm, 0, TASK_COMM_LEN);
	memset(nameofppid, 0, TASK_COMM_LEN);

	/*ppid = task_ppid_nr(current);*/
	parent_task = rcu_dereference(current->real_parent);
	if (parent_task) {
		get_task_comm(nameofppid, parent_task);
		ppid = parent_task->pid;
	}

	dcs_event = (struct kernel_packet_info *)dcs_stack;
	dcs_event->type = 0;
	strncpy(dcs_event->log_tag, dcs_event_tag, sizeof(dcs_event->log_tag));
	strncpy(dcs_event->event_id, dcs_event_id, sizeof(dcs_event->event_id));
	dcs_event_payload = kmalloc(256, GFP_ATOMIC);
	if (dcs_event_payload == NULL)
		return;
	memset(dcs_event_payload, 0, 256);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	dcs_event->payload_length = snprintf(dcs_event_payload, 256,
		"%d$$old_euid@@%d$$old_fsuid@@%d$$sys_call_number@@%d$$addr_limit@@%lx$$curr_uid@@%d$$"
		"curr_euid@@%d$$curr_fsuid@@%d$$curr_name@@%s$$ppid@@%d$$ppidname@@%s$$enforce@@%d\n",
		uid, euid, egid, callnum, get_fs(), current_uid().val, current_euid().val, current_fsuid().val,
		get_task_comm(comm, current), ppid, nameofppid, selinux_is_enabled());
#else
	dcs_event->payload_length = snprintf(dcs_event_payload, 256,
		"%d$$old_euid@@%d$$old_egid@@%d$$sys_call_number@@%d$$addr_limit@@%lx$$curr_uid@@%d$$"
		"curr_euid@@%d$$curr_egid@@%d$$curr_name@@%s$$ppid@@%d$$ppidname@@%s$$enforce@@%d\n",
		uid, euid, egid, callnum, get_fs(), current_uid().val, current_euid().val, current_egid().val,
		get_task_comm(comm, current), ppid, nameofppid, selinux_enabled);
#endif
	pr_info(SG_ROOT_HOOK "%s, payload:%s.\n", __func__, dcs_event_payload);
	memcpy(dcs_event->payload, dcs_event_payload, strlen(dcs_event_payload));

	ret = kevent_send_to_user(dcs_event);
	if (ret)
		pr_err(SG_ROOT_HOOK "%s, send to user failed, ret %d.\n", __func__, ret);

	kfree(dcs_event_payload);
}

void oplus_root_killed(void)
{
	pr_err(SG_ROOT_HOOK "%s, Kill the process of escalation, pid = %d, uid = %d.\n", __func__, current->pid, current_uid().val);
	send_sig(SIGKILL, current, 0);
}

void oplus_root_check_pre_handler(void *data, struct pt_regs *regs, long id)
{
	struct oplus_task_struct *ots_cur;

	ots_cur = get_oplus_task_struct(current);
	if (IS_ERR_OR_NULL(ots_cur))
		return;

	ots_cur->sg_scno = regs->syscallno;
	ots_cur->sg_uid = current_uid().val;
	ots_cur->sg_euid = current_euid().val;
	ots_cur->sg_gid = current_gid().val;
	ots_cur->sg_egid = current_egid().val;
	ots_cur->sg_flag = 1;
}

#define   __NR_SETREUID32       203
#define   __NR_SETREGID32       204
#define   __NR_SETRESUID32      208
#define   __NR_SETRESGID32      210
#define   __NR_SETUID32         213
#define   __NR_SETGID32         214

/* use linux/unistd.h asm/unistd.h instead
 *#define   __NR_setregid         143
 *#define   __NR_setgid           144
 *#define   __NR_setreuid         145
 *#define   __NR_setuid           146
 *#define   __NR_setresuid        147
 *#define   __NR_setresgid        149
 */

void oplus_root_check_post_handler(void *data, struct pt_regs *regs, long ret)
{
	int scno;
	struct oplus_task_struct *ots_cur;
#if defined(WHITE_LIST_SUPPORT)
	char nameofppid[TASK_COMM_LEN];
	struct task_struct *parent_task = NULL;
#endif

	ots_cur = get_oplus_task_struct(current);
	if (IS_ERR_OR_NULL(ots_cur))
		return;
	if (ots_cur->sg_flag != 1)
		return;

	scno = ots_cur->sg_scno;

	/* no need judge if unlocked here */
	if ((ots_cur->sg_uid != 0) && !is_unlocked()) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
		if ((ots_cur->sg_uid > current_uid().val) || (ots_cur->sg_euid > current_euid().val)
		 || (ots_cur->sg_gid > current_gid().val) || (ots_cur->sg_egid > current_egid().val)) {
#else
		if ((ots_cur->sg_uid > current_uid().val) || (ots_cur->sg_euid > current_euid().val)
		 || (ots_cur->sg_gid > current_gid().val) || (ots_cur->sg_egid > current_egid().val) || (get_fs() > KERNEL_ADDR_LIMIT)) {
#endif
			if ((scno != __NR_SETREUID32) && (scno != __NR_SETREGID32) && (scno != __NR_SETRESUID32)
			 && (scno != __NR_SETRESGID32) && (scno != __NR_SETUID32) && (scno != __NR_SETGID32)
			 && (scno != __NR_setreuid) && (scno != __NR_setregid) && (scno != __NR_setresuid)
			 && (scno != __NR_setresgid) && (scno != __NR_setuid) && (scno != __NR_setgid)) {
				#if defined(WHITE_LIST_SUPPORT)
				memset(nameofppid, 0, TASK_COMM_LEN);
				parent_task = rcu_dereference(current->real_parent);
				if (parent_task)
					get_task_comm(nameofppid, parent_task);
				if (strncmp(nameofppid, "dumpstate", 9)) {
					oplus_root_check_succ(ots_cur->sg_uid, ots_cur->sg_euid, ots_cur->sg_egid, ots_cur->sg_scno);
					oplus_root_killed();
				}
				#else
				oplus_root_check_succ(ots_cur->sg_uid, ots_cur->sg_euid, ots_cur->sg_egid, ots_cur->sg_scno);
				oplus_root_killed();
				#endif
			}
		}
	}
}
