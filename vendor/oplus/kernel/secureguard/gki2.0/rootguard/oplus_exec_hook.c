// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 *
 * exec hook: check suspicious cmd or binary to be executed.
 */
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/mm.h>
#include <linux/stat.h>
#include <linux/fcntl.h>
#include <linux/swap.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/kprobes.h>
#include <linux/binfmts.h>
#include <linux/cred.h>
#include <linux/unistd.h>
#include "oplus_exec_hook.h"
#include "oplus_guard_general.h"
#include "oplus_kevent.h"

/* secureguard sub-module string for print */
#define SG_EXEC_HOOK	"[secureguard][exec_hook]"

static int oplus_RWO_root_check(struct task_struct *p)
{
	struct task_struct *tgid_task, *parent_task = NULL;
	char data_buff[128];
	const char *event_type = "exec2";

	if (!p || p->pid == 1)
		return 0;

	if (CHECK_ROOT_CREDS(p)) {
		if (p->tgid != p->pid) {
			/* get tgid's task and cred */
			tgid_task = find_task_by_vpid(p->tgid);
			get_task_struct(tgid_task);
			/* get tgid's uid */
			if (!CHECK_ROOT_CREDS(tgid_task)) {
				pr_err(SG_EXEC_HOOK "%s, Found task process %s, uid:%d, tgid_uid: %d.\n", __func__,
					p->comm, p->cred->uid.val, tgid_task->cred->uid.val);
				report_security_event(event_type, EXEC2_EVENT, "");
				return 1;
			}
		} else {
			parent_task = rcu_dereference(p->real_parent);
			if (!CHECK_ROOT_CREDS(parent_task)) {
				sprintf(data_buff, "parent(%s),comm(%s)", parent_task->comm, p->comm);
				pr_err(SG_EXEC_HOOK "%s, Detect curr process %s(%d), parent process %s(%d).\n", __func__,
					p->comm, p->cred->uid.val, parent_task->comm, parent_task->cred->uid.val);
				report_security_event(event_type, EXEC2_EVENT, data_buff);
				return 1;
			}
		}
	}
	return 0;
}

static void oplus_report_execveat(const char *path, const char *dcs_event_id)
{
	struct kernel_packet_info *dcs_event;
	char dcs_stack[sizeof(struct kernel_packet_info) + 256];
	const char *dcs_event_tag = "kernel_event";
	/* const char* dcs_event_id = "execve_report"; */
	char *dcs_event_payload = NULL;
	int uid = current_uid().val;
	/*const struct cred *cred = current_cred();*/

	dcs_event = (struct kernel_packet_info *)dcs_stack;
	dcs_event_payload = dcs_stack +
		sizeof(struct kernel_packet_info);

	dcs_event->type = 3;

	strncpy(dcs_event->log_tag, dcs_event_tag,
		sizeof(dcs_event->log_tag));
	strncpy(dcs_event->event_id, dcs_event_id,
		sizeof(dcs_event->event_id));

	dcs_event->payload_length = snprintf(dcs_event_payload, 256,
		"%d,path@@%s", uid, path);
	if (dcs_event->payload_length < 256)
		dcs_event->payload_length += 1;

	kevent_send_to_user(dcs_event);

	pr_err(SG_EXEC_HOOK "%s, common %s result %s\n", __func__, path, dcs_event_id);
}

static int oplus_check_execveat_perm(void)
{
	char *absolute_path_buf = NULL;
	char *absolute_path = NULL;
	int need_block = 0;
	struct path *p_f_path = &(current->mm->exe_file->f_path);

	if (p_f_path == NULL)
		goto out_ret;

	absolute_path_buf = (char *)__get_free_page(__GFP_ATOMIC);
	if (absolute_path_buf == NULL)
		goto out_ret;

	absolute_path = d_path(p_f_path, absolute_path_buf, PAGE_SIZE);
	if (IS_ERR(absolute_path))
		goto out_free;
	/* pr_alert("[DEBUG]:current_uid().val is %u, absolute_path is %s\n", current_uid().val, absolute_path);*/

	if (strncmp(absolute_path, "/data", 5))
		goto out_free;

	if ((!strncmp(absolute_path, "/data/local/tmp", 15))
		|| (!strncmp(absolute_path, "/data/nativetest", 16))
		|| (!strncmp(absolute_path, "/data/nativetest64", 18))) {
		if (oplus_RWO_root_check(current)) {
			/*Note: Do not block execve now*/
			/*retval = -EPERM;*/
		}
		goto out_free;
	}

	if (!uid_eq(current_uid(), GLOBAL_ROOT_UID)) {
		oplus_report_execveat(absolute_path, "execve_report");
	} else {
		oplus_report_execveat(absolute_path, "execve_block");
		need_block = -1;
		goto out_free;
	}
out_free:
	free_page((unsigned long)absolute_path_buf);

out_ret:
	return need_block;
}

static int oplus_exec_block(void)
{
	int  retval = 0;

	if (!is_unlocked()) {
		retval = oplus_check_execveat_perm();
		if (retval < 0)
			return retval;
	}
	return retval;
}

/* execve syscall number, include unistd.h */
/* #define __NR_execve     221 */

void oplus_exe_block_ret_handler(void *data, struct pt_regs *regs, long ret)
{
	int retval = 0;
	int syscallno = regs->syscallno;

	/* if (current->android_kabi_reserved4 == __NR_execve) { */
	if (syscallno == __NR_execve) {
		retval = oplus_exec_block();
		if (retval) {
			pr_err(SG_EXEC_HOOK "%s, retval: %d, syscall no: %d, pid: %d, uid: %d.\n", __func__,
				retval, syscallno, current->pid, current_uid().val);
			/* do_exit(SIGKILL); */
			send_sig(SIGKILL, current, 0);
		}
	}
}
