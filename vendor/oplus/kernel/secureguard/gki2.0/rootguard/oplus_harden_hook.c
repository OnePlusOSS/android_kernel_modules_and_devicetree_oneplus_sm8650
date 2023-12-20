// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 *
 * harden hook: check caller's capability(xxid) before set_xx syscall
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/cred.h>
#include <linux/socket.h>
#include <linux/unistd.h>
#include "oplus_harden_hook.h"
#include "oplus_guard_general.h"
#include "oplus_kevent.h"

/*kevent string*/
#define OPLUS_HARDEN_CAPABILITY_EVENT_NAME	"kernel_event"
#define OPLUS_HARDEN_CAPABILITY_EVENT_ID	"capa_harden"
#define OPLUS_HARDEN_CAPABILITY_EVENT_TYPE	5

#define OPLUS_ANDROID_THIRD_PART_APK_UID	10000
#define OPLUS_ANDROID_ROOT_UID			0

#define OPLUS_SETID_ID	1
#define OPLUS_SETID_RE	2
#define OPLUS_SETID_RES	4

#define OPLUS_HARDEN_UID		0
#define OPLUS_HARDEN_EUID		1
#define OPLUS_HARDEN_RESUID		2
#define OPLUS_HARDEN_GID		3
#define OPLUS_HARDEN_EGID		4
#define OPLUS_HARDEN_RESGID		5
#define OPLUS_HARDEN_NET_RAW		0xf0
#define OPLUS_HARDEN_NET_RAW_FLAG	0xff

/* secureguard sub-module string for print */
#define SG_HARDEN_HOOK    "[secureguard][harden_hook]"

static int selinux_enabled = 1;

static void oplus_secure_harden_kevent(uid_t new_id, uid_t new_eid, uid_t new_sid, int flag, unsigned char uid_or_gid)
{
	int ret = -1;
	struct kernel_packet_info *dcs_event;
	char dcs_stack[sizeof(struct kernel_packet_info) + 256];
	char *dcs_event_payload = NULL;
	char comm[TASK_COMM_LEN];

	memset(comm, 0, TASK_COMM_LEN);

	dcs_event = (struct kernel_packet_info *)dcs_stack;
	dcs_event->type = OPLUS_HARDEN_CAPABILITY_EVENT_TYPE;
	strncpy(dcs_event->log_tag, OPLUS_HARDEN_CAPABILITY_EVENT_NAME, sizeof(dcs_event->log_tag));
	strncpy(dcs_event->event_id, OPLUS_HARDEN_CAPABILITY_EVENT_ID, sizeof(dcs_event->event_id));

	dcs_event_payload = kmalloc(256, GFP_ATOMIC);
	if (dcs_event_payload == NULL)
		return;

	memset(dcs_event_payload, 0, 256);

	if ((uid_or_gid >= OPLUS_HARDEN_UID) && (uid_or_gid <= OPLUS_HARDEN_RESUID)) {
		dcs_event->payload_length = snprintf(dcs_event_payload, 256,
		"%d$$new_euid@@%d$$new_suid@@%d$$set_id_flag@@%d$$addr_limit@@%lx$$curr_uid@@%d$$curr_euid@@%d$$curr_suid@@%d$$curr_name@@%s$$enforce@@%d\n",
		new_id, new_eid, new_sid, flag, get_fs(), current_uid().val, current_euid().val, current_suid().val, get_task_comm(comm, current), selinux_enabled);
	} else if ((uid_or_gid >= OPLUS_HARDEN_GID) && (uid_or_gid <= OPLUS_HARDEN_RESGID)) {
		dcs_event->payload_length = snprintf(dcs_event_payload, 256,
		"%d$$new_egid@@%d$$new_sgid@@%d$$set_id_flag@@%d$$addr_limit@@%lx$$curr_gid@@%d$$curr_egid@@%d$$curr_sgid@@%d$$curr_name@@%s$$enforce@@%d\n",
		new_id, new_eid, new_sid, flag, get_fs(), current_gid().val, current_egid().val, current_sgid().val, get_task_comm(comm, current), selinux_enabled);
	} else {
		dcs_event->payload_length = snprintf(dcs_event_payload, 256,
		"%d$$type@@%d$$protocol@@%d$$set_id_flag@@%d$$addr_limit@@%lx$$curr_gid@@%d$$curr_egid@@%d$$curr_sgid@@%d$$curr_name@@%s$$enforce@@%d\n",
		new_id, new_eid, new_sid, flag, get_fs(), current_gid().val, current_egid().val, current_sgid().val, get_task_comm(comm, current), selinux_enabled);
	}

	pr_info(SG_HARDEN_HOOK "%s, dcs_event_payload:%s.\n", __func__, dcs_event_payload);

	memcpy(dcs_event->payload, dcs_event_payload, strlen(dcs_event_payload));

	ret = kevent_send_to_user(dcs_event);
	if (ret != 0)
		pr_err(SG_HARDEN_HOOK "%s, send to user failed, ret %d.\n", __func__, ret);

	kfree(dcs_event_payload);
}
/*
 * There are 2 blocking solution:
 * 1.Intercept knowledge and raise the right to increase the system permission.
 * 2.Only the interception permission is raised to ROOT.(Currently used)
 */
/*UID*/
static int oplus_harden_setuid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_uid = 0;
	unsigned int new_uid = 0;

	cur_uid = current_uid().val;
	new_uid = regs->regs[0];

	if (capable(CAP_SETUID) == true) {
		if (cur_uid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if (new_uid == OPLUS_ANDROID_ROOT_UID) {
				pr_err(SG_HARDEN_HOOK "%s, incident, old uid %u, new uid %u.\n", __func__, cur_uid, new_uid);
				oplus_secure_harden_kevent(new_uid, new_uid, new_uid, OPLUS_SETID_ID, OPLUS_HARDEN_UID);
				/* restore the uid */
				/*regs->regs[0] = cur_uid;*/
			}
		}
	}
	return ret;
}

/*EUID*/
static int oplus_harden_setreuid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_uid = 0;
	unsigned int cur_euid = 0;
	unsigned int new_uid = 0;
	unsigned int new_euid = 0;

	cur_uid = current_uid().val;
	cur_euid = current_euid().val;

	new_uid = regs->regs[0];
	new_euid = regs->regs[1];

	if (capable(CAP_SETUID) == true) {
		if (cur_uid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if ((new_uid == OPLUS_ANDROID_ROOT_UID) || (new_euid == OPLUS_ANDROID_ROOT_UID)) {
				pr_err(SG_HARDEN_HOOK "%s, incident, old uid %u, new uid %u, new euid %u.\n", __func__, cur_uid, new_uid, new_euid);
				oplus_secure_harden_kevent(new_uid, new_euid, new_uid, OPLUS_SETID_RE, OPLUS_HARDEN_EUID);
			}
		}
	}
	return ret;
}

/*RESUID*/
static int oplus_harden_setresuid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_uid = 0;
	unsigned int cur_euid = 0;
	unsigned int cur_suid = 0;
	unsigned int new_uid = 0;
	unsigned int new_euid = 0;
	unsigned int new_suid = 0;

	cur_uid = current_uid().val;
	cur_euid = current_euid().val;
	cur_suid = current_suid().val;

	new_uid = regs->regs[0];
	new_euid = regs->regs[1];
	new_suid = regs->regs[2];

	if (capable(CAP_SETUID) == true) {
		if (cur_uid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if ((new_uid == OPLUS_ANDROID_ROOT_UID) || (new_euid == OPLUS_ANDROID_ROOT_UID) || (new_suid == OPLUS_ANDROID_ROOT_UID)) {
				pr_err(SG_HARDEN_HOOK "%s, incident, old uid %u, new uid %u, new euid %u, new suid %u.\n", __func__,
					   cur_uid, new_uid, new_euid, new_suid);
				oplus_secure_harden_kevent(new_uid, new_euid, new_suid, OPLUS_SETID_RES, OPLUS_HARDEN_RESUID);
			}
		}
	}
	return ret;
}

/*GID*/
static int oplus_harden_setgid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_gid = 0;
	unsigned int new_gid = 0;

	cur_gid = current_gid().val;
	new_gid = regs->regs[0];

	if (capable(CAP_SETGID) == true) {
		if (cur_gid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if (new_gid == OPLUS_ANDROID_ROOT_UID) {
				pr_err(SG_HARDEN_HOOK "%s, incident, old gid %u, new gid %u.\n", __func__, cur_gid, new_gid);
				oplus_secure_harden_kevent(new_gid, new_gid, new_gid, OPLUS_SETID_ID, OPLUS_HARDEN_GID);
				/*restore the gid*/
				/*regs->regs[0] = cur_gid;*/
			}
		}
	}
	return ret;
}

/*EGID*/
static int oplus_harden_setregid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_gid = 0;
	unsigned int cur_egid = 0;
	unsigned int new_gid = 0;
	unsigned int new_egid = 0;

	cur_gid = current_uid().val;
	cur_egid = current_euid().val;
	new_gid = regs->regs[0];
	new_egid = regs->regs[1];

	if (capable(CAP_SETGID) == true) {
		if (cur_gid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if ((new_gid == OPLUS_ANDROID_ROOT_UID) || (new_egid == OPLUS_ANDROID_ROOT_UID)) {
				pr_err(SG_HARDEN_HOOK "%s, incident, old gid %u, new gid %u, new egid %u.\n", __func__, cur_gid, new_gid, new_egid);
				oplus_secure_harden_kevent(new_gid, new_egid, new_gid, OPLUS_SETID_RE, OPLUS_HARDEN_EGID);
			}
		}
	}
	return ret;
}

/*RESGID*/
static int oplus_harden_setresgid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_gid = 0;
	unsigned int cur_egid = 0;
	unsigned int cur_sgid = 0;
	unsigned int new_gid = 0;
	unsigned int new_egid = 0;
	unsigned int new_sgid = 0;

	cur_gid = current_gid().val;
	cur_egid = current_egid().val;
	cur_sgid = current_sgid().val;
	new_gid = regs->regs[0];
	new_egid = regs->regs[1];
	new_sgid = regs->regs[2];

	if (capable(CAP_SETGID) == true) {
		if (cur_gid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if ((new_gid == OPLUS_ANDROID_ROOT_UID) || (new_egid == OPLUS_ANDROID_ROOT_UID) || (new_sgid == OPLUS_ANDROID_ROOT_UID)) {
				pr_err(SG_HARDEN_HOOK "%s, incident, old gid %u, new gid %u, new egid %u, new sgid %u.\n", __func__,
					   cur_gid, new_gid, new_egid, new_sgid);
				oplus_secure_harden_kevent(new_gid, new_egid, new_sgid, OPLUS_SETID_RES, OPLUS_HARDEN_RESGID);
			}
		}
	}
	return ret;
}

/* include unistd.h
 *#define __NR_setregid   143
 *#define __NR_setgid     144
 *#define __NR_setreuid   145
 *#define __NR_setuid     146
 *#define __NR_setresuid  147
 *#define __NR_setresgid  149
 *#define __NR_socket     198
 */
void oplus_harden_pre_handler(void *data, struct pt_regs *regs, long id)
{
	int scno;
	int ret = 0;

	scno = regs->syscallno;
	if (is_unlocked())
		return;

	switch (scno) {
	case __NR_setregid:
		ret = oplus_harden_setregid(regs);
		break;
	case __NR_setgid:
		ret = oplus_harden_setgid(regs);
		break;
	case __NR_setreuid:
		ret = oplus_harden_setreuid(regs);
		break;
	case __NR_setuid:
		ret = oplus_harden_setuid(regs);
		break;
	case __NR_setresuid:
		ret = oplus_harden_setresuid(regs);
		break;
	case __NR_setresgid:
		ret = oplus_harden_setresgid(regs);
		break;
	default:
		break;
	}
}
