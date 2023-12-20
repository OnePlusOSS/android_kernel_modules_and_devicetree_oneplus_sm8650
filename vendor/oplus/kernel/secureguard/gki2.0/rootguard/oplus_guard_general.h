/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 */
#ifndef _OPLUS_GUARD_GENERAL_H
#define _OPLUS_GUARD_GENERAL_H

#include <linux/version.h>

#define KERNEL_ADDR_LIMIT	0x0000008000000000
#define OPLUS_ARRAY_SIZE(arr)	(sizeof(arr) / sizeof((arr)[0]))

#define CHECK_ROOT_CREDS(x)	(uid_eq(x->cred->uid, GLOBAL_ROOT_UID) || gid_eq(x->cred->gid, GLOBAL_ROOT_GID) \
				|| uid_eq(x->cred->euid, GLOBAL_ROOT_UID) || gid_eq(x->cred->egid, GLOBAL_ROOT_GID))
#define CHECK_SHELL_CREDS(x)	(uid_eq(x->cred->uid, GLOBAL_SHELL_UID) || gid_eq(x->cred->gid, GLOBAL_SHELL_GID) \
				|| uid_eq(x->cred->euid, GLOBAL_SHELL_UID) || gid_eq(x->cred->egid, GLOBAL_SHELL_GID))

enum kernel_kevent_type {
	KEVENT_ROOT_EVENT = 0,
	KEVENT_STRING,
	KEVENT_REMOUNT_EVENT,
	KEVENT_EXEC_EVENT,
	SELINUX_DISABLE,	    /* = 4 */
	KEVENT_HARDEN_EVENT,	    /* = 5 */
	CPU_INFO,		    /* = 6 */
	SET_XATTR,		    /* = 7 */
	MCAST_MSFILTER_IP4,	    /* = 8 */
	MCAST_JOIN_GROUP_IP4,	    /* = 9 */
	IP_MSFILTER_IP4,	    /* = 10 */
	MCAST_JOIN_GROUP_IP6,	    /* = 11 */
	MCAST_MSFILTER_IP6,	    /* = 12 */
	SEPOLICY_RL,		    /* = 13 */
	EXEC2_EVENT,		    /* = 14 */
};

bool is_unlocked(void);
void oplus_boot_state_init(void);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
unsigned long get_fs(void);
#endif

#endif /* _OPLUS_GUARD_GENERAL_H */
