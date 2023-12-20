// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 *
 * secure harden: heapspary check and selinux policy reload check.
 */
#include <linux/pgtable.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/bootconfig.h>
#include <linux/device.h>
#include <linux/version.h>
#include "oplus_secure_harden.h"
#include "oplus_guard_general.h"
#include "oplus_kevent.h"

/* secureguard sub-module string for print */
#define SG_MIX_HARDEN    "[secureguard][secure_harden]"

/* Heapspary layout[x]: PPID, COUNT, TIME */
/* pid type is int, count can use int, tv.tv_sec type is time64_t = long */
/* using int will be better, or using struct NOT array */
unsigned int heapspary_ip4[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned int heapspary_cpuinfo[3] = {0, 0, 0};
unsigned int heapspary_xttr[3] = {0, 0, 0};
unsigned int heapspary_ip6[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

/*Used for hook function name by Kprobe.*/
static char func_name_sepolicy_reload[NAME_MAX] = "sel_write_load";
static char func_name_socket[NAME_MAX]		= "ip_setsockopt";	/* ipv4 */
static char func_name_socket_ip6[NAME_MAX]	= "do_ipv6_setsockopt";	/* ipv6 */
static char func_name_cpu_info[NAME_MAX]	= "cpuinfo_open";
static char func_name_setxattr[NAME_MAX]	= "setxattr";

/*
 * oplus_heapspray_check:
 * By detecting how often the function is called by the same process, judge whether is a heapspary.
 * If an exception is detected, exception handling it.(e.g.. kill process)
 * @type: Type of entered
 * @Return: Void type, non-return.
 */
void oplus_heapspray_check(unsigned int type)
{
	struct timespec64 ts;
	const char *event_type = "heapspray";
	unsigned int new_ppid = current->real_parent->pid;

	/*bypass if root process && unlocked state */
	if ((!current_uid().val) || is_unlocked())
		return;

	ktime_get_real_ts64(&ts);
	/*
	 * task_ppid_nr() calls init_ppid_ns. But init_ppid_ns is not include into whilelist by Android R + K5.4.
	 * May support Android S + Kernel 5.10.
	 * unsigned int new_ppid = task_ppid_nr(current);
	 */
	switch (type) {
	case CPU_INFO:
		/* Only detect the same caller. */
		if (new_ppid == heapspary_cpuinfo[0]) {
			/* The time interval greater than 10s, then count again to avoid normal process being intercepted. */
			if ((ts.tv_sec - heapspary_cpuinfo[2]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_cpuinfo[1] = 0;
				heapspary_cpuinfo[2] = ts.tv_sec;
			/*For the first record, the initial value needs to be set.*/
			} else if (!heapspary_cpuinfo[2]) {
				heapspary_cpuinfo[2] = ts.tv_sec;
			/*Detect abnormal process: 1.Exceed the limit of 200 times within 10s of time intercal.*/
			} else if (heapspary_cpuinfo[1] > HEAPSPART_COUNT_LIMIT) {
				pr_err(SG_MIX_HARDEN "%s:CPU_INFO may be abnormal! (tiem diff: %lld)\n", __func__, ts.tv_sec - heapspary_cpuinfo[2]);
				heapspary_cpuinfo[1] = 0;
				heapspary_cpuinfo[2] = 0;
				/*force_sig is not include in whilelist*/
				report_security_event(event_type, type, "");
			}
			heapspary_cpuinfo[1]++;
		} else {
			/*Record the first call of different process.*/
			heapspary_cpuinfo[0] = new_ppid;
			heapspary_cpuinfo[1] = 0;
			heapspary_cpuinfo[2] = 0;
		}
		break;

	case SET_XATTR:
		if (new_ppid == heapspary_xttr[0]) {
			if ((ts.tv_sec - heapspary_xttr[2]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_xttr[1] = 0;
				heapspary_xttr[2] = ts.tv_sec;
			} else if (!heapspary_xttr[2]) {
				heapspary_xttr[2] = ts.tv_sec;
			} else if (heapspary_xttr[1] > HEAPSPART_COUNT_LIMIT) {
				pr_err(SG_MIX_HARDEN "%s:SET_XATTR may be abnormal! (tiem diff: %lld)\n", __func__, ts.tv_sec - heapspary_xttr[2]);
				heapspary_xttr[1] = 0;
				heapspary_xttr[2] = 0;
				report_security_event(event_type, type, "");
			}
			heapspary_xttr[1]++;
		} else {
			heapspary_xttr[0] = new_ppid;
			heapspary_xttr[1] = 0;
			heapspary_xttr[2] = 0;
		}
		break;

	case MCAST_MSFILTER_IP4:
		if (new_ppid == heapspary_ip4[0]) {
			if ((ts.tv_sec - heapspary_ip4[2]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_ip4[1] = 0;
				heapspary_ip4[2] = ts.tv_sec;
			} else if (!heapspary_ip4[2]) {
				heapspary_ip4[2] = ts.tv_sec;
			} else if (heapspary_ip4[1] > HEAPSPART_COUNT_LIMIT) {
				pr_err(SG_MIX_HARDEN "%s:MCAST_MSFILTER_IP4 may be abnormal! (tiem diff: %lld)\n", __func__, ts.tv_sec - heapspary_ip4[2]);
				heapspary_ip4[1] = 0;
				heapspary_ip4[2] = 0;
				/* do_exit(SIGKILL); */
				report_security_event(event_type, type, "");
			}
			heapspary_ip4[1]++;
		} else {
			heapspary_ip4[0] = new_ppid;
			heapspary_ip4[1] = 0;
			heapspary_ip4[2] = 0;
		}
		break;

	case MCAST_JOIN_GROUP_IP4:
		if (new_ppid == heapspary_ip4[3]) {
			if ((ts.tv_sec - heapspary_ip4[5]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_ip4[4] = 0;
				heapspary_ip4[5] = ts.tv_sec;
			} else if (!heapspary_ip4[5]) {
				heapspary_ip4[5] = ts.tv_sec;
			} else if (heapspary_ip4[4] > HEAPSPART_COUNT_LIMIT) {
				pr_err(SG_MIX_HARDEN "%s:MCAST_JOIN_GROUP_IP4 may be abnormal! (tiem diff: %lld)\n", __func__, ts.tv_sec - heapspary_ip4[5]);
				heapspary_ip4[4] = 0;
				heapspary_ip4[5] = 0;
				/* do_exit(SIGKILL); */
				report_security_event(event_type, type, "");
			}
			heapspary_ip4[4]++;
		} else {
			heapspary_ip4[3] = new_ppid;
			heapspary_ip4[4] = 0;
			heapspary_ip4[5] = 0;
		}
		break;

	case IP_MSFILTER_IP4:
		if (new_ppid == heapspary_ip4[6]) {
			if ((ts.tv_sec - heapspary_ip4[8]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_ip4[7] = 0;
				heapspary_ip4[8] = ts.tv_sec;
			} else if (!heapspary_ip4[8]) {
				heapspary_ip4[8] = ts.tv_sec;
			} else if (heapspary_ip4[7] > HEAPSPART_COUNT_LIMIT) {
				pr_err(SG_MIX_HARDEN "%s:IP_MSFILTER_IP4 may be abnormal! (tiem diff: %lld)\n", __func__, ts.tv_sec - heapspary_ip4[8]);
				heapspary_ip4[7] = 0;
				heapspary_ip4[8] = 0;
				/* do_exit(SIGKILL); */
				report_security_event(event_type, type, "");
			}
			heapspary_ip4[7]++;
		} else {
			heapspary_ip4[6] = new_ppid;
			heapspary_ip4[7] = 0;
			heapspary_ip4[8] = 0;
		}
		break;

	case MCAST_JOIN_GROUP_IP6:
		if (new_ppid == heapspary_ip6[0]) {
			if ((ts.tv_sec - heapspary_ip6[2]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_ip6[1] = 0;
				heapspary_ip6[2] = ts.tv_sec;
			} else if (!heapspary_ip6[2]) {
				heapspary_ip6[2] = ts.tv_sec;
			} else if (heapspary_ip6[1] > HEAPSPART_COUNT_LIMIT) {
				pr_err(SG_MIX_HARDEN "%s:MCAST_JOIN_GROUP_IP6 may be abnormal! (tiem diff: %lld)\n", __func__, ts.tv_sec - heapspary_ip6[2]);
				heapspary_ip6[1] = 0;
				heapspary_ip6[2] = 0;
				/* do_exit(SIGKILL); */
				report_security_event(event_type, type, "");
			}
			heapspary_ip6[1]++;
		} else {
			heapspary_ip6[0] = new_ppid;
			heapspary_ip6[1] = 0;
			heapspary_ip6[2] = 0;
		}
		break;

	case MCAST_MSFILTER_IP6:
		if (new_ppid == heapspary_ip6[3]) {
			if ((ts.tv_sec - heapspary_ip6[5]) >= HEAPSPARY_TIME_LIMIT) {
				heapspary_ip6[4] = 0;
				heapspary_ip6[5] = ts.tv_sec;
			} else if (!heapspary_ip6[5]) {
				heapspary_ip6[5] = ts.tv_sec;
			} else if (heapspary_ip6[4] > HEAPSPART_COUNT_LIMIT) {
				pr_err(SG_MIX_HARDEN "%s:MCAST_MSFILTER_IP6 may be abnormal! (tiem diff: %lld)\n", __func__, ts.tv_sec - heapspary_ip6[5]);
				heapspary_ip6[4] = 0;
				heapspary_ip6[5] = 0;
				report_security_event(event_type, type, "");
			}
			heapspary_ip6[4]++;
		} else {
			heapspary_ip6[3] = new_ppid;
			heapspary_ip6[4] = 0;
			heapspary_ip6[5] = 0;
		}
		break;
	} /*switch(type)*/
}

/*
 * oplus_sepolicy_reload:
 * Only the init process is allowed to load sepolicy, and the other process calls will be bolcked.
 * @type: Void
 * @Return: Void type, non-return.
 */
void oplus_sepolicy_reload(void)
{
	const char *event_type = "spolicy_reload";

	if (is_unlocked())
		return;

	if (!is_global_init(current)) {
		pr_err(SG_MIX_HARDEN "%s:Detected illegal porcess reload policy!!!\n", __func__);
		/* do_exit(SIGKILL); */
		report_security_event(event_type, SEPOLICY_RL, "");
	}
}

static int entry_handler_socket(struct kretprobe_instance *ri, struct pt_regs *regs)
{
	int socket_type = regs->regs[1];

	if (socket_type == MCAST_MSFILTER)
		oplus_heapspray_check(MCAST_MSFILTER_IP4);
	else if (socket_type == MCAST_JOIN_GROUP)
		oplus_heapspray_check(MCAST_JOIN_GROUP_IP4);
	else if (socket_type == IP_MSFILTER)
		oplus_heapspray_check(IP_MSFILTER_IP4);
	return 0;
}

static int entry_handler_socket_ip6(struct kretprobe_instance *ri, struct pt_regs *regs)
{
	int socket_type = regs->regs[1];

	if (socket_type == MCAST_JOIN_GROUP)
		oplus_heapspray_check(MCAST_JOIN_GROUP_IP4);
	else if (socket_type == IP_MSFILTER)
		oplus_heapspray_check(IP_MSFILTER_IP4);
	return 0;
}

static int entry_handler_cpuinfo(struct kretprobe_instance *ri, struct pt_regs *regs)
{
	oplus_heapspray_check(CPU_INFO);
	return 0;
}

static int entry_handler_setxattr(struct kretprobe_instance *ri, struct pt_regs *regs)
{
	oplus_heapspray_check(SET_XATTR);
	return 0;
}

static int entry_handler_sepolicy_reload(struct kretprobe_instance *ri, struct pt_regs *regs)
{
	oplus_sepolicy_reload();
	return 0;
}

static struct kretprobe sepolicy_reload_kretprobe = {
	.entry_handler  = entry_handler_sepolicy_reload,
	.maxactive	= 20,
};

static struct kretprobe socket_kretprobe = {
	.entry_handler  = entry_handler_socket,
	.data_size	= sizeof(struct pt_regs),
	.maxactive	= 300,
};
static struct kretprobe socket_ip6_kretprobe = {
	.entry_handler  = entry_handler_socket_ip6,
	.data_size	= sizeof(struct pt_regs),
	.maxactive	= 300,
};
static struct kretprobe cpuinfo_kretprobe = {
	.entry_handler  = entry_handler_cpuinfo,
	.data_size	= sizeof(struct pt_regs),
	.maxactive	= 300,
};
static struct kretprobe setxattr_kretprobe = {
	.entry_handler  = entry_handler_setxattr,
	.data_size	= sizeof(struct pt_regs),
	.maxactive	= 300,
};

static int oplus_harden_init_succeed = 0;
int oplus_harden_init(void)
{
	int ret = 0;

	sepolicy_reload_kretprobe.kp.symbol_name = func_name_sepolicy_reload;
	socket_kretprobe.kp.symbol_name = func_name_socket;
	socket_ip6_kretprobe.kp.symbol_name = func_name_socket_ip6;
	cpuinfo_kretprobe.kp.symbol_name = func_name_cpu_info;
	setxattr_kretprobe.kp.symbol_name = func_name_setxattr;

#ifdef CONFIG_OPLUS_FEATURE_SECURE_SRGUARD
	ret = register_kretprobe(&sepolicy_reload_kretprobe);
	if (ret < 0) {
		pr_err(SG_MIX_HARDEN "%s:register sepolicy_write_load FAILED! ret %d.\n", __func__, ret);
		goto sepolicy_reload_kretprobe_failed;
	}
#endif

#ifdef CONFIG_OPLUS_FEATURE_SECURE_SOCKETGUARD
	ret = register_kretprobe(&socket_kretprobe);
	if (ret < 0) {
		pr_err(SG_MIX_HARDEN "%s:register do_setsocket_opt FAILED! ret %d.\n", __func__, ret);
		goto socket_kretprobe_failed;
	}
	ret = register_kretprobe(&socket_ip6_kretprobe);
	if (ret < 0) {
		pr_err(SG_MIX_HARDEN "%s:register do_ip6_setsocket_opt FAILED! ret %d.\n", __func__, ret);
		goto socket_ip6_kretprobe_failed;
	}
	ret = register_kretprobe(&cpuinfo_kretprobe);
	if (ret < 0) {
		pr_err(SG_MIX_HARDEN "%s:register func_name_cpu_info FAILED! ret %d.\n", __func__, ret);
		goto cpuinfo_kretprobe_failed;
	}

	ret = register_kretprobe(&setxattr_kretprobe);
	if (ret < 0) {
		pr_err(SG_MIX_HARDEN "%s:register func_name_setxattr FAILED! ret %d.\n", __func__, ret);
		goto setxattr_kretprobe_failed;
	}
#endif

	pr_info(SG_MIX_HARDEN "%s:secure_harden has been register.\n", __func__);
	oplus_harden_init_succeed = 1;
	return 0;

#ifdef CONFIG_OPLUS_FEATURE_SECURE_SOCKETGUARD
setxattr_kretprobe_failed:
	unregister_kretprobe(&cpuinfo_kretprobe);
cpuinfo_kretprobe_failed:
	unregister_kretprobe(&socket_ip6_kretprobe);
socket_ip6_kretprobe_failed:
	unregister_kretprobe(&socket_kretprobe);
socket_kretprobe_failed:
#endif
#ifdef CONFIG_OPLUS_FEATURE_SECURE_SRGUARD
	unregister_kretprobe(&sepolicy_reload_kretprobe);
sepolicy_reload_kretprobe_failed:
#endif
	return ret;
}

void oplus_harden_exit(void)
{
	if (oplus_harden_init_succeed != 1)
		return;
#ifdef CONFIG_OPLUS_FEATURE_SECURE_SRGUARD
	unregister_kretprobe(&sepolicy_reload_kretprobe);
#endif

#ifdef CONFIG_OPLUS_FEATURE_SECURE_SOCKETGUARD
	unregister_kretprobe(&socket_kretprobe);
	unregister_kretprobe(&socket_ip6_kretprobe);
	unregister_kretprobe(&cpuinfo_kretprobe);
	unregister_kretprobe(&setxattr_kretprobe);
#endif
	pr_info(SG_MIX_HARDEN "%s:secure_harden has been unregister.\n", __func__);
}
