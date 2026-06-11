// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/types.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/limits.h>
#include <linux/printk.h>      /* for pr_err, pr_info etc */
#include <linux/mutex.h>
#include <linux/fcntl.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/atomic.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>

#define CREATE_TRACE_POINTS
#include "fdleak_trace.h"
#define FDLEAK_APPID 20120
#define FDLEAK_DCS_TAG "CriticalLog"
#define FDLEAK_DCS_EVENTID "oplus_fdleak"

#define FDLEAK_CHECK_LOG_TAG "[fdleak_check]"
#define FD_MAX 32768
#define DEFAULT_THRESHOLD (FD_MAX/2)
#define DEFAULT_DUMP_THRESHOLD (DEFAULT_THRESHOLD + 500)
#define TASK_COMM_LEN			24
#define THRESHOLD_LEN                   10
#define MAX_SYMBOL_LEN 64
#define TASK_WHITE_LIST_MAX  128
#define MAX_REPORT_ENTRIES   128

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
static char symbol[MAX_SYMBOL_LEN] = "__alloc_fd";
#else
static char symbol[MAX_SYMBOL_LEN] = "get_unused_fd_flags";
#endif

module_param_string(symbol, symbol, sizeof(symbol), 0644);
int load_threshold = DEFAULT_THRESHOLD;
int dump_threshold = DEFAULT_DUMP_THRESHOLD;
static int fdleak_enable = 0;

struct fdleak_white_list_struct {
	char comm[TASK_COMM_LEN];
	int load_threshold;
	int dump_threshold;
};

struct fdleak_report_entry {
        pid_t leader_pid;
        char leader_comm[TASK_COMM_LEN];
        int reported;
};
static struct fdleak_report_entry report_entries[MAX_REPORT_ENTRIES];
static DEFINE_SPINLOCK(report_lock);

static struct fdleak_white_list_struct white_list[TASK_WHITE_LIST_MAX] = {
	{"fdleak_example", 2048, 2560},
	{"composer", 19500, 20000},
	{"surfaceflinger", 19500, 20000},
};

static long get_timestamp_ms(void)
{
	struct timespec64 now;
	ktime_get_real_ts64(&now);
	return timespec64_to_ns(&now) / NSEC_PER_MSEC;
}

static ssize_t fdleak_proc_read(struct file *file, char __user *buf,
		size_t count, loff_t *off)
{
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 6, 0))
	char page[2048] = {0};
#else
	char *page = kzalloc(2048, GFP_KERNEL);
        if (!page)
                return -ENOMEM;
#endif
	int len = 0;
	int i;

        len += snprintf(page + len, 2048 - len, "fdleak_enable = %d\n\n", fdleak_enable);
	for(i = 0; i < ARRAY_SIZE(white_list); i++) {
                if (!strlen(white_list[i].comm))
                        break;
		len += snprintf(&page[len], 2048 - len, "fdleak_detect_task = %s load_threshold = %d dump_threshold = %d\n",
					white_list[i].comm, white_list[i].load_threshold, white_list[i].dump_threshold);
	}

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf, page, (len < count ? len : count))) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	   kfree(page);
#endif
	   return -EFAULT;
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	kfree(page);
#endif
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static ssize_t fdleak_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *off)
{
	int tmp_load_threshold = 0;
	int tmp_dump_threshold = 0;
	char tmp_task[TASK_COMM_LEN] = {0};
	char tmp_cmd[TASK_COMM_LEN] = {0};
	int tmp_enable = 0;
	int ret = 0;
	char buffer[64] = {0};
	int max_len[] = {TASK_COMM_LEN, THRESHOLD_LEN, THRESHOLD_LEN};
	int part;
	char delim[] = {' ', ' ', '\n'};
	char *start, *end;
	int i;

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count)) {
		pr_err(FDLEAK_CHECK_LOG_TAG "%s: read proc input error.\n", __func__);
		return count;
	}

	buffer[count] = '\0';
	if (sscanf(buffer, "%15s %d", tmp_cmd, &tmp_enable) == 2) {
		if (strncmp(tmp_cmd, "enable", sizeof("enable")) == 0) {
			fdleak_enable = (tmp_enable != 0) ? 1 : 0;
			return count;
		}
	}
	/* validate the length of each of the 3 parts */
	start = buffer;
	for (part = 0; part < 3; part++) {
		end = strchr(start, delim[part]);
		if (end == NULL || (end - start) > max_len[part]) {
			return count;
		}
		start = end + 1;
	}

	ret = sscanf(buffer, "%15s %d %d", tmp_task, &tmp_load_threshold, &tmp_dump_threshold);
	if(ret <= 0) {
		pr_err(FDLEAK_CHECK_LOG_TAG "%s: input error\n", __func__);
		return count;
	}
	for (i = 0; i < ARRAY_SIZE(white_list); i++) {
		if (strlen(white_list[i].comm) && !strcmp(white_list[i].comm, tmp_task)) {
			white_list[i].load_threshold = tmp_load_threshold;
			white_list[i].dump_threshold = tmp_dump_threshold;
			break;
		} else if (strlen(white_list[i].comm)) {
			continue;
		} else {
			strncpy(white_list[i].comm, tmp_task, strlen(tmp_task));
			white_list[i].load_threshold = tmp_load_threshold;
			white_list[i].dump_threshold = tmp_dump_threshold;
			break;
		}
	}
	return count;
}

static struct proc_ops fdleak_proc_pops = {
	.proc_read = fdleak_proc_read,
	.proc_write = fdleak_proc_write,
	.proc_lseek = default_llseek,
};

static bool update_report_state(pid_t pid, const char *comm)
{
	bool should_report = false;
	int i, empty_slot = -1;

	spin_lock(&report_lock);
	for (i = 0; i < MAX_REPORT_ENTRIES; i++) {
		if (report_entries[i].leader_pid == pid &&
			!strncmp(report_entries[i].leader_comm, comm, TASK_COMM_LEN)) {
			if (!report_entries[i].reported) {
				report_entries[i].reported = true;
				should_report = true;
			}
			goto unlock;
		}

		if (empty_slot == -1 && !report_entries[i].leader_pid)
			empty_slot = i;
	}

	if (empty_slot != -1) {
		report_entries[empty_slot].leader_pid = pid;
		strscpy(report_entries[empty_slot].leader_comm, comm,
				sizeof(report_entries[empty_slot].leader_comm));
		report_entries[empty_slot].reported = true;
		should_report = true;
	}

unlock:
	spin_unlock(&report_lock);
	return should_report;
}

static int ret_handler(struct kretprobe_instance *kri, struct pt_regs *regs)
{
	pid_t leader_pid = 0;
	char leader_comm[TASK_COMM_LEN] = {0};
	struct task_struct *leader = NULL;
	const int fd = regs_return_value(regs);

	if (!fdleak_enable)
		return 0;
	if (fd != -EMFILE)
		return 0;

	rcu_read_lock();
	leader = rcu_dereference(current->group_leader);
	if (likely(leader && pid_alive(leader))) {
		leader_pid = leader->pid;
		snprintf(leader_comm, TASK_COMM_LEN, "%llu", (unsigned long long)leader->start_time);
	}
	rcu_read_unlock();

	if (unlikely(!leader_pid))
		return 0;

	if (update_report_state(leader_pid, leader_comm)) {
		trace_oplus_fdleak(get_timestamp_ms(), FDLEAK_APPID, FDLEAK_DCS_TAG, FDLEAK_DCS_EVENTID, leader_pid, leader_comm);
		pr_info(FDLEAK_CHECK_LOG_TAG "FDLEAK[%s:%d] via [%s:%d]\n",
			leader_comm, leader_pid, current->comm, current->pid);
		}

	return 0;
}

/* For each probe you need to allocate a kprobe structure */
static struct kretprobe g_krp = {
	.handler = ret_handler,
	.maxactive = 10,
};

static int __init fdleak_check_init(void)
{
	int ret;

	g_krp.kp.symbol_name = symbol;
	if(!proc_create("fdleak_detect", 0666, NULL, &fdleak_proc_pops)) {
		pr_err(FDLEAK_CHECK_LOG_TAG "proc node fdleak_detect create failed\n");
		return -ENOENT;
	}
	ret = register_kretprobe(&g_krp);
	if (ret < 0) {
		pr_err(FDLEAK_CHECK_LOG_TAG "oplus_fdleak_check, register_kretprobe failed, return %d\n", ret);
		remove_proc_entry("fdleak_detect", NULL);
		return ret;
	}
	pr_info(FDLEAK_CHECK_LOG_TAG "oplus_fdleak_check, planted kretprobe at %p\n", g_krp.kp.addr);

	return 0;
}

static void __exit fdleak_check_exit(void)
{
	unregister_kretprobe(&g_krp);
	remove_proc_entry("fdleak_detect", NULL);
	pr_info("oplus_fdleak_check, kretprobe at %p unregistered\n", g_krp.kp.addr);
}

MODULE_DESCRIPTION("oplus fdleak check");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wei.Li");

module_init(fdleak_check_init);
module_exit(fdleak_check_exit);

