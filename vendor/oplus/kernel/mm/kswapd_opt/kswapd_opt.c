// SPDX-License-Identifier: GPL-2.0-only
/*
 * kswapd_opt, contain some optimisation to reduce kswapd running overhead
 * for some high-order allocation
 *
 * Copyright (C) 2023-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "kswapd_opt: " fmt

#ifdef CONFIG_OPLUS_FEATURE_KSWAPD_OPT
#define CONFIG_ALLOC_ADJUST_FLAGS 1
#define CONFIG_ALLOC_ORDER_STAT 1
#define CONFIG_KSWAPS_LOAD_STAT 1
#define CONFIG_KSWAPD_NICE 1
#endif

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/jump_label.h>
#include <linux/sched.h>
#include <linux/string.h>
#ifdef CONFIG_ALLOC_ADJUST_FLAGS
#include <trace/hooks/iommu.h>
#endif
#if defined(CONFIG_ALLOC_ORDER_STAT) || defined(CONFIG_ALLOC_ADJUST_FLAGS)
#include <trace/hooks/mm.h>
#endif
#ifdef CONFIG_KSWAPS_LOAD_STAT
#include <trace/hooks/vmscan.h>
#include <trace/events/vmscan.h>
#endif

#if defined(CONFIG_ALLOC_ADJUST_FLAGS) || defined(CONFIG_ALLOC_ORDER_STAT) || defined(CONFIG_KSWAPS_LOAD_STAT)
#define KBUF_LEN 10
static bool is_digit_str(const char *str)
{
	return strspn(str, "0123456789") == strlen(str);
}
#endif

#ifdef CONFIG_ALLOC_ADJUST_FLAGS
DEFINE_STATIC_KEY_TRUE(alloc_adjust_enable);
static bool g_alloc_adjust_enabled = true;
static struct proc_dir_entry *alloc_adjust_ctrl_entry;

static int alloc_adjust_ctrl_show(struct seq_file *m, void *v)
{
	if (g_alloc_adjust_enabled)
		seq_printf(m, "1\n");
	else
		seq_printf(m, "0\n");

	return 0;
}

static int alloc_adjust_ctrl_open(struct inode *inode, struct file *file)
{
	return single_open(file, alloc_adjust_ctrl_show, NULL);
}

static ssize_t alloc_adjust_ctrl_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char kbuf[KBUF_LEN] = {0};
	char *str;
	int val;

	if (count > KBUF_LEN - 1) {
		pr_warn("input too long\n");
		return -EINVAL;
	}

	if (copy_from_user(kbuf, buf, count))
		return -EINVAL;

	kbuf[count] = 0;
	str = strstrip(kbuf);
	if (!str) {
		pr_warn("input empty\n");
		return -EINVAL;
	}

	if (!is_digit_str(str)) {
		pr_warn("input invalid, not a digit string\n");
		return -EINVAL;
	}

	if (kstrtoint(str, 0, &val)) {
		pr_warn("not a valid number\n");
		return -EINVAL;
	}

	g_alloc_adjust_enabled = !!val;
	if (g_alloc_adjust_enabled)
		static_branch_enable(&alloc_adjust_enable);
	else
		static_branch_disable(&alloc_adjust_enable);

	return count;
}

static const struct proc_ops proc_alloc_adjust_ctrl_ops = {
	.proc_open = alloc_adjust_ctrl_open,
	.proc_read = seq_read,
	.proc_write = alloc_adjust_ctrl_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void create_alloc_adjust_ctrl_proc(void)
{
	struct proc_dir_entry *root_dir_entry = proc_mkdir("oplus_mem", NULL);

	alloc_adjust_ctrl_entry = proc_create(root_dir_entry ?
			"alloc_adjust_ctrl" : "oplus_mem/alloc_adjust_ctrl",
			0660, root_dir_entry, &proc_alloc_adjust_ctrl_ops);

	if (!alloc_adjust_ctrl_entry)
		pr_err("kswapd_debug_proc create failed, ENOMEM\n");
}

static void remove_alloc_adjust_ctrl_proc(void)
{
	if (alloc_adjust_ctrl_entry) {
		proc_remove(alloc_adjust_ctrl_entry);
		alloc_adjust_ctrl_entry = NULL;
	}
}

static void alloc_adjust_flags(void *data, unsigned int order, gfp_t *flags)
{
	if (!static_branch_likely(&alloc_adjust_enable))
		return;

	if (order > PAGE_ALLOC_COSTLY_ORDER)
		*flags &= ~__GFP_RECLAIM;
}

static void kvmalloc_adjust_flags(void *data, unsigned int order, gfp_t *flags)
{
	if (!static_branch_likely(&alloc_adjust_enable))
		return;

	if (order > PAGE_ALLOC_COSTLY_ORDER)
		*flags &= ~__GFP_RECLAIM;
}

static int register_alloc_adjust_flags(void)
{
	return register_trace_android_vh_adjust_alloc_flags(alloc_adjust_flags, NULL);
}

static void unregister_alloc_adjust_flags(void)
{
	unregister_trace_android_vh_adjust_alloc_flags(alloc_adjust_flags, NULL);
}

static int register_kvmalloc_adjust_flags(void)
{
	return register_trace_android_vh_adjust_kvmalloc_flags(kvmalloc_adjust_flags, NULL);
}

static void unregister_kvmalloc_adjust_flags(void)
{
	unregister_trace_android_vh_adjust_kvmalloc_flags(kvmalloc_adjust_flags, NULL);
}
#else
static int register_alloc_adjust_flags(void)
{
	return 0;
}

static void unregister_alloc_adjust_flags(void)
{
}

static int register_kvmalloc_adjust_flags(void)
{
	return 0;
}

static void unregister_kvmalloc_adjust_flags(void)
{
}

static void create_alloc_adjust_ctrl_proc(void)
{
}

static void remove_alloc_adjust_ctrl_proc(void)
{
}
#endif

#ifdef CONFIG_ALLOC_ORDER_STAT
DEFINE_STATIC_KEY_FALSE(kswapd_debug);
static bool g_kswapd_debug = false;
static atomic64_t alloc_stats[MAX_ORDER];

static struct proc_dir_entry *kswapd_debug_entry;

static int kswapd_debug_show(struct seq_file *m, void *v)
{
	int i;
	if (g_kswapd_debug) {
		seq_printf(m, "order\t count\n");
		for (i = 0; i < MAX_ORDER; i++)
			seq_printf(m, "%d\t %lld\n", i, atomic64_read(&alloc_stats[i]));
	} else {
		seq_printf(m, "0\n");
	}

	return 0;
}

static int kswapd_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, kswapd_debug_show, NULL);
}

static ssize_t kswapd_debug_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char kbuf[KBUF_LEN] = {0};
	char *str;
	int val;

	if (count > KBUF_LEN - 1) {
		pr_warn("input too long\n");
		return -EINVAL;
	}

	if (copy_from_user(kbuf, buf, count))
		return -EINVAL;

	kbuf[count] = 0;
	str = strstrip(kbuf);
	if (!str) {
		pr_warn("input empty\n");
		return -EINVAL;
	}

	if (!is_digit_str(str)) {
		pr_warn("input invalid, not a digit string\n");
		return -EINVAL;
	}

	if (kstrtoint(str, 0, &val)) {
		pr_warn("not a valid number\n");
		return -EINVAL;
	}

	g_kswapd_debug = !!val;
	if (g_kswapd_debug)
		static_branch_enable(&kswapd_debug);
	else
		static_branch_disable(&kswapd_debug);

	return count;
}

static const struct proc_ops proc_kswapd_debug_ops = {
	.proc_open = kswapd_debug_open,
	.proc_read = seq_read,
	.proc_write = kswapd_debug_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void alloc_order_stat(void *data, gfp_t gfp_mask, unsigned int order, unsigned long delta)
{
	if (!static_branch_unlikely(&kswapd_debug))
		return;

	if ((gfp_mask & __GFP_KSWAPD_RECLAIM) == 0)
		return;

	if (order >= MAX_ORDER)
		return;

	atomic64_inc(&alloc_stats[order]);
}

static int register_alloc_pages_slowpath(void)
{
	return register_trace_android_vh_alloc_pages_slowpath(alloc_order_stat, NULL);
}

static void unregister_alloc_pages_slowpath(void)
{
	unregister_trace_android_vh_alloc_pages_slowpath(alloc_order_stat, NULL);
}

static void create_kswapd_debug_proc(void)
{
	kswapd_debug_entry = proc_create("oplus_mem/kswapd_debug",
			0660, NULL, &proc_kswapd_debug_ops);

	if (!kswapd_debug_entry)
		pr_err("kswapd_debug_proc create failed, ENOMEM\n");
}

static void remove_kswapd_debug_proc(void)
{
	if (kswapd_debug_entry) {
		proc_remove(kswapd_debug_entry);
		kswapd_debug_entry = NULL;
	}
}
#else
static int register_alloc_pages_slowpath(void)
{
	return 0;
}

static void unregister_alloc_pages_slowpath(void)
{
}

static void create_kswapd_debug_proc(void)
{
}

static void remove_kswapd_debug_proc(void)
{
}
#endif

#ifdef CONFIG_KSWAPS_LOAD_STAT
#define NS_PER_MS 1000000
static u64 kswapd_start_time[MAX_ORDER];
static u64 kswapd_runtime_sum[MAX_ORDER];
static bool g_kswapd_load_stat_enabled;
DEFINE_STATIC_KEY_FALSE(kswapd_load_stat_enable);
static struct proc_dir_entry *kswapd_load_stat_entry;

static void kswapd_load_stat_start(void *data, int nid, int zid, int order)
{
	if (!static_branch_unlikely(&kswapd_load_stat_enable))
		return;

	if (order >= MAX_ORDER || order < 0)
		return;

	kswapd_start_time[order] = current->stime;
}

static void kswapd_load_stat_end(void *data, int nid,
		unsigned int highest_zoneidx, unsigned int alloc_order,
		unsigned int reclaim_order)
{
	if (!static_branch_unlikely(&kswapd_load_stat_enable))
		return;

	if (alloc_order >= MAX_ORDER)
		return;

	kswapd_runtime_sum[alloc_order] += current->stime -
		kswapd_start_time[alloc_order];
}

static int register_kswapd_load_stat(void)
{
	int ret;

	ret = register_trace_mm_vmscan_kswapd_wake(kswapd_load_stat_start, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_vmscan_kswapd_done(kswapd_load_stat_end, NULL);
	if (ret)
		unregister_trace_mm_vmscan_kswapd_wake(kswapd_load_stat_start, NULL);

	return ret;
}

static void unregister_kswapd_load_stat(void)
{
	unregister_trace_mm_vmscan_kswapd_wake(kswapd_load_stat_start, NULL);
	unregister_trace_android_vh_vmscan_kswapd_done(kswapd_load_stat_end, NULL);
}

static int kswapd_load_stat_show(struct seq_file *m, void *p)
{
	int i;
	if (g_kswapd_load_stat_enabled) {
		seq_printf(m, "order\t runtime(ms)\n");
		for (i = 0; i < MAX_ORDER; i++)
			seq_printf(m, "%d\t %llu\n", i, kswapd_runtime_sum[i] / NS_PER_MS);
	} else {
		seq_printf(m, "0\n");
	}

	return 0;
}

static int kswapd_load_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, kswapd_load_stat_show, NULL);
}

static ssize_t kswapd_load_stat_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char kbuf[KBUF_LEN] = {0};
	char *str;
	int val;

	if (count > KBUF_LEN - 1) {
		pr_warn("input too long\n");
		return -EINVAL;
	}

	if (copy_from_user(kbuf, buf, count))
		return -EINVAL;

	kbuf[count] = 0;
	str = strstrip(kbuf);
	if (!str) {
		pr_warn("input empty\n");
		return -EINVAL;
	}

	if (!is_digit_str(str)) {
		pr_warn("input invalid, not a digit string\n");
		return -EINVAL;
	}

	if (kstrtoint(str, 0, &val)) {
		pr_warn("not a valid number\n");
		return -EINVAL;
	}

	g_kswapd_load_stat_enabled = !!val;
	if (g_kswapd_load_stat_enabled)
		static_branch_enable(&kswapd_load_stat_enable);
	else
		static_branch_disable(&kswapd_load_stat_enable);

	return count;
}

static const struct proc_ops proc_kswapd_load_stat_ops = {
	.proc_open  = kswapd_load_stat_open,
	.proc_read  = seq_read,
	.proc_write = kswapd_load_stat_write,
	.proc_lseek = seq_lseek,
	.proc_release   = single_release,
};

static void create_kswapd_load_stat_proc(void)
{
	kswapd_load_stat_entry = proc_create("oplus_mem/kswapd_load_stat",
			0660, NULL, &proc_kswapd_load_stat_ops);

	if (!kswapd_load_stat_entry)
		pr_err("kswapd_load_stat_proc create failed, ENOMEM\n");
}

static void remove_kswapd_load_stat_proc(void)
{
	if (kswapd_load_stat_entry) {
		proc_remove(kswapd_load_stat_entry);
		kswapd_load_stat_entry = NULL;
	}
}
#else
static int register_kswapd_load_stat(void)
{
	return 0;
}

static void unregister_kswapd_load_stat(void)
{
}

static void create_kswapd_load_stat_proc(void)
{
}

static void remove_kswapd_load_stat_proc(void)
{
}
#endif

#if IS_ENABLED(CONFIG_KSWAPD_NICE)
#define KSWAPD_COMM "kswapd0"
static struct proc_dir_entry *kswapd_nice_entry;
static int global_kswapd_pid = -1;
static int global_kswapd_nice = 0;

static ssize_t kswapd_nice_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char kbuf[KBUF_LEN] = {0};
	char *str;
	int val = -1;
	struct task_struct *tsk = NULL;

	if (count > KBUF_LEN - 1) {
		pr_warn("input too long\n");
		return -EINVAL;
	}

	if (copy_from_user(kbuf, buf, count))
		return -EINVAL;

	kbuf[count] = 0;
	str = strstrip(kbuf);
	if (!str) {
		pr_warn("input empty\n");
		return -EINVAL;
	}

	if (kstrtoint(str, 0, &val)) {
		pr_warn("not a valid number\n");
		return -EINVAL;
	}
	if (val == global_kswapd_nice || val < MIN_NICE || val > MAX_NICE) {
		pr_warn("not a valid nice: %d \n", val);
		return -EINVAL;
	}
	/* start to set nice of kswapd0 */
	rcu_read_lock();
	for_each_process(tsk) {
		if (tsk->flags & PF_KTHREAD) {
			if (!strncmp(tsk->comm, KSWAPD_COMM,
				     sizeof(KSWAPD_COMM) - 1)) {
				global_kswapd_pid = tsk -> pid;
				global_kswapd_nice = val;
				set_user_nice(tsk, val);
				break;
			}
		}
	}
	rcu_read_unlock();
	return count;
}

static ssize_t kswapd_nice_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[20];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "pid=%d nice=%d \n", global_kswapd_pid, global_kswapd_nice);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_kswapd_nice_ops = {
	.proc_read  = kswapd_nice_read,
	.proc_write = kswapd_nice_write,
};

static void create_kswapd_nice_proc(void)
{
	kswapd_nice_entry = proc_create("oplus_mem/kswapd_nice",
		0660, NULL, &proc_kswapd_nice_ops);

	if (!kswapd_nice_entry)
		pr_err("kswapd_nice_entry create failed, ENOMEM\n");
}

static void remove_kswapd_nice_proc(void)
{
	if (kswapd_nice_entry) {
		proc_remove(kswapd_nice_entry);
		kswapd_nice_entry = NULL;
	}
}

#else
static void create_kswapd_nice_proc(void)
{
}

static void remove_kswapd_nice_proc(void)
{
}
#endif

static int __init kswapd_opt_init(void)
{
	int ret = 0;

	ret = register_alloc_adjust_flags();
	if (ret)
		pr_err("alloc_adjust_flags vendor_hook register failed: %d\n", ret);

	ret = register_kvmalloc_adjust_flags();
	if (ret)
		pr_err("kvmalloc_adjust_flags vendor_hook register failed: %d\n", ret);

	create_alloc_adjust_ctrl_proc();

	ret  = register_alloc_pages_slowpath();
	if (ret)
		pr_err("alloc_pages_slowpath vendor_hook register failed: %d\n", ret);
	else
		create_kswapd_debug_proc();

	ret = register_kswapd_load_stat();
	if (ret)
		pr_err("kswapd_load_stat vendor_hook regist failed: %d\n", ret);
	else
		create_kswapd_load_stat_proc();

	create_kswapd_nice_proc();
	pr_info("%s init done\n", __func__);
	return 0;
}

static void __exit kswapd_opt_exit(void)
{
	remove_alloc_adjust_ctrl_proc();
	unregister_alloc_adjust_flags();
	unregister_kvmalloc_adjust_flags();
	unregister_alloc_pages_slowpath();
	remove_kswapd_debug_proc();
	unregister_kswapd_load_stat();
	remove_kswapd_load_stat_proc();
	remove_kswapd_nice_proc();
	pr_info("%s exit\n", __func__);
}

module_init(kswapd_opt_init);
module_exit(kswapd_opt_exit);
MODULE_LICENSE("GPL v2");
