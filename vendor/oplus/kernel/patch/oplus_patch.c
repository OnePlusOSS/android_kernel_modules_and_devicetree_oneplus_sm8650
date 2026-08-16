/* SPDX-License-Identifier: GPL-2.0-only */
/*  
* Copyright (C) 2018-2025 Oplus. All rights reserved. 
*/ 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/atomic.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/preempt.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include "oplus_patch.h"

#define LOG_BUF_SIZE 1024



/* global variable */
static struct proc_dir_entry *log_file, *ctl_file;
static struct log_entry *log_buffer;
static unsigned int log_index = 0;
static atomic_t log_enabled = ATOMIC_INIT(1);
static LIST_HEAD(hook_list);
static DEFINE_MUTEX(hook_list_lock);
static DEFINE_SPINLOCK(log_lock);

/* function declaration */
static int log_show(struct seq_file *m, void *v);
static int log_open(struct inode *inode, struct file *file);
static ssize_t ctl_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static ssize_t ctl_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);

void log_add(const char *func_name);
static int hook_register(const char *name, void *pre_handler, void *entry_handler, int type);
void hook_unregister(const char *name, int type);

/* log recording */
void log_add(const char *func_name)
{
        unsigned long flags;
        unsigned int idx;
        struct log_entry *entry;

        if (!atomic_read(&log_enabled) || !log_buffer)
                return;

        spin_lock_irqsave(&log_lock, flags);

        idx = log_index;
        entry = &log_buffer[idx];

        get_task_comm(entry->comm, current);
        entry->pid = task_pid_nr(current);
        strscpy(entry->func, func_name, sizeof(entry->func));

        /* publish entry fields before advancing head */
        smp_wmb();
        log_index = (idx + 1) % LOG_BUF_SIZE;

        spin_unlock_irqrestore(&log_lock, flags);
}

/* /proc/kprobe_log interface */
static int log_open(struct inode *inode, struct file *file)
{
        return single_open(file, log_show, NULL);
}

static int log_show(struct seq_file *m, void *v)
{
        unsigned long flags;
        unsigned int i, head, idx;
        struct log_entry *snapshot;

        snapshot = kmalloc_array(LOG_BUF_SIZE, sizeof(*snapshot), GFP_KERNEL);
        if (!snapshot)
                return -ENOMEM;

        spin_lock_irqsave(&log_lock, flags);
        head = log_index;
        memcpy(snapshot, log_buffer, LOG_BUF_SIZE * sizeof(*snapshot));
        spin_unlock_irqrestore(&log_lock, flags);

        for (i = 0; i < LOG_BUF_SIZE; i++) {
                idx = (head + i) % LOG_BUF_SIZE;
                if (!snapshot[idx].func[0])
                        continue;
                seq_printf(m, "[%4u] %16s (%5d): %s\n",
                           i, snapshot[idx].comm, snapshot[idx].pid, snapshot[idx].func);
        }
        kfree(snapshot);
        return 0;
}

static const struct proc_ops log_proc_ops = {
        .proc_open = log_open,
        .proc_read = seq_read,
        .proc_lseek = seq_lseek,
        .proc_release = single_release,
};

/* enable/disable hook */
int hook_enable(const char *name, int type)
{
        struct hook_entry *entry;
        int ret = 0;

        mutex_lock(&hook_list_lock);
        list_for_each_entry(entry, &hook_list, list) {
                if (strcmp(entry->name, name) == 0 && entry->type == type && !entry->enabled) {
                        if (type == KPROBE_TYPE_KPROBE) {
                                ret = register_kprobe(&entry->kp);
                                if (ret < 0) {
                                        pr_err("Failed to enable kprobe on %s: %d\n", name, ret);
                                        goto out;
                                }
                                entry->enabled = 1;
                                pr_debug("Enabled kprobe: %s\n", name);
                        }
                        else if (type == KPROBE_TYPE_KRETPROBE) {
                                ret = register_kretprobe(&entry->rp);
                                if (ret < 0) {
                                        pr_err("Failed to enable kretprobe on %s: %d\n", name, ret);
                                        goto out;
                                }
                                entry->enabled = 1;
                                pr_debug("Enabled kretprobe: %s\n", name);
                        }
                        else {
                                pr_err("Unknown probe type %d for %s\n", type, name);
                                ret = -EINVAL;
                                goto out;
                        }
                        goto out;
                }
        }
        pr_debug("No disabled probe found: %s [%s]\n", name,
                type == KPROBE_TYPE_KPROBE ? "kprobe" : "kretprobe");
        ret = -ENOENT;

out:
        mutex_unlock(&hook_list_lock);
        return ret;
}

void hook_disable(const char *name, int type)
{
        struct hook_entry *entry;

        mutex_lock(&hook_list_lock);
        list_for_each_entry(entry, &hook_list, list) {
                if (strcmp(entry->name, name) == 0 && entry->type == type && entry->enabled) {
                        if (type == KPROBE_TYPE_KPROBE) {
                                unregister_kprobe(&entry->kp);
                        } else if (type == KPROBE_TYPE_KRETPROBE) {
                                unregister_kretprobe(&entry->rp);
                        }
                        entry->enabled = 0;
                        break;
                }
        }
        mutex_unlock(&hook_list_lock);
}

void hook_enable_all(void)
{
        struct hook_entry *entry;
        int ret;

        mutex_lock(&hook_list_lock);
        list_for_each_entry(entry, &hook_list, list) {
                if (entry->enabled)
                        continue;

                if (entry->type == KPROBE_TYPE_KPROBE) {
                        ret = register_kprobe(&entry->kp);
                        if (ret < 0) {
                                pr_err("Failed to enable kprobe on %s: %d\n", entry->name, ret);
                                continue;
                        }
                        entry->enabled = 1;
                        pr_debug("Enabled kprobe: %s\n", entry->name);
                }
                else if (entry->type == KPROBE_TYPE_KRETPROBE) {
                        ret = register_kretprobe(&entry->rp);
                        if (ret < 0) {
                                pr_err("Failed to enable kretprobe on %s: %d\n", entry->name, ret);
                                continue;
                        }
                        entry->enabled = 1;
                        pr_debug("Enabled kretprobe: %s\n", entry->name);
                }
                else {
                        pr_err("Unknown probe type %d for %s\n", entry->type, entry->name);
                        continue;
                }
        }
        mutex_unlock(&hook_list_lock);
}

void hook_disable_all(void)
{
        struct hook_entry *entry;

        mutex_lock(&hook_list_lock);
        list_for_each_entry(entry, &hook_list, list) {
                if (entry->enabled) {
                        if (entry->type == KPROBE_TYPE_KPROBE) {
                                unregister_kprobe(&entry->kp);
                        } else if (entry->type == KPROBE_TYPE_KRETPROBE) {
                                unregister_kretprobe(&entry->rp);
                        }
                        entry->enabled = 0;
                }
        }
        mutex_unlock(&hook_list_lock);
}

/* /proc/kprobe_ctl write read */
static ssize_t ctl_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
        struct hook_entry *entry;
        char *tmp;
        size_t off = 0;
        ssize_t ret = 0;

        tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
        if (!tmp)
                return -ENOMEM;

        mutex_lock(&hook_list_lock);
        list_for_each_entry(entry, &hook_list, list) {
                int len;

                if (off >= PAGE_SIZE - 1)
                        break;

                len = scnprintf(tmp + off, PAGE_SIZE - off,
                        "%s [%s]: %s\n",
                        entry->name,
                        entry->type == KPROBE_TYPE_KPROBE ? "kprobe" : "kretprobe",
                        entry->enabled ? "enabled" : "disabled");

                if (len < 0) {
                        len = 0;
                }
                off += len;
        }
        tmp[off] = '\0';
        mutex_unlock(&hook_list_lock);

        ret = simple_read_from_buffer(buf, count, ppos, tmp, off);
        kfree(tmp);
        return ret;
}

static bool is_valid_symbol_n(const char *sym, size_t max_len)
{
        size_t i;
        unsigned char c0;
        unsigned char c;

        if (!sym || max_len == 0)
                return false;

        /*First char must be alpha or '_', which also rejects empty string ('\0')*/
        c0 = sym[0];
        if (!isalpha(c0) && c0 != '_')
                return false;

        for (i = 1; i < max_len; i++) {
                c = sym[i];
                if (c == '\0') {
                        return true;
                }
                if (!isalnum(c) && c != '_') {
                        return false;
                }
        }

        return false;
}

static ssize_t ctl_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
        char cmd[128];
        size_t len;
        int ret = 0;

        if (count >= sizeof(cmd) - 1)
                return -EINVAL;

        if (copy_from_user(cmd, buf, count))
                return -EFAULT;

        cmd[count] = '\0';
        cmd[strcspn(cmd, "\n")] = '\0';
        len = strlen(cmd);

        if (strncmp(cmd, "enable", 6) == 0 && len == 6) {
                atomic_set(&log_enabled, 1);
        } else if (strncmp(cmd, "disable", 7) == 0 && len == 7) {
                atomic_set(&log_enabled, 0);
        } else if (strncmp(cmd, "clear", 5) == 0 && len == 5) {
                unsigned long flags;
                spin_lock_irqsave(&log_lock, flags);
                if (log_buffer)
                        memset(log_buffer, 0, LOG_BUF_SIZE * sizeof(*log_buffer));
                log_index = 0;
                spin_unlock_irqrestore(&log_lock, flags);
        } else if (strncmp(cmd, "enable_all", 10) == 0 && len == 10) {
                hook_enable_all();
        } else if (strncmp(cmd, "disable_all", 11) == 0 && len == 11) {
                hook_disable_all();
        } else if (strncmp(cmd, "enable_kprobe:", 14) == 0) {
                char symbol_name[65];
                size_t sym_len;

                if (len <= 14)
                        return -EINVAL;
                sym_len = len - 14;
                if (sym_len > 64)
                        return -EINVAL;

                memcpy(symbol_name, cmd + 14, sym_len);
                symbol_name[sym_len] = '\0';

                if (!is_valid_symbol_n(symbol_name, sym_len + 1))
                        return -EINVAL;

                ret = hook_enable(symbol_name, KPROBE_TYPE_KPROBE);
                if (ret < 0)
                        return ret;
        } else if (strncmp(cmd, "disable_kprobe:", 15) == 0) {
                char symbol_name[65];
                size_t sym_len;

                if (len <= 15)
                        return -EINVAL;
                sym_len = len - 15;
                if (sym_len > 64)
                        return -EINVAL;

                memcpy(symbol_name, cmd + 15, sym_len);
                symbol_name[sym_len] = '\0';

                if (!is_valid_symbol_n(symbol_name, sym_len + 1))
                        return -EINVAL;

                hook_disable(symbol_name, KPROBE_TYPE_KPROBE);
        } else if (strncmp(cmd, "enable_kretprobe:", 17) == 0) {
                char symbol_name[65];
                size_t sym_len;
                if (len <= 17)
                        return -EINVAL;

                sym_len = len - 17;
                if (sym_len > 64)
                        return -EINVAL;

                memcpy(symbol_name, cmd + 17, sym_len);
                symbol_name[sym_len] = '\0';

                if (!is_valid_symbol_n(symbol_name, sym_len + 1))
                        return -EINVAL;

                ret = hook_enable(symbol_name, KPROBE_TYPE_KRETPROBE);
                if (ret < 0)
                        return ret;
        } else if (strncmp(cmd, "disable_kretprobe:", 18) == 0) {
                char symbol_name[65];
                size_t sym_len;

                if (len <= 18)
                        return -EINVAL;

                sym_len = len - 18;
                if (sym_len > 64)
                        return -EINVAL;

                memcpy(symbol_name, cmd + 18, sym_len);
                symbol_name[sym_len] = '\0';

                if (!is_valid_symbol_n(symbol_name, sym_len + 1))
                        return -EINVAL;

                hook_disable(symbol_name, KPROBE_TYPE_KRETPROBE);
        } else {
                pr_info("Unknown command: %s\n", cmd);
                return -EINVAL;
        }

        return count;
}

static const struct proc_ops ctl_proc_ops = {
        .proc_read = ctl_read,
        .proc_write = ctl_write,
        .proc_lseek = noop_llseek,
};

/* hook register */
static int hook_register(const char *name, void *pre_handler, void *entry_handler, int type)
{
        struct hook_entry *entry;
        int ret;

        entry = kzalloc(sizeof(*entry), GFP_KERNEL);
        if (!entry)
                return -ENOMEM;

        entry->name = name;
        entry->pre_handler = pre_handler;
        entry->entry_handler = entry_handler;
        entry->type = type;
        entry->enabled = 1;

        /* initialize kprobe */
        if (type == KPROBE_TYPE_KPROBE) {
                entry->kp.symbol_name = name;
                entry->kp.pre_handler = (kprobe_pre_handler_t)pre_handler;
                entry->kp.post_handler = NULL;

                ret = register_kprobe(&entry->kp);
                if (ret < 0) {
                        pr_err("Failed to register kprobe on %s: %d\n", name, ret);
                        kfree(entry);
                        return ret;
                }
        }
        /* initialize kretprobe */
        else if (type == KPROBE_TYPE_KRETPROBE) {
                entry->rp.kp.symbol_name = name;
                entry->rp.handler = (kretprobe_handler_t)entry_handler;
                entry->rp.entry_handler = NULL;  /* optional: a handler called immediately at the function entry */
                entry->rp.data_size = 0;
                entry->rp.maxactive = 1;

                ret = register_kretprobe(&entry->rp);
                if (ret < 0) {
                        pr_err("Failed to register kretprobe on %s: %d\n", name, ret);
                        kfree(entry);
                        return ret;
                }
        }
        else {
                pr_err("Unknown hook type: %d\n", type);
                kfree(entry);
                return -EINVAL;
        }

        mutex_lock(&hook_list_lock);
        list_add_tail(&entry->list, &hook_list);
        mutex_unlock(&hook_list_lock);

        return 0;
}

/* hook logout */
void hook_unregister(const char *name, int type)
{
        struct hook_entry *entry, *tmp;

        mutex_lock(&hook_list_lock);
        list_for_each_entry_safe(entry, tmp, &hook_list, list) {
                if (strcmp(entry->name, name) == 0 && (type == -1 || entry->type == type)) {
                        if (entry->enabled) {
                                if (entry->type == KPROBE_TYPE_KPROBE)
                                        unregister_kprobe(&entry->kp);
                                else if (entry->type == KPROBE_TYPE_KRETPROBE)
                                        unregister_kretprobe(&entry->rp);
                        }
                        list_del(&entry->list);
                        kfree(entry);
                }
        }
        mutex_unlock(&hook_list_lock);
}

/* example hook function */
static int handler_pre_first(struct kprobe *p, struct pt_regs *regs)
{
        unsigned long func_start = (unsigned long)p->addr;

        pr_info("kprobe: addr:%lx\n", func_start);
        log_add(p->symbol_name);

        return 0;
}

static int handler_pre_second(struct kprobe *p, struct pt_regs *regs)
{
        unsigned long func_start = (unsigned long)p->addr;

        pr_info("kprobe: addr:%lx\n", func_start);
        log_add(p->symbol_name);

        return 0;
}

static int handler_ret(struct kretprobe_instance *ri, struct pt_regs *regs)
{
        pr_err("kretprobe handler_ret.\n");
        return 0;
}

/* initialization/exit */
static int __init oplus_patch_init(void)
{
        int ret = 0;

        log_buffer = kmalloc_array(LOG_BUF_SIZE, sizeof(*log_buffer), GFP_KERNEL);
        if (!log_buffer) {
                pr_err("oplus_patch: Failed to allocate log buffer\n");
                return -ENOMEM;
        }

        memset(log_buffer, 0, LOG_BUF_SIZE * sizeof(*log_buffer));

        mutex_init(&hook_list_lock);

        log_file = proc_create("kprobe_log", 0444, NULL, &log_proc_ops);
        if (!log_file) {
                pr_err("oplus_patch: Failed to create /proc/kprobe_log\n");
                ret = -ENOMEM;
                goto free_buffer;
        }

        ctl_file = proc_create("kprobe_ctl", 0600, NULL, &ctl_proc_ops);
        if (!ctl_file) {
                pr_err("oplus_patch: Failed to create /proc/kprobe_ctl\n");
                ret = -ENOMEM;
                goto remove_log_file;
        }

        /* Register probes — failures are tolerated */
        if (hook_register("dummy_first_function", handler_pre_first, NULL, KPROBE_TYPE_KPROBE)) {
                pr_warn("oplus_patch: Failed to register kprobe on dummy_first_function\n");
        }

        if (hook_register("dummy_second_function", handler_pre_second, NULL, KPROBE_TYPE_KPROBE)) {
                pr_warn("oplus_patch: Failed to register kprobe on dummy_second_function\n");
        }

        if (hook_register("dummy_third_function", NULL, handler_ret, KPROBE_TYPE_KRETPROBE)) {
                pr_warn("oplus_patch: Failed to register kretprobe on dummy_third_function\n");
        }

        /*
         * Even if all hook_register() calls failed, the module loads successfully.
         * The exit function will safely clean up whatever was actually registered.
         */
        pr_info("oplus_patch module loaded.\n");
        return 0;

remove_log_file:
        remove_proc_entry("kprobe_log", NULL);
        log_file = NULL;

free_buffer:
        kfree(log_buffer);
        log_buffer = NULL;

        return ret;
}

static void __exit oplus_patch_exit(void)
{
        struct hook_entry *entry, *tmp;

        mutex_lock(&hook_list_lock);
        list_for_each_entry_safe(entry, tmp, &hook_list, list) {
                if (entry->enabled) {
                        if (entry->type == KPROBE_TYPE_KPROBE)
                                unregister_kprobe(&entry->kp);
                        else if (entry->type == KPROBE_TYPE_KRETPROBE)
                                unregister_kretprobe(&entry->rp);
                }
                list_del(&entry->list);
                kfree(entry);
        }
        mutex_unlock(&hook_list_lock);

        if (ctl_file) {
                remove_proc_entry("kprobe_ctl", NULL);
                ctl_file = NULL;
        }
        if (log_file) {
                remove_proc_entry("kprobe_log", NULL);
                log_file = NULL;
        }

        kfree(log_buffer);
        log_buffer = NULL;
        mutex_destroy(&hook_list_lock);

        pr_info("oplus_patch module unloaded.\n");
}

module_init(oplus_patch_init);
module_exit(oplus_patch_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jingchun Wang");
MODULE_DESCRIPTION("Oplus patch module support both kprobe and kretprobe ");
