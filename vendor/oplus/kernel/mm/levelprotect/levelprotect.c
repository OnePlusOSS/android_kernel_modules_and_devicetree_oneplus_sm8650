// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "mapped_protect: " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <trace/hooks/vmscan.h>
#include <trace/hooks/mm.h>
#include <linux/swap.h>
#include <linux/proc_fs.h>
#include <linux/gfp.h>
#include <linux/printk.h>
#include <linux/mmzone.h>
#include <linux/mm.h>
#include <linux/mm_inline.h>
#include <linux/pagemap.h>
#include <linux/page-flags.h>
#include <linux/debugfs.h>
#include <linux/memcontrol.h>
#include <linux/file.h>

#define MAX_BUF_LEN (10)
#define MAX_INODE_LEN (1024)
#define MAX_INODE_NUM (20)

struct inode_protect_struct {
	unsigned long ino;
	int mapcount;
	unsigned long protect_count;
	struct list_head list;
};

enum folio_references {
	FOLIOREF_RECLAIM,
	FOLIOREF_RECLAIM_CLEAN,
	FOLIOREF_KEEP,
	FOLIOREF_ACTIVATE,
};
static atomic_t level_protect_enable = ATOMIC_INIT(0);

spinlock_t inode_protect_lock;
LIST_HEAD(inode_protect_array);
int inode_cur_num = 0;
unsigned long memavail_noprotected = 0;
extern void fput(struct file *file);

static inline bool folio_evictable(struct folio *folio) {
	bool ret;

	/* Prevent address_space of inode and swap cache from being freed */
	rcu_read_lock();
	ret = !mapping_unevictable(folio_mapping(folio)) && !folio_test_mlocked(folio);
	rcu_read_unlock();
	return ret;
}

int folio_mapcount(struct folio *folio)
{
	int i, compound, nr, ret;

	if (likely(!folio_test_large(folio)))
		return atomic_read(&folio->_mapcount) + 1;

	compound = folio_entire_mapcount(folio);
	if (folio_test_hugetlb(folio))
		return compound;
	ret = compound;
	nr = folio_nr_pages(folio);
	for (i = 0; i < nr; i++)
		ret += atomic_read(&folio_page(folio, i)->_mapcount) + 1;
	/* File pages has compound_mapcount included in _mapcount */
	if (!folio_test_anon(folio))
		return ret - compound * nr;
	if (folio_test_double_map(folio))
		ret -= nr;
	return ret;
}

static bool mem_available_is_low(void) {
	long available = si_mem_available();

	if (available < memavail_noprotected)
		return true;

	return false;
}

static int update_protect_mapcount(int mapcount) {
	int protect_mapcount = mapcount;
	long available = si_mem_available();
	if (available < 102400)
		protect_mapcount += 5;
	else if (available < 115200)
		protect_mapcount += 4;
	else if (available < 128000)
		protect_mapcount += 3;
	else if (available < 140800)
		protect_mapcount += 2;
	else if (available < 153600)
		protect_mapcount += 1;
	return protect_mapcount;
}

static struct inode_protect_struct *is_inode_in_list(unsigned long ino) {
	struct inode_protect_struct *inode_protect_entry = NULL;
	struct inode_protect_struct *next;
	if (list_empty(&inode_protect_array)) {
		return NULL;
	}

	list_for_each_entry_safe(inode_protect_entry, next, &inode_protect_array, list) {
		if (inode_protect_entry->ino == ino) {
			return inode_protect_entry;
		}
	}
	return NULL;
}

static void page_should_be_level_protect(void *data, struct folio *folio, unsigned long nr_scanned, s8 priority,
				   u64 *data1, int *should_protect)
{
	int file;

	file = folio_is_file_lru(folio);

	if (unlikely(!folio_evictable(folio))) {
		*should_protect = 0;
		return;
	}

	if (unlikely(mem_available_is_low())) {
		*should_protect = 0;
		return;
	}

	if (file && atomic_read(&level_protect_enable) && folio_mapping(folio)) {
		struct inode_protect_struct *inode_protect = NULL;
		int protect_mapcount = 0;
		unsigned long ino = -1;
		struct inode *f_inode;
		struct address_space *mapping;
		mapping = folio_mapping(folio);
		if (mapping != NULL) {
			f_inode = mapping->host;
			if (f_inode != NULL) {
				ino = f_inode->i_ino;
			}
		}
		spin_lock_irq(&inode_protect_lock);
		inode_protect = is_inode_in_list(ino);
		if (inode_protect != NULL) {
			protect_mapcount = update_protect_mapcount(inode_protect->mapcount);
			spin_unlock_irq(&inode_protect_lock);
			if (folio_mapcount(folio) > protect_mapcount) {
				inode_protect->protect_count = inode_protect->protect_count+1;
				*should_protect = FOLIOREF_ACTIVATE;
				return;
			}
		} else {
			spin_unlock_irq(&inode_protect_lock);
		}
	}

	*should_protect = 0;
}

static int register_mapped_protect_vendor_hooks(void)
{
	int ret = 0;

	ret = register_trace_android_vh_page_should_be_protected(page_should_be_level_protect, NULL);
	if (ret != 0) {
		pr_err("register page_should_be_level_protect vendor_hooks failed! ret=%d\n", ret);
		goto out;
	}
out:
	return ret;
}

static void unregister_mapped_protect_vendor_hooks(void)
{
	unregister_trace_android_vh_page_should_be_protected(page_should_be_level_protect, NULL);
	return;
}

static unsigned int level_protect_update_mapcount(unsigned long num_pages)
{
	unsigned int mapcount = 0;
	unsigned long num_pages_mb = num_pages >> 8;
	if (num_pages_mb < 15)
		mapcount = 0;
	else if (num_pages_mb < 30)
		mapcount = 1;
	else if (num_pages_mb < 40)
		mapcount = 2;
	else if (num_pages_mb < 50)
		mapcount = 3;
	else if (num_pages_mb < 60)
		mapcount = 4;
	else if (num_pages_mb < 70)
		mapcount = 5;
	else if (num_pages_mb < 80)
		mapcount = 6;
	else if (num_pages_mb < 90)
		mapcount = 7;
	else if (num_pages_mb < 100)
		mapcount = 8;
	else
		mapcount = 10;
	return mapcount;
}

static ssize_t level_protect_enable_ops_write(struct file *file,
					      const char __user *buff, size_t len, loff_t *ppos)
{
	int ret;
	char kbuf[MAX_BUF_LEN] = {'\0'};
	char *str;
	int val;

	if (len > MAX_BUF_LEN - 1) {
		return -EINVAL;
	}

	if (copy_from_user(&kbuf, buff, len))
		return -EFAULT;
	kbuf[len] = '\0';

	str = strstrip(kbuf);
	if (!str) {
		return -EINVAL;
	}

	ret = kstrtoint(str, 10, &val);
	if (ret) {
		return -EINVAL;
	}

	if (val < 0 || val > INT_MAX) {
		return -EINVAL;
	}

	printk("level_protect_enable is %d\n", val);
	atomic_set(&level_protect_enable, val);

	return len;
}

static ssize_t level_protect_enable_ops_read(struct file *file,
					     char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[MAX_BUF_LEN] = {'\0'};
	int len;

	len = snprintf(kbuf, MAX_BUF_LEN, "%d\n", atomic_read(&level_protect_enable));

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf + *off, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static ssize_t level_protect_inode_delete_ops_write(struct file *file,
						    const char __user *buff, size_t len, loff_t *ppos)
{
	char kbuf[MAX_INODE_LEN] = {'\0'};
	char *str;
	long fd = -1;
	unsigned long ino = 0;
	int fd_len;
	int str_len = len;
	struct file *protect_file;

	if (str_len > MAX_INODE_LEN - 1) {
		str_len = MAX_INODE_LEN - 1;
	}

	if (copy_from_user(&kbuf, buff, str_len))
		return -EFAULT;

	kbuf[str_len] = '\0';

	str = strstrip(kbuf);

	if (!str) {
		pr_err("buff %s is invalid\n", kbuf);
		return -EINVAL;
	}

	fd_len = sscanf(str, "%ld\n", &fd);
	if (fd_len > 0) {
		struct inode_protect_struct *inode_protect_entry = NULL;
		protect_file = fget(fd);
		if (!protect_file) {
			pr_err("protect_file is null\n");
			return len;
		}
		if (!file_inode(protect_file)) {
			fput(protect_file);
			return len;
		}
		ino = file_inode(protect_file)->i_ino;
		fput(protect_file);
		spin_lock_irq(&inode_protect_lock);
		inode_protect_entry = is_inode_in_list(ino);
		if (!inode_protect_entry) {
			spin_unlock_irq(&inode_protect_lock);
			return len;
		} else {
			pr_info("delete inode %lu\n", inode_protect_entry->ino);
			list_del(&inode_protect_entry->list);
			kfree(inode_protect_entry);
			spin_unlock_irq(&inode_protect_lock);
			inode_cur_num--;
		}
	}
	return len;
}

static ssize_t oplus_level_protect_inode_write(struct file *file,
					       const char __user *buff, size_t len, loff_t *ppos)
{
	char kbuf[MAX_INODE_LEN] = {'\0'};
	char *str;

	int str_len = len;
	struct file *protect_file;
	long fd = -1;
	unsigned long num_pages = 0;
	unsigned int inode_mapcount = -1;
	unsigned long ino = 0;
	struct inode_protect_struct *inode_protect_entry = NULL;
	int fd_len;

	if (str_len > MAX_INODE_LEN - 1) {
		str_len = MAX_INODE_LEN - 1;
	}

	if (inode_cur_num == MAX_INODE_NUM) {
		pr_info("inode over 20\n");
		return len;
	}

	if (copy_from_user(&kbuf, buff, str_len))
		return -EFAULT;

	kbuf[str_len] = '\0';

	str = strstrip(kbuf);

	if (!str) {
		pr_err("buff %s is invalid\n", kbuf);
		return -EINVAL;
	}

	fd_len = sscanf(str, "%ld,%d\n", &fd, &inode_mapcount);
	pr_info("fd %lu mapcount %d\n", fd, inode_mapcount);
	if (fd_len > 0) {
		protect_file = fget(fd);
		if (!protect_file) {
			pr_err("protect_file is null\n");
			return len;
		}
		if (!file_inode(protect_file)) {
			pr_err("fd error\n");
			fput(protect_file);
			return len;
		}
		ino = file_inode(protect_file)->i_ino;
		fput(protect_file);

		if (!is_inode_in_list(ino)) {
			inode_protect_entry = kmalloc(sizeof(struct inode_protect_struct), GFP_KERNEL);
		}

		if (inode_protect_entry == NULL) {
			pr_err("inode_protect_entry err\n");
			return len;
		}

		spin_lock_irq(&inode_protect_lock);
		memset(inode_protect_entry, 0, sizeof(struct inode_protect_struct));
		inode_protect_entry->ino = ino;
		if (inode_mapcount == -1) {
			num_pages = (i_size_read(file_inode(protect_file)) + PAGE_SIZE - 1) / PAGE_SIZE;
			inode_mapcount = level_protect_update_mapcount(num_pages);
		}
		inode_protect_entry->mapcount = inode_mapcount;
		inode_protect_entry->protect_count = 0;
		pr_info("add to inode list %lu, mapcount %d\n", inode_protect_entry->ino, inode_protect_entry->mapcount);
		list_add(&inode_protect_entry->list, &inode_protect_array);
		inode_cur_num++;
		spin_unlock_irq(&inode_protect_lock);
	}
	return len;
}

static int inode_protect_show(struct seq_file *m, void *arg)
{
	struct inode_protect_struct *inode_protect_entry = NULL;
	struct inode_protect_struct *next;
	spin_lock_irq(&inode_protect_lock);
	if (list_empty(&inode_protect_array)) {
		spin_unlock_irq(&inode_protect_lock);
		return 0;
	}

	list_for_each_entry_safe(inode_protect_entry, next, &inode_protect_array, list) {
		seq_printf(m, "%-20lu%20d%20lu\n", inode_protect_entry->ino, inode_protect_entry->mapcount, inode_protect_entry->protect_count);
	}
	spin_unlock_irq(&inode_protect_lock);
	return 0;
}

static const struct proc_ops level_protect_inode_delete_ops = {
	.proc_write = level_protect_inode_delete_ops_write,
};

static const struct proc_ops level_protect_enable_ops = {
	.proc_write = level_protect_enable_ops_write,
	.proc_read = level_protect_enable_ops_read,
};

static const struct proc_ops level_protect_inode_ops = {
	.proc_write = oplus_level_protect_inode_write,
};

static int __init level_protect_init(void)
{
	static struct proc_dir_entry *level_protect_mapcount_entry;
	static struct proc_dir_entry *level_protect_enable_entry;
	static struct proc_dir_entry *level_protect_inode_entry;
	int ret = 0;

	ret = register_mapped_protect_vendor_hooks();
	if (ret != 0) {
		return ret;
	}
	level_protect_enable_entry = proc_create("level_protect_enable", 0666, NULL, &level_protect_enable_ops);
	level_protect_mapcount_entry = proc_create("level_protect_inode_delete", 0666, NULL, &level_protect_inode_delete_ops);
	level_protect_inode_entry = proc_create("level_protect_inode", 0666, NULL, &level_protect_inode_ops);
	memavail_noprotected = totalram_pages() / 10;
	proc_create_single("level_protect_inode_show", 0, NULL, inode_protect_show);
	pr_info("mapped_protect_init succeed!\n");
	return 0;
}

static void __exit level_protect_exit(void)
{
	remove_proc_entry("level_protect_inode", NULL);
	remove_proc_entry("level_protect_enable", NULL);
	remove_proc_entry("level_protect_inode_show", NULL);
	pr_info("level_protect_exit exit succeed!\n");
	unregister_mapped_protect_vendor_hooks();

	return;
}

module_init(level_protect_init);
module_exit(level_protect_exit);

MODULE_LICENSE("GPL v2");
