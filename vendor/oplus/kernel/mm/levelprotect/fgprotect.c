// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "fg_protect: " fmt

#include <linux/module.h>
#include <linux/memcontrol.h>
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


#define MAPCOUNT_PROTECT_THRESHOLD (20)
#define RETRY_GET_MAPCOUNT (3)
#define MAX_BUF_LEN (10)

/* Subset of vm_event_item to report for memcg event stats */
const unsigned int memcg_vm_event_stat[] = {
	PGPGIN,
	PGPGOUT,
	PGSCAN_KSWAPD,
	PGSCAN_DIRECT,
	PGSTEAL_KSWAPD,
	PGSTEAL_DIRECT,
	PGFAULT,
	PGMAJFAULT,
	PGREFILL,
	PGACTIVATE,
	PGDEACTIVATE,
	PGLAZYFREE,
	PGLAZYFREED,
#if defined(CONFIG_MEMCG_KMEM) && defined(CONFIG_ZSWAP)
	ZSWPIN,
	ZSWPOUT,
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	THP_FAULT_ALLOC,
	THP_COLLAPSE_ALLOC,
#endif
};

#define NR_MEMCG_EVENTS ARRAY_SIZE(memcg_vm_event_stat)
int mem_cgroup_events_index[NR_VM_EVENT_ITEMS] __read_mostly;

struct memcg_vmstats {
	/* Aggregated (CPU and subtree) page state & events */
	long			state[MEMCG_NR_STAT];
	unsigned long		events[NR_MEMCG_EVENTS];

	/* Non-hierarchical (CPU aggregated) page state & events */
	long			state_local[MEMCG_NR_STAT];
	unsigned long		events_local[NR_MEMCG_EVENTS];

	/* Pending child counts during tree propagation */
	long			state_pending[MEMCG_NR_STAT];
	unsigned long		events_pending[NR_MEMCG_EVENTS];

	/* Stats updates since the last flush */
	atomic64_t		stats_updates;
};

unsigned long fg_mapped_file = 0;

static atomic_long_t fg_mapcount_debug_1[21] = {
	ATOMIC_INIT(0)
};

static atomic_t fg_mapcount = ATOMIC_INIT(0);
static atomic_long_t fg_protect_count = ATOMIC_INIT(0);
static atomic_t fg_mapcount_enable = ATOMIC_INIT(0);


unsigned long  memavail_noprotected = 0;
static bool fg_protect_setup = false;
extern s64 get_mem_cgroup_app_uid(struct mem_cgroup *memcg);
extern bool is_fg(int uid);
extern struct mem_cgroup *get_next_memcg(struct mem_cgroup *prev);
extern void get_next_memcg_break(struct mem_cgroup *memcg);
enum folio_references {
	FOLIOREF_RECLAIM,
	FOLIOREF_RECLAIM_CLEAN,
	FOLIOREF_KEEP,
	FOLIOREF_ACTIVATE,
};

static inline bool folio_evictable(struct folio *folio) {
	bool ret;

	/* Prevent address_space of inode and swap cache from being freed */
	rcu_read_lock();
	ret = !mapping_unevictable(folio_mapping(folio)) && !folio_test_mlocked(folio);
	rcu_read_unlock();
	return ret;
}

static bool mem_available_is_low(void)
{
	long available = si_mem_available();

	if (available < memavail_noprotected)
		return true;

	return false;
}

static void update_fg_mapcount(long fg_mapped_file) {
	unsigned long fg_mapped_file_mb = fg_mapped_file >> 8;
	if (fg_mapped_file_mb  < 100)
		atomic_set(&fg_mapcount, 0);
	else if (fg_mapped_file_mb  < 150)
		atomic_set(&fg_mapcount, 1);
	else if (fg_mapped_file_mb  < 200)
		atomic_set(&fg_mapcount, 2);
	else if (fg_mapped_file_mb  < 250)
		atomic_set(&fg_mapcount, 3);
	else if (fg_mapped_file_mb  < 300)
		atomic_set(&fg_mapcount, 4);
	else if (fg_mapped_file_mb  < 350)
		atomic_set(&fg_mapcount, 5);
	else if (fg_mapped_file_mb  < 400)
		atomic_set(&fg_mapcount, 6);
	else if (fg_mapped_file_mb  < 450)
		atomic_set(&fg_mapcount, 7);
	else if (fg_mapped_file_mb  < 500)
		atomic_set(&fg_mapcount, 8);
	else if (fg_mapped_file_mb  < 550)
		atomic_set(&fg_mapcount, 9);
	else if (fg_mapped_file_mb  < 600)
		atomic_set(&fg_mapcount, 10);
	else
		atomic_set(&fg_mapcount, 20);
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

void page_should_be_fg_protect(struct folio *folio,
				int *should_protect)
{
	int file;
	struct mem_cgroup *memcg = NULL;
	int uid;
	long fg_mapped_file = 0;

	file = folio_is_file_lru(folio);

	if (unlikely(!fg_protect_setup)) {
		*should_protect = 0;
		return;
	}

	if (unlikely(!folio_evictable(folio))) {
		*should_protect = 0;
		return;
	}

	if (unlikely(mem_available_is_low())) {
		*should_protect = 0;
		return;
	}

	if (atomic_read(&fg_mapcount_enable) && file && folio_memcg(folio)) {
		memcg = folio_memcg(folio);
		uid = (int)get_mem_cgroup_app_uid(memcg);
		if (is_fg(uid) && memcg->vmstats) {
			fg_mapped_file = READ_ONCE(memcg->vmstats->state[NR_FILE_MAPPED]);
			update_fg_mapcount(fg_mapped_file);
			if (folio_mapcount(folio) > atomic_read(&fg_mapcount)) {
				*should_protect = FOLIOREF_ACTIVATE;
				atomic_long_add(1, &fg_protect_count);
				return;
			}
		}
	}

	*should_protect = 0;
}

EXPORT_SYMBOL(page_should_be_fg_protect);

static void page_should_be_level_protect(void *data, struct folio *folio, unsigned long nr_scanned, s8 priority,
				   u64 *data1, int *should_protect) {
	page_should_be_fg_protect(folio, should_protect);
	return;
}

static int get_nr_gens(struct lruvec *lruvec, int type)
{
	return lruvec->lrugen.max_seq - lruvec->lrugen.min_seq[type] + 1;
}

static void walk_lruvec_mglru(struct lruvec *lruvec)
{
    struct lru_gen_folio *lrugen = &lruvec->lrugen;
    int gen, type, zone;

    for (type = 0; type < ANON_AND_FILE; type++) {
        int max_gen = get_nr_gens(lruvec, type);
        for (gen = 0; gen < max_gen; gen++) {
            for (zone = 0; zone < MAX_NR_ZONES; zone++) {
                struct list_head *head = &lrugen->folios[gen][type][zone];
                struct folio *folio;

                if (list_empty(head))
                    continue;

                spin_lock_irq(&lruvec->lru_lock);
                list_for_each_entry(folio, head, lru) {
			if (folio_mapcount(folio) < 20)
				atomic_long_add(1, &fg_mapcount_debug_1[folio_mapcount(folio)]);
			else
				atomic_long_add(1, &fg_mapcount_debug_1[20]);
                }
                spin_unlock_irq(&lruvec->lru_lock);
            }
        }
    }
}

static void walk_lruvec(struct lruvec *lruvec) {
	struct page *page;
	int lru;
	spin_lock_irq(&lruvec->lru_lock);
	for_each_evictable_lru(lru) {
		int file = is_file_lru(lru);
		if (file) {
			list_for_each_entry(page, &lruvec->lists[lru], lru) {
				if (!page) {
					continue;
				}
				if (page_mapcount(page) < 20)
					atomic_long_add(1, &fg_mapcount_debug_1[page_mapcount(page)]);
				else
					atomic_long_add(1, &fg_mapcount_debug_1[20]);
			}
		}
	}
	spin_unlock_irq(&lruvec->lru_lock);
}

static void do_traversal_fg_memcg(void)
{
	struct lruvec* lruvec;
	struct mem_cgroup *memcg = NULL;
	pg_data_t *pgdat = NODE_DATA(0);
	struct lru_gen_folio *lrugen;


	while ((memcg = get_next_memcg(memcg))) {
		if (is_fg(get_mem_cgroup_app_uid(memcg))) {
			fg_mapped_file = READ_ONCE(memcg->vmstats->state[NR_FILE_MAPPED]);
			lruvec = mem_cgroup_lruvec(memcg, pgdat);
			lrugen = &lruvec->lrugen;

			if (lrugen->enabled) {
				walk_lruvec_mglru(lruvec);
			} else {
				walk_lruvec(lruvec);
			}

			get_next_memcg_break(memcg);
			break;
		}
	}
}

static int register_mapped_protect_vendor_hooks(void)
{
	int ret = 0;

	ret = register_trace_android_vh_page_should_be_protected(page_should_be_level_protect, NULL);
	if (ret != 0) {
		pr_err("register page_should_be_level_protect vendor_hooks failed! ret=%d\n", ret);
	}

	return ret;
}

static void unregister_mapped_protect_vendor_hooks(void)
{
	unregister_trace_android_vh_page_should_be_protected(page_should_be_level_protect, NULL);
	return;
}

static int fg_protect_show(struct seq_file *m, void *arg)
{
	int i = 0;
	for (i = 0; i < 21; i++) {
		atomic_long_set(&fg_mapcount_debug_1[i], 0);
	}
	do_traversal_fg_memcg();

	seq_printf(m,
		   "fg_mapcount:     %d\n",
		   atomic_read(&fg_mapcount));
	seq_printf(m,
		   "fg_mapped_file:     %lu\n", fg_mapped_file);

	for (i = 0; i < 21; i++) {
		seq_printf(m,
			   "fg_mapcount_debug_%d:     %lu\n",
			   i, atomic_long_read(&fg_mapcount_debug_1[i]));
	}

	seq_putc(m, '\n');

	return 0;
}

static ssize_t fg_mapcount_enable_ops_write(struct file *file,
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

	printk("fg_mapcount_ops_write is %d\n", val);
	atomic_set(&fg_mapcount_enable, val);

	return len;
}


static ssize_t fg_mapcount_enable_ops_read(struct file *file,
			char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[MAX_BUF_LEN] = {'\0'};
	int len;

	len = snprintf(kbuf, MAX_BUF_LEN, "%d\n", atomic_read(&fg_mapcount_enable));

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf + *off, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static ssize_t fg_protect_count_ops_write(struct file *file,
			const char __user *buff, size_t len, loff_t *ppos)
{
	int ret;
	char kbuf[MAX_BUF_LEN] = {'\0'};
	char *str;
	unsigned long val;

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

	ret = kstrtol(str, 10, &val);
	if (ret) {
		return -EINVAL;
	}

	if (val < 0 || val > INT_MAX) {
		return -EINVAL;
	}

	printk("fg_mapcount_ops_write is %lu\n", val);
	atomic_long_set(&fg_protect_count, val);

	return len;
}

static ssize_t fg_protect_count_ops_read(struct file *file,
			char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[MAX_BUF_LEN] = {'\0'};
	int len;

	len = snprintf(kbuf, MAX_BUF_LEN, "%lu\n", atomic_long_read(&fg_protect_count));

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf + *off, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static const struct proc_ops fg_mapcount_enable_ops = {
	.proc_write = fg_mapcount_enable_ops_write,
	.proc_read = fg_mapcount_enable_ops_read,
};

static const struct proc_ops fg_protect_count_ops = {
	.proc_write = fg_protect_count_ops_write,
	.proc_read = fg_protect_count_ops_read,
};

static int __init fg_protect_init(void)
{
	static struct proc_dir_entry *enable_entry;
	static struct proc_dir_entry *protect_count_entry;

#ifndef CONFIG_OPLUS_LEVEL_PROTECT
	int ret = 0;
	ret = register_mapped_protect_vendor_hooks();
	if (ret != 0)
		return ret;
#endif
	memavail_noprotected = totalram_pages() / 10;

	fg_protect_setup = true;
	proc_create_single("fg_protect_show", 0, NULL, fg_protect_show);
	enable_entry = proc_create("fg_protect_enable", 0666, NULL, &fg_mapcount_enable_ops);
	protect_count_entry = proc_create("fg_protect_count", 0666, NULL, &fg_protect_count_ops);

	pr_info("fg_protect_init succeed!\n");
	return 0;
}

static void __exit fg_protect_exit(void)
{
#ifndef CONFIG_OPLUS_LEVEL_PROTECT
	unregister_mapped_protect_vendor_hooks();
#endif
	pr_info("fg_protect_exit exit succeed!\n");

	return;
}

module_init(fg_protect_init);
module_exit(fg_protect_exit);

MODULE_LICENSE("GPL v2");
