// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/bio.h>
#include <linux/bitops.h>
#include <linux/freezer.h>
#include <linux/kernel.h>
#include <linux/psi.h>
#include <linux/kfifo.h>
#include <linux/swap.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/vmstat.h>

#include "kcompressd.h"
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_OSVELTE)
#include "../mm_osvelte/mm-config.h"
#endif /* CONFIG_OPLUS_FEATURE_MM_OSVELTE */

#define INIT_QUEUE_SIZE		4096
#define DEFAULT_NR_KCOMPRESSD	1

static atomic_t enable_kcompressd;
static unsigned int nr_kcompressd;
static unsigned int queue_size_per_kcompressd;
static struct kcompress *kcompress;

enum run_state {
	KCOMPRESSD_NOT_STARTED = 0,
	KCOMPRESSD_RUNNING,
	KCOMPRESSD_SLEEPING,
};

struct kcompressd_para {
	wait_queue_head_t *kcompressd_wait;
	struct kfifo *write_fifo;
	atomic_t *running;
};

static struct kcompressd_para *kcompressd_para;

struct write_work {
	void *mem;
	struct bio *bio;
	compress_callback cb;
};

enum vm_kcomp_event_item {
	KCOMP_WRITE_SUCCESS_COUNT,
	KCOMP_WRITE_FAIL_COUNT,
	KCOMP_WRITE_SKIP_SHMEM_COUNT,
	NR_VM_KCOMP_EVENT_ITEMS
};

char *vm_kcomp_event_text[NR_VM_KCOMP_EVENT_ITEMS] = {
	"kcomp_write_success_count",
	"kcomp_write_fail_count",
	"kcomp_write_ship_shmem_count",
};

struct vm_kcomp_event_state {
	unsigned long event[NR_VM_KCOMP_EVENT_ITEMS];
};
DEFINE_PER_CPU(struct vm_kcomp_event_state, vm_kcomp_event_states) = {{0}};

inline void count_vm_kcomp_events(enum vm_kcomp_event_item item, long delta)
{
	this_cpu_add(vm_kcomp_event_states.event[item], delta);
}

inline void count_vm_kcomp_event(enum vm_kcomp_event_item item)
{
	count_vm_kcomp_events(item, 1);
}

void all_vm_kcomp_events(unsigned long *ret)
{
	int cpu;
	int i;

	memset(ret, 0, NR_VM_KCOMP_EVENT_ITEMS * sizeof(unsigned long));

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		struct vm_kcomp_event_state *this = &per_cpu(vm_kcomp_event_states,
							   cpu);

		for (i = 0; i < NR_VM_KCOMP_EVENT_ITEMS; i++)
			ret[i] += this->event[i];
	}
	cpus_read_unlock();
}

static int kcompressd_proc_stat_show(struct seq_file *s, void *v)
{
	int i;
	unsigned long events[NR_VM_KCOMP_EVENT_ITEMS];

	all_vm_kcomp_events(events);

	seq_printf(s, "enable_kcompressd %d\n", kcompressd_enabled());

	for (i = 0; i < NR_VM_KCOMP_EVENT_ITEMS; i++)
		seq_printf(s, "%s %lu\n", vm_kcomp_event_text[i], events[i]);

	return 0;
}

int kcompressd_enabled(void)
{
	return likely(atomic_read(&enable_kcompressd));
}
EXPORT_SYMBOL(kcompressd_enabled);

static void kcompressd_try_to_sleep(struct kcompressd_para *p)
{
	DEFINE_WAIT(wait);

	if (!kfifo_is_empty(p->write_fifo))
		return;

	if (freezing(current) || kthread_should_stop())
		return;

	atomic_set(p->running, KCOMPRESSD_SLEEPING);
	prepare_to_wait(p->kcompressd_wait, &wait, TASK_INTERRUPTIBLE);

	/*
	 * After a short sleep, check if it was a premature sleep. If not, then
	 * go fully to sleep until explicitly woken up.
	 */
	if (!kthread_should_stop() && kfifo_is_empty(p->write_fifo))
		schedule();

	finish_wait(p->kcompressd_wait, &wait);
	atomic_set(p->running, KCOMPRESSD_RUNNING);
}

static int kcompressd(void *para)
{
	struct task_struct *tsk = current;
	struct kcompressd_para *p = (struct kcompressd_para *)para;

	tsk->flags |= PF_MEMALLOC | PF_KSWAPD;
	set_freezable();

	while (!kthread_should_stop()) {
		bool ret;

		kcompressd_try_to_sleep(p);
		ret = try_to_freeze();
		if (kthread_should_stop())
			break;

		if (ret)
			continue;

		while (!kfifo_is_empty(p->write_fifo)) {
			struct write_work entry;

			if (sizeof(struct write_work) == kfifo_out(p->write_fifo,
						&entry, sizeof(struct write_work))) {
				entry.cb(entry.mem, entry.bio);
				bio_put(entry.bio);
				count_vm_kcomp_event(KCOMP_WRITE_SUCCESS_COUNT);
			}
		}

		usleep_range(1000, 2000);

	}

	tsk->flags &= ~(PF_MEMALLOC | PF_KSWAPD);
	atomic_set(p->running, KCOMPRESSD_NOT_STARTED);
	return 0;
}

static int init_write_queue(void)
{
	int i;
	unsigned int queue_len = queue_size_per_kcompressd * sizeof(struct write_work);

	for (i = 0; i < nr_kcompressd; i++) {
		if (kfifo_alloc(&kcompress[i].write_fifo,
					queue_len, GFP_KERNEL)) {
			pr_err("Failed to alloc kfifo %d\n", i);
			return -ENOMEM;
		}
	}
	return 0;
}

static void clean_bio_queue(int idx)
{
	struct write_work entry;

	while (sizeof(struct write_work) == kfifo_out(&kcompress[idx].write_fifo,
				&entry, sizeof(struct write_work))) {
		bio_put(entry.bio);
		entry.cb(entry.mem, entry.bio);
	}
	kfifo_free(&kcompress[idx].write_fifo);
}

static int kcompress_update(void)
{
	int i;
	int ret;

	kcompress = kvmalloc_array(nr_kcompressd, sizeof(struct kcompress), GFP_KERNEL);
	if (!kcompress)
		return -ENOMEM;

	kcompressd_para = kvmalloc_array(nr_kcompressd, sizeof(struct kcompressd_para), GFP_KERNEL);
	if (!kcompressd_para) {
		kfree(kcompress);
		return -ENOMEM;
	}

	ret = init_write_queue();
	if (ret) {
		kfree(kcompressd_para);
		kfree(kcompress);
		pr_err("Initialization of writing to FIFOs failed!!\n");
		return ret;
	}

	for (i = 0; i < nr_kcompressd; i++) {
		init_waitqueue_head(&kcompress[i].kcompressd_wait);
		spin_lock_init(&kcompress[i].write_fifo_lock);
		kcompressd_para[i].kcompressd_wait = &kcompress[i].kcompressd_wait;
		kcompressd_para[i].write_fifo = &kcompress[i].write_fifo;
		kcompressd_para[i].running = &kcompress[i].running;
	}

	return 0;
}

static void stop_all_kcompressd_thread(void)
{
	int i;

	for (i = 0; i < nr_kcompressd; i++) {
		kthread_stop(kcompress[i].kcompressd);
		kcompress[i].kcompressd = NULL;
		clean_bio_queue(i);
	}
}

static const struct kernel_param_ops param_ops_change_nr_kcompressd = {
	.set = NULL,
	.get = &param_get_uint,
	.free = NULL,
};

module_param_cb(nr_kcompressd, &param_ops_change_nr_kcompressd,
		&nr_kcompressd, 0444);
MODULE_PARM_DESC(nr_kcompressd, "Number of pre-created daemon for page compression");

static const struct kernel_param_ops param_ops_change_queue_size_per_kcompressd = {
	.set = NULL,
	.get = &param_get_uint,
	.free = NULL,
};

module_param_cb(queue_size_per_kcompressd, &param_ops_change_queue_size_per_kcompressd,
		&queue_size_per_kcompressd, 0444);
MODULE_PARM_DESC(queue_size_per_kcompressd,
		"Size of queue for kcompressd");

int schedule_bio_write(void *mem, struct bio *bio, compress_callback cb)
{
	int i;
	long count;
	bool submit_success = false;
	size_t sz_work = sizeof(struct write_work);
	struct page *page;

	struct write_work entry = {
		.mem = mem,
		.bio = bio,
		.cb = cb
	};

	if (unlikely(!atomic_read(&enable_kcompressd)))
		return -EBUSY;

	if (!nr_kcompressd || !current_is_kswapd())
		return -EBUSY;

	bio_get(bio);
	count = bio_sectors(bio);
	count_vm_events(PGPGOUT, -count);


	/*
	 * We exclude shmem pages by the existence of page->mapping,
	 * because when a shmem page is added to the swap cache, its page->mapping
	 * is set to NULL in the shmem_delete_from_page_cache function.
	*/
	page = bio->bi_io_vec->bv_page;
	if (!page->mapping) {
		bio_put(bio);
		count_vm_kcomp_event(KCOMP_WRITE_SKIP_SHMEM_COUNT);
		return -EBUSY;
	}

	if(op_is_sync(bio->bi_opf)){
		bio_put(bio);
		pr_warn("kcompressd : op should not be sync, bio->bi_opf:%d\n", bio->bi_opf);
		WARN_ON(1);
		return -EBUSY;
	}

	for (i = 0; i < nr_kcompressd; i++) {
		submit_success =
			(kfifo_avail(&kcompress[i].write_fifo) >= sz_work) &&
			(sz_work == kfifo_in_spinlocked_noirqsave(&kcompress[i].write_fifo, &entry, sz_work, &kcompress[i].write_fifo_lock));

		if (submit_success) {
			switch (atomic_read(&kcompress[i].running)) {
			case KCOMPRESSD_NOT_STARTED:
				atomic_set(&kcompress[i].running, KCOMPRESSD_RUNNING);
				kcompress[i].kcompressd = kthread_run(kcompressd,
						&kcompressd_para[i], "kcompressd:%d", i);
				if (IS_ERR(kcompress[i].kcompressd)) {
					atomic_set(&kcompress[i].running, KCOMPRESSD_NOT_STARTED);
					pr_warn("Failed to start kcompressd:%d\n", i);
					clean_bio_queue(i);
				}
				break;
			case KCOMPRESSD_RUNNING:
				break;
			case KCOMPRESSD_SLEEPING:
				wake_up_interruptible(&kcompress[i].kcompressd_wait);
				break;
			default:
				break;
			}
			return 0;
		}
	}

	bio_put(bio);
	count_vm_kcomp_event(KCOMP_WRITE_FAIL_COUNT);
	return -EBUSY;
}
EXPORT_SYMBOL(schedule_bio_write);

bool should_fallback_to_submit_bio(struct page *page, unsigned int op)
{
	if (op_is_write(op)) {
		/*
		 * We exclude shmem pages by the existence of page->mapping,
		 * because when a shmem page is added to the swap cache, its page->mapping
		 * is set to NULL in the shmem_delete_from_page_cache function.
		*/
		if (!page->mapping)
			return false;

		if (kcompressd_enabled() && current_is_kswapd())
			return true;
	}

	return false;
}

int kcompressd_init(void)
{
	int ret = 0;
	struct proc_dir_entry *root_dir_entry;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_OSVELTE)
	struct config_kcompressed *config;

	/* check cmdline if rus disble */
	if (oplus_test_mm_feature_disable(COMFD1_KCOMPRESSED)) {
		pr_info("kcompressed diabled by cmdline\n");
		return 0;
	}

	config = oplus_read_mm_config(module_name_kcompressed);
	if (!config || !config->enable) {
		pr_info("%s is disabled in config\n", module_name_kcompressed);
		return 0;
	}
#endif /* CONFIG_OPLUS_FEATURE_MM_OSVELTE */


	root_dir_entry = proc_mkdir("oplus_mem", NULL);
	proc_create_single((root_dir_entry ?
				"kcompressd_debug" : "oplus_mem/kcompressd_debug"),
				0444, root_dir_entry, kcompressd_proc_stat_show);

	nr_kcompressd = DEFAULT_NR_KCOMPRESSD;
	queue_size_per_kcompressd = INIT_QUEUE_SIZE;

	ret = kcompress_update();
	if (ret) {
		pr_err("Init kcompressd failed!\n");
		return ret;
	}

	pr_err("kcompressd_init succeed!\n");
	atomic_set(&enable_kcompressd, true);

	return ret;
}

void kcompressd_exit(void)
{
	if(!kcompressd_enabled())
		return;

	atomic_set(&enable_kcompressd, false);
	stop_all_kcompressd_thread();

	kvfree(kcompress);
	kvfree(kcompressd_para);
}
