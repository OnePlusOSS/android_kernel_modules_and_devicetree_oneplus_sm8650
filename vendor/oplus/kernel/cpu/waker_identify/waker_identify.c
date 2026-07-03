// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Oplus. All rights reserved.
 */
#include <linux/sched.h>
#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/list.h>
#include <trace/events/sched.h>
#include <trace/hooks/sched.h>
#include <linux/seq_file.h>
#include <linux/sort.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include "waker_identify.h"


#define MAX_TID_COUNT 32
#define RESULT_PAGE_SIZE 3072
#define MAX_WAKER_COUNT 32
#define DEAFAULT_HOLD_KTIME_MS 5000
#define MAX_HANDLE_NUM 10000
#define MAX_BREADTH 2
#define INVALID_THREAD_IDX -1
#define RESULT_NUM_MAX 7

struct proc_dir_entry *waker_identify_dir;

int handle_current = 1;
int handle_show = 1;
int result_search_num = 3;
int global_enable = 1;
int global_doing_free = 0;
atomic_t global_debug = ATOMIC_INIT(0);

struct wake_message {
	int wake_source_index;
	int wake_count;
};

struct related_thread {
	pid_t tgid;
	pid_t pid;
	struct task_struct *task;
	int total_count;
	int waker_num;
	bool visted;
	struct wake_message waker_array[MAX_WAKER_COUNT];
	DECLARE_BITMAP(waker_bitmap, MAX_TID_COUNT);
};

struct rt_info_list_node {
	int handle;
	int start_ktime;
	int seed_index;
	int seed_pid;
	int stopped;
	struct task_struct *seed_task;
	struct list_head list_node;
	int app_uid;
	int in_uid;
	int for_wakee;
	int total_count;
	struct related_thread rt_thread[MAX_TID_COUNT];
};
static struct delayed_work wq_for_kfree;
struct list_head rt_info_list = LIST_HEAD_INIT(rt_info_list);
static DEFINE_RWLOCK(rt_info_rwlock);

static inline u32 task_util(struct task_struct *p)
{
	return READ_ONCE(p->se.avg.util_avg);
}


static bool get_task_name(pid_t pid, struct task_struct *in_task, char *name)
{
	struct task_struct *task = NULL;
	int ret = false;

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (task && (task == in_task)) {
		strncpy(name, task->comm, TASK_COMM_LEN);
		ret = true;
	}
	rcu_read_unlock();
	return ret;
}

void init_rt_info_list(struct rt_info_list_node *new_node)
{
	if (new_node) {
		new_node->handle = handle_current % MAX_HANDLE_NUM;
		handle_current = (handle_current + 1) % MAX_HANDLE_NUM;
		new_node->seed_index = 0;
		new_node->total_count = 1;
		new_node->start_ktime = ktime_to_ms(ktime_get());
	}
}

void remove_rt_info_node(struct rt_info_list_node *node)
{
	if (!node)
		return;
	pr_info("[waker_identify] %d removed", node->seed_pid);
	list_del(&node->list_node);
	kfree(node);
}

void remove_rt_info_node_by_wq(struct work_struct *wq_for_kfree)
{
	struct rt_info_list_node *tmp, *tmp_for_safe;

	write_lock(&rt_info_rwlock);
	list_for_each_entry_safe(tmp, tmp_for_safe, &rt_info_list, list_node) {
		if (ktime_to_ms(ktime_get()) - tmp->start_ktime > DEAFAULT_HOLD_KTIME_MS) {
			list_del(&tmp->list_node);
			pr_info("[waker_identify] %d removed", tmp->seed_pid);
			kfree(tmp);
		}
	}
	global_doing_free = 0;
	write_unlock(&rt_info_rwlock);
}

static int cmp_waker_count(const void *ax, const void *bx)
{
	struct wake_message *prev, *next;

	prev = (struct wake_message *)ax;
	next = (struct wake_message *)bx;

	if (unlikely(prev == NULL || next == NULL))
		return 0;

	if (prev->wake_count > next->wake_count)
		return -1;
	else if (prev->wake_count < next->wake_count)
		return 1;
	else
		return 0;
}

static inline void reset_rthread_results(int *rthread_results)
{
	int i;

	for (i = 0; i < RESULT_NUM_MAX; i++)
		rthread_results[i] = INVALID_THREAD_IDX;
}


static inline void populate_rthread(struct rt_info_list_node *node, int *rthread_results, int result_index, int index)
{
	int i = 0;
	int waker_index = 0;
	int curr_waker_index;
	struct related_thread *curr_waker_thread = NULL;
	struct related_thread *thread = &node->rt_thread[index];

	if (unlikely(thread == NULL))
		return;

	/*
	 * need to sort the wakers only if
	 * the number is bigger than MAX_BREADTH
	 */
	if (thread->waker_num >= MAX_BREADTH)
		sort(thread->waker_array, thread->waker_num,
		     sizeof(struct wake_message), &cmp_waker_count, NULL);

	do {
		if (waker_index < thread->waker_num) {
			curr_waker_index = thread->waker_array[waker_index].wake_source_index;
			if (unlikely(curr_waker_index >= MAX_TID_COUNT)) {
				waker_index++;
				rthread_results[result_index++] = INVALID_THREAD_IDX;
				continue;
			}

			curr_waker_thread =
				&node->rt_thread[curr_waker_index];
			if (unlikely(curr_waker_thread == NULL)) {
				waker_index++;
				rthread_results[result_index++] = INVALID_THREAD_IDX;
				continue;
			}

			if (curr_waker_thread->visted) {
				waker_index++;
				continue;
			} else {
				rthread_results[result_index++] = curr_waker_index;
				curr_waker_thread->visted = true;
				waker_index++;
			}
		} else {
			rthread_results[result_index++] = INVALID_THREAD_IDX;
		}

		i++;
	} while (i < MAX_BREADTH);
}

static inline void hierarchical_bfs_traverse(struct rt_info_list_node *node, int *rthread_results)
{
	int i = 0;
	int seed_index;
	int result_start_index;
	struct related_thread *first_thread = NULL;

	reset_rthread_results(rthread_results);

	seed_index = node->seed_index;

	/* visit the first thread */
	first_thread = &node->rt_thread[seed_index];
	if (unlikely(first_thread == NULL))
		return;

	first_thread->visted = true;
	rthread_results[0] = seed_index;

	do {
		seed_index = rthread_results[i];
		if (seed_index != INVALID_THREAD_IDX) {
			result_start_index = i * MAX_BREADTH + 1;
			populate_rthread(node, rthread_results, result_start_index,
					 seed_index);
		}
		i++;
	} while (i < result_search_num);
}

static inline int get_child_rthread_index(struct rt_info_list_node *node, struct task_struct *task)
{
	struct related_thread *threads = node->rt_thread;
	int head, tail, pivot;
	pid_t target_pid = task->pid;

	head = 0;
	tail = node->total_count - 1;

	while (tail >= head) {
		pivot = (head + tail) / 2;
		if (threads[pivot].pid == target_pid)
			return pivot;
		else if (threads[pivot].pid < target_pid)
			head = pivot + 1;
		else if (threads[pivot].pid > target_pid)
			tail = pivot - 1;
	}

	return MAX_TID_COUNT;
}

static int bsearch_insert_index(struct rt_info_list_node *node, struct task_struct *task)
{
	struct related_thread *threads = node->rt_thread;
	int head, tail, pivot;
	int target_pid = task->pid;

	head = 0;
	tail = node->total_count - 1;

	if ((threads[head].pid > target_pid) && (node->total_count < MAX_TID_COUNT))
		return head;

	if ((threads[tail].pid < target_pid) && (node->total_count < MAX_TID_COUNT)) {
		tail++;
		return tail;
	}

	while (tail >= head) {
		pivot = (head + tail) / 2;
		if (threads[pivot].pid == target_pid)
			return pivot;
		else if ((pivot > 0) && (target_pid > threads[pivot - 1].pid) && (target_pid < threads[pivot].pid))
			return pivot;
		else if (threads[pivot].pid < target_pid)
			head = pivot + 1;
		else if (threads[pivot].pid > target_pid)
			tail = pivot - 1;
	}

	return INT_MAX;
}

static inline void fix_child_rthread_waker(struct rt_info_list_node *node, int threshold)
{
	int total_count = node->total_count;
	struct related_thread *thread = NULL;
	int index, waker_index, waker_bit_index;

	for (index = 0; index < total_count; index++) {
		thread = &node->rt_thread[index];

		if (thread == NULL || thread->waker_num == 0)
			continue;

		for (waker_index = 0; waker_index < thread->waker_num; waker_index++) {
			waker_bit_index = thread->waker_array[waker_index].wake_source_index;
			if (waker_bit_index < threshold)
				continue;

			__clear_bit(waker_bit_index, thread->waker_bitmap);
			waker_bit_index++;
			__set_bit(waker_bit_index, thread->waker_bitmap);
			thread->waker_array[waker_index].wake_source_index = waker_bit_index;
		}
	}
}

static inline void init_child_rthread(struct rt_info_list_node *node, int index, struct task_struct *task)
{
	struct related_thread *thread = &node->rt_thread[index];
	int i;

	if (unlikely(thread == NULL))
		return;

	thread->tgid = task->tgid;
	thread->pid = task->pid;
	thread->task = task;
	thread->total_count = 0;
	thread->visted = false;
	for (i = 0; i < thread->waker_num; i++) {
		thread->waker_array[i].wake_source_index = INVALID_THREAD_IDX;
		thread->waker_array[i].wake_count = 0;
	}
	thread->waker_num = 0;
	memset(thread->waker_bitmap, 0, BITS_TO_LONGS(MAX_TID_COUNT) * sizeof(unsigned long));
}



static ssize_t waker_info_proc_write(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	char page[32] = {0};
	int ret;
	int handel;

	ret = simple_write_to_buffer(page, sizeof(page) - 1, ppos, buf, count);
	if (ret <= 0)
		return ret;

	ret = sscanf(page, "%d", &handel);
	if (ret != 1)
		return -EINVAL;

	if (handel < 0 || handel > MAX_HANDLE_NUM)
		return -EINVAL;

	write_lock(&rt_info_rwlock);
	handle_show = handel;
	write_unlock(&rt_info_rwlock);

	return count;
}

static int waker_info_show(struct seq_file *m, void *v)
{
	int i, index, wake_source_index, wake_array_index;
	int result_index[RESULT_NUM_MAX];
	struct rt_info_list_node *tmp;
	char *page;
	struct related_thread *thread;
	char task_name[TASK_COMM_LEN];
	ssize_t len = 0;

	if (list_empty(&rt_info_list))
		return -ESRCH;

	page = kzalloc(RESULT_PAGE_SIZE, GFP_ATOMIC);

	if (!page)
		return -ENOMEM;

	write_lock(&rt_info_rwlock);
	list_for_each_entry(tmp, &rt_info_list, list_node) {
		if (tmp->handle == handle_show)
			goto out;
	}

	write_unlock(&rt_info_rwlock);
	len += snprintf(page + len, RESULT_PAGE_SIZE - len, "NULL");
	seq_puts(m, page);
	kfree(page);
	return 0;

out:
	tmp->stopped = 1;
	hierarchical_bfs_traverse(tmp, result_index);
	thread = tmp->rt_thread;
	for (i = 1; i < RESULT_NUM_MAX; i++) {
		wake_source_index = (i - 1) / MAX_BREADTH;
		index = result_index[i];
		wake_array_index = i - 1 - wake_source_index * MAX_BREADTH;
		wake_source_index = result_index[(i - 1) / MAX_BREADTH];
		if ((index >= 0) && (get_task_name(thread[index].pid, thread[index].task, task_name))) {
			len += snprintf(page + len, RESULT_PAGE_SIZE - len, "%d;%d;%s;%u;%d;%d;%d;%d\n",
				thread[index].tgid, thread[index].pid, task_name, thread[wake_source_index].waker_array[wake_array_index].wake_count,
					task_util(thread[index].task), thread[wake_source_index].pid, task_nice(thread[index].task)+120, thread[wake_source_index].total_count);
			if (unlikely(atomic_read(&global_debug))) {
				pr_info("[waker_identify]: %d;%d;%s;%u;%d;%d;%d;%d\n",
					thread[index].tgid, thread[index].pid, task_name, thread[wake_source_index].waker_array[wake_array_index].wake_count,
					task_util(thread[index].task), thread[wake_source_index].pid, task_nice(thread[index].task)+120, thread[wake_source_index].total_count);
			}
		} else {
			len += snprintf(page + len, RESULT_PAGE_SIZE - len, "%d;%d;%s;%d;%d;%d;%d;%d\n",
				-1, -1, "-1", -1, -1, -1, -1, -1);
			if (unlikely(atomic_read(&global_debug))) {
				pr_info("[waker_identify]: %d;%d;%s;%d;%d;%d;%d;%d\n",
					-1, -1, "-1", -1, -1, -1, -1, -1);
			}
		}
	}
	remove_rt_info_node(tmp);
	write_unlock(&rt_info_rwlock);
	if (len > 0)
		seq_puts(m, page);

	kfree(page);

	return 0;
}


static int waker_info_proc_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, waker_info_show, inode);
}

static const struct proc_ops waker_info_proc_ops = {
	.proc_open		= waker_info_proc_open,
	.proc_write		= waker_info_proc_write,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};


static int rt_info_show(struct seq_file *m, void *v)
{
	char *page;
	ssize_t len = 0;


	page = kzalloc(RESULT_PAGE_SIZE, GFP_ATOMIC);

	if (!page)
		return -ENOMEM;

	read_lock(&rt_info_rwlock);
	len += snprintf(page + len, RESULT_PAGE_SIZE - len, "%d", handle_current-1);
	read_unlock(&rt_info_rwlock);

	if (len > 0)
		seq_puts(m, page);

	kfree(page);

	return 0;
}

static int rt_info_proc_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, rt_info_show, inode);
}


static ssize_t rt_info_proc_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	int ret;
	char page[128] = {0};
	char *iter = page;
	struct task_struct *task = NULL;
	pid_t pid;
	int in_uid, for_wakee;
	struct rt_info_list_node *new_node;

	ret = simple_write_to_buffer(page, sizeof(page), ppos, buf, count);
	if (ret <= 0)
		return ret;

	new_node = kzalloc(sizeof(*new_node), GFP_ATOMIC);
	if (!new_node)
		return -ENOMEM;

	ret = sscanf(iter, "%d;%d;%d", &pid, &in_uid, &for_wakee);
	if (ret != 3) {
		kfree(new_node);
		return -EINVAL;
	}

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (task)
		get_task_struct(task);
	rcu_read_unlock();

	if (task) {
		write_lock(&rt_info_rwlock);
		init_rt_info_list(new_node);
		new_node->seed_task = task;
		new_node->seed_pid = pid;
		new_node->stopped = 0;
		new_node->in_uid = in_uid;
		new_node->for_wakee = for_wakee;
		new_node->app_uid = task_uid(task).val;
		init_child_rthread(new_node, 0, task);
		list_add(&new_node->list_node, &rt_info_list);
		write_unlock(&rt_info_rwlock);
	} else {
		kfree(new_node);
	}

	if (task){
		put_task_struct(task);
		pr_info("[waker_identify] %d start", pid);
	}

	return count;
}

static const struct proc_ops rt_info_proc_ops = {
	.proc_open		= rt_info_proc_open,
	.proc_write		= rt_info_proc_write,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static int enable_show(struct seq_file *m, void *v)
{
	char *page;
	ssize_t len = 0;


	page = kzalloc(RESULT_PAGE_SIZE, GFP_ATOMIC);
	if (!page)
		return -ENOMEM;

	read_lock(&rt_info_rwlock);
	len += snprintf(page + len, RESULT_PAGE_SIZE - len, "%d", global_enable);
	read_unlock(&rt_info_rwlock);

	if (len > 0)
		seq_puts(m, page);

	kfree(page);

	return 0;
}

static int enable_proc_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, enable_show, inode);
}


static ssize_t enable_proc_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	char page[32] = {0};
	int ret;
	int enable;

	ret = simple_write_to_buffer(page, sizeof(page) - 1, ppos, buf, count);
	if (ret <= 0)
		return ret;

	ret = sscanf(page, "%d", &enable);
	if (ret != 1)
		return -EINVAL;
	enable = (enable == 0 ? 0 : 1);
	write_lock(&rt_info_rwlock);
	global_enable = enable;
	write_unlock(&rt_info_rwlock);

	return count;
}

static const struct proc_ops enable_proc_ops = {
	.proc_open		= enable_proc_open,
	.proc_write		= enable_proc_write,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static int debug_enable_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u", atomic_read(&global_debug));
	return 0;
}

static int debug_enable_proc_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, debug_enable_show, inode);
}


static ssize_t debug_enable_proc_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	char page[32] = {0};
	int ret;
	int enable;

	ret = simple_write_to_buffer(page, sizeof(page) - 1, ppos, buf, count);
	if (ret <= 0)
		return ret;

	ret = sscanf(page, "%d", &enable);
	if (ret != 1)
		return -EINVAL;
	enable = !!enable;
	atomic_set(&global_debug, enable);

	return count;
}

static const struct proc_ops debug_enable_proc_ops = {
	.proc_open		= debug_enable_proc_open,
	.proc_write		= debug_enable_proc_write,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

inline int need_exit(struct rt_info_list_node *tmp, struct task_struct *task)
{
	if (ktime_to_ms(ktime_get()) - tmp->start_ktime > DEAFAULT_HOLD_KTIME_MS) {
		if (!global_doing_free) {
			global_doing_free = 1;
			/* BUG  7499724 KASAN DEBUG
				1 kasan wakeup swapd-> try_to_wake_up hook
				2 try_to_wake_up->kfree
				3 kfree wake up swapd
				4 infinite loop */
			queue_delayed_work(system_wq, &wq_for_kfree, msecs_to_jiffies(1));
		}
		return 1;
	}

	if (!global_enable)
		return 1;

	if (tmp->stopped)
		return 1;

	if (current->pid == 0)
		return 1;

	if (tmp->in_uid && ((task_uid(current).val != tmp->app_uid) || (task_uid(task).val != tmp->app_uid)))
		return 1;

	return 0;
}

int waker_identify(struct rt_info_list_node *tmp, struct task_struct *task)
{
	int waker_index, wakee_index, j, last_index, waker_count;
	struct related_thread *thread;

	if (need_exit(tmp, task))
		return -1;

	wakee_index = get_child_rthread_index(tmp, task);
	if (wakee_index == MAX_TID_COUNT)
		return -1;

	waker_index = bsearch_insert_index(tmp, current);
	if (unlikely(waker_index == INT_MAX))
		return -1;

	if (tmp->rt_thread[waker_index].pid == current->pid) {
		thread = tmp->rt_thread;
		waker_count = thread[wakee_index].waker_num;
		if ((!test_bit(waker_index, thread[wakee_index].waker_bitmap)) && (waker_count < MAX_WAKER_COUNT)) {
			thread[wakee_index].waker_array[waker_count].wake_source_index = waker_index;
			thread[wakee_index].waker_array[waker_count].wake_count = 1;
			thread[wakee_index].waker_num++;
			__set_bit(waker_index, thread[wakee_index].waker_bitmap);
			thread[wakee_index].total_count++;
			return 1;
		} else {
			for (j = 0; j < thread[wakee_index].waker_num; j++) {
				if (thread[wakee_index].waker_array[j].wake_source_index == waker_index) {
					thread[wakee_index].waker_array[j].wake_count++;
					thread[wakee_index].total_count++;
					return 1;
				}
			}
		}
	} else if (tmp->total_count < MAX_TID_COUNT) {
		thread = tmp->rt_thread;
		waker_count = thread[wakee_index].waker_num;

		last_index = tmp->total_count-1;
		for (j = last_index; j >= waker_index; j--) {
			thread[j + 1] = thread[j];

			if ((thread[j].pid == tmp->seed_pid) && (j == tmp->seed_index))
				tmp->seed_index++;

			if (j == wakee_index)
				wakee_index++;
		}

		init_child_rthread(tmp, waker_index, current);
		tmp->total_count++;
		fix_child_rthread_waker(tmp, waker_index);

		if ((!test_bit(waker_index, thread[wakee_index].waker_bitmap)) && (waker_count < MAX_WAKER_COUNT)) {
			thread[wakee_index].waker_array[waker_count].wake_source_index = waker_index;
			thread[wakee_index].waker_array[waker_count].wake_count = 1;
			thread[wakee_index].waker_num++;
			__set_bit(waker_index, thread[wakee_index].waker_bitmap);
			thread[wakee_index].total_count++;
			return 1;
		}
	}
	return -1;
}

int wakee_identify(struct rt_info_list_node *tmp, struct task_struct *task)
{
	int waker_index, wakee_index, j, last_index, waker_count;
	struct related_thread *thread;

	if (need_exit(tmp, task))
		return -1;

	waker_index = get_child_rthread_index(tmp, current);
	if (waker_index == MAX_TID_COUNT)
		return -1;

	wakee_index = bsearch_insert_index(tmp, task);
	if (unlikely(wakee_index == INT_MAX))
		return -1;

	if (tmp->rt_thread[wakee_index].pid == task->pid) {
		thread = tmp->rt_thread;
		waker_count = thread[waker_index].waker_num;

		if ((!test_bit(wakee_index, thread[waker_index].waker_bitmap)) && (waker_count < MAX_WAKER_COUNT)) {
			thread[waker_index].waker_array[waker_count].wake_source_index = wakee_index;
			thread[waker_index].waker_array[waker_count].wake_count = 1;
			thread[waker_index].waker_num++;
			__set_bit(wakee_index, thread[waker_index].waker_bitmap);
			thread[waker_index].total_count++;
			return 1;
		} else {
			for (j = 0; j < thread[waker_index].waker_num; j++) {
				if (thread[waker_index].waker_array[j].wake_source_index == wakee_index) {
					thread[waker_index].waker_array[j].wake_count++;
					thread[waker_index].total_count++;
					return 1;
				}
			}
		}
	} else if (tmp->total_count < MAX_TID_COUNT) {
		thread = tmp->rt_thread;
		waker_count = thread[waker_index].waker_num;

		last_index = tmp->total_count-1;
		for (j = last_index; j >= wakee_index; j--) {
			thread[j + 1] = thread[j];

			if ((thread[j].pid == tmp->seed_pid) && (j == tmp->seed_index))
				tmp->seed_index++;

			if (j == waker_index)
				waker_index++;
		}

		init_child_rthread(tmp, wakee_index, task);
		tmp->total_count++;
		fix_child_rthread_waker(tmp, wakee_index);

		if ((!test_bit(wakee_index, thread[waker_index].waker_bitmap)) && (waker_count < MAX_WAKER_COUNT)) {
			thread[waker_index].waker_array[waker_count].wake_source_index = wakee_index;
			thread[waker_index].waker_array[waker_count].wake_count = 1;
			thread[waker_index].waker_num++;
			__set_bit(wakee_index, thread[waker_index].waker_bitmap);
			thread[waker_index].total_count++;
			return 1;
		}
	}
	return -1;
}

static void try_to_wake_up_success_hook_for_waker_identify(void *unused, struct task_struct *task)
{
	struct rt_info_list_node *tmp, *tmp_for_safe;
	int ret;

	if (in_interrupt())
		return;

	if (write_trylock(&rt_info_rwlock)) {
		if (list_empty(&rt_info_list)) {
			write_unlock(&rt_info_rwlock);
			return;
		}

		list_for_each_entry_safe(tmp, tmp_for_safe, &rt_info_list, list_node) {
			if (tmp->for_wakee) {
				ret = wakee_identify(tmp, task);
				continue;
			} else {
				ret = waker_identify(tmp, task);
				continue;
			}
		}
		write_unlock(&rt_info_rwlock);
	}
}
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
static void register_rt_info_vendor_hooks(void)
{
	/* Register vender hook in kernel/sched/core.c */
	register_trace_android_rvh_try_to_wake_up_success(try_to_wake_up_success_hook_for_waker_identify, NULL);
}
#else
static void register_rt_info_vendor_hooks(void)
{
	/* Register vender hook in kernel/sched/core.c */
	register_trace_android_rvh_try_to_wake_up(try_to_wake_up_success_hook_for_waker_identify, NULL);
}
#endif

static int __init waker_identify_init(void)
{
	waker_identify_dir = proc_mkdir("waker_identify", NULL);
	if (!waker_identify_dir) {
		pr_err("fail to mkdir /proc/waker_identify\n");
		return -ENOMEM;
	}

	register_rt_info_vendor_hooks();

	INIT_DELAYED_WORK(&wq_for_kfree, remove_rt_info_node_by_wq);

	proc_create_data("waker_info", 0664, waker_identify_dir, &waker_info_proc_ops, NULL);
	proc_create_data("rt_info", 0664, waker_identify_dir, &rt_info_proc_ops, NULL);
	proc_create_data("enable", 0664, waker_identify_dir, &enable_proc_ops, NULL);
	proc_create_data("debug_enable", 0664, waker_identify_dir, &debug_enable_proc_ops, NULL);
	return 0;
}

static void __exit waker_identify_exit(void)
{
}

module_init(waker_identify_init);
module_exit(waker_identify_exit);
MODULE_LICENSE("GPL v2");

