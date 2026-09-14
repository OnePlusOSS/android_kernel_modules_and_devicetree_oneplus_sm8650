#ifndef CONFIG_OPLUS_MM_FRR_KUNIT
#define pr_fmt(fmt) "[FRR] " fmt
#endif
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sort.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/version.h>
// #ifdef CONFIG_OPLUS_FEATURE_MM_OSVELTE
#include "../mm_osvelte/mm-config.h"
// #endif

#include <trace/hooks/fs.h>
#include <trace/hooks/mm.h>

#undef CONFIG_OPLUS_MM_FRR_DEBUG_LOCAL

#define FILEPATH_STR_MAX (256)
#define FRR_PROC_CMD_BUF_MAX (128)
#define IO_INFO_RANGE_STR_MAX (64)
#define IO_INFO_FORMAT_MAGIC (0x4D414749434E554D) // MAGICNUM
#define IO_INFO_RECORD_MAX (96 * 1024)
#define IO_MONITOR_FORMAT_BUF_MAX (2 * 1024 * 1024)

typedef enum {
	IO_MONITOR_STATUS_INIT,
	IO_MONITOR_STATUS_START,
	IO_MONITOR_STATUS_STOP,
	IO_MONITOR_STATUS_FINISH,
	IO_MONITOR_STATUS_FINISHED,
	IO_MONITOR_STATUS_RESET,
	IO_MONITOR_STATUS_UNKNOWN,
} io_monitor_status;

typedef enum {
	IO_MONITOR_TYPE_READ,
	IO_MONITOR_TYPE_WRITE,
	IO_MONITOR_TYPE_UNKNOWN,
} io_monitor_type;

typedef struct {
	struct file *file;
	struct inode *inode;
	pgoff_t offset;
	pgoff_t size;
	bool valid;
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	/*
	 * when compact, io_info size maybe changed,
	 * so we need to record the original size.
	 */
	pgoff_t orgin_size;
#endif
} io_info;

typedef struct {
	io_info *r_buf;
	unsigned int cur;
	unsigned int max;
	bool should_fput;
} io_info_buffer;

typedef enum {
	IO_MONITOR_STATISTICS_LOST, // io_info lost
	IO_MONITOR_STATISTICS_RESULT_LOST, // io_info to result buffer lost
	IO_MONITOR_STATISTICS_MAX,
} io_monitor_statistics;

typedef struct {
	io_info_buffer *buf;
	spinlock_t m_lock;
	io_monitor_type type;
	io_monitor_status status;
	int tgid;
	char *format_buf;
	unsigned int format_buf_size;
	unsigned int format_buf_cur;
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	char *debug_buf;
	unsigned int debug_buf_size;
	unsigned int debug_buf_cur;
	unsigned int statistics[IO_MONITOR_STATISTICS_MAX];
	unsigned int format_time_ns;
	unsigned int debug_time_ns;
#endif
} io_monitor;

static io_monitor *g_read_monitor = NULL;

/* io info functions */
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG_LOCAL
static void io_info_show(io_info *info)
{
	if (!info)
		return;

	pr_info("inode %p, offset %lu, size %lu, orign size %lu, valid %s\n",
		info->inode, info->offset, info->size, info->orgin_size,
		info->valid ? "true" : "false");
}
#endif
#endif

static int _io_info_compare_by_inode(const void *a, const void *b)
{
	io_info *a_info = (io_info *)a;
	io_info *b_info = (io_info *)b;

	if ((uintptr_t)a_info->inode > (uintptr_t)b_info->inode) {
		return 1;
	} else if ((uintptr_t)a_info->inode < (uintptr_t)b_info->inode) {
		return -1;
	} else {
		if (a_info->offset > b_info->offset)
			return 1;
		else if (a_info->offset < b_info->offset)
			return -1;
		else
			return 0;
	}
}

static void _io_info_swap(void *a, void *b, int size)
{
	io_info tmp;
	io_info *info_a = (io_info *)a;
	io_info *info_b = (io_info *)b;

	memcpy(&tmp, info_a, sizeof(io_info));
	memcpy(info_a, info_b, sizeof(io_info));
	memcpy(info_b, &tmp, sizeof(io_info));
}

/* io info buffer functions */
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG_LOCAL
static void io_info_buffer_show(io_info_buffer *buffer)
{
	int i = 0;

	if (!buffer)
		return;

	for (i = 0; i < buffer->cur; i++) {
		io_info_show(&buffer->r_buf[i]);
	}
}
#endif
#endif

static void io_info_buffer_compact(io_info_buffer *buf)
{
	int i = 0;
	io_info *pre = NULL;

	if (!buf)
		return;

	sort(buf->r_buf, buf->cur, sizeof(io_info),
		&_io_info_compare_by_inode, &_io_info_swap);

	pre = &buf->r_buf[0];
	for (i = 1; i < buf->cur; i++) {
		if (buf->r_buf[i].inode == pre->inode) {
			if ((pre->offset + pre->size) < buf->r_buf[i].offset) {
				pre = &buf->r_buf[i];
			} else if ((pre->offset + pre->size) > buf->r_buf[i].offset) {
				if ((pre->offset + pre->size) < (buf->r_buf[i].offset + buf->r_buf[i].size)) {
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
					if (pre->orgin_size == 0)
						pre->orgin_size = pre->size;
#endif
					pre->size = buf->r_buf[i].offset + buf->r_buf[i].size - pre->offset;
				}
				buf->r_buf[i].valid = false;
			} else {
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
				if (pre->orgin_size == 0)
					pre->orgin_size = pre->size;
#endif
				pre->size += buf->r_buf[i].size;
				buf->r_buf[i].valid = false;
			}
		} else {
			pre = &buf->r_buf[i];
		}
	}

#ifdef CONFIG_OPLUS_MM_FRR_DEBUG_LOCAL
	io_info_buffer_show(buf);
#endif
}

static inline void set_io_info_buffer_should_fput_flag(io_info_buffer *buf)
{
	buf->should_fput = true;
}

static inline void clear_io_info_buffer_should_fput_flag(io_info_buffer *buf)
{
	buf->should_fput = false;
}

static inline bool get_io_info_buffer_should_fput_flag(io_info_buffer *buf)
{
	return buf->should_fput;
}

static void io_info_buffer_put_file(io_info_buffer *buf)
{
	int i = 0;

	if (!get_io_info_buffer_should_fput_flag(buf))
		return;
	for (i = 0; i < buf->cur; i++) {
		if (buf->r_buf[i].file) {
			fput(buf->r_buf[i].file);
			buf->r_buf[i].file = NULL;
		}
	}
	clear_io_info_buffer_should_fput_flag(buf);
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	pr_info("io_info_buffer_put_file done\n");
#endif
}

static inline void io_info_buffer_reset(io_info_buffer *buf)
{
	io_info_buffer_put_file(buf);
	buf->cur = 0;
}

static io_info_buffer *io_info_buffer_alloc(unsigned int max)
{
	io_info_buffer *io_info_buf = NULL;

	io_info_buf = kmalloc(sizeof(io_info_buffer), GFP_KERNEL);
	if (!io_info_buf) {
		pr_err("Failed to kmalloc io_info_buffer \n");
		return NULL;
	}

	io_info_buf->r_buf = vzalloc(max * sizeof(io_info));
	if (!io_info_buf->r_buf) {
		pr_err("Failed to vzalloc r_buf\n");
		kfree(io_info_buf);
		return NULL;
	}

	io_info_buf->max = max;
	io_info_buf->cur = 0;
	clear_io_info_buffer_should_fput_flag(io_info_buf);
	return io_info_buf;
}

static inline void io_info_buffer_free(io_info_buffer *buf)
{
	io_info_buffer_reset(buf);
	if (buf) {
		if (buf->r_buf)
			vfree(buf->r_buf);
		kfree(buf);
	}
}

static inline bool io_info_buffer_check(io_info_buffer *buffer)
{
	if (buffer->cur == buffer->max)
		return false;

	return true;
}

static bool io_info_buffer_record(io_info_buffer *buffer,
	struct file *file, pgoff_t offset, unsigned long req_size)
{
	io_info *info = NULL;

	if (!io_info_buffer_check(buffer)) {
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
		pr_err("io_info_buffer is full\n");
#endif
		return false;
	}

#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	if (file == NULL)
		pr_err("io_info_buffer_record, file is null\n");
#endif
#ifndef CONFIG_OPLUS_MM_FRR_KUNIT
	get_file(file);
#endif
	info = buffer->r_buf + buffer->cur;
	info->file = file;
#ifndef CONFIG_OPLUS_MM_FRR_KUNIT
	info->inode = file_inode(file);
#else
	info->inode = NULL;
#endif
	info->offset = offset;
	info->size = req_size;
	info->valid = true;
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	info->orgin_size = 0;
#endif
	buffer->cur++;
	return true;
}

struct backing_file_cp {
	struct file file;
	struct path user_path;
};

static inline const struct path *get_real_path(struct file *f)
{
	if (f == NULL)
		return NULL;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return &f->f_path;
#else
	if (unlikely(f->f_mode & FMODE_BACKING))
		return &(container_of(f, struct backing_file_cp, file)->user_path);
	return &f->f_path;
#endif
}

/* io monitor function */
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
static void io_monitor_count(io_monitor *monitor, io_monitor_statistics type)
{
	if (monitor && type < IO_MONITOR_STATISTICS_MAX) {
		monitor->statistics[type]++;
	}
}

static bool __write_line_to_debug_buf(io_monitor *monitor, char *str, unsigned int len)
{
	if (len + 1 < monitor->debug_buf_size  - monitor->debug_buf_cur) {
		memcpy(monitor->debug_buf + monitor->debug_buf_cur, str, len);
		monitor->debug_buf_cur += len;
		monitor->debug_buf[monitor->debug_buf_cur] = '\n';
		monitor->debug_buf_cur++;
		return true;
	}
	return false;
}

/* This function call should be protected by monitor->m_lock */
static void io_monitor_fill_debug_buf(io_monitor *monitor)
{
	int i = 0;
	io_info *pre = NULL;
	char f_path_str[FILEPATH_STR_MAX] = {0};
	char *f_path = NULL;
	unsigned int tmp_len = 0;
	char range_str[IO_INFO_RANGE_STR_MAX] = {0};
	const struct path *real_path = NULL;
	io_info_buffer *buf = monitor->buf;
	ktime_t time_delta;
	ktime_t time_start = ktime_get();

	for (i = 0; i < buf->cur; i++) {
RETRY:
		if (buf->r_buf[i].valid) {
			if (pre == NULL) {
				real_path = get_real_path(buf->r_buf[i].file);
				if (real_path == NULL)
					continue;

				f_path = d_path(real_path, f_path_str, FILEPATH_STR_MAX);
				if (IS_ERR(f_path)) {
					pr_err("fail to get path, ignore this record\n");
					continue;
				}
				if (!__write_line_to_debug_buf(monitor, f_path, strlen(f_path)))
					break;
				tmp_len = scnprintf(range_str, IO_INFO_RANGE_STR_MAX, "[%lu,%lu]",
					buf->r_buf[i].offset, buf->r_buf[i].size);
				if (!__write_line_to_debug_buf(monitor, range_str, tmp_len))
					break;
				pre = &buf->r_buf[i];
			} else {
				if (pre->inode == buf->r_buf[i].inode) {
					tmp_len = scnprintf(range_str, IO_INFO_RANGE_STR_MAX, "[%lu,%lu]",
						buf->r_buf[i].offset, buf->r_buf[i].size);
					if (!__write_line_to_debug_buf(monitor, range_str, tmp_len))
						break;
				} else {
					pre = NULL;
					goto RETRY;
				}
			}
		}
	}
	time_delta = ktime_sub(ktime_get(), time_start);
	monitor->debug_time_ns = ktime_to_ns(time_delta);
}
#endif

static bool __write_to_format_buf(io_monitor *monitor, char *str, unsigned int len, u64 value)
{
	u64 *tmp;

	if (str == NULL) {
		if (sizeof(u64) + sizeof(u64) < monitor->format_buf_size - monitor->format_buf_cur) {
			tmp = (u64 *)(monitor->format_buf + monitor->format_buf_cur);
			*tmp = value;
			monitor->format_buf_cur += sizeof(u64);
			return true;
		}
	} else {
		if (len + sizeof(u64) < monitor->format_buf_size - monitor->format_buf_cur) {
			memcpy(monitor->format_buf + monitor->format_buf_cur, str, len);
			monitor->format_buf_cur += len;
			return true;
		}
	}
	return false;
}

static bool __write_magicnum_to_format_buf(io_monitor *monitor)
{
	u64 *tmp;

	if (sizeof(u64) < monitor->format_buf_size - monitor->format_buf_cur) {
		tmp = (u64 *)(monitor->format_buf + monitor->format_buf_cur);
		*tmp = IO_INFO_FORMAT_MAGIC;
		monitor->format_buf_cur += sizeof(u64);
		return true;
	}
	return false;
}

static bool __write_range_to_format_buf(io_monitor *monitor, u64 offset, u64 size)
{
	u64 *tmp;

	if (sizeof(u64) * 2 + sizeof(u64) < monitor->format_buf_size  - monitor->format_buf_cur) {
		tmp = (u64 *)(monitor->format_buf + monitor->format_buf_cur);
		*tmp = offset;
		tmp++;
		*tmp = size;
		monitor->format_buf_cur += sizeof(u64) * 2;
		return true;
	}
	return false;
}

/*
 * This function call should be protected by monitor->m_lock
 * |-----u64----|--------------------|---u64--|--u64---|--u64---|--u64---|-----|-----u64-----|
 * | f_path len |       f_path       | offset |  size  | offset |  size  | ... |  end_magic  |
 * |-----------------------------------------------------------------------------------------|
 */
static void io_monitor_fill_format_buf(io_monitor *monitor)
{
	int i = 0;
	io_info *pre = NULL;
	char f_path_str[FILEPATH_STR_MAX] = {0};
	char *f_path = NULL;
	u64 tmp_len = 0;
	const struct path *real_path = NULL;
	io_info_buffer *buf = monitor->buf;
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	ktime_t time_start = ktime_get();
	ktime_t time_delta;
#endif

	for (i = 0; i < buf->cur; i++) {
RETRY:
		if (buf->r_buf[i].valid) {
			if (pre == NULL) {
				real_path = get_real_path(buf->r_buf[i].file);
				if (!real_path)
					continue;

				f_path = d_path(real_path, f_path_str, FILEPATH_STR_MAX);
				if (IS_ERR(f_path)) {
					pr_err("fail to get path, ignore this record\n");
					continue;
				}
				tmp_len = strlen(f_path);
				if (!__write_to_format_buf(monitor, NULL, 0, tmp_len))
					break;
				if (!__write_to_format_buf(monitor, f_path, tmp_len, 0))
					break;
				if (!__write_range_to_format_buf(monitor,
					buf->r_buf[i].offset, buf->r_buf[i].size))
					break;
				pre = &buf->r_buf[i];
			} else {
				if (pre->inode == buf->r_buf[i].inode) {
					if (!__write_range_to_format_buf(monitor, buf->r_buf[i].offset,
						buf->r_buf[i].size))
						break;
				} else {
					pre = NULL;
					__write_magicnum_to_format_buf(monitor);
					goto RETRY;
				}
			}
		}
	}
	__write_magicnum_to_format_buf(monitor);
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	time_delta = ktime_sub(ktime_get(), time_start);
	monitor->format_time_ns = ktime_to_ns(time_delta);
#endif
}

static io_monitor *io_monitor_alloc(io_monitor_type type, unsigned int max, unsigned int buf_size)
{
	io_monitor *monitor = kzalloc(sizeof(io_monitor), GFP_KERNEL);

	if (!monitor) {
		pr_err("io_monitor_alloc: failed to allocate memory for io_monitor\n");
		return NULL;
	}

	monitor->buf = io_info_buffer_alloc(max);
	if (!monitor->buf) {
		kfree(monitor);
		pr_err("io_monitor_alloc: failed to allocate memory for io_info_buffer\n");
		return NULL;
	}
	monitor->format_buf = vzalloc(buf_size);
	if (!monitor->format_buf) {
		io_info_buffer_free(monitor->buf);
		kfree(monitor);
		return NULL;
	}
	monitor->format_buf_size = buf_size;
	monitor->format_buf_cur = 0;
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	monitor->debug_buf = vzalloc(buf_size);
	if (!monitor->debug_buf) {
		io_info_buffer_free(monitor->buf);
		vfree(monitor->format_buf);
		kfree(monitor);
		return NULL;
	}
	monitor->debug_buf_size = buf_size;
	monitor->debug_buf_cur = 0;
#endif
	monitor->type = type;
	monitor->tgid = -1;
	monitor->status = IO_MONITOR_STATUS_UNKNOWN;
	spin_lock_init(&monitor->m_lock);
	return monitor;
}

static void io_monitor_free(io_monitor *monitor)
{
	if (monitor == NULL)
		return;

	io_info_buffer_free(monitor->buf);
	monitor->buf = NULL;
	vfree(monitor->format_buf);
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	vfree(monitor->debug_buf);
#endif
	kfree(monitor);
}

static void io_monitor_reset(io_monitor *monitor)
{
	io_info_buffer_reset(monitor->buf);

	monitor->format_buf_cur = 0;
	memset(monitor->format_buf, 0, monitor->format_buf_size);
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	monitor->debug_buf_cur = 0;
	monitor->debug_time_ns = 0;
	monitor->format_time_ns = 0;
	memset(monitor->debug_buf, 0, monitor->debug_buf_size);
#endif
	monitor->tgid = -1;
	monitor->status = IO_MONITOR_STATUS_INIT;
}

static inline io_monitor *get_io_monitor(io_monitor_type type)
{
	return g_read_monitor;
}

/* This function call should be protected by monitor->m_lock */
static bool io_monitor_record_check(io_monitor *monitor, int target_tgid)
{
	return monitor &&
		   (monitor->tgid == target_tgid) &&
		   (monitor->status == IO_MONITOR_STATUS_START);
}

static void io_monitor_record(struct file *file, pgoff_t offset,
	unsigned long req_size, io_monitor_type type)
{
	int target_tgid = -1;
	io_monitor *monitor = get_io_monitor(type);

	/* quick check */
	if (file == NULL || monitor == NULL ||
		monitor->status != IO_MONITOR_STATUS_START)
		return;

	target_tgid = (int)task_tgid_nr(current);
	/* quick check */
	if (target_tgid != monitor->tgid)
		return;

	spin_lock(&monitor->m_lock);
	if (offset >= ULONG_MAX || req_size >= ULONG_MAX ||
		offset + req_size < offset)
		goto FAIL;

	/* repeat check */
	if (!io_monitor_record_check(monitor, target_tgid)) {
		pr_err("io_monitor_record_check fail\n");
		goto FAIL;
	}

	if (!io_info_buffer_record(monitor->buf, file, offset, req_size)) {
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
		io_monitor_count(monitor, IO_MONITOR_STATISTICS_LOST);
#endif
	}
FAIL:
	spin_unlock(&monitor->m_lock);
	return;
}

static void io_monitor_finish_handle(io_monitor *monitor)
{
	if (monitor == NULL)
		return;

	io_info_buffer_compact(monitor->buf);
	io_monitor_fill_format_buf(monitor);
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	io_monitor_fill_debug_buf(monitor);
#endif
	io_info_buffer_put_file(monitor->buf);
	monitor->status = IO_MONITOR_STATUS_FINISHED;
}

static bool io_monitor_status_set(io_monitor_status status,
	io_monitor_type type, int tgid)
{
	io_monitor *monitor = NULL;
	bool ret = false;

	if ((status < IO_MONITOR_STATUS_INIT  || status > IO_MONITOR_STATUS_FINISH)
		&& status != IO_MONITOR_STATUS_RESET) {
		pr_err("input monitor status invaild\n");
		return false;
	}

	monitor = get_io_monitor(type);
	if (monitor == NULL)
		return false;

	spin_lock(&monitor->m_lock);
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	pr_info("tgid[%d, %d], monitor status will change, %d->%d, \n",
		monitor->tgid, tgid,
		monitor->status, status);
#endif
	switch (status) {
	case IO_MONITOR_STATUS_INIT:
		if (monitor->status == IO_MONITOR_STATUS_INIT ||
			monitor->status == IO_MONITOR_STATUS_FINISHED ||
			monitor->status == IO_MONITOR_STATUS_STOP ||
			monitor->status == IO_MONITOR_STATUS_UNKNOWN) {
			io_monitor_reset(monitor);
			monitor->tgid = tgid;
			monitor->status = status;
			ret = true;
		} else {
			pr_err("monitor can not change to INIT status\n");
		}
		break;
	case IO_MONITOR_STATUS_START:
		if (monitor->status == IO_MONITOR_STATUS_INIT) {
			if (monitor->tgid != tgid) {
				pr_err("monitor can not change to START status, tgid is not equal\n");
				break;
			}
			set_io_info_buffer_should_fput_flag(monitor->buf);
			monitor->status = status;
			ret = true;
		} else {
			pr_err("monitor can not change to START status\n");
		}
		break;
	case IO_MONITOR_STATUS_STOP:
		if (monitor->status == IO_MONITOR_STATUS_START ||
			monitor->status == IO_MONITOR_STATUS_STOP) {
			if (monitor->tgid != tgid) {
				pr_err("monitor can not change to STOP status, tgid is not equal\n");
				break;
			}
			monitor->status = status;
			ret = true;
		} else {
			pr_err("monitor can not change to STOP status\n");
		}
		break;
	case IO_MONITOR_STATUS_FINISH:
		if (monitor->status == IO_MONITOR_STATUS_STOP) {
			if (monitor->tgid != tgid) {
				pr_err("monitor can not change to FINISH status, tgid is not equal\n");
				break;
			}
			monitor->status = status;
			io_monitor_finish_handle(monitor);
			ret = true;
		} else {
			pr_err("monitor can not change to FINISH status\n");
		}
		break;
	case IO_MONITOR_STATUS_RESET:
		pr_info("force monitor %d status switch to INIT\n", monitor->tgid);
		io_monitor_reset(monitor);
		monitor->status = IO_MONITOR_STATUS_UNKNOWN;
		ret = true;
		break;
	default:
		break;
	}
	spin_unlock(&monitor->m_lock);
	return ret;
}

static void trace_android_vh_filemap_read_hook(void *data, struct file *file, loff_t pos, size_t size)
{
	pgoff_t index = pos >> PAGE_SHIFT;
	pgoff_t last_index = (pos + size + PAGE_SIZE - 1) >> PAGE_SHIFT;

	io_monitor_record(file, index, last_index - index, IO_MONITOR_TYPE_READ);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
static void trace_android_vh_filemap_map_pages_hook(void *data, struct file *file,
	pgoff_t orig_start_pgoff, pgoff_t last_pgoff, vm_fault_t ret)
{
	io_monitor_record(file, orig_start_pgoff, last_pgoff - orig_start_pgoff + 1,
		IO_MONITOR_TYPE_READ);
}
#else
static void trace_android_vh_filemap_map_pages_hook(void *data, struct file *file,
	pgoff_t orig_start_pgoff, pgoff_t first_pgoff, pgoff_t last_pgoff, vm_fault_t ret)
{
	io_monitor_record(file, orig_start_pgoff, last_pgoff - orig_start_pgoff + 1,
		IO_MONITOR_TYPE_READ);
}
#endif

static void trace_android_vh_do_read_fault_hook(void *data,
	struct vm_fault *vmf, unsigned long fault_around_bytes)
{
	if (!vmf)
		return;

	io_monitor_record(vmf->vma->vm_file, vmf->pgoff, 1, IO_MONITOR_TYPE_READ);
}

static void io_monitor_hook_register(void)
{
	register_trace_android_vh_filemap_read(trace_android_vh_filemap_read_hook, NULL);
	register_trace_android_vh_filemap_map_pages(trace_android_vh_filemap_map_pages_hook, NULL);
	register_trace_android_vh_do_read_fault(trace_android_vh_do_read_fault_hook, NULL);
}

static void io_monitor_hook_unregister(void)
{
	unregister_trace_android_vh_filemap_read(trace_android_vh_filemap_read_hook, NULL);
	unregister_trace_android_vh_filemap_map_pages(trace_android_vh_filemap_map_pages_hook, NULL);
	unregister_trace_android_vh_do_read_fault(trace_android_vh_do_read_fault_hook, NULL);
}

typedef enum {
	FRR_OK = 0,
	FRR_EINVAL_PARA = EINVAL,
	FRR_EINVAL_TASK = EINVAL,
	FRR_EINVAL_USERBUF = EFAULT,
	FRR_ESTATUS = EIO,
	FRR_EMONITOR_NOT_INIT = ENODEV,
} frr_errno_t;

static ssize_t oplus_frr_write(struct file *file, const char __user *buf,
				size_t count, loff_t *off)
{
	char buffer[FRR_PROC_CMD_BUF_MAX] = {0};
	int status = 0;
	int rv;
	struct task_struct *task;
	bool ret;
	pid_t pid = -1;
	pid_t tgid = -1;
	char *tmp_buffer;
	char *tmp_strsep_buffer;

	if (count > sizeof(buffer) - 1)
		return -FRR_EINVAL_PARA;

	if (copy_from_user(buffer, buf, count))
		return -FRR_EINVAL_USERBUF;

	tmp_buffer = buffer;
	tmp_strsep_buffer = strsep(&tmp_buffer, " ");
	if (!tmp_strsep_buffer) {
		pr_err("monitor type error, strsep fail\n");
		return -FRR_EINVAL_PARA;
	}
	rv = kstrtoint(tmp_strsep_buffer, 10, &status);
	if (rv < 0) {
		pr_err("monitor type error, kstrtoint fail\n");
		return -FRR_EINVAL_PARA;
	}

	tmp_strsep_buffer = strsep(&tmp_buffer, " ");
	if (!tmp_strsep_buffer) {
		if (status != IO_MONITOR_STATUS_RESET) {
			pr_err("monitor pid error, strsep fail, parse status:%d\n", status);
			return -FRR_EINVAL_PARA;
		}
	} else {
		rv = kstrtoint(tmp_strsep_buffer, 10, &pid);
		if (rv < 0) {
			pr_err("monitor pid error, kstrtoint fail, parse status:%d\n", status);
			return -FRR_EINVAL_PARA;
		}
	}

	if (status != IO_MONITOR_STATUS_RESET) {
		rcu_read_lock();
		task = find_task_by_vpid(pid);
		if (!task) {
			rcu_read_unlock();
			pr_err("monitor task is null by pid %d, status:%d\n", pid, status);
			return -FRR_EINVAL_TASK;
		}

		get_task_struct(task);
		rcu_read_unlock();
		tgid = task_tgid_nr(task);
	} else {
		tgid = pid;
	}
	ret = io_monitor_status_set((io_monitor_status)status,
							  IO_MONITOR_TYPE_READ,
							  tgid);
	if (status != IO_MONITOR_STATUS_RESET)
		put_task_struct(task);

	return ret ? count : -FRR_ESTATUS;
}

ssize_t oplus_frr_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	io_monitor *monitor = NULL;
	ssize_t  len = 0;

	monitor = get_io_monitor(IO_MONITOR_TYPE_READ);
	spin_lock(&monitor->m_lock);
	if (!monitor || monitor->format_buf == NULL ||
		monitor->status != IO_MONITOR_STATUS_FINISHED) {
		pr_err("io monitor failed, status not ready\n");
		spin_unlock(&monitor->m_lock);
		return -FRR_EMONITOR_NOT_INIT;
	}
	spin_unlock(&monitor->m_lock);

	if (*ppos < 0 || *ppos >= monitor->format_buf_cur) {
		if (*ppos < 0) {
			pr_warn("io monitor failed, Invalid offset %lld\n", *ppos);
			return -FRR_EINVAL_PARA;
		}
		return 0;
	}

	len = min_t(ssize_t, count, monitor->format_buf_cur - *ppos);
	if (len <= 0)
		return 0;

	if (!access_ok(buf, len)) {
		// pr_err_ratelimited("io monitor failed, Invalid user buffer\n");
		pr_err("io monitor failed, Invalid user buffer\n");
		return -FRR_EINVAL_USERBUF;
	}

	if (copy_to_user(buf, monitor->format_buf + *ppos, len)) {
		pr_err("io monitor failed, copy_to_user error\n");
		return -FRR_EINVAL_USERBUF;
	}

	*ppos += len;
	return len;
}

static const struct proc_ops oplus_frr_fops = {
	.proc_read    = oplus_frr_read,
	.proc_write   = oplus_frr_write,
};

#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
static int oplus_frr_debug_show(struct seq_file *m, void *v)
{
	io_monitor *monitor = get_io_monitor(IO_MONITOR_TYPE_READ);
	char *s_pos, *e_pos;
	int line_len;

	seq_printf(m, "===============================================\n");
	seq_printf(m, "monitor status: %d, tgid: %d\n", monitor->status, monitor->tgid);
	seq_printf(m, "io_info nr: %u, in_info max: %u\n", monitor->buf->cur, monitor->buf->max);
	seq_printf(m, "io_info lost: %u, io_info format lost: %u\n",
		monitor->statistics[IO_MONITOR_STATISTICS_LOST],
		monitor->statistics[IO_MONITOR_STATISTICS_RESULT_LOST]);
	seq_printf(m, "format buffer handle time: %u ns, debug buffer handle time: %u ns\n",
		monitor->format_time_ns, monitor->debug_time_ns);
	seq_printf(m, "===============================================\n");
	if (monitor->status != IO_MONITOR_STATUS_FINISHED)
		return 0;
	s_pos = monitor->debug_buf;
	while ((e_pos = strchr(s_pos, '\n')) != NULL) {
		line_len = e_pos - s_pos;
		seq_printf(m, "%.*s\n", line_len, s_pos);
		s_pos = e_pos + 1;
	}
	seq_printf(m, "===============================================\n");
	return 0;
}

static int oplus_frr_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, oplus_frr_debug_show, NULL);
}

static const struct proc_ops oplus_frr_debug_fops = {
	.proc_open    = oplus_frr_debug_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};
#endif

static bool oplus_frr_proc_create(void)
{
	struct proc_dir_entry *pe;
	struct proc_dir_entry *root_dir_entry = proc_mkdir("oplus_mem", NULL);

	pe = proc_create((root_dir_entry ? "oplus_frr" : "oplus_mem/oplus_frr"),
		0666, root_dir_entry, &oplus_frr_fops);
	if (!pe) {
		pr_err("Failed to register oplus_frr proc interface\n");
		return false;
	}
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	pe = proc_create((root_dir_entry ? "oplus_frr_debug" : "oplus_mem/oplus_frr_debug"),
		0666, root_dir_entry, &oplus_frr_debug_fops);
	if (!pe) {
		pr_err("Failed to register oplus_frr_debug proc interface\n");
		return false;
	}
#endif
	return true;
}

static void oplus_frr_proc_remove(void)
{
	remove_proc_entry("oplus_mem/oplus_frr", NULL);
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
	remove_proc_entry("oplus_mem/oplus_frr_debug", NULL);
#endif
}

static bool io_monitor_init(void)
{
	/* file read record */
	g_read_monitor = io_monitor_alloc(IO_MONITOR_TYPE_READ,
		IO_INFO_RECORD_MAX, IO_MONITOR_FORMAT_BUF_MAX);
	if (g_read_monitor == NULL)
		return false;
	return true;
}

#ifndef CONFIG_OPLUS_MM_FRR_KUNIT
static int __init file_read_record_init(void)
#else
static __attribute__((unused)) int file_read_record_init(void)
#endif
{
// #ifdef CONFIG_OPLUS_FEATURE_MM_OSVELTE
	struct config_frr *config = NULL;

	if (oplus_test_mm_feature_disable(COMFD1_FRR)) {
		pr_info("FRR is disabled in cmdline\n");
		return 0;
	}

	config = oplus_read_mm_config(module_name_frr);
	if (!config || !config->enable) {
		pr_info("%s is disabled in dts\n", module_name_frr);
		return 0;
	}
// #endif
	if (!oplus_frr_proc_create())
	   return -1;

	if (!io_monitor_init()) {
		pr_info("File Read Record init fail\n");
		remove_proc_entry("oplus_mem/oplus_frr", NULL);
#ifdef CONFIG_OPLUS_MM_FRR_DEBUG
		remove_proc_entry("oplus_mem/oplus_frr_debug", NULL);
#endif
		return -1;
	}
	io_monitor_hook_register();
	pr_info("File Read Record init ok\n");
	return 0;
}

#ifndef CONFIG_OPLUS_MM_FRR_KUNIT
static void __exit file_read_record_exit(void)
#else
static __attribute__((unused)) void __exit file_read_record_exit(void)
#endif
{
	oplus_frr_proc_remove();
	io_monitor_free(g_read_monitor);
	io_monitor_hook_unregister();
}

#ifndef CONFIG_OPLUS_MM_FRR_KUNIT
module_init(file_read_record_init);
module_exit(file_read_record_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("yuanshuai");
MODULE_DESCRIPTION("File Read Record");
#endif
