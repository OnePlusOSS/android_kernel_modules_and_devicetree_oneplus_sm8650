// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2023 Oplus. All rights reserved.
*/
#ifndef _RESCTRL_H_
#define _RESCTRL_H_
#include <linux/compiler_types.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/blkdev.h>
#include <linux/version.h>
#include <linux/part_stat.h>


#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a, b, c)                                                \
	(((a) << 16) + ((b) << 8) + ((c) > 255 ? 255 : (c)))
#endif

#ifndef LINUX_VERSION_CODE
#error "Need LINUX_VERSION_CODE"
#endif

#define LINUX_KERNEL_606                                                    \
	((LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)) &&                   \
	 (LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 256)))

#define LINUX_KERNEL_601                                                       \
	((LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) &&                   \
	 (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 256)))
#define LINUX_KERNEL_515                                                       \
	((LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)) &&                   \
	 (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 256)))
#define LINUX_KERNEL_510                                                       \
	((LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)) &&                   \
	 (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 256)))
#define LINUX_KERNEL_504                                                       \
	((LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)) &&                    \
	 (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 256)))
#define LINUX_KERNEL_419                                                       \
	((LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)) &&                   \
	 (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 256)))

#if (!LINUX_KERNEL_601 && !LINUX_KERNEL_606 && !LINUX_KERNEL_515 && !LINUX_KERNEL_510 && !LINUX_KERNEL_419 &&            \
	 !LINUX_KERNEL_504)
#error "Kernel version not supported"
#endif

enum {
	RESCTRL_LOG_LEVEL_ERROR,
	RESCTRL_LOG_LEVEL_WARN,
	RESCTRL_LOG_LEVEL_NOTICE,
	RESCTRL_LOG_LEVEL_INFO,
	RESCTRL_LOG_LEVEL_DEBUG,
	RESCTRL_LOG_LEVEL_TEMP,
};

/*
 * we know min is 4K and max is 512K
 * but we still keep the 1 and other col
 */
#define IOC_DIST_TIMESTAT 21
#define IOC_DIST_SEGMENTS 10

#define RESCTRL_READ 0
#define RESCTRL_WRITE 1
#define RESCTRL_OP_MAX 2


struct ioc_dist_rw {
	u64 dist[IOC_DIST_SEGMENTS][IOC_DIST_TIMESTAT];
	u64 high;
	u64 last_sum;
	u64 last_high;
};

struct ioc_dist_stat {
	struct ioc_dist_rw rw[2];
};

/*this winow  for cal nSecs avg|max bps*/
#define IOC_LOOP_XSECS_WINDOW 60
struct iocost_config {
	bool module_init;
	bool feature_enable;
	bool debug_enable;
	struct timer_list timer;
	u32 read_lat;
	u32 write_lat;
	u32 ppm_start;
	struct block_device *part;
	unsigned long sectors[RESCTRL_OP_MAX];
	u64 nsecs[RESCTRL_OP_MAX];
	u64 latest_bps[RESCTRL_OP_MAX];
	u64 max_bps[RESCTRL_OP_MAX];
	u64 avg_bps[RESCTRL_OP_MAX];
	u32 xsec_cnt;
	unsigned long xsec_sectors[RESCTRL_OP_MAX];
	u64 xsec_nsecs[RESCTRL_OP_MAX];
	u64 xsec_avg_bps[RESCTRL_OP_MAX];
	u64 xsec_max_bps[RESCTRL_OP_MAX];

	char part_name[DISK_NAME_LEN];
	u32 ioc_loop_xsecs;
	struct ioc_dist_stat __percpu *ioc_dist_stat;
};

extern struct iocost_config iocost_config;

__printf(3, 4) void resctrl_printk(int log_level, const char *func,
				   const char *fmt, ...);

#define resctrl_err(fmt, ...)                                                 \
	printk_ratelimited(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#define resctrl_warn(fmt, ...)                                                 \
	printk_ratelimited(KERN_WARNING pr_fmt(fmt), ##__VA_ARGS__)
#ifdef CONFIG_RESCTRL_DEBUG
#define resctrl_notice(fmt, ...)                                               \
	printk_ratelimited(KERN_NOTICE pr_fmt(fmt), ##__VA_ARGS__)
#define resctrl_info(fmt, ...)                                                 \
	printk_ratelimited(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#define resctrl_debug(fmt, ...)                                                \
	printk_ratelimited(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#define resctrl_temp(fmt, ...)                                                 \
	resctrl_printk(RESCTRL_LOG_LEVEL_TEMP, __func__, KERN_DEBUG fmt,       \
		       ##__VA_ARGS__)
#else
#define resctrl_notice(sbi, fmt, ...)                                          \
	do {                                                                   \
	} while (0)
#define resctrl_info(sbi, fmt, ...)                                            \
	do {                                                                   \
	} while (0)
#define resctrl_debug(sbi, fmt, ...)                                           \
	do {                                                                   \
	} while (0)
#define resctrl_temp(sbi, fmt, ...)                                            \
	do {                                                                   \
	} while (0)
#endif

#define REGISTER_TRACE_VH(vender_hook, handler)                                \
{                                                                      \
	ret = register_trace_##vender_hook(handler, NULL);             \
	if (ret) {                                                     \
		pr_info("failed to register_" #vender_hook             \
			", ret=%d\n",                                  \
			ret);                                          \
		return ret;                                            \
	}                                                              \
}

#define UNREGISTER_TRACE_VH(vender_hook, handler)                              \
{                                                                      \
	ret = unregister_trace_##vender_hook(handler, NULL);           \
	pr_info("%s handler register success", #handler);              \
}

static inline ssize_t sched_data_to_user(char __user *buff, size_t count,
					 loff_t *off, char *format_str, int len)
{
	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, format_str, (len < count ? len : count)))
		return -EFAULT;
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

void android_vh_blk_account_io_done_handler(void *unused, struct request *rq);

int  ioc_iop_read(void);
void ioc_dist_clear_stat(void);
void ioc_dist_get_stat(int rw, u64 *request, int time_dist);
void ioc_timer_fn(struct timer_list *timer);
int ioc_latest_bps_read(char *page);
int ioc_max_bps_read(char *page);
int ioc_avg_bps_read(char *page);
int ioc_xsecs_max_bps_read(char *page);
int ioc_xsecs_avg_bps_read(char *page);



void ioc_register_tracepoint_probes(void);
void ioc_unregister_tracepoint_probes(void);


static inline bool ioc_high_read(u32 value) {
	if (iocost_config.read_lat == 0)
		return false;

	if (value > iocost_config.read_lat)
		return true;
	return false;
}

static inline bool ioc_high_write(u32 value) {
	if (iocost_config.write_lat == 0)
		return false;

	if (value > iocost_config.write_lat)
		return true;
	return false;
}

#endif /* _RESCTRL_H_ */
