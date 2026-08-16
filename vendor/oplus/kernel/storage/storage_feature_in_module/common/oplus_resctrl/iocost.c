// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2023 Oplus. All rights reserved.
*/
#include "resctrl.h"

#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/part_stat.h>
#include <trace/events/block.h>
#include <linux/ctype.h>

#define IOC_DEF_MIN_IOPS 640
#define IOC_DEF_PPM_HIGH 80
#define IOC_DEF_PPM_MID (IOC_DEF_PPM_HIGH >> 1)
#define IOC_DEF_PPM_LOW (IOC_DEF_PPM_HIGH >> 2)

static const char *__bdevname(struct block_device *bdev, char *buf)
{
	struct gendisk *hd = bdev->bd_disk;
	int partno = bdev->bd_partno;

	if (!partno)
		snprintf(buf, BDEVNAME_SIZE, "%s", hd->disk_name);
	else if (isdigit(hd->disk_name[strlen(hd->disk_name)-1]))
		snprintf(buf, BDEVNAME_SIZE, "%sp%d", hd->disk_name, partno);
	else
		snprintf(buf, BDEVNAME_SIZE, "%s%d", hd->disk_name, partno);

	return buf;
}

static unsigned int ioc_ppm_high = 0;
struct iocost_config iocost_config = {0};

static void ioc_dist_update_stat(int rw, u64 on_q_us, u64 rq_wait_us,
				 int segments)
{
	int i = 0;
	int j = 0;
	while (segments > 0) {
		if (i >= (IOC_DIST_SEGMENTS - 1))
			break;
		i++;
		segments = segments >> 1;
	}

	if (rw == RESCTRL_READ) {
		if (ioc_high_read(on_q_us))
			this_cpu_inc(iocost_config.ioc_dist_stat->rw[rw].high);
	}
	else {
		if (ioc_high_write(on_q_us))
			this_cpu_inc(iocost_config.ioc_dist_stat->rw[rw].high);
	}

	while (on_q_us > 0) {
		if (j >= (IOC_DIST_TIMESTAT - 1))
			break;
		j++;
		on_q_us = on_q_us >> 1;
	}

	this_cpu_inc(iocost_config.ioc_dist_stat->rw[rw].dist[i][j]);
}

void ioc_dist_clear_stat(void)
{
	int cpu;

	for_each_online_cpu(cpu) {
		struct ioc_dist_stat *this = per_cpu_ptr(iocost_config.ioc_dist_stat, cpu);
		memset(this, 0, sizeof(struct ioc_dist_stat));
	}
}

void ioc_dist_get_stat(int rw, u64 *request, int time_dist)
{
	int cpu;
	int i;

	for_each_online_cpu(cpu) {
		struct ioc_dist_stat *this = per_cpu_ptr(iocost_config.ioc_dist_stat, cpu);

		for (i = 0; i < IOC_DIST_SEGMENTS; i++)
			request[i] += this->rw[rw].dist[i][time_dist];
	}
}

void android_vh_blk_account_io_done_handler(void *unused, struct request *rq)
{
	unsigned int segments;
	int rw = RESCTRL_READ;
	u64 on_q_ns = ktime_get_ns() - rq->start_time_ns;
	u64 rq_wait_ns = 0;
	segments = blk_rq_stats_sectors(rq) >> 3;

	switch (req_op(rq) & REQ_OP_MASK) {
	case REQ_OP_READ:
		rw = RESCTRL_READ;
		break;
	case REQ_OP_WRITE:
		rw = RESCTRL_WRITE;
		break;
	default:
		resctrl_debug("resctrl android_vh_blk_account_io_done_handler op=%d, error no support!",
					req_op(rq));
		return;
	}

	/* update the stat */
	ioc_dist_update_stat(rw, on_q_ns / 1000, rq_wait_ns / 1000, segments);
	return;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
void android_vh_block_rq_complete_handler(void *unused, struct request *rq,
                      blk_status_t error, unsigned int nr_bytes)
#else
void android_vh_block_rq_complete_handler(void *unused, struct request *rq,
                      int error, unsigned int nr_bytes)
#endif
{
	unsigned int segments;
	int rw = RESCTRL_READ;
	u64 on_q_ns = ktime_get_ns() - rq->start_time_ns;
	u64 rq_wait_ns = 0;
	segments = (blk_rq_stats_sectors(rq) == 0 ? blk_rq_sectors(rq): blk_rq_stats_sectors(rq)) >> 3;

	if (error || !nr_bytes || !iocost_config.module_init || !iocost_config.feature_enable)
		return;

	/*update the userdata disk block_device*/
	if(rq->part && rq->part->bd_disk && iocost_config.part != rq->part) {
		char buffer[DISK_NAME_LEN] = {0, };
		if (strcmp(__bdevname(rq->part, buffer), iocost_config.part_name) == 0) {
			iocost_config.part = rq->part;
		}
		else {
			return;
		}
	}

	switch (req_op(rq) & REQ_OP_MASK) {
	case REQ_OP_READ:
		rw = RESCTRL_READ;
		break;
	case REQ_OP_WRITE:
		rw = RESCTRL_WRITE;
		break;
	default:
		return;
	}
	/* update the stat */
	ioc_dist_update_stat(rw, on_q_ns / 1000, rq_wait_ns / 1000, segments);

	return;
}

static void part_stat_read_all(struct block_device *part,
		struct disk_stats *stat)
{
	int cpu;

	memset(stat, 0, sizeof(struct disk_stats));
	for_each_possible_cpu(cpu) {
		struct disk_stats *ptr = per_cpu_ptr(part->bd_stats, cpu);
		int group;

		for (group = 0; group < NR_STAT_GROUPS; group++) {
			stat->nsecs[group] += ptr->nsecs[group];
			stat->sectors[group] += ptr->sectors[group];
			stat->ios[group] += ptr->ios[group];
			stat->merges[group] += ptr->merges[group];
		}

		stat->io_ticks += ptr->io_ticks;
	}
}

static void ioc_userdata_bps_calc(void)
{
	struct block_device *hd = iocost_config.part;
	struct disk_stats stat;
	u64 diff_sector;
	u64 diff_nsecs;
	u64 current_time_ns;

	if (!hd)
		return;

	current_time_ns = ktime_get_ns();
	part_stat_read_all(hd, &stat);


	if (iocost_config.nsecs[STAT_READ]) {
		/*update read bps*/
		diff_sector = stat.sectors[STAT_READ] - iocost_config.sectors[RESCTRL_READ];
		diff_nsecs = current_time_ns - iocost_config.nsecs[RESCTRL_READ];

		/*latest bsp ,max bsp, avg bps*/
		iocost_config.latest_bps[RESCTRL_READ] = div64_u64(diff_sector * 512, diff_nsecs/1000/1000);
		if (iocost_config.max_bps[RESCTRL_READ] < iocost_config.latest_bps[RESCTRL_READ])
			iocost_config.max_bps[RESCTRL_READ] = iocost_config.latest_bps[RESCTRL_READ];

		iocost_config.avg_bps[RESCTRL_READ] =
			div64_u64(stat.sectors[STAT_READ] * 512, stat.nsecs[STAT_READ]/1000/1000);
	}
	if (iocost_config.nsecs[STAT_WRITE]) {
		/*upadte write bps*/
		diff_sector = stat.sectors[STAT_WRITE] - iocost_config.sectors[RESCTRL_WRITE];
		diff_nsecs = current_time_ns - iocost_config.nsecs[RESCTRL_WRITE];

		/*latest bsp ,max bsp, avg bps*/
		iocost_config.latest_bps[RESCTRL_WRITE] = div64_u64(diff_sector * 512, diff_nsecs/1000/1000);
		if (iocost_config.max_bps[RESCTRL_WRITE] < iocost_config.latest_bps[RESCTRL_WRITE])
			iocost_config.max_bps[RESCTRL_WRITE] = iocost_config.latest_bps[RESCTRL_WRITE];

		iocost_config.avg_bps[RESCTRL_WRITE] =
			div64_u64(stat.sectors[STAT_WRITE] * 512, stat.nsecs[STAT_WRITE]/1000/1000);
	}

	iocost_config.sectors[RESCTRL_READ] = stat.sectors[STAT_READ];
	iocost_config.sectors[RESCTRL_WRITE] = stat.sectors[STAT_WRITE];

	iocost_config.nsecs[RESCTRL_READ] = current_time_ns;
	iocost_config.nsecs[RESCTRL_WRITE] = current_time_ns;

	/*calc Xsec max avg bps*/
	if (iocost_config.xsec_cnt == 0) {
		iocost_config.xsec_sectors[RESCTRL_READ] = stat.sectors[STAT_READ];
		iocost_config.xsec_sectors[RESCTRL_WRITE] = stat.sectors[STAT_WRITE];

		iocost_config.xsec_nsecs[RESCTRL_READ] = current_time_ns;
		iocost_config.xsec_nsecs[RESCTRL_WRITE] = current_time_ns;

	} else if (iocost_config.xsec_cnt == iocost_config.ioc_loop_xsecs) {
		/*update read bps*/
		diff_sector = stat.sectors[STAT_READ] - iocost_config.xsec_sectors[RESCTRL_READ];
		diff_nsecs = current_time_ns - iocost_config.xsec_nsecs[RESCTRL_READ];
		iocost_config.xsec_avg_bps[RESCTRL_READ] =
			div64_u64(diff_sector * 512, diff_nsecs/1000/1000);

		diff_sector = stat.sectors[STAT_WRITE] - iocost_config.xsec_sectors[RESCTRL_WRITE];
		diff_nsecs = current_time_ns - iocost_config.xsec_nsecs[RESCTRL_WRITE];
		iocost_config.xsec_avg_bps[RESCTRL_WRITE] =
			div64_u64(diff_sector * 512, diff_nsecs/1000/1000);


	} else if (iocost_config.xsec_cnt == 1) {
		/*update read bps*/
		diff_sector = stat.sectors[STAT_READ] - iocost_config.xsec_sectors[RESCTRL_READ];
		diff_nsecs = current_time_ns - iocost_config.xsec_nsecs[RESCTRL_READ];
		iocost_config.xsec_max_bps[RESCTRL_READ] =
			div64_u64(diff_sector * 512, diff_nsecs/1000/1000);

		diff_sector = stat.sectors[STAT_WRITE] - iocost_config.xsec_sectors[RESCTRL_WRITE];
		diff_nsecs = current_time_ns - iocost_config.xsec_nsecs[RESCTRL_WRITE];

		iocost_config.xsec_max_bps[RESCTRL_WRITE] =
			div64_u64(diff_sector * 512, diff_nsecs/1000/1000);

	} else {
		if (iocost_config.xsec_max_bps[RESCTRL_READ] < iocost_config.latest_bps[RESCTRL_READ])
			iocost_config.xsec_max_bps[RESCTRL_READ] = iocost_config.latest_bps[RESCTRL_READ];

		if (iocost_config.xsec_max_bps[RESCTRL_WRITE] < iocost_config.latest_bps[RESCTRL_WRITE])
			iocost_config.xsec_max_bps[RESCTRL_WRITE] = iocost_config.latest_bps[RESCTRL_WRITE];
	}

	resctrl_printk(RESCTRL_LOG_LEVEL_DEBUG, __func__,
        "resctrl  userdata_bps of read: %llu write: %llu! nsecs:%llu %llu sectors:%lu %lu",
		    iocost_config.latest_bps[0], iocost_config.latest_bps[1],
			    iocost_config.nsecs[0], iocost_config.nsecs[1],
			        iocost_config.sectors[0], iocost_config.sectors[1]);
}

static void ioc_lat_calc(u64 *nr_high, u64 *nr_all)
{
	int cpu, i, j, rw;

	for_each_online_cpu(cpu) {
		struct ioc_dist_stat *this = per_cpu_ptr(iocost_config.ioc_dist_stat, cpu);

		for (rw = RESCTRL_READ; rw <= RESCTRL_WRITE; rw++) {
			u64 this_all = 0;
			u64 this_high = this->rw[rw].high;

			for (i = 0; i < IOC_DIST_SEGMENTS; i++) {
				for (j = 0; j < IOC_DIST_TIMESTAT; j++) {
					this_all += this->rw[rw].dist[i][j];
				}
			}

			nr_all[rw] += this_all - this->rw[rw].last_sum;
			nr_high[rw] += this_high - this->rw[rw].last_high;

			this->rw[rw].last_sum = this_all;
			this->rw[rw].last_high = this_high;
		}
	}
}

#define IOC_READ_PPM_MASK_LOW 		0x01
#define IOC_READ_PPM_MASK_MID 		0x02
#define IOC_READ_PPM_MASK_HIGH	 	0x04
#define IOC_WRITE_PPM_MASK_LOW 		0x10
#define IOC_WRITE_PPM_MASK_MID 		0x20
#define IOC_WRITE_PPM_MASK_HIGH	 	0x40

void ioc_timer_fn(struct timer_list *timer)
{
	u64 nr_high[2] = {0};
	u64 nr_all[2] = {0};

        if (!iocost_config.feature_enable)
                return;

	ioc_userdata_bps_calc();
	ioc_lat_calc(nr_high, nr_all);

        resctrl_printk(RESCTRL_LOG_LEVEL_DEBUG, __func__,
            "### nr_high=%llu, nr_all=%llu ppm=%d\n",
                nr_high[0], nr_all[0], ioc_ppm_high);
	if (nr_all[RESCTRL_READ] >= IOC_DEF_MIN_IOPS) {
		if ((nr_high[RESCTRL_READ]*100/nr_all[RESCTRL_READ])  >= IOC_DEF_PPM_HIGH) {
			ioc_ppm_high |=  IOC_READ_PPM_MASK_HIGH;
		} else if ((nr_high[RESCTRL_READ] * 100 /nr_all[RESCTRL_READ])  >= IOC_DEF_PPM_MID) {
			ioc_ppm_high |= IOC_READ_PPM_MASK_MID;
		}
		else if ((nr_high[RESCTRL_READ] * 100 /nr_all[RESCTRL_READ])  >= IOC_DEF_PPM_LOW) {
			ioc_ppm_high |= IOC_READ_PPM_MASK_LOW;
		} else {
			ioc_ppm_high &= ~0x0F;
		}
	} else {
		if (ioc_ppm_high != 0) {
			ioc_ppm_high &= ~0x0F;
		}
	}

	if (nr_all[RESCTRL_WRITE] >= IOC_DEF_MIN_IOPS) {
		if ((nr_high[RESCTRL_WRITE]*100/nr_all[RESCTRL_WRITE])  >= IOC_DEF_PPM_HIGH) {
			ioc_ppm_high |= IOC_WRITE_PPM_MASK_HIGH;
		} else if ((nr_high[RESCTRL_WRITE] * 100 /nr_all[RESCTRL_WRITE])  >= IOC_DEF_PPM_MID) {
			ioc_ppm_high |= IOC_WRITE_PPM_MASK_MID;
		}
		else if ((nr_high[RESCTRL_WRITE] * 100 /nr_all[RESCTRL_WRITE])  >= IOC_DEF_PPM_LOW) {
			ioc_ppm_high |= IOC_WRITE_PPM_MASK_LOW;
		} else {
			ioc_ppm_high &= ~0xF0;
		}
	} else {
		ioc_ppm_high &= ~0xF0;
	}

	if (iocost_config.ppm_start) {
		if (iocost_config.xsec_cnt < iocost_config.ioc_loop_xsecs)
			iocost_config.xsec_cnt++;
		else
			iocost_config.xsec_cnt = 0;
		iocost_config.timer.expires = jiffies + 1*HZ;
		add_timer(&iocost_config.timer);
	}
        resctrl_printk(RESCTRL_LOG_LEVEL_DEBUG, __func__,
            "### nr_high=%llu, nr_all=%llu ppm=%d\n",
                nr_high[0], nr_all[0], ioc_ppm_high);
}

int ioc_latest_bps_read(char *page)
{
	int len;

	len = sprintf(page, "%llu,%llu\n",
			iocost_config.latest_bps[RESCTRL_READ], iocost_config.latest_bps[RESCTRL_WRITE]);
	return len;
}

int ioc_max_bps_read(char *page)
{
	int len;

	len = sprintf(page, "%llu,%llu\n",
			iocost_config.max_bps[RESCTRL_READ], iocost_config.max_bps[RESCTRL_WRITE]);
	return len;
}

int ioc_avg_bps_read(char *page)
{
	int len;

	len = sprintf(page, "%llu,%llu\n",
			iocost_config.avg_bps[RESCTRL_READ], iocost_config.avg_bps[RESCTRL_WRITE]);
	return len;
}

int ioc_xsecs_max_bps_read(char *page)
{
	int len;

	len = sprintf(page, "%llu,%llu\n",
			iocost_config.xsec_max_bps[RESCTRL_READ], iocost_config.xsec_max_bps[RESCTRL_WRITE]);
	return len;
}

int ioc_xsecs_avg_bps_read(char *page)
{
	int len;

	len = sprintf(page, "%llu,%llu\n",
			iocost_config.xsec_avg_bps[RESCTRL_READ], iocost_config.xsec_avg_bps[RESCTRL_WRITE]);
	return len;
}

int ioc_iop_read(void)
{
	return ioc_ppm_high;
}

struct {
	const char *name;
	void *callback;
	struct tracepoint *tp;
	void *data;
} tracepoint_probes[] = {
	{"block_rq_complete", android_vh_block_rq_complete_handler, NULL, NULL},
	{NULL, NULL, NULL, NULL}
};

void ioc_register_tracepoint_probes(void)
{
	int ret;

	ret = register_trace_block_rq_complete(android_vh_block_rq_complete_handler, NULL);
	return;
}

void ioc_unregister_tracepoint_probes(void)
{
	unregister_trace_block_rq_complete(android_vh_block_rq_complete_handler, NULL);

	return;
}

