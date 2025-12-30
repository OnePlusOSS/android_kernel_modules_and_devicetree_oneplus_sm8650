// SPDX-License-Identifier: GPL-2.0
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/perf_event.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched/mm.h>
#include <linux/freezer.h>
#include <linux/ktime.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/swap.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/tick.h>
#include <linux/kernel_stat.h>
#include <linux/energy_model.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/sched/cputime.h>
#include <dvfsrc-exp.h>
#include "../../../drivers/gpu/mediatek/gpufreq/v2/include/gpufreq_v2.h"
#include <soc/mediatek/dramc.h>
#include <slbc_ipi.h>
#include <slbc.h>
#include "../../../drivers/misc/mediatek/qos/mtk_qos_sram.h"
#include "../../../drivers/misc/mediatek/qos/mtk_qos_share.h"
#include "../../../drivers/misc/mediatek/qos/mtk_qos_ipi.h"

#include "osml.h"

/* in dvfsrc/dvfsrc-helper.c */
extern int osml_get_dvfsrc_sw_bw(int idx);

#define MAX_REPORT_SIZE 80000
#define MAX_PEVENT_SIZE 16
#define MAX_STRING_LEN 50
#define OSML_MAX_CLUSTER 3
#define NSEC_PER_USEC 1000L
#define NSEC_TO_USEC(val) (val / NSEC_PER_USEC)
#define DEADLINE_MS 1000
#define DEFAULT_USLEEP_TIME 5000
#define DEFAULT_PEVENT_SIZE 7
#define DEFAULT_NR_CORES 8

#define PROC_OSML_SETTINGS_DIR "osml_settings"

#define OSML_CTL_NODE "osml_ctl"
#define OSML_IOC_MAGIC 'k'
#define OSML_IOC_COLLECT _IOWR(OSML_IOC_MAGIC, 0, struct osml_parcel)

#define OSML_PROC_WRITE(_name) \
static ssize_t _name##_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) \
{ \
	int val; \
	char page[32] = {0}; \
	if (simple_write_to_buffer(page, sizeof(page), ppos, buf, count) <= 0) \
		return -EINVAL; \
	if (sscanf(page, "%d\n", &val) <= 0) { \
		pr_err("error setting argument. argument should be positive\n"); \
		return -EINVAL; \
	} \
	_name = val; \
	return count; \
} \

#define OSML_PROC_READ(_name) \
static ssize_t _name##_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) \
{ \
	char page[32] = {0}; \
	int len; \
	len = sprintf(page, "%d\n", _name); \
	return simple_read_from_buffer(buf, count, ppos, page, len); \
} \

#define OSML_ARRAY_PROC_READ(_name, _len) \
static ssize_t _name##_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) \
{ \
	char page[PAGE_SIZE] = {0}; \
	int len = 0, idx; \
	for (idx = 0; idx < _len - 1; idx++) \
		len += sprintf(page + len, "%d, ", _name[idx]); \
	len += sprintf(page + len, "%d\n", _name[idx]); \
	return simple_read_from_buffer(buf, count, ppos, page, len); \
} \

#define OSML_RW_PROC_OPS(_name) \
static const struct proc_ops _name##_proc_ops = { \
	.proc_write		= _name##_proc_write, \
	.proc_read		= _name##_proc_read, \
	.proc_lseek		= default_llseek, \
}; \


enum {
	OSML_EVENT,
	IOC_EVENT,
	MAX_LIST_SIZE,
};
/* event_base 0 ~ 9 */
enum event_base {
	TS,
	TS_DELTA,
	FRAME_NUMBER,
	FRAME_OWNER,
	FRAME_DURATION,
	FRAME_ALLOW_DURATION,
	FRAME_VSYNC,
	MAX_EVENT_BASE, // 7
};
/* event_cpu 10 ~ 49 */
enum event_cpu {
	CPU_LOAD_0 = 10,
	CPU_LOAD_1,
	CPU_LOAD_2,
	CPU_LOAD_3,
	CPU_LOAD_4,
	CPU_LOAD_5,
	CPU_LOAD_6,
	CPU_LOAD_7,
	CPU_FREQ_CLUSTER_0,
	CPU_FREQ_CLUSTER_1,
	CPU_FREQ_CLUSTER_2, // 20
	CPU_SCALING_MAX_FREQ_0,
	CPU_SCALING_MIN_FREQ_0,
	CPU_SCALING_MAX_FREQ_1,
	CPU_SCALING_MIN_FREQ_1,
	CPU_SCALING_MAX_FREQ_2,
	CPU_SCALING_MIN_FREQ_2,
	L3_FREQ,
	MAX_EVENT_CPU, // 28
};
/* event_mem 50 ~ 89 */
enum event_mem {
	DDR_FREQ = 50,
	DVFSRC_CURR_DVFS_OPP,
	DVFSRC_CURR_VCORE_UV,
	DVFSRC_CURR_VCORE_OPP,
	DVFSRC_SW_REQ_VCORE_OPP,
	DVFSRC_SW_BW,
	EMIBW_MON,
	EMI_OPY_BW,
	EMI_TP,
	DRAMC_OPY_BW,
	MEM_BW, // 60
	MEM_CALC_BW,
	LLCC_FREQ,
	DDR_BW,
	LLCC_BW,
	LLCC_CPU_CAPACITY,
	LLCC_GPU_CAPACITY,
	LLCC_USAGE,
	LLCC_CPU_RD_HIT_RATE,
	LLCC_GPU_RD_HIT_RATE,
	LLCC_CPU_RD_HIT_BW,
	LLCC_GPU_RD_HIT_BW,
	MAX_EVENT_MEM, // 72
};
/* event_gpu 90 ~ 129 */
enum event_gpu {
	GPU_FREQ = 90,
	GED_GPU_FREQ,
	MAX_EVENT_GPU,
};
/* event_other 130 ~ 150 */
enum event_other {
	CUSTOM_PEVENT = 130,
	MAX_EVENT_OTHER,
};
#define MONITOR_SIZE 150

struct perf_data {
	struct perf_event *pevent;
	unsigned long prev_count;
	unsigned long last_delta;
};

struct event_data {
	int uid;
	int buf_idx;
	int event_idx;
	int pdata_cnt;
	struct perf_data *pdata;
	char *title;
	struct list_head osml_event_node;
};

struct osml_monitor {
	int event_size;
	long long *buf;
};

struct osml_cpuinfo {
	u64 clus_id;
	u64 num_cpu;
	u64 *pwr_tbl;
	u64 max_state;
};

struct osml_parcel {
	u64 pid;
	u64 pevent_val[MAX_PEVENT_SIZE];
};

static int nr_cores = 8;
static unsigned int record_cnt;
struct task_struct *osml_polling_tsk;
static struct workqueue_struct *osml_workq;
static char *monitor_case[MONITOR_SIZE] = {
	// event_base 0 ~ 9
	"timestamp", "timestamp_delta", "frame_number", "frame_owner", "frame_duration",
	"allow_frame_duration", "Vsync", "", "", "",
	// event_cpu 10 ~ 49
	"cpu_load_0", "cpu_load_1", "cpu_load_2", "cpu_load_3", "cpu_load_4",
	"cpu_load_5", "cpu_load_6", "cpu_load_7", "cpu_freq_cluster_0", "cpu_freq_cluster_1",
	"cpu_freq_cluster_2", "scaling_max_freq_0", "scaling_min_freq_0", "scaling_max_freq_1", "scaling_min_freq_1",
	"scaling_max_freq_2", "scaling_min_freq_2", "l3_freq", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	// event_mem 50 ~ 89
	"ddr_freq", "dvfsrc_curr_dvfs_opp", "dvfsrc_curr_vcore_uv", "dvfsrc_curr_vcore_opp", "dvfsrc_sw_req_vcore_opp",
	"dvfsrc_sw_bw", "emibw_mon", "emi_opy_bw", "emi_tp", "dramc_opy_bw",
	"mem_bw", "mem_calc_bw", "LLCC_freq", "ddr_bw", "LLCC_bw",
	"llcc_cpu_capacity", "llcc_gpu_capacity", "llcc_usage", "llcc_cpu_rd_hit_rate", "llcc_gpu_rd_hit_rate",
	"llcc_cpu_rd_hit_bw", "llcc_gpu_rd_hit_bw", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	// event_gpu 90 ~ 129
	"gpu_freq", "ged_gpu_freq", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	// event_ohter 130 ~ 149
	"custom_pevent", "", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
	"", "", "", "", "",
};
struct osml_monitor pmonitor = {
	.buf = NULL,
};
static struct osml_cpuinfo cpu_clus[OSML_MAX_CLUSTER] = {
	{-1, 0, NULL, -1},
	{-1, 0, NULL, -1},
	{-1, 0, NULL, -1},
};

static int fg_pid;
static atomic_t frame_number = ATOMIC_INIT(-1);
static atomic_t frame_owner = ATOMIC_INIT(0);
static atomic_t frame_duration = ATOMIC_INIT(0);
static atomic_t vsync_period = ATOMIC_INIT(0);

static int osml_debug;
static int osml_enable;
static int osml_pid;
static int osml_sample_rate = 1000000;
static int sample_type = 1; // 0:frame trigger, 1:polling
static int sample_dimension; // 0:system-wide, 1:task-wide
static int sample_events[MONITOR_SIZE];
static int sample_events_cnt = MONITOR_SIZE;
static int monitor_type;

static char pevent_name[MAX_PEVENT_SIZE][MAX_STRING_LEN] = {
	"raw-inst-retired", "raw-cpu-cycles", "raw-ll-cache-rd", "raw-ll-cache-miss-rd",
	"raw-stall", "raw-stall-backend-mem", "raw-l3d-cache-refill",
};
static int pevent_id[MAX_PEVENT_SIZE] = {
	0x8, 0x11, 0x36, 0x37,
	0x3c, 0x4005, 0x2a,
};
static int pevent_type[MAX_PEVENT_SIZE] = {
	PERF_TYPE_RAW, PERF_TYPE_RAW, PERF_TYPE_RAW, PERF_TYPE_RAW,
	PERF_TYPE_RAW, PERF_TYPE_RAW, PERF_TYPE_RAW,
};
static int pevent_cnt = DEFAULT_PEVENT_SIZE;

enum {
	LLCC_CPU_USAGE,
	LLCC_GPU_USAGE,
	LLCC_OTHER_USAGE,
	MAX_USAGE,
};
static char llcc_cache_usage_name[MAX_USAGE][MAX_STRING_LEN] = {
	"llcc_cpu_usage", "llcc_gpu_usage", "llcc_other_usage",
};

enum {
	DVFSRC_APU_SW_BW,
	DVFSRC_CPU_SW_BW,
	DVFSRC_GPU_SW_BW,
	DVFSRC_TOTAL_SW_BW,
	MAX_DVFSRC_SW_BW,
};
#define DVFSRC_MM_SW_BW 3
#define DVFSRC_MD_SW_BW 4
#define MAX_TOTAL_SW_BW_SIZE 5
static char dvfsrc_sw_bw_name[MAX_DVFSRC_SW_BW][MAX_STRING_LEN] = {
	"dvfsrc_apu_sw_bw", "dvfsrc_cpu_sw_bw", "dvfsrc_gpu_sw_bw",
	"dvfsrc_total_sw_bw",
};

enum {
	EMIBW_MON_TOTAL,
	EMIBW_MON_CPU,
	EMIBW_MON_GPU,
	EMIBW_MON_MM,
	EMIBW_MON_MD,
	MAX_EMIBW_MON,
};
static char emibw_mon_name[MAX_EMIBW_MON][MAX_STRING_LEN] = {
	"emibw_mon_total", "emibw_mon_cpu", "emibw_mon_gpu", "emibw_mon_mm", "emibw_mon_md",
};

/* emi occupied bandwidth */
enum {
	EMI_OPY_BW_TOTAL,
	EMI_OPY_BW_CPU,
	EMI_OPY_BW_GPU,
	EMI_OPY_BW_MM,
	MAX_EMI_OPY_BW,
};
static char emi_opy_bw_name[MAX_EMI_OPY_BW][MAX_STRING_LEN] = {
	"emi_opy_bw_total", "emi_opy_bw_cpu", "emi_opy_bw_gpu", "emi_opy_bw_mm",
};
/* emi data bandwidth (throughput) */
enum {
	EMI_TP_TOTAL,
	EMI_TP_CPU,
	EMI_TP_GPU,
	EMI_TP_MM,
	MAX_EMI_TP,
};
static char emi_tp_name[MAX_EMI_TP][MAX_STRING_LEN] = {
	"emi_tp_total", "emi_tp_cpu", "emi_tp_gpu", "emi_tp_mm",
};
/* dramc occupied bandwidth */
enum {
	DRAMC_OPY_BW_TOTAL,
	DRAMC_OPY_BW_CPU,
	DRAMC_OPY_BW_GPU,
	DRAMC_OPY_BW_MM,
	MAX_DRAMC_OPY_BW,
};
static char dramc_opy_bw_name[MAX_DRAMC_OPY_BW][MAX_STRING_LEN] = {
	"dramc_opy_bw_total", "dramc_opy_bw_cpu", "dramc_opy_bw_gpu", "dramc_opy_bw_mm",
};

enum {
	MEM_BW_READ_0,
	MEM_BW_READ_1,
	MEM_BW_WRITE_0,
	MEM_BW_WRITE_1,
	MAX_MEM_BW,
};
static char mem_bw_name[MAX_MEM_BW][MAX_STRING_LEN] = {
	"mem_bw_read_0", "mem_bw_read_1", "mem_bw_write_0", "mem_bw_write_1",
};

/* memory calculating bandwidth (ddr byte count ip)/(active time) */
enum {
	MEM_CALC_BW_READ,
	MEM_CALC_BW_WRITE,
	MEM_CALC_BW_CPU,
	MEM_CALC_BW_GPU,
	MEM_CALC_BW_MM,
	MEM_CALC_BW_OTHERS,
	MAX_MEM_CALC_BW,
};
static char mem_calc_bw_name[MAX_MEM_CALC_BW][MAX_STRING_LEN] = {
	"mem_calc_bw_read", "mem_calc_bw_write", "mem_calc_bw_cpu", "mem_calc_bw_gpu",
	"mem_calc_bw_mm", "mem_calc_bw_others",
};

static struct list_head list_event_head[MAX_LIST_SIZE] = {
	LIST_HEAD_INIT(list_event_head[OSML_EVENT]),
	LIST_HEAD_INIT(list_event_head[IOC_EVENT])
};
static DEFINE_MUTEX(list_mutex_lock);

static dev_t osml_ctl_dev;
static struct class *driver_class;
static struct cdev cdev;
static int cdev_status;
unsigned int data_diving_time;

OSML_PROC_WRITE(osml_debug);
OSML_PROC_READ(osml_debug);
OSML_RW_PROC_OPS(osml_debug);
OSML_PROC_WRITE(sample_type);
OSML_PROC_READ(sample_type);
OSML_RW_PROC_OPS(sample_type);
OSML_PROC_WRITE(sample_dimension);
OSML_PROC_READ(sample_dimension);
OSML_RW_PROC_OPS(sample_dimension);

static ssize_t sample_events_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int val, idx = 0;
	char *token, *page_tmp, *page;

	page_tmp = page = kzalloc(sizeof(char) * PAGE_SIZE, GFP_KERNEL);

	if (simple_write_to_buffer(page, PAGE_SIZE, ppos, buf, count) <= 0) {
		kfree(page_tmp);
		return -EINVAL;
	}

	token = strsep(&page, " ,");
	while (token != NULL && idx < MONITOR_SIZE && idx >= TS) {
		if (token[0] && token[0] != ',') {
			if (kstrtoint(token, 0, &val)) {
				pr_err("error setting argument. argument should be integer\n");
				kfree(page_tmp);
				return -EINVAL;
			}
			sample_events[idx] = val;
			idx++;
		}
		token = strsep(&page, " ,");
	}
	sample_events_cnt = idx;
	kfree(page_tmp);

	return count;
}
OSML_ARRAY_PROC_READ(sample_events, sample_events_cnt);
OSML_RW_PROC_OPS(sample_events);

static ssize_t pevent_name_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int idx = 0;
	char *token, *page_tmp, *page;

	page_tmp = page = kzalloc(sizeof(char) * PAGE_SIZE, GFP_KERNEL);

	if (simple_write_to_buffer(page, PAGE_SIZE, ppos, buf, count) <= 0) {
		kfree(page_tmp);
		return -EINVAL;
	}

	token = strsep(&page, " ,");
	while (token != NULL && idx < MAX_PEVENT_SIZE) {
		if (token[0] && token[0] != ',') {
			strncpy(pevent_name[idx], token, MAX_STRING_LEN);
			idx++;
		}
		token = strsep(&page, " ,");
	}
	kfree(page_tmp);

	return count;
}

static ssize_t pevent_name_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char page[PAGE_SIZE] = {0};
	int len = 0, idx;

	for (idx = 0; idx < pevent_cnt - 1; idx++)
		len += sprintf(page + len, "%s, ", pevent_name[idx]);
	len += sprintf(page + len, "%s\n", pevent_name[idx]);

	return simple_read_from_buffer(buf, count, ppos, page, len);
}
OSML_RW_PROC_OPS(pevent_name);

static ssize_t pevent_id_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int val, idx = 0;
	char *token, *page_tmp, *page;

	page_tmp = page = kzalloc(sizeof(char) * PAGE_SIZE, GFP_KERNEL);

	if (simple_write_to_buffer(page, PAGE_SIZE, ppos, buf, count) <= 0) {
		kfree(page_tmp);
		return -EINVAL;
	}

	token = strsep(&page, " ,");
	while (token != NULL && idx < MAX_PEVENT_SIZE) {
		if (token[0] && token[0] != ',') {
			if (kstrtoint(token, 0, &val)) {
				pr_err("error setting argument. argument should be integer\n");
				kfree(page_tmp);
				return -EINVAL;
			}
			pevent_id[idx] = val;
			idx++;
		}
		token = strsep(&page, " ,");
	}
	pevent_cnt = idx;
	kfree(page_tmp);

	return count;
}
OSML_ARRAY_PROC_READ(pevent_id, pevent_cnt);
OSML_RW_PROC_OPS(pevent_id);

static ssize_t pevent_type_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int val, idx = 0;
	char *token, *page_tmp, *page;

	page_tmp = page = kzalloc(sizeof(char) * PAGE_SIZE, GFP_KERNEL);

	if (simple_write_to_buffer(page, PAGE_SIZE, ppos, buf, count) <= 0) {
		kfree(page_tmp);
		return -EINVAL;
	}

	token = strsep(&page, " ,");
	while (token != NULL && idx < MAX_PEVENT_SIZE) {
		if (token[0] && token[0] != ',') {
			if (kstrtoint(token, 0, &val)) {
				pr_err("error setting argument. argument should be integer\n");
				kfree(page_tmp);
				return -EINVAL;
			}
			pevent_type[idx] = val;
			idx++;
		}
		token = strsep(&page, " ,");
	}
	kfree(page_tmp);

	return count;
}
OSML_ARRAY_PROC_READ(pevent_type, pevent_cnt);
OSML_RW_PROC_OPS(pevent_type);

static int osml_report_proc_show(struct seq_file *m, void *v)
{
	unsigned int local_report_cnt = 0;
	int row, col, idx = 0;
	struct event_data *event;

	if (!pmonitor.buf) {
		seq_printf(m, "sample buffer not init\n");
		return 0;
	}

	local_report_cnt = record_cnt;

	if (!local_report_cnt) {
		seq_printf(m, "no data recorded\n");
		return 0;
	}

	mutex_lock(&list_mutex_lock);
	list_for_each_entry(event, &list_event_head[OSML_EVENT], osml_event_node) {
		seq_printf(m, "%s,", event->title);
	}
	mutex_unlock(&list_mutex_lock);
	seq_printf(m, "\n");

	for (row = 0; row < local_report_cnt; row++) {
		idx = row * pmonitor.event_size;
		for (col = 0; col < pmonitor.event_size; col++)
			seq_printf(m, "%lld,", pmonitor.buf[idx + col]);
		seq_printf(m, "\n");
	}

	return 0;
}

static int osml_report_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, osml_report_proc_show, NULL);
}

static const struct proc_ops osml_report_proc_ops = {
	.proc_open = osml_report_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static DEFINE_MUTEX(enable_mutex_lock);
static ssize_t osml_reset_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int val;
	char page[32] = {0};

	if (simple_write_to_buffer(page, sizeof(page), ppos, buf, count) <= 0)
		return -EINVAL;

	if (sscanf(page, "%d\n", &val) < 0)
		return -EINVAL;

	if (val != 1)
		return -EINVAL;

	mutex_lock(&enable_mutex_lock);
	if (pmonitor.buf)
		memset(pmonitor.buf, 0, sizeof(long long) * pmonitor.event_size * MAX_REPORT_SIZE);
	mutex_unlock(&enable_mutex_lock);

	record_cnt = 0;
	atomic_set(&frame_number, -1);
	fg_pid = 0;

	pr_info("sample data reset\n");

	return count;
}

static const struct proc_ops osml_reset_proc_ops = {
	.proc_write		= osml_reset_proc_write,
	.proc_lseek		= default_llseek,
};

#define MIN_SAMPLE_MS	4U
static ssize_t osml_sample_rate_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned int val;
	char page[32] = {0};

	if (simple_write_to_buffer(page, sizeof(page), ppos, buf, count) <= 0)
		return -EINVAL;

	if (sscanf(page, "%u\n", &val) <= 0) {
		pr_err("error setting argument. argument should be positive\n");
		return -EINVAL;
	}

	val = max(val, MIN_SAMPLE_MS);

	osml_sample_rate = val * 1000;
	return count;
}

static ssize_t osml_sample_rate_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char page[32] = {0};
	int len;

	len = sprintf(page, "%u\n", osml_sample_rate / 1000);
	return simple_read_from_buffer(buf, count, ppos, page, len);
}
OSML_RW_PROC_OPS(osml_sample_rate);

static int get_thread_count(int ioc_pid, int *event_uid)
{
	struct task_struct *task, *p, *t;
	u64 pid, count = 0;

	pid = osml_pid ? osml_pid : fg_pid;
	pid = (ioc_pid == -1) ? pid : ioc_pid;
	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (!task) {
		rcu_read_unlock();
		pr_err("get_thread_count find_task_by_vpid failed.");
		return 1;
	}
	*event_uid = __task_cred((task))->uid.val;

	for_each_process_thread(p, t) {
		if (__task_cred((t))->uid.val == *event_uid)
			count++;
	}
	rcu_read_unlock();

	return count;
}

static void init_perf_event(int event_config, int event_type, struct event_data *eventd, bool dimension)
{
	struct perf_event_attr attr;
	struct perf_event *event;
	struct task_struct *p, *t;
	int i = 0;

	memset(&attr, 0, sizeof(struct perf_event_attr));
	attr.config = event_config;
	attr.type = event_type;
	attr.size = sizeof(struct perf_event_attr);
	attr.inherit = 1;

	if (!dimension) {
		for (i = 0; i < eventd->pdata_cnt; i++) {
			event = perf_event_create_kernel_counter(&attr, i, NULL, NULL, NULL);
			if (IS_ERR(event)) {
				(eventd->pdata + i)->pevent = NULL;
				pr_err("osml event create failed. error no: %ld\n, event_config: %d, event_type: %d", PTR_ERR(event), event_config, event_type);
				break;
			}
			perf_event_enable(event);
			(eventd->pdata + i)->pevent = event;
		}
	} else {
		rcu_read_lock();
		for_each_process_thread(p, t) {
			if (__task_cred((t))->uid.val == eventd->uid && eventd->pdata_cnt > i) {
				get_task_struct(t);
				rcu_read_unlock();

				event = perf_event_create_kernel_counter(&attr, -1, t, NULL, NULL);
				if (IS_ERR(event)) {
					(eventd->pdata + i)->pevent = NULL;
					pr_err("osml event create failed. error no: %ld\n, event_config: %d, event_type: %d", PTR_ERR(event), event_config, event_type);
				} else {
					perf_event_enable(event);
					(eventd->pdata + i)->pevent = event;
				}
				i++;

				rcu_read_lock();
				put_task_struct(t);
			}
		}
		rcu_read_unlock();
	}
}

static int init_sample_events(void)
{
	int index, count = 0, offset = 0, total_pevent = 0;
	bool monitor_dimension = sample_dimension;
	struct event_data *event;

	pr_info("init_sample_events initialize");

	for (index = 0; index < pmonitor.event_size; index++) {
		short sample_event = sample_events[index - offset];

		if ((MAX_EVENT_BASE <= sample_event && sample_event < CPU_LOAD_0) ||
				(MAX_EVENT_CPU <= sample_event && sample_event < DDR_FREQ) ||
				(MAX_EVENT_MEM <= sample_event && sample_event < GPU_FREQ) ||
				(MAX_EVENT_GPU <= sample_event && sample_event < CUSTOM_PEVENT)) {
			pr_err("event:%d isn't exist!, option is %d ~ %d, %d ~ %d, %d ~ %d, %d ~ %d, %d ~ %d\n",
					sample_event, TS, MAX_EVENT_BASE - 1, CPU_LOAD_0, MAX_EVENT_CPU - 1, DDR_FREQ, MAX_EVENT_MEM - 1,
					GPU_FREQ, MAX_EVENT_GPU - 1, CUSTOM_PEVENT, MAX_EVENT_OTHER - 1);
			return 1;
		}

		event = kzalloc(sizeof(struct event_data), GFP_KERNEL);
		if (!event) {
			pr_err("init_sample_events event:%d kzalloc failed.", index);
			continue;
		}

		event->buf_idx = index;
		event->event_idx = sample_event;

		switch (event->event_idx) {
		case CUSTOM_PEVENT:
			event->title = pevent_name[count];
			if (total_pevent < MAX_PEVENT_SIZE) {
				event->pdata_cnt = !monitor_dimension ? nr_cores : get_thread_count(-1, &(event->uid));
				event->pdata = kzalloc(sizeof(struct perf_data) * event->pdata_cnt, GFP_KERNEL);
				if (!event->pdata) {
					pr_err("init_sample_events event->pdata kzalloc failed.");
					kfree(event);
					continue;
				}
				init_perf_event(pevent_id[count], pevent_type[count], event, monitor_dimension);
				count++;
				if (count == pevent_cnt) {
					count = 0;
				} else {
					offset++;
				}
				total_pevent++;
			} else
				pr_warn("%d perf-events are the limit.", MAX_PEVENT_SIZE);
			break;
		case LLCC_USAGE:
			event->title = llcc_cache_usage_name[count];
			count++;
			if (count == MAX_USAGE) {
				count = 0;
			} else {
				offset++;
			}
			break;
		case DVFSRC_SW_BW:
			event->title = dvfsrc_sw_bw_name[count];
			count++;
			if (count == MAX_DVFSRC_SW_BW) {
				count = 0;
			} else {
				offset++;
			}
			break;
		case EMIBW_MON:
			event->title = emibw_mon_name[count];
			count++;
			if (count == MAX_EMIBW_MON) {
				count = 0;
			} else {
				offset++;
			}
			break;
		case EMI_OPY_BW:
			event->title = emi_opy_bw_name[count];
			count++;
			if (count == MAX_EMI_OPY_BW) {
				count = 0;
			} else {
				offset++;
			}
			break;
		case EMI_TP:
			event->title = emi_tp_name[count];
			count++;
			if (count == MAX_EMI_TP) {
				count = 0;
			} else {
				offset++;
			}
			break;
		case DRAMC_OPY_BW:
			event->title = dramc_opy_bw_name[count];
			count++;
			if (count == MAX_DRAMC_OPY_BW) {
				count = 0;
			} else {
				offset++;
			}
			break;
		case MEM_BW:
			event->title = mem_bw_name[count];
			count++;
			if (count == MAX_MEM_BW) {
				count = 0;
			} else {
				offset++;
			}
			break;
		case MEM_CALC_BW:
			event->title = mem_calc_bw_name[count];
			count++;
			if (count == MAX_MEM_CALC_BW) {
				count = 0;
			} else {
				offset++;
			}
			break;
		default:
			event->title = monitor_case[event->event_idx];
			break;
		}
		INIT_LIST_HEAD(&event->osml_event_node);
		mutex_lock(&list_mutex_lock);
		list_add_tail(&(event->osml_event_node), &list_event_head[OSML_EVENT]);
		mutex_unlock(&list_mutex_lock);
		if (osml_debug)
			pr_info("init_sample_events list_head %p, evnet_id %d, event_name %s",
					&list_event_head[OSML_EVENT], event->event_idx, event->title);
	}
	return 0;
}

static void release_event(int list_type)
{
	int event_idx = 0;
	struct event_data *event, *next;

	mutex_lock(&list_mutex_lock);
	list_for_each_entry_safe(event, next, &list_event_head[list_type], osml_event_node) {
		if (osml_debug)
			pr_info("release_event %p %d %p\n", &list_event_head[list_type], event->event_idx, &event->osml_event_node);

		if (event->pdata) {
			for (event_idx = 0; event_idx < event->pdata_cnt; event_idx++) {
				if (!(event->pdata + event_idx)->pevent)
					continue;
				perf_event_disable((event->pdata + event_idx)->pevent);
				perf_event_release_kernel((event->pdata + event_idx)->pevent);
				(event->pdata + event_idx)->pevent = NULL;
			}
			kfree(event->pdata);
		}
		list_del(&event->osml_event_node);
		kfree(event);
	}
	mutex_unlock(&list_mutex_lock);
	pr_info("release_event sample event released.");
}

static ssize_t osml_enable_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned int val, i;
	char page[32] = {0};

	if (simple_write_to_buffer(page, sizeof(page), ppos, buf, count) <= 0)
		return -EINVAL;

	if (sscanf(page, "%u", &val) < 0) {
		pr_err("error setting argument. argument should be 1 or 0\n");
		return -EINVAL;
	}

	if (osml_enable == !!val)
		return count;

	mutex_lock(&enable_mutex_lock);
	osml_enable = !!val;
	if (osml_enable) {

		monitor_type = sample_type;
		pmonitor.event_size = 0;

		for (i = 0; i < sample_events_cnt; i++) {
			if (sample_events[i] == CUSTOM_PEVENT) {
				pmonitor.event_size += pevent_cnt - 1;
			} else if (sample_events[i] == LLCC_USAGE) {
				pmonitor.event_size += MAX_USAGE - 1;
			} else if (sample_events[i] == DVFSRC_SW_BW) {
				pmonitor.event_size += MAX_DVFSRC_SW_BW - 1;
			} else if (sample_events[i] == EMIBW_MON) {
				pmonitor.event_size += MAX_EMIBW_MON - 1;
			} else if (sample_events[i] == EMI_OPY_BW) {
				pmonitor.event_size += MAX_EMI_OPY_BW - 1;
			} else if (sample_events[i] == EMI_TP) {
				pmonitor.event_size += MAX_EMI_TP - 1;
			} else if (sample_events[i] == DRAMC_OPY_BW) {
				pmonitor.event_size += MAX_DRAMC_OPY_BW - 1;
			} else if (sample_events[i] == MEM_BW) {
				pmonitor.event_size += MAX_MEM_BW - 1;
			} else if (sample_events[i] == MEM_CALC_BW) {
				pmonitor.event_size += MAX_MEM_CALC_BW - 1;
			}
		}

		pmonitor.event_size += sample_events_cnt;
		if (!pmonitor.buf) {
			pmonitor.buf = (long long *)vzalloc(sizeof(long long) * pmonitor.event_size * MAX_REPORT_SIZE);
			if (!pmonitor.buf) {
				pr_err("vzalloc failed.\n");
				goto err_buf;
			}
		}

		if (init_sample_events())
			goto osml_disable;

		if (monitor_type) {
			if (IS_ERR(osml_polling_tsk))
				goto osml_disable;
			wake_up_process(osml_polling_tsk);
		}

		pr_info("osml monitor enable.");
	} else
		goto osml_disable;

	mutex_unlock(&enable_mutex_lock);
	return count;

osml_disable:
	release_event(OSML_EVENT);
err_buf:
	vfree(pmonitor.buf);
	pmonitor.buf = NULL;
	osml_enable = 0;
	record_cnt = 0;
	fg_pid = 0;
	atomic_set(&frame_number, -1);
	atomic_set(&frame_owner, -1);
	atomic_set(&frame_duration, -1);
	atomic_set(&vsync_period, -1);

	pr_info("osml monitor disable.\n");

	mutex_unlock(&enable_mutex_lock);
	return count;
}
OSML_PROC_READ(osml_enable);
OSML_RW_PROC_OPS(osml_enable);

static ssize_t osml_pid_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned int val;
	char page[32] = {0};

	if (simple_write_to_buffer(page, sizeof(page), ppos, buf, count) <= 0)
		return -EINVAL;

	if (sscanf(page, "%u\n", &val) < 0) {
		pr_err("error setting argument. argument should be positive\n");
		return -EINVAL;
	}

	if (osml_enable) {
		pr_err("need to disable osml before assign pid!\n");
		return -EINVAL;
	}

	osml_pid = val;
	pr_info("osml assign osml_pid %d.", osml_pid);
	return count;
}
OSML_PROC_READ(osml_pid);
OSML_RW_PROC_OPS(osml_pid);

static inline u64 read_pevent(struct perf_data *pdatas, int pdata_cnt)
{
	u64 total, enabled, running, sum = 0;
	struct perf_data *pdata;
	int i;

	for (i = 0; i < pdata_cnt; i++) {
		pdata = pdatas + i;
		if (!pdata->pevent)
			continue;

		total = perf_event_read_value(pdata->pevent, &enabled,
				&running);
		if (!pdata->prev_count)
			pdata->last_delta = 0;
		else
			pdata->last_delta = total - pdata->prev_count;

		pdata->prev_count = total;
		sum += pdata->last_delta;
	}

	return sum;
}

u64 jiffies64_to_nsecs(u64 j)
{
#if !(NSEC_PER_SEC % HZ)
	return (NSEC_PER_SEC / HZ) * j;
# else
	return div_u64(j * HZ_TO_NSEC_NUM, HZ_TO_NSEC_DEN);
#endif
}

// keep this light way and not get to sleep
// 1. called from walt irq work
// 2. called with spin lock held
// TODO maybe this part can off load to kworker
static int osml_update_cpu_load(int cpu)
{
	static u64 prev_busy_time[DEFAULT_NR_CORES] = {0};
	static u64 prev_wall_time[DEFAULT_NR_CORES] = {0};
	u64 busy_time = 0, wall_time, load, duration;
	u64 sload = 0;
	struct kernel_cpustat kcpustat;

	wall_time = jiffies64_to_nsecs(get_jiffies_64());

	kcpustat_cpu_fetch(&kcpustat, cpu);
	busy_time = kcpustat.cpustat[CPUTIME_USER];
	busy_time += kcpustat.cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat.cpustat[CPUTIME_IRQ];
	busy_time += kcpustat.cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat.cpustat[CPUTIME_STEAL];
	busy_time += kcpustat.cpustat[CPUTIME_NICE];

	load = busy_time - prev_busy_time[cpu];
	duration = wall_time - prev_wall_time[cpu];
	prev_busy_time[cpu] = busy_time;
	prev_wall_time[cpu] = wall_time;

	if (duration == 0) {
		if (osml_debug)
			pr_err("osml sload: %llu %lld\n", load, duration);
		return -1;
	}

	sload = load * 100 / duration;

	if (osml_debug)
		pr_info("osml cpu %d, sload: %lld, duration: %lld, busy_time %lld",
				cpu, sload, duration, load);

	return sload;
}

static void ioc_collect(struct osml_parcel *p)
{
	struct event_data *event, *next;
	list_for_each_entry_safe(event, next, &list_event_head[IOC_EVENT], osml_event_node) {
		if (event->pdata)
			p->pevent_val[event->buf_idx] = read_pevent(event->pdata, event->pdata_cnt);
	}
}

static void init_ioc_event(int ioc_pid)
{
	struct event_data *event;
	int i;

	for (i = 0; i < pevent_cnt; i++) {
		event = kzalloc(sizeof(struct event_data), GFP_KERNEL);
		if (!event) {
			pr_err("init_ioc_events event:%d kzalloc failed.", i);
			continue;
		}
		event->pdata_cnt = get_thread_count(ioc_pid, &(event->uid));
		event->pdata = kzalloc(sizeof(struct perf_data) * event->pdata_cnt, GFP_KERNEL);
		if (!event->pdata) {
			pr_err("init_ioc_events event->pdata kzalloc failed.");
			kfree(event);
			continue;
		}
		event->buf_idx = i;
		event->event_idx = pevent_id[i];
		init_perf_event(pevent_id[i], pevent_type[i], event, 1);
		INIT_LIST_HEAD(&event->osml_event_node);
		list_add_tail(&(event->osml_event_node), &list_event_head[IOC_EVENT]);
	}
	pr_info("init_ioc_event done.");
}

static long osml_ctl_ioctl(struct file *file, unsigned int cmd, unsigned long __user arg)
{
	static u64 pid;
	struct osml_parcel p;

	if (_IOC_TYPE(cmd) != OSML_IOC_MAGIC)
		return 0;

	if (osml_debug)
		pr_info("%s: cmd: %x, arg: %lu, OSML_IOC_COLLECT %lx", __func__, cmd, arg, OSML_IOC_COLLECT);
	switch (cmd) {
	case OSML_IOC_COLLECT:
	{
		if (copy_from_user(&p, (struct osml_parcel *) arg, sizeof(struct osml_parcel)))
			return 0;

		if (pid != p.pid) {
			release_event(IOC_EVENT);
			pid = p.pid;
			init_ioc_event(pid);
		}

		ioc_collect(&p);

		if (copy_to_user((struct osml_parcel __user *) arg, &p, sizeof(p)))
			return 0;
		break;
	}
	default:
		pr_err("osml_ctl_ioctl cmd did not exist.");
		return -1;
	}
	return 0;
}

static const struct file_operations osml_ctl_ops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = osml_ctl_ioctl,
	.compat_ioctl = osml_ctl_ioctl,
};

static void ioc_exit(void)
{
	cdev_del(&cdev);
	device_destroy(driver_class, osml_ctl_dev);
	class_destroy(driver_class);
	unregister_chrdev_region(osml_ctl_dev, 1);
}

static int ioc_init(void)
{
	int rc;
	struct device *class_dev;

	rc = alloc_chrdev_region(&osml_ctl_dev, 0, 1, OSML_CTL_NODE);
	if (rc < 0) {
		pr_err("alloc_chrdev_region failed %d", rc);
		return 0;
	}

	driver_class = class_create(THIS_MODULE, OSML_CTL_NODE);
	if (IS_ERR(driver_class)) {
		pr_err("class_create failed %ld\n", PTR_ERR(driver_class));
		goto exit_unreg_chrdev_region;
	}

	class_dev = device_create(driver_class, NULL, osml_ctl_dev, NULL, OSML_CTL_NODE);
	if (IS_ERR(class_dev)) {
		pr_err("class_device_create failed %ld\n", PTR_ERR(class_dev));
		goto exit_destroy_class;
	}

	cdev_init(&cdev, &osml_ctl_ops);
	cdev.owner = THIS_MODULE;
	cdev_status = cdev_add(&cdev, MKDEV(MAJOR(osml_ctl_dev), 0), 1);
	if (cdev_status < 0) {
		pr_err("cdev_add failed %d\n", cdev_status);
		goto exit_destroy_device;
	}
	pr_info("ioc init done");
	return 0;

exit_destroy_device:
	device_destroy(driver_class, osml_ctl_dev);
exit_destroy_class:
	class_destroy(driver_class);
exit_unreg_chrdev_region:
	unregister_chrdev_region(osml_ctl_dev, 1);
	return -1;
}

static void collect_info(void)
{
	u64 row, idx, select_idx;
	struct cpufreq_policy *policy;
	static long long last_ts_us;
	struct event_data *event;
	int cpu_usage, gpu_usage, other_usage;
	ktime_t ktime = ktime_get();
	int bw_idx = -1;

	mutex_lock(&enable_mutex_lock);

	if (!pmonitor.buf) {
		mutex_unlock(&enable_mutex_lock);
		return;
	}

	if (record_cnt >= (unsigned int) MAX_REPORT_SIZE) {
		pr_err("report data is full.\n");
		mutex_unlock(&enable_mutex_lock);
		return;
	}

	row = record_cnt * pmonitor.event_size;
	mutex_lock(&list_mutex_lock);
	list_for_each_entry(event, &list_event_head[OSML_EVENT], osml_event_node) {
		idx = row + event->buf_idx;
		if (osml_debug)
			pr_info("%s event_idx %d, uid %d, pdata_cnt %d", event->title, event->event_idx, event->uid, event->pdata_cnt);

		switch (event->event_idx) {
		case TS:
			pmonitor.buf[idx] = ktime_to_ms(ktime);
			break;
		case TS_DELTA:
			if (row)
				pmonitor.buf[idx] = ktime_to_us(ktime) - last_ts_us;
			break;
		case FRAME_NUMBER:
			pmonitor.buf[idx] = atomic_read(&frame_number);
			break;
		case FRAME_OWNER:
			pmonitor.buf[idx] = atomic_read(&frame_owner);
			break;
		case FRAME_DURATION:
			pmonitor.buf[idx] = atomic_read(&frame_duration);
			break;
		case FRAME_VSYNC:
			pmonitor.buf[idx] = atomic_read(&vsync_period);
			break;
		case FRAME_ALLOW_DURATION:
			pmonitor.buf[idx] = atomic_read(&vsync_period) - DEADLINE_MS;
			break;
		case CPU_LOAD_0 ... CPU_LOAD_7:
			pmonitor.buf[idx] = osml_update_cpu_load(event->event_idx - CPU_LOAD_0);
			break;
		case CPU_FREQ_CLUSTER_0 ... CPU_FREQ_CLUSTER_2:
			select_idx = event->event_idx - CPU_FREQ_CLUSTER_0;
			if (cpu_clus[select_idx].clus_id == -1)
				break;
			policy = cpufreq_cpu_get_raw(cpu_clus[select_idx].clus_id);
			if (policy)
				pmonitor.buf[idx] = policy->cur;
			break;
		case CPU_SCALING_MAX_FREQ_0 ... CPU_SCALING_MIN_FREQ_2:
			select_idx = (event->event_idx - CPU_SCALING_MAX_FREQ_0) / 2;
			if (cpu_clus[select_idx].clus_id == -1)
				break;
			policy = cpufreq_cpu_get_raw(cpu_clus[select_idx].clus_id);
			if (policy) {
				select_idx = (event->event_idx - CPU_SCALING_MAX_FREQ_0) % 2;
				if (select_idx)
					pmonitor.buf[idx] = policy->min;
				else
					pmonitor.buf[idx] = policy->max;
			}
			break;
		case DDR_FREQ:
			pmonitor.buf[idx] = (long long)mtk_dvfsrc_query_opp_info(MTK_DVFSRC_CURR_DRAM_KHZ);
			break;
		case DVFSRC_CURR_DVFS_OPP:
			pmonitor.buf[idx] = (long long)mtk_dvfsrc_query_opp_info(MTK_DVFSRC_CURR_DVFS_OPP);
			break;
		case DVFSRC_CURR_VCORE_UV:
			pmonitor.buf[idx] = (long long)mtk_dvfsrc_query_opp_info(MTK_DVFSRC_CURR_VCORE_UV);
			break;
		case DVFSRC_CURR_VCORE_OPP:
			pmonitor.buf[idx] = (long long)mtk_dvfsrc_query_opp_info(MTK_DVFSRC_CURR_VCORE_OPP);
			break;
		case DVFSRC_SW_REQ_VCORE_OPP:
			pmonitor.buf[idx] = (long long)mtk_dvfsrc_query_opp_info(MTK_DVFSRC_SW_REQ_VCORE_OPP);
			break;
		case DVFSRC_SW_BW:
		{
			u32 dvfsrc_sw_bw[MAX_TOTAL_SW_BW_SIZE + 1] = {0};
			for (select_idx = 0; select_idx < MAX_TOTAL_SW_BW_SIZE; select_idx++) {
				dvfsrc_sw_bw[select_idx] = osml_get_dvfsrc_sw_bw(select_idx);
				dvfsrc_sw_bw[MAX_TOTAL_SW_BW_SIZE] += dvfsrc_sw_bw[select_idx];
			}
			pmonitor.buf[idx] = dvfsrc_sw_bw[DVFSRC_APU_SW_BW];
			pmonitor.buf[idx + DVFSRC_CPU_SW_BW] = dvfsrc_sw_bw[DVFSRC_CPU_SW_BW];
			pmonitor.buf[idx + DVFSRC_GPU_SW_BW] = dvfsrc_sw_bw[DVFSRC_GPU_SW_BW];
			pmonitor.buf[idx + DVFSRC_TOTAL_SW_BW] = dvfsrc_sw_bw[MAX_TOTAL_SW_BW_SIZE];
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			break;
		}
		case EMIBW_MON:
		{
			u32 emibw_mon[MAX_EMIBW_MON];
			for (select_idx = 0; select_idx < MAX_EMIBW_MON; select_idx++) {
				emibw_mon[select_idx] = (qos_sram_read(select_idx) & 0x7FFFFFFF) ^ 12345600;
			}
			pmonitor.buf[idx] = emibw_mon[QOS_DEBUG_0];
			pmonitor.buf[idx + EMIBW_MON_CPU] = emibw_mon[QOS_DEBUG_1];
			pmonitor.buf[idx + EMIBW_MON_GPU] = emibw_mon[QOS_DEBUG_2];
			pmonitor.buf[idx + EMIBW_MON_MM] = emibw_mon[QOS_DEBUG_3];
			pmonitor.buf[idx + EMIBW_MON_MD] = emibw_mon[QOS_DEBUG_4];
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			break;
		}
		case EMI_OPY_BW:
		{
			u32 emi_opy_bw[MAX_EMI_OPY_BW] = {0};
			if (bw_idx == -1)
				bw_idx = qos_rec_get_hist_idx();
			for (select_idx = 0; select_idx < MAX_EMI_OPY_BW; select_idx++) {
				emi_opy_bw[select_idx] = (qos_rec_get_hist_bw(bw_idx, select_idx) & 0x7FFFFFFF) ^ 12345600;
			}
			pmonitor.buf[idx] = emi_opy_bw[EMI_OPY_BW_TOTAL];
			pmonitor.buf[idx + EMI_OPY_BW_CPU] = emi_opy_bw[EMI_OPY_BW_CPU];
			pmonitor.buf[idx + EMI_OPY_BW_GPU] = emi_opy_bw[EMI_OPY_BW_GPU];
			pmonitor.buf[idx + EMI_OPY_BW_MM] = emi_opy_bw[EMI_OPY_BW_MM];
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			break;
		}
		case EMI_TP:
		{
			u32 emi_tp[MAX_EMI_TP] = {0};
			if (bw_idx == -1)
				bw_idx = qos_rec_get_hist_idx();
			for (select_idx = 0; select_idx < MAX_EMI_TP; select_idx++) {
				emi_tp[select_idx] = (qos_rec_get_hist_data_bw(bw_idx, select_idx) & 0x7FFFFFFF) ^ 12345600;
			}
			pmonitor.buf[idx] = emi_tp[EMI_TP_TOTAL];
			pmonitor.buf[idx + EMI_TP_CPU] = emi_tp[EMI_TP_CPU];
			pmonitor.buf[idx + EMI_TP_GPU] = emi_tp[EMI_TP_GPU];
			pmonitor.buf[idx + EMI_TP_MM] = emi_tp[EMI_TP_MM];
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			break;
		}
		case DRAMC_OPY_BW:
		{
			u32 dramc_opy_bw[MAX_DRAMC_OPY_BW] = {0};
			if (bw_idx == -1)
				bw_idx = qos_rec_get_hist_idx();
			for (select_idx = 0; select_idx < MAX_DRAMC_OPY_BW; select_idx++) {
				dramc_opy_bw[select_idx] = (qos_rec_get_dramc_hist_bw(bw_idx, select_idx) & 0x7FFFFFFF) ^ 12345600;
			}
			pmonitor.buf[idx] = dramc_opy_bw[DRAMC_OPY_BW_TOTAL];
			pmonitor.buf[idx + DRAMC_OPY_BW_CPU] = dramc_opy_bw[DRAMC_OPY_BW_CPU];
			pmonitor.buf[idx + DRAMC_OPY_BW_GPU] = dramc_opy_bw[DRAMC_OPY_BW_GPU];
			pmonitor.buf[idx + DRAMC_OPY_BW_MM] = dramc_opy_bw[DRAMC_OPY_BW_MM];
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			break;
		}
		case MEM_BW:
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			break;
		case MEM_CALC_BW:
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			break;
		case L3_FREQ:
			pmonitor.buf[idx] = -1;
			break;
		case LLCC_FREQ:
			pmonitor.buf[idx] = -1;
			break;
		case DDR_BW:
			pmonitor.buf[idx] = (long long)mtk_dramc_get_data_rate();
			break;
		case LLCC_BW:
			pmonitor.buf[idx] = -1;
			break;
		case GPU_FREQ:
			pmonitor.buf[idx] = (long long)gpufreq_get_cur_freq(TARGET_DEFAULT);
			break;
		case GED_GPU_FREQ:
			pmonitor.buf[idx] = (long long)osml_ged_dvfs_get_gpu_cur_freq();
			break;
		case CUSTOM_PEVENT:
			if (event->pdata)
				pmonitor.buf[idx] = read_pevent(event->pdata, event->pdata_cnt);
			break;
		case LLCC_CPU_CAPACITY:
			pmonitor.buf[idx] = slbc_get_cache_size(ID_CPU);
			break;
		case LLCC_GPU_CAPACITY:
			pmonitor.buf[idx] = slbc_get_cache_size(ID_GPU);
			break;
		case LLCC_USAGE:
			slbc_get_cache_usage(&cpu_usage, &gpu_usage, &other_usage);
			pmonitor.buf[idx] = cpu_usage;
			pmonitor.buf[idx + LLCC_GPU_USAGE] = gpu_usage;
			pmonitor.buf[idx + LLCC_OTHER_USAGE] = other_usage;
			event = list_next_entry(event, osml_event_node);
			event = list_next_entry(event, osml_event_node);
			break;
		case LLCC_CPU_RD_HIT_RATE:
			pmonitor.buf[idx] = slbc_get_cache_hit_rate(ID_CPU);
			break;
		case LLCC_GPU_RD_HIT_RATE:
			pmonitor.buf[idx] = slbc_get_cache_hit_rate(ID_GPU);
			break;
		case LLCC_CPU_RD_HIT_BW:
			pmonitor.buf[idx] = slbc_get_cache_hit_bw(ID_CPU);
			break;
		case LLCC_GPU_RD_HIT_BW:
			pmonitor.buf[idx] = slbc_get_cache_hit_bw(ID_GPU);
			break;
		}
	}
	mutex_unlock(&list_mutex_lock);
	data_diving_time = ktime_us_delta(ktime_get(), ktime);
	last_ts_us = ktime_to_us(ktime);
	record_cnt++;

	mutex_unlock(&enable_mutex_lock);
}

static void osml_trigger_fn(struct work_struct *work)
{
	if (osml_enable)
		collect_info();
}
static DECLARE_WORK(osml_work, osml_trigger_fn);

static ssize_t frame_duration_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned int pid_in, duration_in, expect_duration_in;
	char page[1024] = {0};

	if (!osml_enable || monitor_type)
		return count;

	if (simple_write_to_buffer(page, sizeof(page), ppos, buf, count) <= 0)
		return -EINVAL;

	if (sscanf(page, "%u,%u,%u\n", &pid_in, &duration_in, &expect_duration_in) < 0) {
		pr_err("error setting argument. argument should be positive\n");
		return -EINVAL;
	}

	if (!fg_pid && pid_in == osml_pid) {
		fg_pid = osml_pid;
		atomic_set(&frame_owner, pid_in);
	}

	if (osml_debug)
		pr_info("osml_frame %s\n", buf);

	if (osml_workq && fg_pid == pid_in) {
		atomic_inc(&frame_number);
		atomic_set(&frame_duration, duration_in);
		atomic_set(&vsync_period, expect_duration_in);
		queue_work(osml_workq, &osml_work);
	}

	return count;
}

static const struct proc_ops frame_duration_proc_ops = {
	.proc_write		= frame_duration_proc_write,
	.proc_lseek		= default_llseek,
};

static int osml_polling_fn(void *p)
{
	unsigned int usleep_time;

	while (1) {
		if (!osml_enable) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			set_current_state(TASK_RUNNING);
		} else {
			usleep_time = osml_sample_rate >= data_diving_time ?
				osml_sample_rate - data_diving_time : DEFAULT_USLEEP_TIME;
			usleep_range(usleep_time, usleep_time + 100);
		}

		if (osml_enable)
			collect_info();

		if (kthread_should_stop())
			break;
	}
	return 0;
}

static void prepare_cpuinfo(void)
{
	unsigned int cpu, last_val = 0, clus_idx = 0, i, total_state = 0;
	struct em_perf_domain *domain;

	for_each_possible_cpu(cpu) {
		domain = em_cpu_get(cpu);
		if (last_val != domain->table->frequency) {
			last_val = domain->table->frequency;
			cpu_clus[clus_idx].max_state = domain->nr_perf_states;
			total_state += cpu_clus[clus_idx].max_state;
			cpu_clus[clus_idx].clus_id = cpu;
			cpu_clus[clus_idx].pwr_tbl = kzalloc(sizeof(u64) * cpu_clus[clus_idx].max_state, GFP_KERNEL);
			if (cpu_clus[clus_idx].pwr_tbl) {
				for (i = 0; i < cpu_clus[clus_idx].max_state; i++) {
					cpu_clus[clus_idx].pwr_tbl[i] =
						domain->table[i].frequency;
				}
				cpu_clus[clus_idx].num_cpu = 1;
				clus_idx++;
			} else
				pr_err("osml power_table kzalloc failed.");
		} else {
			if (clus_idx)
				cpu_clus[clus_idx - 1].num_cpu++;
		}
	}

}

static void create_osml_proc(struct proc_dir_entry *parent_dir)
{
	struct proc_dir_entry *osml_setting_dir;
	osml_setting_dir = proc_mkdir(PROC_OSML_SETTINGS_DIR, parent_dir);
	if (!osml_setting_dir) {
		pr_warn("fail to mkdir /proc/oplus_slc/osml_settings");
		return;
	}

	proc_create_data("debug", 0664, osml_setting_dir, &osml_debug_proc_ops, NULL);
	proc_create_data("enable", 0664, osml_setting_dir, &osml_enable_proc_ops, NULL);
	proc_create_data("reset", 0664, osml_setting_dir, &osml_reset_proc_ops, NULL);
	proc_create_data("pid", 0664, osml_setting_dir, &osml_pid_proc_ops, NULL);
	proc_create_data("sample_type", 0664, osml_setting_dir, &sample_type_proc_ops, NULL);
	proc_create_data("sample_dimension", 0664, osml_setting_dir, &sample_dimension_proc_ops, NULL);
	proc_create_data("sample_rate_ms", 0664, osml_setting_dir, &osml_sample_rate_proc_ops, NULL);
	proc_create_data("sample_events", 0664, osml_setting_dir, &sample_events_proc_ops, NULL);
	proc_create_data("pevent_name", 0664, osml_setting_dir, &pevent_name_proc_ops, NULL);
	proc_create_data("pevent_id", 0664, osml_setting_dir, &pevent_id_proc_ops, NULL);
	proc_create_data("pevent_type", 0664, osml_setting_dir, &pevent_type_proc_ops, NULL);
	proc_create_data("frame_duration", 0664, osml_setting_dir, &frame_duration_proc_ops, NULL);
	proc_create_data("osml_report", S_IFREG | 0444, parent_dir, &osml_report_proc_ops, NULL);
}

void osml_init(struct proc_dir_entry *parent_dir)
{
	int i, j;

	pr_info("osml init");

	nr_cores = num_possible_cpus();
	prepare_cpuinfo();

	// init sample_events
	for (i = 0, j = 0; i < MONITOR_SIZE; i++) {
		if ((TS <= i && i < MAX_EVENT_BASE) || (CPU_LOAD_0 <= i && i < MAX_EVENT_CPU) ||
				(DDR_FREQ <= i && i < MAX_EVENT_MEM) || (GPU_FREQ <= i && i < MAX_EVENT_GPU) ||
				(CUSTOM_PEVENT <= i && i < MAX_EVENT_OTHER)) {
			sample_events[j] = i;
			j++;
		}
	}
	sample_events_cnt = j;

	ioc_init();

	osml_polling_tsk = kthread_run(osml_polling_fn, 0, "osml_monitor");
	if (IS_ERR(osml_polling_tsk)) {
		pr_err("Failed to start osml_polling_task");
	}

	osml_workq = alloc_ordered_workqueue("osml_wq", WQ_HIGHPRI);
	if (!osml_workq) {
		pr_err("alloc work queue fail");
	}

	create_osml_proc(parent_dir);

}

void osml_exit(void)
{
	struct workqueue_struct *tmp;
	int i;

	if (osml_polling_tsk)
		kthread_stop(osml_polling_tsk);
	if (osml_workq) {
		tmp = osml_workq;
		osml_workq = NULL;
		destroy_workqueue(tmp);
	}

	if (osml_enable) {
		osml_enable = 0;
		release_event(OSML_EVENT);
		vfree(pmonitor.buf);
		pmonitor.buf = NULL;
	}

	if (!cdev_status)
		ioc_exit();

	for (i = 0; i < OSML_MAX_CLUSTER; i++) {
		if (cpu_clus[i].pwr_tbl) {
			kfree(cpu_clus[i].pwr_tbl);
			cpu_clus[i].pwr_tbl = NULL;
		}
	}

	pr_info("osml_exit\n");
}

