#include <linux/fs.h>
#include <linux/proc_fs.h>
#include "../sched_assist/sa_fair.h"
#include "../sched_assist/sa_common.h"
#if IS_ENABLED(CONFIG_SCHED_WALT)
#include <../../../kernel/sched/walt/walt.h>
#else
#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>
#endif
#include "perf_schedinfo.h"

u64 cpu_schedinfo[NR_CPUS];
static DEFINE_MUTEX(mutex);

enum cpu_state get_cpu_state(int cpu)
{
	enum cpu_state state;
	if (cpu_online(cpu)) {
#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
		if (oplus_cpu_halted(cpu))
			state = CPU_PAUSED;
		else
			state = CPU_NORMAL;
#else
		state = CPU_NORMAL;
#endif
	} else
		state = CPU_OFFLINE;
	return state;
}

#if 0//IS_ENABLED(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM)
unsigned int sched_get_cpu_util_pct(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	struct walt_rq *wrq = (struct walt_rq *) cpu_rq(cpu)->android_vendor_data1;
	u64 util;
	unsigned long capacity, flags;
	unsigned int busy, walt_divisor;;

	raw_spin_lock_irqsave(&rq->__lock, flags);

	capacity = capacity_orig_of(cpu);
	walt_divisor = sched_ravg_window >> SCHED_CAPACITY_SHIFT;
	util = wrq->prev_runnable_sum + wrq->grp_time.prev_runnable_sum;
	do_div(util, walt_divisor);

	raw_spin_unlock_irqrestore(&rq->__lock, flags);

	util = (util >= capacity) ? capacity : util;
	busy = div64_ul((util * 100), capacity);
	return busy;
}
#else
unsigned int sched_get_cpu_util_pct(int cpu)
{
	struct cfs_rq *cfs_rq;
	unsigned int util, busy;
	unsigned long capacity;

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	if (sched_feat(UTIL_EST))
		util = max(util, READ_ONCE(cfs_rq->avg.util_est.enqueued));

	capacity = capacity_orig_of(cpu);

	util = (util >= capacity) ? capacity : util;
	busy = div64_ul((util * 100), capacity);
	return busy;
}
#endif


static ssize_t proc_perf_schedinfo_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	int cpu, size = 0;
	char buffer[MAX_MSG_SIZE];
	ssize_t ret;
	mutex_lock(&mutex);
	memset(cpu_schedinfo, 0, sizeof(u64) * NR_CPUS);
	for_each_possible_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);
		struct oplus_rq *orq = get_oplus_rq(rq);
		struct cfs_rq *cfs_rq = &rq->cfs;
		struct rt_rq *rt_rq = &rq->rt;
		unsigned int cfs_running = cfs_rq->h_nr_running;
		unsigned int rt_running = rt_rq->rt_nr_running;
		unsigned int ux_running = orq->nr_running;
		unsigned int busy_pct = sched_get_cpu_util_pct(cpu);
		u64 *cpu_info = &cpu_schedinfo[cpu];

		if (cfs_running >= SCHED_MAX_CFS_R)
			cfs_running = SCHED_MAX_CFS_R - 1;
		if (rt_running >= SCHED_MAX_RT_R)
			rt_running = SCHED_MAX_RT_R - 1;
		if (ux_running >= SCHED_MAX_UX_R)
			ux_running = SCHED_MAX_UX_R - 1;

		if (busy_pct >= SCHED_MAX_CPU_UTIL_PCT)
			busy_pct = SCHED_MAX_CPU_UTIL_PCT - 1;

		*cpu_info += get_cpu_state(cpu);

		*cpu_info += cfs_running * CFS_R_MULT_UNIT;
		*cpu_info += rt_running * RT_R_MULT_UNIT;
		*cpu_info += ux_running * UX_R_MULT_UNIT;

		*cpu_info += curr_capacity_of(cpu) * CPU_CAP_MULT_UNIT;
		*cpu_info += busy_pct * CPU_UTIL_PCT_MULT_UNIT;

		size += sprintf(buffer + size, "cpu%d:%llu ",
				cpu, *cpu_info);
	}
	buffer[size] = '\0';

	ret = simple_read_from_buffer(buf, count, ppos, buffer, size + 1);
	mutex_unlock(&mutex);
	return ret;
}

static const struct proc_ops proc_perf_schedinfo_ops = {
	.proc_read      =       proc_perf_schedinfo_read,
	.proc_lseek     =       default_llseek,
};
struct proc_dir_entry *schedinfo_proc_init(
			struct proc_dir_entry *pde)
{
	struct proc_dir_entry *entry = NULL;
	entry = proc_create("perf_schedinfo", S_IRUGO, pde, &proc_perf_schedinfo_ops);

	return entry;
}

struct proc_dir_entry *perf_schedinfo_init(struct proc_dir_entry *pde)
{
	return schedinfo_proc_init(pde);
}

void perf_schedinfo_exit(
			struct proc_dir_entry *pde)
{
	remove_proc_entry("perf_schedinfo", pde);
}
