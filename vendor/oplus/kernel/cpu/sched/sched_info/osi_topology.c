// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <uapi/linux/sched/types.h>

#include "osi_topology.h"

struct cluster_info cluster[CPU_NUMS];
u32 cluster_num;
struct cpumask all_cpu;
struct cpumask silver_cpu;
struct cpumask gold_cpu;


void update_cpu_mask(void)
{
	unsigned int cpu, min_capacity = arch_scale_cpu_capacity(0);

	for_each_possible_cpu(cpu) {
		if (arch_scale_cpu_capacity(cpu) <= min_capacity)
			min_capacity = arch_scale_cpu_capacity(cpu);
		cpumask_set_cpu(cpu, &gold_cpu);
	}
	for_each_possible_cpu(cpu) {
		if (arch_scale_cpu_capacity(cpu) == min_capacity)
			cpumask_clear_cpu(cpu, &gold_cpu);
	}
	cpumask_andnot(&silver_cpu, &all_cpu, &gold_cpu);
	pr_info("cpu_highcap_mask:%lx, %lx, %lx\n", cpumask_bits(&all_cpu)[0],
		cpumask_bits(&silver_cpu)[0], cpumask_bits(&gold_cpu)[0]);
}

void cluster_init(void)
{
	u32 cpu;
	u32 cluster_id;
	struct cpu_topology *cpu_topo;

	memset(&cluster, 0, sizeof(struct cluster_info) * CPU_NUMS);
	for_each_possible_cpu(cpu) {
		cpu_topo = &cpu_topology[cpu];
		cpumask_set_cpu(cpu, &all_cpu);
		cluster_id = topology_cluster_id(cpu);
		cluster_num = max(cluster_id, cluster_num);
		if (cluster_id > CPU_NUMS)
			continue;
		cluster[cluster_id].cpu_nr = cpumask_weight(&cpu_topo->cluster_sibling);
		cluster[cluster_id].start_cpu = cpumask_first(&cpu_topo->cluster_sibling);
	}
	cluster_num += 1;
	update_cpu_mask();
}


u32 get_start_cpu(u32 cpu)
{
	u32 start_cpu;
	struct cpu_topology *cpu_topo;

	cpu_topo = &cpu_topology[cpu];
	start_cpu = cpumask_first(&cpu_topo->cluster_sibling);
	return start_cpu;
}

u32 get_cluster_id(u32 cpu)
{
	u32 cluster_id;

	cluster_id = topology_cluster_id(cpu);
	return cluster_id;
}

bool is_cluster_cpu(u32 cpu)
{
	u32 start_cpu;
	struct cpu_topology *cpu_topo;

	cpu_topo = &cpu_topology[cpu];
	start_cpu = cpumask_first(&cpu_topo->cluster_sibling);
	if (start_cpu == cpu)
		return true;
	return false;
}

