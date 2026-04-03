/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */

#ifndef _OPLUS_SA_PIPELINE_H_
#define _OPLUS_SA_PIPELINE_H_

void oplus_pipeline_init(struct proc_dir_entry *pde);
int oplus_get_task_pipeline_cpu(struct task_struct *task);
bool oplus_pipeline_low_latency_task(int pipeline_cpu);
#if IS_ENABLED(CONFIG_SCHED_WALT)
void qcom_rearrange_pipeline_preferred_cpus(unsigned int divisor);
#endif
#if IS_ENABLED(CONFIG_MTK_FPSGO_V3)
void mtk_rearrange_pipeline_preferred_cpus(struct task_struct *p, const struct cpumask *in_mask);
#endif
bool oplus_is_pipeline_scene(void);
bool oplus_pipeline_task_skip_cpu(struct task_struct *task, unsigned int dst_cpu);
bool oplus_pipeline_rt_skip_prime_cpu(unsigned int dst_cpu);
bool oplus_lowend_platform_pipeline_task_ux(struct task_struct *task);

typedef int (*core_ctl_set_boost_t)(bool boost);
typedef int (*core_ctl_set_cluster_boost_t)(int idx, bool boost);

extern core_ctl_set_boost_t oplus_core_ctl_set_boost;
extern core_ctl_set_cluster_boost_t oplus_core_ctl_set_cluster_boost;

#endif /* _OPLUS_SA_PIPELINE_H_ */
