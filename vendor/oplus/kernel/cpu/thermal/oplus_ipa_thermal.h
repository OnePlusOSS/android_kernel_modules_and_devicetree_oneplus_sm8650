/*
 * Copyright (c) 2021 oplus Technology(Shanghai) Corp.,Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _oplus_IPA_THERMAL_H
#define _oplus_IPA_THERMAL_H
#include <linux/kthread.h>
#include <uapi/linux/thermal.h>

#define VIRTUAL_TEMP			45000
#define MCELSIUS			1000
#define THERMAL_SENSOR_PROBE_NUM	16
#define CDEV_GRANT_NUM	6
#define MAX_POWER_BUDGET	20000
#define MIN_POWER_BUDGET	-2000
#define CPU_POWER_MAX	9999
#define CLUSTER_NUM	6

struct oplus_ipa_param {
	s64 err_integral;
	s32 prev_err;
	int trip_switch_on;
	int trip_control_temp;

	u32 sustainable_power;
	s32 k_po;
	s32 k_pu;
	s32 k_i;
	s32 k_d;
	s32 i_max;
	s32 integral_cutoff;

	int polling_delay_on;
	int polling_delay_off;

	bool switched_on;
	bool use_power_budget;
	int power_buget;
	bool use_thermalcontrol_limit;
	unsigned int cdev_min_power[CDEV_GRANT_NUM];
	int clt_state_limit[CLUSTER_NUM];
};

struct oplus_thermal_data {
	int id;
	unsigned long max_cdev;
	void __iomem *base;
	struct kthread_worker thermal_worker;
	struct mutex lock;
	struct thermal_zone_device *tzd;
	unsigned int ntrip;
	bool enabled;
	struct list_head node;
	char thermal_name[THERMAL_NAME_LENGTH + 1];
	struct device_node *np;
	int temperature;
	bool use_ipa_thermal;
	struct kthread_delayed_work ipa_work;
	struct oplus_ipa_param *ipa_param;
	struct cpumask thermal_work_affinity;
};

#endif /* _oplus_IPA_THERMAL_H */
