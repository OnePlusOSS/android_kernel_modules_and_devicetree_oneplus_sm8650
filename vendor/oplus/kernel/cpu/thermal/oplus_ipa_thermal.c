// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/thermal.h>
#include <linux/threads.h>
#include <uapi/linux/sched/types.h>

#include <linux/cpufreq.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/version.h>

#include <drivers/thermal/thermal_core.h>
#include "oplus_ipa_thermal.h"
#include "horae_shell_temp.h"

#define FRAC_BITS 10
#define int_to_frac(x) ((x) << FRAC_BITS)
#define frac_to_int(x) ((x) >> FRAC_BITS)

#define INVALID_TRIP -1

#define CREATE_TRACE_POINTS
#include <trace/events/thermal_power_allocator.h>
#undef CREATE_TRACE_POINTS
#include <trace/hooks/thermal.h>


extern int get_current_shell_temp(void);

/**
 * mul_frac() - multiply two fixed-point numbers
 * @x:	first multiplicand
 * @y:	second multiplicand
 *
 * Return: the result of multiplying two fixed-point numbers.  The
 * result is also a fixed-point number.
 */
static inline s64 mul_frac(s64 x, s64 y)
{
	return (x * y) >> FRAC_BITS;
}

/**
 * div_frac() - divide two fixed-point numbers
 * @x:	the dividend
 * @y:	the divisor
 *
 * Return: the result of dividing two fixed-point numbers.  The
 * result is also a fixed-point number.
 */
static inline s64 div_frac(s64 x, s64 y)
{
	return div_s64(x << FRAC_BITS, y);
}

static atomic_t oplus_thermal_in_suspend;

static unsigned int num_of_devices;

/* list of multiple instance for each thermal sensor */
static LIST_HEAD(thermal_instance_list);

static void oplus_thermal_control(struct platform_device *pdev, bool on)
{
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);

	data->enabled = on;
}

static int get_temp_value(int tz, int *temp)
{
	*temp = get_current_shell_temp();

	return 0;
}

static int oplus_get_temp(struct thermal_zone_device *p, int *temp)
{
	struct oplus_thermal_data *data = p->devdata;
	int soc_tm_temp = 0;

	if (!data)
		return -EINVAL;

	mutex_lock(&data->lock);
	get_temp_value(data->id, &soc_tm_temp);

	*temp = soc_tm_temp;

	data->temperature = *temp;

	mutex_unlock(&data->lock);
	return 0;
}

static void start_ipa_polling(struct oplus_thermal_data *data, int delay)
{
	kthread_mod_delayed_work(&data->thermal_worker, &data->ipa_work,
				 msecs_to_jiffies(delay));
}

static void reset_ipa_trips(struct oplus_thermal_data *data)
{
	struct thermal_zone_device *tz = data->tzd;
	struct oplus_ipa_param *params = data->ipa_param;
	int i;
	bool found_first_passive = false;

	params->trip_switch_on = INVALID_TRIP;
	params->trip_control_temp = INVALID_TRIP;

	for (i = 0; i < tz->num_trips; i++) {
		enum thermal_trip_type type;
		int ret;

		ret = tz->ops->get_trip_type(tz, i, &type);
		if (ret) {
			dev_warn(&tz->device, "Failed to get trip point %d type: %d\n", i, ret);
			continue;
		}

		if (type == THERMAL_TRIP_PASSIVE && !found_first_passive) {
			params->trip_switch_on = i;
			found_first_passive = true;
		} else if (type == THERMAL_TRIP_ACTIVE) {
			params->trip_control_temp = i;
		} else {
			break;
		}
	}

	if (found_first_passive) {
		int temp = params->trip_control_temp;
		params->trip_control_temp = params->trip_switch_on;
		params->trip_switch_on = temp;
	}
}

static void reset_ipa_params(struct oplus_thermal_data *data)
{
	data->ipa_param->err_integral = 0;
	data->ipa_param->prev_err = 0;
	data->ipa_param->sustainable_power = CPU_POWER_MAX;
}

static void allow_maximum_power(struct oplus_thermal_data *data)
{
	struct thermal_instance *instance;
	struct thermal_zone_device *tz = data->tzd;
	int control_temp = data->ipa_param->trip_control_temp;

	mutex_unlock(&data->lock);
	mutex_lock(&tz->lock);
	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		if (instance->trip != control_temp ||
		    (!cdev_is_power_actor(instance->cdev)))
			continue;
		instance->target = 0;

		data->max_cdev = instance->target;
		mutex_lock(&instance->cdev->lock);
		instance->cdev->updated = false;
		mutex_unlock(&instance->cdev->lock);
		thermal_cdev_update(instance->cdev);
	}

	mutex_unlock(&tz->lock);
	mutex_lock(&data->lock);
}

static int
power_actor_set_power(struct thermal_cooling_device *cdev,
		      struct thermal_instance *instance, u32 power)
{
	unsigned long state;
	int ret;

	ret = cdev->ops->power2state(cdev, power, &state);
	if (ret)
		return ret;

	instance->target = clamp_val(state, instance->lower, instance->upper);
	mutex_lock(&cdev->lock);
	cdev->updated = false;
	mutex_unlock(&cdev->lock);
	thermal_cdev_update(cdev);

	return 0;
}

/**
 * divvy_up_power() - divvy the allocated power between the actors
 * @req_power:	each actor's requested power
 * @max_power:	each actor's maximum available power
 * @num_actors:	size of the @req_power, @max_power and @granted_power's array
 * @total_req_power: sum of @req_power
 * @power_range:	total allocated power
 * @granted_power:	output array: each actor's granted power
 * @extra_actor_power:	an appropriately sized array to be used in the
 *			function as temporary storage of the extra power given
 *			to the actors
 *
 * This function divides the total allocated power (@power_range)
 * fairly between the actors.  It first tries to give each actor a
 * share of the @power_range according to how much power it requested
 * compared to the rest of the actors.  For example, if only one actor
 * requests power, then it receives all the @power_range.  If
 * three actors each requests 1mW, each receives a third of the
 * @power_range.
 *
 * If any actor received more than their maximum power, then that
 * surplus is re-divvied among the actors based on how far they are
 * from their respective maximums.
 *
 * Granted power for each actor is written to @granted_power, which
 * should've been allocated by the calling function.
 */
static void divvy_up_power(u32 *req_power, u32 *max_power, int num_actors,
			   u32 total_req_power, u32 power_range,
			   u32 *granted_power, u32 *extra_actor_power)
{
	u32 extra_power, capped_extra_power;
	int i;

	/*
	 * Prevent division by 0 if none of the actors request power.
	 */
	if (!total_req_power)
		total_req_power = 1;

	capped_extra_power = 0;
	extra_power = 0;
	for (i = 0; i < num_actors; i++) {
		u64 req_range = (u64)req_power[i] * power_range;

		granted_power[i] = DIV_ROUND_CLOSEST_ULL(req_range,
							 total_req_power);

		if (granted_power[i] > max_power[i]) {
			extra_power += granted_power[i] - max_power[i];
			granted_power[i] = max_power[i];
		}

		extra_actor_power[i] = max_power[i] - granted_power[i];
		capped_extra_power += extra_actor_power[i];
	}

	if (!extra_power)
		return;

	/*
	 * Re-divvy the reclaimed extra among actors based on
	 * how far they are from the max
	 */
	extra_power = min(extra_power, capped_extra_power);
	if (capped_extra_power > 0) {
		for (i = 0; i < num_actors; i++) {
			u64 extra_range = (u64)extra_actor_power[i] * extra_power;

			granted_power[i] += DIV_ROUND_CLOSEST_ULL(extra_range,
							 capped_extra_power);
		}
	}
}

/**
 * pid_controller() - PID controller
 * @tz:	thermal zone we are operating in
 * @control_temp:	the target temperature in millicelsius
 * @max_allocatable_power:	maximum allocatable power for this thermal zone
 *
 * This PID controller increases the available power budget so that the
 * temperature of the thermal zone gets as close as possible to
 * @control_temp and limits the power if it exceeds it.  k_po is the
 * proportional term when we are overshooting, k_pu is the
 * proportional term when we are undershooting.  integral_cutoff is a
 * threshold below which we stop accumulating the error.  The
 * accumulated error is only valid if the requested power will make
 * the system warmer.  If the system is mostly idle, there's no point
 * in accumulating positive error.
 *
 * Return: The power budget for the next period.
 */
static u32 pid_controller(struct oplus_thermal_data *data,
			  int control_temp,
			  u32 max_allocatable_power)
{
	struct thermal_zone_device *tz = data->tzd;
	struct oplus_ipa_param *params = data->ipa_param;
	s64 p, i, d, power_range;
	s32 err, max_power_frac;
	u32 sustainable_power;

	max_power_frac = int_to_frac(max_allocatable_power);

	sustainable_power = params->sustainable_power;

	err = control_temp - tz->temperature;
	err = int_to_frac(err);

	/* Calculate the proportional term */
	p = mul_frac(err < 0 ? params->k_po : params->k_pu, err);

	/*
	 * Calculate the integral term
	 *
	 * if the error is less than cut off allow integration (but
	 * the integral is limited to max power)
	 */
	i = mul_frac(params->k_i, params->err_integral);
	if (err < int_to_frac(params->integral_cutoff)) {
		s64 i_next = i + mul_frac(params->k_i, err);

		if (abs(i_next) < max_power_frac) {
			i = i_next;
			params->err_integral += err;
		}
	}

	/*
	 * Calculate the derivative term
	 *
	 * We do err - prev_err, so with a positive k_d, a decreasing
	 * error (i.e. driving closer to the line) results in less
	 * power being applied, slowing down the controller)
	 */
	d = mul_frac(params->k_d, err - params->prev_err);
	d = div_frac(d, jiffies_to_msecs(tz->passive_delay_jiffies));
	params->prev_err = err;

	power_range = p + i + d;

	/* feed-forward the known sustainable dissipatable power */
	power_range = sustainable_power + frac_to_int(power_range);

	power_range = clamp(power_range, (s64)0, (s64)max_allocatable_power);

	trace_thermal_power_allocator_pid(tz, frac_to_int(err),
					  frac_to_int(params->err_integral),
					  frac_to_int(p), frac_to_int(i),
					  frac_to_int(d), power_range);

	pr_debug("thermal err:%d,err_integral:%lld, p:%lld, i:%lld, d:%lld\r\n",
					  frac_to_int(err), frac_to_int(params->err_integral),
					  frac_to_int(p), frac_to_int(i), frac_to_int(d));

	return power_range;
}

struct cpufreq_cooling_device {
        u32 last_load;
        unsigned int cpufreq_state;
        unsigned int max_level;
        struct em_perf_domain *em;
        struct cpufreq_policy *policy;
#ifndef CONFIG_SMP
        struct time_in_idle *idle_time;
#endif
        struct freq_qos_request qos_req;
};

static int oplus_ipa_controller(struct oplus_thermal_data *data, int control_temp)
{
	struct thermal_zone_device *tz = data->tzd;
	struct oplus_ipa_param *params = data->ipa_param;
	struct thermal_instance *instance;
	u32 *req_power, *max_power, *granted_power, *extra_actor_power;
	u32 *weighted_req_power;
	u32 total_req_power, max_allocatable_power, total_weighted_req_power;
	u32 total_granted_power, power_range;
	int i, j, num_actors, total_weight, ret = 0;
	int trip_max_desired_temperature = params->trip_control_temp;
	int power_buget = params->power_buget;
	bool use_power_budget = params->use_power_budget;
	bool use_thermalcontrol_limit = params->use_thermalcontrol_limit;
	int power_gap;
	bool relarge_ganted_power;

	mutex_unlock(&data->lock);
	mutex_lock(&tz->lock);

	num_actors = 0;
	total_weight = 0;
	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		if ((instance->trip == trip_max_desired_temperature) &&
		    cdev_is_power_actor(instance->cdev)) {
			num_actors++;
			total_weight += instance->weight;
		}
	}

	if (!num_actors) {
		ret = -ENODEV;
		goto unlock;
	}

	/*
	 * We need to allocate five arrays of the same size:
	 * req_power, max_power, granted_power, extra_actor_power and
	 * weighted_req_power.  They are going to be needed until this
	 * function returns.  Allocate them all in one go to simplify
	 * the allocation and deallocation logic.
	 */
	BUILD_BUG_ON(sizeof(*req_power) != sizeof(*max_power));
	BUILD_BUG_ON(sizeof(*req_power) != sizeof(*granted_power));
	BUILD_BUG_ON(sizeof(*req_power) != sizeof(*extra_actor_power));
	BUILD_BUG_ON(sizeof(*req_power) != sizeof(*weighted_req_power));
	req_power = kcalloc(num_actors * 5, sizeof(*req_power), GFP_KERNEL);
	if (!req_power) {
		ret = -ENOMEM;
		goto unlock;
	}

	max_power = &req_power[num_actors];
	granted_power = &req_power[2 * num_actors];
	extra_actor_power = &req_power[3 * num_actors];
	weighted_req_power = &req_power[4 * num_actors];

	i = 0;
	total_weighted_req_power = 0;
	total_req_power = 0;
	max_allocatable_power = 0;

	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		int weight;
		struct thermal_cooling_device *cdev = instance->cdev;
		struct cpufreq_cooling_device *cpufreq_cdev = instance->cdev->devdata;
		int level;

		if (instance->trip != trip_max_desired_temperature)
			continue;

		if (!cdev_is_power_actor(cdev))
			continue;

		if (cdev->ops->get_requested_power(cdev, &req_power[i]))
			continue;

		if (!total_weight)
			weight = 1 << FRAC_BITS;
		else
			weight = instance->weight;

		weighted_req_power[i] = frac_to_int(weight * req_power[i]);


		if (use_thermalcontrol_limit && data->ipa_param->clt_state_limit[i] > 0 && data->ipa_param->sustainable_power < CPU_POWER_MAX) {
			for(level = cpufreq_cdev->max_level; level > 0; level--) {
				if(data->ipa_param->clt_state_limit[i] >= cpufreq_cdev->em->table[level].frequency)
					break;
			}
			if (cdev->ops->state2power(cdev, (cpufreq_cdev->max_level - level),
						   &max_power[i])) {
				continue;
			}
		} else {
			if (cdev->ops->state2power(cdev, instance->lower,
						&max_power[i]))
				continue;
		}

		total_req_power += req_power[i];
		max_allocatable_power += max_power[i];
		total_weighted_req_power += weighted_req_power[i];

		i++;
	}

	power_range = pid_controller(data, control_temp, max_allocatable_power);
	trace_android_vh_thermal_power_cap(&power_range);
	divvy_up_power(weighted_req_power, max_power, num_actors,
		       total_weighted_req_power, power_range, granted_power,
		       extra_actor_power);

	total_granted_power = 0;
	i = 0;
	relarge_ganted_power = false;
	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		struct cpufreq_cooling_device *cpufreq_cdev = instance->cdev->devdata;
		struct cpufreq_policy *policy = cpufreq_cdev->policy;
		u32 ncpus, last_load;
		if (instance->trip != trip_max_desired_temperature)
			continue;

		if (!cdev_is_power_actor(instance->cdev))
			continue;

		if (params->cdev_min_power[i] > granted_power[i]) {
			granted_power[i] = params->cdev_min_power[i];
			pr_debug("thermal: type: %s granted_power[%d]: %u params->cdev_min_power[%d]=%u\n", instance->cdev->type, i, granted_power[i], i, params->cdev_min_power[i]);
		}
		ncpus = cpumask_weight(policy->related_cpus);
		if(ncpus == 0) {
			continue;
		}
		last_load = cpufreq_cdev->last_load;
		if(last_load == 0) {
			last_load = 1;
		}

		if(use_power_budget) {
			if(last_load / ncpus > 50) {
				relarge_ganted_power = true;
				if(power_buget > 0) {
					if(req_power[i] > granted_power[i]) {
						power_buget = power_buget - (req_power[i] * 6 / 5 - granted_power[i]);
						granted_power[i] = req_power[i] * 6 / 5;
					} else {
						granted_power[i] = granted_power[i] * 5 / 4;
						power_buget = power_buget -  granted_power[i] * 1 / 2;
					}
					pr_debug("thermal power_gap req_power: %d granted_power=%d i=%d, power_budget:%d\n", req_power[i], granted_power[i], i, power_buget);
				}
			}
		}
		total_granted_power += granted_power[i];

		granted_power[i] = granted_power[i] / ncpus;

		power_actor_set_power(instance->cdev, instance, (granted_power[i] * last_load + 100) /100);

		i++;
	}

	if(use_power_budget) {
		if(!relarge_ganted_power) {
			power_gap = total_req_power - power_range;
			if((power_gap > 0) && (power_buget > 0)) {
				power_buget = power_buget - power_gap;
			} else {
				if ((power_gap < 0 && power_buget < MAX_POWER_BUDGET) || (power_gap > 0 && power_buget > MIN_POWER_BUDGET)) {
					power_buget = power_buget - power_gap;
				}
			}
			pr_debug("thermal power_gap res:%d,power_range:%d,power_budget:%d\r\n",
						power_gap, power_range, power_buget);
		}
		mutex_lock(&data->lock);
		params->power_buget = power_buget;
		mutex_unlock(&data->lock);
	}

	trace_thermal_power_allocator(tz, req_power, total_req_power,
                                      granted_power, total_granted_power,
                                      num_actors, power_range,
                                      max_allocatable_power, tz->temperature,
                                      control_temp - tz->temperature);

	for(j = 0; j < num_actors; j++) {
		pr_debug("thermal cluster[%d] req_power:%d, granted_power:%d\n", j, req_power[j], granted_power[j]);
	}
	pr_debug("thermal total=%d total_weighted=%d, total_granted=%d power_range=%d max_alloc=%d\n", total_req_power, total_weighted_req_power,
								total_granted_power, power_range, max_allocatable_power);

	kfree(req_power);
unlock:
	mutex_unlock(&tz->lock);
	mutex_lock(&data->lock);

	return ret;
}

int oplus_thermal_zone_device_is_enabled(struct thermal_zone_device *tz)
{
	enum thermal_device_mode mode;
	mutex_lock(&tz->lock);
	mode = tz->mode;
	mutex_unlock(&tz->lock);
	return mode == THERMAL_DEVICE_ENABLED;
}

static void oplus_ipa_thermal(struct oplus_thermal_data *data)
{
	struct thermal_zone_device *tz = data->tzd;
	struct oplus_ipa_param *params = data->ipa_param;
	int ret = 0;
	int switch_on_temp, control_temp, delay;

	if (atomic_read(&oplus_thermal_in_suspend))
		return;

	if (tz) {
		if (params->sustainable_power >= CPU_POWER_MAX) {
			mutex_lock(&data->lock);
			goto polling;
		}

		if (!oplus_thermal_zone_device_is_enabled(tz)) {
			mutex_lock(&data->lock);
			reset_ipa_params(data);
			allow_maximum_power(data);
			params->switched_on = false;
			goto polling;
		}
	}

	mutex_lock(&tz->lock);
	tz->last_temperature = tz->temperature;
	tz->temperature = get_current_shell_temp();
	mutex_unlock(&tz->lock);


	mutex_lock(&data->lock);

	ret = tz->ops->get_trip_temp(tz, params->trip_switch_on,
				     &switch_on_temp);

	if (!ret && tz->temperature < switch_on_temp) {
		reset_ipa_params(data);
		allow_maximum_power(data);
		params->switched_on = false;
		goto polling;
	}

	params->switched_on = true;

	ret = tz->ops->get_trip_temp(tz, params->trip_control_temp,
				     &control_temp);
	if (ret) {
		pr_warn("Failed to get the maximum desired temperature: %d\n",
			ret);
		goto polling;
	}

	ret = oplus_ipa_controller(data, control_temp);

	if (ret) {
		pr_debug("Failed to calculate ipa controller: %d\n",
			 ret);
		goto polling;
	}

polling:
	if (params->switched_on)
		delay = params->polling_delay_on;
	else
		delay = params->polling_delay_off;

	if (delay)
		start_ipa_polling(data, delay);

	mutex_unlock(&data->lock);
}

static void oplus_ipa_polling(struct kthread_work *work)
{
	struct oplus_thermal_data *data =
			container_of(work, struct oplus_thermal_data,
				     ipa_work.work);

	pr_debug("oplus_ipa_polling start.\n");
	oplus_ipa_thermal(data);
}

static int oplus_thermal_pm_notify(struct notifier_block *nb,
			       unsigned long mode, void *_unused)
{
	struct oplus_thermal_data *data;

	switch (mode) {
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
	case PM_SUSPEND_PREPARE:
		atomic_set(&oplus_thermal_in_suspend, 1);
		list_for_each_entry(data, &thermal_instance_list, node) {
			if (data->use_ipa_thermal)
				kthread_cancel_delayed_work_sync(&data->ipa_work);
		}
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		atomic_set(&oplus_thermal_in_suspend, 0);
		list_for_each_entry(data, &thermal_instance_list, node) {
			if (data->use_ipa_thermal)
				start_ipa_polling(data, 0);
		}
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block oplus_thermal_pm_nb = {
	.notifier_call = oplus_thermal_pm_notify,
};

static const struct of_device_id oplus_thermal_match[] = {
	{ .compatible = "oplus,oplus-thermal", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, oplus_thermal_match);

static int oplus_thermal_work_init(struct platform_device *pdev)
{
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);
	struct cpumask mask;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO / 4 - 1 };
	struct task_struct *thread;
	int ret = 0;

	kthread_init_worker(&data->thermal_worker);
	thread = kthread_create(kthread_worker_fn, &data->thermal_worker,
				"thermal_%s", data->thermal_name);
	if (IS_ERR(thread)) {
		dev_err(&pdev->dev, "failed to create thermal thread: %ld\n",
			PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	cpumask_and(&mask, cpu_possible_mask, &data->thermal_work_affinity);
	set_cpus_allowed_ptr(thread, &mask);

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		dev_warn(&pdev->dev, "thermal failed to set SCHED_FIFO\n");
		return ret;
	}

	wake_up_process(thread);

	return ret;
}

static int oplus_map_dt_data(struct platform_device *pdev)
{
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);
	const char *thermal_name, *buf;
	int ret;

	if (!data || !pdev->dev.of_node)
		return -ENODEV;

	data->np = pdev->dev.of_node;

	if (of_property_read_u32(pdev->dev.of_node, "id", &data->id)) {
		dev_err(&pdev->dev, "Failed to get THERMAL ID\n");
		return -ENODEV;
	}

	if (of_property_read_string(pdev->dev.of_node, "thermal_name", &thermal_name))
		dev_err(&pdev->dev, "Failed to get thermal_name\n");
	else
		strncpy(data->thermal_name, thermal_name, THERMAL_NAME_LENGTH);

	ret = of_property_read_string(pdev->dev.of_node, "thermal_work_affinity", &buf);
	if (!ret)
		cpulist_parse(buf, &data->thermal_work_affinity);

	if (of_property_read_bool(pdev->dev.of_node, "use-ipa-thermal")) {
		struct oplus_ipa_param *params;
		u32 value, i;

		data->use_ipa_thermal = true;

		params = kzalloc(sizeof(*params), GFP_KERNEL);
		if (!params)
			return -ENOMEM;

		of_property_read_u32(pdev->dev.of_node, "polling_delay_on",
				     &params->polling_delay_on);
		if (!params->polling_delay_on)
			dev_err(&pdev->dev, "No input polling_delay_on\n");

		of_property_read_u32(pdev->dev.of_node, "polling_delay_off",
				     &params->polling_delay_off);

		ret = of_property_read_u32(pdev->dev.of_node, "k_po",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input k_po\n");
		else
			params->k_po = value;

		ret = of_property_read_u32(pdev->dev.of_node, "k_pu",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input k_pu\n");
		else
			params->k_pu = value;

		ret = of_property_read_u32(pdev->dev.of_node, "k_i",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input k_i\n");
		else
			params->k_i = value;

		ret = of_property_read_u32(pdev->dev.of_node, "k_d",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input k_d\n");
		else
			params->k_d = value;

		ret = of_property_read_u32(pdev->dev.of_node, "i_max",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input i_max\n");
		else
			params->i_max = value;

		ret = of_property_read_u32(pdev->dev.of_node, "integral_cutoff",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input integral_cutoff\n");
		else
			params->integral_cutoff = value;

		ret = of_property_read_u32(pdev->dev.of_node, "sustainable_power",
					   &value);
		if (ret < 0)
			dev_err(&pdev->dev, "No input sustainable_power\n");
		else
			params->sustainable_power = value;

		ret = of_property_read_u32_array(pdev->dev.of_node, "cdev_min_power", params->cdev_min_power, CDEV_GRANT_NUM);
		if (ret < 0)
			dev_err(&pdev->dev, "No input cdev_min_power\n");

		ret = of_property_read_u32(pdev->dev.of_node, "use-power-budget",
					   &value);
		if (ret < 0) {
			params->use_power_budget = false;
			dev_err(&pdev->dev, "No input use-power-budget\n");
		} else {
			if(value > 0) {
				params->use_power_budget = true;
			} else {
				params->use_power_budget = false;
			}
		}

		ret = of_property_read_u32(pdev->dev.of_node, "use-thermalcontrol-limit",
					   &value);
		if (ret < 0) {
			params->use_thermalcontrol_limit = false;
			dev_err(&pdev->dev, "No input use-power-budget\n");
		} else {
			if(value > 0) {
				params->use_thermalcontrol_limit = true;
			} else {
				params->use_thermalcontrol_limit = false;
			}
		}

		ret = of_property_read_u32_array(pdev->dev.of_node, "clt_state_limit", params->clt_state_limit, CLUSTER_NUM);
		if (ret < 0) {
			for(i = 0; i < CLUSTER_NUM; i++) {
				params->clt_state_limit[i] = 0;
			}
		}

		params->power_buget = 0;

		data->ipa_param = params;
	} else {
		data->use_ipa_thermal = false;
	}

	return 0;
}

static const struct thermal_zone_device_ops oplus_sensor_ops = {
	.get_temp = oplus_get_temp,
};

static ssize_t
cdev_min_power_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);

	if (data->ipa_param) {
		char *p = buf;
		int i;
		for (i=0; i < CDEV_GRANT_NUM; i++) {
			p += sprintf(p, "%d ", data->ipa_param->cdev_min_power[i]);
		}
		return p - buf;
	}
	else
		return -EIO;
}

static ssize_t
sustainable_power_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);

	if (data->ipa_param)
		return sprintf(buf, "%u\n", data->ipa_param->sustainable_power);
	else
		return -EIO;
}



static ssize_t
sustainable_power_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);
	u32 sustainable_power;

	if (!data->ipa_param)
		return -EIO;

	if (kstrtou32(buf, 10, &sustainable_power))
		return -EINVAL;


	if (data->use_ipa_thermal) {
		mutex_lock(&data->lock);
		data->ipa_param->sustainable_power = sustainable_power;
		if (data->ipa_param->sustainable_power >= CPU_POWER_MAX) {
			allow_maximum_power(data);
			reset_ipa_params(data);
			dev_err(&pdev->dev, "thermal: allow_maximum_power:%u\n", data->ipa_param->sustainable_power);
		}
		mutex_unlock(&data->lock);

		start_ipa_polling(data, 0);
	}

	return count;
}

static ssize_t
polling_delay_on_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);

	if (data->ipa_param)
		return sprintf(buf, "%u\n", data->ipa_param->polling_delay_on);
	else
		return -EIO;
}

static ssize_t
polling_delay_on_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);
	u32 polling_delay_on;

	if (!data->ipa_param)
		return -EIO;

	if (kstrtou32(buf, 10, &polling_delay_on))
		return -EINVAL;

	data->ipa_param->polling_delay_on = polling_delay_on;

	/*
	 * This sysfs node is mainly used for debugging and could race with
	 * suspend/resume path as we don't use a lock to avoid it. The race
	 * could cause ipa-polling work re-queued after suspend so the pid
	 * sample time might not run as our expectation. Please do NOT use
	 * this for the production line.
	 */
	if (data->use_ipa_thermal) {
		WARN(1, "could potentially race with suspend/resume path!");
		start_ipa_polling(data, 0);
	}

	return count;
}

static ssize_t
polling_delay_off_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);

	if (data->ipa_param)
		return sprintf(buf, "%u\n", data->ipa_param->polling_delay_off);
	else
		return -EIO;
}

static ssize_t
polling_delay_off_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);
	u32 polling_delay_off;

	if (!data->ipa_param)
		return -EIO;

	if (kstrtou32(buf, 10, &polling_delay_off))
		return -EINVAL;

	data->ipa_param->polling_delay_off = polling_delay_off;

	/*
	 * This sysfs node is mainly used for debugging and could race with
	 * suspend/resume path as we don't use a lock to avoid it. The race
	 * could cause ipa-polling work re-queued after suspend so the pid
	 * sample time might not run as our expectation. Please do NOT use
	 * this for the production line.
	 */
	if (data->use_ipa_thermal) {
		WARN(1, "could potentially race with suspend/resume path!");
		start_ipa_polling(data, 0);
	}

	return count;
}

static ssize_t
clt_state_limit_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	char *ptr;
	int i;
	struct platform_device *pdev = to_platform_device(dev);
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);

	if (data == NULL || data->ipa_param == NULL) {
		return -EIO;
	}

	ptr = buf;
	for(i = 0; i < CLUSTER_NUM; i++) {
		ptr += sprintf(ptr, "%d ", data->ipa_param->clt_state_limit[i]);
	}

	pr_debug("clt_state_limit_show: %s\n", buf);
	return ptr - buf;
}

static ssize_t
clt_state_limit_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	int values[CLUSTER_NUM], i, ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);

	if (!data || !data->ipa_param)
		return -EIO;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	ret = sscanf(buf, "%d %d %d %d %d %d ", &values[0], &values[1], &values[2],
			 &values[3], &values[4], &values[5]);
#else
	ret = sscanf(buf, "%d %d %d %d %d %d ", &values[3], &values[2], &values[0],
			 &values[1], &values[4], &values[5]);
#endif
	if (values[1] == 0) {
		ret = sscanf(buf, "%d %d %d %d %d %d ", &values[2], &values[1], &values[0],
				  &values[3], &values[4], &values[5]);
	}

	if (ret != CLUSTER_NUM) {
		dev_err(dev, "Invalid input format: %s\n", buf);
		return -EINVAL;
	}
	pr_debug("clt_state_limit_store: %s\n", buf);
	mutex_lock(&data->lock);
	for (i = 0; i < CLUSTER_NUM; i++) {
		data->ipa_param->clt_state_limit[i] = values[i];
	}
	mutex_unlock(&data->lock);

	return count;
}

#define create_s32_param_attr(name)						\
	static ssize_t								\
	name##_show(struct device *dev, struct device_attribute *devattr,	\
		    char *buf)							\
	 {									\
		struct platform_device *pdev = to_platform_device(dev);			\
		struct oplus_thermal_data *data = platform_get_drvdata(pdev);		\
											\
		if (data->ipa_param)							\
			return sprintf(buf, "%d\n", data->ipa_param->name);		\
		else									\
			return -EIO;							\
	}									\
										\
	static ssize_t								\
	name##_store(struct device *dev, struct device_attribute *devattr,	\
		     const char *buf, size_t count)				\
	 {									\
		struct platform_device *pdev = to_platform_device(dev);		\
		struct oplus_thermal_data *data = platform_get_drvdata(pdev);	\
		s32 value;							\
										\
		if (!data->ipa_param)						\
			return -EIO;						\
										\
		if (kstrtos32(buf, 10, &value))					\
			return -EINVAL;						\
										\
		data->ipa_param->name = value;					\
										\
		return count;							\
	}									\
	static DEVICE_ATTR_RW(name)

static DEVICE_ATTR_RW(sustainable_power);
static DEVICE_ATTR_RW(polling_delay_off);
static DEVICE_ATTR_RW(polling_delay_on);
static DEVICE_ATTR_RO(cdev_min_power);
static DEVICE_ATTR_RW(clt_state_limit);
create_s32_param_attr(k_po);
create_s32_param_attr(k_pu);
create_s32_param_attr(k_i);
create_s32_param_attr(k_d);
create_s32_param_attr(i_max);
create_s32_param_attr(integral_cutoff);
create_s32_param_attr(use_power_budget);
create_s32_param_attr(use_thermalcontrol_limit);

static struct attribute *oplus_thermal_attrs[] = {
	&dev_attr_polling_delay_off.attr,
	&dev_attr_polling_delay_on.attr,
	&dev_attr_sustainable_power.attr,
	&dev_attr_cdev_min_power.attr,
	&dev_attr_k_po.attr,
	&dev_attr_k_pu.attr,
	&dev_attr_k_i.attr,
	&dev_attr_k_d.attr,
	&dev_attr_i_max.attr,
	&dev_attr_integral_cutoff.attr,
	&dev_attr_use_power_budget.attr,
	&dev_attr_clt_state_limit.attr,
	&dev_attr_use_thermalcontrol_limit.attr,
	NULL,
};

static const struct attribute_group oplus_thermal_attr_group = {
	.attrs = oplus_thermal_attrs,
};

static int oplus_thermal_probe(struct platform_device *pdev)
{
	struct oplus_thermal_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(struct oplus_thermal_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	mutex_init(&data->lock);

	ret = oplus_map_dt_data(pdev);
	if (ret)
		goto err_sensor;

	data->tzd = devm_thermal_of_zone_register(&pdev->dev, 0, data,
						    &oplus_sensor_ops);
	if (IS_ERR(data->tzd)) {
		ret = PTR_ERR(data->tzd);
		dev_err(&pdev->dev, "Failed to register sensor: %d\n", ret);
		goto err_sensor;
	}

	thermal_zone_device_disable(data->tzd);

	ret = oplus_thermal_work_init(pdev);
	if (ret) {
		dev_err(&pdev->dev, "cannot initialize oplus interrupt work\n");
		goto err_thermal;
	}

	if (data->use_ipa_thermal)
		kthread_init_delayed_work(&data->ipa_work, oplus_ipa_polling);

	oplus_thermal_control(pdev, true);

	ret = sysfs_create_group(&pdev->dev.kobj, &oplus_thermal_attr_group);
	if (ret)
		dev_err(&pdev->dev, "cannot create oplus thermal attr group");
	mutex_lock(&data->lock);
	list_add_tail(&data->node, &thermal_instance_list);
	num_of_devices++;
	mutex_unlock(&data->lock);

	if (data->use_ipa_thermal) {
		reset_ipa_trips(data);
		reset_ipa_params(data);
		start_ipa_polling(data, 0);
	}

	if (list_is_singular(&thermal_instance_list))
		register_pm_notifier(&oplus_thermal_pm_nb);

	return 0;

err_thermal:
	devm_thermal_of_zone_unregister(&pdev->dev, data->tzd);
err_sensor:
	return ret;
}

static int oplus_thermal_remove(struct platform_device *pdev)
{
	struct oplus_thermal_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tzd = data->tzd;
	struct oplus_thermal_data *devnode;

	devm_thermal_of_zone_unregister(&pdev->dev, tzd);
	oplus_thermal_control(pdev, false);

	mutex_lock(&data->lock);
	list_for_each_entry(devnode, &thermal_instance_list, node) {
		if (devnode->id == data->id) {
			list_del(&devnode->node);
			num_of_devices--;
			break;
		}
	}
	mutex_unlock(&data->lock);

	return 0;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int oplus_thermal_suspend(struct device *dev)
{
	return 0;
}

static int oplus_thermal_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(oplus_thermal_pm,
			 oplus_thermal_suspend, oplus_thermal_resume);
#define OPLUS_THERMAL_PM	(&oplus_thermal_pm)
#else
#define OPLUS_THERMAL_PM	NULL
#endif

static struct platform_driver oplus_thermal_driver = {
	.driver = {
		.name   = "oplus-thermal",
		.pm     = OPLUS_THERMAL_PM,
		.of_match_table = oplus_thermal_match,
		.suppress_bind_attrs = true,
	},
	.probe = oplus_thermal_probe,
	.remove	= oplus_thermal_remove,
};

module_platform_driver(oplus_thermal_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("oplus thermal driver");
