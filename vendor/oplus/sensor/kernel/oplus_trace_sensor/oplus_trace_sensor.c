#define pr_fmt(fmt) "<sensor_trace>" fmt

#include <linux/init.h>
#include <linux/module.h>
#include "oplus_trace_sensor.h"
#define CREATE_TRACE_POINTS
#include "trace_sensor_fault.h"
#include <soc/oplus/boot/oplus_project.h>

static uint16_t fault_list[] = {
	PS_INIT_FAIL_ID,
	PS_I2C_ERR_ID,
	PS_ESD_REST_ID,
	PS_NO_INTERRUPT_ID,
	ALS_INIT_FAIL_ID,
	ALS_I2C_ERR_ID,
	ALS_ESD_REST_ID,
	ALS_NO_INTERRUPT_ID,
	ACCEL_INIT_FAIL_ID,
	ACCEL_I2C_ERR_ID,
	ACCEL_ESD_REST_ID,
	ACCEL_NO_INTERRUPT_ID,
	ACCEL_ORIGIN_DATA_TO_ZERO_ID,
	ACCEL_DATA_BLOCK_ID,
	ACCEL_DATA_FULL_RANGE_ID,
	ACCEL_SUB_DATA_BLOCK_ID,
	GYRO_INIT_FAIL_ID,
	GYRO_I2C_ERR_ID,
	GYRO_ESD_REST_ID,
	GYRO_NO_INTERRUPT_ID,
	GYRO_ORIGIN_DATA_TO_ZERO_ID,
	GYRO_DATA_BLOCK_ID,
	GYRO_SUB_DATA_BLOCK_ID,
	MAG_INIT_FAIL_ID,
	MAG_I2C_ERR_ID,
	MAG_ESD_REST_ID,
	MAG_NO_INTERRUPT_ID,
	MAG_ORIGIN_DATA_TO_ZERO_ID,
	MAG_DATA_BLOCK_ID,
	MAG_DATA_FULL_RANGE_ID,
	MAG_NO_DATA_ID,
	MAG_ENABLE_FAIL_ID,
	SAR_INIT_FAIL_ID,
	SAR_I2C_ERR_ID,
	SAR_ESD_REST_ID,
	SAR_NO_INTERRUPT_ID,
	SAR_ORIGIN_DATA_TO_ZERO_ID,
	BAROMETER_I2C_ERR_ID,
	HALL_I2C_ERR_ID,
};

static long get_timestamp_ms(void)
{
	struct timespec64 now;
	ktime_get_real_ts64(&now);
	return timespec64_to_ns(&now) / NSEC_PER_MSEC;
}

int oplus_trace_sensor_fault_report(uint16_t event_id, char* fb_event_id, char* fb_field, uint32_t error_count)
{
	int i = 0;
	unsigned int version = get_eng_version();
	pr_err("oplus_trace_sensor_fault_report version =%d\n", version);
	if (version != PREVERSION) {
		return 0;
	}

	if (IS_ERR_OR_NULL((const void*)fb_event_id) || IS_ERR_OR_NULL((const void*)fb_field)) {
		return -1;
	}

	for (i = 0; i < sizeof(fault_list)/sizeof(uint16_t); i++) {
		if (fault_list[i] == event_id) {
			pr_info("trace event_id =%d, error_count =%d\n", event_id, error_count);
			trace_sensor_fault(get_timestamp_ms(), SENSOR_FAULT_APP_ID,
				SENSOR_FAULT_LOG_TAG, fb_event_id, fb_field, (int32_t)error_count);
			break;
		}
	}

	return 0;
}
EXPORT_SYMBOL(oplus_trace_sensor_fault_report);

int oplus_trace_sensor_crash_report(char* subsys)
{
	unsigned int version = get_eng_version();
	char* reason = subsys;
	pr_err("oplus_trace_sensor_crash_report version =%d\n", version);
	if (version != PREVERSION) {
		return 0;
	}

	if (IS_ERR_OR_NULL((const void*)reason)) {
		reason = "subsys_crash";
	}
	trace_sensor_fault(get_timestamp_ms(), SENSOR_FAULT_APP_ID,
		SENSOR_FAULT_LOG_TAG, SENSOR_STABILITY_TYPE, reason, (int32_t)1);
	return 0;
}
EXPORT_SYMBOL(oplus_trace_sensor_crash_report);

static int __init oplus_trace_sensor_init(void)
{
	pr_info("oplus_trace_sensor_init call\n");
	return 0;
}

static void __exit oplus_trace_sensor_exit(void)
{
	pr_info("oplus_trace_sensor_exit call\n");
}

module_init(oplus_trace_sensor_init);
module_exit(oplus_trace_sensor_exit);

MODULE_AUTHOR("Shirong.Long");
MODULE_LICENSE("GPL");
