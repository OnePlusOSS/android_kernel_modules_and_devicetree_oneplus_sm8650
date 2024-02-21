// SPDX-License-Identifier: GPL-2.0
#ifndef __OPLUS_CAM_KEVENT_FB__
#define __OPLUS_CAM_KEVENT_FB__

#define MAX_ID 19
#define MAX_BUF_LEN 512
#define PAYLOAD_LENGTH 256
#define RECORD_KB_EVENT_TIME 30 // 30s
#define CAM_OLC_EXCEPTION_HEADER_ID 0x10025000
#define QCOM_EXCEPTION_HARDWARE_MODULE 8 //mtk have used 0-6; qcom use begin 8

typedef enum {
	EXCEP_PROBE,
	EXCEP_CLOCK,
	EXCEP_VOLTAGE,
	EXCEP_GPIO,
	EXCEP_I2C,
	EXCEP_SOF_TIMEOUT,
	EXCEP_CRC,
	EXCEP_ACTUATOR,
	EXCEP_EEPROM,
	MAX_EXCEP_TYPE
} cam_excep_type;

struct cam_fb_conf {
	int excepId;
	unsigned char *fb_field;
	unsigned char *fb_event_id;
	long long record_time;
};

int cam_event_proc_init(void);
void cam_event_proc_exit(void);
int cam_olc_raise_exception(int excep_tpye, unsigned char *pay_load);
const unsigned char *acquire_event_field(int excepId);
#define KEVENT_FB_SNESOR_WR_FAILED(connext, msg, err)                                 \
	do {                                                                             \
		snprintf(                                                                \
			connext, sizeof(connext),                                        \
			"FBField@@%s$$ExceptId@@0x%x$$detailData@@ErrMsg=%s,SensorId=%d", \
			acquire_event_field(EXCEP_I2C), EXCEP_I2C, msg, err);            \
		cam_olc_raise_exception(EXCEP_I2C, connext);                             \
	} while (0)

#define KEVENT_FB_SNESOR_PROBE_FAILED(connext, msg, sensor)                               \
	do {                                                                              \
		snprintf(                                                                 \
			connext, sizeof(connext),                                         \
			"FBField@@%s$$ExceptId@@0x%x$$detailData@@ErrMsg=%s,SensorId=%d", \
			acquire_event_field(EXCEP_PROBE), EXCEP_PROBE, msg,               \
			sensor);                                                          \
		cam_olc_raise_exception(EXCEP_PROBE, connext);                            \
	} while (0)

#define KEVENT_FB_EEPRPOM_WR_FAILED(connext, msg, err)                                   \
	do {                                                                             \
		snprintf(                                                                \
			connext, sizeof(connext),                                        \
			"FBField@@%s$$ExceptId@@0x%x$$detailData@@ErrMsg=%s,ErrCode=%d", \
			acquire_event_field(EXCEP_EEPROM), EXCEP_EEPROM, msg, err);            \
		cam_olc_raise_exception(EXCEP_EEPROM, connext);                             \
	} while (0)

#define KEVENT_FB_ACTUATOR_CTL_FAILED(connext, msg, err)                                 \
	do {                                                                             \
		snprintf(                                                                \
			connext, sizeof(connext),                                        \
			"FBField@@%s$$ExceptId@@0x%x$$detailData@@ErrMsg=%s,ErrCode=%d", \
			acquire_event_field(EXCEP_ACTUATOR), EXCEP_ACTUATOR, msg, err);            \
		cam_olc_raise_exception(EXCEP_ACTUATOR, connext);                             \
	} while (0)

#define KEVENT_FB_CRC_FAILED(connext, msg, err)                                 \
	do {                                                                             \
		snprintf(                                                                \
			connext, sizeof(connext),                                        \
			"FBField@@%s$$ExceptId@@0x%x$$detailData@@ErrMsg=%s,ErrCode=%d", \
			acquire_event_field(EXCEP_CRC), EXCEP_CRC, msg, err);            \
		cam_olc_raise_exception(EXCEP_CRC, connext);                             \
	} while (0)

#define KEVENT_FB_FRAME_ERROR(connext, msg, err)                                 \
	do {                                                                             \
		snprintf(                                                                \
			connext, sizeof(connext),                                        \
			"FBField@@%s$$ExceptId@@0x%x$$detailData@@ErrMsg=%s,ErrCode=%d", \
			acquire_event_field(EXCEP_SOF_TIMEOUT), EXCEP_SOF_TIMEOUT, msg, err);            \
		cam_olc_raise_exception(EXCEP_SOF_TIMEOUT, connext);                             \
	} while (0)
#endif /*__OPLUS_CAM_KEVENT_FB__*/

