/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _HBP_EXCEPTION_
#define _HBP_EXCEPTION_

typedef enum {
	/*0-99 for kernel*/
	EXCEP_HARDWARE = 0,
	EXCEP_GESTURE,
	EXCEP_GESTURE_READ,
	EXCEP_FINGERPRINT,
	EXCEP_FACE_DETECT,
	EXCEP_REPORT,
	EXCEP_PROBE,
	EXCEP_RESUME,
	EXCEP_SUSPEND,
	EXCEP_TEST_AUTO,
	EXCEP_TEST_BLACKSCREEN,
	EXCEP_BUS,
	EXCEP_ALLOC,
	EXCEP_FW_UPDATE,
	EXCEP_GRIP,
	EXCEP_IRQ,
	EXCEP_BUS_READY,
	/*100 for native touchpanel*/
	EXCEP_TOUCH_MONITOR = 100,
	EXCEP_TOUCH_STATE_MACHINE,
	EXCEP_INIT_STATE,
	EXCEP_ALGO_FRAME = 0xA0,
	EXCEP_ERROR = 255,
} hbp_excep_type;

#define SIG_SCREEN_ON_NO_ACK                     "sig_screen_on_no_ack"
#define SIG_SCREEN_OFF_NO_ACK                    "sig_screen_off_no_ack"


#define MAX_BUS_ERROR_COUNT                 15
#define MAX_BUS_UPDATE_COUNT                2
#define MAX_BUS_NOT_READY_UPDATE_COUNT      1

struct exception_data {
	void  *chip_data; /*debug info data*/
	unsigned int exception_upload_count;
	unsigned int bus_error_count;
	unsigned int bus_error_upload_count;
	unsigned int bus_not_ready_upload_count;
};

int hbp_tp_exception_report(void *tp_exception_data, hbp_excep_type excep_tpye, void *summary, unsigned int summary_size);

#endif /*_HBP_EXCEPTION_*/
