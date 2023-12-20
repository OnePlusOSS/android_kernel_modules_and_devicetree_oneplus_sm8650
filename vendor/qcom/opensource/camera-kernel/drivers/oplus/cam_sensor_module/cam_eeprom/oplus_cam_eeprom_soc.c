/********************************************************************
 * Copyright (c) 2020-2030 OPLUS Mobile Comm Corp.,Ltd. All Rights Reserved.
 * OPLUS_FEATURE_CAMERA_COMMON
 * File                  : oplus_cam_eeprom_soc.c
 * Version               : 1.0
 * Description           : add some oem code modifications for cam_eeprom_soc.c.
 * Date                  : 2023-07-10
 * -------------Rivision History-----------------------------------------
 * <version>      <date>       <author>       <desc>
 * 1.0            2023-07-10   zhangzhanzhao    add for engineermode
 * OPLUS Mobile Comm Proprietary and Confidential.
 *********************************************************************/


#include <linux/of.h>
#include <linux/of_gpio.h>
#include <cam_sensor_cmn_header.h>
#include <cam_sensor_util.h>
#include <cam_sensor_io.h>
#include <cam_req_mgr_util.h>

#include "cam_eeprom_soc.h"
#include "cam_debug_util.h"

#include "oplus_cam_eeprom_soc.h"


struct mutex actuator_ois_eeprom_shared_mutex;
bool actuator_ois_eeprom_shared_mutex_init_flag = false;

void cam_eeprom_parse_dt_oem(struct cam_eeprom_ctrl_t *e_ctrl, struct device_node *of_node) {
	int id = 0;
	int ret = 0;

	ret = of_property_read_u32(of_node, "actuator_ois_eeprom_merge", &id);
	if (ret) {
		e_ctrl->actuator_ois_eeprom_merge_flag = 0;
		CAM_DBG(CAM_OIS, "get actuator_ois_eeprom_merge_flag failed rc:%d, default %d", ret, e_ctrl->actuator_ois_eeprom_merge_flag);
	} else {
		e_ctrl->actuator_ois_eeprom_merge_flag = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read actuator_ois_eeprom_merge_flag success, value:%d", e_ctrl->actuator_ois_eeprom_merge_flag);

		e_ctrl->actuator_ois_eeprom_merge_mutex = &actuator_ois_eeprom_shared_mutex;
		if (!actuator_ois_eeprom_shared_mutex_init_flag) {
			mutex_init(e_ctrl->actuator_ois_eeprom_merge_mutex);
			actuator_ois_eeprom_shared_mutex_init_flag = true;
		}
	}
}

