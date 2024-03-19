// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */
#include <linux/module.h>
#include "cam_sensor_util.h"
#include "cam_sensor_dev.h"
#include "cam_sensor_soc.h"
#include "cam_sensor_core.h"
#include "oplus_cam_sensor_core.h"

bool is_ftm_current_test = false;
struct cam_sensor_i2c_reg_setting sensor_setting;

struct cam_sensor_settings sensor_init_settings = {
#include "cam_sensor_initsettings.h"
};

bool chip_version_old = FALSE;
int cam_ftm_power_down(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	CAM_ERR(CAM_SENSOR,"FTM stream off");
	if (s_ctrl->sensordata->slave_info.sensor_id == 0x0615||
		s_ctrl->sensordata->slave_info.sensor_id == 0x766 ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x766E||
		s_ctrl->sensordata->slave_info.sensor_id == 0x766F||
		s_ctrl->sensordata->slave_info.sensor_id == 0x709 ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x581||
		s_ctrl->sensordata->slave_info.sensor_id == 0x890||
		s_ctrl->sensordata->slave_info.sensor_id == 0x989 ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x888 ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x6442 ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x858 ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x9186||
		s_ctrl->sensordata->slave_info.sensor_id == 0xa18a ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x355 ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x8206||
		s_ctrl->sensordata->slave_info.sensor_id == 0x8202 ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x809)
	{
		sensor_setting.reg_setting = sensor_init_settings.streamoff.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.streamoff.addr_type;
		sensor_setting.data_type = sensor_init_settings.streamoff.data_type;
		sensor_setting.size = sensor_init_settings.streamoff.size;
		sensor_setting.delay = sensor_init_settings.streamoff.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info),&sensor_setting);
		if(rc < 0)
		{
			/* If the I2C reg write failed for the first section reg, send
			the result instead of keeping writing the next section of reg. */
			CAM_ERR(CAM_SENSOR, "FTM Failed to stream off setting,rc=%d.",rc);
		}
		else
		{
			CAM_ERR(CAM_SENSOR, "FTM successfully to stream off");
		}
	}
	rc = cam_sensor_power_down(s_ctrl);
	CAM_ERR(CAM_SENSOR, "FTM power down rc=%d",rc);
	return rc;
}

int cam_ftm_power_up(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	rc = cam_sensor_power_up(s_ctrl);
	CAM_ERR(CAM_SENSOR, "FTM power up sensor id 0x%x,result %d",s_ctrl->sensordata->slave_info.sensor_id,rc);
	if(rc < 0)
	{
		CAM_ERR(CAM_SENSOR, "FTM power up faild!");
		return rc;
	}
	is_ftm_current_test =true;
	if (s_ctrl->sensordata->slave_info.sensor_id == 0x0615)
	{
		uint32_t vendor_id =0;
		SensorRegWrite(s_ctrl,0x0A02,0x7F);//Specify OTP Page Address for Read
		SensorRegWrite(s_ctrl,0x0A00,0x01);//Turn ON OTP Read Mode
		camera_io_dev_read(
		&(s_ctrl->io_master_info),
		s_ctrl->sensordata->id_info.sensor_id_reg_addr,
		&vendor_id,s_ctrl->sensordata->id_info.sensor_addr_type,
		CAMERA_SENSOR_I2C_TYPE_BYTE,FALSE);
		CAM_ERR(CAM_SENSOR, "IMX615 vendor/fab id expect 0x%x read  0x%x",s_ctrl->sensordata->id_info.sensor_id,vendor_id);
		if(s_ctrl->sensordata->id_info.sensor_id <= 56 && vendor_id <= 56)
		{
			CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x Fab1/3",s_ctrl->sensordata->slave_info.sensor_id);
			sensor_setting.reg_setting = sensor_init_settings.imx615_setting.reg_setting;
			sensor_setting.addr_type = sensor_init_settings.imx615_setting.addr_type;
			sensor_setting.data_type = sensor_init_settings.imx615_setting.data_type;
			sensor_setting.size = sensor_init_settings.imx615_setting.size;
			sensor_setting.delay = sensor_init_settings.imx615_setting.delay;
			rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
		}
		else if(s_ctrl->sensordata->id_info.sensor_id > 56 && vendor_id > 56)
		{
			CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x Fab2",s_ctrl->sensordata->slave_info.sensor_id);
			sensor_setting.reg_setting = sensor_init_settings.imx615_setting_fab2.reg_setting;
			sensor_setting.addr_type = sensor_init_settings.imx615_setting_fab2.addr_type;
			sensor_setting.data_type = sensor_init_settings.imx615_setting_fab2.data_type;
			sensor_setting.size = sensor_init_settings.imx615_setting_fab2.size;
			sensor_setting.delay = sensor_init_settings.imx615_setting_fab2.delay;
			rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
		}
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x766 ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x766E ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x766F ||
		s_ctrl->sensordata->slave_info.sensor_id == 0x890)
	{
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.imx766_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.imx766_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.imx766_setting.data_type;
		sensor_setting.size = sensor_init_settings.imx766_setting.size;
		sensor_setting.delay = sensor_init_settings.imx766_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x989)
	{
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.imx989_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.imx989_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.imx989_setting.data_type;
		sensor_setting.size = sensor_init_settings.imx989_setting.size;
		sensor_setting.delay = sensor_init_settings.imx989_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x581)
	{
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.imx581_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.imx581_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.imx581_setting.data_type;
		sensor_setting.size = sensor_init_settings.imx581_setting.size;
		sensor_setting.delay = sensor_init_settings.imx581_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x709)
	{
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.imx709_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.imx709_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.imx709_setting.data_type;
		sensor_setting.size = sensor_init_settings.imx709_setting.size;
		sensor_setting.delay = sensor_init_settings.imx709_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x888)
	{
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.imx888_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.imx888_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.imx888_setting.data_type;
		sensor_setting.size = sensor_init_settings.imx888_setting.size;
		sensor_setting.delay = sensor_init_settings.imx888_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x6442)
	{
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.ov64b40_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.ov64b40_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.ov64b40_setting.data_type;
		sensor_setting.size = sensor_init_settings.ov64b40_setting.size;
		sensor_setting.delay = sensor_init_settings.ov64b40_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x858)
	{
		oplus_shift_sensor_mode(s_ctrl);
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.imx858_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.imx858_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.imx858_setting.data_type;
		sensor_setting.size = sensor_init_settings.imx858_setting.size;
		sensor_setting.delay = sensor_init_settings.imx858_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x9186)
	{
		oplus_shift_sensor_mode(s_ctrl);
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.lyt808_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.lyt808_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.lyt808_setting.data_type;
		sensor_setting.size = sensor_init_settings.lyt808_setting.size;
		sensor_setting.delay = sensor_init_settings.lyt808_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0xa18a)
	{
		oplus_shift_sensor_mode(s_ctrl);
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.imx06A_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.imx06A_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.imx06A_setting.data_type;
		sensor_setting.size = sensor_init_settings.imx06A_setting.size;
		sensor_setting.delay = sensor_init_settings.imx06A_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x355)
	{
		oplus_shift_sensor_mode(s_ctrl);
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.imx355_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.imx355_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.imx355_setting.data_type;
		sensor_setting.size = sensor_init_settings.imx355_setting.size;
		sensor_setting.delay = sensor_init_settings.imx355_setting.delay;
                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
        }
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x8206 ||
	    s_ctrl->sensordata->slave_info.sensor_id == 0x8202)
	{
		oplus_shift_sensor_mode(s_ctrl);
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.imx882_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.imx882_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.imx882_setting.data_type;
		sensor_setting.size = sensor_init_settings.imx882_setting.size;
		sensor_setting.delay = sensor_init_settings.imx882_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else if (s_ctrl->sensordata->slave_info.sensor_id == 0x809)
	{
		oplus_shift_sensor_mode(s_ctrl);
		CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		sensor_setting.reg_setting = sensor_init_settings.imx809_setting.reg_setting;
		sensor_setting.addr_type = sensor_init_settings.imx809_setting.addr_type;
		sensor_setting.data_type = sensor_init_settings.imx809_setting.data_type;
		sensor_setting.size = sensor_init_settings.imx809_setting.size;
		sensor_setting.delay = sensor_init_settings.imx809_setting.delay;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	}
	else
	{
		CAM_ERR(CAM_SENSOR, "FTM unknown sensor id 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		rc = -1;
	}
	if (rc < 0)
	{
		CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor setting");
		goto power_down;
	}
	else
	{
		CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor setting");
	}
	return rc;
power_down:
	CAM_ERR(CAM_SENSOR, "FTM wirte setting failed,do power down");
	cam_sensor_power_down(s_ctrl);
	return rc;
}

bool cam_ftm_if_do(void)
{
	CAM_DBG(CAM_SENSOR, "ftm state :%d",is_ftm_current_test);
	return is_ftm_current_test;
}

int32_t cam_sensor_update_id_info(struct cam_cmd_probe_v2 *probe_info,
	struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	s_ctrl->sensordata->id_info.sensor_slave_addr =
		probe_info->pipeline_delay;
	s_ctrl->sensordata->id_info.sensor_id_reg_addr =
		probe_info->reg_addr;
	s_ctrl->sensordata->id_info.sensor_id_mask =
		probe_info->data_mask;
	s_ctrl->sensordata->id_info.sensor_id =
		probe_info->expected_data;
	s_ctrl->sensordata->id_info.sensor_addr_type =
		probe_info->addr_type;
	s_ctrl->sensordata->id_info.sensor_data_type =
		probe_info->data_type;

	CAM_ERR(CAM_SENSOR,
		"vendor_slave_addr:  0x%x, vendor_id_Addr: 0x%x, vendorID: 0x%x, vendor_mask: 0x%x",
		s_ctrl->sensordata->id_info.sensor_slave_addr,
		s_ctrl->sensordata->id_info.sensor_id_reg_addr,
		s_ctrl->sensordata->id_info.sensor_id,
		s_ctrl->sensordata->id_info.sensor_id_mask);
	return rc;
}

int cam_sensor_change_wideaf_to_sleep_mode(struct cam_sensor_ctrl_t *s_ctrl)
{
	int                                rc = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_settings = {0};
	struct cam_sensor_i2c_reg_array    i2c_reg_array = {0};
	uint16_t sid = 0;
	uint32_t mode = 0;;

	msleep(200);
	CAM_INFO(CAM_SENSOR, "start oplus_cam_actuator_dw9827c sid= %d ",s_ctrl->io_master_info.cci_client->sid);
	sid = s_ctrl->io_master_info.cci_client->sid;
	s_ctrl->io_master_info.cci_client->sid = 0x74;//wide af sid

	//standby mode
	i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.size = 1;
	i2c_reg_array.reg_addr = 0x02;
	i2c_reg_array.reg_data = 0x40;
	i2c_reg_array.delay = 1;
	i2c_reg_settings.reg_setting = &i2c_reg_array;
	rc = camera_io_dev_write(&s_ctrl->io_master_info,&i2c_reg_settings);
	if(rc)
	{
		CAM_ERR(CAM_SENSOR, "page enable failed rc %d",rc);
	}
	msleep(1);

	//check wide sleep mode or not
	rc=camera_io_dev_read(&(s_ctrl->io_master_info),0x7d,&mode,CAMERA_SENSOR_I2C_TYPE_BYTE,CAMERA_SENSOR_I2C_TYPE_BYTE,FALSE);
	if(rc)
	{
		CAM_ERR(CAM_SENSOR, " read failed rc %d",rc);
	}
	if((mode&0x10))
	{
		CAM_INFO(CAM_SENSOR, "mode= %d is sleep mode now",(mode & 0x10));
		s_ctrl->io_master_info.cci_client->sid = sid;
		return 0;
	}

	//if not sleep mode need update
	// pt off
	i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.size = 1;
	i2c_reg_array.reg_addr = 0x34;
	i2c_reg_array.reg_data = 0x85;
	i2c_reg_array.delay = 1;
	i2c_reg_settings.reg_setting = &i2c_reg_array;
	rc = camera_io_dev_write(&s_ctrl->io_master_info,&i2c_reg_settings);
	if(rc)
	{
		CAM_ERR(CAM_SENSOR, "page enable failed rc %d",rc);
	}
	msleep(1);

	i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.size = 1;
	i2c_reg_array.reg_addr = 0x7D;
	i2c_reg_array.reg_data = 0x12;
	i2c_reg_array.delay = 1;
	i2c_reg_settings.reg_setting = &i2c_reg_array;
	rc = camera_io_dev_write(&s_ctrl->io_master_info,&i2c_reg_settings);
	if(rc)
	{
		CAM_ERR(CAM_SENSOR, "page enable failed rc %d",rc);
	}
	msleep(1);

	i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.size = 1;
	i2c_reg_array.reg_addr = 0x03;
	i2c_reg_array.reg_data = 0x01;
	i2c_reg_array.delay = 1;
	i2c_reg_settings.reg_setting = &i2c_reg_array;
	rc = camera_io_dev_write(&s_ctrl->io_master_info,&i2c_reg_settings);
	if(rc)
	{
		CAM_ERR(CAM_SENSOR, "page enable failed rc %d",rc);
	}
	msleep(10);

	i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.size = 1;
	i2c_reg_array.reg_addr = 0x04;
	i2c_reg_array.reg_data = 0x01;
	i2c_reg_array.delay = 1;
	i2c_reg_settings.reg_setting = &i2c_reg_array;
	rc = camera_io_dev_write(&s_ctrl->io_master_info,&i2c_reg_settings);
	if(rc)
	{
		CAM_ERR(CAM_SENSOR, "page enable failed rc %d",rc);
	}
	msleep(3);

	//check wide sleep mode or not
	rc=camera_io_dev_read(&(s_ctrl->io_master_info),0x7d,&mode,CAMERA_SENSOR_I2C_TYPE_BYTE,CAMERA_SENSOR_I2C_TYPE_BYTE,FALSE);
	if(rc)
	{
		CAM_ERR(CAM_SENSOR, " read failed rc %d",rc);
	}

	CAM_INFO(CAM_SENSOR, "mode= 0x%x ",mode);

	rc=camera_io_dev_read(&(s_ctrl->io_master_info),0x02,&mode,CAMERA_SENSOR_I2C_TYPE_BYTE,CAMERA_SENSOR_I2C_TYPE_BYTE,FALSE);
	if(rc)
	{
		CAM_ERR(CAM_SENSOR, " read failed rc %d",rc);
	}
	CAM_INFO(CAM_SENSOR, "mode1= 0x%x ",mode);


	s_ctrl->io_master_info.cci_client->sid = sid;
	msleep(5);
	CAM_INFO(CAM_SENSOR, " oplus_cam_actuator_dw9827c done");
	return rc;
}


int cam_sensor_match_id_oem(struct cam_sensor_ctrl_t *s_ctrl,uint32_t chip_id)
{
	uint32_t vendor_id =0;
	uint32_t read_status = 0;
	int rc=0;
	if(chip_id == CAM_IMX615_SENSOR_ID){
		SensorRegWrite(s_ctrl,0x0A02,0x7F);//Specify OTP Page Address for Read
		SensorRegWrite(s_ctrl,0x0A00,0x01);//Turn ON OTP Read Mode
		rc=camera_io_dev_read(
			&(s_ctrl->io_master_info),
			0xA01,&read_status,s_ctrl->sensordata->id_info.sensor_addr_type,
			CAMERA_SENSOR_I2C_TYPE_BYTE,FALSE);//Read Fab Id
		if(rc == 0 && read_status == 0x01)
		{
			rc=camera_io_dev_read(
			&(s_ctrl->io_master_info),
			s_ctrl->sensordata->id_info.sensor_id_reg_addr,
			&vendor_id,s_ctrl->sensordata->id_info.sensor_addr_type,
			CAMERA_SENSOR_I2C_TYPE_BYTE,FALSE);

			CAM_ERR(CAM_SENSOR, "Read vendor_id_addr=0x%x vendor_id: 0x%x expected vendor_id 0x%x: rc=%d",
			s_ctrl->sensordata->id_info.sensor_id_reg_addr,
			vendor_id,
			s_ctrl->sensordata->id_info.sensor_id,
			rc);
		}
		/*if vendor_id id is 180(0xB4),it is Fab2 module if vendor_id <= 56(0x38),it is Fab1(0x34) or Fab3(0x38) module*/
		if(vendor_id > 56){
			if(s_ctrl->sensordata->id_info.sensor_id > 56)
			{
				return 0;
			}
			else
			{
				return -1;
			}
		}
		else if(vendor_id <= 56)
		{
			if(s_ctrl->sensordata->id_info.sensor_id <= 56)
			{
				return 0;
			}
			else
			{
				return -1;
			}
		}
	}
	if(s_ctrl->is_update_wide_to_sleep)
	{
		cam_sensor_change_wideaf_to_sleep_mode(s_ctrl);
	}
	return 0;
}

void cam_sensor_get_dt_data(struct cam_sensor_ctrl_t *s_ctrl)
{
		int32_t rc = 0;
		struct device_node *of_node = s_ctrl->of_node;

		rc = of_property_read_u32(of_node, "is-support-laser",&s_ctrl->is_support_laser);
		if ( rc < 0) {
			CAM_DBG(CAM_SENSOR, "Invalid sensor params");
			s_ctrl->is_support_laser = 0;
		}

		rc = of_property_read_u32(of_node, "enable_qsc_write_in_advance",&s_ctrl->sensor_qsc_setting.enable_qsc_write_in_advance);
		if ( rc < 0)
		{
			CAM_DBG(CAM_SENSOR, "Invalid sensor params");
			s_ctrl->sensor_qsc_setting.enable_qsc_write_in_advance = 0;
		}
		else
		{
		    CAM_INFO(CAM_SENSOR, "enable_qsc_write_in_advance = %d",s_ctrl->sensor_qsc_setting.enable_qsc_write_in_advance);
		}

		rc = of_property_read_u32(of_node, "qsc_reg_addr",&s_ctrl->sensor_qsc_setting.qsc_reg_addr);
		if ( rc < 0)
		{
			CAM_DBG(CAM_SENSOR, "Invalid sensor params");
			s_ctrl->sensor_qsc_setting.qsc_reg_addr = 0;
		}
		else
		{
		    CAM_INFO(CAM_SENSOR, "qsc_reg_addr = 0x%x",s_ctrl->sensor_qsc_setting.qsc_reg_addr);
		}

		rc = of_property_read_u32(of_node, "eeprom_slave_addr",&s_ctrl->sensor_qsc_setting.eeprom_slave_addr);
		if ( rc < 0)
		{
			CAM_DBG(CAM_SENSOR, "Invalid sensor params");
			s_ctrl->sensor_qsc_setting.eeprom_slave_addr = 0;
		}
		else
		{
			CAM_INFO(CAM_SENSOR, "eeprom_slave_addr = 0x%x",s_ctrl->sensor_qsc_setting.eeprom_slave_addr);
		}

		rc = of_property_read_u32(of_node, "qsc_data_size",&s_ctrl->sensor_qsc_setting.qsc_data_size);
		if ( rc < 0)
		{
			CAM_DBG(CAM_SENSOR, "Invalid sensor params");
			s_ctrl->sensor_qsc_setting.qsc_data_size = 0;
		}
		else
		{
			CAM_INFO(CAM_SENSOR, "qsc_data_size = %d",s_ctrl->sensor_qsc_setting.qsc_data_size);
		}

		rc = of_property_read_u32(of_node, "write_qsc_addr",&s_ctrl->sensor_qsc_setting.write_qsc_addr);
		if ( rc < 0)
		{
			CAM_DBG(CAM_SENSOR, "Invalid sensor params");
			s_ctrl->sensor_qsc_setting.write_qsc_addr = 0;
		}
		else
		{
			CAM_INFO(CAM_SENSOR, "write_qsc_addr = 0x%x",s_ctrl->sensor_qsc_setting.write_qsc_addr);
		}

		rc = of_property_read_u32(of_node, "change_wide_af_sleep", &s_ctrl->is_update_wide_to_sleep);
		if (rc)
		{
			s_ctrl->is_update_wide_to_sleep = 0;
			CAM_ERR(CAM_SENSOR, "get is_update_wide_to_sleep failed rc:%d, default %d", rc, s_ctrl->is_update_wide_to_sleep);
		}
		else
		{
			CAM_INFO(CAM_SENSOR, "read is_update_wide_to_sleep success, value:%d", s_ctrl->is_update_wide_to_sleep);
		}
}

#ifdef OPLUS_FEATURE_CAMERA_COMMON
int oplus_shift_sensor_mode(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc=0;
	struct cam_camera_slave_info *slave_info;
	struct cam_sensor_i2c_reg_array ov32c_array;
	struct cam_sensor_i2c_reg_setting ov32c_array_write;

	slave_info = &(s_ctrl->sensordata->slave_info);

	if(slave_info->sensor_id == 0x3243 && slave_info->sensor_slave_addr == 0x20)
	{
		s_ctrl->io_master_info.cci_client->sid = 0x30 >> 1;
		ov32c_array.reg_addr = 0x1001;
		ov32c_array.reg_data = 0x4;
		ov32c_array.delay = 0x00;
		ov32c_array.data_mask = 0x00;
		ov32c_array_write.reg_setting = &ov32c_array;
		ov32c_array_write.size = 1;
		ov32c_array_write.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		ov32c_array_write.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		ov32c_array_write.delay = 1;

		rc = camera_io_dev_write(&(s_ctrl->io_master_info),&ov32c_array_write);
		CAM_INFO(CAM_SENSOR, "write result %d", rc);
		mdelay(1);
		s_ctrl->io_master_info.cci_client->sid = 0x20 >> 1;
	}
	else if(slave_info->sensor_id == 0x3243 && slave_info->sensor_slave_addr == 0x6c)
	{
		s_ctrl->io_master_info.cci_client->sid = 0x7c >> 1;
		ov32c_array.reg_addr = 0x1001;
		ov32c_array.reg_data = 0x4;
		ov32c_array.delay = 0x00;
		ov32c_array.data_mask = 0x00;
		ov32c_array_write.reg_setting = &ov32c_array;
		ov32c_array_write.size = 1;
		ov32c_array_write.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		ov32c_array_write.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		ov32c_array_write.delay = 1;

		rc = camera_io_dev_write(&(s_ctrl->io_master_info),&ov32c_array_write);
		CAM_INFO(CAM_SENSOR, "write result %d", rc);
		mdelay(1);
		s_ctrl->io_master_info.cci_client->sid = 0x6c >> 1;
	}
	else{
		CAM_INFO(CAM_SENSOR, "Invalid ov32c slave address");
	}
	return rc;
}
#endif

int sensor_start_thread(void *arg)
{
	struct cam_sensor_ctrl_t *s_ctrl = (struct cam_sensor_ctrl_t *)arg;
	int rc = 0;
	CAM_ERR(CAM_SENSOR, "Enter sensor_start_thread!");
	if (!s_ctrl)
	{
		CAM_ERR(CAM_SENSOR, "s_ctrl is NULL");
		return -1;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	//power up for sensor
	mutex_lock(&(s_ctrl->sensor_power_state_mutex));
	if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_OFF)
	{
		CAM_ERR(CAM_SENSOR, "Enter CAM_SENSOR is POWER_OFF now,so power ON!");
		rc = cam_sensor_power_up(s_ctrl);
		if(rc < 0) {
			CAM_ERR(CAM_SENSOR, "sensor power up faild!");
		}
		else
		{
			CAM_INFO(CAM_SENSOR, "sensor power up success sensor id 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
			s_ctrl->sensor_power_state = CAM_SENSOR_POWER_ON;
			s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
			s_ctrl->sensor_qsc_setting.qscsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
		}
	}
	else
	{
		CAM_INFO(CAM_SENSOR, "sensor have power up!");
	}
	mutex_unlock(&(s_ctrl->sensor_power_state_mutex));

	//write initsetting for sensor
	if (rc == 0)
	{
		mutex_lock(&(s_ctrl->sensor_initsetting_mutex));
		if(s_ctrl->sensor_initsetting_state == CAM_SENSOR_SETTING_WRITE_INVALID)
		{
			CAM_ERR(CAM_SENSOR, "Enter CAM_SENSOR_SETTING_WRITE_INVALID!");
			oplus_shift_sensor_mode(s_ctrl);
			rc = camera_io_dev_write(&(s_ctrl->io_master_info), &(s_ctrl->sensor_init_setting));
			CAM_ERR(CAM_SENSOR, "Enter CAM_SENSOR_SETTING_WRITE_INVALID Done!");
			if(rc < 0)
			{
				CAM_ERR(CAM_SENSOR, "write setting failed!");
			}
			else
			{
				CAM_INFO(CAM_SENSOR, "write setting success!");
				s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_SUCCESS;
			}
		}
		else
		{
			CAM_INFO(CAM_SENSOR, "sensor setting have write!");
		}
		mutex_unlock(&(s_ctrl->sensor_initsetting_mutex));
	}
	// write QSC init
	if (rc == 0)
	{
		mutex_lock(&(s_ctrl->sensor_initsetting_mutex));
		if(s_ctrl->sensor_qsc_setting.enable_qsc_write_in_advance && s_ctrl->sensor_qsc_setting.read_qsc_success)
		{
			rc = camera_io_dev_write_continuous(&(s_ctrl->io_master_info),&(s_ctrl->sensor_qsc_setting.qsc_setting),CAM_SENSOR_I2C_WRITE_SEQ);
			if(rc < 0)
			{
				CAM_ERR(CAM_SENSOR, "write qsc failed!");
			}
			else
			{
				CAM_INFO(CAM_SENSOR, "write qsc success!");
				s_ctrl->sensor_qsc_setting.qscsetting_state = CAM_SENSOR_SETTING_WRITE_SUCCESS;
			}
		}
		mutex_unlock(&(s_ctrl->sensor_initsetting_mutex));
	}

	if (s_ctrl->sensor_init_setting.reg_setting != NULL)
	{
		kfree(s_ctrl->sensor_init_setting.reg_setting);
		s_ctrl->sensor_init_setting.reg_setting = NULL;
	}
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int cam_sensor_power_up_advance(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	bool skipPowerUp = false;
	mutex_lock(&(s_ctrl->sensor_power_state_mutex));

	if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_OFF)
	{
		if (skipPowerUp == false)
		{
			rc = cam_sensor_power_up(s_ctrl);
		}

		if(rc < 0)
		{
			CAM_ERR(CAM_SENSOR,
				"Sensor Power up failed for %s sensor_id:0x%x, slave_addr:0x%x",
				s_ctrl->sensor_name,
				s_ctrl->sensordata->slave_info.sensor_id,
				s_ctrl->sensordata->slave_info.sensor_slave_addr
				);
		}
		else
		{
			CAM_INFO(CAM_SENSOR,
				"Sensor Power up success for %s sensor_id:0x%x, slave_addr:0x%x",
				s_ctrl->sensor_name,
				s_ctrl->sensordata->slave_info.sensor_id,
				s_ctrl->sensordata->slave_info.sensor_slave_addr
				);
			s_ctrl->sensor_power_state = CAM_SENSOR_POWER_ON;
			mutex_lock(&(s_ctrl->sensor_initsetting_mutex));
			s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
			mutex_unlock(&(s_ctrl->sensor_initsetting_mutex));
		}
	}
	else
	{
		CAM_INFO(CAM_SENSOR, "sensor have power up!");
	}
	mutex_unlock(&(s_ctrl->sensor_power_state_mutex));
	return rc;
}

int cam_sensor_power_down_advance(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	bool skipPowerOff = false;
	mutex_lock(&(s_ctrl->sensor_power_state_mutex));
	if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_ON)
	{
		if (skipPowerOff == false)
		{
			rc = cam_sensor_power_down(s_ctrl);
		}

		if(rc < 0)
		{
			CAM_ERR(CAM_SENSOR,
				"Power down failed for %s sensor_id: 0x%x, slave_addr: 0x%x",
				s_ctrl->sensor_name,
				s_ctrl->sensordata->slave_info.sensor_id,
				s_ctrl->sensordata->slave_info.sensor_slave_addr
				);
		}
		else
		{
			CAM_INFO(CAM_SENSOR,
				"Power down success for %s sensor_id: 0x%x, slave_addr: 0x%x",
				s_ctrl->sensor_name,
				s_ctrl->sensordata->slave_info.sensor_id,
				s_ctrl->sensordata->slave_info.sensor_slave_addr
			);
			s_ctrl->sensor_power_state = CAM_SENSOR_POWER_OFF;
			mutex_lock(&(s_ctrl->sensor_initsetting_mutex));
			s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
			s_ctrl->sensor_qsc_setting.qscsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
			mutex_unlock(&(s_ctrl->sensor_initsetting_mutex));
		}
	}
	else
	{
		CAM_INFO(CAM_SENSOR, "sensor have power down!");
	}
	mutex_unlock(&(s_ctrl->sensor_power_state_mutex));
	return rc;
}

int cam_sensor_start(struct cam_sensor_ctrl_t *s_ctrl, void *arg)
{
	int rc = 0;
	int i = 0;
	struct cam_control *cmd = (struct cam_control *)arg;
	struct cam_sensor_i2c_reg_array *reg_setting = NULL;
	struct cam_oem_initsettings *initsettings = vzalloc(sizeof(struct cam_oem_initsettings));
	if (initsettings == NULL) {
		CAM_ERR(CAM_EEPROM, "failed to allocate memory!!!!");
		return rc;
	}
	if (!s_ctrl || !cmd)
	{
		CAM_ERR(CAM_SENSOR, "cam_sensor_start s_ctrl or arg is null ");
		return -1;
	}

	memset(initsettings, 0, sizeof(struct cam_oem_initsettings));

	if (copy_from_user(initsettings, u64_to_user_ptr(cmd->handle), cmd->size))
	{
		CAM_ERR(CAM_SENSOR, "initsettings copy_to_user failed ");
	}

	reg_setting = (struct cam_sensor_i2c_reg_array *)kmalloc((sizeof(struct cam_sensor_i2c_reg_array) * initsettings->size), GFP_KERNEL);
	if (reg_setting == NULL)
	{
		CAM_ERR(CAM_EEPROM, "failed to allocate initsettings memory!!!!");
		if (initsettings != NULL)
		{
			vfree(initsettings);
			initsettings = NULL;
		}
		return rc;
	}

	for (i = 0; i < initsettings->size; i++)
	{
		reg_setting[i].reg_addr = initsettings->reg_setting[i].reg_addr;
		reg_setting[i].reg_data = initsettings->reg_setting[i].reg_data;
		reg_setting[i].delay = initsettings->reg_setting[i].delay;
		reg_setting[i].data_mask = 0x00;
	}
	s_ctrl->sensor_init_setting.addr_type = initsettings->addr_type;
	s_ctrl->sensor_init_setting.data_type = initsettings->data_type;
	s_ctrl->sensor_init_setting.size = initsettings->size;

	if (initsettings != NULL)
	{
		vfree(initsettings);
		initsettings = NULL;
	}
	s_ctrl->sensor_init_setting.reg_setting = reg_setting;

	mutex_lock(&(s_ctrl->sensor_power_state_mutex));
	if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_OFF)
	{
		s_ctrl->sensor_open_thread = kthread_run(sensor_start_thread, s_ctrl, s_ctrl->device_name);
		if (!s_ctrl->sensor_open_thread)
		{
			CAM_ERR(CAM_SENSOR, "create sensor start thread failed");
			if (reg_setting != NULL)
			{
				kfree(reg_setting);
				reg_setting = NULL;
			}
			rc = -1;
		}
		else
		{
			CAM_INFO(CAM_SENSOR, "create sensor start thread success");
		}
	}
	else
	{
		CAM_INFO(CAM_SENSOR, "sensor have power up");
	}
	mutex_unlock(&(s_ctrl->sensor_power_state_mutex));

	return rc;
}

int cam_sensor_stop(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	CAM_INFO(CAM_SENSOR,"sensor do stop");
	mutex_lock(&(s_ctrl->cam_sensor_mutex));

	//power off for sensor
	mutex_lock(&(s_ctrl->sensor_power_state_mutex));
	if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_ON)
	{
		rc = cam_sensor_power_down(s_ctrl);
		if(rc < 0)
		{
			CAM_ERR(CAM_SENSOR, "sensor power down faild!");
		}
		else
		{
			CAM_INFO(CAM_SENSOR, "sensor power down success sensor id 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
			s_ctrl->sensor_power_state = CAM_SENSOR_POWER_OFF;
			mutex_lock(&(s_ctrl->sensor_initsetting_mutex));
			s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
			s_ctrl->sensor_qsc_setting.qscsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
			mutex_unlock(&(s_ctrl->sensor_initsetting_mutex));
		}
	}
	else
	{
		CAM_INFO(CAM_SENSOR, "sensor have power down!");
		s_ctrl->sensor_power_state = CAM_SENSOR_POWER_OFF;
		mutex_lock(&(s_ctrl->sensor_initsetting_mutex));
		s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
		s_ctrl->sensor_qsc_setting.qscsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
		mutex_unlock(&(s_ctrl->sensor_initsetting_mutex));
	}
	mutex_unlock(&(s_ctrl->sensor_power_state_mutex));

	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int32_t post_cam_sensor_driver_cmd(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;

	switch (cmd->op_code) {
		case CAM_OEM_GET_ID : {
			if (copy_to_user((void __user *)cmd->handle,&s_ctrl->soc_info.index, sizeof(uint32_t)))
			{
				CAM_ERR(CAM_SENSOR,"copy camera id to user fail ");
			}
			break;
		}
		case CAM_OEM_IO_CMD : {
			cam_sensor_start(s_ctrl, arg);
			break;
		}
	}
	return rc;
}

int oplus_cam_sensor_apply_settings(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	if (s_ctrl->i2c_data.qsc_settings.is_settings_valid &&
		(s_ctrl->i2c_data.qsc_settings.request_id == 0)) {
		if (s_ctrl->sensor_initsetting_state == CAM_SENSOR_SETTING_WRITE_SUCCESS) {
			if(!cam_sensor_bypass_qsc(s_ctrl))
			{
				trace_int("KMD_QSC1", 1);
				rc = cam_sensor_apply_settings(s_ctrl, 0, CAM_SENSOR_PACKET_OPCODE_SENSOR_QSC);
				trace_int("KMD_QSC1", 0);
				if (rc) {
					CAM_WARN(CAM_SENSOR, "%s:retry apply qsc settings %d", s_ctrl->sensor_name, rc);
					trace_int("KMD_QSC2", 1);
					rc = cam_sensor_apply_settings(s_ctrl, 0, CAM_SENSOR_PACKET_OPCODE_SENSOR_QSC);
					trace_int("KMD_QSC2", 0);
					if (rc) {
						CAM_WARN(CAM_SENSOR, "%s: Failed apply qsc settings %d",
							s_ctrl->sensor_name, rc);
						delete_request(&s_ctrl->i2c_data.qsc_settings);
						return rc;
					}
				}
				rc = delete_request(&s_ctrl->i2c_data.qsc_settings);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR,"%s: Fail in deleting the qsc settings",
						s_ctrl->sensor_name);
					return rc;
				}
			}
			else
			{
				rc = delete_request(&s_ctrl->i2c_data.qsc_settings);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR,"%s: Fail in deleting the qsc settings",
						s_ctrl->sensor_name);
					return rc;
				}
				CAM_INFO(CAM_SENSOR, "%s: cam_sensor_bypass_qsc", s_ctrl->sensor_name);
			}
		}
		else
			CAM_ERR(CAM_SENSOR,"%s: init setting not readly",s_ctrl->sensor_name);
	}
	if (s_ctrl->i2c_data.pdc_settings.is_settings_valid &&
		(s_ctrl->i2c_data.pdc_settings.request_id == 0)) {
		if (s_ctrl->sensor_initsetting_state == CAM_SENSOR_SETTING_WRITE_SUCCESS) {
			trace_int("KMD_PDC1", 1);
			rc = cam_sensor_apply_settings(s_ctrl, 0, CAM_SENSOR_PACKET_OPCODE_SENSOR_PDC);
			trace_int("KMD_PDC1", 0);
			if (rc) {
				CAM_WARN(CAM_SENSOR, "%s:retry apply pdc settings %d", s_ctrl->sensor_name, rc);
				trace_int("KMD_PDC2", 1);
				rc = cam_sensor_apply_settings(s_ctrl, 0, CAM_SENSOR_PACKET_OPCODE_SENSOR_PDC);
				trace_int("KMD_PDC2", 0);
				if (rc) {
					CAM_WARN(CAM_SENSOR, "%s: Failed apply pdc settings %d",
						s_ctrl->sensor_name, rc);
					delete_request(&s_ctrl->i2c_data.pdc_settings);
					return rc;
				}
			}
			rc = delete_request(&s_ctrl->i2c_data.pdc_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,"%s: Fail in deleting the pdc settings",
					s_ctrl->sensor_name);
				return rc;
			}
		}
		else
			CAM_ERR(CAM_SENSOR,"%s: init setting not readly",s_ctrl->sensor_name);
	}
	if (s_ctrl->i2c_data.awbotp_settings.is_settings_valid &&
		(s_ctrl->i2c_data.awbotp_settings.request_id == 0)) {
		if (s_ctrl->sensor_initsetting_state == CAM_SENSOR_SETTING_WRITE_SUCCESS) {
			trace_int("KMD_AWB1", 1);
			rc = cam_sensor_apply_settings(s_ctrl, 0, CAM_SENSOR_PACKET_OPCODE_SENSOR_AWBOTP);
			trace_int("KMD_AWB1", 0);
			if (rc) {
				CAM_WARN(CAM_SENSOR, "%s:retry apply awb settings %d", s_ctrl->sensor_name, rc);
				trace_int("KMD_AWB2", 1);
				rc = cam_sensor_apply_settings(s_ctrl, 0, CAM_SENSOR_PACKET_OPCODE_SENSOR_AWBOTP);
				trace_int("KMD_AWB2", 0);
				if (rc) {
					CAM_WARN(CAM_SENSOR, "%s: Failed apply awb settings %d",
						s_ctrl->sensor_name, rc);
					delete_request(&s_ctrl->i2c_data.awbotp_settings);
					return rc;
				}
			}
			rc = delete_request(&s_ctrl->i2c_data.awbotp_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,"%s: Fail in deleting the awb settings",
					s_ctrl->sensor_name);
				return rc;
			}
		}
		else
			CAM_ERR(CAM_SENSOR,"%s: init setting not readly",s_ctrl->sensor_name);
	}
	if (s_ctrl->i2c_data.lsc_settings.is_settings_valid &&
		(s_ctrl->i2c_data.lsc_settings.request_id == 0)) {
		if (s_ctrl->sensor_initsetting_state == CAM_SENSOR_SETTING_WRITE_SUCCESS) {
			trace_int("KMD_LSC1", 1);
			rc = cam_sensor_apply_settings(s_ctrl, 0, CAM_SENSOR_PACKET_OPCODE_SENSOR_LSC);
			trace_int("KMD_LSC1", 0);
			if (rc) {
				CAM_WARN(CAM_SENSOR, "%s:retry apply lsc settings %d", s_ctrl->sensor_name, rc);
				trace_int("KMD_LSC2", 1);
				rc = cam_sensor_apply_settings(s_ctrl, 0, CAM_SENSOR_PACKET_OPCODE_SENSOR_LSC);
				trace_int("KMD_LSC2", 0);
				if (rc) {
					CAM_WARN(CAM_SENSOR, "%s: Failed apply lsc settings %d",
						s_ctrl->sensor_name, rc);
					delete_request(&s_ctrl->i2c_data.lsc_settings);
					return rc;
				}
			}
			rc = delete_request(&s_ctrl->i2c_data.lsc_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,"%s: Fail in deleting the lsc settings",
					s_ctrl->sensor_name);
				return rc;
			}
		}
		else
			CAM_ERR(CAM_SENSOR,"%s: init setting not readly",s_ctrl->sensor_name);
	}
	if (s_ctrl->i2c_data.resolution_settings.is_settings_valid &&
		(s_ctrl->i2c_data.resolution_settings.request_id == 0)) {
		if (s_ctrl->sensor_initsetting_state == CAM_SENSOR_SETTING_WRITE_SUCCESS) {
			trace_int("KMD_RES1", 1);
			rc = cam_sensor_apply_settings(s_ctrl, 0, CAM_SENSOR_PACKET_OPCODE_SENSOR_RESOLUTION);
			trace_int("KMD_RES1", 0);
			if (rc) {
				CAM_WARN(CAM_SENSOR, "%s:retry apply res settings %d", s_ctrl->sensor_name, rc);
				trace_int("KMD_RES2", 1);
				rc = cam_sensor_apply_settings(s_ctrl, 0, CAM_SENSOR_PACKET_OPCODE_SENSOR_RESOLUTION);
				trace_int("KMD_RES2", 0);
				if (rc) {
					CAM_WARN(CAM_SENSOR, "%s: Failed apply res settings %d",
						s_ctrl->sensor_name, rc);
					delete_request(&s_ctrl->i2c_data.resolution_settings);
					return rc;
				}
			}
			rc = delete_request(&s_ctrl->i2c_data.resolution_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,"%s: Fail in deleting the res settings",
					s_ctrl->sensor_name);
				return rc;
			}
		}
		else
			CAM_ERR(CAM_SENSOR,"%s: init setting not readly",s_ctrl->sensor_name);
	}
	return rc;
}

bool cam_sensor_bypass_qsc(struct cam_sensor_ctrl_t *s_ctrl)
{
	bool is_need_bypass = false;
	if(s_ctrl->sensor_qsc_setting.enable_qsc_write_in_advance && s_ctrl->sensor_qsc_setting.read_qsc_success)
	{
		if(s_ctrl->sensor_qsc_setting.qscsetting_state == CAM_SENSOR_SETTING_WRITE_INVALID)
		{
			CAM_INFO(CAM_SENSOR,"qsc setting write failed before ,need write again");
			return is_need_bypass;
		}
		else
		{
			CAM_INFO(CAM_SENSOR,"qsc setting have write  before , no need write again");
			is_need_bypass = true;
			return is_need_bypass;
		}
	}
	else
	{
		CAM_DBG(CAM_SENSOR,"no need compare,don't bypass");
		return is_need_bypass;
	}
}

int cam_sensor_read_qsc(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc=0;

	s_ctrl->sensor_qsc_setting.read_qsc_success = false;
	if(s_ctrl->sensor_qsc_setting.enable_qsc_write_in_advance)
	{
		uint8_t      *data;
		uint16_t     temp_slave_id = 0;
		int          i=0;

		data =  kzalloc(sizeof(uint8_t)*s_ctrl->sensor_qsc_setting.qsc_data_size, GFP_KERNEL);
		if(!data)
		{
			CAM_ERR(CAM_SENSOR,"kzalloc data failed");
			s_ctrl->sensor_qsc_setting.read_qsc_success = false;
			return rc;
		}
		temp_slave_id = s_ctrl->io_master_info.cci_client->sid;
		s_ctrl->io_master_info.cci_client->sid = (s_ctrl->sensor_qsc_setting.eeprom_slave_addr >> 1);
		rc = camera_io_dev_read_seq(&s_ctrl->io_master_info,
					s_ctrl->sensor_qsc_setting.qsc_reg_addr,data,
					CAMERA_SENSOR_I2C_TYPE_WORD,
					CAMERA_SENSOR_I2C_TYPE_BYTE,
					s_ctrl->sensor_qsc_setting.qsc_data_size);
		s_ctrl->io_master_info.cci_client->sid = temp_slave_id;
		if(rc)
		{
			CAM_ERR(CAM_SENSOR,"read qsc data failed");
			s_ctrl->sensor_qsc_setting.read_qsc_success = false;
			kfree(data);
			return rc;
		}

		if(s_ctrl->sensor_qsc_setting.qsc_setting.reg_setting == NULL)
		{
			s_ctrl->sensor_qsc_setting.qsc_setting.reg_setting = kzalloc(sizeof(struct cam_sensor_i2c_reg_array)*s_ctrl->sensor_qsc_setting.qsc_data_size, GFP_KERNEL);
			if (!s_ctrl->sensor_qsc_setting.qsc_setting.reg_setting)
			{
				CAM_ERR(CAM_SENSOR,"allocate qsc data failed");
				s_ctrl->sensor_qsc_setting.read_qsc_success = false;
				kfree(data);
				return rc;
			}
		}

		for(i = 0;i < s_ctrl->sensor_qsc_setting.qsc_data_size;i++)
		{
			s_ctrl->sensor_qsc_setting.qsc_setting.reg_setting[i].reg_addr = s_ctrl->sensor_qsc_setting.write_qsc_addr;
			s_ctrl->sensor_qsc_setting.qsc_setting.reg_setting[i].reg_data = *(data+i);
			s_ctrl->sensor_qsc_setting.qsc_setting.reg_setting[i].data_mask= 0x00;
			s_ctrl->sensor_qsc_setting.qsc_setting.reg_setting[i].delay    = 0x00;
			CAM_DBG(CAM_SENSOR,"read qsc data 0x%x i=%d",s_ctrl->sensor_qsc_setting.qsc_setting.reg_setting[i].reg_data,i);
		}

		s_ctrl->sensor_qsc_setting.qsc_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		s_ctrl->sensor_qsc_setting.qsc_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		s_ctrl->sensor_qsc_setting.qsc_setting.size = s_ctrl->sensor_qsc_setting.qsc_data_size;
		s_ctrl->sensor_qsc_setting.qsc_setting.delay = 1;

		s_ctrl->sensor_qsc_setting.read_qsc_success = true;
		kfree(data);
	}

	return 0;
}
int SensorRegWrite(struct cam_sensor_ctrl_t *s_ctrl,uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;
	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x00,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.delay = 0x00,
	};

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(&(s_ctrl->io_master_info),&i2c_write);
		if (rc < 0)
		{
			CAM_ERR(CAM_SENSOR, "Sensor[%s] write 0x%x failed, retry:%d",
				s_ctrl->sensor_name,
				addr,
				i+1);
		}
		else
		{
			return rc;
		}
	}
	return rc;
}
