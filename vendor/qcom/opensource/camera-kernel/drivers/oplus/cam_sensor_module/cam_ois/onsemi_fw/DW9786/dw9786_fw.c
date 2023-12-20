/**
  ******************************************************************************
  * File Name          : DW9786_SET_API.C
  * Description        : Main program c file
  * DongWoon Anatech   :
  * Version            : 1.6
  ******************************************************************************
  *
  * COPYRIGHT(c) 2021 DWANATECH
  * Revision History
  * 2021.03.11 draft
	* 2023.05.12 apply OPLUS fw register map
  ******************************************************************************
**/

//#define DEBUG_API
//#ifdef DEBUG_API
//#include <wchar.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <tchar.h>
//#include "string.h"
//#include "dw9786 set api rev1.6.h"
//#include "combine.h"
//#include "func.h"
//#else
#ifndef DW9786_FW_C
#define DW9786_FW_C
//#endif

#include <linux/types.h>
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "fw_download_interface.h"

#include "DW9786_OPLUS_LUOYANG_FW_DVT.h" //EVT FW change in here
#include "DW9786_OPLUS_LUOYANG_FW_EVT.h" //EVT FW change in here
#include "DW9786_OPLUS_LUOYANG_FW_T0.h" //T0 FW change in here

#include "dw9786_fw.h"


typedef int(*ptr_func)(const char*, ...);
#endif

//#define USER_PT_A		write_reg_16bit_value_16bit(o_ctrl, 0xEDB0, 0xA23F)
//#define USER_PT_B 		write_reg_16bit_value_16bit(o_ctrl, 0xEDBC, 0xDB01)
//#define CODE_PT 		write_reg_16bit_value_16bit(o_ctrl, 0xE2F8, 0xC0DE)
//#define LOGIC_RESET		write_reg_16bit_value_16bit(o_ctrl, 0xE008, 0x0001)

/*Global buffer for flash download*/
int g_downloadByForce = 0;

int wait_check_register(struct cam_ois_ctrl_t *o_ctrl,unsigned short reg, unsigned short ref)
{
	/*
	reg : read target register
	ref : compare reference data
	*/
	int i = 0;
	int ret = 0;
	uint32_t r_data;
	for (i = 0; i < LOOP_A; i++) {
		read_reg_16bit_value_16bit(o_ctrl, reg, &r_data); //Read status
		if (r_data == ref) {
			ret = FUNC_PASS;
			break;
		}
		else {
			if (i >= LOOP_B) {
				CAM_ERR(CAM_OIS,"[wait_check_register] fail: 0x%04X", r_data);
				return EXECUTE_ERROR;
			}
		}
		mdelay(WAIT_TIME);
	}
	return ret;
}

int dw9786_id_check(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Error code definition
	0 : No Error
	-1 : DW9786_CHIP_ID miss match
	*/
	uint32_t chip_id;
	read_reg_16bit_value_16bit(o_ctrl, 0xE018, &chip_id); // ID read
	/* Check the chip_id of OIS ic */
	if (chip_id != DW9786_CHIP_ID) {
		CAM_ERR(CAM_OIS,"[dw9786_id_check] the module's OIS IC is not dw9786 (0x%04X)", chip_id);
		return -1;
	}
	CAM_ERR(CAM_OIS,"[dw9786_id_check] the module's OIS IC is dw9786 (0x%04X)", chip_id);
	return 0;
}

int dw9786_auto_read_check(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Error code definition
	0 : No Error
	others : auto read fail
	*/
	uint32_t autord_rv_error = 0;
	//int fw_check_sum_read = 0;
	dw9786_chip_enable(o_ctrl,MODE_ON);
	mdelay(5);
	CAM_ERR(CAM_OIS,"[dw9786_auto_read_check] start....");
	/* Check if flash data is normally auto read */
	read_reg_16bit_value_16bit(o_ctrl, 0xED5C, &autord_rv_error);
	if (autord_rv_error != AUTORD_RV_OK) {
		dw9786_chip_enable(o_ctrl,MODE_OFF);
		CAM_ERR(CAM_OIS,"[dw9786_auto_read_check] dw9786 auto_read fail (0x%04X)", autord_rv_error);
		if ((autord_rv_error & 0xFFFF) == 0x0001)
			CAM_ERR(CAM_OIS,"[dw9786_auto_read_check] dw9786 LUT_ERR");
		if ((autord_rv_error & 0xFFFF) == 0x0002)
			CAM_ERR(CAM_OIS,"[dw9786_auto_read_check] dw9786 PGM_ERR");
		if ((autord_rv_error & 0xFFFF) == 0x0004)
			CAM_ERR(CAM_OIS,"[dw9786_auto_read_check] dw9786 LDT0_ERR");
		if ((autord_rv_error & 0xFFFF) == 0x0008)
			CAM_ERR(CAM_OIS,"[dw9786_auto_read_check] dw9786 RED_ERR");
		if ((autord_rv_error & 0xFFFF) == 0x0010)
			CAM_ERR(CAM_OIS,"[dw9786_auto_read_check] dw9786 DCT_ERR");
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_auto_read_check] dw9786 auto-lead function passed");
	}
	return autord_rv_error;
}

int dw9786_download_fw(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Error code definition
	0 : No Error
	-1 : Download Fail
	*/
	uint32_t pjt = 0, ver = 0, dat = 0;
	uint32_t project = 0, version = 0, date = 0, checksum = 0;
	int ac_id = 0;
	int rc = -1;
	/* Check the firmware version */
	//dw9786_chip_enable(MODE_ON);

	rc = dw9786_reset(o_ctrl);
	if(rc != FUNC_PASS){
		CAM_ERR(CAM_OIS,"[dw9786_download_fw] reset failed...");
		return rc;
	}
	read_reg_16bit_value_16bit(o_ctrl, 0x9800, &pjt);
	read_reg_16bit_value_16bit(o_ctrl, 0x9802, &ver);
	read_reg_16bit_value_16bit(o_ctrl, 0x9804, &dat);

	read_reg_16bit_value_16bit(o_ctrl, 0xB70E, &ac_id);
	CAM_ERR(CAM_OIS,"[dw9786_download_fw] read actuator_id %d",ac_id);
	if(ac_id == 1){
		project = DW9786_PJT_VERSION_T0;
		version = DW9786_FW_VERSION_T0;
		date = DW9786_FW_DATE_T0;
		checksum = DW9786_REF_MCS_CHECKSUM_T0;
	}else if (ac_id == 2){
		project = DW9786_PJT_VERSION_EVT;
		version = DW9786_FW_VERSION_EVT;
		date = DW9786_FW_DATE_EVT;
		checksum = DW9786_REF_MCS_CHECKSUM_EVT;
	}else if(ac_id == 4){
		project = DW9786_PJT_VERSION_DVT;
		version = DW9786_FW_VERSION_DVT;
		date = DW9786_FW_DATE_DVT;
		checksum = DW9786_REF_MCS_CHECKSUM_DVT;
	}else{
		CAM_ERR(CAM_OIS,"[dw9786_download_fw] read actuator_id failed %d",ac_id);
		return FUNC_FAIL;
	}
	CAM_ERR(CAM_OIS,"[dw9786_download_fw] module prj_info:[0x%.4x] ver:[0x%.4x] date:[0x%.4x]", pjt, ver, dat);
	CAM_ERR(CAM_OIS,"[dw9786_download_fw] fw prj_info:[0x%.4x] ver:[0x%.4x] date:[0x%.4x]", project, version, date);
	if ((dat < date) || (g_downloadByForce)) {
		CAM_ERR(CAM_OIS,"[dw9786_download_fw] the new firmware version is checked and the update starts.");
		if (dw9786_code_update(o_ctrl,ac_id,checksum) != FUNC_PASS) {
			CAM_ERR(CAM_OIS,"[dw9786_download_fw]return fail");
			return FUNC_FAIL;
		}
		else if (dw9786_checksum_read(o_ctrl) != checksum) {
			CAM_ERR(CAM_OIS,"[dw9786_download_fw] start download firmware due to a checksum error.");
		}
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_download_fw] this is the latest version.");
		/*if (dw9786_checksum_read(o_ctrl) != DW9786_REF_MCS_CHECKSUM) {
			CAM_ERR(CAM_OIS,"[dw9786_download_fw] start download firmware due to a checksum error.");
			if (dw9786_code_update(o_ctrl) != FUNC_PASS) {
				return FUNC_FAIL;
			}
		}*/
	}
    //write_reg_16bit_value_16bit(o_ctrl,dw_Gyro_gain_y, 0x00);
    //dw9786_set_store(o_ctrl);
	CAM_ERR(CAM_OIS,"[dw9786_download_fw]return pass");
	return FUNC_PASS;
}

// FLASH_MCS FW File : Hex
int dw9786_code_update(struct cam_ois_ctrl_t *o_ctrl,int ac_id, uint32_t checksum)
{
	//int idx = 0;
	int i = 0;
	//int msg = 0;
	uint8_t txdata[MCS_PKT_SIZE+2];
	int rc = 0;
	uint16_t addr = 0;
	unsigned int checksum_criteria, checksum_flash;
	unsigned char* DW9786_FW_ARRAY = NULL;
	if(ac_id == 1){
		DW9786_FW_ARRAY = DW9786_FW_T0;
	}else if(ac_id == 2){
		DW9786_FW_ARRAY = DW9786_FW_EVT;
	}else if(ac_id == 4){
		DW9786_FW_ARRAY = DW9786_FW_DVT;
	}else{
		CAM_ERR(CAM_OIS,"[dw9786_code_update]ac_id inputed error !!!");
	}
	CAM_ERR(CAM_OIS,"[dw9786_code_update] start....");
	/*MCS start -----------------------------------------------------------------------------*/
	dw9786_flash_acess(o_ctrl, MCS_CODE);

	for (i = 0; i < FMC_PAGE; i++) {
		if (i == 0) addr = MCS_START_ADDRESS;
		rc = write_reg_16bit_value_16bit(o_ctrl, 0xED08, addr); // Set erase address
		rc = write_reg_16bit_value_16bit(o_ctrl, 0xED0C, 0x0002); // Sector Erase(2KB)
		if(rc < 0){
			CAM_ERR(CAM_OIS,"[dw9786_code_update] Set erase address %d failed, and then exit",addr);
			break;
		}
		addr += 0x800;
		mdelay(5);
	}
	// flash fw write
	for (i = 0; i < MCS_SIZE; i += MCS_PKT_SIZE) {
		if (i == 0) write_reg_16bit_value_16bit(o_ctrl, 0xED28, MCS_START_ADDRESS); // 16bit write
		// program sequential write 2K byte
		txdata[0] = (FW_START_ADDRESS >> 8);
		txdata[1] = (FW_START_ADDRESS & 0xFF);
		memcpy(txdata+2, &DW9786_FW_ARRAY[i], MCS_PKT_SIZE);
		rc = i2c_block_write_reg(o_ctrl, txdata, MCS_PKT_SIZE+2); // 16bit write
		if(rc < 0){
			CAM_ERR(CAM_OIS,"[dw9786_code_update] Write DW9786_FW[%d] failed, and then exit",i);
			return rc;
		}

	}
	mdelay(20);
	dw9786_reset(o_ctrl);
	mdelay(20);
	write_reg_16bit_value_16bit(o_ctrl, 0xB836, 0x000E);

	//set_Store
	dw9786_set_store(o_ctrl);
	/* Checksum calculation for mcs data */
	checksum_criteria = checksum;
	/* Set the checksum area */
	checksum_flash = dw9786_checksum_func(o_ctrl, MCS_CODE);
	if (checksum_criteria != checksum_flash) {
		rc = CHECKSUM_ERROR;
		CAM_ERR(CAM_OIS,"[dw9786_code_update] flash checksum failed!, checksum_criteria: 0x%08X, checksum_flash: 0x%08X", checksum_criteria, checksum_flash);
		dw9786_chip_enable(o_ctrl, MODE_OFF);
		CAM_ERR(CAM_OIS,"[dw9786_code_update] firmware download failed.");
		return rc;
	}
	CAM_ERR(CAM_OIS,"[dw9786_code_update] flash checksum passed!, checksum_criteria: 0x%08X, checksum_flash: 0x%08X", checksum_criteria, checksum_flash);
	CAM_ERR(CAM_OIS,"[dw9786_code_update] firmware download successful.");
	return rc;
}

void dw9786_flash_acess(struct cam_ois_ctrl_t *o_ctrl, int type)
{
	write_reg_16bit_value_16bit(o_ctrl, 0xE000, MODE_ON);
	write_reg_16bit_value_16bit(o_ctrl, 0xE004, MODE_OFF);
	if (type == MCS_LUT) {
		write_reg_16bit_value_16bit(o_ctrl, 0xEDBC, 0xDB01);
	}
	else if (type == MCS_CODE) {
		write_reg_16bit_value_16bit(o_ctrl, 0xE2F8, 0xC0DE);
	}
	else if (type == MCS_USER) {
		write_reg_16bit_value_16bit(o_ctrl, 0xEDB0, 0xA23F);
	}
	write_reg_16bit_value_16bit(o_ctrl, 0xED00, 0x0000);
	mdelay(1);
}


unsigned int checksum_exe(unsigned char *data, int leng)
{
	unsigned int csum = 0xFFFFFFFF;
	for (int i = 0; i < leng; i += 4) {
		csum += (unsigned int)(*(data + i) << 24) + (unsigned int)(*(data + i + 1) << 16) + (unsigned int)(*(data + i + 2) << 8) + (unsigned int)(*(data + i + 3));		
	}
	return ~csum;
}

int dw9786_checksum_func(struct cam_ois_ctrl_t *o_ctrl, int type)
{
	/*
	Error code definition
	0 : No Error
	-1 : CHECKSUM_ERROR
	*/
	uint32_t csh, csl;
	unsigned int mcs_checksum_flash;
	//CAM_ERR(CAM_OIS,"[dw9786_checksum_func] start....");
	/* Set the checksum area */
	if (type == MCS_LUT) {
		/* not used */
	}
	else if (type == MCS_CODE) {
		dw9786_flash_acess(o_ctrl, MCS_CODE);
		write_reg_16bit_value_16bit(o_ctrl, 0xED48, MCS_START_ADDRESS);
		write_reg_16bit_value_16bit(o_ctrl, 0xED4C, MCS_CHECKSUM_SIZE);
		write_reg_16bit_value_16bit(o_ctrl, 0xED50, 0x0001);
	}
	wait_check_register(o_ctrl, 0xED04, 0x00);
	read_reg_16bit_value_16bit(o_ctrl, 0xED54, &csh);
	read_reg_16bit_value_16bit(o_ctrl, 0xED58, &csl);
	mcs_checksum_flash = ((unsigned int)(csh << 16)) | csl;
	CAM_ERR(CAM_OIS,"[dw9786_checksum_func] flash memory checksum , [0x%08X]", mcs_checksum_flash);
	return mcs_checksum_flash;
}

int dw9786_checksum_read(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Error code definition
	0 : No Error
	-1 : CHECKSUM_ERROR
	*/
	uint32_t csh, csl;
	unsigned int checksum_register;
	//CAM_ERR(CAM_OIS,"[dw9786_checksum_func] start....");

	/* Set the checksum area */
	read_reg_16bit_value_16bit(o_ctrl, 0xED60, &csh);
	read_reg_16bit_value_16bit(o_ctrl, 0xED64, &csl);
	checksum_register = ((unsigned int)(csh << 16)) | csl;
	CAM_ERR(CAM_OIS,"[dw9786_checksum_read] flash memory checksum , [0x%08X]", checksum_register);
	return checksum_register;
}

struct _FACT_ADJ_DW dw9786_gyro_ofs_calibration(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	* dw9786 gyro offset calibration
	Error code definition
	-1 : EXECUTE_ERROR
	0 : No Error

	*/
	//int msg = 0;
	uint32_t x_ofs, y_ofs, gyro_status;
	struct _FACT_ADJ_DW FADJCAL = { 0xFFFF, 0xFFFF};
	CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] start....");

	write_reg_16bit_value_16bit(o_ctrl, 0xB026, 0x0006); // gyro offset calibration
	mdelay(100);

	if (wait_check_register(o_ctrl, 0xB020, 0x6000) == FUNC_PASS) {
		write_reg_16bit_value_16bit(o_ctrl, 0xB022, 0x0001);		// Lens ofs calibration execute command
		mdelay(100);
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] gyro offset EXECUTE_ERROR");
		return FADJCAL;
	}


	if (wait_check_register(o_ctrl, 0xB020, 0x6001) == FUNC_PASS) { // when calibration is done, Status changes to 0x6001
		CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] calibration function finish");
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] calibration function error");
		return FADJCAL;
	}

	read_reg_16bit_value_16bit(o_ctrl, 0xB80C, &x_ofs); //x gyro offset
	read_reg_16bit_value_16bit(o_ctrl, 0xB80E, &y_ofs); //y gyro offset
	read_reg_16bit_value_16bit(o_ctrl, 0xB838, &gyro_status); //gyro offset status
	CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] x gyro offset: 0x%04X(%d)", x_ofs, (short)x_ofs);
	CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] y gyro offset: 0x%04X(%d)", y_ofs, (short)y_ofs);
	CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] gyro_status: 0x%04X", gyro_status);

	if((gyro_status & 0x8000) == 0x8000) { // Read Gyro offset cailbration result status
		if (gyro_status & X_AXIS_GYRO_OFS_CAL_PASS) {
			//msg += OK;
			CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] x_axis gyro offset calibration pass");
		}
		if (gyro_status & X_AXIS_GYRO_OFS_MAX_ERR) {
			//msg += X_AXIS_GYRO_OFS_MAX_ERR;
			CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] x_axis gyro offset max. error");
		}
		if (gyro_status & Y_AXIS_GYRO_OFS_CAL_PASS) {
			//msg += OK;
			CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] y_axis gyro offset calibration pass");
		}
		if (gyro_status & Y_AXIS_GYRO_OFS_MAX_ERR) {
			//msg += Y_AXIS_GYRO_OFS_MAX_ERR;
			CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] y_axis gyro offset max. error");
		}
	}
	else {
		//msg += GYRO_OFS_CAL_DONE_FAIL;
		CAM_ERR(CAM_OIS,"[dw9786_gyro_ofs_calibration] calibration done fail");
	}
	
	dw9786_set_store(o_ctrl);
	
	FADJCAL.gl_GX_OFS = x_ofs;
	FADJCAL.gl_GY_OFS = y_ofs;
	
	return FADJCAL;
}

void dw9786_chip_info(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t r_data;
	CAM_ERR(CAM_OIS,"[dw9786_chip_info] start....");

	read_reg_16bit_value_16bit(o_ctrl, 0xE018, &r_data);
	CAM_ERR(CAM_OIS,"[dw9786_chip_info] product_id: 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl, 0xB000, &r_data);
	CAM_ERR(CAM_OIS,"[dw9786_chip_info] chip_id: 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl, 0xB002, &r_data);
	CAM_ERR(CAM_OIS,"[dw9786_chip_info] Fw_ver_H: 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl, 0xB004, &r_data);
	CAM_ERR(CAM_OIS,"[dw9786_chip_info] Fw_ver_L: 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl, 0xB006, &r_data);
	CAM_ERR(CAM_OIS,"[dw9786_chip_info] Fw_date: 0x%04X", r_data);
}

void dw9786_fw_info(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t r_data;
	CAM_ERR(CAM_OIS,"[dw9786_fw_info] start....");

	read_reg_16bit_value_16bit(o_ctrl, 0x9800, &r_data);
	CAM_ERR(CAM_OIS,"[dw9786_fw_info] set version: 0x%02X [MSB], project no.:0x%02X [LSB]", r_data >> 8, r_data & 0xff);
	read_reg_16bit_value_16bit(o_ctrl, 0x9802, &r_data);
	CAM_ERR(CAM_OIS,"[dw9786_fw_info] fw version: 0x%02X [MSB], pid version:0x%02X [LSB]", r_data >> 8, r_data & 0xff);
	read_reg_16bit_value_16bit(o_ctrl, 0x9804, &r_data);
	CAM_ERR(CAM_OIS,"[dw9786_fw_info] fw_date: 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl, 0x9806, &r_data);
	CAM_ERR(CAM_OIS,"[dw9786_fw_info] act: 0x%02X [MSB], module:0x%02X [LSB]", r_data >> 8, r_data & 0xff);
}

int dw9786_module_store(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Error code definition
	0 : No Error
	-1 : EXECUTE_ERROR
	*/
	CAM_ERR(CAM_OIS,"[dw9786_module_store] start....");
	write_reg_16bit_value_8bit(o_ctrl, 0x97E6, 0xEA); //USER_PT_C off
	write_reg_16bit_value_8bit(o_ctrl, 0x3FFF, 0x00); //User memory dummy write
	mdelay(10); //Absolutely necessary

	write_reg_16bit_value_16bit(o_ctrl, 0xB026, 0x000A); //Store&Erase

	if (wait_check_register(o_ctrl, 0xB020, 0xA000) == FUNC_PASS) {
		write_reg_16bit_value_16bit(o_ctrl, 0xEDB0, 0xA23F);
		write_reg_16bit_value_16bit(o_ctrl, 0xB03E, 0xA6A6); // seleect module_cal memory
		mdelay(1);
		write_reg_16bit_value_16bit(o_ctrl, 0xB022, 0x0001); // store execute
		mdelay(100);
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_module_store] execute fail");
		return EXECUTE_ERROR;
	}

	if (wait_check_register(o_ctrl, 0xB020, 0xA001) == FUNC_PASS) {
		CAM_ERR(CAM_OIS,"[dw9786_module_store] complete");
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_module_store] fail");
		return EXECUTE_ERROR;
	}

	return FUNC_PASS;
}

int dw9786_set_store(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Error code definition
	0 : No Error
	-1 : EXECUTE_ERROR
	*/
	CAM_ERR(CAM_OIS,"[dw9786_set_store] start....");
	write_reg_16bit_value_8bit(o_ctrl, 0x97E6, 0xEA); //USER_PT_C off
	write_reg_16bit_value_8bit(o_ctrl, 0x3FFF, 0x00); //User memory dummy write
	mdelay(10); //Absolutely necessary

	write_reg_16bit_value_16bit(o_ctrl, 0xB026, 0x000A); //Store&Erase

	if (wait_check_register(o_ctrl, 0xB020, 0xA000) == FUNC_PASS) {
		write_reg_16bit_value_16bit(o_ctrl, 0xEDB0, 0xA23F);
		write_reg_16bit_value_16bit(o_ctrl, 0xB03E, 0x5959); // seleect set_cal memory
		mdelay(1);
		write_reg_16bit_value_16bit(o_ctrl, 0xB022, 0x0001); //Store execute
		mdelay(100);
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_set_store] execute fail");
		return EXECUTE_ERROR;
	}

	if (wait_check_register(o_ctrl, 0xB020, 0xA001) == FUNC_PASS) {
		CAM_ERR(CAM_OIS,"[dw9786_set_store] complete");
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_set_store] fail");
		return EXECUTE_ERROR;
	}

	return FUNC_PASS;
}

int dw9786_module_erase(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Error code definition
	0 : No Error
	-1 : EXECUTE_ERROR
	*/

	//uint32_t r_data;
	CAM_ERR(CAM_OIS,"[dw9786_erase] start....");

	write_reg_16bit_value_16bit(o_ctrl, 0xB026, 0x000B); //Store&Erase

	if (wait_check_register(o_ctrl, 0xB020, 0xB000) == FUNC_PASS) {	//Read Status
		write_reg_16bit_value_16bit(o_ctrl, 0xEDB0, 0xA23F);
		write_reg_16bit_value_16bit(o_ctrl, 0xB03E, 0xA6A6); // seleect module_cal memory
		mdelay(1);
		write_reg_16bit_value_16bit(o_ctrl, 0xB022, 0x0001); //Erase
		mdelay(100);
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_erase] fail");
		return EXECUTE_ERROR;
	}

	if (wait_check_register(o_ctrl, 0xB020, 0xB001) == FUNC_PASS) {	//Read Status
		CAM_ERR(CAM_OIS,"[dw9786_erase] complete");
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_erase] fail");
		write_reg_16bit_value_16bit(o_ctrl, 0x0220, 0x0000); // Code PT on
		return EXECUTE_ERROR;
	}

	return FUNC_PASS;
}

int dw9786_set_erase(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Error code definition
	0 : No Error
	-1 : EXECUTE_ERROR
	*/

	//uint32_t r_data;
	CAM_ERR(CAM_OIS,"[dw9786_erase] start....");

	write_reg_16bit_value_16bit(o_ctrl, 0xB026, 0x000B); //Store&Erase

	if (wait_check_register(o_ctrl, 0xB020, 0xB000) == FUNC_PASS) {	//Read Status
		write_reg_16bit_value_16bit(o_ctrl, 0xEDB0, 0xA23F);
		write_reg_16bit_value_16bit(o_ctrl, 0xB03E, 0x5959); // seleect set_cal memory
		mdelay(1);
		write_reg_16bit_value_16bit(o_ctrl, 0xB022, 0x0001); //Erase
		mdelay(100);
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_erase] fail");
		return EXECUTE_ERROR;
	}

	if (wait_check_register(o_ctrl, 0xB020, 0xB001) == FUNC_PASS) {	//Read Status
		CAM_ERR(CAM_OIS,"[dw9786_erase] complete");
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_erase] fail");
		write_reg_16bit_value_16bit(o_ctrl, 0x0220, 0x0000); // Code PT on
		return EXECUTE_ERROR;
	}

	return FUNC_PASS;
}

int dw9786_reset(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = FUNC_PASS;
	CAM_ERR(CAM_OIS,"[dw9786_reset] start....");

	rc = write_reg_16bit_value_16bit(o_ctrl, 0xE000, 0x0000); //shutdown mode
	if(rc != FUNC_PASS){
	    CAM_ERR(CAM_OIS,"[dw9786_reset] write 0xE000 fail, exit....");
        return rc;
    }
	mdelay(10);
	rc = write_reg_16bit_value_16bit(o_ctrl, 0xE000, 0x0001); //standby mode
	mdelay(5);
	rc = write_reg_16bit_value_16bit(o_ctrl, 0xE2FC, 0xAC1E);
	rc = write_reg_16bit_value_16bit(o_ctrl, 0xE164, 0x0008);
	rc = write_reg_16bit_value_16bit(o_ctrl, 0xE2FC, 0x0000);
	mdelay(1);
	rc = write_reg_16bit_value_16bit(o_ctrl, 0xE004, 0x0001); //idle mode MCU active
	mdelay(100);
	CAM_ERR(CAM_OIS,"[dw9786_reset] reset execution complete");
	return rc;
}

int dw9786_mcu_active(struct cam_ois_ctrl_t *o_ctrl, unsigned short mode)
{
	if (mode == 0) {
		CAM_ERR(CAM_OIS,"[dw9786_mcu_active] dw9786_standby");
		write_reg_16bit_value_16bit(o_ctrl, 0xE004, mode);
	}
	else {
		CAM_ERR(CAM_OIS,"[dw9786_mcu_active] dw9786_idle mode");
		write_reg_16bit_value_16bit(o_ctrl, 0xE004, mode);
	}
	return FUNC_PASS;
}

int dw9786_chip_enable(struct cam_ois_ctrl_t *o_ctrl, unsigned short en)
{
	if (en == 0) {
		CAM_ERR(CAM_OIS,"[dw9786_chip_enable] dw9786_sleep");
		write_reg_16bit_value_16bit(o_ctrl, 0xE000, en);
	}
	else if(en == 1) {
		CAM_ERR(CAM_OIS,"[dw9786_chip_enable] dw9786_standby");
		write_reg_16bit_value_16bit(o_ctrl, 0xE000, en);
	}
	return FUNC_PASS;
}

int dw9786_user_protection(struct cam_ois_ctrl_t *o_ctrl)
{
	write_reg_16bit_value_16bit(o_ctrl, 0xEDB0, 0xA23F);
	CAM_ERR(CAM_OIS,"[dw9786_user_protection] excute");
	return FUNC_PASS;
}

int dw9786_servo_on(struct cam_ois_ctrl_t *o_ctrl)
{
	write_reg_16bit_value_16bit(o_ctrl, 0xB026, 0x0001); //operate mode
	write_reg_16bit_value_16bit(o_ctrl, 0xB022, 0x0002); //servo on
	write_reg_16bit_value_16bit(o_ctrl, 0xB024, 0x0002); //af servo on

	if(wait_check_register(o_ctrl, 0xB020, 0x1022) == FUNC_PASS) { //busy check
		CAM_ERR(CAM_OIS,"[dw9786_servo_on] servo on success");
		return FUNC_PASS;
	}

	return FUNC_FAIL;
}

int dw9786_servo_off(struct cam_ois_ctrl_t *o_ctrl)
{
	write_reg_16bit_value_16bit(o_ctrl, 0xB026, 0x0001); //operate mode
	write_reg_16bit_value_16bit(o_ctrl, 0xB022, 0x0000); //x,y servo off
	write_reg_16bit_value_16bit(o_ctrl, 0xB024, 0x0000); //af servo off

	if(wait_check_register(o_ctrl, 0xB020, 0x1000) == FUNC_PASS) {	//busy check
		CAM_ERR(CAM_OIS,"[dw9786_servo_off] servo off success");
		return FUNC_PASS;
	}

	return FUNC_FAIL;
}

int dw9786_ois_on(struct cam_ois_ctrl_t *o_ctrl)
{
	//uint32_t dat;

	write_reg_16bit_value_16bit(o_ctrl, 0xB026, 0x0001); //operate mode
	write_reg_16bit_value_16bit(o_ctrl, 0xB022, 0x0001); //x,y ois on
	write_reg_16bit_value_16bit(o_ctrl, 0xB024, 0x0002); //af servo on

	if(wait_check_register(o_ctrl, 0xB010, 0x1021) == FUNC_PASS) {
		CAM_ERR(CAM_OIS,"[dw9786_ois_on] ois on success");
		return FUNC_PASS;
	}

	return FUNC_FAIL;
}

int dw9786_spi_master_mode(struct cam_ois_ctrl_t *o_ctrl)
{
	CAM_ERR(CAM_OIS,"[dw9786_spi_master_mode] set to gyro spi master mode.");
	write_reg_16bit_value_16bit(o_ctrl, 0xB040, 0x0000); //spi master mode
	return 0;
}

int dw9786_spi_intercept_mode(struct cam_ois_ctrl_t *o_ctrl)
{
	CAM_ERR(CAM_OIS,"[dw9786_spi_intercept_mode] set to gyro spi intercept mode.");
	write_reg_16bit_value_16bit(o_ctrl, 0xB040, 0x0001); //spi intercept mode
	return 0;
}
void DW9786_WriteGyroGainToFlash(struct cam_ois_ctrl_t *o_ctrl,uint32_t X_gain, uint32_t Y_gain)
{
    uint32_t X_ori, Y_ori;
    read_reg_16bit_value_16bit(o_ctrl, dw_Gyro_gain_x, &X_ori);//read 0xB806
    read_reg_16bit_value_16bit(o_ctrl, dw_Gyro_gain_y, &Y_ori);//read 0xB808
    CAM_INFO(CAM_OIS, "[DW9786_WriteGyroGainToFlash] oldGyorGain  X_gain= 0x%x  Y_gain= 0x%x",X_ori, Y_ori);
    CAM_INFO(CAM_OIS, "[DW9786_WriteGyroGainToFlash] newGyorGain  X_gain= 0x%x  Y_gain= 0x%x",X_gain, Y_gain);
    write_reg_16bit_value_16bit(o_ctrl,dw_Gyro_gain_x, X_gain);
    write_reg_16bit_value_16bit(o_ctrl,dw_Gyro_gain_y, Y_gain);
    //dw9786_set_store(o_ctrl);
    read_reg_16bit_value_16bit(o_ctrl, dw_Gyro_gain_x, &X_ori);//read 0xB806
    read_reg_16bit_value_16bit(o_ctrl, dw_Gyro_gain_y, &Y_ori);//read 0xB808
    CAM_INFO(CAM_OIS, "[DW9786_WriteGyroGainToFlash] after write  X_gain= 0x%x  Y_gain= 0x%x",X_ori, Y_ori);

}

void DW9786_StoreGyroGainToFlash(struct cam_ois_ctrl_t *o_ctrl)
{
    uint32_t X_ori, Y_ori;
    read_reg_16bit_value_16bit(o_ctrl, dw_Gyro_gain_x, &X_ori);//read 0xB806
    read_reg_16bit_value_16bit(o_ctrl, dw_Gyro_gain_y, &Y_ori);//read 0xB808
    CAM_INFO(CAM_OIS, "[DW9786_WriteGyroGainToFlash] store GyorGain  X_gain= 0x%x  Y_gain= 0x%x",X_ori, Y_ori);
    dw9786_set_store(o_ctrl);
}

#if 0

int dw9786_read_mcs_flash(struct cam_ois_ctrl_t *o_ctrl)
{
	//unsigned short buf_R[40960];
	uint32_t rc =0;
	//40KB
	write_reg_16bit_value_16bit(o_ctrl, 0xE000, 0x0000); //shutdown mode
	mdelay(2);
	write_reg_16bit_value_16bit(o_ctrl, 0xE000, 0x0001); //standby mode
	mdelay(5);
	write_reg_16bit_value_16bit(o_ctrl, 0xE2F8, 0xC0DE); //code pt off
	mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl, 0xED00, 0x0000); //select mcs
	mdelay(1);

	/* read fw flash */
	for (int i = 0; i < MCS_SIZE; i += MCS_PKT_SIZE) {
		if (i == 0) write_reg_16bit_value_16bit(o_ctrl, 0xED28, MCS_START_ADDRESS); // 16bit write		
		/* program sequential write 2K byte */
		rc = i2c_block_write_reg(o_ctrl, 0xED2C, (unsigned char*)(DW9786_FW + i), MCS_PKT_SIZE); // 16bit write
    }

	return rc;
}

int write_reg_16bit_value_8bit(o_ctrl, unsigned short addr, unsigned char data)
{
	u8 ret;
	u8 tx[1];
	tx[0] = (u8)data;
	ret = JKS_I2C_Write_Mem(&hi2c1, USER_MEM_ID, addr, I2C_16BIT, (uint8_t*)tx, 1, I2C_CLK);
	return ret;
}

int read_reg_16bit_value_8bit(o_ctrl, unsigned short addr, unsigned char* data)
{
	u8 tx[1];
	JKS_I2C_Read_Mem(&hi2c1, USER_MEM_ID, addr, I2C_16BIT, (uint8_t*)tx, 1, I2C_CLK);
	*data = (unsigned short)tx[0] << 8 | tx[1];
	return 0;
}

int write_reg_16bit_value_16bit(o_ctrl, unsigned short addr, unsigned short data)
{
	u8 ret;
	u8 tx[2];
	tx[0] = data >> 8;
	tx[1] = (u8)data;
	ret = JKS_I2C_Write_Mem(&hi2c1, DW9786_ID, addr, I2C_16BIT, (uint8_t*)tx, 2, I2C_CLK);
	return ret;
}


int read_reg_16bit_value_16bit(o_ctrl, unsigned short addr, unsigned short* data)
{
	u8 tx[2];
	JKS_I2C_Read_Mem(&hi2c1, DW9786_ID, addr, I2C_16BIT, (uint8_t*)tx, 2, I2C_CLK);
	*data = (unsigned short)tx[0] << 8 | tx[1];
	return 0;
}

unsigned short data_raw[40960];
int i2c_block_read_reg(unsigned short addr, unsigned short *PcSetDat, int size)
{
	JKS_I2C_Read_Mem(&hi2c1, g_i2c_main_id, addr, I2C_16BIT, (uint8_t *) data_raw, datalen*2, g_i2c_clk);
	for(int i=0; i<datalen; i++)
	{
		PcSetDat[i] = ((uint8_t)(data_raw[i] >> 8) & 0xff) + ((uint8_t)(data_raw[i] & 0xff) << 8);
	}
	return 0;
}

int i2c_block_write_reg(o_ctrl, unsigned short addr, unsigned char* data, int size)
{
	JKS_I2C_Write_Mem(&hi2c1, DW9786_ID, addr, I2C_16BIT, (uint8_t*)data, size, I2C_CLK);
	return 0;
}
#endif
