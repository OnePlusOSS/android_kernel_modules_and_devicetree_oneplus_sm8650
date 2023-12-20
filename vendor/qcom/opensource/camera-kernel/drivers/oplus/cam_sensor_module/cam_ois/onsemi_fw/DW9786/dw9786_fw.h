/**
  ******************************************************************************
  * File Name          : DW9786_SET_API.h
  * Description        : Main program header
  * DongWoon Anatech   :
  * Version            : 1.6
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 DWANATECH
  * DW9786 Setup Program for Huawei
  * Revision History
  * 2021.03.11 draft
	* 2023.05.12 apply OPLUS fw register map
  ******************************************************************************
**/

#ifndef _DW9786_OIS_H_
#define _DW9786_OIS_H_

#define LOOP_A									200
#define LOOP_B									LOOP_A-1
#define WAIT_TIME								200
#define MODE_ON									1
#define MODE_OFF								0
#define MCS_SEC									0

#define TIME_OVER								-1
#define COMMAND_ERROR							1
#define EXECUTE_ERROR							2
#define CHECKSUM_ERROR							4
#define VERIFY_ERROR							8

#define X_AXIS									1
#define Y_AXIS									2

#define GYRO_OFS_CAL_DONE_FAIL					0xFF
#define X_AXIS_GYRO_OFS_CAL_PASS				0x1
#define X_AXIS_GYRO_OFS_MAX_ERR					0x10

#define Y_AXIS_GYRO_OFS_CAL_PASS				0x2
#define Y_AXIS_GYRO_OFS_MAX_ERR					0x20

#define OK										0
#define NG										1
#define FUNC_PASS								0
#define FUNC_FAIL								-1

#define OPEN_INI_FILE_NG						1
#define OPEN_FW_FILE_ERROR						2
#define OPEN_DATA_FILE_ERROR					4
#define LACK_OF_SETTING_PARA					8
#define EXTFUNC_LOAD_ERROR						4

#define DW9786_CHIP_ID							0x9786
#define AUTORD_RV_OK							0x0000

#define MCS_START_ADDRESS						0x6000
#define LUT_START_ADDRESS						0x4000
#define dw_Gyro_gain_x                          0xB806
#define dw_Gyro_gain_y                          0xB808


/*-----------------Flash message define-----------------*/
#define FMC_PAGE								20
#define MCS_CODE								1
#define MCS_LUT									2
#define MCS_USER								3
#define MCS_START_ADDRESS						0x6000
#define MCS_PKT_SIZE							128
#define MCS_SIZE_W								0x5000
#define MCS_SIZE								0xA000
#define MCS_CHECKSUM_SIZE						10240

#define FW_START_ADDRESS                        0xED2C

struct _FACT_ADJ_DW{
	uint32_t	gl_GX_OFS;
	uint32_t	gl_GY_OFS;
};


struct _FACT_ADJ_DW dw9786_gyro_ofs_calibration(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_spi_master_mode(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_spi_intercept_mode(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_module_store(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_set_store(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_module_erase(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_set_erase(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_reset(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_mcu_active(struct cam_ois_ctrl_t *o_ctrl, unsigned short mode);
int dw9786_chip_enable(struct cam_ois_ctrl_t *o_ctrl, unsigned short en);
int dw9786_user_protection(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_download_fw(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_gyro_direction_setting(struct cam_ois_ctrl_t *o_ctrl, int gyro_arrangement, int gyro_degree);
int dw9786_set_gyro_select(struct cam_ois_ctrl_t *o_ctrl, char gyro_info);
void dw9786_chip_info(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_checksum_read(struct cam_ois_ctrl_t *o_ctrl);
int dw9786_checksum_func(struct cam_ois_ctrl_t *o_ctrl, int type);
void dw9786_flash_acess(struct cam_ois_ctrl_t *o_ctrl, int type);
int dw9786_code_update(struct cam_ois_ctrl_t *o_ctrl, int ac_id, uint32_t checksum);
int dw9786_auto_read_check(struct cam_ois_ctrl_t *o_ctrl);
int wait_check_register(struct cam_ois_ctrl_t *o_ctrl, unsigned short reg, unsigned short ref);
unsigned int checksum_exe(unsigned char *data, int leng);
void DW9786_WriteGyroGainToFlash(struct cam_ois_ctrl_t *o_ctrl,uint32_t X_gain, uint32_t Y_gain);
void DW9786_StoreGyroGainToFlash(struct cam_ois_ctrl_t *o_ctrl);

#endif



