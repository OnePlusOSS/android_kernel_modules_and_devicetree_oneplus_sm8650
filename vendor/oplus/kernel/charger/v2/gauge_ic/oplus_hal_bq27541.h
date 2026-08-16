/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2022 Oplus. All rights reserved.
 */

#ifndef __OPLUS_BQ27541_H__
#define __OPLUS_BQ27541_H__

#include <linux/regmap.h>
#include <oplus_chg_ic.h>
#include <oplus_mms_gauge.h>

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
#include <debug-kit.h>
#endif

#define OPLUS_USE_FAST_CHARGER
#define DRIVER_VERSION			"1.1.0"

/* Bq28Z610 standard data commands */
#define BQ28Z610_REG_TI			0x0c
#define BQ28Z610_REG_AI			0x14

/* Bq27541 standard data commands */
#define BQ27541_REG_CNTL		0x00
#define BQ27541_REG_AR			0x02
#define BQ27541_REG_ARTTE		0x04
#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_NAC			0x0C
#define BQ27541_REG_FAC			0x0e
#define BQ27541_REG_RM			0x10
#define BQ27541_REG_FCC			0x12
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_TTE			0x16
#define BQ27541_REG_TTF			0x18
#define BQ27541_REG_SI			0x1a
#define BQ27541_REG_STTE		0x1c
#define BQ27541_REG_MLI			0x1e
#define BQ27541_REG_MLTTE		0x20
#define BQ27541_REG_AE			0x22
#define BQ27541_REG_AP			0x24
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_INTTEMP		0x28
#define BQ27541_REG_CC			0x2a
#define BQ27541_REG_SOH			0x2E
#define BQ27541_REG_SOC			0x2c
#define BQ27541_REG_NIC			0x2e
#define BQ27541_REG_ICR			0x30
#define BQ27541_REG_LOGIDX		0x32
#define BQ27541_REG_LOGBUF		0x34
#define BQ27541_REG_DOD0		0x36
#define BQ27541_FLAG_DSC		BIT(0)
#define BQ27541_FLAG_FC			BIT(9)
#define BQ27541_CS_DLOGEN		BIT(15)
#define BQ27541_CS_SS			BIT(13)

/* Control subcommands */
#define BQ27541_SUBCMD_CTNL_STATUS		0x0000
#define BQ27541_SUBCMD_DEVCIE_TYPE		0x0001
#define BQ27541_SUBCMD_FW_VER			0x0002
#define BQ27541_SUBCMD_HW_VER			0x0003
#define BQ27541_SUBCMD_DF_CSUM			0x0004
#define BQ27541_SUBCMD_PREV_MACW		0x0007
#define BQ27541_SUBCMD_CHEM_ID			0x0008
#define BQ27541_SUBCMD_BD_OFFSET		0x0009
#define BQ27541_SUBCMD_INT_OFFSET		0x000a
#define BQ27541_SUBCMD_CC_VER			0x000b
#define BQ27541_SUBCMD_OCV			0x000c
#define BQ27541_SUBCMD_BAT_INS			0x000d
#define BQ27541_SUBCMD_BAT_REM			0x000e
#define BQ27541_SUBCMD_SET_HIB			0x0011
#define BQ27541_SUBCMD_CLR_HIB			0x0012
#define BQ27541_SUBCMD_SET_SLP			0x0013
#define BQ27541_SUBCMD_CLR_SLP			0x0014
#define BQ27541_SUBCMD_FCT_RES			0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG		0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG		0x0019
#define BQ27541_SUBCMD_SEALED			0x0020
#define BQ27541_SUBCMD_ENABLE_IT		0x0021
#define BQ27541_SUBCMD_DISABLE_IT		0x0023
#define BQ27541_SUBCMD_CAL_MODE			0x0040
#define BQ27541_SUBCMD_RESET			0x0041
#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN	(-2731)
#define BQ27541_INIT_DELAY			((HZ)*1)


/*----------------------- Bq27411 standard data commands----------------------------------------- */

#define BQ27411_REG_CNTL		0x00
#define BQ27411_REG_TEMP		0x02
#define BQ27411_REG_VOLT		0x04
#define BQ27411_REG_FLAGS		0x06
#define BQ27411_REG_NAC			0x08
#define BQ27411_REG_FAC			0x0a
#define BQ27411_REG_RM			0x0c
#define BQ27411_REG_FCC			0x2c
#define BQ27411_REG_AI			0x10
#define BQ27411_REG_SI			0x12
#define BQ27411_REG_MLI			0x14
#define BQ27411_REG_AP			0x18
#define BQ27411_REG_SOC			0x1c
#define BQ27411_REG_INTTEMP		0x1e
#define BQ27411_REG_SOH			0x20
#define BQ27411_REG_FC			0x0e
#define BQ27411_REG_QM			0x16
#define BQ27411_REG_PD			0x1a
#define BQ27411_REG_RCU			0x28
#define BQ27411_REG_RCF			0x2a
#define BQ27411_REG_FCU			0x2c
#define BQ27411_REG_FCF			0x2e
#define BQ27411_REG_SOU			0x30
#define BQ27411_REG_DO0			0x66
#define BQ27411_REG_DOE			0x68
#define BQ27411_REG_TRM			0x6a
#define BQ27411_REG_PC			0x6c
#define BQ27411_REG_QS			0x6e
#define BQ27411_FLAG_DSC		BIT(0)
#define BQ27411_FLAG_FC			BIT(9)
#define BQ27411_CS_DLOGEN		BIT(15)
#define BQ27411_CS_SS			BIT(13)

/* Bq27411 sub commands */
#define BQ27411_SUBCMD_CNTL_STATUS	0x0000
#define BQ27411_SUBCMD_DEVICE_TYPE	0x0001
#define BQ27411_SUBCMD_FW_VER		0x0002
#define BQ27411_SUBCMD_DM_CODE		0x0004
#define BQ27411_SUBCMD_CONFIG_MODE	0x0006
#define BQ27411_SUBCMD_PREV_MACW	0x0007
#define BQ27411_SUBCMD_CHEM_ID		0x0008
#define BQ27411_SUBCMD_SET_HIB		0x0011
#define BQ27411_SUBCMD_CLR_HIB		0x0012
#define BQ27411_SUBCMD_SET_CFG		0x0013
#define BQ27411_SUBCMD_SEALED		0x0020
#define BQ27411_SUBCMD_RESET		0x0041
#define BQ27411_SUBCMD_SOFTRESET	0x0042
#define BQ27411_SUBCMD_EXIT_CFG		0x0043
#define BQ27411_SUBCMD_ENABLE_DLOG	0x0018
#define BQ27411_SUBCMD_DISABLE_DLOG	0x0019
#define BQ27411_SUBCMD_ENABLE_IT	0x0021
#define BQ27411_SUBCMD_DISABLE_IT	0x0023
#define BQ27541_BQ27411_CMD_INVALID	0xFF

/*----------------------- Bq27541 standard data commands-----------------------------------------*/
#define BQ27541_BQ27411_REG_CNTL		0
#define BQ27541_BQ27411_CS_DLOGEN		BIT(15)
#define BQ27541_BQ27411_CS_SS			BIT(13)
#define BQ27541_BQ27411_SUBCMD_CTNL_STATUS	0x0000
#define BQ27541_BQ27411_SUBCMD_ENABLE_IT	0x0021
#define BQ27541_BQ27411_SUBCMD_ENABLE_DLOG	0x0018
#define BQ27541_BQ27411_SUBCMD_DEVICE_TYPE	0x0001
#define BQ27541_BQ27411_SUBCMD_FW_VER		0x0002
#define BQ27541_BQ27411_SUBCMD_DISABLE_DLOG	0x0019
#define BQ27541_BQ27411_SUBCMD_DISABLE_IT	0x0023

#define CAPACITY_SALTATE_COUNTER		4
#define CAPACITY_SALTATE_COUNTER_NOT_CHARGING	20
#define CAPACITY_SALTATE_COUNTER_80		30
#define CAPACITY_SALTATE_COUNTER_90		40
#define CAPACITY_SALTATE_COUNTER_95		60
#define CAPACITY_SALTATE_COUNTER_FULL		120

#define BATTERY_2700MA				0
#define BATTERY_3000MA				1
#define TYPE_INFO_LEN				8

#define DEVICE_TYPE_BQ27541			0x0541
#define DEVICE_TYPE_BQ27411			0x0421
#define DEVICE_TYPE_BQ28Z610			0xFFA5
#define DEVICE_TYPE_ZY0602			0x0602
#define DEVICE_TYPE_ZY0603			0xA5FF

#define DEVICE_BQ27541				0
#define DEVICE_BQ27411				1
#define DEVICE_BQ28Z610				2
#define DEVICE_ZY0602				3
#define DEVICE_ZY0603				4
#define DEVICE_NFG8011B			5

#define DEVICE_TYPE_FOR_VOOC_BQ27541		0
#define DEVICE_TYPE_FOR_VOOC_BQ27411		1


#define CONTROL_CMD				0x00
#define CONTROL_STATUS				0x00
#define SEAL_POLLING_RETRY_LIMIT		100
#define BQ27541_UNSEAL_KEY			0x11151986
#define BQ27411_UNSEAL_KEY			0x80008000

#define BQ27541_RESET_SUBCMD			0x0041
#define BQ27411_RESET_SUBCMD			0x0042
#define SEAL_SUBCMD				0x0020

#define BQ27411_CONFIG_MODE_POLLING_LIMIT	100
#define BQ27411_CONFIG_MODE_BIT			BIT(4)
#define BQ27411_BLOCK_DATA_CONTROL		0x61
#define BQ27411_DATA_CLASS_ACCESS		0x003e
#define BQ27411_CC_DEAD_BAND_ID			0x006b
#define BQ27411_CC_DEAD_BAND_ADDR		0x42
#define BQ27411_CHECKSUM_ADDR			0x60
#define BQ27411_CC_DEAD_BAND_POWERUP_VALUE	0x11
#define BQ27411_CC_DEAD_BAND_SHUTDOWN_VALUE	0x71

#define BQ27411_OPCONFIGB_ID			0x0040
#define BQ27411_OPCONFIGB_ADDR			0x42
#define BQ27411_OPCONFIGB_POWERUP_VALUE		0x07
#define BQ27411_OPCONFIGB_SHUTDOWN_VALUE	0x0f

#define BQ27411_DODATEOC_ID			0x0024
#define BQ27411_DODATEOC_ADDR			0x48
#define BQ27411_DODATEOC_POWERUP_VALUE		0x32
#define BQ27411_DODATEOC_SHUTDOWN_VALUE		0x32

/*----------------------- Bq27541 standard data commands-----------------------------------------*/
#define BQ28Z610_REG_CNTL1			0x3e
#define BQ28Z610_REG_CNTL2			0x60
#define BQ28Z610_SEAL_POLLING_RETRY_LIMIT	100

#define BQ28Z610_SEAL_STATUS			0x0054
#define BQ28Z610_SEAL_SUBCMD			0x0030
#define BQ28Z610_UNSEAL_SUBCMD1			0x0414
#define BQ28Z610_UNSEAL_SUBCMD2			0x3672
#define BQ28Z610_CNTL1_DA_CONFIG		0x46fd
#define BQ28Z610_SEAL_BIT			(BIT(0) | BIT(1))

#define BQ28Z610_SEAL_SHIFT			8
#define BQ28Z610_SEAL_VALUE			3
#define BQ28Z610_MAC_CELL_VOLTAGE_ADDR		0x40
#define BQ28Z610_REG_CNTL1_SIZE			4

#define BQ28Z610_REG_I2C_SIZE			3

#define BQ28Z610_REG_GAUGE_EN			0x0057
#define BQ28Z610_GAUGE_EN_BIT			BIT(3)

#define BQ28Z610_MAC_CELL_VOLTAGE_EN_ADDR	0x3E
#define BQ28Z610_MAC_CELL_VOLTAGE_CMD		0x0071
#define BQ28Z610_MAC_CELL_VOLTAGE_ADDR		0x40
#define BQ28Z610_MAC_CELL_VOLTAGE_SIZE		4 /* total 34byte,only read 4byte(aaAA bbBB) */

#define ZY602_MAC_CELL_DOD0_EN_ADDR		0x00
#define ZY602_MAC_CELL_DOD0_CMD			0x00E3
#define ZY602_MAC_CELL_DOD0_ADDR		0x40
#define ZY602_MAC_CELL_DOD0_SIZE		12

#define BQ28Z610_MAC_CELL_DOD0_EN_ADDR		0x3E
#define BQ28Z610_MAC_CELL_DOD0_CMD		0x0074
#define BQ28Z610_MAC_CELL_DOD0_ADDR		0x4A
#define BQ28Z610_MAC_CELL_DOD0_SIZE		6

#define ZY0603_MAC_CELL_SOCCAL0			0x56

#define ZY602_MAC_CELL_QMAX_EN_ADDR		0x00
#define ZY602_MAC_CELL_QMAX_CMD			0x00E4
#define ZY602_MAC_CELL_QMAX_ADDR_A		0x40
#define ZY602_MAC_CELL_QMAX_SIZE_A		18

#define BQ28Z610_MAC_CELL_QMAX_EN_ADDR		0x3E
#define BQ28Z610_MAC_CELL_QMAX_CMD		0x0075
#define BQ28Z610_MAC_CELL_QMAX_ADDR_A		0x40
#define BQ28Z610_MAC_CELL_QMAX_ADDR_B		0x48
#define BQ28Z610_MAC_CELL_QMAX_SIZE_A		4
#define BQ28Z610_MAC_CELL_QMAX_SIZE_B		2

#define BQ28Z610_MAC_CELL_BALANCE_TIME_EN_ADDR	0x3E
#define BQ28Z610_MAC_CELL_BALANCE_TIME_CMD	0x0076
#define BQ28Z610_MAC_CELL_BALANCE_TIME_ADDR	0x40
#define BQ28Z610_MAC_CELL_BALANCE_TIME_SIZE	4 /* total 10byte,only read 4byte(aaAA bbBB) */

#define BQ28Z610_OPERATION_STATUS_EN_ADDR	0x3E
#define BQ28Z610_OPERATION_STATUS_CMD		0x0054
#define BQ28Z610_OPERATION_STATUS_ADDR		0x40
#define BQ28Z610_OPERATION_STATUS_SIZE		4
#define BQ28Z610_BALANCING_CONFIG_BIT		BIT(28)

#define BQ28Z610_DEVICE_CHEMISTRY_EN_ADDR	0x3E
#define BQ28Z610_DEVICE_CHEMISTRY_CMD		0x4B
#define BQ28Z610_DEVICE_CHEMISTRY_ADDR		0x40
#define BQ28Z610_DEVICE_CHEMISTRY_SIZE		4

#define BQ28Z610_DEEP_DISCHG_CHECK 		0xFF
#define BQ28Z610_DEEP_DISCHG_NUM_CMD		0x4082
#define BQ28Z610_DEEP_DISCHG_NAME_CMD		0x004A
#define BQ28Z610_DEEP_DISCHG_SIZE		12
#define BQ28Z610_DEEP_DISCHG_CEHECK_SIZE	14
#define BQ28Z610_TERM_VOLT_CMD			0x45BE
#define BQ28Z610_TERM_VOLT_SIZE			4
#define BQ28Z610_TERM_VOLT_S_CMD		0x45C3
#define BQ28Z610_TERM_VOLT_CHECK_ADDR		0x60
#define BQ28Z610_TERM_VOLT_CHECK_SIZE		6
#define BQ28Z610_DEEP_DISCHG_SHIFT_MASK		8

#define BQ28Z610_BATT_SN_EN_ADDR		0x3E
#define BQ28Z610_BATT_SN_CMD			0x004C
#define BQ28Z610_BATT_SN_READ_BUF_LEN		22
#define BQ28Z610_BATTINFO_NO_CHECKSUM		0x00
#define BQ28Z610_BATT_SN_RETRY_MAX		3

#define BQ28Z610_VCT_NAME_CMD			0x46F8
#define BQ28Z610_VCT_SIZE			4
#define BQ28Z610_VCT_MASK			0xFF
#define BQ28Z610_VCT_CHECK_SIZE			2
#define BQ28Z610_VCT_SHIFT			8
#define BQ28Z610_VCT_CHECK_ADDR			0x60

#define ZY0602_BATT_SN_EN_ADDR			0x3F
#define ZY0602_BATT_SN_READ_ADDR		0x40
#define ZY0602_BATT_SN_READ_BUF_LEN		32

#define ZY0603_CMDMASK_ALTMAC_R			0x08000000
#define ZY0603_CMDMASK_ALTMAC_W			0x88000000
#define ZY0603_FWVERSION			0x0002
#define ZY0603_FWVERSION_DATA_LEN		2


#define ZY0603_CURRENT_REG			0x0C
#define ZY0603_VOL_REG				0x08

#define ZY0603_READ_QMAX_CMD			0x0075
#define ZY0603_QMAX_DATA_OFFSET			0
#define ZY0603_QMAX_LEN				4
#define ZY0603_MAX_QMAX_THRESHOLD		32500

#define ZY0603_READ_SFRCC_CMD			0x0073
#define ZY0603_SFRCC_DATA_OFFSET		12
#define ZY0603_SFRCC_LEN			2

#define ZY0603_CTRL_REG				0x00
#define ZY0603_CMD_ALTMAC			0x3E
#define ZY0603_CMD_ALTBLOCK 			0x40
#define ZY0603_CMD_ALTCHK 			0x60
#define ZY0603_CMD_SBS_DELAY 			3000
#define ZY0603_SOFT_RESET_VERSION_THRESHOLD	0x0015
#define ZY0603_SOFT_RESET_CMD			0x21

#define ZY0603_MODELRATIO_REG			0x4714
#define ZY0603_GFCONFIG_CHGP_REG		0x4752
#define ZY0603_GFCONFIG_R2D_REG			0x475A
#define ZY0603_GFMAXDELTA_REG			0x479B

#define BQ27541_BLOCK_SIZE			32
#define BQ28Z610_EXTEND_DATA_SIZE		34
#define BQ28Z610_REG_TRUE_FCC			0x0073
#define BQ28Z610_TRUE_FCC_NUM_SIZE		2
#define BQ28Z610_TRUE_FCC_OFFSET		8
#define BQ28Z610_FCC_SYNC_CMD			0x0043

#define BQ28Z610_BATT_INFO_RETRY_MAX		3
#define BQ28Z610_BATTINFO_EN_ADDR		0x3E
#define BQ28Z610_BATTINFO_MANUDATE_CMD		0x004D
#define BQ28Z610_BATT_MANU_DATE_READ_BUF_LEN	4

#define BQ28Z610_BATTINFO_VDMDATA_CMD		0x0070
#define BQ28Z610_BATT_VDM_DATA_READ_BUF_LEN	34
#define BQ28Z610_BATTINFO_DEFAULT_CHECKSUM	0xFF

#define BQ28Z610_BATTINFO_NO_CHECKSUM		0x00
#define BQ28Z610_BATT_USED_FLAG			23

#define BQ28Z610_BATT_FIRST_USAGE_DATE_WLEN	5
#define BQ28Z610_BATT_FIRST_USAGE_DATE_WADDR	0x404E
#define BQ28Z610_BATT_FIRST_USAGE_DATE_CHECK	17
#define BQ28Z610_BATT_FIRST_USAGE_DATE_L	15
#define BQ28Z610_BATT_FIRST_USAGE_DATE_H	16

#define BQ28Z610_BATT_UI_CYCLE_COUNT_CHECK	22
#define BQ28Z610_BATT_UI_CYCLE_COUNT_L		20
#define BQ28Z610_BATT_UI_CYCLE_COUNT_H		21

#define BQ28Z610_BATT_UI_SOH			18
#define BQ28Z610_BATT_UI_SOH_CHECK		19
#define BQ28Z610_BATT_WRITE_CHECK_SUM_ADDR	0x60
#define BQ28Z610_BATT_USED_FLAG_CHECK		24

#define BQ28Z610_BATT_UI_CC_WLEN		5
#define BQ28Z610_BATT_UI_CC_WADDR		0x4053
#define BQ28Z610_BATT_UI_SOH_WLEN		4
#define BQ28Z610_BATT_UI_SOH_WADDR		0x4051
#define BQ28Z610_BATT_USED_FLAG_WLEN		4
#define BQ28Z610_BATT_USED_FLAG_WADDR		0x4056

#define BQ28Z610_REG_DELTA_VOL                  0x4219
#define BQ28Z610_REG_CELL0_RA                   0x4116
#define BQ28Z610_REG_CELL1_RA                   0x4156

#define U_DELAY_1_MS	1000
#define U_DELAY_5_MS	5000
#define M_DELAY_10_S	10000

typedef enum {
	DOUBLE_SERIES_WOUND_CELLS = 0,
	SINGLE_CELL,
	DOUBLE_PARALLEL_WOUND_CELLS,
} SCC_CELL_TYPE;

typedef enum {
	TI_GAUGE = 0,
	SW_GAUGE,
	NFG_GAUGE,
	UNKNOWN_GAUGE_TYPE,
} SCC_GAUGE_TYPE;

enum gauge_not_allow_op_type {
	SUSPEND = 1,
	FPGA_ERR,
	SHUTDOWN,
};

enum rst_type {
	FPGA_RST = 1,
	I2C_RST,
};

#define BCC_PARMS_COUNT		19
#define BCC_PARMS_COUNT_LEN	69
#define ZY0602_KEY_INDEX	0X02

#define BQ28Z610_DATAFLASHBLOCK		0x3e
#define BQ28Z610_SUBCMD_CHEMID		0x0006
#define BQ28Z610_SUBCMD_GAUGEING_STATUS	0x0056
#define BQ28Z610_SUBCMD_DA_STATUS1	0x0071
#define BQ28Z610_SUBCMD_IT_STATUS1	0x0073
#define BQ28Z610_SUBCMD_IT_STATUS2	0x0074
#define BQ28Z610_SUBCMD_IT_STATUS3	0x0075
#define BQ28Z610_SUBCMD_CB_STATUS	0x0076
#define BQ28Z610_SUBCMD_TRY_COUNT	3
#define CALIB_TIME_CHECK_ARGS		6

#define GAUGE_SUBCMD_TRY_COUNT	3
#define GAUGE_EXTERN_DATAFLASHBLOCK	0x3e
#define GAUGE_SUBCMD_DEVICE_TYPE	0X0001

struct cmd_address {
	/*      bq27411 standard cmds     */
	u8 reg_cntl;
	u8 reg_temp;
	u8 reg_volt;
	u8 reg_flags;
	u8 reg_nac;
	u8 reg_fac;
	u8 reg_rm;
	u8 reg_fcc;
	u8 reg_ai;
	u8 reg_si;
	u8 reg_mli;
	u8 reg_ap;
	u8 reg_soc;
	u8 reg_soc_centi;
	u8 reg_inttemp;
	u8 reg_soh;
	u8 reg_fc; /* add gauge reg print log start */
	u8 reg_qm;
	u8 reg_pd;
	u8 reg_rcu;
	u8 reg_rcf;
	u8 reg_fcu;
	u8 reg_fcf;
	u8 reg_sou;
	u8 reg_do0;
	u8 reg_doe;
	u8 reg_trm;
	u8 reg_pc;
	u8 reg_qs; /* add gauge reg print log end */
	u16 flag_dsc;
	u16 flag_fc;
	u16 cs_dlogen;
	u16 cs_ss;

	/*     bq27541 external standard cmds      */
	u8 reg_ar;
	u8 reg_artte;
	u8 reg_tte;
	u8 reg_ttf;
	u8 reg_stte;
	u8 reg_mltte;
	u8 reg_ae;
	u8 reg_ttecp;
	u8 reg_cc;
	u8 reg_nic;
	u8 reg_icr;
	u8 reg_logidx;
	u8 reg_logbuf;
	u8 reg_dod0;

	/*      bq27411 sub cmds       */
	u16 subcmd_cntl_status;
	u16 subcmd_device_type;
	u16 subcmd_fw_ver;
	u16 subcmd_dm_code;
	u16 subcmd_prev_macw;
	u16 subcmd_chem_id;
	u16 subcmd_set_hib;
	u16 subcmd_clr_hib;
	u16 subcmd_set_cfg;
	u16 subcmd_sealed;
	u16 subcmd_reset;
	u16 subcmd_softreset;
	u16 subcmd_exit_cfg;
	u16 subcmd_enable_dlog;
	u16 subcmd_disable_dlog;
	u16 subcmd_enable_it;
	u16 subcmd_disable_it;

	/*      bq27541 external sub cmds     */
	u16 subcmd_hw_ver;
	u16 subcmd_df_csum;
	u16 subcmd_bd_offset;
	u16 subcmd_int_offset;
	u16 subcmd_cc_ver;
	u16 subcmd_ocv;
	u16 subcmd_bat_ins;
	u16 subcmd_bat_rem;
	u16 subcmd_set_slp;
	u16 subcmd_clr_slp;
	u16 subcmd_fct_res;
	u16 subcmd_cal_mode;
};

#define BQ27541_AUTHENTICATE_OK		0x56
#define AUTHEN_MESSAGE_MAX_COUNT	30
struct bq27541_authenticate_data {
	uint8_t result;
	uint8_t message_offset;
	uint8_t message_len;
	uint8_t message[AUTHEN_MESSAGE_MAX_COUNT]; /* 25, larger than 20 bytes */
};

#define BQ27541_AUTHENTICATE_DATA_COUNT		sizeof(struct bq27541_authenticate_data)
#define SMEM_RESERVED_BOOT_INFO_FOR_APPS	418
#define GAUGE_AUTH_MSG_LEN			20
#define WLS_AUTH_RANDOM_LEN			8
#define WLS_AUTH_ENCODE_LEN			8
#define GAUGE_SHA256_AUTH_MSG_LEN		32
#define UFCS_AUTH_MSG_LEN			16

typedef struct {
	int result;
	unsigned char msg[GAUGE_AUTH_MSG_LEN];
	unsigned char rcv_msg[GAUGE_AUTH_MSG_LEN];
} oplus_gauge_auth_result;

struct wls_chg_auth_result {
	unsigned char random_num[WLS_AUTH_RANDOM_LEN];
	unsigned char encode_num[WLS_AUTH_ENCODE_LEN];
};

typedef struct {
	unsigned char msg[UFCS_AUTH_MSG_LEN];
} oplus_ufcs_auth_result;

struct oplus_gauge_sha256_auth_result {
	unsigned char msg[GAUGE_SHA256_AUTH_MSG_LEN];
	unsigned char rcv_msg[GAUGE_SHA256_AUTH_MSG_LEN];
};

typedef struct {
	oplus_gauge_auth_result rst_k0;
	oplus_gauge_auth_result rst_k1;
	struct wls_chg_auth_result wls_auth_data;
	oplus_gauge_auth_result rst_k2;
	oplus_ufcs_auth_result ufcs_k0;
	struct oplus_gauge_sha256_auth_result sha256_rst_k0;
} oplus_gauge_auth_info_type;

struct oplus_gauge_sha256_auth{
	unsigned char random[GAUGE_SHA256_AUTH_MSG_LEN];
	unsigned char ap_encode[GAUGE_SHA256_AUTH_MSG_LEN];
	unsigned char gauge_encode[GAUGE_SHA256_AUTH_MSG_LEN];
};

struct fcc_ra_vdelta_curve {
	int delta_vmax;
	int delta_cc;
};

struct fcc_ra_config {
	int ra_vd_cc0;
	int ra_t_cc0;
	int ra_cc1;
	int l_cc;
	int ra_cnts;
	int dbg_cc;
};

#define FCC_RA_CC_THR_NUM 5
#define FCC_RA_DEFAULT_NUM 15
struct ra_k_curve {
	uint32_t k0;
	uint32_t k1;
	uint32_t k2;
	uint32_t k3;
	uint32_t k4;
};

struct fcc_ra_t_curve {
	uint32_t cc_index;
	uint32_t delta_cc0;
	uint32_t delta_cc1;
	uint32_t fcc_thr;
	uint32_t cc_thr[FCC_RA_CC_THR_NUM];
	uint32_t fcc0_thr[FCC_RA_CC_THR_NUM];
	uint32_t extreme_thr[FCC_RA_CC_THR_NUM];
	struct ra_k_curve k_curve[FCC_RA_CC_THR_NUM];
};

struct fcc_ra_res_curve {
	uint32_t ra_default[FCC_RA_DEFAULT_NUM];
	uint32_t ra_thr[FCC_RA_DEFAULT_NUM];
	uint32_t ra_0[FCC_RA_DEFAULT_NUM];
	uint32_t ra_1[FCC_RA_DEFAULT_NUM];
	uint32_t ra_cali[FCC_RA_DEFAULT_NUM];
};

#define FCC_INFO_LEN 1023
struct fcc_track_info {
	unsigned char ra0_msg[FCC_INFO_LEN];
	unsigned char vdelta_msg[FCC_INFO_LEN];
	unsigned char ra_t_msg[FCC_INFO_LEN];
	int ra0_index;
	int vdelta_index;
	int ra_t_index;
	int ra_t_err;
};

struct chip_bq27541 {
	struct i2c_client *client;
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CHG_DEBUG_KIT)
	struct regmap *regmap;
	struct oplus_device_bus *odb;
#endif

	atomic_t locked;

	int soc_pre;
	int soc_centi_pre;
	int temp_pre;
	int batt_vol_pre;
	int current_pre;
	int cc_pre;
	int soh_pre;
	int fcc_pre;
	int rm_pre;
	int fc_pre; /* add gauge reg print log start */
	int qm_pre;
	int pd_pre;
	int rcu_pre;
	int rcf_pre;
	int fcu_pre;
	int fcf_pre;
	int sou_pre;
	int do0_pre;
	int passed_q_pre;
	int doe_pre;
	int trm_pre;
	int pc_pre;
	int qs_pre; /* add gauge reg print log end */
	int volt_1_pre;
	int volt_2_pre;
	int dod0_1_pre;
	int dod0_2_pre;
	int dod_passed_q_pre;
	int qmax_1_pre;
	int qmax_2_pre;
	int qmax_passed_q_pre;
	int car_c_pre;
	int device_type;
	int device_type_for_vooc;
	struct cmd_address cmd_addr;
	unsigned long operate_allow;
	int batt_cell_1_vol;
	int batt_cell_2_vol;
	int batt_cell_max_vol;
	int batt_cell_min_vol;
	int max_vol_pre;
	int min_vol_pre;
	int batt_num;

	bool fcc_too_small_checking;
	struct work_struct fcc_too_small_check_work;
	struct work_struct imp_model_check_work;

	bool modify_soc_smooth;
	bool modify_soc_calibration;

	bool battery_full_param; /* only for wite battery full param in guage dirver probe on 7250 platform */
	int sha1_key_index;
	struct delayed_work afi_update;
	bool afi_update_done;
	bool protect_check_done;
	bool disabled;
	bool error_occured;
	bool need_check;
	unsigned int afi_count;
	unsigned int zy_dts_qmax_min;
	unsigned int zy_dts_qmax_max;
	const u8 *static_df_checksum_3e;
	const u8 *static_df_checksum_60;
	const u8 **afi_buf;
	unsigned int *afi_buf_len;
	u8 *bq28z610_afi_buf;
	int bq28z610_afi_cnt;
	bool fast_sampling_enable;
	bool batt_bq28z610;
	bool batt_bq27z561;
	bool batt_nfg8011b;
	bool batt_zy0603;
	bool bq28z610_need_balancing;
	bool enable_sleep_mode;
	bool zy0603_soc_opt;
	int bq28z610_device_chem;
	int fg_soft_version;
	int gauge_num;
	struct mutex chip_mutex;
	struct mutex calib_time_mutex;
	struct mutex bq28z610_alt_manufacturer_access;
	atomic_t i2c_err_count;
	bool i2c_err;
	oplus_gauge_auth_result auth_data;
	struct bq27541_authenticate_data *authenticate_data;
	struct oplus_gauge_sha256_auth *sha256_authenticate_data;
	struct file_operations *authenticate_ops;

	int batt_dod0_1;
	int batt_dod0_2;
	int batt_dod0_passed_q;

	int batt_qmax_1;
	int batt_qmax_2;
	int batt_qmax_passed_q;
	int bcc_buf[BCC_PARMS_COUNT];

	bool calib_info_init;
	bool calib_info_save_support;
	int dod_time;
	int qmax_time;
	int dod_time_pre;
	int qmax_time_pre;
	int calib_check_args_pre[CALIB_TIME_CHECK_ARGS];

	bool support_sha256_hmac;
	bool support_extern_cmd;
	bool gauge_type_error;
	bool support_eco_design;

	struct delayed_work check_iic_recover;
	/* workaround for I2C pull SDA can't trigger error issue 230504153935012779 */
	bool i2c_rst_ext;
	bool err_status;
	/* end workaround 230504153935012779 */

	struct mutex pinctrl_lock;
	struct pinctrl *pinctrl;
	struct pinctrl_state *id_not_pull;
	struct pinctrl_state *id_pull_up;
	struct pinctrl_state *id_pull_down;
	int id_gpio;
	int id_match_status;
	int id_value;
	bool fpga_support;
#if IS_ENABLED(CONFIG_OPLUS_CHG_TEST_KIT)
	struct test_feature *battery_id_gpio_test;
	struct test_feature *fpga_fg_test;
#endif
	struct battery_manufacture_info battinfo;
	struct delayed_work get_manu_battinfo_work;
	int deep_dischg_count_pre;
	int deep_term_volt_pre;
	int vct_pre;
	bool dsg_enable;
	u8 chem_id[CHEM_ID_LENGTH + 1];
	int last_cc_pre;
	struct work_struct fgpa_reset_start_work;
	struct work_struct fgpa_reset_end_work;
	struct work_struct i2c_reset_end_work;
	struct delayed_work fgpa_i2c_timeout_work;
	struct work_struct fgpa_track_work;
	int fpga_rst_time;
	int i2c_err_time;
	atomic_t i2c_err_timeout_flag;
	unsigned long rst_ing;
	int gauge_type;
	int bq28z610_seal_flag;
	bool sn_match;

	struct mutex imp_model_lock;
	u8 *imp_model_data;
	bool imp_model_checking;
	struct delayed_work track_fcc_ra0_work;
	struct delayed_work track_fcc_vdelta_work;
	struct delayed_work track_fcc_ra_t_work;
	bool fcc_ra0_support;

	bool fcc_vdelta_support;
	struct fcc_ra_vdelta_curve ra_vd_curve;

	int fcc_ra_t_support;
	struct fcc_ra_t_curve ra_t_curve;
	struct fcc_ra_res_curve ra_res_curve;
	struct fcc_ra_config ra_config;
	struct fcc_track_info fcc_msg;
};

struct gauge_track_info_reg {
	int addr;
	int len;
	int start_index;
	int end_index;
};

struct gauge_sili_ic_alg_cfg_map {
	u32 map_src;
	u32 map_des;
};

extern bool oplus_gauge_ic_chip_is_null(
	struct oplus_chg_ic_dev *ic_dev);
int bq27541_i2c_txsubcmd(struct chip_bq27541 *chip, int cmd, int writeData);
int bq27541_read_i2c(struct chip_bq27541 *chip, int cmd, int *returnData);
int bq27541_read_i2c_block(struct chip_bq27541 *chip, u8 cmd, u8 length, u8 *returnData);
int bq27541_write_i2c_block(struct chip_bq27541 *chip, u8 cmd, u8 length, u8 *writeData);
int bq27541_i2c_txsubcmd_onebyte(struct chip_bq27541 *chip, u8 cmd, u8 writeData);
bool is_return_pre_value(struct chip_bq27541 *chip);
bool bq8z610_deep_init(struct chip_bq27541 *chip);
void bq8z610_deep_deinit(struct chip_bq27541 *chip);
int bq27541_get_battery_cc(struct chip_bq27541 *chip);
int bq27541_get_battery_fcc(struct chip_bq27541 *chip);
int bq28z610_get_2cell_voltage(struct chip_bq27541 *chip);
int bq28z610_get_qmax_parameters(struct chip_bq27541 *chip, int *batt_qmax_1,
	int *batt_qmax_2, int *batt_qmax_passed_q);
int bq27541_get_battery_soc(struct chip_bq27541 *chip);
int bq27541_get_battery_temperature(struct chip_bq27541 *chip);

#endif /* __OPLUS_BQ27541_H__ */
