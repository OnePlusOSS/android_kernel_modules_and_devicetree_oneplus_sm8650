// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2024 Oplus. All rights reserved.
 */

#ifndef __OPLUS_HAL_SC96257_H__
#define __OPLUS_HAL_SC96257_H__

#define SC96257_RX_PWR_15W		0x1E
#define SC96257_RX_PWR_11W		0x16
#define SC96257_RX_PWR_10W		0x14
#define SC96257_RX_PWR_5W		0x0A

#define SC96257_MTP_VOL_MV		5500

#define CUSTOMER_REGISTERS_BASE_ADDR	0x1000

#define FIRMWARE_LENGTH			(32 * 1024)

#define FW_VERSION_OFFSET		8
#define INVALID_FW_VERSION0		0x0
#define INVALID_FW_VERSION1		0xFFFFFFFF

#define FW_CHECKSUM_UNKNOWN		0
#define FW_CHECKSUM_OK			1
#define FW_CHECKSUM_FAIL		-1

#define SRAM_BASE_ADDRESS		0x1000

#define CRC_POLY			0x04c11db7
#define CRC_SIZE			4

#define FOD_SIZE			16

#define CHIP_SIZE			(32 * 1024)
#define SC_SECTOR_SIZE			256
#define CRC_SECTOR			64
#define PGM_WORD			4
#define PGM_INFO_SIZE			64

#define INFO_LENGTH			128

#define REG_TM				0x0011
#define TM_ST_BIT			0
#define TM_DATA				0xED

#define REG_ATE				0x0015

#define REG_RST				0x0076
#define RST_PW0				0x96
#define RST_PW1				0x00

#define REG_IDLE			0x0077
#define IDLE_PW0			0x5A
#define IDLE_PW1			0x00

#define REG_DIG				0x007F

#define REG_WDT				0xD5
#define WDT_CLOSE_PW0			0x33
#define WDT_CLOSE_PW1			0xCA
#define WDT_CLOSE_PW2			0

#define ATE_BOOT_BIT			3
/*REG_DIG*/
#define DIG_TM_BIT			0

#define DIG_PW0				0x51
#define DIG_PW1				0x49
#define DIG_PW2				0x57
#define DIG_PW3				0x45
#define DIG_PW4				0x4E

#define PGM_HEADER_SIZE			8
#define PGM_MSG_SIZE			0x100
#define PGM_XOR_LEN			6
#define PGM_XOR_INDEX			2

#define PGM_FRIMWARE_SIZE		64

/*The PGM will update the Status in the SRAM as follows:*/
#define PGM_STATE_NONE			0x00/*reset value*/

#define PGM_STATE_TIMEOUT		0x7f

#define PGM_STATE_DONE			0xa5/*operating done*/
#define PGM_STATE_BUSY			0xbb/*operating busy*/

#define PGM_STATE_ERRLEN		0xe0/*wrong message length*/
#define PGM_STATE_ERRXOR		0xe1/*wrong check sum*/
#define PGM_STATE_ERRPROG		0xe2/*read/write fail*/
#define PGM_STATE_ERRTIMEOUT		0xe3/*read/write timeout*/
#define PGM_STATE_ERRAUTH		0xe4/*authentication failed*/
#define PGM_STATE_ND			0xff/*command is not defined*/

/*AP command list*/
#define PGM_SRAM_BASE			0x180

#define PGM_CMD_NONE			0x00/*none command*/

#define PGM_CMD_INFO			0xa0/*get PGM information*/
#define PGM_CMD_MARGIN			0xb0/*set margin*/

#define PGM_CMD_VERIFY			0xc0/*get verify result*/
#define PGM_CMD_READ_CODE		0xc1/*read code memory*/
#define PGM_CMD_WRITE_CODE		0xc2/*write code memory*/

#define PGM_CMD_READ_TRIM		0xc3/*read data from TRIM area*/
#define PGM_CMD_WRITE_TRIM		0xc4/*write data to TRIM area*/
#define PGM_CMD_READ_CUST		0xc5/*read data from CUST area*/
#define PGM_CMD_WRITE_CUST		0xc6/*write data to CUST area*/
/*#define PGM_CMD_CRC_CODE		0xc7 read opt/mtp/*/
#define PGM_CMD_READ_ADC		0xca/*read adc*/

#define PGM_CMD_RANDOM			0xcc/*gets the random number used for authentication calculations*/
#define PGM_CMD_AUTH			0xcd/*authentication unlock command*/

#define PGM_CMD_CHIP_ERASE		0xce/*full chip erase*/
#define PGM_CMD_PAGE_ERASE		0xe9/*page erase*/
#define PGM_CMD_TRIM_ERASE		0xea/*trim area erase*/
#define PGM_CMD_CUST_ERASE		0xeb/*customer area erase*/
#define PGM_CMD_SECTOR_ERASE		0xee/*sector erase*/

#define PGM_STATE_ADDR			0x0000
#define PGM_CMD_ADDR			0x0001
#define PGM_LENGTH_ADDR			0x0004
#define PGM_CHECKSUM_ADDR		0x0006
#define PGM_MSG_ADDR			0x0008

#define READ_MAIN			0
#define READ_TRIM			1
#define READ_CUST			2

#define WRITE_MAIN			0
#define WRITE_TRIM			1
#define WRITE_CUST			2

#define CRC_CHECK_MARGIN0		0
#define CRC_CHECK_MARGIN7		7

#define MAX_ASK_SIZE			22
#define MAX_FSK_SIZE			10

#define WAKE_REG			0x0010
#define WAKE_PATTERN			0xAA

/**************work mode*************/
enum {
	WORK_MODE_RX,
	WORK_MODE_TX,
};

/*************AP command************/
/*sys*/
#define AP_CMD_CHIP_RESET		BIT(1)
#define AP_CMD_SEND_PPP			BIT(2)
#define AP_CMD_LVP_CHANGE		BIT(3)
#define AP_CMD_OVP_CHANGE		BIT(4)
#define AP_CMD_OCP_CHANGE		BIT(5)
#define AP_CMD_MAX_I_CHANGE		BIT(6)
#define AP_CMD_OTP_CHANGE		BIT(7)
#define AP_CMD_HW_WDT_DISABLE		BIT(10)
#define AP_CMD_HW_WDGT_FEED		BIT(11)
#define AP_CMD_REFRESH			BIT(31)
/*rx only*/
#define AP_CMD_RX_V2X_ON		BIT(8)
#define AP_CMD_RX_V2X_OFF		BIT(9)
#define AP_CMD_RX_SEND_EPT		BIT(12)
#define AP_CMD_RX_RENEG			BIT(13)
#define AP_CMD_RX_VOUT_CHANGE		BIT(14)
#define AP_CMD_RX_VOUT_ENABLE		BIT(15)
#define AP_CMD_RX_VOUT_DISABLE		BIT(16)
#define AP_CMD_RX_RPP_8BIT		BIT(17)
#define AP_CMD_RX_RPP_16BIT		BIT(18)
#define AP_CMD_RX_FOD_ENABLE		BIT(20)
/*tx only*/
#define AP_CMD_TX_FRQ_SHIFT		BIT(8)
#define AP_CMD_TX_ENABLE		BIT(9)
#define AP_CMD_TX_DISABLE		BIT(18)
#define AP_CMD_TX_START_PING		BIT(19)
#define AP_CMD_TX_FPWM_AUTO		BIT(20)
#define AP_CMD_TX_FPWM_MANUAL		BIT(21)
#define AP_CMD_TX_FPWM_UPDATE		BIT(22)
#define AP_CMD_TX_FPWM_CONFIG		BIT(23)
#define AP_CMD_TX_FOD_ENABLE		BIT(24)
#define AP_CMD_TX_FOD_DISABLE		BIT(25)
#define AP_CMD_TX_PING_OCP_CHANGE	BIT(26)
#define AP_CMD_TX_PING_OVP_CHANGE	BIT(27)
#define AP_CMD_TX_OPEN_LOOP		BIT(28)
#define AP_CMD_TX_ENABLE_WLSPEN		BIT(29)

/***************mode**************/
/*sys*/
#define STANDBY_MODE			BIT(0)
#define RX_MODE				BIT(1)
#define TX_MODE				BIT(2)
#define BPP_MODE			BIT(3)
#define EPP_MODE			BIT(4)
#define PRIVATE_MODE			BIT(5)
#define RPP_16BIT_MODE			BIT(6)
#define VSYS_MODE			BIT(7)
#define AC_MODE				BIT(8)
#define SLEEP_MODE			BIT(9)
#define HW_WDT_MODE			BIT(10)
/*tx only*/
#define FOD_MODE			BIT(11)
#define BRG_HALF_MODE			BIT(12)
#define TX_PING_MODE			BIT(14)
#define TX_PT_MODE			BIT(15)
/*rx only*/
#define PPP_BUSY_MODE			BIT(13)

/**************v2x state*************/
#define V2X_STATE_AC_DRV_R		BIT(9)
#define V2X_STATE_AC_PRESENT		BIT(8)
#define V2X_STATE_MIN			BIT(6)
#define V2X_STATE_MAX			BIT(5)
#define V2X_STATE_VV_OV			BIT(4)
#define V2X_STATE_VV_UV			BIT(3)
#define V2X_STATE_OVP			BIT(2)
#define V2X_STATE_UCP			BIT(1)
#define V2X_STATE_ON			BIT(0)

/**************irq*************/
/*sys*/
#define WP_IRQ_OCP			BIT(0)
#define WP_IRQ_OVP			BIT(1)
#define WP_IRQ_OVP0			BIT(2)
#define WP_IRQ_OVP1			BIT(3)
#define WP_IRQ_LVP			BIT(4)
#define WP_IRQ_OTP			BIT(5)
#define WP_IRQ_OTP_160			BIT(6)
#define WP_IRQ_SLEEP			BIT(7)
#define WP_IRQ_MODE_CHANGE		BIT(8)
#define WP_IRQ_PKT_RECV			BIT(9)
#define WP_IRQ_PPP_TIMEOUT		BIT(10)
#define WP_IRQ_PPP_SUCCESS		BIT(11)
#define WP_IRQ_AFC			BIT(12)
#define WP_IRQ_POWER_PROFILE		BIT(13)
/*rx only*/
#define WP_IRQ_RX_POWER_ON		BIT(14)
#define WP_IRQ_RX_SS_PKT		BIT(15)
#define WP_IRQ_RX_ID_PKT		BIT(16)
#define WP_IRQ_RX_CFG_PKT		BIT(17)
#define WP_IRQ_RX_READY			BIT(18)
#define WP_IRQ_RX_LDO_ON		BIT(19)
#define WP_IRQ_RX_LDO_OFF		BIT(20)
#define WP_IRQ_RX_LDO_OPP		BIT(21)
#define WP_IRQ_RX_SCP			BIT(22)
#define WP_IRQ_RX_RENEG_SUCCESS		BIT(23)
#define WP_IRQ_RX_RENEG_TIMEOUT		BIT(24)
#define WP_IRQ_RX_RENEG_FAIL		BIT(25)
#define WP_IRQ_RX_V2X_UCP		BIT(26)
#define WP_IRQ_RX_V2X_OVP		BIT(27)
#define WP_IRQ_RX_V2X_VV_UVP		BIT(28)
#define WP_IRQ_RX_V2X_VV_OVP		BIT(29)
#define WP_IRQ_RX_EPP_ID_SUCCESS	BIT(30)
/*tx only*/
#define WP_IRQ_TX_DET_RX		BIT(14)
#define WP_IRQ_TX_EPT			BIT(15)
#define WP_IRQ_TX_PT			BIT(16)
#define WP_IRQ_TX_FOD			BIT(17)
#define WP_IRQ_TX_DET_TX		BIT(18)
#define WP_IRQ_TX_CEP_TIMEOUT		BIT(19)
#define WP_IRQ_TX_RPP_TIMEOUT		BIT(20)
#define WP_IRQ_TX_PING			BIT(21)
#define WP_IRQ_TX_SS_PKT		BIT(22)
#define WP_IRQ_TX_ID_PKT		BIT(23)
#define WP_IRQ_TX_CFG_PKT		BIT(24)
#define WP_IRQ_TX_POCP			BIT(25)
#define WP_IRQ_TX_READY			BIT(26)
#define WP_IRQ_TX_AC_PRESENT		BIT(27)
#define WP_IRQ_TX_VAC_PRESENT		BIT(28)
#define WP_IRQ_TX_BLE_MAC_ADDR		BIT(30)
#define WP_IRQ_TX_WLSPEN_CHG_STATUS	BIT(31)

/**************ept reason*************/
enum ept_reason_e {
	/*The Receiver may use this value if it does not have a specific reason for terminating the power transfer,
	 *or if none of the other values listed in Table 6-6 is appropriate.
	 */
	EPT_UNKNOWN			= 0x00,
	/*The Receiver should use this value if it determines that the battery of the Mobile Device is fully charged.
	 *On receipt of an End Power Transfer Packet containing this value, the Transmitter should set any charged
	 *indication on its user interface that is associated with the Receiver.
	 */
	EPT_CHG_COMPLETE		= 0x01,
	/*The Receiver may use this value if it has encountered some internal problem, e.g. a software or logic error.*/
	EPT_INTERNAL_FAULT		= 0x02,
	/*The Receiver should use this value if it has measured a temperature within the Mobile Device that exceeds a limit.*/
	EPT_OVER_TEMPERATURE		= 0x03,
	/*The Receiver should use this value if it has measured a voltage within the Mobile Device that exceeds a limit.*/
	EPT_OVER_VOLTAGE		= 0x04,
	/*The Receiver should use this value if it has measured a current within the Mobile Device that exceeds a limit.*/
	EPT_OVER_CURRENT		= 0x05,
	/*The Receiver should use this value if it has determined a problem with the battery of the Mobile Device.*/
	EPT_BATTERY_FAILURE		= 0x06,
	EPT_RECONFIGURE			= 0x07,
	/*The Receiver should use this value if it determines that the Transmitter does not respond to
	 *Control Error Packets as expected (i.e. does not increase/decrease its Primary Cell current appropriately).
	 */
	EPT_NO_RESPONSE			= 0x08,
	EPT_RESERVED09			= 0x09,
	/*A Power Receiver should use this value if it cannot negotiate a suitable Guaranteed Power level.*/
	EPT_NEGOTIATION_FAILURE		= 0x0a,
	/*A Power Receiver should use this value if sees a need for Foreign Object Detection with no power transfer
	 *in progress (see Section 11.3, FOD based on quality factor change). To enable such detection, the power
	 *transfer has to be terminated. Typically, the Power Transmitter then performs Foreign Object Detection before
	 *restarting the power transfer.
	 */
	EPT_RESTART_POWER_TRANSFER	= 0x0b,
};

#define SC96257_RX_MODE_BPP		BPP_MODE
#define SC96257_RX_MODE_EPP		EPP_MODE

#define SC96257_ERR_IRQ_VALUE		0xFFFFFFFF

#define SC96257_TX_ERR_MASK		(BIT(0) | BIT(1) | BIT(2) | BIT(4) | BIT(5) | BIT(15) | BIT(17) | BIT(18) | BIT(19) | BIT(20))
#define SC96257_TX_ERR_RXAC		WP_IRQ_TX_DET_TX
#define SC96257_TX_ERR_OCP		WP_IRQ_OCP
#define SC96257_TX_ERR_OVP		WP_IRQ_OVP
#define SC96257_TX_ERR_VRECTOVP		WP_IRQ_OVP0
#define SC96257_TX_ERR_LVP		WP_IRQ_LVP
#define SC96257_TX_ERR_FOD		WP_IRQ_TX_FOD
#define SC96257_TX_ERR_OTP		WP_IRQ_OTP
#define SC96257_TX_ERR_CEPTIMEOUT	WP_IRQ_TX_CEP_TIMEOUT
#define SC96257_TX_ERR_RXEPT		WP_IRQ_TX_EPT
#define SC96257_TX_ERR_RPPTIMEOUT	WP_IRQ_TX_RPP_TIMEOUT

#define SC96257_TX_STATUS_MASK		(BIT(26) | BIT(21) | BIT(16))
#define SC96257_TX_STATUS_READY		WP_IRQ_TX_READY
#define SC96257_TX_STATUS_DIGITALPING	WP_IRQ_TX_PING
#define SC96257_TX_STATUS_TRANSFER	WP_IRQ_TX_PT

#define SC96257_FOD_PARM_LEN_MAX	32

#endif /* __OPLUS_HAL_SC96257_H__ */
