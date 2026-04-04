/***********************************************************
** Copyright (C), 2025-2025 Oplus. All rights reserved.
** File: oplus_hal_nu2118a.h
** Description: cp ic
** Date: 2025-11-14
** -----------Revision History: -------------------------------
** <author>        <data>    <version >       <desc>
****************************************************************/
#ifndef _OPLUS_NU2118A_H_
#define _OPLUS_NU2118A_H_

/************************VOOCPHY REG***********************************/

/* Register 00h */
#define NU2118A_REG_00 0x00
#define NU2118A_BAT_OVP_DIS_MASK 0x80
#define NU2118A_BAT_OVP_DIS_SHIFT 7
#define NU2118A_BAT_OVP_ENABLE 0
#define NU2118A_BAT_OVP_DISABLE 1

/* Register 01h */
#define NU2118A_REG_01 0x01

/* Register 02h */
#define NU2118A_REG_02 0x02

/* Register 03h */
#define NU2118A_REG_03 0x03

/* Register 04h */
#define NU2118A_REG_04 0x04

/* Register 05h */
#define NU2118A_REG_05 0x05
#define NU2118A_CHG_EN_MASK 0x80
#define NU2118A_PMID2OUT_OVP_DIS_SHIFT 7
#define NU2118A_PMID2OUT_OVP_ENABLE 0
#define NU2118A_PMID2OUT_OVP_DISABLE 1

/* Register 06h */
#define NU2118A_REG_06 0x06

/* Register 07h */
#define NU2118A_REG_07 0x07
#define NU2118A_CHG_EN_MASK 0x80
#define NU2118A_CHG_EN_SHIFT 7
#define NU2118A_CHG_ENABLE 1
#define NU2118A_CHG_DISABLE 0

#define NU2118A_REG_RESET_MASK 0x40
#define NU2118A_REG_RESET_SHIFT 6
#define NU2118A_NO_REG_RESET 0
#define NU2118A_RESET_REG 1

/* Register 08h */
#define NU2118A_REG_08 0x08
#define NU2118A_WATCHDOG_MASK                0x07
#define NU2118A_WATCHDOG_SHIFT               0
#define NU2118A_WATCHDOG_DIS                 0
#define NU2118A_WATCHDOG_200MS               1
#define NU2118A_WATCHDOG_500MS               2
#define NU2118A_WATCHDOG_1S                  3
#define NU2118A_WATCHDOG_5S                  4
#define NU2118A_WATCHDOG_30S                 5

/* Register 09h */
#define NU2118A_REG_09 0x09

/* Register 0Ah */
#define NU2118A_REG_0A 0x0A
#define NU2118A_CFLY_PRECHG_TIMEOUT_MASK     (BIT(4) | BIT(5))
#define NU2118A_CFLY_PRECHG_TIMEOUT_SHIFT    4
#define NU2118A_CFLY_PRECHG_20_MS            1

/* Register 0Bh */
#define NU2118A_REG_0B 0x0B

/* Register 0Ch */
#define NU2118A_REG_0C 0x0C
#define NU2118A_IBUS_UCP_DIS_MASK 0x80
#define NU2118A_IBUS_UCP_DIS_SHIFT 7
#define NU2118A_IBUS_UCP_ENABLE 0
#define NU2118A_IBUS_UCP_DISABLE 1

/* Register 0Dh */
#define NU2118A_REG_0D 0x0D

/* Register 0Eh */
#define NU2118A_REG_0E 0x0E

/* Register 0Fh */
#define NU2118A_REG_0F 0x0F

/* Register 10h */
#define NU2118A_REG_10 0x10
#define NU2118A_CP_SWITCHING_STAT_MASK 0x10
#define NU2118A_CP_SWITCHING_STAT_SHIFT 4

/* Register 11h */
#define NU2118A_REG_11 0x11
#define NU2118A_VBAT_OVP_FLAG_MASK 0x40
#define NU2118A_VBAT_OVP_FLAG_SHIFT 6
#define NU2118A_IBAT_OCP_FLAG_MASK 0x20
#define NU2118A_IBAT_OCP_FLAG_SHIFT 5
#define NU2118A_VBUS_OVP_FLAG_MASK 0x10
#define NU2118A_VBUS_OVP_FLAG_SHIFT 4
#define NU2118A_IBUS_OCP_FLAG_MASK 0x08
#define NU2118A_IBUS_OCP_FLAG_SHIFT 3

/* Register 12h */
#define NU2118A_REG_12 0x12
#define NU2118A_TSD_FLAG_MASK 0x80
#define NU2118A_TSD_FLAG_SHIFT 7
#define NU2118A_PMID2VOUT_OVP_FLAG_MASK 0x20
#define NU2118A_PMID2VOUT_OVP_FLAG_SHIFT 5
#define NU2118A_PMID2VOUT_UVP_FLAG_MASK 0x10
#define NU2118A_PMID2VOUT_UVP_FLAG_SHIFT 4

/* Register 13h */
#define NU2118A_REG_13 0x13

/* Register 14h */
#define NU2118A_REG_14 0x14
#define NU2118A_SS_TIMEOUT_FLAG_MASK 0x04
#define NU2118A_SS_TIMEOUT_FLAG_SHIFT 2

#define NU2118A_PIN_DIAG_FALL_FLAG_MASK 0x01
#define NU2118A_PIN_DIAG_FALL_FLAG_SHIFT 0

#define NU2118A_POWER_NG_FLAG_MASK 0x10

/* Register 15h */
#define NU2118A_REG_15 0x15

/* Register 16h */
#define NU2118A_REG_16 0x16

/* Register 17h */
#define NU2118A_REG_17 0x17

/* Register 18h */
#define NU2118A_REG_18 0x18
#define NU2118A_ADC_ENABLE_SHIFT 7

/* Register 19h */
#define NU2118A_REG_19 0x19

/* Register 1Ah */
#define NU2118A_REG_1A 0x1A
#define NU2118A_IBUS_POL_H_MASK 0x1F
#define NU2118A_IBUS_ADC_LSB 1

/* Register 1Bh */
#define NU2118A_REG_1B 0x1B

/* Register 1Ch */
#define NU2118A_REG_1C 0x1C
#define NU2118A_VBUS_POL_H_MASK 0x7F
#define NU2118A_VBUS_ADC_LSB 1

/* Register 1Dh */
#define NU2118A_REG_1D 0x1D

/* Register 1Eh */
#define NU2118A_REG_1E 0x1E
#define NU2118A_VAC_POL_H_MASK 0x7F
#define NU2118A_VAC_ADC_LSB 1

/* Register 1Fh */
#define NU2118A_REG_1F 0x1F
#define NU2118A_VAC_POL_L_MASK 0xFF

/* Register 20h */
#define NU2118A_REG_20 0x20
#define NU2118A_VOUT_POL_H_MASK 0x1F
#define NU2118A_VOUT_ADC_LSB 1

/* Register 21h */
#define NU2118A_REG_21 0x21

/* Register 22h */
#define NU2118A_REG_22 0x22
#define NU2118A_VBAT_POL_H_MASK 0x1F
#define NU2118A_VBAT_ADC_LSB 1

/* Register 23h */
#define NU2118A_REG_23 0x23
#define NU2118A_VBAT_POL_L_MASK 0xFF

/* Register 24h */
#define NU2118A_REG_24 0x24

/* Register 2Bh */
#define NU2118A_REG_2B 0x2B
#define NU2118A_VOOC_EN_MASK 0x80
#define NU2118A_VOOC_EN_SHIFT 7
#define NU2118A_VOOC_ENABLE 1
#define NU2118A_VOOC_DISABLE 0

#define NU2118A_SOFT_RESET_MASK 0x02
#define NU2118A_SOFT_RESET_SHIFT 1
#define NU2118A_SOFT_RESET 1

#define NU2118A_SEND_SEQ_MASK 0x01
#define NU2118A_SEND_SEQ_SHIFT 0
#define NU2118A_SEND_SEQ 1

/* Register 2Ch */
#define NU2118A_REG_2C 0x2C
#define NU2118A_TX_WDATA_POL_H_MASK 0x03

/* Register 2Dh */
#define NU2118A_REG_2D 0x2D
#define NU2118A_TX_WDATA_POL_L_MASK 0xFF

/* Register 2Eh */
#define NU2118A_REG_2E 0x2E
#define NU2118A_RX_RDATA_POL_H_MASK 0xFF

/* Register 2Fh */
#define NU2118A_REG_2F 0x2F

/* Register 30h */
#define NU2118A_REG_30 0x30

/* Register 31h */
#define NU2118A_REG_31 0x31
#define NU2118A_PRE_WDATA_POL_H_MASK 0x03

/* Register 32h */
#define NU2118A_REG_32 0x32
#define NU2118A_PRE_WDATA_POL_L_MASK 0xFF

/* Register 33h */
#define NU2118A_REG_33 0x33

/* Register 35h */
#define NU2118A_REG_35 0x35

/* Register 36h */
#define NU2118A_REG_36 0x36

/* Register 3Ah */
#define NU2118A_REG_3A 0x3A

/* Register 3Ch */
#define NU2118A_REG_3C 0x3C

/************************UFCS REG***********************************/
#define UFCS_BTB_TEMP_MAX 80
#define UFCS_USB_TEMP_MAX 80
#define UFCS_BTB_USB_OVER_CNTS 9

#define UFCS_UCT_POWER_VOLT 5000
#define UFCS_UCT_POWER_CURR 1000

#define UFCS_HARDRESET_RETRY_CNTS 3
#define UFCS_HANDSHAKE_RETRY_CNTS 1

/********************** I2C Slave Addr **************************/
#define NU2118A_I2C_ADDR (0x6E)

#define NU2118A_MAX_REG (0x0264)
#define NU2118A_FLAG_NUM (3)
#define NU2118A_TRACK_NUM (5)


/************************Timer***********************************/
#define UFCS_HANDSHAKE_TIMEOUT (300)
#define UFCS_PING_TIMEOUT (300)
#define UFCS_ACK_TIMEOUT (50)
#define UFCS_MSG_TIMEOUT (300)
#define UFCS_POWER_RDY_TIMEOUT (550)


/********************** SC2201 I2C reg map **********************/
#define NU2118A_ENABLE_REG_NUM (5)

#define NU2118A_ADDR_UFCS_CTRL0 (0x39)

#define SEND_SOURCE_HARDRESET BIT(0)
#define SEND_CABLE_HARDRESET BIT(1)
#define FLAG_BAUD_RATE_VALUE (BIT(4) | BIT(3))
#define FLAG_BAUD_NUM_SHIFT (3)
#define NU2118A_CMD_EN_CHIP (0x88)
#define NU2118A_CMD_DIS_CHIP (0X00)
#define NU2118A_MASK_EN_HANDSHAKE BIT(5)
#define NU2118A_CMD_EN_HANDSHAKE BIT(5)

#define NU2118A_ADDR_GENERAL_INT_FLAG1 (0x3A)
#define NU2118A_CMD_CLR_TX_RX (0x30)


#define NU2118A_ADDR_GENERAL_INT_FLAG2 (0x3B)
#define NU2118A_FLAG_ACK_RECEIVE_TIMEOUT BIT(0)
#define NU2118A_FLAG_MSG_TRANS_FAIL BIT(1)
#define NU2118A_FLAG_RX_OVERFLOW BIT(3)
#define NU2118A_FLAG_DATA_READY BIT(4)
#define NU2118A_FLAG_SENT_PACKET_COMPLETE BIT(5)
#define NU2118A_FLAG_HANDSHAKE_SUCCESS BIT(6)
#define NU2118A_FLAG_HANDSHAKE_FAIL BIT(7)

#define NU2118A_ADDR_GENERAL_INT_FLAG3 (0x3C)
#define NU2118A_FLAG_HARD_RESET BIT(0)
#define NU2118A_FLAG_CRC_ERROR BIT(1)
#define NU2118A_FLAG_BAUD_RATE_CHANGE BIT(2)
#define NU2118A_FLAG_START_FAIL BIT(3)
#define NU2118A_FLAG_LENGTH_ERROR BIT(4)
#define NU2118A_FLAG_DATA_BYTE_TIMEOUT BIT(5)
#define NU2118A_FLAG_TRAINING_BYTE_ERROR BIT(6)
#define NU2118A_FLAG_BAUD_RATE_ERROR BIT(7)

#define NU2118A_ADDR_GENERAL_INT_FLAG4 (0x3D)
#define NU2118A_FLAG_BUS_CONFLICT BIT(6)


#define NU2118A_ADDR_UFCS_INT_MASK0 (0x3E)
#define NU2118A_CMD_MASK_ACK_TIMEOUT (0x01)

#define NU2118A_ADDR_UFCS_INT_MASK1 (0x3F)
#define NU2118A_MASK_TRANING_BYTE_ERROR (0x40)

#define NU2118A_ADDR_UFCS_OPTION (0x40)
#define NU2118A_BUFFER_OPTION_CONFIG (0x4)
#define NU2118A_BAUDRATE_CHECK_CONFIG BIT(1)
#define NU2118A_END_CHECK_ENABLE BIT(0)

/*tx_buffer*/
#define NU2118A_ADDR_TX_LENGTH (0x65)

#define NU2118A_ADDR_TX_BUFFER0 (0x66)

#define NU2118A_ADDR_TX_BUFFER52 (0x9A)

/*rx_buffer*/
#define NU2118A_ADDR_RX_LENGTH (0x9B)
#define NU2118A_ADDR_RX_BUFFER0 (0x9C)
#define NU2118A_ADDR_RX_BUFFER63 (0xDB)
#define NU2118A_LEN_MAX 64

/*********************command buffer**************/
/*avoid to rewrite the baudrate flags*/
#define NU2118A_CMD_SND_CMP BIT(2)
#define NU2118A_MASK_SND_CMP BIT(2)

#define CP_NU2118A_PIN_DIAG_FALL_FLAG_MASK		BIT(0)
#define CP_NU2118A_POWER_NG_MASK				BIT(4)
#define CP_NU2118A_TSD_FLAG_MASK				BIT(7)
#define CP_NU2118A_VBAT_OVP_FLAG_MASK			BIT(6)
#define CP_NU2118A_VBUS_OVP_FLAG_MASK			BIT(4)
#define CP_NU2118A_IBUS_OCP_FLAG_MASK			BIT(3)
#define CP_NU2118A_VOUT_OVP_FLAG_MASK			BIT(7)
#define CP_NU2118A_PMID2OUT_OVP_FLAG_MASK		BIT(5)
#define CP_NU2118A_PMID2OUT_UVP_FLAG_MASK		BIT(4)
#define CP_NU2118A_SS_TIMEOUT_FLAG_MASK		BIT(2)
#define CP_NU2118A_WD_TIMEOUT_FLAG_MASK		BIT(3)
#define CP_NU2118A_IBAT_OCP_FLAG_MASK			BIT(5)
#define CP_NU2118A_VOUT_INVALID_REVERSE_FLAG_MASK	BIT(6)

#endif /*_OPLUS_NU2118A_H_*/

