// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2025 Oplus. All rights reserved.
 */

#ifndef __SGM41515_HEADER__
#define __SGM41515_HEADER__

/* Address:00h */
#define REG00_SGM41515_ADDRESS                         0x00
#define REG00_SGM41515_HIZ_MODE_MASK                   BIT(7)
#define REG00_SGM41515_HIZ_MODE_DISABLE                0x00
#define REG00_SGM41515_HIZ_MODE_ENABLE                 BIT(7)

#define REG00_SGM41515_STAT_DIS_MASK                   (BIT(6) | BIT(5))
#define REG00_SGM41515_STAT_DIS_ENABLE                 0x00
#define REG00_SGM41515_STAT_DIS_DISABLE                (BIT(6) | BIT(5))

#define REG00_SGM41515_INPUT_CURRENT_LIMIT_MASK        (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_SHIFT       0
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_OFFSET      100
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_STEP        100
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_MAX         3800
#define REG00_SGM41515_INIT_INPUT_CURRENT_LIMIT_500MA  500
#define REG00_SGM41515_INIT_INPUT_CURRENT_LIMIT_2000MA 2000
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_100MA       0x00
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_300MA       (BIT(1) | BIT(0))
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_500MA       (BIT(2) | BIT(0))
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_900MA       (BIT(3) | BIT(1))
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_1200MA      (BIT(3) | BIT(2))
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_1500MA      (BIT(3) | BIT(2) |BIT(1) | BIT(0))
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_1700MA      (BIT(4) | BIT(0))
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_2000MA      (BIT(4) | BIT(2))
#define REG00_SGM41515_INPUT_CURRENT_LIMIT_3000MA      (BIT(4) | BIT(3) |BIT(2) |BIT(1))

/* Address:01h */
#define REG01_SGM41515_ADDRESS                         0x01

#define REG01_SGM41515_PFM_DIS_MASK                    BIT(7)
#define REG01_SGM41515_PFM_DIS_ENABLE                  0x00
#define REG01_SGM41515_PFM_DIS_DISABLE                 BIT(7)

#define REG01_SGM41515_WDT_TIMER_RESET_MASK            BIT(6)
#define REG01_SGM41515_WDT_TIMER_NORMAL                0x00
#define REG01_SGM41515_WDT_TIMER_RESET                 BIT(6)

#define REG01_SGM41515_OTG_MASK                        BIT(5)
#define REG01_SGM41515_OTG_DISABLE                     0x00
#define REG01_SGM41515_OTG_ENABLE                      BIT(5)

#define REG01_SGM41515_CHARGING_MASK                   BIT(4)
#define REG01_SGM41515_CHARGING_DISABLE                0x00
#define REG01_SGM41515_CHARGING_ENABLE                 BIT(4)

#define REG01_SGM41515_SYS_VOL_LIMIT_MASK              (BIT(3) | BIT(2) | BIT(1))
#define REG01_SGM41515_SYS_VOL_LIMIT_2600MV            0x00
#define REG01_SGM41515_SYS_VOL_LIMIT_2800MV            BIT(1)
#define REG01_SGM41515_SYS_VOL_LIMIT_3000MV            BIT(2)
#define REG01_SGM41515_SYS_VOL_LIMIT_3200MV            (BIT(2) | BIT(1))
#define REG01_SGM41515_SYS_VOL_LIMIT_3400MV            BIT(3)
#define REG01_SGM41515_SYS_VOL_LIMIT_3500MV            (BIT(3) | BIT(1))
#define REG01_SGM41515_SYS_VOL_LIMIT_3600MV            (BIT(3) | BIT(2))
#define REG01_SGM41515_SYS_VOL_LIMIT_3700MV            (BIT(3) | BIT(2) | BIT(1))

#define REG01_SGM41515_VBAT_FALLING_MASK               BIT(0)
#define REG01_SGM41515_VBAT_FALLING_3000MV             0x00
#define REG01_SGM41515_VBAT_FALLING_2500MV             BIT(0)

/* Address:02h */
#define REG02_SGM41515_ADDRESS                         0x02

#define REG02_SGM41515_OTG_CURRENT_LIMIT_MASK          BIT(7)
#define REG02_SGM41515_OTG_CURRENT_LIMIT_500MA         0
#define REG02_SGM41515_OTG_CURRENT_LIMIT_1200MA        BIT(7)
#define REG02_SGM41515_BOOSTI_1200MA                   1200

#define REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_MASK     (BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_SHIFT    0
#define REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_OFFSET   0
#define REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_STEP     60
#define REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_2000MA   2000
#define REG02_SGM41515_FAST_CHG_CURRENT_LIMIT_500MA    500

/* Address:03h */
#define REG03_SGM41515_ADDRESS                         0x03

#define REG03_SGM41515_PRE_CHG_CURRENT_LIMIT_MASK      (BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define REG03_SGM41515_PRE_CHG_CURRENT_LIMIT_SHIFT     4
#define REG03_SGM41515_PRE_CHG_CURRENT_LIMIT_OFFSET    60
#define REG03_SGM41515_PRE_CHG_CURRENT_LIMIT_STEP      60

#define REG03_SGM41515_TERM_CHG_CURRENT_LIMIT_MASK     (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG03_SGM41515_TERM_CHG_CURRENT_LIMIT_SHIFT    0
#define REG03_SGM41515_TERM_CHG_CURRENT_LIMIT_OFFSET   60
#define REG03_SGM41515_TERM_CHG_CURRENT_LIMIT_STEP     60

/* Address:04h */
#define REG04_SGM41515_ADDRESS                         0x04

#define REG04_SGM41515_CHG_VOL_LIMIT_MASK              (BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3))
#define REG04_SGM41515_CHG_VOL_LIMIT_SHIFT             3
#define REG04_SGM41515_CHG_VOL_LIMIT_OFFSET            3856
#define REG04_BQ25601D_CHG_VOL_LIMIT_OFFSET            3847
#define REG04_SGM41515_CHG_VOL_LIMIT_STEP              32

#define REG04_SGM41515_RECHG_THRESHOLD_VOL_MASK        BIT(0)
#define REG04_SGM41515_RECHG_THRESHOLD_VOL_100MV       0x00
#define REG04_SGM41515_RECHG_THRESHOLD_VOL_200MV       BIT(0)
#define REG04_SGM41515_RECHG_VOLTAGE_200MV             200

/* Address:05h */
#define REG05_SGM41515_ADDRESS                         0x05

#define REG05_SGM41515_TERMINATION_MASK                BIT(7)
#define REG05_SGM41515_TERMINATION_DISABLE             0x00
#define REG05_SGM41515_TERMINATION_ENABLE              BIT(7)

#define REG05_SGM41515_WATCHDOG_TIMER_MASK             (BIT(5) | BIT(4))
#define REG05_SGM41515_WATCHDOG_TIMER_DISABLE          0x00
#define REG05_SGM41515_WATCHDOG_TIMER_40S              BIT(4)
#define REG05_SGM41515_WATCHDOG_TIMER_80S              BIT(5)
#define REG05_SGM41515_WATCHDOG_TIMER_160S             (BIT(5) | BIT(4))

#define REG05_SGM41515_CHG_SAFETY_TIMER_MASK           BIT(3)
#define REG05_SGM41515_CHG_SAFETY_TIMER_DISABLE        0x00
#define REG05_SGM41515_CHG_SAFETY_TIMER_ENABLE         BIT(3)

#define REG05_SGM41515_FAST_CHG_TIMEOUT_MASK           BIT(2)
#define REG05_SGM41515_FAST_CHG_TIMEOUT_7H             0x00
#define REG05_SGM41515_FAST_CHG_TIMEOUT_16H            BIT(2)

/* Address:06h */
#define REG06_SGM41515_ADDRESS                         0x06
#define REG06_SGM41515_OVP_MASK                        (BIT(7) | BIT(6))
#define REG06_SGM41515_OVP_5P5V                        0
#define REG06_SGM41515_OVP_6P5V                        BIT(6)
#define REG06_SGM41515_OVP_10P5V                       BIT(7)
#define REG06_SGM41515_OVP_14P0V                       (BIT(7) | BIT(6))

#define REG06_SGM41515_OTG_VLIM_MASK                   (BIT(5) | BIT(4))
#define REG06_SGM41515_OTG_VLIM_SHIFT                  4
#define REG06_SGM41515_OTG_VLIM_OFFSET                 4850
#define REG06_SGM41515_OTG_VLIM_MAXMV                  5300
#define REG06_SGM41515_OTG_VLIM_STEP                   150
#define REG06_SGM41515_OTG_VLIM_5150MV                 BIT(5)
#define REG06_SGM41515_OTG_VLIM_5000MV                 5000

#define REG06_SGM41515_VINDPM_MASK                     (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG06_SGM41515_VINDPM_STEP_MV                  100
#define REG06_SGM41515_VINDPM_OFFSET                   3900
#define REG06_SGM41515_VINDPM_SHIFT                    0

#define SGM41515_VINDPM_THRESHOLD_3900MV               3900
#define SGM41515_VINDPM_THRESHOLD_5900MV               5900
#define SGM41515_VINDPM_THRESHOLD_7500MV               7500
#define SGM41515_VINDPM_THRESHOLD_10500MV              10500
#define SGM41515_VINDPM_THRESHOLD_MAX                  12000

/* Address:07h */
#define REG07_SGM41515_ADDRESS                         0x07

#define REG07_SGM41515_IINDET_EN_MASK                  BIT(7)
#define REG07_SGM41515_IINDET_EN_DET_COMPLETE          0x00
#define REG07_SGM41515_IINDET_EN_FORCE_DET             BIT(7)
#define REG07_SGM41515_BATFET_DIS_MASK                 BIT(5)
#define REG07_SGM41515_BATFET_DIS_ON                   BIT(5)
#define REG07_SGM41515_BATFET_DIS_OFF                  0x00
#define REG07_SGM41515_BATFET_DLY_MASK                 BIT(3)
#define REG07_SGM41515_BATFET_DLY_IMMEDIATELY          0x00
#define REG07_SGM41515_BATFET_DLY_DEFAULT              BIT(3)

#define SGM41515_BATFET_RST_ENABLE                     1
#define SGM41515_BATFET_RST_DISABLE                    0

#define REG07_SGM41515_BATFET_RST_EN_MASK              BIT(2)
#define REG07_SGM41515_BATFET_RST_EN_SHIFT             2

/* Address:08h */
#define REG08_SGM41515_ADDRESS                         0x08

#define REG08_SGM41515_VBUS_STAT_MASK                  (BIT(7) | BIT(6) | BIT(5))
#define REG08_SGM41515_VBUS_STAT_UNKNOWN               0x00
#define REG08_SGM41515_VBUS_STAT_SDP                   BIT(5)
#define REG08_SGM41515_VBUS_STAT_CDP                   BIT(6)
#define REG08_SGM41515_VBUS_STAT_DCP                   (BIT(6) | BIT(5))
#define REG08_SGM41515_VBUS_STAT_OCP                   (BIT(7) | BIT(5))
#define REG08_SGM41515_VBUS_STAT_FLOAT                 (BIT(7) | BIT(6))
#define REG08_SGM41515_VBUS_STAT_OTG_MODE              (BIT(7) | BIT(6) | BIT(5))

#define REG08_SGM41515_CHG_STAT_MASK                   (BIT(4) | BIT(3))
#define REG08_SGM41515_CHG_STAT_NO_CHARGING            0x00
#define REG08_SGM41515_CHG_STAT_PRE_CHARGING           BIT(3)
#define REG08_SGM41515_CHG_STAT_FAST_CHARGING          BIT(4)
#define REG08_SGM41515_CHG_STAT_CHG_TERMINATION        (BIT(4) | BIT(3))

#define REG08_SGM41515_POWER_GOOD_STAT_MASK            BIT(2)
#define REG08_SGM41515_POWER_GOOD_STAT_NOT_GOOD        0x00
#define REG08_SGM41515_POWER_GOOD_STAT_GOOD            BIT(2)

/* Address:09h */
#define REG09_SGM41515_ADDRESS                         0x09

#define REG09_SGM41515_WDT_FAULT_MASK                  BIT(7)
#define REG09_SGM41515_WDT_FAULT_NORMAL                0x00
#define REG09_SGM41515_WDT_FAULT_EXPIRATION            BIT(7)

#define REG09_SGM41515_OTG_FAULT_MASK                  BIT(6)
#define REG09_SGM41515_OTG_FAULT_NORMAL                0x00
#define REG09_SGM41515_OTG_FAULT_ABNORMAL              BIT(6)

#define REG09_SGM41515_CHG_FAULT_MASK                  (BIT(5) | BIT(4))
#define REG09_SGM41515_CHG_FAULT_NORMAL                0x00
#define REG09_SGM41515_CHG_FAULT_INPUT_ERROR           BIT(4)
#define REG09_SGM41515_CHG_FAULT_THERM_SHUTDOWN        BIT(5)
#define REG09_SGM41515_CHG_FAULT_TIMEOUT_ERROR         (BIT(5) | BIT(4))

#define REG09_SGM41515_BAT_FAULT_MASK                  BIT(3)
#define REG09_SGM41515_BAT_FAULT_NORMAL                0x00
#define REG09_SGM41515_BAT_FAULT_BATOVP                BIT(3)

#define REG09_SGM41515_NTC_FAULT_MASK                  (BIT(2) | BIT(1) | BIT(0))
#define REG09_SGM41515_NTC_FAULT_NORMAL                0x00
#define REG09_SGM41515_NTC_FAULT_WARM                  BIT(1)
#define REG09_SGM41515_NTC_FAULT_COOL                  (BIT(1) | BIT(0))
#define REG09_SGM41515_NTC_FAULT_COLD                  (BIT(2) | BIT(0))
#define REG09_SGM41515_NTC_FAULT_HOT                   (BIT(2) | BIT(1))

/* Address:0Ah */
#define REG0A_SGM41515_ADDRESS                         0x0A

#define REG0A_SGM41515_BUS_GD_MASK                     BIT(7)
#define REG0A_SGM41515_BUS_GD_NO                       0x00
#define REG0A_SGM41515_BUS_GD_YES                      BIT(7)

#define REG0A_SGM41515_VINDPM_MASK                     BIT(6)
#define REG0A_SGM41515_NOT_VINDPM                      0x00
#define REG0A_SGM41515_IN_VINDPM                       BIT(6)

#define REG0A_SGM41515_VINDPM_INT_MASK                 BIT(1)
#define REG0A_SGM41515_VINDPM_INT_ALLOW                0x00
#define REG0A_SGM41515_VINDPM_INT_NOT_ALLOW            BIT(1)

#define REG0A_SGM41515_IINDPM_INT_MASK                 BIT(0)
#define REG0A_SGM41515_IINDPM_INT_ALLOW                0x00
#define REG0A_SGM41515_IINDPM_INT_NOT_ALLOW            BIT(0)

/* Address:0Bh */
#define REG0B_SGM41515_ADDRESS                         0x0B

#define REG0B_SGM41515_REG_RST_MASK                    BIT(7)
#define REG0B_SGM41515_REG_RST_KEEP                    0x00
#define REG0B_SGM41515_REG_RST_RESET                   BIT(7)

#define REG0B_SGM41515_PN_MASK                         (BIT(6) | BIT(5) | BIT(4) | BIT(3))
#define REG0B_SGM41515_PN                              BIT(4)
#define SGM41515_DEVID_SHIFT                           3
#define REG0B_SY6974B_PN                               BIT(6)
/*
  part id:
    0000=SGM41515
    0001=SGM41515A or SGM41515D
    1100=SGM41541
    1101=SGM41542
    1110=SGM41512SA or SGM41512SD
*/
#define SGM41515_PART_ID                               0x00
#define SGM41515D_PART_ID                              BIT(3) >> SGM41515_DEVID_SHIFT
#define SGM41541_PART_ID                               (BIT(6) | BIT(5)) >> SGM41515_DEVID_SHIFT
#define SGM41542_PART_ID                               (BIT(6) | BIT(5) | BIT(3)) >> SGM41515_DEVID_SHIFT
#define SGM41512SD_PART_ID                             (BIT(6) | BIT(5) | BIT(4)) >> SGM41515_DEVID_SHIFT

/* Address:0Dh */
#define REG0D_SGM41515_ADDRESS                         0x0d

#define REG0D_SGM41515_DP_VSEL_MASK                    (BIT(4) | BIT(3))
#define REG0D_SGM41515_DP_VSEL_SHIFT                   3
#define REG0D_SGM41515_DP_600MV                        0x2
#define REG0D_SGM41515_DM_VSEL_MASK                    (BIT(2) | BIT(1))
#define REG0D_SGM41515_DM_VSEL_SHIFT                   1
#define REG0D_SGM41515_DPDM_VSEL_MASK                  (BIT(4) | BIT(3) | BIT(2) | BIT(1))
#define REG0D_SGM41515_DPDM_VSEL_SHIFT                 1
#define REG0D_SGM41515_DP_600MV_DM_HIZ                 0x8
#define REG0D_SGM41515_DP_3300MV_DM_600MV              0xe

/* Address:0Eh */
#define REG0E_SGM41515_ADDRESS                         0x0E
#define REG0E_SGM41515_REG_INPUT_DET_MASK              BIT(7)

/* Address:0Fh */
#define REG0F_SGM41515_ADDRESS                         0x0F
#define REG0F_SGM41515_VINDPM_THRESHOLD_OFFSET_MASK    GENMASK(1, 0)
#define REG0F_SGM41515_VINDPM_THRESHOLD_OFFSET_SHIFT   (BIT(1) | BIT(0)))
#define REG0F_SGM41515_VINDPM_THRESHOLD_OFFSET_3900MV  0
#define REG0F_SGM41515_VINDPM_THRESHOLD_OFFSET_5900MV  BIT(0)
#define REG0F_SGM41515_VINDPM_THRESHOLD_OFFSET_7500MV  BIT(1)
#define REG0F_SGM41515_VINDPM_THRESHOLD_OFFSET_10500MV (BIT(1) | BIT(0)))
#define REG0F_SGM41515_VREG_FT_MASK                 (BIT(7) | BIT(6))
#define REG0F_SGM41515_VREG_FT_SHIFT                6
#define REG0F_SGM41515_VREG_FT_DISABLE              0x00  /* 00 = Disable */
#define REG0F_SGM41515_VREG_FT_PLUS_8MV             0x01  /* 01 = VREG + 8mV */
#define REG0F_SGM41515_VREG_FT_MINUS_8MV            0x02  /* 10 = VREG - 8mV */
#define REG0F_SGM41515_VREG_FT_MINUS_16MV           0x03  /* 11 = VREG - 16mV */

/* Address:10h */
#define REG10_SGM41515_ADDRESS                         0x10

/* Smart tuning thresholds */
#define SMART_TUNING_MAX_ADJUSTMENT        12

/* Other */
#define SGM41515_FIRST_REG                             0x00
#define SGM41515_DUMP_MAX_REG                          0x0F
#define SGM41515_LAST_REG                              0x0F
#define SGM41515_REG_NUMBER                            0x15
#define SGM41515_DEFAULT_PRECHG_CURRENT                480
#define SGM41515_DEFAULT_TERMINATION_VOLTAGE           4500

#define USB_HW_AICL_POINT       4600
#define USB_SW_AICL_POINT       4620
#define HW_AICL_POINT_DEFAULT   4500
#define SW_AICL_POINT_DEFAULT   4520
#define SGM41515_FAKE_VBUS_5V   5000
#define SGM41515_FAKE_VBUS_9V   9000

#define INIT_WORK_OTHER_DELAY      200
#define REPORT_BC12_COMPLETE_DELAY 30

enum SGM41515_VINDPM_OS {
	VINDPM_OS_3900mV,
	VINDPM_OS_5900mV,
	VINDPM_OS_7500mV,
	VINDPM_OS_10500mV,
};
#endif /*__SGM41515_HEADER__*/
