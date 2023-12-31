/*
 * Copyright (c) 2015-2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef REG_STRUCT_H
#define REG_STRUCT_H

#define MISSING_REGISTER 0
#define UNSUPPORTED_REGISTER_OFFSET 0xffffffff

/**
 * is_register_supported() - return true if the register offset is valid
 * @reg: register address being checked
 *
 * Return: true if the register offset is valid
 */
static inline bool is_register_supported(uint32_t reg)
{
	return (reg != MISSING_REGISTER) &&
		(reg != UNSUPPORTED_REGISTER_OFFSET);
}

struct targetdef_s {
	uint32_t d_RTC_SOC_BASE_ADDRESS;
	uint32_t d_RTC_WMAC_BASE_ADDRESS;
	uint32_t d_SYSTEM_SLEEP_OFFSET;
	uint32_t d_WLAN_SYSTEM_SLEEP_OFFSET;
	uint32_t d_WLAN_SYSTEM_SLEEP_DISABLE_LSB;
	uint32_t d_WLAN_SYSTEM_SLEEP_DISABLE_MASK;
	uint32_t d_CLOCK_CONTROL_OFFSET;
	uint32_t d_CLOCK_CONTROL_SI0_CLK_MASK;
	uint32_t d_RESET_CONTROL_OFFSET;
	uint32_t d_RESET_CONTROL_MBOX_RST_MASK;
	uint32_t d_RESET_CONTROL_SI0_RST_MASK;
	uint32_t d_WLAN_RESET_CONTROL_OFFSET;
	uint32_t d_WLAN_RESET_CONTROL_COLD_RST_MASK;
	uint32_t d_WLAN_RESET_CONTROL_WARM_RST_MASK;
	uint32_t d_GPIO_BASE_ADDRESS;
	uint32_t d_GPIO_PIN0_OFFSET;
	uint32_t d_GPIO_PIN1_OFFSET;
	uint32_t d_GPIO_PIN0_CONFIG_MASK;
	uint32_t d_GPIO_PIN1_CONFIG_MASK;
	uint32_t d_SI_CONFIG_BIDIR_OD_DATA_LSB;
	uint32_t d_SI_CONFIG_BIDIR_OD_DATA_MASK;
	uint32_t d_SI_CONFIG_I2C_LSB;
	uint32_t d_SI_CONFIG_I2C_MASK;
	uint32_t d_SI_CONFIG_POS_SAMPLE_LSB;
	uint32_t d_SI_CONFIG_POS_SAMPLE_MASK;
	uint32_t d_SI_CONFIG_INACTIVE_CLK_LSB;
	uint32_t d_SI_CONFIG_INACTIVE_CLK_MASK;
	uint32_t d_SI_CONFIG_INACTIVE_DATA_LSB;
	uint32_t d_SI_CONFIG_INACTIVE_DATA_MASK;
	uint32_t d_SI_CONFIG_DIVIDER_LSB;
	uint32_t d_SI_CONFIG_DIVIDER_MASK;
	uint32_t d_SI_BASE_ADDRESS;
	uint32_t d_SI_CONFIG_OFFSET;
	uint32_t d_SI_TX_DATA0_OFFSET;
	uint32_t d_SI_TX_DATA1_OFFSET;
	uint32_t d_SI_RX_DATA0_OFFSET;
	uint32_t d_SI_RX_DATA1_OFFSET;
	uint32_t d_SI_CS_OFFSET;
	uint32_t d_SI_CS_DONE_ERR_MASK;
	uint32_t d_SI_CS_DONE_INT_MASK;
	uint32_t d_SI_CS_START_LSB;
	uint32_t d_SI_CS_START_MASK;
	uint32_t d_SI_CS_RX_CNT_LSB;
	uint32_t d_SI_CS_RX_CNT_MASK;
	uint32_t d_SI_CS_TX_CNT_LSB;
	uint32_t d_SI_CS_TX_CNT_MASK;
	uint32_t d_BOARD_DATA_SZ;
	uint32_t d_BOARD_EXT_DATA_SZ;
	uint32_t d_MBOX_BASE_ADDRESS;
	uint32_t d_LOCAL_SCRATCH_OFFSET;
	uint32_t d_CPU_CLOCK_OFFSET;
	uint32_t d_LPO_CAL_OFFSET;
	uint32_t d_GPIO_PIN10_OFFSET;
	uint32_t d_GPIO_PIN11_OFFSET;
	uint32_t d_GPIO_PIN12_OFFSET;
	uint32_t d_GPIO_PIN13_OFFSET;
	uint32_t d_CLOCK_GPIO_OFFSET;
	uint32_t d_CPU_CLOCK_STANDARD_LSB;
	uint32_t d_CPU_CLOCK_STANDARD_MASK;
	uint32_t d_LPO_CAL_ENABLE_LSB;
	uint32_t d_LPO_CAL_ENABLE_MASK;
	uint32_t d_CLOCK_GPIO_BT_CLK_OUT_EN_LSB;
	uint32_t d_CLOCK_GPIO_BT_CLK_OUT_EN_MASK;
	uint32_t d_ANALOG_INTF_BASE_ADDRESS;
	uint32_t d_WLAN_MAC_BASE_ADDRESS;
	uint32_t d_FW_INDICATOR_ADDRESS;
	uint32_t d_FW_CPU_PLL_CONFIG;
	uint32_t d_DRAM_BASE_ADDRESS;
	uint32_t d_SOC_CORE_BASE_ADDRESS;
	uint32_t d_CORE_CTRL_ADDRESS;
	uint32_t d_CE_COUNT;
	uint32_t d_MSI_NUM_REQUEST;
	uint32_t d_MSI_ASSIGN_FW;
	uint32_t d_MSI_ASSIGN_CE_INITIAL;
	uint32_t d_PCIE_INTR_ENABLE_ADDRESS;
	uint32_t d_PCIE_INTR_CLR_ADDRESS;
	uint32_t d_PCIE_INTR_FIRMWARE_MASK;
	uint32_t d_PCIE_INTR_CE_MASK_ALL;
	uint32_t d_CORE_CTRL_CPU_INTR_MASK;
	uint32_t d_WIFICMN_PCIE_BAR_REG_ADDRESS;
	/* htt_rx.c */
	/* htt tx */
	uint32_t d_MSDU_LINK_EXT_3_TCP_OVER_IPV4_CHECKSUM_EN_MASK;
	uint32_t d_MSDU_LINK_EXT_3_TCP_OVER_IPV6_CHECKSUM_EN_MASK;
	uint32_t d_MSDU_LINK_EXT_3_UDP_OVER_IPV4_CHECKSUM_EN_MASK;
	uint32_t d_MSDU_LINK_EXT_3_UDP_OVER_IPV6_CHECKSUM_EN_MASK;
	uint32_t d_MSDU_LINK_EXT_3_TCP_OVER_IPV4_CHECKSUM_EN_LSB;
	uint32_t d_MSDU_LINK_EXT_3_TCP_OVER_IPV6_CHECKSUM_EN_LSB;
	uint32_t d_MSDU_LINK_EXT_3_UDP_OVER_IPV4_CHECKSUM_EN_LSB;
	uint32_t d_MSDU_LINK_EXT_3_UDP_OVER_IPV6_CHECKSUM_EN_LSB;
	/* copy_engine.c */
	uint32_t d_SR_WR_INDEX_ADDRESS;
	uint32_t d_DST_WATERMARK_ADDRESS;
	/* htt_rx.c */
	uint32_t d_RX_MSDU_END_4_FIRST_MSDU_MASK;
	uint32_t d_RX_MSDU_END_4_FIRST_MSDU_LSB;
	uint32_t d_RX_MPDU_START_0_RETRY_LSB;
	uint32_t d_RX_MPDU_START_0_RETRY_MASK;
	uint32_t d_RX_MPDU_START_0_SEQ_NUM_MASK;
	uint32_t d_RX_MPDU_START_0_SEQ_NUM_LSB;
	uint32_t d_RX_MPDU_START_2_PN_47_32_LSB;
	uint32_t d_RX_MPDU_START_2_PN_47_32_MASK;
	uint32_t d_RX_MPDU_START_2_TID_LSB;
	uint32_t d_RX_MPDU_START_2_TID_MASK;
	uint32_t d_RX_MSDU_END_1_EXT_WAPI_PN_63_48_MASK;
	uint32_t d_RX_MSDU_END_1_EXT_WAPI_PN_63_48_LSB;
	uint32_t d_RX_MSDU_END_1_KEY_ID_OCT_MASK;
	uint32_t d_RX_MSDU_END_1_KEY_ID_OCT_LSB;
	uint32_t d_RX_MSDU_END_4_LAST_MSDU_MASK;
	uint32_t d_RX_MSDU_END_4_LAST_MSDU_LSB;
	uint32_t d_RX_ATTENTION_0_MCAST_BCAST_MASK;
	uint32_t d_RX_ATTENTION_0_MCAST_BCAST_LSB;
	uint32_t d_RX_ATTENTION_0_FRAGMENT_MASK;
	uint32_t d_RX_ATTENTION_0_FRAGMENT_LSB;
	uint32_t d_RX_ATTENTION_0_MPDU_LENGTH_ERR_MASK;
	uint32_t d_RX_FRAG_INFO_0_RING2_MORE_COUNT_MASK;
	uint32_t d_RX_FRAG_INFO_0_RING2_MORE_COUNT_LSB;
	uint32_t d_RX_MSDU_START_0_MSDU_LENGTH_MASK;
	uint32_t d_RX_MSDU_START_0_MSDU_LENGTH_LSB;
	uint32_t d_RX_MSDU_START_2_DECAP_FORMAT_OFFSET;
	uint32_t d_RX_MSDU_START_2_DECAP_FORMAT_MASK;
	uint32_t d_RX_MSDU_START_2_DECAP_FORMAT_LSB;
	uint32_t d_RX_MPDU_START_0_ENCRYPTED_MASK;
	uint32_t d_RX_MPDU_START_0_ENCRYPTED_LSB;
	uint32_t d_RX_ATTENTION_0_MORE_DATA_MASK;
	uint32_t d_RX_ATTENTION_0_MSDU_DONE_MASK;
	uint32_t d_RX_ATTENTION_0_TCP_UDP_CHKSUM_FAIL_MASK;
	/* end */

	/* PLL start */
	uint32_t d_EFUSE_OFFSET;
	uint32_t d_EFUSE_XTAL_SEL_MSB;
	uint32_t d_EFUSE_XTAL_SEL_LSB;
	uint32_t d_EFUSE_XTAL_SEL_MASK;
	uint32_t d_BB_PLL_CONFIG_OFFSET;
	uint32_t d_BB_PLL_CONFIG_OUTDIV_MSB;
	uint32_t d_BB_PLL_CONFIG_OUTDIV_LSB;
	uint32_t d_BB_PLL_CONFIG_OUTDIV_MASK;
	uint32_t d_BB_PLL_CONFIG_FRAC_MSB;
	uint32_t d_BB_PLL_CONFIG_FRAC_LSB;
	uint32_t d_BB_PLL_CONFIG_FRAC_MASK;
	uint32_t d_WLAN_PLL_SETTLE_TIME_MSB;
	uint32_t d_WLAN_PLL_SETTLE_TIME_LSB;
	uint32_t d_WLAN_PLL_SETTLE_TIME_MASK;
	uint32_t d_WLAN_PLL_SETTLE_OFFSET;
	uint32_t d_WLAN_PLL_SETTLE_SW_MASK;
	uint32_t d_WLAN_PLL_SETTLE_RSTMASK;
	uint32_t d_WLAN_PLL_SETTLE_RESET;
	uint32_t d_WLAN_PLL_CONTROL_NOPWD_MSB;
	uint32_t d_WLAN_PLL_CONTROL_NOPWD_LSB;
	uint32_t d_WLAN_PLL_CONTROL_NOPWD_MASK;
	uint32_t d_WLAN_PLL_CONTROL_BYPASS_MSB;
	uint32_t d_WLAN_PLL_CONTROL_BYPASS_LSB;
	uint32_t d_WLAN_PLL_CONTROL_BYPASS_MASK;
	uint32_t d_WLAN_PLL_CONTROL_BYPASS_RESET;
	uint32_t d_WLAN_PLL_CONTROL_CLK_SEL_MSB;
	uint32_t d_WLAN_PLL_CONTROL_CLK_SEL_LSB;
	uint32_t d_WLAN_PLL_CONTROL_CLK_SEL_MASK;
	uint32_t d_WLAN_PLL_CONTROL_CLK_SEL_RESET;
	uint32_t d_WLAN_PLL_CONTROL_REFDIV_MSB;
	uint32_t d_WLAN_PLL_CONTROL_REFDIV_LSB;
	uint32_t d_WLAN_PLL_CONTROL_REFDIV_MASK;
	uint32_t d_WLAN_PLL_CONTROL_REFDIV_RESET;
	uint32_t d_WLAN_PLL_CONTROL_DIV_MSB;
	uint32_t d_WLAN_PLL_CONTROL_DIV_LSB;
	uint32_t d_WLAN_PLL_CONTROL_DIV_MASK;
	uint32_t d_WLAN_PLL_CONTROL_DIV_RESET;
	uint32_t d_WLAN_PLL_CONTROL_OFFSET;
	uint32_t d_WLAN_PLL_CONTROL_SW_MASK;
	uint32_t d_WLAN_PLL_CONTROL_RSTMASK;
	uint32_t d_WLAN_PLL_CONTROL_RESET;
	uint32_t d_SOC_CORE_CLK_CTRL_OFFSET;
	uint32_t d_SOC_CORE_CLK_CTRL_DIV_MSB;
	uint32_t d_SOC_CORE_CLK_CTRL_DIV_LSB;
	uint32_t d_SOC_CORE_CLK_CTRL_DIV_MASK;
	uint32_t d_RTC_SYNC_STATUS_PLL_CHANGING_MSB;
	uint32_t d_RTC_SYNC_STATUS_PLL_CHANGING_LSB;
	uint32_t d_RTC_SYNC_STATUS_PLL_CHANGING_MASK;
	uint32_t d_RTC_SYNC_STATUS_PLL_CHANGING_RESET;
	uint32_t d_RTC_SYNC_STATUS_OFFSET;
	uint32_t d_SOC_CPU_CLOCK_OFFSET;
	uint32_t d_SOC_CPU_CLOCK_STANDARD_MSB;
	uint32_t d_SOC_CPU_CLOCK_STANDARD_LSB;
	uint32_t d_SOC_CPU_CLOCK_STANDARD_MASK;
	/* PLL end */

	uint32_t d_SOC_POWER_REG_OFFSET;
	uint32_t d_PCIE_INTR_CAUSE_ADDRESS;
	uint32_t d_SOC_RESET_CONTROL_ADDRESS;
	uint32_t d_SOC_RESET_CONTROL_PCIE_RST_SHORT_OVRD_MASK;
	uint32_t d_SOC_RESET_CONTROL_PCIE_RST_SHORT_OVRD_LSB;
	uint32_t d_SOC_RESET_CONTROL_CE_RST_MASK;
	uint32_t d_SOC_RESET_CONTROL_CPU_WARM_RST_MASK;
	uint32_t d_CPU_INTR_ADDRESS;
	uint32_t d_SOC_LF_TIMER_CONTROL0_ADDRESS;
	uint32_t d_SOC_LF_TIMER_CONTROL0_ENABLE_MASK;
	uint32_t d_SOC_LF_TIMER_STATUS0_ADDRESS;

	/* chip id start */
	uint32_t d_SI_CONFIG_ERR_INT_MASK;
	uint32_t d_SI_CONFIG_ERR_INT_LSB;
	uint32_t d_GPIO_ENABLE_W1TS_LOW_ADDRESS;
	uint32_t d_GPIO_PIN0_CONFIG_LSB;
	uint32_t d_GPIO_PIN0_PAD_PULL_LSB;
	uint32_t d_GPIO_PIN0_PAD_PULL_MASK;

	uint32_t d_SOC_CHIP_ID_ADDRESS;
	uint32_t d_SOC_CHIP_ID_VERSION_MASK;
	uint32_t d_SOC_CHIP_ID_VERSION_LSB;
	uint32_t d_SOC_CHIP_ID_REVISION_MASK;
	uint32_t d_SOC_CHIP_ID_REVISION_LSB;
	uint32_t d_SOC_CHIP_ID_REVISION_MSB;
	uint32_t d_FW_AXI_MSI_ADDR;
	uint32_t d_FW_AXI_MSI_DATA;
	uint32_t d_WLAN_SUBSYSTEM_CORE_ID_ADDRESS;

	/* chip id end */

	uint32_t d_A_SOC_CORE_SCRATCH_0_ADDRESS;
	uint32_t d_A_SOC_CORE_SCRATCH_1_ADDRESS;
	uint32_t d_A_SOC_CORE_SCRATCH_2_ADDRESS;
	uint32_t d_A_SOC_CORE_SCRATCH_3_ADDRESS;
	uint32_t d_A_SOC_CORE_SCRATCH_4_ADDRESS;
	uint32_t d_A_SOC_CORE_SCRATCH_5_ADDRESS;
	uint32_t d_A_SOC_CORE_SCRATCH_6_ADDRESS;
	uint32_t d_A_SOC_CORE_SCRATCH_7_ADDRESS;
	uint32_t d_A_SOC_CORE_SPARE_0_REGISTER;
	uint32_t d_PCIE_INTR_FIRMWARE_ROUTE_MASK;
	uint32_t d_A_SOC_CORE_PCIE_INTR_CAUSE_GRP1;
	uint32_t d_A_SOC_CORE_SPARE_1_REGISTER;
	uint32_t d_A_SOC_CORE_PCIE_INTR_CLR_GRP1;
	uint32_t d_A_SOC_CORE_PCIE_INTR_ENABLE_GRP1;
	uint32_t d_A_SOC_PCIE_PCIE_SCRATCH_0;
	uint32_t d_A_SOC_PCIE_PCIE_SCRATCH_1;
	uint32_t d_A_WIFI_APB_1_A_WFSS_CE_TARGET_HOST_DELTA;
	uint32_t d_A_SOC_PCIE_PCIE_SCRATCH_2;
	uint32_t d_A_SOC_CORE_PCIE_INTR_ENABLE_GRP0_Q6_MASK;

	uint32_t d_WLAN_DEBUG_INPUT_SEL_OFFSET;
	uint32_t d_WLAN_DEBUG_INPUT_SEL_SRC_MSB;
	uint32_t d_WLAN_DEBUG_INPUT_SEL_SRC_LSB;
	uint32_t d_WLAN_DEBUG_INPUT_SEL_SRC_MASK;
	uint32_t d_WLAN_DEBUG_CONTROL_OFFSET;
	uint32_t d_WLAN_DEBUG_CONTROL_ENABLE_MSB;
	uint32_t d_WLAN_DEBUG_CONTROL_ENABLE_LSB;
	uint32_t d_WLAN_DEBUG_CONTROL_ENABLE_MASK;
	uint32_t d_WLAN_DEBUG_OUT_OFFSET;
	uint32_t d_WLAN_DEBUG_OUT_DATA_MSB;
	uint32_t d_WLAN_DEBUG_OUT_DATA_LSB;
	uint32_t d_WLAN_DEBUG_OUT_DATA_MASK;
	uint32_t d_AMBA_DEBUG_BUS_OFFSET;
	uint32_t d_AMBA_DEBUG_BUS_PCIE_DEBUG_SEL_MSB;
	uint32_t d_AMBA_DEBUG_BUS_PCIE_DEBUG_SEL_LSB;
	uint32_t d_AMBA_DEBUG_BUS_PCIE_DEBUG_SEL_MASK;
	uint32_t d_AMBA_DEBUG_BUS_SEL_MSB;
	uint32_t d_AMBA_DEBUG_BUS_SEL_LSB;
	uint32_t d_AMBA_DEBUG_BUS_SEL_MASK;

#ifdef QCA_WIFI_3_0_ADRASTEA
	uint32_t d_Q6_ENABLE_REGISTER_0;
	uint32_t d_Q6_ENABLE_REGISTER_1;
	uint32_t d_Q6_CAUSE_REGISTER_0;
	uint32_t d_Q6_CAUSE_REGISTER_1;
	uint32_t d_Q6_CLEAR_REGISTER_0;
	uint32_t d_Q6_CLEAR_REGISTER_1;
#endif
#ifdef CONFIG_BYPASS_QMI
	uint32_t d_BYPASS_QMI_TEMP_REGISTER;
#endif
	uint32_t d_WIFICMN_INT_STATUS_ADDRESS;
};

struct hostdef_s {
	uint32_t d_INT_STATUS_ENABLE_ERROR_LSB;
	uint32_t d_INT_STATUS_ENABLE_ERROR_MASK;
	uint32_t d_INT_STATUS_ENABLE_CPU_LSB;
	uint32_t d_INT_STATUS_ENABLE_CPU_MASK;
	uint32_t d_INT_STATUS_ENABLE_COUNTER_LSB;
	uint32_t d_INT_STATUS_ENABLE_COUNTER_MASK;
	uint32_t d_INT_STATUS_ENABLE_MBOX_DATA_LSB;
	uint32_t d_INT_STATUS_ENABLE_MBOX_DATA_MASK;
	uint32_t d_ERROR_STATUS_ENABLE_RX_UNDERFLOW_LSB;
	uint32_t d_ERROR_STATUS_ENABLE_RX_UNDERFLOW_MASK;
	uint32_t d_ERROR_STATUS_ENABLE_TX_OVERFLOW_LSB;
	uint32_t d_ERROR_STATUS_ENABLE_TX_OVERFLOW_MASK;
	uint32_t d_COUNTER_INT_STATUS_ENABLE_BIT_LSB;
	uint32_t d_COUNTER_INT_STATUS_ENABLE_BIT_MASK;
	uint32_t d_INT_STATUS_ENABLE_ADDRESS;
	uint32_t d_CPU_INT_STATUS_ENABLE_BIT_LSB;
	uint32_t d_CPU_INT_STATUS_ENABLE_BIT_MASK;
	uint32_t d_HOST_INT_STATUS_ADDRESS;
	uint32_t d_CPU_INT_STATUS_ADDRESS;
	uint32_t d_ERROR_INT_STATUS_ADDRESS;
	uint32_t d_ERROR_INT_STATUS_WAKEUP_MASK;
	uint32_t d_ERROR_INT_STATUS_WAKEUP_LSB;
	uint32_t d_ERROR_INT_STATUS_RX_UNDERFLOW_MASK;
	uint32_t d_ERROR_INT_STATUS_RX_UNDERFLOW_LSB;
	uint32_t d_ERROR_INT_STATUS_TX_OVERFLOW_MASK;
	uint32_t d_ERROR_INT_STATUS_TX_OVERFLOW_LSB;
	uint32_t d_COUNT_DEC_ADDRESS;
	uint32_t d_HOST_INT_STATUS_CPU_MASK;
	uint32_t d_HOST_INT_STATUS_CPU_LSB;
	uint32_t d_HOST_INT_STATUS_ERROR_MASK;
	uint32_t d_HOST_INT_STATUS_ERROR_LSB;
	uint32_t d_HOST_INT_STATUS_COUNTER_MASK;
	uint32_t d_HOST_INT_STATUS_COUNTER_LSB;
	uint32_t d_RX_LOOKAHEAD_VALID_ADDRESS;
	uint32_t d_WINDOW_DATA_ADDRESS;
	uint32_t d_WINDOW_READ_ADDR_ADDRESS;
	uint32_t d_WINDOW_WRITE_ADDR_ADDRESS;
	uint32_t d_SOC_GLOBAL_RESET_ADDRESS;
	uint32_t d_RTC_STATE_ADDRESS;
	uint32_t d_RTC_STATE_COLD_RESET_MASK;
	uint32_t d_PCIE_LOCAL_BASE_ADDRESS;
	uint32_t d_PCIE_SOC_WAKE_RESET;
	uint32_t d_PCIE_SOC_WAKE_ADDRESS;
	uint32_t d_PCIE_SOC_WAKE_V_MASK;
	uint32_t d_RTC_STATE_V_MASK;
	uint32_t d_RTC_STATE_V_LSB;
	uint32_t d_FW_IND_EVENT_PENDING;
	uint32_t d_FW_IND_INITIALIZED;
	uint32_t d_FW_IND_HELPER;
	uint32_t d_RTC_STATE_V_ON;
#if defined(SDIO_3_0)
	uint32_t d_HOST_INT_STATUS_MBOX_DATA_MASK;
	uint32_t d_HOST_INT_STATUS_MBOX_DATA_LSB;
#endif
	uint32_t d_PCIE_SOC_RDY_STATUS_ADDRESS;
	uint32_t d_PCIE_SOC_RDY_STATUS_BAR_MASK;
	uint32_t d_SOC_PCIE_BASE_ADDRESS;
	uint32_t d_MSI_MAGIC_ADR_ADDRESS;
	uint32_t d_MSI_MAGIC_ADDRESS;
	uint32_t d_HOST_CE_COUNT;
	uint32_t d_ENABLE_MSI;
	uint32_t d_MUX_ID_MASK;
	uint32_t d_TRANSACTION_ID_MASK;
	uint32_t d_DESC_DATA_FLAG_MASK;
	uint32_t d_A_SOC_PCIE_PCIE_BAR0_START;
	uint32_t d_FW_IND_HOST_READY;
};

struct host_shadow_regs_s {
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_0;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_1;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_2;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_3;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_4;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_5;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_6;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_7;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_8;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_9;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_10;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_11;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_12;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_13;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_14;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_15;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_16;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_17;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_18;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_19;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_20;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_21;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_22;
	uint32_t d_A_LOCAL_SHADOW_REG_VALUE_23;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_0;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_1;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_2;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_3;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_4;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_5;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_6;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_7;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_8;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_9;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_10;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_11;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_12;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_13;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_14;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_15;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_16;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_17;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_18;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_19;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_20;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_21;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_22;
	uint32_t d_A_LOCAL_SHADOW_REG_ADDRESS_23;
};


/*
 * @d_DST_WR_INDEX_ADDRESS: Destination ring write index
 *
 * @d_SRC_WATERMARK_ADDRESS: Source ring watermark
 *
 * @d_SRC_WATERMARK_LOW_MASK: Bits indicating low watermark from Source ring
 *			      watermark
 *
 * @d_SRC_WATERMARK_HIGH_MASK: Bits indicating high watermark from Source ring
 *			       watermark
 *
 * @d_DST_WATERMARK_LOW_MASK: Bits indicating low watermark from Destination
 *			      ring watermark
 *
 * @d_DST_WATERMARK_HIGH_MASK: Bits indicating high watermark from Destination
 *			       ring watermark
 *
 * @d_CURRENT_SRRI_ADDRESS: Current source ring read index.The Start Offset
 *			    will be reflected after a CE transfer is completed.
 *
 * @d_CURRENT_DRRI_ADDRESS: Current Destination ring read index. The Start
 *			    Offset will be reflected after a CE transfer
 *			    is completed.
 *
 * @d_HOST_IS_SRC_RING_HIGH_WATERMARK_MASK: Source ring high watermark
 *					    Interrupt Status
 *
 * @d_HOST_IS_SRC_RING_LOW_WATERMARK_MASK: Source ring low watermark
 *					   Interrupt Status
 *
 * @d_HOST_IS_DST_RING_HIGH_WATERMARK_MASK: Destination ring high watermark
 *					    Interrupt Status
 *
 * @d_HOST_IS_DST_RING_LOW_WATERMARK_MASK: Source ring low watermark
 *					   Interrupt Status
 *
 * @d_HOST_IS_ADDRESS: Host Interrupt Status Register
 *
 * @d_MISC_IS_ADDRESS: Miscellaneous Interrupt Status Register
 *
 * @d_HOST_IS_COPY_COMPLETE_MASK: Bits indicating Copy complete interrupt
 *				  status from the Host Interrupt Status
 *				  register
 *
 * @d_CE_WRAPPER_BASE_ADDRESS: Copy Engine Wrapper Base Address
 *
 * @d_CE_WRAPPER_INTERRUPT_SUMMARY_ADDRESS: CE Wrapper summary for interrupts
 *					    to host
 *
 * @d_CE_WRAPPER_INDEX_BASE_LOW: The LSB Base address to which source and
 *				 destination read indices are written
 *
 * @d_CE_WRAPPER_INDEX_BASE_HIGH: The MSB Base address to which source and
 *				  destination read indices are written
 *
 * @d_HOST_IE_ADDRESS: Host Line Interrupt Enable Register
 *
 * @d_HOST_IE_COPY_COMPLETE_MASK: Bits indicating Copy complete interrupt
 *				  enable from the IE register
 *
 * @d_HOST_IE_SRC_TIMER_BATCH_MASK: Bits indicating src timer batch interrupt
 *					enable from the IE register
 *
 * @d_HOST_IE_DST_TIMER_BATCH_MASK: Bits indicating dst timer batch interrupt
 *					enable from the IE register
 *
 * @d_SR_BA_ADDRESS: LSB of Source Ring Base Address
 *
 * @d_SR_BA_ADDRESS_HIGH: MSB of Source Ring Base Address
 *
 * @d_SR_SIZE_ADDRESS: Source Ring size - number of entries and Start Offset
 *
 * @d_CE_CTRL1_ADDRESS: CE Control register
 *
 * @d_CE_CTRL1_DMAX_LENGTH_MASK: Destination buffer Max Length used for error
 *				 check
 *
 * @d_DR_BA_ADDRESS: Destination Ring Base Address Low
 *
 * @d_DR_BA_ADDRESS_HIGH: Destination Ring Base Address High
 *
 * @d_DR_SIZE_ADDRESS: Destination Ring size - number of entries Start Offset
 *
 * @d_CE_CMD_REGISTER: Implements commands to all CE Halt Flush
 *
 * @d_CE_MSI_ADDRESS: CE MSI LOW Address register
 *
 * @d_CE_MSI_ADDRESS_HIGH: CE MSI High Address register
 *
 * @d_CE_MSI_DATA: CE MSI Data Register
 *
 * @d_CE_MSI_ENABLE_BIT: Bit in CTRL1 register indication the MSI enable
 *
 * @d_MISC_IE_ADDRESS: Miscellaneous Interrupt Enable Register
 *
 * @d_MISC_IS_AXI_ERR_MASK:
 *		Bit in Misc IS indicating AXI Timeout Interrupt status
 *
 * @d_MISC_IS_DST_ADDR_ERR_MASK:
 *		Bit in Misc IS indicating Destination Address Error
 *
 * @d_MISC_IS_SRC_LEN_ERR_MASK: Bit in Misc IS indicating Source Zero Length
 *				Error Interrupt status
 *
 * @d_MISC_IS_DST_MAX_LEN_VIO_MASK: Bit in Misc IS indicating Destination Max
 *				    Length Violated Interrupt status
 *
 * @d_MISC_IS_DST_RING_OVERFLOW_MASK: Bit in Misc IS indicating Destination
 *				      Ring Overflow Interrupt status
 *
 * @d_MISC_IS_SRC_RING_OVERFLOW_MASK: Bit in Misc IS indicating Source Ring
 *				      Overflow Interrupt status
 *
 * @d_SRC_WATERMARK_LOW_LSB: Source Ring Low Watermark LSB
 *
 * @d_SRC_WATERMARK_HIGH_LSB: Source Ring Low Watermark MSB
 *
 * @d_DST_WATERMARK_LOW_LSB: Destination Ring Low Watermark LSB
 *
 * @d_DST_WATERMARK_HIGH_LSB: Destination Ring High Watermark LSB
 *
 * @d_CE_WRAPPER_INTERRUPT_SUMMARY_HOST_MSI_MASK:
 *		Bits in d_CE_WRAPPER_INTERRUPT_SUMMARY_ADDR
 *		indicating Copy engine miscellaneous interrupt summary
 *
 * @d_CE_WRAPPER_INTERRUPT_SUMMARY_HOST_MSI_LSB:
 *		Bits in d_CE_WRAPPER_INTERRUPT_SUMMARY_ADDR
 *		indicating Host interrupts summary
 *
 * @d_CE_CTRL1_DMAX_LENGTH_LSB:
 *		LSB of Destination buffer Max Length used for error check
 *
 * @d_CE_CTRL1_SRC_RING_BYTE_SWAP_EN_MASK:
 *		Bits indicating Source ring Byte Swap enable.
 *		Treats source ring memory organisation as big-endian.
 *
 * @d_CE_CTRL1_DST_RING_BYTE_SWAP_EN_MASK:
 *		Bits indicating Destination ring byte swap enable.
 *		Treats destination ring memory organisation as big-endian
 *
 * @d_CE_CTRL1_SRC_RING_BYTE_SWAP_EN_LSB:
 *		LSB of Source ring Byte Swap enable
 *
 * @d_CE_CTRL1_DST_RING_BYTE_SWAP_EN_LSB:
 *		LSB of Destination ring Byte Swap enable
 *
 * @d_CE_WRAPPER_DEBUG_OFFSET: Offset of CE OBS BUS Select register
 *
 * @d_CE_WRAPPER_DEBUG_SEL_MSB:
 *		MSB of Control register selecting inputs for trace/debug
 *
 * @d_CE_WRAPPER_DEBUG_SEL_LSB:
 *		LSB of Control register selecting inputs for trace/debug
 *
 * @d_CE_WRAPPER_DEBUG_SEL_MASK:
 *		Bit mask for trace/debug Control register
 *
 * @d_CE_DEBUG_OFFSET: Offset of Copy Engine FSM Debug Status
 *
 * @d_CE_DEBUG_SEL_MSB: MSB of Copy Engine FSM Debug Status
 *
 * @d_CE_DEBUG_SEL_LSB: LSB of Copy Engine FSM Debug Status
 *
 * @d_CE_DEBUG_SEL_MASK: Bits indicating Copy Engine FSM Debug Status
 *
 * @d_HOST_CMEM_ADDRESS: Base address of CMEM
 *
 * @d_CE_SRC_BATCH_TIMER_THRESH_MASK: SRC ring timer threshold for interrupt
 *
 * @d_CE_SRC_BATCH_COUNTER_THRESH_MASK: SRC ring counter threshold for
 *					interrupt
 *
 * @d_CE_SRC_BATCH_TIMER_THRESH_LSB: LSB for src ring timer threshold
 *
 * @d_CE_SRC_BATCH_COUNTER_THRESH_LSB: LSB for src ring counter threshold
 *
 * @d_CE_DST_BATCH_TIMER_THRESH_MASK: DST ring timer threshold for interrupt
 *
 * @d_CE_DST_BATCH_COUNTER_THRESH_MASK: DST ring counter threshold for
 *					interrupt
 *
 * @d_CE_DST_BATCH_TIMER_THRESH_LSB: LSB for dst ring timer threshold
 *
 * @d_CE_DST_BATCH_COUNTER_THRESH_LSB: LSB for dst ring counter threshold
 *
 */
struct ce_reg_def {
	/* copy_engine.c */
	uint32_t d_DST_WR_INDEX_ADDRESS;
	uint32_t d_SRC_WATERMARK_ADDRESS;
	uint32_t d_SRC_WATERMARK_LOW_MASK;
	uint32_t d_SRC_WATERMARK_HIGH_MASK;
	uint32_t d_DST_WATERMARK_LOW_MASK;
	uint32_t d_DST_WATERMARK_HIGH_MASK;
	uint32_t d_CURRENT_SRRI_ADDRESS;
	uint32_t d_CURRENT_DRRI_ADDRESS;
	uint32_t d_HOST_IS_SRC_RING_HIGH_WATERMARK_MASK;
	uint32_t d_HOST_IS_SRC_RING_LOW_WATERMARK_MASK;
	uint32_t d_HOST_IS_DST_RING_HIGH_WATERMARK_MASK;
	uint32_t d_HOST_IS_DST_RING_LOW_WATERMARK_MASK;
	uint32_t d_HOST_IS_ADDRESS;
	uint32_t d_MISC_IS_ADDRESS;
	uint32_t d_HOST_IS_COPY_COMPLETE_MASK;
	uint32_t d_CE_WRAPPER_BASE_ADDRESS;
	uint32_t d_CE_WRAPPER_INTERRUPT_SUMMARY_ADDRESS;
	uint32_t d_CE_DDR_ADDRESS_FOR_RRI_LOW;
	uint32_t d_CE_DDR_ADDRESS_FOR_RRI_HIGH;
	uint32_t d_HOST_IE_ADDRESS;
	uint32_t d_HOST_IE_ADDRESS_2;
	uint32_t d_HOST_IE_COPY_COMPLETE_MASK;
	uint32_t d_HOST_IE_SRC_TIMER_BATCH_MASK;
	uint32_t d_HOST_IE_DST_TIMER_BATCH_MASK;
	uint32_t d_SR_BA_ADDRESS;
	uint32_t d_SR_BA_ADDRESS_HIGH;
	uint32_t d_SR_SIZE_ADDRESS;
	uint32_t d_CE_CTRL1_ADDRESS;
	uint32_t d_CE_CTRL1_DMAX_LENGTH_MASK;
	uint32_t d_DR_BA_ADDRESS;
	uint32_t d_DR_BA_ADDRESS_HIGH;
	uint32_t d_DR_SIZE_ADDRESS;
	uint32_t d_CE_CMD_REGISTER;
	uint32_t d_CE_MSI_ADDRESS;
	uint32_t d_CE_MSI_ADDRESS_HIGH;
	uint32_t d_CE_MSI_DATA;
	uint32_t d_CE_MSI_ENABLE_BIT;
	uint32_t d_MISC_IE_ADDRESS;
	uint32_t d_MISC_IS_AXI_ERR_MASK;
	uint32_t d_MISC_IS_DST_ADDR_ERR_MASK;
	uint32_t d_MISC_IS_SRC_LEN_ERR_MASK;
	uint32_t d_MISC_IS_DST_MAX_LEN_VIO_MASK;
	uint32_t d_MISC_IS_DST_RING_OVERFLOW_MASK;
	uint32_t d_MISC_IS_SRC_RING_OVERFLOW_MASK;
	uint32_t d_SRC_WATERMARK_LOW_LSB;
	uint32_t d_SRC_WATERMARK_HIGH_LSB;
	uint32_t d_DST_WATERMARK_LOW_LSB;
	uint32_t d_DST_WATERMARK_HIGH_LSB;
	uint32_t d_CE_WRAPPER_INTERRUPT_SUMMARY_HOST_MSI_MASK;
	uint32_t d_CE_WRAPPER_INTERRUPT_SUMMARY_HOST_MSI_LSB;
	uint32_t d_CE_CTRL1_DMAX_LENGTH_LSB;
	uint32_t d_CE_CTRL1_SRC_RING_BYTE_SWAP_EN_MASK;
	uint32_t d_CE_CTRL1_DST_RING_BYTE_SWAP_EN_MASK;
	uint32_t d_CE_CTRL1_SRC_RING_BYTE_SWAP_EN_LSB;
	uint32_t d_CE_CTRL1_DST_RING_BYTE_SWAP_EN_LSB;
	uint32_t d_CE_CTRL1_IDX_UPD_EN_MASK;
	uint32_t d_CE_WRAPPER_DEBUG_OFFSET;
	uint32_t d_CE_WRAPPER_DEBUG_SEL_MSB;
	uint32_t d_CE_WRAPPER_DEBUG_SEL_LSB;
	uint32_t d_CE_WRAPPER_DEBUG_SEL_MASK;
	uint32_t d_CE_DEBUG_OFFSET;
	uint32_t d_CE_DEBUG_SEL_MSB;
	uint32_t d_CE_DEBUG_SEL_LSB;
	uint32_t d_CE_DEBUG_SEL_MASK;
	uint32_t d_CE0_BASE_ADDRESS;
	uint32_t d_CE1_BASE_ADDRESS;
	uint32_t d_A_WIFI_APB_3_A_WCMN_APPS_CE_INTR_ENABLES;
	uint32_t d_A_WIFI_APB_3_A_WCMN_APPS_CE_INTR_STATUS;
	uint32_t d_HOST_IE_ADDRESS_3;
	uint32_t d_HOST_IE_REG1_CE_LSB;
	uint32_t d_HOST_IE_REG2_CE_LSB;
	uint32_t d_HOST_IE_REG3_CE_LSB;
	uint32_t d_HOST_CE_ADDRESS;
	uint32_t d_HOST_CMEM_ADDRESS;
	uint32_t d_PMM_SCRATCH_BASE;
	uint32_t d_CE_SRC_BATCH_TIMER_THRESH_MASK;
	uint32_t d_CE_SRC_BATCH_COUNTER_THRESH_MASK;
	uint32_t d_CE_SRC_BATCH_TIMER_THRESH_LSB;
	uint32_t d_CE_SRC_BATCH_COUNTER_THRESH_LSB;
	uint32_t d_CE_DST_BATCH_TIMER_THRESH_MASK;
	uint32_t d_CE_DST_BATCH_COUNTER_THRESH_MASK;
	uint32_t d_CE_DST_BATCH_TIMER_THRESH_LSB;
	uint32_t d_CE_DST_BATCH_COUNTER_THRESH_LSB;
	uint32_t d_CE_SRC_BATCH_TIMER_INT_SETUP;
	uint32_t d_CE_DST_BATCH_TIMER_INT_SETUP;
};

#endif
