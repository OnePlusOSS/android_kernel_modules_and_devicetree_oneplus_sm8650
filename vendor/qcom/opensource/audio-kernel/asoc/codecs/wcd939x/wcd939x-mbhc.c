// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asoc/wcdcal-hwdep.h>
#include <asoc/wcd-mbhc-v2-api.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include "wcd939x-registers.h"
#include "internal.h"
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
#include <linux/soc/qcom/wcd939x-i2c.h>
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
#include "feedback/oplus_audio_kernel_fb.h"
#ifdef dev_err_ratelimited
#undef dev_err_ratelimited
#define dev_err_ratelimited dev_err_ratelimited_fb
#endif
#ifdef pr_err_ratelimited
#undef pr_err_ratelimited
#define pr_err_ratelimited pr_err_ratelimited_fb
#endif
#ifdef pr_err
#undef pr_err
#define pr_err pr_err_fb_delay
#endif
#endif /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */

#define WCD939X_ZDET_SUPPORTED          true
/* Z value defined in milliohm */
#define WCD939X_ZDET_VAL_32             32000
#define WCD939X_ZDET_VAL_400            400000
#define WCD939X_ZDET_VAL_1200           1200000
#define WCD939X_ZDET_VAL_100K           100000000
/* Z floating defined in ohms */
#define WCD939X_ZDET_FLOATING_IMPEDANCE 0x0FFFFFFE

#define WCD939X_ZDET_NUM_MEASUREMENTS   900
#define WCD939X_MBHC_GET_C1(c)          ((c & 0xC000) >> 14)
#define WCD939X_MBHC_GET_X1(x)          (x & 0x3FFF)
/* Z value compared in milliOhm */
#define WCD939X_MBHC_IS_SECOND_RAMP_REQUIRED(z) false
#define WCD939X_MBHC_ZDET_CONST         (1071 * 1024)
#define WCD939X_MBHC_MOISTURE_RREF      R_24_KOHM

#define OHMS_TO_MILLIOHMS 1000
#define FLOAT_TO_FIXED_XTALK (1UL << 16)
#define MAX_XTALK_ALPHA 255
#define MIN_RL_EFF_MOHMS 1
#define MAX_RL_EFF_MOHMS 900000
#define HD2_CODE_BASE_VALUE 0x1D
#define HD2_CODE_INV_RESOLUTION 4201025
#define FLOAT_TO_FIXED_LINEARIZER (1UL << 12)
#define MIN_TAP_OFFSET -1023
#define MAX_TAP_OFFSET 1023
#define MIN_TAP 0
#define MAX_TAP 1023
#define RDOWN_TIMER_PERIOD_MSEC 100

#define WCD_USBSS_WRITE true
#define WCD_USBSS_READ false
#define WCD_USBSS_EXT_LIN_EN 0x3D
#define WCD_USBSS_EXT_SW_CTRL_1 0x43
#define WCD_USBSS_MG1_BIAS 0x25
#define WCD_USBSS_MG2_BIAS 0x29

static struct wcd_mbhc_register
	wcd_mbhc_registers[WCD_MBHC_REG_FUNC_MAX] = {
	WCD_MBHC_REGISTER("WCD_MBHC_L_DET_EN",
			  WCD939X_MBHC_MECH, 0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_GND_DET_EN",
			  WCD939X_MBHC_MECH, 0x40, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MECH_DETECTION_TYPE",
			  WCD939X_MBHC_MECH, 0x20, 5, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MIC_CLAMP_CTL",
			  WCD939X_PLUG_DETECT_CTL, 0x30, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ELECT_DETECTION_TYPE",
			  WCD939X_MBHC_ELECT, 0x08, 3, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HS_L_DET_PULL_UP_CTRL",
			  WCD939X_MECH_DET_CURRENT, 0x1F, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HS_L_DET_PULL_UP_COMP_CTRL",
			  WCD939X_MBHC_MECH, 0x04, 2, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_PLUG_TYPE",
			  WCD939X_MBHC_MECH, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_GND_PLUG_TYPE",
			  WCD939X_MBHC_MECH, 0x08, 3, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_SW_HPH_LP_100K_TO_GND",
			  WCD939X_MBHC_MECH, 0x01, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ELECT_SCHMT_ISRC",
			  WCD939X_MBHC_ELECT, 0x06, 1, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_FSM_EN",
			  WCD939X_MBHC_ELECT, 0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_INSREM_DBNC",
			  WCD939X_PLUG_DETECT_CTL, 0x0F, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_BTN_DBNC",
			  WCD939X_CTL_1, 0x03, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HS_VREF",
			  WCD939X_CTL_2, 0x03, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HS_COMP_RESULT",
			  WCD939X_MBHC_RESULT_3, 0x08, 3, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_IN2P_CLAMP_STATE",
			  WCD939X_MBHC_RESULT_3, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MIC_SCHMT_RESULT",
			  WCD939X_MBHC_RESULT_3, 0x20, 5, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_SCHMT_RESULT",
			  WCD939X_MBHC_RESULT_3, 0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHR_SCHMT_RESULT",
			  WCD939X_MBHC_RESULT_3, 0x40, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_OCP_FSM_EN",
			  WCD939X_HPH_OCP_CTL, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_BTN_RESULT",
			  WCD939X_MBHC_RESULT_3, 0x07, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_BTN_ISRC_CTL",
			  WCD939X_MBHC_ELECT, 0x70, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ELECT_RESULT",
			  WCD939X_MBHC_RESULT_3, 0xFF, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MICB_CTRL",
			  WCD939X_MICB2, 0xC0, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPH_CNP_WG_TIME",
			  WCD939X_CNP_WG_TIME, 0xFF, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHR_PA_EN",
			  WCD939X_HPH, 0x40, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_PA_EN",
			  WCD939X_HPH, 0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPH_PA_EN",
			  WCD939X_HPH, 0xC0, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_SWCH_LEVEL_REMOVE",
			  WCD939X_MBHC_RESULT_3, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_PULLDOWN_CTRL",
			  0, 0, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ANC_DET_EN",
			  WCD939X_CTL_BCS, 0x02, 1, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_FSM_STATUS",
			  WCD939X_FSM_STATUS, 0x01, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MUX_CTL",
			  WCD939X_CTL_2, 0x70, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MOISTURE_STATUS",
			  WCD939X_FSM_STATUS, 0x20, 5, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHR_GND",
			  WCD939X_PA_CTL2, 0x40, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_GND",
			  WCD939X_PA_CTL2, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_OCP_DET_EN",
			  WCD939X_L_TEST, 0x01, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHR_OCP_DET_EN",
			  WCD939X_R_TEST, 0x01, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_OCP_STATUS",
			  WCD939X_INTR_STATUS_0, 0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHR_OCP_STATUS",
			  WCD939X_INTR_STATUS_0, 0x20, 5, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ADC_EN",
			  WCD939X_CTL_1, 0x08, 3, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ADC_COMPLETE", WCD939X_FSM_STATUS,
			  0x40, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ADC_TIMEOUT", WCD939X_FSM_STATUS,
			  0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ADC_RESULT", WCD939X_ADC_RESULT,
			  0xFF, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MICB2_VOUT", WCD939X_MICB2, 0x3F, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ADC_MODE",
			  WCD939X_CTL_1, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_DETECTION_DONE",
			  WCD939X_CTL_1, 0x04, 2, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ELECT_ISRC_EN",
			  WCD939X_MBHC_ZDET, 0x02, 1, 0),
};

static const struct wcd_mbhc_intr intr_ids = {
	.mbhc_sw_intr =  WCD939X_IRQ_MBHC_SW_DET,
	.mbhc_btn_press_intr = WCD939X_IRQ_MBHC_BUTTON_PRESS_DET,
	.mbhc_btn_release_intr = WCD939X_IRQ_MBHC_BUTTON_RELEASE_DET,
	.mbhc_hs_ins_intr = WCD939X_IRQ_MBHC_ELECT_INS_REM_LEG_DET,
	.mbhc_hs_rem_intr = WCD939X_IRQ_MBHC_ELECT_INS_REM_DET,
	.hph_left_ocp = WCD939X_IRQ_HPHL_OCP_INT,
	.hph_right_ocp = WCD939X_IRQ_HPHR_OCP_INT,
};

struct wcd939x_mbhc_zdet_param {
	u16 ldo_ctl;
	u16 noff;
	u16 nshift;
	u16 btn5;
	u16 btn6;
	u16 btn7;
};

static int wcd939x_mbhc_request_irq(struct snd_soc_component *component,
				  int irq, irq_handler_t handler,
				  const char *name, void *data)
{
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);

	return wcd_request_irq(&wcd939x->irq_info, irq, name, handler, data);
}

static void wcd939x_mbhc_irq_control(struct snd_soc_component *component,
				   int irq, bool enable)
{
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);

	if (enable)
		wcd_enable_irq(&wcd939x->irq_info, irq);
	else
		wcd_disable_irq(&wcd939x->irq_info, irq);
}

static int wcd939x_mbhc_free_irq(struct snd_soc_component *component,
			       int irq, void *data)
{
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);

	wcd_free_irq(&wcd939x->irq_info, irq, data);

	return 0;
}

static void wcd939x_mbhc_clk_setup(struct snd_soc_component *component,
				 bool enable)
{
	if (enable)
		snd_soc_component_update_bits(component, WCD939X_CTL_1,
				    0x80, 0x80);
	else
		snd_soc_component_update_bits(component, WCD939X_CTL_1,
				    0x80, 0x00);
}

static int wcd939x_mbhc_btn_to_num(struct snd_soc_component *component)
{
	return snd_soc_component_read(component, WCD939X_MBHC_RESULT_3) & 0x7;
}

static void wcd939x_mbhc_mbhc_bias_control(struct snd_soc_component *component,
					 bool enable)
{
	if (enable)
		snd_soc_component_update_bits(component, WCD939X_MBHC_ELECT,
				    0x01, 0x01);
	else
		snd_soc_component_update_bits(component, WCD939X_MBHC_ELECT,
				    0x01, 0x00);
}

static void wcd939x_mbhc_program_btn_thr(struct snd_soc_component *component,
				       s16 *btn_low, s16 *btn_high,
				       int num_btn, bool is_micbias)
{
	int i;
	int vth;

	if (num_btn > WCD_MBHC_DEF_BUTTONS) {
		dev_err_ratelimited(component->dev, "%s: invalid number of buttons: %d\n",
			__func__, num_btn);
		return;
	}

	for (i = 0; i < num_btn; i++) {
		vth = ((btn_high[i] * 2) / 25) & 0x3F;
		snd_soc_component_update_bits(component, WCD939X_MBHC_BTN0 + i,
				    0xFC, vth << 2);
		dev_dbg(component->dev, "%s: btn_high[%d]: %d, vth: %d\n",
			__func__, i, btn_high[i], vth);
	}
}

static bool wcd939x_mbhc_lock_sleep(struct wcd_mbhc *mbhc, bool lock)
{
	struct snd_soc_component *component = mbhc->component;
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);

	wcd939x->wakeup((void*)wcd939x, lock);

	return true;
}

static int wcd939x_mbhc_register_notifier(struct wcd_mbhc *mbhc,
					struct notifier_block *nblock,
					bool enable)
{
	struct wcd939x_mbhc *wcd939x_mbhc;

	wcd939x_mbhc = container_of(mbhc, struct wcd939x_mbhc, wcd_mbhc);

	if (enable)
		return blocking_notifier_chain_register(&wcd939x_mbhc->notifier,
							nblock);
	else
		return blocking_notifier_chain_unregister(
				&wcd939x_mbhc->notifier, nblock);
}

static bool wcd939x_mbhc_micb_en_status(struct wcd_mbhc *mbhc, int micb_num)
{
	u8 val = 0;

	if (micb_num == MIC_BIAS_2) {
		val = ((snd_soc_component_read(mbhc->component,
								WCD939X_MICB2) & 0xC0)
			>> 6);
		if (val == 0x01)
			return true;
	}
	return false;
}

static bool wcd939x_mbhc_hph_pa_on_status(struct snd_soc_component *component)
{
	return (snd_soc_component_read(component, WCD939X_HPH) & 0xC0) ?
									true : false;
}

static void wcd939x_mbhc_hph_l_pull_up_control(
							struct snd_soc_component *component,
							int pull_up_cur)
{
	/* Default pull up current to 2uA */
	if (pull_up_cur > HS_PULLUP_I_OFF || pull_up_cur < HS_PULLUP_I_3P0_UA ||
	    pull_up_cur == HS_PULLUP_I_DEFAULT)
		pull_up_cur = HS_PULLUP_I_2P0_UA;

	dev_dbg(component->dev, "%s: HS pull up current:%d\n",
		__func__, pull_up_cur);

	snd_soc_component_update_bits(component,
				WCD939X_MECH_DET_CURRENT,
			    0x1F, pull_up_cur);
}

static int wcd939x_mbhc_request_micbias(struct snd_soc_component *component,
					int micb_num, int req)
{
	int ret = 0;

	ret = wcd939x_micbias_control(component, micb_num, req, false);

	return ret;
}

static void wcd939x_mbhc_micb_ramp_control(struct snd_soc_component *component,
					   bool enable)
{
	if (enable) {
		snd_soc_component_update_bits(component, WCD939X_MICB2_RAMP,
				    0x1C, 0x0C);
		snd_soc_component_update_bits(component, WCD939X_MICB2_RAMP,
				    0x80, 0x80);
	} else {
		snd_soc_component_update_bits(component, WCD939X_MICB2_RAMP,
				    0x80, 0x00);
		snd_soc_component_update_bits(component, WCD939X_MICB2_RAMP,
				    0x1C, 0x00);
	}
}

static struct firmware_cal *wcd939x_get_hwdep_fw_cal(struct wcd_mbhc *mbhc,
						   enum wcd_cal_type type)
{
	struct wcd939x_mbhc *wcd939x_mbhc;
	struct firmware_cal *hwdep_cal;
	struct snd_soc_component *component = mbhc->component;

	wcd939x_mbhc = container_of(mbhc, struct wcd939x_mbhc, wcd_mbhc);

	if (!component) {
		pr_err_ratelimited("%s: NULL component pointer\n", __func__);
		return NULL;
	}
	hwdep_cal = wcdcal_get_fw_cal(wcd939x_mbhc->fw_data, type);
	if (!hwdep_cal)
		dev_err_ratelimited(component->dev, "%s: cal not sent by %d\n",
			__func__, type);

	return hwdep_cal;
}

static int wcd939x_mbhc_micb_ctrl_threshold_mic(
							struct snd_soc_component *component,
							int micb_num, bool req_en)
{
	struct wcd939x_pdata *pdata = dev_get_platdata(component->dev);
	int rc, micb_mv;

	if (micb_num != MIC_BIAS_2)
		return -EINVAL;
	/*
	 * If device tree micbias level is already above the minimum
	 * voltage needed to detect threshold microphone, then do
	 * not change the micbias, just return.
	 */
	if (pdata->micbias.micb2_mv >= WCD_MBHC_THR_HS_MICB_MV)
		return 0;

	micb_mv = req_en ? WCD_MBHC_THR_HS_MICB_MV : pdata->micbias.micb2_mv;

	rc = wcd939x_mbhc_micb_adjust_voltage(component, micb_mv, MIC_BIAS_2);

	return rc;
}

static inline void wcd939x_mbhc_get_result_params(struct wcd939x_priv *wcd939x,
						s16 *d1_a, u16 noff,
						int32_t *zdet)
{
	int i;
	int val, val1;
	s16 c1;
	s32 x1, d1;
	int32_t denom;
	int minCode_param[] = {
			3277, 1639, 820, 410, 205, 103, 52, 26
	};
	struct wcd939x_mbhc *wcd939x_mbhc = wcd939x->mbhc;

	regmap_update_bits(wcd939x->regmap, WCD939X_MBHC_ZDET, 0x20, 0x20);
	for (i = 0; i < WCD939X_ZDET_NUM_MEASUREMENTS; i++) {
		regmap_read(wcd939x->regmap, WCD939X_MBHC_RESULT_2, &val);
		if (val & 0x80)
			break;
	}
	val = val << 0x8;
	regmap_read(wcd939x->regmap, WCD939X_MBHC_RESULT_1, &val1);
	val |= val1;
	wcd939x_mbhc->rdown_prev_iter = val;
	regmap_update_bits(wcd939x->regmap, WCD939X_MBHC_ZDET, 0x20, 0x00);
	x1 = WCD939X_MBHC_GET_X1(val);
	c1 = WCD939X_MBHC_GET_C1(val);
	/* If ramp is not complete, give additional 5ms */
	if ((c1 < 2) && x1)
		usleep_range(5000, 5050);

	if (!c1 || !x1) {
		dev_dbg(wcd939x->dev,
			"%s: Impedance detect ramp error, c1=%d, x1=0x%x\n",
			__func__, c1, x1);
		goto ramp_down;
	}
	d1 = d1_a[c1];
	denom = (x1 * d1) - (1 << (14 - noff));
	if (denom > 0)
		*zdet = (WCD939X_MBHC_ZDET_CONST * 1000) / denom;
	else if (x1 < minCode_param[noff])
		*zdet = WCD939X_ZDET_FLOATING_IMPEDANCE;

	dev_dbg(wcd939x->dev, "%s: d1=%d, c1=%d, x1=0x%x, z_val=%d(milliOhm)\n",
		__func__, d1, c1, x1, *zdet);
ramp_down:
	i = 0;
	wcd939x_mbhc->rdown_timer_complete = false;
	mod_timer(&wcd939x_mbhc->rdown_timer, jiffies + msecs_to_jiffies(RDOWN_TIMER_PERIOD_MSEC));
	while (x1) {
		regmap_read(wcd939x->regmap,
				 WCD939X_MBHC_RESULT_1, &val);
		regmap_read(wcd939x->regmap,
				 WCD939X_MBHC_RESULT_2, &val1);
		val = val << 0x08;
		val |= val1;
		x1 = WCD939X_MBHC_GET_X1(val);
		i++;
		if (i == WCD939X_ZDET_NUM_MEASUREMENTS)
			break;
		if (wcd939x_mbhc->rdown_timer_complete && wcd939x_mbhc->rdown_prev_iter == val)
			break;
		wcd939x_mbhc->rdown_prev_iter = val;
	}
	del_timer(&wcd939x_mbhc->rdown_timer);
}

static void wcd939x_mbhc_zdet_ramp(struct snd_soc_component *component,
				 struct wcd939x_mbhc_zdet_param *zdet_param,
				 int32_t *zl, int32_t *zr, s16 *d1_a)
{
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);
	int32_t zdet = 0;

	snd_soc_component_update_bits(component, WCD939X_ZDET_ANA_CTL, 0xF0,
				      0x80 | (zdet_param->ldo_ctl << 4));
	snd_soc_component_update_bits(component, WCD939X_MBHC_BTN5, 0xFC,
			    zdet_param->btn5);
	snd_soc_component_update_bits(component, WCD939X_MBHC_BTN6, 0xFC,
			    zdet_param->btn6);
	snd_soc_component_update_bits(component, WCD939X_MBHC_BTN7, 0xFC,
			    zdet_param->btn7);
	snd_soc_component_update_bits(component, WCD939X_ZDET_ANA_CTL,
				0x0F, zdet_param->noff);
	snd_soc_component_update_bits(component, WCD939X_ZDET_RAMP_CTL,
				0x0F, zdet_param->nshift);
	snd_soc_component_update_bits(component, WCD939X_ZDET_RAMP_CTL,
				0x70, 0x60); /*acc1_min_63 */

	if (!zl)
		goto z_right;
	/* Start impedance measurement for HPH_L */
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_ZDET, 0x80, 0x80);
	dev_dbg(wcd939x->dev, "%s: ramp for HPH_L, noff = %d\n",
		__func__, zdet_param->noff);
	wcd939x_mbhc_get_result_params(wcd939x, d1_a, zdet_param->noff, &zdet);
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_ZDET, 0x80, 0x00);

	*zl = zdet;

z_right:
	if (!zr)
		return;
	/* Start impedance measurement for HPH_R */
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_ZDET, 0x40, 0x40);
	dev_dbg(wcd939x->dev, "%s: ramp for HPH_R, noff = %d\n",
		__func__, zdet_param->noff);
	wcd939x_mbhc_get_result_params(wcd939x, d1_a, zdet_param->noff, &zdet);
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_ZDET, 0x40, 0x00);

	*zr = zdet;
}

static inline void wcd939x_wcd_mbhc_qfuse_cal(
					struct snd_soc_component *component,
					int32_t *z_val, int flag_l_r)
{
	s16 q1;
	int q1_cal;

	q1 = snd_soc_component_read(component,
			WCD939X_EFUSE_REG_21 + flag_l_r);
	if (q1 & 0x80)
		q1_cal = (10000 - ((q1 & 0x7F) * 10));
	else
		q1_cal = (10000 + (q1 * 10));
	if (q1_cal > 0)
		*z_val = ((*z_val) * 10000) / q1_cal;
}

static void rdown_timer_callback(struct timer_list *timer)
{
	struct wcd939x_mbhc *wcd939x_mbhc = container_of(timer, struct wcd939x_mbhc, rdown_timer);

	wcd939x_mbhc->rdown_timer_complete = true;
}

static void update_hd2_codes(struct regmap *regmap, u32 r_gnd_res_tot_mohms, u32 r_load_eff)
{
	u64 hd2_delta = 0;

	if (!regmap)
		return;
	hd2_delta = (HD2_CODE_INV_RESOLUTION * (u64) r_gnd_res_tot_mohms +
		    FLOAT_TO_FIXED_XTALK * (u64) ((r_gnd_res_tot_mohms + r_load_eff) / 2)) /
		    (FLOAT_TO_FIXED_XTALK * (u64) (r_gnd_res_tot_mohms + r_load_eff));
	if (hd2_delta >= HD2_CODE_BASE_VALUE) {
		regmap_update_bits(regmap, WCD939X_RDAC_HD2_CTL_L, 0x1F, 0x00);
		regmap_update_bits(regmap, WCD939X_RDAC_HD2_CTL_R, 0x1F, 0x00);
	} else {
		regmap_update_bits(regmap, WCD939X_RDAC_HD2_CTL_L, 0x1F,
				   HD2_CODE_BASE_VALUE - hd2_delta);
		regmap_update_bits(regmap, WCD939X_RDAC_HD2_CTL_R, 0x1F,
				   HD2_CODE_BASE_VALUE - hd2_delta);
	}
}

static u8 get_xtalk_scale(u32 gain)
{
	u8 i;
	int target, residue;

	if (gain == 0)
		return MAX_XTALK_SCALE;

	target = FLOAT_TO_FIXED_XTALK / ((int) gain);
	residue = target;

	for (i = 0; i <= MAX_XTALK_SCALE; i++) {
		residue = target - (1 << ((int)((u32) i)));
		if (residue < 0)
			return i;
	}
	return MAX_XTALK_SCALE;
}

static u8 get_xtalk_alpha(u32 gain, u8 scale)
{
	u32 two_exp_scale, round_offset, alpha;

	if (gain == 0)
		return MIN_XTALK_ALPHA;

	two_exp_scale = 1 << ((u32) scale);
	round_offset = FLOAT_TO_FIXED_XTALK / 2;
	alpha = (((gain * two_exp_scale - FLOAT_TO_FIXED_XTALK) * 255) + round_offset)
		    / FLOAT_TO_FIXED_XTALK;
	return (alpha <= MAX_XTALK_ALPHA) ? ((u8) alpha) : MAX_XTALK_ALPHA;
}

static u32 get_v_common_gnd_factor(u32 r_gnd_res_tot_mohms, u32 r_load_eff_mohms,
				   u32 r_aud_res_tot_mohms)
{
	/* Proof 1: The numerator does not overflow.
	 * r_gnd_res_tot_mohms = r_gnd_int_fet_mohms + r_gnd_ext_fet_mohms + r_gnd_par_tot_mohms =
	 * r_gnd_int_fet_mohms + r_gnd_ext_fet_mohms + r_gnd_par_route1_mohms +
	 * r_gnd_par_route2_mohms
	 *
	 * r_gnd_int_fet_mohms, r_gnd_ext_fet_mohms, r_gnd_par_route{1,2}_mohms are all less
	 * than MAX_USBCSS_HS_IMPEDANCE_MOHMS
	 * -->
	 * FLOAT_TO_FIXED_XTALK * r_gnd_res_tot_mohms <=
	 * FLOAT_TO_FIXED_XTALK * 4 * MAX_USBCSS_HS_IMPEDANCE_MOHMS =
	 * (1 << 16) * 4 * 20,000 = 65,536 * 80,000 = 3,932,160,000 <= 2^32 - 1 =
	 * 4,294,967,295 = U32_MAX
	 *
	 * Proof 2: The denominator is greater than 0.
	 * r_load_eff_mohms >= MIN_RL_EFF_MOHMS = 1 > 0
	 * -->
	 * r_load_eff_mohms + r_aud_res_tot_mohms + r_gnd_res_tot_mohms > 0
	 *
	 * Proof 3: The deonominator does not overflow.
	 * r_load_eff_mohms <= MAX_RL_EFF_MOHMS
	 * r_aud_res_tot_mohms and r_gnd_res_tot_mohms <= MAX_USBCSS_HS_IMPEDANCE_MOHMS
	 * -->
	 * r_load_eff_mohms + r_aud_res_tot_mohms + r_gnd_res_tot_mohms <=
	 * MAX_RL_EFF_MOHMS + 2 * MAX_USBCSS_HS_IMPEDANCE_MOHMS = 900,000 + 2 * 20,000 = 940,000
	 * <= U32_MAX = 2^32 - 1 = 4,294,967,295
	 */
	return FLOAT_TO_FIXED_XTALK * r_gnd_res_tot_mohms /
	       (r_load_eff_mohms + r_aud_res_tot_mohms + r_gnd_res_tot_mohms);
}

static u32 get_v_feedback_tap_factor_digital(u32 r_gnd_int_fet_mohms, u32 r_gnd_par_route1_mohms,
					    u32 r_load_eff_mohms, u32 r_gnd_res_tot_mohms,
					    u32 r_aud_res_tot_mohms)
{
	/* Proof 4: The numerator does not overflow.
	 * r_gnd_int_fet_mohms and r_gnd_par_route1_mohms <= MAX_USBCSS_HS_IMPEDANCE_MOHMS
	 * -->
	 * FLOAT_TO_FIXED_XTALK * (r_gnd_int_fet_mohms + r_gnd_par_route1_mohms) <=
	 * FLOAT_TO_FIXED_XTALK * 2 * MAX_USBCSS_HS_IMPEDANCE_MOHMS =
	 * (1 << 16) * 2 * 20,000 = 65,536 * 40,000 = 2,621,440,000 <= 2^32 - 1 =
	 * 4,294,967,295 = U32_MAX
	 *
	 * The denominator is greater than 0: See Proof 2
	 * The deonominator does not overflow: See Proof 3
	 */
	return FLOAT_TO_FIXED_XTALK * (r_gnd_int_fet_mohms + r_gnd_par_route1_mohms) /
	       (r_load_eff_mohms + r_gnd_res_tot_mohms + r_aud_res_tot_mohms);
}

static u32 get_v_feedback_tap_factor_analog(u32 r_gnd_par_route2_mohms, u32 r_load_eff_mohms,
					    u32 r_gnd_res_tot_mohms, u32 r_aud_res_tot_mohms)
{
	/* Proof 5: The numerator does not overflow.
	 * r_gnd_res_tot_mohms = r_gnd_int_fet_mohms + r_gnd_ext_fet_mohms + r_gnd_par_tot_mohms =
	 * r_gnd_int_fet_mohms + r_gnd_ext_fet_mohms + r_gnd_par_route1_mohms +
	 * r_gnd_par_route2_mohms
	 *
	 *  r_gnd_res_tot_mohms - r_gnd_par_route2_mohms =
	 *  r_gnd_int_fet_mohms + r_gnd_ext_fet_mohms + r_gnd_par_route1_mohms
	 *
	 * r_gnd_int_fet_mohms, r_gnd_ext_fet_mohms, r_gnd_par_route1_mohms
	 * <= MAX_USBCSS_HS_IMPEDANCE_MOHMS = 20,000
	 * -->
	 * FLOAT_TO_FIXED_XTALK * (r_gnd_int_fet_mohms + r_gnd_ext_fet_mohms +
	 *			   r_gnd_par_route1_mohms)
	 * <= FLOAT_TO_FIXED_XTALK * 3 * MAX_USBCSS_HS_IMPEDANCE_MOHMS =
	 * (1 << 16) * 3 * 20,000 = 65,536 * 60,000 = 3,932,160,000 <= 2^32 - 1 =
	 * 4,294,967,295 = U32_MAX
	 *
	 *  The denominator is greater than 0: See Proof 2
	 *  The deonominator does not overflow: See Proof 3
	 */
	return FLOAT_TO_FIXED_XTALK * (r_gnd_res_tot_mohms - r_gnd_par_route2_mohms) /
	       (r_load_eff_mohms + r_gnd_res_tot_mohms + r_aud_res_tot_mohms);
}

static u32 get_xtalk_gain(u32 v_common_gnd_factor, u32 v_feedback_tap_factor)
{
	return v_common_gnd_factor - v_feedback_tap_factor;
}

static void update_xtalk_scale_and_alpha(struct wcd939x_pdata *pdata, struct regmap *regmap)
{
	u32 r_gnd_res_tot_mohms = 0, r_gnd_int_fet_mohms = 0, v_common_gnd_factor = 0;
	u32 v_feedback_tap_factor = 0, xtalk_gain = 0;

	if (!pdata || pdata->usbcss_hs.xtalk_config == XTALK_NONE)
		return;

	/* Orientation-dependent ground impedance parameters */
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU2_ORIENTATION_A) {
		r_gnd_res_tot_mohms = pdata->usbcss_hs.r_gnd_sbu2_res_tot_mohms;
		r_gnd_int_fet_mohms = pdata->usbcss_hs.r_gnd_sbu2_int_fet_mohms;
	} else if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU1_ORIENTATION_B) {
		r_gnd_res_tot_mohms = pdata->usbcss_hs.r_gnd_sbu1_res_tot_mohms;
		r_gnd_int_fet_mohms = pdata->usbcss_hs.r_gnd_sbu1_int_fet_mohms;
	} else {
		pdata->usbcss_hs.scale_l = MAX_XTALK_SCALE;
		pdata->usbcss_hs.alpha_l = MIN_XTALK_ALPHA;
		pdata->usbcss_hs.scale_r = MAX_XTALK_SCALE;
		pdata->usbcss_hs.alpha_r = MIN_XTALK_ALPHA;
		return;
	}
#endif

	/* Recall assumptions about L and R channel impedance parameters being equivalent */
	/* Xtalk gain calculation */
	v_common_gnd_factor = get_v_common_gnd_factor(r_gnd_res_tot_mohms,
						      pdata->usbcss_hs.r_load_eff_l_mohms,
						      pdata->usbcss_hs.r_aud_res_tot_l_mohms);
	if (pdata->usbcss_hs.xtalk_config == XTALK_ANALOG) {
		v_feedback_tap_factor = get_v_feedback_tap_factor_analog(
						pdata->usbcss_hs.r_gnd_par_route2_mohms,
						pdata->usbcss_hs.r_load_eff_l_mohms,
						r_gnd_res_tot_mohms,
						pdata->usbcss_hs.r_aud_res_tot_l_mohms);
		/* Update HD2 codes for analog xtalk */
		update_hd2_codes(regmap, r_gnd_res_tot_mohms, pdata->usbcss_hs.r_load_eff_l_mohms);
	} else {
		v_feedback_tap_factor = get_v_feedback_tap_factor_digital(
						r_gnd_int_fet_mohms,
						pdata->usbcss_hs.r_gnd_par_route1_mohms,
						pdata->usbcss_hs.r_load_eff_l_mohms,
						r_gnd_res_tot_mohms,
						pdata->usbcss_hs.r_aud_res_tot_l_mohms);
	}
	xtalk_gain = get_xtalk_gain(v_common_gnd_factor, v_feedback_tap_factor);
	/* Store scale and alpha values */
	pdata->usbcss_hs.scale_l = get_xtalk_scale(xtalk_gain);
	pdata->usbcss_hs.alpha_l = get_xtalk_alpha(xtalk_gain, pdata->usbcss_hs.scale_l);
	pdata->usbcss_hs.scale_r = pdata->usbcss_hs.scale_l;
	pdata->usbcss_hs.alpha_r = pdata->usbcss_hs.alpha_l;
}

static void update_ext_fet_res(struct wcd939x_pdata *pdata, u32 r_gnd_ext_fet_mohms)
{
	if (!pdata)
		return;

	pdata->usbcss_hs.r_gnd_ext_fet_mohms = (r_gnd_ext_fet_mohms > MAX_USBCSS_HS_IMPEDANCE_MOHMS)
					       ? MAX_USBCSS_HS_IMPEDANCE_MOHMS
					       : r_gnd_ext_fet_mohms;
	pdata->usbcss_hs.r_aud_ext_fet_l_mohms = pdata->usbcss_hs.r_gnd_ext_fet_mohms;
	pdata->usbcss_hs.r_aud_ext_fet_r_mohms = pdata->usbcss_hs.r_gnd_ext_fet_mohms;
	pdata->usbcss_hs.r_gnd_sbu1_res_tot_mohms = get_r_gnd_res_tot_mohms(
							pdata->usbcss_hs.r_gnd_sbu1_int_fet_mohms,
							pdata->usbcss_hs.r_gnd_ext_fet_mohms,
							pdata->usbcss_hs.r_gnd_par_tot_mohms);
	pdata->usbcss_hs.r_gnd_sbu2_res_tot_mohms = get_r_gnd_res_tot_mohms(
							pdata->usbcss_hs.r_gnd_sbu2_int_fet_mohms,
							pdata->usbcss_hs.r_gnd_ext_fet_mohms,
							pdata->usbcss_hs.r_gnd_par_tot_mohms);
	pdata->usbcss_hs.r_aud_res_tot_l_mohms = get_r_aud_res_tot_mohms(
							pdata->usbcss_hs.r_aud_int_fet_l_mohms,
							pdata->usbcss_hs.r_aud_ext_fet_l_mohms);
	pdata->usbcss_hs.r_aud_res_tot_r_mohms = get_r_aud_res_tot_mohms(
							pdata->usbcss_hs.r_aud_int_fet_r_mohms,
							pdata->usbcss_hs.r_aud_ext_fet_r_mohms);
}

static void get_linearizer_taps(struct wcd939x_pdata *pdata, u32 *aud_tap, u32 *gnd_tap)
{
	u32 r_gnd_res_tot_mohms = 0, r_gnd_int_fet_mohms = 0, v_aud1 = 0, v_aud2 = 0;
	u32 v_gnd_denom = 0, v_gnd1 = 0, v_gnd2 = 0, aud_denom = 0, gnd_denom = 0;

	if (!pdata)
		goto err_data;

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Orientation-dependent ground impedance parameters */
	if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU2_ORIENTATION_A) {
		r_gnd_res_tot_mohms = pdata->usbcss_hs.r_gnd_sbu2_res_tot_mohms;
		r_gnd_int_fet_mohms = pdata->usbcss_hs.r_gnd_sbu2_int_fet_mohms;
	} else if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU1_ORIENTATION_B) {
		r_gnd_res_tot_mohms = pdata->usbcss_hs.r_gnd_sbu1_res_tot_mohms;
		r_gnd_int_fet_mohms = pdata->usbcss_hs.r_gnd_sbu1_int_fet_mohms;
	} else {
		goto err_data;
	}
#endif

	/* Proof 6: Neither aud_denom nor gnd_denom is 0 and neither overflows.
	 * MIN_K_TIMES_100 = -50 <= MAX_K_TIMES_100 <= 10,000 = k_aud_times_100
	 * -->
	 * 0 < 410 = 0.1 * 4,096 = 0.1 * FLOAT_TO_FIXED_LINEARIZER < {aud,gnd}_denom <
	 * 101 * FLOAT_TO_FIXED_LINEARIZER =
	 * 101 * (1 << 12) < 413,696 <= 4,294,967,295 = U32_MAX
	 */
	aud_denom = (u32) (FLOAT_TO_FIXED_LINEARIZER +
			   (FLOAT_TO_FIXED_LINEARIZER * pdata->usbcss_hs.k_aud_times_100 / 100));
	gnd_denom = (u32) (FLOAT_TO_FIXED_LINEARIZER +
			   (FLOAT_TO_FIXED_LINEARIZER * pdata->usbcss_hs.k_gnd_times_100 / 100));

	/* Proof 7: v_aud2 does not overflow.
	 * MIN_RL_EFF_MOHMS = 1 = <= pdata->usbcss_hs.r_load_eff_l_mohms <= MAX_RL_EFF_MOHMS =
	 * 900,000
	 *
	 * pdata->usbcss_hs.r_gnd_par_tot_mohms = r_gnd_par_route1_mohms + r_gnd_par_route2_mohms
	 * <= 2 * MAX_USBCSS_HS_IMPEDANCE_MOHMS = 4,0000
	 *
	 * r_gnd_int_fet_mohms, pdata->usbcss_hs.r_gnd_ext_fet_mohms, r_gnd_par_route1_mohms,
	 * r_gnd_par_route2_mohms <= MAX_USBCSS_HS_IMPEDANCE_MOHMS = 20,000
	 * -->
	 * 1 <= v_aud2 <= MAX_RL_EFF_MOHMS + 4 * MAX_USBCSS_HS_IMPEDANCE_MOHMS =
	 * 900,000 + 4 * 20,000 = 980,000 <= 4,294,967,295 = U32_MAX
	 */
	v_aud2 = pdata->usbcss_hs.r_load_eff_l_mohms - pdata->usbcss_hs.r3 + r_gnd_int_fet_mohms +
		 pdata->usbcss_hs.r_gnd_ext_fet_mohms + pdata->usbcss_hs.r_gnd_par_tot_mohms;

	/* Proof 8: v_aud1 does not overflow.
	 * pdata->usbcss_hs.r_aud_ext_fet_l_mohms <= MAX_USBCSS_HS_IMPEDANCE_MOHMS = 20,000
	 * From Proof 7,
	 * 1 <= v_aud2 <= MAX_RL_EFF_MOHMS + 4 * MAX_USBCSS_HS_IMPEDANCE_MOHMS <= S32_MAX
	 * -->
	 * 1 <= v_aud1 <= MAX_RL_EFF_MOHMS + 5 * MAX_USBCSS_HS_IMPEDANCE_MOHMS =
	 * 900,000 + 5 * 20,000 = 1,000,000 <= 2,147,483,647 = S32_MAX
	 */
	v_aud1 = v_aud2 + pdata->usbcss_hs.r_aud_ext_fet_l_mohms;

	/* Proof 9: The numerator of v_aud1 does not overflow.
	 * From Proof 8, v_aud1 was less than or equal to 1,000,000
	 * Thus, the new v_aud1 numerator is less than or equal to
	 * FLOAT_TO_FIXED_LINEARIZER * 1,000,000 =
	 * 4,096 * 1,000,000 = 4,096,000,000 <= 4,294,967,295 = U32_MAX
	 *
	 * Proof 10: The denominator of v_aud1 is not 0.
	 * From Proof 8, v_aud1 was greater than or equal to 1 > 0
	 *
	 * Proof 11: The denominator does not overflow.
	 * From Proof 8, v_aud1 was less than or equal to 1,000,000
	 * Thus, the new v_aud1 denominator is less than or equal to
	 * 1,000,000 + pdata->usbcss_hs.r_aud_int_fet_l_mohms = 1,000,000 + 20,000 = 1,020,000 <=
	 * 4,294,967,295 = U32_MAX
	 */
	v_aud1 = FLOAT_TO_FIXED_LINEARIZER * v_aud1 /
		 (v_aud1 + pdata->usbcss_hs.r_aud_int_fet_l_mohms);

	/* Proof 12: The numerator of v_aud2 does not overflow.
	 * From Proof 7, v_aud2 was less than or equal to 980,000
	 * Thus, the new v_aud2 numerator is less than or equal to
	 * FLOAT_TO_FIXED_LINEARIZER * 980,000 =
	 * 4,096 * 980,000 = 4,014,080,000 <= 4,294,967,295 = U32_MAX
	 *
	 * Proof 13: The denominator of v_aud2 is not 0.
	 * From Proof 7, v_aud2 was greater than or equal to 1 > 0
	 *
	 * Proof 14: The denominator does not overflow.
	 * From Proof 7, v_aud2 was less than or equal to 980,000
	 * Thus, the new v_aud2 denominator is less than or equal to
	 * 980,000 + pdata->usbcss_hs.r_aud_int_fet_l_mohms pdata->usbcss_hs.r_aud_int_fet_l_mohms =
	 * 980,000 + 20,000 + + 20,000 = 1,020,000 <= 4,294,967,295 = U32_MAX
	 */
	v_aud2 = FLOAT_TO_FIXED_LINEARIZER * v_aud2 /
		 (v_aud2 + pdata->usbcss_hs.r_aud_ext_fet_l_mohms +
		  pdata->usbcss_hs.r_aud_int_fet_l_mohms);

	/* Proof 15: The numerator of aud_tap does not overflow.
	 * Looking at the formula for v_aud1 from Proofs 9 to 11, the greatest value of v_aud1 is
	 * FLOAT_TO_FIXED_LINEARIZER = 4,096
	 * Looking at the formula for v_aud2 from Proofs 12 to 14, the greatest value of v_aud2 is
	 * FLOAT_TO_FIXED_LINEARIZER = 4,096
	 * From Proof 6, aud_denom <= 413,696
	 * Thus, the numerator <= 1,000 * 4,096 + 10 * 10,000 * 4,096 + 413,696 / 2 =
	 * 4,096,000 + 409,600,000 + 206,848 = 413,902,848 <= 4,294,967,295 = U32_MAX
	 *
	 * Proof 16: The denominator of aud_tap is not 0.
	 * From Proof 6, aud_denom > 410 > 0
	 *
	 * Proof 17: The denominator of aud_tap does not overflow
	 * From Proof 6, aud_denom <= 413,696 <= 4,294,967,295 = U32_MAX
	 *
	 * Proof 18: The result of aud_tap does not overflow.
	 * From Proof 15, the numerator <= 413,902,848 and from Proof 16, the denominator > 410
	 * Thus, the divsion will be at most 1,009,519.
	 * pdata->usbcss_hs.aud_tap_offset <= MAX_TAP_OFFSET = 1,023
	 * The sum will thus be bounded by 1,009,519 + 1,023 = 1,010,542 <= 2,147,483,647 = S32_MAX
	 * Note: aud_tap won't underflow either since pdata->usbcss_hs.aud_tap_offset >= -1,023
	 */
	*aud_tap = (u32) ((s32) ((1000 * v_aud1 + 10 * pdata->usbcss_hs.k_aud_times_100 * v_aud2
				  + aud_denom / 2) / aud_denom) + pdata->usbcss_hs.aud_tap_offset);
	if (*aud_tap > MAX_TAP)
		*aud_tap = MAX_TAP;
	else if (*aud_tap < MIN_TAP)
		*aud_tap = MIN_TAP;

	/* Proof 19: v_gnd_denom does not overflow.
	 * r_gnd_res_tot_mohms = r_gnd_int_fet_mohms + r_gnd_ext_fet_mohms + r_gnd_par_tot_mohms
	 *
	 * r_gnd_int_fet_mohms, r_gnd_ext_fet_mohms, r_gnd_par_tot_mohms,
	 * pdata->usbcss_hs.r_aud_ext_fet_l_mohms, pdata->usbcss_hs.r_aud_int_fet_l_mohms are all
	 * <= MAX_USBCSS_HS_IMPEDANCE_MOHMS = 20,000
	 *
	 * pdata->usbcss_hs.r_load_eff_l_mohms <= MAX_RL_EFF_MOHMS = 900,000
	 *
	 * --> v_gnd_denom <= 3 * 20,000 + 900,000 + 2 * 20,000 = 60,000 + 900,000 + 40,000 =
	 * 1,000,000 <= 4,294,967,295 = U32_MAX
	 *
	 * Proof 20: v_gnd_denom is not 0.
	 * pdata->usbcss_hs.r_load_eff_l_mohms >= MIN_RL_EFF_MOHMS = 1
	 * -->  v_gnd_denom >= 1 > 0
	 */

	v_gnd_denom = (r_gnd_res_tot_mohms + pdata->usbcss_hs.r_load_eff_l_mohms -
		       pdata->usbcss_hs.r3 + pdata->usbcss_hs.r_aud_ext_fet_l_mohms +
		       pdata->usbcss_hs.r_aud_int_fet_l_mohms);

	/* Proof 21: v_gnd1 numerator does not overflow.
	 * r_gnd_int_fet_mohms <= MAX_USBCSS_HS_IMPEDANCE_MOHMS = 20,000
	 * --> v_gnd1 numerator <= 4,096 * 20,000 = 81,920,000 <= 4,294,967,295 = U32_MAX
	 *
	 * v_gnd1 denominator is not 0: See Proof 20
	 * v_gnd1 denominator does not overflow: See Proof 19
	 */
	v_gnd1 = FLOAT_TO_FIXED_LINEARIZER * r_gnd_int_fet_mohms / v_gnd_denom;

	/* Proof 22: v_gnd2 numerator does not overflow.
	 * r_gnd_int_fet_mohms <= MAX_USBCSS_HS_IMPEDANCE_MOHMS = 20,000
	 * pdata->usbcss_hs.r_load_eff_l_mohms <= MAX_RL_EFF_MOHMS = 900,000
	 * --> v_gnd2 numerator <= 4,096 * (20,000 + 900,000) = 4,096 * 920,000 = 3,768,320,000
	 * <= 4,294,967,295 = U32_MAX
	 *
	 * v_gnd2 denominator is not 0: See Proof 20
	 * v_gnd2 denominator does not overflow: See Proof 19
	 */
	v_gnd2 = FLOAT_TO_FIXED_LINEARIZER * (r_gnd_int_fet_mohms +
					      pdata->usbcss_hs.r_gnd_ext_fet_mohms) / v_gnd_denom;

	/* Proof 23: The numerator of gnd_tap does not overflow.
	 * Looking at the formula for v_gnd1 from Proof 21, and considering that
	 * r_gnd_res_tot_mohms = r_gnd_int_fet_mohms + r_gnd_ext_fet_mohms + r_gnd_par_tot_mohms,
	 * the greatest value of v_gnd1 is FLOAT_TO_FIXED_LINEARIZER = 4,096.
	 * Looking at the formula for v_aud2 from Proof 22 and again at the definintion of
	 * r_gnd_res_tot_mohms, the greatest value of v_gnd2 is FLOAT_TO_FIXED_LINEARIZER = 4,096
	 * From Proof 6, gnd_denom <= 413,696
	 * Thus, the numerator <= 1,000 * 4,096 + 10 * 10,000 * 4,096 + 413,696 / 2 =
	 * 4,096,000 + 409,600,000 + 206,848 = 413,902,848 <= 4,294,967,295 = U32_MAX
	 *
	 * Proof 24: The denominator of gnd_tap is not 0.
	 * From Proof 6, gnd_denom > 410 > 0
	 *
	 * Proof 25: The denominator of gnd_tap does not overflow
	 * From Proof 6, gnd_denom <= 413,696 <= 4,294,967,295 = U32_MAX
	 *
	 * Proof 26: The result of aud_tap does not overflow.
	 * From Proof 15, the numerator <= 413,902,848 and from Proof 16, the denominator > 410
	 * Thus, the divsion will be at most 1,009,519.
	 * pdata->usbcss_hs.aud_tap_offset <= MAX_TAP_OFFSET = 1,023
	 * The sum will thus be bounded by 1,009,519 + 1,023 = 1,010,542 <= 2,147,483,647 = S32_MAX
	 * Note: gnd_tap won't underflow either since pdata->usbcss_hs.aud_tap_offset >= -1,023
	 */
	*gnd_tap = (u32) ((s32) ((1000 * v_gnd1 + 10 * pdata->usbcss_hs.k_gnd_times_100 * v_gnd2
				  + gnd_denom / 2) / gnd_denom) + pdata->usbcss_hs.gnd_tap_offset);
	if (*gnd_tap > MAX_TAP)
		*gnd_tap = MAX_TAP;
	else if (*gnd_tap < MIN_TAP)
		*gnd_tap = MIN_TAP;
	return;

err_data:
	*aud_tap = 0;
	*gnd_tap = 0;
}

struct usbcss_hs_attr {
	struct wcd939x_priv *priv;
	struct kobj_attribute attr;
	int index;
};

static char *usbcss_sysfs_files[] = {
	"rdson",
	"r2",
	"r3",
	"r4",
	"r5",
	"r6",
	"r7",
	"lin-k-aud",
	"lin-k-gnd",
	"xtalk_config",
};

static ssize_t usbcss_sysfs_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf,
		size_t count)
{
	struct usbcss_hs_attr *usbc_attr;
	struct wcd939x_priv *wcd939x;
	struct wcd939x_pdata *pdata;
	struct wcd939x_usbcss_hs_params *usbcss_hs;
	long val;
	int rc;
	u32 aud_tap = 0, gnd_tap = 0;
	bool update_xtalk = false, update_linearizer = false;

	usbc_attr = container_of(attr, struct usbcss_hs_attr, attr);
	wcd939x = usbc_attr->priv;
	pdata = dev_get_platdata(wcd939x->dev);

	if (!wcd939x || !pdata)
		return -EINVAL;

	usbcss_hs = &pdata->usbcss_hs;

	rc = kstrtol(buf, 0, &val);
	if (rc)
		return rc;

	if (strcmp(attr->attr.name, "rdson") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->r_gnd_ext_fet_customer_mohms = val;
		update_linearizer = usbcss_hs->xtalk_config == XTALK_ANALOG;
	} else if (strcmp(attr->attr.name, "r2") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->r_conn_par_load_pos_mohms = val;
	} else if (strcmp(attr->attr.name, "r3") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->r3 = val;
		update_linearizer = true;
	} else if (strcmp(attr->attr.name, "r4") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->r4 = val;
		update_xtalk = true;
		update_linearizer = true;

		switch (usbcss_hs->xtalk_config) {
		case XTALK_DIGITAL:
			usbcss_hs->r_gnd_par_route2_mohms = usbcss_hs->r6 + val;
			break;
		case XTALK_ANALOG:
			usbcss_hs->r_gnd_par_route1_mohms = usbcss_hs->r5 + val;
			break;
		case XTALK_NONE:
			fallthrough;
		default:
			return count;
		}
	} else if (strcmp(attr->attr.name, "r5") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->r5 = val;

		switch (usbcss_hs->xtalk_config) {
		case XTALK_ANALOG:
			update_xtalk = true;
			update_linearizer = true;
			usbcss_hs->r_gnd_par_route1_mohms = val + usbcss_hs->r4;
			break;
		case XTALK_DIGITAL:
			fallthrough;
		case XTALK_NONE:
			fallthrough;
		default:
			return count;
		}
	} else if (strcmp(attr->attr.name, "r6") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->r6 = val;

		switch (usbcss_hs->xtalk_config) {
		case XTALK_DIGITAL:
			update_xtalk = true;
			update_linearizer = true;
			usbcss_hs->r_gnd_par_route2_mohms = val + usbcss_hs->r4;
			break;
		case XTALK_ANALOG:
			fallthrough;
		case XTALK_NONE:
			fallthrough;
		default:
			return count;
		}
	} else if (strcmp(attr->attr.name, "r7") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->r7 = val;

		switch (usbcss_hs->xtalk_config) {
		case XTALK_DIGITAL:
			update_xtalk = true;
			update_linearizer = true;
			usbcss_hs->r_gnd_par_route1_mohms = val;
			break;
		case XTALK_ANALOG:
			fallthrough;
		case XTALK_NONE:
			fallthrough;
		default:
			return count;
		}
	} else if (strcmp(attr->attr.name, "lin-k-aud") == 0) {
		if (val < MIN_K_TIMES_100 || val > MAX_K_TIMES_100) {
			dev_err(wcd939x->dev, "%s: Value %d out of bounds. Min: %d, Max: %d\n",
			__func__, val, MIN_K_TIMES_100, MAX_K_TIMES_100);
			return count;
		}
		usbcss_hs->k_aud_times_100 = val;
		update_linearizer = true;
	} else if (strcmp(attr->attr.name, "lin-k-gnd") == 0) {
		if (val < MIN_K_TIMES_100 || val > MAX_K_TIMES_100) {
			dev_err(wcd939x->dev, "%s: Value %d out of bounds. Min: %d, Max: %d\n",
			__func__, val, MIN_K_TIMES_100, MAX_K_TIMES_100);
			return count;
		}
		usbcss_hs->k_gnd_times_100 = val;
		update_linearizer = true;
	} else if (strcmp(attr->attr.name, "xtalk_config") == 0) {
		pdata->usbcss_hs.xtalk_config = val;
		update_xtalk = true;

		switch (val) {
		case XTALK_NONE:
			usbcss_hs->scale_l = MAX_XTALK_SCALE;
			usbcss_hs->scale_r = MAX_XTALK_SCALE;
			usbcss_hs->alpha_l = MIN_XTALK_ALPHA;
			usbcss_hs->alpha_r = MIN_XTALK_ALPHA;
			break;
		case XTALK_DIGITAL:
			usbcss_hs->r_gnd_par_route2_mohms = usbcss_hs->r6 + usbcss_hs->r4;
			usbcss_hs->r_gnd_par_route1_mohms = usbcss_hs->r7;
			update_linearizer = true;
			break;
		case XTALK_ANALOG:
			usbcss_hs->r_gnd_par_route1_mohms = usbcss_hs->r5 + usbcss_hs->r4;
			usbcss_hs->r_gnd_par_route2_mohms = 1;
			update_linearizer = true;
			break;
		default:
			return count;
		}
	}

	if (update_xtalk) {
		update_xtalk_scale_and_alpha(pdata, wcd939x->regmap);
		regmap_update_bits(wcd939x->regmap, WCD939X_HPHL_RX_PATH_SEC0,
				0x1F, pdata->usbcss_hs.scale_l);
		regmap_update_bits(wcd939x->regmap, WCD939X_HPHL_RX_PATH_SEC1,
				0xFF, pdata->usbcss_hs.alpha_l);
		regmap_update_bits(wcd939x->regmap, WCD939X_HPHL_RX_PATH_SEC0 + 1,
				0x1F, pdata->usbcss_hs.scale_r);
		regmap_update_bits(wcd939x->regmap, WCD939X_HPHL_RX_PATH_SEC1 + 1,
				0xFF, pdata->usbcss_hs.alpha_r);
		dev_err(wcd939x->dev, "%s: Updated xtalk thru sysfs\n",
			__func__);
	}

	if (update_linearizer) {
		get_linearizer_taps(pdata, &aud_tap, &gnd_tap);
		wcd_usbss_set_linearizer_sw_tap(aud_tap, gnd_tap);
		dev_err(wcd939x->dev, "%s: Updated linearizer thru sysfs\n",
			__func__);
	}

	return count;
}

static ssize_t usbcss_sysfs_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct usbcss_hs_attr *usbc_attr;
	struct wcd939x_priv *wcd939x;
	struct wcd939x_pdata *pdata;

	usbc_attr = container_of(attr, struct usbcss_hs_attr, attr);
	wcd939x = usbc_attr->priv;
	pdata = dev_get_platdata(wcd939x->dev);

	if (strcmp(attr->attr.name, "rdson") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.r_gnd_ext_fet_customer_mohms);
	else if (strcmp(attr->attr.name, "r2") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.r_conn_par_load_pos_mohms);
	else if (strcmp(attr->attr.name, "r3") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.r3);
	else if (strcmp(attr->attr.name, "r4") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.r4);
	else if (strcmp(attr->attr.name, "r5") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.r5);
	else if (strcmp(attr->attr.name, "r6") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.r6);
	else if (strcmp(attr->attr.name, "r7") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.r7);
	else if (strcmp(attr->attr.name, "lin-k-aud") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.k_aud_times_100);
	else if (strcmp(attr->attr.name, "lin-k-gnd") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.k_gnd_times_100);
	else if (strcmp(attr->attr.name, "xtalk_config") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.xtalk_config);

	return 0;
}

static int create_sysfs_entry_file(struct wcd939x_priv *wcd939x, char *name, int mode,
		int index, struct kobject *parent)
{
	struct usbcss_hs_attr *usbc_attr;
	char *name_copy;

	usbc_attr = devm_kmalloc(wcd939x->dev, sizeof(*usbc_attr), GFP_KERNEL);
	if (!usbc_attr)
		return -ENOMEM;

	name_copy = devm_kstrdup(wcd939x->dev, name, GFP_KERNEL);
	if (!name_copy)
		return -ENOMEM;

	usbc_attr->priv = wcd939x;
	usbc_attr->index = index;
	usbc_attr->attr.attr.name = name_copy;
	usbc_attr->attr.attr.mode = mode;
	usbc_attr->attr.show = usbcss_sysfs_show;
	usbc_attr->attr.store = usbcss_sysfs_store;
	sysfs_attr_init(&usbc_attr->attr.attr);

	return sysfs_create_file(parent, &usbc_attr->attr.attr);
}

static int usbcss_hs_sysfs_init(struct wcd939x_priv *wcd939x)
{
	int rc = 0;
	int i = 0;
	struct kobject *kobj = NULL;

	if (!wcd939x || !wcd939x->dev) {
		pr_err("%s: Invalid wcd939x private data.\n", __func__);
		return -EINVAL;
	}

	kobj = kobject_create_and_add("usbcss_hs", kernel_kobj);
	if (!kobj) {
		dev_err(wcd939x->dev, "%s: Could not create the USBC-SS HS kobj.\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(usbcss_sysfs_files); i++) {
		rc = create_sysfs_entry_file(wcd939x, usbcss_sysfs_files[i],
				0644, i, kobj);
	}

	return 0;
}

static void wcd939x_wcd_mbhc_calc_impedance(struct wcd_mbhc *mbhc, uint32_t *zl, uint32_t *zr)
{
	struct snd_soc_component *component = mbhc->component;
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);
	struct wcd939x_pdata *pdata = dev_get_platdata(wcd939x->dev);
	s16 reg0, reg1, reg2, reg3, reg4;
	uint32_t zdiff_val = 0, r_gnd_int_fet_mohms = 0, rl_eff_mohms = 0, r_gnd_ext_fet_mohms = 0;
	uint32_t aud_tap = 0, gnd_tap = 0;
	uint32_t *zdiff = &zdiff_val;
	int32_t z1L, z1R, z1Ls, z1Diff;
	int zMono, z_diff1, z_diff2;
	bool is_fsm_disable = false;
	struct wcd939x_mbhc_zdet_param zdet_param = {4, 0, 6, 0x18, 0x60, 0x78};
	struct wcd939x_mbhc_zdet_param *zdet_param_ptr = &zdet_param;
	s16 d1[] = {0, 30, 30, 6};
	uint32_t cached_regs[4][2] = {{WCD_USBSS_EXT_LIN_EN, 0}, {WCD_USBSS_EXT_SW_CTRL_1, 0},
				      {WCD_USBSS_MG1_BIAS, 0}, {WCD_USBSS_MG2_BIAS, 0}};
	uint32_t l_3_6V_regs[4][2] = {{WCD_USBSS_EXT_LIN_EN, 0x00}, {WCD_USBSS_EXT_SW_CTRL_1, 0x00},
				      {WCD_USBSS_MG1_BIAS, 0x0E}, {WCD_USBSS_MG2_BIAS, 0x0E}};
	uint32_t diff_regs[2][2] = {{WCD_USBSS_EXT_LIN_EN, 0x00}, {WCD_USBSS_EXT_SW_CTRL_1, 0xE8}};

	WCD_MBHC_RSC_ASSERT_LOCKED(mbhc);

	/* Turn on RX supplies */
	if (wcd939x->version == WCD939X_VERSION_2_0) {
		/* Start up Buck/Flyback, Enable RX bias, Use MBHC RCO for MBHC Zdet, Enable Vneg */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x4C, 0x4C);
		/* Wait 100us for settling */
		usleep_range(100, 110);
		/* Enable VNEGDAC_LDO */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x10, 0x10);
		/* Wait 100us for settling */
		usleep_range(100, 110);
		/* Keep PA left/right channels disabled */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x01, 0x01);
		/* Enable VPOS */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x20, 0x20);
		/* Wait 500us for settling */
		usleep_range(500, 510);
	}

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Cache relevant USB-SS registers */
	wcd_usbss_register_update(cached_regs, WCD_USBSS_READ, ARRAY_SIZE(cached_regs));
#endif

	/* Store register values */
	reg0 = snd_soc_component_read(component, WCD939X_MBHC_BTN5);
	reg1 = snd_soc_component_read(component, WCD939X_MBHC_BTN6);
	reg2 = snd_soc_component_read(component, WCD939X_MBHC_BTN7);
	reg3 = snd_soc_component_read(component, WCD939X_CTL_CLK);
	reg4 = snd_soc_component_read(component, WCD939X_ZDET_ANA_CTL);

	/* Disable the detection FSM */
	if (snd_soc_component_read(component, WCD939X_MBHC_ELECT) & 0x80) {
		is_fsm_disable = true;
		regmap_update_bits(wcd939x->regmap,
				   WCD939X_MBHC_ELECT, 0x80, 0x00);
	}

	/* For NO-jack, disable L_DET_EN before Z-det measurements */
	if (mbhc->hphl_swh)
		regmap_update_bits(wcd939x->regmap,
				   WCD939X_MBHC_MECH, 0x80, 0x00);

	/* Turn off 100k pull down on HPHL */
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_MECH, 0x01, 0x00);

	/* Disable surge protection before impedance detection.
	 * This is done to give correct value for high impedance.
	 */
	regmap_update_bits(wcd939x->regmap,
                          WCD939X_HPHLR_SURGE_EN, 0xC0, 0x00);
	/* 1ms delay needed after disable surge protection */
	usleep_range(1000, 1010);

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Disable sense switch and MIC for USB-C analog platforms */
	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		wcd_usbss_set_switch_settings_enable(SENSE_SWITCHES, USBSS_SWITCH_DISABLE);
		wcd_usbss_set_switch_settings_enable(MIC_SWITCHES, USBSS_SWITCH_DISABLE);
	}
#endif

	/* L-channel impedance */
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	wcd_usbss_register_update(l_3_6V_regs, WCD_USBSS_WRITE, ARRAY_SIZE(l_3_6V_regs));
#endif
	wcd939x_mbhc_zdet_ramp(component, zdet_param_ptr, &z1L, NULL, d1);
	if ((z1L == WCD939X_ZDET_FLOATING_IMPEDANCE) || (z1L > WCD939X_ZDET_VAL_100K)) {
		*zl = WCD939X_ZDET_FLOATING_IMPEDANCE;
	} else {
		*zl = z1L;
		wcd939x_wcd_mbhc_qfuse_cal(component, zl, 0);
	}
	/* Differential measurement for USB-C analog platforms */
	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		dev_dbg(component->dev, "%s: effective impedance on HPH_L = %d(mohms)\n",
			__func__, *zl);
		goto diff_impedance;
	}
	dev_dbg(component->dev, "%s: impedance on HPH_L = %d(mohms)\n",
		__func__, *zl);

	/* R-channel impedance */
	wcd939x_mbhc_zdet_ramp(component, zdet_param_ptr, NULL, &z1R, d1);
	if ((z1R == WCD939X_ZDET_FLOATING_IMPEDANCE) || (z1R > WCD939X_ZDET_VAL_100K)) {
		*zr = WCD939X_ZDET_FLOATING_IMPEDANCE;
	} else {
		*zr = z1R;
		wcd939x_wcd_mbhc_qfuse_cal(component, zr, 4);
	}
	dev_dbg(component->dev, "%s: impedance on HPH_R = %d(mohms)\n",
		__func__, *zr);
		/* Convert from mohms to ohms (rounded) */
	*zl = (*zl + OHMS_TO_MILLIOHMS / 2) / OHMS_TO_MILLIOHMS;
	*zr = (*zr + OHMS_TO_MILLIOHMS / 2) / OHMS_TO_MILLIOHMS;
	goto mono_stereo_detection;

diff_impedance:
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Disable AGND switch */
	wcd_usbss_set_switch_settings_enable(AGND_SWITCHES, USBSS_SWITCH_DISABLE);
	wcd_usbss_register_update(diff_regs, WCD_USBSS_WRITE, ARRAY_SIZE(diff_regs));
#endif
	/* Enable HPHR NCLAMP */
	regmap_update_bits(wcd939x->regmap, WCD939X_HPHLR_SURGE_MISC1, 0x08, 0x08);

	/* Diffrential impedance */
	wcd939x_mbhc_zdet_ramp(component, zdet_param_ptr, &z1Diff, NULL, d1);
	if ((z1Diff == WCD939X_ZDET_FLOATING_IMPEDANCE) || (z1Diff > WCD939X_ZDET_VAL_100K)) {
		*zdiff = WCD939X_ZDET_FLOATING_IMPEDANCE;
	} else {
		*zdiff = z1Diff;
		wcd939x_wcd_mbhc_qfuse_cal(component, zdiff, 0);
	}
	dev_dbg(component->dev, "%s: effective impedance on HPH_diff after calib = %d(mohms)\n",
		__func__, *zdiff);
	/* Disable HPHR NCLAMP */
	regmap_update_bits(wcd939x->regmap, WCD939X_HPHLR_SURGE_MISC1, 0x08, 0x00);
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Enable AGND switch */
	wcd_usbss_set_switch_settings_enable(AGND_SWITCHES, USBSS_SWITCH_ENABLE);
	/* Get ground internal resistance based on orientation */
	if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU2_ORIENTATION_A) {
		r_gnd_int_fet_mohms = pdata->usbcss_hs.r_gnd_sbu2_int_fet_mohms;
	} else if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU1_ORIENTATION_B) {
		r_gnd_int_fet_mohms = pdata->usbcss_hs.r_gnd_sbu1_int_fet_mohms;
	} else {
		*zl = 0;
		*zr = 0;
		dev_dbg(component->dev, "%s: Invalid SBU switch orientation\n", __func__);
		goto zdet_complete;
	}
#endif
	/* Compute external fet and effective load impedance */
	r_gnd_ext_fet_mohms = *zl - *zdiff / 2 + pdata->usbcss_hs.r_surge_mohms / 2 -
			      pdata->usbcss_hs.r_gnd_par_tot_mohms - r_gnd_int_fet_mohms;
	rl_eff_mohms = *zdiff / 2 - pdata->usbcss_hs.r_aud_int_fet_r_mohms -
		       pdata->usbcss_hs.r_gnd_ext_fet_mohms - pdata->usbcss_hs.r_surge_mohms / 2 -
		       pdata->usbcss_hs.r_gnd_par_tot_mohms;
	/* Store values */
	*zl = (rl_eff_mohms - pdata->usbcss_hs.r_conn_par_load_pos_mohms - pdata->usbcss_hs.r3 +
		   OHMS_TO_MILLIOHMS / 2) / OHMS_TO_MILLIOHMS;
	*zr = *zl;

	/* Update USBC-SS HS params */
	if (rl_eff_mohms > MAX_RL_EFF_MOHMS)
		rl_eff_mohms = MAX_RL_EFF_MOHMS;
	else if (rl_eff_mohms == 0)
		rl_eff_mohms = MIN_RL_EFF_MOHMS;
	pdata->usbcss_hs.r_load_eff_l_mohms = rl_eff_mohms;
	pdata->usbcss_hs.r_load_eff_r_mohms = rl_eff_mohms;
	update_ext_fet_res(pdata, r_gnd_ext_fet_mohms);
	update_xtalk_scale_and_alpha(pdata, wcd939x->regmap);
	dev_dbg(component->dev, "%s: Xtalk scale is 0x%x and alpha is 0x%x\n",
		__func__, pdata->usbcss_hs.scale_l, pdata->usbcss_hs.alpha_l);
	get_linearizer_taps(pdata, &aud_tap, &gnd_tap);
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	wcd_usbss_set_linearizer_sw_tap(aud_tap, gnd_tap);
#endif
	dev_dbg(component->dev, "%s: Linearizer aud_tap is 0x%x and gnd_tap is 0x%x\n",
		__func__, aud_tap, gnd_tap);

mono_stereo_detection:
	/* Mono/stereo detection */
	if ((*zl == WCD939X_ZDET_FLOATING_IMPEDANCE) && (*zr == WCD939X_ZDET_FLOATING_IMPEDANCE)) {
		dev_dbg(component->dev,
			"%s: plug type is invalid or extension cable\n",
			__func__);
		goto zdet_complete;
	}
	if ((*zl == WCD939X_ZDET_FLOATING_IMPEDANCE) ||
	    (*zr == WCD939X_ZDET_FLOATING_IMPEDANCE) ||
	    ((*zl < WCD_MONO_HS_MIN_THR) && (*zr > WCD_MONO_HS_MIN_THR)) ||
	    ((*zl > WCD_MONO_HS_MIN_THR) && (*zr < WCD_MONO_HS_MIN_THR))) {
		dev_dbg(component->dev,
			"%s: Mono plug type with one ch floating or shorted to GND\n",
			__func__);
		mbhc->hph_type = WCD_MBHC_HPH_MONO;
		goto zdet_complete;
	}
	snd_soc_component_update_bits(component, WCD939X_R_ATEST, 0x02, 0x02);
	snd_soc_component_update_bits(component, WCD939X_PA_CTL2, 0x40, 0x01);
	wcd939x_mbhc_zdet_ramp(component, zdet_param_ptr, &z1Ls, NULL, d1);
	snd_soc_component_update_bits(component, WCD939X_PA_CTL2, 0x40, 0x00);
	snd_soc_component_update_bits(component, WCD939X_R_ATEST, 0x02, 0x00);
	z1Ls /= 1000;
	wcd939x_wcd_mbhc_qfuse_cal(component, &z1Ls, 0);
	/* Parallel of left Z and 9 ohm pull down resistor */
	zMono = ((*zl) * 9) / ((*zl) + 9);
	z_diff1 = (z1Ls > zMono) ? (z1Ls - zMono) : (zMono - z1Ls);
	z_diff2 = ((*zl) > z1Ls) ? ((*zl) - z1Ls) : (z1Ls - (*zl));
	if ((z_diff1 * (*zl + z1Ls)) > (z_diff2 * (z1Ls + zMono))) {
		dev_dbg(component->dev, "%s: stereo plug type detected\n",
			__func__);
		mbhc->hph_type = WCD_MBHC_HPH_STEREO;
	} else {
		dev_dbg(component->dev, "%s: MONO plug type detected\n",
			__func__);
		mbhc->hph_type = WCD_MBHC_HPH_MONO;
	}

zdet_complete:
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Enable sense switch and MIC for USB-C analog platforms */
	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		wcd_usbss_set_switch_settings_enable(SENSE_SWITCHES, USBSS_SWITCH_ENABLE);
		wcd_usbss_set_switch_settings_enable(MIC_SWITCHES, USBSS_SWITCH_ENABLE);
	}
#endif
	/* Enable surge protection again after impedance detection */
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_HPHLR_SURGE_EN, 0xC0, 0xC0);

	snd_soc_component_write(component, WCD939X_MBHC_BTN5, reg0);
	snd_soc_component_write(component, WCD939X_MBHC_BTN6, reg1);
	snd_soc_component_write(component, WCD939X_MBHC_BTN7, reg2);
	/* Turn on 100k pull down on HPHL */
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_MECH, 0x01, 0x01);

	/* For NO-jack, re-enable L_DET_EN after Z-det measurements */
	if (mbhc->hphl_swh)
		regmap_update_bits(wcd939x->regmap,
				   WCD939X_MBHC_MECH, 0x80, 0x80);

	snd_soc_component_write(component, WCD939X_ZDET_ANA_CTL, reg4);
	snd_soc_component_write(component, WCD939X_CTL_CLK, reg3);
	if (is_fsm_disable)
		regmap_update_bits(wcd939x->regmap,
				   WCD939X_MBHC_ELECT, 0x80, 0x80);

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	wcd_usbss_register_update(cached_regs, WCD_USBSS_WRITE, ARRAY_SIZE(cached_regs));
#endif

	/* Turn off RX supplies */
	if (wcd939x->version == WCD939X_VERSION_2_0) {
		/* Set VPOS to be controlled by RX */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x20, 0x00);
		/* Wait 500us for settling */
		usleep_range(500, 510);
		/* Set PA Left/Right channels and VNEGDAC_LDO to be controlled by RX */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x11, 0x00);
		/* Wait 100us for settling */
		usleep_range(100, 110);
		/* Set Vneg mode and enable to be controlled by RX */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x06, 0x00);
		/* Wait 100us for settling */
		usleep_range(100, 110);
		/* Set RX bias to be controlled by RX and set Buck/Flyback back to SWR Rx clock */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x48, 0x00);
	}
}

static void wcd939x_mbhc_gnd_det_ctrl(struct snd_soc_component *component,
			bool enable)
{
	if (enable) {
		snd_soc_component_update_bits(component, WCD939X_MBHC_MECH,
				    0x02, 0x02);
		snd_soc_component_update_bits(component, WCD939X_MBHC_MECH,
				    0x40, 0x40);
	} else {
		snd_soc_component_update_bits(component, WCD939X_MBHC_MECH,
				    0x40, 0x00);
		snd_soc_component_update_bits(component, WCD939X_MBHC_MECH,
				    0x02, 0x00);
	}
}

static void wcd939x_mbhc_hph_pull_down_ctrl(struct snd_soc_component *component,
					  bool enable)
{
	if (enable) {
		snd_soc_component_update_bits(component, WCD939X_PA_CTL2,
				    0x40, 0x40);
		snd_soc_component_update_bits(component, WCD939X_PA_CTL2,
				    0x10, 0x10);
	} else {
		snd_soc_component_update_bits(component, WCD939X_PA_CTL2,
				    0x40, 0x00);
		snd_soc_component_update_bits(component, WCD939X_PA_CTL2,
				    0x10, 0x00);
	}
}

static void wcd939x_mbhc_moisture_config(struct wcd_mbhc *mbhc)
{
	struct snd_soc_component *component = mbhc->component;

	if ((mbhc->moist_rref == R_OFF) ||
	    (mbhc->mbhc_cfg->enable_usbc_analog)) {
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
				    0x0C, R_OFF << 2);
		return;
	}

	/* Do not enable moisture detection if jack type is NC */
	if (!mbhc->hphl_swh) {
		dev_dbg(component->dev, "%s: disable moisture detection for NC\n",
			__func__);
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
				    0x0C, R_OFF << 2);
		return;
	}

	snd_soc_component_update_bits(component, WCD939X_CTL_2,
			    0x0C, mbhc->moist_rref << 2);
}

static void wcd939x_mbhc_moisture_detect_en(struct wcd_mbhc *mbhc, bool enable)
{
	struct snd_soc_component *component = mbhc->component;

	if (enable)
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
					0x0C, mbhc->moist_rref << 2);
	else
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
				    0x0C, R_OFF << 2);
}

static bool wcd939x_mbhc_get_moisture_status(struct wcd_mbhc *mbhc)
{
	struct snd_soc_component *component = mbhc->component;
	bool ret = false;

	if ((mbhc->moist_rref == R_OFF) ||
	    (mbhc->mbhc_cfg->enable_usbc_analog)) {
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
				    0x0C, R_OFF << 2);
		goto done;
	}

	/* Do not enable moisture detection if jack type is NC */
	if (!mbhc->hphl_swh) {
		dev_dbg(component->dev, "%s: disable moisture detection for NC\n",
			__func__);
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
				    0x0C, R_OFF << 2);
		goto done;
	}

	/*
	 * If moisture_en is already enabled, then skip to plug type
	 * detection.
	 */
	if ((snd_soc_component_read(component, WCD939X_CTL_2) & 0x0C))
		goto done;

	wcd939x_mbhc_moisture_detect_en(mbhc, true);
	/* Read moisture comparator status */
	ret = ((snd_soc_component_read(component, WCD939X_FSM_STATUS)
				& 0x20) ? 0 : 1);

done:
	return ret;

}

static void wcd939x_mbhc_moisture_polling_ctrl(struct wcd_mbhc *mbhc,
						bool enable)
{
	struct snd_soc_component *component = mbhc->component;

	snd_soc_component_update_bits(component,
			WCD939X_MOISTURE_DET_POLLING_CTRL,
			0x04, (enable << 2));
}

static void wcd939x_mbhc_bcs_enable(struct wcd_mbhc *mbhc,
						  bool bcs_enable)
{
	if (bcs_enable)
		wcd939x_disable_bcs_before_slow_insert(mbhc->component, false);
	else
		wcd939x_disable_bcs_before_slow_insert(mbhc->component, true);
}

static void wcd939x_surge_reset_routine(struct wcd_mbhc *mbhc)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(mbhc->component);

	regcache_mark_dirty(wcd939x->regmap);
	regcache_sync(wcd939x->regmap);
}

static void wcd939x_mbhc_zdet_leakage_resistance(struct wcd_mbhc *mbhc,
							bool enable)
{
	if (enable)
		snd_soc_component_update_bits(mbhc->component, WCD939X_ZDET_BIAS_CTL,
				0x80, 0x80); /* disable 1M pull-up */
	else
		snd_soc_component_update_bits(mbhc->component, WCD939X_ZDET_BIAS_CTL,
				0x80, 0x00); /* enable 1M pull-up */
}

static const struct wcd_mbhc_cb mbhc_cb = {
	.request_irq = wcd939x_mbhc_request_irq,
	.irq_control = wcd939x_mbhc_irq_control,
	.free_irq = wcd939x_mbhc_free_irq,
	.clk_setup = wcd939x_mbhc_clk_setup,
	.map_btn_code_to_num = wcd939x_mbhc_btn_to_num,
	.mbhc_bias = wcd939x_mbhc_mbhc_bias_control,
	.set_btn_thr = wcd939x_mbhc_program_btn_thr,
	.lock_sleep = wcd939x_mbhc_lock_sleep,
	.register_notifier = wcd939x_mbhc_register_notifier,
	.micbias_enable_status = wcd939x_mbhc_micb_en_status,
	.hph_pa_on_status = wcd939x_mbhc_hph_pa_on_status,
	.hph_pull_up_control_v2 = wcd939x_mbhc_hph_l_pull_up_control,
	.mbhc_micbias_control = wcd939x_mbhc_request_micbias,
	.mbhc_micb_ramp_control = wcd939x_mbhc_micb_ramp_control,
	.get_hwdep_fw_cal = wcd939x_get_hwdep_fw_cal,
	.mbhc_micb_ctrl_thr_mic = wcd939x_mbhc_micb_ctrl_threshold_mic,
	.compute_impedance = wcd939x_wcd_mbhc_calc_impedance,
	.mbhc_gnd_det_ctrl = wcd939x_mbhc_gnd_det_ctrl,
	.hph_pull_down_ctrl = wcd939x_mbhc_hph_pull_down_ctrl,
	.mbhc_moisture_config = wcd939x_mbhc_moisture_config,
	.mbhc_get_moisture_status = wcd939x_mbhc_get_moisture_status,
	.mbhc_moisture_polling_ctrl = wcd939x_mbhc_moisture_polling_ctrl,
	.mbhc_moisture_detect_en = wcd939x_mbhc_moisture_detect_en,
	.bcs_enable = wcd939x_mbhc_bcs_enable,
	.surge_reset_routine = wcd939x_surge_reset_routine,
	.zdet_leakage_resistance = wcd939x_mbhc_zdet_leakage_resistance,
};

static int wcd939x_get_hph_type(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
					snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_mbhc *wcd939x_mbhc = wcd939x_soc_get_mbhc(component);
	struct wcd_mbhc *mbhc;

	if (!wcd939x_mbhc) {
		dev_err_ratelimited(component->dev, "%s: mbhc not initialized!\n", __func__);
		return -EINVAL;
	}

	mbhc = &wcd939x_mbhc->wcd_mbhc;

	ucontrol->value.integer.value[0] = (u32) mbhc->hph_type;
	dev_dbg(component->dev, "%s: hph_type = %u\n", __func__, mbhc->hph_type);

	return 0;
}

static int wcd939x_hph_impedance_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	uint32_t zl, zr;
	bool hphr;
	struct soc_multi_mixer_control *mc;
	struct snd_soc_component *component =
					snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_mbhc *wcd939x_mbhc = wcd939x_soc_get_mbhc(component);

	if (!wcd939x_mbhc) {
		dev_err_ratelimited(component->dev, "%s: mbhc not initialized!\n", __func__);
		return -EINVAL;
	}

	mc = (struct soc_multi_mixer_control *)(kcontrol->private_value);
	hphr = mc->shift;
	wcd_mbhc_get_impedance(&wcd939x_mbhc->wcd_mbhc, &zl, &zr);
	dev_dbg(component->dev, "%s: zl=%u(ohms), zr=%u(ohms)\n", __func__, zl, zr);
	ucontrol->value.integer.value[0] = hphr ? zr : zl;

	return 0;
}

static const struct snd_kcontrol_new hph_type_detect_controls[] = {
	SOC_SINGLE_EXT("HPH Type", 0, 0, UINT_MAX, 0,
		       wcd939x_get_hph_type, NULL),
};

static const struct snd_kcontrol_new impedance_detect_controls[] = {
	SOC_SINGLE_EXT("HPHL Impedance", 0, 0, UINT_MAX, 0,
		       wcd939x_hph_impedance_get, NULL),
	SOC_SINGLE_EXT("HPHR Impedance", 0, 1, UINT_MAX, 0,
		       wcd939x_hph_impedance_get, NULL),
};

/*
 * wcd939x_mbhc_get_impedance: get impedance of headphone
 * left and right channels
 * @wcd939x_mbhc: handle to struct wcd939x_mbhc *
 * @zl: handle to left-ch impedance
 * @zr: handle to right-ch impedance
 * return 0 for success or error code in case of failure
 */
int wcd939x_mbhc_get_impedance(struct wcd939x_mbhc *wcd939x_mbhc,
			     uint32_t *zl, uint32_t *zr)
{
	if (!wcd939x_mbhc) {
		pr_err_ratelimited("%s: mbhc not initialized!\n", __func__);
		return -EINVAL;
	}
	if (!zl || !zr) {
		pr_err_ratelimited("%s: zl or zr null!\n", __func__);
		return -EINVAL;
	}

	return wcd_mbhc_get_impedance(&wcd939x_mbhc->wcd_mbhc, zl, zr);
}
EXPORT_SYMBOL(wcd939x_mbhc_get_impedance);

/*
 * wcd939x_mbhc_hs_detect: starts mbhc insertion/removal functionality
 * @codec: handle to snd_soc_component *
 * @mbhc_cfg: handle to mbhc configuration structure
 * return 0 if mbhc_start is success or error code in case of failure
 */
int wcd939x_mbhc_hs_detect(struct snd_soc_component *component,
			 struct wcd_mbhc_config *mbhc_cfg)
{
	struct wcd939x_priv *wcd939x = NULL;
	struct wcd939x_mbhc *wcd939x_mbhc = NULL;

	if (!component) {
		pr_err_ratelimited("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	wcd939x = snd_soc_component_get_drvdata(component);
	if (!wcd939x) {
		pr_err_ratelimited("%s: wcd939x is NULL\n", __func__);
		return -EINVAL;
	}

	wcd939x_mbhc = wcd939x->mbhc;
	if (!wcd939x_mbhc) {
		dev_err_ratelimited(component->dev, "%s: mbhc not initialized!\n", __func__);
		return -EINVAL;
	}

	return wcd_mbhc_start(&wcd939x_mbhc->wcd_mbhc, mbhc_cfg);
}
EXPORT_SYMBOL(wcd939x_mbhc_hs_detect);

/*
 * wcd939x_mbhc_hs_detect_exit: stop mbhc insertion/removal functionality
 * @component: handle to snd_soc_component *
 */
void wcd939x_mbhc_hs_detect_exit(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x = NULL;
	struct wcd939x_mbhc *wcd939x_mbhc = NULL;

	if (!component) {
		pr_err_ratelimited("%s: component is NULL\n", __func__);
		return;
	}

	wcd939x = snd_soc_component_get_drvdata(component);
	if (!wcd939x) {
		pr_err_ratelimited("%s: wcd939x is NULL\n", __func__);
		return;
	}

	wcd939x_mbhc = wcd939x->mbhc;
	if (!wcd939x_mbhc) {
		dev_err_ratelimited(component->dev, "%s: mbhc not initialized!\n", __func__);
		return;
	}
	wcd_mbhc_stop(&wcd939x_mbhc->wcd_mbhc);
}
EXPORT_SYMBOL(wcd939x_mbhc_hs_detect_exit);

/*
 * wcd939x_mbhc_ssr_down: stop mbhc during
 * wcd939x subsystem restart
 * mbhc: pointer to wcd937x_mbhc structure
 * component: handle to snd_soc_component *
 */
void wcd939x_mbhc_ssr_down(struct wcd939x_mbhc *mbhc,
			struct snd_soc_component *component)
{
	struct wcd_mbhc *wcd_mbhc = NULL;

	if (!mbhc || !component)
		return;

	wcd_mbhc = &mbhc->wcd_mbhc;
	if (!wcd_mbhc) {
		dev_err_ratelimited(component->dev, "%s: wcd_mbhc is NULL\n", __func__);
		return;
	}

	wcd939x_mbhc_hs_detect_exit(component);
	wcd_mbhc_deinit(wcd_mbhc);
}
EXPORT_SYMBOL(wcd939x_mbhc_ssr_down);

/*
 * wcd939x_mbhc_post_ssr_init: initialize mbhc for
 * wcd939x post subsystem restart
 * @mbhc: poniter to wcd939x_mbhc structure
 * @component: handle to snd_soc_component *
 *
 * return 0 if mbhc_init is success or error code in case of failure
 */
int wcd939x_mbhc_post_ssr_init(struct wcd939x_mbhc *mbhc,
			     struct snd_soc_component *component)
{
	int ret = 0;
	struct wcd_mbhc *wcd_mbhc = NULL;

	if (!mbhc || !component)
		return -EINVAL;

	wcd_mbhc = &mbhc->wcd_mbhc;
	if (wcd_mbhc == NULL) {
		pr_err("%s: wcd_mbhc is NULL\n", __func__);
		return -EINVAL;
	}

	/* Reset detection type to insertion after SSR recovery */
	snd_soc_component_update_bits(component, WCD939X_MBHC_MECH,
				0x20, 0x20);
	#ifdef OPLUS_ARCH_EXTENDS
	/* Modify for headphone volume match to impedance */
	if (wcd_mbhc->enable_hp_impedance_detect)
		ret = wcd_mbhc_init(wcd_mbhc, component, &mbhc_cb, &intr_ids,
					wcd_mbhc_registers, true);
	else
		ret = wcd_mbhc_init(wcd_mbhc, component, &mbhc_cb, &intr_ids,
					wcd_mbhc_registers, false);
	#else /* OPLUS_ARCH_EXTENDS */
	ret = wcd_mbhc_init(wcd_mbhc, component, &mbhc_cb, &intr_ids,
			    wcd_mbhc_registers, WCD939X_ZDET_SUPPORTED);
	#endif /* OPLUS_ARCH_EXTENDS */
	if (ret) {
		dev_err(component->dev, "%s: mbhc initialization failed\n",
			__func__);
		goto done;
	}

done:
	return ret;
}
EXPORT_SYMBOL(wcd939x_mbhc_post_ssr_init);

/*
 * wcd939x_mbhc_init: initialize mbhc for wcd939x
 * @mbhc: poniter to wcd939x_mbhc struct pointer to store the configs
 * @codec: handle to snd_soc_component *
 * @fw_data: handle to firmware data
 *
 * return 0 if mbhc_init is success or error code in case of failure
 */
int wcd939x_mbhc_init(struct wcd939x_mbhc **mbhc,
				struct snd_soc_component *component,
				struct fw_info *fw_data)
{
	struct wcd939x_mbhc *wcd939x_mbhc = NULL;
	struct wcd_mbhc *wcd_mbhc = NULL;
	int ret = 0;
	struct wcd939x_pdata *pdata;
	struct wcd939x_priv *wcd939x;
	#ifdef OPLUS_ARCH_EXTENDS
	/* Add for headphone volume match to impedance */
	u32 enable_hp_impedance_detect = 0;
	int rc = 0;
	const char *mbhc_enable_hp_impedance_detect = "oplus,mbhc_enable_hp_impedance_detect";
	#endif /* OPLUS_ARCH_EXTENDS */

	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	wcd939x_mbhc = devm_kzalloc(component->dev, sizeof(struct wcd939x_mbhc),
				    GFP_KERNEL);
	if (!wcd939x_mbhc)
		return -ENOMEM;

	wcd939x_mbhc->fw_data = fw_data;
	BLOCKING_INIT_NOTIFIER_HEAD(&wcd939x_mbhc->notifier);
	wcd_mbhc = &wcd939x_mbhc->wcd_mbhc;
	if (wcd_mbhc == NULL) {
		pr_err("%s: wcd_mbhc is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	/* Setting default mbhc detection logic to ADC */
	wcd_mbhc->mbhc_detection_logic = WCD_DETECTION_ADC;

	/* Down ramp timer set-up */
	timer_setup(&wcd939x_mbhc->rdown_timer, rdown_timer_callback, 0);
	wcd939x_mbhc->rdown_prev_iter = 0;
	wcd939x_mbhc->rdown_timer_complete = false;

	pdata = dev_get_platdata(component->dev);
	if (!pdata) {
		#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
		dev_err_fb_fatal_delay(component->dev, "%s: pdata pointer is NULL\n", __func__);
		#else /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */
		dev_err(component->dev, "%s: pdata pointer is NULL\n",
			__func__);
		#endif /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */
		ret = -EINVAL;
		goto err;
	}
	wcd_mbhc->micb_mv = pdata->micbias.micb2_mv;

	#ifdef OPLUS_ARCH_EXTENDS
	/* Add for headphone volume match to impedance */
	wcd_mbhc->enable_hp_impedance_detect = false;
	if (of_find_property(component->dev->of_node, mbhc_enable_hp_impedance_detect, NULL)) {
		rc = of_property_read_u32(component->dev->of_node, mbhc_enable_hp_impedance_detect, &enable_hp_impedance_detect);
		if (!rc) {
			if (enable_hp_impedance_detect) {
				wcd_mbhc->enable_hp_impedance_detect= true;
			} else {
				wcd_mbhc->enable_hp_impedance_detect = false;
			}
		} else {
			dev_info(component->dev, "%s: Looking up %s property in node %s failed\n",
				__func__, mbhc_enable_hp_impedance_detect, component->dev->of_node->full_name);
		}
	} else {
		dev_info(component->dev, "%s: %s DT property not found\n", __func__, mbhc_enable_hp_impedance_detect);
	}
	dev_info(component->dev, "%s:enable_hp_impedance_detect(%d)\n", __func__, wcd_mbhc->enable_hp_impedance_detect);
	#endif /* OPLUS_ARCH_EXTENDS */

	#ifdef OPLUS_ARCH_EXTENDS
	/* Modify for headphone volume match to impedance */
	if (wcd_mbhc->enable_hp_impedance_detect)
		ret = wcd_mbhc_init(wcd_mbhc, component, &mbhc_cb,
					&intr_ids, wcd_mbhc_registers,
					true);
	else
		ret = wcd_mbhc_init(wcd_mbhc, component, &mbhc_cb,
					&intr_ids, wcd_mbhc_registers,
					false);
	#else /* OPLUS_ARCH_EXTENDS */
	ret = wcd_mbhc_init(wcd_mbhc, component, &mbhc_cb,
				&intr_ids, wcd_mbhc_registers,
				WCD939X_ZDET_SUPPORTED);
	#endif /* OPLUS_ARCH_EXTENDS */
	if (ret) {
		#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
		dev_err_fb_fatal_delay(component->dev, "%s: mbhc initialization failed\n", __func__);
		#else /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */
		dev_err(component->dev, "%s: mbhc initialization failed\n",
			__func__);
		#endif /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */
		goto err;
	}

	(*mbhc) = wcd939x_mbhc;
	snd_soc_add_component_controls(component, impedance_detect_controls,
				   ARRAY_SIZE(impedance_detect_controls));
	snd_soc_add_component_controls(component, hph_type_detect_controls,
				   ARRAY_SIZE(hph_type_detect_controls));

	wcd939x = dev_get_drvdata(component->dev);
	if (!wcd939x) {
		dev_err(component->dev, "%s: wcd939x pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}
	usbcss_hs_sysfs_init(wcd939x);

	return 0;
err:
	if (wcd939x_mbhc)
		del_timer(&wcd939x_mbhc->rdown_timer);
	devm_kfree(component->dev, wcd939x_mbhc);
	return ret;
}
EXPORT_SYMBOL(wcd939x_mbhc_init);

/*
 * wcd939x_mbhc_deinit: deinitialize mbhc for wcd939x
 * @codec: handle to snd_soc_component *
 */
void wcd939x_mbhc_deinit(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x;
	struct wcd939x_mbhc *wcd939x_mbhc;

	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return;
	}

	wcd939x = snd_soc_component_get_drvdata(component);
	if (!wcd939x) {
		pr_err("%s: wcd939x is NULL\n", __func__);
		return;
	}

	wcd939x_mbhc = wcd939x->mbhc;
	if (wcd939x_mbhc) {
		del_timer(&wcd939x_mbhc->rdown_timer);
		wcd_mbhc_deinit(&wcd939x_mbhc->wcd_mbhc);
		devm_kfree(component->dev, wcd939x_mbhc);
	}
}
EXPORT_SYMBOL(wcd939x_mbhc_deinit);
