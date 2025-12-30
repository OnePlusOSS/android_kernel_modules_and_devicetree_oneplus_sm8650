// SPDX-License-Identifier: GPL-2.0
/*
 * Awinic high voltage LRA haptic driver
 *
 * Copyright (c) 2021-2023 awinic. All Rights Reserved.
 *
 * Author: wangzhi <wangzhi@awinic.com>
 */

#include "haptic_hv.h"
#include "haptic_hv_reg.h"

#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
#include "../../aw8697_haptic/haptic_feedback.h"
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
static struct vmax_map vmax_map[] = {
	{0,    0x11, 0x00},
	{100,  0x11, 0x05},
	{200,  0x11, 0x0b},
	{300,  0x11, 0x10},
	{400,  0x11, 0x15},
	{500,  0x11, 0x1b},
	{600,  0x11, 0x20},
	{700,  0x11, 0x25},
	{800,  0x11, 0x2b},
	{900,  0x11, 0x30},
	{1000, 0x11, 0x35},
	{1100, 0x11, 0x3b},
	{1200, 0x11, 0x40},
	{1300, 0x11, 0x45},
	{1400, 0x11, 0x4b},
	{1500, 0x11, 0x50},
	{1600, 0x11, 0x55},
	{1700, 0x11, 0x5b},
	{1800, 0x11, 0x60},
	{1900, 0x11, 0x65},
	{2000, 0x11, 0x6b},
	{2100, 0x11, 0x70},
	{2200, 0x11, 0x75},
	{2300, 0x11, 0x7b},
	{2400, 0x11, 0x80},
};
#endif

/******************************************************
 *
 * aw8693xs codec
 *
 ******************************************************/
#ifdef AW_SND_SOC_CODEC
#ifdef KERNEL_OVER_4_19
static const struct aw_componet_codec_ops aw_componet_codec_ops = {
	.aw_snd_soc_kcontrol_codec = snd_soc_kcontrol_component,
	.aw_snd_soc_codec_get_drvdata = snd_soc_component_get_drvdata,
	.aw_snd_soc_add_codec_controls = snd_soc_add_component_controls,
	.aw_snd_soc_unregister_codec = snd_soc_unregister_component,
	.aw_snd_soc_register_codec = snd_soc_register_component,
};
#else
static const struct aw_componet_codec_ops aw_componet_codec_ops = {
	.aw_snd_soc_kcontrol_codec = snd_soc_kcontrol_codec,
	.aw_snd_soc_codec_get_drvdata = snd_soc_codec_get_drvdata,
	.aw_snd_soc_add_codec_controls = snd_soc_add_codec_controls,
	.aw_snd_soc_unregister_codec = snd_soc_unregister_codec,
	.aw_snd_soc_register_codec = snd_soc_register_codec,
};
#endif

static aw_snd_soc_codec_t *aw_get_codec(struct snd_soc_dai *dai)
{
#ifdef KERNEL_OVER_4_19
	return dai->component;
#else
	return dai->codec;
#endif
}

static void aw8693xs_i2s_enable(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL4,
					 AW8693XS_BIT_SYSCTRL4_EN_DLL_MASK,
					 AW8693XS_BIT_SYSCTRL4_EN_DLL_ON);
		i2c_w_bits(aw_haptic, AW8693XS_REG_I2SCFG1,
					 AW8693XS_BIT_I2SCFG1_I2S_EN_MASK,
					 AW8693XS_BIT_I2SCFG1_I2S_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL4,
					 AW8693XS_BIT_SYSCTRL4_EN_DLL_MASK,
					 AW8693XS_BIT_SYSCTRL4_EN_DLL_OFF);
		i2c_w_bits(aw_haptic, AW8693XS_REG_I2SCFG1,
					 AW8693XS_BIT_I2SCFG1_I2S_EN_MASK,
					 AW8693XS_BIT_I2SCFG1_I2S_DISABLE);
	}
}

static int aw8693xs_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	return 0;
}

static int aw8693xs_set_fmt(struct snd_soc_dai *dai, uint32_t fmt)
{
	aw_dev_info("fmt=0x%X", fmt);

	return 0;
}

static int aw8693xs_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id, uint32_t freq, int dir)
{
	aw_dev_info("freq=%d", freq);

	return 0;
}

static int aw8693xs_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	uint8_t mask = 0;
	uint8_t reg_val = 0;
	uint8_t bit_width = 0;
	uint32_t sample_rate = 0;
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic = aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_dev_err("steam is capture");
		return 0;
	}

	sample_rate = params_rate(params);
	aw_dev_info("sample rate = %u", sample_rate);
	switch (sample_rate) {
	case 48000:
		reg_val |= AW8693XS_BIT_I2SCFG2_I2S_SR_48K;
		break;
	case 96000:
		reg_val |= AW8693XS_BIT_I2SCFG2_I2S_SR_96K;
		break;
	default:
		reg_val |= AW8693XS_BIT_I2SCFG2_I2S_SR_48K;
		aw_dev_err("default use 48K");
		break;
	}

	bit_width = params_width(params);
	aw_dev_info("bit width = %d", bit_width);
	switch (bit_width) {
	case 16:
		reg_val |= AW8693XS_BIT_I2SCFG2_BCK_MODE_16;
		break;
	case 24:
		reg_val |= AW8693XS_BIT_I2SCFG2_BCK_MODE_24;
		break;
	case 32:
		reg_val |= AW8693XS_BIT_I2SCFG2_BCK_MODE_32;
		break;
	default:
		reg_val |= AW8693XS_BIT_I2SCFG2_BCK_MODE_32;
		aw_dev_err("default use 32 bit");
		break;
	}

	mask = AW8693XS_BIT_I2SCFG2_I2S_SR_MASK & AW8693XS_BIT_I2SCFG2_BCK_MODE_MASK;
	mutex_lock(&aw_haptic->lock);
	i2c_w_bits(aw_haptic, AW8693XS_REG_I2SCFG2, mask, reg_val);
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static int aw8693xs_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	uint8_t reg_val = 0;
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic = aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_dev_err("steam is capture");
		return 0;
	}
	aw_dev_info("mute state=%d", mute);
	if (mute) {
		mutex_lock(&aw_haptic->lock);
		aw8693xs_i2s_enable(aw_haptic, false);
		mutex_unlock(&aw_haptic->lock);
	} else {
		mutex_lock(&aw_haptic->lock);
		aw8693xs_i2s_enable(aw_haptic, true);
		usleep_range(1000, 1500);
		i2c_r_bytes(aw_haptic, AW8693XS_REG_GLBRD5, &reg_val, AW_I2C_BYTE_ONE);
		if (reg_val != 0x0a) {
			aw_dev_err("i2s config err, glb_state=0x%02X", reg_val);
			aw8693xs_i2s_enable(aw_haptic, false);
		}
		mutex_unlock(&aw_haptic->lock);
	}

	return 0;
}

static void aw8693xs_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic = aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_dev_err("steam is capture");
		return;
	}
	mutex_lock(&aw_haptic->lock);
	aw8693xs_i2s_enable(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);
}

static const struct snd_soc_dai_ops aw8693xs_dai_ops = {
	.startup = aw8693xs_startup,
	.set_fmt = aw8693xs_set_fmt,
	.set_sysclk = aw8693xs_set_dai_sysclk,
	.hw_params = aw8693xs_hw_params,
	.mute_stream = aw8693xs_mute,
	.shutdown = aw8693xs_shutdown,
};

static const struct snd_soc_dapm_widget aw8693xs_dapm_widgets[] = {
	// SND_SOC_DAPM_DAC("DACL", NULL, SND_SOC_NOPM, 0, 0),
	// SND_SOC_DAPM_DAC("DACR", NULL, SND_SOC_NOPM, 0, 0),

	// SND_SOC_DAPM_OUTPUT("OUT_A"),
	// SND_SOC_DAPM_OUTPUT("OUT_B"),
	// SND_SOC_DAPM_OUTPUT("OUT_C"),
	// SND_SOC_DAPM_OUTPUT("OUT_D"),
	/* playback */
	SND_SOC_DAPM_AIF_IN("AIF_RX", "Speaker_Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("audio_out"),
};

static const struct snd_soc_dapm_route aw8693xs_dapm_routes[] = {
	// { "DACL",  NULL, "Playback" },
	// { "DACR",  NULL, "Playback" },

	// { "OUT_A", NULL, "DACL" },
	// { "OUT_B", NULL, "DACL" },
	// { "OUT_C", NULL, "DACR" },
	// { "OUT_D", NULL, "DACR" },
	{"audio_out", NULL, "AIF_RX"},
};

static struct snd_soc_dai_driver aw8693xs_dai[] = {
	{
		.name = "aw8693xs-aif",
		.id = 1,
		.playback = {
			.stream_name = "Speaker_Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &aw8693xs_dai_ops,
	},
};

static int aw8693xs_codec_probe(aw_snd_soc_codec_t *codec)
{
	uint8_t reg_val[2] = { 0 };
	struct aw_haptic *aw_haptic = aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	reg_val[0] |= AW8693XS_BIT_I2SCFG1_SLOT_NUM_I2S;
	reg_val[0] |= AW8693XS_BIT_I2SCFG1_I2S_MODE_PHILIP;
	reg_val[0] |= AW8693XS_BIT_I2SCFG1_RX_SLOTVLD_1;

	reg_val[1] |= AW8693XS_BIT_I2SCFG2_FSYN_TYP_ONE_SLOT;
	reg_val[1] |= AW8693XS_BIT_I2SCFG2_I2S_INT_ENABLE;

	i2c_w_bytes(aw_haptic, AW8693XS_REG_I2SCFG1, reg_val, AW_I2C_BYTE_TWO);

	return 0;
}

#ifdef KERNEL_OVER_4_19
static void aw8693xs_codec_remove(aw_snd_soc_codec_t *component)
{
	aw_dev_info("enter");
}
#else
static int aw8693xs_codec_remove(aw_snd_soc_codec_t *codec)
{
	aw_dev_info("enter");

	return 0;
}
#endif

static aw_snd_soc_codec_driver_t soc_codec_dev_aw8693xs = {
	.probe = aw8693xs_codec_probe,
	.remove = aw8693xs_codec_remove,
	.dapm_widgets		= aw8693xs_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(aw8693xs_dapm_widgets),
	.dapm_routes		= aw8693xs_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(aw8693xs_dapm_routes),
};

static int aw8693xs_snd_soc_init(struct device *dev)
{
	int ret = 0;
	struct snd_soc_dai_driver *dai;

	/* register codec */
	dai = devm_kzalloc(dev, sizeof(aw8693xs_dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	memmove(dai, aw8693xs_dai, sizeof(aw8693xs_dai));
	aw_dev_info("dai->name(%s)", dai->name);

	ret = aw_componet_codec_ops.aw_snd_soc_register_codec(dev, &soc_codec_dev_aw8693xs,
							      dai, ARRAY_SIZE(aw8693xs_dai));
	if (ret < 0) {
		aw_dev_err("failed to register aw8693xs: %d", ret);
		return ret;
	}

	return 0;
}
#endif

static void aw8693xs_reg_unlock(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t reg_val = 0;

	if (flag) {
		/* Unlock register */
		reg_val = AW8693XS_BIT_TMCFG_TM_UNLOCK;
		i2c_w_bytes(aw_haptic, AW8693XS_REG_TMCFG, &reg_val, AW_I2C_BYTE_ONE);
	} else {
		/* Lock register */
		reg_val = AW8693XS_BIT_TMCFG_TM_LOCK;
		i2c_w_bytes(aw_haptic, AW8693XS_REG_TMCFG, &reg_val, AW_I2C_BYTE_ONE);
	}
}

static int aw8693xs_check_qualify(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_EFCFG6, &reg_val, AW_I2C_BYTE_ONE);
	if (!(reg_val & 0x80)) {
		aw_dev_err("unqualified chip");
		return -ERANGE;
	}
	aw_dev_info("qualified chip");

	return 0;
}

static void aw8693xs_set_pwm(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_PWM_48K:
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL4,
					 AW8693XS_BIT_SYSCTRL4_WAVDAT_MODE_MASK,
					 AW8693XS_BIT_SYSCTRL4_WAVDAT_48K);
		break;
	case AW_PWM_24K:
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL4,
					 AW8693XS_BIT_SYSCTRL4_WAVDAT_MODE_MASK,
					 AW8693XS_BIT_SYSCTRL4_WAVDAT_24K);
		break;
	case AW_PWM_12K:
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL4,
					 AW8693XS_BIT_SYSCTRL4_WAVDAT_MODE_MASK,
					 AW8693XS_BIT_SYSCTRL4_WAVDAT_12K);
		break;
	case AW_PWM_8K:
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL4,
					 AW8693XS_BIT_SYSCTRL4_WAVDAT_MODE_MASK,
					 AW8693XS_BIT_SYSCTRL4_WAVDAT_8K);
		break;
	default:
		aw_dev_err("error param");
		break;
	}
}

static void aw8693xs_set_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	i2c_w_bytes(aw_haptic, AW8693XS_REG_PLAYCFG2, &gain, AW_I2C_BYTE_ONE);
}

static void aw8693xs_set_bst_peak_cur(struct aw_haptic *aw_haptic)
{

}
#ifdef OPLUS_FEATURE_CHG_BASIC
static void aw8693xs_set_trig_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	i2c_w_bytes(aw_haptic, AW8693XS_TRGCFG9, &gain, AW_I2C_BYTE_ONE);
}
static void aw8693xs_set_bst_vol(struct aw_haptic *aw_haptic, uint8_t bst_vol)
{
	uint8_t min_bst_vol = 0x05;
	uint8_t max_bst_vol = 0x19;

	bst_vol = (bst_vol > max_bst_vol) ? max_bst_vol : bst_vol;
	bst_vol = (bst_vol < min_bst_vol) ? min_bst_vol : bst_vol;

	i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG1,
				 AW8693XS_BIT_PLAYCFG1_BST_VOUT_VREFSET_MASK, bst_vol);
	aw_dev_info("%s: bst_vol 0x%02X", __func__, bst_vol);
}
#else
static void aw8693xs_set_bst_vol(struct aw_haptic *aw_haptic, uint32_t bst_vol)
{
	uint8_t reg_val = 0;

	bst_vol = (bst_vol < AW8693XS_BST_VOL_MIN) ? AW8693XS_BST_VOL_MIN : bst_vol;
	bst_vol = (bst_vol > AW8693XS_BST_VOL_MAX) ? AW8693XS_BST_VOL_MAX : bst_vol;
	reg_val = AW8693XS_BST_VOL_FORMULA(bst_vol);
	aw_info("set bst vol = %u mV, reg_val = 0x%02x", bst_vol, reg_val);

	i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG1,
				 AW8693XS_BIT_PLAYCFG1_BST_VOUT_VREFSET_MASK, reg_val);
}
#endif
static void aw8693xs_set_wav_seq(struct aw_haptic *aw_haptic, uint8_t wav, uint8_t seq)
{
	i2c_w_bytes(aw_haptic, AW8693XS_REG_WAVCFG1 + wav, &seq, AW_I2C_BYTE_ONE);
}

static void aw8693xs_set_wav_loop(struct aw_haptic *aw_haptic, uint8_t wav, uint8_t loop)
{
	if (wav % 2) {
		loop <<= 0;
		i2c_w_bits(aw_haptic, AW8693XS_REG_WAVCFG9 + (wav / 2),
					 AW8693XS_BIT_WAVLOOP_SEQ_EVEN_MASK, loop);
	} else {
		loop <<= 4;
		i2c_w_bits(aw_haptic, AW8693XS_REG_WAVCFG9 + (wav / 2),
					 AW8693XS_BIT_WAVLOOP_SEQ_ODD_MASK, loop);
	}
}

static void aw8693xs_set_rtp_data(struct aw_haptic *aw_haptic, uint8_t *data, uint32_t len)
{
	i2c_w_bytes(aw_haptic, AW8693XS_REG_RTPDATA, data, len);
}

static void aw8693xs_set_rtp_aei(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSINTM,
					 AW8693XS_BIT_SYSINTM_FF_AEM_MASK,
					 AW8693XS_BIT_SYSINTM_FF_AEM_ON);
	} else {
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSINTM,
					 AW8693XS_BIT_SYSINTM_FF_AEM_MASK,
					 AW8693XS_BIT_SYSINTM_FF_AEM_OFF);
	}
}

static void aw8693xs_set_ram_addr(struct aw_haptic *aw_haptic)
{
	uint8_t ram_addr[2] = {0};

	ram_addr[0] = AW8693XS_BASEADDR_H(aw_haptic->ram.base_addr);
	ram_addr[1] = AW8693XS_BASEADDR_L(aw_haptic->ram.base_addr);

	i2c_w_bytes(aw_haptic, AW8693XS_REG_RAMADDRH, ram_addr, AW_I2C_BYTE_TWO);
}

static void aw8693xs_set_base_addr(struct aw_haptic *aw_haptic)
{
	uint8_t rtp_addr[2] = {0};

	rtp_addr[0] = AW8693XS_BASEADDR_H(aw_haptic->ram.base_addr);
	rtp_addr[1] = AW8693XS_BASEADDR_L(aw_haptic->ram.base_addr);

	i2c_w_bits(aw_haptic, AW8693XS_REG_RTPCFG1,
				 AW8693XS_BIT_RTPCFG1_BASE_ADDR_H_MASK, rtp_addr[0]);
	i2c_w_bytes(aw_haptic, AW8693XS_REG_RTPCFG2, &rtp_addr[1], AW_I2C_BYTE_ONE);
}

static void aw8693xs_auto_brake_mode(struct aw_haptic *aw_haptic, bool flag)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (flag && aw_haptic->auto_break_mode_support) {
#else
	if (flag) {
#endif
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8693XS_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8693XS_BIT_PLAYCFG3_BRK_DISABLE);
	}
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static void aw8693xs_f0_auto_brake_mode(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8693XS_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8693XS_BIT_PLAYCFG3_BRK_DISABLE);
	}
}

static uint8_t aw8693xs_get_chip_state(struct aw_haptic *aw_haptic)
{
	uint8_t chip_state_val = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSST, &chip_state_val, AW_I2C_BYTE_ONE);
	return chip_state_val;
}
#endif

static void aw8693xs_f0_detect(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		i2c_w_bits(aw_haptic, AW8693XS_REG_CONTCFG1,
					 AW8693XS_BIT_CONTCFG1_BEMF_CYCLE_MASK,
					 AW8693XS_BIT_CONTCFG1_BEMF_CYCLE_3);
	} else {
		i2c_w_bits(aw_haptic, AW8693XS_REG_CONTCFG1,
					 AW8693XS_BIT_CONTCFG1_BEMF_CYCLE_MASK,
					 AW8693XS_BIT_CONTCFG1_BEMF_CYCLE_0);
	}
}

static uint8_t aw8693xs_get_glb_state(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_GLBRD5, &state, AW_I2C_BYTE_ONE);

	return state;
}

static void aw8693xs_play_go(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t reg_val = 0;

	aw_dev_info("enter, flag = %d", flag);

	if (flag) {
		reg_val = AW8693XS_BIT_PLAYCFG4_GO_ON;
		i2c_w_bytes(aw_haptic, AW8693XS_REG_PLAYCFG4, &reg_val, AW_I2C_BYTE_ONE);
	} else {
		reg_val = AW8693XS_BIT_PLAYCFG4_STOP_ON;
		i2c_w_bytes(aw_haptic, AW8693XS_REG_PLAYCFG4, &reg_val, AW_I2C_BYTE_ONE);
	}
}

static int aw8693xs_wait_enter_standby(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int count = 100;

	while (count--) {
		reg_val = aw8693xs_get_glb_state(aw_haptic);
		if (reg_val == AW8693XS_BIT_GLBRD5_STATE_STANDBY) {
			aw_dev_info("entered standby!");
			return 0;
		}
		usleep_range(2000, 2500);
	}
	aw_dev_err("do not enter standby automatically");

	return -ERANGE;
}

static void aw8693xs_bst_mode_config(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_BST_BOOST_MODE:
		aw_dev_info("haptic bst mode = bst");
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG1,
					 AW8693XS_BIT_PLAYCFG1_BST_MODE_MASK,
					 AW8693XS_BIT_PLAYCFG1_BST_MODE);
		break;
	case AW_BST_BYPASS_MODE:
		aw_dev_info("haptic bst mode = bypass");
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG1,
					 AW8693XS_BIT_PLAYCFG1_BST_MODE_MASK,
					 AW8693XS_BIT_PLAYCFG1_BST_MODE_BYPASS);
		break;
	default:
		aw_dev_err("mode = %d error", mode);
		break;
	}
}

static void aw8693xs_vbat_ref_config(struct aw_haptic *aw_haptic, uint16_t vbat_ref)
{
	uint8_t reg_val = 0;

	aw_dev_info("enter");
	if (vbat_ref <= 3450)
		reg_val = AW8693XS_BIT_DETCFG1_VBAT_REF_3_3;
	else if (vbat_ref <= 3800)
		reg_val = AW8693XS_BIT_DETCFG1_VBAT_REF_3_6;
	else if (vbat_ref <= 4100)
		reg_val = AW8693XS_BIT_DETCFG1_VBAT_REF_4_0;
	else if (vbat_ref <= 4350)
		reg_val = AW8693XS_BIT_DETCFG1_VBAT_REF_4_2;
	else if (vbat_ref <= 4700)
		reg_val = AW8693XS_BIT_DETCFG1_VBAT_REF_4_5;
	else if (vbat_ref <= 4900)
		reg_val = AW8693XS_BIT_DETCFG1_VBAT_REF_4_8;
	else if (vbat_ref <= 5250)
		reg_val = AW8693XS_BIT_DETCFG1_VBAT_REF_5_0;
	else
		reg_val = AW8693XS_BIT_DETCFG1_VBAT_REF_5_5;

	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1,
					AW8693XS_BIT_DETCFG1_VBAT_REF_MASK, reg_val);
}

static void aw8693xs_vbat_mode_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	if (flag == AW_CONT_VBAT_HW_COMP_MODE) {
		i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1,
					 AW8693XS_BIT_DETCFG1_VBAT_MODE_MASK,
					 AW8693XS_BIT_DETCFG1_VBAT_MODE_HW);
	} else {
		i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1,
					 AW8693XS_BIT_DETCFG1_VBAT_MODE_MASK,
					 AW8693XS_BIT_DETCFG1_VBAT_MODE_SW);
	}
}

static void aw8693xs_play_mode(struct aw_haptic *aw_haptic, uint8_t play_mode)
{
	switch (play_mode) {
	case AW_STANDBY_MODE:
		aw_dev_info("enter standby mode");
		aw_haptic->play_mode = AW_STANDBY_MODE;
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL3,
					 AW8693XS_BIT_SYSCTRL3_STANDBY_MASK,
					 AW8693XS_BIT_SYSCTRL3_STANDBY_ON);
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL3,
					 AW8693XS_BIT_SYSCTRL3_STANDBY_MASK,
					 AW8693XS_BIT_SYSCTRL3_STANDBY_OFF);
		break;
	case AW_RAM_MODE:
		aw_dev_info("enter ram mode");
		aw_haptic->play_mode = AW_RAM_MODE;
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8693XS_BIT_PLAYCFG3_PLAY_MODE_RAM);
#ifndef OPLUS_FEATURE_CHG_BASIC
		aw8693xs_auto_brake_mode(aw_haptic, false);
#else
		aw8693xs_auto_brake_mode(aw_haptic, true);
#endif
		aw8693xs_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		aw8693xs_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RAM_LOOP_MODE:
		aw_dev_info("enter ram loop mode");
		aw_haptic->play_mode = AW_RAM_LOOP_MODE;
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8693XS_BIT_PLAYCFG3_PLAY_MODE_RAM);
		aw8693xs_auto_brake_mode(aw_haptic, true);
#ifndef OPLUS_FEATURE_CHG_BASIC
		/* bst mode Already configured in vibrator_work_routine func, close here */
		aw8693xs_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
#endif
		aw8693xs_vbat_mode_config(aw_haptic, AW_CONT_VBAT_HW_COMP_MODE);
		break;
	case AW_RTP_MODE:
		aw_dev_info("enter rtp mode");
		aw_haptic->play_mode = AW_RTP_MODE;
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8693XS_BIT_PLAYCFG3_PLAY_MODE_RTP);
		aw8693xs_auto_brake_mode(aw_haptic, true);
		aw8693xs_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		aw8693xs_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_TRIG_MODE:
		aw_dev_info("enter trig mode");
		aw_haptic->play_mode = AW_TRIG_MODE;
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8693XS_BIT_PLAYCFG3_PLAY_MODE_RAM);
		aw8693xs_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_CONT_MODE:
		aw_dev_info("enter cont mode");
		aw_haptic->play_mode = AW_CONT_MODE;
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8693XS_BIT_PLAYCFG3_PLAY_MODE_CONT);
		aw8693xs_auto_brake_mode(aw_haptic, true);
		aw8693xs_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
		aw8693xs_vbat_mode_config(aw_haptic, AW_CONT_VBAT_HW_COMP_MODE);
		break;
	default:
		aw_dev_err("play mode %d error", play_mode);
		break;
	}
}

static void aw8693xs_ram_init(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL3,
					 AW8693XS_BIT_SYSCTRL3_EN_RAMINIT_MASK,
					 AW8693XS_BIT_SYSCTRL3_EN_RAMINIT_ON);
		usleep_range(500, 600);
	} else {
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL3,
					 AW8693XS_BIT_SYSCTRL3_EN_RAMINIT_MASK,
					 AW8693XS_BIT_SYSCTRL3_EN_RAMINIT_OFF);
	}
}

static void aw8693xs_stop(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = 0;

	aw_dev_info("enter");
	aw_haptic->play_mode = AW_STANDBY_MODE;
	reg_val = AW8693XS_BIT_PLAYCFG4_STOP_ON;
	i2c_w_bytes(aw_haptic, AW8693XS_REG_PLAYCFG4, &reg_val, AW_I2C_BYTE_ONE);
	ret = aw8693xs_wait_enter_standby(aw_haptic);
	if (ret < 0) {
		aw_dev_err("force to enter standby mode!");
		aw8693xs_play_mode(aw_haptic, AW_STANDBY_MODE);
	}
}

static void aw8693xs_upload_lra(struct aw_haptic *aw_haptic, uint32_t flag)
{
	uint8_t cali_data = 0;
	uint8_t reg_val[2] = { AW_TRIM_EFUSE, AW_TRIM_REGISTER };

	aw8693xs_reg_unlock(aw_haptic, true);
	i2c_w_bytes(aw_haptic, AW8693XS_REG_TRIMCFG3, &reg_val[1], AW_I2C_BYTE_ONE);
	switch (flag) {
	case AW_WRITE_ZERO:
		aw_dev_info("write zero to trim_lra!");
		i2c_w_bytes(aw_haptic, AW8693XS_REG_TRIMCFG3, &reg_val[0], AW_I2C_BYTE_ONE);
		break;
	case AW_F0_CALI_LRA:
		aw_dev_info("write f0_cali_data to trim_lra = 0x%02X", aw_haptic->f0_cali_data);
		cali_data = aw_haptic->f0_cali_data;
		i2c_w_bytes(aw_haptic, AW8693XS_REG_TRIMCFG4, &cali_data, AW_I2C_BYTE_ONE);
		break;
	case AW_OSC_CALI_LRA:
		aw_dev_info("write osc_cali_data to trim_lra = 0x%02X", aw_haptic->osc_cali_data);
		cali_data = aw_haptic->osc_cali_data;
		i2c_w_bytes(aw_haptic, AW8693XS_REG_TRIMCFG4, &cali_data, AW_I2C_BYTE_ONE);
		break;
	default:
		aw_dev_err("error param, write f0_cali_data to trim_lra!");
		cali_data = aw_haptic->f0_cali_data;
		i2c_w_bytes(aw_haptic, AW8693XS_REG_TRIMCFG4, &cali_data, AW_I2C_BYTE_ONE);
		break;
	}
	aw8693xs_reg_unlock(aw_haptic, false);
}

static uint8_t aw8693xs_get_trim_lra(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_TRIMCFG4, &reg_val, AW_I2C_BYTE_ONE);

	return reg_val;
}

static void aw8693xs_protect_config(struct aw_haptic *aw_haptic, uint8_t prtime, uint8_t prlvl)
{
	uint8_t reg_val = 0;

	i2c_w_bits(aw_haptic, AW8693XS_REG_PWMCFG1,
				 AW8693XS_BIT_PWMCFG1_PRC_EN_MASK,
				 AW8693XS_BIT_PWMCFG1_PRC_DISABLE);
	if (prlvl != 0) {
		/* Enable protection mode */
		aw_dev_info("enable protection mode");
		reg_val = AW8693XS_BIT_PWMCFG3_PR_ENABLE |
			  (prlvl & (~AW8693XS_BIT_PWMCFG3_PRLVL_MASK));
		i2c_w_bytes(aw_haptic, AW8693XS_REG_PWMCFG3, &reg_val, AW_I2C_BYTE_ONE);
		i2c_w_bytes(aw_haptic, AW8693XS_REG_PWMCFG4, &prtime, AW_I2C_BYTE_ONE);
	} else {
		/* Disable */
		aw_dev_info("disable protection mode");
		i2c_w_bits(aw_haptic, AW8693XS_REG_PWMCFG3,
					 AW8693XS_BIT_PWMCFG3_PR_EN_MASK,
					 AW8693XS_BIT_PWMCFG3_PR_DISABLE);
	}
}

static void aw8693xs_cont_config(struct aw_haptic *aw_haptic)
{
	/* uint8_t drv1_time = 0xFF; */
	uint8_t drv2_time = 0xFF;

	aw8693xs_play_mode(aw_haptic, AW_CONT_MODE);
	i2c_w_bits(aw_haptic, AW8693XS_REG_CONTCFG6,
				 AW8693XS_BIT_CONTCFG6_TRACK_EN_MASK,
				 (aw_haptic->info.is_enabled_track_en << 7));
	i2c_w_bits(aw_haptic, AW8693XS_REG_CONTCFG6,
				 AW8693XS_BIT_CONTCFG6_DRV1_LVL_MASK,
				 aw_haptic->info.cont_drv1_lvl);
	i2c_w_bytes(aw_haptic, AW8693XS_REG_CONTCFG7,
			     &aw_haptic->info.cont_drv2_lvl, AW_I2C_BYTE_ONE);
	/* i2c_w_bytes(aw_haptic, AW8693XS_REG_CONTCFG8, &drv1_time, AW_I2C_BYTE_ONE); */
	i2c_w_bytes(aw_haptic, AW8693XS_REG_CONTCFG9, &drv2_time, AW_I2C_BYTE_ONE);
	aw8693xs_play_go(aw_haptic, true);
}

static void aw8693xs_one_wire_init(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	aw_dev_info("enter");

	/* if enable one-wire, trig1 priority must be less than trig2 and trig3 */
	reg_val = 0x6C;
	i2c_w_bytes(aw_haptic, AW8693XS_REG_GLBCFG4, &reg_val, AW_I2C_BYTE_ONE);

	reg_val = AW8693XS_BIT_START_DLY_2P5MS;
	i2c_w_bytes(aw_haptic, AW8693XS_REG_GLBCFG2, &reg_val, AW_I2C_BYTE_ONE);
	i2c_w_bits(aw_haptic, AW8693XS_REG_TRGCFG8,
				 AW8693XS_BIT_TRGCFG8_TRG_ONEWIRE_MASK,
				 AW8693XS_BIT_TRGCFG8_TRG_ONEWIRE_ENABLE);
}

static void aw8693xs_trig1_param_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->trig[0].trig_level   = aw_haptic->info.trig_cfg[0];
	aw_haptic->trig[0].trig_polar   = aw_haptic->info.trig_cfg[1];
	aw_haptic->trig[0].pos_enable   = aw_haptic->info.trig_cfg[2];
	aw_haptic->trig[0].pos_sequence = aw_haptic->info.trig_cfg[3];
	aw_haptic->trig[0].neg_enable   = aw_haptic->info.trig_cfg[4];
	aw_haptic->trig[0].neg_sequence = aw_haptic->info.trig_cfg[5];
	aw_haptic->trig[0].trig_brk     = aw_haptic->info.trig_cfg[6];
	aw_haptic->trig[0].trig_bst     = aw_haptic->info.trig_cfg[7];
}

static void aw8693xs_trig2_param_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->trig[1].trig_level   = aw_haptic->info.trig_cfg[8];
	aw_haptic->trig[1].trig_polar   = aw_haptic->info.trig_cfg[9];
	aw_haptic->trig[1].pos_enable   = aw_haptic->info.trig_cfg[10];
	aw_haptic->trig[1].pos_sequence = aw_haptic->info.trig_cfg[11];
	aw_haptic->trig[1].neg_enable   = aw_haptic->info.trig_cfg[12];
	aw_haptic->trig[1].neg_sequence = aw_haptic->info.trig_cfg[13];
	aw_haptic->trig[1].trig_brk     = aw_haptic->info.trig_cfg[14];
	aw_haptic->trig[1].trig_bst     = aw_haptic->info.trig_cfg[15];
}

static void aw8693xs_trig3_param_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->trig[2].trig_level   = aw_haptic->info.trig_cfg[16];
	aw_haptic->trig[2].trig_polar   = aw_haptic->info.trig_cfg[17];
	aw_haptic->trig[2].pos_enable   = aw_haptic->info.trig_cfg[18];
	aw_haptic->trig[2].pos_sequence = aw_haptic->info.trig_cfg[19];
	aw_haptic->trig[2].neg_enable   = aw_haptic->info.trig_cfg[20];
	aw_haptic->trig[2].neg_sequence = aw_haptic->info.trig_cfg[21];
	aw_haptic->trig[2].trig_brk     = aw_haptic->info.trig_cfg[22];
	aw_haptic->trig[2].trig_bst     = aw_haptic->info.trig_cfg[23];
}

static void aw8693xs_trig1_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[0].trig_level)
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG1_MODE_LEVEL;
	else
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG1_MODE_EDGE;

	if (aw_haptic->trig[0].trig_polar)
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG1_POLAR_LOW;
	else
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG1_POLAR_HIGH;

	if (aw_haptic->trig[0].trig_brk)
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG1_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG1_AUTO_BRK_DISABLE;

	if (aw_haptic->trig[0].trig_bst)
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG1_BST_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG1_BST_DISABLE;

	i2c_w_bits(aw_haptic, AW8693XS_REG_TRGCFG7,
				 (AW8693XS_BIT_TRGCFG7_TRG1_MODE_MASK &
				  AW8693XS_BIT_TRGCFG7_TRG1_POLAR_MASK &
				  AW8693XS_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK &
				  AW8693XS_BIT_TRGCFG7_TRG1_BST_MASK), trig_config);

	trig_config = 0;
	if (aw_haptic->trig[0].pos_enable)
		trig_config |= AW8693XS_BIT_TRG_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[0].pos_sequence;

	i2c_w_bytes(aw_haptic, AW8693XS_REG_TRGCFG1, &trig_config, AW_I2C_BYTE_ONE);

	trig_config = 0;
	if (aw_haptic->trig[0].neg_enable)
		trig_config |= AW8693XS_BIT_TRG_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[0].neg_sequence;

	i2c_w_bytes(aw_haptic, AW8693XS_REG_TRGCFG4, &trig_config, AW_I2C_BYTE_ONE);
}

static void aw8693xs_trig2_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[1].trig_level)
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG2_MODE_LEVEL;
	else
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG2_MODE_EDGE;

	if (aw_haptic->trig[1].trig_polar)
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG2_POLAR_LOW;
	else
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG2_POLAR_HIGH;

	if (aw_haptic->trig[1].trig_brk)
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG2_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG2_AUTO_BRK_DISABLE;

	if (aw_haptic->trig[1].trig_bst)
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG2_BST_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRGCFG7_TRG2_BST_DISABLE;

	i2c_w_bits(aw_haptic, AW8693XS_REG_TRGCFG7,
				 (AW8693XS_BIT_TRGCFG7_TRG2_MODE_MASK &
				  AW8693XS_BIT_TRGCFG7_TRG2_POLAR_MASK &
				  AW8693XS_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK &
				  AW8693XS_BIT_TRGCFG7_TRG2_BST_MASK), trig_config);

	trig_config = 0;
	if (aw_haptic->trig[1].pos_enable)
		trig_config |= AW8693XS_BIT_TRG_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[1].pos_sequence;

	i2c_w_bytes(aw_haptic, AW8693XS_REG_TRGCFG2, &trig_config, AW_I2C_BYTE_ONE);

	trig_config = 0;
	if (aw_haptic->trig[1].neg_enable)
		trig_config |= AW8693XS_BIT_TRG_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[1].neg_sequence;

	i2c_w_bytes(aw_haptic, AW8693XS_REG_TRGCFG5, &trig_config, AW_I2C_BYTE_ONE);
}

static void aw8693xs_trig3_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[2].trig_level)
		trig_config |= AW8693XS_BIT_TRGCFG8_TRG3_MODE_LEVEL;
	else
		trig_config |= AW8693XS_BIT_TRGCFG8_TRG3_MODE_EDGE;

	if (aw_haptic->trig[2].trig_polar)
		trig_config |= AW8693XS_BIT_TRGCFG8_TRG3_POLAR_LOW;
	else
		trig_config |= AW8693XS_BIT_TRGCFG8_TRG3_POLAR_HIGH;

	if (aw_haptic->trig[2].trig_brk)
		trig_config |= AW8693XS_BIT_TRGCFG8_TRG3_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRGCFG8_TRG3_AUTO_BRK_DISABLE;

	if (aw_haptic->trig[2].trig_bst)
		trig_config |= AW8693XS_BIT_TRGCFG8_TRG3_BST_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRGCFG8_TRG3_BST_DISABLE;

	i2c_w_bits(aw_haptic, AW8693XS_REG_TRGCFG8,
				 (AW8693XS_BIT_TRGCFG8_TRG3_MODE_MASK &
				  AW8693XS_BIT_TRGCFG8_TRG3_POLAR_MASK &
				  AW8693XS_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK &
				  AW8693XS_BIT_TRGCFG8_TRG3_BST_MASK), trig_config);

	trig_config = 0;
	if (aw_haptic->trig[2].pos_enable)
		trig_config |= AW8693XS_BIT_TRG_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[2].pos_sequence;

	i2c_w_bytes(aw_haptic, AW8693XS_REG_TRGCFG3, &trig_config, AW_I2C_BYTE_ONE);

	trig_config = 0;
	if (aw_haptic->trig[2].neg_enable)
		trig_config |= AW8693XS_BIT_TRG_ENABLE;
	else
		trig_config |= AW8693XS_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[2].neg_sequence;

	i2c_w_bytes(aw_haptic, AW8693XS_REG_TRGCFG6, &trig_config, AW_I2C_BYTE_ONE);
}

static void aw8693xs_auto_bst_enable(struct aw_haptic *aw_haptic, uint8_t flag)
{
	aw_haptic->auto_boost = flag;

	if (flag) {
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_AUTO_BST_MASK,
					 AW8693XS_BIT_PLAYCFG3_AUTO_BST_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW8693XS_REG_PLAYCFG3,
					 AW8693XS_BIT_PLAYCFG3_AUTO_BST_MASK,
					 AW8693XS_BIT_PLAYCFG3_AUTO_BST_DISABLE);
	}
}

static void aw8693xs_interrupt_setup(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);

	/* edge int mode */
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL4,
				 AW8693XS_BIT_SYSCTRL4_INT_MODE_MASK,
				 AW8693XS_BIT_SYSCTRL4_INT_MODE_POSEDGE);
	/* int enable */
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSINTM,
				 (AW8693XS_BIT_SYSINTM_BST_SCPM_MASK &
				  AW8693XS_BIT_SYSINTM_BST_OVPM_MASK &
				  AW8693XS_BIT_SYSINTM_UVLM_MASK &
				  AW8693XS_BIT_SYSINTM_OCDM_MASK &
				  AW8693XS_BIT_SYSINTM_OTM_MASK),
				 (AW8693XS_BIT_SYSINTM_BST_SCPM_ON |
				  AW8693XS_BIT_SYSINTM_BST_OVPM_OFF |
				  AW8693XS_BIT_SYSINTM_UVLM_ON |
				  AW8693XS_BIT_SYSINTM_OCDM_ON |
				  AW8693XS_BIT_SYSINTM_OTM_ON));
}

static int aw8693xs_judge_rtp_going(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state = 0;
	uint8_t rtp_state = 0;

	glb_state = aw8693xs_get_glb_state(aw_haptic);
	if (glb_state == AW8693XS_BIT_GLBRD5_STATE_RTP_GO) {
		rtp_state = 1;
		aw_dev_info("rtp is going on");
	}

	return rtp_state;
}

static void aw8693xs_get_ram_data(struct aw_haptic *aw_haptic, char *buf)
{
	int i = 0;
	int size = 0;

	while (i < aw_haptic->ram.len) {
		if ((aw_haptic->ram.len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			size = aw_haptic->ram.len - i;
		else
			size = AW_RAMDATA_RD_BUFFER_SIZE;

		i2c_r_bytes(aw_haptic, AW8693XS_REG_RAMDATA, buf + i, size);

		i += size;
	}
}

static void aw8693xs_get_first_wave_addr(struct aw_haptic *aw_haptic, uint8_t *wave_addr)
{
	uint8_t reg_val[3] = {0};

	i2c_r_bytes(aw_haptic, AW8693XS_REG_RAMDATA, reg_val, AW_I2C_BYTE_THREE);

	wave_addr[0] = reg_val[1];
	wave_addr[1] = reg_val[2];
}

static void aw8693xs_get_wav_seq(struct aw_haptic *aw_haptic, uint32_t len)
{
	uint32_t i = 0;
	uint8_t reg_val[AW_SEQUENCER_SIZE] = {0};

	len = (len > AW_SEQUENCER_SIZE) ? AW_SEQUENCER_SIZE : len;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_WAVCFG1, reg_val, len);
	for (i = 0; i < len; i++)
		aw_haptic->seq[i] = reg_val[i];
}

static void aw8693xs_get_bemf_peak(struct aw_haptic *aw_haptic, uint16_t *peak)
{
	uint8_t reg_val[6] = { 0 };
	uint32_t f0_reg = 0;
	int i = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_CONTRD18, reg_val, 4);
	peak[0] = (reg_val[1] << 8) | (reg_val[0] & (~AW8693XS_BIT_CONTRD18_BEMF_PEAK1_L_MASK));
	peak[1] = (reg_val[2] << 8) | (reg_val[0] & (~AW8693XS_BIT_CONTRD18_BEMF_PEAK2_L_MASK));
	peak[2] = (reg_val[3] << 8) | (reg_val[0] & (~AW8693XS_BIT_CONTRD18_BEMF_PEAK3_L_MASK));
	i2c_r_bytes(aw_haptic, AW8693XS_REG_CONTRD28, reg_val, 3);
	peak[3] = (reg_val[0] << 8) | (reg_val[2] & (~AW8693XS_BIT_CONTRD30_BEMF_PEAK4_L_MASK));
	peak[4] = (reg_val[1] << 8) | (reg_val[2] & (~AW8693XS_BIT_CONTRD30_BEMF_PEAK5_L_MASK));

	for (i = 0; i < 5; i++)
		aw_dev_info("peak[%d] = %d", i, peak[i]);

	i2c_r_bytes(aw_haptic, AW8693XS_REG_CONTRD22, reg_val, 6);
	f0_reg = ((reg_val[0] + reg_val[1]) << 4) | (reg_val[4] & 0x0F + (reg_val[4] >> 4) & 0x0F);
	aw_haptic->f0 = AW8693XS_F0_FORMULA(f0_reg);
	aw_dev_info("lra_f0_1 = %d", aw_haptic->f0);
	f0_reg = ((reg_val[1] + reg_val[2]) << 4) | (reg_val[4] & 0x0F + (reg_val[5] >> 4) & 0x0F);
	aw_haptic->f0 = AW8693XS_F0_FORMULA(f0_reg);
	aw_dev_info("lra_f0_2 = %d", aw_haptic->f0);
	f0_reg = ((reg_val[2] + reg_val[3]) << 4) | ((reg_val[5] >> 4) & 0x0F + reg_val[5] & 0x0F);
	aw_haptic->f0 = AW8693XS_F0_FORMULA(f0_reg);
	aw_dev_info("lra_f0_3 = %d", aw_haptic->f0);
}

static size_t aw8693xs_get_wav_loop(struct aw_haptic *aw_haptic, char *buf)
{
	uint8_t i = 0;
	uint8_t reg_val[AW_SEQUENCER_LOOP_SIZE] = {0};
	size_t count = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_WAVCFG9, reg_val, AW_SEQUENCER_LOOP_SIZE);

	for (i = 0; i < AW_SEQUENCER_LOOP_SIZE; i++) {
		aw_haptic->loop[i * 2 + 0] = (reg_val[i] >> 4) & 0x0F;
		aw_haptic->loop[i * 2 + 1] = (reg_val[i] >> 0) & 0x0F;

		count += snprintf(buf + count, PAGE_SIZE - count, "seq%d loop: 0x%02x\n",
				  i * 2 + 1, aw_haptic->loop[i * 2 + 0]);
		count += snprintf(buf + count, PAGE_SIZE - count, "seq%d loop: 0x%02x\n",
				  i * 2 + 2, aw_haptic->loop[i * 2 + 1]);
	}

	return count;
}

static void aw8693xs_irq_clear(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);
	aw_dev_dbg("reg SYSINT=0x%02X", reg_val);
}

static uint8_t aw8693xs_get_prctmode(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_PWMCFG3, &reg_val, AW_I2C_BYTE_ONE);
	reg_val >>= 7;

	return reg_val;
}
#ifdef OPLUS_FEATURE_CHG_BASIC
static void aw8693xs_dump_rtp_regs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_array[4] = {0};
	uint8_t reg_name[][10] = {
		{"SYSINT"},
		{"SYSINTM"},
		{"SYSST2"},
	};
	int i = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSINT, reg_array, AW_I2C_BYTE_THREE);

	for (i = 0 ; i < AW_I2C_BYTE_THREE; i++) {
		aw_dev_info("%s REG_%s(0x%02x) = 0x%02X\n", __func__,
			    reg_name[i], i+2, reg_array[i]);
	}

	i2c_r_bytes(aw_haptic, AW8693XS_REG_PLAYCFG4, &reg_array[0],
		    AW_I2C_BYTE_ONE);
	aw_dev_info("%s REG_G0(0x09) = 0x%02X\n", __func__,
		    reg_array[0]);
	i2c_r_bytes(aw_haptic, AW8693XS_REG_GLBRD5, &reg_array[0],
		   AW_I2C_BYTE_ONE);
	aw_dev_info("%s REG_GLBRD5(0x3F) = 0x%02X\n", __func__,
		    reg_array[0]);
	i2c_r_bytes(aw_haptic, AW8693XS_REG_PLAYCFG1, &reg_array[0],
		   AW_I2C_BYTE_ONE);
	aw_dev_info("%s REG_Vout(0x06) = 0x%02X\n", __func__,
		    reg_array[0]);
}

static void aw8693xs_op_clean_status(struct aw_haptic *aw_haptic)
{
	aw_haptic->audio_ready = false;
	aw_haptic->haptic_ready = false;
	aw_haptic->pre_haptic_number = 0;
	aw_haptic->rtp_routine_on = 0;

	aw_dev_info("%s enter\n", __func__);
}

static int aw8693xs_get_irq_state(struct aw_haptic *aw_haptic)
{
	int ret = -1;
	uint8_t reg_val = 0;

	aw_dev_dbg("%s: enter\n", __func__);
	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);
	aw_dev_dbg("reg SYSINT=0x%02X", reg_val);

	if (reg_val & AW8693XS_BIT_SYSINT_BST_SCPI) {
		aw8693xs_op_clean_status(aw_haptic);
		aw_dev_err("chip scp int error");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_BST_OVPI) {
		aw8693xs_op_clean_status(aw_haptic);
		ret = AW_IRQ_BST_OVP;
		aw_dev_err("chip ov int error");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_UVLI) {
		aw8693xs_op_clean_status(aw_haptic);
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		(void)oplus_haptic_track_uvlo(HAPTIC_UVLO_MODE_TRACK, reg_val, "aw8693xs_enter_uvlo_mode");
#endif
		aw_dev_err("chip uvlo int error");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_OCDI) {
		aw8693xs_op_clean_status(aw_haptic);
		aw_dev_err("chip over current int error");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_OTI) {
		aw8693xs_op_clean_status(aw_haptic);
		aw_dev_err("chip over temperature int error");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_DONEI) {
		aw8693xs_op_clean_status(aw_haptic);
		aw_dev_info("chip playback done");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_FF_AFI) {
		aw_dev_info("%s:aw_haptic rtp mode fifo almost full!", __func__);
	}

	if (reg_val & AW8693XS_BIT_SYSINT_FF_AEI)
		ret = 0;

	return ret;
}

#else
static int aw8693xs_get_irq_state(struct aw_haptic *aw_haptic)
{
	int ret = 0;
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);
	aw_dev_dbg("reg SYSINT=0x%02X", reg_val);

	if (reg_val & AW8693XS_BIT_SYSINT_BST_SCPI) {
		ret = AW_IRQ_BST_SCP;
		aw_dev_err("chip scp int error");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_BST_OVPI) {
		ret = AW_IRQ_BST_OVP;
		aw_dev_err("chip ov int error");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_UVLI) {
		ret = AW_IRQ_UVLO;
		aw_dev_err("chip uvlo int error");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_OCDI) {
		ret = AW_IRQ_OCD;
		aw_dev_err("chip over current int error");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_OTI) {
		ret = AW_IRQ_OT;
		aw_dev_err("chip over temperature int error");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_DONEI) {
		ret = AW_IRQ_DONE;
		aw_dev_info("chip playback done");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_FF_AFI) {
		ret = AW_IRQ_ALMOST_FULL;
		aw_dev_info("aw_haptic rtp mode fifo almost full!");
	}

	if (reg_val & AW8693XS_BIT_SYSINT_FF_AEI)
		ret = AW_IRQ_ALMOST_EMPTY;

	return ret;
}
#endif

static int aw8693xs_read_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint32_t f0_reg = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_CONTRD22, reg_val, AW_I2C_BYTE_ONE);
	if (reg_val[0] == 0) {
		aw_haptic->f0 = 0;
		aw_dev_err("read_f0 error, 0x85=0");
		return -ERANGE;
	}

#ifdef AW_LRA_F0_DEFAULT
	/* lra_f0 */
	i2c_r_bytes(aw_haptic, AW8693XS_REG_CONTRD14, reg_val, AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | reg_val[1];
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_dev_err("lra_f0 is error, f0_reg = 0");
		return -ERANGE;
	}
	aw_haptic->f0 = AW8693XS_F0_FORMULA(f0_reg);
	aw_dev_info("lra_f0 = %d", aw_haptic->f0);
//#else
	/* cont_f0 */
	i2c_r_bytes(aw_haptic, AW8693XS_REG_CONTRD16, reg_val, AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | reg_val[1];
	if (!f0_reg) {
		aw_haptic->cont_f0 = 0;
		aw_dev_err("cont_f0 is error, f0_reg = 0");
		return -ERANGE;
	}
	aw_haptic->cont_f0 = AW8693XS_F0_FORMULA(f0_reg);
	aw_dev_info("cont_f0 = %d", aw_haptic->cont_f0);
#endif
	return 0;
}

static int aw8693xs_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t brk_en_default = 0;
	uint8_t cont_config[3] = {0};
	int drv_width = 0;
	int f_pre = 0;
	int ret = 0;

	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw8693xs_stop(aw_haptic);
	/* set cont f0 hw vbat_ref */
	aw8693xs_vbat_ref_config(aw_haptic, AW_CONT_F0_VBAT_REFER);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG2,
				 AW8693XS_BIT_DETCFG2_D2S_GAIN_MASK,
				 aw_haptic->info.d2s_gain);
	/* f0 calibrate work mode */
	aw8693xs_play_mode(aw_haptic, AW_CONT_MODE);
	/* enable f0 detect */
	aw8693xs_f0_detect(aw_haptic, true);
	/* cont config */
	i2c_w_bits(aw_haptic, AW8693XS_REG_CONTCFG6,
				 AW8693XS_BIT_CONTCFG6_TRACK_EN_MASK,
				 (aw_haptic->info.is_enabled_track_en << 7));
	/* enable auto break */
	i2c_r_bytes(aw_haptic, AW8693XS_REG_PLAYCFG3, &reg_val, AW_I2C_BYTE_ONE);
	brk_en_default = reg_val & (~AW8693XS_BIT_PLAYCFG3_BRK_EN_MASK);
#ifdef OPLUS_FEATURE_CHG_BASIC
	aw8693xs_f0_auto_brake_mode(aw_haptic, true);
#else
	aw8693xs_auto_brake_mode(aw_haptic, true);
#endif
	/* f0 driver level & time */
	i2c_w_bits(aw_haptic, AW8693XS_REG_CONTCFG6,
				 AW8693XS_BIT_CONTCFG6_DRV1_LVL_MASK,
				 aw_haptic->info.cont_drv1_lvl);
	cont_config[0] = aw_haptic->info.cont_drv2_lvl;
	cont_config[1] = aw_haptic->info.cont_drv1_time;
	cont_config[2] = aw_haptic->info.cont_drv2_time;
	i2c_w_bytes(aw_haptic, AW8693XS_REG_CONTCFG7, cont_config, AW_I2C_BYTE_THREE);
	/* TRACK_MARGIN */
	if (!aw_haptic->info.cont_track_margin) {
		aw_dev_err("aw_haptic->info.cont_track_margin = 0");
	} else {
		i2c_w_bytes(aw_haptic, AW8693XS_REG_CONTCFG11,
				     &aw_haptic->info.cont_track_margin, AW_I2C_BYTE_ONE);
	}
	/* DRV_WIDTH */
	if (!aw_haptic->info.f0_pre)
		return -ERANGE;
	drv_width = AW_DRV_WIDTH_FARMULA(aw_haptic->info.f0_pre, aw_haptic->info.cont_brk_gain,
					 aw_haptic->info.cont_track_margin);
	if (drv_width < AW_REG_VALUE_MIN)
		drv_width = AW_REG_VALUE_MIN;
	else if (drv_width > AW_REG_VALUE_MAX)
		drv_width = AW_REG_VALUE_MAX;
	cont_config[1] = (uint8_t)drv_width;

	f_pre = AW8693XS_F_PRE_FORMULA(aw_haptic->info.f0_pre);
	cont_config[0] = (f_pre > AW_REG_VALUE_MAX) ? AW_REG_VALUE_MAX : f_pre;
	i2c_w_bytes(aw_haptic, AW8693XS_REG_CONTCFG2, cont_config, AW_I2C_BYTE_TWO);

	/* play go */
	aw8693xs_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw8693xs_wait_enter_standby(aw_haptic);
	ret = aw8693xs_read_f0(aw_haptic);
	aw8693xs_f0_detect(aw_haptic, false);
	/* recover auto break config */
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (brk_en_default)
		aw8693xs_f0_auto_brake_mode(aw_haptic, true);
	else
		aw8693xs_f0_auto_brake_mode(aw_haptic, false);
#else
	if (brk_en_default)
		aw8693xs_auto_brake_mode(aw_haptic, true);
	else
		aw8693xs_auto_brake_mode(aw_haptic, false);
#endif
	/* recover default hw vbat_ref */
	aw8693xs_vbat_ref_config(aw_haptic, AW_VBAT_REFER);

	return ret;
}

static int aw8693xs_get_trim_osc_code(struct aw_haptic *aw_haptic)
{
	uint8_t osc_trim_s = 0;
	int ret = 0;

	aw8693xs_reg_unlock(aw_haptic, true);
	i2c_r_bytes(aw_haptic, AW8693XS_REG_EFCFG9, &osc_trim_s, AW_I2C_BYTE_ONE);
	/* Get default osc cali data */
	aw_haptic->osc_cali_data = osc_trim_s;
	aw8693xs_reg_unlock(aw_haptic, false);
	if (osc_trim_s > 128)
		ret = osc_trim_s - 256;
	else
		ret = osc_trim_s;

	aw_dev_info("osc_trim_s = %d", ret);

	return ret;
}

static int aw8693xs_ram_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t brk_en_default = 0;
	int ret = 0;

	if (!aw_haptic->ram_init) {
		aw_dev_err("ram init failed, not allow to play!");
		return -ERANGE;
	}
	if (aw_haptic->ram.ram_num < AW_RAM_GET_F0_SEQ) {
		aw_dev_err("miss ram get f0 waveform!");
		return -ERANGE;
	}
	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw8693xs_stop(aw_haptic);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG2,
				 AW8693XS_BIT_DETCFG2_D2S_GAIN_MASK,
				 aw_haptic->info.d2s_gain);
	/* f0 calibrate work mode */
	aw8693xs_play_mode(aw_haptic, AW_RAM_MODE);
	/* enable f0 detect */
	aw8693xs_f0_detect(aw_haptic, true);
	/* enable auto break */
	i2c_r_bytes(aw_haptic, AW8693XS_REG_PLAYCFG3, &reg_val, AW_I2C_BYTE_ONE);
	brk_en_default = reg_val & (~AW8693XS_BIT_PLAYCFG3_BRK_EN_MASK);
#ifdef OPLUS_FEATURE_CHG_BASIC
	aw8693xs_f0_auto_brake_mode(aw_haptic, true);
	aw8693xs_set_bst_vol(aw_haptic, aw_haptic->vmax);
#else
	aw8693xs_auto_brake_mode(aw_haptic, true);
	aw8693xs_set_bst_vol(aw_haptic, 8000);
#endif
	aw8693xs_set_wav_seq(aw_haptic, 0x00, AW_RAM_GET_F0_SEQ);
	aw8693xs_set_wav_seq(aw_haptic, 0x01, 0x00);
	aw8693xs_set_wav_loop(aw_haptic, 0x00, 0x02);
	/* play go */
	aw8693xs_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw8693xs_wait_enter_standby(aw_haptic);
	ret = aw8693xs_read_f0(aw_haptic);
	/* restore default config */
	aw8693xs_set_bst_vol(aw_haptic, aw_haptic->vmax);
	aw8693xs_f0_detect(aw_haptic, false);
	/* recover auto break config */
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (brk_en_default)
		aw8693xs_f0_auto_brake_mode(aw_haptic, true);
	else
		aw8693xs_f0_auto_brake_mode(aw_haptic, false);
#else
	if (brk_en_default)
		aw8693xs_auto_brake_mode(aw_haptic, true);
	else
		aw8693xs_auto_brake_mode(aw_haptic, false);
#endif

	return ret;
}

static uint8_t aw8693xs_rtp_get_fifo_afs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSST, &reg_val, AW_I2C_BYTE_ONE);
	reg_val &= AW8693XS_BIT_SYSST_FF_AFS;
	reg_val >>= 3;

	return reg_val;
}

static uint8_t aw8693xs_rtp_get_fifo_aes(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSST, &reg_val, AW_I2C_BYTE_ONE);
	reg_val &= AW8693XS_BIT_SYSST_FF_AES;
	reg_val >>= 4;

	return reg_val;
}

static uint8_t aw8693xs_get_osc_status(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSST2, &state, AW_I2C_BYTE_ONE);
	state &= AW8693XS_BIT_SYSST2_FF_EMPTY;

	return state;
}

static int aw8693xs_select_d2s_gain(uint8_t reg)
{
	int d2s_gain = 0;

	switch (reg) {
	case AW8693XS_BIT_DETCFG2_D2S_GAIN_1:
		d2s_gain = 1;
		break;
	case AW8693XS_BIT_DETCFG2_D2S_GAIN_2:
		d2s_gain = 2;
		break;
	case AW8693XS_BIT_DETCFG2_D2S_GAIN_4:
		d2s_gain = 4;
		break;
	case AW8693XS_BIT_DETCFG2_D2S_GAIN_8:
		d2s_gain = 8;
		break;
	case AW8693XS_BIT_DETCFG2_D2S_GAIN_10:
		d2s_gain = 10;
		break;
	case AW8693XS_BIT_DETCFG2_D2S_GAIN_16:
		d2s_gain = 16;
		break;
	case AW8693XS_BIT_DETCFG2_D2S_GAIN_20:
		d2s_gain = 20;
		break;
	case AW8693XS_BIT_DETCFG2_D2S_GAIN_40:
		d2s_gain = 40;
		break;
	default:
		d2s_gain = -1;
		break;
	}

	return d2s_gain;
}

static void aw8693xs_get_lra_resistance(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t reg_array[2] = {0};
	uint8_t adc_fs_default = 0;
	uint8_t d2s_gain_default = 0;
	int d2s_gain = 0;
	uint32_t lra_code = 0;

	aw8693xs_stop(aw_haptic);
	aw8693xs_ram_init(aw_haptic, true);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG2,
				 AW8693XS_BIT_DETCFG2_DET_SEQ0_MASK,
				 AW8693XS_BIT_DETCFG2_DET_SEQ0_RL);
	i2c_r_bytes(aw_haptic,  AW8693XS_REG_DETCFG1, &reg_val, AW_I2C_BYTE_ONE);
	adc_fs_default = reg_val & (~AW8693XS_BIT_DETCFG1_ADC_FS_MASK);
	i2c_r_bytes(aw_haptic, AW8693XS_REG_DETCFG2, &reg_val, AW_I2C_BYTE_ONE);
	d2s_gain_default = reg_val & (~AW8693XS_BIT_DETCFG2_D2S_GAIN_MASK);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG2,
					AW8693XS_BIT_DETCFG2_D2S_GAIN_MASK,
					AW8693XS_BIT_DETCFG2_D2S_GAIN_10);
	d2s_gain = aw8693xs_select_d2s_gain(AW8693XS_BIT_DETCFG2_D2S_GAIN_10);
	if (d2s_gain <= 0) {
		aw_dev_err("d2s_gain is error");
		return;
	}
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1,
				 (AW8693XS_BIT_DETCFG1_ADC_FS_MASK &
				  AW8693XS_BIT_DETCFG1_DET_GO_MASK),
				 (AW8693XS_BIT_DETCFG1_ADC_FS_96KHZ |
				  AW8693XS_BIT_DETCFG1_DET_GO_ON));
	usleep_range(3000, 3500);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1,
				 AW8693XS_BIT_DETCFG1_DET_GO_MASK,
				 AW8693XS_BIT_DETCFG1_DET_GO_OFF);
	/* restore default config*/
	aw8693xs_ram_init(aw_haptic, false);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1,
				 AW8693XS_BIT_DETCFG1_ADC_FS_MASK, adc_fs_default);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG2,
				 AW8693XS_BIT_DETCFG2_D2S_GAIN_MASK, d2s_gain_default);
	i2c_r_bytes(aw_haptic, AW8693XS_REG_DETRD1, reg_array, AW_I2C_BYTE_TWO);
	lra_code = ((reg_array[0] & (~AW8693XS_BIT_DETRD1_AVG_DATA_H_MASK)) << 8) + reg_array[1];
	aw_haptic->lra = AW8693XS_LRA_FORMULA(lra_code, d2s_gain);
}

static void aw8693xs_set_repeat_seq(struct aw_haptic *aw_haptic, uint8_t seq)
{
	aw8693xs_set_wav_seq(aw_haptic, 0x00, seq);
	aw8693xs_set_wav_loop(aw_haptic, 0x00, AW8693XS_WAVLOOP_INIFINITELY);
}

static void aw8693xs_get_vbat(struct aw_haptic *aw_haptic)
{
	uint8_t reg_array[2] = {0};
	uint32_t vbat_code = 0;

	aw8693xs_stop(aw_haptic);
	aw8693xs_ram_init(aw_haptic, true);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG2, AW8693XS_BIT_DETCFG2_DET_SEQ0_MASK,
				 AW8693XS_BIT_DETCFG2_DET_SEQ0_VBAT);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1, AW8693XS_BIT_DETCFG1_DET_GO_MASK,
				 AW8693XS_BIT_DETCFG1_DET_GO_ON);
	usleep_range(3000, 3500);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1, AW8693XS_BIT_DETCFG1_DET_GO_MASK,
				 AW8693XS_BIT_DETCFG1_DET_GO_OFF);
	i2c_r_bytes(aw_haptic, AW8693XS_REG_DETRD1, reg_array, AW_I2C_BYTE_TWO);
	aw8693xs_ram_init(aw_haptic, false);

	vbat_code = ((reg_array[0] & (~AW8693XS_BIT_DETRD1_AVG_DATA_H_MASK)) << 8) + reg_array[1];
	aw_haptic->vbat = AW8693XS_VBAT_FORMULA(vbat_code);

	if (aw_haptic->vbat > AW_VBAT_MAX) {
		aw_haptic->vbat = AW_VBAT_MAX;
		aw_dev_info("vbat max limit = %d", aw_haptic->vbat);
	}
	if (aw_haptic->vbat < AW_VBAT_MIN) {
		aw_haptic->vbat = AW_VBAT_MIN;
		aw_dev_info("vbat min limit = %d", aw_haptic->vbat);
	}
	aw_dev_info("awinic->vbat=%dmV, vbat_code=0x%02X", aw_haptic->vbat, vbat_code);
}

static ssize_t aw8693xs_get_reg(struct aw_haptic *aw_haptic, ssize_t len, char *buf)
{
	uint8_t i = 0;
	uint8_t reg_array[AW8693XS_REG_CONTRD30 + 1] = {0};

	i2c_r_bytes(aw_haptic, AW8693XS_REG_RSTCFG, reg_array, AW8693XS_REG_RTPDATA);
	i2c_r_bytes(aw_haptic, (AW8693XS_REG_RTPDATA + 1),
			    &reg_array[AW8693XS_REG_RTPDATA + 1],
			    (AW8693XS_REG_RAMDATA - AW8693XS_REG_RTPDATA - 1));
	i2c_r_bytes(aw_haptic, (AW8693XS_REG_RAMDATA + 1),
			    &reg_array[AW8693XS_REG_RAMDATA + 1],
			    (AW8693XS_REG_CONTRD30 - AW8693XS_REG_RAMDATA));

	for (i = 0; i <= AW8693XS_REG_CONTRD30; i++)
		if ((i != AW8693XS_REG_RTPDATA) && (i != AW8693XS_REG_RAMDATA))
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02X=0x%02X\n", i, reg_array[i]);

	return len;
}

static int aw8693xs_offset_os_code(struct aw_haptic *aw_haptic, enum d2s_gain_sel d2s_gain_sel)
{
	int os_code = 0;
	int d2s_gain = 0;
	uint8_t reg_val[2] = { 0 };
	uint8_t d2s_reg_val = 0;

	if (d2s_gain_sel == AW8693XS_D2S_GAIN) {
		d2s_reg_val = aw_haptic->info.d2s_gain;
		i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG2,
					AW8693XS_BIT_DETCFG2_D2S_GAIN_MASK, d2s_reg_val);
	} else if (d2s_gain_sel == AW8693XS_BEMF_D2S_GAIN) {
		d2s_reg_val = aw_haptic->info.bemf_d2s_gain;
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL6,
				AW8693XS_BIT_SYSCTRL6_BEMF_D2S_GAIN_MASK,
				d2s_reg_val);
	}

	aw8693xs_ram_init(aw_haptic, true);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG2, AW8693XS_BIT_DETCFG2_DET_SEQ0_MASK,
				 AW8693XS_BIT_DETCFG2_DET_SEQ0_OS);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1, AW8693XS_BIT_DETCFG1_DET_GO_MASK,
				 AW8693XS_BIT_DETCFG1_DET_GO_ON);
	if (d2s_gain_sel == AW8693XS_BEMF_D2S_GAIN) {
		usleep_range(3000, 3500);
		i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1, AW8693XS_BIT_DETCFG1_DET_GO_MASK,
					AW8693XS_BIT_DETCFG1_DET_GO_ON_BEMF_D2S_GAIN);
	}
	usleep_range(3000, 3500);
	i2c_w_bits(aw_haptic, AW8693XS_REG_DETCFG1, AW8693XS_BIT_DETCFG1_DET_GO_MASK,
				AW8693XS_BIT_DETCFG1_DET_GO_OFF);
	i2c_r_bytes(aw_haptic, AW8693XS_REG_DETRD1, reg_val, AW_I2C_BYTE_TWO);
	aw8693xs_ram_init(aw_haptic, false);
	d2s_gain = aw8693xs_select_d2s_gain(d2s_reg_val);
	if (d2s_gain <= 0) {
		aw_dev_err("d2s_gain is error");
		return -20;
	}
	os_code = ((reg_val[0] & (~AW8693XS_BIT_DETRD1_AVG_DATA_H_MASK)) << 8) | reg_val[1];
	os_code = AW8693XS_OS_FORMULA(os_code, d2s_gain);

	return os_code;
}

static void aw8693xs_offset_cali(struct aw_haptic *aw_haptic)
{
	int os_code = 0;

	os_code = aw8693xs_offset_os_code(aw_haptic, AW8693XS_D2S_GAIN);
	aw_dev_info("first os_code = %d", os_code);
	if (os_code > 15 || os_code < -15) {
		aw_dev_info("first offset calibration out of range");
		sw_reset(aw_haptic);
	}

	os_code = aw8693xs_offset_os_code(aw_haptic, AW8693XS_BEMF_D2S_GAIN);
	aw_dev_info("second os_code = %d", os_code);
	if (os_code > 15 || os_code < -15) {
		aw_dev_info("second offset calibration out of range");
		return;
	}

	return;
}

static void aw8693xs_trig_init(struct aw_haptic *aw_haptic)
{
	aw_dev_info("enter");
#ifdef OPLUS_FEATURE_CHG_BASIC
	if(!aw_haptic->info.trig_gain) {
		i2c_w_bytes(aw_haptic, AW8693XS_REG_TRGCFG9,
			&aw_haptic->info.trig_gain, AW_I2C_BYTE_ONE);
	}
#endif
	if (aw_haptic->info.is_enabled_one_wire) {
		aw_dev_info("one wire is enabled");
		aw8693xs_one_wire_init(aw_haptic);
	} else {
		aw8693xs_trig1_param_init(aw_haptic);
		aw8693xs_trig1_param_config(aw_haptic);
	}
	aw8693xs_trig2_param_init(aw_haptic);
	aw8693xs_trig3_param_init(aw_haptic);
	aw8693xs_trig2_param_config(aw_haptic);
	aw8693xs_trig3_param_config(aw_haptic);
}

#ifdef AW_CHECK_RAM_DATA
static int aw8693xs_check_ram_data(struct aw_haptic *aw_haptic,
				  uint8_t *cont_data, uint8_t *ram_data, uint32_t len)
{
	int i = 0;

	for (i = 0; i < len; i++) {
		if (ram_data[i] != cont_data[i]) {
			aw_dev_err("check ramdata error, addr=0x%04x, ram_data=0x%02x, file_data=0x%02x",
				i, ram_data[i], cont_data[i]);
			return -ERANGE;
		}
	}

	return 0;
}
#endif

static int aw8693xs_container_update(struct aw_haptic *aw_haptic,
				    struct aw_haptic_container *awinic_cont)
{
	uint8_t ae_addr_h = 0;
	uint8_t af_addr_h = 0;
	uint8_t ae_addr_l = 0;
	uint8_t af_addr_l = 0;
	uint8_t reg_array[3] = {0};
	uint32_t shift = 0;
	int i = 0;
	int len = 0;
	int ret = 0;

#ifdef AW_CHECK_RAM_DATA
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};
#endif
	mutex_lock(&aw_haptic->lock);
	aw_haptic->ram.baseaddr_shift = 2;
	aw_haptic->ram.ram_shift = 4;
	/* Enter standby mode */
	aw8693xs_stop(aw_haptic);
	/* RAMINIT Enable */
	aw8693xs_ram_init(aw_haptic, true);
	/* base addr */
	shift = aw_haptic->ram.baseaddr_shift;
	aw_haptic->ram.base_addr = (awinic_cont->data[0 + shift] << 8) |
				   (awinic_cont->data[1 + shift]);
	aw_dev_info("base_addr = %d", aw_haptic->ram.base_addr);

	/* set FIFO_AE and FIFO_AF addr */
	ae_addr_h = AW8693XS_FIFO_AE_ADDR_H(aw_haptic->ram.base_addr);
	af_addr_h = AW8693XS_FIFO_AF_ADDR_H(aw_haptic->ram.base_addr);
	reg_array[0] = ae_addr_h | af_addr_h;
	reg_array[1] = AW8693XS_FIFO_AE_ADDR_L(aw_haptic->ram.base_addr);
	reg_array[2] = AW8693XS_FIFO_AF_ADDR_L(aw_haptic->ram.base_addr);
	i2c_w_bytes(aw_haptic, AW8693XS_REG_RTPCFG3, reg_array, AW_I2C_BYTE_THREE);

	/* get FIFO_AE and FIFO_AF addr */
	i2c_r_bytes(aw_haptic, AW8693XS_REG_RTPCFG3, reg_array, AW_I2C_BYTE_THREE);
	ae_addr_h = ((reg_array[0]) & AW8693XS_BIT_RTPCFG3_FIFO_AEH) >> 4;
	ae_addr_l = reg_array[1];
	aw_dev_info("almost_empty_threshold = %d", ((ae_addr_h << 8) | ae_addr_l));
	af_addr_h = ((reg_array[0]) & AW8693XS_BIT_RTPCFG3_FIFO_AFH);
	af_addr_l = reg_array[2];
	aw_dev_info("almost_full_threshold = %d", ((af_addr_h << 8) | af_addr_l));

	aw8693xs_set_base_addr(aw_haptic);
	aw8693xs_set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_WR_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_WR_BUFFER_SIZE;

		i2c_w_bytes(aw_haptic, AW8693XS_REG_RAMDATA, &awinic_cont->data[i], len);
		i += len;
	}

#ifdef AW_CHECK_RAM_DATA
	aw8693xs_set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_RD_BUFFER_SIZE;

		i2c_r_bytes(aw_haptic, AW8693XS_REG_RAMDATA, ram_data, len);
		ret = aw8693xs_check_ram_data(aw_haptic, &awinic_cont->data[i], ram_data, len);
		if (ret < 0)
			break;
		i += len;
	}
	if (ret)
		aw_dev_err("ram data check sum error");
	else
		aw_dev_info("ram data check sum pass");
#endif
	/* RAMINIT Disable */
	aw8693xs_ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);

	return ret;
}

static unsigned long aw8693xs_get_theory_time(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t fre_val = 0;
	unsigned long theory_time = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSCTRL4, &reg_val, AW_I2C_BYTE_ONE);
	fre_val = reg_val & (~AW8693XS_BIT_SYSCTRL4_WAVDAT_MODE_MASK);
	if (fre_val == AW8693XS_BIT_SYSCTRL4_WAVDAT_8K)
		theory_time = aw_haptic->rtp_len * 1000 / 8;	/*8K*/
	if (fre_val == AW8693XS_BIT_SYSCTRL4_WAVDAT_12K)
		theory_time = aw_haptic->rtp_len * 1000 / 12;	/*12K*/
	if (fre_val == AW8693XS_BIT_SYSCTRL4_WAVDAT_24K)
		theory_time = aw_haptic->rtp_len * 1000 / 24;	/*24K*/
	if (fre_val == AW8693XS_BIT_SYSCTRL4_WAVDAT_48K)
		theory_time = aw_haptic->rtp_len * 1000 / 48;	/*48K*/
	aw_dev_info("%s: microsecond:%ld  theory_time = %ld\n",
		    __func__, aw_haptic->microsecond, theory_time);
	return theory_time;
}

static void aw8693xs_parse_dt(struct device *dev, struct aw_haptic *aw_haptic,
			     struct device_node *np)
{
	uint8_t duration_time[3];
	uint8_t trig_config_temp[24];
	uint32_t val = 0;
	int i =0;
	uint32_t max_boost_voltage = 0;
	uint8_t vmax[VMAX_GAIN_NUM_V2];
	uint8_t gain[VMAX_GAIN_NUM_V2];

	val = of_property_read_u8(np, "aw8693xs_gain_bypass", &aw_haptic->info.gain_bypass);
	if (val != 0)
		aw_dev_info("aw8693xs_gain_bypass not found");

	val = of_property_read_u32(np, "f0_pre", &aw_haptic->info.f0_pre);
	if (val != 0)
		aw_dev_info("f0_pre not found");

	val = of_property_read_u8(np, "aw8693xs_f0_cali_percent", &aw_haptic->info.f0_cali_percent);
	if (val != 0)
		aw_dev_info("aw8693xs_f0_cali_percent not found");

	val = of_property_read_u8(np, "aw8693xs_cont_drv1_lvl", &aw_haptic->info.cont_drv1_lvl);
	if (val != 0)
		aw_dev_info("aw8693xs_cont_drv1_lvl not found");

	val = of_property_read_u32(np, "aw8693xs_cont_lra_vrms", &aw_haptic->info.cont_lra_vrms);
	if (val != 0)
		aw_dev_info("aw8693xs_cont_lra_vrms not found");

	val = of_property_read_u8(np, "aw8693xs_cont_drv1_time", &aw_haptic->info.cont_drv1_time);
	if (val != 0)
		aw_dev_info("aw8693xs_cont_drv1_time not found");

	val = of_property_read_u8(np, "aw8693xs_cont_drv2_time", &aw_haptic->info.cont_drv2_time);
	if (val != 0)
		aw_dev_info("aw8693xs_cont_drv2_time not found");

	val = of_property_read_u8(np, "aw8693xs_cont_brk_time", &aw_haptic->info.cont_brk_time);
	if (val != 0)
		aw_dev_info("aw8693xs_cont_brk_time not found");

	val = of_property_read_u8(np, "aw8693xs_cont_track_margin",
				  &aw_haptic->info.cont_track_margin);
	if (val != 0)
		aw_dev_info("aw8693xs_cont_track_margin not found");

	val = of_property_read_u8(np, "aw8693xs_cont_brk_gain", &aw_haptic->info.cont_brk_gain);
	if (val != 0)
		aw_dev_info("aw8693xs_cont_brk_gain not found");

	val = of_property_read_u8(np, "aw8693xs_d2s_gain", &aw_haptic->info.d2s_gain);
	if (val != 0)
		aw_dev_info("aw8693xs_d2s_gain not found");

	val = of_property_read_u8(np, "aw8693xs_bemf_d2s_gain", &aw_haptic->info.bemf_d2s_gain);
	if (val != 0)
		aw_dev_info("aw8693xs_bemf_d2s_gain not found");

	val = of_property_read_u8_array(np, "aw8693xs_duration_time", duration_time,
					ARRAY_SIZE(duration_time));
	if (val != 0)
		aw_dev_info("aw8693xs_duration_time not found");
	else
		memmove(aw_haptic->info.duration_time, duration_time, sizeof(duration_time));

	val = of_property_read_u8_array(np, "aw8693xs_trig_config",
					trig_config_temp, ARRAY_SIZE(trig_config_temp));
	if (val != 0)
		aw_dev_info("aw8693xs_trig_config not found");
	else
		memmove(aw_haptic->info.trig_cfg, trig_config_temp, sizeof(trig_config_temp));

	aw_haptic->info.is_enabled_track_en = of_property_read_bool(np,
						"aw8693xs_is_enabled_track_en");
	aw_dev_info("track en = %d", aw_haptic->info.is_enabled_track_en);

	aw_haptic->info.is_enabled_inter_brake = of_property_read_bool(np,
						"aw8693xs_is_enabled_inter_brake");
	aw_dev_info("inter brake = %d", aw_haptic->info.is_enabled_inter_brake);

	aw_haptic->info.is_enabled_low_power = of_property_read_bool(np,
						"aw8693xs_is_enabled_low_power");
	aw_dev_info("lower power = %d", aw_haptic->info.is_enabled_low_power);

	aw_haptic->info.is_enabled_vbat_pro = of_property_read_bool(np,
						"aw8693xs_is_enabled_vbat_pro");
	aw_dev_info("vbat pro = %d", aw_haptic->info.is_enabled_vbat_pro);

	aw_haptic->info.is_enabled_auto_bst = of_property_read_bool(np,
						"aw8693xs_is_enabled_auto_bst");
	aw_dev_info("auto bst = %d", aw_haptic->info.is_enabled_auto_bst);

	aw_haptic->info.is_enabled_one_wire = of_property_read_bool(np,
						 "aw8693xs_is_enabled_one_wire");
	aw_dev_info("onewire = %d", aw_haptic->info.is_enabled_one_wire);

	/* UVLO_ADJ(0-1.9v,1-2v,2-2.1v,3-2.2v,4-2.3v,5-2.4v,6-2.5v,7-2.6v) */
	val = of_property_read_u8(np, "aw8693xs_uvlo_adj_default", &aw_haptic->info.uvlo_adj_default);
	if (val != 0)
		aw_dev_info("aw8693xs_uvlo_adj_default not found");

	/* pro0_ipeak(0-1.5v,1-1.75v,2-2v,3-2.25v,4-2.5v,5-2.75v,6-3v.....,12-4.5v,13-4.75v) */
	val = of_property_read_u8(np, "aw8693xs_set_pro0_ipeak", &aw_haptic->info.set_pro0_ipeak);
	if (val != 0)
		aw_dev_info("aw8693xs_set_pro0_ipeak not found");

	/* pro1_uvlo(0-1.9v,1-2v,2-2.1v,3-2.2v,4-2.3v,5-2.4v,6-2.5v.....,14-3.3v,15-3.4v) */
	val = of_property_read_u8(np, "aw8693xs_set_pro1_uvlo", &aw_haptic->info.set_pro1_uvlo);
	if (val != 0)
		aw_dev_info("aw8693xs_set_pro1_uvlo not found");

	/* pro2_uvlo(0-1.9v,1-2v,2-2.1v,3-2.2v,4-2.3v,5-2.4v,6-2.5v.....,14-3.3v,15-3.4v) */
	val = of_property_read_u8(np, "aw8693xs_set_pro2_uvlo", &aw_haptic->info.set_pro2_uvlo);
	if (val != 0)
		aw_dev_info("aw8693xs_set_pro2_uvlo not found");

	/* pro3_uvlo(0-1.9v,1-2v,2-2.1v,3-2.2v,4-2.3v,5-2.4v,6-2.5v.....,14-3.3v,15-3.4v) */
	val = of_property_read_u8(np, "aw8693xs_set_pro3_uvlo", &aw_haptic->info.set_pro3_uvlo);
	if (val != 0)
		aw_dev_info("aw8693xs_set_pro3_uvlo not found");

	if (of_property_read_u32(np, "vbat_pro1_bst_default", &val))
		aw_haptic->info.vbat_pro1_bst_default = AW8693XS_VBAT_PRO1_BST_DEFAULT;
	else
		aw_haptic->info.vbat_pro1_bst_default = val;
	aw_dev_info("%s: vbat_pro1_bst_default=%d\n", __func__, aw_haptic->info.vbat_pro1_bst_default);

	if (of_property_read_u32(np, "vbat_pro1_bst_ipeak_default", &val))
		aw_haptic->info.vbat_pro1_bst_ipeak_default = AW8693XS_VBAT_PRO1_BST_IPEAK_DEFAULT;
	else
		aw_haptic->info.vbat_pro1_bst_ipeak_default = val;
	aw_dev_info("%s: vbat_pro1_bst_ipeak_default=%d\n", __func__, aw_haptic->info.vbat_pro1_bst_ipeak_default);

	if (of_property_read_u32(np, "vbat_pro2_bst_default", &val))
		aw_haptic->info.vbat_pro2_bst_default = AW8693XS_VBAT_PRO2_BST_DEFAULT;
	else
		aw_haptic->info.vbat_pro2_bst_default = val;
	aw_dev_info("%s: vbat_pro2_bst_default=%d\n", __func__, aw_haptic->info.vbat_pro2_bst_default);

	if (of_property_read_u32(np, "vbat_pro2_bst_ipeak_default", &val))
		aw_haptic->info.vbat_pro2_bst_ipeak_default = AW8693XS_VBAT_PRO2_BST_IPEAK_DEFAULT;
	else
		aw_haptic->info.vbat_pro2_bst_ipeak_default = val;
	aw_dev_info("%s: vbat_pro2_bst_ipeak_default=%d\n", __func__, aw_haptic->info.vbat_pro2_bst_ipeak_default);

	if (of_property_read_u32(np, "vbat_pro3_gain", &val))
		aw_haptic->info.vbat_pro3_gain = AW8693XS_PRO3_GAIN_0;
	else
		aw_haptic->info.vbat_pro3_gain = val;
	aw_dev_info("%s: vbat_pro3_gain=%d\n", __func__, aw_haptic->info.vbat_pro3_gain);

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (of_property_read_u32(np, "aw8693xs_boost_voltage", &max_boost_voltage))
		aw_haptic->max_boost_vol = AW8693XS_BST_VOL_DEFAULT;
	else
		aw_haptic->max_boost_vol = (uint8_t)max_boost_voltage;
	aw_dev_info("%s: boost_voltage=%d\n", __func__, aw_haptic->max_boost_vol);

	val = of_property_read_u8(np, "aw8693xs_trig_gain", &aw_haptic->info.trig_gain);
	if (val != 0)
		aw_dev_info("aw8693xs_trig_gain not found");

	val = of_property_read_u8_array(np, "aw8693xs_vmax",
							vmax, ARRAY_SIZE(vmax));
	if (val != 0) {
		aw_dev_info("aw8693xs_vmax not found");
	} else {
		for (i = 0; i < ARRAY_SIZE(vmax); i++) {
			vmax_map[i].vmax = vmax[i];
			aw_dev_info("aw8693xs vmax_map vmax: 0x%x vmax: 0x%x", vmax_map[i].vmax, vmax[i]);
		}
	}
	val = of_property_read_u8_array(np, "aw8693xs_gain",
						gain, ARRAY_SIZE(gain));
	if (val != 0) {
		aw_dev_info("aw8693xs_gain not found");
	} else {
		for (i = 0; i < ARRAY_SIZE(gain); i++) {
			vmax_map[i].gain = gain[i];
			aw_dev_info("aw8693xs vmax_map gain: 0x%x gain: 0x%x", vmax_map[i].gain, gain[i]);
		}
	}
#endif
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static int aw8693xs_convert_level_to_vmax(struct aw_haptic *aw_haptic, struct vmax_map *map, int val)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(vmax_map); i++) {
		if (val == vmax_map[i].level) {
			map->vmax = vmax_map[i].vmax;
			map->gain = vmax_map[i].gain;
			break;
		}
	}
	if (i == ARRAY_SIZE(vmax_map)) {
		map->vmax = vmax_map[i - 1].vmax;
		map->gain = vmax_map[i - 1].gain;
	}
	if (map->vmax > aw_haptic->max_boost_vol)
		map->vmax = aw_haptic->max_boost_vol;

	return i;
}

static void aw8693xs_haptic_value_init(struct aw_haptic *aw_haptic)
{
	if (aw_haptic->device_id == DEVICE_ID_0816) {
		aw_haptic->info.f0_pre = AW8693XS_0816_F0_PRE;
		aw_haptic->info.cont_drv2_time = AW8693XS_0815_CONT_DRV2_TIME;
		aw_haptic->info.cont_lra_vrms = AW8693XS_0816_LRA_VRMS;
		aw_haptic->info.cont_track_margin = AW8693XS_0816_CONT_TRACK_MARGIN;
	} else {
		aw_haptic->info.f0_pre = AW8693XS_0815_F0_PRE;
		aw_haptic->info.cont_drv2_time = AW8693XS_0815_CONT_DRV2_TIME;
		aw_haptic->info.cont_lra_vrms = AW8693XS_LRA_VRMS;
		aw_haptic->info.cont_track_margin = AW8693XS_0815_CONT_TRACK_MARGIN;
	}

	aw_haptic->info.f0_cali_percent = AW8693XS_0815_F0_CALI_PERCEN;
	aw_haptic->info.cont_drv1_lvl = AW8693XS_0815_CONT_DRV1_LVL;
	aw_haptic->info.cont_drv1_time = AW8693XS_0815_CONT_DRV1_TIME;
	if (!aw_haptic->info.cont_brk_time)
		aw_haptic->info.cont_brk_time = AW8693XS_0815_CONT_BRK_TIME;
	if (!aw_haptic->info.cont_brk_gain)
		aw_haptic->info.cont_brk_gain = AW8693XS_0815_CONT_BRK_GAIN;

	aw_haptic->info.gain_bypass = AW8693XS_GAIN_BYPASS;
	if (!aw_haptic->info.bemf_d2s_gain)
		aw_haptic->info.bemf_d2s_gain = AW8693XS_BEMF_D2S_GAIN_DEFAULT;

	if (!aw_haptic->info.d2s_gain)
		aw_haptic->info.d2s_gain = AW8693XS_D2S_GAIN_DEFAULT;
	aw_haptic->info.bst_vol_default = AW8693XS_BST_VOL_DEFAULT;

	if (!aw_haptic->info.uvlo_adj_default)
		aw_haptic->info.uvlo_adj_default = AW8693XS_VBAT_UVLO_ADJ_DEFAULT;
	if (!aw_haptic->info.set_pro1_uvlo)
		aw_haptic->info.set_pro1_uvlo = AW8693XS_VBAT_PRO_UVLO_FORMULA(AW8693XS_VBAT_PRO1_UVLO_DEFAULT);
	if (!aw_haptic->info.set_pro2_uvlo)
		aw_haptic->info.set_pro2_uvlo = AW8693XS_VBAT_PRO_UVLO_FORMULA(AW8693XS_VBAT_PRO2_UVLO_DEFAULT);
	if (!aw_haptic->info.set_pro3_uvlo)
		aw_haptic->info.set_pro3_uvlo = AW8693XS_VBAT_PRO_UVLO_FORMULA(AW8693XS_VBAT_PRO3_UVLO_DEFAULT);
	if (!aw_haptic->info.set_pro0_ipeak)
		aw_haptic->info.set_pro0_ipeak = AW8693XS_IPEAK_FORMULA(AW8693XS_VBAT_PRO0_BST_IPEAK_DEFAUL);

	aw_haptic->vbat_pro_params[0] = aw_haptic->info.vbat_pro1_bst_default;
	aw_haptic->vbat_pro_params[1] = aw_haptic->info.vbat_pro1_bst_ipeak_default;
	aw_haptic->vbat_pro_params[2] = aw_haptic->info.vbat_pro2_bst_default;
	aw_haptic->vbat_pro_params[3] = aw_haptic->info.vbat_pro2_bst_ipeak_default;
	aw_haptic->vbat_pro_params[4] = aw_haptic->info.vbat_pro3_gain;
}

static bool aw8693xs_check_trig_status(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW8693XS_REG_SYSER, &reg_val, AW_I2C_BYTE_ONE);
	if (reg_val & AW8693XS_BIT_SYSER_TRIG1_EVENT || reg_val & AW8693XS_BIT_SYSER_TRIG23_EVENT)
		return true;

	return false;
}
#endif

static void aw8693xs_set_vbat_pro_params(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = { 0 };
	uint8_t i = 0;

	/* vbat pro1 and vbat pro2 bstmax and bst ipeak set */
	for (i = 0; i < 2; i++) {
		uint32_t bst_max = aw_haptic->vbat_pro_params[2 * i];
		uint32_t ipeak = aw_haptic->vbat_pro_params[2 * i + 1];

		bst_max = (bst_max < AW8693XS_PRO_BSTMAX_MIN) ? AW8693XS_PRO_BSTMAX_MIN : bst_max;
		bst_max = (bst_max > AW8693XS_PRO_BSTMAX_MAX) ? AW8693XS_PRO_BSTMAX_MAX : bst_max;
		ipeak = (ipeak < AW8693XS_PRO_IPEAK_MIN) ? AW8693XS_PRO_IPEAK_MIN : ipeak;
		ipeak = (ipeak > AW8693XS_PRO_IPEAK_MAX) ? AW8693XS_PRO_IPEAK_MAX : ipeak;
		reg_val[0] = AW8693XS_BST_MAX_FORMULA(bst_max);
		reg_val[1] = AW8693XS_IPEAK_FORMULA(ipeak);
		aw_dev_info("set bst_max = %u mV, reg_val[0] = 0x%02x", bst_max, reg_val[0]);
		aw_dev_info("set ipeak = %u mA, reg_val[1] = 0x%02x", ipeak, reg_val[1]);

		if (i == 0) {
			reg_val[0] = (reg_val[0] << 4) | reg_val[1];
			i2c_w_bytes(aw_haptic, AW8693XS_REG_SYSCTRL8, &reg_val[0], AW_I2C_BYTE_ONE);
		} else if (i == 1) {
			reg_val[0] = (reg_val[0] << 4) | reg_val[1];
			i2c_w_bytes(aw_haptic, AW8693XS_REG_SYSCTRL9, &reg_val[0], AW_I2C_BYTE_ONE);
		}
	}

	/* vbat pro3 output gain set */
	switch (aw_haptic->vbat_pro_params[4]) {
		case AW8693XS_PRO3_GAIN_1:
			reg_val[0] = AW8693XS_BIT_SYSCTRL7_PRO3_GAIN_1;
			aw_dev_info("set vbat pro3_gain = 1");
			break;
		case AW8693XS_PRO3_GAIN_3_4:
			reg_val[0] = AW8693XS_BIT_SYSCTRL7_PRO3_GAIN_0P75;
			aw_dev_info("set vbat pro3_gain = 0.75");
			break;
		case AW8693XS_PRO3_GAIN_1_2:
			reg_val[0] = AW8693XS_BIT_SYSCTRL7_PRO3_GAIN_0P5;
			aw_dev_info("set vbat pro3_gain = 0.5");
			break;
		case AW8693XS_PRO3_GAIN_0:
			reg_val[0] = AW8693XS_BIT_SYSCTRL7_PRO3_GAIN_0;
			aw_dev_info("set vbat pro3_gain = 0");
			break;
		default:
			reg_val[0] = AW8693XS_BIT_SYSCTRL7_PRO3_GAIN_0P5;
			aw_dev_err("vbat pro3 gain error, default pro3 gain = 1/2");
			break;
	}
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL7,
		    AW8693XS_BIT_SYSCTRL7_PRO3_GAIN_MASK, reg_val[0]);
}

static void aw8693xs_enable_vbat_pro(struct aw_haptic *aw_haptic, bool enable)
{
	if (enable) {
		aw_dev_info("vbat pro enable");
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL7,
				AW8693XS_BIT_SYSCTRL7_VBAT_PRO_MASK,
				AW8693XS_BIT_SYSCTRL7_VBAT_PRO_ENABLE);
	} else {
		aw_dev_info("vbat pro disable");
		i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL11,
				AW8693XS_BIT_SYSCTRL7_VBAT_PRO_MASK,
				AW8693XS_BIT_SYSCTRL7_VBAT_PRO_DISABLE);
	}
}

static void aw8693xs_misc_para_init(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[8] = {0};
	uint32_t drv2_lvl = 0;

	/* Get osc trim val */
	aw_haptic->osc_trim_s = aw8693xs_get_trim_osc_code(aw_haptic);
	/* Set I2C broadcast addr */
	reg_val[0] = aw_haptic->i2c->addr;
	i2c_w_bytes(aw_haptic, AW8693XS_REG_SYSCTRL5, reg_val, AW_I2C_BYTE_ONE);
	/* Cont drv2 lvl */
	drv2_lvl = AW8693XS_DRV2_LVL_FORMULA(aw_haptic->info.f0_pre,
					    aw_haptic->info.cont_lra_vrms);
	aw_haptic->info.cont_drv2_lvl = (drv2_lvl > AW_DRV2_LVL_MAX) ? AW_DRV2_LVL_MAX : drv2_lvl;

	/* Get vmax */
#ifdef OPLUS_FEATURE_CHG_BASIC
	aw_haptic->vmax = aw_haptic->max_boost_vol;
#else
	aw_haptic->vmax = aw_haptic->info.bst_vol_default;
#endif
	/* Get gain */
	i2c_r_bytes(aw_haptic, AW8693XS_REG_PLAYCFG2, reg_val, AW_I2C_BYTE_ONE);
	aw_haptic->gain = reg_val[0];
	/* Get wave_seq */
	i2c_r_bytes(aw_haptic, AW8693XS_REG_WAVCFG1, reg_val, AW_I2C_BYTE_EIGHT);
	aw_haptic->index = reg_val[0];
	memmove(aw_haptic->seq, reg_val, AW_SEQUENCER_SIZE);
	/* Set gain_bypass */
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL4,
				 AW8693XS_BIT_SYSCTRL4_GAIN_BYPASS_MASK,
				 aw_haptic->info.gain_bypass);

	i2c_w_bits(aw_haptic, AW8693XS_REG_CONTCFG5,
				 AW8693XS_BIT_CONTCFG5_BRK_GAIN_MASK, aw_haptic->info.cont_brk_gain);
	i2c_w_bytes(aw_haptic, AW8693XS_REG_CONTCFG10,
			     &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);
	aw8693xs_protect_config(aw_haptic, AW8693XS_BIT_PWMCFG4_PRTIME_DEFAULT_VALUE,
			       AW8693XS_BIT_PWMCFG3_PRLVL_DEFAULT_VALUE);
	/* Set vbat ref voltage */
	aw8693xs_vbat_ref_config(aw_haptic, AW_VBAT_REFER);

	/* set INT_BRK */
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL6,
				 AW8693XS_BIT_SYSCTRL6_INT_BRK_MASK,
				 aw_haptic->info.is_enabled_inter_brake << 6);
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL6,
				 AW8693XS_BIT_SYSCTRL6_INTBRK_MD_MASK,
				 AW8693XS_BIT_SYSCTRL6_INTBRK_MD_DELAY);
	/* set DRV_SWITCH */
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL6,
				 AW8693XS_BIT_SYSCTRL6_DRV_SWITCH_MASK,
				 AW8693XS_BIT_SYSCTRL6_DRV_SWITCH_ON);
	/* uvlo adj default */
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL12,
				 AW8693XS_BIT_SYSCTRL12_UVLO_ADJ_MASK,
				 aw_haptic->info.uvlo_adj_default << 4);
	/*set_pro1_uvlo*/
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL11,
				 AW8693XS_BIT_SYSCTRL11_PRO1_UVLO_MASK,
				 aw_haptic->info.set_pro1_uvlo << 4);
	/*set_pro2_uvlo*/
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL11,
				 AW8693XS_BIT_SYSCTRL11_PRO2_UVLO_MASK,
				 aw_haptic->info.set_pro2_uvlo);
	/*set_pro3_uvlo*/
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL12,
				 AW8693XS_BIT_SYSCTRL12_PRO3_UVLO_MASK,
				 aw_haptic->info.set_pro3_uvlo);

	reg_val[0] = AW8693XS_BIT_SYSCTRL7_INIT_VAL;
	reg_val[1] = AW8693XS_BIT_ANACFG17_INIT_VAL;
	reg_val[2] = AW8693XS_BIT_ANACFG20_INIT_VAL;
	/* low power mode and IPEAK 4A */
	i2c_w_bits(aw_haptic, AW8693XS_REG_SYSCTRL7,
				 AW8693XS_BIT_SYSCTRL7_PRO0_IPEAK_MASK,
				 aw_haptic->info.set_pro0_ipeak);

	aw8693xs_reg_unlock(aw_haptic, true);
	i2c_w_bytes(aw_haptic, AW8693XS_REG_ANACFG17, &reg_val[1], AW_I2C_BYTE_ONE);
	i2c_w_bytes(aw_haptic, AW8693XS_REG_ANACFG20, &reg_val[2], AW_I2C_BYTE_ONE);
	aw8693xs_reg_unlock(aw_haptic, false);
	/* set vbat pro params */
	aw8693xs_set_vbat_pro_params(aw_haptic);
	/* set vbat pro enable or disable */
	aw8693xs_enable_vbat_pro(aw_haptic, aw_haptic->info.is_enabled_vbat_pro);

}

/******************************************************
 *
 * Extern function : sysfs attr
 *
 ******************************************************/
 static ssize_t vbat_pro_params_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d\n",
		aw_haptic->vbat_pro_params[0], aw_haptic->vbat_pro_params[1],
		aw_haptic->vbat_pro_params[2], aw_haptic->vbat_pro_params[3],
		aw_haptic->vbat_pro_params[4]);

	return len;
}

static ssize_t vbat_pro_params_store(struct device *dev,
	 struct device_attribute *attr,
	 const char *buf, size_t count)
{
	int param[5] = {0};
	cdev_t *cdev = dev_get_drvdata(dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	if (sscanf(buf, "%d %d %d %d %d", &param[0], &param[1], &param[2], &param[3], &param[4]) == 5) {
		aw_haptic->vbat_pro_params[0] = param[0];
		aw_haptic->vbat_pro_params[1] = param[1];
		aw_haptic->vbat_pro_params[2] = param[2];
		aw_haptic->vbat_pro_params[3] = param[3];
		aw_haptic->vbat_pro_params[4] = param[4];
	}

	aw8693xs_set_vbat_pro_params(aw_haptic);
	return count;
}

static ssize_t cont_drv_lvl_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_lvl = 0x%02X, cont_drv2_lvl = 0x%02X\n",
			aw_haptic->info.cont_drv1_lvl, aw_haptic->info.cont_drv2_lvl);

	return len;
}

static ssize_t cont_drv_lvl_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	uint32_t databuf[2] = {0};
	uint8_t reg_array[2] = {0};
	cdev_t *cdev = dev_get_drvdata(dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_lvl = databuf[0];
		aw_haptic->info.cont_drv2_lvl = databuf[1];
		mutex_lock(&aw_haptic->lock);
		i2c_r_bytes(aw_haptic, AW8693XS_REG_CONTCFG6, reg_array, AW_I2C_BYTE_ONE);
		reg_array[0] &= AW8693XS_BIT_CONTCFG6_DRV1_LVL_MASK;
		reg_array[0] |= aw_haptic->info.cont_drv1_lvl;
		reg_array[1] = aw_haptic->info.cont_drv2_lvl;
		i2c_w_bytes(aw_haptic, AW8693XS_REG_CONTCFG6, reg_array, AW_I2C_BYTE_TWO);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t cont_drv_time_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_time = 0x%02X, cont_drv2_time = 0x%02X\n",
			aw_haptic->info.cont_drv1_time, aw_haptic->info.cont_drv2_time);

	return len;
}

static ssize_t cont_drv_time_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint8_t reg_array[2] = {0};
	uint32_t databuf[2] = {0};
	cdev_t *cdev = dev_get_drvdata(dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_time = databuf[0];
		aw_haptic->info.cont_drv2_time = databuf[1];
		reg_array[0] = (uint8_t)aw_haptic->info.cont_drv1_time;
		reg_array[1] = (uint8_t)aw_haptic->info.cont_drv2_time;
		mutex_lock(&aw_haptic->lock);
		i2c_w_bytes(aw_haptic, AW8693XS_REG_CONTCFG8, reg_array, AW_I2C_BYTE_TWO);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t cont_brk_time_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	len += snprintf(buf + len, PAGE_SIZE - len, "cont_brk_time = 0x%02X\n",
			aw_haptic->info.cont_brk_time);

	return len;
}

static ssize_t cont_brk_time_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	rc = kstrtou8(buf, 0, &aw_haptic->info.cont_brk_time);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	i2c_w_bytes(aw_haptic, AW8693XS_REG_CONTCFG10,
			     &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t trig_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t i = 0;
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	for (i = 0; i < AW_TRIG_NUM; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"trig%d: trig_level=%d, trig_polar=%d, pos_enable=%d, pos_sequence=%d, neg_enable=%d, neg_sequence=%d trig_brk=%d, trig_bst=%d\n",
				i + 1,
				aw_haptic->trig[i].trig_level,
				aw_haptic->trig[i].trig_polar,
				aw_haptic->trig[i].pos_enable,
				aw_haptic->trig[i].pos_sequence,
				aw_haptic->trig[i].neg_enable,
				aw_haptic->trig[i].neg_sequence,
				aw_haptic->trig[i].trig_brk,
				aw_haptic->trig[i].trig_bst);
	}

	return len;
}

static ssize_t trig_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	uint32_t databuf[9] = {0};
	cdev_t *cdev = dev_get_drvdata(dev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	if (sscanf(buf, "%u %u %u %u %u %u %u %u %u", &databuf[0], &databuf[1],
		   &databuf[2], &databuf[3], &databuf[4], &databuf[5],
		   &databuf[6], &databuf[7], &databuf[8]) == 9) {
		aw_dev_info("%d, %d, %d, %d, %d, %d, %d, %d, %d",
			databuf[0], databuf[1], databuf[2], databuf[3],
			databuf[4], databuf[5], databuf[6], databuf[7],
			databuf[8]);
		if (databuf[0] < 1 || databuf[0] > 3) {
			aw_dev_info("input trig_num out of range!");
			return count;
		}
		if (databuf[0] == 1 && aw_haptic->info.is_enabled_one_wire) {
			aw_dev_info("trig1 pin used for one wire!");
			return count;
		}
		if (!aw_haptic->ram_init) {
			aw_dev_err("ram init failed, not allow to play!");
			return count;
		}
		if (databuf[4] > aw_haptic->ram.ram_num ||
		    databuf[6] > aw_haptic->ram.ram_num) {
			aw_dev_err("input seq value out of range!");
			return count;
		}
		databuf[0] -= 1;

		aw_haptic->trig[databuf[0]].trig_level = databuf[1];
		aw_haptic->trig[databuf[0]].trig_polar = databuf[2];
		aw_haptic->trig[databuf[0]].pos_enable = databuf[3];
		aw_haptic->trig[databuf[0]].pos_sequence = databuf[4];
		aw_haptic->trig[databuf[0]].neg_enable = databuf[5];
		aw_haptic->trig[databuf[0]].neg_sequence = databuf[6];
		aw_haptic->trig[databuf[0]].trig_brk = databuf[7];
		aw_haptic->trig[databuf[0]].trig_bst = databuf[8];
		mutex_lock(&aw_haptic->lock);
		switch (databuf[0]) {
		case 0:
			aw8693xs_trig1_param_config(aw_haptic);
			break;
		case 1:
			aw8693xs_trig2_param_config(aw_haptic);
			break;
		case 2:
			aw8693xs_trig3_param_config(aw_haptic);
			break;
		}
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t rtp_auto_sin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	ssize_t len = 0;
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp_cnt = %u\n", aw_haptic->rtp_cnt);

	return len;
}

static ssize_t rtp_auto_sin_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
#else
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic, vib_dev);
#endif
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		aw_dev_err("kstrtouint fail");
		return rc;
	}
	mutex_lock(&aw_haptic->lock);
	if ((val > 0) && (val < aw_haptic->rtp_num)) {
		aw_haptic->state = 1;
		aw_haptic->rtp_file_num = val;
		i2c_w_bits(aw_haptic, AW8693XS_REG_RTPCFG1,
					 AW8693XS_BIT_RTPCFG1_RTP_AUTO_SIN_MASK,
					 AW8693XS_BIT_RTPCFG1_RTP_AUTO_SIN_ENABLE);
	} else if (val == 0) {
		aw_haptic->state = 0;
		i2c_w_bits(aw_haptic, AW8693XS_REG_RTPCFG1,
					 AW8693XS_BIT_RTPCFG1_RTP_AUTO_SIN_MASK,
					 AW8693XS_BIT_RTPCFG1_RTP_AUTO_SIN_DISABLE);
	} else {
		aw_haptic->state = 0;
		aw_dev_err("input number error:%d", val);
	}
	mutex_unlock(&aw_haptic->lock);
	queue_work(aw_haptic->work_queue, &aw_haptic->rtp_work);

	return count;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static ssize_t trig_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", aw_haptic->trig_gain);

	return 0;
}

static ssize_t trig_gain_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		aw_dev_err("%s:trig kstrtouint fail\n", __func__);
		return rc;
	}
	aw_dev_info("%s: trig val [%d] \n", __func__, val);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->trig_gain = val;
	aw8693xs_set_trig_gain(aw_haptic, aw_haptic->trig_gain);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t check_trig_status_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
	bool status = false;
	size_t count;

	status = aw8693xs_check_trig_status(aw_haptic);
	count = snprintf(buf, PAGE_SIZE, "%d\n", status);
	aw_dev_info("%s: status=%d\n", __func__, status);

	return count;
}

static ssize_t check_trig_status_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct haptic_common_data *oh =  container_of(cdev, struct haptic_common_data, cdev);
	struct aw_haptic *aw_haptic = oh->chip_data;
	int val;
	int rc;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_dev_info("%s: value=%d\n", __func__, val);

	if (val == 0) {
		aw8693xs_check_trig_status(aw_haptic);
	}

	return count;
}
#endif

static DEVICE_ATTR_RW(vbat_pro_params);
static DEVICE_ATTR_RW(cont_drv_lvl);
static DEVICE_ATTR_RW(cont_drv_time);
static DEVICE_ATTR_RW(cont_brk_time);
static DEVICE_ATTR_RW(trig);
static DEVICE_ATTR_RW(rtp_auto_sin);
#ifdef OPLUS_FEATURE_CHG_BASIC
static DEVICE_ATTR_RW(trig_gain);
static DEVICE_ATTR_RW(check_trig_status);
#endif

static struct attribute *aw8693xs_vibrator_attributes[] = {
	&dev_attr_vbat_pro_params.attr,
	&dev_attr_cont_drv_lvl.attr,
	&dev_attr_cont_drv_time.attr,
	&dev_attr_cont_brk_time.attr,
	&dev_attr_trig.attr,
	&dev_attr_rtp_auto_sin.attr,
#ifdef OPLUS_FEATURE_CHG_BASIC
	&dev_attr_trig_gain.attr,
	&dev_attr_check_trig_status.attr,
#endif
	NULL
};

static struct attribute_group aw8693xs_vibrator_attribute_group = {
	.attrs = aw8693xs_vibrator_attributes
};

static int aw8693xs_creat_node(struct haptic_common_data *oh)
{
	int ret = 0;

	ret = sysfs_create_group(&oh->cdev.dev->kobj,
				 &aw8693xs_vibrator_attribute_group);
	if (ret < 0) {
		aw_dev_err("error create aw8693xs sysfs attr files");
		return ret;
	}
	return 0;
}

struct aw_haptic_func aw8693xs_func_list = {
	.play_stop = aw8693xs_stop,
	.ram_init = aw8693xs_ram_init,
	.get_vbat = aw8693xs_get_vbat,
	.creat_node = aw8693xs_creat_node,
	.get_f0 = aw8693xs_get_f0,
	.ram_get_f0 = aw8693xs_ram_get_f0,
	.cont_config = aw8693xs_cont_config,
	.offset_cali = aw8693xs_offset_cali,
	.get_irq_state = aw8693xs_get_irq_state,
	.check_qualify = aw8693xs_check_qualify,
	.juge_rtp_going = aw8693xs_judge_rtp_going,
	.set_bst_peak_cur = aw8693xs_set_bst_peak_cur,
	.get_theory_time = aw8693xs_get_theory_time,
	.get_lra_resistance = aw8693xs_get_lra_resistance,
	.set_pwm = aw8693xs_set_pwm,
	.play_mode = aw8693xs_play_mode,
	.set_bst_vol = aw8693xs_set_bst_vol,
	.interrupt_setup = aw8693xs_interrupt_setup,
	.set_repeat_seq = aw8693xs_set_repeat_seq,
	.auto_bst_enable = aw8693xs_auto_bst_enable,
	.vbat_mode_config = aw8693xs_vbat_mode_config,
	.set_wav_seq = aw8693xs_set_wav_seq,
	.set_wav_loop = aw8693xs_set_wav_loop,
	.set_ram_addr = aw8693xs_set_ram_addr,
	.set_rtp_data = aw8693xs_set_rtp_data,
	.container_update = aw8693xs_container_update,
	.protect_config = aw8693xs_protect_config,
	.parse_dt = aw8693xs_parse_dt,
	.trig_init = aw8693xs_trig_init,
	.irq_clear = aw8693xs_irq_clear,
	.get_wav_loop = aw8693xs_get_wav_loop,
	.play_go = aw8693xs_play_go,
	.misc_para_init = aw8693xs_misc_para_init,
	.set_rtp_aei = aw8693xs_set_rtp_aei,
	.set_gain = aw8693xs_set_gain,
	.upload_lra = aw8693xs_upload_lra,
	.bst_mode_config = aw8693xs_bst_mode_config,
	.get_reg = aw8693xs_get_reg,
	.get_prctmode = aw8693xs_get_prctmode,
	.get_trim_lra = aw8693xs_get_trim_lra,
	.get_ram_data = aw8693xs_get_ram_data,
	.get_first_wave_addr = aw8693xs_get_first_wave_addr,
	.get_glb_state = aw8693xs_get_glb_state,
	.get_osc_status = aw8693xs_get_osc_status,
	.rtp_get_fifo_afs = aw8693xs_rtp_get_fifo_afs,
	.rtp_get_fifo_aes = aw8693xs_rtp_get_fifo_aes,
	.get_wav_seq = aw8693xs_get_wav_seq,
	.get_bemf_peak = aw8693xs_get_bemf_peak,
#ifdef AW_SND_SOC_CODEC
	.snd_soc_init = aw8693xs_snd_soc_init,
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
	.haptic_value_init = aw8693xs_haptic_value_init,
	.get_chip_state = aw8693xs_get_chip_state,
	.convert_level_to_vmax = aw8693xs_convert_level_to_vmax,
	.dump_rtp_regs = aw8693xs_dump_rtp_regs,
#endif
};
