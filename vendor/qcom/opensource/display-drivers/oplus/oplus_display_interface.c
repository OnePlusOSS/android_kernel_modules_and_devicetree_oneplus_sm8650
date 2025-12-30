/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_interface.c
** Description : oplus display interface
** Version : 1.0
** Date : 2022/05/30
** Author : Display
******************************************************************/
#include <drm/drm_print.h>
#include <drm/drm_connector.h>
#include <linux/msm_drm_notify.h>
#include <linux/module.h>
#include "oplus_bl.h"
#include "oplus_display_interface.h"
#include "oplus_display_panel_common.h"
#include "oplus_display_private_api.h"
#include "oplus_display_high_frequency_pwm.h"
#include "sde_color_processing.h"
#include "sde_encoder_phys.h"
#include "sde_trace.h"

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "oplus_adfr.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

bool refresh_rate_change = false;
extern bool oplus_pwm_onepluse_switch;
extern int dc_apollo_enable;
extern int oplus_dimlayer_hbm;
extern unsigned int oplus_dsi_log_type;
extern int oplus_debug_max_brightness;
bool already_readid = false;
struct panel_id panel_id;
extern u32 bl_lvl;
bool is_lhbm_panel = false;
extern bool g_gamma_regs_read_done;

int oplus_panel_cmd_print(struct dsi_panel *panel, enum dsi_cmd_set_type type)
{
	u32 count;

	count = panel->cur_mode->priv_info->cmd_sets[type].count;
	if (count == 0) {
		LCD_DEBUG("[%s] dsi_cmd: %s is null\n", panel->oplus_priv.vendor_name,
				cmd_set_prop_map[type]);
		return 0;
	}

	switch (type) {
	case DSI_CMD_SET_ROI:
	case DSI_CMD_ESD_SWITCH_PAGE:
	case DSI_CMD_SKIPFRAME_DBV:
	case DSI_CMD_DEFAULT_SWITCH_PAGE:
		/* Do nothing */
		break;
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	case DSI_CMD_ADFR_MIN_FPS_0:
	case DSI_CMD_ADFR_MIN_FPS_1:
	case DSI_CMD_ADFR_MIN_FPS_2:
	case DSI_CMD_ADFR_MIN_FPS_3:
	case DSI_CMD_ADFR_MIN_FPS_4:
	case DSI_CMD_ADFR_MIN_FPS_5:
	case DSI_CMD_ADFR_MIN_FPS_6:
	case DSI_CMD_ADFR_MIN_FPS_7:
	case DSI_CMD_ADFR_MIN_FPS_8:
	case DSI_CMD_ADFR_MIN_FPS_9:
	case DSI_CMD_ADFR_MIN_FPS_10:
	case DSI_CMD_ADFR_MIN_FPS_11:
	case DSI_CMD_ADFR_MIN_FPS_12:
	case DSI_CMD_ADFR_MIN_FPS_13:
	case DSI_CMD_ADFR_MIN_FPS_14:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_0:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_1:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_2:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_3:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_4:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_5:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_6:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_7:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_8:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_9:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_10:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_11:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_12:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_13:
	case DSI_CMD_HPWM_ADFR_MIN_FPS_14:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_0:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_1:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_2:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_3:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_4:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_5:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_6:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_7:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_8:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_9:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_10:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_11:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_12:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_13:
	case DSI_CMD_BIGDC_ADFR_MIN_FPS_14:
	case DSI_CMD_ADFR_FAKEFRAME:
		if (panel->cur_mode->priv_info->oplus_adfr_idle_min_fps_log) {
			ADFR_DEBUG("[%s] dsi_cmd: %s, count=%d\n", panel->oplus_priv.vendor_name,
					cmd_set_prop_map[type], count);
			panel->cur_mode->priv_info->oplus_adfr_idle_min_fps_log = false;
		} else {
			ADFR_INFO("[%s] dsi_cmd: %s, count=%d\n", panel->oplus_priv.vendor_name,
					cmd_set_prop_map[type], count);
		}
		break;
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */
	default:
		LCD_INFO("[%s] dsi_cmd: %s, count=%d\n", panel->oplus_priv.vendor_name,
				cmd_set_prop_map[type], count);
		break;
	}

	return 0;
}

int oplus_panel_cmd_switch(struct dsi_panel *panel, enum dsi_cmd_set_type *type)
{
	enum dsi_cmd_set_type type_store = *type;
	u32 count;
	unsigned int last_refresh_rate = panel->last_refresh_rate;

	/* switch the command when pwm turbo is enabled */
	if (oplus_panel_pwm_turbo_is_enabled(panel)) {
		switch (*type) {
		case DSI_CMD_HBM_ON:
			*type = DSI_CMD_PWM_TURBO_HBM_ON;
			break;
		case DSI_CMD_HBM_OFF:
			*type = DSI_CMD_PWM_TURBO_HBM_OFF;
			break;
		case DSI_CMD_AOR_ON:
			*type = DSI_CMD_PWM_TURBO_AOR_ON;
			break;
		case DSI_CMD_AOR_OFF:
			*type = DSI_CMD_PWM_TURBO_AOR_OFF;
			break;
		case DSI_CMD_SET_TIMING_SWITCH:
			*type = DSI_CMD_PWM_TURBO_TIMING_SWITCH;
			break;
		default:
			break;
		}
	}

	if (panel->pwm_params.pwm_switch_support_dc
		&& !panel->pwm_params.pwm_switch_support_extend_mode) {
		if(oplus_panel_pwm_onepulse_is_enabled(panel)) {
			switch (*type) {
			case DSI_CMD_SET_TIMING_SWITCH:
				*type = DSI_CMD_TIMMING_PWM_SWITCH_ONEPULSE;
				break;
			case DSI_CMD_SET_NOLP:
				*type = DSI_CMD_SET_NOLP_ONEPULSE;
				break;
			default:
				break;
			}
		}
	} else if (panel->pwm_params.pwm_switch_support_extend_mode) {
		if (*type == DSI_CMD_SET_OFF) {
			panel->pwm_params.oplus_aod_mutual_fps_flag = false;
		}
		if (panel->pwm_params.oplus_dynamic_pulse == ONE_ONE_PULSE
			|| (panel->pwm_params.oplus_dynamic_pulse == ONE_EIGHTEEN_PULSE
			&& panel->bl_config.bl_level > 1162)) {
			switch (*type) {
			case DSI_CMD_SET_TIMING_SWITCH:
				*type = DSI_CMD_TIMMING_PWM_SWITCH_ONEPULSE;
				break;
			case DSI_CMD_SET_NOLP:
				panel->pwm_params.oplus_aod_mutual_fps_flag = true;
				panel->pwm_params.aod_off_timestamp = ktime_get();
				LCD_DEBUG("aod_off_timestamp:%lu\n", ktime_to_ms(panel->pwm_params.aod_off_timestamp));
				*type = DSI_CMD_SET_NOLP_ONEPULSE;
				break;
			case DSI_CMD_SET_OFF:
				panel->pwm_params.oplus_aod_mutual_fps_flag = false;
				break;
			default:
				break;
			}
		}
		panel->pwm_params.oplus_pulse_mutual_fps_flag--;
		if (panel->pwm_params.oplus_pulse_mutual_fps_flag < PULSE_MUTUAL_FPS_LOWER_LIMIT) {
			panel->pwm_params.oplus_pulse_mutual_fps_flag = 0;
		}
	} else {
		/* switch the command when pwm onepulse is enabled */
		if (oplus_panel_pwm_onepulse_is_enabled(panel)&&
		  		!panel->pwm_params.directional_onepulse_switch) {
			switch (*type) {
			case DSI_CMD_PWM_SWITCH_HIGH:
				if (!strcmp(panel->name, "AA545 P 3 A0005 dsc cmd mode panel") ||
						!strcmp(panel->name, "AC090 P 3 A0005 dsc cmd mode panel")) {
					oplus_pwm_onepluse_switch = true;
					return 0;
				}
				*type = DSI_CMD_PWM_SWITCH_ONEPULSE;
				break;
			case DSI_CMD_PWM_SWITCH_LOW:
				*type = DSI_CMD_PWM_SWITCH_ONEPULSE_LOW;
				break;
			case DSI_CMD_SET_TIMING_SWITCH:
				if (panel->pwm_params.oplus_pwm_switch_state == PWM_SWITCH_DC_STATE)
					*type = DSI_CMD_TIMMING_PWM_SWITCH_ONEPULSE;
				break;
			case DSI_CMD_HBM_ON:
				*type = DSI_CMD_HBM_ON_ONEPULSE;
				break;
			default:
				break;
			}
		}
		if (panel->pwm_params.directional_onepulse_switch) {
			if(oplus_panel_pwm_onepulse_is_used(panel)) {
				switch (*type) {
				case DSI_CMD_SET_TIMING_SWITCH: {
					*type = DSI_CMD_TIMMING_PWM_SWITCH_ONEPULSE;
					break;
				}
				default:
					break;
				}
			}
		}
	}

	if (*type == DSI_CMD_SET_ON && oplus_panel_id_compatibility(panel)) {
		*type = DSI_CMD_SET_COMPATIBILITY_ON;
	}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		if (oplus_ofp_need_to_do_aod_off_compensation()) {
			switch (*type) {
			case DSI_CMD_SET_NOLP:
				if (panel->cur_mode->priv_info->cmd_sets[DSI_CMD_AOD_OFF_COMPENSATION].count) {
					*type = DSI_CMD_AOD_OFF_COMPENSATION;
				}
				break;
			case DSI_CMD_SET_NOLP_ONEPULSE:
				if (panel->cur_mode->priv_info->cmd_sets[DSI_CMD_AOD_OFF_COMPENSATION_ONEPULSE].count) {
					*type = DSI_CMD_AOD_OFF_COMPENSATION_ONEPULSE;
				}
				break;
			default:
				break;
			}
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	count = panel->cur_mode->priv_info->cmd_sets[*type].count;
	if (count == 0) {
		LCD_DEBUG("[%s] %s is undefined, restore to %s\n",
				panel->oplus_priv.vendor_name,
				cmd_set_prop_map[*type],
				cmd_set_prop_map[type_store]);
		*type = type_store;
	}

	if (!strcmp(panel->oplus_priv.vendor_name , "BOE_ILI7838E")
		|| !strcmp(panel->oplus_priv.vendor_name , "AB781")
		|| !strcmp(panel->oplus_priv.vendor_name , "AB714")
		|| !strcmp(panel->oplus_priv.vendor_name , "AB715")) {
		if (*type == DSI_CMD_SET_TIMING_SWITCH || *type == DSI_CMD_TIMMING_PWM_SWITCH_ONEPULSE) {
			oplus_sde_early_wakeup(panel);
			oplus_wait_for_vsync(panel);
			if (last_refresh_rate == 60 || last_refresh_rate == 90) {
				oplus_need_to_sync_te(panel);
			} else {
				usleep_range(200, 200);
			}
		}
	}

	return 0;
}

#define BACKLIGHT_CACHE_MAX 50
void oplus_printf_backlight_cmd_log(u16 bl_lvl) {
	struct dsi_display *display;
	struct timespec64 now;
	struct tm broken_time;
	static time64_t time_last = 0;
	static u32 bl_count;
	static u16 backlight[BACKLIGHT_CACHE_MAX];
	static struct timespec64 past_times[BACKLIGHT_CACHE_MAX];

	int i = 0;
	int len = 0;
	char backlight_log_buf[1024];

	display = get_main_display();
	if(strcmp(display->panel->name, "AA545 P 3 A0005 dsc cmd mode panel"))
		return;
	ktime_get_real_ts64(&now);
	time64_to_tm(now.tv_sec, 0, &broken_time);
	if (now.tv_sec - time_last >= 60) {
		pr_info("<%s> backlight_cmd time:%02d:%02d:%02d.%03ld,cmd:%04x\n",
			display->panel->oplus_priv.vendor_name, broken_time.tm_hour, broken_time.tm_min,
			broken_time.tm_sec, now.tv_nsec / 1000000, bl_lvl);
		time_last = now.tv_sec;
	}

	backlight[bl_count] = bl_lvl;
	past_times[bl_count] = now;
	bl_count++;
	if (bl_count >= BACKLIGHT_CACHE_MAX) {
		bl_count = 0;
		memset(backlight_log_buf, 0, sizeof(backlight_log_buf));
		for (i = 0; i < BACKLIGHT_CACHE_MAX; i++) {
			time64_to_tm(past_times[i].tv_sec, 0, &broken_time);
			len += snprintf(backlight_log_buf + len, sizeof(backlight_log_buf) - len,
				"%02d:%02d:%02d.%03ld:%04x,", broken_time.tm_hour, broken_time.tm_min,
				broken_time.tm_sec, past_times[i].tv_nsec / 1000000, backlight[i]);
		}
		pr_info("<%s> len:%d backlight_cmd %s\n", display->panel->oplus_priv.vendor_name, len, backlight_log_buf);
	}
}

void oplus_ctrl_print_cmd_desc(struct dsi_ctrl *dsi_ctrl, struct dsi_cmd_desc *cmd)
{
	char buf[512];
	int len = 0;
	size_t i;
	const struct mipi_dsi_msg *msg = &cmd->msg;
	char *tx_buf = (char*)msg->tx_buf;
	u16 cmd51;
	memset(buf, 0, sizeof(buf));

	/* Packet Info */
	len += snprintf(buf, sizeof(buf) - len,  "%02X ", msg->type);
	len += snprintf(buf + len, sizeof(buf) - len, "%02X ", 0x00);
	len += snprintf(buf + len, sizeof(buf) - len, "%02X ", msg->channel);
	len += snprintf(buf + len, sizeof(buf) - len, "%02X ", (unsigned int)msg->flags);
	len += snprintf(buf + len, sizeof(buf) - len, "%02X ", cmd->post_wait_ms);
	len += snprintf(buf + len, sizeof(buf) - len, "%02X %02X ", msg->tx_len >> 8, msg->tx_len & 0x00FF);

	/* Packet Payload */
	for (i = 0 ; i < msg->tx_len ; i++) {
		len += snprintf(buf + len, sizeof(buf) - len, "%02X ", tx_buf[i]);
		/* Break to prevent show too long command */
		if (i > 160)
			break;
	}

	if (tx_buf[0] == 0x51 && msg->tx_len == 3) {
		cmd51 = (tx_buf[1] << 8) | tx_buf[2];
		oplus_printf_backlight_cmd_log(cmd51);
	}

	/* DSI_CTRL_ERR(dsi_ctrl, "%s\n", buf); */
	LCD_DEBUG_CMD("dsi_cmd: %s\n", buf);
}

int oplus_panel_init(struct dsi_panel *panel)
{
	int rc = 0;
	static bool panel_need_init = true;
	struct dsi_display *display = container_of(&panel, struct dsi_display, panel);

	if (!panel_need_init)
		return 0;

	if (!display) {
		LCD_ERR("display is null\n");
		return 0;
	}

	LCD_INFO("Send panel init dcs\n");

	mutex_lock(&panel->panel_lock);

	rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_PANEL_INIT);
	if (!rc)
		panel_need_init = false;

	mutex_unlock(&panel->panel_lock);

	/*add for panel init code compatibility*/
	oplus_panel_id_compatibility_init(display);

	return rc;
}

/*add for panel init code compatibility*/
int oplus_panel_id_compatibility_init(struct dsi_display *display)
{
	int rc = 0;
	int compatibility_cmd = DSI_OPTIMIZE_INIT_ON;
	struct dsi_panel *panel;

	if (display == NULL || display->panel == NULL) {
		return rc;
	}

	panel = display->panel;
	if(!panel->oplus_priv.panel_init_compatibility_enable) {
		return rc;
	}

	if (!already_readid) {
		mutex_lock(&display->panel->panel_lock);
		rc = oplus_display_panel_get_id_unlock(&panel_id);
		mutex_unlock(&display->panel->panel_lock);
		if (rc < 0) {
			LCD_ERR("panel id init compatibility get panel id failed!\n");
			return rc;
		}
		else {
			/* printf first read panel id */
			LCD_INFO("kernel first read panel id DA = 0x%02X, DB = 0x%02X, DC = 0x%02X\n", panel_id.DA, panel_id.DB, panel_id.DC);
			already_readid = true;
		}
	}
	if (!strcmp(panel->name, "AC172 P 3 A0023 dsc cmd mode panel")) {
		return rc;
	}
	if (already_readid && panel_id.DA == 0x3E) {
		if (panel_id.DB == 0x93) {
			/* init code common on 93 vrr*/
			compatibility_cmd = DSI_OPTIMIZE_INIT_SPLIT_ON;
		} else if (panel_id.DB == 0x94) {
			/* init code common on 94 vrr*/
			compatibility_cmd = DSI_OPTIMIZE_INIT_ON;
		} else {
			return rc;
		}
	} else {
		/* ID1 not is 3E && ID2 not is 93/94 return */
		return rc;
	}
	mutex_lock(&panel->panel_lock);
	rc = dsi_panel_tx_cmd_set(panel, compatibility_cmd);
	mutex_unlock(&panel->panel_lock);
	if (rc) {
		LCD_ERR("Send panel id compatibility init code failed! \n");
	}
	return rc;
}

/*add for panel id compatibility by qcom,mdss-dsi-on-command*/
bool oplus_panel_id_compatibility(struct dsi_panel *panel)
{
	/* power on printf panel id */
	LCD_INFO("panel id DA = 0x%02X, DB = 0x%02X, DC = 0x%02X\n", panel_id.DA, panel_id.DB, panel_id.DC);
	if (already_readid && panel_id.DA == 0x3E && panel_id.DB >= 0x97) {
		return true;
	}

	return false;
}

int oplus_panel_gpio_request(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_panel_reset_config *r_config;
	struct drm_panel_esd_config *esd_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;
	esd_config = &panel->esd_config;

	if (gpio_is_valid(r_config->panel_vout_gpio)) {
		rc = gpio_request(r_config->panel_vout_gpio, "panel_vout_gpio");
		if (rc) {
			LCD_ERR("request for panel_vout_gpio failed, rc=%d\n", rc);
			if (gpio_is_valid(r_config->panel_vout_gpio))
				gpio_free(r_config->panel_vout_gpio);
		}
	}
	if (gpio_is_valid(r_config->panel_vddr_aod_en_gpio)) {
		rc = gpio_request(r_config->panel_vddr_aod_en_gpio, "panel_vddr_aod_en_gpio");
		if (rc) {
			LCD_ERR("request for panel_vddr_aod_en_gpio failed, rc=%d\n", rc);
			if (gpio_is_valid(r_config->panel_vddr_aod_en_gpio))
				gpio_free(r_config->panel_vddr_aod_en_gpio);
		}
	}
	if (gpio_is_valid(r_config->panel_vddr_vout_gpio2)) {
		rc = gpio_request(r_config->panel_vddr_vout_gpio2, "panel_vddr_vout_gpio2");
		if (rc) {
			LCD_ERR("request for panel_vddr_vout_gpio2 failed, rc=%d\n", rc);
			if (gpio_is_valid(r_config->panel_vddr_vout_gpio2))
				gpio_free(r_config->panel_vddr_vout_gpio2);
		}
	}

	if (gpio_is_valid(esd_config->mipi_err_flag_gpio)) {
		rc = gpio_request(esd_config->mipi_err_flag_gpio , "mipi_err_flag_gpio");
		if (rc) {
			LCD_ERR("request for mipi_err_irp_gpio failed, rc=%d\n", rc);
			if (gpio_is_valid(esd_config->mipi_err_flag_gpio))
				gpio_free(esd_config->mipi_err_flag_gpio);
		}
	}

	return rc;
}

int oplus_panel_gpio_release(struct dsi_panel *panel)
{
	struct dsi_panel_reset_config *r_config;
	struct drm_panel_esd_config *esd_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;
	esd_config = &panel->esd_config;

	if (gpio_is_valid(r_config->panel_vout_gpio))
		gpio_free(r_config->panel_vout_gpio);
	if (gpio_is_valid(r_config->panel_vddr_aod_en_gpio))
		gpio_free(r_config->panel_vddr_aod_en_gpio);
	if (gpio_is_valid(r_config->panel_vddr_vout_gpio2))
		gpio_free(r_config->panel_vddr_vout_gpio2);
	if (gpio_is_valid(esd_config->mipi_err_flag_gpio))
		gpio_free(esd_config->mipi_err_flag_gpio);

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	oplus_adfr_gpio_release(panel);
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

	return 0;
}

int oplus_panel_gpio_pre_on(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	r_config = &panel->reset_config;

	if (panel->oplus_priv.gpio_pre_on &&
			gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		rc = gpio_direction_output(panel->reset_config.panel_vout_gpio, 1);
		if (rc)
			LCD_ERR("unable to set dir for panel_vout_gpio rc=%d\n", rc);
		gpio_set_value(panel->reset_config.panel_vout_gpio, 1);
	}

	return rc;
}

int oplus_panel_gpio_on(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	if (!strcmp(panel->oplus_priv.vendor_name , "A0005")
		|| !strcmp(panel->oplus_priv.vendor_name , "BOE_ILI7838E")
		|| !strcmp(panel->oplus_priv.vendor_name , "BOE_7838E")
		|| !strcmp(panel->oplus_priv.vendor_name , "A0001")
		|| !strcmp(panel->oplus_priv.vendor_name , "AB714")
		|| !strcmp(panel->oplus_priv.vendor_name , "AB715")
		|| !strcmp(panel->oplus_priv.vendor_name , "A0004")
		|| !strcmp(panel->oplus_priv.vendor_name , "A0020")
		|| !strcmp(panel->oplus_priv.vendor_name , "AB781")
		|| !strcmp(panel->oplus_priv.vendor_name , "AC223")
		|| !strcmp(panel->oplus_priv.vendor_name , "AA592")
		|| !strcmp(panel->oplus_priv.vendor_name , "A0034"))
		return 0;

	r_config = &panel->reset_config;
	if (!panel->oplus_priv.gpio_pre_on
		&& gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		rc = gpio_direction_output(panel->reset_config.panel_vout_gpio, 1);
		if (rc)
			LCD_ERR("unable to set dir for panel_vout_gpio rc=%d\n", rc);
		gpio_set_value(panel->reset_config.panel_vout_gpio, 1);
	}

	if (gpio_is_valid(panel->reset_config.panel_vddr_aod_en_gpio)) {
		rc = gpio_direction_output(panel->reset_config.panel_vddr_aod_en_gpio, 1);
		if (rc)
			LCD_ERR("unable to set dir for panel_vddr_aod_en_gpio rc=%d\n", rc);
		gpio_set_value(panel->reset_config.panel_vddr_aod_en_gpio, 1);
	}

	return rc;
}

int oplus_panel_gpio_off(struct dsi_panel *panel)
{
	struct dsi_panel_reset_config *r_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}
	if (!strcmp(panel->oplus_priv.vendor_name , "A0005")
		|| !strcmp(panel->oplus_priv.vendor_name , "BOE_ILI7838E")
		|| !strcmp(panel->oplus_priv.vendor_name , "BOE_7838E")
		|| !strcmp(panel->oplus_priv.vendor_name , "A0001")
		|| !strcmp(panel->oplus_priv.vendor_name , "AB714")
		|| !strcmp(panel->oplus_priv.vendor_name , "AB715")
		|| !strcmp(panel->oplus_priv.vendor_name , "A0004")
		|| !strcmp(panel->oplus_priv.vendor_name , "A0020")
		|| !strcmp(panel->oplus_priv.vendor_name , "AB781")
		|| !strcmp(panel->oplus_priv.vendor_name , "AC223")
		|| !strcmp(panel->oplus_priv.vendor_name , "AA592")
		|| !strcmp(panel->oplus_priv.vendor_name , "A0034"))
		return 0;

	r_config = &panel->reset_config;
	if (gpio_is_valid(panel->reset_config.panel_vout_gpio))
		gpio_set_value(panel->reset_config.panel_vout_gpio, 0);

	if (gpio_is_valid(panel->reset_config.panel_vddr_aod_en_gpio))
		gpio_set_value(panel->reset_config.panel_vddr_aod_en_gpio, 0);

	return 0;
}

int oplus_panel_vddr_on(struct dsi_display *display, const char *vreg_name)
{
	int rc = 0;

	if (!display || !display->panel) {
		LCD_ERR("display or display panel is null, power vddr failed!\n");
		return -ENODEV;
	}
	if ((!strcmp(display->panel->oplus_priv.vendor_name , "A0005")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "BOE_ILI7838E")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "BOE_7838E")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "A0001")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "AB714")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "AB715")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "A0004")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "A0020")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "AB781")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "AC223")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "A0014")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "A0034"))
		&& !strcmp(vreg_name, "vddio")) {
		if (gpio_is_valid(display->panel->reset_config.panel_vout_gpio)) {
			rc = gpio_direction_output(display->panel->reset_config.panel_vout_gpio, 1);
			if (rc)
				LCD_ERR("unable to set dir for panel_vout_gpio rc=%d\n", rc);
			gpio_set_value(display->panel->reset_config.panel_vout_gpio, 1);
		}

		if (gpio_is_valid(display->panel->reset_config.panel_vddr_vout_gpio2)) {
			rc = gpio_direction_output(display->panel->reset_config.panel_vddr_vout_gpio2, 1);
			if (rc)
				LCD_ERR("unable to set dir for panel_vddr_aod_en_gpio rc=%d\n", rc);
			gpio_set_value(display->panel->reset_config.panel_vddr_vout_gpio2, 1);
		}
		usleep_range(5000, 5100);
	}

	return rc;
}

int oplus_panel_vddr_off(struct dsi_display *display, const char *vreg_name)
{
	int rc = 0;

	if (!display || !display->panel) {
		LCD_ERR("display or display panel is null, power vddr failed!\n");
		return -ENODEV;
	}

	if ((!strcmp(display->panel->oplus_priv.vendor_name , "A0005")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "BOE_ILI7838E")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "BOE_7838E")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "A0001")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "AB714")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "AB715")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "A0004")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "A0020")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "AB781")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "AC223")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "A0014")
		|| !strcmp(display->panel->oplus_priv.vendor_name , "A0034"))
		&& !strcmp(vreg_name, "vci")) {
		if (gpio_is_valid(display->panel->reset_config.panel_vout_gpio)) {
			gpio_set_value(display->panel->reset_config.panel_vout_gpio, 0);
		}
		usleep_range(5000, 5100);
		if (gpio_is_valid(display->panel->reset_config.panel_vddr_vout_gpio2)) {
			gpio_set_value(display->panel->reset_config.panel_vddr_vout_gpio2, 0);
		}
		usleep_range(3000, 3100);
	}

	return rc;
}


int oplus_panel_gpio_parse(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils;
	struct dsi_panel_reset_config *r_config;
	struct drm_panel_esd_config *esd_config;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}
	utils = &panel->utils;
	r_config = &panel->reset_config;
	esd_config = &panel->esd_config;

	panel->reset_config.panel_vout_gpio = utils->get_named_gpio(utils->data,
								"qcom,platform-panel-vout-gpio", 0);

	if (!gpio_is_valid(panel->reset_config.panel_vout_gpio)) {
		LCD_ERR("[%s] failed get panel_vout_gpio\n", panel->oplus_priv.vendor_name);
	}
	panel->reset_config.panel_vddr_aod_en_gpio = utils->get_named_gpio(utils->data,
								"qcom,platform-panel-vddr-aod-en-gpio", 0);

	if (!gpio_is_valid(panel->reset_config.panel_vddr_aod_en_gpio)) {
		LCD_ERR("[%s] failed get panel_vddr_aod_en_gpio\n", panel->oplus_priv.vendor_name);
	}

	panel->reset_config.panel_vddr_vout_gpio2 = utils->get_named_gpio(utils->data,
								"qcom,platform-panel-vddr-vout-gpio2", 0);
	if (!gpio_is_valid(panel->reset_config.panel_vddr_vout_gpio2)) {
		LCD_ERR("[%s] failed get panel_vddr_vout_gpio2\n", panel->oplus_priv.vendor_name);
	}

	esd_config->mipi_err_flag_gpio = utils->get_named_gpio(utils->data,
								"oplus,esd_mipi_err_gpio", 0);
	if (!gpio_is_valid(esd_config->mipi_err_flag_gpio)) {
		LCD_ERR("[%s] failed get oplus,esd_mipi_err_gpio\n", panel->oplus_priv.vendor_name);
	}

	return 0;
}

int oplus_panel_parse_vsync_config(
				struct dsi_display_mode *mode,
				struct dsi_parser_utils *utils)
{
	int rc;
	struct dsi_display_mode_priv_info *priv_info;

	priv_info = mode->priv_info;

	rc = utils->read_u32(utils->data, "oplus,apollo-panel-vsync-period",
				  &priv_info->vsync_period);
	if (rc) {
		LCD_DEBUG("panel prefill lines are not defined rc=%d\n", rc);
		priv_info->vsync_period = 1000000 / mode->timing.refresh_rate;
	}

	rc = utils->read_u32(utils->data, "oplus,apollo-panel-vsync-width",
				  &priv_info->vsync_width);
	if (rc) {
		LCD_DEBUG("panel vsync width not defined rc=%d\n", rc);
		priv_info->vsync_width = priv_info->vsync_period >> 1;
	}

	rc = utils->read_u32(utils->data, "oplus,apollo-panel-async-bl-delay",
				  &priv_info->async_bl_delay);
	if (rc) {
		LCD_DEBUG("panel async backlight delay to bottom of frame was disabled rc=%d\n", rc);
		priv_info->async_bl_delay = 0;
	} else {
		if(priv_info->async_bl_delay >= priv_info->vsync_period) {
			LCD_ERR("async backlight delay value was out of vsync period\n");
			priv_info->async_bl_delay = priv_info->vsync_width;
		}
	}

	priv_info->refresh_rate = mode->timing.refresh_rate;

	LCD_INFO("vsync width = %d, vsync period = %d, refresh rate = %d\n", priv_info->vsync_width, priv_info->vsync_period, priv_info->refresh_rate);

	return 0;
}

int oplus_panel_mult_frac(int bright)
{
	int bl_lvl = 0;
	struct dsi_display *display = get_main_display();

	if (!display || !display->drm_conn || !display->drm_conn->state) {
		LCD_ERR("failed to find dsi display\n");
		return false;
	}

	if (oplus_debug_max_brightness) {
		bl_lvl = mult_frac(bright, oplus_debug_max_brightness,
			display->panel->bl_config.brightness_max_level);
	} else if (bright == 0) {
			bl_lvl = 0;
	} else {
		if (display->panel->oplus_priv.bl_remap && display->panel->oplus_priv.bl_remap_count) {
			int i = 0;
			int count = display->panel->oplus_priv.bl_remap_count;
			struct oplus_brightness_alpha *lut = display->panel->oplus_priv.bl_remap;

			for (i = 0; i < display->panel->oplus_priv.bl_remap_count; i++) {
				if (display->panel->oplus_priv.bl_remap[i].brightness >= bright)
					break;
			}

			if (i == 0)
				bl_lvl = lut[0].alpha;
			else if (i == count)
				bl_lvl = lut[count - 1].alpha;
			else
				bl_lvl = oplus_interpolate(bright, lut[i-1].brightness,
						lut[i].brightness, lut[i-1].alpha, lut[i].alpha);
		} else if (bright > display->panel->bl_config.brightness_normal_max_level) {
			bl_lvl = oplus_interpolate(bright,
					display->panel->bl_config.brightness_normal_max_level,
					display->panel->bl_config.brightness_max_level,
					display->panel->bl_config.bl_normal_max_level,
					display->panel->bl_config.bl_max_level);
		} else {
			bl_lvl = mult_frac(bright, display->panel->bl_config.bl_normal_max_level,
					display->panel->bl_config.brightness_normal_max_level);
		}
	}

	return bl_lvl;
}

int oplus_panel_event_data_notifier_trigger(struct dsi_panel *panel,
		enum panel_event_notification_type notif_type,
		u32 data,
		bool early_trigger)
{
	struct panel_event_notification notifier;
	enum panel_event_notifier_tag panel_type;
	char tag_name[256];

	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	if (!strcmp(panel->type, "secondary")) {
		panel_type = PANEL_EVENT_NOTIFICATION_SECONDARY;
	} else {
		panel_type = PANEL_EVENT_NOTIFICATION_PRIMARY;
	}

	snprintf(tag_name, sizeof(tag_name),
		"oplus_panel_event_data_notifier_trigger : [%s] type=0x%X, data=%d, early_trigger=%d",
		panel->type, notif_type, data, early_trigger);
	OPLUS_LCD_TRACE_BEGIN(tag_name);

	LCD_DEBUG_COMMON("[%s] type=0x%X, data=%d, early_trigger=%d\n",
			panel->type, notif_type, data, early_trigger);

	memset(&notifier, 0, sizeof(notifier));

	notifier.panel = &panel->drm_panel;
	notifier.notif_type = notif_type;
	notifier.notif_data.data = data;
	notifier.notif_data.early_trigger = early_trigger;

	panel_event_notification_trigger(panel_type, &notifier);

	OPLUS_LCD_TRACE_END(tag_name);
	return 0;
}
EXPORT_SYMBOL(oplus_panel_event_data_notifier_trigger);

int oplus_event_data_notifier_trigger(
		enum panel_event_notification_type notif_type,
		u32 data,
		bool early_trigger)
{
	struct dsi_display *display = oplus_display_get_current_display();

	if (!display || !display->panel) {
		LCD_ERR("Oplus Features config No display device\n");
		return -ENODEV;
	}

	oplus_panel_event_data_notifier_trigger(display->panel,
			notif_type, data, early_trigger);

	return 0;
}
EXPORT_SYMBOL(oplus_event_data_notifier_trigger);

int oplus_panel_backlight_notifier(struct dsi_panel *panel, u32 bl_lvl)
{
	u32 threshold = panel->bl_config.dc_backlight_threshold;
	bool dc_mode = panel->bl_config.oplus_dc_mode;

	if (dc_mode && (bl_lvl > 1 && bl_lvl < threshold)) {
		dc_mode = false;
		oplus_panel_event_data_notifier_trigger(panel,
				DRM_PANEL_EVENT_DC_MODE, dc_mode, true);
	} else if (!dc_mode && bl_lvl >= threshold) {
		dc_mode = true;
		oplus_panel_event_data_notifier_trigger(panel,
				DRM_PANEL_EVENT_DC_MODE, dc_mode, true);
	}

	oplus_panel_event_data_notifier_trigger(panel,
			DRM_PANEL_EVENT_BACKLIGHT, bl_lvl, true);

	return 0;
}
EXPORT_SYMBOL(oplus_panel_backlight_notifier);

int oplus_panel_set_pinctrl_state(struct dsi_panel *panel, bool enable)
{
	int rc = 0;
	struct pinctrl_state *state;

	if (panel->host_config.ext_bridge_mode)
		return 0;

	if (!panel->pinctrl.pinctrl)
		return 0;

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	rc = oplus_adfr_te_source_vsync_switch_set_pinctrl_state(panel, enable);
	if (rc) {
		goto error;
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

	/* oplus panel pinctrl */
	if (panel->oplus_priv.pinctrl_enabled) {
		if (enable)
			state = panel->pinctrl.oplus_panel_active;
		else
			state = panel->pinctrl.oplus_panel_suspend;

		rc = pinctrl_select_state(panel->pinctrl.pinctrl, state);
		if (rc)
			LCD_ERR("[%s] failed to set oplus pin state, rc=%d\n",
					panel->oplus_priv.vendor_name, rc);
	}

error:
	return rc;
}

int oplus_panel_pinctrl_init(struct dsi_panel *panel)
{
	int rc = 0, count = 0;
	const char *pinctrl_name;

	if (panel->host_config.ext_bridge_mode)
		return 0;

	panel->pinctrl.pinctrl = devm_pinctrl_get(panel->parent);
	if (IS_ERR_OR_NULL(panel->pinctrl.pinctrl)) {
		rc = PTR_ERR(panel->pinctrl.pinctrl);
		LCD_ERR("failed to get pinctrl, rc=%d\n", rc);
		goto error;
	}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	rc = oplus_adfr_te_source_vsync_switch_pinctrl_init(panel);
	if (rc) {
		goto error;
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

	/* oplus panel pinctrl */
	count = of_property_count_strings(panel->panel_of_node,
			"oplus,dsi-pinctrl-names");
	if (OPLUS_PINCTRL_NAMES_COUNT == count) {
		of_property_read_string_index(panel->panel_of_node,
				"oplus,dsi-pinctrl-names", 0, &pinctrl_name);
		panel->pinctrl.oplus_panel_active =
				pinctrl_lookup_state(panel->pinctrl.pinctrl, pinctrl_name);
		if (IS_ERR_OR_NULL(panel->pinctrl.oplus_panel_active)) {
			rc = PTR_ERR(panel->pinctrl.oplus_panel_active);
			LCD_ERR("[%s] failed to get pinctrl: %s, rc=%d\n",
					panel->oplus_priv.vendor_name, pinctrl_name, rc);
			goto error;
		}

		of_property_read_string_index(panel->panel_of_node,
				"oplus,dsi-pinctrl-names", 1, &pinctrl_name);
		panel->pinctrl.oplus_panel_suspend =
				pinctrl_lookup_state(panel->pinctrl.pinctrl, pinctrl_name);
		if (IS_ERR_OR_NULL(panel->pinctrl.oplus_panel_suspend)) {
			rc = PTR_ERR(panel->pinctrl.oplus_panel_suspend);
			LCD_ERR("[%s] failed to get pinctrl: %s, rc=%d\n",
					panel->oplus_priv.vendor_name, pinctrl_name, rc);
			goto error;
		}

		panel->oplus_priv.pinctrl_enabled = true;
		LCD_INFO("[%s] successfully init oplus panel pinctrl, rc=%d\n",
				panel->oplus_priv.vendor_name, rc);
	} else if (count >= 0) {
		LCD_ERR("[%s] invalid oplus,dsi-pinctrl-names, count=%d\n",
				panel->oplus_priv.vendor_name, count);
	}

error:
	return rc;
}

void oplus_check_refresh_rate(const int old_rate, const int new_rate)
{
	if (old_rate != new_rate)
		refresh_rate_change = true;
	else
		refresh_rate_change = false;
}

int oplus_display_send_dcs_lock(struct dsi_display *display,
		enum dsi_cmd_set_type type)
{
	int rc = 0;

	if (!display || !display->panel) {
		LCD_ERR("invalid display panel\n");
		return -ENODEV;
	}

	if (display->panel->power_mode == SDE_MODE_DPMS_OFF) {
		LCD_WARN("display panel is in off status\n");
		return -EINVAL;
	}

	if (type < DSI_CMD_SET_MAX) {
		mutex_lock(&display->display_lock);
		/* enable the clk vote for CMD mode panels */
		if (display->config.panel_mode == DSI_OP_CMD_MODE) {
			rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_ON);
			if (rc) {
				LCD_ERR("failed to enable DSI clocks, rc=%d\n", rc);
				mutex_unlock(&display->display_lock);
				return -EFAULT;
			}
		}

		mutex_lock(&display->panel->panel_lock);
		rc = dsi_panel_tx_cmd_set(display->panel, type);
		mutex_unlock(&display->panel->panel_lock);

		/* disable the clk vote for CMD mode panels */
		if (display->config.panel_mode == DSI_OP_CMD_MODE) {
			rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
			if (rc) {
				LCD_ERR("failed to disable DSI clocks, rc=%d\n", rc);
			}
		}
		mutex_unlock(&display->display_lock);
	} else {
		LCD_ERR("dcs[%d] is out of range", type);
		return -EINVAL;
	}

	return rc;
}

int oplus_panel_cmd_reg_replace(struct dsi_panel *panel, enum dsi_cmd_set_type type,
		u8 cmd, u8 *replace_reg, size_t replace_reg_len)
{
	int rc = 0;
	struct dsi_cmd_desc *cmds;
	size_t tx_len;
	u8 *tx_buf;
	u32 count;
	u8 *payload;
	u32 size;
	int i;

	if(!panel) {
		LCD_ERR("invalid display panel\n");
		return -ENODEV;
	}
	if(!replace_reg) {
		LCD_ERR("invalid cmd reg\n");
		return -ENODEV;
	}

	cmds = panel->cur_mode->priv_info->cmd_sets[type].cmds;
	count = panel->cur_mode->priv_info->cmd_sets[type].count;
	for (i = 0; i < count; i++) {
		tx_len = cmds[i].msg.tx_len;
		tx_buf = (u8 *)cmds[i].msg.tx_buf;
		if (cmd == tx_buf[0]) {
			if ((tx_len - 1) != replace_reg_len) {
				tx_len = replace_reg_len + 1;
				size = tx_len * sizeof(u8);
				payload = kzalloc(size, GFP_KERNEL);
				if (!payload) {
					rc = -ENOMEM;
					return rc;
				}
				payload[0] = tx_buf[0];
				if (tx_buf) {
					kfree(tx_buf);
				}
				tx_buf = payload;
				cmds[i].msg.tx_len = tx_len;
			}
			tx_buf++;
			memcpy(tx_buf, replace_reg, replace_reg_len);
			break;
		}
	}

	return 0;
}

int oplus_panel_cmdq_pack_handle(void *dsi_panel, enum dsi_cmd_set_type type, bool before_cmd)
{
	struct dsi_panel *panel = dsi_panel;
	struct dsi_display_mode *mode;

	LCD_DEBUG("start\n");

	if (!panel || !panel->cur_mode) {
		LCD_ERR("invalid panel param\n");
		return -EINVAL;
	}
	mode = panel->cur_mode;

	if(!panel->oplus_priv.cmdq_pack_support || !mode->priv_info->cmd_sets[type].pack) {
		return 0;
	}

	OPLUS_LCD_TRACE_BEGIN("oplus_panel_cmdq_pack_handle");
	OPLUS_LCD_TRACE_INT("oplus_dsi_cmd_set_type", type);
	OPLUS_LCD_TRACE_INT("before_cmd", before_cmd);

	if (before_cmd) {
		if (panel->oplus_priv.cmdq_pack_state) {
			/*2 cmdqs in 1 frame */
			LCD_INFO("[%s] dsi_cmd: %s block to the next frame\n",
					panel->oplus_priv.vendor_name,
					cmd_set_prop_map[type]);
			oplus_sde_early_wakeup(panel);
			oplus_wait_for_vsync(panel);
			if (panel->cur_mode->timing.refresh_rate == 60 || panel->cur_mode->timing.refresh_rate == 90
			|| (panel->cur_mode->timing.refresh_rate == 120 && panel->last_refresh_rate == 90)) {
				oplus_need_to_sync_te(panel);
			} else if (panel->cur_mode->timing.refresh_rate == 120) {
				usleep_range(1000, 1020);
			}
		} else {
			if (strcmp(panel->oplus_priv.vendor_name , "AB781")) {
				/* force cmdq sending during half-past frame */
				oplus_need_to_sync_te(panel);
			}
		}
	} else {
		panel->oplus_priv.cmdq_pack_state = true;
	}

	OPLUS_LCD_TRACE_END("oplus_panel_cmdq_pack_handle");
	LCD_DEBUG("end\n");

	return 0;
}

int oplus_panel_cmdq_pack_status_reset(void *sde_connector)
{
	struct sde_connector *c_conn = sde_connector;
	struct dsi_display *display = NULL;

	LCD_DEBUG("start\n");

	if (!c_conn) {
		LCD_ERR("invalid c_conn param\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		LCD_WARN("not in dsi mode, should not reset cmdq pack status\n");
		return 0;
	}

	display = c_conn->display;
	if (!display || !display->panel) {
		LCD_DEBUG("invalid display params\n");
		return -EINVAL;
	}

	if(!display->panel->oplus_priv.cmdq_pack_support) {
		return 0;
	}

	display->panel->oplus_priv.cmdq_pack_state = false;

	LCD_DEBUG("cmdq pack state is %d\n", display->panel->oplus_priv.cmdq_pack_state);
	SDE_ATRACE_INT("oplus_panel_cmdq_pack_status_reset",
			display->panel->oplus_priv.cmdq_pack_state);

	LCD_DEBUG("end\n");

	return 0;
}

int oplus_panel_pwm_switch_cmdq_delay_handle(void *dsi_panel, enum dsi_cmd_set_type type)
{
	struct dsi_panel *panel = dsi_panel;
	struct dsi_display_mode *mode;
	int i;
	LCD_DEBUG("start\n");

	if (!panel || !panel->cur_mode) {
		LCD_ERR("invalid panel param\n");
		return -EINVAL;
	}
	mode = panel->cur_mode;

	if(!panel->oplus_priv.cmdq_pack_support || !mode->priv_info->cmd_sets[type].pack) {
		return 0;
	}

	OPLUS_LCD_TRACE_BEGIN("oplus_panel_pwm_switch_cmdq_delay_handle");
	if (panel->oplus_priv.pwm_sw_cmd_te_cnt > 0 && panel->oplus_priv.pwm_sw_cmd_te_cnt <= 2) {
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#ifdef OPLUS_FEATURE_DISPLAY_HIGH_PRECISION
		if ((type >= DSI_CMD_ADFR_MIN_FPS_0 && type <= DSI_CMD_HPWM_ADFR_MIN_FPS_14)
			|| type == DSI_CMD_ADFR_HIGH_PRECISION_FPS_0
			|| type == DSI_CMD_HPWM_ADFR_HIGH_PRECISION_FPS_0) {
#else
		if (type >= DSI_CMD_ADFR_MIN_FPS_0 && type <= DSI_CMD_HPWM_ADFR_MIN_FPS_14) {
#endif
			oplus_panel_set_pwm_switch_next_cmdq(type);
			queue_work(panel->oplus_pwm_switch_send_next_cmdq_wq, &panel->oplus_pwm_switch_send_next_cmdq_work);
			LCD_INFO("[%s] dsi_cmd: cmd %s turn to oplus_pwm_switch_send_next_cmdq_work to send\n",
				panel->oplus_priv.vendor_name,
				cmd_set_prop_map[type]);
			OPLUS_LCD_TRACE_END("oplus_panel_pwm_switch_cmdq_delay_handle");
			return 1;
		}
#endif
		LCD_INFO("[%s] dsi_cmd: %s block to the next 2 frames due to 2 frames cmdq\n",
					panel->oplus_priv.vendor_name,
					cmd_set_prop_map[type]);
		for (i = panel->oplus_priv.pwm_sw_cmd_te_cnt; i > 0; i--) {
			oplus_sde_early_wakeup(panel);
			oplus_wait_for_vsync(panel);
		}
		if (panel->cur_mode->timing.refresh_rate == 60 || panel->cur_mode->timing.refresh_rate == 90) {
			oplus_need_to_sync_te(panel);
		} else if (panel->cur_mode->timing.refresh_rate == 120) {
			usleep_range(1000, 1020);
		}
	}

	OPLUS_LCD_TRACE_END("oplus_panel_pwm_switch_cmdq_delay_handle");
	LCD_DEBUG("end\n");

	return 0;
}

int oplus_panel_send_asynchronous_cmd(void)
{
	int rc = 0;

	rc = oplus_display_panel_set_demura2_offset();

	return rc;
}

#define to_sde_encoder_phys_cmd(x) \
	container_of(x, struct sde_encoder_phys_cmd, base)

int oplus_set_osc_status(struct drm_encoder *drm_enc) {
	struct sde_encoder_virt *sde_enc = NULL;
	struct sde_encoder_phys *phys_encoder = NULL;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct sde_connector_state *c_state;
	int rc = 0;
	struct sde_encoder_phys_cmd *cmd_enc = NULL;
	bool sync_osc;
	u32 osc_status;

	sde_enc = to_sde_encoder_virt(drm_enc);
	phys_encoder = sde_enc->phys_encs[0];

	if (phys_encoder == NULL)
		return -EFAULT;
	if (phys_encoder->connector == NULL)
		return -EFAULT;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return -EFAULT;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	c_state = to_sde_connector_state(c_conn->base.state);

	display = c_conn->display;
	if (display == NULL)
		return -EFAULT;

	cmd_enc = to_sde_encoder_phys_cmd(phys_encoder);
	if (cmd_enc == NULL) {
		return -EFAULT;
	}

	sync_osc = c_conn->osc_need_update;
	c_conn->osc_need_update = false;

	if (sync_osc) {
		char tag_name[32];
		osc_status = sde_connector_get_property(c_conn->base.state, CONNECTOR_PROP_SET_OSC_STATUS);
		snprintf(tag_name, sizeof(tag_name), "sync_osc_status:%d", osc_status);
		SDE_ATRACE_BEGIN(tag_name);
		LCD_INFO("osc_status = %d", osc_status);
		oplus_display_panel_set_osc_track(osc_status);
		SDE_ATRACE_END(tag_name);
	}

	return rc;
}

int oplus_panel_cmd_reg_replace_specific_row(struct dsi_panel *panel, struct dsi_display_mode *mode,
		enum dsi_cmd_set_type type, u8 *replace_reg, size_t replace_reg_len, u32 row)
{
	int rc = 0;
	struct dsi_cmd_desc *cmds = NULL;
	size_t tx_len = 0;
	u8 *tx_buf = NULL;
	u32 count = 0;
	u8 *payload = NULL;
	u32 size = 0;
	u32 index = 0;

	if(!panel) {
		DSI_ERR("invalid display panel\n");
		return -ENODEV;
	}
	if(!replace_reg) {
		DSI_ERR("invalid cmd reg\n");
		return -ENODEV;
	}

	if (!mode) {
		LCD_INFO("mode is null, use panel cur_mode\n");
		mode = panel->cur_mode;
	}
	cmds = mode->priv_info->cmd_sets[type].cmds;
	count = mode->priv_info->cmd_sets[type].count;

	if (row > count) {
		DSI_ERR("Exceeding the number of rows of the command\n");
		return -EFAULT;
	}
	index = row - 1;

	tx_len = cmds[index].msg.tx_len;
	tx_buf = (u8 *)cmds[index].msg.tx_buf;
	if ((tx_len - 1) != replace_reg_len) {
		tx_len = replace_reg_len + 1;
		size = tx_len * sizeof(u8);
		payload = kzalloc(size, GFP_KERNEL);
		if (!payload) {
			rc = -ENOMEM;
			return rc;
		}
		payload[0] = tx_buf[0];
		if (tx_buf) {
			kfree(tx_buf);
		}
		tx_buf = payload;
		cmds[index].msg.tx_len = tx_len;
	}
	tx_buf++;
	memcpy(tx_buf, replace_reg, replace_reg_len);

	return 0;
}
