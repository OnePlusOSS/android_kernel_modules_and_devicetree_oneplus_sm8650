/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_display_panel_feature.c
** Description : oplus display panel char dev  /dev/oplus_panel
** Version : 1.0
** Date : 2021/11/17
** Author : Display
******************************************************************/
#include <drm/drm_mipi_dsi.h>
#include "dsi_parser.h"
#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_clk.h"
#include "oplus_bl.h"
#include <linux/ktime.h>
#include "oplus_display_panel_feature.h"
#include "oplus_display_private_api.h"
#include "oplus_display_interface.h"
#include "oplus_display_high_frequency_pwm.h"
#include "oplus_display_panel_common.h"
#include "sde_trace.h"

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "oplus_adfr.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
#include "oplus_display_temp_compensation.h"
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#if defined(CONFIG_PXLW_IRIS)
#include "dsi_iris_api.h"
#endif

#define REFRESH_RATE_FPS_60HZ 60
#define REFRESH_RATE_FPS_90HZ 90
#define REFRESH_RATE_FPS_120HZ 120
#define REFRESH_RATE_FPS_144HZ 144
#define REFRESH_RATE_FPS_165HZ 165

int video_cur_refresh_rate = 0;
int target_refresh_rate = 0;

extern int lcd_closebl_flag;
extern u32 oplus_last_backlight;
int switch_pwm_in_pre_bl = 0;
bool g_oplus_send_fps_code = false;
extern int oplus_sync_power_state;
extern int dsi_cmd_set_type_status;

int oplus_panel_get_serial_number_info(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = NULL;
	int ret = 0;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}
	utils = &panel->utils;

	panel->oplus_ser.serial_number_support = utils->read_bool(utils->data,
			"oplus,dsi-serial-number-enabled");
	LCD_INFO("oplus,dsi-serial-number-enabled: %s\n", panel->oplus_ser.serial_number_support ? "true" : "false");

	if (panel->oplus_ser.serial_number_support) {
		panel->oplus_ser.is_reg_lock = utils->read_bool(utils->data, "oplus,dsi-serial-number-lock");
		LCD_INFO("oplus,dsi-serial-number-lock: %s\n", panel->oplus_ser.is_reg_lock ? "true" : "false");

		ret = utils->read_u32(utils->data, "oplus,dsi-serial-number-reg",
				&panel->oplus_ser.serial_number_reg);
		if (ret) {
			LCD_INFO("failed to get oplus,dsi-serial-number-reg\n");
			panel->oplus_ser.serial_number_reg = 0xA1;
		}

		ret = utils->read_u32(utils->data, "oplus,dsi-serial-number-index",
				&panel->oplus_ser.serial_number_index);
		if (ret) {
			LCD_INFO("failed to get oplus,dsi-serial-number-index\n");
			/* Default sync start index is set 5 */
			panel->oplus_ser.serial_number_index = 7;
		}

		ret = utils->read_u32(utils->data, "oplus,dsi-serial-number-read-count",
				&panel->oplus_ser.serial_number_conut);
		if (ret) {
			LCD_INFO("failed to get oplus,dsi-serial-number-read-count\n");
			/* Default  read conut 5 */
			panel->oplus_ser.serial_number_conut = 5;
		}

		panel->oplus_ser.is_switch_page = utils->read_bool(utils->data,
			"oplus,dsi-serial-number-switch-page");
		LCD_INFO("oplus,dsi-serial-number-switch-page: %s", panel->oplus_ser.is_switch_page ? "true" : "false");
	}
	return 0;
}

void oplus_panel_parse_ignore_mode_config(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_parser_utils *utils = &panel->utils;
	char payload[128] = "";
	u32 cnt = 0;

	panel->oplus_priv.ignore_mode_count = utils->count_u32_elems(utils->data,
			"oplus,factory-ignore-mode");
	if (panel->oplus_priv.ignore_mode_count < 1) {
		LCD_INFO("factory ignore mode is NULL!\n");
		panel->oplus_priv.ignore_mode_count = 0;
		return;
	}

	panel->oplus_priv.ignore_mode = kcalloc(panel->oplus_priv.ignore_mode_count,
			sizeof(u32), GFP_KERNEL);
	if (!panel->oplus_priv.ignore_mode) {
		LCD_ERR("factory ignore mode list alloc failed!\n");
		return;
	}

	rc = utils->read_u32_array(utils->data,
			"oplus,factory-ignore-mode",
			panel->oplus_priv.ignore_mode,
			panel->oplus_priv.ignore_mode_count);

	if (rc) {
		LCD_ERR("factory ignore mode list parse failed!\n");
		return;
	}

	for (int i = 0; i < panel->oplus_priv.ignore_mode_count; i++) {
		cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, "[%u]", panel->oplus_priv.ignore_mode[i]);
	}
	LCD_INFO("parse ignore mode count: %d, mode_list: %s\n", panel->oplus_priv.ignore_mode_count, payload);

	return;
}

int oplus_panel_features_config(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = NULL;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		LCD_INFO("iris secondary panel no need config\n");
		return 0;
	}
#endif

	utils = &panel->utils;
	panel->oplus_priv.dp_support = utils->get_property(utils->data,
			"oplus,dp-enabled", NULL);

	if (!panel->oplus_priv.dp_support) {
		LCD_INFO("Failed to found panel dp support, using null dp config\n");
		panel->oplus_priv.dp_support = false;
	}

	panel->oplus_priv.cabc_enabled = utils->read_bool(utils->data,
			"oplus,dsi-cabc-enabled");
	LCD_INFO("oplus,dsi-cabc-enabled: %s\n", panel->oplus_priv.cabc_enabled ? "true" : "false");

	panel->oplus_priv.dre_enabled = utils->read_bool(utils->data,
			"oplus,dsi-dre-enabled");
	LCD_INFO("oplus,dsi-dre-enabled: %s\n", panel->oplus_priv.dre_enabled ? "true" : "false");

	panel->oplus_priv.panel_init_compatibility_enable = utils->read_bool(utils->data,
			"oplus,panel_init_compatibility_enable");
	LCD_INFO("oplus,panel_init_compatibility_enable: %s\n",
			panel->oplus_priv.panel_init_compatibility_enable ? "true" : "false");

	oplus_pwm_turbo_probe(panel);

	oplus_panel_get_serial_number_info(panel);

	oplus_panel_parse_ignore_mode_config(panel);
	panel->oplus_priv.vid_timming_switch_enabled = utils->read_bool(utils->data,
			"oplus,dsi-vid-timming-switch_enable");
	LCD_INFO("oplus,dsi-vid-timming-switch_enable: %s\n",
			panel->oplus_priv.vid_timming_switch_enabled ? "true" : "false");

	panel->oplus_priv.vid_timming_switch_post_enabled = utils->read_bool(utils->data,
			"oplus,dsi_vid_timming_switch_post_enable");
	LCD_INFO("oplus,dsi_vid_timming_switch_post_enable: %s\n",
		panel->oplus_priv.vid_timming_switch_post_enabled ? "true" : "false");

	panel->oplus_priv.dimming_setting_before_bl_0_enable = utils->read_bool(utils->data,
			"oplus,dsi-dimming-setting-before-bl-0-enable");
	LCD_INFO("oplus,dsi-dimming-setting-before-bl-0-enable: %s\n",
			panel->oplus_priv.dimming_setting_before_bl_0_enable ? "true" : "false");

	panel->oplus_priv.vidmode_backlight_async_wait_enable = utils->read_bool(utils->data,
			"oplus,dsi-vidmode-backlight-async-wait-enable");
	LCD_INFO("oplus,dsi-vidmode-backlight-async-wait-enable: %s\n",
			panel->oplus_priv.vidmode_backlight_async_wait_enable ? "true" : "false");

	panel->oplus_priv.set_backlight_not_do_esd_reg_read_enable = utils->read_bool(utils->data,
			"oplus,dsi-set-backlight-not-do-esd-reg-read-enable");
	LCD_INFO("oplus,dsi-set-backlight-not-do-esd-reg-read-enable: %s\n",
			panel->oplus_priv.set_backlight_not_do_esd_reg_read_enable ? "true" : "false");

	panel->oplus_priv.gamma_compensation_support = utils->read_bool(utils->data,
			"oplus,gamma-compensation-support");
	LCD_INFO("oplus,gamma-compensation-support: %s\n",
			panel->oplus_priv.gamma_compensation_support ? "true" : "false");

	panel->oplus_priv.dozedisable_esdcheck_delay = utils->read_bool(utils->data,
			"oplus,dozedisable-esdcheck-delay");
	LCD_INFO("oplus,dozedisable-esdcheck-delay: %s\n",
			panel->oplus_priv.dozedisable_esdcheck_delay ? "true" : "false");

	panel->oplus_priv.enable_dsi_cmd_package = utils->read_bool(utils->data,
		"oplus,enable_dsi_cmd_package");
	LCD_INFO("oplus,enable_dsi_cmd_package: %s\n",
		panel->oplus_priv.enable_dsi_cmd_package ? "true" : "false");

	panel->oplus_priv.vid_fps_switch_compenstate_enable = utils->read_bool(utils->data,
		"oplus,video-mode-fps-switch-compenstate-enable");
	LCD_INFO("oplus,video-mode-fps-switch-compenstate-enable: %s\n",
		panel->oplus_priv.vid_fps_switch_compenstate_enable ? "true" : "false");

	panel->oplus_priv.doze_disable_esdcheck = utils->read_bool(utils->data,
			"oplus,doze_disable_esdcheck");
	LCD_INFO("oplus,doze_disable_esdcheck: %s\n",
			panel->oplus_priv.doze_disable_esdcheck ? "true" : "false");

	panel->oplus_priv.ramless_aod_mode_cmd_switch_support = utils->read_bool(utils->data,
			"oplus,ramless_aod_mode_cmd_switch_support");
	LCD_INFO("oplus,ramless_aod_mode_cmd_switch_support: %s\n",
			panel->oplus_priv.ramless_aod_mode_cmd_switch_support ? "true" : "false");

	return 0;
}

int oplus_panel_post_on_backlight(void *display, struct dsi_panel *panel, u32 bl_lvl)
{
	struct dsi_display *dsi_display = display;
	int rc = 0;

	if (!panel || !dsi_display) {
		LCD_ERR("oplus post backlight No panel device\n");
		return -ENODEV;
	}

	LCD_DEBUG_BACKLIGHT("[%s] display backlight changed: %d -> %d\n",
			panel->oplus_priv.vendor_name, panel->bl_config.bl_level, bl_lvl);

	/* Add some delay to avoid screen flash */
	if (panel->need_power_on_backlight && bl_lvl) {
		panel->need_power_on_backlight = false;
		rc = dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
		rc |= dsi_panel_tx_cmd_set(panel, DSI_CMD_POST_ON_BACKLIGHT);
		rc |= dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_OFF);
		if (rc) {
			LCD_ERR("[%s] failed to send %s, rc=%d\n",
				panel->oplus_priv.vendor_name,
				cmd_set_prop_map[DSI_CMD_POST_ON_BACKLIGHT],
				rc);
		}

		atomic_set(&panel->esd_pending, 0);
	}
	return 0;
}

bool oplus_panel_switch_vid_mode_compenstate_post(struct dsi_panel *panel, struct drm_crtc *crtc, int refresh_rate)
{
	u32 rc = 0;
	int dsi_cmd_vid_switch = -1;
	bool cmd_switch_flag = false;
	target_refresh_rate = refresh_rate;

	if (target_refresh_rate == 165) {
		switch (video_cur_refresh_rate) {
		case REFRESH_RATE_FPS_60HZ:
		case REFRESH_RATE_FPS_90HZ:
		case REFRESH_RATE_FPS_120HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_ENTER_165HZ;
			break;
		case REFRESH_RATE_FPS_144HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_144HZ_ENTER_165HZ;
			break;
		}
	} else if (video_cur_refresh_rate == 165) {
		switch (target_refresh_rate) {
		case REFRESH_RATE_FPS_60HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_165HZ_ENTER_60HZ;
			break;
		case REFRESH_RATE_FPS_90HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_165HZ_ENTER_90HZ;
			break;
		case REFRESH_RATE_FPS_120HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_165HZ_ENTER_120HZ;
			break;
		case REFRESH_RATE_FPS_144HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_165HZ_ENTER_144HZ;
			break;
		}
	} else if (video_cur_refresh_rate == 144) {
		switch (target_refresh_rate) {
		case REFRESH_RATE_FPS_60HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_144HZ_ENTER_60HZ;
			break;
		case REFRESH_RATE_FPS_90HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_144HZ_ENTER_90HZ;
			break;
		case REFRESH_RATE_FPS_120HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_144HZ_ENTER_120HZ;
			break;
		case REFRESH_RATE_FPS_165HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_144HZ_ENTER_165HZ;
			break;
		}
	} else if (target_refresh_rate == 144) {
		switch (video_cur_refresh_rate) {
		case REFRESH_RATE_FPS_60HZ:
		case REFRESH_RATE_FPS_90HZ:
		case REFRESH_RATE_FPS_120HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_ENTER_144HZ;
			break;
		case REFRESH_RATE_FPS_165HZ:
			dsi_cmd_vid_switch = DSI_CMD_FPS_165HZ_ENTER_144HZ;
			break;
		}
	}

	LCD_INFO("panel : dsi_cmd_vid_switch = %d, target_refresh_rate = %d, "
		"video_cur_refresh_rate = %d\n", dsi_cmd_vid_switch,
		target_refresh_rate, video_cur_refresh_rate);

	if (panel->esd_config.status_mode == ESD_MODE_PANEL_ERROR_FLAG) {
		/*skip esd check when vedio mode switch timming gamma*/
		atomic_set(&panel->esd_pending, 1);
	}

	if (dsi_cmd_vid_switch != -1) {
		mutex_lock(&panel->panel_lock);
		rc = dsi_panel_tx_cmd_set(panel, dsi_cmd_vid_switch);
		if (rc) {
			LCD_ERR("[%s] failed to send fps compenstate cmds, rc=%d\n",
				panel->name, rc);
		}
		cmd_switch_flag = true;
		switch (video_cur_refresh_rate) {
		case REFRESH_RATE_FPS_60HZ:
			usleep_range(9000, 9000+100);
			break;
		case REFRESH_RATE_FPS_90HZ:
			usleep_range(3000, 3000+100);
			break;
		}
		mutex_unlock(&panel->panel_lock);
	}

	dsi_cmd_vid_switch = -1;
	video_cur_refresh_rate = target_refresh_rate;

	return cmd_switch_flag;
}

void oplus_panel_switch_vid_mode_post(struct dsi_display *display, struct dsi_display_mode *mode)
{
	u32 rc = 0;
	int refresh_rate = 0;
	static int cur_refresh_rate = 0;
	int dsi_cmd_vid_switch = 0;
	struct dsi_panel *panel = NULL;
	struct drm_crtc *crtc = NULL;

	if (!display || !display->panel) {
		LCD_INFO("display/panel is null!\n");
		return;
	}

	if (!mode) {
		LCD_INFO("dsi_display_mode is null!\n");
		return;
	}

	panel = display->panel;
	crtc = display->drm_conn->state->crtc;

	if (!panel->oplus_priv.vid_timming_switch_post_enabled) {
		return;
	}

	if (panel->power_mode == SDE_MODE_DPMS_OFF) {
		LCD_INFO("display panel in off status,power_mode = %d\n", panel->power_mode);
		return;
	}

	if (!dsi_panel_initialized(panel)) {
		LCD_ERR("should not set panel hbm if panel is not initialized\n");
		return;
	}

	refresh_rate = mode->timing.refresh_rate;
		LCD_INFO("oplus_panel_switch_vid_mode_post refresh %d\n", refresh_rate);

	if (panel->oplus_priv.vid_fps_switch_compenstate_enable && panel->oplus_priv.vid_timming_switch_post_enabled) {
		if (oplus_panel_switch_vid_mode_compenstate_post(panel, crtc, refresh_rate))
			return;
	}

	if (refresh_rate == 120) {
		dsi_cmd_vid_switch = DSI_CMD_VID_120_SWITCH;
	} else if (refresh_rate == 60) {
		dsi_cmd_vid_switch = DSI_CMD_VID_60_SWITCH;
	} else if (refresh_rate == 90) {
		dsi_cmd_vid_switch = DSI_CMD_VID_90_SWITCH;
	} else if (refresh_rate == 144) {
		dsi_cmd_vid_switch = DSI_CMD_VID_144_SWITCH;
	} else {
		return;
	}
	g_oplus_send_fps_code = true;

	mutex_lock(&panel->panel_lock);
	rc = dsi_panel_tx_cmd_set(panel, dsi_cmd_vid_switch);
	mutex_unlock(&panel->panel_lock);
	if (rc) {
		LCD_INFO("[%s] failed to send DSI_CMD_VID_SWITCH cmds, rc=%d\n",
			panel->name, rc);
	}

	cur_refresh_rate = refresh_rate;

	return;
}

void oplus_panel_switch_vid_mode(struct dsi_display *display, struct dsi_display_mode *mode)
{
	int rc = 0;
	int refresh_rate = 0;
	int dsi_cmd_vid_switch = 0;
	int te_count = 1;
	u32 current_vblank;
	struct dsi_panel *panel = NULL;
	struct drm_crtc *crtc = NULL;

	if (!display || !display->panel) {
		LCD_INFO("display/panel is null!\n");
		return;
	}

	if (!mode) {
		LCD_INFO("dsi_display_mode is null!\n");
		return;
	}

	panel = display->panel;
	crtc = display->drm_conn->state->crtc;
	if (panel->power_mode == SDE_MODE_DPMS_OFF) {
		LCD_INFO("display panel in off status,power_mode = %d", panel->power_mode);
		return;
	}

	if (!dsi_panel_initialized(panel)) {
		OFP_ERR("should not set panel hbm if panel is not initialized\n");
		return;
	}

	if (!panel->oplus_priv.vid_timming_switch_enabled) {
		LCD_ERR("oplus_panel_switch_vid_mode not support\n");
		return;
	}

	refresh_rate = mode->timing.refresh_rate;
		LCD_INFO("oplus_panel_switch_vid_mode refresh %d\n", refresh_rate);

	if (refresh_rate == 120) {
		dsi_cmd_vid_switch = DSI_CMD_VID_120_SWITCH;
	} else if (refresh_rate == 90) {
		dsi_cmd_vid_switch = DSI_CMD_VID_90_SWITCH;
	} else if (refresh_rate == 60) {
		dsi_cmd_vid_switch = DSI_CMD_VID_60_SWITCH;
	} else if (refresh_rate == 144) {
		dsi_cmd_vid_switch = DSI_CMD_VID_144_SWITCH;
	} else {
		return;
	}
	if (panel->esd_config.status_mode == ESD_MODE_PANEL_MIPI_ERR_FLAG) {
		/*skip esd check when vedio mode switch timming gamma*/
		atomic_set(&panel->esd_pending, 1);
	}

	if (!strcmp(panel->name, "AB964 p 1 A0017 dsc video mode panel")) {
		SDE_ATRACE_BEGIN("wait_for_vblank");
		current_vblank = drm_crtc_vblank_count(crtc);
		current_vblank = current_vblank + te_count;
		if (refresh_rate == 120) {
			rc = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc), current_vblank == drm_crtc_vblank_count(crtc), usecs_to_jiffies(4100 + 100));
			if (!rc) {
				OFP_ERR("crtc wait_event_timeout\n");
			}
		}
		if (refresh_rate == 60) {
			rc = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc), current_vblank == drm_crtc_vblank_count(crtc), usecs_to_jiffies(8300 + 100));
			if (!rc) {
				OFP_ERR("crtc wait_event_timeout\n");
			}
		}
		SDE_ATRACE_END("wait_for_vblank");
	}

	SDE_ATRACE_BEGIN("oplus_panel_switch_vid_mode");

	mutex_lock(&panel->panel_lock);
	rc = dsi_panel_tx_cmd_set(panel, dsi_cmd_vid_switch);
	mutex_unlock(&panel->panel_lock);
	if (rc) {
		LCD_INFO("[%s] failed to send DSI_CMD_VID_SWITCH cmds, rc=%d\n",
			panel->name, rc);
	}
	SDE_ATRACE_END("oplus_panel_switch_vid_mode");

	return;
}

u32 oplus_panel_silence_backlight(struct dsi_panel *panel, u32 bl_lvl)
{
	u32 bl_temp = 0;
	if (!panel) {
		LCD_ERR("Oplus Features config No panel device\n");
		return -ENODEV;
	}

	bl_temp = bl_lvl;

	if (lcd_closebl_flag) {
		LCD_INFO("silence reboot we should set backlight to zero\n");
		bl_temp = 0;
	}
	return bl_temp;
}

void oplus_panel_update_backlight(struct dsi_panel *panel,
		struct mipi_dsi_device *dsi, u32 bl_lvl)
{
	int rc = 0;
	u64 inverted_dbv_bl_lvl = 0;
	int need_delay_te_cnt = 0;
	int i = 0;
	panel->pwm_params.pack_backlight = false;
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	if (oplus_adfr_osync_backlight_filter(panel, bl_lvl)) {
		return;
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		oplus_ofp_lhbm_backlight_update(NULL, panel, &bl_lvl);
		if (oplus_ofp_backlight_filter(panel, bl_lvl)) {
			return;
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
	if (oplus_temp_compensation_is_supported()) {
		oplus_temp_compensation_cmd_set(panel, OPLUS_TEMP_COMPENSATION_BACKLIGHT_SETTING);
	}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

#ifdef OPLUS_FEATURE_DISPLAY
	if (panel->bl_config.oplus_limit_max_bl_mode) {
		if (bl_lvl > panel->bl_config.oplus_limit_max_bl)
			bl_lvl = panel->bl_config.oplus_limit_max_bl;
	}

	if (panel->oplus_priv.dimming_setting_before_bl_0_enable) {
		if (bl_lvl == 0) {
			if (!strcmp(panel->name, "AA577 P 3 A0020 dsc cmd mode panel")) {
				if (oplus_last_backlight > 0) {
					rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_DIMMING_SETTING);
					oplus_sde_early_wakeup(panel);
					oplus_wait_for_vsync(panel);
				}
			} else {
				rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_DIMMING_SETTING);
			}
		}
	}
#endif

	oplus_temp_compensation_wait_for_vsync_set = false;

	/* backlight value mapping */
	oplus_panel_global_hbm_mapping(panel, &bl_lvl);

	/* pwm switch due to backlight change*/
	oplus_panel_pwm_switch(panel, &bl_lvl);

	oplus_panel_backlight_demura_dbv_switch(panel, bl_lvl);

#ifdef OPLUS_FEATURE_DISPLAY_HIGH_PRECISION
	oplus_adfr_high_precision_switch_state(panel);
#endif /* OPLUS_FEATURE_DISPLAY_HIGH_PRECISION */

	if (!panel->oplus_priv.need_sync && panel->cur_mode->priv_info->async_bl_delay) {
		if (panel->oplus_priv.disable_delay_bl_count > 0) {
			panel->oplus_priv.disable_delay_bl_count--;
		} else if (panel->oplus_priv.disable_delay_bl_count == 0) {
			oplus_apollo_async_bl_delay(panel);
		} else {
			DSI_INFO("invalid disable_delay_bl_count\n");
			panel->oplus_priv.disable_delay_bl_count = 0;
		}
	}
	/* will inverted display brightness value */
	if (panel->bl_config.bl_inverted_dbv)
		inverted_dbv_bl_lvl = (((bl_lvl & 0xff) << 8) | (bl_lvl >> 8));
	else
		inverted_dbv_bl_lvl = bl_lvl;

	/* need to delay 51 to the next frame of pwm switch cmd */
	if (switch_pwm_in_pre_bl == 1 && panel->oplus_priv.pwm_sw_cmd_te_cnt > 0) {
		if (oplus_ofp_get_hbm_state()) {
			LCD_INFO("lhbm on state, cancel delay 51 to next frame operation\n");
		} else {
			oplus_sde_early_wakeup(panel);
			/* use pwm_sw_cmd_te_cnt to caculate interval from pwm_cmd then delay to next 2 frames of pwm_sw cmd */
			need_delay_te_cnt = panel->oplus_priv.pwm_sw_cmd_te_cnt;
			for (i = 0; i < need_delay_te_cnt; i++) {
				oplus_wait_for_vsync(panel);
			}
			if (panel->cur_mode->timing.refresh_rate == 90 || panel->cur_mode->timing.refresh_rate == 60) {
				oplus_need_to_sync_te(panel);
			} else {
				/* avoid sending 51 in vproch */
				usleep_range(1200, 1800);
			}
			LCD_INFO("bl_lvl %d delay to next 2 frames of pwm_switch cmd\n", bl_lvl);
		}
	}
	switch_pwm_in_pre_bl = 0;

	if(panel->pwm_params.pack_backlight == false) {
		mutex_lock(&panel->panel_tx_lock);

		if (panel->oplus_priv.dsi_cmd_need_to_package) {
			dsi_cmd_set_type_status = 0;
			panel->oplus_priv.dsi_cmd_need_to_package = false;
		}

#if defined(CONFIG_PXLW_IRIS)
		if (iris_is_chip_supported() && iris_is_pt_mode(panel))
			rc = iris_update_backlight(inverted_dbv_bl_lvl);
		else
#endif
			rc = mipi_dsi_dcs_set_display_brightness(dsi, inverted_dbv_bl_lvl);

		mutex_unlock(&panel->panel_tx_lock);
		if (rc < 0)
			LCD_ERR("failed to update dcs backlight:%d\n", bl_lvl);
	} else {
		LCD_INFO("The cmd of backlight %d was packed in other cmd\n", bl_lvl);
		switch_pwm_in_pre_bl = 1;
		panel->oplus_priv.pwm_sw_cmd_te_cnt = 2;
	}
#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && !iris_is_pt_mode(panel))
		rc = iris_update_backlight_value(bl_lvl);
#endif

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
	if (oplus_temp_compensation_is_supported()) {
		oplus_temp_compensation_first_half_frame_cmd_set(panel);
	}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		oplus_ofp_lhbm_dbv_vdc_update(panel, bl_lvl, false);
		oplus_ofp_lhbm_dbv_alpha_update(panel, bl_lvl, false);
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	LCD_DEBUG_BACKLIGHT("[%s] panel backlight changed: %d -> %d\n",
			panel->oplus_priv.vendor_name, oplus_last_backlight, bl_lvl);

	oplus_panel_pwm_onepulse_switch(panel);

	oplus_last_backlight = bl_lvl;
}

void oplus_printf_backlight_log(struct dsi_display *display, u32 bl_lvl) {
	struct timespec64 now;
	struct tm broken_time;
	static time64_t time_last = 0;
	struct backlight_log *bl_log;
	int i = 0;
	int len = 0;
	char backlight_log_buf[1024];

	ktime_get_real_ts64(&now);
	time64_to_tm(now.tv_sec, 0, &broken_time);
	if (now.tv_sec - time_last >= 60) {
		pr_info("<%s> dsi_display_set_backlight time:%02d:%02d:%02d.%03ld,bl_lvl:%d\n",
			display->panel->oplus_priv.vendor_name, broken_time.tm_hour, broken_time.tm_min,
			broken_time.tm_sec, now.tv_nsec / 1000000, bl_lvl);
		time_last = now.tv_sec;
	}

	if (!strcmp(display->display_type, "secondary")) {
		bl_log = &oplus_bl_log[DISPLAY_SECONDARY];
	} else {
		bl_log = &oplus_bl_log[DISPLAY_PRIMARY];
	}

	bl_log->backlight[bl_log->bl_count] = bl_lvl;
	bl_log->past_times[bl_log->bl_count] = now;
	bl_log->bl_count++;
	if (bl_log->bl_count >= BACKLIGHT_CACHE_MAX) {
		bl_log->bl_count = 0;
		memset(backlight_log_buf, 0, sizeof(backlight_log_buf));
		for (i = 0; i < BACKLIGHT_CACHE_MAX; i++) {
			time64_to_tm(bl_log->past_times[i].tv_sec, 0, &broken_time);
			len += snprintf(backlight_log_buf + len, sizeof(backlight_log_buf) - len,
				"%02d:%02d:%02d.%03ld:%d,", broken_time.tm_hour, broken_time.tm_min,
				broken_time.tm_sec, bl_log->past_times[i].tv_nsec / 1000000, bl_log->backlight[i]);
		}
		pr_info("<%s> len:%d dsi_display_set_backlight %s\n", display->panel->oplus_priv.vendor_name, len, backlight_log_buf);
	}
}

