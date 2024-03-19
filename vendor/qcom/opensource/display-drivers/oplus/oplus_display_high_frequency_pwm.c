/***************************************************************
** Copyright (C),  2023,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_high_frequency_pwm.c
** Description : oplus high frequency PWM
** Version : 1.0
** Date : 2023/07/11
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Ping      2023/07/11        1.0           Build this moudle
******************************************************************/
#include <linux/notifier.h>
#include <linux/msm_drm_notify.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#include "oplus_display_private_api.h"
#include "oplus_display_high_frequency_pwm.h"
#include "oplus_display_interface.h"
#include "oplus_display_panel_common.h"

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "oplus_adfr.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

/* -------------------- extern ---------------------------------- */
extern u32 oplus_last_backlight;
extern bool refresh_rate_change;

static u32 pwm_switch_cmd_restore = 0;
static u32 pwm_switch_state_before = 0;
u32 bl_lvl = 0;
static struct oplus_pwm_turbo_params g_oplus_pwm_turbo_params = {0};

struct oplus_pwm_turbo_params *oplus_pwm_turbo_get_params(void)
{
	return &g_oplus_pwm_turbo_params;
}
EXPORT_SYMBOL(oplus_pwm_turbo_get_params);

inline bool oplus_panel_pwm_turbo_is_enabled(struct dsi_panel *panel)
{
	if (!panel) {
		LCD_ERR("oplus_panel_pwm_turbo_is_enabled Invalid\n");
		return false;
	}

	return (bool)(panel->pwm_params.pwm_turbo_support
			&& panel->pwm_params.pwm_turbo_enabled);
}
EXPORT_SYMBOL(oplus_panel_pwm_turbo_is_enabled);

inline bool oplus_panel_pwm_turbo_switch_state(struct dsi_panel *panel)
{
	if (!panel) {
		LCD_ERR("panel is NULL\n");
		return false;
	}

	return (bool)(panel->pwm_params.pwm_turbo_support
			&& panel->pwm_params.pwm_turbo_enabled
			&& panel->pwm_params.oplus_pwm_switch_state);
}
EXPORT_SYMBOL(oplus_panel_pwm_turbo_switch_state);

inline bool pwm_turbo_support(void)
{
	bool pwm_support = false;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	pwm_support = pwm_params->config > 0 ? true:false;
	return pwm_support;
}
EXPORT_SYMBOL(pwm_turbo_support);

inline bool get_pwm_turbo_states(void)
{
	bool states = false;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	states = pwm_params->hpwm_mode > 0 ? true:false;
	return states;
}
EXPORT_SYMBOL(get_pwm_turbo_states);

int oplus_pwm_turbo_probe(struct dsi_panel *panel)
{
	int rc = 0;
	int val = 0;
	struct dsi_parser_utils *utils = NULL;

	if (!panel) {
		LCD_ERR("pwm_turbo_probe Invalid panel params\n");
		return -EINVAL;
	}

	utils = &panel->utils;

	panel->pwm_params.pwm_turbo_support = utils->read_bool(utils->data,
			"oplus,pwm-turbo-support");
	LCD_INFO("oplus,pwm-turbo-support: %s\n",
			panel->pwm_params.pwm_turbo_support ? "true" : "false");
	panel->pwm_params.pwm_turbo_enabled = utils->read_bool(utils->data,
			"oplus,pwm-turbo-enabled-default");
	LCD_INFO("oplus,pwm-turbo-enabled-default: %s\n",
			panel->pwm_params.pwm_turbo_enabled ? "true" : "false");

	panel->pwm_params.pwm_switch_support = utils->read_bool(utils->data,
			"oplus,pwm-switch-support");
	LCD_INFO("oplus,pwm-switch-support: %s\n",
			panel->pwm_params.pwm_switch_support ? "true" : "false");

	panel->pwm_params.pwm_switch_restore_support = utils->read_bool(utils->data,
			"oplus,pwm-switch-restore-support");
	LCD_INFO("oplus,pwm-switch-restore-support: %s\n",
			panel->pwm_params.pwm_switch_restore_support ? "true" : "false");

	panel->pwm_params.pwm_wait_te_tx = utils->read_bool(utils->data,
			"oplus,pwm-switch-wait-te-tx");
	LCD_INFO("oplus,pwm-switch-wait-te-tx: %s\n",
			panel->pwm_params.pwm_wait_te_tx ? "true" : "false");

	rc = utils->read_u32(utils->data, "oplus,pwm-switch-backlight-threshold", &val);
	if (rc) {
		panel->pwm_params.pwm_switch_support = false;
	} else {
		panel->pwm_params.pwm_bl_threshold = val;
	}
	LCD_INFO("[%s] oplus,pwm-switch-backlight-threshold=%d\n",
			panel->oplus_priv.vendor_name,
			panel->pwm_params.pwm_bl_threshold);

	panel->pwm_params.pwm_onepulse_support = utils->read_bool(utils->data,
			"oplus,pwm-onepulse-support");
	LCD_INFO("oplus,pwm-onepulse-support: %s\n",
			panel->pwm_params.pwm_onepulse_support ? "true" : "false");
	panel->pwm_params.pwm_onepulse_enabled = utils->read_bool(utils->data,
			"oplus,pwm-onepulse-default-enabled");
	LCD_INFO("oplus,pwm-onepulse-default-enabled: %s\n",
			panel->pwm_params.pwm_onepulse_enabled ? "true" : "false");

	panel->pwm_params.pwm_power_on = false;
	panel->pwm_params.pwm_hbm_state = false;
	PWM_TURBO_INFO("pwm_turbo oplus_pwm_turbo_probe successful\n");
	return 0;
}

inline int get_pwm_turbo_plus_bl(void)
{
	int bl = 0;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	bl = pwm_params->hpwm_bl;
	return bl;
}
EXPORT_SYMBOL(get_pwm_turbo_plus_bl);

int oplus_pwm_set_power_on(struct dsi_panel *panel)
{
	if (!panel) {
		LCD_ERR("Invalid panel params\n");
		return -EINVAL;
	}

	if (panel->pwm_params.pwm_switch_support) {
		panel->pwm_params.pwm_power_on = true;
	}

	return 0;
}

int oplus_hbm_pwm_state(struct dsi_panel *panel, bool hbm_state)
{
	if (!panel) {
		LCD_ERR("Invalid panel params\n");
		return -EINVAL;
	}

	if (panel->pwm_params.pwm_switch_support && hbm_state) {
		if (oplus_panel_pwm_onepulse_is_enabled(panel)) {
			oplus_panel_event_data_notifier_trigger(panel, DRM_PANEL_EVENT_PWM_TURBO, 1, true);
		} else {
			oplus_panel_event_data_notifier_trigger(panel, DRM_PANEL_EVENT_PWM_TURBO, !hbm_state, true);
		}
	}

	if (panel->pwm_params.pwm_switch_support) {
		panel->pwm_params.pwm_hbm_state = hbm_state;

		if (!hbm_state) {
			panel->pwm_params.pwm_power_on = true;
		}
	}
	LCD_INFO("set oplus pwm_hbm_state = %d\n", hbm_state);
	return 0;
}

void oplus_pwm_disable_duty_set_work_handler(struct work_struct *work)
{
	int rc = 0;
	struct dsi_panel *panel = container_of(work, struct dsi_panel, oplus_pwm_disable_duty_set_work);
	unsigned int refresh_rate = panel->cur_mode->timing.refresh_rate;

	oplus_sde_early_wakeup(panel);
	oplus_wait_for_vsync(panel);
	if (refresh_rate == 60) {
		oplus_need_to_sync_te(panel);
	}

	if (oplus_panel_pwm_onepulse_is_enabled(panel)) {
		if (refresh_rate == 90) {
			oplus_need_to_sync_te(panel);
		}
	}

	mutex_lock(&panel->panel_lock);
	if (panel->power_mode != SDE_MODE_DPMS_ON || !panel->panel_initialized) {
		LCD_WARN("display panel in off status\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}
	usleep_range(120, 120);
	if (panel->pwm_params.pwm_switch_restore_support) {
		rc = dsi_panel_tx_cmd_set(panel, pwm_switch_cmd_restore);
	}

	mutex_unlock(&panel->panel_lock);

	if (rc) {
		LCD_ERR("[%s]failed to send pwm_switch_cmd_restore cmds rc = %d\n", panel->name, rc);
	}

	return;
}

int oplus_panel_pwm_switch_wait_te_tx_cmd(struct dsi_panel *panel, u32 pwm_switch_cmd)
{
	int rc = 0;
	unsigned int refresh_rate = panel->cur_mode->timing.refresh_rate;

	if (!panel || !panel->cur_mode) {
		LCD_ERR("[DISP][ERR][%s:%d]Invalid panel params\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (panel->pwm_params.pwm_power_on) {
		rc = dsi_panel_tx_cmd_set(panel, pwm_switch_cmd);
		panel->pwm_params.pwm_power_on = false;
		panel->pwm_params.oplus_pwm_switch_state_changed = false;
		return rc;
	}

	if ((pwm_switch_state_before != panel->pwm_params.oplus_pwm_switch_state) || (panel->pwm_params.oplus_pwm_switch_state_changed == true)) {
		oplus_sde_early_wakeup(panel);
		oplus_wait_for_vsync(panel);
		if (refresh_rate == 60) {
			oplus_need_to_sync_te(panel);
		} else {
			usleep_range(120, 120);
		}

		if (oplus_panel_pwm_onepulse_is_enabled(panel)) {
			if (refresh_rate == 90) {
				oplus_need_to_sync_te(panel);
			}
		}

		rc = dsi_panel_tx_cmd_set(panel, pwm_switch_cmd);

		if (panel->oplus_priv.pwm_create_thread) {
			queue_work(panel->oplus_pwm_disable_duty_set_wq, &panel->oplus_pwm_disable_duty_set_work);
		}
	}
	panel->pwm_params.pwm_power_on = false;
	panel->pwm_params.oplus_pwm_switch_state_changed = false;

	return rc;
}

int oplus_panel_pwm_switch_tx_cmd(struct dsi_panel *panel)
{
	int rc = 0;
	u32 pwm_switch_cmd = 0;

	if (!panel) {
		LCD_ERR("oplus_panel_pwm_switch_tx_cmd Invalid panel params\n");
		return -EINVAL;
	}

	if (!panel->pwm_params.pwm_switch_support) {
		return rc;
	}

	if (panel->pwm_params.pwm_hbm_state) {
		LCD_INFO("panel pwm_hbm_state true disable pwm switch!\n");
		return rc;
	}

	if (panel->pwm_params.oplus_pwm_switch_state == PWM_SWITCH_HIGH_STATE) {
		pwm_switch_cmd = DSI_CMD_PWM_SWITCH_HIGH;
		pwm_switch_cmd_restore = DSI_CMD_PWM_SWITCH_HIGH_RESTORE;
		if (panel->pwm_params.pwm_power_on) {
			if ((!strcmp(panel->name, "enzo boe_ili7838e 1264 2780 evt dsc cmd mode panel")
			 || !strcmp(panel->name, "enzo boe_ili7838e 1264 2780 pvt bd dsc cmd mode panel"))
			  && oplus_panel_pwm_onepulse_is_enabled(panel)) {
				pwm_switch_cmd = DSI_CMD_PWM_SWITCH_3TO1;
			} else {
				pwm_switch_cmd = DSI_CMD_TIMMING_PWM_SWITCH_HIGH;
			}
		}
	} else {
		pwm_switch_cmd = DSI_CMD_PWM_SWITCH_LOW;
		pwm_switch_cmd_restore = DSI_CMD_PWM_SWITCH_LOW_RESTORE;
		if (panel->pwm_params.pwm_power_on)
			pwm_switch_cmd = DSI_CMD_TIMMING_PWM_SWITCH_LOW;
	}

	if (panel->pwm_params.pwm_wait_te_tx) {
		oplus_panel_pwm_switch_wait_te_tx_cmd(panel, pwm_switch_cmd);
	} else {
		rc = dsi_panel_tx_cmd_set(panel, pwm_switch_cmd);
		panel->pwm_params.pwm_power_on = false;
		panel->pwm_params.oplus_pwm_switch_state_changed = false;
	}

	return 0;
}

int oplus_panel_pwm_switch(struct dsi_panel *panel, u32 *backlight_level)
{
	int rc = 0;
	bl_lvl = *backlight_level;

	if (!panel) {
		LCD_ERR("oplus_panel_pwm_switch Invalid panel params\n");
		return -EINVAL;
	}

	if (!panel->pwm_params.pwm_switch_support) {
		return rc;
	}

	if (panel->pwm_params.pwm_hbm_state) {
		LCD_INFO("panel pwm_hbm_state true disable pwm switch!\n");
		return rc;
	}

	pwm_switch_state_before = panel->pwm_params.oplus_pwm_switch_state;

	if (bl_lvl > panel->pwm_params.pwm_bl_threshold) {
		panel->pwm_params.oplus_pwm_switch_state = PWM_SWITCH_HIGH_STATE;
	} else {
		panel->pwm_params.oplus_pwm_switch_state = PWM_SWITCH_LOW_STATE;
	}
	if (pwm_switch_state_before != panel->pwm_params.oplus_pwm_switch_state) {
		panel->pwm_params.oplus_pwm_switch_state_changed = true;
	}

	if (strcmp(panel->name, "enzo boe_ili7838e 1264 2780 evt dsc cmd mode panel")
	    && strcmp(panel->name, "enzo boe_ili7838e 1264 2780 pvt bd dsc cmd mode panel")) {
		/* 3 pulse code == 1 pulse func open && backlight low state*/
		if ((panel->pwm_params.oplus_pwm_switch_state_changed == true || oplus_last_backlight == 0)
				&& oplus_panel_pwm_onepulse_is_enabled(panel)
				&& panel->pwm_params.oplus_pwm_switch_state == PWM_SWITCH_LOW_STATE) {
			rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_PWM_SWITCH_THREEPULSE);
		}
	}

	if (panel->pwm_params.oplus_pwm_switch_state_changed == true
			|| oplus_last_backlight == 0
			|| panel->pwm_params.pwm_power_on) {
		rc = oplus_panel_pwm_switch_tx_cmd(panel);
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
		oplus_adfr_set_min_fps_updated(panel);
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */
	}
	oplus_panel_event_data_notifier_trigger(panel, DRM_PANEL_EVENT_PWM_TURBO, !(panel->pwm_params.oplus_pwm_switch_state), true);

	return 0;
}

int oplus_panel_pwm_switch_timing_switch(struct dsi_panel *panel)
{
	int rc = 0;
	u32 pwm_switch_cmd = DSI_CMD_TIMMING_PWM_SWITCH_LOW;

	if (!panel->pwm_params.pwm_switch_support)
		return rc;

	if (panel->pwm_params.pwm_hbm_state) {
		LCD_INFO("panel pwm_hbm_state true disable pwm switch!\n");
		return rc;
	}

	if (panel->pwm_params.oplus_pwm_switch_state  == PWM_SWITCH_HIGH_STATE) {
		pwm_switch_cmd = DSI_CMD_TIMMING_PWM_SWITCH_HIGH;
	}

	rc = dsi_panel_tx_cmd_set(panel, pwm_switch_cmd);

	return rc;
}

int oplus_panel_send_pwm_turbo_dcs_unlock(struct dsi_panel *panel, bool enabled)
{
	int rc = 0;

	if (!panel) {
		LCD_ERR("oplus_panel_send_pwm_turbo_dcs_unlock Invalid panel params\n");
		return -EINVAL;
	}

	if (enabled)
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_PWM_TURBO_ON);
	else
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_PWM_TURBO_OFF);

	return rc;
}

int oplus_panel_update_pwm_turbo_lock(struct dsi_panel *panel, bool enabled)
{
	int rc = 0;

	if (!panel) {
		LCD_ERR("oplus_panel_update_pwm_turbo_lock Invalid panel params\n");
		return -EINVAL;
	}

	oplus_panel_event_data_notifier_trigger(panel,
			DRM_PANEL_EVENT_PWM_TURBO, enabled, true);

	mutex_lock(&panel->panel_lock);

	panel->pwm_params.pwm_turbo_enabled = enabled;
	if(panel->power_mode != SDE_MODE_DPMS_OFF)
		rc = oplus_panel_send_pwm_turbo_dcs_unlock(panel, enabled);
	else
		LCD_WARN("Skip send pwm turbo dcs, because display panel is off\n");

	mutex_unlock(&panel->panel_lock);

	return rc;
}

int oplus_display_panel_get_pwm_turbo(void *data)
{
	int rc = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;
	uint32_t *enabled = data;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->pwm_params.pwm_turbo_support) {
		LCD_WARN("Falied to get pwm turbo status, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&panel->panel_lock);

	*enabled = panel->pwm_params.pwm_turbo_enabled;

	mutex_unlock(&panel->panel_lock);
	mutex_unlock(&display->display_lock);
	LCD_INFO("Get pwm turbo status: %d\n", *enabled);

	return rc;
}

int oplus_display_panel_set_pwm_turbo(void *data)
{
	int rc = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;
	uint32_t *enabled = data;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->pwm_params.pwm_turbo_support) {
		LCD_WARN("Falied to set pwm turbo status, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	LCD_INFO("Set pwm turbo status: %d\n", *enabled);

	if (*enabled == panel->pwm_params.pwm_turbo_enabled) {
		LCD_WARN("Skip setting duplicate pwm turbo status: %d\n", *enabled);
		rc = -EFAULT;
		return rc;
	}

	mutex_lock(&display->display_lock);
	rc = oplus_panel_update_pwm_turbo_lock(panel, *enabled);
	mutex_unlock(&display->display_lock);

	return rc;
}

ssize_t oplus_get_pwm_turbo_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	int rc = 0;
	u32 enabled = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->pwm_params.pwm_turbo_support) {
		LCD_ERR("Falied to get pwm turbo status, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&panel->panel_lock);

	enabled = panel->pwm_params.pwm_turbo_enabled;

	mutex_unlock(&panel->panel_lock);
	mutex_unlock(&display->display_lock);
	LCD_INFO("Get pwm turbo status: %d\n", enabled);

	return sprintf(buf, "%d\n", enabled);
}

ssize_t oplus_set_pwm_turbo_debug(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int rc = 0;
	u32 enabled = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->pwm_params.pwm_turbo_support) {
		LCD_ERR("Falied to set pwm turbo status, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	sscanf(buf, "%du", &enabled);
	LCD_INFO("Set pwm turbo status: %d\n", enabled);

	mutex_lock(&display->display_lock);
	oplus_panel_update_pwm_turbo_lock(panel, enabled);
	mutex_unlock(&display->display_lock);

	return count;
}

inline bool oplus_panel_pwm_onepulse_is_enabled(struct dsi_panel *panel)
{
	if (!panel) {
		LCD_ERR("Invalid panel\n");
		return false;
	}
	return (bool)(panel->pwm_params.pwm_onepulse_support &&
			panel->pwm_params.pwm_onepulse_enabled);
}

inline bool oplus_panel_pwm_onepulse_switch_state(struct dsi_panel *panel)
{
	if (!panel) {
		LCD_ERR("panel is NULL\n");
		return false;
	}

	return (bool)(panel->pwm_params.pwm_onepulse_support
			&& panel->pwm_params.pwm_onepulse_enabled
			&& panel->pwm_params.oplus_pwm_switch_state);
}

int oplus_panel_update_pwm_pulse_lock(struct dsi_panel *panel, bool enabled)
{
	int rc = 0;

	mutex_lock(&panel->panel_lock);
	panel->pwm_params.pwm_onepulse_enabled = enabled;

	if (!strcmp(panel->name, "enzo boe_ili7838e 1264 2780 evt dsc cmd mode panel")
	      || !strcmp(panel->name, "enzo boe_ili7838e 1264 2780 pvt bd dsc cmd mode panel")) {
		if (panel->pwm_params.oplus_pwm_switch_state == PWM_SWITCH_HIGH_STATE) {
			if (panel->pwm_params.pwm_onepulse_enabled) {
				rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_PWM_SWITCH_3TO1);
			} else {
				rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_PWM_SWITCH_1TO3);
			}
		}
	} else {
		panel->pwm_params.oplus_pwm_switch_state_changed = true;
		/* 3 pulse code == 1 pulse func close && backlight high state*/
		if (panel->pwm_params.oplus_pwm_switch_state == PWM_SWITCH_HIGH_STATE
				&& !oplus_panel_pwm_onepulse_is_enabled(panel)) {
			rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_PWM_SWITCH_THREEPULSE);
		}
	}

	mutex_unlock(&panel->panel_lock);

	return rc;
}

int oplus_display_panel_get_pwm_pulse(void *data)
{
	int rc = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;
	uint32_t *enabled = data;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->pwm_params.pwm_onepulse_support) {
		LCD_WARN("Falied to get pwm pulse status, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&panel->panel_lock);

	*enabled = panel->pwm_params.pwm_onepulse_enabled;

	mutex_unlock(&panel->panel_lock);
	mutex_unlock(&display->display_lock);
	LCD_INFO("Get pwm onepulse status: %d\n", *enabled);

	return rc;
}

int oplus_display_panel_set_pwm_pulse(void *data)
{
	int rc = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;
	uint32_t *enabled = data;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->pwm_params.pwm_onepulse_support) {
		LCD_WARN("Falied to set pwm onepulse status, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	LCD_INFO("Set pwm onepulse status: %d\n", *enabled);

	if (*enabled == panel->pwm_params.pwm_onepulse_enabled) {
		LCD_WARN("Skip setting duplicate pwm onepulse status: %d\n", *enabled);
		rc = -EFAULT;
		return rc;
	}

	mutex_lock(&display->display_lock);
	rc = oplus_panel_update_pwm_pulse_lock(panel, *enabled);
	mutex_unlock(&display->display_lock);

	return rc;
}
/* end for pwm onepulse switch */
/* add onepulse switch debug */
ssize_t oplus_get_pwm_pulse_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	int rc = 0;
	u32 enabled = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->pwm_params.pwm_onepulse_support) {
		LCD_ERR("Falied to get pwm pulse status, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	mutex_lock(&display->display_lock);
	mutex_lock(&panel->panel_lock);

	enabled = panel->pwm_params.pwm_onepulse_enabled;

	mutex_unlock(&panel->panel_lock);
	mutex_unlock(&display->display_lock);
	LCD_INFO("Get pwm pulse status: %d\n", enabled);

	return sysfs_emit(buf, "%d\n", enabled);
}

ssize_t oplus_set_pwm_pulse_debug(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int rc = 0;
	u32 enabled = 0;
	struct dsi_display *display = get_main_display();
	struct dsi_panel *panel = NULL;

	if (!display || !display->panel) {
		LCD_ERR("Invalid display or panel\n");
		rc = -EINVAL;
		return rc;
	}

	panel = display->panel;

	if (!panel->pwm_params.pwm_onepulse_support) {
		LCD_ERR("Falied to set pwm onepulse status, because it is unsupport\n");
		rc = -EFAULT;
		return rc;
	}

	rc = kstrtou32(buf, 10, &enabled);
	if (rc) {
		LCD_WARN("%s cannot be converted to u32", buf);
		return count;
	}
	LCD_INFO("Set pwm onepulse status: %d\n", enabled);

	mutex_lock(&display->display_lock);
	oplus_panel_update_pwm_pulse_lock(panel, enabled);
	mutex_unlock(&display->display_lock);

	return count;
}
/* end onepulse switch debug */
