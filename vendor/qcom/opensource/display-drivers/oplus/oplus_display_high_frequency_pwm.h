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

#ifndef _OPLUS_DISPLAY_HIGH_FREQUENCY_PWM_H_
#define _OPLUS_DISPLAY_HIGH_FREQUENCY_PWM_H_

/* please just only include linux common head file  */
#include <linux/err.h>
#include "dsi_panel.h"

enum oplus_pwm_trubo_type {
	OPLUS_PWM_TRUBO_CLOSE = 0,
	OPLUS_PWM_TRUBO_SWITCH = 1,
	OPLUS_PWM_TRUBO_GLOBAL_OPEN_NO_SWITCH = 2,
};

enum oplus_pwm_turbo_log_level {
	OPLUS_PWM_TURBO_LOG_LEVEL_ERR = 0,
	OPLUS_PWM_TURBO_LOG_LEVEL_WARN = 1,
	OPLUS_PWM_TURBO_LOG_LEVEL_INFO = 2,
	OPLUS_PWM_TURBO_LOG_LEVEL_DEBUG = 3,
};

enum PWM_SWITCH_STATE{
	PWM_SWITCH_LOW_STATE = 0,
	PWM_SWITCH_HIGH_STATE,
};

/* -------------------- extern ---------------------------------- */
extern unsigned int oplus_lcd_log_level;


/* -------------------- pwm turbo debug log-------------------------------------------  */
#define PWM_TURBO_ERR(fmt, arg...)	\
	do {	\
		if (oplus_lcd_log_level >= OPLUS_LOG_LEVEL_ERR)	\
			pr_err("[PWM_TURBO][ERR][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define PWM_TURBO_WARN(fmt, arg...)	\
	do {	\
		pr_warn("[PWM_TURBO][WARN][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define PWM_TURBO_INFO(fmt, arg...)	\
	do { \
		if(1) \
			pr_info("[PWM_TURBO][INFO][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define PWM_TURBO_DEBUG(fmt, arg...)	\
	do {	\
		if (oplus_lcd_log_level >= OPLUS_LOG_LEVEL_DEBUG)	 \
			pr_info("[PWM_TURBO][DEBUG][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

/* -------------------- function implementation ---------------------------------------- */
inline bool oplus_panel_pwm_turbo_is_enabled(struct dsi_panel *panel);
inline bool oplus_panel_pwm_turbo_switch_state(struct dsi_panel *panel);
struct oplus_pwm_turbo_params *oplus_pwm_turbo_get_params(void);
int oplus_pwm_turbo_probe(struct dsi_panel *panel);
inline bool oplus_panel_pwm_turbo_is_enabled(struct dsi_panel *panel);
int oplus_panel_pwm_switch_timing_switch(struct dsi_panel *panel);
int oplus_panel_send_pwm_turbo_dcs_unlock(struct dsi_panel *panel, bool enabled);
int oplus_panel_update_pwm_turbo_lock(struct dsi_panel *panel, bool enabled);
int oplus_panel_pwm_switch(struct dsi_panel *panel, u32 *backlight_level);
int oplus_panel_pwm_switch_tx_cmd(struct dsi_panel *panel);
int oplus_pwm_set_power_on(struct dsi_panel *panel);
int oplus_hbm_pwm_state(struct dsi_panel *panel, bool hbm_state);
int oplus_display_panel_set_pwm_turbo(void *data);
int oplus_display_panel_get_pwm_turbo(void *data);
inline bool oplus_panel_pwm_onepulse_is_enabled(struct dsi_panel *panel);
inline bool oplus_panel_pwm_onepulse_switch_state(struct dsi_panel *panel);
int oplus_panel_send_pwm_pulse_dcs_unlock(struct dsi_panel *panel, bool enabled);
int oplus_panel_update_pwm_pulse_lock(struct dsi_panel *panel, bool enabled);
int oplus_display_panel_set_pwm_pulse(void *data);
int oplus_display_panel_get_pwm_pulse(void *data);
int oplus_panel_pwm_switch_wait_te_tx_cmd(struct dsi_panel *panel, u32 pwm_switch_cmd);
void oplus_pwm_disable_duty_set_work_handler(struct work_struct *work);
/* -------------------- oplus api nodes ----------------------------------------------- */
ssize_t oplus_get_pwm_turbo_debug(struct kobject *obj, struct kobj_attribute *attr, char *buf);
ssize_t oplus_set_pwm_turbo_debug(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count);
ssize_t oplus_get_pwm_pulse_debug(struct kobject *obj, struct kobj_attribute *attr, char *buf);
ssize_t oplus_set_pwm_pulse_debug(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count);
/* config */

#endif /* _OPLUS_DISPLAY_HIGH_FREQUENCY_PWM_H_ */
