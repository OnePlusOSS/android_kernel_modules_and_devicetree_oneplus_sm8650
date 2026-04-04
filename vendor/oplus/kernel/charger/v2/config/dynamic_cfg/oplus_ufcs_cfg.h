// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */

#define LOG_BUF_SIZE	4096
static char *g_log_buf;

static void oplus_ufcs_update_misc_config(
	struct oplus_param_head *param_head, struct oplus_ufcs *chip)
{
	struct oplus_cfg_data_head *data_head;
	struct oplus_ufcs_config *config = &chip->config;
	int32_t buf;
	int rc;

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,target_vbus_mv");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,target_vbus_mv data error, rc=%d\n", rc);
			break;
		}
		config->target_vbus_mv = le32_to_cpu(buf);
		chg_info("[TEST]:target_vbus_mv = %d\n", config->target_vbus_mv);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,curr_max_ma");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,curr_max_ma data error, rc=%d\n", rc);
			break;
		}
		config->curr_max_ma = le32_to_cpu(buf);
		chg_info("[TEST]:curr_max_ma = %d\n", config->curr_max_ma);
		vote(chip->ufcs_curr_votable, MAX_VOTER, true, config->curr_max_ma, false);
		break;
	}
}

static void oplus_ufcs_update_charge_strategy_config(
	struct oplus_param_head *param_head, struct oplus_ufcs *chip)
{
	struct oplus_cfg_data_head *data_head;
	int32_t buf;
	int i, rc;
	ssize_t data_len;
	int32_t array_buf[7];
	int log_index = 0;

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_warm_allow_vol");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_warm_allow_vol data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_warm_allow_vol = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_warm_allow_vol = %d\n", chip->limits.ufcs_warm_allow_vol);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_warm_allow_soc");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_warm_allow_soc data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_warm_allow_soc = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_warm_allow_soc = %d\n", chip->limits.ufcs_warm_allow_soc);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_strategy_normal_current");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_strategy_normal_current data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_strategy_normal_current = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_strategy_normal_current = %d\n", chip->limits.ufcs_strategy_normal_current);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_over_high_or_low_current");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_over_high_or_low_current data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_over_high_or_low_current = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_over_high_or_low_current = %d\n", chip->limits.ufcs_over_high_or_low_current);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_full_cool_sw_vbat");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_full_cool_sw_vbat data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_full_cool_sw_vbat = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_full_cool_sw_vbat = %d\n", chip->limits.ufcs_full_cool_sw_vbat);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_full_normal_sw_vbat");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_full_normal_sw_vbat data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_full_normal_sw_vbat = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_full_normal_sw_vbat = %d\n", chip->limits.ufcs_full_normal_sw_vbat);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_full_normal_hw_vbat");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_full_normal_hw_vbat data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_full_normal_hw_vbat = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_full_normal_hw_vbat = %d\n", chip->limits.ufcs_full_normal_hw_vbat);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_full_cool_sw_vbat_third");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_full_cool_sw_vbat_third data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_full_cool_sw_vbat_third = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_full_cool_sw_vbat_third = %d\n", chip->limits.ufcs_full_cool_sw_vbat_third);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_full_normal_sw_vbat_third");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_full_normal_sw_vbat_third data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_full_normal_sw_vbat_third = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_full_normal_sw_vbat_third = %d\n", chip->limits.ufcs_full_normal_sw_vbat_third);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_full_normal_hw_vbat_third");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_full_normal_hw_vbat_third data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_full_normal_hw_vbat_third = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_full_normal_hw_vbat_third = %d\n", chip->limits.ufcs_full_normal_hw_vbat_third);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_full_warm_vbat");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_full_warm_vbat data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_full_warm_vbat = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_full_warm_vbat = %d\n", chip->limits.ufcs_full_warm_vbat);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_timeout_third");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_timeout_third data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_timeout_third = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_timeout_third = %d\n", chip->limits.ufcs_timeout_third);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_timeout_oplus");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_timeout_oplus data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_timeout_oplus = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_timeout_oplus = %d\n", chip->limits.ufcs_timeout_oplus);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_ibat_over_third");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_ibat_over_third data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_ibat_over_third = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_ibat_over_third = %d\n", chip->limits.ufcs_ibat_over_third);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_ibat_over_oplus");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_ibat_over_oplus data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_ibat_over_oplus = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_ibat_over_oplus = %d\n", chip->limits.ufcs_ibat_over_oplus);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_bcc_max_curr");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_bcc_max_curr data error, rc=%d\n", rc);
			break;
		}
		chip->bcc_max_curr = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_bcc_max_curr = %d\n", chip->bcc_max_curr);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_bcc_min_curr");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_bcc_min_curr data error, rc=%d\n", rc);
			break;
		}
		chip->bcc_min_curr = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_bcc_min_curr = %d\n", chip->bcc_min_curr);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_bcc_exit_curr");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_bcc_exit_curr data error, rc=%d\n", rc);
			break;
		}
		chip->bcc_exit_curr = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_bcc_exit_curr = %d\n", chip->bcc_exit_curr);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_strategy_batt_high_temp");
	while (data_head != NULL) {
		data_len = oplus_cfg_get_data_size(data_head);
		if (data_len / sizeof(array_buf[0]) > 6) {
			chg_err("Too much configuration data, data_len=%ld\n", data_len / sizeof(array_buf[0]));
			break;
		}
		if ((data_len % sizeof(array_buf[0])) != 0) {
			chg_err("configuration data format error\n");
			break;
		}
		rc = oplus_cfg_get_data(data_head, (u8 *)array_buf, data_len);
		if (rc < 0) {
			chg_err("get oplus,ufcs_strategy_batt_high_temp data error, rc=%d\n", rc);
			break;
		}
		for (i = 0; i < 6; i++) {
			if (i >= (data_len / sizeof(array_buf[0])))
				array_buf[i] = 0;
			if (g_log_buf) {
				log_index += snprintf(g_log_buf + log_index,
					LOG_BUF_SIZE - log_index - 1, "%s%d",
					(i == 0) ? " " : ", ", le32_to_cpu(array_buf[i]));
				g_log_buf[log_index] = 0;
			}
		}
		chip->limits.ufcs_strategy_batt_high_temp0 = le32_to_cpu(array_buf[0]);
		chip->limits.ufcs_strategy_batt_high_temp1 = le32_to_cpu(array_buf[1]);
		chip->limits.ufcs_strategy_batt_high_temp2 = le32_to_cpu(array_buf[2]);
		chip->limits.ufcs_strategy_batt_low_temp0 = le32_to_cpu(array_buf[3]);
		chip->limits.ufcs_strategy_batt_low_temp1 = le32_to_cpu(array_buf[4]);
		chip->limits.ufcs_strategy_batt_low_temp2 = le32_to_cpu(array_buf[5]);
		if (g_log_buf)
			chg_info("[TEST]:ufcs_strategy_batt_high_temp = {%s }\n", g_log_buf);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_strategy_high_current");
	while (data_head != NULL) {
		data_len = oplus_cfg_get_data_size(data_head);
		if (data_len / sizeof(array_buf[0]) > 6) {
			chg_err("Too much configuration data, data_len=%ld\n", data_len / sizeof(array_buf[0]));
			break;
		}
		if ((data_len % sizeof(array_buf[0])) != 0) {
			chg_err("configuration data format error\n");
			break;
		}
		rc = oplus_cfg_get_data(data_head, (u8 *)array_buf, data_len);
		if (rc < 0) {
			chg_err("get oplus,ufcs_strategy_high_current data error, rc=%d\n", rc);
			break;
		}
		for (i = 0; i < 6; i++) {
			if (i >= (data_len / sizeof(array_buf[0])))
				array_buf[i] = 0;
			if (g_log_buf) {
				log_index += snprintf(g_log_buf + log_index,
					LOG_BUF_SIZE - log_index - 1, "%s%d",
					(i == 0) ? " " : ", ", le32_to_cpu(array_buf[i]));
				g_log_buf[log_index] = 0;
			}
		}
		chip->limits.ufcs_strategy_high_current0 = le32_to_cpu(array_buf[0]);
		chip->limits.ufcs_strategy_high_current1 = le32_to_cpu(array_buf[1]);
		chip->limits.ufcs_strategy_high_current2 = le32_to_cpu(array_buf[2]);
		chip->limits.ufcs_strategy_low_current0 = le32_to_cpu(array_buf[3]);
		chip->limits.ufcs_strategy_low_current1 = le32_to_cpu(array_buf[4]);
		chip->limits.ufcs_strategy_low_current2 = le32_to_cpu(array_buf[5]);
		if (g_log_buf)
			chg_info("[TEST]:ufcs_strategy_high_current = {%s }\n", g_log_buf);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_charge_strategy_temp");
	while (data_head != NULL) {
		data_len = oplus_cfg_get_data_size(data_head);
		if (data_len / sizeof(array_buf[0]) > 7) {
			chg_err("Too much configuration data, data_len=%ld\n", data_len / sizeof(array_buf[0]));
			break;
		}
		if ((data_len % sizeof(array_buf[0])) != 0) {
			chg_err("configuration data format error\n");
			break;
		}
		chip->limits.ufcs_strategy_temp_num = data_len / sizeof(array_buf[0]);
		rc = oplus_cfg_get_data(data_head, (u8 *)array_buf, data_len);
		if (rc < 0) {
			chg_err("get oplus,ufcs_charge_strategy_temp data error, rc=%d\n", rc);
			break;
		}
		for (i = 0; i < 7; i++) {
			if (i >= (data_len / sizeof(array_buf[0])))
				array_buf[i] = 0;
			if (g_log_buf) {
				log_index += snprintf(g_log_buf + log_index,
					LOG_BUF_SIZE - log_index - 1, "%s%d",
					(i == 0) ? " " : ", ", le32_to_cpu(array_buf[i]));
				g_log_buf[log_index] = 0;
			}
		}
		chip->limits.ufcs_batt_over_low_temp = le32_to_cpu(array_buf[0]);
		chip->limits.ufcs_little_cold_temp = le32_to_cpu(array_buf[1]);
		chip->limits.ufcs_cool_temp = le32_to_cpu(array_buf[2]);
		chip->limits.ufcs_little_cool_temp = le32_to_cpu(array_buf[3]);
		chip->limits.ufcs_normal_low_temp = le32_to_cpu(array_buf[4]);
		chip->limits.ufcs_normal_high_temp = le32_to_cpu(array_buf[5]);
		chip->limits.ufcs_batt_over_high_temp = le32_to_cpu(array_buf[6]);
		chip->limits.default_ufcs_normal_high_temp =
			chip->limits.ufcs_normal_high_temp;
		chip->limits.default_ufcs_normal_low_temp =
			chip->limits.ufcs_normal_low_temp;
		chip->limits.default_ufcs_little_cool_temp =
			chip->limits.ufcs_little_cool_temp;
		chip->limits.default_ufcs_cool_temp = chip->limits.ufcs_cool_temp;
		chip->limits.default_ufcs_little_cold_temp =
			chip->limits.ufcs_little_cold_temp;
		if (g_log_buf)
			chg_info("[TEST]:ufcs_charge_strategy_temp = {%s }\n", g_log_buf);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_little_cool_high_temp");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus,ufcs_little_cool_high_temp data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_little_cool_high_temp = le32_to_cpu(buf);
		chip->limits.default_ufcs_little_cool_high_temp = chip->limits.ufcs_little_cool_high_temp;
		chg_info("[TEST]:ufcs_little_cool_high_temp = %d\n", chip->limits.ufcs_little_cool_high_temp);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,ufcs_low_curr_full_strategy_temp");
	while (data_head != NULL) {
		data_len = oplus_cfg_get_data_size(data_head);
		if (data_len / sizeof(array_buf[0]) > 4) {
			chg_err("Too much configuration data, data_len=%ld\n", data_len / sizeof(array_buf[0]));
			break;
		}
		if ((data_len % sizeof(array_buf[0])) != 0) {
			chg_err("configuration data format error\n");
			break;
		}
		rc = oplus_cfg_get_data(data_head, (u8 *)array_buf, data_len);
		if (rc < 0) {
			chg_err("get oplus,ufcs_low_curr_full_strategy_temp data error, rc=%d\n", rc);
			break;
		}
		for (i = 0; i < 4; i++) {
			if (i >= (data_len / sizeof(array_buf[0])))
				array_buf[i] = 0;
			if (g_log_buf) {
				log_index += snprintf(g_log_buf + log_index,
					LOG_BUF_SIZE - log_index - 1, "%s%d",
					(i == 0) ? " " : ", ", le32_to_cpu(array_buf[i]));
				g_log_buf[log_index] = 0;
			}
		}
		chip->limits.ufcs_low_curr_full_cool_temp = le32_to_cpu(array_buf[0]);
		chip->limits.ufcs_low_curr_full_little_cool_temp = le32_to_cpu(array_buf[1]);
		chip->limits.ufcs_low_curr_full_normal_low_temp = le32_to_cpu(array_buf[2]);
		chip->limits.ufcs_low_curr_full_normal_high_temp = le32_to_cpu(array_buf[3]);
		if (g_log_buf)
			chg_info("[TEST]:ufcs_low_curr_full_strategy_temp = {%s }\n", g_log_buf);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus_spec,ufcs_low_temp");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus_spec,ufcs_low_temp data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_low_temp = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_low_temp = %d\n", chip->limits.ufcs_low_temp);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus_spec,ufcs_high_temp");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus_spec,ufcs_high_temp data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_high_temp = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_high_temp = %d\n", chip->limits.ufcs_high_temp);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus_spec,ufcs_low_soc");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus_spec,ufcs_low_soc data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_low_soc = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_low_soc = %d\n", chip->limits.ufcs_low_soc);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus_spec,ufcs_high_soc");
	while (data_head != NULL) {
		rc = oplus_cfg_get_data(data_head, (u8 *)&buf, sizeof(buf));
		if (rc < 0) {
			chg_err("get oplus_spec,ufcs_high_soc data error, rc=%d\n", rc);
			break;
		}
		chip->limits.ufcs_high_soc = le32_to_cpu(buf);
		chg_info("[TEST]:ufcs_high_soc = %d\n", chip->limits.ufcs_high_soc);
		break;
	}
}

static void oplus_ufcs_update_low_curr_full_curves_config(
	struct oplus_param_head *param_head, struct oplus_ufcs *chip)
{
	struct oplus_cfg_data_head *data_head;
	int32_t buf[FULL_UFCS_SYS_MAX][3];
	int i, n, rc;
	ssize_t data_len;
	int log_index = 0;

	data_head = oplus_cfg_find_param_by_name(param_head, "ufcs_charge_low_curr_full:strategy_temp_little_cool");
	while (data_head != NULL) {
		data_len = oplus_cfg_get_data_size(data_head);
		if (data_len / sizeof(buf[0]) / 3 > FULL_UFCS_SYS_MAX) {
			chg_err("Too much configuration data, data_len=%ld\n", data_len / sizeof(buf[0]) / 3);
			break;
		}
		chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL].full_curve_num =
			data_len / sizeof(buf[0]) / 3;
		rc = oplus_cfg_get_data(data_head, (u8 *)buf, data_len);
		if (rc < 0) {
			chg_err("get ufcs_charge_low_curr_full:strategy_temp_little_cool data error, rc=%d\n", rc);
			break;
		}
		for (i = 0; i < FULL_UFCS_SYS_MAX; i++) {
			for (n = 0; n < 3; n++) {
				if (i >= data_len / sizeof(buf[0]) / 3)
					buf[i][n] = 0;
				if (g_log_buf) {
					log_index += snprintf(g_log_buf + log_index,
						LOG_BUF_SIZE - log_index - 1, "%s%d",
						(i == 0) ? " " : ", ", le32_to_cpu(buf[i][n]));
					g_log_buf[log_index] = 0;
				}
			}
			chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL].full_curves[i].iterm =
				le32_to_cpu(buf[i][0]);
			chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL].full_curves[i].vterm =
				le32_to_cpu(buf[i][1]);
			chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_LITTLE_COOL].full_curves[i].iterm =
				le32_to_cpu(buf[i][2]);
		}
		if (g_log_buf)
			chg_info("[TEST]:ufcs_charge_low_curr_full:strategy_temp_little_cool = {%s }\n", g_log_buf);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "ufcs_charge_low_curr_full:strategy_temp_normal_low");
	while (data_head != NULL) {
		data_len = oplus_cfg_get_data_size(data_head);
		if (data_len / sizeof(buf[0]) / 3 > FULL_UFCS_SYS_MAX) {
			chg_err("Too much configuration data, data_len=%ld\n", data_len / sizeof(buf[0]) / 3);
			break;
		}
		chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW].full_curve_num =
			data_len / sizeof(buf[0]) / 3;
		rc = oplus_cfg_get_data(data_head, (u8 *)buf, data_len);
		if (rc < 0) {
			chg_err("get ufcs_charge_low_curr_full:strategy_temp_normal_low data error, rc=%d\n", rc);
			break;
		}
		for (i = 0; i < FULL_UFCS_SYS_MAX; i++) {
			for (n = 0; n < 3; n++) {
				if (i >= data_len / sizeof(buf[0]) / 3)
					buf[i][n] = 0;
				if (g_log_buf) {
					log_index += snprintf(g_log_buf + log_index,
						LOG_BUF_SIZE - log_index - 1, "%s%d",
						(i == 0) ? " " : ", ", le32_to_cpu(buf[i][n]));
					g_log_buf[log_index] = 0;
				}
			}
			chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW].full_curves[i].iterm =
				le32_to_cpu(buf[i][0]);
			chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW].full_curves[i].vterm =
				le32_to_cpu(buf[i][1]);
			chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_LOW].full_curves[i].iterm =
				le32_to_cpu(buf[i][2]);
		}
		if (g_log_buf)
			chg_info("[TEST]:ufcs_charge_low_curr_full:strategy_temp_normal_low = {%s }\n", g_log_buf);
		break;
	}

	data_head = oplus_cfg_find_param_by_name(param_head, "ufcs_charge_low_curr_full:strategy_temp_normal_high");
	while (data_head != NULL) {
		data_len = oplus_cfg_get_data_size(data_head);
		if (data_len / sizeof(buf[0]) / 3 > FULL_UFCS_SYS_MAX) {
			chg_err("Too much configuration data, data_len=%ld\n", data_len / sizeof(buf[0]) / 3);
			break;
		}
		chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH].full_curve_num =
			data_len / sizeof(buf[0]) / 3;
		rc = oplus_cfg_get_data(data_head, (u8 *)buf, data_len);
		if (rc < 0) {
			chg_err("get ufcs_charge_low_curr_full:strategy_temp_normal_high data error, rc=%d\n", rc);
			break;
		}
		for (i = 0; i < FULL_UFCS_SYS_MAX; i++) {
			for (n = 0; n < 3; n++) {
				if (i >= data_len / sizeof(buf[0]) / 3)
					buf[i][n] = 0;
				if (g_log_buf) {
					log_index += snprintf(g_log_buf + log_index,
						LOG_BUF_SIZE - log_index - 1, "%s%d",
						(i == 0) ? " " : ", ", le32_to_cpu(buf[i][n]));
					g_log_buf[log_index] = 0;
				}
			}
			chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH].full_curves[i].iterm =
				le32_to_cpu(buf[i][0]);
			chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH].full_curves[i].vterm =
				le32_to_cpu(buf[i][1]);
			chip->low_curr_full_curves_temp[UFCS_LOW_CURR_FULL_CURVE_TEMP_NORMAL_HIGH].full_curves[i].iterm =
				le32_to_cpu(buf[i][2]);
		}
		if (g_log_buf)
			chg_info("[TEST]:ufcs_charge_low_curr_full:strategy_temp_normal_high = {%s }\n", g_log_buf);
		break;
	}
}

static void oplus_ufcs_update_new_low_curr_full_curves_config(
	struct oplus_param_head *param_head, struct oplus_ufcs *chip)
{
	struct oplus_chg_strategy *strategy;

	strategy = oplus_chg_strategy_alloc_by_param_head("low_curr_full_strategy", "ufcs_oplus_lcf_strategy", param_head);
	if (IS_ERR_OR_NULL(strategy)) {
		chg_err("alloc ufcs_oplus_lcf_strategy error, rc=%ld", PTR_ERR(strategy));
	} else {
		(void)oplus_chg_strategy_release(chip->oplus_lcf_strategy[0]);
		chip->oplus_lcf_strategy[0] = strategy;
		chg_info("[TEST]: update oplus_lcf_strategy startegy\n");
	}

	strategy = oplus_chg_strategy_alloc_by_param_head("low_curr_full_strategy", "ufcs_third_lcf_strategy", param_head);
	if (IS_ERR_OR_NULL(strategy)) {
		chg_err("alloc ufcs_third_lcf_strategy error, rc=%ld", PTR_ERR(strategy));
	} else {
		(void)oplus_chg_strategy_release(chip->third_lcf_strategy[0]);
		chip->third_lcf_strategy[0] = strategy;
		chg_info("[TEST]: update third_lcf_strategy startegy\n");
	}
}

static void oplus_ufcs_update_lcf_curves_strategy_config(
	struct oplus_param_head *param_head, struct oplus_ufcs *chip)
{
	if (oplus_get_chg_spec_version() >= OPLUS_CHG_SPEC_VER_V3P7)
		oplus_ufcs_update_new_low_curr_full_curves_config(param_head, chip);
	else
		oplus_ufcs_update_low_curr_full_curves_config(param_head, chip);
}

#define STRATEGY_NAME_LEN 128
static void oplus_ufcs_update_curves_config(
	struct oplus_param_head *param_head, struct oplus_ufcs *chip)
{
	struct oplus_chg_strategy *strategy;
	struct oplus_cfg_data_head *data_head;
	char name[STRATEGY_NAME_LEN] = { 0 };
	ssize_t data_len;
	int rc = 0;

	data_head = oplus_cfg_find_param_by_name(param_head, "oplus,curve_strategy_name");
	if (data_head == NULL)
		return;
	data_len = oplus_cfg_get_data_size(data_head);
	if (data_len >= STRATEGY_NAME_LEN) {
		chg_err("strategy name is too long\n");
		return;
	}
	rc = oplus_cfg_get_data(data_head, name, data_len);
	if (rc < 0) {
		chg_err("get oplus,curve_strategy_name error, rc=%d\n", rc);
		return;
	}
	name[data_len] = 0;
	chg_info("[TEST]:curve_strategy_name = %s\n", name);

	strategy = oplus_chg_strategy_alloc_by_param_head(name, "ufcs_charge_oplus_strategy", param_head);
	if (IS_ERR_OR_NULL(strategy)) {
		chg_err("alloc oplus curve startegy error, rc=%ld", PTR_ERR(strategy));
	} else {
		(void)oplus_chg_strategy_release(chip->oplus_curve_strategy);
		chip->oplus_curve_strategy = strategy;
		chg_info("[TEST]: update ufcs oplus curve startegy\n");
	}

	strategy = oplus_chg_strategy_alloc_by_param_head(name, "ufcs_charge_third_strategy", param_head);
	if (IS_ERR_OR_NULL(strategy)) {
		chg_err("alloc third curve startegy error, rc=%ld", PTR_ERR(strategy));
	} else {
		(void)oplus_chg_strategy_release(chip->third_curve_strategy);
		chip->third_curve_strategy = strategy;
		chg_info("[TEST]: update ufcs third curve startegy\n");
	}
}

static int oplus_ufcs_update_config(void *data, struct oplus_param_head *param_head)
{
	struct oplus_ufcs *chip;

	if (data == NULL) {
		chg_err("data is NULL\n");
		return -EINVAL;
	}
	if (param_head == NULL) {
		chg_err("param_head is NULL\n");
		return -EINVAL;
	}
	chip = (struct oplus_ufcs *)data;

	g_log_buf = kzalloc(LOG_BUF_SIZE, GFP_KERNEL);
	oplus_ufcs_update_misc_config(param_head, chip);
	oplus_ufcs_update_charge_strategy_config(param_head, chip);
	oplus_ufcs_update_lcf_curves_strategy_config(param_head, chip);
	oplus_ufcs_update_curves_config(param_head, chip);

	if (g_log_buf != NULL) {
		kfree(g_log_buf);
		g_log_buf = NULL;
	}

	return 0;
}

static int oplus_ufcs_reg_debug_config(struct oplus_ufcs *chip)
{
	int rc;

	chip->debug_cfg.type = OPLUS_CHG_UFCS_PARAM;
	chip->debug_cfg.update = oplus_ufcs_update_config;
	chip->debug_cfg.priv_data = chip;
	rc = oplus_cfg_register(&chip->debug_cfg);
	if (rc < 0)
		chg_err("ufcs cfg register error, rc=%d\n", rc);

	return 0;
}

static void oplus_ufcs_unreg_debug_config(struct oplus_ufcs *chip)
{
	oplus_cfg_unregister(&chip->debug_cfg);
}
