// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2021 Oplus. All rights reserved.
 */

#ifndef __OPLUS_CHG_MUTUAL_H__
#define __OPLUS_CHG_MUTUAL_H__

#define mutual_info_to_param(cmd, data_size) \
	((((cmd) & 0xff) << 16) | ((data_size) & 0xffff))
#define mutual_info_to_cmd(param) (((param) >> 16) & 0xff)
#define mutual_info_to_data_size(param) ((param) & 0xffff)

enum oplus_chg_mutual_cmd_type {
	CMD_WLS_THIRD_PART_AUTH,
	CMD_GAUGE_CALIB_OBTAIN,
	CMD_GAUGE_CALIB_UPDATE,
	CMD_DEC_CV_DOWN_OBTAIN,
	CMD_DEC_CV_DOWN_UPDATE,
	CMD_PRECHG_R_OBTAIN,
	CMD_DISCHG_BOOST_ERR_OBTAIN,
};

enum oplus_chg_mutual_cmd_error {
	CMD_ACK_OK,
	CMD_ERROR_CHIP_NULL,
	CMD_ERROR_DATA_NULL,
	CMD_ERROR_DATA_INVALID,
	CMD_ERROR_HIDL_NOT_READY,
	CMD_ERROR_TIME_OUT,
};

struct oplus_chg_mutual_notifier {
	const char *name;
	enum oplus_chg_mutual_cmd_type cmd;
	struct notifier_block nb;
	struct list_head list;
	void *priv_data;
};

int oplus_chg_reg_mutual_notifier(struct oplus_chg_mutual_notifier *notifier);
int oplus_chg_unreg_mutual_notifier(struct oplus_chg_mutual_notifier *notifier);
int oplus_chg_set_mutual_cmd(enum oplus_chg_mutual_cmd_type cmd, u32 data_size, const void *data_buf);

#endif /* __OPLUS_CHG_MUTUAL_H__ */
