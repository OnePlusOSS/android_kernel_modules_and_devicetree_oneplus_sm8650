// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */

/* This file is used internally */

#ifndef __OPLUS_UFCS_NOTIFY_H__
#define __OPLUS_UFCS_NOTIFY_H__

struct ufcs_class;

enum ufcs_notify_state {
	UFCS_NOTIFY_SOURCE_HW_RESET = 0,
	UFCS_NOTIFY_CABLE_HW_RESET,
	UFCS_NOTIFY_POWER_CHANGE,
	UFCS_NOTIFY_EXIT,
	UFCS_NOTIFY_TEST_MODE_CHANGED,
	UFCS_NOTIFY_VOL_ACC_TEST_MODE_CHANGED,
	UFCS_NOTIFY_ERR_FLAG,
	UFCS_NOTIFY_FIFO_OVERFLOW,
	UFCS_NOTIFY_RESTART,
};

enum ufcs_notify_state_value {
	UFCS_NOTIFY_EXIT_BY_CRASH = 0,
	UFCS_NOTIFY_RESTART_FROM_CRASH,
	UFCS_NOTIFY_FGRESET_EXIT_COMM,
	UFCS_NOTIFY_FGRESET_RESTART_COMM,
	UFCS_NOTIFY_UFCS_RESET_NOTIFY,
};
#if IS_ENABLED(CONFIG_OPLUS_UFCS_CLASS)
int ufcs_reg_event_notifier(struct notifier_block *nb);
void ufcs_unreg_event_notifier(struct notifier_block *nb);
void ufcs_send_state(enum ufcs_notify_state state, void *v);
#else

__maybe_unused
static inline int ufcs_reg_event_notifier(struct notifier_block *nb)
{
	return -EINVAL;
}

__maybe_unused
static inline void ufcs_unreg_event_notifier(struct notifier_block *nb)
{
	return;
}

__maybe_unused
static inline void ufcs_send_state(enum ufcs_notify_state state, void *v)
{
	return;
}

#endif /* CONFIG_OPLUS_UFCS_CLASS */

#endif /* __OPLUS_UFCS_NOTIFY_H__ */
