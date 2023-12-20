// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 *
 * Common functions for secureguard.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include "oplus_guard_general.h"

/* secureguard sub-module string for print */
#define SG_COMM    "[secureaguard][COMM]"

enum{
	BOOT_STATE__GREEN,
	BOOT_STATE__ORANGE,
	BOOT_STATE__YELLOW,
	BOOT_STATE__RED,
};

static int __ro_after_init g_boot_state = BOOT_STATE__GREEN;

bool is_unlocked(void)
{
	return g_boot_state == BOOT_STATE__ORANGE;
}

extern char verified_bootstate[];

void oplus_boot_state_init(void)
{
	pr_err(SG_COMM "%s, verified_bootstate is %s .\n", __func__, verified_bootstate);
	if (strstr(verified_bootstate, "orange"))
		g_boot_state = BOOT_STATE__ORANGE;
	else
		g_boot_state = BOOT_STATE__GREEN;
}
