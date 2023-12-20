// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 *
 * secureguard moudle main entry
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/syscalls.h>
#include "oplus_secure_hook.h"
#include "oplus_secure_harden.h"
#include "oplus_kevent.h"
#include "oplus_guard_general.h"

#define SG_MAIN    "[secureguard][main_init]"

static int __init __nocfi oplus_secureguard_init(void)
{
	int ret = 0;

	pr_info(SG_MAIN "%s, secureguard main init start.\n", __func__);

	oplus_boot_state_init();
	if (is_unlocked()) {
		/* if unlocked, init & eixt will do nothing */
		pr_err(SG_MAIN "%s, detected unlocked, init do noting.\n", __func__);
		return 0;
	}

	/* as keventupload is the basic func, if failed then directly return */
	ret = oplus_keventupload_init();
	if (ret) {
		pr_err(SG_MAIN "%s, SG kevent init failed. ret %d\n", __func__, ret);
		return 0;
	}

	/* add-on security checking funcs/module have their own success flag,
	 * and resource freee policy, so here just warn.
	 */
	ret = oplus_harden_init();
	if (ret)
		pr_err(SG_MAIN "%s, SG secure harden init failed. ret %d\n", __func__, ret);

	ret = oplus_hook_init();
	if (ret)
		pr_err(SG_MAIN "%s, SG secure hook init failed. ret %d\n", __func__, ret);

	pr_info(SG_MAIN "%s, secureguard main init end & succeed.\n", __func__);
	return 0;
}

static void __exit __nocfi oplus_secureguard_exit(void)
{
	if (is_unlocked()) {
		/* if unlocked, init & eixt will do nothing */
		pr_err(SG_MAIN "%s, detected unlocked, exit do noting.\n", __func__);
	} else {
		oplus_hook_exit();
		oplus_harden_exit();
		oplus_keventupload_exit();
	}
}

module_init(oplus_secureguard_init);
module_exit(oplus_secureguard_exit);
MODULE_LICENSE("GPL");
