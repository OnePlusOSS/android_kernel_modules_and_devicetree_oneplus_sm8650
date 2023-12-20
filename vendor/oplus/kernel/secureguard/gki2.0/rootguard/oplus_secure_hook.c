// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 *
 * secureguard hook check init
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/cred.h>
#include <linux/kallsyms.h>
#include <linux/tracepoint.h>
#include <trace/events/syscalls.h>
#include "oplus_secure_hook.h"
#include "oplus_root_hook.h"
#include "oplus_exec_hook.h"
#include "oplus_harden_hook.h"
#include "oplus_guard_general.h"

/*any vendor hook tracepoint will work, in order to find __tracepoint_sys_enter and __tracepoint_sys_exit*/
extern struct tracepoint  __tracepoint_dma_fence_emit;

#define OPLUS_SYS_EXIT_STRING		"__tracepoint_sys_exit"
#define OPLUS_SYS_ENTER_STRING		"__tracepoint_sys_enter"
#define OPLUS_TRACEPOINT_STRING		"__tracepoint_"

/* secureguard sub-module string for print */
#define SG_HOOK_INIT	"[secureguard][hook_init]"

struct tracepoint *p_tp_sys_enter;
struct tracepoint *p_tp_sys_exit;

/*
 * probe: handler funtion
 * data: data
 * probe_reg_flag: 1:probe register  0:probe not register
 */
struct oplus_hook_str {
	void *probe;
	void *data;
	unsigned short probe_reg_flag;
};

static struct oplus_hook_str oplus_pre_hook_array[] = {
#ifdef CONFIG_OPLUS_FEATURE_SECURE_ROOTGUARD
	{oplus_root_check_pre_handler, NULL, 0},
#endif /* CONFIG_OPLUS_FEATURE_SECURE_ROOTGUARD */
#ifdef CONFIG_OPLUS_FEATURE_SECURE_CAPGUARD
	{oplus_harden_pre_handler, NULL, 0},
#endif /* CONFIG_OPLUS_FEATURE_SECURE_CAPGUARD */
};

static struct oplus_hook_str oplus_post_hook_array[] = {
#ifdef CONFIG_OPLUS_FEATURE_SECURE_EXECGUARD
	{oplus_exe_block_ret_handler, NULL, 0},
#endif /* CONFIG_OPLUS_FEATURE_SECURE_EXECGUARD */
#ifdef CONFIG_OPLUS_FEATURE_SECURE_ROOTGUARD
	{oplus_root_check_post_handler, NULL, 0},
#endif /* CONFIG_OPLUS_FEATURE_SECURE_ROOTGUARD */
};

static unsigned int oplus_get_pre_hook_num(void)
{
	return OPLUS_ARRAY_SIZE(oplus_pre_hook_array);
}

static unsigned int oplus_get_post_hook_num(void)
{
	return OPLUS_ARRAY_SIZE(oplus_post_hook_array);
}

static struct oplus_hook_str *oplus_get_pre_hook(void)
{
	struct oplus_hook_str *p_p_oplus_hook_str = NULL;

	p_p_oplus_hook_str = oplus_pre_hook_array;
	return p_p_oplus_hook_str;
}

static struct oplus_hook_str *oplus_get_post_hook(void)
{
	struct oplus_hook_str *p_p_oplus_hook_str = NULL;

	p_p_oplus_hook_str = oplus_post_hook_array;
	return p_p_oplus_hook_str;
}

static int oplus_hook_init_succeed = 0;
int oplus_hook_init(void)
{
	int ret = 0;
	char test[KSYM_SYMBOL_LEN];
	char test1[KSYM_SYMBOL_LEN], test2[KSYM_SYMBOL_LEN];
	unsigned long base_addr;
	unsigned long sys_exit_addr;
	unsigned long sys_enter_addr;
	unsigned char sys_exit_flag = 0;
	unsigned char sys_enter_flag = 0;
	int range_out = 0;	/* out of range flag */

	unsigned int pre_hook_num = 0;
	unsigned int post_hook_num = 0;
	int cnt = 0;

	struct oplus_hook_str *p_pre_hook = NULL;
	struct oplus_hook_str *p_post_hook = NULL;

	base_addr = (unsigned long)(&__tracepoint_dma_fence_emit);
	pr_info(SG_HOOK_INIT "%s, __tracepoint_oplus_test base_addr at 0x%lx\n", __func__, base_addr);

	/* Calc the address of the sys_exit/sys_enter */
	for (;;) {
		cnt++;
		sprint_symbol(test, base_addr - cnt*(sizeof(__tracepoint_dma_fence_emit)));
		if (strncmp(test, OPLUS_SYS_EXIT_STRING, strlen(OPLUS_SYS_EXIT_STRING)) == 0) {
			sys_exit_addr = base_addr - cnt*(sizeof(__tracepoint_dma_fence_emit));
			sys_exit_flag = 1;
		}

		if (strncmp(test, OPLUS_SYS_ENTER_STRING, strlen(OPLUS_SYS_ENTER_STRING)) == 0) {
			sys_enter_addr = base_addr - cnt*(sizeof(__tracepoint_dma_fence_emit));
			sys_enter_flag = 1;
		}

		/* The calc of the target address is successful, and the calc is breaked. */
		if ((sys_exit_flag == 1) && (sys_enter_flag == 1))
			break;

		/* When the calc function range exceeds the tracepoint area, the calc stops and break it. */
		if (strncmp(test, OPLUS_TRACEPOINT_STRING, strlen(OPLUS_TRACEPOINT_STRING)) != 0) {
			range_out = 1;
			sprint_symbol(test1, (base_addr - cnt*(sizeof(__tracepoint_dma_fence_emit))));
			pr_err(SG_HOOK_INIT "%s, sys_enter/exit could NOT be found, trace size:0x%lx, last symbol:%s.\n",
				__func__, sizeof(__tracepoint_dma_fence_emit), test1);
			break;
		}
	}

	if (sys_exit_flag == 0 || sys_enter_flag == 0 || range_out == 1) {
		pr_err(SG_HOOK_INIT "%s, Calc traceponit failed! sys_enter_flag:%d, sys_exit_flag:%d, range_out:%d.\n",
			__func__, sys_enter_flag, sys_exit_flag, range_out);
		return ret;
	}
	p_tp_sys_exit = (struct tracepoint *)sys_exit_addr;
	p_tp_sys_enter = (struct tracepoint *)sys_enter_addr;
	sprint_symbol(test1, sys_exit_addr);
	sprint_symbol(test2, sys_enter_addr);
	pr_info(SG_HOOK_INIT "%s: sys_exit_addr 0x%lx\n", test1, sys_exit_addr);
	pr_info(SG_HOOK_INIT "%s: sys_enter_addr 0x%lx\n", test2, sys_enter_addr);

	pre_hook_num = oplus_get_pre_hook_num();
	if (pre_hook_num == 0) {
		pr_err(SG_HOOK_INIT "%s:pre_hook_num 0 err.\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	post_hook_num = oplus_get_post_hook_num();
	if (post_hook_num == 0) {
		pr_err(SG_HOOK_INIT "%s:post_hook_num 0 err.\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	p_pre_hook = oplus_get_pre_hook();
	if (p_pre_hook == NULL) {
		pr_err(SG_HOOK_INIT "%s:p_pre_hook NULL!\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	p_post_hook = oplus_get_post_hook();
	if (p_post_hook == NULL) {
		pr_err(SG_HOOK_INIT "%s:p_post_hook NULL!\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	for (cnt = 0; cnt < pre_hook_num; cnt++) {
		/*register pre hook*/
		ret = tracepoint_probe_register(p_tp_sys_enter, p_pre_hook[cnt].probe, NULL);
		if (ret) {
			pr_err(SG_HOOK_INIT "%s:register_trace_sys_enter failed! ret=%d.\n", __func__, ret);
			ret = -EPERM;
			goto exit;
		}
		p_pre_hook[cnt].probe_reg_flag = 1;
	}

	for (cnt = 0; cnt < post_hook_num; cnt++) {
		/*register post hook*/
		ret = tracepoint_probe_register(p_tp_sys_exit, p_post_hook[cnt].probe, NULL);
		if (ret) {
			pr_err(SG_HOOK_INIT "%s:register_trace_sys_exit failed! ret=%d.\n", __func__, ret);
			ret = -EPERM;
			goto exit;
		}
		p_post_hook[cnt].probe_reg_flag = 1;
	}
	oplus_hook_init_succeed = 1;
	return 0;

exit:
	if (ret) {
		for (cnt = post_hook_num - 1; cnt >= 0; cnt--) {
			/*unregister post hook*/
			if (p_post_hook[cnt].probe_reg_flag == 1) {
				tracepoint_probe_unregister(p_tp_sys_exit, p_post_hook[cnt].probe, NULL);
				p_post_hook[cnt].probe_reg_flag = 0;
			}
		}

		for (cnt = pre_hook_num - 1; cnt >= 0; cnt--) {
			/*unregister post hook*/
			if (p_pre_hook[cnt].probe_reg_flag == 1) {
				tracepoint_probe_unregister(p_tp_sys_enter, p_pre_hook[cnt].probe, NULL);
				p_pre_hook[cnt].probe_reg_flag = 0;
			}
		}
	}
	return ret;
}

void oplus_hook_exit(void)
{
	unsigned int pre_hook_num = 0;
	unsigned int post_hook_num = 0;
	int cnt = 0;

	struct oplus_hook_str *p_pre_hook = NULL;
	struct oplus_hook_str *p_post_hook = NULL;

	if (oplus_hook_init_succeed != 1)
		return;

	pre_hook_num = oplus_get_pre_hook_num();
	post_hook_num = oplus_get_post_hook_num();

	p_pre_hook = oplus_get_pre_hook();
	p_post_hook = oplus_get_post_hook();

	for (cnt = post_hook_num - 1; cnt >= 0; cnt--) {
		/*unregister post hook*/
		if (p_post_hook[cnt].probe_reg_flag == 1) {
			tracepoint_probe_unregister(p_tp_sys_exit, p_post_hook[cnt].probe, NULL);
			p_post_hook[cnt].probe_reg_flag = 0;
		}
	}

	for (cnt = pre_hook_num - 1; cnt >= 0; cnt--) {
		/*unregister post hook*/
		if (p_pre_hook[cnt].probe_reg_flag == 1) {
			tracepoint_probe_unregister(p_tp_sys_enter, p_pre_hook[cnt].probe, NULL);
			p_pre_hook[cnt].probe_reg_flag = 0;
		}
	}
}
