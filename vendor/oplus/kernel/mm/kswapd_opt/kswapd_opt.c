// SPDX-License-Identifier: GPL-2.0-only
/*
 * kswapd_opt, contain some optimisation to reduce kswapd running overhead for some high-order allocation
 *
 * Copyright (C) 2023-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "kswapd_opt: " fmt

#include <linux/types.h>
#include <linux/mm.h>
#ifdef CONFIG_ALLOC_ADJUST_FLAGS
#include <trace/hooks/iommu.h>
#include <trace/hooks/mm.h>
#endif

#ifdef CONFIG_ALLOC_ADJUST_FLAGS
static void alloc_adjust_flags(void *data, unsigned int order, gfp_t *flags)
{
	if (order > PAGE_ALLOC_COSTLY_ORDER)
		*flags &= ~__GFP_RECLAIM;
}

static void kvmalloc_adjust_flags(void *data, unsigned int order, gfp_t *flags)
{
	if (order > PAGE_ALLOC_COSTLY_ORDER)
		*flags &= ~__GFP_RECLAIM;
}

static int register_alloc_adjust_flags(void)
{
	return register_trace_android_vh_adjust_alloc_flags(alloc_adjust_flags, NULL);
}

static void unregister_alloc_adjust_flags(void)
{
	unregister_trace_android_vh_adjust_alloc_flags(alloc_adjust_flags, NULL);
}

static int register_kvmalloc_adjust_flags(void)
{
	return register_trace_android_vh_adjust_kvmalloc_flags(kvmalloc_adjust_flags, NULL);
}

static void unregister_kvmalloc_adjust_flags(void)
{
	unregister_trace_android_vh_adjust_kvmalloc_flags(kvmalloc_adjust_flags, NULL);
}
#else
static int register_alloc_adjust_flags(void)
{
	return 0;
}

static void unregister_alloc_adjust_flags(void)
{
}

static int register_kvmalloc_adjust_flags(void)
{
	return 0;
}

static void unregister_kvmalloc_adjust_flags(void)
{
}
#endif

void __init kswapd_opt_init(void)
{
	int ret = 0;

	ret = register_alloc_adjust_flags();
	if (ret)
		pr_warn("alloc_adjust_flags vendor_hook register failed: %d\n", ret);

	ret = register_kvmalloc_adjust_flags();
	if (ret)
		pr_warn("kvmalloc_adjust_flags vendor_hook register failed: %d\n", ret);
}

void __exit kswapd_opt_exit(void)
{
	unregister_alloc_adjust_flags();
	unregister_kvmalloc_adjust_flags();
}
