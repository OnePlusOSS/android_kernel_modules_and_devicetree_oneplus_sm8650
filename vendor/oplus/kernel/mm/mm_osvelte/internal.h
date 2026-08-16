/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2021 Oplus. All rights reserved.
 */
#ifndef _OSVELTE_INTERNAL_H
#define _OSVELTE_INTERNAL_H

#include <asm/ioctls.h>
#include "common.h"

#define KMODULE_NAME "oplus_bsp_mm_osvelte"

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_OSVELTE_DBG)
#define DEV_NAME "osvelte_dbg"
#else
#define DEV_NAME "osvelte"
#endif

#define DEV_PATH "/dev/" DEV_NAME

#define OSVELTE_LOG_TAG DEV_NAME

/* experimental feature */
#define OSVELTE_FEATURE_USE_HASHLIST 1

#define OSVELTE_MAJOR		(0)
#define OSVELTE_MINOR		(2)
#define OSVELTE_PATCH_NUM	(3)
#define OSVELTE_VERSION (OSVELTE_MAJOR << 16 | OSVELTE_MINOR)

#define CMD_COMMON_MIN		CMD_OSVELTE_SET_SCENE
#define CMD_COMMON_MAX		CMD_OSVELTE_CLEAR_SCENE
#define CMD_COMMON_INVALID	0xFFFFFFFE

#define OSVELTE_STATIC_ASSERT(c)				\
{								\
	enum { OSVELTE_static_assert = 1 / (int)(!!(c)) };	\
}

#define osvelte_info(fmt, ...)      \
	pr_info(OSVELTE_LOG_TAG ": " fmt, ##__VA_ARGS__)

#define osvelte_err(fmt, ...)      \
	pr_err(OSVELTE_LOG_TAG ": " fmt, ##__VA_ARGS__)

#define MM_LOG_LVL 1
enum {
	MM_LOG_VERBOSE = 0,
	MM_LOG_INFO,
	MM_LOG_DEBUG,
	MM_LOG_ERR,
};

static inline char mm_loglvl_to_char(int l)
{
	switch (l) {
	case MM_LOG_VERBOSE:
		return 'V';
	case MM_LOG_INFO:
		return 'I';
	case MM_LOG_DEBUG:
		return 'D';
	case MM_LOG_ERR:
		return 'E';
	}
	return '?';
}

#define osvelte_log(l, f, ...) do {					\
	if (l >= MM_LOG_LVL) 						\
		printk(KERN_ERR "%s %5d %5d %c %-16s: %s:%d "f,		\
		       OSVELTE_LOG_TAG, current->tgid, current->pid,	\
		       mm_loglvl_to_char(l), current->comm, __func__,	\
		       __LINE__,  ##__VA_ARGS__);			\
} while (0)

#define osvelte_loge(f, ...)						\
	osvelte_log(MM_LOG_ERR, f, ##__VA_ARGS__)

#define osvelte_logi(f, ...)						\
	osvelte_log(MM_LOG_INFO, f, ##__VA_ARGS__)

#define osvelte_logd(f, ...)						\
	osvelte_log(MM_LOG_DEBUG, f, ##__VA_ARGS__)
#endif /* _OSVELTE_INTERNAL_H */
