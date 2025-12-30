/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#ifndef __OPLUS_DEBUG_AUTH_H__
#define __OPLUS_DEBUG_AUTH_H__

#include <linux/kernel.h>

#if IS_ENABLED(CONFIG_OPLUS_DEBUG_AUTH)
ssize_t oplus_debug_auth_get_data(const char *buf, size_t size, const char **ret_buf);
bool oplus_debug_auth_is_cert_valid(void);
#else
static inline ssize_t oplus_debug_auth_get_data(const char *buf, size_t size, const char **ret_buf)
{
	*ret_buf = buf;
	return size;
}

static inline bool oplus_debug_auth_is_cert_valid(void)
{
	return true;
}
#endif /* CONFIG_OPLUS_DEBUG_AUTH */

#endif /* __OPLUS_DEBUG_AUTH_H__ */
