/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 */
#ifndef _OPLUS_ROOT_HOOK_H
#define _OPLUS_ROOT_HOOK_H

void oplus_root_check_pre_handler(void *data, struct pt_regs *regs, long id);
void oplus_root_check_post_handler(void *data, struct pt_regs *regs, long ret);

#endif /* _OPLUS_ROOT_HOOK_H */
