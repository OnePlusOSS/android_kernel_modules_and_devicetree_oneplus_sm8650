/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 */
#ifndef _OPLUS_EXEC_HOOK_H
#define _OPLUS_EXEC_HOOK_H

void oplus_exe_block_ret_handler(void *data, struct pt_regs *regs, long ret);

#endif /* _OPLUS_EXEC_HOOK_H */
