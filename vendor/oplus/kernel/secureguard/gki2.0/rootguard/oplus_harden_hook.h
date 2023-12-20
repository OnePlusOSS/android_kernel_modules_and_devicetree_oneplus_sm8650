/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 */
#ifndef _OPLUS_HARDEN_HOOK_H
#define _OPLUS_HARDEN_HOOK_H

void oplus_harden_pre_handler(void *data, struct pt_regs *regs, long id);

#endif /* _OPLUS_HARDEN_HOOK_H */
