/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 */
#ifndef _OPLUS_SECURE_HARDEN_H
#define _OPLUS_SECURE_HARDEN_H

#define MCAST_MSFILTER		48
#define MCAST_JOIN_GROUP	42
#define IP_MSFILTER		41

#define HEAPSPARY_TIME_LIMIT	10
#define HEAPSPART_COUNT_LIMIT	200

int oplus_harden_init(void);
void oplus_harden_exit(void);

#endif /* _OPLUS_SECURE_HARDEN_H */
