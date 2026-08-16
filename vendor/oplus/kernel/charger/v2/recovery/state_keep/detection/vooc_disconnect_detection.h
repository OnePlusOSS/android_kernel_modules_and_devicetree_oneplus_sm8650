// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#ifndef __RECOVERY_VOOC_DISCONNECT_DETECT__

#include <linux/device.h>

int vooc_disconnect_detection_init(struct device_node *node);
void vooc_disconnect_monitor_exit(void);

#endif /* __RECOVERY_VOOC_DISCONNECT_DETECT__ */
