/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 */
#ifndef _OPLUS_KEVENT_H
#define _OPLUS_KEVENT_H

#include <linux/types.h>

struct kernel_packet_info {
	int type;			/* 0:root,1:only string,other number represent other type */
	char log_tag[32];		/* logTag */
	char event_id[20];		/* eventID */
	size_t payload_length;		/* Length of packet data */
	unsigned char payload[0];	/* Optional packet data */
} __packed;

int kevent_send_to_user(struct kernel_packet_info *userinfo);
void report_security_event(const char *event_name, unsigned int event_type, const char *more);
int oplus_keventupload_init(void);
void oplus_keventupload_exit(void);

#endif /* _OPLUS_KEVENT_H */
