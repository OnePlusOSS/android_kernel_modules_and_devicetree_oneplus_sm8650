/* SPDX-License-Identifier: GPL-2.0-only */
/*  
* Copyright (C) 2018-2025 Oplus. All rights reserved. 
*/ 

#ifndef _OPLUS_PATCH_H_
#define _OPLUS_PATCH_H_

#include <linux/kprobes.h>

#define TASK_COMM_LEN 16
#define KPROBE_TYPE_KPROBE     0
#define KPROBE_TYPE_KRETPROBE  1

struct log_entry {
        char comm[TASK_COMM_LEN];
        pid_t pid;
        char func[64];
        s64 duration_ns;  /*record duration*/
};

struct hook_entry {
        struct list_head list;
        int type;           /*KPROBE_TYPE_KPROBE or KPROBE_TYPE_KRETPROBE*/
        int enabled;
        const char *name;

        /*probe struct*/
        struct kprobe kp;
        struct kretprobe rp;

        void *pre_handler;      /*fuction handler*/
        void *entry_handler;
};

#endif /*_OPLUS_PATCH_H_*/
