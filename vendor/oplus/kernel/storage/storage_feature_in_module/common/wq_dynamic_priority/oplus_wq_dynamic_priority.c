#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <linux/string.h>
#include <linux/sched.h>

#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#include "oplus_wq_dynamic_priority.h"

#define WQ_UX    (1 << 14)

#define VIRTUAL_KWORKER_NICE (-1000)

/* ---------alloc_workqueue--------- */
struct config_wq_flags {
    char *target_str;
    unsigned int new_flags;
};

static struct config_wq_flags oplus_wq_config[] = {
    { "loop", WQ_UNBOUND | WQ_FREEZABLE | WQ_HIGHPRI | WQ_UX },
    { "kverityd", WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UX | WQ_UNBOUND },
    // Add more strings and flags as needed.
    { NULL, 0 } // Terminate array with NULL
};

static int handler_alloc_workqueue_pre(struct kprobe *p, struct pt_regs *regs)
{
    const char *fmt = (const char *)regs->regs[0];
    unsigned int flags = (unsigned int)regs->regs[1];

    struct config_wq_flags *item = oplus_wq_config;
    if(fmt) {
        while (item->target_str) {
            if (!strncmp(fmt, item->target_str, strlen(item->target_str)) && (item->new_flags != flags)) {
                printk(KERN_INFO "alloc_workqueue: matching fmt '%s', modifying flags from 0x%x to 0x%x\n", fmt, flags, item->new_flags);
                regs->regs[1] = item->new_flags;
                break;
            }
            item++;
        }
    }
    return 0;
}

static struct kprobe oplus_alloc_workqueue_kp = {
    .symbol_name = "alloc_workqueue",
    .pre_handler = handler_alloc_workqueue_pre,
};

/* ---------alloc_unbound_pwq--------- */
static int handler_alloc_unbound_pwq_pre(struct kprobe *p, struct pt_regs *regs)
{
    struct workqueue_struct *wq = (struct workqueue_struct *)regs->regs[0];
    struct workqueue_attrs *attrs = (struct workqueue_attrs *)regs->regs[1];

    int old_nice=0;

    if (wq && (wq->flags & WQ_UX)) {
        if (attrs) {
            old_nice = attrs->nice;
            if (old_nice != VIRTUAL_KWORKER_NICE) {
                attrs->nice = VIRTUAL_KWORKER_NICE;
                printk(KERN_INFO "alloc_unbound_pwq: modifying nice from %d to %d\n", old_nice, attrs->nice);
            }
        }
    }
    return 0;
}

static struct kprobe oplus_alloc_unbound_pwq_kp = {
    .symbol_name = "alloc_unbound_pwq",
    .pre_handler = handler_alloc_unbound_pwq_pre,
};

/* ---------apply_wqattrs_prepare--------- */
static int handler_apply_wqattrs_ret(struct kretprobe_instance *ri, struct pt_regs *regs)
{
    struct apply_wqattrs_ctx *ctx = (struct apply_wqattrs_ctx *)regs_return_value(regs);
    if (ctx && (ctx->wq) && (ctx->wq->flags & WQ_UX)) {
        printk(KERN_INFO "apply_wqattrs_prepare: modifying nice from %d to %d\n", ctx->attrs->nice, VIRTUAL_KWORKER_NICE);
        ctx->attrs->nice = VIRTUAL_KWORKER_NICE;
    }
    return 0;
}

static struct kretprobe oplus_apply_wqattrs_krp = {
    .kp = {
        .symbol_name = "apply_wqattrs_prepare",
    },
    .handler = handler_apply_wqattrs_ret,
};

/* ---------worker_attach_to_pool--------- */
static int handler_worker_attach_to_pool_pre(struct kprobe *p, struct pt_regs *regs)
{
    struct worker *worker = (struct worker *)regs->regs[0];
    struct worker_pool *pool = (struct worker_pool *)regs->regs[1];

    if ((worker && worker->task) && (pool && pool->attrs)) {
        if (pool->attrs->nice == VIRTUAL_KWORKER_NICE) {
            oplus_set_ux_state_lock(worker->task, SA_TYPE_LIGHT, -1, true);
            printk(KERN_INFO "worker_attach_to_pool:comm:%s set UX and set nice to %d\n", worker->task->comm, MIN_NICE);
            set_user_nice(worker->task, MIN_NICE);
        }
    }
    return 0;
}

static struct kprobe oplus_worker_attach_to_pool_kp = {
    .symbol_name = "worker_attach_to_pool",
    .pre_handler = handler_worker_attach_to_pool_pre,
};


/* ---------kblockd_schedule_work--------- */
static struct workqueue_struct *oplus_kblockd_workqueue;
static int handler_kblockd_schedule_work_pre(struct kprobe *p, struct pt_regs *regs) {
    //printk(KERN_INFO "kblockd_schedule_work:use oplus kblockd workqueue\n");
    regs->regs[1] = (u64)oplus_kblockd_workqueue;
    return 0;
}

static int handler_kblockd_mod_delayed_work_on_pre(struct kprobe *p, struct pt_regs *regs) {
    //printk(KERN_INFO "kblockd_mod_delayed_work_on:use oplus kblockd workqueue\n");
    regs->regs[1] = (u64)oplus_kblockd_workqueue;
    return 0;
}

static struct kprobe oplus_kblockd_schedule_work_kp = {
    .symbol_name = "kblockd_schedule_work",
    .offset = 0x1c,
    .pre_handler = handler_kblockd_schedule_work_pre,
};

static struct kprobe oplus_kblockd_mod_delayed_work_on_kp = {
    .symbol_name = "kblockd_mod_delayed_work_on",
    .offset = 0x1c,
    .pre_handler = handler_kblockd_mod_delayed_work_on_pre,
};

static bool kprobe_init_successful=false;
static int __init oplus_wq_kprobe_init(void)
{
    int ret;

    ret = register_kprobe(&oplus_alloc_workqueue_kp);
    if (ret < 0) {
        printk(KERN_ERR "register_kprobe alloc_workqueue failed, returned %d\n", ret);
        goto kp_alloc_workqueue_fail;
    }

    ret = register_kprobe(&oplus_alloc_unbound_pwq_kp);
    if (ret < 0) {
        printk(KERN_ERR "register_kprobe alloc_unbound_pwq failed, returned %d\n", ret);
        goto kp_alloc_unbound_pwq_fail;
    }

    ret = register_kretprobe(&oplus_apply_wqattrs_krp);
    if (ret < 0) {
        printk(KERN_ERR "register_kretprobe apply_wqattrs failed, returned %d\n", ret);
        goto kp_apply_wqattrs_fail;
    }

    ret = register_kprobe(&oplus_worker_attach_to_pool_kp);
    if (ret < 0) {
        printk(KERN_ERR "register_kprobe worker_attach_to_pool failed, returned %d\n", ret);
        goto kp_worker_attach_to_pool_fail;
    }

    oplus_kblockd_workqueue = alloc_workqueue("opluskblockd",WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UX | WQ_UNBOUND, 0);

    if (!oplus_kblockd_workqueue) {
        printk(KERN_ERR "alloc  oplus_kblockd_workqueue fail!\n");
        goto kp_kblockd_schedule_work_kp_fail;
    }

    ret = register_kprobe(&oplus_kblockd_schedule_work_kp);
    if (ret < 0) {
        printk(KERN_ERR "register_kprobe oplus_kblockd_schedule_work_kp failed, returned %d\n", ret);
        goto kp_kblockd_schedule_work_kp_fail;
    }

    ret = register_kprobe(&oplus_kblockd_mod_delayed_work_on_kp);
    if (ret < 0) {
        printk(KERN_ERR "register_kprobe oplus_kblockd_mod_delayed_work_on_kp failed, returned %d\n", ret);
        goto kp_kblockd_mod_delayed_work_on_kp_fail;
    }

    printk(KERN_ERR "%s successful!\n", __func__);
    kprobe_init_successful = true;
    return 0;

kp_kblockd_mod_delayed_work_on_kp_fail:
    unregister_kprobe(&oplus_kblockd_schedule_work_kp);
kp_kblockd_schedule_work_kp_fail:
    unregister_kprobe(&oplus_worker_attach_to_pool_kp);
kp_worker_attach_to_pool_fail:
    unregister_kretprobe(&oplus_apply_wqattrs_krp);
kp_apply_wqattrs_fail:
    unregister_kprobe(&oplus_alloc_unbound_pwq_kp);
kp_alloc_unbound_pwq_fail:
    unregister_kprobe(&oplus_alloc_workqueue_kp);
kp_alloc_workqueue_fail:

    kprobe_init_successful = false;
    return ret;
}

static void __exit oplus_wq_kprobe_exit(void)
{
	//工作队列未销毁
    if(kprobe_init_successful) {
        unregister_kprobe(&oplus_alloc_workqueue_kp);
        unregister_kprobe(&oplus_alloc_unbound_pwq_kp);
        unregister_kretprobe(&oplus_apply_wqattrs_krp);
        unregister_kprobe(&oplus_worker_attach_to_pool_kp);
        unregister_kprobe(&oplus_kblockd_schedule_work_kp);
        unregister_kprobe(&oplus_kblockd_mod_delayed_work_on_kp);
        printk(KERN_INFO "kprobe unregistered\n");
    } else {
        printk(KERN_INFO "kprobe needn't unregistered\n");
    }
}

module_init(oplus_wq_kprobe_init);
module_exit(oplus_wq_kprobe_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("lijiang");
MODULE_DESCRIPTION("A kernel module using kprobe to hook alloc_workqueue function");
