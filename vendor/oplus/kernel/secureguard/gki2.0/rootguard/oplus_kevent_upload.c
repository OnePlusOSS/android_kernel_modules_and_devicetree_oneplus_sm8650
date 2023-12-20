// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Oplus. All rights reserved.
 *
 * kevent upload: report suspicious events to user layer daemon
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/netlink.h>
#include <net/net_namespace.h>
#include <linux/proc_fs.h>
#include <net/sock.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <net/genetlink.h>
#include "oplus_kevent.h"

enum {
	SECURE_GUARD_CMD_ATTR_UNSPEC = 0,
	SECURE_GUARD_CMD_ATTR_MSG,
	SECURE_GUARD_CMD_ATTR_OPT,
	__SECURE_GUARD_CMD_ATTR_MAX,
};
#define SECURE_GUARD_CMD_ATTR_MAX	(__SECURE_GUARD_CMD_ATTR_MAX - 1)

enum {
	SECURE_GUARD_CMD_GENL_UNSPEC = 0,
	SECURE_GUARD_CMD_GENL_SENDPID,
	SECURE_GUARD_CMD_GENL_UPLOAD,
};

/* secureguard sub-module string for print */
#define SG_KEVENT    "[secureguard][kevent]"

#define OPLUS_SECURE_GUARD_DEGBU
#ifdef OPLUS_SECURE_GUARD_DEGBU
#define  PRINT_FORMAT(t, l)	print_format(t, l)
#else
#define  PRINT_FORMAT(t, l)
#endif

#define OPLUS_KEVENT_MAX_UP_PALOAD_LEN		2048
#define OPLUS_SECURE_GUARD_PROTOCAL_NAME	"secure_guard"
#define OPLUS_SECURE_GUARD_GENL_VERSION		0x01

static void oplus_kevent_send_to_user(void *data, struct kernel_packet_info *userinfo, int *p_retval);
static int oplus_security_keventupload_flag;
static unsigned int kevent_pid;

static struct nla_policy security_kevent_genl_policy[SECURE_GUARD_CMD_ATTR_MAX + 1] = {
	[SECURE_GUARD_CMD_ATTR_MSG] = { .type = NLA_NUL_STRING },
	[SECURE_GUARD_CMD_ATTR_OPT] = { .type = NLA_U32 },
};

static void print_format(unsigned char *temp, unsigned int len)
{
	unsigned int i = 0;

	printk(" ");
	for (i = 0; i < len; i++) {
		if (temp[i] < 16)
			printk("0%x", temp[i]);
		else
			printk("%x", temp[i]);

		if (i%32 == 31)
			printk("\n");
		if ((i%16 == 7) || (i%16 == 15))
			printk(" ");
	}
	if (len%16 != 15)
		printk("\n");
	printk("\n\n");
}

static int security_keventupload_sendpid_cmd(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na = NULL;
	unsigned int *p_data = NULL;

	pr_info(SG_KEVENT "%sm kernel recv cmd.\n", __func__);
	if (info->attrs[SECURE_GUARD_CMD_ATTR_MSG]) {
		na = info->attrs[SECURE_GUARD_CMD_ATTR_MSG];
		PRINT_FORMAT(nla_data(na),  nla_len(na));
		pr_info(SG_KEVENT "%s, nla_len(na) is %d.\n", __func__, nla_len(na));
		p_data = nla_data(na);
		kevent_pid = *p_data;
		pr_info(SG_KEVENT "%s, kevent_pid is 0x%x.\n", __func__, kevent_pid);
	}

	return 0;
}

static const struct genl_ops oplus_security_ops[] = {
	{
		.cmd		= SECURE_GUARD_CMD_GENL_SENDPID,
		.doit		= security_keventupload_sendpid_cmd,
		.policy		= security_kevent_genl_policy,
	},
};

static struct genl_family oplus_security_family __ro_after_init = {
	.id		= 0,
	.hdrsize	= 0,
	.name		= OPLUS_SECURE_GUARD_PROTOCAL_NAME,
	.version	= OPLUS_SECURE_GUARD_GENL_VERSION,
	.maxattr	= SECURE_GUARD_CMD_ATTR_MAX,
	.module		= THIS_MODULE,
	.policy		= security_kevent_genl_policy,
	.ops		= oplus_security_ops,
	.n_ops		= ARRAY_SIZE(oplus_security_ops),
};

static inline int genl_msg_prepare_usr_msg(unsigned char cmd, size_t size, pid_t pid, struct sk_buff **skbp)
{
	struct sk_buff *skb;

	/* create a new netlink msg */
	skb = genlmsg_new(size, GFP_KERNEL|GFP_ATOMIC);
	if (skb == NULL)
		return -ENOMEM;

	/* Add a new netlink message to an skb */
	genlmsg_put(skb, pid, 0, &oplus_security_family, 0, cmd);

	*skbp = skb;
	return 0;
}

static inline int genl_msg_mk_usr_msg(struct sk_buff *skb, int type, void *data, int len)
{
	int ret;

	/* add a netlink attribute to a socket buffer */
	ret = nla_put(skb, type, len, data);

	return ret;
}

static void oplus_kevent_send_to_user(void *data, struct kernel_packet_info *userinfo, int *p_retval)
{
#ifdef CONFIG_DEBUG_ATOMIC_SLEEP
	(void)data;
	(void)userinfo;
	(void)p_retval;
	pr_info("userdebug mode & CONFIG_DEBUG_ATOMIC_SLEEP == y, no report.\n");
	return;
#else
	int ret = 0;

	struct sk_buff *skbuff = NULL;
	void *head = NULL;
	size_t data_len = 0;
	size_t attr_len = 0;
	*p_retval = 0;

	/*max_len */
	pr_info(SG_KEVENT "%s, start.\n", __func__);
	if (userinfo->payload_length >= OPLUS_KEVENT_MAX_UP_PALOAD_LEN) {
		pr_err(SG_KEVENT "%s, payload_length out of range\n", __func__);
		*p_retval = -1;
		return;
	}

	data_len = userinfo->payload_length + sizeof(struct kernel_packet_info);
	attr_len = nla_total_size(data_len);
	/*pr_info("[OPLUS_SECURITY DEBUG]:data_len is %u, attr_len is %u\n", data_len, attr_len);*/

	ret = genl_msg_prepare_usr_msg(SECURE_GUARD_CMD_GENL_UPLOAD, attr_len, kevent_pid, &skbuff);
	if (ret) {
		pr_err(SG_KEVENT "%s, genl_msg_prepare_usr_msg err, ret %d.\n", __func__, ret);
		*p_retval = -1;
		return;
	}

	ret = genl_msg_mk_usr_msg(skbuff, SECURE_GUARD_CMD_ATTR_MSG, userinfo, data_len);
	if (ret) {
		kfree_skb(skbuff);
		*p_retval = ret;
		return;
	}

	head = genlmsg_data(nlmsg_data(nlmsg_hdr(skbuff)));
	genlmsg_end(skbuff, head);

	ret = genlmsg_unicast(&init_net, skbuff, kevent_pid);
	if (ret < 0) {
		*p_retval = ret;
		return;
	}

	*p_retval = 0;
#endif
}

int kevent_send_to_user(struct kernel_packet_info *userinfo)
{
	int ret = 0;

	oplus_kevent_send_to_user(NULL, userinfo, &ret);

	return ret;
}

/*
 * report_security_event:
 * This is a kernel api for burying points, which transfers the key information of security events to Native for analysis and burying points.
 * @event_name: Event type of security check. (e.g..root_check/capa/heapspary...)
 * @event_type: Security event fuctions type. (e.g. MCAST_JOIN_GROUP/IP_M.. of Heapsparty...)
 * @more: (Additional information)
 * @Return: Void type, non-return.
 */
void report_security_event(const char *event_name, unsigned int event_type, const char *more)
{
	struct kernel_packet_info *dcs_event;
	char dcs_stack[sizeof(struct kernel_packet_info) + 256];
	const char *dcs_event_tag = "kernel_event";
	const char *dcs_event_id = event_name;
	char *dcs_event_payload = NULL;

	dcs_event = (struct kernel_packet_info *)dcs_stack;
	dcs_event_payload = dcs_stack + sizeof(struct kernel_packet_info);
	dcs_event->type = event_type;/*set type of security event*/
	strncpy(dcs_event->log_tag, dcs_event_tag,
	sizeof(dcs_event->log_tag));
	strncpy(dcs_event->event_id, dcs_event_id,
	sizeof(dcs_event->event_id));
	/*accrding type, chosse array*/
	dcs_event->payload_length = snprintf(dcs_event_payload, 256, "$$uid@@%d$$EVENT_TYPE@@%d$$current_name@@%s$$additional@@%s\n",
		current_uid().val, event_type, current->comm, more);
	if (dcs_event->payload_length < 256)
		dcs_event->payload_length += 1;

	kevent_send_to_user(dcs_event);
}

int oplus_keventupload_init(void)
{
	int ret = 0;

	/*register gen_link family*/
	ret = genl_register_family(&oplus_security_family);
	if (ret) {
		pr_err(SG_KEVENT "%s, genl_register_family failed, ret %d.\n", __func__, ret);
		return ret;
	}

	oplus_security_keventupload_flag = 1;
	pr_info(SG_KEVENT "%s,registered gen_link family %s OK.\n", __func__, OPLUS_SECURE_GUARD_PROTOCAL_NAME);
	return 0;
}

void oplus_keventupload_exit(void)
{
	if (oplus_security_keventupload_flag)
		genl_unregister_family(&oplus_security_family);
}
