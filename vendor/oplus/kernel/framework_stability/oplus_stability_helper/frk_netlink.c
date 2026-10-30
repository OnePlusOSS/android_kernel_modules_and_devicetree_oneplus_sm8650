#include "frk_netlink.h"


#define MAX_DATA_SIZE 128
static __u32 client_pid;

static struct genl_family oplus_frk_genl_family;

int send_to_frk(int event_type, size_t len, const int *data)
{
	int i;
	int ret = 0;
	void *head;
	size_t size;
	struct sk_buff *skb = NULL;
	int msg[MAX_DATA_SIZE];
	int total_size = len + MSG_START_IDX;

	if (client_pid <= 0) {
		return ret;
	}

	if (total_size > MAX_DATA_SIZE) {
		printk("frk_netlink: send oversize(%d,MAX:%d) data!\n",
			total_size, MAX_DATA_SIZE);
		return -EINVAL;
	}

	msg[0] = event_type;
	msg[1] = len;

	/* fill the data */
	for (i = 0; i + MSG_START_IDX < total_size; i++) {
		msg[i + MSG_START_IDX] = data[i];
	}

	len = sizeof(int) * total_size;
	size = nla_total_size(len);

	/* create a new netlink msg */
	skb = genlmsg_new(size, GFP_ATOMIC);
	if (IS_ERR_OR_NULL(skb)) {
		pr_err("frk_netlink: new genlmsg alloc failed\n");
		return -ENOMEM;
	}

	/* Add a new netlink message to an skb */
	genlmsg_put(skb, client_pid, 0, &oplus_frk_genl_family,
				0, OPLUS_FRK_CMD_SEND);

	/* add a netlink attribute to a socket buffer */
	if (nla_put(skb, OPLUS_FRK_ATTR_MSG_GENL, len, &msg)) {
		pr_err("frk_netlink: genl_msg_mk_usr_msg failed!\n");
		kfree_skb(skb);
		return -EMSGSIZE;
	}

	head = genlmsg_data(nlmsg_data(nlmsg_hdr(skb)));
	genlmsg_end(skb, head);

	/* send to user */
	ret = genlmsg_unicast(&init_net, skb, client_pid);
	if (ret < 0) {
		pr_err("frk_netlink: genlmsg_unicast failed! err = %d\n", ret);
		return ret;
	}

	return 0;
}

static int recv_from_frk(struct sk_buff *skb, struct genl_info *info)
{
	if (!skb || !info)
		return -EINVAL;
	if (!netlink_capable(skb, CAP_NET_ADMIN))
		return -EPERM;
	if (!info->snd_portid)
		return -EINVAL;

	client_pid = info->snd_portid;
	pr_info("frk_netlink: recv_from_frk: %u\n", info->snd_portid);
	return 0;
}

static const struct genl_ops oplus_frk_genl_ops[] = {
	{
		.cmd = OPLUS_FRK_CMD_RECV,
		.flags = GENL_ADMIN_PERM,
		.doit = recv_from_frk,
		.dumpit = NULL,
		.maxattr = 0,
	},
};

static struct genl_family oplus_frk_genl_family = {
	.id = OPLUS_FRK_GENL_ID_GENERATE,
	.hdrsize = 0,
	.name = OPLUS_FRK_FAMILY_NAME,
	.version = OPLUS_FRK_FAMILY_VER,
	.maxattr = OPLUS_FRK_ATTR_MSG_MAX,
	.ops = oplus_frk_genl_ops,
	.n_ops = ARRAY_SIZE(oplus_frk_genl_ops),
	.module = THIS_MODULE,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	.resv_start_op = __OPLUS_FRK_CMD_MAX,
#endif
};

void create_frk_netlink(int unused)
{
	if (genl_register_family(&oplus_frk_genl_family) != 0)
		pr_err("cpu_netlink: genl_register_family error!\n");
}

void destroy_frk_netlink(void)
{
	genl_unregister_family(&oplus_frk_genl_family);
}
