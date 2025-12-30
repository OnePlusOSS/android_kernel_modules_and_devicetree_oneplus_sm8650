// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021-2021 Oplus. All rights reserved.
 */

/* OPLUS Micro Message Service */

#define pr_fmt(fmt) "[MMS]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <oplus_chg_module.h>
#include <oplus_mms.h>
#include <debug/oplus_debug_auth.h>

struct class *oplus_mms_class;
struct oplus_mms_call_head {
	const char *name;
	struct list_head list;
	mms_callback_t call;
	void *data;
};

static struct device_type oplus_mms_dev_type;
static struct workqueue_struct	*mms_wq;
static DEFINE_MUTEX(wait_list_lock);
static LIST_HEAD(oplus_mms_wait_list);
static void oplus_mms_call(struct oplus_mms *topic);
static struct mms_item *oplus_mms_get_item(struct oplus_mms *mms, u32 id)
{
	struct mms_item *item_table = mms->desc->item_table;
	int item_num = mms->desc->item_num;
	int i;

	for (i = 0; i < item_num; i++) {
		if (item_table[i].desc.item_id == id)
			return &item_table[i];
	}

	return NULL;
}

bool oplus_mms_item_is_str(struct oplus_mms *mms, u32 id)
{
	struct mms_item *item;

	if (mms == NULL) {
		chg_err("mms is NULL");
		return false;
	}
	item = oplus_mms_get_item(mms, id);
	if (item == NULL) {
		chg_err("%s item(=%d) not found\n", mms->desc->name, id);
		return false;
	}

	return item->desc.str_data;
}

int oplus_mms_set_update_mode(struct oplus_mms *mms, bool update)
{
	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}

	mms->force_update = update;
	if (mms->desc->set_update_mode)
		mms->desc->set_update_mode(mms, update);

	return 0;
}

int oplus_mms_get_update_mode(struct oplus_mms *mms)
{
	int val = 0;
	if (mms == NULL) {
		chg_err("mms is NULL");
		return 0;
	}

	if (mms->force_update)
		val = 1;

	return val;
}

int oplus_mms_set_item_enable(struct oplus_mms *mms, u32 item_id)
{
	struct mms_item *item;

	if (mms == NULL || mms->desc == NULL ||
	    !mms->desc->item_table) {
		chg_err("mms or item_table is NULL");
		return -ENODEV;
	}
	item = oplus_mms_get_item(mms, item_id);
	if (item == NULL) {
		chg_err("topic(=%s) item(=%d) not found\n", mms->desc->name,
			item_id);
		return -EINVAL;
	}
	item->disabled = false;

	return 0;
}

int oplus_mms_set_item_disable(struct oplus_mms *mms, u32 item_id)
{
	struct mms_item *item;

	if (mms == NULL || mms->desc == NULL ||
	    !mms->desc->item_table) {
		chg_err("mms or item_table is NULL");
		return -ENODEV;
	}
	item = oplus_mms_get_item(mms, item_id);
	if (item == NULL) {
		chg_err("topic(=%s) item(=%d) not found\n", mms->desc->name,
			item_id);
		return -EINVAL;
	}
	item->disabled = true;

	return 0;
}

int oplus_mms_get_item_data(struct oplus_mms *mms, u32 item_id,
			    union mms_msg_data *data, bool update)
{
	struct mms_item *item;

	if (mms == NULL || mms->desc == NULL ||
	    !mms->desc->item_table) {
		chg_err("mms or item_table is NULL");
		return -ENODEV;
	}

	if (data == NULL) {
		chg_err("data is NULL");
		return -EINVAL;
	}
	item = oplus_mms_get_item(mms, item_id);
	if (item == NULL) {
		chg_err("topic(=%s) item(=%d) not found\n", mms->desc->name,
			item_id);
		return -EINVAL;
	}

	if (item->disabled) {
		chg_info("topic(=%s) item(=%d) is disabled\n",
			 mms->desc->name, item_id);
		return -ENOTSUPP;
	}

#if IS_ENABLED(CONFIG_OPLUS_CHG_MMS_DEBUG) && IS_ENABLED(CONFIG_OPLUS_DEBUG_AUTH)
	if (item->overwritten && oplus_debug_auth_is_cert_valid()) {
		memcpy(data, &item->overwrite_data, sizeof(union mms_msg_data));
		return 0;
	}
#endif

	if (update || mms->force_update)
		oplus_mms_item_update(mms, item_id, false);

	read_lock(&item->lock);
	memcpy(data, &item->data, sizeof(union mms_msg_data));
	read_unlock(&item->lock);

	return 0;
}

bool oplus_mms_item_update(struct oplus_mms *mms, u32 item_id, bool check_update)
{
	struct mms_item *item;
	union mms_msg_data data = { 0 };
	bool update;

	if (mms == NULL || mms->desc == NULL ||
	    !mms->desc->item_table) {
		chg_err("mms or item_table is NULL");
		return false;
	}
	item = oplus_mms_get_item(mms, item_id);
	if (item == NULL) {
		chg_err("topic(=%s) item(=%d) not found\n", mms->desc->name,
			item_id);
		return false;
	}
	if (item->desc.update == NULL) {
		return false;
	}
	mutex_lock(&item->update_lock);
	item->desc.update(mms, &data);
	mutex_unlock(&item->update_lock);
	write_lock(&item->lock);
	memcpy(&item->data, &data, sizeof(union mms_msg_data));
	if (!check_update) {
		item->updated = false;
		goto out;
	} else {
		item->updated = true;
	}

	if (item->desc.str_data) {
		/* Dynamic string data cannot be judged whether to update */
		item->pre_data.strval = item->data.strval;
		item->updated = true;
		goto out;
	} else {
		if (item->desc.up_thr_enable) {
			if ((item->data.intval > item->pre_data.intval) &&
			    (item->pre_data.intval < item->desc.update_thr_up) &&
			    (item->data.intval >= item->desc.update_thr_up)) {
				item->pre_data.intval = item->data.intval;
				item->updated = true;
				goto out;
			} else {
				item->updated = false;
			}
		}
		if (item->desc.update_thr_down) {
			if ((item->data.intval < item->pre_data.intval) &&
			    (item->pre_data.intval > item->desc.update_thr_down) &&
			    (item->data.intval < item->desc.update_thr_down)) {
				item->pre_data.intval = item->data.intval;
				item->updated = true;
				goto out;
			} else {
				item->updated = false;
			}
		}
		if (item->desc.dead_thr_enable) {
			if (abs(item->data.intval - item->pre_data.intval) >=
			    item->desc.dead_zone_thr) {
				item->pre_data.intval = item->data.intval;
				item->updated = true;
				goto out;
			} else {
				item->updated = false;
			}
		}
	}

out:
	update = item->updated;
	write_unlock(&item->lock);

	return update;
}

static int oplus_mms_item_update_by_msg(struct oplus_mms *mms, u32 item_id,
					struct mms_msg *msg)
{
	struct mms_item *item;

	if (mms == NULL || mms->desc == NULL ||
	    !mms->desc->item_table) {
		chg_err("mms or item_table is NULL");
		return -EINVAL;
	}
	item = oplus_mms_get_item(mms, item_id);
	if (item == NULL) {
		chg_err("topic(=%s) item(=%d) not found\n", mms->desc->name,
			item_id);
		return -EINVAL;
	}
	if (msg->payload == MSG_LOAD_NULL) {
		chg_err("msg payload type is MSG_LOAD_NULL\n");
		return -EINVAL;
	}

	write_lock(&item->lock);
	if (msg->payload == MSG_LOAD_INT)
		item->data.intval = *((int *)msg->buf);
	else
		item->data.strval = (char *)msg->buf;
	if (item->desc.str_data) {
		/* Dynamic string data cannot be judged whether to update */
		item->pre_data.strval = item->data.strval;
		item->updated = true;
		goto out;
	} else {
		if (item->desc.up_thr_enable) {
			if ((item->data.intval > item->pre_data.intval) &&
			    (item->pre_data.intval <
			     item->desc.update_thr_up) &&
			    (item->data.intval >= item->desc.update_thr_up)) {
				item->pre_data.intval = item->data.intval;
				item->updated = true;
				goto out;
			} else {
				item->updated = false;
			}
		}
		if (item->desc.update_thr_down) {
			if ((item->data.intval < item->pre_data.intval) &&
			    (item->pre_data.intval >
			     item->desc.update_thr_down) &&
			    (item->data.intval < item->desc.update_thr_down)) {
				item->pre_data.intval = item->data.intval;
				item->updated = true;
				goto out;
			} else {
				item->updated = false;
			}
		}
		if (item->desc.dead_thr_enable) {
			if (abs(item->data.intval - item->pre_data.intval) >=
			    item->desc.dead_zone_thr) {
				item->pre_data.intval = item->data.intval;
				item->updated = true;
				goto out;
			} else {
				item->updated = false;
			}
		}
	}

out:
	write_unlock(&item->lock);

	return 0;
}

void oplus_mms_topic_update(struct oplus_mms *topic, bool publish)
{
	topic->desc->update(topic, publish);
}

struct mms_msg *oplus_mms_alloc_msg(enum mms_msg_type type,
				    enum mms_msg_prio prio, u32 item_id)
{
	struct mms_msg *msg;

	msg = kzalloc(sizeof(struct mms_msg), GFP_KERNEL);
	if (msg == NULL)
		return NULL;

	msg->type = type;
	msg->prio = prio;
	msg->payload = MSG_LOAD_NULL;
	msg->item_id = item_id;

	return msg;
}

struct mms_msg *oplus_mms_alloc_int_msg(enum mms_msg_type type,
					enum mms_msg_prio prio, u32 item_id,
					int data)
{
	struct mms_msg *msg;

	msg = kzalloc(sizeof(struct mms_msg) + sizeof(int), GFP_KERNEL);
	if (msg == NULL)
		return NULL;

	msg->type = type;
	msg->prio = prio;
	msg->payload = MSG_LOAD_INT;
	msg->item_id = item_id;
	*((int *)msg->buf) = data;

	return msg;
}

__printf(4, 5)
struct mms_msg *oplus_mms_alloc_str_msg(enum mms_msg_type type,
					enum mms_msg_prio prio, u32 item_id,
					const char *format, ...)
{
	struct mms_msg *msg;
	va_list args;

	msg = kzalloc(sizeof(struct mms_msg) + TOPIC_MSG_STR_BUF, GFP_KERNEL);
	if (msg == NULL)
		return NULL;

	msg->type = type;
	msg->prio = prio;
	msg->payload = MSG_LOAD_STR;
	msg->item_id = item_id;
	va_start(args, format);
	vsnprintf(msg->buf, TOPIC_MSG_STR_BUF, format, args);
	va_end(args);

	return msg;
}

static void oplus_mms_notify_caller(struct oplus_mms *mms, struct mms_msg *msg)
{
	struct mms_subscribe *subs, *tmp;
	LIST_HEAD(callback_list);
	bool sync = msg->sync;

	rcu_read_lock();
	list_for_each_entry_rcu(subs, &mms->subscribe_list, list) {
		if (!subs->callback)
			continue;
		if (sync)
			list_add_tail(&subs->callback_list_sync, &callback_list);
		else
			list_add_tail(&subs->callback_list, &callback_list);
	}
	rcu_read_unlock();

	/*
	 * For messages with payload, you need to ensure that the
	 * notification is bound to the message and is not interrupted
	 * by other messages.
	 */
	if (msg->payload != MSG_LOAD_NULL)
		oplus_mms_item_update_by_msg(mms, msg->item_id, msg);

	/*
	* The callback function needs to be called outside
	* the RCU critical section.
	*/
	if (sync) {
		list_for_each_entry_safe(subs, tmp, &callback_list, callback_list_sync) {
			if (subs->callback)
				subs->callback(subs, msg->type, msg->item_id, true);
			list_del_init(&subs->callback_list_sync);
		}
	} else {
		list_for_each_entry_safe(subs, tmp, &callback_list, callback_list) {
			if (subs->callback)
				subs->callback(subs, msg->type, msg->item_id, false);
			list_del_init(&subs->callback_list);
		}
	}
}

static int __oplus_mms_publish_msg(struct oplus_mms *mms, struct mms_msg *msg)
{
	struct mms_msg *temp_msg = NULL;
	struct list_head *temp_list = NULL;
	bool update = true;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}
	if (msg == NULL) {
		chg_err("msg is NULL\n");
		return -EINVAL;
	}

	/*
	 * Messages that are not updated regularly need to actively update
	 * the content of the message.
	 * The message must be updated before being added to the message queue
	 * to prevent the message from not being updated after it is sent.
	 */
	if (msg->type != MSG_TYPE_TIMER && msg->payload == MSG_LOAD_NULL)
		update = oplus_mms_item_update(mms, msg->item_id, true);

	/*
	 * Do not publish when the message does not
	 * meet the publishing conditions.
	 */
	if (!update) {
		kfree(msg);
		return (int)update;
	}

	/* sync msg need to be processed directly here */
	if (msg->sync) {
		mutex_lock(&mms->sync_msg_lock);
		oplus_mms_notify_caller(mms, msg);
		mutex_unlock(&mms->sync_msg_lock);
		return (int)update;
	}

	mutex_lock(&mms->msg_lock);
	if (list_empty(&mms->msg_list)) {
		list_add_tail_rcu(&msg->list, &mms->msg_list);
	} else {
		list_for_each_entry_rcu(temp_msg, &mms->msg_list, list) {
			if (temp_msg->prio > msg->prio) {
				temp_list = &temp_msg->list;
				break;
			}
		}
		list_add_tail_rcu(&msg->list,
				  !temp_list ? &mms->msg_list : temp_list);
	}
	synchronize_rcu();
	mutex_unlock(&mms->msg_lock);

	/* All messages are processed before the device is allowed to sleep */
	spin_lock(&mms->changed_lock);
	if (!mms->changed) {
		mms->changed = true;
		pm_stay_awake(&mms->dev);
	}
	spin_unlock(&mms->changed_lock);

	queue_delayed_work(mms_wq, &mms->msg_work, 0);

	return (int)update;
}

int oplus_mms_publish_msg(struct oplus_mms *mms, struct mms_msg *msg)
{
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}
	if (msg == NULL) {
		chg_err("msg is NULL\n");
		return -EINVAL;
	}

	rc = __oplus_mms_publish_msg(mms, msg);
	if (rc < 0)
		return rc;
	return 0;
}

int oplus_mms_publish_msg_sync(struct oplus_mms *mms, struct mms_msg *msg)
{
	int rc;

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return -EINVAL;
	}
	if (msg == NULL) {
		chg_err("msg is NULL\n");
		return -EINVAL;
	}

	msg->sync = true;
	rc = __oplus_mms_publish_msg(mms, msg);
	if (rc <= 0)
		return rc;
	/*
	 * sync need to be released by the sender.
	 */
	kfree(msg);

	return 0;
}

int oplus_mms_publish_ic_err_msg(struct oplus_mms *topic, u32 item_id,
				 struct oplus_chg_ic_err_msg *err_msg)
{
	struct mms_msg *topic_msg;
	int rc;

	topic_msg =
		oplus_mms_alloc_str_msg(MSG_TYPE_ITEM, MSG_PRIO_HIGH, item_id,
					"[%s]-[%d]-[%d]:%s", err_msg->ic->name,
					err_msg->type, err_msg->sub_type,
					err_msg->msg);
	if (topic_msg == NULL) {
		chg_err("alloc topic msg error\n");
		return -ENOMEM;
	}

	rc = oplus_mms_publish_msg_sync(topic, topic_msg);
	if (rc < 0) {
		chg_err("publish topic msg error, rc=%d\n", rc);
		kfree(topic_msg);
	}

	return rc;
}

int oplus_mms_analysis_ic_err_msg(char *buf, size_t buf_size, int *name_index,
				  int *type, int *sub_type, int *msg_index)
{
	char *str;
	int index = 0;
	int type_index, sub_type_index;

	if (buf == NULL) {
		chg_err("buf is NULL\n");
		return -EINVAL;
	}
	if (type == NULL) {
		chg_err("type is NULL\n");
		return -EINVAL;
	}
	if (sub_type == NULL) {
		chg_err("sub_type is NULL\n");
		return -EINVAL;
	}
	if (msg_index == NULL) {
		chg_err("msg_index is NULL\n");
		return -EINVAL;
	}

	str = buf;

	/* name index */
	if (*str != '[') {
		chg_err("buf data is error\n");
		return -EINVAL;
	}
	index++;
	str = buf + index;
	*name_index = index;

	/* type */
	while (*str != 0 && index < buf_size) {
		if (strncmp("]-[", str, 3) == 0) {
			index = index + 3;
			type_index = index;
			break;
		}
		index++;
		str++;
	}
	if (*str == 0 || index >= buf_size) {
		chg_err("buf data is error\n");
		return -EINVAL;
	}
	*str = 0;
	str = buf + index;

	/* sub type */
	while (*str != 0 && index < buf_size) {
		if (strncmp("]-[", str, 3) == 0) {
			index = index + 3;
			sub_type_index = index;
			break;
		}
		index++;
		str++;
	}
	if (*str == 0 || index >= buf_size) {
		chg_err("buf data is error\n");
		return -EINVAL;
	}
	*str = 0;
	str = buf + index;

	/* msg index */
	while (*str != 0 && index < buf_size) {
		if (strncmp("]:", str, 2) == 0) {
			index = index + 2;
			*msg_index = index;
			break;
		}
		index++;
		str++;
	}
	if (*str == 0 || index >= buf_size) {
		chg_err("buf data is error\n");
		return -EINVAL;
	}
	*str = 0;

	if(sscanf(buf + type_index, "%d", type) != 1) {
		chg_err("get ic err type error\n");
		return -EINVAL;
	}
	if(sscanf(buf + sub_type_index, "%d", sub_type) != 1) {
		chg_err("get ic sub err type error\n");
		return -EINVAL;
	}

	return 0;
}

int oplus_mms_wait_topic(const char *name, mms_callback_t call, void *data)
{
	struct oplus_mms *topic;
	struct oplus_mms_call_head *head;

	topic = oplus_mms_get_by_name(name);
	if (topic != NULL) {
		call(topic, data);
		return 0;
	}

	head = kzalloc(sizeof(struct oplus_mms_call_head), GFP_KERNEL);
	if (head == NULL) {
		chg_err("alloc call_head memory error\n");
		return -ENODEV;
	}
	head->name = name;
	head->call = call;
	head->data = data;
	mutex_lock(&wait_list_lock);
	list_add(&head->list, &oplus_mms_wait_list);
	mutex_unlock(&wait_list_lock);

	/*
	 * The topic here may have been registered, but the registered
	 * callback function has not been called, so we need to check
	 * again here.
	 */
	topic = oplus_mms_get_by_name(name);
	if (topic == NULL)
		return 0;
	if (!topic->initialized)
		return 0;
	oplus_mms_call(topic);

	return 0;
}

static void oplus_mms_call(struct oplus_mms *topic)
{
	struct oplus_mms_call_head *head, *tmp;
	LIST_HEAD(tmp_list);

	mutex_lock(&wait_list_lock);
	list_for_each_entry_safe(head, tmp, &oplus_mms_wait_list, list) {
		if (strcmp(head->name, topic->desc->name) != 0)
			continue;
		list_del(&head->list);
		list_add(&head->list, &tmp_list);
	}
	mutex_unlock(&wait_list_lock);

	list_for_each_entry_safe(head, tmp, &tmp_list, list) {
		head->call(topic, head->data);
		list_del(&head->list);
		kfree(head);
	}
}

__printf(4, 5)
struct mms_subscribe *oplus_mms_subscribe(
	struct oplus_mms *mms, void *priv_data,
	void (*callback)(struct mms_subscribe *, enum mms_msg_type, u32, bool),
	const char *format, ...)
{
	struct mms_subscribe *subs, *subs_temp;
	va_list args;
	char name[TOPIC_NAME_MAX] = {0};

	if (mms == NULL) {
		chg_err("mms is NULL\n");
		return ERR_PTR(-EINVAL);
	}
	if (priv_data == NULL) {
		chg_err("priv_data is NULL\n");
		return ERR_PTR(-EINVAL);
	}
	if (callback == NULL) {
		chg_err("callback is NULL\n");
		return ERR_PTR(-EINVAL);
	}

	va_start(args, format);
	(void)vsnprintf(name, TOPIC_NAME_MAX, format, args);
	va_end(args);
	spin_lock(&mms->subscribe_lock);
	list_for_each_entry_rcu(subs_temp, &mms->subscribe_list, list) {
		if (!strcmp(name, subs_temp->name)) {
			spin_unlock(&mms->subscribe_lock);
			chg_info("There are the same name(%s)\n", subs_temp->name);
			return subs_temp;
		}
	}
	spin_unlock(&mms->subscribe_lock);

	subs = kzalloc(sizeof(struct mms_subscribe), GFP_KERNEL);
	if (subs == NULL) {
		chg_err("alloc subs memory error\n");
		return ERR_PTR(-ENOMEM);
	}
	snprintf(subs->name, TOPIC_NAME_MAX, "%s", name);
	subs->priv_data = priv_data;
	subs->callback = callback;
	subs->mms = mms;
	INIT_LIST_HEAD(&subs->callback_list);
	INIT_LIST_HEAD(&subs->callback_list_sync);

	spin_lock(&mms->subscribe_lock);
	list_add_tail_rcu(&subs->list, &mms->subscribe_list);
	spin_unlock(&mms->subscribe_lock);
	atomic_inc(&mms->use_cnt);

	return subs;
}

int oplus_mms_unsubscribe(struct mms_subscribe *subs)
{
	struct oplus_mms *mms;

	if (subs == NULL) {
		chg_err("subs is NULL");
		return -ENODEV;
	}
	mms = subs->mms;

	spin_lock(&mms->subscribe_lock);
	list_del_rcu(&subs->list);
	spin_unlock(&mms->subscribe_lock);
	atomic_dec(&mms->use_cnt);
	synchronize_rcu();
	kfree(subs);

	return 0;
}

int oplus_mms_subs_move_to_top(struct mms_subscribe *subs)
{
	struct mms_subscribe *subs_temp;
	struct oplus_mms *mms;
	bool find = false;

	if (subs == NULL) {
		chg_err("subs is NULL");
		return -EINVAL;
	}
	mms = subs->mms;
	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}

	spin_lock(&mms->subscribe_lock);
	list_for_each_entry_rcu(subs_temp, &mms->subscribe_list, list) {
		if (subs_temp == subs) {
			find = true;
			break;
		}
	}
	if (find) {
		list_del_rcu(&subs->list);
		list_add_rcu(&subs->list, &mms->subscribe_list);
	}
	spin_unlock(&mms->subscribe_lock);

	if (find) {
		synchronize_rcu();
	} else {
		chg_err("%s topic not find %s subs\n", mms->desc->name, subs->name);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(oplus_mms_subs_move_to_top);

int oplus_mms_subs_move_to_down(struct mms_subscribe *subs)
{
	struct mms_subscribe *subs_temp;
	struct oplus_mms *mms;
	bool find = false;

	if (subs == NULL) {
		chg_err("subs is NULL");
		return -EINVAL;
	}
	mms = subs->mms;
	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}

	spin_lock(&mms->subscribe_lock);
	list_for_each_entry_rcu(subs_temp, &mms->subscribe_list, list) {
		if (subs_temp == subs) {
			find = true;
			break;
		}
	}
	if (find) {
		list_del_rcu(&subs->list);
		list_add_tail_rcu(&subs->list, &mms->subscribe_list);
	}
	spin_unlock(&mms->subscribe_lock);

	if (find) {
		synchronize_rcu();
	} else {
		chg_err("%s topic not find %s subs\n", mms->desc->name, subs->name);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(oplus_mms_subs_move_to_down);

int oplus_mms_set_publish_interval(struct oplus_mms *mms, int time_ms)
{
	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}
	mms->normal_update_interval = time_ms;

	return 0;
}

int oplus_mms_stop_publish(struct oplus_mms *mms)
{
	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}
	mms->normal_update_interval = 0;
	cancel_delayed_work(&mms->update_work);

	return 0;
}

int oplus_mms_restore_publish(struct oplus_mms *mms)
{
	if (mms == NULL) {
		chg_err("mms is NULL");
		return -ENODEV;
	}
	mms->normal_update_interval = mms->static_update_interval;
	queue_delayed_work(mms_wq, &mms->update_work,
			   msecs_to_jiffies(mms->normal_update_interval));

	return 0;
}

#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
static ssize_t item_id_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", mms->debug_item_id);
}

static ssize_t item_id_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	int val;

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	mms->debug_item_id = val;
	chg_info("debug_item_id=%d\n", val);

	return count;
}
static DEVICE_ATTR_RW(item_id);

static ssize_t data_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	union mms_msg_data data = { 0 };
	ssize_t rc;

	rc = oplus_mms_get_item_data(mms, mms->debug_item_id, &data, false);
	if (rc < 0)
		return rc;

	if (oplus_mms_item_is_str(mms, mms->debug_item_id))
		rc = sprintf(buf, "%s\n", data.strval);
	else
		rc = sprintf(buf, "%d\n", data.intval);

	return rc;
}
static DEVICE_ATTR_RO(data);

static ssize_t subscribe_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	struct mms_subscribe *subs;
	ssize_t len = 0;

	rcu_read_lock();
	list_for_each_entry_rcu(subs, &mms->subscribe_list, list)
		len += sprintf(buf + len, "%s\n", subs->name);
	rcu_read_unlock();

	return len;
}
static DEVICE_ATTR_RO(subscribe);

#if IS_ENABLED(CONFIG_OPLUS_DEBUG_AUTH)
static ssize_t overwrite_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	union mms_msg_data data;
	struct mms_item *item;
	ssize_t rc;

	item = oplus_mms_get_item(mms, mms->debug_item_id);
	if (item == NULL) {
		chg_err("%s item(=%d) not found\n", mms->desc->name, mms->debug_item_id);
		return -EINVAL;
	}
	if (!item->overwritten) {
		rc = sprintf(buf, "None\n");
		return rc;
	}
	data = item->overwrite_data;

	if (oplus_mms_item_is_str(mms, mms->debug_item_id))
		rc = sprintf(buf, "%s\n", data.strval);
	else
		rc = sprintf(buf, "%d\n", data.intval);

	return rc;
}

static ssize_t overwrite_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	union mms_msg_data data;
	struct mms_item *item;
	const char *data_buf;
	ssize_t data_size;

	data_size = oplus_debug_auth_get_data(buf, count, &data_buf);
	if (data_size < 0) {
		chg_err("get data error, rc=%zd\n", data_size);
		return data_size;
	}

	item = oplus_mms_get_item(mms, mms->debug_item_id);
	if (item == NULL) {
		chg_err("%s item(=%d) not found\n", mms->desc->name, mms->debug_item_id);
		return -EINVAL;
	}

	if (oplus_mms_item_is_str(mms, mms->debug_item_id)) {
		data.strval = devm_kzalloc(dev, data_size + 1, GFP_KERNEL);
		if (data.strval == NULL) {
			chg_err("alloc overwrite_data buf error\n");
			return -ENOMEM;
		}
		if (item->overwritten)
			devm_kfree(dev, item->overwrite_data.strval);
		item->overwrite_data.strval = data.strval;
		chg_info("%d: overwrite_data=%s\n", mms->debug_item_id, data.strval);
		memcpy(data.strval, data_buf, data_size);
	} else {
		if (kstrtos32(data_buf, 0, &data.intval)) {
			chg_err("buf error\n");
			return -EINVAL;
		}
		item->overwrite_data.intval = data.intval;
		chg_info("%d: overwrite_data=%d\n", mms->debug_item_id, data.intval);
	}
	item->overwritten = true;

	return count;
}
static DEVICE_ATTR_RW(overwrite);

static ssize_t clean_overwrite_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	struct mms_item *item;

	item = oplus_mms_get_item(mms, mms->debug_item_id);
	if (item == NULL) {
		chg_err("%s item(=%d) not found\n", mms->desc->name, mms->debug_item_id);
		return -EINVAL;
	}
	if (!item->overwritten)
		return count;

	if (oplus_mms_item_is_str(mms, mms->debug_item_id)) {
		devm_kfree(dev, item->overwrite_data.strval);
		item->overwrite_data.strval = NULL;
	}
	item->overwritten = false;

	return count;
}
static DEVICE_ATTR_WO(clean_overwrite);

static ssize_t publish_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	struct mms_msg *msg;
	enum mms_msg_type type;
	enum mms_msg_prio prio;
	const char *data_buf;
	ssize_t data_size;
	ssize_t rc;

	data_size = oplus_debug_auth_get_data(buf, count, &data_buf);
	if (data_size < 0) {
		chg_err("get data error, rc=%zd\n", data_size);
		return data_size;
	}

	if(sscanf(data_buf, "%u,%u", &type, &prio) != 2) {
		chg_err("get msg info error\n");
		return -EINVAL;
	}

	msg = oplus_mms_alloc_msg(type, prio, (type == MSG_TYPE_TIMER) ? 0 : mms->debug_item_id);
	if (msg == NULL) {
		chg_err("alloc msg buf error\n");
		return -ENOMEM;
	}
	rc = oplus_mms_publish_msg(mms, msg);
	if (rc < 0) {
		chg_err("publish msg error, rc=%zd\n", rc);
		kfree(msg);
		return -EFAULT;
	}
	chg_info("%d: publish, type=%u, prio=%u\n", mms->debug_item_id, type, prio);

	return count;
}
static DEVICE_ATTR_WO(publish);
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */

static ssize_t items_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	int i = 0;
	ssize_t len = 0;

	for (i = 0; i < mms->desc->item_num - 1; i++)
		len += sprintf(buf + len, "%d,", mms->desc->item_table[i].desc.item_id);
	len += sprintf(buf + len, "%d\n", mms->desc->item_table[i].desc.item_id);

	return len;
}
static DEVICE_ATTR_RO(items);

static ssize_t is_str_data_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct oplus_mms *mms = dev_get_drvdata(dev);
	struct mms_item *item;
	ssize_t rc;

	item = oplus_mms_get_item(mms, mms->debug_item_id);
	if (item == NULL) {
		chg_err("%s item(=%d) not found\n", mms->desc->name, mms->debug_item_id);
		return -EINVAL;
	}

	if (item->desc.str_data)
		rc = sprintf(buf, "True\n");
	else
		rc = sprintf(buf, "False\n");

	return rc;
}
static DEVICE_ATTR_RO(is_str_data);

static struct device_attribute *oplus_mms_attributes[] = {
	&dev_attr_item_id,
	&dev_attr_data,
	&dev_attr_subscribe,
#if IS_ENABLED(CONFIG_OPLUS_DEBUG_AUTH)
	&dev_attr_overwrite,
	&dev_attr_clean_overwrite,
	&dev_attr_publish,
#endif /* CONFIG_OPLUS_DEBUG_AUTH */
	&dev_attr_items,
	&dev_attr_is_str_data,
	NULL
};
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */

#if IS_ENABLED(CONFIG_OPLUS_CHG_MMS_PUBLISH_USERSPACE)
enum {
	TOPIC_ENV_ITEM = 0,
	TOPIC_ENV_DATA,
	TOPIC_ENV_MAX,
};

static void userspace_subs_callback(struct mms_subscribe *subs, enum mms_msg_type type, u32 id, bool sync)
{
	struct oplus_mms *mms = subs->priv_data;
	struct mms_item *item;
	char *item_env_buf, *data_env_buf;
	char *env[TOPIC_ENV_MAX + 1] = { 0 };
	int item_env_len, data_env_len;

	/*
	 * The importance and urgency of timing messages are generally not high.
	 * Frequent sending will cause excessive power consumption. For such
	 * information, it should be proactively obtained in the user space as
	 * needed.
	 */
	if (type != MSG_TYPE_ITEM)
		return;

	item_env_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (!item_env_buf)
		return;
	data_env_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (!data_env_buf) {
		free_page((unsigned long)item_env_buf);
		return;
	}
	item_env_len = snprintf(item_env_buf, PAGE_SIZE, "CHANGED_ITEM=");
	data_env_len = snprintf(data_env_buf, PAGE_SIZE, "ITEM_DATA=");

	snprintf(item_env_buf + item_env_len, PAGE_SIZE - item_env_len, "%d", id);
	item = oplus_mms_get_item(mms, id);
	if (item == NULL) {
		snprintf(data_env_buf + data_env_len, PAGE_SIZE - data_env_len,
			 "NA");
	} else {
		read_lock(&item->lock);
		if (item->desc.str_data)
			snprintf(data_env_buf + data_env_len,
				 PAGE_SIZE - data_env_len, "\"%s\"",
				 item->data.strval);
		else
			snprintf(data_env_buf + data_env_len,
				 PAGE_SIZE - data_env_len, "%d",
				 item->data.intval);
		read_unlock(&item->lock);
	}

	env[TOPIC_ENV_ITEM] = item_env_buf;
	env[TOPIC_ENV_DATA] = data_env_buf;
	kobject_uevent_env(&mms->dev.kobj, KOBJ_CHANGE, env);

	free_page((unsigned long)item_env_buf);
	free_page((unsigned long)data_env_buf);
}
#endif /* CONFIG_OPLUS_CHG_MMS_PUBLISH_USERSPACE */

static void oplus_mms_update_work(struct work_struct *work)
{
	struct oplus_mms *mms = container_of(work, struct oplus_mms,
					update_work.work);

	mms->desc->update(mms, true);

	if (mms->normal_update_interval > 0)
		queue_delayed_work(mms_wq, &mms->update_work,
				   msecs_to_jiffies(mms->normal_update_interval));
}

static void oplus_mms_msg_work(struct work_struct *work)
{
	struct oplus_mms *mms = container_of(work, struct oplus_mms,
					msg_work.work);
	struct mms_msg *msg;

	rcu_read_lock();
	if (list_empty(&mms->msg_list)) {
		rcu_read_unlock();
		spin_lock(&mms->changed_lock);
		if (likely(mms->changed)) {
			mms->changed = false;
			pm_relax(&mms->dev);
		}
		spin_unlock(&mms->changed_lock);
		return;
	}
	msg = list_entry_rcu(mms->msg_list.next, struct mms_msg, list);
	rcu_read_unlock();
	oplus_mms_notify_caller(mms, msg);

	mutex_lock(&mms->msg_lock);
	list_del_rcu(&msg->list);
	mutex_unlock(&mms->msg_lock);
	synchronize_rcu();
	kfree(msg);

	queue_delayed_work(mms_wq, &mms->msg_work, 0);
}

static void oplus_mms_callback_work(struct work_struct *work)
{
	struct oplus_mms *mms = container_of(work, struct oplus_mms,
					callback_work);

	oplus_mms_call(mms);
}

static int oplus_mms_match_device_by_name(struct device *dev, const void *data)
{
	const char *name = data;
	struct oplus_mms *mms = dev_get_drvdata(dev);

	return strcmp(mms->desc->name, name) == 0;
}

struct oplus_mms *oplus_mms_get_by_name(const char *name)
{
	struct oplus_mms *mms = NULL;
	struct device *dev = class_find_device(oplus_mms_class, NULL, name,
					oplus_mms_match_device_by_name);

	if (dev) {
		mms = dev_get_drvdata(dev);
		atomic_inc(&mms->use_cnt);
	}

	return mms;
}

void oplus_mms_put(struct oplus_mms *mms)
{
	might_sleep();

	atomic_dec(&mms->use_cnt);
	put_device(&mms->dev);
}

static void oplus_mms_dev_release(struct device *dev)
{
	struct oplus_mms *mms = to_oplus_mms(dev);
	dev_dbg(dev, "%s\n", __func__);
	kfree(mms);
}

static struct oplus_mms *__must_check
__oplus_mms_register(struct device *parent, const struct oplus_mms_desc *desc,
		     const struct oplus_mms_config *cfg, bool ws)
{
	struct device *dev;
	struct oplus_mms *mms;
	int rc;
	int i;
#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
	struct device_attribute **attrs;
	struct device_attribute *attr;
#endif

	if (!parent)
		chg_info("Expected proper parent device for '%s'\n", desc->name);

	if (!desc || !desc->name || !desc->item_table || !desc->item_num)
		return ERR_PTR(-EINVAL);

	if (desc->update_interval && !desc->update)
		return ERR_PTR(-EINVAL);

	mms = kzalloc(sizeof(struct oplus_mms), GFP_KERNEL);
	if (!mms)
		return ERR_PTR(-ENOMEM);

	dev = &mms->dev;

	device_initialize(dev);

	dev->class = oplus_mms_class;
	dev->type = &oplus_mms_dev_type;
	dev->parent = parent;
	dev->release = oplus_mms_dev_release;
	dev_set_drvdata(dev, mms);
	mms->desc = desc;
	mms->force_update = false;
	if (cfg) {
		dev->groups = cfg->attr_grp;
		mms->drv_data = cfg->drv_data;
		mms->of_node =
			cfg->fwnode ? to_of_node(cfg->fwnode) : cfg->of_node;
		if (cfg->update_interval)
			mms->static_update_interval = cfg->update_interval;
	}
	if (mms->static_update_interval == 0)
		mms->static_update_interval = desc->update_interval;
	mms->normal_update_interval = mms->static_update_interval;

	rc = dev_set_name(dev, "%s", desc->name);
	if (rc)
		goto dev_set_name_failed;

	spin_lock_init(&mms->subscribe_lock);
	mutex_init(&mms->msg_lock);
	mutex_init(&mms->sync_msg_lock);
	spin_lock_init(&mms->changed_lock);
	for (i = 0; i < desc->item_num; i++) {
		desc->item_table[i].disabled = false;
		rwlock_init(&desc->item_table[i].lock);
		mutex_init(&desc->item_table[i].update_lock);
	}
	INIT_DELAYED_WORK(&mms->update_work, oplus_mms_update_work);
	INIT_DELAYED_WORK(&mms->msg_work, oplus_mms_msg_work);
	INIT_WORK(&mms->callback_work, oplus_mms_callback_work);
	INIT_LIST_HEAD(&mms->subscribe_list);
	INIT_LIST_HEAD(&mms->msg_list);

	rc = device_add(dev);
	if (rc)
		goto device_add_failed;

	rc = device_init_wakeup(dev, ws);
	if (rc)
		goto wakeup_init_failed;

#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
	attrs = oplus_mms_attributes;
	while ((attr = *attrs++)) {
		rc = device_create_file(&mms->dev, attr);
		if (rc) {
			chg_err("device create file fail, rc=%d\n", rc);
			goto device_create_file_err;
		}
	}
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */

	atomic_inc(&mms->use_cnt);
	mms->initialized = true;

#if IS_ENABLED(CONFIG_OPLUS_CHG_MMS_PUBLISH_USERSPACE)
	mms->userspace_subs =
		oplus_mms_subscribe(mms, mms, userspace_subs_callback, "userspace");
	if (IS_ERR_OR_NULL(mms->userspace_subs)) {
		chg_err("userspace subscribe %s topic error, rc=%ld\n",
			mms->desc->name,
			PTR_ERR(mms->userspace_subs));
	}
#endif /* CONFIG_OPLUS_CHG_MMS_PUBLISH_USERSPACE */

	kobject_uevent(&dev->kobj, KOBJ_CHANGE);

	queue_delayed_work(mms_wq, &mms->update_work, 0);
	schedule_work(&mms->callback_work);

	return mms;

#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
device_create_file_err:
	device_init_wakeup(&mms->dev, false);
	device_unregister(&mms->dev);
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */
wakeup_init_failed:
device_add_failed:
dev_set_name_failed:
	put_device(dev);
	return ERR_PTR(rc);
}

struct oplus_mms *__must_check oplus_mms_register(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg)
{
	return __oplus_mms_register(parent, desc, cfg, true);
}

struct oplus_mms *__must_check
oplus_mms_register_no_ws(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg)
{
	return __oplus_mms_register(parent, desc, cfg, false);
}

static void devm_oplus_mms_release(struct device *dev, void *res)
{
	struct oplus_mms **mms = res;

	oplus_mms_unregister(*mms);
}

struct oplus_mms *__must_check
devm_oplus_mms_register(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg)
{
	struct oplus_mms **ptr, *mms;

	ptr = devres_alloc(devm_oplus_mms_release, sizeof(*ptr), GFP_KERNEL);

	if (!ptr)
		return ERR_PTR(-ENOMEM);
	mms = __oplus_mms_register(parent, desc, cfg, true);
	if (IS_ERR(mms)) {
		devres_free(ptr);
	} else {
		*ptr = mms;
		devres_add(parent, ptr);
	}
	return mms;
}

struct oplus_mms *__must_check
devm_oplus_mms_register_no_ws(struct device *parent,
		const struct oplus_mms_desc *desc,
		const struct oplus_mms_config *cfg)
{
	struct oplus_mms **ptr, *mms;

	ptr = devres_alloc(devm_oplus_mms_release, sizeof(*ptr), GFP_KERNEL);

	if (!ptr)
		return ERR_PTR(-ENOMEM);
	mms = __oplus_mms_register(parent, desc, cfg, false);
	if (IS_ERR(mms)) {
		devres_free(ptr);
	} else {
		*ptr = mms;
		devres_add(parent, ptr);
	}
	return mms;
}

void oplus_mms_unregister(struct oplus_mms *mms)
{
#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
	struct device_attribute **attrs;
	struct device_attribute *attr;
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */

	WARN_ON(atomic_dec_return(&mms->use_cnt));
	mms->removing = true;
	cancel_delayed_work_sync(&mms->update_work);
	cancel_delayed_work_sync(&mms->msg_work);
	cancel_work_sync(&mms->callback_work);
	sysfs_remove_link(&mms->dev.kobj, "powers");
#if IS_ENABLED(CONFIG_OPLUS_CHG_MMS_PUBLISH_USERSPACE)
	if (!IS_ERR_OR_NULL(mms->userspace_subs))
		oplus_mms_unsubscribe(mms->userspace_subs);
#endif /* CONFIG_OPLUS_CHG_MMS_PUBLISH_USERSPACE */
#ifdef CONFIG_OPLUS_CHG_MMS_DEBUG
	attrs = oplus_mms_attributes;
	while ((attr = *attrs++))
		device_remove_file(&mms->dev, attr);
#endif /* CONFIG_OPLUS_CHG_MMS_DEBUG */
	device_init_wakeup(&mms->dev, false);
	device_unregister(&mms->dev);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0))
static int oplus_mms_uevent(const struct device *dev, struct kobj_uevent_env *env)
#else
static int oplus_mms_uevent(struct device *dev, struct kobj_uevent_env *env)
#endif
{
	return 0;
}

void *oplus_mms_get_drvdata(struct oplus_mms *mms)
{
	return mms->drv_data;
}

static __init int oplus_mms_class_init(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
	oplus_mms_class = class_create("oplus_mms");
#else
	oplus_mms_class = class_create(THIS_MODULE, "oplus_mms");
#endif

	if (IS_ERR(oplus_mms_class))
		return PTR_ERR(oplus_mms_class);

	oplus_mms_class->dev_uevent = oplus_mms_uevent;

	mms_wq = alloc_workqueue("mms_wq", WQ_UNBOUND | WQ_FREEZABLE | WQ_HIGHPRI, 0);
	if (!mms_wq) {
		pr_err("alloc mms_wq error\n");
		class_destroy(oplus_mms_class);
		return ENOMEM;
	}

	return 0;
}

static __exit void oplus_mms_class_exit(void)
{
	destroy_workqueue(mms_wq);
	class_destroy(oplus_mms_class);
}

oplus_chg_module_core_register(oplus_mms_class);
