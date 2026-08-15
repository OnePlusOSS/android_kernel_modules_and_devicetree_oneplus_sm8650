// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/slab.h>

#include "inc/tcpci.h"
#include "inc/tcpci_typec.h"

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
#include "pd_dpm_prv.h"
#include "inc/tcpm.h"
#ifdef CONFIG_RECV_BAT_ABSENT_NOTIFY
#include "mtk_battery.h"
#endif /* CONFIG_RECV_BAT_ABSENT_NOTIFY */
#endif /* CONFIG_USB_POWER_DELIVERY */

#define TCPC_CORE_VERSION		"2.0.16_G"

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
#ifdef CONFIG_TCPC_NOTIFIER_LATE_SYNC
#ifdef CONFIG_RECV_BAT_ABSENT_NOTIFY
static int fg_bat_notifier_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct pd_port *pd_port = container_of(nb, struct pd_port, fg_bat_nb);
	struct tcpc_device *tcpc = pd_port->tcpc;

	switch (event) {
	case EVENT_BATTERY_PLUG_OUT:
		dev_info(&tcpc->dev, "%s: fg battery absent\n", __func__);
		schedule_work(&pd_port->fg_bat_work);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}
#endif /* CONFIG_RECV_BAT_ABSENT_NOTIFY */
#endif /* CONFIG_TCPC_NOTIFIER_LATE_SYNC */
#endif /* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_TCPC_NOTIFIER_LATE_SYNC
static int __tcpc_class_complete_work(struct device *dev, void *data)
{
	struct tcpc_device *tcpc = dev_get_drvdata(dev);
#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
#ifdef CONFIG_RECV_BAT_ABSENT_NOTIFY
	struct notifier_block *fg_bat_nb = &tcpc->pd_port.fg_bat_nb;
	int ret = 0;
#endif /* CONFIG_RECV_BAT_ABSENT_NOTIFY */
#endif /* CONFIG_USB_POWER_DELIVERY */

	if (tcpc != NULL) {
		pr_info("%s = %s\n", __func__, dev_name(dev));
#if 1
		tcpc_device_irq_enable(tcpc);
#else
		schedule_delayed_work(&tcpc->init_work,
			msecs_to_jiffies(1000));
#endif

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
#ifdef CONFIG_RECV_BAT_ABSENT_NOTIFY
		fg_bat_nb->notifier_call = fg_bat_notifier_call;
#if 0 /* CONFIG_MTK_GAUGE_VERSION == 30 */
		ret = register_battery_notifier(fg_bat_nb);
#endif
		if (ret < 0) {
			pr_notice("%s: register bat notifier fail\n", __func__);
			return -EINVAL;
		}
#endif /* CONFIG_RECV_BAT_ABSENT_NOTIFY */
#endif /* CONFIG_USB_POWER_DELIVERY */
	}
	return 0;
}


#ifdef OPLUS_FEATURE_CHG_BASIC
//#ifndef CONFIG_QGKI
#define OPLUS_RETRY_COUNT 60
#define OPLUS_WORK_DELAY round_jiffies_relative(msecs_to_jiffies(500))
static struct delayed_work tcpc_complete_work;
static void oplus_tcpc_complete_work(struct work_struct *data)
{

	static int count = 0;
	struct tcpc_device *tcpc_dev;

	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	if (!tcpc_dev) {
		count++;
		pr_info("%s type_c_port0 not found retry count=%d\n", __func__, count);
		if (count < OPLUS_RETRY_COUNT)
			schedule_delayed_work(&tcpc_complete_work, OPLUS_WORK_DELAY);
		return;
	}

	if (!IS_ERR(tcpc_class)) {
		class_for_each_device(tcpc_class, NULL, NULL,
			__tcpc_class_complete_work);
	}
}
//#endif /* !CONFIG_QGKI */
#endif /* OPLUS_FEATURE_CHG_BASIC */

static __init int tcpc_class_complete_init(void)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
//#ifdef CONFIG_QGKI
	struct tcpc_device *tcpc_dev;
#ifndef OPLUS_CHG_SEPARATE_MUSE_TCPC
	return 0;
#endif
	pr_info("%s\n", __func__);
	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	if (!tcpc_dev) {
		pr_info("%s type_c_port0 not found,start retry\n", __func__);
		INIT_DELAYED_WORK(&tcpc_complete_work, oplus_tcpc_complete_work);
		schedule_delayed_work(&tcpc_complete_work, OPLUS_WORK_DELAY);
		return 0;
	}

	if (!IS_ERR(tcpc_class)) {
		class_for_each_device(tcpc_class, NULL, NULL,
			__tcpc_class_complete_work);
	}
//#endif /* CONFIG_QGKI */
#endif /* OPLUS_FEATURE_CHG_BASIC */
	return 0;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
void tcpc_late_sync(void)
{
#ifndef CONFIG_QGKI
	if (!IS_ERR(tcpc_class)) {
		class_for_each_device(tcpc_class, NULL, NULL,
			__tcpc_class_complete_work);
	}
#endif /* !CONFIG_QGKI */
}
EXPORT_SYMBOL_GPL(tcpc_late_sync);
#endif /* OPLUS_FEATURE_CHG_BASIC */

late_initcall_sync(tcpc_class_complete_init);
#endif /* CONFIG_TCPC_NOTIFIER_LATE_SYNC */

MODULE_DESCRIPTION("Richtek TypeC Port Control Core");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com>");
MODULE_VERSION(TCPC_CORE_VERSION);
MODULE_LICENSE("GPL");