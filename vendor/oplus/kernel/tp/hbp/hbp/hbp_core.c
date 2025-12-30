#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/syscalls.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <uapi/linux/sched/types.h>
#include <linux/input.h>
#include <linux/input/mt.h>

#include "hbp_core.h"
#include "utils/debug.h"
#include "hbp_spi.h"
#include "hbp_tui.h"

#include "hbp_power.h"
extern void hbp_power_ctrl(struct hbp_device *hbp_dev, struct power_sequeue sq[]);
extern void hbp_power_type_ctrl(struct hbp_device *hbp_dev, enum power_type type, bool en);

struct hbp_core *g_hbp;
struct task_struct *suspend_task = NULL;

void split_touch_cmdline(void);
int hbp_init_vm_mem(struct device_node *np, struct hbp_core *hbp);
bool hbp_tui_mem_map(struct hbp_device *hbp_dev, const char *vm_env);
int hbp_register_sysfs(struct hbp_core *hbp);

extern struct hbp_device *hbp_device_create(void *priv,
		struct hbp_core *hbp,
		struct device *dev,
		struct dev_operations *dev_ops,
		int id);


int hbp_exception_report(hbp_excep_type excep_tpye, void *summary, unsigned int summary_size)
{
	struct hbp_core *hbp = g_hbp;

	if (!hbp) {
		return -EINVAL;
	}

	hbp_tp_exception_report(&hbp->exception_data, excep_tpye, summary, summary_size);

	return 0;
}
EXPORT_SYMBOL(hbp_exception_report);

static struct hbp_device *__hbp_find_device(void *priv)
{
	int i = 0;
	struct hbp_core *hbp = g_hbp;
	struct hbp_device *hbp_dev = NULL;

	if (!hbp) {
		return NULL;
	}

	for (i = 0; i < MAX_DEVICES; i++) {
		hbp_dev = hbp->devices[i];
		if (hbp_dev && (hbp_dev->priv == priv)) {
			return hbp_dev;
		}
	}

	return NULL;
}

static int hbp_match_device_id(struct device_node *np, struct hbp_core *hbp)
{
	int i = 0, cnt = 0;
	int id = MAX_DEVICES;
	int ret = 0;
	struct device_node *dev_np;

	cnt = of_count_phandle_with_args(hbp->dev->of_node, "hbp,devices", NULL);
	for (i = 0; i < cnt; i++) {
		dev_np = of_parse_phandle(hbp->dev->of_node, "hbp,devices", i);
		hbp_info("dev_np %s np %s\n", dev_np->name, np->name);
		if (dev_np && (dev_np == np)) {
			ret = of_property_read_u32(np, "device,id", &id);
			if (ret < 0) {
				hbp_err("Failed to read device id\n");
				return ret;
			}
			break;
		}
	}

	hbp_info("matched device id: %d\n", id);
	return id;
}

int hbp_match_bus(struct device *dev, struct bus_operations **bus_ops)
{
	struct device_node *np = NULL;
	struct device *bus_dev = NULL;
	struct spi_bus *bus;

	np = of_parse_phandle(dev->of_node, "device,attached_bus", 0);
	if (!np) {
		hbp_err("Failed to find attached bus\n");
		return -ENODEV;
	}

	bus_dev = bus_find_device_by_name(&platform_bus_type, NULL, np->full_name);
	if (!bus_dev) {
		hbp_err("Failed to match bus: %s, defer retry\n", np->full_name);
		return -EPROBE_DEFER;
	}

	hbp_info("matched bus:%s\n", np->full_name);
	bus = (struct spi_bus *)bus_dev->platform_data;
	if (bus) {
		*bus_ops = &bus->spi_ops;
	}

	return 0;
}

int hbp_register_devices(void *priv,
			 struct device *dev,
			 struct dev_operations *dev_ops,
			 struct chip_info *chip,
			 struct bus_operations **bus_ops)
{
	int id = MAX_DEVICES;
	struct device_node *np = dev->of_node;
	struct hbp_core *hbp = g_hbp;
	struct hbp_device *hbp_dev;
	int ret = 0;

	if (!hbp) {
		return -EPROBE_DEFER;
	}

	//match bus, spi or i2c
	ret = hbp_match_bus(dev, bus_ops);
	if (ret < 0) {
		return ret;
	}
	hbp_info("%s start.\n", dev->of_node->name);

	id = hbp_match_device_id(np, hbp);
	if (id < 0 || id >= MAX_DEVICES) {
		hbp_err("device id not match\n");
		return -ENODEV;
	}

	if (hbp->devices[id]) {
		hbp_info("device already registered\n");
		return 0;
	}

	hbp_dev = hbp_device_create(priv, hbp, dev, dev_ops, id);
	if (!hbp_dev) {
		return -ENODEV;
	} else if (hbp_dev && hbp_dev->errReason < 0) {
		hbp_err("device create error\n");
		ret = hbp_dev->errReason;
		if (hbp_dev) {
			kfree(hbp_dev);
			hbp_dev = NULL;
			hbp_err("hbp_dev is free\n");
		}
		return ret;
	}

	hbp->devices[id] = hbp_dev;

	hbp->dev_info.device[id].id = id;
	memcpy(hbp->dev_info.device[id].ic_name, chip->ic_name, strlen(chip->ic_name));
	memcpy(hbp->dev_info.device[id].vendor, chip->vendor, strlen(chip->vendor));

	hbp->dev_info.device[id].max_x = hbp_dev->hw.resolution.x;
	hbp->dev_info.device[id].max_y = hbp_dev->hw.resolution.y;
	hbp->dev_info.panels_attached++;

	hbp_info("device %s %s registed, panel attached %d\n",
		 hbp->dev_info.device[id].ic_name,
		 hbp->dev_info.device[id].vendor,
		 hbp->dev_info.panels_attached);

	return 0;
}
EXPORT_SYMBOL(hbp_register_devices);

int hbp_unregister_devices(void *priv)
{
	struct hbp_device *hbp_dev = __hbp_find_device(priv);

	if (hbp_dev) {
		//hbp_unregister_irq(hbp_dev);
		//kfree(hbp_dev);
	}

	return 0;
}
EXPORT_SYMBOL(hbp_unregister_devices);

//TODO:for tddi ic
bool hbp_power_on_in_suspend(int index)
{
	struct hbp_device *hbp_dev;

	if (!g_hbp || !g_hbp->devices[index]) {
		return false;
	}

	hbp_dev = g_hbp->devices[index];

	return 1;//hbp_features_in_suspend(&hbp_dev->features);
}
EXPORT_SYMBOL(hbp_power_on_in_suspend);

void hbp_dev_power_type_ctrl(void *priv, enum power_type type, bool en)
{
	struct hbp_device *hbp_dev = __hbp_find_device(priv);

	if (hbp_dev) {
		hbp_power_type_ctrl(hbp_dev, type, en);
	} else {
		hbp_err("%s: hbp_dev is null.\n", __func__);
	}
}
EXPORT_SYMBOL(hbp_dev_power_type_ctrl);

void hbp_dev_healthinfo_report(void *priv, char *report)
{
	struct hbp_device *hbp_dev = __hbp_find_device(priv);

	if (hbp_dev) {
		hbp_healthinfo_report(&hbp_dev->monitor_data, report);
	} else {
		hbp_err("%s: hbp_dev is null.\n", __func__);
	}
}
EXPORT_SYMBOL(hbp_dev_healthinfo_report);

static int hbp_sync_with_daemon(struct hbp_core *hbp, int id, hbp_panel_event event)
{
	int ret = 0;

	hbp->states[id].id = id;
	hbp->states[id].state = event;
	if (hbp->devices[id]->screenoff_ifp) {
		hbp->states[id].value |= STATE_BIT_IFP_DOWN;
	} else {
		hbp->states[id].value &= ~STATE_BIT_IFP_DOWN;
	}

	hbp_debug("states[%d].value = %d\n", id, hbp->states[id].value);

	hbp->state_st = STATE_WAKEUP;
	wake_up_interruptible(&hbp->state_event);

	hbp->state_ack = ACK_WAITQ;
	ret = wait_event_timeout(hbp->ack_event,
				 (hbp->state_ack == ACK_WAKEUP),
				 msecs_to_jiffies(DAEMON_ACK_TIMEOUT));
	if (!ret) {
		hbp_err("failed to wait manager ack %d\n", hbp->state_ack);
	}

	hbp->devices[id]->state = event;
	hbp_info("sync event %d ret %d\n", event, ret);
	return 0;
}

hbp_panel_event hbp_panel_event_convert(hbp_panel_event event)
{
	//TODO:
	//(1) if oncell panel, ignore suspend event, only use early suspend event
	//to avoid repeat early suspend or suspend event
	//(2) if tddi ic, need update
	if (event == HBP_PANEL_EVENT_SUSPEND) {
		return HBP_PANEL_EVENT_EARLY_SUSPEND;
	}

	if (event == HBP_PANEL_EVENT_RESUME) {
		return HBP_PANEL_EVENT_EARLY_RESUME;
	}

	return event;
}

void hbp_state_notify(struct hbp_core *hbp, int id, hbp_panel_event event)
{
	hbp_debug("notify id %d event %d\n", id, event);

	if (!hbp || id >= MAX_DEVICES || !hbp->devices[id]) {
		hbp_err("invalid device id = %d \n", id);
		return;
	}

	event = hbp_panel_event_convert(event);

	if (hbp->states[id].id == id &&
		hbp->states[id].state == event) {
		hbp_info("same screen notify event %d, ignore\n", event);
		return;
	}

	switch (event) {
	case HBP_PANEL_EVENT_EARLY_SUSPEND:
		hbp->devices[id]->screenoff_ifp = false;
		hbp_info("tp_suspend start\n");
		break;
	case HBP_PANEL_EVENT_EARLY_RESUME:
		hbp_info("tp_resume start\n");
		break;
	default:
		break;
	}

	hbp_sync_with_daemon(hbp, id, event);
}

static int hbp_core_open(struct inode *inode, struct file *file)
{
	struct hbp_device *hbp_dev = NULL;
	int i = 0;

	file->private_data = (void *)g_hbp;

	for (i = 0; i < MAX_DEVICES; i++) {
		hbp_dev = g_hbp->devices[i];
		if (hbp_dev && hbp_dev->dev_ops && hbp_dev->dev_ops->enable_hbp_mode) {
			hbp_dev->dev_ops->enable_hbp_mode(hbp_dev->priv, true);
		}
	}

	g_hbp->in_hbp_mode = true;

	return 0;
}

static int hbp_core_cdev_release(struct inode *inode, struct file *file)
{
	struct hbp_device *hbp_dev = NULL;
	int i = 0;

	file->private_data = NULL;

	for (i = 0; i < MAX_DEVICES; i++) {
		hbp_dev = g_hbp->devices[i];
		if (hbp_dev && hbp_dev->dev_ops && hbp_dev->dev_ops->enable_hbp_mode) {
			hbp_dev->dev_ops->enable_hbp_mode(hbp_dev->priv, false);
		}
	}

	g_hbp->in_hbp_mode = false;

	return 0;
}

static long hbp_core_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct hbp_core *hbp = (struct hbp_core *)filp->private_data;

	if (!hbp) {
		hbp_err("hbp is NULL\n");
		return -EFAULT;
	}

	switch (cmd) {
	case HBP_CORE_DEVINFO:
		if (copy_to_user((void __user *)arg, &hbp->dev_info, sizeof(struct device_info))) {
			hbp_err("failed to copy device info to user\n");
			return -EFAULT;
		}
		break;
	case HBP_CORE_POWER_IN_SLEEP:
		hbp->power_in_sleep = !!arg;
		break;
	case HBP_CORE_GET_STATE:
		if (copy_to_user((void __user *)arg, &hbp->states[0], sizeof(struct device_state)*MAX_DEVICES)) {
			hbp_err("failed to copy state to user\n");
			return -EFAULT;
		}
		break;
	case HBP_CORE_STATE_ACK:
		hbp->state_ack = ACK_WAKEUP;
		wake_up_all(&hbp->ack_event);
		break;
	case HBP_CORE_GET_GESTURE_COORD:
		mutex_lock(&hbp->gesture_mtx);
		if (copy_to_user((void __user *)arg, &hbp->gesture_info, sizeof(struct gesture_info))) {
			hbp_err("failed to copy gesture info to user\n");
			mutex_unlock(&hbp->gesture_mtx);
			return -EFAULT;
		}
		mutex_unlock(&hbp->gesture_mtx);
		break;
	default:
		ret = -EIO;
		break;
	}

	return ret;
}

void hbp_core_set_gesture_coord(struct gesture_info *gesture)
{
	struct hbp_core *hbp = g_hbp;

	mutex_lock(&hbp->gesture_mtx);
	memcpy(&g_hbp->gesture_info, gesture, sizeof(struct gesture_info));
	mutex_unlock(&hbp->gesture_mtx);
}

static __poll_t hbp_core_poll(struct file *file, poll_table *wait)
{
	struct hbp_core *hbp = g_hbp;
	__poll_t mask = 0;

	poll_wait(file, &hbp->state_event, wait);
	if (hbp->state_st == STATE_WAKEUP) {
		hbp->state_st = STATE_WAITQ;
		mask = EPOLLIN | EPOLLRDNORM;
	}
	return mask;
}

static struct file_operations hbp_core_fops = {
	.owner = THIS_MODULE,
	.open = hbp_core_open,
	.poll = hbp_core_poll,
	.unlocked_ioctl = hbp_core_unlocked_ioctl,
	.release = hbp_core_cdev_release,
};

static int core_register_dev(struct hbp_core *hbp)
{

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	hbp->cls = class_create(HBP_CORE);
#else
	hbp->cls = class_create(THIS_MODULE, HBP_CORE);
#endif
	if (IS_ERR(hbp->cls)) {
		hbp_fatal("Failed to class create\n");
		return -EINVAL;
	}

	hbp->major = register_chrdev(0, HBP_CORE, &hbp_core_fops);
	if (hbp->major < 0) {
		hbp_fatal("Failed to register char device \n");
		goto err_cls;
	}

	hbp->cdev = device_create(hbp->cls, NULL, MKDEV(hbp->major, 0), hbp, HBP_CORE);
	if (IS_ERR(hbp->cdev)) {
		hbp_fatal("Failed to create device\n");
		goto err_chrdev;
	}

	return 0;

err_chrdev:
	unregister_chrdev(hbp->major, HBP_CORE);
err_cls:
	class_destroy(hbp->cls);

	return -EINVAL;
}

static int hbp_main_dt(struct device_node *np, struct hbp_core *hbp)
{
	int ret = 0;
    u32 panel_expected  = 1;

	ret = of_property_read_string(np, "hbp,project", &hbp->prj);
	if (ret < 0) {
		hbp_err("failed to read project\n");
		goto end;
	}

	ret = of_property_read_u32(np, "hbp,panels", &panel_expected);
	if (ret < 0) {
		hbp_err("failed to read supported panels, set to 1\n");
	}

	hbp->dev_info.panels_expect = panel_expected;
	hbp_info("hbp expect panels = %d", hbp->dev_info.panels_expect);

	memcpy(&hbp->dev_info.project, hbp->prj, strlen(hbp->prj));

	ret = hbp_init_vm_mem(np, hbp);
	if (ret < 0) {
		hbp_err("failed to init vm memory %d\n", ret);
		return ret;
	}

end:
	return 0;
}

static int hbp_core_probe(struct platform_device *pdev)
{
	int ret = -EINVAL;
	struct hbp_core *hbp = NULL;

	hbp_info("enter.\n");

	split_touch_cmdline();

	hbp = (struct hbp_core *)kzalloc(sizeof(struct hbp_core), GFP_KERNEL);
	if (!hbp) {
		hbp_err("failed to alloc memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, hbp);
	hbp->dev = &pdev->dev;

	ret = hbp_main_dt(pdev->dev.of_node, hbp);
	if (ret < 0) {
		hbp_err("failed to parse device tree\n");
		goto exit;
	}

	hbp->ws = wakeup_source_register(hbp->dev, "hbp_ws");
	if (!hbp->ws) {
		hbp_err("failed to register wakeup source\n");
		goto exit;
	}

	hbp->state_st = STATE_WAKEUP;
	init_waitqueue_head(&hbp->state_event);
	hbp->state_ack = ACK_WAKEUP;
	init_waitqueue_head(&hbp->ack_event);

	mutex_init(&hbp->gesture_mtx);

	ret = core_register_dev(hbp);
	if (ret < 0) {
		hbp_err("failed to register hbp device\n");
		goto exit;
	}

	hbp_register_sysfs(hbp);

	g_hbp = hbp;

exit:
	hbp_info("exit %d.\n", ret);
	return ret;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void hbp_core_remove(struct platform_device *pdev)
#else
static int hbp_core_remove(struct platform_device *pdev)
#endif
{
	struct hbp_core *hbp = platform_get_drvdata(pdev);

	/*free device first*/

	device_destroy(hbp->cls, MKDEV(hbp->major, 0));
	unregister_chrdev(hbp->major, HBP_CORE);
	class_destroy(hbp->cls);

	kfree(hbp);

	hbp_info("exit.\n");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
#else
	return 0;
#endif
}

static void hbp_core_irq_wake(struct hbp_core *hbp, bool wake)
{
	struct hbp_device *hbp_dev = NULL;
	int i = 0;

	for (i = 0; i < MAX_DEVICES; i++) {
		hbp_dev = hbp->devices[i];
		if (hbp_dev) {
			hbp_set_irq_wake(hbp_dev, wake);
		}
	}
}

static int hbp_core_pm_suspend(struct device *dev)
{
	struct hbp_core *hbp = platform_get_drvdata(to_platform_device(dev));

	hbp_core_irq_wake(hbp, true);
	suspend_task = get_current();
	return 0;
}

static int hbp_core_pm_resume(struct device *dev)
{
	struct hbp_core *hbp = platform_get_drvdata(to_platform_device(dev));

	hbp_core_irq_wake(hbp, false);
	return 0;
}

static const struct dev_pm_ops hbp_core_pm_ops = {
	.suspend = hbp_core_pm_suspend,
	.resume = hbp_core_pm_resume,
};


static struct of_device_id hbp_core_of_match[] = {
	{ .compatible = "oplus,hbp_core" },
	{},
};

static struct platform_driver hbp_core_driver_platform = {
	.driver = {
		.name		= "hbp_core",
		.owner		= THIS_MODULE,
		.of_match_table = hbp_core_of_match,
		.pm = &hbp_core_pm_ops,
	},
	.probe	= hbp_core_probe,
	.remove = hbp_core_remove,
};

static int __init register_hbp_core_driver(void)
{
	hw_interface_init();
	return platform_driver_register(&hbp_core_driver_platform);
}

module_init(register_hbp_core_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OPLUS HBP");
MODULE_AUTHOR("OPLUS.");

