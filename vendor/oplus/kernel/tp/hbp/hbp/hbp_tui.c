#include <linux/delay.h>
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
#include <linux/vmalloc.h>
#include <uapi/linux/sched/types.h>

#include "utils/debug.h"
#include "hbp_tui.h"
#include "hbp_core.h"

#if IS_ENABLED(CONFIG_TOUCHPANEL_TRUSTED_TOUCH)
#include <linux/pinctrl/qcom-pinctrl.h>

int hbp_init_vm_mem_each(struct hbp_vm_mem *mem, struct device_node *tui_np, struct hbp_core *hbp)
{
	int ret = 0;
	int i = 0, io_count = 0, gpio_count = 0;
	int gpio;
	struct resource res;

	/*init vm memory info*/
	ret = of_property_read_string(tui_np, "vm,env-type", &mem->env_type);
	if (ret < 0) {
		return ret;
	}
	ret = of_property_read_u32_array(tui_np, "vm,reset-reg", (int32_t *)&mem->reset_mem, 3);
	if (ret < 0) {
		return ret;
	}
	ret = of_property_read_u32_array(tui_np, "vm,intr-reg", (int32_t *)&mem->intr_mem, 3);
	if (ret < 0) {
		return ret;
	}

	io_count = of_property_count_u32_elems(tui_np, "vm,io-bases");
	gpio_count = of_gpio_named_count(tui_np, "vm,gpio-list");

	mem->iomem_size = io_count / 2 + gpio_count;
	mem->iomem = kcalloc(mem->iomem_size, sizeof(struct vm_iomem), GFP_KERNEL);
	if (!mem->iomem) {
		return -ENOMEM;
	}
	for (i = 0; i < gpio_count; i++) {
		gpio = of_get_named_gpio(tui_np, "vm,gpio-list", i);
		if (gpio < 0 ||
			!gpio_is_valid(gpio) ||
			!msm_gpio_get_pin_address(gpio, &res)) {
			hbp_err("failed to get valid gpio or res\n");
			kfree(mem->iomem);
			return -EINVAL;
		}
		mem->iomem[i].base = res.start;
		mem->iomem[i].size = resource_size(&res);
	}

	ret = of_property_read_u32_array(tui_np, "vm,io-bases",
						(uint32_t *)&mem->iomem[i],
						io_count);
	if (ret < 0) {
		hbp_err("failed to read io bases\n");
		kfree(mem->iomem);
		return -EINVAL;
	}

	return 0;
}

int hbp_init_vm_mem(struct device_node *np, struct hbp_core *hbp)
{
	int ret = 0;
	struct device_node *tui_np;
	struct hbp_vm_mem *mem;
	int i = 0;

	/*init vm memory info*/
	for_each_compatible_node(tui_np, NULL, "tui,environment") {
		mem = (struct hbp_vm_mem *)kzalloc(sizeof(*mem), GFP_KERNEL);
		if (!mem) {
			hbp_err("failed to malloc memory\n");
			return -ENOMEM;
		}

		ret = hbp_init_vm_mem_each(mem, tui_np, hbp);
		if (ret < 0) {
			hbp_err("failed to init vm memory %d\n", i);
			kfree(mem);
			continue;
		}

		if (i < 2 * MAX_DEVICES) {
			hbp->tui_mem[i++] = mem;
		}
	}

	return 0;
}


static enum hbp_env check_vm_env(const char *vm_env, int id)
{
	int i = 0;
	char *supported_env[] = {"primary-tvm",
				 "primary-pvm",
				 "secondary-pvm",
				 "secondary-tvm",
				 NULL
				};
	if (id == 0) {
		if (strstr(vm_env, "primary")) {
			goto env_match;
		}
		goto end;
	} else {
		if (strstr(vm_env, "secondary")) {
			goto env_match;
		}
		goto end;
	}

env_match:
	for (i = 0; supported_env[i]; i++) {
		if (!strcmp(supported_env[i], vm_env)) {
			if (strstr(vm_env, "pvm")) {
				return ENV_PVM;
			} else if (strstr(vm_env, "tvm")) {
				return ENV_TVM;
			} else {
				return ENV_NORMAL;
			}
		}
	}

end:
	hbp_err("check vm env failed %s: %d\n", vm_env, id);
	return ENV_INVAL;
}

bool hbp_tui_mem_map(struct hbp_core *hbp, struct hbp_device *hbp_dev, const char *vm_env)
{
	int i = 0;

	hbp_debug("vm env:%s\n", vm_env);

	hbp_dev->vm_info = (struct hbp_vm_info *)kzalloc(sizeof(struct hbp_vm_info), GFP_KERNEL);
	if (!hbp_dev->vm_info) {
		hbp_err("failed to malloc memory\n");
		return false;
	}

	for (i = 0; i < 2*MAX_DEVICES; i++) {
		if (hbp->tui_mem[i] && !strcmp(hbp->tui_mem[i]->env_type, vm_env)) {
			hbp_dev->vm_info->mem = hbp->tui_mem[i];
			hbp_dev->env = check_vm_env(vm_env, hbp_dev->id);
			if (hbp_dev->env == ENV_INVAL) {
				hbp_dev->env = ENV_NORMAL;
				goto err_exit;
			}
			return true;
		}
	}

err_exit:
	kfree(hbp_dev->vm_info);
	hbp_dev->vm_info = NULL;
	return false;
}

#else
bool hbp_tui_mem_map(struct hbp_core *hbp, struct hbp_device *hbp_dev, const char *vm_env)
{
	return false;
}
int hbp_init_vm_mem(struct device_node *np, struct hbp_core *hbp)
{
	return 0;
}
#endif

