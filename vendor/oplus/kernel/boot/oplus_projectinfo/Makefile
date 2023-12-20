# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.
#ifdef OPLUS_BUG_STABILITY
ifeq ($(CONFIG_KASAN),y)
subdir-ccflags-y += -Wno-format
endif
#endif /*VENDOR_EDIT*/
ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
oplus_bsp_boot_projectinfo-objs 	+= qcom/oplus_project.o
else
oplus_bsp_boot_projectinfo-objs 	+= mtk/oplus_project.o

endif
obj-$(CONFIG_OPLUS_FEATURE_OPROJECT) += oplus_bsp_boot_projectinfo.o
ifeq ($(CONFIG_MTK_SECURITY_SW_SUPPORT), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/masp/asfv2/asf_inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/masp/asfv2/asf_export_inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/masp/asfv2/module
endif
