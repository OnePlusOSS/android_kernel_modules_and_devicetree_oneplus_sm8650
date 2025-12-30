# SPDX-License-Identifier: GPL-2.0-only

KCOV_INSTRUMENT := n
KCSAN_SANITIZE := n
LINUXINCLUDE += -I$(srctree)/
flags-y += -I$(srctree)/block
obj-$(CONFIG_OPLUS_RESCTRL) += oplus_resctrl.o

oplus_resctrl-y := resctrl.o
oplus_resctrl-$(CONFIG_OPLUS_RESCTRL) += iocost.o

