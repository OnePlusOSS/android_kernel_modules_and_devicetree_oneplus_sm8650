subdir-ccflags-y += -Wall -Werror -DCONFIG_RT_REGMAP
ccflags-y += -I$(srctree)/../kernel_device_modules-6.6/drivers/power/supply

tcpc_class-$(CONFIG_DUAL_ROLE_USB_INTF)	+= tcpci_dual_role.o

ifdef CONFIG_OPLUS_CHARGER_MTK
subdir-ccflags-y += -I$(srctree)/drivers/power/supply/
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/rt-regmap/
subdir-ccflags-y += -I$(srctree)/drivers/mfd/

obj-$(CONFIG_TCPC_RT1711H)		+= tcpc_rt1711h.o
obj-$(CONFIG_TCPC_SGM7220)		+= tcpc_sgm7220.o
obj-$(CONFIG_TCPC_AW35616)		+= tcpc_aw35616.o
obj-$(CONFIG_TCPC_UPM7610)		+= tcpc_upm7610.o
obj-$(CONFIG_TCPC_CPS8851)		+= tcpc_cps8851.o

tcpc_class-objs := tcpci_core.o tcpci_typec.o tcpci_timer.o tcpm.o tcpci.o tcpci_alert.o

ifdef CONFIG_USB_POWER_DELIVERY
tcpc_class-objs += tcpci_event.o \
		pd_core.o pd_policy_engine.o pd_process_evt.o \
		pd_dpm_core.o pd_dpm_uvdm.o pd_dpm_alt_mode_dp.o pd_dpm_pdo_select.o\
		pd_dpm_reaction.o \
		pd_process_evt_snk.o pd_process_evt_src.o pd_process_evt_vdm.o \
		pd_process_evt_drs.o pd_process_evt_prs.o pd_process_evt_vcs.o \
		pd_process_evt_dbg.o pd_process_evt_tcp.o pd_process_evt_com.o \
		pd_policy_engine_src.o pd_policy_engine_snk.o pd_policy_engine_ufp.o pd_policy_engine_vcs.o \
		pd_policy_engine_dfp.o pd_policy_engine_dr.o pd_policy_engine_drs.o pd_policy_engine_prs.o \
		pd_policy_engine_dbg.o pd_policy_engine_com.o pd_dpm_alt_mode_dc.o
endif
obj-$(CONFIG_TCPC_CLASS) += tcpc_class.o tcpci_late_sync.o pd_dbg_info.o rt-regmap.o rt_pd_manager.o
else
ifeq ($(OPLUS_VND_ENV_SEPARATE_CHG_MUSE),yes)
ccflags-y += -DOPLUS_CHG_SEPARATE_MUSE_TCPC
obj-$(CONFIG_TCPC_ET7303)		+= tcpc_et7303.o
else
obj-$(CONFIG_TCPC_RT1711H)		+= tcpc_rt1711h.o
endif

tcpc_class-y += tcpci_core.o tcpci_typec.o tcpci_timer.o tcpm.o tcpci.o tcpci_alert.o
ifdef CONFIG_USB_POWER_DELIVERY
tcpc_class-y += tcpci_event.o pd_core.o pd_policy_engine.o pd_process_evt.o \
		pd_dpm_core.o pd_dpm_uvdm.o pd_dpm_alt_mode_dp.o pd_dpm_pdo_select.o\
		pd_dpm_reaction.o pd_process_evt_snk.o pd_process_evt_src.o pd_process_evt_vdm.o \
		pd_process_evt_drs.o pd_process_evt_prs.o pd_process_evt_vcs.o \
		pd_process_evt_dbg.o pd_process_evt_tcp.o pd_process_evt_com.o \
		pd_policy_engine_src.o pd_policy_engine_snk.o pd_policy_engine_ufp.o pd_policy_engine_vcs.o \
		pd_policy_engine_dfp.o pd_policy_engine_dr.o pd_policy_engine_drs.o pd_policy_engine_prs.o \
		pd_policy_engine_dbg.o pd_policy_engine_com.o pd_dpm_alt_mode_dc.o
endif
obj-$(CONFIG_TCPC_CLASS)		+= tcpc_class.o tcpci_late_sync.o pd_dbg_info.o rt-regmap.o
endif
