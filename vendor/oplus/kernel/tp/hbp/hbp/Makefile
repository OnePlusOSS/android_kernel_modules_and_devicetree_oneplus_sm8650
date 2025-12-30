oplus_hbp_core-y += hbp_core.o
oplus_hbp_core-y += hbp_notify.o
oplus_hbp_core-y += hbp_tui.o
oplus_hbp_core-y += hbp_frame.o
oplus_hbp_core-y += utils/debug.o
oplus_hbp_core-y += utils/platform.o
oplus_hbp_core-y += chips/touch_custom.o
oplus_hbp_core-y += hbp_device.o
oplus_hbp_core-y += hbp_power.o
oplus_hbp_core-y += hbp_spi.o
oplus_hbp_core-y += hbp_sysfs.o
oplus_hbp_core-y += hbp_exception.o
oplus_hbp_core-y += hbp_healthinfo.o

CFLAGS_hbp_core.o := -I$(src)

obj-$(CONFIG_TOUCHSCREEN_HBP_CORE) += oplus_hbp_core.o

obj-y += chips/
