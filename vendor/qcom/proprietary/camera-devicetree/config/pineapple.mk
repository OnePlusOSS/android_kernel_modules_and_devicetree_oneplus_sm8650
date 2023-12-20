dtbo-$(CONFIG_ARCH_PINEAPPLE)	:= pineapple-camera.dtbo
#dtbo-$(CONFIG_ARCH_PINEAPPLE)	+= pineapple-camera-sensor-cdp.dtbo \
#									pineapple-camera-sensor-mtp.dtbo \
#									pineapple-camera-sensor-qrd.dtbo
#OPLUS_DTS_OVERLAY start
dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/waffle-camera-overlay.dtbo \

dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/waffle-camera-overlay-evb.dtbo \

dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/pangu-camera-overlay.dtbo \

dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/enzo-camera-overlay.dtbo \

dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/pangu-camera-overlay-evb.dtbo \
#OPLUS_DTS_OVERLAY end
