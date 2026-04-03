dtbo-$(CONFIG_ARCH_PINEAPPLE)	:= pineapple-camera.dtbo
#dtbo-$(CONFIG_ARCH_PINEAPPLE)	+= pineapple-camera-v2.dtbo \
#									pineapple-camera-sensor-cdp.dtbo \
#									pineapple-camera-sensor-mtp.dtbo \
#									pineapple-camera-sensor-hdk.dtbo \
#									pineapple-camera-sensor-qrd.dtbo \
#									pineapple-camera-sensor-aim500.dtbo
#OPLUS_DTS_OVERLAY start
dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/waffle-camera-overlay.dtbo \

dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/waffle-camera-overlay-evb.dtbo \

dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/caihong-camera-overlay.dtbo \

dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/corvette-camera-overlay.dtbo \

dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/giulia-camera-overlay.dtbo \

dtbo-$(CONFIG_ARCH_PINEAPPLE) += oplus/giuliaC-camera-overlay.dtbo \
#OPLUS_DTS_OVERLAY end

dtbo-$(CONFIG_ARCH_CLIFFS)    += cliffs-camera.dtbo
#OPLUS_DTS_OVERLAY start
dtbo-$(CONFIG_ARCH_CLIFFS)    += oplus/audi-camera-overlay.dtbo \

dtbo-$(CONFIG_ARCH_CLIFFS)    += oplus/avalon-camera-overlay.dtbo \

dtbo-$(CONFIG_ARCH_CLIFFS)    += oplus/lexus-camera-overlay.dtbo \

#OPLUS_DTS_OVERLAY end
#dtbo-$(CONFIG_ARCH_CLIFFS)	+= cliffs-camera-sensor-cdp.dtbo \
#								cliffs-camera-sensor-mtp.dtbo \
#								cliffs-camera-sensor-qrd.dtbo

dtbo-$(CONFIG_ARCH_VOLCANO)     += volcano-camera.dtbo
#dtbo-$(CONFIG_ARCH_VOLCANO)     += volcano-camera-qrd.dtbo
#dtbo-$(CONFIG_ARCH_VOLCANO)     += volcano-camera-sensor-mtp.dtbo
#dtbo-$(CONFIG_ARCH_VOLCANO)     += volcano-camera-sensor-idp.dtbo
#dtbo-$(CONFIG_ARCH_VOLCANO)     += volcano-camera-sensor-qrd.dtbo

#OPLUS_DTS_OVERLAY start
dtbo-$(CONFIG_ARCH_VOLCANO)    += oplus/prado-camera-overlay.dtbo
#OPLUS_DTS_OVERLAY end
