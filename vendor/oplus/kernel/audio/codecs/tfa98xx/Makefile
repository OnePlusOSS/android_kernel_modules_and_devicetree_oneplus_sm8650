# make sure that the environment variables ARCH and CROSS_COMPILE
# are set for your architecture and cross compiler
#
# e.g. for Beaglebone black:
# export ARCH := arm
# export CROSS_COMPILE := arm-linux-gnueabihf-
#
# e.g. Samsung Galaxy S6:
# export ARCH := arm64
# export CROSS_COMPILE := aarch64-linux-android-

#MODULE_NAME = snd-soc-tfa98xx

# additional debugging
#TFA_DEBUG=1

ifeq ($(DEVICE_MODULES_PATH),)
    DEVICE_MODULES_PATH = $(srctree)
else
endif

subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/sound/soc/mediatek/common
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/sound/soc/mediatek/audio_dsp
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/sound/soc/mediatek/common
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/audio_ipi/include
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/base

# debugging support (also enables trace_printk)
ccflags-y += -I$(DEVICE_MODULES_PATH)/sound/soc/codecs/tfa98xx/inc
ccflags-y += -DDEBUG -DTFA_NON_DSP_SOLUTION
ccflags-y += -Werror
ccflags-y += -DOPLUS_ARCH_EXTENDS -DOPLUS_ARCH_EXTENDS
ccflags-y += $(call cc-option,-Wno-date-time,)

snd-soc-tfa98xx-objs += tfa98xx.o
snd-soc-tfa98xx-objs += tfa_container.o
snd-soc-tfa98xx-objs += tfa_dsp.o
snd-soc-tfa98xx-objs += tfa_init.o

ifdef CONFIG_OPLUS_FEATURE_MM_FEEDBACK
snd-soc-tfa98xx-objs += oplus_tfa98xx_feedback.o
endif

ifdef TFA_DEBUG
ccflags-y += -DTFA_DEBUG -DDEBUG
snd-soc-tfa98xx-objs += tfa_debug.o
endif

obj-$(CONFIG_SND_SOC_TFA9873) += snd-soc-tfa98xx.o
obj-$(CONFIG_SND_SOC_TFA9874) += snd-soc-tfa98xx.o

