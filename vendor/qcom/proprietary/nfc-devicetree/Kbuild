
ifeq ($(CONFIG_ARCH_PINEAPPLE),y)
dtbo-y += nxp/pineapple-nfc.dtbo \
          nxp/pineapple-nfc-atp.dtbo \
          nxp/pineapple-nfc-cdp.dtbo \
          nxp/pineapple-nfc-mtp.dtbo \
          nxp/pineapple-nfc-qrd.dtbo \
          nxp/pineapple-nfc-rcm.dtbo

dtbo-y += st/pineapple-nfc.dtbo \
          st/pineapple-nfc-mtp.dtbo \
          st/pineapple-nfc-cdp.dtbo
endif

ifeq ($(CONFIG_ARCH_BLAIR),y)
dtbo-y += nxp/blair-nfc.dtbo \
          nxp/blair-nfc-atp.dtbo \
          nxp/blair-nfc-cdp.dtbo \
          nxp/blair-nfc-mtp.dtbo \
          nxp/blair-nfc-qrd.dtbo

endif

ifeq ($(CONFIG_ARCH_HOLI),y)
dtbo-y += nxp/holi-nfc.dtbo \
          nxp/holi-pm6125-nfc.dtbo
endif

always-y	:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
