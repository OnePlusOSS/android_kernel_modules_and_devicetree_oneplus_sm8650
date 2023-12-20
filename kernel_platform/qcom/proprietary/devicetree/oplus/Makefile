PINEAPPLE_BASE_DTB += ../qcom/pineapple.dtb ../qcom/pineapple-v2.dtb
PINEAPPLE_APQ_BASE_DTB += ../qcom/pineapplep.dtb ../qcom/pineapplep-v2.dtb

OPLUS_PINEAPPLE_BOARDS += waffle-22825-pineapple-overlay.dtbo\
                          waffle-22825-pineapple-overlay-EVB.dtbo\
                          waffle-22825-pineapple-overlay-EVT1.dtbo\
                          waffle-22877-pineapple-overlay.dtbo\
                          waffle-22877-pineapple-overlay-EVB.dtbo\
                          waffle-22877-pineapple-overlay-EVT1.dtbo\
                          pangu-22111-pineapple-overlay.dtbo\
                          enzo-23607-pineapple-overlay.dtbo

dtb-y += $(OPLUS_PINEAPPLE_BOARDS) $(PINEAPPLE_BASE_DTB) $(PINEAPPLE_APQ_BASE_DTB)

always-y	:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
