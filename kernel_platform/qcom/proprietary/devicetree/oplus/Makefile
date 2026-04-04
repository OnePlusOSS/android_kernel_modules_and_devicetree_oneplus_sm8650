PINEAPPLE_BASE_DTB += ../qcom/pineapple.dtb ../qcom/pineapple-v2.dtb ../qcom/cliffs.dtb ../qcom/cliffs7.dtb ../qcom/volcano.dtb ../qcom/volcano6.dtb ../qcom/volcano6i.dtb
PINEAPPLE_APQ_BASE_DTB += ../qcom/pineapplep.dtb ../qcom/pineapplep-v2.dtb

OPLUS_PINEAPPLE_BOARDS += waffle-22825-pineapple-overlay.dtbo\
                          waffle-22825-pineapple-overlay-EVB.dtbo\
                          waffle-22825-pineapple-overlay-EVT1.dtbo\
                          waffle-22877-pineapple-overlay.dtbo\
                          waffle-22877-pineapple-overlay-EVB.dtbo\
                          waffle-22877-pineapple-overlay-EVT1.dtbo\
                          corvette-23814-pineapple-overlay.dtbo\
                          caihong-23926-pineapple-overlay.dtbo\
                          caihong-23976-pineapple-overlay.dtbo\
                          caihong-23926-pineapple-overlay-T0.dtbo\
                          caihong-23976-pineapple-overlay-T0.dtbo\
                          audi-23803-cliffs-overlay.dtbo\
                          audi-23865-cliffs-overlay.dtbo\
                          avalon-24211-cliffs-overlay.dtbo\
                          giulia-23851-pineapple-overlay-T0.dtbo\
                          giulia-23851-pineapple-overlay.dtbo\
                          giulia-23867-pineapple-overlay-T0.dtbo\
                          giulia-23867-pineapple-overlay-T0-EU.dtbo\
                          giulia-23867-pineapple-overlay.dtbo\
                          lexus-24881-cliffs-overlay.dtbo\
                          lexus-24882-cliffs-overlay.dtbo\
                          lexus-24882-EU-cliffs-overlay.dtbo\
                          prado-25811-volcano-overlay.dtbo
dtb-y += $(OPLUS_PINEAPPLE_BOARDS) $(PINEAPPLE_BASE_DTB) $(PINEAPPLE_APQ_BASE_DTB)

always-y	:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
