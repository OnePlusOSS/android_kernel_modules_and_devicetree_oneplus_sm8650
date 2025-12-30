load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module", "oplus_ddk_get_kernel_version", "oplus_ddk_get_target", "oplus_ddk_get_variant", "bazel_support_platform")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def version_compare(v1, v2):
    v1_parts = [int(x) for x in v1.split(".")]
    v2_parts = [int(x) for x in v2.split(".")]
    return v1_parts >= v2_parts

def define_oplus_local_modules():
    target = oplus_ddk_get_target()
    variant  = oplus_ddk_get_variant()
    kernel_build_variant = "{}_{}".format(target, variant)
    kernel_version = oplus_ddk_get_kernel_version()
    bazel_support_target = oplus_ddk_get_target()


    if bazel_support_platform == "qcom" :
        panel_event_notifier_ko_deps = []
        tp_others_ko_deps = []
        ko_deps = [
                "//vendor/oplus/kernel/tp/hbp/hbp:oplus_hbp_core",
        ]
        copts = []
    else :
        if version_compare(kernel_version, "6.12") :
            panel_event_notifier_ko_deps = [
                "//kernel_device_modules-{}/drivers/gpu/drm/mediatek/mediatek_v2:mtk_disp_notify".format(kernel_version),
                "//kernel_device_modules-{}/drivers/gpu/drm/mediatek/mediatek_v2:mtk_panel_ext".format(kernel_version),
            ]
            tp_others_ko_deps = [
                "//kernel_device_modules-{}/drivers/soc/oplus/boot:oplus_bsp_boot_projectinfo".format(kernel_version),
                "//kernel_device_modules-{}/drivers/misc/mediatek/boot_common:mtk_boot_common".format(kernel_version),
                "//kernel_device_modules-{}/drivers/soc/oplus/boot:oplusboot".format(kernel_version),
                "//kernel_device_modules-{}/drivers/soc/oplus/device_info:device_info".format(kernel_version),
                "//kernel_device_modules-{}/drivers/base/touchpanel_notify:oplus_bsp_tp_notify".format(kernel_version),
                "//kernel_device_modules-{}/drivers/base/kernelFwUpdate:oplus_bsp_fw_update".format(kernel_version),
            ]
            ko_deps = [
                "//vendor/oplus/kernel/tp/hbp/hbp:oplus_hbp_core",
            ]
            copts = [
                "-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/",
                "-I$(DEVICE_MODULES_PATH)/drivers/gpu/drm/mediatek/mediatek_v2/",
            ]
        else :
            panel_event_notifier_ko_deps = []
            tp_others_ko_deps = []
            ko_deps = [
                "//vendor/oplus/kernel/tp/hbp/hbp:oplus_hbp_core",
            ]
            copts = []

    define_oplus_ddk_module(
        name = "oplus_hbp_core",
        srcs = native.glob([
            "**/*.h",
            "hbp_core.c",
            "hbp_notify.c",
            "hbp_tui.c",
            "hbp_frame.c",
            "utils/debug.c",
            "utils/platform.c",
            "chips/touch_custom.c",
            "hbp_device.c",
            "hbp_power.c",
            "hbp_spi.c",
            "hbp_sysfs.c",
            "hbp_exception.c",
            "hbp_healthinfo.c"
        ]),
        includes = ["."],
        ko_deps = panel_event_notifier_ko_deps + tp_others_ko_deps,
        copts = copts,
        local_defines = [
                 "BUILD_BY_BAZEL",
        ],
        conditional_defines = {
            "mtk":  ["CONFIG_TOUCHPANEL_MTK_PLATFORM", "CONFIG_DRM_MEDIATEK"],
            "qcom":  ["CONFIG_QCOM_PANEL_EVENT_NOTIFIER"],
        },
    )

    define_oplus_ddk_module(
        name = "oplus_ft3683g",
        srcs = native.glob([
            "**/*.h",
            "chips/focal/ft3683g/fhp_core.c",
        ]),
        includes = ["."],
        ko_deps = [
            "//vendor/oplus/kernel/tp/hbp/hbp:oplus_hbp_core",
        ],
        local_defines = [
                 "BUILD_BY_BAZEL",
        ],
        conditional_defines = {
            "mtk":  ["CONFIG_TOUCHPANEL_MTK_PLATFORM", "CONFIG_DRM_MEDIATEK"],
            "qcom":  ["CONFIG_QCOM_PANEL_EVENT_NOTIFIER"],
        },

    )

    define_oplus_ddk_module(
        name = "oplus_bsp_tp_hbp_syna_s3910",
        srcs = native.glob([
            "**/*.h",
            "chips/synaptics/s3910/syna_tcm2.c",
            "chips/synaptics/s3910/syna_tcm2_sysfs.c",
            "chips/synaptics/s3910/tcm/synaptics_touchcom_core_v1.c",
            "chips/synaptics/s3910/tcm/synaptics_touchcom_core_v2.c",
            "chips/synaptics/s3910/tcm/synaptics_touchcom_func_base.c",
            "chips/synaptics/s3910/tcm/synaptics_touchcom_func_touch.c",
            "chips/synaptics/s3910/tcm/synaptics_touchcom_func_reflash.c",
            "chips/synaptics/s3910/tcm/synaptics_touchcom_func_romboot.c",
        ]),
        includes = ["."],
        ko_deps = ko_deps + panel_event_notifier_ko_deps + tp_others_ko_deps,
        copts = copts,
        local_defines = [
            "BUILD_BY_BAZEL",
        ],
        conditional_defines = {
            "mtk":  ["CONFIG_TOUCHPANEL_MTK_PLATFORM"],
            "qcom":  ["CONFIG_QCOM_PANEL_EVENT_NOTIFIER"],
        },
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_tp_hbp_goodix_gt99x6",
        srcs = native.glob([
            "**/*.h",
            "chips/goodix/gt99x6/gt99x6_core.c",
            "chips/goodix/gt99x6/gtx8_tools.c",
        ]),
        includes = ["."],
        ko_deps = [
            "//vendor/oplus/kernel/tp/hbp/hbp:oplus_hbp_core",
        ],
        local_defines = [
            "BUILD_BY_BAZEL",
        ],
        conditional_defines = {
            "mtk":  ["CONFIG_TOUCHPANEL_MTK_PLATFORM", "CONFIG_DRM_MEDIATEK"],
            "qcom":  ["CONFIG_QCOM_PANEL_EVENT_NOTIFIER"],
        },
    )

    ddk_headers(
        name = "config_headers",
        hdrs  = native.glob([
            "**/*.h",
        ]),
        includes = ["."],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_hbp_core",
        module_list = [
            "oplus_hbp_core",
            "oplus_ft3683g",
            "oplus_bsp_tp_hbp_syna_s3910",
            "oplus_bsp_tp_hbp_goodix_gt99x6"
        ],
    )
