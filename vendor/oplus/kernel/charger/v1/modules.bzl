load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/kleaf:hermetic_tools.bzl", "hermetic_genrule")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module",
    "oplus_ddk_get_kernel_version",
    "bazel_support_platform")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

load(":kleaf-scripts/prebuild.bzl", "oplus_chg_ic_prebuild")
load(":kleaf-scripts/targets.bzl", "oplus_modules_get_target_variant")
load(":kleaf-scripts/filter_target.bzl", "filter_deps_map")

conditional_ko_deps = {
    "CONFIG_OPLUS_ADSP_CHARGER": {
        True: [
            "//soc-repo:{target_variant}/drivers/soc/qcom/panel_event_notifier",
            "//soc-repo:{target_variant}/drivers/soc/qcom/qti_pmic_glink",
            "//soc-repo:{target_variant}/drivers/soc/qcom/pdr_interface",
            "//soc-repo:{target_variant}/drivers/soc/qcom/qmi_helpers",
            "//soc-repo:{target_variant}/drivers/remoteproc/rproc_qcom_common",
            "//soc-repo:{target_variant}/drivers/rpmsg/qcom_smd",
            "//soc-repo:{target_variant}/drivers/rpmsg/qcom_glink_smem",
            "//soc-repo:{target_variant}/drivers/rpmsg/qcom_glink",
            "//soc-repo:{target_variant}/kernel/trace/qcom_ipc_logging",
            "//soc-repo:{target_variant}/drivers/soc/qcom/minidump",
            "//soc-repo:{target_variant}/drivers/soc/qcom/smem",
            "//soc-repo:{target_variant}/drivers/soc/qcom/debug_symbol",
            "//soc-repo:{target_variant}/drivers/dma-buf/heaps/qcom_dma_heaps",
            "//soc-repo:{target_variant}/drivers/iommu/msm_dma_iommu_mapping",
            "//soc-repo:{target_variant}/drivers/soc/qcom/mem_buf/mem_buf_dev",
            "//soc-repo:{target_variant}/drivers/soc/qcom/secure_buffer",
            "//soc-repo:{target_variant}/drivers/firmware/qcom/qcom-scm",
            "//soc-repo:{target_variant}/drivers/virt/gunyah/gh_rm_drv",
            "//soc-repo:{target_variant}/drivers/virt/gunyah/gh_msgq",
            "//soc-repo:{target_variant}/drivers/virt/gunyah/gh_dbl",
            "//soc-repo:{target_variant}/arch/arm64/gunyah/gh_arm_drv",
        ],
    },
    "CONFIG_OPLUS_CHARGER_MTK": {
        True: [
                "//kernel_device_modules-{}/drivers/misc/mediatek/typec/tcpc:tcpc_class".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/gpu/drm/mediatek/mediatek_v2:mediatek-drm".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:charger_class".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:adapter_class".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_charger_algorithm_class".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6357_battery".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6375-battery".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6375-charger".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6379-battery".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6379-chg".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_2p_charger".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_battery_manager".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_chg_type_det".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_hvbpc".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pd_adapter".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pd_charging".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep20".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep40".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep45".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep50".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep50p".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:rt9490-charger".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:rt9758-charger".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:rt9759".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/usb/mtu3:mtu3".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/gpu/drm/mediatek/mediatek_v2:mtk_disp_notify".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/base/magtransfer:oplus_magcvr_notify".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/soc/oplus/device_info:device_info".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/soc/oplus/boot:oplus_bsp_boot_projectinfo".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/misc/mediatek/boot_common:mtk_boot_common".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/misc/mediatek/usb/usb20:musb_hdrc".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/base/kernelFwUpdate:oplus_bsp_fw_update".format(oplus_ddk_get_kernel_version()),
        ],
    },
    "CONFIG_OPLUS_CHARGER_MTK6789S": {
        True: [
            "//kernel_device_modules-{}/drivers/misc/mediatek/typec/tcpc:tcpc_sgm7220".format(oplus_ddk_get_kernel_version()),
            "//kernel_device_modules-{}/drivers/misc/mediatek/typec/tcpc:tcpc_wusb3801x".format(oplus_ddk_get_kernel_version()),
        ],
    },
    "CONFIG_OPLUS_CHG_TEST_KIT": {
        True: [
            "{target_variant}_test-kit"
        ],
    },
    "CONFIG_OPLUS_DYNAMIC_CONFIG": {
        True: [
            "{target_variant}_oplus_cfg"
        ],
    },
    "CONFIG_OPLUS_UFCS_CLASS": {
        True: [
            "{target_variant}_ufcs_class"
        ],
    },
    "CONFIG_DISABLE_OPLUS_FUNCTION": {
        False: [
            "//vendor/oplus/kernel/device_info/device_info/bazel:device_info",
            "//vendor/oplus/kernel/boot:oplus_bsp_bootmode",
            "//vendor/oplus/kernel/boot:oplus_bsp_boot_projectinfo",
            "//vendor/oplus/kernel/touchpanel/kernelFwUpdate/bazel:oplus_bsp_fw_update",
        ],
    },
    "CONFIG_OPLUS_FEATURE_FEEDBACK": {
        True: [
             "//vendor/oplus/kernel/dft/bazel:oplus_bsp_dft_kernel_fb",
        ],
    },
    "CONFIG_OPLUS_FEATURE_OLC": {
        True: [
            "//vendor/oplus/kernel/dft/bazel:oplus_bsp_dft_olc",
        ],
    },
}

conditional_hdr_deps = {
    "CONFIG_OPLUS_CHG_TEST_KIT": {
        True: [
            ":test_kit_headers"
        ],
    },
    "CONFIG_OPLUS_DYNAMIC_CONFIG": {
        True: [
            ":oplus_cfg_headers"
        ],
    },
    "CONFIG_OPLUS_UFCS_CLASS": {
        True: [
            ":ufcs_class_headers"
        ],
    },
    "CONFIG_DISABLE_OPLUS_FUNCTION": {
        False: [
            "//vendor/oplus/kernel/touchpanel/kernelFwUpdate/bazel:oplus_bsp_fw_update_headers",
        ],
    },
}

conditional_copt_deps = {
    "CONFIG_OPLUS_CHARGER_MTK": {
        True: [
                "-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/typec/tcpc/inc",
                "-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/mt-plat",
                "-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include",
                "-I$(DEVICE_MODULES_PATH)/drivers/power/supply",
                "-I$(DEVICE_MODULES_PATH)/drivers/gpu/drm/mediatek/mediatek_v2",
                "-I$(DEVICE_MODULES_PATH)/drivers/base/kernelFwUpdate"
        ],
    },
}

def define_oplus_chg_module():
    module_list = []

    oplus_chg_ic_prebuild("oplus_chg")

    ddk_headers(
        name = "oplus_chg_headers",
        hdrs  = native.glob([
            "v1/**/*.h",
        ]),
        includes = [
            "v1",
        ]
    )

    target = oplus_modules_get_target_variant()
    ko_deps = filter_deps_map(target, conditional_ko_deps)
    hdr_deps = filter_deps_map(target, conditional_hdr_deps)
    copt_deps = filter_deps_map(target, conditional_copt_deps)

    define_oplus_ddk_module(
        name = "{}_oplus_chg".format(target),
        out = "oplus_chg.ko",
        srcs = native.glob([
            "v1/**/*.h",
            "v1/wireless_ic/oplus_chargepump.h",
            "v1/oplus_adapter.c",
            "v1/oplus_charger.c",
            "v1/oplus_debug_info.c",
            "v1/oplus_pps_ops_manager.c",
            "v1/oplus_chg_exception.c",
            "v1/oplus_chg_track.c",
            "v1/oplus_ufcs.c",
            "v1/oplus_gauge.c",
            "v1/oplus_quirks.c",
            "v1/oplus_chg_voter.c",
            "v1/oplus_vooc.c",
            "v1/oplus_chg_comm.c",
            "v1/oplus_pps.c",
            "v1/oplus_region_check.c",
            "v1/oplus_battery_log.c",
            "v1/oplus_chg_ops_manager.c",
            "v1/oplus_configfs.c",
            "v1/oplus_wireless.c",
            "v1/oplus_chg_core.c",
            "v1/oplus_short.c",
            "v1/adapter_ic/oplus_stm.c",
            "v1/oplus_chg_audio_switch.c",
            "v1/charger_ic/oplus_short_ic.c",
            "v1/charger_ic/oplus_switching.c",
            "v1/gauge_ic/oplus_sh366002.c",
            "v1/gauge_ic/oplus_nfg1000a.c",
            "v1/gauge_ic/oplus_bq27541.c",
            "v1/gauge_ic/oplus_bqfs.c",
            "v1/vooc_ic/oplus_vooc_fw.c",
            "v1/wireless_ic/oplus_nu1619.c",
            "v1/wireless_ic/oplus_chargepump.c",
            "v1/gauge_ic/oplus_optiga/ECC/Optiga_Ecc.c",
            "v1/gauge_ic/oplus_optiga/ECC/Optiga_Math.c",
            "v1/gauge_ic/oplus_optiga/Platform/board.c",
            "v1/gauge_ic/oplus_optiga/SWI/Optiga_Auth.c",
            "v1/gauge_ic/oplus_optiga/SWI/Optiga_Nvm.c",
            "v1/gauge_ic/oplus_optiga/SWI/Optiga_Swi.c",
            "v1/gauge_ic/oplus_optiga/oplus_optiga.c",
            "v1/op_wlchg_v2/oplus_chg_cfg.c",
            "v1/ufcs/oplus_ufcs_protocol.c",
            "v1/ufcs/ufcs_ic/oplus_sc8547a.c",
            "v1/voocphy/oplus_adsp_voocphy.c",
            "v1/voocphy/oplus_cp_intf.c",
            "v1/voocphy/oplus_sc8547.c",
            "v1/voocphy/oplus_hl7138.c",
            "v1/voocphy/oplus_hl7138_slave.c",
        ]),
        conditional_srcs = {
            "CONFIG_OPLUS_SM6375R_CHARGER": {
                True: [
                    "v1/gauge_ic/oplus_sm5602.c",
                    "v1/wireless_ic/oplus_p922x.c",
                    "v1/wireless_ic/oplus_ra9530.c"
                ],
            },
            "CONFIG_OPLUS_CHARGER_MAXIM": {
                True: [
                    "v1/gauge_ic/oplus_maxim/oplus_ds28e30/1wire_protocol.c",
                    "v1/gauge_ic/oplus_maxim/oplus_ds28e30/bignum.c",
                    "v1/gauge_ic/oplus_maxim/oplus_ds28e30/deep_cover_coproc_sw.c",
                    "v1/gauge_ic/oplus_maxim/oplus_ds28e30/ds28e30.c",
                    "v1/gauge_ic/oplus_maxim/oplus_ds28e30/ecc_generate_key.c",
                    "v1/gauge_ic/oplus_maxim/oplus_ds28e30/ecdsa_generic_api.c",
                    "v1/gauge_ic/oplus_maxim/oplus_ds28e30/ecdsa_high.c",
                    "v1/gauge_ic/oplus_maxim/oplus_ds28e30/sha256_stone.c",
                    "v1/gauge_ic/oplus_maxim/oplus_ds28e30/ucl_rng.c",
                    "v1/gauge_ic/oplus_maxim/oplus_ds28e30/ucl_sha256.c",
                    "v1/gauge_ic/oplus_maxim/oplus_maxim.c"
                ],
            },
            "CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER": {
                True: [
                    "v1/op_wlchg_v2/oplus_chg_wls_cfg.c",
                ],
            },
            "CONFIG_OPLUS_CHARGER_MTK6789S": {
                True: [
                    "v1/charger_ic/oplus_usbtemp.c",
                    "v1/op_wlchg_v2/hal/oplus_chg_ic.c",
                    "v1/op_wlchg_v2/hal/wls_chg_fast.c",
                    "v1/op_wlchg_v2/hal/wls_chg_normal.c",
                    "v1/op_wlchg_v2/hal/wls_chg_rx.c",
                    "v1/op_wlchg_v2/oplus_chg_strategy.c",
                    "v1/op_wlchg_v2/oplus_chg_wls.c",
                    "v1/voocphy/oplus_voocphy.c",
                    "v1/charger_ic/oplus_battery_mtk6789S.c",
                    "v1/charger_ic/oplus_sc6607_charger.c",
                    "v1/charger_ic/oplus_bq2589x_gki.c",
                    "v1/charger_ic/oplus_sgm41512.c",
                    "v1/charger_ic/oplus_sgm41542.c"
                ],
            },
            "CONFIG_OPLUS_CHG_BOB_IC": {
                True: [
                    "v1/bob_ic/oplus_tps6128xd.c",
                ],
            },
        },
        includes = [
            "v1",
            "v1/adapter_ic",
            "v1/wireless_ic",
            "v1/chargepump_ic",
            "v1/op_wlchg_v2",
            "v1/ufcs",
            "v1/ufcs/ufcs_ic",
            "v1/voocphy",
            "v1/gauge_ic"
        ],
        ko_deps = ko_deps,
        hdrs = hdr_deps,
        local_defines = [
            "OPLUS_CHG_KO_BUILD",
            "OPLUS_FEATURE_CHG_BASIC"
        ],
        conditional_defines = {
        },
        copts = [
            "-Werror=parentheses",
            "-Werror=implicit-fallthrough",
            "-Werror=format"
        ] + copt_deps,
        kconfig = ":kconfig.oplus_chg.generated",
        defconfig = ":oplus_chg_{}_defconfig".format(target),
    )

    module_list.extend(filter_deps_map(target, {
         "CONFIG_OPLUS_CHG": "{}_oplus_chg".format(target)
    }))

    return module_list
