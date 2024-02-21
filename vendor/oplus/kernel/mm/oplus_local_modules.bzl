load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_bsp_memleak_detect_simple",
        srcs = native.glob([
            "**/*.h",
            "memleak_detect/slub_track_simple.c",
            "memleak_detect/vmalloc_track_simple.c",
        ]),
        includes = ["."],
        )

    define_oplus_ddk_module(
        name = "oplus_bsp_hybridswap_zram",
        conditional_srcs = {
            "CONFIG_CONT_PTE_HUGEPAGE": {
                True: ["hybridswap_zram/hybridswap/hybridswapd_chp.c"],
            }
        },

        srcs = native.glob([
            "**/*.h",
            "hybridswap_zram/zcomp.c",
            "hybridswap_zram/zram_drv.c",
            "hybridswap_zram/hybridswap/hybridmain.c",
            "hybridswap_zram/hybridswap/hybridswapd.c",
            "hybridswap_zram/hybridswap/hybridswap.c",
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/mm:oplus_bsp_zsmalloc",
        ],
        includes = ["."],
        local_defines = ["CONFIG_HYBRIDSWAP","CONFIG_HYBRIDSWAP_SWAPD","CONFIG_HYBRIDSWAP_CORE"],
        )

    define_oplus_ddk_module(
        name = "oplus_bsp_zsmalloc",
        conditional_srcs = {
            "CONFIG_CONT_PTE_HUGEPAGE": {
                True: ["thp_zsmalloc/thp_zsmalloc.c"],
            }
        },

        srcs = native.glob([
            "**/*.h",
            "thp_zsmalloc/zsmalloc.c",
        ]),
        includes = ["."],
        )

    define_oplus_ddk_module(
        name = "oplus_bsp_sigkill_diagnosis",
        srcs = native.glob([
            "sigkill_diagnosis/sigkill_diagnosis.c",
        ]),
        includes = ["."],
        )

    define_oplus_ddk_module(
        name = "oplus_bsp_zram_opt",
        srcs = native.glob([
            "**/*.h",
            "zram_opt/zram_opt.c",
            "kswapd_opt/kswapd_opt.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_DYNAMIC_TUNING_SWAPPINESS", "CONFIG_OPLUS_BALANCE_ANON_FILE_RECLAIM", "CONFIG_HYBRIDSWAP_SWAPD"],
        conditional_defines = {
            "qcom":["CONFIG_ALLOC_ADJUST_FLAGS"],
        },
        )

    define_oplus_ddk_module(
        name = "oplus_bsp_proactive_compact",
        srcs = native.glob([
            "**/*.h",
            "proactive_compact/proactive_compact.c",
        ]),
        includes = ["."],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_bsp_mm",
        module_list = [
            "oplus_bsp_memleak_detect_simple",
            "oplus_bsp_sigkill_diagnosis",
            "oplus_bsp_proactive_compact",
            "oplus_bsp_zram_opt",
            "oplus_bsp_hybridswap_zram",
            "oplus_bsp_zsmalloc",
        ],
    )

