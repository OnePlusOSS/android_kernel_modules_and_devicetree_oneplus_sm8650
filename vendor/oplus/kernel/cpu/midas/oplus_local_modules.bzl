load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():


    define_oplus_ddk_module(
        name = "oplus_bsp_midas",
        srcs = native.glob([
            "**/*.h",
            "v1_gki/binder_stats_dev.c",
            "v1_gki/dispcap_dev.c",
            "v1_gki/midas_dev.c",
            "v1_gki/midas_ioctl.c",
            "v1_gki/midas_module.c",
            "v1_gki/vpu_pw_off_latency_proc.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_FEATURE_MIDAS_GKI","CONFIG_OPLUS_FEATURE_BINDER_STATS_ENABLE"],
    )
    ddk_copy_to_dist_dir(
        name = "oplus_bsp_midas_list",
        module_list = [
            "oplus_bsp_midas",
        ],
    )

