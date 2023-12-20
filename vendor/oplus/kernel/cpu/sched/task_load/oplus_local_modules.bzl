load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_bsp_task_load",
        srcs = native.glob([
            "task_load.c",
            "task_load.h",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_FEATURE_TASK_LOAD"],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_bsp_task_load",
        module_list = [
            "oplus_bsp_task_load",
        ],
    )
