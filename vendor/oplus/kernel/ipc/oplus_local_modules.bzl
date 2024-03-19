load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_binder_strategy",
        srcs = native.glob([
            "binder_main.c",
            "binder_sched.c",
            "binder_sysfs.c",
            "*.h",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_BINDER_STRATEGY", "CONFIG_OPLUS_BINDER_PRIO_SKIP"],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_binder_strategy",
        module_list = [
            "oplus_binder_strategy",
        ],
    )
