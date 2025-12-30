load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_patch",
        srcs = native.glob([
            "oplus_patch.h",
            "oplus_patch.c",
        ]),

        conditional_defines = {
            "mtk":  ["CONFIG_OPLUS_PATCH"],
            "qcom": ["CONFIG_OPLUS_PATCH"],
        },
        includes = ["."],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_patch",
        module_list = ["oplus_patch"],
    )
