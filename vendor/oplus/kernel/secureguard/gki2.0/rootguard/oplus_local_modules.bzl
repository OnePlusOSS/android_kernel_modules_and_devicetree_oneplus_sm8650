load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_secure_guard",
        srcs = native.glob([
            "*.c",
            "*.h",
        ]),
        includes = ["."],
        local_defines = [
	    "WHITE_LIST_SUPPORT",
	    "CONFIG_OPLUS_FEATURE_SECURE_ROOTGUARD",
	    "CONFIG_OPLUS_FEATURE_SECURE_CAPGUARD",
	    "CONFIG_OPLUS_FEATURE_SECURE_EXECGUARD",
	    "CONFIG_OPLUS_FEATURE_SECURE_SRGUARD",
	    "CONFIG_OPLUS_FEATURE_SECURE_SOCKETGUARD",
	],
        conditional_defines = {
            "qcom":  ["QCOM_PLATFORM"],
            "mtk":   ["MTK_PLATFORM"],
        },
    )

    ddk_copy_to_dist_dir(
        name = "oplus_secureguard",
        module_list = [
            "oplus_secure_guard",
        ],
    )

