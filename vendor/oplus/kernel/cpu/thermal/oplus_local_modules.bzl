load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "horae_shell_temp",
        srcs = native.glob([
            "**/*.h",
            "horae_shell_temp.c",
        ]),
        includes = ["."],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "thermal_pa_adc",
        srcs = native.glob([
            "**/*.h",
            "thermal_pa_adc.c",
        ]),
        includes = ["."],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_ipa_thermal",
        srcs = native.glob([
            "**/*.h",
            "oplus_ipa_thermal.c",
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/cpu/thermal:horae_shell_temp",
        ],
        includes = ["."],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    ddk_copy_to_dist_dir(
        name = "oplus_horae",
        module_list = [
            "horae_shell_temp",
            "oplus_ipa_thermal",
            "thermal_pa_adc",
        ],
    )
