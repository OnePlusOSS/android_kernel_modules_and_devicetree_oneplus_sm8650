load("//build/kernel/kleaf:kernel.bzl", "ddk_module")
load("//msm-kernel:target_variants.bzl", "get_all_la_variants")
load(":oplus_modules_variant.bzl",
"bazel_support_target",
"bazel_support_variant"
)
bazel_support_platform = "qcom"

def define_oplus_ddk_module(
    name,
    srcs = None,
    header_deps = [],
    ko_deps = [],
    hdrs = None,
    includes = None,
    conditional_srcs = None,
    conditional_defines = None,
    linux_includes = None,
    out = None,
    local_defines = None,
    copts = None):

    if srcs == None:
        srcs = native.glob(
            [
                "**/*.c",
                "**/*.h",
            ],
            exclude = [
                ".*",
                ".*/**",
            ],
        )

    if out == None:
        out = name + ".ko"

    flattened_conditional_defines = None
    if conditional_defines:
        for config_vendor, config_defines in conditional_defines.items():
            if config_vendor == bazel_support_platform:
                if flattened_conditional_defines:
                    flattened_conditional_defines = flattened_conditional_defines + config_defines
                else:
                    flattened_conditional_defines = config_defines

    if flattened_conditional_defines:
        if local_defines:
            local_defines =  local_defines + flattened_conditional_defines
        else:
            local_defines = flattened_conditional_defines

    #fail("debug need variable {}".format(local_defines))

    #for (targets, variant) in get_all_la_variants():
    for targets in bazel_support_target:
        for variant in bazel_support_variant:
            ddk_module(
                name = "{}".format(name),
                srcs = srcs,
                out = "{}".format(out),
                local_defines = local_defines,
                includes = includes,
                conditional_srcs = conditional_srcs,
                linux_includes = linux_includes,
                hdrs = hdrs,
                deps = ["//msm-kernel:all_headers"] + header_deps + ko_deps,
                kernel_build = "//msm-kernel:{}_{}".format(targets,variant),
                visibility = ["//visibility:public"]
            )


