load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//msm-kernel:target_variants.bzl", "get_all_la_variants")
load(":oplus_modules_variant.bzl",
"bazel_support_target",
"bazel_support_variant"
)

def ddk_copy_to_dist_dir(
        name = None,
        module_list = []):

    data = []

    if name == None:
        name = "ddk_oplus_default"

    for module in module_list:
        data.append(":{}".format(module))

    #for (targets, variant) in get_all_la_variants():
    for targets in bazel_support_target:
        for variant in bazel_support_variant:
            copy_to_dist_dir(
                name = "{}_{}_{}_dist".format(targets,variant,name),
                data = data,
                dist_dir = "out/target/product/{}_{}/dlkm/lib/modules/".format(targets,variant),
                flat = True,
                wipe_dist_dir = False,
                allow_duplicate_filenames = False,
                mode_overrides = {"**/*": "644"},
            )

