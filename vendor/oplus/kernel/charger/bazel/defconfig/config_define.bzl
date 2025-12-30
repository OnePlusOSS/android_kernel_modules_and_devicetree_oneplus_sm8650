load(":defconfig/oplus_canoe_perf_config.bzl", "oplus_canoe_perf_config")
load(":defconfig/oplus_canoe_consolidate_config.bzl", "oplus_canoe_consolidate_config")

oplus_config = {
    "qcom": {
        "canoe": {
            "perf": oplus_canoe_perf_config,
            "consolidate": oplus_canoe_perf_config | oplus_canoe_consolidate_config
        }
    },
}
