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
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
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
            "hybridswap_zram/kcompressd.c",
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/mm:oplus_bsp_zsmalloc",
            "//vendor/oplus/kernel/mm:oplus_bsp_zstdn",
            "//vendor/oplus/kernel/mm:oplus_bsp_zstdn_o",
        ],
        includes = ["."],
        local_defines = [
            "CONFIG_HYBRIDSWAP",
            "CONFIG_HYBRIDSWAP_SWAPD",
            "CONFIG_HYBRIDSWAP_CORE",
            "CONFIG_CRYPTO_LZ4K",
            "CONFIG_KCOMPRESSD",
            "CONFIG_CRYPTO_ZSTDN",
            "CONFIG_CRYPTO_ZSTDN_O"
        ],
        conditional_defines = {
             "qcom":  ["CONFIG_QCOM_PANEL_EVENT_NOTIFIER"],
             "mtk":  ["CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY"],
        },
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
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
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_level_protect",

        srcs = native.glob([
            "**/*.h",
            "levelprotect/levelprotect.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_LEVEL_PROTECT"],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_sigkill_diagnosis",
        srcs = native.glob([
            "sigkill_diagnosis/sigkill_diagnosis.c",
        ]),
        includes = ["."],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_zram_opt",
        srcs = native.glob([
            "**/*.h",
            "zram_opt/zram_opt.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_DYNAMIC_TUNING_SWAPPINESS", "CONFIG_OPLUS_BALANCE_ANON_FILE_RECLAIM", "CONFIG_HYBRIDSWAP_SWAPD","CONFIG_OPLUS_EXTRA_FREE_KBYTES"],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_proactive_compact",
        srcs = native.glob([
            "**/*.h",
            "proactive_compact/proactive_compact.c",
        ]),
        includes = ["."],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_lz4k",
        srcs = native.glob([
            "**/*.h",
            "hybridswap_zram/lz4k/lz4k.c",
            "hybridswap_zram/lz4k/lz4k_compress.c",
            "hybridswap_zram/lz4k/lz4k_decompress.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_CRYPTO_LZ4K"],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_kswapd_opt",
        srcs = native.glob([
            "kswapd_opt/kswapd_opt.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_FEATURE_KSWAPD_OPT"],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_uxmem_opt",
        srcs = native.glob([
            "**/*.h",
            "uxmem_opt/uxmem_opt.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_FEATURE_UXMEM_OPT"],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_dynamic_readahead",
        srcs = native.glob([
            "**/*.h",
            "dynamic_readahead/dynamic_readahead.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_OPLUS_FEATURE_DYNAMIC_READAHEAD"],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_kshrink_slabd",
        srcs = native.glob([
            "**/*.h",
            "async_reclaim_opt/kshrink_slabd/kshrink_slabd.c",
        ]),
        includes = ["."],
        local_defines = ["CONFIG_KSHRINK_SLABD"],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_exit_mm_optimize",
        srcs = native.glob([
            "**/*.h",
            "exit_mm_optimize/exit_mm_optimize.c",
        ]),
        ko_deps = [
            "//vendor/oplus/kernel/mm:oplus_bsp_hybridswap_zram",
        ],
        includes = ["."],
        copts = select({
            "//build/kernel/kleaf:kocov_is_true": ["-fprofile-arcs", "-ftest-coverage"],
            "//conditions:default": [],
        }),
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_zstdn",
        srcs = native.glob([
            "**/*.h",
            "hybridswap_zram/zstd/include/*.h",
            "hybridswap_zram/zstd/common/*.h",
            "hybridswap_zram/zstd/compress/*.h",
            "hybridswap_zram/zstd/decompress/*.h",
            "hybridswap_zram/zstd/crypto_zstd.c",
            "hybridswap_zram/zstd/zstd_compress_module.c",
            "hybridswap_zram/zstd/xxhash.c",
            "hybridswap_zram/zstd/common/debug.c",
            "hybridswap_zram/zstd/common/entropy_common.c",
            "hybridswap_zram/zstd/common/error_private.c",
            "hybridswap_zram/zstd/common/fse_decompress.c",
            "hybridswap_zram/zstd/common/zstd_common.c",
            "hybridswap_zram/zstd/compress/fse_compress.c",
            "hybridswap_zram/zstd/compress/hist.c",
            "hybridswap_zram/zstd/compress/huf_compress.c",
            "hybridswap_zram/zstd/compress/zstd_compress.c",
            "hybridswap_zram/zstd/compress/zstd_compress_literals.c",
            "hybridswap_zram/zstd/compress/zstd_compress_sequences.c",
            "hybridswap_zram/zstd/compress/zstd_compress_superblock.c",
            "hybridswap_zram/zstd/compress/zstd_double_fast.c",
            "hybridswap_zram/zstd/compress/zstd_fast.c",
            "hybridswap_zram/zstd/compress/zstd_lazy.c",
            "hybridswap_zram/zstd/compress/zstd_ldm.c",
            "hybridswap_zram/zstd/compress/zstd_opt.c",
            "hybridswap_zram/zstd/zstd_decompress_module.c",
            "hybridswap_zram/zstd/decompress/huf_decompress.c",
            "hybridswap_zram/zstd/decompress/zstd_ddict.c",
            "hybridswap_zram/zstd/decompress/zstd_decompress.c",
            "hybridswap_zram/zstd/decompress/zstd_decompress_block.c"
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_bsp_zstdn_o",
        srcs = native.glob([
            "**/*.h",
            "zstd_o/include/*.h",
            "zstd_o/common/*.h",
            "zstd_o/compress/*.h",
            "zstd_o/decompress/*.h",
            "zstd_o/crypto_zstd.c",
            "zstd_o/zstd_compress_module.c",
            "zstd_o/xxhash.c",
            "zstd_o/common/debug.c",
            "zstd_o/common/entropy_common.c",
            "zstd_o/common/error_private.c",
            "zstd_o/common/fse_decompress.c",
            "zstd_o/common/zstd_common.c",
            "zstd_o/compress/fse_compress.c",
            "zstd_o/compress/hist.c",
            "zstd_o/compress/huf_compress.c",
            "zstd_o/compress/zstd_compress.c",
            "zstd_o/compress/zstd_compress_literals.c",
            "zstd_o/compress/zstd_compress_sequences.c",
            "zstd_o/compress/zstd_compress_superblock.c",
            "zstd_o/compress/zstd_double_fast.c",
            "zstd_o/compress/zstd_fast.c",
            "zstd_o/compress/zstd_lazy.c",
            "zstd_o/compress/zstd_ldm.c",
            "zstd_o/compress/zstd_opt.c",
            "zstd_o/zstd_decompress_module.c",
            "zstd_o/decompress/huf_decompress.c",
            "zstd_o/decompress/zstd_ddict.c",
            "zstd_o/decompress/zstd_decompress.c",
            "zstd_o/decompress/zstd_decompress_block.c"
        ]),
        includes = ["."],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_bsp_mm",
        module_list = [
            "oplus_bsp_memleak_detect_simple",
            "oplus_bsp_sigkill_diagnosis",
            "oplus_bsp_zram_opt",
            "oplus_exit_mm_optimize",
            "oplus_bsp_level_protect",
            "oplus_bsp_proactive_compact",
            "oplus_bsp_hybridswap_zram",
            "oplus_bsp_zsmalloc",
            "oplus_bsp_lz4k",
            "oplus_bsp_kswapd_opt",
            "oplus_bsp_kshrink_slabd",
            "oplus_bsp_zstdn",
            "oplus_bsp_zstdn_o",
            # TODO: qcom convert to GKI implementation
            #"oplus_bsp_uxmem_opt",
            #"oplus_bsp_dynamic_readahead",
        ],
    )
