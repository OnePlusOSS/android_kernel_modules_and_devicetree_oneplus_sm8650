#!/bin/bash

source kernel_platform/oplus/build/oplus_setup.sh $1 $2
init_build_environment

function download_prebuild_image() {

    if [[ ! -e "${ORIGIN_IMAGE}/vendor_boot.img" ]]; then
        mkdir -p ${ORIGIN_IMAGE}

        if [ -z ${IMAGE_SERVER} ]; then
            echo ""
            echo ""
            echo "you need input base version like this:"
            echo "http://xxx..xxx.com/xxx/userdebug/xxx_userdebug"
            echo ""
            echo "or you exit it and then exoprt like this:"
            echo "export IMAGE_SERVER=http://xxx..xxx.com/xxx/userdebug/xxx_userdebug"
            read IMAGE_SERVER
        fi

        if ! wget -qS ${IMAGE_SERVER}/compile.ini; then
            echo "server can't connect,please set IMAGE_SERVER and try again"
            return
        fi

        wget ${IMAGE_SERVER}/compile.ini  -O ${ORIGIN_IMAGE}/compile.ini
        OFP_DRI=`cat ${ORIGIN_IMAGE}/compile.ini | grep "ofp_folder =" | awk '{print $3 }'`
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/boot.img -O ${ORIGIN_IMAGE}/boot.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/vendor_boot.img -O ${ORIGIN_IMAGE}/vendor_boot.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/system_dlkm.img -O ${ORIGIN_IMAGE}/system_dlkm.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/vendor_dlkm.img -O ${ORIGIN_IMAGE}/vendor_dlkm.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/dtbo.img -O ${ORIGIN_IMAGE}/dtbo.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/init_boot.img -O ${ORIGIN_IMAGE}/init_boot.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/vbmeta.img  -O ${ORIGIN_IMAGE}/vbmeta.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/vbmeta_system.img  -O ${ORIGIN_IMAGE}/vbmeta_system.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/vbmeta_vendor.img  -O ${ORIGIN_IMAGE}/vbmeta_vendor.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/META/misc_info.txt -O ${ORIGIN_IMAGE}/misc_info.txt
        wget ${IMAGE_SERVER}/${OFP_DRI}/META/vendor_dlkm_image_info.txt -O ${ORIGIN_IMAGE}/vendor_dlkm_image_info.txt
        wget ${IMAGE_SERVER}/${OFP_DRI}/META/system_dlkm_image_info.txt -O ${ORIGIN_IMAGE}/system_dlkm_image_info.txt
        wget ${IMAGE_SERVER}/${OFP_DRI}/META/file_contexts.bin -O ${ORIGIN_IMAGE}/file_contexts.bin
        cp ${TOOLS}/testkey_rsa4096.pem ${ORIGIN_IMAGE}/
        cp ${TOOLS}/testkey_rsa2048.pem ${ORIGIN_IMAGE}/
        cp ${TOOLS}/testkey.avbpubkey ${ORIGIN_IMAGE}/
    fi

}

function get_image_info() {

    ${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/boot.img >  ${ORIGIN_IMAGE}/local_boot_image_info.txt
    ${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/vendor_boot.img >  ${ORIGIN_IMAGE}/local_vendor_boot_image_info.txt
    ${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/system_dlkm.img >  ${ORIGIN_IMAGE}/local_system_dlkm_info.txt
    ${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/vendor_dlkm.img >  ${ORIGIN_IMAGE}/local_vendor_dlkm_image_info.txt
    ${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/dtbo.img >  ${ORIGIN_IMAGE}/local_dtbo_image_info.txt
    ${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/vbmeta.img >  ${ORIGIN_IMAGE}/local_vbmeta_image_info.txt
    ${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/vbmeta_system.img >  ${ORIGIN_IMAGE}/local_vbmeta_system_image_info.txt
    ${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/vbmeta_vendor.img >  ${ORIGIN_IMAGE}/local_vbmeta_vendor_image_info.txt
}

function sign_boot_image() {

    algorithm=$(awk -F '[= ]' '$1=="avb_boot_algorithm" {$1="";print}' ${ORIGIN_IMAGE}/misc_info.txt)
    partition_size=$(awk -F '[= ]' '$1=="boot_size" {$1="";print}' ${ORIGIN_IMAGE}/misc_info.txt)
    footer_args=$(awk -F '[= ]' '$1=="avb_boot_add_hash_footer_args" {$1="";print}' ${ORIGIN_IMAGE}/misc_info.txt)
    partition_name=boot
    salt=`uuidgen | sed 's/-//g'`
    ${AVBTOOL} add_hash_footer \
        --image ${IMAGE_OUT}/boot.img  \
        --partition_name ${partition_name} \
        --partition_size ${partition_size}\
        --algorithm ${algorithm} \
        --key ${ORIGIN_IMAGE}/testkey_rsa4096.pem \
        --salt ${salt} \
        ${footer_args}
}

function sign_vendor_boot_image() {

    algorithm=$(awk -F '[= ]' '$1=="avb_vendor_boot_algorithm" {$1="";print}' ${ORIGIN_IMAGE}/misc_info.txt)
    partition_size=$(awk -F '[= ]' '$1=="vendor_boot_size" {$1="";print}' ${ORIGIN_IMAGE}/misc_info.txt)
    footer_args=$(awk -F '[= ]' '$1=="avb_vendor_boot_add_hash_footer_args" {$1="";print}' ${ORIGIN_IMAGE}/misc_info.txt)
    partition_name=vendor_boot
    salt=`uuidgen | sed 's/-//g'`

    if [ -z "$algorithm" ]; then
     algorithm="SHA256_RSA4096"
    fi

    if [ -z "$partition_size" ]; then
     partition_size=`cat ${ORIGIN_IMAGE}/local_vendor_boot_image_info.txt | grep "Image size:" | awk '{print $3 }'`
    fi

    ${AVBTOOL} add_hash_footer \
        --image ${IMAGE_OUT}/vendor_boot.img  \
        --partition_name ${partition_name} \
        --partition_size ${partition_size}\
        --algorithm ${algorithm} \
        --key ${ORIGIN_IMAGE}/testkey_rsa4096.pem \
        --salt ${salt} \
        ${footer_args}
}

function sign_dtbo_image() {

    algorithm=$(awk -F '[= ]' '$1=="avb_dtbo_algorithm" {$1="";print}' ${ORIGIN_IMAGE}/misc_info.txt)
    partition_size=$(awk -F '[= ]' '$1=="dtbo_size" {$1="";print}' ${ORIGIN_IMAGE}/misc_info.txt)
    footer_args=$(awk -F '[= ]' '$1=="avb_dtbo_add_hash_footer_args" {$1="";print}' ${ORIGIN_IMAGE}/misc_info.txt)
    partition_name=dtbo
    salt=`uuidgen | sed 's/-//g'`
    if [ -z "$algorithm" ]; then
     algorithm="SHA256_RSA4096"
    fi
    ${AVBTOOL} add_hash_footer \
        --image ${IMAGE_OUT}/dtbo.img  \
        --partition_name ${partition_name} \
        --partition_size ${partition_size}\
        --algorithm ${algorithm} \
        --key ${ORIGIN_IMAGE}/testkey_rsa4096.pem \
        --salt ${salt} \
        ${footer_args}
}

function sign_vendor_dlkm_image() {

    algorithm=$(awk -F '[= ]' '$1=="avb_vendor_dlkm_algorithm" {$1="";print}' ${ORIGIN_IMAGE}/vendor_dlkm_image_info.txt)
    partition_size=$(awk -F '[= ]' '$1=="vendor_dlkm_size" {$1="";print}' ${ORIGIN_IMAGE}/vendor_dlkm_image_info.txt)
    footer_args=$(awk -F '[= ]' '$1=="avb_vendor_dlkm_add_hashtree_footer_args" {$1="";print}' ${ORIGIN_IMAGE}/vendor_dlkm_image_info.txt)
    partition_name=vendor_dlkm
    salt=`uuidgen | sed 's/-//g'`

    if [ -z "$algorithm" ]; then
     algorithm="SHA256_RSA4096"
    fi

    if [ -z "$partition_size" ]; then
     partition_size=`cat ${ORIGIN_IMAGE}/local_vendor_dlkm_image_info.txt | grep "Image size:" | awk '{print $3 }'`
    fi

    ${AVBTOOL} add_hashtree_footer \
        --partition_name ${partition_name} \
        --partition_size ${partition_size}\
        --do_not_generate_fec \
        --image ${IMAGE_OUT}/vendor_dlkm.img  \
        --hash_algorithm sha256 \
        --salt ${salt}  \
        ${footer_args}
}

function sign_system_dlkm_image() {

    algorithm=$(awk -F '[= ]' '$1=="avb_system_dlkm_algorithm" {$1="";print}' ${ORIGIN_IMAGE}/system_dlkm_image_info.txt)
    partition_size=$(awk -F '[= ]' '$1=="system_dlkm_size" {$1="";print}' ${ORIGIN_IMAGE}/system_dlkm_image_info.txt)
    footer_args=$(awk -F '[= ]' '$1=="avb_add_hashtree_footer_args" {$1="";print}' ${ORIGIN_IMAGE}/system_dlkm_image_info.txt)
    partition_name=system_dlkm
    salt=`uuidgen | sed 's/-//g'`

    if [ -z "$algorithm" ]; then
     algorithm="SHA256_RSA4096"
    fi

    if [ -z "$partition_size" ]; then
     partition_size=`cat ${ORIGIN_IMAGE}/local_system_dlkm_info.txt | grep "Image size:" | awk '{print $3 }'`
    fi

    ${AVBTOOL} add_hashtree_footer \
        --partition_name ${partition_name} \
        --partition_size ${partition_size}\
        --do_not_generate_fec \
        --image ${IMAGE_OUT}/system_dlkm.img  \
        --hash_algorithm sha256 \
        --salt ${salt}  \
        ${footer_args}
}

function sign_prebuild_image() {
    sign_boot_image
    sign_vendor_boot_image
    sign_dtbo_image
    sign_vendor_dlkm_image
    sign_system_dlkm_image
}

rebuild_boot_image() {
    echo "rebuild boot.img"
    rm -rf ${BOOT_TMP_IMAGE}/*
    boot_mkargs=$(${PYTHON_TOOL} ${UNPACK_BOOTIMG_TOOL} --boot_img ${ORIGIN_IMAGE}/boot.img --out ${BOOT_TMP_IMAGE} --format=mkbootimg)
    cp ${TOPDIR}/kernel_platform/out/msm-kernel-${variants_platform}-${variants_type}/dist/Image ${BOOT_TMP_IMAGE}/kernel
    bash -c "${PYTHON_TOOL} ${MKBOOTIMG_PATH} ${boot_mkargs} -o ${IMAGE_OUT}/boot.img"
    sign_boot_image
}

rebuild_dtb_image() {
    echo "rebuild dtb.img"
    cp ${KERNEL_OUT}/dtbs/dtb.img ${VENDOR_BOOT_TMP_IMAGE}/origin/dtb
}

vendor_boot_modules_update() {

    echo "vendor_boot module update"

    ko_list=`cat ${ACKDIR}/oplus/prebuild/local_add_vendor_boot.txt`
    for ko in  $ko_list
    do
        if [ -e ${DIST_INSTALL}/${ko} ]; then
           cp ${DIST_INSTALL}/${ko}  ${VENDOR_BOOT_TMP_IMAGE}/ramdisk00/lib/modules/
           echo $ko_list >>  ${VENDOR_BOOT_TMP_IMAGE}/ramdisk00/lib/modules/modules.load
        fi
    done
}
rebuild_vendor_boot_image() {
    echo "rebuild vendor_boot.img"
    rm -rf ${VENDOR_BOOT_TMP_IMAGE}/*
    boot_mkargs=$(${PYTHON_TOOL} ${UNPACK_BOOTIMG_TOOL} --boot_img ${ORIGIN_IMAGE}/vendor_boot.img --out ${VENDOR_BOOT_TMP_IMAGE}/origin --format=mkbootimg)
    rebuild_dtb_image
    index="00"
    for index in  $index
    do
        echo " index  $index "
        mv ${VENDOR_BOOT_TMP_IMAGE}/origin/vendor_ramdisk${index} ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4
        ${LZ4} -d ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4
        rm ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4
        mkdir -p ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}
        mv ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index} ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}/vendor_ramdisk${index}
        pushd  ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}
        ${CPIO} -idu < ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}/vendor_ramdisk${index}

        popd
        rm ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}/vendor_ramdisk${index}

        vendor_boot_modules_update
        ${MKBOOTFS} ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index} > ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}
        ${LZ4} -l -12 --favor-decSpeed ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}
        mv ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4 ${VENDOR_BOOT_TMP_IMAGE}/origin/vendor_ramdisk${index}
        rm ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}
    done
    bash -c "${PYTHON_TOOL} ${MKBOOTIMG_PATH} ${boot_mkargs} --vendor_boot ${IMAGE_OUT}/vendor_boot.img"
    #sign_vendor_boot_image
}

rebuild_vendor_dlkm_image() {
    echo "rebuild vendor_dlkm.img"
    mkdir -p ${VENDOR_DLKM_TMP_IMAGE}
    ${SIMG2IMG} ${ORIGIN_IMAGE}/vendor_dlkm.img  ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm.img
    ${TOOLS}/7z_new ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm.img ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm_out
    ${BUILD_IMAGE} ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm_out ${TOOLS}/vendor_dlkm_image_info.txt ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm.img /dev/null
}

vendor_dlkm_modules_update() {
    echo "vendor_dlkm module update"

    ko_list=`cat ${ACKDIR}/oplus/prebuild/local_add_vendor_dlkm.txt`
    for ko in  $ko_list
    do
        if [ -e ${DIST_INSTALL}/${ko} ]; then
           cp ${DIST_INSTALL}/${ko}  ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm/lib/modules/
           echo $ko_list >>  ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm/lib/modules/modules.load
        fi
    done
}
modules_update_check() {
    echo "modules_update_check update"
    find ${DIST_INSTALL}/ -name "*.ko" -printf "%f\n" | sort -bfd| uniq  > ${DIST_INSTALL}/local_origin_list_ko_sort.txt
    cat ${VENDOR_BOOT_TMP_IMAGE}/origin_list_ko_sort.txt  \
        ${VENDOR_DLKM_TMP_IMAGE}/origin_list_ko_sort.txt \
        | sort -bfd| uniq  > ${DIST_INSTALL}/server_origin_list_ko_sort.txt
}
rebuild_vendor_dlkm_erofs_image() {
    echo "rebuild vendor_dlkm.img"
    rm -rf ${VENDOR_DLKM_TMP_IMAGE}/*
    mkdir -p ${VENDOR_DLKM_TMP_IMAGE}
    ${SIMG2IMG} ${ORIGIN_IMAGE}/vendor_dlkm.img  ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm.img
    ${TOOLS}/erofs_unpack.sh ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm.img  ${VENDOR_DLKM_TMP_IMAGE}/mnt ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm
    vendor_dlkm_modules_update
    touch ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm/lib/modules/readme.txt
    ${EROFS} -z lz4hc,9 --mount-point vendor_dlkm --file-contexts ${ORIGIN_IMAGE}/file_contexts.bin --block-list-file ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm.map ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm_repack.img  ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm

    ${IMG2SIMG} ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm_repack.img  ${IMAGE_OUT}/vendor_dlkm.img
    #sign_vendor_dlkm_image

}
rebuild_system_dlkm_erofs_image() {
    echo "rebuild system_dlkm.img"
    rm -rf ${SYSTEM_DLKM_TMP_IMAGE}/*
    mkdir -p ${SYSTEM_DLKM_TMP_IMAGE}
    ${SIMG2IMG} ${ORIGIN_IMAGE}/system_dlkm.img  ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm.img
    ${TOOLS}/erofs_unpack.sh ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm.img  ${SYSTEM_DLKM_TMP_IMAGE}/mnt ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm
    ${EROFS} -z lz4hc,9 --mount-point system_dlkm  --file-contexts ${ORIGIN_IMAGE}/file_contexts.bin --block-list-file ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm.map ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm_repack.img  ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm

    ${IMG2SIMG} ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm_repack.img  ${IMAGE_OUT}/system_dlkm.img
    #sign_system_dlkm_image
}

rebuild_dtbo_image() {
    echo "rebuild dtbo.img"
    cp ${KERNEL_OUT}/dtbs/dtbo.img ${IMAGE_OUT}/
    sign_dtbo_image
}

rebuild_vbmeta_image() {
    avb_vbmeta_args=$(awk -F '[= ]' '$1=="avb_vbmeta_args" {$1="";print}' ${ORIGIN_IMAGE}/misc_info.txt)
    ${AVBTOOL} make_vbmeta_image \
        --chain_partition boot:4:${ORIGIN_IMAGE}/testkey.avbpubkey \
        --chain_partition vendor_boot:6:${ORIGIN_IMAGE}/testkey.avbpubkey \
        --chain_partition dtbo:3:${ORIGIN_IMAGE}/testkey.avbpubkey \
        --chain_partition recovery:1:${ORIGIN_IMAGE}/testkey.avbpubkey \
        --chain_partition vbmeta_system:2:${ORIGIN_IMAGE}/testkey.avbpubkey \
        --chain_partition vbmeta_vendor:5:${ORIGIN_IMAGE}/testkey.avbpubkey \
        --include_descriptors_from_image ${ORIGIN_IMAGE}/init_boot.img  \
        ${avb_vbmeta_args} \
        --algorithm SHA256_RSA4096  \
        --key ${ORIGIN_IMAGE}/testkey_rsa4096.pem \
        --output ${IMAGE_OUT}/vbmeta.img
}
build_start_time
download_prebuild_image
get_image_info
rebuild_boot_image
rebuild_vendor_boot_image
rebuild_dtbo_image
# rebuild_vendor_dlkm_erofs_image
# rebuild_system_dlkm_erofs_image
# rebuild_vbmeta_image
print_end_help
build_end_time