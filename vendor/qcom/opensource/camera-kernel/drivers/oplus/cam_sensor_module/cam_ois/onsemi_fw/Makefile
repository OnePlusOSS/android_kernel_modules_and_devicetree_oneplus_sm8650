# SPDX-License-Identifier: GPL-2.0-only

ccflags-y += -I$(srctree)/techpack/camera/include/uapi
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_utils
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_cpas/include
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_io
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_res_mgr
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_utils
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_req_mgr
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_cci
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_smmu
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_core/
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_ois/

obj-$(CONFIG_SPECTRA_OPLUS) += LC898124/
obj-$(CONFIG_SPECTRA_OPLUS) += LC898129/
obj-$(CONFIG_SPECTRA_OPLUS) += BU24721/
obj-$(CONFIG_SPECTRA_OPLUS) += SEM1217S/
obj-$(CONFIG_SPECTRA_OPLUS) += DW9786/

obj-$(CONFIG_SPECTRA_OPLUS) += fw_download_interface.o

