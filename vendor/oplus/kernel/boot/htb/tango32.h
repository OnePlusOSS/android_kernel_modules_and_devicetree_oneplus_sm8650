/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */

#ifndef TANGO32_H_
#define TANGO32_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define TANGO32_IOCTL_BASE 't'

#define TANGO32_ABI_MAJOR 2
#define TANGO32_ABI_MINOR 0

/*
 * Returns the ABI version for this module.
 *
 * Minor version increments indicate API additions and major version increments
 * indicate breaking changes.
 */
struct tango32_abi_version {
	__u16 major;
	__u16 minor;
};
#define TANGO32_GET_VERSION                                                    \
	_IOR(TANGO32_IOCTL_BASE, 0xa0, struct tango32_abi_version)

/*
 * Sets the given fields in the kernel memory map for the current process.
 *
 * This is similar to prctl_mm_map but with 2 differences:
 * - exe_fd is removed.
 * - all fields are u64.
 *
 * This is primarily provided to support kernels (like Android's) which are
 * built without CONFIG_CHECKPOINT_RESTORE.
 */
struct tango32_mm {
	__u64 start_code;
	__u64 end_code;
	__u64 start_data;
	__u64 end_data;
	__u64 start_brk;
	__u64 brk;
	__u64 start_stack;
	__u64 arg_start;
	__u64 arg_end;
	__u64 env_start;
	__u64 env_end;
	__u64 auxv;
	__u64 auxv_size;
};
#define TANGO32_SET_MM _IOW(TANGO32_IOCTL_BASE, 0xa1, struct tango32_mm)

/*
 * Sets the mmap_base value for the current process.
 */
#define TANGO32_SET_MMAP_BASE _IO(TANGO32_IOCTL_BASE, 0xa2)

/*
 * Executes a compat (32-bit) ioctl on the given file descriptor.
 *
 * Note that this does not handle generic file descriptor ioctls that are
 * normally handled by do_vfs_ioctl().
 */
struct tango32_compat_ioctl {
	__u32 fd;
	__u32 cmd;
	__u32 arg;
};
#define TANGO32_COMPAT_IOCTL                                                   \
	_IOW(TANGO32_IOCTL_BASE, 0xa3, struct tango32_compat_ioctl)

/*
 * Executes a compat (32-bit) set_robust_list on the current thread.
 */
struct tango32_compat_robust_list {
	__u32 head;
	__u32 len;
};
#define TANGO32_COMPAT_SET_ROBUST_LIST                                         \
	_IOW(TANGO32_IOCTL_BASE, 0xa4, struct tango32_compat_robust_list)

/*
 * Executes a compat (32-bit) get_robust_list on the current thread.
 */
#define TANGO32_COMPAT_GET_ROBUST_LIST                                         \
	_IOR(TANGO32_IOCTL_BASE, 0xa5, struct tango32_compat_robust_list)

/*
 * Executes a compat (32-bit) getdents64 syscall.
 *
 * This has special semantics on ext4 since it returns directory offsets with a
 * different encoding that fits in 32 bits.
 */
struct tango32_compat_getdents64 {
	__u32 fd;
	__u32 count;
	__u64 dirp;
};
#define TANGO32_COMPAT_GETDENTS64                                              \
	_IOW(TANGO32_IOCTL_BASE, 0xa6, struct tango32_compat_getdents64)

/*
 * Executes a compat (32-bit) lseek syscall.
 *
 * This has special semantics on ext4 since it operates on directory offsets
 * with a different encoding that fits in 32 bits.
 */
struct tango32_compat_lseek {
	__u32 fd;
	__u32 whence;
	__u64 offset;
	__u64 result;
};
#define TANGO32_COMPAT_LSEEK                                              \
	_IOWR(TANGO32_IOCTL_BASE, 0xa7, struct tango32_compat_lseek)

#endif /* TANGO32_H_ */
