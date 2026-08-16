#ifndef __OPLUS_IO_MONITOR__
#define __OPLUS_IO_MONITOR__

#include <linux/list.h>
#include <linux/types.h>
#include <uapi/linux/magic.h>

#define PID_LENGTH			32
#define PID_HASH_LEGNTH		200
#define DEFAULT_OUTPUT_ROWS	5
#define MAX_INTERAVL_MS		60000
#define TASK_COMM_LEN		16

#ifndef FUSE_SUPER_MAGIC
#define FUSE_SUPER_MAGIC	0x65735546
#endif

struct oplus_task_io_info {
	u64 write_bytes;
	pid_t tgid;
	uid_t uid;
	unsigned short used;
	char comm[TASK_COMM_LEN];
};

struct oplus_task {
	spinlock_t t_lock;
	struct oplus_task_io_info *taskio_list;
};

#endif
