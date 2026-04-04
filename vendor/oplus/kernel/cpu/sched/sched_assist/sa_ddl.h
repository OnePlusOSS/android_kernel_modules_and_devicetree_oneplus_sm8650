#ifndef _OPLUS_SA_DDL_H_
#define _OPLUS_SA_DDL_H_

#define BUFFER_SIZE_DDL	(512)
#define SAVED_SIZE	(50)
#define MAX_GUARDS_SIZE	(BUFFER_SIZE_DDL - SAVED_SIZE)
#define MAX_DDL_LIMIT (5000)
#define MAX_DDL_RTHRES (150)
#define NUM_DDL_HIT_ITEM (50)

struct proc_dir_entry;

enum ddl_cmd_id {
	SET_THREAD_DDL,
	SET_PROCESS_DDL,
	DDL_CMD_MAX,
};

struct ddl_ioctl_data {
	pid_t pid;
	u64 ddl;
};

struct ddl_sinfo_data {
	char	comm[TASK_COMM_LEN];
	u64	hit;
};

#define DDL_MAGIC (0XDD)
#define IOCTL_SET_THREAD_DDL \
	_IOW(DDL_MAGIC, SET_THREAD_DDL, struct ddl_ioctl_data)
#define IOCTL_SET_PROCESS_DDL \
	_IOW(DDL_MAGIC, SET_PROCESS_DDL, struct ddl_ioctl_data)


u64 oplus_get_task_ddl(struct task_struct *task);
void oplus_set_task_ddl(struct task_struct *task, u64 ddl);
void oplus_enqueue_ddl_node(struct rq *rq, struct task_struct *p);
void oplus_dequeue_ddl_node(struct rq *rq, struct task_struct *p);
void oplus_task_ddl_tint(struct rq *rq, struct task_struct *next);
bool oplus_ddl_within_limit(struct rq *rq, struct task_struct *task);
void oplus_ddl_check_preempt(struct rq *rq, struct task_struct *p, struct task_struct *curr, bool *preempt, bool *nopreempt);
void oplus_replace_next_task_ddl(struct rq *rq, struct task_struct **p,
	struct sched_entity **se, bool *repick, bool simple);
void oplus_sched_ddl_init(struct proc_dir_entry *pde);
void oplus_ddl_preempt_tint(struct rq *rq, struct task_struct *prev);
#endif
