// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2025 Oplus. All rights reserved.
 */
#include <kunit/test.h>
#include <kunit/fff.h>
//#include "fff.h"
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mmzone.h>
#include <linux/fdtable.h>

#include "../file_read_record.c"
#include "../../mm/internal.h"

DEFINE_FFF_GLOBALS;

/*
 * 测试用例描述：io_monitor_record信息记录接口测试，本函数是io访问记录的入口函数
 * 	本例测试此接口记录功能仅在io monitor状态START情况下才会记录
 * 预置条件：无
 * 参数描述：
 * file：IO访问的文件，本例中传入空
 * offset：访问偏移
 * req_size: 访问大小
 * type：读访问记录
 * 返回值描述：仅在io monitor状态START情况实际值记录信息条目才会增长
 */
static void io_monitor_record_kunit_test_case1(struct kunit *test)
{
	bool ret = false;
	pid_t tgid = task_tgid_nr(current);
	int i = 0;
	io_monitor *monitor = get_io_monitor(IO_MONITOR_TYPE_READ);

	/* UNKOWN can not record */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	io_monitor_record(NULL, i, 1, IO_MONITOR_TYPE_READ);
	KUNIT_EXPECT_EQ(test, monitor->buf->cur, i);
	/* INIT can not record */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	io_monitor_record(NULL, i, 1, IO_MONITOR_TYPE_READ);
	KUNIT_EXPECT_EQ(test, monitor->buf->cur, i);
	/* START can record */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	io_monitor_record(NULL, i, 1, IO_MONITOR_TYPE_READ);
	i++;
	KUNIT_EXPECT_EQ(test, monitor->buf->cur, i);
	/* STOP can not record */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	io_monitor_record(NULL, i, 1, IO_MONITOR_TYPE_READ);
	KUNIT_EXPECT_EQ(test, monitor->buf->cur, i);

	/* FINISH can not record */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	io_monitor_record(NULL, i, 1, IO_MONITOR_TYPE_READ);
	KUNIT_EXPECT_EQ(test, monitor->buf->cur, i);
}

/*
 * 测试用例描述：io_monitor_record信息记录接口测试，本函数是io访问记录的入口函数
 * 	本例测试此接口记录信息需要判断当前监控的进程是否匹配，如果不匹配不应记录
 * 预置条件：无
 * 参数描述：
 * file：IO访问的文件，本例中传入空
 * offset：访问偏移
 * req_size: 访问大小
 * type：读访问记录
 * 返回值描述：预期尝试记录2倍IO_INFO_RECORD_MAX大小记录，但实际值记录为0
 */
static void io_monitor_record_kunit_test_case2(struct kunit *test)
{
	bool ret = false;
	int i = 0;
	io_monitor *monitor = get_io_monitor(IO_MONITOR_TYPE_READ);

	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, 123);
	KUNIT_EXPECT_EQ(test, ret, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, 123);
	KUNIT_EXPECT_EQ(test, ret, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, 123);
	KUNIT_EXPECT_EQ(test, ret, true);

	for (i = 0; i < IO_INFO_RECORD_MAX * 2; i++) {
		io_monitor_record(NULL, i, 1, IO_MONITOR_TYPE_READ);
	}
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, 123);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->cur, 0);
}

/*
 * 测试用例描述：io_monitor_record信息记录接口测试，本函数是io访问记录的入口函数
 * 	本例测试此接口记录信息条目不能超出IO_INFO_RECORD_MAX
 * 预置条件：无
 * 参数描述：
 * file：IO访问的文件，本例中传入空
 * offset：访问偏移
 * req_size: 访问大小
 * type：读访问记录
 * 返回值描述：预期尝试记录IO_INFO_RECORD_MAX + 1条信息，但实际值记录IO_INFO_RECORD_MAX
 */
static void io_monitor_record_kunit_test_case3(struct kunit *test)
{
	bool ret = false;
	pid_t tgid = task_tgid_nr(current);
	int i = 0;
	io_monitor *monitor = get_io_monitor(IO_MONITOR_TYPE_READ);

	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);

	for (i = 0; i < IO_INFO_RECORD_MAX + 1; i++) {
		io_monitor_record(NULL, i, 1, IO_MONITOR_TYPE_READ);
	}
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->cur, IO_INFO_RECORD_MAX);
}

/*
 * 测试用例描述：io_monitor状态机测试，本用例主要测试FINISHED和UNKOWN状态设置功能是否正常
 * 	主要测试io_monitor_status_set接口
 * 预置条件：FINISHED和UNKOWN状态不可设置
 * 参数描述：
 *  IO_MONITOR_STATUS_START：START状态标记
 *  IO_MONITOR_TYPE_READ：代表当前监控读操作，请注意目前仅支持监控读操作
 *  tgid: 待监控进程的tgid，Kunit测试用例中以current替代
 * 返回值描述：返回值为true代表设置成功，false代表设置失败
 */
static void io_monitor_status_chage_kunit_test_case6(struct kunit *test)
{
	bool ret = false;
	pid_t tgid = task_tgid_nr(current);
	io_monitor *monitor = get_io_monitor(IO_MONITOR_TYPE_READ);

	/* STOP -> FINISHED fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISHED, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_UNKNOWN, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);

	/* UNKOWN -> UNKWON fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_UNKNOWN, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISHED, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
}

/*
 * 测试用例描述：io_monitor状态机测试，本用例主要测试FINISH状态设置功能是否正常
 * 	主要测试io_monitor_status_set接口
 * 预置条件：仅当STOP时才能够成功设置为FINISH状态
 * 参数描述：
 *  IO_MONITOR_STATUS_FINISH：FINISH状态标记，FINISH标记被设置后实际标记未FINISHED
 *  IO_MONITOR_TYPE_READ：代表当前监控读操作，请注意目前仅支持监控读操作
 *  tgid: 待监控进程的tgid，Kunit测试用例中以current替代
 * 返回值描述：返回值为true代表设置成功，false代表设置失败
 */
static void io_monitor_status_chage_kunit_test_case5(struct kunit *test)
{
	bool ret = false;
	pid_t tgid = task_tgid_nr(current);
	io_monitor *monitor = get_io_monitor(IO_MONITOR_TYPE_READ);

	/* STOP -> FINISH */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* different tgid set FINISH fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
		ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, -1);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);

	/* set UNKWON to FINISH fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* set INIT to FINISH fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* set START to FINISH fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);

	/* set FINISH to FINISH fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
}

/*
 * 测试用例描述：io_monitor状态机测试，本用例主要测试STOP状态设置功能是否正常
 * 	主要测试io_monitor_status_set接口
 * 预置条件：仅当START或STOP时才能够成功设置为STOP状态
 * 参数描述：
 *  IO_MONITOR_STATUS_STOP：STOP状态标记
 *  IO_MONITOR_TYPE_READ：代表当前监控读操作，请注意目前仅支持监控读操作
 *  tgid: 待监控进程的tgid，Kunit测试用例中以current替代
 * 返回值描述：返回值为true代表设置成功，false代表设置失败
 */
static void io_monitor_status_chage_kunit_test_case4(struct kunit *test)
{
	bool ret = false;
	pid_t tgid = task_tgid_nr(current);
	io_monitor *monitor = get_io_monitor(IO_MONITOR_TYPE_READ);

	/* START -> STOP */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);

	/* different tgid set STOP fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, -1);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);

	/* set UNKWON to STOP fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* set INIT to STOP fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* set STOP to STOP ok */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);

	/* set FINISHED to START fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
}

/*
 * 测试用例描述：io_monitor状态机测试，本用例主要测试START状态设置功能是否正常
 * 	主要测试io_monitor_status_set接口
 * 预置条件：仅当INIT时才能够成功设置为START状态
 * 参数描述：
 *  IO_MONITOR_STATUS_START：START状态标记
 *  IO_MONITOR_TYPE_READ：代表当前监控读操作，请注意目前仅支持监控读操作
 *  tgid: 待监控进程的tgid，Kunit测试用例中以current替代
 * 返回值描述：返回值为true代表设置成功，false代表设置失败
 */
static void io_monitor_status_chage_kunit_test_case3(struct kunit *test)
{
	bool ret = false;
	pid_t tgid = task_tgid_nr(current);
	io_monitor *monitor = get_io_monitor(IO_MONITOR_TYPE_READ);

	/* INIT -> START */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);

	/* different tgid set START fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, -1);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* set UNKWON to START fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* set START to START fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);

	/* set STOP to START fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);

	/* set FINISHED to START fail */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
}

/*
 * 测试用例描述：io_monitor状态机测试，本用例主要测试INIT状态设置功能是否正常
 * 	主要测试io_monitor_status_set接口
 * 预置条件：仅当INIT，FINISHED，STOP及UNKOWN时才能够成功设置为INIT状态
 * 参数描述：
 *  IO_MONITOR_STATUS_INIT：INIT状态标记
 *  IO_MONITOR_TYPE_READ：代表当前监控读操作，请注意目前仅支持监控读操作
 *  tgid: 待监控进程的tgid，Kunit测试用例中以current替代
 * 返回值描述：返回值为true代表设置成功，false代表设置失败
 */
static void io_monitor_status_chage_kunit_test_case2(struct kunit *test)
{
	bool ret = false;
	pid_t tgid = task_tgid_nr(current);
	io_monitor *monitor = get_io_monitor(IO_MONITOR_TYPE_READ);

	/* UNKOWN -> INIT */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	/* INIT -> INIT */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	/* STOP -> INIT */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	/* FINISHED -> INIT */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* START -> INIT FAIL */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	/* error tgid can not set INIT */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, -1);
	KUNIT_EXPECT_EQ(test, ret, false);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
}

/*
 * 测试用例描述：io_monitor状态机测试，本用例主要测试RESET状态设置功能是否正常
 * 	主要测试io_monitor_status_set接口
 * 预置条件：无论当前io_monitor状态如何，RESET调用都应保证成功
 * 参数描述：
 *  IO_MONITOR_STATUS_RESET：RESET状态标记
 *  IO_MONITOR_TYPE_READ：代表当前监控读操作，请注意目前仅支持监控读操作
 *  tgid: 待监控进程的tgid，Kunit测试用例中以current替代
 * 返回值描述：返回值为true代表设置成功，false代表设置失败
 */
static void io_monitor_status_chage_kunit_test_case1(struct kunit *test)
{
	bool ret = false;
	pid_t tgid = task_tgid_nr(current);
	io_monitor *monitor = get_io_monitor(IO_MONITOR_TYPE_READ);

	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, -1);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* INIT -> RESET */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* START -> RESET */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* STOP -> RESET */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);

	/* FINISH -> RESET */
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_INIT, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_START, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_STOP, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, true);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_FINISH, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
	ret = io_monitor_status_set(IO_MONITOR_STATUS_RESET, IO_MONITOR_TYPE_READ, tgid);
	KUNIT_EXPECT_EQ(test, ret, true);
	KUNIT_EXPECT_EQ(test, monitor->buf->should_fput, false);
}

/*
 * 测试用例描述：初始化io_monitor模块，本身是测试io_monitor_init接口是否正常，也是其它测试的基础
 * 预置条件：无
 * 参数描述：无
 * 返回值描述：返回值为true代表设置成功，false代表设置失败
 */
static void io_monitor_init_kunit_test_case(struct kunit *test)
{
	bool ret = false;

	ret = io_monitor_init();
	KUNIT_EXPECT_EQ(test, ret, true);
}

/*
 * 测试用例描述：释放io_monitor模块，本身是测试io_monitor_free接口是否正常，也是其它测试的基础
 * 预置条件：无
 * 参数描述：无
 * 返回值描述：固定返回true
 */
static void io_monitor_free_kunit_test_case(struct kunit *test)
{
	bool ret = true;

	io_monitor_free(get_io_monitor(IO_MONITOR_TYPE_READ));
	KUNIT_EXPECT_EQ(test, ret, true);
}

static struct kunit_case frr_kunit_test_cases[] = {
	KUNIT_CASE(io_monitor_status_chage_kunit_test_case1),
	KUNIT_CASE(io_monitor_status_chage_kunit_test_case2),
	KUNIT_CASE(io_monitor_status_chage_kunit_test_case3),
	KUNIT_CASE(io_monitor_status_chage_kunit_test_case4),
	KUNIT_CASE(io_monitor_status_chage_kunit_test_case5),
	KUNIT_CASE(io_monitor_status_chage_kunit_test_case6),
	KUNIT_CASE(io_monitor_record_kunit_test_case1),
	KUNIT_CASE(io_monitor_record_kunit_test_case2),
	KUNIT_CASE(io_monitor_record_kunit_test_case3),
	{}
};

static struct kunit_suite frr_kunit_test_suite = {
	.name = "frr_test_test_cases",
	.init = io_monitor_init_kunit_test_case,
	.exit = io_monitor_free_kunit_test_case,
	.test_cases = frr_kunit_test_cases,
};

kunit_test_suite(frr_kunit_test_suite);

MODULE_LICENSE("GPL v2");
