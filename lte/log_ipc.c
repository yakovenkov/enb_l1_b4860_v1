/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <smartdsp_os.h>
#include <lte_enodeb.h>
//#include <uec_lib.h>
#include <log.h>
#include <log_ipc.h>
#include <string.h>
#include <time.h>
#include <stdarg.h>
#include <secure_id.h>

#include "os_hw_timer.h"
#include "hw_timers.h"

#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_OPT_OFF
#pragma opt_level = "O3"
#endif

#define MAX_LOG_INFO 256
#define MAX_LOG_TOTAL 4096
#define MAX_LOG_MSGS 128

static uint8_t *data_log_bufs; //[MAX_LOG_MSGS][MAX_LOG_TOTAL] __attribute__((section(".local_data_ddr1_bss")));
static uint32_t data_log_ptr = 0;

static int8_t g_level_char[] =
	{ 'D', 'I', 'W', 'E' };

static log_info_cat_t log_info_cat[] =
	{ /* DTRX */
		{ .name = "TRX", .enabled = 1, .loglevel = LOGL_NONE, },
	/* DCONFIG */
		{ .name = "CONFIG", .enabled = 1, .loglevel = LOGL_NONE, },
	/* DRACH */
		{ .name = "RACH", .enabled = 1, .loglevel = LOGL_NONE, },
	/* DFAPI */
		{ .name = "FAPI", .enabled = 1, .loglevel = LOGL_NONE, } };

#define MAX_DARGS_B4860 64
#define MAX_DHEX_B4860	64
#define MAX_DBUF	1024

typedef struct
{
	uint32_t ready;
	uint32_t tick;
	log_comp_id_t comp;
	log_level_t level;
	uint32_t frame;
	uint32_t subframe;
	const char *file;
	const char *func;
	int line;
	const char *msg;
	int32_t nargs;
	int32_t args[MAX_DARGS_B4860];
//	int32_t hexlen;
//	uint8_t hex[MAX_DHEX_B4860];
} dentry_b4860_t;

typedef struct
{
	uint32_t flag;
	uint32_t tick;
	log_comp_id_t comp;
	log_level_t level;
	uint32_t frame;
	uint32_t subframe;
	//const char *file;
	//const char *func;
	int line;
	uint8_t msg[256];
	int32_t nargs;
	int32_t args[MAX_DARGS_B4860];
} dentry_ipc_t;

/*static */dentry_b4860_t g_log_dbuf[MAX_DBUF] __attribute__((section(".local_data_ddr0_cacheable_bss")));
/*static */uint32_t g_log_dbuf_ptr_wr = 0;
/*static */uint32_t g_log_dbuf_ptr_rd = 0;

#define TASK_LOG_IPC_STACK_SIZE	16384
static os_event_handle evt_new_log_entry;
static os_task_handle task_log_ipc_handle;

ARCH_DECLARE_STATIC_STACK(task_log_ipc_stack, TASK_LOG_IPC_STACK_SIZE);
// __attribute__((section(".local_data_ddr0_cacheable_bss")));
//static uint8_t task_log_ipc_stack[TASK_LOG_IPC_STACK_SIZE] __attribute__((section(".local_data_ddr0_cacheable_bss")));
//#pragma align task_log_ipc_stack ARCH_CACHE_LINE_SIZE

void log_ipc_task_body(os_task_arg arg);
dentry_b4860_t *get_dentry_wr();
dentry_b4860_t *get_dentry_rd();

static secure_id_t log_sec_id = SECURE_ID_NONE;

static os_timer_handle log_timer_handle;

static void log_timer_handler(os_hwi_arg timer_num)
{

}

void log_init(const lte_enodeb_t *enodeb)
{
	os_status status;
	os_task_init_param_t task_init_params;

	log_sec_id = get_secure_id();

	data_log_ptr = 0;

	g_log_dbuf_ptr_wr = 0;
	g_log_dbuf_ptr_rd = 0;

	/* find force timer 2 type 64bit HWT */
	log_timer_handle = 2;
	status = osHwTimerFindForce(HWT_SOC_64BIT | ((os_timer_handle) log_timer_handle));
	OS_ASSERT_COND(status == OS_SUCCESS);

	// Настройка таймера, разрешение 1мкс 
	status = osHwTimerCreate(log_timer_handle, OS_TIMER_FREE_RUN | OS_TIMER_CHAINED,
			(0xffffffff00000000ull | g_core_clock), 0, log_timer_handler, OS_HWI_PRIORITY0);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osHwTimerStart(log_timer_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	data_log_bufs = osMalloc(MAX_LOG_MSGS * MAX_LOG_TOTAL, OS_MEM_HET_DDR0_SHARED_CACHEABLE);

	OS_ASSERT_COND(data_log_bufs != NULL);

	status = osEventSemaphoreFind(&evt_new_log_entry);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventSemaphoreCreate(evt_new_log_entry, 0);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osTaskFind(&task_log_ipc_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	task_init_params.task_function = log_ipc_task_body;
	task_init_params.task_arg = (uint32_t) 0;
	task_init_params.top_of_stack = (uint32_t) task_log_ipc_stack;
	task_init_params.stack_size = TASK_LOG_IPC_STACK_SIZE;
	task_init_params.task_priority = TASK_PRIORITY_IPC_LOG;
	task_init_params.task_name = "IPC log task";
	task_init_params.private_data = 0;

	status = osTaskCreate(task_log_ipc_handle, &task_init_params);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osTaskActivate(task_log_ipc_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);
}

uint32_t log_get_timer_value()
{
	os_timer_interval val;
	osHwTimerValueGet(log_timer_handle, &val);

	return 0xffffffff - (val >> 32);
}

static uint8_t *get_log_buffer()
{
	uint8_t *buf;

	osHwiSwiftDisable();

	buf = data_log_bufs + data_log_ptr * MAX_LOG_TOTAL;

	data_log_ptr++;

	if (data_log_ptr >= MAX_LOG_MSGS)
		data_log_ptr = 0;

	osHwiSwiftEnable();

	return buf;
}

os_status log_set_level(log_comp_id_t comp, log_level_t level)
{
	if (level >= LOGL_DEBUG && level <= LOGL_NONE && comp >= DTRX && comp < DLASTCOMP)
		log_info_cat[comp].loglevel = level;

	return OS_SUCCESS;
}

#define BOUND_ARG(b) b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], \
		b[10], b[11], b[12], b[13], b[14], b[15], b[16], b[17], b[18], b[19], \
		b[20], b[21], b[22], b[23], b[24], b[25], b[26], b[27], b[28], b[29], \
		b[30], b[31]

#define byte2char(b) (((b) >= 0x00 && (b) <= 0x09) ? ((b) + '0') : ((b) - 0x0a + 'a'))

void log_ipc_e(dentry_b4860_t *e)
{
	int32_t len;
	int8_t g_buff_info[MAX_LOG_INFO];
	int8_t *g_buff_total;
	fapi_ipc_msg_t *msg;
	
	msg = (fapi_ipc_msg_t *) get_log_buffer();
	msg->channel_id = FAPI_CHANNEL_LOG;
	msg->body_addr = ALIGN_ADDRESS((uint32_t)(uint8_t *) msg + sizeof(fapi_ipc_msg_t), 16);
	
	g_buff_total = (int8_t *) msg->body_addr;
	
	g_buff_total[0] = '\0';
	
	len = snprintf(g_buff_info, MAX_LOG_INFO - 1, e->msg, BOUND_ARG(e->args));
	uint32_t t = ((uint32_t) e->tick);
	
	/* Print without filename */
	len += snprintf(g_buff_total, MAX_LOG_TOTAL - 1, "[%c:%06u:%04u:%u:%s] [%s:%d] ", g_level_char[e->level], t, e->frame,
		e->subframe, log_info_cat[e->comp].name, e->func, e->line);
	
	strncat(g_buff_total, g_buff_info, MAX_LOG_TOTAL - 1);
	
	msg->length = MAX_LOG_TOTAL;
	
	ipc_send_msg(msg);
}

void log_ipc_task_body(os_task_arg arg)
{
	dentry_b4860_t *e;
	while (1)
	{
		while ((e = get_dentry_rd()) == NULL)
		{
			osEventSemaphorePend(evt_new_log_entry, 1);
			//osTaskDelay(1);
		}
	
		if (e == NULL)
			continue;
	
		log_ipc_e(e);
	
		e->ready = 0;
	}
}
	
dentry_b4860_t *get_dentry_wr()
{
	dentry_b4860_t *e;
	uint32_t tmp_ptr_wr;
	
	osHwiSwiftDisable();
	
	e = &g_log_dbuf[g_log_dbuf_ptr_wr];
	tmp_ptr_wr = (g_log_dbuf_ptr_wr + 1) & (MAX_DBUF - 1);
	
	if (tmp_ptr_wr == g_log_dbuf_ptr_rd || e->ready != 0)
		e = NULL;
	else
		g_log_dbuf_ptr_wr = tmp_ptr_wr;
	
	osHwiSwiftEnable();
	
	return e;
}

dentry_b4860_t *get_dentry_rd()
{
	dentry_b4860_t *e = NULL;
	
	osHwiSwiftDisable();
	if (g_log_dbuf_ptr_rd != g_log_dbuf_ptr_wr)
	{
		e = &g_log_dbuf[g_log_dbuf_ptr_rd];
	
		if (e->ready)
			g_log_dbuf_ptr_rd = (g_log_dbuf_ptr_rd + 1) & (MAX_DBUF - 1);
		else
			e = NULL;
	}
	osHwiSwiftEnable();
	
	return e;
}

void raw_dprintf_line(int32_t nargs, log_comp_id_t comp, log_level_t level, uint64_t tick, uint32_t frame,
		uint32_t subframe, const char *file, const char *func, int32_t line, va_list args)
{
	int i;
	dentry_b4860_t *e = get_dentry_wr();
	
	if (!e)
	return;
	
	e->comp = comp;
	e->level = level;
	e->tick = tick;
	e->frame = frame;
	e->subframe = subframe;
	e->file = file;
	e->func = func;
	e->line = line;
	
	e->msg = va_arg(args, const char*);
	//	e->hexlen = 0;
	
	if (nargs > 1)
	{
		e->nargs = nargs - 1;
		for (i = 0; i < nargs - 1; i++)
		{
			e->args[i] = va_arg(args, int32_t);
		}
	}
	else
		e->nargs = 0;
	
	e->ready = 1;
}

void raw_dprintf_line_direct(int32_t nargs, log_comp_id_t comp, log_level_t level, uint64_t tick, uint32_t frame,
		uint32_t subframe, const char *file, const char *func, int32_t line, const char *msg, va_list args)
{
	int i;
	
	int32_t len;
	fapi_ipc_msg_t *ipc_msg;
	dentry_ipc_t *e;
	uint64_t phys_addr;
	
	ipc_msg = (fapi_ipc_msg_t *) get_log_buffer();
	ipc_msg->channel_id = FAPI_CHANNEL_LOG;
	ipc_msg->body_addr = ALIGN_ADDRESS((uint32_t)(uint8_t *) ipc_msg + sizeof(fapi_ipc_msg_t), 16);
	e = (dentry_ipc_t *) ipc_msg->body_addr;
	
	e->flag = 1;
	e->tick = tick;
	e->comp = comp;
	e->level = level;
	e->line = line;
	e->frame = frame;
	e->subframe = subframe;
	
	uint32_t *src32 = (uint32_t *) msg;
	uint32_t *dst32 = (uint32_t *) e->msg;
	
	for (int i = 0; i < 256 / (16 * 4); i++)
	{
		Word32 dc[16];
		__ld_16l(src32, &dc[0], &dc[1], &dc[2], &dc[3], &dc[4], &dc[5], &dc[6], &dc[7], &dc[8], &dc[9], &dc[10], &dc[11],
		&dc[12], &dc[13], &dc[14], &dc[15]);
		__st_16l(dst32, dc[0], dc[1], dc[2], dc[3], dc[4], dc[5], dc[6], dc[7], dc[8], dc[9], dc[10], dc[11], dc[12], dc[13],
		dc[14], dc[15]);
		
		src32 += 16;
		dst32 += 16;
	}

	e->msg[255] = 0;
	
	e->args[0] = e->tick;
	e->args[1] = e->frame;
	e->args[2] = e->subframe;
	e->nargs = 3;
	
	if (nargs > MAX_DARGS_B4860 - 3)
	nargs = MAX_DARGS_B4860 - 3;
	
	if (nargs > 0)
	{
		e->nargs += nargs;
		for (i = 0; i < nargs; i++)
		{
			e->args[i + 3] = va_arg(args, int32_t);
		}
	}
	
	ipc_msg->length = MAX_LOG_TOTAL;
	
	ipc_send_msg(ipc_msg);
}

void log_line_string(int nargs, log_comp_id_t comp, log_level_t level, const int8_t *file, const int8_t *func,
		int32_t line, ...)
{
	va_list args;
	uint32_t frame, subframe;
	
	uint64_t tick = log_get_timer_value();
	
	if (level < log_info_cat[comp].loglevel)
		return;

	osHwiSwiftDisable();
	frame = g_enodeb_inst[0].system_frame_no;
	subframe = g_enodeb_inst[0].system_subframe_no;
	osHwiSwiftEnable();
	
	va_start(args, line);
	
	raw_dprintf_line(nargs, comp, level, tick, frame, subframe, file, func, line, args);
	
	va_end(args);

	osEventSemaphorePost(evt_new_log_entry, NULL);
}

void log_line_direct(int nargs, log_comp_id_t comp, log_level_t level, const int8_t *file, const int8_t *func,
		int32_t line, const char *msg, ...)
{
	va_list args;
	uint32_t frame, subframe;
	
	uint64_t tick = log_get_timer_value();
	
	if (level < log_info_cat[comp].loglevel)
		return;
	
	osHwiSwiftDisable();
	frame = g_enodeb_inst[0].system_frame_no;
	subframe = g_enodeb_inst[0].system_subframe_no;
	osHwiSwiftEnable();
	
	va_start(args, msg);
	
	raw_dprintf_line_direct(nargs, comp, level, tick, frame, subframe, file, func, line, msg, args);
	
	va_end(args);
}
