/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <smartdsp_os.h>
#include <lte_enodeb.h>
//#include <maple_lib.h>
#include <trx_lte.h>
#include <liblte_msc8157.h>
//#include <uec_lib.h>
#include <log.h>
#include <trans.h>
#include <math.h>
#include <string.h>

#include "maple_pe_init.h"
#include "maple.h"
#include "maple_init.h"
#include "maple_pdsch_init.h"
#include "maple_pdsch.h"

//#include "maple_map.h"

#include <dsp/dsp_kernels.h>

#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_MAPLE_OUTPUT && !defined DEBUG_OPT_OFF && !defined(PUSCH_TEST)
#pragma opt_level = "O3"
#else
//#pragma opt_level = "O3"
#endif

//static uint8_t mem_lte_subframe_bufs[MEM_PART_SIZE(TRX_NUM_OF_SLOTS)];
lte_subframe_t *data_lte_dl_subframe_pool[TRX_NUM_OF_SLOTS] __attribute__((aligned(ARCH_CACHE_LINE_SIZE)));

/* Пул сабфреймов для приема аплинка */
lte_ul_subframe_t *data_lte_ul_subframe_pool[TRX_NUM_OF_UL_JOBS] __attribute__((aligned(ARCH_CACHE_LINE_SIZE)));

lte_ul_pusch_descr_t *data_lte_ul_pusch_descr_pool[TRX_NUM_OF_PUSCH_JOBS] __attribute__((aligned(ARCH_CACHE_LINE_SIZE)));

uint8_t ul_harq_data_buf[TRX_NUM_OF_UL_HARQ_BUFFERS][TRX_UL_HARQ_BUFFER_SIZE] __attribute__((section(".shared_data_ddr0_cacheable_bss")));
lte_ul_harq_descr_t ul_harq_descr_buf[65536] __attribute__((section(".shared_data_ddr0_cacheable_bss")));

ARCH_DECLARE_STATIC_STACK(trx_ul_stacks, TRX_UL_STACK_SIZE);// __attribute__((section(".local_data_ddr0_cacheable_bss")));

ARCH_DECLARE_STATIC_STACK(trx_dl_stacks, TRX_DL_STACK_SIZE);// __attribute__((section(".local_data_ddr0_cacheable_bss")));

ARCH_DECLARE_STATIC_STACK(trx_maple_pdsch_stacks, TRX_MAPLE_PDSCH_STACK_SIZE) __attribute__((section(".local_data_ddr0_cacheable_bss")));

ARCH_DECLARE_STATIC_STACK(trx_slot_rx_stacks, TRX_BURST_RX_STACK_SIZE) __attribute__((section(".local_data_ddr0_cacheable_bss")));

ARCH_DECLARE_STATIC_STACK(trx_prach_rx_stacks, TRX_PRACH_RX_STACK_SIZE) __attribute__((section(".local_data_ddr0_cacheable_bss")));

#ifdef DEBUG_TX_BUFFER_TEST1
/* Отладочный буфер для выходных отсчетов */

/* Количество сэмплов для отладки */
#define TRX_DEBUG_BUFFER_SAMPLES 15360 * 10

/* Отладочныцй буфер */
Complex16 debug_tx_buffer[NUM_TRX][TRX_DEBUG_BUFFER_SAMPLES] __attribute__((section(".shared_data_ddr1_bss")));
#pragma align debug_tx_buffer ARCH_CACHE_LINE_SIZE

#endif

#if defined(DEBUG_TX_CPRI) || defined(DEBUG_PDPE_OUTPUT) || defined(DEBUG_TX_CPRI_ON_DEMAND)
/* Отладочный буфер для выходных отсчетов */
/* Количество сэмплов для отладки */
#define DEBUG_TX_BUFFER_SIZE (30720 * 100 * sizeof(Complex16))
uint8_t debug_tx_buffer[DEBUG_TX_BUFFER_SIZE] __attribute__((section(".shared_data_ddr0_cacheable_bss"), aligned(ARCH_CACHE_LINE_SIZE)));
uint32_t debug_tx_buffer_ptr = 0;

#ifdef DEBUG_PDPE_OUTPUT
int32_t pdsch_pdpe_dump = 0;
int32_t pdsch_pdpe_dump_cpri = 0;
#endif

#ifdef DEBUG_TX_CPRI_ON_DEMAND
int32_t pdsch_dump_cpri_tx = 0;
int32_t pdsch_dump_cpri_tx_count = 0;
#endif
#endif

#ifdef DEBUG_MAPLE_OUTPUT
/* Отладочный буфер для выходных отсчетов MAPLE */
/* Отладочныцй буфер */
Complex16 trx_maple_debug_buffer[TRX_MAPLE_DEBUG_BUFFER_SIZE] __attribute__((section(".log_event_ddr1")));
#pragma align trx_maple_debug_buffer ARCH_CACHE_LINE_SIZE

/* Указатель на текущую позицию в отладочном буфере */
static uint32_t trx_maple_debug_buffer_ptr;
#endif

uint64_t trx_cycle_counters[TRX_CYCLES_MAX][TRX_CYCLE_NUM_LOOPS];
lte_prach_t rx_prach_buffer[LTE_PRACH_RX_SUBFRAMES] __attribute__((section(".local_data_ddr0_cacheable_bss"), aligned(16)));

static os_status trx_init_ul_task(lte_trx_t *trx)
{
	os_status status;
	os_task_init_param_t task_init_params;
	int32_t i;

	OS_ASSERT_COND(trx != NULL);

	status = osEventQueueFind(&trx->evq_pusch_ready);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventQueueCreate(trx->evq_pusch_ready, TRX_NUM_OF_PUSCH_READY_JOBS);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventQueueFind(&trx->evq_pusch_ctrl_ready);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventQueueCreate(trx->evq_pusch_ctrl_ready, TRX_NUM_OF_TVPE_READY_JOBS);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventQueueFind(&trx->evq_tvpe_ready);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventQueueCreate(trx->evq_tvpe_ready, TRX_NUM_OF_TVPE_READY_JOBS);
	OS_ASSERT_COND(status == OS_SUCCESS);

	// Инициализация UL HARQ буферов
	trx->ul_harq_descr = ul_harq_descr_buf;
	for(i=0; i<TRX_NUM_OF_UL_HARQ_BUFFERS; i++)
	{
		trx->ul_harq_buf[i] = ul_harq_data_buf[i];
	}
	
	status = osTaskFind(&trx->task_ul_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	//ARCH_DECLARE_STATIC_STACK(trx_ul_stacks, TRX_UL_STACK_SIZE) __attribute__((section(".local_data_ddr0_cacheable_bss")));

	trx->task_ul_stack = trx_ul_stacks;
	//trx->task_ul_stack = osAlignedMalloc(TRX_UL_STACK_SIZE, OS_MEM_DDR0_LOCAL_CACHEABLE, ARCH_STACK_ALIGN(TRX_UL_STACK_SIZE));

	task_init_params.task_function = trx_ul_task;
	task_init_params.task_arg = (uint32_t) trx;
	task_init_params.top_of_stack = (uint32_t) trx->task_ul_stack;
	task_init_params.stack_size = TRX_UL_STACK_SIZE;
	task_init_params.task_priority = TASK_PRIORITY_UL_TASK;
	task_init_params.task_name = "TRX ULSCH/ULCCH task";
	task_init_params.private_data = 0;

	status = osTaskCreate(trx->task_ul_handle, &task_init_params);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osTaskActivate(trx->task_ul_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	return OS_SUCCESS;
}

static os_status trx_init_dl_task(lte_trx_t *trx)
{
	os_status status;
	os_task_init_param_t task_init_params;
	int32_t i;

	OS_ASSERT_COND(trx != NULL);
	
	for(i=0; i<10; i++)
	{
		trx->fapi_dl_config[i] = NULL;
		trx->fapi_ul_config[i] = NULL;
		trx->fapi_tx_req[i] = NULL;
		trx->fapi_hi_dci0[i] = NULL;
	}

	/* Создание задачи шедулера данных L1 */
	status = osTaskFind(&trx->task_dl_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	trx->task_dl_stack = trx_dl_stacks;

	task_init_params.task_function = trx_dl_task;
	task_init_params.task_arg = (uint32_t) trx;
	task_init_params.top_of_stack = (uint32_t) trx->task_dl_stack;
	task_init_params.stack_size = TRX_DL_STACK_SIZE;
	task_init_params.task_priority = TASK_PRIORITY_DL_TASK;
	task_init_params.task_name = "TRX DLSCH/DLCCH task";
	task_init_params.private_data = 0;

	status = osTaskCreate(trx->task_dl_handle, &task_init_params);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osTaskActivate(trx->task_dl_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

#ifdef USE_PDSCH_QUEUE	
	/* Поток обработки MAPLE PDSCH */
	status = osTaskFind(&trx->task_maple_pdsch_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	trx->task_maple_pdsch_stack = trx_maple_pdsch_stacks[trx->no];

	task_init_params.task_function = trx_maple_pdsch;
	task_init_params.task_arg = (uint32_t) trx;
	task_init_params.top_of_stack = (uint32_t) trx->task_maple_pdsch_stack;
	task_init_params.stack_size = TRX_MAPLE_PDSCH_STACK_SIZE;
	task_init_params.task_priority = OS_TASK_PRIORITY_11;
	task_init_params.task_name = "TRX MAPLE PDSCH task";
	task_init_params.private_data = 0;

	status = osTaskCreate(trx->task_maple_pdsch_handle, &task_init_params);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osTaskActivate(trx->task_maple_pdsch_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);
#endif
	return OS_SUCCESS;
}

static os_status trx_init_trx_tasks(lte_trx_t *trx)
{
	os_status status;
	os_task_init_param_t task_init_params;

	OS_ASSERT_COND(trx != NULL);

	/* Событие индикатора нового сбафрейма */
	/*
	 status = osEventQueueFind(&trx->evq_subframe_ind);
	 OS_ASSERT_COND(status == OS_SUCCESS);

	 status = osEventQueueCreate(trx->evq_subframe_ind, TRX_NUM_OF_SLOTS);
	 OS_ASSERT_COND(status == OS_SUCCESS);
	 */

	status = osEventSemaphoreFind(&trx->evt_subframe_ind);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventSemaphoreCreate(trx->evt_subframe_ind, 0);
	OS_ASSERT_COND(status == OS_SUCCESS);

	/* 
	 * Создание задачи обрабокти принятых PRACH 
	 */
	status = osEventQueueFind(&trx->evq_prach_rx);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventQueueCreate(trx->evq_prach_rx, TRX_NUM_OF_PRACH_MSGS);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventQueueFind(&trx->evq_ul_pufft_ready);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventQueueCreate(trx->evq_ul_pufft_ready, TRX_NUM_OF_PRACH_MSGS);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osTaskFind(&trx->task_prach_rx_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	trx->task_prach_rx_stack = trx_prach_rx_stacks;

	task_init_params.task_function = trx_prach_task;
	task_init_params.task_arg = (uint32_t) trx;
	task_init_params.top_of_stack = (uint32_t) trx->task_prach_rx_stack;
	task_init_params.stack_size = TRX_PRACH_RX_STACK_SIZE;
	task_init_params.task_priority = TASK_PRIORITY_PRACH_TASK;
	task_init_params.task_name = "TRX RX PRACH task";
	task_init_params.private_data = 0;

	status = osTaskCreate(trx->task_prach_rx_handle, &task_init_params);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osTaskActivate(trx->task_prach_rx_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	return OS_SUCCESS;
}

void trx_create_dl_subframe_pool()
{
	int32_t i;

	for (i = 0; i < TRX_NUM_OF_SLOTS; i++)
	{
		//lte_subframe_t *sf = osAlignedMalloc(sizeof(lte_subframe_t), OS_MEM_DDR0_LOCAL_CACHEABLE, ARCH_CACHE_LINE_SIZE);
		lte_subframe_t *sf = osAlignedMalloc(sizeof(lte_subframe_t), OS_MEM_DDR0_SHARED_CACHEABLE, ARCH_CACHE_LINE_SIZE);
		OS_ASSERT_COND(sf != NULL);
		
		
		/*
		 sf->job = osAlignedMalloc(sizeof(lte_pdsch_job_t), OS_MEM_DDR1_LOCAL, ARCH_CACHE_LINE_SIZE);
		 OS_ASSERT_COND(sf->job != NULL);
		 
		 sf->pdcch_syms = osAlignedMalloc(PDCCH_NUM_MAX * PDCCH_SIZE_MAX, OS_MEM_DDR1_LOCAL, ARCH_CACHE_LINE_SIZE);
		 OS_ASSERT_COND(sf->pdcch_syms != NULL);
		 
		 sf->tb_map = osAlignedMalloc(64 * sizeof(maple_pdsch_tb_header_t), OS_MEM_DDR1_LOCAL, ARCH_CACHE_LINE_SIZE);
		 OS_ASSERT_COND(sf->tb_map != NULL);
		 
		 sf->cw_map = osAlignedMalloc(64 * sizeof(maple_pdsch_cw_header_t), OS_MEM_DDR1_LOCAL, ARCH_CACHE_LINE_SIZE);
		 OS_ASSERT_COND(sf->cw_map != NULL);
		 */
		data_lte_dl_subframe_pool[i] = sf;
	}
}

void trx_create_ul_subframe_pool()
{
	int32_t i;

	for (i = 0; i < TRX_NUM_OF_UL_JOBS; i++)
	{
		//lte_ul_subframe_t *sf = osAlignedMalloc(sizeof(lte_ul_subframe_t), OS_MEM_DDR0_LOCAL_CACHEABLE, ARCH_CACHE_LINE_SIZE);
		lte_ul_subframe_t *sf = osAlignedMalloc(sizeof(lte_ul_subframe_t), OS_MEM_DDR0_SHARED_CACHEABLE, ARCH_CACHE_LINE_SIZE);
		OS_ASSERT_COND(sf != NULL);
		data_lte_ul_subframe_pool[i] = sf;
	}

	for (i = 0; i < TRX_NUM_OF_PUSCH_JOBS; i++)
	{
		//lte_ul_pusch_descr_t *descr = osAlignedMalloc(sizeof(lte_ul_pusch_descr_t), OS_MEM_DDR0_LOCAL_CACHEABLE, ARCH_CACHE_LINE_SIZE);
		lte_ul_pusch_descr_t *descr = osAlignedMalloc(sizeof(lte_ul_pusch_descr_t), OS_MEM_DDR0_SHARED_CACHEABLE, ARCH_CACHE_LINE_SIZE);
		OS_ASSERT_COND(descr != NULL);

		descr->cop_job.job_id = (void *) (i & 0xff);
		descr->cop_job.device_specific = &descr->pusch_bd;
		descr->pusch_bd.user_status_ptr = &descr->status;

		data_lte_ul_pusch_descr_pool[i] = descr;
	}
}

os_status trx_init_mod_slots(lte_trx_t *trx)
{
	os_status status;
	OS_ASSERT_COND(trx != NULL);

	//trx->mem_lte_subframe_buf = mem_lte_subframe_bufs;
	trx_create_dl_subframe_pool();
	trx_create_ul_subframe_pool();

	trx->lte_dl_subframe_pool = data_lte_dl_subframe_pool;
	trx->lte_dl_subframe_pool_ptr = 0;

	trx->lte_ul_subframe_pool = data_lte_ul_subframe_pool;
	trx->lte_ul_subframe_pool_ptr = 0;

	trx->ul_pusch_pool = data_lte_ul_pusch_descr_pool;
	trx->ul_pusch_pool_ptr = 0;

	status = osQueueFind(&trx->evq_tx_mod_subframe, FALSE);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osQueueCreate(trx->evq_tx_mod_subframe, TRX_NUM_OF_SLOTS / 2);
	OS_ASSERT_COND(status == OS_SUCCESS);

	return OS_SUCCESS;
}

os_status lte_trx_init(lte_enodeb_t *enodeb)
{
	os_status status;
	lte_trx_t *trx = &enodeb->trx;

	OS_ASSERT_COND(trx != NULL);
	OS_ASSERT_COND(enodeb != NULL);

#ifdef DEBUG_MAPLE_OUTPUT
	trx_maple_debug_buffer_ptr = 0;
#endif

	memset(trx, 0, sizeof(lte_trx_t));

	trx->enodeb = enodeb;
	trx->frame_no_rx = 0;
	trx->subframe_no_rx = 0;
	//trx->sym_no_rx = 0;

	trx->num_reaped_jobs = 0;
	trx->num_disp_jobs = 0;
	trx->maple_job_id = 0;

	/*
	 trx->cur_rx_burst = NULL;
	 trx->cur_rx_burst_pos = 0;
	 trx->cur_tx_burst = NULL;
	 trx->cur_tx_burst_pos = 0;
	 */

	/* Задержка по-умолчанию 1 символ (без учета задержек тракта) */
	trx->max_expected_delay = 5;
	//trx->tsc_min_power = 100;
	
	trx->rx_prach_buffer = rx_prach_buffer;

	status = trx_init_mod_slots(trx);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = trx_init_ul_task(trx);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = trx_init_dl_task(trx);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = trx_init_trx_tasks(trx);
	OS_ASSERT_COND(status == OS_SUCCESS);
	/*
	 status = trx_dsp_init(trx);
	 OS_ASSERT_COND(status == OS_SUCCESS);
	 */

	return OS_SUCCESS;
}
