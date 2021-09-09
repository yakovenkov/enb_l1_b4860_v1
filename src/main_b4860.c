/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include "smartdsp_os.h"
#include "os_config.h"
//#include "os_cache.h"
#include "sc39xx_cache.h"
#include "heterogeneous/b486x_heterogeneous.h"
#include "heterogeneous/b486x_heterogeneous_ipc.h"
#include "ipc/b486x_ipc_init.h"
#include "ipc/b486x_ipc.h"
#include "b486x_heterogeneous.h"
#include "b486x_heterogeneous_ipc.h"
#include "smartdsp_os_device.h"
#include "memmap.h"
#include "maple_pe_init.h"
#include "maple.h"
#include "maple_init.h"
#include "maple_pdsch_init.h"
#include "maple_pdsch.h"
#include <maple_ftpe.h>
#include <maple_ftpe_init.h>
#include "maple_pufft.h"
#include "maple_pufft_init.h"
#include "maple_pusch.h"
#include "maple_pusch_init.h"
#include "maple_trace.h"
#include "maple_tvpe.h"
#include "maple_tvpe_init.h"

#include "hw_timers32_init.h"
#include "hw_timers32.h"

#include "cpri.h"

//#include "maple_map.h"

#include "app_config.h"
#include <log.h>

#define ANT2_TEST_GPO89
#define SF_IND_UL_TTI_IRQ
#define USE_TVPE

//#define DEBUG_STUB

#include "cpri_b4860.h"

#include <lte_enodeb.h>
#include <log.h>

//#define NO_PRINT
#ifdef NO_PRINT
#define printf(...)
#else
#include <stdio.h>
#endif
#include <string.h>

//#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_MAPLE_OUTPUT && !defined DEBUG_OPT_OFF
#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_OPT_OFF
#pragma opt_level = "O3"
#endif

// Raw IQ RX buffer for 10 subframes
Complex16 rx_buffer_raw[MAPLE_NUM_SECTORS][MAPLE_NUM_ANT][10 * LTE_SAMPLES_PER_SUBFRAME_MAX]  __attribute__((section(".shared_data_ddr0_cacheable_bss")));

sio_channel_t rx_channels[LTE_N_ANTENNAS_MAX];
sio_channel_t tx_channels[LTE_N_ANTENNAS_MAX];

uint32_t tti_ul_count = 0;
uint32_t tti_dl_count = 0;

static volatile int32_t g_ipc_pa_ready = 0;

fapi_ipc_cfg_t g_fapi_ipc_cfg = {0};

mem_manager_t mem_mngr[NUM_MEM_MNGR] =
	{ 0 };

uint8_t *results = NULL;

//void *ack_addr[PDSCH_NUM_ANT];

volatile uint32_t num_reaped_jobs = 0;
volatile uint32_t num_disp_jobs = 0;

cop_dev_handle maple_handle[NUM_MAPLES_USED] = { 0 };
cop_dev_handle pdsch_handle[NUM_PDSCH_USED] = { 0 };
cop_channel_t pdsch_ch_handle[LTE_N_ANTENNAS_MAX];

cop_dev_handle      pufft_handle[NUM_PUFFTS_USED] = { 0 };
cop_channel_t       pufft_ch_handle[NUM_PUFFTS_USED] = { 0 };

cop_dev_handle      pusch_handle[NUM_PUSCH_USED];
cop_channel_t       pusch_ch_handle[LTE_N_ANTENNAS_MAX];

static cop_dev_handle ftpe_handle;
static cop_channel_t ftpe_ch_handle;
static os_event_handle evt_fft_done;
static uint32_t ftpe_dispatch_fail_counter = 0;
static cop_dev_handle         tvpe_handle[NUM_TVPE_USED]    = {0};
cop_channel_t          tvpe_ch_handle[NUM_TVPE_USED] = {0};

typedef struct
{
	cop_job_handle cop_job;
	maple_ftpe_job_t ftpe_job;
} fft_ifft_job_t;
static fft_ifft_job_t fft_ifft_job __attribute__((section(".local_data_ddr0_cacheable_bss")));
#pragma align fft_ifft_job    16

// FIXME: SC3900 workaround
#define PDSCH_NUM_ANT	8
volatile uint32_t sym_int_count[PDSCH_NUM_ANT] =
	{ 0 };

//static void fill_pufft_job_buf_addr(lte_enodeb_t *enodeb, lte_ul_subframe_t *sf, Complex16 *buf_in);
void fill_pufft_job_buf_addr(lte_enodeb_t *enodeb, lte_ul_subframe_t *sf, int32_t buf_no);

/***************************************************************************
 * IPC variables
 ***************************************************************************/

static void *ipc_ch[NUM_IPC_CH];
static volatile uint32_t ipc_wait_pa_ready;
static uint8_t ipc_buf_manager[MEM_PART_SIZE(FAPI_IPC_MAX_PA2SC_MSGS_NUM)];
static uint8_t *ipc_buf_space;
static os_mem_part_t *ipc_buf_pool;

/***************************************************************************/

/**** MAPLE-B3 ********/
/*
 * Makes sure that the order of this array is aligned with maple_pufft_init_params
 * and maple_init_params
 */
static const void * maple_mbus[] = {
#if (MAPLE_0_PUFFT == ON)
                (const void*)((void*)SOC_DSP_MAPLE_MBUS_DEFAULT),
#endif
#if (MAPLE_1_PUFFT == ON)
                (const void*)((void*)(SOC_DSP_MAPLE_MBUS_DEFAULT + 0x400000)),
#endif
#if (MAPLE_2_PUFFT == ON)
                (const void*)((void*)(SOC_DSP_MAPLE_MBUS_DEFAULT + 0x800000))
#endif
};

static const void * maple_sbus[] = {
#if (MAPLE_0_PUFFT == ON)
                (const void*)(void*)((uint32_t *)(SOC_DSP_MAPLE_DEFAULT)),
#endif
#if (MAPLE_1_PUFFT == ON)
                (const void*)(void*)((uint32_t *)(SOC_DSP_MAPLE_DEFAULT + 0x10000)),
#endif
#if (MAPLE_2_PUFFT == ON)
                (const void*)(void*)((uint32_t *)(SOC_DSP_MAPLE_DEFAULT + 0x20000))
#endif
};

#if (MAPLE_0_PUFFT == ON)
static const os_hwi_handle  maple_int_base[] = {
                OS_INT_MAPLE_0_CH_0,
#endif
#if (MAPLE_1_PUFFT == ON)
                OS_INT_MAPLE_1_CH_0,
#endif
#if (MAPLE_2_PUFFT == ON)
                OS_INT_MAPLE_2_CH_0
#endif
};

/*****************/
/* PDSCH configuration */
/*
uint32_t MPSCPs[MAPLE_NUM_SECTORS] =
{
		PDSCH_USE_1_FTPE | PDSCH_SECTOR_BA_ALOCATION_0 | PDSCH_SECTOR_RES_ALOCATION_1 
		| PDSCH_MODE_BW_5MHZ | PDSCH_NUM_VIRT_ANT_1 | PDSCH_NUM_PHYS_ANT_1 | PDSCH_NUM_PBCH_ANT_PORTS_2 
		| PDSCH_NUM_CELL_REF_PORTS_1
};
*/
/*****************/
/* PDSCH configuration */
uint8_t MPTACs[MAPLE_NUM_SECTORS][16] =
{
	/* Secotr # 0 */
	0x1, 	0x1, 	0x0, 	0xd,
	0x0, 	0x0, 	0x0, 	0x0,
	0x0, 	0x0, 	0x0, 	0x0,
	0x1, 	0x1, 	0x0, 	0x0
};
/*****************/

uint8_t core_mmu_task_id = 0;

extern dsp_plat_map_t  *g_dsp_plat_map;
extern uint8_t          g_heap_ddr0_cacheable[];
extern uint8_t          g_shared_mem_heap_ddr0_cacheable[];
extern uint8_t          g_shared_mem_heap_ddr0_noncacheable[];

//Define soc timers for maple tmr_int_grp 0-3:
soc_timer32_num_t app_timer_alloc[3][4]  __attribute__((section(".os_shared_data_cacheable"))) = {
		 // was
		 //{ SOC_TIMER_CPRI_RX , SOC_TIMER32_21, SOC_TIMER32_26, SOC_TIMER32_31 }
		 // my version
		{ SOC_TIMER_CPRI_RX , SOC_TIMER32_1, SOC_TIMER32_2, SOC_TIMER32_3 }
		 
		 // was
		 //,{ SOC_TIMER32_0 , SOC_TIMER32_5, SOC_TIMER32_10, SOC_TIMER32_15 }
		 // my version
		 ,{ SOC_TIMER32_16 , SOC_TIMER32_21, SOC_TIMER32_26, SOC_TIMER32_31 }
		 
		,{ SOC_TIMER32_8 , SOC_TIMER32_13, SOC_TIMER32_18, SOC_TIMER32_23 } };

maple_lte_mbus_memmap_t * g_maple0_mbus = (maple_lte_mbus_memmap_t *)0x79000000;

os_virt_ptr ant_in_virt_base[MAPLE_NUM_SECTORS][MAPLE_NUM_ANT] OS_ATT_HEAP(".shared_data_ddr0_cacheable_bss");
/**< Keeps antenna input virtual base addresses for all sectors */
volatile uint32_t timer_sets_ack[PUFFT_SOC_TIMER_SETS] OS_ATT_HEAP(".shared_data_ddr0_cacheable_bss");
/**< &timer_sets_ack[x] is soc_timer_conf[x].ack_addr, timer_sets_ack[x] will hold ack data as written by MAPLE;
 *   All SkyBlue registers of the SoC should be accesses using a fixed steering value (4�h3) which does not go through any of the M2 cluster,
 *   but directly to the CHB fabric and from there to SkyBlue tree. This steering value should be fixed and coded in the uCode.*/

#if (USING_MAPLE_MMU == FALSE)
os_phys_ptr ant_in_phys_base[TEST_NUM_SECTORS][TEST_NUM_ANT] OS_ATT_HEAP(".shared_data_ddr0_cacheable_bss");
/**< Keeps antenna input physical base addresses for all sectors */
#endif
/***************************************************************************/

void sweep_cache(uint32_t addr, int size, uint32_t mode)
{
	// FIXME: SC3900
#if 0
	uint32_t cache_hi, cache_lo;
	cache_hi = CACHE_OP_HIGH_ADDR(addr, size, ARCH_CACHE_LINE_SIZE);
	cache_lo = CACHE_OP_LOW_ADDR(addr, ARCH_CACHE_LINE_SIZE);

#if (L2CACHE_ENABLE == ON)
	if (mode == CACHE_FLUSH)
	{
		osCacheL1L2DataFlush((const os_virt_ptr) cache_lo, (const os_virt_ptr) cache_hi, MMU_SHARED_PID);
	}
	else
	{
		osCacheL1L2DataSweep((const os_virt_ptr) cache_lo, (const os_virt_ptr) cache_hi, MMU_SHARED_PID, mode);
	}
#elif (DCACHE_ENABLE == ON)
	osCacheDataSweep((const os_virt_ptr)cache_lo, (const os_virt_ptr)cache_hi, MMU_SHARED_PID, mode);
#endif    /* L2CACHE_ENABLE == ON */
#endif
}

void sweep_cache_async(uint32_t addr, int size, uint32_t mode)
{
	// FIXME: SC3900
#if 0
	uint32_t cache_hi, cache_lo;
	cache_hi = CACHE_OP_HIGH_ADDR(addr, size, ARCH_CACHE_LINE_SIZE);
	cache_lo = CACHE_OP_LOW_ADDR(addr, ARCH_CACHE_LINE_SIZE);

#if (L2CACHE_ENABLE == ON)
	if (mode == CACHE_FLUSH)
	{
		osCacheL1L2DataFlushAsync((const os_virt_ptr) cache_lo, (const os_virt_ptr) cache_hi, MMU_SHARED_PID);
	}
	else
	{
		osCacheL1L2DataSweepAsync((const os_virt_ptr) cache_lo, (const os_virt_ptr) cache_hi, MMU_SHARED_PID, mode);
	}
#elif (DCACHE_ENABLE == ON)
	osCacheDataSweep((const os_virt_ptr)cache_lo, (const os_virt_ptr)cache_hi, MMU_SHARED_PID, mode);
#endif    /* L2CACHE_ENABLE == ON */
#endif
}

#if 1
/***************************************************************************/
void get_pointer(void** ptr, os_mem_part_t *pool)
{
	do
	{
		osHwiSwiftDisable();
		*ptr = osMemBlockGet(pool);
#if (USING_INTERRUPT == FALSE)
		if (*ptr == NULL)
		{
			int i;
			for (i = 0; i < TEST_CHANNEL_NUM; i++)
			osCopChannelCtrl(&pdsch_ch_handle[i], MAPLE_PDSCH_CMD_RX_POLL, NULL);
		}
#endif
		osHwiSwiftEnable();
	}
	while (*ptr == NULL);
}
/***************************************************************************/
inline void return_pointer(os_mem_part_t *pool, void *ptr)
{
	if (ptr != NULL)
		osMemBlockFree(pool, ptr);
}
#endif

/***************************************************************************/
static void mapleErrorCb(uint32_t device, uint32_t error_type)
{
	maple_mmu_err_t mmu_err;
	/* Rely on the order inside maple_init_params_t */
	uint8_t maple_num = (device - MAPLE_LW_ID_0);
	
	app_maple_error_detect(maple_handle[maple_num], device, error_type, &mmu_err);
	
	OS_ASSERT;
}

/***************************************************************************/
static void mapleDeviceErrorCb(void *param, uint32_t status)
{
	VAR_UNUSED(param);
	OS_ASSERT_COND( (status & MAPLE_SYS_ERR) || (status & MAPLE_ECC_ERR));
	printf(" Maple Device Error happened on core %d\n", osGetCoreID());
	OS_ASSERT;
}
/***************************************************************************/
static void appPdschDispatchCb(void *cop_job, void * syms_comp)
{
#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_OPT_OFF
#pragma opt_level = "O3"
#endif

	cop_job_handle *job = (cop_job_handle *) cop_job;
	lte_subframe_t *cur_sf;
	lte_trx_t *trx;
	os_status status;
	lte_pdsch_job_t *lte_job;
	maple_pcr_opcodes_t opcode;
	uint64_t maple_bd_cycles;

	OS_ASSERT_COND((uint32_t)syms_comp > 0);
	OS_ASSERT_COND(job != NULL);

	// FIXME: динамический номер eNodeB
	cur_sf = (lte_subframe_t *)g_enodeb_inst[0].trx.maple_jobs[((uint32_t)job->job_id) & 0xff];

	trx = &g_enodeb_inst[0].trx;

	trx->num_reaped_jobs++;
	
	cur_sf->timestamp_maple_complete = log_get_timer_value();
	
	/* ?????
	if(cur_sf->job.pdsch_job.third_flags & 0x00003000 == 0)
	{
		OS_ASSERT;
	}
	*/
	
	//LOG_EVENT_SF(LOGEVT_DL_SF_MAPLE_DISPATCH, enodeb_inst.system_frame_no, enodeb_inst.system_subframe_no, (uint32_t) cur_sf);
	LOG_EVENT_SF(LOGEVT_DL_SF_MAPLE_DISPATCH, cur_sf->frame_no, cur_sf->subframe_no, (uint32_t) cur_sf);
	
#ifndef B4860
	// Старый вариант обработки для BSC9132 через очередь
	// FIXME: obsolete code
	status = osQueueDequeue(trx->evq_tx_mod_subframe, (uint32_t *) &cur_sf);

	if (status == OS_ERR_Q_EMPTY)
	{
#ifndef DEBUG_MAPLE_OUTPUT
		/* Underrun */
		return;
#else
		/* В режиме отладки просто выходим из обработчика */
		return;
#endif
	}
	else if (status != OS_SUCCESS)
	{
		/* Неожиданная ошибка */
		DEBUG();
		OS_ASSERT;
	}
	
	cur_sf->timestamp_maple_start = osTickTime();
	
#if (USING_INTERRUPT == TRUE)
	/* Add interrupt bit to last job in batch */
	cur_sf->job.pdsch_job.first_flags |= PDSCH_BD_INT_EN;
#endif

	lte_job = &cur_sf->job;
	lte_job->cop_job.job_id = (void *)trx->maple_job_id;
	trx->maple_jobs[trx->maple_job_id] = cur_sf;
	trx->maple_job_id++;

	sweep_cache((uint32_t) lte_job, sizeof(lte_pdsch_job_t), CACHE_FLUSH);
	
	if(lte_job->pdsch_job.third_flags & 0x00003000 == 0)
	{
		OS_ASSERT;
	}

//	do
	{
		int num_jobs = 1;
		status = osCopChannelDispatch(&pdsch_ch_handle[0], &lte_job->cop_job, &num_jobs);

		if (status == OS_SUCCESS)
			trx->num_disp_jobs += num_jobs;
		else
			OS_ASSERT;
	}
//	while (status != OS_SUCCESS);

#if 0
	/* PARSE PCR */
	opcode = MAPLE_PARSE_PDSCH_EDF_BD;
	status = osCopDeviceCtrl(maple_handle, MAPLE_CMD_PCR_ACTIVATE_WITH_POLL, &opcode);
	OS_ASSERT_COND(status == OS_SUCCESS);

	maple_bd_cycles = osTickTime();
	while (lte_job->pdsch_job.first_flags & 0x80000000 == 0)
		;
	maple_bd_cycles = osTickTime() - maple_bd_cycles;

	//OS_WAIT(APP_ANT_DELAY);
//			OS_WAIT(0x1400);
#endif
	/* Enough time to be sure that first BD is processed; */
	/* PCR MAPLE_PDSCH_EXT_SYM_START - the data for first BD is ready */
	opcode = MAPLE_PDSCH_EXT_SYM_START;
	status = osCopDeviceCtrl(maple_handle, MAPLE_CMD_PCR_ACTIVATE_WITH_POLL, &opcode);
	//status = osCopDeviceCtrl(maple_handle, MAPLE_CMD_PCR_ACTIVATE_NO_POLL, &opcode);
	OS_ASSERT_COND(status == OS_SUCCESS);
	
	LOG_EVENT_SF(LOGEVT_DL_SF_MAPLE_DISPATCH, cur_sf->frame_no, cur_sf->subframe_no, (uint32_t) cur_sf);
	//LOG_EVENT_SF(LOGEVT_DL_SF_MAPLE_DISPATCH, enodeb_inst.system_frame_no, enodeb_inst.system_subframe_no, (uint32_t) cur_sf);
	cur_sf->timestamp_maple_ext_sym_start = osTickTime();
	
	/* Установка семафора, заменена на event queue */
	/* В режиме TDD вроде как правильно ставить тут */
	if(enodeb_inst.state == L1_RUNNING && enodeb_inst.fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_FDD)
	{
		status = osEventSemaphorePost(enodeb_inst.trx.evt_subframe_ind, NULL);
	}
#endif
}

/***************************************************************************/
static void appPdschSymCb(uint32_t ant, os_virt_ptr job_ptr)
{
	/*
	 * status has antenna number
	 * job_ptr is a pointer to cop_job relevant for this interrupt
	 */
	os_status status;
	maple_pcr_opcodes_t opcode;

	cop_job_handle * cop_job = (cop_job_handle *) job_ptr;
	//OS_ASSERT_COND( ((uint32_t)cop_job->job_id < TEST_MAX_TEST_NUM) && ((uint32_t)cop_job->job_id >= 0));
	sym_int_count[ant]++;
}

static void mapleFtpeDispatchCb(void *job_handle, void * error_status)
{
	/*
	 app_ftpe_job *job = (app_ftpe_job *) job_handle;
	 maple_ftpe_job_t *ftpe_job = &job->ftpe_job;
	 cop_job_id id = (cop_job_id) job->cop_job.job_id;

	 VAR_UNUSED(error_status);
	 OS_ASSERT_COND(job != NULL);
	 */
	//if (id == (cop_job_id)1)
	osEventSemaphorePost(evt_fft_done, NULL);
}

volatile uint32_t num_pufft_acks = 0;
volatile uint32_t num_pufft_jobs = 0;
static void maple_pufft_dispatch_cb(void *cop_job, void * job_status)
{
	cop_job_handle *job = (cop_job_handle *) cop_job;
	maple_pufft_job_t *pufft_job = (maple_pufft_job_t *)job->device_specific;
	
    lte_ul_subframe_t    *sf;
    // FIXME: динамический номер eNodeB
    lte_trx_t *trx = &g_enodeb_inst[0].trx;
    uint32_t id;
    maple_pufft_status_t *pufft_status;
    os_status status;
    int32_t n_bds;
    static pufft_sat = 0;

    pufft_status = (maple_pufft_status_t*) &job_status;
    if(pufft_status->saturation_st != 0)
    	pufft_sat = 1;
    
    // Если это дескриптор без прерывания, то выходим, это не последний символ сабфрейма
    if((pufft_job->first_flags & PUFFT_INT_EN) == 0)
    	return;
    
    num_pufft_jobs++;
    
    id = (uint32_t)job->job_id;
    
    LOG_EVENT(LOGEVT_UL_PUFFT_SYM, id);
    if(id == 255)
    {
    	return;
    }
    
    sf = trx->lte_ul_subframe_pool[id];
    
    /* Прерывание обработки PUFFT предыдущего сабфрейма приходит в текущем сабфрейме
     * Соответственно, обработанные данные соответствуют предыдущему сабфрейму
     */ 
    // FIXME: динамический номер eNodeB
    sf->frame_no = g_enodeb_inst[0].prev_system_frame_no;
    sf->subframe_no = g_enodeb_inst[0].prev_system_subframe_no;
    
    LOG_EVENT_SF(LOGEVT_UL_PUFFT_SUBFRAME, sf->frame_no, sf->subframe_no, 0);
    osEventQueuePost(trx->evq_ul_pufft_ready, (uint32_t) sf, NULL);
    
    if(pufft_sat)
    {
    	pufft_sat = 0;
    	//trx->pufft_ova_scl++;
    	//INFO(DTRX, "PUFFT sat scale %i\n", trx->pufft_ova_scl);
    }
    
#ifdef CPRI_PUFFT_TIMER
	/* Выбор следующего свободного сабфрейма из пула */
	sf = trx->lte_ul_subframe_pool[trx->lte_ul_subframe_pool_ptr];
	trx->lte_ul_subframe_pool_ptr++;
	if (trx->lte_ul_subframe_pool_ptr >= TRX_NUM_OF_UL_JOBS)
	{
		trx->lte_ul_subframe_pool_ptr = 0;
	}

	/* Прерывание PUFFT происходит ПОСЛЕ прерывания CPRI RX
	 * Переключение буферов active <-> process производится в прерывании CPRI RX
	 * Буфер active в этот момент времени заполняется CPRI, соответственно PUFFT должен работать с ним
	 */
	//fill_pufft_job_buf_addr(trx->enodeb, sf, trx->iq_rx_buffer[trx->cpri_rx_active_buffer_no]);
	fill_pufft_job_buf_addr(trx->enodeb, sf, trx->cpri_rx_active_buffer_no);

	n_bds = sf->n_pufft_jobs;
	status = osCopChannelDispatch(&pufft_ch_handle[0], &sf->pufft_job[0].cop_job, &n_bds);
	OS_ASSERT_COND(status == OS_SUCCESS);
	
	//INFO(DTRX,"PUFFT CB a=%i p=%i\n", trx->cpri_rx_active_buffer_no, trx->cpri_rx_process_buffer_no);
#endif

}

/***************************************************************************/
static void maple_pufft_ack_cb(uint32_t status, os_virt_ptr cop_job)
{
	extern void cpri_timer_rx_cb(os_hwi_arg timer_num);
	//uint32_t num_pufft_acks = 0;
	
    VAR_UNUSED(cop_job);
    VAR_UNUSED(status);

	/* Инкремент таймера, отвечающего за запуск PUFFT по событиям CPRI RX
	 * Прерывание таймера не используется, т.к. при работу с PUFFT обработка значений должна
	 * происходить из прерывания PUFFT_INT_ACK 
	 * (раздел 1.4.13.2 PUFFT2_EDF, Host and CPRI: Interfaces, handshake and responsibilities)
	 */
	cpri_timer_rx_cb(SOC_TIMER_CPRI_RX);

    num_pufft_acks++;
}
/***************************************************************************/

uint32_t num_users_done = 0;
uint32_t num_ctrl_done = 0;
volatile uint32_t ctrl_proc = 0;
volatile uint32_t users_proc = 0;

static void maplePuschDispatchCb(void *job_handle, void *user)
{
	VAR_UNUSED(job_handle);
	VAR_UNUSED(user);
	num_users_done++;
}
static void maplePuschAllCtrlDoneCb(void *job_handle, void *user)
{
	VAR_UNUSED(job_handle);
	VAR_UNUSED(user);
	num_ctrl_done++;
	//ctrl_proc++;
	
	//osEventQueuePost(g_enodeb_inst[0].trx.evq_pusch_ctrl_ready, (uint32_t)0, NULL);
}

static void maplePuschCtrlCb(void *job_handle, void  *user)
{
    uint32_t i;
    uint32_t user_id = (uint32_t)user;
    os_status status = OS_SUCCESS;
    cop_job_handle *cop_job = (cop_job_handle *)job_handle;
    maple_pusch_job_t   *job = (maple_pusch_job_t *)cop_job->device_specific;
    uint32_t job_id = (uint32_t)cop_job->job_id;
    // CQI ready for processing
    ctrl_proc++;
}

/***************************************************************************/
static void maplePuschAllUsersDoneCb(void *job_handle, void  *param)
{
	cop_job_handle        *job = (cop_job_handle *)job_handle;
	lte_ul_pusch_descr_t *pusch_descr;
	maple_pusch_job_t *pusch_job;
	maple_pusch_sh_t     *sh;

	uint8_t id = (uint8_t)job->job_id & 0xff;
	
	LOG_EVENT(LOGEVT_UL_PUSCH_READY, id);

    pusch_job = (maple_pusch_job_t *)((cop_job_handle *)job_handle)->device_specific;
    OS_ASSERT_COND(pusch_job != NULL);
    sh = (maple_pusch_sh_t *)pusch_job->seg_param_ptr;

    // FIXME: динамический номер eNodeB
    pusch_descr = g_enodeb_inst[0].trx.ul_pusch_pool[id];
    
    users_proc = 1;
    
    osEventQueuePost(g_enodeb_inst[0].trx.evq_pusch_ready, (uint32_t)pusch_descr, NULL);
}

static void maplePuschPuTermCb(void *job_handle, void  *param)
{

}
/***************************************************************************/
static void maplePuschEqTermCb(void *job_handle, void  *param)
{
    maple_pusch_job_t *pusch_job;
    uint32_t i,j , layer;
    uint32_t user_id, seg_id;
    os_status status = OS_SUCCESS;
    uint32_t eqpe_output_size;
    uint32_t eqpe_scales_output_size;
    uint32_t nv_beta_output_size;
    uint32_t rows, cols, lvs;
    uint8_t *eq_rslt_iter, *eq_rslt;
    maple_pusch_sh_t  * sh;
    maple_pusch_uph_t * uph;
    uint32_t job_id;
    OS_ASSERT_COND(job_handle != NULL);
 
    VAR_UNUSED(param);
 
    pusch_job = (maple_pusch_job_t *)((cop_job_handle *)job_handle)->device_specific;
    job_id = (uint32_t)((cop_job_handle *)job_handle)->job_id;
    sh = (maple_pusch_sh_t *)pusch_job->seg_param_ptr;
    uph = (maple_pusch_uph_t *)pusch_job->user_param_ptr;
}

static void mapleTvpeDispatchCb(void *cop_job, void * error_status)
{
	cop_job_handle      *job = (cop_job_handle *)cop_job;
	uint32_t            job_id;
	lte_ul_pusch_descr_t *pusch_descr;
	
	OS_ASSERT_COND(job != NULL);
	job_id       = (uint32_t)job->job_id;
	
	uint8_t id = (uint8_t)job->job_id & 0xff;
	
	// FIXME: динамический номер eNodeB
	pusch_descr = g_enodeb_inst[0].trx.ul_pusch_pool[id];
	
	osEventQueuePost(g_enodeb_inst[0].trx.evq_tvpe_ready, (uint32_t)pusch_descr, NULL);
}

#if 1
/***************************************************************************/
static void appMemMngrCreate(uint32_t num_buffers, uint32_t buff_size, uint32_t alignment, os_mem_type mem_type, app_mem_mngr_t mngr_type)
{
	if ((buff_size != 0) && (num_buffers != 0))
	{
		mem_mngr[mngr_type].mem_manager = osMalloc(MEM_PART_SIZE(num_buffers), mem_type);
		OS_ASSERT_COND(mem_mngr[mngr_type].mem_manager != NULL);

		mem_mngr[mngr_type].space = osAlignedMalloc(MEM_PART_DATA_SIZE(num_buffers, buff_size, alignment), mem_type, alignment);
		OS_ASSERT_COND(mem_mngr[mngr_type].space != NULL);

		mem_mngr[mngr_type].pool = osMemPartCreate(buff_size, num_buffers, mem_mngr[mngr_type].space, alignment, OFFSET_0_BYTES,
				(os_mem_part_t *) mem_mngr[mngr_type].mem_manager
#if (OS_MULTICORE == ON)
				, 0
#endif
				);
		OS_ASSERT_COND(mem_mngr[mngr_type].pool != NULL);
	}
}

void MemMngrCreate(uint32_t num_buffers, uint32_t buff_size, uint32_t alignment, os_mem_type mem_type, mem_manager_t *mngr)
{
	OS_ASSERT_COND(mngr != NULL);

	if ((buff_size != 0) && (num_buffers != 0))
	{
		mngr->mem_manager = osMalloc(MEM_PART_SIZE(num_buffers), mem_type);
		OS_ASSERT_COND(mngr->mem_manager != NULL);

		mngr->space = osAlignedMalloc(MEM_PART_DATA_SIZE(num_buffers, buff_size, alignment), mem_type, alignment);
		OS_ASSERT_COND(mngr->space != NULL);

		mngr->pool = osMemPartCreate(buff_size, num_buffers, mngr->space, alignment, OFFSET_0_BYTES, (os_mem_part_t *) mngr->mem_manager
#if (OS_MULTICORE == ON)
				, 0
#endif
				);
		OS_ASSERT_COND(mngr->pool != NULL);
	}
}
#endif

void fill_pufft_job_buf_addr(lte_enodeb_t *enodeb, lte_ul_subframe_t *sf, int32_t buf_no)
{
	int32_t job_no;
	int32_t sym_pos = 0;
	int32_t ant_no;

	for (job_no = 0; job_no < LTE_NSYMB_PER_SUBFRAME; job_no++)
	{
		lte_pufft_job_t *job = &sf->pufft_job[job_no];

		int32_t cp_len = (job_no == 0 || job_no == 7) ? enodeb->fp.CPRI_CP0_LEN : enodeb->fp.CPRI_CPx_LEN;
		
		sym_pos += cp_len;
		
		for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
		{
			// Адрес отсчетов во входном буфере без CP
			job->pufft_job.antx_input[ant_no] = &enodeb->trx.iq_rx_buffer[ant_no][buf_no][sym_pos];//&buf_in[sym_pos];
			
			/* Фиктивное значение в статуса для определения того, что дескриптор обработан
			 * По-идее значение 0x66 не может возвращаться из PUFFT
			 */
			job->status[0] = 0x66;
			
			if(job->pufft_job.first_flags & PUFFT_OVA_SCL)
				job->pufft_job.second_flags = PUFFT_ADP_OVA_SCL(enodeb->trx.pufft_ova_scl);
			else
				job->pufft_job.second_flags = PUFFT_ADP_OVA_SCL(0);

		}
		
		sym_pos += enodeb->fp.CPRI_SYMBOL_LEN;
	}
	
	app_store_barrier();
}

/***************************************************************************/
os_status maple_fft_ifft(Complex16 *iq_in, Complex16 *iq_out, int32_t len, int32_t inverse)
{

	os_status status;
	int32_t i;
	int32_t num_bds;
	uint32_t fft_size_code;

	// Выход без запуска, используется при отладке
	//return OS_SUCCESS;
	
	switch (len)
	{
		case 128:
			fft_size_code = 35;
			break;

		case 256:
			fft_size_code = 36;
			break;

		case 512:
			fft_size_code = 37;
			break;

		case 1024:
			fft_size_code = 38;
			break;

		case 2048:
			fft_size_code = 39;
			break;

		default:
			OS_ASSERT;
	}

	fft_ifft_job.cop_job.job_id = (cop_job_id) 100;
	fft_ifft_job.cop_job.device_specific = &fft_ifft_job.ftpe_job;

	fft_ifft_job.ftpe_job.first_flags = FTPE_BD_CFG0_INT_EN;
	fft_ifft_job.ftpe_job.second_flags = ((inverse == 0) ? 0 : FTPE_BD_CFG1_ITE);// | FTPE_BD_CFG1_OVA_SCL;
	fft_ifft_job.ftpe_job.third_flags = FTPE_BD_CFG3_ADP_OVA_SCL(0);
	fft_ifft_job.ftpe_job.fourth_flags = 0; //FTPE_BD_CFG3_ADP_OVA_SCL(12);
	fft_ifft_job.ftpe_job.fifth_flags = 0; //FTPE_BD_CFG4_CPS(LTE_CP0_LEN - 1) | FTPE_BD_CFG4_BD_RPT(0);
	fft_ifft_job.ftpe_job.extension_ptr = NULL;
	fft_ifft_job.ftpe_job.buffer_size = (uint32_t) fft_size_code;
	fft_ifft_job.ftpe_job.input = (void *) (iq_in);
	fft_ifft_job.ftpe_job.output = (void *) (iq_out);

	fft_ifft_job.ftpe_job.pre_operation_ptr = NULL;
	
	fft_ifft_job.ftpe_job.post_multiplier_ptr = NULL;
	fft_ifft_job.ftpe_job.bd_rpt_stat_ptr = NULL;
	fft_ifft_job.ftpe_job.in_exp_ptr = NULL;
	//job0->ftpe_job.bd_rpt_stat_ptr = &bd_stats;
	fft_ifft_job.cop_job.next = NULL;

//	sweep_cache((uint32_t) &fft_ifft_job, sizeof(fft_ifft_job_t), CACHE_FLUSH);
//	sweep_cache((uint32_t) iq_in, len * sizeof(Complex16), CACHE_FLUSH);
	num_bds = 1;
	
	osEventSemaphoreReset(evt_fft_done, 0);

	while (osCopChannelDispatch(&ftpe_ch_handle, &fft_ifft_job.cop_job, &num_bds) != OS_SUCCESS)
	{
		ftpe_dispatch_fail_counter++;
		num_bds = 1;
		osCopChannelCtrl(&ftpe_ch_handle, MAPLE_FTPE_CMD_RX_POLL, NULL);
	}
	
	/*
	 status = osCopChannelCtrl(&ftpe_ch_handle, MAPLE_FTPE_CMD_RX_POLL, NULL);
	 if(status != OS_SUCCESS)
	 printf("POLL error!\n");
	 
	 */
	status = osEventSemaphorePend(evt_fft_done, 0);
	OS_ASSERT_COND(status == OS_SUCCESS);

	sweep_cache((uint32_t) iq_out, len * sizeof(Complex16), CACHE_FLUSH /*CACHE_INVALIDATE*/);
	
	return OS_SUCCESS;
}

os_status lte_maple_pufft_start(lte_enodeb_t *enodeb, lte_ul_subframe_t *sf)
{
	//int32_t num_bds = 8;
	os_status status;
	lte_trx_t *trx = &enodeb->trx;
    
	//fill_pufft_job_buf_addr(enodeb, sf, trx->iq_rx_buffer[trx->cpri_rx_process_buffer_no]);
	fill_pufft_job_buf_addr(enodeb, sf, trx->cpri_rx_process_buffer_no);
	
	status = osCopChannelDispatch(&pufft_ch_handle[0], &sf->pufft_job[0].cop_job, &sf->n_pufft_jobs);
	return status;
}


static 	maple_open_params_t maple_open_params =
		{ 0 };
static maple3w_dev_init_params_t  dev_init_params_w    = {0}; /* Use WCDMA structure for WCDMA */
static maple3lw_dev_init_params_t dev_init_params_lw   = {0}; /* Use LTE structure for LTE */

#if (MAPLE_DIRECT_ACCESSES == TRUE)
/* This function is used for maple_init() ucode function */
INLINE long mapleInitDirectWrite(void    *dest_addr,
                                 void    *src_addr,
                                 uint32_t size,
                                 uint32_t dev_id,
                                 uint32_t param)
{
    uint32_t          count;
    void              *d_addr;
    void              *s_addr;
 
    VAR_UNUSED(dev_id);
    VAR_UNUSED(param);
    count  = size;
    d_addr = dest_addr;
    s_addr = src_addr;
    while((count >= 8) &&
         ((((uint32_t)s_addr)& 0x7) == 0) &&
         ((((uint32_t)d_addr)& 0x7) == 0))
    {
      *((uint64_t *)d_addr) = *((uint64_t *)s_addr);
      d_addr                = (void *)(((uint32_t)d_addr) + 8);
      s_addr                = (void *)(((uint32_t)s_addr) + 8);
      count                -= 8;
    }
    while(count >= 4)
    {
      *((uint32_t *)d_addr) = *((uint32_t *)s_addr);
      d_addr                = (void *)(((uint32_t)d_addr) + 4);
      s_addr                = (void *)(((uint32_t)s_addr) + 4);
      count                -= 4;
    }
    if(count >= 2)
    {
      *((uint16_t *)d_addr) = *((uint16_t *)s_addr);
      d_addr                = (void *)(((uint32_t)d_addr) + 2);
      s_addr                = (void *)(((uint32_t)s_addr) + 2);
      count                -= 2;
    }
    if(count > 0)
    {
      *((uint8_t *)d_addr) = *((uint8_t *)s_addr);
    }
    return (long)size;
}
#endif // (MAPLE_DIRECT_ACCESSES == TRUE)

static os_status reset_maples()
{
    uint32_t maple_pcr_register;
    uint32_t maple_ready_register;
    uint32_t maple_status_register, maple_gcr_register;
    uint32_t volatile data = 0;
    uint32_t volatile maple_status = 0;
    uint32_t volatile maple_gcr_status = 0;
 
    uint32_t sbus_base;
    const uint32_t maple_pcr               = 0x4000;
    const uint32_t maple_status_offset     = 0x4414;
    const uint32_t maple_ready_offset      = 0x420C;
    const uint32_t maple_gcr_offset        = 0x4500;
    const uint32_t maple_soft_reset_opcode = 0x37;
    uint32_t maple_num;
 
    for(maple_num = 0; maple_num < MAPLE_W_ID_2; maple_num++)
    {
        sbus_base = (SOC_DSP_MAPLE_DEFAULT + 0x10000 * maple_num);
 
		maple_ready_register  = sbus_base + maple_ready_offset;
		maple_pcr_register    = sbus_base + maple_pcr;
		maple_status_register = sbus_base + maple_status_offset;
		maple_gcr_register     = sbus_base + maple_gcr_offset;
	
		mapleDirectRead((void *)maple_ready_register, (void *)&data,sizeof(uint32_t),NULL,NULL);
		mapleDirectRead((void *)maple_status_register, (void *)&maple_status,sizeof(uint32_t),NULL,NULL);
		mapleDirectRead((void *)maple_gcr_register, (void *)&maple_gcr_status,sizeof(uint32_t),NULL,NULL);
	
		// Check if MAPLE RISC in on
		if (!(data & 0x80000000))
			continue;
		
#define __MAPLE_INIT_BEGIN  0x1
#define __MAPLE_INIT_END    0x2
#define __MAPLE_RESET_BEGIN 0x4
#define __MAPLE_RESET_END   0x8
#define __MAPLE_GCR_READY   0x10

		// If Soft Reset started or if gcr is off, no need to issue another
		if ((maple_status & __MAPLE_RESET_BEGIN) || !(maple_gcr_status & __MAPLE_GCR_READY))
			continue;

		// If a MAPLE init was issued, need to wait until it's over
		if (maple_status & __MAPLE_INIT_BEGIN)
		{
			while(!(maple_status & __MAPLE_INIT_END))
				mapleDirectRead((void *)maple_status_register, (void *)&maple_status, sizeof(uint32_t), NULL, NULL);
		}

		// Stop new dma commands and wait untill dma queue is empty
		data = (0x00010000| maple_soft_reset_opcode);
		mapleInitDirectWrite((void *)maple_pcr_register,
								  (void *)&data,sizeof(uint32_t),0,0);
	
		do
		{
		  mapleDirectRead((void *)maple_pcr_register,
								 (void *)&data,sizeof(uint32_t),0,0);
		}
		while (data & 0x00010000);
	
		// Apply soft reset.
		data = 0x80000000;
		mapleInitDirectWrite((void *)maple_pcr_register,
								 (void *)&data,sizeof(uint32_t),0,0);
		// Wait until Soft Reset ends.
		do
		{
			mapleDirectRead((void *)maple_status_register, (void *)&maple_status,sizeof(uint32_t),NULL,NULL);
		}
		while(!(maple_status & __MAPLE_RESET_END));
    }
    return OS_SUCCESS;
}

/***************************************************************************/
static void appMapleInit(lte_enodeb_t *enodeb, int32_t n_sectors, int maple_num)
{
    extern maple_init_params_t  maple_init_params;
    extern uint32_t VirtSharedDDR0_b;
    extern uint32_t SharedDDR0_size;
 
    os_mmu_segment_handle       segment_num = 0;
    cop_dev_open_params_t       cop_dev_open_params = {0};
    //maple_open_params_t         maple_open_params   = {0};
    char*                       maple_name          = maple_init_params.device_init[maple_num].name;
    maple_mmu_seg_update_t maple_segments[10]        = {0};
    uint8_t core_num = osGetCoreID();
    os_status status = OS_SUCCESS;

    uint32_t module, timer;
    void* ack_address;
    extern tmr32b_map_t      *soc_timer32_module;
    int32_t i;
    
    reset_maples();

    core_mmu_task_id = core_num + 1; /* Each core used different Maple MMU task id */
 
    /* Open MAPLE device */
 
    /* Only one core will handle errors */
    if (core_num == osGetSocMasterCore())
        maple_open_params.error_callback    = mapleErrorCb;
    else
        maple_open_params.error_callback    = NULL;

    maple_open_params.mbus_base         = (void *)maple_mbus[maple_num];
    maple_open_params.sbus_base         = (void *)maple_sbus[maple_num];
    maple_open_params.config_mbus_mmu   = TRUE;
    maple_open_params.config_sbus_mmu   = FALSE;    /**< Already part of ccsr map */

    cop_dev_open_params.dispatch_callback   = NULL;
    cop_dev_open_params.error_callback      = NULL;
    cop_dev_open_params.lld_params          = &maple_open_params;

    /* No need to load MapleB3W micro code into memory if it's not used in this test*/
#if (MAPLE_2 == ON)
    // hidden assumption for this test is that dev_id = 2 is WCDMA
    if (maple_init_params.device_init[maple_num].dev_id == MAPLE_W_ID_2)
    {
        maple_open_params.dev_init_params   = &dev_init_params_w;
        maple_open_params.maple_init  = MAPLE_B3W_INIT;
        /* MapleB3 Firmware parameters */
        dev_init_params_w.ucode       = MAPLE_B3W_UCODE;
        dev_init_params_w.mbus_base   = maple_open_params.mbus_base;
        dev_init_params_w.sbus_base   = maple_open_params.sbus_base;
    }
    else
#endif
#if (MAPLE_0 == ON) || (MAPLE_1 == ON)
{
    maple_open_params.dev_init_params = &dev_init_params_lw;
	maple_open_params.maple_init = MAPLE_B3LW_INIT;
	/* MapleB3 Firmware parameters */
	dev_init_params_lw.ucode = MAPLE_B3LW_UCODE;
	dev_init_params_lw.mbus_base = maple_open_params.mbus_base;
	dev_init_params_lw.sbus_base = maple_open_params.sbus_base;

	dev_init_params_lw.ecc_protect = FALSE;
	dev_init_params_lw.config_write = NULL;
	dev_init_params_lw.config_read = NULL;
	dev_init_params_lw.dev_id = 0;
	dev_init_params_lw.config_param = 0;
#ifdef SIMULATOR
	dev_init_params_lw.timer_period = 0x3FFFF;
#else
	dev_init_params_lw.timer_period = 0;
#endif
	dev_init_params_lw.mode1 = 0x02000000;
	dev_init_params_lw.mode2 = 0;
	//dev_init_params_lw.tvpe_mode = 0;
	
    dev_init_params_lw.tvpe_mode   = 0x00002000;
    dev_init_params_lw.pusch_mode  = 0x8000007A;// | 0x0000001;
    
    // Конфигурация режима PDSCH  взависимости от количества секторов
    switch(n_sectors)
    {
    	case 1:
    		dev_init_params_lw.pdsch_mode.MPDSCHCP = PDSCH_NUM_SECTORS_1 | PDSCH_MNOSEM_EN;
    		break;
    	
    	case 2:
    		dev_init_params_lw.pdsch_mode.MPDSCHCP = PDSCH_NUM_SECTORS_2 | PDSCH_MNOSEM_EN;
    		break;
    		
    	case 3:
    		dev_init_params_lw.pdsch_mode.MPDSCHCP = PDSCH_NUM_SECTORS_3 | PDSCH_MNOSEM_EN;
    		break;
    	
    	case 4:
    		dev_init_params_lw.pdsch_mode.MPDSCHCP = PDSCH_NUM_SECTORS_4 | PDSCH_MNOSEM_EN;
    		break;
    		
    	default:
    		dev_init_params_lw.pdsch_mode.MPDSCHCP = PDSCH_NUM_SECTORS_1 | PDSCH_MNOSEM_EN;
    }
	
    switch(enodeb->fp.LTE_N_RB_DL)
    {
    	case 15:
    		enodeb->fp.MPSCP = PDSCH_MODE_BW_3MHZ;
    		break;
    		
    	case 25:
    		enodeb->fp.MPSCP = PDSCH_MODE_BW_5MHZ;
    		break;
    		
    	case 50:
    		enodeb->fp.MPSCP = PDSCH_MODE_BW_10MHZ;
    		break;
    		
    	case 100:
    		enodeb->fp.MPSCP = PDSCH_MODE_BW_20MHZ;
    		break;
    }
    
    switch(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX)
    {
    	case 1:
    		enodeb->fp.MPSCP |= PDSCH_USE_1_FTPE | PDSCH_NUM_VIRT_ANT_1 | PDSCH_NUM_PHYS_ANT_1 | PDSCH_NUM_PBCH_ANT_PORTS_2 | PDSCH_NUM_CELL_REF_PORTS_1;
    		break;
    		
    	case 2:
    		enodeb->fp.MPSCP |= PDSCH_USE_2_FTPE | PDSCH_NUM_VIRT_ANT_2 | PDSCH_NUM_PHYS_ANT_2 | PDSCH_NUM_PBCH_ANT_PORTS_2 | PDSCH_NUM_CELL_REF_PORTS_2;
    		break;
    		
    	case 4:
    		enodeb->fp.MPSCP |= PDSCH_USE_3_FTPE | PDSCH_NUM_VIRT_ANT_4 | PDSCH_NUM_PHYS_ANT_4 | PDSCH_NUM_PBCH_ANT_PORTS_4 | PDSCH_NUM_CELL_REF_PORTS_4;
    		break;
    	
    	case 8:
    		enodeb->fp.MPSCP |= PDSCH_USE_3_FTPE | PDSCH_NUM_VIRT_ANT_8 | PDSCH_NUM_PHYS_ANT_8 | PDSCH_NUM_PBCH_ANT_PORTS_4 | PDSCH_NUM_CELL_REF_PORTS_4;
    		break;
    }
    
	enodeb->fp.MPSCP |= PDSCH_SECTOR_BA_ALOCATION_0 | PDSCH_SECTOR_RES_ALOCATION_1; 

#if 0
	for (i = 0; i < LTE_N_SECTORS; i++)
	{
		/* PDSCH Sector configuration */
		dev_init_params_lw.pdsch_mode.MPSCPs[i] = MPSCPs[i];
		dev_init_params_lw.pdsch_mode.MPTACSs[i][PDSCH_SSS_TGT_ANT] = MPTACs[i][PDSCH_SSS_TGT_ANT];
		dev_init_params_lw.pdsch_mode.MPTACSs[i][PDSCH_PSS_TGT_ANT] = MPTACs[i][PDSCH_PSS_TGT_ANT];
		dev_init_params_lw.pdsch_mode.MPTACSs[i][PDSCH_POS_TGT_ANT] = MPTACs[i][PDSCH_POS_TGT_ANT];
		dev_init_params_lw.pdsch_mode.MPTACSs[i][PDSCH_MBSFN_TGT_ANT] = MPTACs[i][PDSCH_MBSFN_TGT_ANT];
	}
#endif
	
	uint8_t ant_refsig_mask = (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 1) ? 0x01 : 
			((enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2) ? 0x03 :
			((enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 4) ? 0x0f :
			((enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 8) ? 0xff : 0x00)));
	
	for (i = 0; i < n_sectors; i++)
	{
		/* PDSCH Sector configuration */
		dev_init_params_lw.pdsch_mode.MPSCPs[i] = g_enodeb_inst[i].fp.MPSCP;
		dev_init_params_lw.pdsch_mode.MPTACSs[i][PDSCH_SSS_TGT_ANT] = ant_refsig_mask;//MPTACs[i][PDSCH_SSS_TGT_ANT];
		dev_init_params_lw.pdsch_mode.MPTACSs[i][PDSCH_PSS_TGT_ANT] = ant_refsig_mask;//MPTACs[i][PDSCH_PSS_TGT_ANT];
		dev_init_params_lw.pdsch_mode.MPTACSs[i][PDSCH_POS_TGT_ANT] = MPTACs[i][PDSCH_POS_TGT_ANT];
		dev_init_params_lw.pdsch_mode.MPTACSs[i][PDSCH_MBSFN_TGT_ANT] = MPTACs[i][PDSCH_MBSFN_TGT_ANT];
		
		/* Конфигурация альтернативного размера FFT для полосы 3МГц (FFT == 256), т.к. RRU поддерживает
		 * только Fs=7.68 (размерность FFT == 512)
		 */
		if(enodeb->fp.LTE_N_RB_DL == 25)
		{
			dev_init_params_lw.pdsch_mode.MPTACSs[i][PDSCH_ALT_DBW_PARAM] = 0x0;
		}
		else if(enodeb->fp.LTE_N_RB_DL < 25)
		{
			/* ABBEN | ALT_DBW == 0x08 | 0x02  == 512 отсчетов */
			dev_init_params_lw.pdsch_mode.MPTACSs[i][PDSCH_ALT_DBW_PARAM] = 0x0a;
		}
		else
		{
			ERROR(DTRX, "Unsupported DL subframe size of %i samples\n", enodeb->fp.LTE_N_RB_DL);
		}
	}
}
#endif
 
    //printf("\nLoading Ucode on %s...\n", maple_name);
    maple_handle[maple_num] = osCopDeviceOpen(maple_name, &cop_dev_open_params);
    OS_ASSERT_COND(maple_handle[maple_num] != NULL);
    //printf("Opened %s device - Done \n", maple_name);

#if (USING_MAPLE_MMU == TRUE)
    /*** Initialize Maple MMU ***/
    status = app_mmmu_enable(maple_handle[maple_num], TRUE, OS_MEM_SHARED_CACHEABLE, MAPLE_STEERING_CHB);
    OS_ASSERT_COND(status == OS_SUCCESS);
    //printf("Maple Address Translation on %s - Enabled...\n", maple_name);

    maple_segments[0] = app_mmmu_segment_add(maple_handle[maple_num], g_heap_ddr0_cacheable, core_mmu_task_id);
    //printf("Added segment %d tid = %d virt addr = 0x%x phys addr = 0x%016llX size = 0x%x \n", maple_segments[0].descriptor, core_mmu_task_id, maple_segments[0].attributes.virt_addr, maple_segments[0].attributes.phys_addr, maple_segments[0].attributes.size);
 
    /* Adding shared segment with shared task ID */
    if (osGetCoreID() == osGetMasterCore())
    {
        maple_segments[1] = app_mmmu_segment_add(maple_handle[maple_num], g_shared_mem_heap_ddr0_cacheable, 0);
        //printf("Added segment %d tid = %d virt addr = 0x%x phys addr = 0x%016llX size = 0x%x \n", maple_segments[1].descriptor, 0, maple_segments[1].attributes.virt_addr, maple_segments[1].attributes.phys_addr, maple_segments[1].attributes.size);
  
        // define maple mmu descriptor for SOC timers interrupt ack_address:
        module = MAPLE_PUFFT_GET_TIMERS_MODULE(app_timer_alloc[0][0]);
        timer  = MAPLE_PUFFT_GET_TIMERS_CHANNEL(app_timer_alloc[0][0]);
        ack_address = (void *)&soc_timer32_module[module].tmr[timer].tmr_sctl;
 
        maple_segments[3] = app_mmmu_segment_add(maple_handle[maple_num], ack_address, 0);
    }
    
    maple_segments[2] = app_mmmu_segment_add(maple_handle[maple_num], 0x60000000, core_mmu_task_id);
    /*
    printf("Added segment %d tid = %d virt addr = 0x%x phys addr = 0x%016llX size = 0x%x \n", maple_segments[2].descriptor,
    		core_mmu_task_id, maple_segments[2].attributes.virt_addr, maple_segments[2].attributes.phys_addr,
    		maple_segments[2].attributes.size);
	*/
#else
    /*** Disable Maple MMU ***/
    osCopDeviceCtrl(maple_handle[maple_num], MAPLE_CMD_MMU_DISABLE, NULL);
    printf("Maple Address Translation on %s - Disabled...\n", maple_name);
#endif
    
    osWaitForAllCores();
#if 0
    // MAPLE-B3 trace
    osCopDeviceCtrl(maple_handle,
    		MAPLE_CMD_SET_TRACE_EVENTS,
    		(void *)(MAPLE_EN_TRACE_EVENT_PDSCH_STARTED_FINAL_PROCESSING_OF_AN_OFDM_SYMBOL |
    				MAPLE_EN_TRACE_EVENT_PDSCH_DETERMINED_NUMBER_OF_CONTROL_DATA_AND_PAD_OFDM_SYMBOLS |
    				MAPLE_EN_TRACE_EVENT_PDSCH_STARTED_PARSING_USER_HEADERS |
    				MAPLE_EN_TRACE_EVENT_PDSCH_IS_PREFETCHING_OFDM_SYMBOL_INFORMATION |
    				MAPLE_EN_TRACE_EVENT_PDSCH_BD_DONE));
#endif
}

/*****************************************************************************/
static void appPufftInit(int pufft_num)
{
    extern maple_pufft_init_params_t maple_pufft_init_params;
    cop_dev_open_params_t           cop_dev_open_params = {0};
    pufft_open_params_t             pufft_open_params    = {0};
    char*                           pufft_name           = maple_pufft_init_params.pe_init[pufft_num].name;
    int ii, ant;
    lte_enodeb_t *enodeb;
 
    uint8_t  core_id = osGetCoreID();
    uint32_t i, module, timer;
    extern tmr32b_map_t      *soc_timer32_module;
	//extern soc_timer32_num_t app_timer_alloc[3][4];

    pufft_open_params.maple_handle           = maple_handle[pufft_num];
    pufft_open_params.maple_pe_bd_priority   = MAPLE_PE_BD_RING_H_L;
    pufft_open_params.maple_pe_num_bd        = MAPLE_PE_NUM_BD_RINGS_8;
 
    for (ii = 0; ii < NUM_ENODEB; ii++)
    {
    	enodeb = &g_enodeb_inst[ii];
    	
    	if(enodeb->state != L1_CONFIGURED)
    		continue;
    	
#if (USING_MAPLE_MMU == TRUE)
        pufft_open_params.pre_mult_addr[enodeb->sector]  = (void *)enodeb->ul_shift_dds;
        pufft_open_params.post_mult_addr[enodeb->sector] = NULL;//(void *)post_mult_vec_128;
#else
        os_phys_ptr phys_ptr = NULL;
        os_status status = osMmuDataVirtToPhys((os_virt_ptr)pre_mult_vec_128, &phys_ptr);
        OS_ASSERT_COND(phys_ptr != NULL);
        OS_ASSERT_COND(status == OS_SUCCESS);
        pufft_open_params.pre_mult_addr[sector]  = (void *)phys_ptr;

        status = osMmuDataVirtToPhys((os_virt_ptr)post_mult_vec_128, &phys_ptr);
        OS_ASSERT_COND(phys_ptr != NULL);
        OS_ASSERT_COND(status == OS_SUCCESS);
        pufft_open_params.post_mult_addr[sector]  = (void *)phys_ptr;

#endif
        pufft_open_params.sector_config[enodeb->sector]  = PUFFT_INIT_FTPE_INDX(1) | PUFFT_INIT_OVFL_CNT(2);
        pufft_open_params.antx_in_size[enodeb->sector]   = IQ_RX_BUFFER_SIZE_DEFAULT * NUM_CPRI_BUFFERS; //LTE_SAMPLES_PER_SUBFRAME * 4; // was *2;
        //pufft_open_params.antx_in_size[enodeb->sector]   = enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4 * NUM_CPRI_BUFFERS; //LTE_SAMPLES_PER_SUBFRAME * 4; // was *2;
 
        for(ant = 0; ant < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant++)
        {
#if (USING_MAPLE_MMU == FALSE)
            pufft_open_params.antx_in_addr[sector][ant] = (void *)ant_in_phys_base[sector][ant];
#else
            //pufft_open_params.antx_in_addr[sector][ant] = ant_in_virt_base[sector][ant];
            pufft_open_params.antx_in_addr[enodeb->sector][ant] = enodeb->trx.iq_rx_buffer[ant][0];
#endif
        }
 
#ifndef OS_B4_REV1
        /*
        pufft_open_params.fc_base_real[sector] = 0x5d44;
        pufft_open_params.fc_base_imag[sector] = 0xbc1a;
 
        pufft_open_params.fc_shift_real[sector] = 0xf33438;
        pufft_open_params.fc_shift_imag[sector] = 0xc9c35c;
 	 	*/
        // TODO: сделать генерацию UL DDS (7.5кГц) через MAPLE-B3
        pufft_open_params.fc_base_real[enodeb->sector] = 0;
        pufft_open_params.fc_base_imag[enodeb->sector] = 0;
 
        pufft_open_params.fc_shift_real[enodeb->sector] = 0;
        pufft_open_params.fc_shift_imag[enodeb->sector] = 0;

#endif
    }
    
     //Set ACK address and ACK data to be used by MPISR:
    for (i = 0 ; i < PUFFT_SOC_TIMER_SETS; i++)
    {
    	module = MAPLE_PUFFT_GET_TIMERS_MODULE(app_timer_alloc[pufft_num][i]);
    	timer  = MAPLE_PUFFT_GET_TIMERS_CHANNEL(app_timer_alloc[pufft_num][i]);
 
#if (USING_MAPLE_MMU == FALSE)
        os_phys_ptr phys_addr = NULL;
    	os_status status = osMmuDataVirtToPhys((const os_virt_ptr)&soc_timer32_module[module].tmr[timer].tmr_sctl, &phys_addr);
    	pufft_open_params.soc_timer_conf[i].ack_addr = (void *)phys_addr;
        OS_ASSERT_COND(status == OS_SUCCESS);
#else
    	pufft_open_params.soc_timer_conf[i].ack_addr = (void *)&soc_timer32_module[module].tmr[timer].tmr_sctl;
#endif

    	pufft_open_params.soc_timer_conf[i].ack_data = MAPLE_PUFFT_GET_TIMERS_CLEAR_EVENT_VALUE;
 
    }
    
    cop_dev_open_params.dispatch_callback   = maple_pufft_dispatch_cb;
    cop_dev_open_params.error_callback      = NULL;
    cop_dev_open_params.lld_params          = &pufft_open_params;

    //printf("Opening PUFFT %s...\n", pufft_name);
    pufft_handle[pufft_num] = osCopDeviceOpen(pufft_name, &cop_dev_open_params);
    OS_ASSERT_COND(pufft_handle[pufft_num] != NULL);
    //printf("Opened %s device - Done \n", pufft_name);
}

/*****************************************************************************/
static void appPufftChannelInit(int pufft_num, int ch_num)
{
    maple_pufft_ch_open_params_t maple_pufft_ch_open_params = {0};
    cop_ch_open_params_t        ch_open_params;
    os_status                   status;
    int i = 0;
 
    maple_pufft_ch_open_params.ch_params.flags                 = 0;
    maple_pufft_ch_open_params.ch_params.no_automatic_reap     = FALSE;
    maple_pufft_ch_open_params.ch_params.high_priority         = FALSE;
    maple_pufft_ch_open_params.ch_params.int_enable            = TRUE;
    maple_pufft_ch_open_params.ch_params.no_translation        = USING_MAPLE_MMU;
    /* Each interrupt line is assigned to one channel, it keeps better performance.
     * Set it to FALSE if you use more than one channel per core with the same interrupt line. */
    maple_pufft_ch_open_params.ch_params.single_channel_to_int = TRUE;
    maple_pufft_ch_open_params.ch_params.channel_location      = OS_MEM_LOCAL_CACHEABLE;
    maple_pufft_ch_open_params.ch_params.int_dispatcher        = osHwiDispatcher;
    //maple_pufft_ch_open_params.ch_params.int_num               = (os_hwi_handle)(maple_int_base[pufft_num] + OS_SOC_MAX_NUM_OF_CORES  + ch_num);
    maple_pufft_ch_open_params.ch_params.int_num               = (os_hwi_handle)(maple_int_base[pufft_num] + ch_num);
    maple_pufft_ch_open_params.ch_params.int_priority          = OS_HWI_PRIORITY4;
    /* Each core works with mmu_task_id = coreID + 1 on each Maple*/
    maple_pufft_ch_open_params.ch_params.mmu_task_id           = core_mmu_task_id;
 
    // FIXME: check int_num
    maple_pufft_ch_open_params.add_int[MAPLE_PUFFT_ACK_INT].int_enable      = ON;
    maple_pufft_ch_open_params.add_int[MAPLE_PUFFT_ACK_INT].int_dispatcher  = osHwiDispatcher;
    maple_pufft_ch_open_params.add_int[MAPLE_PUFFT_ACK_INT].int_num         = (os_hwi_handle)(maple_int_base[pufft_num] + ch_num + 1);
    maple_pufft_ch_open_params.add_int[MAPLE_PUFFT_ACK_INT].int_priority    = OS_HWI_PRIORITY4;
    maple_pufft_ch_open_params.add_int[MAPLE_PUFFT_ACK_INT].int_callback    = maple_pufft_ack_cb;
 
#if (USING_MAPLE_MMU == TRUE)
    /* All the inputs are at cacheable DDR0 */
    for (i = 0; i < MAPLE_PUFFT_STEERING_LAST; i++)
    {
        if (osGetClusterNum() == 0)
        {
            maple_pufft_ch_open_params.ch_params.steering_bits[i] = MAPLE_STEERING_CHB;//MAPLE_STEERING_DSP_CLUSTER_0;
        }
        else if (osGetClusterNum() == 1)
        {
            maple_pufft_ch_open_params.ch_params.steering_bits[i] = MAPLE_STEERING_DSP_CLUSTER_1;
        }
        else if (osGetClusterNum() == 2)
        {
            maple_pufft_ch_open_params.ch_params.steering_bits[i] = MAPLE_STEERING_DSP_CLUSTER_2;
        }
        else
            OS_ASSERT;
    }
    maple_pufft_ch_open_params.ch_params.steering_bits[MAPLE_PUFFT_STEERING_PRE_MULTI_VECTOR_PTR]  = MAPLE_STEERING_CHB;
    maple_pufft_ch_open_params.ch_params.steering_bits[MAPLE_PUFFT_STEERING_POST_MULTI_VECTOR_PTR] = MAPLE_STEERING_CHB;
#else // USING_MAPLE_MMU == FALSE
    for (i = 0; i < MAPLE_PUFFT_STEERING_LAST; i++)
    {
    	maple_pufft_ch_open_params.ch_params.steering_bits[i] = MAPLE_STEERING_CHB;
    }
#endif // USING_MAPLE_MMU
 
    ch_open_params.channel_num                      = ch_num;
    ch_open_params.num_jobs                         = 14*2;//PUFFT_MAX_NUM_BD_FOR_DISPACTH;
    ch_open_params.callback_parameter               = (void*)ch_open_params.channel_num;
    ch_open_params.error_callback_parameter         = 0;
    ch_open_params.heap                             = OS_MEM_LOCAL_CACHEABLE;
    ch_open_params.lld_params                       = &maple_pufft_ch_open_params;

    //printf("Opening PUFFT channel %d...\n", ch_num);
    status = osCopChannelOpen(pufft_handle[pufft_num], &pufft_ch_handle[pufft_num] , &ch_open_params);
    OS_ASSERT_COND(pufft_ch_handle[pufft_num] != NULL);
    //printf("PUFFT channel %d open - Done\n", ch_num);
 
#if (USING_MAPLE_MMU == TRUE)
    /* Make sure that channel does not translate virtual to physical */
    if (pufft_ch_handle[pufft_num] != NULL)
    {
        status = osCopChannelCtrl(&pufft_ch_handle[pufft_num], MAPLE_PE_CH_CMD_VIRT_TRANS_DISABLE, NULL);
        OS_ASSERT_COND(status == OS_SUCCESS);
    }
#else
    /* Make sure that channel translates virtual to physical */
    if (pufft_ch_handle[pufft_num] != NULL)
    {
        status = osCopChannelCtrl(&pufft_ch_handle[pufft_num], MAPLE_PE_CH_CMD_VIRT_TRANS_ENABLE, NULL);
        OS_ASSERT_COND(status == OS_SUCCESS);
    }
#endif
}

/***************************************************************************/
static os_status appFtpeInit(int32_t maple_num, int32_t ftpe_num, int32_t ch_num)
{
    os_status status = OS_SUCCESS;
 
    cop_dev_open_params_t           cop_dev_open_params       = {0};
    maple_open_params_t             maple_open_params         = {0};
    ftpe_open_params_t              ftpe_open_params          = {0};
    cop_ch_open_params_t            ch_open_params            = {0};
    maple_ftpe_ch_open_params_t     maple_ftpe_ch_open_params = {0};
    int core_id, i;
    char *maple_name, *ftpe_name;
    extern maple_ftpe_init_params_t maple_ftpe_init_params;
    extern maple_init_params_t maple_init_params;    
    
    core_id = osGetCoreID();

    /***** Open FTPE device *****/
    ftpe_open_params.maple_handle           = maple_handle[maple_num];
    ftpe_open_params.maple_pe_bd_priority   = MAPLE_PE_BD_RING_H_L;
    ftpe_open_params.maple_pe_num_bd        = MAPLE_PE_NUM_BD_RINGS_6;
    ftpe_open_params.ftpe_config            = 0;
    cop_dev_open_params.dispatch_callback   = mapleFtpeDispatchCb;
    cop_dev_open_params.lld_params          = &ftpe_open_params;

#if 0
    /* Initialize data size sets */
    static ftpe_size_params_t  data_size_sets[FTPE_PARAM_DSS_NUM] = {0};

    for (i = 0; i< FTPE_PARAM_DSS_NUM; i++)
    {
        data_size_sets[i].ds0 = 36; // tl = 128
        data_size_sets[i].ds1 = 90; // tl = 256
        data_size_sets[i].ds2 = 150;// tl = 512
        data_size_sets[i].ds3 = 300;// tl = 1024
        data_size_sets[i].ds4 = 450;// tl = 1536
        data_size_sets[i].ds5 = 600;// tl = 2048
    }
    /* Initialize data size sets */
    ftpe_open_params.ftpe_data_size_sets    = &data_size_sets;
#else
    // Use hardware defaults
    ftpe_open_params.ftpe_data_size_sets    = NULL;
#endif
    maple_name = maple_init_params.device_init[maple_num].name;
    ftpe_name = maple_ftpe_init_params.pe_init[maple_num * 3 + ftpe_num].name;
    //printf("Opening %s...\n", ftpe_name);
    ftpe_handle = osCopDeviceOpen(ftpe_name, &cop_dev_open_params);
    OS_ASSERT_COND(ftpe_handle != NULL);
    //printf("Opening %s - Done \n", ftpe_name);
 
    /***** Open Shared FTPE channel *****/
    /* Each core works with mmu_task_id = coreID + 1 on each Maple*/
    maple_ftpe_ch_open_params.mmu_task_id = core_mmu_task_id;
    /* All the inputs are at shared_data_ddr0_cacheable */
#if (USING_MAPLE_MMU == TRUE)
    for (i = 0; i < MAPLE_FTPE_STEERING_LAST; i++)
    {
        if (osGetClusterNum() == 0)
        	maple_ftpe_ch_open_params.steering_bits[i] = MAPLE_STEERING_DSP_CLUSTER_0;
        else if (osGetClusterNum() == 1)
        	maple_ftpe_ch_open_params.steering_bits[i] = MAPLE_STEERING_DSP_CLUSTER_1;
        else if (osGetClusterNum() == 2)
        	maple_ftpe_ch_open_params.steering_bits[i] = MAPLE_STEERING_DSP_CLUSTER_2;
        else
        	OS_ASSERT;
    }
#else
    for (i = 0; i < MAPLE_FTPE_STEERING_LAST; i++)
    {
        maple_ftpe_ch_open_params.steering_bits[i] = MAPLE_STEERING_CHB;
    }
#endif

    maple_ftpe_ch_open_params.flags         = 0;
    maple_ftpe_ch_open_params.high_priority = TRUE;

    maple_ftpe_ch_open_params.int_enable            = TRUE;
    maple_ftpe_ch_open_params.int_dispatcher        = osHwiDispatcher;
    maple_ftpe_ch_open_params.single_channel_to_int = TRUE;
    maple_ftpe_ch_open_params.no_automatic_reap = FALSE;
    maple_ftpe_ch_open_params.channel_location  = OS_MEM_LOCAL_CACHEABLE;
    maple_ftpe_ch_open_params.int_num           = (os_hwi_handle)(maple_int_base[ftpe_num] + 8 + osGetCoreID());
    maple_ftpe_ch_open_params.int_priority      = OS_HWI_PRIORITY3;
    maple_ftpe_ch_open_params.no_translation    = USING_MAPLE_MMU;
    maple_ftpe_ch_open_params.use_go_function   = FALSE;
 
    ch_open_params.channel_num                  = (uint16_t)ch_num;
    ch_open_params.num_jobs                     = FTPE_MAX_NUM_BD_FOR_DISPACTH; /* Max number of jobs channel can handle */
    ch_open_params.callback_parameter           = (void*)ch_open_params.channel_num;
    ch_open_params.error_callback_parameter     = 0;
    ch_open_params.heap                         = OS_MEM_LOCAL_CACHEABLE;
    ch_open_params.lld_params                   = &maple_ftpe_ch_open_params;

    /*
    status = osCopSharedChannelOpen(ftpe_handle, &ftpe_ch_handle , &ch_open_params,
    		ftpe_ch_mode[osGetCoreID()], ftpe_queue[ch_open_params.channel_num]);
    		*/
    status = osCopChannelOpen(ftpe_handle, &ftpe_ch_handle , &ch_open_params);

    OS_ASSERT_COND(status == OS_SUCCESS);
    OS_ASSERT_COND(ftpe_ch_handle != NULL);
 
    osWaitForAllCores();
    
	status = osEventSemaphoreFind(&evt_fft_done);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventSemaphoreCreate(evt_fft_done, 0);
	OS_ASSERT_COND(status == OS_SUCCESS);
    
    return status;
}

/*****************************************************************************/
static void appTvpeInit(int tvpe_num, int ch_num)
{
    extern maple_tvpe_init_params_t maple_tvpe_init_params;
    char*   tvpe_name  = maple_tvpe_init_params.pe_init[tvpe_num].name;
    cop_dev_open_params_t           cop_dev_open_params = {0};
    tvpe_open_params_t              tvpe_open_params    = {0};
    tvpe_viterbi_params_t           tvpe_viterbi_params = {0};
    tvpe_turbo_params_t             tvpe_turbo_params   = {0};
    int i;
 
    // Initialize puncturing vector
    for (i=0; i < NUM_VITERBI_PUNC_VECTORS; i++)
    {
        tvpe_viterbi_params.punturing_vector[i]   = 0x7fffffffffffffff;
        tvpe_viterbi_params.puncturing_pattern[i] = 1;
    }
    
    /* Set polynomials sets */
    tvpe_viterbi_params.polynomial_set[0].m_polygen0 = 0133;
    tvpe_viterbi_params.polynomial_set[0].m_polygen1 = 0171;
    tvpe_viterbi_params.polynomial_set[0].m_polygen2 = 0165;
    tvpe_viterbi_params.polynomial_set[0].m_polygen3 = 0x00;

    //tvpe_turbo_params.apq_threshold         = 150;
 
    tvpe_open_params.maple_handle           = maple_handle[tvpe_num];
    tvpe_open_params.maple_pe_bd_priority   = MAPLE_PE_BD_RING_H_L;
    tvpe_open_params.maple_pe_num_bd        = MAPLE_PE_NUM_BD_RINGS_6;
    tvpe_open_params.tvpe_turbo_params      = &tvpe_turbo_params;
    tvpe_open_params.tvpe_viterbi_params    = &tvpe_viterbi_params;
 
    cop_dev_open_params.dispatch_callback   = mapleTvpeDispatchCb;
    cop_dev_open_params.error_callback      = NULL;
    cop_dev_open_params.lld_params          = &tvpe_open_params;

    printf("Opening TVPE %s...\n", tvpe_name);
    tvpe_handle[tvpe_num] = osCopDeviceOpen(tvpe_name, &cop_dev_open_params);
    OS_ASSERT_COND(tvpe_handle[tvpe_num] != NULL);
    printf("Opened %s device - Done \n", tvpe_name);
#if 1
    maple_tvpe_ch_open_params_t maple_tvpe_ch_open_params = {0};
    cop_ch_open_params_t        ch_open_params            = {0};
    os_status                   status;

    maple_tvpe_ch_open_params.flags                 = 0;
    maple_tvpe_ch_open_params.no_automatic_reap     = FALSE;
    maple_tvpe_ch_open_params.high_priority         = FALSE;
    maple_tvpe_ch_open_params.int_enable            = USING_INTERRUPT;
    maple_tvpe_ch_open_params.no_translation        = USING_MAPLE_MMU;
    /* Each interrupt line is assigned to one channel, it keeps better performance.
     * Set it to FALSE if you use more than one channel per core with the same interrupt line. */
    maple_tvpe_ch_open_params.single_channel_to_int = TRUE;
    maple_tvpe_ch_open_params.channel_location      = OS_MEM_LOCAL_CACHEABLE;
    maple_tvpe_ch_open_params.int_dispatcher        = osHwiDispatcher;
    maple_tvpe_ch_open_params.int_num               = (os_hwi_handle)(maple_int_base[tvpe_num] + 2 + ch_num);
    maple_tvpe_ch_open_params.int_priority          = OS_HWI_PRIORITY3;
    /* Each core works with mmu_task_id = coreID + 1 on each Maple*/
    maple_tvpe_ch_open_params.mmu_task_id           = core_mmu_task_id;

#if (USING_MAPLE_MMU == TRUE)
    /* All the inputs are at cacheable DDR0 */
    if (osGetClusterNum() == 0)
    {
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARD_OUTPUT] = MAPLE_STEERING_DSP_CLUSTER_0;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_INPUT]       = MAPLE_STEERING_DSP_CLUSTER_0;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_SOFT_OUTPUT] = MAPLE_STEERING_DSP_CLUSTER_0;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_E_PARAM_PTR] = MAPLE_STEERING_DSP_CLUSTER_0;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARQ_INPUT]  = MAPLE_STEERING_DSP_CLUSTER_0;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARQ_OUTPUT] = MAPLE_STEERING_DSP_CLUSTER_0;
    }
    else if (osGetClusterNum() == 1)
    {
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARD_OUTPUT] = MAPLE_STEERING_DSP_CLUSTER_1;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_INPUT]       = MAPLE_STEERING_DSP_CLUSTER_1;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_SOFT_OUTPUT] = MAPLE_STEERING_DSP_CLUSTER_1;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_E_PARAM_PTR] = MAPLE_STEERING_DSP_CLUSTER_1;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARQ_INPUT]  = MAPLE_STEERING_DSP_CLUSTER_1;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARQ_OUTPUT] = MAPLE_STEERING_DSP_CLUSTER_1;
    }
    else if (osGetClusterNum() == 2)
    {
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARD_OUTPUT] = MAPLE_STEERING_DSP_CLUSTER_2;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_INPUT]       = MAPLE_STEERING_DSP_CLUSTER_2;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_SOFT_OUTPUT] = MAPLE_STEERING_DSP_CLUSTER_2;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_E_PARAM_PTR] = MAPLE_STEERING_DSP_CLUSTER_2;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARQ_INPUT]  = MAPLE_STEERING_DSP_CLUSTER_2;
        maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARQ_OUTPUT] = MAPLE_STEERING_DSP_CLUSTER_2;
    }
    else
        OS_ASSERT;
#else // not USING_MAPLE_MMU
    maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARD_OUTPUT] = MAPLE_STEERING_CHB;
    maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_INPUT]       = MAPLE_STEERING_CHB;
    maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_SOFT_OUTPUT] = MAPLE_STEERING_CHB;
    maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_E_PARAM_PTR] = MAPLE_STEERING_CHB;
    maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARQ_INPUT]  = MAPLE_STEERING_CHB;
    maple_tvpe_ch_open_params.steering_bits[MAPLE_TVPE_STEERING_HARQ_OUTPUT] = MAPLE_STEERING_CHB;
#endif
 
    ch_open_params.channel_num                      = ch_num;
    ch_open_params.num_jobs                         = NUM_TVPE_USED * 8;
    ch_open_params.callback_parameter               = (void*)ch_open_params.channel_num;
    ch_open_params.error_callback_parameter         = 0;
    ch_open_params.heap                             = OS_MEM_LOCAL_CACHEABLE;
    ch_open_params.lld_params                       = &maple_tvpe_ch_open_params;

    printf("Opening TVPE channel %d...\n", ch_num);
    status = osCopChannelOpen(tvpe_handle[tvpe_num], &tvpe_ch_handle[tvpe_num] , &ch_open_params);
    OS_ASSERT_COND(tvpe_ch_handle[tvpe_num] != NULL);
    printf("TVPE channel %d open - Done\n", ch_num);
#endif
}

static os_status appPuschInit(int maple_num, int pusch_num)
{
	os_status   status;
	cop_dev_open_params_t           cop_dev_open_params={0};
	maple_open_params_t             maple_open_params={0};
	pusch_open_params_t             pusch_open_params={0};
	cop_ch_open_params_t            ch_open_params={0};
	maple_pusch_ch_open_params_t    maple_pusch_ch_open_params={0};
	maple3lw_dev_init_params_t dev_init_params = {0};       /* Use LTE structure for LTE */
	uint8_t channel_num=0, i;
	uint8_t core_id = osGetCoreID();
	
#if (PUSCH_MMSE_EN == ON)
    if (osGetCoreID() == APP_EQPE_CORE)
    	eqpeInit(maple_handle);
    else
    {
#endif
    	
    /* Open pusch device */
    pusch_open_params.maple_handle              = maple_handle[maple_num];
    pusch_open_params.maple_pe_max_num_bd_rings = MAPLE_PE_NUM_BD_RINGS_6;
    pusch_open_params.maple_pe_bd_priority      = MAPLE_PE_BD_RING_H_L;
    cop_dev_open_params.dispatch_callback       = maplePuschAllUsersDoneCb;
    //cop_dev_open_params.error_callback          = mapleErrorCb;
    cop_dev_open_params.lld_params              = &pusch_open_params;

#if (MAPLE_0_PUSCH == ON)
    pusch_handle[pusch_num] = osCopDeviceOpen(PUSCH_DEV_NAME_0, &cop_dev_open_params);
    OS_ASSERT_COND(pusch_handle != NULL);
#else if (MAPLE_1_PUSCH == ON)
    pusch_handle = osCopDeviceOpen(PUSCH_DEV_NAME_1, &cop_dev_open_params);
    OS_ASSERT_COND(pusch_handle != NULL);
#endif


    /* Open pusch channel */
    /* Each core works with mmu_task_id = coreID + 1 on each Maple*/
    maple_pusch_ch_open_params.pe_ch_params.mmu_task_id       = core_id + 1;
    maple_pusch_ch_open_params.pe_ch_params.flags             = 0;
    maple_pusch_ch_open_params.pe_ch_params.no_automatic_reap = FALSE;
    maple_pusch_ch_open_params.pe_ch_params.channel_location  = OS_MEM_LOCAL_CACHEABLE;
    maple_pusch_ch_open_params.pe_ch_params.int_priority      = OS_HWI_PRIORITY0;
    maple_pusch_ch_open_params.pe_ch_params.int_dispatcher   = osHwiDispatcher;
    maple_pusch_ch_open_params.pe_ch_params.high_priority    = TRUE;
    maple_pusch_ch_open_params.pe_ch_params.int_enable       = ON;
    maple_pusch_ch_open_params.pe_ch_params.no_translation   = USING_MAPLE_MMU;
    maple_pusch_ch_open_params.pe_ch_params.single_channel_to_int = TRUE;
    maple_pusch_ch_open_params.done_cb            = maplePuschDispatchCb;
    maple_pusch_ch_open_params.all_ctrl_done_cb   = maplePuschAllCtrlDoneCb;
 
    maple_pusch_ch_open_params.ctrl_interrupt.int_enable  = ON;
    maple_pusch_ch_open_params.ctrl_interrupt.int_callback = maplePuschCtrlCb;
    maple_pusch_ch_open_params.ctrl_interrupt.int_priority = OS_HWI_PRIORITY0;
    maple_pusch_ch_open_params.ctrl_interrupt.int_dispatcher = osHwiDispatcher;

    maple_pusch_ch_open_params.pe_ch_params.high_priority     = TRUE;
 
    maple_pusch_ch_open_params.eq_term_interrupt.int_enable   = OFF;
    maple_pusch_ch_open_params.eq_term_interrupt.int_callback = maplePuschEqTermCb;
    maple_pusch_ch_open_params.eq_term_interrupt.int_priority = OS_HWI_PRIORITY0;
    maple_pusch_ch_open_params.eq_term_interrupt.int_dispatcher = osHwiDispatcher;
 
    maple_pusch_ch_open_params.pu_term_interrupt.int_enable   = OFF;
    maple_pusch_ch_open_params.pu_term_interrupt.int_callback = maplePuschPuTermCb;
    maple_pusch_ch_open_params.pu_term_interrupt.int_priority = OS_HWI_PRIORITY0;
    maple_pusch_ch_open_params.pu_term_interrupt.int_dispatcher = osHwiDispatcher;

    maple_pusch_ch_open_params.pe_ch_params.int_num      = (os_hwi_handle)(OS_INT_MAPLE_0_CH_0 + 12 + (0 * OS_NUM_OF_CORES) + core_id);
    maple_pusch_ch_open_params.ctrl_interrupt.int_num    = (os_hwi_handle)(OS_INT_MAPLE_0_CH_0 + 12 + (1 * OS_NUM_OF_CORES) + core_id);
    		//(os_hwi_handle)(maple_int_base[pusch_num] + 9 + core_id);
    maple_pusch_ch_open_params.eq_term_interrupt.int_num = (os_hwi_handle)(OS_INT_MAPLE_0_CH_0 + 12 + (2 * OS_NUM_OF_CORES) + core_id);
    maple_pusch_ch_open_params.pu_term_interrupt.int_num = (os_hwi_handle)(OS_INT_MAPLE_0_CH_0 + 12 + (3 * OS_NUM_OF_CORES) + core_id);
 
#if (USING_MAPLE_MMU == TRUE)
    /* All the inputs are at cacheable DDR0 */
    for (i = 0; i < MAPLE_PUSCH_STEERING_LAST; i++)
    {
        if (osGetClusterNum() == 0)
        {
            maple_pusch_ch_open_params.pe_ch_params.steering_bits[i] = MAPLE_STEERING_DSP_CLUSTER_0;
        }
        else if (osGetClusterNum() == 1)
        {
            maple_pusch_ch_open_params.pe_ch_params.steering_bits[i] = MAPLE_STEERING_DSP_CLUSTER_1;
        }
        else if (osGetClusterNum() == 2)
        {
            maple_pusch_ch_open_params.pe_ch_params.steering_bits[i] = MAPLE_STEERING_DSP_CLUSTER_2;
        }
        else
            OS_ASSERT;
    }
#else // USING_MAPLE_MMU
    for (i = 0; i < MAPLE_PUSCH_STEERING_LAST; i++)
    {
        maple_pusch_ch_open_params.pe_ch_params.steering_bits[i] = MAPLE_STEERING_CHB;
    }
#endif

    ch_open_params.channel_num                  = (uint16_t) core_id;
    ch_open_params.num_jobs                     = PUSCH_MAX_NUM_BD_FOR_DISPACTH; /* Max number of jobs channel can handle */
    ch_open_params.callback_parameter           = (void*)ch_open_params.channel_num;
    ch_open_params.error_callback_parameter     = 0;
    ch_open_params.lld_params                   = &maple_pusch_ch_open_params;
    ch_open_params.heap                         = OS_MEM_LOCAL_CACHEABLE;

    status = osCopChannelOpen(pusch_handle[pusch_num], &pusch_ch_handle[0], &ch_open_params);
    OS_ASSERT_COND(status == OS_SUCCESS);

#if (USING_MAPLE_MMU == TRUE)
    /* Make sure that channel does not translate virtual to physical */
    if (pusch_ch_handle != NULL)
    {
        status = osCopChannelCtrl(&pusch_ch_handle[0], MAPLE_PE_CH_CMD_VIRT_TRANS_DISABLE, NULL);
        OS_ASSERT_COND(status == OS_SUCCESS);
    }
#else
    /* Make sure that channel translates virtual to physical */
    if (pusch_ch_handle != NULL)
    {
        status = osCopChannelCtrl(&pusch_ch_handle, MAPLE_PE_CH_CMD_VIRT_TRANS_ENABLE, NULL);
        OS_ASSERT_COND(status == OS_SUCCESS);
    }
#endif

#if (PUSCH_MMSE_EN == ON)
    }
#endif
 
    return OS_SUCCESS;
}	

/***************************************************************************/
static os_status appPdschInit(int maple_num, int pdsch_num)
{
    //maple_pdsch_ch_open_params_t ch_open_params      = {0};
    cop_dev_open_params_t        cop_dev_open_params = {0};
    //maple_open_params_t          maple_open_params   = {0};
    //cop_ch_open_params_t         cop_ch_open_params  = {0};
    //maple3lw_dev_init_params_t   dev_init_params     = {0};
    pdsch_sec_reconfig_param_t sec_reconf = {0};
    os_phys_ptr phys_ptr = 0;
    os_status status;
    //uint32_t core_id = osGetCoreID();
    uint32_t ii, j;
    lte_enodeb_t *enodeb;
    pdsch_open_params_t    pdsch_open_params   = {0};
    
    pdsch_open_params.maple_handle = maple_handle[maple_num];
 
    for (ii = 0; ii < NUM_ENODEB; ii++)
    {
    	enodeb = &g_enodeb_inst[ii];
    	
    	if(enodeb->state != L1_CONFIGURED)
    		continue;
    	
        // MNOS, cell_id, mbsfn_id, antx_out_size and BeamForaming (for MAPLE-B3 Rev2) are set using reconfiguration mechanism, see pdsch_sector_reconfigure()
        pdsch_open_params.cell_config[enodeb->sector].cell_id   = enodeb->N_id_cell;
        pdsch_open_params.cell_config[enodeb->sector].mbsfn_id  = 0; // cell_config[i].mbsfn_id;
        pdsch_open_params.cell_config[enodeb->sector].mnos      = 6; // FIXME: change for CPRI to 2

        //pdsch_open_params.antx_out_size[enodeb->sector] = MAX_ANT_DATA_SIZE(enodeb->fp.LTE_N_RB_DL); // MAX_TEST_ANT_DATA_SIZE(sector_bw);
        pdsch_open_params.antx_out_size[enodeb->sector] = MAX_ANT_DATA_SIZE(100); // MAX_TEST_ANT_DATA_SIZE(sector_bw);
        
        for (j = 0; j < enodeb->fp.LTE_N_PHYS_ANTENNAS_TX; j++)
        {
            pdsch_open_params.antx_out_addr[enodeb->sector][j] = 0; // ant_out_base[i][j];
#if (USING_MAPLE_MMU == FALSE)
            status = osMmuDataVirtToPhys(pdsch_open_params.antx_out_addr[i][j], &phys_ptr);
            OS_ASSERT_COND(phys_ptr != NULL);
            pdsch_open_params.antx_out_addr[i][j] = (os_virt_ptr)phys_ptr;
            OS_ASSERT_COND(status == OS_SUCCESS);
#endif
        }
 
		pdsch_open_params.bf_config[MAPLE_PDSCH_BF_CONFIG_CSRS][enodeb->sector]    = 0; // bf_config[MAPLE_PDSCH_BF_CONFIG_CSRS][i];
		pdsch_open_params.bf_config[MAPLE_PDSCH_BF_CONFIG_MBSFNRS][enodeb->sector] = 0; // bf_config[MAPLE_PDSCH_BF_CONFIG_MBSFNRS][i];
		pdsch_open_params.bf_config[MAPLE_PDSCH_BF_CONFIG_POSRS][enodeb->sector]   = 0; // bf_config[MAPLE_PDSCH_BF_CONFIG_POSRS][i];
		pdsch_open_params.bf_config[MAPLE_PDSCH_BF_CONFIG_PSS][enodeb->sector]     = 0; // bf_config[MAPLE_PDSCH_BF_CONFIG_PSS][i];
		pdsch_open_params.bf_config[MAPLE_PDSCH_BF_CONFIG_SSS][enodeb->sector]     = 0; // bf_config[MAPLE_PDSCH_BF_CONFIG_SSS][i];
		pdsch_open_params.bf_config[MAPLE_PDSCH_BF_CONFIG_CSIRS][enodeb->sector]   = 0; // bf_config[MAPLE_PDSCH_BF_CONFIG_CSIRS][i];
		pdsch_open_params.bf_config[MAPLE_PDSCH_BF_CONFIG_PBCH][enodeb->sector]    = 0; // bf_config[MAPLE_PDSCH_BF_CONFIG_PBCH][i];
		pdsch_open_params.bf_config[MAPLE_PDSCH_BF_CONFIG_UE][enodeb->sector]      = 0; // bf_config[MAPLE_PDSCH_BF_CONFIG_UE][i];
    }
	
    cop_dev_open_params.dispatch_callback = appPdschDispatchCb;
    cop_dev_open_params.lld_params = &pdsch_open_params;
    pdsch_handle[pdsch_num] = osCopDeviceOpen(MAPLE_0_PDSCH_NAME, &cop_dev_open_params);
    OS_ASSERT_COND(pdsch_handle[pdsch_num] != NULL);

    return OS_SUCCESS;
}

/***************************************************************************/
static os_status appPdschChannelInit(int pdsch_num, int ch_num)
{
    maple_pdsch_ch_open_params_t ch_open_params      = {0};
    cop_dev_open_params_t        cop_dev_open_params = {0};
    maple_open_params_t          maple_open_params   = {0};
    cop_ch_open_params_t         cop_ch_open_params  = {0};
    maple3lw_dev_init_params_t   dev_init_params     = {0};
    pdsch_sec_reconfig_param_t sec_reconf = {0};
    os_phys_ptr phys_ptr = 0;
    os_status status;
    uint32_t core_id = osGetCoreID();
    int i, j;

	ch_open_params.ch_params.flags = 0;
	ch_open_params.ch_params.no_automatic_reap= FALSE;
	ch_open_params.ch_params.no_translation   = USING_MAPLE_MMU;
	ch_open_params.ch_params.channel_location = OS_MEM_LOCAL_CACHEABLE;
	ch_open_params.ch_params.int_priority     = OS_HWI_PRIORITY3;
	ch_open_params.ch_params.int_dispatcher = osHwiDispatcher;
	ch_open_params.ch_params.high_priority    = TRUE;
	ch_open_params.ch_params.int_enable       = TRUE;
	ch_open_params.ch_params.single_channel_to_int = TRUE;
	// PDSCH int_num after PUFFT interrupts (0-3)
	ch_open_params.ch_params.int_num         = ((os_hwi_handle)(maple_int_base[pdsch_num] + 4 + ch_num));
	ch_open_params.ch_params.mmu_task_id     = core_mmu_task_id;
	for (j = 0; j < MAPLE_PDSCH_STEERING_LAST; j++)
	{
#if (USING_MAPLE_MMU == TRUE)
		if (osGetClusterNum() == 0)
		{
			ch_open_params.ch_params.steering_bits[j] = MAPLE_STEERING_DSP_CLUSTER_0;
		}
		else if (osGetClusterNum() == 1)
		{
			ch_open_params.ch_params.steering_bits[j] = MAPLE_STEERING_DSP_CLUSTER_1;
		}
		else if (osGetClusterNum() == 2)
		{
			ch_open_params.ch_params.steering_bits[j] = MAPLE_STEERING_DSP_CLUSTER_2;
		}
		else
			OS_ASSERT;
#else
	// HW limitation: if MAPLE MMU is OFF steering bits 0,1,2 can't be used
	ch_open_params.ch_params.steering_bits[j] = MAPLE_STEERING_CHB;
#endif
	}
	
	// FIXME: check int_num
	ch_open_params.add_int[MAPLE_PDSCH_SYMBOL_INT].int_enable      = OFF;
	ch_open_params.add_int[MAPLE_PDSCH_SYMBOL_INT].int_callback    = appPdschSymCb;
	ch_open_params.add_int[MAPLE_PDSCH_SYMBOL_INT].int_num         = (os_hwi_handle)(OS_INT_MAPLE_0_CH_0 +  OS_SOC_MAX_NUM_OF_CORES + ch_num);
	ch_open_params.add_int[MAPLE_PDSCH_SYMBOL_INT].int_priority    = OS_HWI_PRIORITY2;
	ch_open_params.add_int[MAPLE_PDSCH_SYMBOL_INT].int_dispatcher  = osHwiDispatcher;

	// FIXME: check int_num
	ch_open_params.add_int[MAPLE_PDSCH_ACK_INT].int_enable      = OFF;
	ch_open_params.add_int[MAPLE_PDSCH_ACK_INT].int_callback    = NULL;
	ch_open_params.add_int[MAPLE_PDSCH_ACK_INT].int_num         = (os_hwi_handle)(OS_INT_MAPLE_0_CH_16 + OS_SOC_MAX_NUM_OF_CORES + ch_num);
	ch_open_params.add_int[MAPLE_PDSCH_ACK_INT].int_priority    = OS_HWI_LAST_PRIORITY;
	ch_open_params.add_int[MAPLE_PDSCH_ACK_INT].int_dispatcher  = osHwiDispatcher;

	cop_ch_open_params.num_jobs                    = PDSCH_MAX_NUM_BD_FOR_DISPACTH;
	cop_ch_open_params.heap = OS_MEM_LOCAL_CACHEABLE;
	cop_ch_open_params.lld_params = &ch_open_params;
	cop_ch_open_params.channel_num =  ch_num ;
	cop_ch_open_params.callback_parameter = (void*)cop_ch_open_params.channel_num;
	cop_ch_open_params.error_callback_parameter = (void*)cop_ch_open_params.channel_num;
	ch_open_params.late_pcr_en = FALSE;

	status = osCopChannelOpen(pdsch_handle[pdsch_num], &pdsch_ch_handle[ch_num], &cop_ch_open_params);
	OS_ASSERT_COND(pdsch_ch_handle[ch_num] != NULL);

#if 0
    osWaitForAllCores();

    // Set padding pattern
    status = osCopDeviceCtrl(pdsch_handle, PDSCH_DEV_CMD_PAD_DATA_SET , &pad_data);
    OS_ASSERT_COND(status == OS_SUCCESS);
 
    if (MPDSCHPDC1P_en)
    {
    	status = osCopDeviceCtrl(pdsch_handle, PDSCH_DEV_CMD_PAD_DATA1_SET , &pad_data1);
    	OS_ASSERT_COND(status == OS_SUCCESS);
    }
 
    if (MPDSCHPDC2P_en)
    {
    	status = osCopDeviceCtrl(pdsch_handle, PDSCH_DEV_CMD_PAD_DATA2_SET , &pad_data2);
    	OS_ASSERT_COND(status == OS_SUCCESS);
    }
#endif
    return OS_SUCCESS;
}

os_status lte_maple_init(lte_enodeb_t *enodeb)
{
	cop_dev_open_params_t cop_dev_open_params;
	pdsch_open_params_t pdsch_open_params =
		{ 0 };
	cop_ch_open_params_t ch_open_params =
		{ 0 };
	maple_pdsch_ch_open_params_t maple_pdsch_ch_open_params =
		{ 0 };
	ftpe_open_params_t ftpe_open_params;
	maple_ftpe_ch_open_params_t maple_ftpe_ch_open_params;
	
    pufft_open_params_t pufft_open_params;
    maple_pufft_ch_open_params_t maple_pufft_ch_open_params;
    
    pusch_open_params_t pusch_open_params;
    maple_pusch_ch_open_params_t maple_pusch_ch_open_params;
    
    pufft_mpisr_ch_params_t mpisr_ch[4];
    uint32_t mpisr_tid;
    
	os_status status;
	uint32_t core_id, ch;
	int i;
    
    // Заполенние таблицы адресов буферов антенн
    for(int sector = 0; sector<MAPLE_NUM_SECTORS; sector++)
    {
    	for(int ant = 0; ant < MAPLE_NUM_ANT; ant++)
    	{
    		ant_in_virt_base[sector][ant] = &rx_buffer_raw[sector][ant][0];
    	}
    }
    
    for(i=0; i<NUM_MAPLES_USED; i++)
    {
    	appMapleInit(enodeb, 1, i);
    }
    
    
    for(i=0; i<NUM_PUFFTS_USED; i++)
    {
    	appPufftInit(i);
    }
    
    for(i=0; i<NUM_PUFFTS_USED; i++)
    {
    	appPufftChannelInit(i, i);
    }
    
#if 0    
    //Configure mpisr channel link:
    mpisr_ch[0].ch_link = 0x80000000;
    mpisr_ch[1].ch_link = 0x40000000;
    mpisr_ch[2].ch_link = 0x20000000;
    mpisr_ch[3].ch_link = 0x10000000;

    //Configure mpisr tid
    mpisr_tid = 0x01020304;
 
    if (osGetCoreID() == osGetMasterCore())
    {
    	for (i = 0 ; i < NUM_PUFFTS_USED ; i++)
    	{
    		int j;
    		for (j = 0 ; j < 4 ; j++)
    		{
    			mpisr_ch[j].mpisr_num = j ;
    			osCopDeviceCtrl(pufft_handle[i], PUFFT_SET_MPISR_CHANNELS, &mpisr_ch[j]);
    		}
 
    		osCopDeviceCtrl(pufft_handle[i], PUFFT_SET_MPISR_TID, &mpisr_tid);
    	}
    }
#endif
    
    for(i=0; i<NUM_PDSCH_USED; i++)
    {
    	appPdschInit(0, i);
    }
    
    appPdschChannelInit(0, 0);
#ifdef USE_TVPE
    appTvpeInit(0, 0);
#endif
    for(i=0; i<NUM_PUSCH_USED; i++)
    {
    	appPuschInit(0, i);
    }
    
    appFtpeInit(0, 0, 0);
    
	return OS_SUCCESS;
}

void lte_maple_close(lte_enodeb_t *enodeb)
{
	/* TODO: 
	 * Activate  Maple Soft Reset, param should be a typecast of maple_dev_init_params_t.
	 */
	
	int32_t i;
	os_status status;
	
	for(i=0; i<NUM_MAPLES_USED; i++)
	{
		status = osCopDeviceCtrl(maple_handle[i], MAPLE_CMD_SOFT_RESET, maple_open_params.dev_init_params);
		OS_ASSERT_COND(status == OS_SUCCESS);
	}

	/* Закрытие каналов PDSCH */
	for (i = 0; i < NUM_PDSCH_USED; i++)
	{
		status = osCopChannelClose(pdsch_ch_handle[i]);
	}
	
	/* Закрытие каналов PUFFT */
	for(i=0; i<NUM_PUFFTS_USED; i++)
	{
		status = osCopChannelClose(pufft_ch_handle[i]);
	}
	
	/* Закрытие каналов PUSCH */
	// TODO: implement PUSCH
	
	for(i=0; i<NUM_PUSCH_USED; i++)
	{
		status = osCopChannelClose(pusch_ch_handle[i]);
	}
	
	/* Закрытие канала eFTPE */
	status = osCopChannelClose(ftpe_ch_handle);
	
	/* Ждем завершение обработки */
	osTaskDelay(11);

	osEventSemaphoreDelete(evt_fft_done);
#ifdef USE_TVPE
	/* Закрытие канала eTVPE */
	for(i=0; i<NUM_TVPE_USED; i++)
	{
		status = osCopChannelClose(tvpe_ch_handle[i]);
	}
	osTaskDelay(11);
#endif
	
	/* Закрытие устройства */
	for(i=0; i<NUM_PUSCH_USED; i++)
	{
		osCopDeviceClose(pusch_handle[i]);
	}
	
	for(i=0; i< NUM_PUFFTS_USED; i++)
	{
		osCopDeviceClose(pufft_handle[i]);
	}
	
	osCopDeviceClose(ftpe_handle);
	
	for(i=0; i<NUM_PDSCH_USED; i++)
	{
		osCopDeviceClose(pdsch_handle[i]);
	}
#ifdef USE_TVPE
	for(i=0; i<NUM_TVPE_USED; i++)
	{
		osCopDeviceClose(tvpe_handle[i]);
	}
#endif
	for(i=0; i<NUM_MAPLES_USED; i++)
	{
		osCopDeviceClose(maple_handle[i]);
	}
}

#define NUM_OF_FAST     100
#define MASS_SMALL_SIZE 25
#define MASS_MED_SIZE   250
#define MASS_BIG_SIZE   2500

static void appBackground(void)
{
	os_status status;
	uint32_t dummy_msg;
	uint32_t prio;
	
	osL1dRecoveryDone();

#ifndef DEBUG_STUB	
    /* Send indication to PA that SC is ready */
    status = osIpcMessageSendPtr(ipc_ch[IPC_DEDICATED_CH_ID], &dummy_msg, 0, FALSE);
    OS_ASSERT_COND(status == OS_SUCCESS);
#endif
	osWait(100000);
 
    //waiting until getting a message back. hold will be released by appRxEcho
    while(g_ipc_pa_ready == 0)
    	;

   	INFO(DTRX, "PA became ready, firing L1 up!\n");
   	
	while (1)
	{
		//for(prio = OS_TASK_PRIORITY_HIGHEST; prio < OS_TASK_PRIORITY_LOWEST; prio++)
		//	osTaskYield(prio, TASK_NULL_ID, NULL);
	}
}

void ipc_send_msg(fapi_ipc_msg_t *msg)
{
	uint64_t phys_addr;
	
	// FIXME: динамический номер eNodeB
	if(!g_ipc_pa_ready)
		return;
	
	LOG_EVENT(LOGEVT_IPC_SEND, msg->channel_id);
	
	osMmuDataVirtToPhys((void *)msg->body_addr, &phys_addr);
	
	osCacheStoreBarrier(L1_L2_CACHED);
	
	msg->body_addr = phys_addr;
	
	osIpcMessageSendPtr(ipc_ch[IPC_SC2PA_CH_ID], msg, msg->length, TRUE);
}

#if 0
os_het_status_t ipc_send_indication(void *ch)
{
    os_ipc_channel_t *channel = (os_ipc_channel_t*)ch;
    uint32_t          indication_addr = 0;
 
    OS_ASSERT_COND(channel != NULL);
    OS_ASSERT_COND(channel->local_channel.producer_initialized == OS_HET_INITIALIZED);
    OS_ASSERT_COND(channel->local_channel.consumer_initialized == OS_HET_INITIALIZED);
    OS_ASSERT_COND(channel->producer_index == osGetCoreID());
 
    //channel->local_channel.ipc_ind = OS_HET_PA_MPIC;
    
    switch (channel->local_channel.ipc_ind)
    {
        case OS_HET_VIRTUAL_INT:
            indication_addr = (uint32_t)&g_dsp_ccsr_map->gic + channel->local_channel.ind_offset;
            WRITE_SYNCIO_UINT32(*((uint32_t*)indication_addr), channel->local_channel.ind_value);
            break;
        case OS_HET_PA_MPIC:
            indication_addr = (uint32_t)&g_dsp_pa_ccsr_map->pic + channel->local_channel.ind_offset;
            WRITE_SYNCIO_UINT32(*((uint32_t*)indication_addr), channel->local_channel.ind_value);
            break;
        case OS_HET_DSP_MESH:
            indication_addr = (uint32_t)&g_dsp_plat_map->mmu + channel->local_channel.ind_offset;
            WRITE_SYNCIO_UINT32(*((uint32_t*)indication_addr), channel->local_channel.ind_value);
            WRITE_SYNCIO_UINT32(*((uint32_t*)indication_addr), 0);
            break;
        default:
            OS_ASSERT;
            break;
    }
 
    return OS_HETERO_SUCCESS;
}
#endif

static os_het_status_t open_producer_ch(void *ch)
{
	os_het_status_t het_status;
	os_ipc_channel_producer_open_params_t producer =
		{ NULL, NULL, L1_L2_CACHED };

	producer.ch = ch;
	het_status = osIpcChannelProducerOpen(&producer);
	OS_ASSERT_COND(het_status == OS_SUCCESS);
	return het_status;
}

static os_het_status_t open_consumer_ch(void *ch, os_het_ipc_ind_t indication, void *callback)
{
	os_het_status_t het_status;
	os_ipc_channel_consumer_open_params_t consumer =
		{ NULL,
		/**< Pointer to the relevant channel as received from osIpcChannelIdFind*/
		OS_HET_IPC_MESSAGE_CH,
		/**< type of channel - message channel or pointer channel*/
		OS_HET_VIRTUAL_INT,
		/**< the indication type to use*/
		OS_HWI_PRIORITY3,
		/**< priority of interrupt, if used*/
		0,
		/**< argument of interrupt, if used*/
		FAPI_IPC_PA2SC_BD_RING_SIZE,
		/**< size of the mailbox BD ring*/
		FAPI_IPC_MAX_PA2SC_MSG_SIZE,
		/**< maximal size of the message that can be passed on this channel*/
		NULL,
		/**< callback function to call when receiving a message*/
		NULL,
		/**< only for a message channel - the buffer pool to allocate the buffers from*/
		L1_L2_CACHED
		/**< caching policy to use in the cannel. according to os_cache.h defines*/
		};

	consumer.ch = ch;
	consumer.indication_type = indication;
	consumer.callback = callback;
	consumer.buffers_pool = ipc_buf_pool;

	het_status = osIpcChannelConsumerOpen(&consumer);
	OS_ASSERT_COND(het_status == OS_SUCCESS);
	
	return het_status;
}

static void ipc_pa_to_sc_dedicated_cb(void* ch, void* data, uint32_t length)
{
	fapi_ipc_cfg_t *ipc_cfg = (fapi_ipc_cfg_t *)data;
	int32_t i;
	
	g_ipc_pa_ready = 1;
	
	if(ipc_cfg->magic != FAPI_IPC_CFG_MAGIC)
		return;
	
	memcpy(&g_fapi_ipc_cfg, ipc_cfg, sizeof(fapi_ipc_cfg_t));
	
	process_license(g_fapi_ipc_cfg.cfg_v1.license);
	
	if(ipc_cfg->version == 1)
	{
		/* Установка логлевелов */
		for(i=DTRX; i<DLASTCOMP; i++)
		{
			log_set_level((log_comp_id_t)i, (log_level_t)ipc_cfg->cfg_v1.loglevel);
		}
		
		/* Инициализация CPRI */
		extern cpri_init_params_t cpri_init_params_array[NUM_OF_USED_CPRI_UNITS];
		extern cpri_init_params_t (*cpri_init_params)[];
		extern cpri_global_init_params_t *cpri_global_params;
		extern os_status cpri_init_status;
		
		cpri_link_rate_t fapi_link_to_cpri_link[] = {CPRI_1228_LINK_RATE, CPRI_2457_LINK_RATE, CPRI_3072_LINK_RATE, CPRI_4914_LINK_RATE,
				CPRI_6144_LINK_RATE, CPRI_98304_LINK_RATE };

		if(ipc_cfg->cfg_v1.n_rrus > NUM_OF_USED_CPRI_UNITS || ipc_cfg->cfg_v1.n_rrus >= NUM_ENODEB)
			ipc_cfg->cfg_v1.n_rrus = (NUM_OF_USED_CPRI_UNITS < NUM_ENODEB) ? NUM_OF_USED_CPRI_UNITS : NUM_ENODEB;
		
		/* Заполнение структуры инициализации CPRI */
		cpri_global_params->cpri_num_of_used_units = ipc_cfg->cfg_v1.n_rrus;
		
		/* Использование скорости из конфига */
		int32_t min_rate = 255, max_rate = 0;
		//int32_t need_reinit = 0;
		
		for(i=0; i<ipc_cfg->cfg_v1.n_rrus; i++)
		{
			/*
			 * Нумерация CPRI статичная, всегда включены 4 интерфейсы: CPRI0, CPRI1, CPRI2, CPRI3
			 * Номер CPRI определяется при конфигуарции TRX, физическую конфигурацию CPRI трогать не надо
			if(cpri_init_params_array[i].cpri_num != (cpri_num_t)ipc_cfg->cfg_v1.rrus[i].cpri_port)
				need_reinit = 1;
			
			cpri_init_params_array[i].cpri_num = (cpri_num_t)ipc_cfg->cfg_v1.rrus[i].cpri_port;
			*/
			
			if(min_rate > ipc_cfg->cfg_v1.rrus[i].speed)
				min_rate = ipc_cfg->cfg_v1.rrus[i].speed;
			
			if(max_rate < ipc_cfg->cfg_v1.rrus[i].speed)
				max_rate = ipc_cfg->cfg_v1.rrus[i].speed;
			
			g_enodeb_inst[i].trx.cpri_no = ipc_cfg->cfg_v1.rrus[i].cpri_port;
			
			if(g_enodeb_inst[i].trx.cpri_no >= NUM_OF_USED_CPRI_UNITS)
			{
				ERROR(DCONFIG, "Invalid CPRI port number %i (valid: 0-%i)!\n",
						g_enodeb_inst[i].trx.cpri_no, NUM_OF_USED_CPRI_UNITS - 1);
				
				g_enodeb_inst[i].trx.cpri_no = 0;
			}
		}
#if 0
		cpri_global_params->group[0].minimal_accepted_link_rate = fapi_link_to_cpri_link[min_rate];
		cpri_global_params->group[0].maximal_desired_link_rate = fapi_link_to_cpri_link[max_rate];

		cpri_global_params->group[1].minimal_accepted_link_rate = fapi_link_to_cpri_link[min_rate];
		cpri_global_params->group[1].maximal_desired_link_rate = fapi_link_to_cpri_link[max_rate];
#else
		/* Использование фиксированной скорости 2.4576Gb */
		cpri_global_params->group[0].minimal_accepted_link_rate = fapi_link_to_cpri_link[1];
		cpri_global_params->group[0].maximal_desired_link_rate = fapi_link_to_cpri_link[1];

		cpri_global_params->group[1].minimal_accepted_link_rate = fapi_link_to_cpri_link[1];
		cpri_global_params->group[1].maximal_desired_link_rate = fapi_link_to_cpri_link[1];
#endif
		
		
		
		if(/*need_reinit || */cpri_global_params->cpri_num_of_used_units > 0 && cpri_init_status == 0)
		{
			/* Инициализация аппаратной части
			 * Если невозможно установить линк, то дальнейшая инициализация не производится
			 */
			if(cpriInitialize(cpri_global_params, cpri_init_params) == OS_SUCCESS)
			{
				cpri_init_status = OS_SUCCESS;
			}
			else
			{
				ERROR(DCONFIG, "Error initializing CPRI link!\n");
				cpri_init_status = OS_ERR_UNKNOWN;
			}
		}
	}
}

fapi_ipc_msg_t *ipc_recv_msg()
{
	void *msg_data;
	
	if(osIpcMessagePtrGet(ipc_ch[IPC_PA2SC_CH_ID], &msg_data) == OS_HETERO_SUCCESS)
		return msg_data;
	else
		return NULL;
}
static int ipc_init()
{
	os_status status;
	int i;
	
	g_ipc_pa_ready = 0;

	REPORT_EVENT(0);
	ipc_buf_space = osMalloc(MEM_PART_DATA_SIZE(FAPI_IPC_MAX_PA2SC_MSGS_NUM, FAPI_IPC_MAX_PA2SC_MSG_SIZE, ALIGNED_8_BYTES),
			OS_MEM_HET_DDR0_SHARED_CACHEABLE);
	OS_ASSERT_COND(ipc_buf_space != NULL);

	ipc_buf_pool = osMemPartCreate(FAPI_IPC_MAX_PA2SC_MSG_SIZE, FAPI_IPC_MAX_PA2SC_MSGS_NUM, ipc_buf_space, 
			ALIGNED_8_BYTES, OFFSET_0_BYTES, (os_mem_part_t *) ipc_buf_manager
#if (OS_MULTICORE == 1)
                                ,0
#endif
                                );
	
	OS_ASSERT_COND(ipc_buf_pool != NULL);

	for (i = 0; i < NUM_IPC_CH; i++)
	{
		ipc_ch[i] = osIpcChannelIdFind(i);
		OS_ASSERT_COND(ipc_ch[i] != NULL);
	}

	/* Open the producer channels */
	open_producer_ch(ipc_ch[IPC_DEDICATED_CH_ID]);
	open_producer_ch(ipc_ch[IPC_SC2PA_CH_ID]);

	/* Open the consumer channels */
	open_consumer_ch(ipc_ch[IPC_MSG_CH_ID], OS_HET_VIRTUAL_INT, ipc_pa_to_sc_dedicated_cb);
	open_consumer_ch(ipc_ch[IPC_PA2SC_CH_ID], OS_HET_VIRTUAL_INT, fapi_ipc_cb);

/*	status = osDebugHookCreate(OS_DEBUG_IPC_BASIC_SEND, debug_hook);
	RETURN_ASSERT_STATUS(status);
	status = osDebugHookCreate(OS_DEBUG_IPC_BASIC_RECEIVE, debug_hook);
	RETURN_ASSERT_STATUS(status);
*/

	REPORT_EVENT(1);
	
	return OS_SUCCESS;
}

static os_status appInit()
{
	os_status status = OS_SUCCESS;
	int32_t i;
	
	// FIXME: SC3900
#if 0
#ifdef ANT2_TEST_GPO89
    extern psc9x3x_pa_ccsr_t           *g_dsp_pa_ccsr_map;        /**< Global variable for accessing the Control, Configuration and Status Registers */
    
    uint32_t    reg;
 
    READ_UINT32(reg, g_dsp_pa_ccsr_map->device_config.pmuxcr2); //mux the pin for GPIO34
//    reg &= 0xFF3FFFFF;
    reg |= 0x00C00000;
    WRITE_UINT32(g_dsp_pa_ccsr_map->device_config.pmuxcr2, reg);
#endif
#endif
    
#ifdef APP_TEST_RECOVERY
	checksum = psc9x3xOsRecoveryCheck();
	status = psc9x3xOsRecoveryInit(&vnmi_handle, AppRecoveryCb, AppProgDataMmuDebugHook, AppProgDataMmuDebugHook,TRUE);
	OS_ASSERT_COND(status == OS_SUCCESS);
#endif // APP_TEST_RECOVERY

#ifndef DEBUG_STUB
	/* Init logging */
	log_init();
	
	/* Init IPC */
	ipc_init();
	
	/* Init FAPI */
	fapi_init();
	
	/* Should be before PDSCH device open */
	//appMemInit();

	liblte_init();
	
	for(i=0; i<NUM_ENODEB; i++)
	{
		lte_enodeb_t *enodeb = lte_enodeb_init(i);
	}
#endif
	
	return status;
}

void main()
{
	os_status status;

#ifdef DEBUG_STUB
	DEBUG();
#endif
	
	/* Отключение 32-бит сатурации для использования 40-бит аккумуляторов */
	setnosat();

	/* OS Initialization - call before any other OS API calls. */
	status = osInitialize();
	if (status != OS_SUCCESS)
		OS_ASSERT;
	
	//osL1dRecoveryDone();

	/* Interrupts are disabled until osStart() is called.
	 You must not enable interrupts at this point !!! */

	/* Place any required initialization code within appInit().
	 Using local variables inside main() is NOT recommended, because
	 its stack is never restored - osStart() never returns !!! */
	
	//DEBUG();
	
	status = appInit();
	if (status != OS_SUCCESS)
		OS_ASSERT;

	/* Start the OS with the background task. OS ticks now start.
	 appBackground() should not return unless there is an error. */
	status = osStart(appBackground);
	if (status != OS_SUCCESS)
		OS_ASSERT;

	/* Execution reaches this point only if an error occurs. */
}
