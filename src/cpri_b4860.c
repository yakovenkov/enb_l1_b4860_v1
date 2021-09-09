/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <time.h>
#include <string.h>
#include "smartdsp_os.h"
#include "os_config.h"
#include "stdio.h"
#include "os_mem.h"
#include "cpri.h"
#include "cpri_init.h"
#include "hw_timers32_init.h"
#include "hw_timers32.h"

#include "app_config.h"
#include "cpri_b4860.h"
#include "fapi_b4860.h"
#include "fapi_interface.h"
#include "fapi.h"

extern cpri_init_params_t cpri_init_params_array[NUM_OF_USED_CPRI_UNITS];
extern cpri_init_params_t (*cpri_init_params)[];
extern cpri_global_init_params_t *cpri_global_params;

uint8_t iq_rx_buffer[NUM_OF_USED_CPRI_UNITS][LTE_N_ANTENNAS_MAX][LTE_SAMPLES_PER_SUBFRAME_MAX * sizeof(Complex16)
		* NUM_CPRI_BUFFERS] __attribute__ ((section(".shared_data_ddr0_cacheable_bss"),aligned (RX_TRANSACTION_SIZE*64))); /* Rx Buffers */
uint8_t iq_tx_buffer[NUM_OF_USED_CPRI_UNITS][LTE_N_ANTENNAS_MAX][LTE_SAMPLES_PER_SUBFRAME_MAX * sizeof(Complex16)
		* NUM_CPRI_BUFFERS] __attribute__ ((section(".shared_data_ddr0_cacheable_bss"),aligned (TX_TRANSACTION_SIZE*64))); /* Tx Buffers */
uint8_t vss_rx_buffer[NUM_OF_USED_CPRI_UNITS][MAX_VSS_RX_BUFFER_SIZE] __attribute__ ((section(".shared_data_ddr0_cacheable_bss"),aligned (VSS_RX_TRANSACTION_SIZE*64))); /* Rx Buffers */
uint8_t vss_tx_buffer[NUM_OF_USED_CPRI_UNITS][MAX_VSS_TX_BUFFER_SIZE] __attribute__ ((section(".shared_data_ddr0_cacheable_bss"),aligned (VSS_TX_TRANSACTION_SIZE*64))); /* Tx Buffers */
uint8_t ethernet_buffers_space[MEM_PART_DATA_SIZE(ETHERNET_NUM_OF_BUFS, ETHERNET_DATA_SIZE, ALIGNED_64_BYTES)] __attribute__ ((section(".shared_data_ddr0_cacheable_bss"),aligned (VSS_TX_TRANSACTION_SIZE*64)));

volatile uint32_t received_symbol;
uint32_t frame_data[ETHERNET_DATA_SIZE];
volatile bool check_error = TRUE; // assert if error occurs

os_status cpri_init_status = 0;

/***
 * CPRI ethernet in/out buffers
 */
uint32_t rx_symbol_count[NUM_OF_CPRI_MODULES] =
	{ 0, 0 };
uint32_t tx_symbol_count[NUM_OF_CPRI_MODULES] =
	{ 0, 0 };
uint32_t rx_total_symbols = 0;
uint32_t tx_total_symbols = 0;

/***
 * CPRI Timers
 */

void cpri_timer_rx_cb(os_hwi_arg timer_num);
static void cpri_timer_tx_cb(os_hwi_arg timer_num);
int32_t is_cpri_counters_ok(lte_enodeb_t *enodeb);

timer32_open_params_t timer32_init_params[] =
	{
	/* дескриптор таймера передачи PDSCH -> CPRI */
		{ 14, //1st TX symbol size
			22, //sum of 1st 2 TX symbol sizes
			SOC_TIMER32_FREE_RUN, SOC_TIMER32_COUNT_RISING_EDGE, //SOC_TIMER32_COUNT_RISING_PRIMARY,
			SOC_TIMER32_COUNT_UP, NULL, //timer0Interrupt,
			OS_HWI_PRIORITY2, SOC_TIMER32_TIN0SOURCE, SOC_TIMER32_SECONDARY_NOT_IN_USE, //SOC_TIMER32_SECONDARY_TIN2,
			FALSE, SOC_TIMER32_PRELOAD_UPON_CMP1, SOC_TIMER32_PRELOAD_NEVER, FALSE, // MPIC interrupt
			TRUE, // MAPLE interrupt
			0 // MAPLE-0
		},
	/* Дескриптор таймера приема CPRI -> PUFFT */
		{
			13, //1st RX symbol size
			21, //sum of 1st 2 RX symbol sizes
			SOC_TIMER32_FREE_RUN, SOC_TIMER32_COUNT_RISING_PRIMARY, SOC_TIMER32_COUNT_UP, cpri_timer_tx_cb,
			OS_HWI_PRIORITY2, SOC_TIMER32_TIN1SOURCE, SOC_TIMER32_SECONDARY_NOT_IN_USE, //SOC_TIMER32_SECONDARY_TIN3,
			FALSE, SOC_TIMER32_PRELOAD_UPON_CMP1, SOC_TIMER32_PRELOAD_NEVER, FALSE, // MPIC interrupt
			FALSE, // MAPLE interrupt
			0 // MAPLE-0
		} };

/*
 * Subframe symbol sizes in terms of IQ_TX_THRESHOLD_SIZE.
 * These sizes are for demonstrating purposes only
 */
static uint32_t tx_symbols_sizes_in_subframes[2][LTE_NSYMB_PER_SUBFRAME] =
	{ 0 };
static volatile uint32_t tx_symbol_idx = 0;
static uint32_t tx_symbol_current_timer_preload_val;

/*
 * Subframe symbol sizes in terms of IQ_TX_THRESHOLD_SIZE.
 * These sizes are for demonstrating purposes only
 */
static uint32_t rx_symbols_sizes_in_subframes[2][LTE_NSYMB_PER_SUBFRAME] =
	{ 0 };
static volatile uint32_t rx_symbol_idx = 0;
static uint32_t rx_symbol_current_timer_preload_val;

static void cpri_timer_tx_cb(os_hwi_arg timer_num)
{
	hwTimer32ClearEvent((soc_timer32_num_t) timer_num);

	if (tx_symbol_idx < LTE_NSYMB_PER_SUBFRAME)
	{
		tx_symbol_current_timer_preload_val += tx_symbols_sizes_in_subframes[0][tx_symbol_idx];
		hwTimer32PreloadSet((soc_timer32_num_t) timer_num, tx_symbol_current_timer_preload_val);
	}
	else if (tx_symbol_idx < 2 * LTE_NSYMB_PER_SUBFRAME)
	{
		tx_symbol_current_timer_preload_val += tx_symbols_sizes_in_subframes[1][tx_symbol_idx - LTE_NSYMB_PER_SUBFRAME];
		hwTimer32PreloadSet((soc_timer32_num_t) timer_num, tx_symbol_current_timer_preload_val);
	}

	tx_symbol_idx++;

	if (tx_symbol_idx >= 2 * LTE_NSYMB_PER_SUBFRAME)
		tx_symbol_idx = 0;
}

void cpri_timer_rx_cb(os_hwi_arg timer_num)
{
	hwTimer32ClearEvent((soc_timer32_num_t) timer_num);

	if (rx_symbol_idx < LTE_NSYMB_PER_SUBFRAME)
	{
		rx_symbol_current_timer_preload_val += rx_symbols_sizes_in_subframes[0][rx_symbol_idx];
		hwTimer32PreloadSet((soc_timer32_num_t) timer_num, rx_symbol_current_timer_preload_val);
	}
	else if (rx_symbol_idx < 2 * LTE_NSYMB_PER_SUBFRAME)
	{
		rx_symbol_current_timer_preload_val += rx_symbols_sizes_in_subframes[1][rx_symbol_idx - LTE_NSYMB_PER_SUBFRAME];
		hwTimer32PreloadSet((soc_timer32_num_t) timer_num, rx_symbol_current_timer_preload_val);
	}

	rx_symbol_idx++;

	if (rx_symbol_idx >= 2 * LTE_NSYMB_PER_SUBFRAME)
		rx_symbol_idx = 0;
}

static void cpri_error_cb(void* param)
{
	cpri_error_param_t* error_param = (cpri_error_param_t*) param;

	//other errors
	if (check_error == FALSE)
	{
		return;
	}

	printf("Error occurred in CPRI %d\n", (uint8_t) error_param->cpri_num);
	printf("Error events are 0X%X\n", (uint8_t) error_param->error_events);
	if ((uint8_t) error_param->error_events & CPRI_RECEIVE_IQ_OVERRUN_ERROR)
	{
		printf("Overrun error encountered\n");
	}
	if ((uint8_t) error_param->error_events & CPRI_TRANSMIT_IQ_UNDERRUN_ERROR)
	{
		printf("Underrun error encountered\n");
	}

	OS_ASSERT;
}

static void cpri_iq_rx_cb(void* param)
{
	cpri_iq_int_cb_param_t* cb_param = (cpri_iq_int_cb_param_t*) param;

#if !defined(DEBUG_TX_BUFFER_TEST) && !defined(DEBUG_OPT_OFF) 
#pragma opt_level = "O3"
#endif

	volatile static uint64_t old_tick, new_tick, interval_cycles, rx_sym_cycles[10];
	volatile uint64_t loop_start, loop_end;
	static uint32_t rx_ul_count = 0;
	//uint32_t symbol_size;
	uint32_t j;
	lte_trx_t *trx;
	lte_ul_subframe_t *sf;
	os_status status;
	int32_t num_bds = LTE_NSYMB_PER_SUBFRAME;
	extern cop_channel_t pufft_ch_handle[NUM_PUFFTS_USED];

	trx = &g_enodeb_inst[0].trx;
	OS_ASSERT_COND(trx != NULL);

	if (cb_param->event_type == CPRI_IQ_THRESHOLD_EVENT)
		return;

	//INFO(DTRX,"CPRI RX CB a=%i p=%i\n", trx->cpri_rx_active_buffer_no, trx->cpri_rx_process_buffer_no);

	new_tick = loop_start = log_get_timer_value();

	interval_cycles = new_tick - old_tick;
	old_tick = new_tick;
	rx_sym_cycles[rx_ul_count] = loop_start;
	rx_ul_count++;
	if (rx_ul_count == 10)
		rx_ul_count = 0;

	rx_total_symbols++;
	rx_symbol_count[cb_param->cpri_num]++;

	/*
	 * Определение номера переданного буфера CPRI по типу события
	 * Используется в trx_dl_task
	 */
	if (cb_param->event_type == CPRI_IQ_FIRST_THRESHOLD_EVENT)
	{
		// Сработал FIRST_THRESHOLD, началась передача буфера 0
		trx->cpri_rx_active_buffer_no = 0;
		trx->cpri_rx_process_buffer_no = 1;
	}
	else if (cb_param->event_type == CPRI_IQ_SECOND_THRESHOLD_EVENT)
	{
		// Сработал SECOND_THRESHOLD, началась передача буфера 1
		trx->cpri_rx_active_buffer_no = 1;
		trx->cpri_rx_process_buffer_no = 0;
	}

	/* PRACH config 1 */
	if (lte_trx_is_prach_subframe(trx))
	{
		/* Принят весь сабфрейм, отправка на обработку */
		/* 
		 * TODO: сделать выбор антенны для PRACH динамическим
		 * как вариант - перебор антенн (round-robin)
		 * сейчас ипользуется антенна 0
		 */
		trx->rx_prach_buffer[trx->rx_prach_no].prach = (Complex16 *) trx->iq_rx_buffer[0][trx->cpri_rx_process_buffer_no];
		trx->rx_prach_buffer[trx->rx_prach_no].frame_no = trx->frame_no_rx;
		trx->rx_prach_buffer[trx->rx_prach_no].subframe_no = trx->subframe_no_rx;
		//trx->rx_prach_buffer[trx->rx_prach_no].sym = trx->sym_no_rx;

		sweep_cache_async((uint32_t) trx->rx_prach_buffer[trx->rx_prach_no].prach,
				(MAXIMUM_SYMBOL_SIZE * LTE_NSYMB_PER_SUBFRAME) * sizeof(Complex16),
				CACHE_FLUSH /*CACHE_INVALIDATE*/);
		//(NUM_OF_SAMPLES1 * LTE_NSYMB_PER_SUBFRAME) * sizeof(Complex16), CACHE_FLUSH /*CACHE_INVALIDATE*/);

		osEventQueuePost(trx->evq_prach_rx, (uint32_t) &trx->rx_prach_buffer[trx->rx_prach_no], NULL);
	}

	trx->rx_prach_no++;
	if (trx->rx_prach_no >= LTE_PRACH_RX_SUBFRAMES)
	{
		trx->rx_prach_no = 0;
	}

	/* Установка счетчиков на текущее системное время
	 * 
	 * В данном случае это можно делать, т.к. прерывание CPRI TX происходит _РАНЬШЕ_ прерывания CPRI RX
	 */

	trx->subframe_no_rx = g_enodeb_inst[0].system_subframe_no;
	trx->frame_no_rx = g_enodeb_inst[0].system_frame_no;

	LOG_EVENT_SF(LOGEVT_RX_SF_SFN, trx->frame_no_rx, trx->subframe_no_rx, 0);

	loop_end = log_get_timer_value();
}

static uint64_t tx_ticks[64];

static void cpri_iq_tx_cb(void* param)
{
	cpri_iq_int_cb_param_t* cb_param = (cpri_iq_int_cb_param_t*) param;
	os_status status;
	lte_trx_t *trx;
	lte_enodeb_t *enodeb;

	if (cb_param->event_type != CPRI_IQ_SECOND_THRESHOLD_EVENT && cb_param->event_type != CPRI_IQ_FIRST_THRESHOLD_EVENT)
	{
		return;
	}

	// FIXME: Определние номера TRX должно быть привязано к cpri_num
	trx = &g_enodeb_inst[0].trx;
	enodeb = trx->enodeb;

	uint64_t cur_tick = log_get_timer_value();
	static tx_tick_cnt = 0;
	tx_ticks[tx_tick_cnt++] = cur_tick;
	if (tx_tick_cnt == 64)
		tx_tick_cnt = 0;

	/*
	 * Определение номера переданного буфера CPRI по типу события
	 * Используется в trx_dl_task
	 */
	if (cb_param->event_type == CPRI_IQ_FIRST_THRESHOLD_EVENT)
	{
		// Сработал FIRST_THRESHOLD, началась передача буфера 0
		trx->cpri_tx_active_buffer_no = 0;
		trx->cpri_tx_prepare_buffer_no = 1;
	}
	else if (cb_param->event_type == CPRI_IQ_SECOND_THRESHOLD_EVENT)
	{
		// Сработал SECOND_THRESHOLD, началась передача буфера 1
		trx->cpri_tx_active_buffer_no = 1;
		trx->cpri_tx_prepare_buffer_no = 0;
	}

	/* Установка семафора, заменена на event queue */
	//if(enodeb_inst.state == L1_RUNNING && enodeb_inst.fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_FDD)
	if (g_enodeb_inst[0].state == L1_RUNNING)
	{
		/* Сохранение предыдущего системного счетчика
		 * Эти значения используются в прерываниях приема, т.к.
		 * текущее прерывание соответствуюет приему предыдущего сабфрейма
		 */

		enodeb->prev_system_frame_no = enodeb->system_frame_no;
		enodeb->prev_system_subframe_no = enodeb->system_subframe_no;

		/* Установка нового системного счетчика */
		enodeb->system_subframe_no = enodeb->system_subframe_no + 1;
		if (enodeb->system_subframe_no == 10)
		{
			enodeb->system_frame_no = (enodeb->system_frame_no + 1) & 1023;
			enodeb->system_subframe_no = 0;
		}

		//INFO(DTRX,"CPRI TX %i\n", cb_param->event_type);
		status = osEventSemaphorePost(g_enodeb_inst[0].trx.evt_subframe_ind, NULL);
	}
}

static void cpri_eth_tx_cb(void *param, uint32_t data, uint32_t error_status)
{
	cpri_num_t cpri_num;
	lte_enodeb_t *enodeb = (lte_enodeb_t *) param;

	cpri_num = cpri_init_params_array[enodeb->trx.cpri_no].cpri_num;
	VAR_UNUSED(error_status);

	/* possibly do something with the frame, or check errors */

	/* delete the frame */
	osFrameRelease((os_frame_t *) data);

	enodeb->trx.cpri_eth_tx_callbacks_counter++;
}

static void cpri_eth_rx_cb(void *param, uint32_t rx_frame, uint32_t error_status)
{
	os_frame_t *frame;
	uint8_t *data;
	uint32_t length;
	lte_enodeb_t *enodeb = (lte_enodeb_t *) param;
	cpri_num_t cpri_num = cpri_init_params_array[enodeb->trx.cpri_no].cpri_num;

	fapi_ipc_msg_t *msg;

	OS_ASSERT_COND(error_status == 0);

	OS_ASSERT_COND(rx_frame != NULL)
	frame = (os_frame_t*) rx_frame;

	data = osFrameSingleBufferGet(frame, &length);

	msg = fapi_alloc_cpri_eth_msg(FAPI_CHANNEL_CPRI_ETH, NULL);

//	msg = (fapi_ipc_msg_t *) get_cpri_eth_buffer();
//	msg->channel_id = FAPI_CHANNEL_CPRI_ETH;
//	msg->body_addr = (uint32_t) ((uint8_t *) msg + sizeof(fapi_ipc_msg_t));

	if (length > (FAPI_IPC_CPRI_ETH_SC2PA_MSG_SIZE - sizeof(fapi_ipc_msg_t)))
		length = FAPI_IPC_CPRI_ETH_SC2PA_MSG_SIZE - sizeof(fapi_ipc_msg_t);

	memcpy(msg->body_addr, data, length);

	msg->length = sizeof(fapi_ipc_msg_t) + length;

	fapi_cpri_eth_send(msg);

	/* delete the frame */
	osFrameRelease(frame);
	enodeb->trx.cpri_eth_rx_counter++;
}

static void cpri_vss_rx_cb(void* param)
{
	static uint32_t vss_rx_counter = 0;
	++vss_rx_counter;
}

static void cpri_vss_tx_cb(void* param)
{
	VAR_UNUSED(param);
}

void cpri_eth_tx(lte_enodeb_t *enodeb, void *buf, int32_t len)
{
	os_frame_t *frame;
	uint8_t *data;
	uint32_t i = 0;
	uint8_t channel_num = 0, device_num = 0;

	device_num = osGetCoreID();

	frame = osFrameGet(enodeb->trx.cpri_eth_frames_pool, enodeb->trx.cpri_eth_buffers_pool);
	OS_ASSERT_COND(frame != NULL);

	data = osFrameBufferNew(frame);
	OS_ASSERT_COND(data != NULL);

	memcpy(data, buf, len);

	/* set the buffer to the frame */
	osFrameSingleBufferSet(frame, data, len + 16);

	/* transmit the frame */
	while (enodeb->trx.cpri_eth_tx_counter - enodeb->trx.cpri_eth_tx_callbacks_counter >= ETHERNET_BD_RING_SIZE)
	{
	}

	while (osBioChannelTx(&enodeb->trx.cpri_eth_chan_tx, frame) != OS_SUCCESS)
	{
	}

#ifdef NO_ETHERNET_INTERRUPTS
	while (osBioChannelCtrl(&(ethernet_test_channels[device_num].demo_bio_tx), CPRI_ETHERNET_CMD_TX_CONFIRM, NULL) != OS_SUCCESS);
#endif  //NO_ETHERNET_INTERRUPTS
	enodeb->trx.cpri_eth_tx_counter++;
}

static void ethernetChannelsOpen(lte_enodeb_t *enodeb)
{
	cpri_ethernet_channel_params_t cpri_ethernet_channel_params;
	cpri_ethernet_open_params_t cpri_ethernet_open_params =
		{ &cpri_error_cb };
	bio_ch_open_params_t ch_open_params;
	os_status status;

	enodeb->trx.cpri_eth_tx_callbacks_counter = 0;
	enodeb->trx.cpri_eth_tx_counter = 0;
	//enodeb->trx.cpri_eth_rx_counter = 0;

	/* channels allocated starting from channel_offset */
	cpri_ethernet_channel_params.bd_ring_steering_bits = SOC_STEERING_BITS_CORE_NET_SLAVES; //DDR
	cpri_ethernet_channel_params.buffer_steering_bits = SOC_STEERING_BITS_CORE_NET_SLAVES; //DDR
	cpri_ethernet_channel_params.bd_ring_attributes.to_chm.liodn = 0;
	cpri_ethernet_channel_params.bd_ring_attributes.to_chm.enhanced = CPRI_NOT_ACTIVE;
	cpri_ethernet_channel_params.bd_ring_attributes.to_chm.etype = CPRI_NOT_ACTIVE;
	cpri_ethernet_channel_params.buffer_attributes.to_chm.liodn = 0;
	cpri_ethernet_channel_params.buffer_attributes.to_chm.enhanced = CPRI_NOT_ACTIVE;
	cpri_ethernet_channel_params.buffer_attributes.to_chm.etype = CPRI_NOT_ACTIVE;

//	for (cpri_num = 0; cpri_num < NUM_OF_USED_CPRI_UNITS; cpri_num++)
	{
		//if (osGetCoreID() != cpri_num)
		//	continue;
		/* open the receive channel */

		ch_open_params.channel_num = 0;
		ch_open_params.frames_pool = enodeb->trx.cpri_eth_frames_pool;
		ch_open_params.callback = cpri_eth_rx_cb;
		ch_open_params.cb_parameter = (void *) enodeb;
		ch_open_params.buffers_pool = enodeb->trx.cpri_eth_buffers_pool;
		ch_open_params.lld_params = &cpri_ethernet_channel_params;
		status = osBioChannelOpen(enodeb->trx.cpri_eth_handle, &enodeb->trx.cpri_eth_chan_rx, BIO_READ | BIO_ACTIVE,
				&ch_open_params);
		OS_ASSERT_COND(status == OS_SUCCESS);
	}

	//for (cpri_num = 0; cpri_num < NUM_OF_USED_CPRI_UNITS; cpri_num++)
	{
		//if (osGetCoreID() != cpri_num)
		//	continue;
		ch_open_params.channel_num = 0;
		ch_open_params.callback = cpri_eth_tx_cb;
		ch_open_params.cb_parameter = (void *) enodeb;
		ch_open_params.buffers_pool = NULL;
		ch_open_params.lld_params = &cpri_ethernet_channel_params;
		status = osBioChannelOpen(enodeb->trx.cpri_eth_handle, &enodeb->trx.cpri_eth_chan_tx, BIO_WRITE | BIO_ACTIVE,
				&ch_open_params);
		OS_ASSERT_COND(status == OS_SUCCESS);
	}
}

/*****************************************************************************/
os_status cpriInit_old(lte_enodeb_t *enodeb)
{

	os_status status;
	sio_dev_open_params_t sio_dev_param;
	int i, channel_num, ant_no;
	char *iq_name = "c_i0";
	char *vss_name = "c_v0";
	cpri_iq_open_params_t cpri_iq_open_params =
		{ NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &cpri_error_cb };

	cpri_vss_open_params_t cpri_vss_open_params =
		{ &cpri_error_cb };

	bio_dev_open_params_t dev_open_params;
	char *ethernet_name = "c_e0";
	uint32_t data;
	uint16_t bfn_number_rx = 0, bfn_number_tx = 0, start_bfn_number;
	uint8_t hfn_number_rx = 0, hfn_number_tx = 0;

	int cpri_num = cpri_init_params_array[enodeb->trx.cpri_no].cpri_num;
	//os_status cpri_init_status = OS_SUCCESS;

	/* Если CPRI уже был проинициализирован, то просто выходим */
	if (enodeb->cpri_initialized)
	{
		return OS_SUCCESS;
	}
#if 0	
	/* Инициализация аппаратной части
	 * Если невозможно установить линк, то дальнейшая инициализация не производится
	 */
	/* Переехало в ipc_pa_to_sc_dedicated_cb */
	cpri_init_status = cpriInitialize(cpri_global_params, cpri_init_params);
#else
	//cpri_init_status = cpri_init_status;
#endif
	/* Инициализация буферов для CPRI Ethernet */
	if (enodeb->trx.cpri_eth_frames_pool == NULL)
		enodeb->trx.cpri_eth_frames_pool = osFramePoolCreate(ETHERNET_FRAMES_IN_POOL, 1, 0, 0,
				OS_MEM_LOCAL_CACHEABLE);
	OS_ASSERT_COND(enodeb->trx.cpri_eth_frames_pool != NULL);

	if (enodeb->trx.cpri_eth_frames_pool == NULL)
		enodeb->trx.cpri_eth_buffers_pool = osMemPartCreate(ETHERNET_DATA_SIZE, ETHERNET_NUM_OF_BUFS,
				ethernet_buffers_space, ALIGNED_64_BYTES, OFFSET_0_BYTES,
				(os_mem_part_t *) enodeb->trx.cpri_eth_mem_manager, 0);
	OS_ASSERT_COND(enodeb->trx.cpri_eth_buffers_pool);

	/* Open cpri iq device */
	sio_dev_param.rx_callback = cpri_iq_rx_cb;
	sio_dev_param.tx_callback = cpri_iq_tx_cb;
	enodeb->trx.rx_iq_callback_param.callback_param = enodeb;
	enodeb->trx.tx_iq_callback_param.callback_param = enodeb;
	sio_dev_param.rx_callback_parameter = &enodeb->trx.rx_iq_callback_param;
	sio_dev_param.tx_callback_parameter = &enodeb->trx.tx_iq_callback_param;
	sio_dev_param.lld_params = &cpri_iq_open_params;

	//iq_name[3] = (char) ('0' + enodeb->trx.cpri_num + CPRI_FIRST_USED_MODULE);
	iq_name[3] = (char) ('0' + cpri_num + CPRI_FIRST_USED_MODULE);
	enodeb->trx.cpri_iq_handle = osSioDeviceOpen(iq_name, &sio_dev_param);
	OS_ASSERT_COND(enodeb->trx.cpri_iq_handle);

	/* Open cpri VSS device */
	sio_dev_param.rx_callback_parameter = NULL;
	sio_dev_param.tx_callback_parameter = NULL;
	sio_dev_param.rx_callback = NULL; //cpri_vss_rx_cb;
	sio_dev_param.tx_callback = NULL;
	sio_dev_param.error_callback = cpri_error_cb;
	sio_dev_param.lld_params = &cpri_vss_open_params;

	//vss_name[3] = (char) ('0' + enodeb->trx.cpri_num + CPRI_FIRST_USED_MODULE);
	vss_name[3] = (char) ('0' + cpri_num + CPRI_FIRST_USED_MODULE);
	enodeb->trx.cpri_vss_handle = osSioDeviceOpen(vss_name, &sio_dev_param);
	OS_ASSERT_COND(enodeb->trx.cpri_vss_handle);

	/* open the CPRI ethernet device */
	dev_open_params.common_pool = NULL;
	dev_open_params.lld_params = NULL;

	//ethernet_name[3] = (char) ('0' + enodeb->trx.cpri_num + CPRI_FIRST_USED_MODULE);
	ethernet_name[3] = (char) ('0' + cpri_num + CPRI_FIRST_USED_MODULE);
	enodeb->trx.cpri_eth_handle = osBioDeviceOpen(ethernet_name, &dev_open_params);
	OS_ASSERT_COND(enodeb->trx.cpri_eth_handle != NULL);

	enodeb->cpri_initialized = 1;

	return cpri_init_status;
}

os_status cpriInit(lte_enodeb_t *enodeb)
{

	os_status status;
	sio_dev_open_params_t sio_dev_param;
	int i, channel_num, ant_no;
	char *iq_name = "c_i0";
	char *vss_name = "c_v0";
	cpri_iq_open_params_t cpri_iq_open_params =
		{ NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &cpri_error_cb };

	cpri_vss_open_params_t cpri_vss_open_params =
		{ &cpri_error_cb };

	bio_dev_open_params_t dev_open_params;
	char *ethernet_name = "c_e0";
	uint32_t data;
	uint16_t bfn_number_rx = 0, bfn_number_tx = 0, start_bfn_number;
	uint8_t hfn_number_rx = 0, hfn_number_tx = 0;

	int cpri_num = cpri_init_params_array[enodeb->trx.cpri_no].cpri_num;
	//os_status cpri_init_status = OS_SUCCESS;

	/* Если CPRI уже был проинициализирован, то просто выходим */
	if (enodeb->cpri_initialized)
	{
		return OS_SUCCESS;
	}
#if 0	
	/* Инициализация аппаратной части
	 * Если невозможно установить линк, то дальнейшая инициализация не производится
	 */
	/* Переехало в ipc_pa_to_sc_dedicated_cb */
	cpri_init_status = cpriInitialize(cpri_global_params, cpri_init_params);
#else
	//cpri_init_status = cpri_init_status;
#endif
	/* Инициализация буферов для CPRI Ethernet */
	if (enodeb->trx.cpri_eth_frames_pool == NULL)
		enodeb->trx.cpri_eth_frames_pool = osFramePoolCreate(ETHERNET_FRAMES_IN_POOL, 1, 0, 0,
				OS_MEM_LOCAL_CACHEABLE);
	OS_ASSERT_COND(enodeb->trx.cpri_eth_frames_pool != NULL);

	if (enodeb->trx.cpri_eth_buffers_pool == NULL)
		enodeb->trx.cpri_eth_buffers_pool = osMemPartCreate(ETHERNET_DATA_SIZE, ETHERNET_NUM_OF_BUFS,
				ethernet_buffers_space, ALIGNED_64_BYTES, OFFSET_0_BYTES,
				(os_mem_part_t *) enodeb->trx.cpri_eth_mem_manager, 0);
	OS_ASSERT_COND(enodeb->trx.cpri_eth_buffers_pool);

	/* Open cpri iq device */
	sio_dev_param.rx_callback = cpri_iq_rx_cb;
	sio_dev_param.tx_callback = cpri_iq_tx_cb;
	enodeb->trx.rx_iq_callback_param.callback_param = enodeb;
	enodeb->trx.tx_iq_callback_param.callback_param = enodeb;
	sio_dev_param.rx_callback_parameter = &enodeb->trx.rx_iq_callback_param;
	sio_dev_param.tx_callback_parameter = &enodeb->trx.tx_iq_callback_param;
	sio_dev_param.lld_params = &cpri_iq_open_params;

	//iq_name[3] = (char) ('0' + enodeb->trx.cpri_num + CPRI_FIRST_USED_MODULE);
	iq_name[3] = (char) ('0' + cpri_num + CPRI_FIRST_USED_MODULE);
	enodeb->trx.cpri_iq_handle = osSioDeviceOpen(iq_name, &sio_dev_param);
	OS_ASSERT_COND(enodeb->trx.cpri_iq_handle);

	/* Open cpri VSS device */
	sio_dev_param.rx_callback_parameter = NULL;
	sio_dev_param.tx_callback_parameter = NULL;
	sio_dev_param.rx_callback = NULL; //cpri_vss_rx_cb;
	sio_dev_param.tx_callback = NULL;
	sio_dev_param.error_callback = cpri_error_cb;
	sio_dev_param.lld_params = &cpri_vss_open_params;

	//vss_name[3] = (char) ('0' + enodeb->trx.cpri_num + CPRI_FIRST_USED_MODULE);
	vss_name[3] = (char) ('0' + cpri_num + CPRI_FIRST_USED_MODULE);
	enodeb->trx.cpri_vss_handle = osSioDeviceOpen(vss_name, &sio_dev_param);
	OS_ASSERT_COND(enodeb->trx.cpri_vss_handle);

	/* open the CPRI ethernet device */
	dev_open_params.common_pool = NULL;
	dev_open_params.lld_params = NULL;

	//ethernet_name[3] = (char) ('0' + enodeb->trx.cpri_num + CPRI_FIRST_USED_MODULE);
	ethernet_name[3] = (char) ('0' + cpri_num + CPRI_FIRST_USED_MODULE);
	enodeb->trx.cpri_eth_handle = osBioDeviceOpen(ethernet_name, &dev_open_params);
	OS_ASSERT_COND(enodeb->trx.cpri_eth_handle != NULL);

	enodeb->cpri_initialized = 1;

	return cpri_init_status;
}

/**
 * Запуск таймеров CPRI -> PUFFT
 */
static void cpri_timers_start(lte_enodeb_t *enodeb)
{
	os_status status;
	int32_t i, j;
	int32_t counter_val = 0, counter_val_trans = 0, counter_val_prev = 0;

	/* Расчет значений счетчиков для запуска PUFFT */
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < LTE_NSYMB_PER_SUBFRAME; j++)
		{
			int32_t cp_len = (j == 0 || j == 7) ? enodeb->fp.CPRI_CP0_LEN : enodeb->fp.CPRI_CPx_LEN;

			// Положение символа
			counter_val += (enodeb->fp.CPRI_SYMBOL_LEN + cp_len) * 4;

			// Количество полных транзакций для принятия символа
			counter_val_trans = (counter_val + 255) / 256;

			//tx_symbols_sizes_in_subframes[i][j] = (LTE_SAMPLES_PER_SUBFRAME * i + (j + 1) * (LTE_SYMBOL_LEN + cp_len + TX_TRANSACTION_SIZE - 1) / TX_TRANSACTION_SIZE) * 4;
			//rx_symbols_sizes_in_subframes[i][j] = (LTE_SAMPLES_PER_SUBFRAME * i + (j + 1) * (LTE_SYMBOL_LEN + cp_len + TX_TRANSACTION_SIZE - 1) / TX_TRANSACTION_SIZE) * 4;

			tx_symbols_sizes_in_subframes[i][j] = counter_val_trans - counter_val_prev;
			rx_symbols_sizes_in_subframes[i][j] = counter_val_trans - counter_val_prev;

			counter_val_prev = counter_val_trans;
		}
	}

	tx_symbol_current_timer_preload_val = tx_symbols_sizes_in_subframes[0][0] + tx_symbols_sizes_in_subframes[0][1];
	rx_symbol_current_timer_preload_val = rx_symbols_sizes_in_subframes[0][0] + rx_symbols_sizes_in_subframes[0][1];

	timer32_init_params[0].interval = rx_symbols_sizes_in_subframes[0][0];
	timer32_init_params[0].preload = rx_symbol_current_timer_preload_val;

	timer32_init_params[1].interval = tx_symbols_sizes_in_subframes[0][0];
	timer32_init_params[1].preload = tx_symbol_current_timer_preload_val;

	// Размеры символов 0 и 1 загружаются при инициализации таймера, следующим идет символ 2
	tx_symbol_idx = 2;
	rx_symbol_idx = 2;

	switch (cpri_init_params_array[enodeb->trx.cpri_no].cpri_num)
	{
		case CPRI_0:
			timer32_init_params[0].primary_source = SOC_TIMER32_TIN0SOURCE;
			break;
		case CPRI_1:
			timer32_init_params[0].primary_source = SOC_TIMER32_TIN1SOURCE;
			break;
		case CPRI_2:
			timer32_init_params[0].primary_source = SOC_TIMER32_TIN2SOURCE;
			break;
		case CPRI_3:
			timer32_init_params[0].primary_source = SOC_TIMER32_TIN3SOURCE;
			break;
	}

	status = hwTimer32Open(SOC_TIMER_CPRI_RX, &timer32_init_params[0]);
	OS_ASSERT_COND(status == OS_SUCCESS);

	//status = hwTimer32Open(SOC_TIMER32_1, &timer32_init_params[1]);
	//OS_ASSERT_COND(status == OS_SUCCESS);

	/*
	 #if (MAPLE_0 == ON)
	 status = hwTimer32Open(app_timer_alloc[0][core_id], &timer32_init_params);
	 hwTimer32MapleInterruptAssign(app_timer_alloc[0][core_id], SOC_TIMER32_MAPLE0);  // Interrupt will be from timer channel 0, in timer module 0, in timer group i
	 OS_ASSERT_COND(status == OS_SUCCESS);
	 #endif

	 #if (MAPLE_1 == ON)
	 status = hwTimer32Open(app_timer_alloc[1][core_id], &timer32_init_params_2);
	 hwTimer32MapleInterruptAssign(app_timer_alloc[1][core_id], SOC_TIMER32_MAPLE1);  // Interrupt will be from timer channel 0, in timer module 0, in timer group i
	 OS_ASSERT_COND(status == OS_SUCCESS);
	 #endif

	 #if (MAPLE_2 == ON)
	 status = hwTimer32Open(app_timer_alloc[2][core_id], &timer32_init_params);
	 hwTimer32MapleInterruptAssign(app_timer_alloc[2][core_id], SOC_TIMER32_MAPLE2);  // Interrupt will be from timer channel 0, in timer module 0, in timer group i
	 OS_ASSERT_COND(status == OS_SUCCESS);
	 #endif
	 */

	status = hwTimer32Start(SOC_TIMER_CPRI_RX);
	OS_ASSERT_COND(status == OS_SUCCESS);

	//status = hwTimer32Start(SOC_TIMER32_1);
	//OS_ASSERT_COND(status == OS_SUCCESS);
}

static void cpri_timers_stop()
{
	hwTimer32Stop(SOC_TIMER_CPRI_RX);
	//hwTimer32Stop(SOC_TIMER32_1);

	hwTimer32ClearEvent(SOC_TIMER_CPRI_RX);
	//hwTimer32ClearEvent(SOC_TIMER32_1);

	hwTimer32Delete(SOC_TIMER_CPRI_RX);
	//hwTimer32Delete(SOC_TIMER32_1);
}

os_status cpri_reinit(lte_enodeb_t *enodeb)
{
	os_status status;
	sio_dev_handle sio_handle[NUM_OF_USED_CPRI_UNITS];
	sio_ch_open_params_t sio_ch_param;
	cpri_channel_params_t cpri_channel_params;
	sio_dev_open_params_t sio_dev_param;
	int i, ant_no;
	char *iq_name = "c_i0";
	char *vss_name = "c_v0";
	cpri_iq_open_params_t cpri_iq_open_params =
		{ NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &cpri_error_cb };
	cpri_vss_open_params_t cpri_vss_open_params =
		{ &cpri_error_cb };
	uint32_t data;
	uint16_t bfn_number_rx = 0, bfn_number_tx = 0, start_bfn_number;
	uint8_t hfn_number_rx = 0, hfn_number_tx = 0;
	os_status cpri_reconfig_status;

	lte_trx_t *trx = &enodeb->trx;

	////////////////////////////////////////
	cpri_reconfiguration_level0_param_t cpri_reconfiguration_level0_param;

	// TODO: сделать заполнение количество блоков CPRI динамическим
//	cpri_global_params_struct.initializing_core = CPRI_CORE0;
//	cpri_global_params_struct.cpri_num_of_used_units = 1;

#if 1	
//	cpri_global_params_struct.group[enodeb->trx.cpri_num].minimal_accepted_link_rate = CPRI_2457_LINK_RATE;
//	cpri_global_params_struct.group[enodeb->trx.cpri_num].maximal_desired_link_rate = CPRI_2457_LINK_RATE;	
	//cpri_reconfiguration_level0_param.minimal_accepted_link_rate = CPRI_2457_LINK_RATE;
	//cpri_reconfiguration_level0_param.maximal_desired_link_rate = CPRI_2457_LINK_RATE;

	cpri_reconfiguration_level0_param.minimal_accepted_link_rate = cpri_global_params->group[0].minimal_accepted_link_rate;
	cpri_reconfiguration_level0_param.maximal_desired_link_rate = cpri_global_params->group[0].maximal_desired_link_rate;

#else
	cpri_global_params_struct.group[enodeb->trx.cpri_num].minimal_accepted_link_rate = CPRI_4914_LINK_RATE;
	cpri_global_params_struct.group[enodeb->trx.cpri_num].maximal_desired_link_rate = CPRI_4914_LINK_RATE;
	cpri_reconfiguration_level0_param.minimal_accepted_link_rate = CPRI_4914_LINK_RATE;
	cpri_reconfiguration_level0_param.maximal_desired_link_rate = CPRI_4914_LINK_RATE;
#endif

	cpri_reconfiguration_level0_param.cpri_init_params = (cpri_init_params_t(*)[]) &cpri_init_params_array;

#if 1
	/*
	 cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->rx_buffer_size = enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4 * NUM_CPRI_BUFFERS;
	 cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->tx_buffer_size = enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4 * NUM_CPRI_BUFFERS;
	 cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->tx_first_threshold = 0;
	 cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->tx_second_threshold = enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4;
	 cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->rx_first_threshold = 0;
	 cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->rx_second_threshold = enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4;
	 */
	// Параметры AxC заполняются по количеству используемых антенн
	cpri_init_params_array[enodeb->trx.cpri_no].iq_init_params->rx_axc_number = enodeb->fp.LTE_N_PHYS_ANTENNAS_RX;
	cpri_init_params_array[enodeb->trx.cpri_no].iq_init_params->tx_axc_number = enodeb->fp.LTE_N_PHYS_ANTENNAS_TX;
	cpri_init_params_array[enodeb->trx.cpri_no].iq_init_params->axc_rx_active = (1 << enodeb->fp.LTE_N_PHYS_ANTENNAS_RX)
			- 1;
	cpri_init_params_array[enodeb->trx.cpri_no].iq_init_params->axc_tx_active = (1 << enodeb->fp.LTE_N_PHYS_ANTENNAS_TX)
			- 1;
	/*
	 switch(enodeb->fapi_config.rf_config.dl_channel_bandwidth)
	 {
	 case 15:
	 cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->mapping_params->axc_oversampling_factor = CPRI_OVERSAMPLING_RATIO_1;
	 break;
	 case 25:
	 cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->mapping_params->axc_oversampling_factor = CPRI_OVERSAMPLING_RATIO_2;
	 break;
	 case 50:
	 cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->mapping_params->axc_oversampling_factor = CPRI_OVERSAMPLING_RATIO_4;
	 break;
	 case 100:
	 cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->mapping_params->axc_oversampling_factor = CPRI_OVERSAMPLING_RATIO_8;
	 break;
	 }
	 */

	//Only one reconfiguration per-group is needed. Could be called from any CPRI IQ device. All cores (in this demo there is only a single core) which use CPRI units from this group must call it.
	cpri_reconfig_status = osSioDeviceCtrl(trx->cpri_iq_handle, CPRI_DEVICE_RECONFIGURATION_LEVEL0,
			&cpri_reconfiguration_level0_param);
	//OS_ASSERT_COND(status == OS_SUCCESS);

	uint32_t cpri_status;
	status = osSioDeviceCtrl(trx->cpri_iq_handle, CPRI_STATUS_QUERY, &cpri_status);
	OS_ASSERT_COND(status == OS_SUCCESS);
#else
	/* Reconfiguration level 2 */
	// Параметры AxC заполняются по количеству используемых антенн
	cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->rx_axc_number = enodeb->fp.LTE_N_PHYS_ANTENNAS_RX;
	cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->tx_axc_number = enodeb->fp.LTE_N_PHYS_ANTENNAS_TX;
	cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->axc_rx_active = (1 << enodeb->fp.LTE_N_PHYS_ANTENNAS_RX) - 1;
	cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params->axc_tx_active = (1 << enodeb->fp.LTE_N_PHYS_ANTENNAS_TX) - 1;

	status = osSioDeviceCtrl(trx->cpri_iq_handle, CPRI_DEVICE_RECONFIGURATION_LEVEL2, cpri_init_params_array[enodeb->trx.cpri_num].iq_init_params);
	OS_ASSERT_COND(status == OS_SUCCESS);
#endif
	DBAR_SCFG();

	return cpri_reconfig_status;
}

void cpri_setup_buffers(lte_enodeb_t *enodeb)
{
	lte_trx_t *trx = &enodeb->trx;
	int32_t ant_no, i;

	trx->iq_buffer_len = enodeb->fp.CPRI_SAMPLES_PER_SUBFRAME * 4; // LTE_IQ_BUFFER_SIZE;

	trx->vss_rx_buffer = vss_rx_buffer[trx->cpri_no];
	trx->vss_tx_buffer = vss_tx_buffer[trx->cpri_no];

	/* Заполнение адресов буферов CPRI RX/TX в структуре trx для необходимого количества антенн */
	for (ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
	{

		trx->iq_rx_buffer[ant_no][0] = (Complex16 *) &iq_rx_buffer[trx->cpri_no][ant_no][0];
		//trx->iq_rx_buffer[ant_no][1] = (Complex16 *) &iq_rx_buffer[trx->cpri_num][ant_no][enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4 /*IQ_RX_BUFFER_SIZE*/];
		trx->iq_rx_buffer[ant_no][1] = (Complex16 *) &iq_rx_buffer[trx->cpri_no][ant_no][IQ_RX_BUFFER_SIZE_DEFAULT];
	}

	for (ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_TX; ant_no++)
	{
		trx->iq_tx_buffer[ant_no][0] = (Complex16 *) &iq_tx_buffer[trx->cpri_no][ant_no][0];
		//trx->iq_tx_buffer[ant_no][1] = (Complex16 *) &iq_tx_buffer[trx->cpri_num][ant_no][enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4 /*IQ_RX_BUFFER_SIZE*/];
		trx->iq_tx_buffer[ant_no][1] = (Complex16 *) &iq_tx_buffer[trx->cpri_no][ant_no][IQ_TX_BUFFER_SIZE_DEFAULT];
	}

	/* FIXME: здесь должно быть заполнение буфера передачи тестовым сигналом для калибровки RRU */
	for (ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_TX; ant_no++)
	{
		//for(i=0; i<enodeb->fp.LTE_SAMPLES_PER_SUBFRAME; i++)
		for (i = 0; i < IQ_TX_BUFFER_SIZE_DEFAULT; i++)
		{
			enodeb->trx.iq_tx_buffer[ant_no][0][i] = 0x1000;
			enodeb->trx.iq_tx_buffer[ant_no][1][i] = 0x1000;
		}
	}

	//for(cpri_num = 0; cpri_num < NUM_OF_USED_CPRI_UNITS; cpri_num++)
	{
		memset(enodeb->trx.vss_rx_buffer, 0, MAX_VSS_RX_BUFFER_SIZE);
		memset(enodeb->trx.vss_tx_buffer, 0, MAX_VSS_TX_BUFFER_SIZE);

		enodeb->trx.vss_tx_buffer[34] = 0x0a;
		enodeb->trx.vss_tx_buffer[33] = 0x0a;
		enodeb->trx.vss_tx_buffer[0] = 0x1;
		enodeb->trx.vss_tx_buffer[1] = 0x0;
		enodeb->trx.vss_tx_buffer[2] = 0x1;

		enodeb->trx.vss_tx_buffer[64 + 34] = 0x0a;
		enodeb->trx.vss_tx_buffer[64 + 33] = 0x0a;
		enodeb->trx.vss_tx_buffer[64 + 0] = 0x1;
		enodeb->trx.vss_tx_buffer[64 + 1] = 0x0;
		enodeb->trx.vss_tx_buffer[64 + 2] = 0x1;
	}
}

void cpri_start(lte_enodeb_t *enodeb)
{
	uint16_t bfn_number_rx = 0, bfn_number_tx = 0, start_bfn_number;
	uint8_t hfn_number_rx = 0, hfn_number_tx = 0;
	int32_t cpri_num = 0;

	sio_dev_handle sio_handle[NUM_OF_USED_CPRI_UNITS];
	sio_ch_open_params_t sio_ch_param;
	cpri_channel_params_t cpri_channel_params;
	sio_dev_open_params_t sio_dev_param;
	int i, ant_no;
	char *iq_name = "c_i0";
	char *vss_name = "c_v0";
	cpri_iq_open_params_t cpri_iq_open_params =
		{ NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &cpri_error_cb };
	cpri_vss_open_params_t cpri_vss_open_params =
		{ &cpri_error_cb };
	uint32_t data;
	os_status status;

	lte_trx_t *trx = &enodeb->trx;

	sio_ch_param.num_of_buffers = NUM_CPRI_BUFFERS;
	cpri_channel_params.steering_bits = SOC_STEERING_BITS_CORE_NET_SLAVES;
	cpri_channel_params.buffer_attributes.to_chm.liodn = 0X0;
	cpri_channel_params.buffer_attributes.to_chm.enhanced = CPRI_NOT_ACTIVE;
	cpri_channel_params.buffer_attributes.to_chm.etype = CPRI_NOT_ACTIVE;

	sio_ch_param.lld_params = &cpri_channel_params;

	osWaitForAllCores();

	/* Open channels  for tx and rx */
	for (ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_TX; ant_no++)
	{
		/* Channel number in frame *slot */
		sio_ch_param.channel_num = (uint16_t) ant_no;

		/* Parameter of callback */
		sio_ch_param.callback_parameter = enodeb; //&iq_test_channels[cpri_num][channel_num];

		/* Buffer base for this channel */
		sio_ch_param.channel_buffers_base = (void*) trx->iq_tx_buffer[ant_no][0];

		sio_ch_param.buffer_size = IQ_TX_BUFFER_SIZE_DEFAULT; // enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4; //IQ_TX_BUFFER_SIZE;
		sio_ch_param.buffer_data_size = IQ_TX_BUFFER_SIZE_DEFAULT; // enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4; //IQ_TX_BUFFER_SIZE;

		status = osSioChannelOpen(trx->cpri_iq_handle, &trx->cpri_iq_chan_tx[ant_no], SIO_WRITE | SIO_ACTIVE,
				&sio_ch_param);
		OS_ASSERT_COND(OS_SUCCESS == status);
		status = osSioChannelClose(&enodeb->trx.cpri_iq_chan_tx[ant_no]);
		OS_ASSERT_COND(status == OS_SUCCESS);
		status = osSioChannelOpen(trx->cpri_iq_handle, &trx->cpri_iq_chan_tx[ant_no], SIO_WRITE | SIO_ACTIVE,
				&sio_ch_param);
		OS_ASSERT_COND(OS_SUCCESS == status);
	}

	for (ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
	{
		/* Channel number in frame *slot */
		sio_ch_param.channel_num = (uint16_t) ant_no;

		/* Parameter of callback */
		sio_ch_param.callback_parameter = enodeb; //&iq_test_channels[cpri_num][channel_num];

		/* Buffer base for this channel */
		sio_ch_param.channel_buffers_base = (void*) trx->iq_rx_buffer[ant_no][0];
		sio_ch_param.buffer_size = IQ_RX_BUFFER_SIZE_DEFAULT; //enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4;//IQ_RX_BUFFER_SIZE;
		sio_ch_param.buffer_data_size = IQ_RX_BUFFER_SIZE_DEFAULT; //enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * 4;//IQ_RX_BUFFER_SIZE;
		status = osSioChannelOpen(trx->cpri_iq_handle, &trx->cpri_iq_chan_rx[ant_no], SIO_READ | SIO_ACTIVE,
				&sio_ch_param);
		OS_ASSERT_COND(OS_SUCCESS == status);
		status = osSioChannelClose(&enodeb->trx.cpri_iq_chan_rx[ant_no]);
		OS_ASSERT_COND(status == OS_SUCCESS);
		status = osSioChannelOpen(trx->cpri_iq_handle, &trx->cpri_iq_chan_rx[ant_no], SIO_READ | SIO_ACTIVE,
				&sio_ch_param);
		OS_ASSERT_COND(OS_SUCCESS == status);

	}

	/* Open VSS channels  for tx and rx */
	//for (cpri_num = 0 ; cpri_num < NUM_OF_USED_CPRI_UNITS ; cpri_num++)
	{
		/* Channel number in frame *slot */
		sio_ch_param.channel_num = 0;
		/* Parameter of callback */
		sio_ch_param.callback_parameter = NULL;

		/* Buffer base for this channel */
		sio_ch_param.channel_buffers_base = trx->vss_tx_buffer;
		sio_ch_param.buffer_size = VSS_TX_THRESHOLD_SIZE;
		sio_ch_param.buffer_data_size = VSS_TX_THRESHOLD_SIZE;
		status = osSioChannelOpen(trx->cpri_vss_handle, &trx->cpri_vss_chan_tx, SIO_WRITE | SIO_ACTIVE, &sio_ch_param);
		OS_ASSERT_COND(OS_SUCCESS == status);

		sio_ch_param.channel_buffers_base = trx->vss_rx_buffer;
		sio_ch_param.buffer_size = VSS_RX_THRESHOLD_SIZE;
		sio_ch_param.buffer_data_size = VSS_RX_THRESHOLD_SIZE;
		status = osSioChannelOpen(trx->cpri_vss_handle, &trx->cpri_vss_chan_rx, SIO_READ | SIO_ACTIVE, &sio_ch_param);
		OS_ASSERT_COND(OS_SUCCESS == status);
	}

	/*initialize ethernet channels */
	ethernetChannelsOpen(enodeb);

	status = osBioDeviceCtrl(trx->cpri_eth_handle, BIO_DEVICE_TX_ENABLE, NULL);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osBioDeviceCtrl(trx->cpri_eth_handle, BIO_DEVICE_RX_ENABLE, NULL);
	OS_ASSERT_COND(status == OS_SUCCESS);

//    osSioDeviceCtrl(sio_handle[cpri_num], CPRI_DEVICE_RESET_ENABLE, NULL);
//    osSioDeviceCtrl(sio_handle[cpri_num], CPRI_DEVICE_RESET_REQUEST, NULL);

	osWaitForAllCores();

	status = osSioDeviceCtrl(trx->cpri_vss_handle, SIO_DEVICE_TX_ENABLE, NULL);
	OS_ASSERT_COND(status == OS_SUCCESS);
	status = osSioDeviceCtrl(trx->cpri_vss_handle, SIO_DEVICE_RX_ENABLE, NULL);
	OS_ASSERT_COND(status == OS_SUCCESS);

	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_RX_ENABLE, NULL);
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_TX_ENABLE, NULL);

	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_RX_DISABLE, NULL);
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_TX_DISABLE, NULL);

	//osSioDeviceCtrl(sio_handle[cpri_num], CPRI_DEVICE_RESET_REQUEST, NULL);
	//uint8_t reset_request = 0;
	//while(reset_request == 0)
	//	osSioDeviceCtrl(test_iq_cpri[0].sio_handle, CPRI_DEVICE_CHECK_RESET_DETECTED, &reset_request);

	//osSioDeviceCtrl(test_iq_cpri[cpri_num].sio_handle, CPRI_DEVICE_RESET_REQUEST, NULL);

	//osWait(20000);

	//osSioDeviceCtrl(test_iq_cpri[cpri_num].sio_handle, CPRI_DEVICE_RESET_REQUEST_DISABLE, NULL);
	//osSioDeviceCtrl(sio_handle[cpri_num], CPRI_DEVICE_DMA_RESTART, NULL);

	/* Запуск таймеров CPRI */
#ifdef CPRI_PUFFT_TIMER
	cpri_timers_start(enodeb);
#endif

#if 0
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_TX_BFN_COUNTER_READ, &bfn_number_tx);
	start_bfn_number = (bfn_number_tx + 8) % 4096;

	while (bfn_number_tx != start_bfn_number)
	{
		osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_TX_BFN_COUNTER_READ, &bfn_number_tx);
	}

	while (hfn_number_tx != 148)
	{
		osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_TX_HFN_COUNTER_READ, &hfn_number_tx);
	}

	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_TX_ENABLE, NULL);

	/*osSioDeviceCtrl(test_iq_cpri[cpri_num].sio_handle, CPRI_DEVICE_RX_BFN_COUNTER_READ, &bfn_number_rx);
	 start_bfn_number = (bfn_number_rx + 8) % 4096;

	 while (bfn_number_rx != start_bfn_number)
	 {
	 osSioDeviceCtrl(test_iq_cpri[cpri_num].sio_handle, CPRI_DEVICE_RX_BFN_COUNTER_READ, &bfn_number_rx);
	 }*/

	while (hfn_number_rx != 149)
	{
		osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_RX_HFN_COUNTER_READ, &hfn_number_rx);
	}
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_RX_ENABLE, NULL);
#else

	int32_t n_cpri_try = 0;
	do
	{
		INFO(DTRX, "Trying to sync CPRI RX/TX\n");

		osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_RX_DISABLE, NULL);
		osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_TX_DISABLE, NULL);

		// Wait 2 BFN
		OS_WAIT(osCoreClockGet() * 67 * 300);

		for (int n_try = 0; n_try < 50; n_try++)
		{
			osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_RX_BFN_COUNTER_READ, &bfn_number_rx);
			start_bfn_number = bfn_number_rx;
			while (bfn_number_rx == start_bfn_number)
			{
				OS_WAIT(osCoreClockGet() * 67 * 150);
				osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_RX_BFN_COUNTER_READ, &bfn_number_rx);
			}
		}

		osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_RX_BFN_COUNTER_READ, &bfn_number_rx);
		start_bfn_number = (bfn_number_rx + 8) % 4096;

		while (bfn_number_rx != start_bfn_number)
		{
			osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_RX_BFN_COUNTER_READ, &bfn_number_rx);
		}

		hfn_number_rx = 0;
		while (hfn_number_rx != 146)
		{
			osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_RX_HFN_COUNTER_READ, &hfn_number_rx);
		}

		/*
		 osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_TX_BFN_COUNTER_READ, &bfn_number_tx);
		 start_bfn_number = (bfn_number_tx + 8) % 4096;
		 
		 while (bfn_number_tx != start_bfn_number)
		 {
		 osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_TX_BFN_COUNTER_READ, &bfn_number_tx);
		 }
		 */

		/*
		 hfn_number_tx = hfn_number_rx;
		 while (hfn_number_tx != hfn_number_rx + 2)
		 {
		 osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_TX_HFN_COUNTER_READ, &hfn_number_tx);
		 }
		 */

		osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_RX_ENABLE, NULL);
		osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_TX_ENABLE, NULL);
		n_cpri_try++;
	}
	while (!is_cpri_counters_ok(enodeb) && n_cpri_try < 10);
#endif

	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_DELAYS_CALCULATE, &g_enodeb_inst[0].trx.cpri_delays);
}

void cpri_stop(lte_enodeb_t *enodeb)
{
	int32_t channel_num;
	os_status status;

	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_RX_DISABLE, NULL);
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, SIO_DEVICE_TX_DISABLE, NULL);

	osBioDeviceCtrl(enodeb->trx.cpri_eth_handle, BIO_DEVICE_TX_DISABLE, NULL);
	osBioDeviceCtrl(enodeb->trx.cpri_eth_handle, BIO_DEVICE_RX_DISABLE, NULL);
	osSioDeviceCtrl(enodeb->trx.cpri_vss_handle, SIO_DEVICE_TX_DISABLE, NULL);
	osSioDeviceCtrl(enodeb->trx.cpri_vss_handle, SIO_DEVICE_RX_DISABLE, NULL);

	// Закрытие устрйоств CPRI и освобождение буферов

	//for (cpri_num = 0 ; cpri_num < NUM_OF_USED_CPRI_UNITS ; cpri_num++)
	{
		for (channel_num = 0; channel_num < enodeb->fp.LTE_N_PHYS_ANTENNAS_TX; channel_num++)
		{
			status = osSioChannelClose(&enodeb->trx.cpri_iq_chan_tx[channel_num]);
			OS_ASSERT_COND(status == OS_SUCCESS);
		}

		for (channel_num = 0; channel_num < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; channel_num++)
		{
			status = osSioChannelClose(&enodeb->trx.cpri_iq_chan_rx[channel_num]);
			OS_ASSERT_COND(status == OS_SUCCESS);
		}

		status = osSioChannelClose(&enodeb->trx.cpri_vss_chan_tx);
		OS_ASSERT_COND(status == OS_SUCCESS);
		status = osSioChannelClose(&enodeb->trx.cpri_vss_chan_rx);
		OS_ASSERT_COND(status == OS_SUCCESS);

		status = osBioChannelClose(&enodeb->trx.cpri_eth_chan_tx);
		OS_ASSERT_COND(status == OS_SUCCESS);
		status = osBioChannelClose(&enodeb->trx.cpri_eth_chan_rx);
		OS_ASSERT_COND(status == OS_SUCCESS);
	}

	/* Останов таймеров CPRI */
#ifdef CPRI_PUFFT_TIMER
	cpri_timers_stop();
#endif
}

void cpri_print_counters(lte_enodeb_t *enodeb)
{
	uint16_t bfn_number_rx = 0, bfn_number_tx = 0;
	uint8_t hfn_number_rx = 0, hfn_number_tx = 0;

	if (cpri_global_params->cpri_num_of_used_units <= 0)
		return;

	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_RX_BFN_COUNTER_READ, &bfn_number_rx);
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_RX_HFN_COUNTER_READ, &hfn_number_rx);
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_TX_BFN_COUNTER_READ, &bfn_number_tx);
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_TX_HFN_COUNTER_READ, &hfn_number_tx);
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_DELAYS_CALCULATE, &g_enodeb_inst[0].trx.cpri_delays);

	INFO(
			DTRX,
			"CPRI rx=%i:%i tx=%i:%i t14=%i t_dl=%i t_ul=%i\n",
			bfn_number_rx, hfn_number_rx, bfn_number_tx, hfn_number_tx, (int32_t)(g_enodeb_inst[0].trx.cpri_delays.t_14), (int32_t)(g_enodeb_inst[0].trx.cpri_delays.tb_delay_dl), (int32_t)(g_enodeb_inst[0].trx.cpri_delays.tb_delay_ul));

}

int32_t is_cpri_counters_ok(lte_enodeb_t *enodeb)
{
	uint16_t bfn_number_rx = 0, bfn_number_tx = 0;
	uint8_t hfn_number_rx = 0, hfn_number_tx = 0;

	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_RX_BFN_COUNTER_READ, &bfn_number_rx);
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_RX_HFN_COUNTER_READ, &hfn_number_rx);
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_TX_BFN_COUNTER_READ, &bfn_number_tx);
	osSioDeviceCtrl(enodeb->trx.cpri_iq_handle, CPRI_DEVICE_TX_HFN_COUNTER_READ, &hfn_number_tx);

	return (bfn_number_tx * 150 + hfn_number_tx) - (bfn_number_rx * 150 + hfn_number_rx) == 2;
}

os_status cpri_init_top(lte_enodeb_t *enodeb)
{
	os_status status = OS_SUCCESS;
	lte_trx_t *trx;
	int32_t ant_no;

	status = cpriInit(enodeb);

	return status;
}
