/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __LTE_ENODEB_H_
#define __LTE_ENODEB_H_

#include <smartdsp_os.h>
//#include "os_cache.h"
#include "sc39xx_cache.h"
#include <app_config.h>
#include <maple_pdsch.h>
#include <maple_pufft.h>
#include <maple_pusch.h>
#include <maple_tvpe.h>
#include "cpri_b4860.h"
//#include <aic.h>
//#include <net_tcp.h>
//#include <cpri.h>
#include <lte_defs.h>
#include <nvi_beta.h>
#include <fapi.h>

#include <secure_id.h>

typedef struct lte_enodeb_params_t lte_enodeb_params_t;
typedef struct lte_enodeb_t lte_enodeb_t;
typedef struct lte_trx_t lte_trx_t;

//#define NUM_ENODEB (MAPLE_NUM_SECTORS)
#define NUM_ENODEB 1

#define TRX_NUM_OF_PRACH_MSGS LTE_PRACH_RX_SUBFRAMES
#define TRX_NUM_OF_SLOTS 16

//#define TRX_NUM_OF_UL_JOBS 32
#define TRX_NUM_OF_UL_JOBS 8

// Количество дескрипторов PUSCH_EDF BD
#define TRX_NUM_OF_PUSCH_JOBS 8
// Количество дескрипторов PUSCH UH
#define TRX_NUM_OF_PUSCH_UPH 64
// Количество дескрипторов PUSCH SH
#define TRX_NUM_OF_PUSCH_SH 64

#define TRX_NUM_OF_PUSCH_READY_JOBS 16
#define TRX_NUM_OF_TVPE_READY_JOBS 8

// Количество буферов HARQ для ULSCH
#define TRX_NUM_OF_UL_HARQ_BUFFERS	8
// Размер одного буфера UL HARQ для сабфрейма
// Максимальный размер: (1200 * 6 * 12) * 3 = 259200
// Т.к. в буфере хранятся данные разных UE и будет необходимо выравнивание, то размер буфера увеличен до 1Мб
#define TRX_UL_HARQ_BUFFER_SIZE (1*1024*1024)
// Время жизни HARQ буфера до его принудительного закрытия (40 сабфреймов)
#define TRX_UL_HARQ_TIMER_MAX	(10 * 4)

/* Размер очереди принятых сабфреймов PRACH */ 
#define LTE_PRACH_RX_SUBFRAMES 8

//#define DEFAULT_TASK_STACK_SIZE (8*1024*1024)
#define DEFAULT_TASK_STACK_SIZE (16*1024)
#define TRX_UL_STACK_SIZE	(128 * 1024) //DEFAULT_TASK_STACK_SIZE
#define TRX_DL_STACK_SIZE	(128 * 1024) //DEFAULT_TASK_STACK_SIZE
#define TRX_MAPLE_PDSCH_STACK_SIZE	DEFAULT_TASK_STACK_SIZE
#define TRX_BURST_RX_STACK_SIZE	DEFAULT_TASK_STACK_SIZE
#define TRX_PRACH_RX_STACK_SIZE	DEFAULT_TASK_STACK_SIZE

#define TO_COMPLEX16(i, q) ((Complex16)((((uint16_t)i << 16) & 0xffff0000) | (((uint16_t)q) & 0x0000ffff)))

typedef uint32_t lte_status_t; /**< Код статуса */

#define LTE_FAIL	0
#define LTE_SUCCESS	1
#define LTE_ERR_L1_INVALID_STATE	32
#define LTE_ERR_L1_INVALID_N_RB_DL	33
#define LTE_ERR_UNSUPPORTED_DCI_PDU_FORMAT	34
#define LTE_ERR_UNSUPPORTED_MODULATION	35
#define LTE_ERR_UNSUPPORTED_RB_ALLOC_TYPE	36
#define LIBLTE_ERROR_PUCCH_DRS 37
#define LTE_ERR_MAPLE_QUEUE_FULL	100

/* Модуль счетчика старших разрядов полного номера фрейма */
#define LTE_VSS_HI_MOD	0

/* Минимальное значение количества таймслотов в очереди для запуска передачи */
#define CPRI_TX_Q_MIN	8

/*
 *  LTE_constants 
 */
#define LTE_NULL 2
#define LTE_PBCH_A 24
#define MAX_NUM_PHICH_GROUPS 56  //110 RBs Ng=2, p.60 36-212, Sec. 6.9

#define PDCCH_SIZE_MAX	1200
#define PDCCH_NUM_MAX	4

/***************************************************************************/
/* PDSCH RB Mapping Table generator                                        */
/***************************************************************************/
PDSCH_RB_MAP_STRUCT_GEN(1_4);  // RB Mapping Table for 1.4MHz
PDSCH_RB_MAP_STRUCT_GEN(3);    // RB Mapping Table for 3MHz
PDSCH_RB_MAP_STRUCT_GEN(5);    // RB Mapping Table for 5MHz
PDSCH_RB_MAP_STRUCT_GEN(10);   // RB Mapping Table for 10MHz
PDSCH_RB_MAP_STRUCT_GEN(15);   // RB Mapping Table for 15Hz
PDSCH_RB_MAP_STRUCT_GEN(20);   // RB Mapping Table for 20MHz

typedef enum
{
	L1_IDLE = 0,
	L1_CONFIGURED,
	L1_RUNNING
} lte_l1_state_t;

typedef enum
{
	TRX_MODE_NORMAL = 0,
} trx_mode_e;

typedef struct
{
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) cop_job_handle cop_job;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pdsch_job_t pdsch_job;
} lte_pdsch_job_t;

typedef struct
{
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) cop_job_handle cop_job;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pufft_job_t pufft_job;
	// Status buffer for ADP_OVA_SCL status for 8 antennas 
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) int8_t status[32];
} lte_pufft_job_t;

/* 
 * Структура статуса MAPLE-B3 PUSCH
 * Не определена в SmartDSP
 */
typedef struct
{
	uint32_t tb0_crc:1;
	uint32_t :6;
	uint32_t tb0_cb:25;

	uint32_t tb1_crc:1;
	uint32_t :6;
	uint32_t tb1_cb:25;
	
	uint32_t :6;
	uint32_t w_index_stat:2;
	uint32_t :3;
	uint32_t ack_d:1;
	uint32_t ack_val:20;
	
	uint32_t reserved;
	
	uint32_t :1;
	uint32_t cqi_amount:15;
	uint32_t :1;
	uint32_t ri_val:15;
	
	uint32_t ack_mp_metric[2];
	uint32_t ack_smp_metric[2];
	uint32_t ri_mp_metric[2];
	uint32_t ri_smp_metric[2];
} maple_pusch_uss_t;

typedef struct
{
	uint32_t busy;
	lte_trx_t *trx;
	uint32_t frame_no;
	uint32_t subframe_no;
	uint32_t num_pdcch_symbols;
	uint32_t num_common_dci;
	uint32_t num_ue_dci;
	uint32_t num_tb_alloc;
	uint32_t rb_ptr;
	
	// Dump CPRI TX buffer flag
	uint32_t dump_cpri_tx;

	uint64_t timestamp_create;
	uint64_t timestamp_map;
	uint64_t timestamp_maple_start;
	uint64_t timestamp_maple_ext_sym_start;
	uint64_t timestamp_maple_complete;

	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) lte_pdsch_job_t job;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 pdcch_syms[LTE_N_ANTENNAS_MAX * PDCCH_NUM_MAX * PDCCH_SIZE_MAX];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pdsch_user_header_t user_map[64];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pdsch_cw_header_t cw_map[64];
	
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) union {
		//uint8_t rb_raw[PDSCH_RB_MAP_TABLE_SIZE_20MHZ];
		maple_pdsch_rb_map_1_4mhz_t rb_1_4;
		maple_pdsch_rb_map_3mhz_t rb_3;
		maple_pdsch_rb_map_5mhz_t rb_5;
		maple_pdsch_rb_map_10mhz_t rb_10;
		maple_pdsch_rb_map_15mhz_t rb_15;
		maple_pdsch_rb_map_20mhz_t rb_20;
	} rb_map_raw;
	
	// Указатели на таблицу RB в слотах 0 и 1
	// Т.к. количество RB и смещения в массиве завист от полосы,
	// то проще всего сделать так.
	uint8_t *rb_map_slot0;
	uint8_t *rb_map_slot1;
	
} lte_subframe_t;

typedef struct
{
	uint32_t frame_no;
	uint32_t subframe_no;
	
	uint32_t n_pufft_jobs;
	
	__attribute__((aligned(16))) lte_pufft_job_t pufft_job[LTE_NSYMB_PER_SUBFRAME];
	
	__attribute__((aligned(256))) Complex16 pufft_buf_a0[LTE_N_ANTENNAS_MAX*LTE_SAMPLES_PER_SUBFRAME_MAX];
	__attribute__((aligned(256))) Complex16 pufft_buf_a0_sym3[LTE_N_ANTENNAS_MAX*1200];
	__attribute__((aligned(256))) Complex16 pufft_buf_a0_sym10[LTE_N_ANTENNAS_MAX*1200];
	
	__attribute__((aligned(256))) Complex16 a0_aligned[LTE_N_ANTENNAS_MAX*LTE_SAMPLES_PER_SUBFRAME_MAX];
	__attribute__((aligned(256))) Complex16 a0_sym3_aligned[LTE_N_ANTENNAS_MAX*1200];
	__attribute__((aligned(256))) Complex16 a0_sym10_aligned[LTE_N_ANTENNAS_MAX*1200];
	
	__attribute__((aligned(256))) int8_t a0_scales_aligned[LTE_N_ANTENNAS_MAX][LTE_NSYMB_PER_SUBFRAME];
} lte_ul_subframe_t;

typedef struct
{
	uint32_t active;
	uint32_t timer;
	uint32_t n_retx;
	uint32_t first_rb;
	uint32_t n_rb;
	void* ptr;
} lte_ul_harq_buf_descr_t;

typedef struct
{
	uint32_t rnti;
	lte_ul_harq_buf_descr_t hd[TRX_NUM_OF_UL_HARQ_BUFFERS];
} lte_ul_harq_descr_t;

typedef struct
{
	uint32_t index;
	uint32_t rnti;
	uint32_t handle;
	uint32_t rballoc;
	uint32_t num_prb;
	
	uint32_t h_pid;
	
	uint32_t has_harq;
	uint32_t has_ri;
	uint32_t has_cqi;
	
	uint32_t Qprime_ACK;
	int32_t dtx_thr;
	uint32_t Qprime_RI;
	uint32_t Qprime_CQI;
	uint32_t Qm;
	
	uint8_t *hard_out;
	uint32_t tbs_bytes;

	uint32_t cqi_n_carrier;
	uint32_t cqi_i_cqi_offset;
	uint32_t cqi_cw;
	uint32_t cqi_n_sym_pusch_initial;
	uint32_t cqi_m_sc_pusch_initial;
	
	/* Указатель на UL_CONFIG.req PDU для пользователя */
	fapi_ul_config_request_pdu_t *ul_config_req_pdu;
	
	lte_ul_harq_buf_descr_t *hbd;
	
	/* Указатели на референы PUSCH DMRS */
	Complex16 *dmrs0;
	Complex16 *dmrs1;
	
	Complex16 *ref_symb0[LTE_N_ANTENNAS_MAX];
	Complex16 *ref_symb1[LTE_N_ANTENNAS_MAX];
	
	/* Оценка TA */
	int32_t ta0[2][LTE_N_ANTENNAS_MAX];
	int32_t ta;
	
	/* Оценка SNR PUSCH */
	int32_t pusch_noise[LTE_N_ANTENNAS_MAX];
	int32_t pusch_sig[LTE_N_ANTENNAS_MAX];
	int32_t pusch_snr[LTE_N_ANTENNAS_MAX];
	int32_t pusch_snr_db[LTE_N_ANTENNAS_MAX];
	
	uint8_t *decouple_ptr;
	
	struct 
	{

		__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 hest0[1200];
		__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 hest1[1200];
	
		__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 hest0_avg[1200];
		__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 hest1_avg[1200];
	
		/* Вспомогательные массивы для оценки TA и FO */
		__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r0[1200];
		__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r1[1200];
	
		__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) int8_t cqi_output[512];
		__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint8_t cqi_output_hard[64];
		__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 dmp_buf[128];
		//__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 harq_in[128];
		//__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 harq_out[128];
		//__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint8_t decouple[65536];
			
		__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint16_t cqi_amount[8][8];
	} d;
	
	//__attribute__((aligned(16))) uint8_t tb[1024];
} lte_ul_pusch_user_descr_t;

typedef struct
{
	lte_ul_pusch_user_descr_t *user;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) cop_job_handle cop_job;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_tvpe_job_t tvpe_job;

} lte_ul_pusch_tvpe_descr_t;

typedef struct
{
	uint32_t frame;
	uint32_t subframe;
	uint32_t n_users;
	uint32_t n_segs;
	uint32_t n_tvpe;
	
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) lte_ul_pusch_user_descr_t users[TRX_NUM_OF_PUSCH_UPH];
	
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) cop_job_handle cop_job;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pusch_job_t pusch_bd;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) segment_in_status_t segment_in_done;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pusch_job_ext_t job_ext;	
	
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pusch_sh_t pusch_sh[TRX_NUM_OF_PUSCH_SH];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pusch_uph_t pusch_uph[TRX_NUM_OF_PUSCH_UPH];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pusch_uss_t status[TRX_NUM_OF_PUSCH_UPH];
	
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) lte_ul_pusch_tvpe_descr_t pusch_tvpe[TRX_NUM_OF_PUSCH_UPH];
	
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint32_t decouple_offset;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint8_t decouple_shared[131072 * 2];
	
} lte_ul_pusch_descr_t;

typedef struct
{
	uint32_t frame_no;
	uint32_t subframe_no;
	//uint32_t sym;
	Complex16 *prach;
} lte_prach_t;

#if 0
typedef struct
{
	lte_trx_t *trx;
	uint32_t frame_no;
	uint32_t slot_no;

	__attribute__((aligned(8))) Complex16 syms[7][LTE_SYMBOL_LEN];
	__attribute__((aligned(8))) Complex16 syms[7][LTE_SYMBOL_LEN];
}lte_slot_t;

typedef struct
{
	lte_trx_t *trx;
	uint32_t frame_no;
	uint32_t slot_no;
	uint32_t len;

	__attribute__((aligned(8))) Complex16 iq[LTE_IQ_PER_SLOT];

}mod_slot_t;
#endif


/**
 * Параметры фрейма 
 */
typedef struct
{
	/// Сэмплрейт
	uint32_t LTE_SAMPLERATE;
	/// Длина символа в отсчетах
	uint32_t LTE_SYMBOL_LEN;
	/// Длина CP символов 0,7
	uint32_t LTE_CP0_LEN;
	/// Длина CP остальных символов
	uint32_t LTE_CPx_LEN;
	/// Количество отсчетов на сабфрейм
	uint32_t LTE_SAMPLES_PER_SUBFRAME;
	
	/* Параметры для CPRI, т.к. они могут расходиться с параметрами LTE */
	/// Сэмплрейт
	uint32_t CPRI_SAMPLERATE;
	/// Длина символа в отсчетах
	uint32_t CPRI_SYMBOL_LEN;
	/// Длина CP символов 0,7
	uint32_t CPRI_CP0_LEN;
	/// Длина CP остальных символов
	uint32_t CPRI_CPx_LEN;
	/// Количество отсчетов на сабфрейм
	uint32_t CPRI_SAMPLES_PER_SUBFRAME;
	uint32_t CPRI_PRACH_SEQ_LEN;
	uint32_t CPRI_PRACH_SEQ_CP;

	/// Параметры для PRACH
	uint32_t LTE_PRACH_SEQ_LEN;
	uint32_t LTE_PRACH_SEQ_CP;
	int32_t LTE_PRACH_Ncs;
//#define LTE_PRACH_Ncs	13
//#define LTE_PRACH_Ncs_125x	16 //(13 * 1.25 = 16.25)
//extern uint32_t LTE_PRACH_Ncs_125x;
//#define LTE_PRACH_Nx	4
	uint32_t LTE_PRACH_Nx;
	uint32_t LTE_PRACH_SEQ_CP_LEN;
	
	/* Коэффициент перевода из семплов в Ts */
	int32_t LTE_PUSCH_TA_Ts_FACTOR;


	/// Количество RB в DL
	uint32_t LTE_N_RB_DL;
	/// Количество RB в UL
	uint32_t LTE_N_RB_UL;
	/// Количество RE в DL
	uint32_t LTE_N_RE;
	/// Количество RE в UL
	uint32_t LTE_N_RE_UL;
	uint32_t LTE_N_SC_DL_HALF;

	/// Размер маски для RB Allocation Type 0
	uint32_t LTE_RB_TYPE0_BITMASK_LENGTH;
	/// Количество RBG
	uint32_t LTE_RBG_SIZE;

//#define LTE_N_SC_DL_HALF	(LTE_N_RB_DL * LTE_N_SC_RB / 2)
	//uint32_t LTE_N_SC_DL_HALF;

	/// Номер первого RE для PSS/SSS
	uint32_t LTE_FIRST_SS_CARRIER;
	
	/// Порог детектора Service Request
	int32_t LTE_SR_THRESHOLD;
	
	/* Количество логических антенн в секторе (spatials) */
	//uint32_t LTE_N_LOG_ANTENNAS_TX;
	//uint32_t LTE_N_LOG_ANTENNAS_RX;

	/* Количество физических антенн в секторе (antennas) */
	uint32_t LTE_N_PHYS_ANTENNAS_TX;
	uint32_t LTE_N_PHYS_ANTENNAS_RX;

	/* Конфигурация PDSCH */
	uint32_t MPSCP;
	
	int32_t LTE_PSS_GAIN_E;
	int32_t LTE_PSS_GAIN_M;
	
	int32_t LTE_SSS_GAIN_E;
	int32_t LTE_SSS_GAIN_M;
	
	int32_t LTE_CSRS_GAIN_E;
	int32_t LTE_CSRS_GAIN_M;
	
	int32_t LTE_PBCH_GAIN_E;
	int32_t LTE_PBCH_GAIN_M;
	
	int32_t LTE_PCFICH_GAIN;
	//int32_t LTE_PHICH_GAIN;
	int32_t LTE_PDCCH_GAIN;
	
	int32_t LTE_CW_GAIN_E_QPSK;
	int32_t LTE_CW_GAIN_M_QPSK;

	int32_t LTE_CW_GAIN_E_QAM16;
	int32_t LTE_CW_GAIN_M_QAM16;

	int32_t LTE_CW_GAIN_E_QAM64;
	int32_t LTE_CW_GAIN_M_QAM64;
	
} lte_frame_params_t;

struct lte_trx_t
{
	lte_enodeb_t *enodeb;
	
	// TRX no, используется в случае нескольких TRX
	uint32_t no;
	
	uint32_t subframe_no_rx;
	uint32_t frame_no_rx;
	
	//uint32_t sym_no_rx;
	
	volatile uint32_t rach_msg3_frame_no;
	volatile uint32_t rach_msg3_subframe_no;
	volatile uint32_t rach_msg3_active;

	uint32_t earfcn;
	uint32_t oversampling;
	uint32_t bsic;

	int32_t power;
	int32_t max_expected_delay;

	/* DL TASK */
	os_task_handle task_dl_handle;
	uint8_t *task_dl_stack;

	/* UL TASK */
	os_task_handle task_ul_handle;
	uint8_t *task_ul_stack;

	/**
	 * Данные слота до OFDM модуляции
	 */
	//os_mem_part_t *mem_lte_subframe;
	//uint8_t *mem_lte_subframe_buf;
	uint32_t lte_dl_subframe_pool_ptr;
	lte_subframe_t **lte_dl_subframe_pool;
	
	lte_ul_subframe_t **lte_ul_subframe_pool;
	uint32_t lte_ul_subframe_pool_ptr;
	
	// Массив дескрипторов HARQ, сразу для всех RNTI для упрощения работы
	lte_ul_harq_descr_t *ul_harq_descr;
	
	// Буферы для процессов HARQ
	uint8_t *ul_harq_buf[TRX_NUM_OF_UL_HARQ_BUFFERS];
	
	// Таймер для HARQ процессов
	uint32_t harq_tti_counter;
	
	lte_ul_pusch_descr_t **ul_pusch_pool;
	uint32_t ul_pusch_pool_ptr;
	
	// PUFFT overall scale, динамически изменяется в зависимости от статуса SATURATE PUFFT
	int32_t pufft_ova_scl;
	
	os_event_handle evq_pusch_ready;
	
	/* Событие готовности PUSCH CQI (демультиплексированные LLR в режиме DECOUPLED) */
	os_event_handle evq_pusch_ctrl_ready;
	
	/* Событие готовности TVPE (PUSCH CQI) */
	os_event_handle evq_tvpe_ready;
	
/*
	os_mem_part_t *mem_lte_job;
	uint8_t *mem_lte_job_buf;
	uint8_t *lte_job_buf;
*/

	os_task_handle task_maple_pdsch_handle;
	uint8_t *task_maple_pdsch_stack;

	//mem_manager_t mm_lte_sf_jobs;
	
#if OLD_CODE
	/**
	 * Модулированные OFDM слоты
	 */
	os_mem_part_t *mem_mod_slot;
	uint8_t *mem_mod_slot_buf;
	uint8_t *mod_slot_buf;
#endif
	
	/**
	 * Очередь сабфреймов на передачу в модулятор
	 */
	os_queue_handle evq_tx_mod_subframe;
	//mod_slot_t *cur_tx_mod_slot;
	uint32_t cur_tx_mod_subframe;
	
	os_task_handle task_prach_rx_handle;
	uint8_t *task_prach_rx_stack;
	os_event_handle evq_prach_rx;
	
	os_event_handle evq_ul_pufft_ready;
	
	os_event_handle evt_subframe_ind;
	//os_event_handle evq_subframe_ind;

	//mod_slot_t *cur_rx_slot;
	uint32_t cur_rx_slot_pos;
	
	/* Очереди FAPI */
	fapi_dl_config_request_t *fapi_dl_config[10];
	fapi_tx_request_t *fapi_tx_req[10];
	fapi_ul_config_request_t *fapi_ul_config[10];
	fapi_hi_dci0_request_t *fapi_hi_dci0[10];
	//fapi_p8_request_t *fapi_p8_request[10];
	fapi_p8_indication_t *fapi_p8_back_indication[10];

	/* Параметры CPRI IQ */
	/* Номер используемого CPRI */
	int32_t cpri_no;
	
	/* Дескрипторы устройств и каналов IQ CPRI */
	sio_dev_handle cpri_iq_handle;
	sio_channel_t cpri_iq_chan_tx[LTE_N_ANTENNAS_MAX];
	sio_channel_t cpri_iq_chan_rx[LTE_N_ANTENNAS_MAX];

	/* Устройство CPRI VSS */
	sio_dev_handle cpri_vss_handle;
	sio_channel_t cpri_vss_chan_tx;
	sio_channel_t cpri_vss_chan_rx;
	
	/* Дескриптор устройства CPRI Ethernet */
	bio_dev_handle cpri_eth_handle;
	
	/* Каналы CPRI Ethernet */
	bio_channel_t cpri_eth_chan_tx;
	bio_channel_t cpri_eth_chan_rx;
	
	/* Фрейм-менеджер для CPRI Ethernet */
	os_frames_pool_t *cpri_eth_frames_pool;
	os_mem_part_t *cpri_eth_buffers_pool;
	uint8_t cpri_eth_mem_manager[MEM_PART_SIZE(ETHERNET_NUM_OF_BUFS)];
	
	/* Счетчики отправленных и обработанных фреймов CPRI Ethernet */
	int32_t cpri_eth_tx_callbacks_counter;
	int32_t cpri_eth_tx_counter;
	int32_t cpri_eth_rx_counter;

	/* Параметры IQ callback */
    cpri_iq_int_cb_param_t rx_iq_callback_param;
	cpri_iq_int_cb_param_t tx_iq_callback_param;
	
	/* Номер активного буфера для передачи по CPRI */
	uint32_t cpri_tx_active_buffer_no;
	/* Номер буфера для подготовки данных */
	uint32_t cpri_tx_prepare_buffer_no;

	/* Номер активного буфера для приема по CPRI */
	uint32_t cpri_rx_active_buffer_no;
	/* Номер буфера для обработки данных */
	uint32_t cpri_rx_process_buffer_no;

	/* Размер буфера на прием/передачу */
	uint32_t iq_buffer_len;
	
	/* Указатели на буферы для антенн */
	Complex16 *iq_rx_buffer[LTE_N_ANTENNAS_MAX][2];
	Complex16 *iq_tx_buffer[LTE_N_ANTENNAS_MAX][2];
	
	uint32_t vss_buffer_len;
	uint8_t *vss_rx_buffer;
	uint8_t *vss_tx_buffer;
	
	/* Задержки CPRI */
	cpri_delays_t cpri_delays;
	
	/* Массив дескрипторов PRACH на обработку */
	lte_prach_t *rx_prach_buffer;
	/* Указатель в массиве дескрипторов PRACH */
	volatile uint32_t rx_prach_no;
	
	//****************************************************************************
	// Variables
	//****************************************************************************
/*	cop_dev_handle maple_handle;
	cop_dev_handle pdsch_handle;
	cop_channel_t pdsch_ch_handle[LTE_NB_ANTENNA_TX];
	*/
	volatile uint32_t num_reaped_jobs;
	volatile uint32_t num_disp_jobs;
	
	uint8_t maple_job_id;
	lte_subframe_t *maple_jobs[256];
	
	uint64_t raw_iq_ts_tx;
	uint64_t raw_iq_ts_rx;
	
	//sio_dev_handle aic_dev, aic_adi, aic_adi_lane;
	// old BSC9132
	//aic_int_cb_params_t aic_rx_int_cb_params, aic_tx_int_cb_params;
	
	/* Вектора для декодирования PUSCH CQI */
	int8_t dw26[92];
	int32_t dw_26_O_RCC;
};

/*
typedef enum
{
	ENODEB_UNCONFIGURED, ENODEB_IDLE, ENODEB_ACTIVE
} enodeb_state_t;
*/

typedef struct
{
	uint8_t *rx_buffer; /* Rx Buffers */
	uint8_t *tx_buffer; /* Tx Buffers */
	bio_channel_t bio_tx; /* BIO Tx Channels */
	bio_channel_t bio_rx; /* BIO Rx Channels */
	bio_dev_handle device_handle; /* Handle to the bio device */
	uint8_t *rx_data; /* Test space */
	uint8_t *tx_data; /* Test space */
	volatile uint32_t tx_counter; /* Run-counter */
	volatile uint32_t tx_callbacks_counter; /* Run-counter */
	volatile uint32_t rx_counter; /* Run-counter */
	os_frames_pool_t *frames_pool;
	os_mem_part_t *mem_part;
	uint8_t *mem_buf;
} cpri_ethernet_channel_t;

struct lte_enodeb_params_t
{
	/* LTE eNodeB params */
	uint32_t N_id_1;
	uint32_t N_id_2;
	uint32_t N_ant;
	uint32_t N_rb_dl;
	uint32_t N_sc_rb;
	uint32_t N_sc_rb_dl_2;
	uint32_t Vshift;
	
	/* SIB2 constants */
#if 0
	uint32_t prach_FreqOffset;
#endif
};

typedef struct
{
	uint8_t pbch_d[96 + (3 * (16 + LTE_PBCH_A))];
	uint8_t pbch_w[3 * 3 * (16 + LTE_PBCH_A)];
	// Байтовое представление PBCH (1 бит на 1 байт)
	uint8_t pbch_e[1920];
	// Битовое представление PBCH (8 бит на 1 байт)
	uint8_t pbch_e_bits[1920 / 8];
} lte_enodeb_pbch_t;

typedef struct
{
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pdsch_ss_header_t pss;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pdsch_ss_header_t sss0;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pdsch_ss_header_t sss5;
	
	// MAPLE-B3 generates CSR references internally by specified Ncell_id and Nport params
	//__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pdsch_pos_ref_sig_header_t csrs;
	
	uint32_t lte_gold_table[20][2][14];
	
	/* CSRS для символа 0 каждого сабфрейма
	 * Используется при формировании 0-го символа (канал PCFICH, PHICH, PDCCH)
	 */
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 lte_csrs_sym0[10][LTE_N_ANTENNAS_MAX][1200];

	lte_enodeb_pbch_t pbch;
	// MAPLE-B3 use User Header #0 for PBCH generation
	//__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) maple_pdsch_pbch_header_t pbch_syms;

	// G values (number of bits in RBs)
	// 10 subframes
	// LTE_N_RB_DL resource blocks
	// 1, 2 or 3 PDCCH symbols 
	// 4 mod_orders (1 == BPSK, 2 == QPSK, 4 == QAM16, 6 == QAM64 
	//uint32_t G_table[10][100][3][4];
	
	/* 
	 * Таблицы PUSCH DMRS
	 * 
	 * Генерируются предвариетльно для каждого сабфрейма
	 * Максимальная длина таблицы расчитана на 110 RB (BW = 20MHz)
	 */
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 pusch_dmrs0[10][110][110 * 12];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 pusch_dmrs1[10][110][110 * 12];
	
	 // UL Reference Signals
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) float ulrs_x_q_re[2048];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) float ulrs_x_q_im[2048];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) float ulrs_r_bar_u_v_re[2048];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) float ulrs_r_bar_u_v_im[2048];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint32_t ulrs_c[160];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint32_t pusch_dmrs_c[1120];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint32_t pucch_dmrs_c[1120];
	
	//__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 ul_ref_sigs[30][2][33][1200];
	Complex16 (*ul_ref_sigs_rx)[30][2][33][1200];
	
	/// nPRS for cyclic shift of DRS \note not part of offical UL-ReferenceSignalsPUSCH ASN1 specification.
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint32_t nPRS[20];
	  /// group hopping sequence for DRS \note not part of offical UL-ReferenceSignalsPUSCH ASN1 specification.
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint32_t grouphop_pusch[20];
	  /// sequence hopping sequence for DRS \note not part of offical UL-ReferenceSignalsPUSCH ASN1 specification.
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint32_t seqhop_pusch[20];

	  /// group hopping sequence for DRS \note not part of offical UL-ReferenceSignalsPUSCH ASN1 specification.
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint32_t grouphop_pucch[20];
	  /// sequence hopping sequence for DRS \note not part of offical UL-ReferenceSignalsPUSCH ASN1 specification.
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint32_t seqhop_pucch[20];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint8_t n_cs_cell[20][LTE_NSYMB_PER_SUBFRAME / 2];
	
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r_pucch_1[10][100][72];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r_pucch_2[10][100][72];
	
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r_pucch_sr[10][100][96];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r_pucch_sr_srs[10][100][96];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r_pucch_ack[10][100][96];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r_pucch_ack_srs[10][100][96];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r_pucch_nack[10][100][96];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r_pucch_nack_srs[10][100][96];
	
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r_pucch_fmt2[10][100][120];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 r_pucch_fmt2_srs[10][100][120];
	
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) uint32_t scrambling_pucch2[10][65536];
	
} lte_enodeb_refsigs_t;

struct lte_enodeb_t
{
	uint32_t no;
	
	uint32_t sector;
	
	secure_id_t secure_id;

	/* Текущее системное время */
	volatile uint32_t system_frame_no;
	volatile uint32_t system_subframe_no;

	/* Предыдущее системное время, используется в обработчиках прерываний,
	 * т.к. прервыание _ПРЕДЫДУЩЕГО_ сабфрейма происходит в _ТЕКУЩЕМ_ системном времени
	 * 
	 * Чтобы не пересчитывать его обратно - просто используются эти значения
	 */
	volatile uint32_t prev_system_frame_no;
	volatile uint32_t prev_system_subframe_no;

	uint32_t sync_clock_tx; /* Флаг синхронизации по FN передатчика */
	
	/* Флаг остановки eNodeB */
	volatile uint32_t flag_stop;
	
	uint32_t cpri_initialized;

	//uint32_t ipc_pa_ready;
	
	lte_frame_params_t fp;
	lte_enodeb_params_t lte_enodeb_params;
	lte_enodeb_refsigs_t *refsigs;

	uint32_t N_id_cell;
	
	uint8_t pbch_pdu[4];
	/// Флаг корректного PBCH PDU, сбрасывается после 4 передачи
	int32_t pbch_pdu_ok;
	/// REGs assigned to PCFICH
	uint32_t pcfich_reg[4];
	/// Index of first REG assigned to PCFICH
	uint32_t pcfich_first_reg_idx;

	/// REGs assigned to PHICH
	uint32_t phich_reg[MAX_NUM_PHICH_GROUPS][3];
	
	lte_l1_state_t state;
	lte_trx_t trx;
	fapi_config_t fapi_config;
	
	/* Таблицы PRACH */
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 prach_dds[30720];
	
	int32_t n_prach_x_u;
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) float _Complex prach_x_u_f[64][1024];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 prach_x_u[64][1024];
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 prach_x_u_fft[64][1024];
	
	/* Таблица DDS для сдвига на половину частотного канала при обрабтоку UL */
	/* В BSC9132 используется массив непосредственно в структуре
	 * Для B4860 это казатель на массив в DDR
	 */
#ifdef B4860
	Complex16 *ul_shift_dds;
#else
	__attribute__((aligned(ARCH_CACHE_LINE_SIZE))) Complex16 ul_shift_dds[2048];
#endif

};

/* DCI */
#define NUM_DCI_MAX 32
typedef struct
{
	uint32_t Num_ue_spec_dci;
	uint32_t Num_common_dci;
	uint32_t nCCE;
	DCI_ALLOC_t dci_alloc[NUM_DCI_MAX];
} DCI_PDU;

extern Complex16 liblte_pss_mod[3][62] __attribute__((aligned(8)));
extern Complex16 liblte_sss_mod[3][168][63][2];

extern Complex16 rx_buffer_raw[MAPLE_NUM_SECTORS][MAPLE_NUM_ANT][10 * LTE_SAMPLES_PER_SUBFRAME_MAX];
extern Complex16 tx_buffer_raw[MAPLE_NUM_SECTORS][MAPLE_NUM_ANT][10 * LTE_SAMPLES_PER_SUBFRAME_MAX];

extern lte_enodeb_t g_enodeb_inst[NUM_ENODEB];

extern fapi_ipc_cfg_t g_fapi_ipc_cfg;

os_status lte_enodeb_init_trxs(lte_enodeb_t *bts);
lte_status_t lte_enodeb_configure(lte_enodeb_t *enodeb, fapi_config_t *new_config);
lte_enodeb_t *lte_enodeb_init(int32_t enodeb_num);
lte_status_t lte_enodeb_start(lte_enodeb_t *enodeb);
lte_status_t lte_enodeb_stop(lte_enodeb_t *enodeb);


void sweep_cache(uint32_t addr, int size, uint32_t mode);
void sweep_cache_async(uint32_t addr, int size, uint32_t mode);
void MemMngrCreate(uint32_t num_buffers, uint32_t buff_size, uint32_t alignment, os_mem_type mem_type, mem_manager_t *mngr);
os_status lte_maple_init(lte_enodeb_t *enodeb);
void lte_maple_close(lte_enodeb_t *enodeb);
os_status lte_maple_pufft_start(lte_enodeb_t *enodeb, lte_ul_subframe_t *sf);
os_status lte_aic_open_channels();
os_status lte_aic_close_channels();
os_status lte_maple_fft_ifft(Complex16 *iq_in, Complex16 *iq_out, int32_t len, int32_t inverse);
void ipc_send_msg(fapi_ipc_msg_t *msg);
fapi_ipc_msg_t *ipc_recv_msg();


#endif
