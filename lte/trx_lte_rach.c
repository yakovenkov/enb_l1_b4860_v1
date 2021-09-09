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

#define PRACH_ONLY_ONE_PREAMBLE

#define COMPLEX_DIV

// Debug PRACH DDS output
//#define DEBUG_PRACH_DDS
//#define DEBUG_PRACH_DETECT_DDS_ON_DETECT

//#define SCALE_MEAN

// SC3900 workaround
#define creal16(x) extract_h(x)
#define cimag16(x) extract_l(x)

#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_MAPLE_OUTPUT && !defined DEBUG_OPT_OFF && !defined(PUSCH_TEST)
#pragma opt_level = "O3"
#else
//#pragma opt_level = "O3"
#endif

#ifdef TRX_DEBUG_BUFFER

#define DEBUG_PRACH_BUFFER_STEP(e) (NUM_OF_SAMPLES1(e) * LTE_NSYMB_PER_SUBFRAME)
//#define DEBUG_PRACH_BUFFER_SIZE (DEBUG_PRACH_BUFFER_STEP * 512)
#define DEBUG_PRACH_BUFFER_SIZE (8192*1024)

Complex16 debug_prach_buffer[DEBUG_PRACH_BUFFER_SIZE] __attribute__((section(".shared_data_ddr0_cacheable_bss"), aligned(16)));
uint32_t debug_prach_buffer_ptr = 0;
uint32_t dbg_addr_mark = 0;
#define DBG_MARK_OUT_I { dbg_buf[dbg_addr_mark++] = V_swapb(out_i); }

#else
#define DBG_MARK_OUT_I	;
#endif

int32_t lte_trx_is_rach_msg3(lte_enodeb_t *enodeb, lte_ul_subframe_t *sf);
static lte_status_t trx_ul_process_uci_pdu(lte_enodeb_t *enodeb, int32_t ant_no, fapi_ul_config_request_pdu_t *ul_req_pdu, uint32_t srs_present,
		lte_ul_subframe_t *rx_sf, int8_t *scale_vector, fapi_harq_indication_pdu_t *harq_pdu,
		fapi_sr_indication_pdu_t *sr_pdu,
		fapi_cqi_indication_pdu_t *cqi_pdu,
		uint8_t *cqi_raw_data,
		int32_t *sr_detected, uint32_t dump_results);

/*
 static uint8_t tx_buffer[NUM_OF_ANTENNAS][TX_BUFFER_SIZE] __attribute__((section(".local_data_ddr1_nocacheable")));
 #pragma align tx_buffer 16
 */
static DCI_PDU g_dci_pdu;

extern cop_dev_handle maple_handle[NUM_MAPLES_USED];
extern cop_dev_handle pdsch_handle;
extern cop_channel_t pdsch_ch_handle[LTE_N_ANTENNAS_MAX];
extern cop_channel_t pusch_ch_handle[LTE_N_ANTENNAS_MAX];

extern volatile uint32_t enodeb_flag_stop;

static lte_status_t trx_maple_pdsch_direct(lte_subframe_t *cur_sf, uint32_t first_run);

/* DSP fucntions */
void sc3850_fir_real_16x16_asm_dec2(Word16 x[], Word16 h[], Word16 y[], Word16 nr, Word16 nh);
//int sc3850_vector_complex_mult_conj_asm(Complex16 *restrict x, Complex16 *restrict y, Complex16 *restrict z, int N);
int sc3850_vector_complex_mult_conj_sc3900(short *restrict input, short *restrict coef, short *restrict result, int N);
os_status maple_fft_ifft(Complex16 *iq_in, Complex16 *iq_out, int32_t len, int32_t inverse);

extern uint32_t dftsizes[33];
extern Complex16 ul_ref_sigs_rx[30][2][33][1200];


/**
 * Определение сабфрейма PRACH в зависимости от конфигурации
 * @param trx
 */
int32_t lte_trx_is_prach_subframe(lte_trx_t *trx)
{
	/* Сейчас поддерживается только Config Index 1 */
	if ((trx->frame_no_rx & 1) == 0 && (trx->subframe_no_rx == 1))
		return 1;

	return 0;
}

/**
 * Определиние сабфрейма с RACH Msg3
 */
int32_t lte_trx_is_rach_msg3(lte_enodeb_t *enodeb, lte_ul_subframe_t *sf)
{
	/* Сейчас поддерживается только Config Index 1 */
	if (enodeb->trx.rach_msg3_active)
	{
		if ((sf->frame_no == enodeb->trx.rach_msg3_frame_no) && (sf->subframe_no == enodeb->trx.rach_msg3_subframe_no))
		{
			enodeb->trx.rach_msg3_active = 0;
			return 1;
		}
	}

	return 0;
}

/**
 * Процесс обработки принятых PRACH 
 * @param p
 */

#define HB_FILTER_LEN	16
#define HB_FILTER_LEN_HALF	8

Word16 coeffs_hb1[HB_FILTER_LEN] =
	{ -868, 0, 1445, 0, -3062, 0, 10286, 16383, 10286, 0, -3062, 0, 1445, 0, -868, 0 // Padding zero
	};
#pragma align coeffs_hb1    16

Word16 coeffs_hb1_half[HB_FILTER_LEN_HALF] __attribute__((aligned(16))) =
	{ -868, 1445, -3062, 10286, 10286, -3062, 1445, -868 };

#define FIR_4_LEN 16
Word16 coeffs_fir4[FIR_4_LEN] __attribute__((aligned(16))) =
	{ -114, -159, -140, 291, 1449, 3284, 5246, 6524, 6524, 5246, 3284, 1449, 291, -140, -159, -114 };

Complex16 coeffs_fir4_cplx[FIR_4_LEN] __attribute__((aligned(16))) =
	{ TO_COMPLEX16(-114, 0), TO_COMPLEX16(-159, 0), TO_COMPLEX16(-140, 0), TO_COMPLEX16(291, 0), TO_COMPLEX16(1449, 0), TO_COMPLEX16(3284, 0),
		TO_COMPLEX16(5246, 0), TO_COMPLEX16(6524, 0), TO_COMPLEX16(6524, 0), TO_COMPLEX16(5246, 0), TO_COMPLEX16(3284, 0), TO_COMPLEX16(1449, 0),
		TO_COMPLEX16(291, 0), TO_COMPLEX16(-140, 0), TO_COMPLEX16(-159, 0), TO_COMPLEX16(-114, 0) };

#define FIR_FILTER_LEN	16
Word16 coeffs_fir1[FIR_FILTER_LEN] __attribute__((aligned(16))) =
	{ 102, 145, -103, -893, -1134, 1224, 6296, 10743, 10743, 6296, 1224, -1134, -893, -103, 145, 102 };

Complex16 coeffs_fir3_cplx[FIR_FILTER_LEN] __attribute__((aligned(16))) =
	{ TO_COMPLEX16(102, 0), TO_COMPLEX16(145, 0), TO_COMPLEX16(-103, 0), TO_COMPLEX16(-893, 0), TO_COMPLEX16(-1134, 0), TO_COMPLEX16(1224, 0),
		TO_COMPLEX16(6296, 0), TO_COMPLEX16(10743, 0), TO_COMPLEX16(10743, 0), TO_COMPLEX16(6296, 0), TO_COMPLEX16(1224, 0), TO_COMPLEX16(-1134, 0),
		TO_COMPLEX16(-893, 0), TO_COMPLEX16(-103, 0), TO_COMPLEX16(145, 0), TO_COMPLEX16(102, 0) };

#define PRACH_BUFFER_PAD	8

static Complex16 prach_seq_filt_cplx[LTE_PRACH_SEQ_CP_LEN_MAX + PRACH_BUFFER_PAD] __attribute__((section(".local_data_ddr0_cacheable_bss"), aligned(16)));
static Complex16 prach_seq_cplx[LTE_PRACH_SEQ_CP_LEN_MAX + PRACH_BUFFER_PAD] __attribute__((section(".local_data_ddr0_cacheable_bss"), aligned(16)));
static Complex16 prach_seq[LTE_PRACH_SEQ_LEN_MAX / 6] __attribute__((section(".local_data_ddr0_cacheable_bss"), aligned(16)));
static Complex16 prach_seq_fft[LTE_PRACH_SEQ_LEN_MAX / 6] __attribute__((section(".local_data_ddr0_cacheable_bss"), aligned(16)));
static Complex16 prach_corr_fft[LTE_PRACH_SEQ_LEN_MAX / 6] __attribute__((section(".local_data_ddr0_cacheable_bss"), aligned(16)));

Word32 prach_corr[LTE_PRACH_SEQ_LEN_MAX / 6] __attribute__((aligned(16))); // __attribute__((section(".local_data_ddr0_cacheable_bss")));

#define PRACH_NEW_HB_LEN	8
Complex16 prach_new_hb1[PRACH_NEW_HB_LEN] =
	{ TO_COMPLEX16(-1074, 0), TO_COMPLEX16(0, 0), TO_COMPLEX16(9264, 0), TO_COMPLEX16(16384, 0), TO_COMPLEX16(9264, 0), TO_COMPLEX16(0, 0),
		TO_COMPLEX16(-1074, 0), TO_COMPLEX16(0, 0) };

Complex16 prach_new_hb2[PRACH_NEW_HB_LEN] =
	{ TO_COMPLEX16(-1212, 0), TO_COMPLEX16(0, 0), TO_COMPLEX16(9384, 0), TO_COMPLEX16(16384, 0), TO_COMPLEX16(9384, 0), TO_COMPLEX16(0, 0),
		TO_COMPLEX16(-1212, 0), TO_COMPLEX16(0, 0) };

#define PRACH_NEW_FIR_LEN 16
Complex16 prach_new_fir3[PRACH_NEW_FIR_LEN] =
	{ TO_COMPLEX16(3117, 0), TO_COMPLEX16(-642, 0), TO_COMPLEX16(-3649, 0), TO_COMPLEX16(-4880, 0), TO_COMPLEX16(-913, 0), TO_COMPLEX16(8185, 0),
		TO_COMPLEX16(17831, 0), TO_COMPLEX16(21974, 0), TO_COMPLEX16(17831, 0), TO_COMPLEX16(8185, 0), TO_COMPLEX16(-913, 0), TO_COMPLEX16(-4880, 0),
		TO_COMPLEX16(-3649, 0), TO_COMPLEX16(-642, 0), TO_COMPLEX16(3117, 0), 0 };

typedef struct prach_s
{
	int32_t y1;
	int32_t y2;
	int32_t y3;
	int32_t denum;
	int32_t num;
	int32_t i_interp;
	int32_t max_i_fr;
	int32_t ta_fr;
} prach_t;

int32_t prach_value[LTE_PRACH_DETECT_MAX];
int32_t prach_preamble[LTE_PRACH_DETECT_MAX];
int32_t prach_ta[LTE_PRACH_DETECT_MAX];
int32_t prach_rnti[LTE_PRACH_DETECT_MAX];
int32_t prach_detected = 0;
prach_t prach_descr[LTE_PRACH_DETECT_MAX];

#define USE_PRACH_FILTER
#define USE_FIR_4
#define USE_FIR_COMPLEX
#define USE_FIR3_COMPLEX

void trx_prach_task(uint32_t p)
{
#if !defined(DEBUG_OPT_OFF)
#pragma opt_level = "O3"
#endif
	
	os_status status;
	lte_trx_t *trx = (lte_trx_t *) p;
	lte_enodeb_t *enodeb = trx->enodeb;
	uint32_t rx_prach_no = 0;
	volatile uint64_t loop_start, loop_end;
	uint32_t loop_counter = 0;
	int32_t i, j;
	lte_prach_t *rx_prach;
	uint32_t first_run = 1;

	//enodeb = trx->enodeb;

	while (1)
	{
		if (enodeb->state != L1_RUNNING)
		{
			first_run = 1;
			osTaskDelay(1);
			continue;
		}

		if (first_run)
		{
//			osEventQueueReset(enodeb_inst.trx.evq_prach_rx, 0);
			first_run = 0;
		}

		/* Ожидание сабфрейма PRACH */
		status = osEventQueuePend(enodeb->trx.evq_prach_rx, (uint32_t *) &rx_prach, 2);
		if (status == OS_ERR_EVENT_QUEUE_TIMEOUT)
		{
			/* Если произошел таймаут - перезапуск внешнего цикла,
			 * т.к. могло измениться изменение состояния работы L1
			 */
			continue;
		}

		OS_ASSERT_COND(status == OS_SUCCESS);
		
		DBAR_IBLL();

		LOG_EVENT_SF(LOGEVT_RACH_TASK_BEGIN, rx_prach->frame_no, rx_prach->subframe_no, 0);
		//INFO(DTRX, "PRACH %i:%i\n", rx_prach->frame_no, rx_prach->subframe_no);
		
		loop_start = log_get_timer_value();
		
		// Контроль когерентности
		//DBAR_HWSYNC();

		//sweep_cache((uint32_t) rx_prach->prach, (NUM_OF_SAMPLES1 * LTE_NSYMB_PER_SUBFRAME) * sizeof(Complex16), CACHE_INVALIDATE);

		/* 
		 * Выбор отсчетов из входного буфера и
		 * снос канала PRACH в 0 
		 */

		// Умножение производится за один вызов, т.к. отсчеты в буфере CPRI идет последовательно (в отличии от BSC9132)
		sc3850_vector_complex_mult_asm((short *) &rx_prach->prach[enodeb->fp.CPRI_PRACH_SEQ_CP],
				(short *)&enodeb->prach_dds[0], (short *)&prach_seq_cplx[PRACH_BUFFER_PAD], enodeb->fp.CPRI_PRACH_SEQ_LEN);

		for (i = 0; i < PRACH_BUFFER_PAD; i++)
		{
			prach_seq_cplx[i] = 0;
		}

#ifdef DEBUG_PRACH_DDS
		Complex16 *dbg_buf = &debug_prach_buffer[debug_prach_buffer_ptr];
		int32_t dbg_sym_no = 0;
		int32_t sym_ptr = 0;
		
		if(debug_prach_buffer_ptr + 20000 >= DEBUG_PRACH_BUFFER_SIZE)
		{
			os_phys_ptr phys_ptr = 0;
			osMmuDataVirtToPhys(debug_prach_buffer, &phys_ptr);
			INFO(DTRX, "Debug PRACH buffer full, address=0x%08x\n", phys_ptr);
			debug_prach_buffer_ptr = 0;
		}
		
		for(i=0; i<LTE_SAMPLES_PER_SUBFRAME; i++)
		{
			dbg_buf[i] = V_swapb2(rx_prach->prach[i]);
		}
				
		dbg_buf = &debug_prach_buffer[debug_prach_buffer_ptr + 10000];
		
		for(i=0; i<LTE_PRACH_SEQ_LEN; i++)
		{
			dbg_buf[i] = V_swapb2(prach_seq_cplx[PRACH_BUFFER_PAD + i]);
		}
		
#ifndef DEBUG_PRACH_DETECT_DDS_ON_DETECT
		debug_prach_buffer_ptr += 20000;
#endif
#endif

		LOG_EVENT(LOGEVT_RACH_TASK_EVENT, 0);

		/* 
		 * Каскадная фильтрация канала PRACH для получения выходной частоту 1.28МГц (1024 точки)
		 * D - коэффициент децимации
		 * Для частоты 30.720МГц D=24 (2, 2, 3, 3)
		 * Для частоты 15.360МГц D=12 (2, 2, 3)
		 * Для частоты 7.680МГц D=6 (2, 3)
		 * Для частота 3.84МГц D=3 (3)
		 */

		int32_t seq_len = enodeb->fp.CPRI_PRACH_SEQ_LEN;
		
		//if(enodeb->fp.LTE_N_RB_DL == 100 || enodeb->fp.LTE_N_RB_DL == 50)
		if(enodeb->fp.CPRI_SYMBOL_LEN == 2048 || enodeb->fp.CPRI_SYMBOL_LEN == 1024)
		{
			/* HBdec2 - HBdec2 - FIRdec3 */
	
			/* HB1 */
			//sc3850_fir_complex_16x16_asm(&prach_seq_cplx[PRACH_BUFFER_PAD], prach_new_hb1, (Word16 *) &prach_seq_filt_cplx, LTE_PRACH_SEQ_LEN, PRACH_NEW_HB_LEN);
			fir_complex_16x16((const Word16 *) &prach_seq_cplx[PRACH_BUFFER_PAD], (const Word16 *)prach_new_hb1, (Word16 *) &prach_seq_filt_cplx,
					seq_len, PRACH_NEW_HB_LEN);
	
			/* Decimate by 2 */
			/* !!!!!!!!!
			 * Задержка фильтра 3 символа, поэтому отсчет индекса j начинается с 3 
			 * */
			for (i = PRACH_BUFFER_PAD, j = 3; i < PRACH_BUFFER_PAD + seq_len / 2; i++, j += 2)
			{
//#pragma loop_unroll 8
				prach_seq_cplx[i] = prach_seq_filt_cplx[j];
			}
	
			for (i = 0; i < PRACH_BUFFER_PAD; i++)
			{
//#pragma loop_unroll 8
				prach_seq_cplx[i] = 0;
			}
	
			LOG_EVENT(LOGEVT_RACH_TASK_EVENT, 1);
			
			// Длина децимированного массива
			seq_len = seq_len / 2;
		}
		
		//if(enodeb->fp.LTE_N_RB_DL == 100 || enodeb->fp.LTE_N_RB_DL == 50 || enodeb->fp.LTE_N_RB_DL == 25)
		if(enodeb->fp.CPRI_SYMBOL_LEN == 2048 || enodeb->fp.CPRI_SYMBOL_LEN == 1024 || enodeb->fp.CPRI_SYMBOL_LEN == 512)
		{
			/* HB2 */
			//sc3850_fir_complex_16x16_asm(&prach_seq_cplx[PRACH_BUFFER_PAD], prach_new_hb2, (Word16 *) &prach_seq_filt_cplx, LTE_PRACH_SEQ_LEN / 2,
			//		PRACH_NEW_HB_LEN);
			fir_complex_16x16(&prach_seq_cplx[PRACH_BUFFER_PAD], prach_new_hb2, (Word16 *) &prach_seq_filt_cplx,
					seq_len, PRACH_NEW_HB_LEN);
	
			/* Decimate by 2 */
			/* !!!!!!!!!
			 * Задержка фальтра 3 символа, поэтому отсчет индекса j начинается с 3 
			 * */
			for (i = PRACH_BUFFER_PAD, j = 3; i < PRACH_BUFFER_PAD + seq_len / 2; i++, j += 2)
			{
//	#pragma loop_unroll 8
				prach_seq_cplx[i] = prach_seq_filt_cplx[j];
			}
	
			for (i = 0; i < PRACH_BUFFER_PAD; i++)
			{
//	#pragma loop_unroll 8
				prach_seq_cplx[i] = 0;
			}
	
			LOG_EVENT(LOGEVT_RACH_TASK_EVENT, 2);
			
			seq_len = seq_len / 2;
		}
		
		//if(enodeb->fp.LTE_N_RB_DL == 100)
		if(enodeb->fp.CPRI_SYMBOL_LEN == 2048)
		{
			/* FIR */
			//sc3850_fir_complex_16x16_asm(&prach_seq_cplx[PRACH_BUFFER_PAD], prach_new_fir3, (Word16 *) &prach_seq_filt_cplx, LTE_PRACH_SEQ_LEN / 4,
					//PRACH_NEW_FIR_LEN);
			fir_complex_16x16(&prach_seq_cplx[PRACH_BUFFER_PAD], prach_new_fir3, (Word16 *) &prach_seq_filt_cplx, seq_len,
					PRACH_NEW_FIR_LEN);

			/* Decimate by 3 */
			/* !!!!!!!!!
			 * Задержка фальтра 7 символа, поэтому отсчет индекса j начинается с 3 
			 * */
			for (i = PRACH_BUFFER_PAD, j = 7; i < seq_len / 3 + PRACH_BUFFER_PAD; i++, j += 3)
			{
				prach_seq_cplx[i] = prach_seq_filt_cplx[j];
			}

			for (i = 0; i < PRACH_BUFFER_PAD; i++)
			{
				prach_seq_cplx[i] = 0;
			}
				
			LOG_EVENT(LOGEVT_RACH_TASK_EVENT, 3);
			
			seq_len = seq_len / 3;
		}
		
		/* FIR */
		//sc3850_fir_complex_16x16_asm(&prach_seq_cplx[PRACH_BUFFER_PAD], prach_new_fir3, (Word16 *) &prach_seq_filt_cplx, LTE_PRACH_SEQ_LEN / 4,
				//PRACH_NEW_FIR_LEN);
		fir_complex_16x16(&prach_seq_cplx[PRACH_BUFFER_PAD], prach_new_fir3, (Word16 *) &prach_seq_filt_cplx, seq_len,
				PRACH_NEW_FIR_LEN);

		/* Decimate by 3 */
		/* !!!!!!!!!
		 * Задержка фальтра 7 символов, поэтому отсчет индекса j начинается с 7 
		 */
		for (i = 0, j = 7; i < seq_len / 3; i++, j += 3)
		{
			prach_seq_cplx[i] = prach_seq_filt_cplx[j];
		}

		LOG_EVENT(LOGEVT_RACH_TASK_EVENT, 4);
		
		seq_len = seq_len / 3;
		
		// После децимаций, при любой частоте, должно получиться 1024 отсчета
		OS_ASSERT_COND(seq_len == 1024);
		
		status = maple_fft_ifft(prach_seq_cplx, prach_seq_fft, seq_len, 0);
		OS_ASSERT_COND(status == OS_SUCCESS);

		LOG_EVENT(LOGEVT_RACH_TASK_EVENT, 5);
		
		int32_t N_pre = 0;
		int32_t N_x_u = 0;
		
#ifdef PRACH_ONLY_ONE_PREAMBLE
			int32_t max_prach_val = -1;
			int32_t max_prach_preamble = -1;
			uint32_t max_prach_idx = 0;
			uint32_t max_prach_ta = 0;
			uint32_t max_prach_start = 0;
			uint32_t max_prach_stop = 0;
#endif
	
		prach_detected = 0;
		
		//while(N_pre < 64 && prach_detected < LTE_PRACH_DETECT_MAX)
		while(N_pre < LTE_PRACH_N_PRE_MAX && prach_detected < LTE_PRACH_DETECT_MAX)
		{
			/* Перемножение на подготовленный вектор ZC */
			int32_t rx_zc_corr = 0;
			
			/* Выбор подготовленной последовательности PRACH */
			Complex16 *prach_x_u_fft = enodeb->prach_x_u_fft[N_x_u++];
	
			for (i = 0, j = 512 + 92; i < 420; i++, j++)
			{
				// В SC3900 операнды поменяны местами, conj берется от первого операнда
				// это выражение вычисляет:
				// prach_seq_fft[j] * conj(prach_x_u_fft[i])
				prach_seq[i] = __mpycx_c_sr_2w(prach_x_u_fft[i], prach_seq_fft[j]);
			}
	
			for (i = 420, j = 0; i < 839; i++, j++)
			{
				// В SC3900 операнды поменяны местами, conj берется от первого операнда
				// это выражение вычисляет:
				// prach_seq_fft[j] * conj(prach_x_u_fft[i])
				prach_seq[i] = __mpycx_c_sr_2w(prach_x_u_fft[i], prach_seq_fft[j]);
			}
	
			LOG_EVENT(LOGEVT_RACH_TASK_EVENT, 6);
			
#define PRACH_FFT_LEN 1024
			seq_len = PRACH_FFT_LEN;
	
			for(i=839; i< seq_len * 2; i++)
			{
//#pragma loop_unroll 8
				prach_seq[i] = 0;
			}
	
			status = maple_fft_ifft(prach_seq, prach_corr_fft, seq_len, 1);
			OS_ASSERT_COND(status == OS_SUCCESS);
	
			LOG_EVENT(LOGEVT_RACH_TASK_EVENT, 7);
	
			/* Расчет среднего значения и поиск пиков */
			int32_t prach_mean = 0;
	
			for (i = 0; i < seq_len; i++)
			{
				Word40 prach_norm = X_mpyd(prach_corr_fft[i], prach_corr_fft[i]);
				int32_t prach_norm_round = X_round(prach_norm);
				prach_mean += prach_norm_round;
	
				/* Переворот вектора корреляции, чтобы было удобнее работать дальше */
				//prach_corr[LTE_PRACH_SEQ_LEN/12 - 1 - i] = _round(L_mpyd(prach_seq_fft[i], prach_seq_fft[i]));
				//prach_corr[i] = _round(L_mpyd(prach_seq_fft[i], prach_seq_fft[i]));
				prach_corr[i] = prach_norm_round;
			}
	
			prach_mean = prach_mean / seq_len;
	
			i = 0;
			//prach_detected = 0;
			int32_t nx_i;
			int32_t ncs_i;
	
			/* TODO: сделать пересчет TA с учетом коэффициента масштабирования времени
			 * при переходе от 839 точек к 1024 точкам
			 * 
			 * Время "растягивается" в 1024/839 ~~ 1.2205 раз
			 */
#if PRACH_FFT_LEN == 1024
#define NCS_FFT_ZC_COEFF ((1024 * 65536) / 839)
#define NCS_839_1024_COEFF ((839 * 65536) / 1024)
#else
#define NCS_FFT_ZC_COEFF ((2048 * 65536) / 839)
//#define NCS_839_2048_COEFF ((839 * 65536) / 2048)
#endif
	
//#define LTE_PRACH_GUARD_SAMPLES 3 // old number of guard samples
#define LTE_PRACH_GUARD_SAMPLES 3
	
			//Word32 Ncs_fr = (LTE_PRACH_Ncs * NCS_FR_COEFF);
			ncs_i = 0;

			/* FIXME: разобраться с этим смещением 
			 * В результирующем массиве корреляции пик, соответствующий TA0, находится на отсчете 1008,
			 * независимо от значения ZCZC
			 * Почему 1008 - надо разобраться
			 * Значение 826 = 1008 / 1.2205 = 1008 / (1024/839)
			 */
			/* Корректирующее значение начала 0 преамбулы
			 * Почему-то преамбула всегда начинается на отсчете 826
			 */
			int32_t start_corr = (826 - 839); /* == 13 */
			int32_t start839 = enodeb->fp.LTE_PRACH_Ncs + start_corr;
			
			// Проверка всех преамбул
			//for (nx_i = 0; nx_i < enodeb->fp.LTE_PRACH_Nx, N_pre < 64; nx_i++)
			for (nx_i = 0; nx_i < enodeb->fp.LTE_PRACH_Nx, N_pre < LTE_PRACH_N_PRE_MAX; nx_i++)
			//for (nx_i = 0; nx_i < 64; nx_i++)
			{
				// Начальный отсчет в поле 839
				// Расположение преамбул обратное, расположение отсчетов в преамбуле прямое
				start839 -= enodeb->fp.LTE_PRACH_Ncs;
				
				if(start839 < 0)
				{
					start839 += 839;
				}
#if 0				
				// Начальный и конечный отсчеты преамбулы в поле 2048
				int32_t i_start =_round(start839 * NCS_FFT_ZC_COEFF);
				int32_t i_stop = _round((start839 + enodeb->fp.LTE_PRACH_Ncs) * NCS_FFT_ZC_COEFF);

#ifndef PRACH_ONLY_ONE_PREAMBLE
				int32_t max_prach_val = -1;
				int32_t max_prach_preamble = -1;
				int32_t max_prach_idx = -1;
				int32_t max_prach_ta = -1;
#endif
				for (i = i_start; i < i_stop; i++)
				{
					/* was: if (prach_corr[i] > 50 * prach_mean && prach_corr[i] > max_prach_val) */
					if (prach_corr[i] > 50 * prach_mean && prach_corr[i] > max_prach_val)
					{
						max_prach_val  = prach_corr[i];
						max_prach_idx = i;
						max_prach_ta = i - i_start;
						max_prach_start = i_start;
						max_prach_stop = i_stop;
						//max_prach_preamble = nx_i;
						max_prach_preamble = N_pre;
					}
				}
#else
				/*
				 * Вариант с зацикливанием буфера prach_corr, т.к. преамбула может переходить через его начало-конец
				 */

				int32_t i_start =_round(start839 * NCS_FFT_ZC_COEFF);
				int32_t Ncs_fft = _round(enodeb->fp.LTE_PRACH_Ncs * NCS_FFT_ZC_COEFF);

#ifndef PRACH_ONLY_ONE_PREAMBLE
				int32_t max_prach_val = -1;
				int32_t max_prach_preamble = -1;
				int32_t max_prach_idx = -1;
				int32_t max_prach_ta = -1;
#endif
				for (i = 0; i < Ncs_fft; i++)
				{
					int32_t corr_val = prach_corr[(i_start + i) % PRACH_FFT_LEN];
					if (corr_val > 50 * prach_mean && corr_val > max_prach_val)
					{
						max_prach_val  = corr_val;
						max_prach_idx = i_start + i;
						max_prach_ta = i;
						max_prach_start = i_start;
						max_prach_stop = i_start + Ncs_fft;
						//max_prach_preamble = nx_i;
						max_prach_preamble = N_pre;
					}
				}
#endif
#ifndef PRACH_ONLY_ONE_PREAMBLE
				if (max_prach_preamble >= 0)
				{
					prach_value[prach_detected] = max_prach_val;
					prach_preamble[prach_detected] = max_prach_preamble;
		
					int32_t y1 = (prach_corr[max_prach_idx - 1]);
					int32_t y2 = (prach_corr[max_prach_idx]);
					int32_t y3 = (prach_corr[max_prach_idx + 1]);
					int32_t denum = 2 * y1 + 2 * y3 - 4 * y2;
					int32_t num = L_deposit_h(y1 - y3);
		
					/* TODO: исправить интерполяцию на квадратичную */
					int32_t i_interp = num / denum;
					int32_t max_i_fr = L_deposit_h(max_prach_ta) + i_interp;
		
					//int32_t ta_fr = max_i_fr * (LTE_SAMPLES_PER_SUBFRAME / 1280 / 2); // NTa в отсчетах Fs
					
					//  пересчет из отсчетов Fs в TA=16*Ts 
					//int32_t ta_fr = max_i_fr * (LTE_SAMPLES_PER_SUBFRAME / 1280 / 2) * (30720 / LTE_SAMPLES_PER_SUBFRAME) / 16;
					//int32_t ta_fr = max_i_fr * (30720) / (16 * 1280 * 2);
					int32_t ta_fr = (max_i_fr * 3) / 4;
					
#if 1				
					INFO(DRACH, "FR i=%i, %i %i %i, %i %i, %i, fr=%i, ta=%i\n",
							max_prach_ta,
							y1, y2, y3,
							num, denum,
							i_interp,
							max_i_fr,
							ta_fr);
#endif
					
					if (ta_fr < 0)
					{
						/* TA < 0
						 * 
						 * Сейчас обнуляем
						 */
						DBG(DRACH, "Negative PRACH at i=%i, %i %i %i, %i %i, %i, fr=%i, ta=%i\n", max_prach_ta, y1, y2, y3, num, denum, i_interp, max_i_fr, ta_fr);
						ta_fr = 0;
					}
		
					prach_ta[prach_detected] = _round(ta_fr);
		
					/* 
					 * RA-RNTI= 1 + t_id+10*f_id
					 * for FDD f_id = 0
					 */
					prach_rnti[prach_detected] = 1 + rx_prach->subframe_no;
					
					prach_detected++;
					
					//break;
					
					if (prach_detected >= LTE_PRACH_DETECT_MAX)
						break;
									
				}
#else
				if (max_prach_preamble >= 0)
				{
					prach_descr[0].y1 = (prach_corr[(max_prach_idx - 1) % PRACH_FFT_LEN]);
					prach_descr[0].y2 = (prach_corr[max_prach_idx % PRACH_FFT_LEN]);
					prach_descr[0].y3 = (prach_corr[(max_prach_idx + 1) % PRACH_FFT_LEN]);
					prach_descr[0].denum = 2 * prach_descr[0].y1 + 2 * prach_descr[0].y3 - 4 * prach_descr[0].y2;
					prach_descr[0].num = L_deposit_h(prach_descr[0].y1 - prach_descr[0].y3);
	
					/* TODO: исправить интерполяцию на квадратичную */
					prach_descr[0].i_interp = prach_descr[0].num / prach_descr[0].denum;
					prach_descr[0].max_i_fr = L_deposit_h(max_prach_ta) + prach_descr[0].i_interp;
					prach_descr[0].ta_fr = (prach_descr[0].max_i_fr * 3) / 2;
				}
#endif
				N_pre++;
			}
		}
		
#ifdef PRACH_ONLY_ONE_PREAMBLE
		if (max_prach_preamble >= 0)
		{
			prach_value[prach_detected] = max_prach_val;
			prach_preamble[prach_detected] = max_prach_preamble;

			
#if 1				
			INFO(DRACH, "FR start %i stop %i idx=%i, ta_i=%i, %i %i %i, %i %i, %i, fr=%i, ta=%i\n",
					max_prach_start,
					max_prach_stop,
					max_prach_idx,
					max_prach_ta,
					prach_descr[0].y1, prach_descr[0].y2,prach_descr[0]. y3,
					prach_descr[0].num, prach_descr[0].denum,
					prach_descr[0].i_interp,
					prach_descr[0].max_i_fr,
					prach_descr[0].ta_fr);
#endif
			
			if (prach_descr[0].ta_fr < 0)
			{
				/* TA < 0
				 * 
				 * Сейчас обнуляем
				 */
				DBG(DRACH, "Negative PRACH at i=%i, %i %i %i, %i %i, %i, fr=%i, ta=%i\n", max_prach_ta,
						prach_descr[0].y1, prach_descr[0].y2, prach_descr[0].y3,
						prach_descr[0].num, prach_descr[0].denum,
						prach_descr[0].i_interp,
						prach_descr[0].max_i_fr, prach_descr[0].ta_fr);
				prach_descr[0].ta_fr = 0;
			}

			prach_ta[prach_detected] = _round(prach_descr[0].ta_fr);

			/* 
			 * RA-RNTI= 1 + t_id+10*f_id
			 * for FDD f_id = 0
			 */
			prach_rnti[prach_detected] = 1 + rx_prach->subframe_no;
			prach_detected = 1;
			
			/*
			 if (prach_detected >= LTE_PRACH_DETECT_MAX)
				break;
			*/
		}
#endif
		
		LOG_EVENT(LOGEVT_RACH_TASK_EVENT, 8);

		if (prach_detected > 0)
		{
			LOG_EVENT_SF(LOGEVT_RACH_DETECT, rx_prach->frame_no, rx_prach->subframe_no, prach_detected);

			fapi_p7_send_rach_indication(rx_prach->frame_no, rx_prach->subframe_no, prach_detected, 
					prach_rnti, prach_preamble, prach_ta, prach_value);
		}

#ifdef DEBUG_PRACH_DETECT_DDS_ON_DETECT

		if(prach_detected > 0)
		{
			Complex16 *dbg_buf;
			int32_t dbg_sym_no = 0;
			int32_t sym_ptr = 0;

#ifdef DEBUG_PRACH_DDS
			// При включенной записи буфера и включенной записе при обнаружении инкремент производится здесь
			debug_prach_buffer_ptr += 20000;
#endif

			if(debug_prach_buffer_ptr + LTE_SAMPLES_PER_SUBFRAME + 10000 >= DEBUG_PRACH_BUFFER_SIZE)
			{
				os_phys_ptr phys_ptr = 0;
				osMmuDataVirtToPhys(debug_prach_buffer, &phys_ptr);
				INFO(DTRX, "Debug PRACH buffer full, address=0x%08x\n", phys_ptr);
				debug_prach_buffer_ptr = 0;
			}
#if 0
			/* Принятый сабфрейм во временной области */
			dbg_buf = &debug_prach_buffer[debug_prach_buffer_ptr + 0];
			for(i=0; i<LTE_SAMPLES_PER_SUBFRAME; i++)
			{
				dbg_buf[i] = V_swapb2(rx_prach->prach[i]);
			}
#endif							
			/* Децимированный массив , 1024 отсчета*/
			dbg_buf = &debug_prach_buffer[debug_prach_buffer_ptr + 0];
			for(i=0; i<1024; i++)
			{
				dbg_buf[i] = V_swapb2(prach_seq_cplx[i]);
			}

			/* Результат кросс-корреляции */
			dbg_buf = &debug_prach_buffer[debug_prach_buffer_ptr] + 2000;
			for(i=0; i<2048; i++)
			{
				dbg_buf[i] = V_swapb2(prach_seq_fft[i]);
			}
			
			debug_prach_buffer_ptr += 10000;
		}
#endif
		
#ifdef DEBUG_PRACH_DETECT_CORR_PEAKS
		if(prach_detected > 0)
		{
#if 1
			{
				Complex16 *dbg_buf = &debug_prach_buffer[debug_prach_buffer_ptr];
				for (i = 0; i < 2048; i++)
				{
					dbg_buf[i] = V_swapb2(prach_corr_fft[i]);
				}

				sweep_cache((uint32_t) dbg_buf, (2048) << 2, CACHE_FLUSH);

				debug_prach_buffer_ptr += 2048;
				if (debug_prach_buffer_ptr >= DEBUG_PRACH_BUFFER_SIZE)
				debug_prach_buffer_ptr = 0;
			}
#endif

			int32_t *dbg_buf = &debug_prach_buffer[debug_prach_buffer_ptr];
#if 0
			dbg_buf[0] = V_swapb(TO_COMPLEX16(32767, 32767));
			dbg_buf[1] = V_swapb(prach_mean);
			dbg_buf[2] = V_swapb(prach_detected);

			for(i=0; i < prach_detected; i++)
			{
				dbg_buf[4 + 2 * i] = V_swapb(prach_max[i]);
				dbg_buf[4 + 2 * i + 1] = V_swapb(prach_max_idx[i]);
			}
#endif
			int32_t j=0;
			for (i=0; i < 2048; i++)
			{
				dbg_buf[i] = V_swapb(prach_corr[j++]);
			}

			sweep_cache((uint32_t) dbg_buf, (2048) << 2, CACHE_FLUSH);
			
			debug_prach_buffer_ptr += 2048;
			if (debug_prach_buffer_ptr >= DEBUG_PRACH_BUFFER_SIZE)
			debug_prach_buffer_ptr = 0;
		}
#endif

		loop_end = log_get_timer_value();
		trx_cycle_counters[TRX_CYCLE_RX_PRACH][loop_counter] = loop_end - loop_start;
		loop_counter = (loop_counter + 1) % TRX_CYCLE_NUM_LOOPS;

		LOG_EVENT_SF(LOGEVT_RACH_TASK_END, rx_prach->frame_no, rx_prach->subframe_no, 0);
	}
}
