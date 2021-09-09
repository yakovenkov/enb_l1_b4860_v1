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
#include <complex.h>
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

#define PRACH_USE_NEW
#define PRACH_USE_GEN
#define PRACH_ONLY_ONE_PREAMBLE

//#define PUSCH_FTPE_DBG
#define PUSCH_EQPE_DBG

/* Сдвиг DMRS вправо для уменьшения амплитуды */
//#define SHIFT_DMRS

//#define H_INTERP
//#define USE_TEST_DATA

#define COMPLEX_DIV

// Настройки для PUSCH
#define SCALE_MEAN // использовать усреднение входных скейлов
#define EQPE_NVI_INTERNAL // использовать расчет NVI/BETA в EQPE

//#define SCALE_MEAN

#define USE_FAST_MEMCPY_FOR_DEBUG

// SC3900 workaround
#define creal16(x) extract_h(x)
#define cimag16(x) extract_l(x)

extern int32_t sqrt_table[65536] __attribute__((section(".shared_data_ddr0_cacheable_bss"), aligned(16)));
int32_t dB_fixed_times10(uint32_t x);

#ifdef TRX_DEBUG_BUFFER

#define DEBUG_UL_BUFFER_STEP(e) (NUM_OF_SAMPLES1(e) * LTE_NSYMB_PER_SUBFRAME)
#define DEBUG_UL_BUFFER_SIZE (8192*1024)

Complex16 debug_ul_buffer[DEBUG_UL_BUFFER_SIZE] __attribute__((section(".shared_data_ddr0_cacheable_bss"), aligned(16)));

uint32_t debug_ul_buffer_ptr = 0;

#if 0
uint32_t dbg_addr_mark = 0;
#define DBG_MARK_OUT_I { dbg_buf[dbg_addr_mark++] = V_swapb(out_i); }
#else
#define DBG_MARK_OUT_I {}
#endif

#if defined(PUSCH_FTPE_DBG) || defined(PUSCH_EQPE_DBG)
Complex16 pusch_ftpe_eqpe_dbg_buf[12 * 1200 * LTE_N_ANTENNAS_MAX] __attribute__((section(".shared_data_ddr0_cacheable_bss"), aligned(16)));
#endif
#else
#define DBG_MARK_OUT_I	;
#endif

static int32_t pusch_estimate(lte_enodeb_t *enodeb, lte_ul_pusch_user_descr_t *user_descr, int32_t Msc);

#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_MAPLE_OUTPUT && !defined DEBUG_OPT_OFF && !defined(PUSCH_TEST)
#pragma opt_level = "O3"
#else
//#pragma opt_level = "O3"
#endif

// Table 8.6.3-3 36.213
uint32_t beta_cqi_times8[16] = {0,   //reserved
                         0,   //reserved
                         9,   //1.125
                         10,  //1.250
                         11,  //1.375
                         13,  //1.625
                         14,  //1.750
                         16,  //2.000
                         18,  //2.250
                         20,  //2.500
                         23,  //2.875
                         25,  //3.125
                         28,  //3.500
                         32,  //4.000
                         40,  //5.000
                         50
                        }; //6.250

// Table 8.6.3-2 36.213
uint32_t beta_ri_times8[16] = {10,   //1.250
                        13,   //1.625
                        16,   //2.000
                        20,   //2.500
                        25,   //3.125
                        32,   //4.000
                        40,   //5.000
                        50,   //6.250
                        64,   //8.000
                        80,   //10.000
                        101,  //12.625
                        127,  //15.875
                        160,  //20.000
                        0,    //reserved
                        0,    //reserved
                        0
                       };   //reserved

// Table 8.6.3-2 36.213
uint32_t beta_harq_times8[16] = {16,  //2.000
                         20,  //2.500
                         25,  //3.125
                         32,  //4.000
                         40,  //5.000
                         50,  //6.250
                         64,  //8.000
                         80,  //10.000
                         101, //12.625
                         127, //15.875
                         160, //20.000
                         248, //31.000
                         400, //50.000
                         640, //80.000
                         808
                        };//126.00

/* Таблица коэффициентов A порога обнаружения HARQ PUSCH
 * Table 3-98
 */
static int32_t maple_a_table[3][4] = 
		{
				{0x0058c, 0x00594, 0x0058d, 0x00594},
				{0x00ac7, 0x00ac4, 0x00b20, 0x00b1e},
				{0x01025, 0x0101c, 0x010aa, 0x010a9}
		};
/* Таблица коэффициентов B порога обнаружения HARQ PUSCH
 * Table 3-99
 */
static int32_t maple_b_table[3][4] =
		{
				{0x00e31, 0x01171, 0x013b5, 0x0168f},
				{0x00f29, 0x011b6, 0x01414, 0x016ad},
				{0x010ba, 0x01268, 0x013eb, 0x0164f}
		};

static uint32_t cqi_precalc_pusch_wideband[16] =
		{
				// 0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
				0x00000000,
				// 0   0   1   1   1   0   0   1   1   1   0   0   1   1   0   0   0   1   1   0   0   1   0   0   1   0   1   1   0   1   1   0
				0x39cc64b6,
				// 0   1   0   1   1   0   1   0   0   1   1   1   0   0   0   0   1   0   0   0   1   0   0   1   1   0   1   1   1   1   1   0
				0x5a7089be,
				// 0   1   1   0   0   0   1   1   1   0   1   1   1   1   0   0   1   1   1   0   1   1   0   1   0   0   0   0   1   0   0   0
				0x63bced08,
				// 1   1   0   0   1   1   0   0   1   0   0   1   0   1   0   1   1   0   1   0   0   1   0   1   1   1   0   1   0   0   1   0
				0xcc95a5d2,
				// 1   1   1   1   0   1   0   1   0   1   0   1   1   0   0   1   1   1   0   0   0   0   0   1   0   1   1   0   0   1   0   0
				0xf559c164,
				// 1   0   0   1   0   1   1   0   1   1   1   0   0   1   0   1   0   0   1   0   1   1   0   0   0   1   1   0   1   1   0   0
				0x96e52c6c,
				// 1   0   1   0   1   1   1   1   0   0   1   0   1   0   0   1   0   1   0   0   1   0   0   0   1   1   0   1   1   0   1   0
				0xaf2948da,
				// 1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1
				0xffffffff,
				// 1   1   0   0   0   1   1   0   0   0   1   1   0   0   1   1   1   0   0   1   1   0   1   1   0   1   0   0   1   0   0   1
				0xc6339b49,
				// 1   0   1   0   0   1   0   1   1   0   0   0   1   1   1   1   0   1   1   1   0   1   1   0   0   1   0   0   0   0   0   1
				0xa58f7641,
				// 1   0   0   1   1   1   0   0   0   1   0   0   0   0   1   1   0   0   0   1   0   0   1   0   1   1   1   1   0   1   1   1
				0x9c4312f7,
				// 0   0   1   1   0   0   1   1   0   1   1   0   1   0   1   0   0   1   0   1   1   0   1   0   0   0   1   0   1   1   0   1
				0x336a5a2d,
				// 0   0   0   0   1   0   1   0   1   0   1   0   0   1   1   0   0   0   1   1   1   1   1   0   1   0   0   1   1   0   1   1
				0x0aa63e9b,
				// 0   1   1   0   1   0   0   1   0   0   0   1   1   0   1   0   1   1   0   1   0   0   1   1   1   0   0   1   0   0   1   1
				0x691ad393,
				// 0   1   0   1   0   0   0   0   1   1   0   1   0   1   1   0   1   0   1   1   0   1   1   1   0   0   1   0   0   1   0   1
				0x50d6b725
		};

static Complex16 pucch_dmrs[LTE_N_SC_RB * 16] __attribute__((aligned(8)));

static Complex16 ce[LTE_N_SC_RB * 16] __attribute__((aligned(8)));
static Complex16 hest_avg0[LTE_N_SC_RB * 16] __attribute__((aligned(8)));
static Complex16 hest_avg1[LTE_N_SC_RB * 16] __attribute__((aligned(8)));

static Complex16 pucch_sym_tmp[LTE_N_SC_RB * 16] __attribute__((aligned(8)));

static Complex16 pucch_sym[LTE_N_SC_RB * 16] __attribute__((aligned(8)));

static uint32_t cqi_precalc_pucch_wideband[16] __attribute__((aligned(8))) = 
		{
			//  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
				0x00000000,
			//	0   0   1   1   1   0   0   1   1   1   0   0   1   1   0   0   0   1   1   0
				0x00039cc6,
			//	0   1   0   1   1   0   1   0   0   1   1   1   0   0   0   0   1   0   0   0
				0x0005a708,
			//	0   1   1   0   0   0   1   1   1   0   1   1   1   1   0   0   1   1   1   0
				0x00063bce,
			//	1   1   0   0   1   1   0   0   1   0   0   1   0   1   0   1   1   0   1   0
				0x000cc95a,
			//	1   1   1   1   0   1   0   1   0   1   0   1   1   0   0   1   1   1   0   0
				0x000f559c,
			//	1   0   0   1   0   1   1   0   1   1   1   0   0   1   0   1   0   0   1   0
				0x00096e52,
			//	1   0   1   0   1   1   1   1   0   0   1   0   1   0   0   1   0   1   0   0
				0x000af294,
			//	1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1
				0x000fffff,
			//	1   1   0   0   0   1   1   0   0   0   1   1   0   0   1   1   1   0   0   1
				0x000c6339,
			//	1   0   1   0   0   1   0   1   1   0   0   0   1   1   1   1   0   1   1   1
				0x000a58f7,
			//	1   0   0   1   1   1   0   0   0   1   0   0   0   0   1   1   0   0   0   1
				0x0009c431,
			//	1   0   1   0   0   1   0   1   1   0   0   0   1   1   1   1   0   1   1   1
				0x000336a5,
			//	0   0   0   0   1   0   1   0   1   0   1   0   0   1   1   0   0   0   1   1
				0x0000aa63,
			//  0   1   1   0   1   0   0   1   0   0   0   1   1   0   1   0   1   1   0   1
				0x000691ad,
			//	0   1   0   1   0   0   0   0   1   1   0   1   0   1   1   0   1   0   1   1
				0x00050d6b
		};

int32_t lte_trx_is_rach_msg3(lte_enodeb_t *enodeb, lte_ul_subframe_t *sf);
static lte_status_t trx_ul_process_uci_pdu(lte_enodeb_t *enodeb, int32_t ant_no, fapi_ul_config_request_pdu_t *ul_req_pdu, uint32_t srs_present,
		lte_ul_subframe_t *rx_sf, int8_t *scale_vector, fapi_harq_indication_pdu_t *harq_pdu,
		fapi_sr_indication_pdu_t *sr_pdu,
		fapi_cqi_indication_pdu_t *cqi_pdu,
		uint8_t *cqi_raw_data,
		int32_t *sr_detected, uint32_t dump_results,
		uint32_t p8_dump_flags,
		uint32_t *dump_buf, uint32_t *dump_len);

extern cop_dev_handle maple_handle[NUM_MAPLES_USED];
extern cop_dev_handle pdsch_handle;
extern cop_channel_t pdsch_ch_handle[LTE_N_ANTENNAS_MAX];
extern cop_channel_t pusch_ch_handle[LTE_N_ANTENNAS_MAX];
extern cop_channel_t tvpe_ch_handle[NUM_TVPE_USED];

extern volatile uint32_t enodeb_flag_stop;

static lte_status_t trx_maple_pdsch_direct(lte_subframe_t *cur_sf, uint32_t first_run);

/* DSP fucntions */
void sc3850_fir_real_16x16_asm_dec2(Word16 x[], Word16 h[], Word16 y[], Word16 nr, Word16 nh);
//int sc3850_vector_complex_mult_conj_asm(Complex16 *restrict x, Complex16 *restrict y, Complex16 *restrict z, int N);
int sc3850_vector_complex_mult_conj_sc3900(short *restrict input, short *restrict coef, short *restrict result, int N);
os_status maple_fft_ifft(Complex16 *iq_in, Complex16 *iq_out, int32_t len, int32_t inverse);

extern uint32_t dftsizes[33];
extern Complex16 ul_ref_sigs_rx[30][2][33][1200];

static int32_t trx_align_pufft(lte_enodeb_t *enodeb, lte_ul_subframe_t *rx_sf)
{
	int32_t i, ant_no, sym;
	int32_t s3_scale, s10_scale;
	int32_t max_scale = -255;
	
	if(enodeb == NULL || rx_sf == NULL)
		return -1;

	s3_scale = (int8_t)rx_sf->pufft_job[3].status[0];
	s10_scale = (int8_t)rx_sf->pufft_job[10].status[0];
	
	for(i=0; i< LTE_NSYMB_PER_SUBFRAME; i++)
	{
		for(ant_no=0; ant_no<enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
		{
			if(max_scale < (int8_t)rx_sf->pufft_job[i].status[ant_no])
				max_scale = (int8_t)rx_sf->pufft_job[i].status[ant_no];
		}
	}
	
	/* Приведение отсчетов к одному скейлу */
	for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
	{
		for(sym = 0; sym < 12; sym++)
		{
			int32_t pufft_job_no = (sym < 3) ? sym : (sym >=3 && sym < 9 ? sym + 1 : sym + 2);
			Complex16 * restrict a0 = &rx_sf->pufft_buf_a0[(ant_no * 12 + sym) * enodeb->fp.LTE_N_RE];
			Complex16 * restrict a0_aligned = &rx_sf->a0_aligned[(ant_no * 12 + sym) * enodeb->fp.LTE_N_RE];
			for (i = 0; i < enodeb->fp.LTE_N_RE; i++)
				a0_aligned[i] = __ash_rgt_2w((max_scale - ((int8_t)rx_sf->pufft_job[pufft_job_no].status[ant_no])), a0[i]);
		}
	}

	for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
	{
		Complex16 * restrict a0 = &rx_sf->pufft_buf_a0_sym3[ant_no * enodeb->fp.LTE_N_RE];
		Complex16 * restrict a0_aligned = &rx_sf->a0_sym3_aligned[ant_no * enodeb->fp.LTE_N_RE];

		for (i = 0; i < enodeb->fp.LTE_N_RE; i++)
			a0_aligned[i] =	__ash_rgt_2w((max_scale - ((int8_t)rx_sf->pufft_job[3].status[ant_no])), a0[i]);
	}

	for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
	{
		Complex16 * restrict a0 = &rx_sf->pufft_buf_a0_sym10[ant_no * enodeb->fp.LTE_N_RE];
		Complex16 * restrict a0_aligned = &rx_sf->a0_sym10_aligned[ant_no * enodeb->fp.LTE_N_RE];

		for (i = 0; i < enodeb->fp.LTE_N_RE; i++)
			a0_aligned[i] = __ash_rgt_2w((max_scale - ((int8_t)rx_sf->pufft_job[10].status[ant_no])), a0[i]);
	}
	/* Обнуление скейлов */
	for(ant_no=0; ant_no<enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
		for(i=0; i< LTE_NSYMB_PER_SUBFRAME; i++)
			rx_sf->a0_scales_aligned[ant_no][i] = 0;
	
	//INFO(DTRX, "Max scale %i H %i %i\n", max_scale, s3_scale, s10_scale);
	int32_t m_s_scale = (s3_scale < s10_scale) ? s3_scale : s10_scale; 
	return max_scale - m_s_scale;
}

fapi_ul_config_request_body_t *trx_get_ul_config(lte_enodeb_t *enodeb, lte_ul_subframe_t *sf)
{
	fapi_ul_config_request_t *ul_cfg_req;
	int32_t found = 0;

	found = 0;
	
	ul_cfg_req = enodeb->trx.fapi_ul_config[sf->subframe_no];
	enodeb->trx.fapi_ul_config[sf->subframe_no] = NULL;
	
	if(ul_cfg_req == NULL)
		return NULL;
	
	if(ul_cfg_req->header.message_id != FAPI_UL_CONFIG_REQUEST)
	{
		found = 0;
		
		return NULL;
	}
	
	uint32_t req_frame_no = ul_cfg_req->sfn_sf >> 4;
	uint32_t req_subframe_no = ul_cfg_req->sfn_sf & 0x0f;

	if (req_frame_no == sf->frame_no && req_subframe_no == sf->subframe_no)
	{
		/* Нашли нужный пакет */
		found = 1;
	}
	else
	{
		INFO(DTRX, "Got stale RX.ind for %i:%i at %i:%i\n", req_frame_no, req_subframe_no, sf->frame_no, sf->subframe_no);
	}

	if (found)
		return &ul_cfg_req->ul_config_request_body;
	else
		return NULL;

}

lte_ul_pusch_descr_t *trx_get_pusch_descr(lte_enodeb_t *enodeb)
{
	lte_ul_pusch_descr_t *descr;
	uint8_t id;

	osHwiSwiftDisable();

	descr = enodeb->trx.ul_pusch_pool[enodeb->trx.ul_pusch_pool_ptr];
	id = enodeb->trx.ul_pusch_pool_ptr & 0xff;
	enodeb->trx.ul_pusch_pool_ptr = (enodeb->trx.ul_pusch_pool_ptr + 1) & (TRX_NUM_OF_PUSCH_JOBS - 1);

	osHwiSwiftEnable();

	memset(&descr->cop_job, 0, sizeof(cop_job_handle));		
	memset(&descr->pusch_bd, 0, sizeof(maple_pusch_job_t));

	descr->cop_job.job_id = (void *) (id);
	descr->cop_job.device_specific = &descr->pusch_bd;	
	descr->pusch_bd.user_status_ptr = &descr->status;
	descr->pusch_bd.user_param_ptr = &descr->pusch_uph;
	descr->pusch_bd.seg_param_ptr = &descr->pusch_sh;
	descr->pusch_bd.segment_in_done_ptr = &descr->segment_in_done;
	descr->pusch_bd.groups_ptr = &descr->job_ext;
	descr->n_segs = 0;
	descr->n_users = 0;
	descr->n_tvpe = 0;
	descr->decouple_offset = 0;
	
	return descr;
}

/* 1q15 */
int16_t  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) S_matrix_1t1r[] =
	{ 23160, 0, 0, 0 };
		/*
		{
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
				32767, 0, 0, 0, 0, 0, 0, 0,
		};
		*/
	//{ 16380, 16380, 0, 0 };
	//{ 8190, 0, 0, 0 };
	//{ 16380, 16380, 16380, 16380 };

int16_t  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) S_matrix_2t2r[] =
	{ 8180, 8180, 0, 0 };

int32_t __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) dtx_thr_arr[20] =
	{ 132000, 132000, 132000, 132000, 132000,
	132000, 132000, 132000, 132000, 132000,
	132000, 132000, 132000, 132000, 132000,
	132000, 132000, 132000, 132000, 132000
	};

/* Матрица интерполяции характеристики канала Hest
 * Формат - пары [w0 w1], 2q14
 * H = w0 * Hest0 + w1 * Hest1
 * Эти коэффициенты для линейной интерполяции между двумя референсными характеристиками 
 */
/*
int16_t W_matrix[] __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) =
	{ 16382, 0, 14893, 1489, 13404, 2978, 11914, 4468, 10425, 5957, 8936, 7446, 7446, 8936, 5957, 10425, 4468, 11914, 2978, 13404, 1489, 14893, 0, 16382 };
*/

#if 1
int16_t  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) W_matrix[] =
{
		23400/2, -7020/2, 21060/2, -4680/2, 18720/2, -2340/2,
/* Sym3		16380, 0, */ 
		14040/2, 2340/2, 11700/2, 4680/2, 9360/2, 7020/2, 7020/2, 9360/2, 4680/2, 11700/2, 2340/2, 14040/2,
/* Sym10	0, 16380, */
        -2340/2, 18720/2, -4680/2, 21060/2, -7020/2, 23400/2
};
#elif(0)
int16_t  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16), packed)) W_matrix[] =
{
		23400/4, 0, 23400/4, 0, 23400/4, 0,
/* Sym3		16380, 0, */ 
		23400/4, 0, 23400/4, 0, 23400/4, 0,
		0, 23400/4, 0, 23400/4, 0, 23400/4,
/* Sym10	0, 16380, */
        0, 23400/4, 0, 23400/4, 0, 23400/4

};
#elif(1)
int16_t  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) W_matrix[] =
{
		23400, -7020, 21060, -4680, 18720, -2340,
/* Sym3		16380, 0, */ 
		14040, 2340, 11700, 4680, 9360, 7020, 7020, 9360, 4680, 11700, 2340, 14040,
/* Sym10	0, 16380, */
        -2340, 18720, -4680, 21060, -7020, 23400
};
#else
int16_t  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16), packed)) W_matrix[] =
{
		1,1,1,1,1,1,
/* Sym3		16380, 0, */ 
		1,1,1,1,1,1, 
		1,1,1,1,1,1,
/* Sym10	0, 16380, */
		1,1,1,1,1,1,
};
#endif

int16_t  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) C_matrix[2048] = {0};

Complex16  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) h_interp[12][25*12] = {0};

/*
int16_t W_matrix[] __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) =
	{ 16382, 0, 16382, 0, 16382, 0, 16382, 0, 16382, 0, 16382, 0, 0, 16382, 0, 16382, 0, 16382, 0, 16382, 0, 16382, 0, 16382 };
*/
static int8_t  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) SCALE_vector[128 * 4];

static int8_t  __attribute__((aligned(16))) pucch_scale_vector[14] = {0};

#if 0
uint16_t nvi_beta[] =
{
	23170, 23170, 23170, 23170, 23170, 23170, 23170, 23170,
	23170, 23170, 23170, 23170, 23170, 23170, 23170, 23170,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	/* NVI */
	0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff,
	0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff, 0x7fff,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
};

#pragma align nvi_beta 16
#endif


void sc3850_complex_div_16x16_fc(Complex16 *restrict in, Complex16 *restrict H, Complex16 *restrict out, int equalizedsbc)
{
	int j;
	
	for(j=0; j<equalizedsbc; j++)
	{
		void __fix2flt_l_2sp(Word32 __Da, Word32 __Db, Word32 __Dc, float*	__Dd, float* __De);
		//void __flt2fix_sp_2x(Word32 __Da, float __Db, float __Dc, Word40* __Dd, Word40* __De);
		//float __fix2flt_l_sp(Word32 __Da, Word32 __Db)
		Word32 __flt2fix_sp_l(Word32 __Da, float __Db);
		
		float i_re, i_im;
		float h_re, h_im;
		
		__fix2flt_l_2sp(0, creal16(in[j]), cimag16(in[j]), &i_re, &i_im);
		__fix2flt_l_2sp(0, creal16(H[j]), cimag16(H[j]), &h_re, &h_im);
		
		complex float c_i, c_h, c_o;
		
		c_i = i_re + 1*I * i_im;
		c_h = h_re + 1*I * h_im;
		
		c_o = c_i / c_h;
		
		Complex16 o;
		
		o = TO_COMPLEX16(__flt2fix_sp_l(15, crealf(c_o)), __flt2fix_sp_l(15, cimagf(c_o)));
		out[j] = o;
	}
}

int32_t invalid_descr = 0;
void trx_init_debug_buffer()
{
	int32_t i;
	debug_ul_buffer_ptr = 0;
	
	for(i = 0; i < DEBUG_UL_BUFFER_SIZE; i++)
		debug_ul_buffer[i] = 0;
}

int32_t cpri_emulate_interrupt_tx(lte_enodeb_t *enodeb);

#ifdef SHIFT_DMRS
Complex16 t_dmrs0[1200] __attribute__((section(".shared_data_ddr0_cacheable_bss"), aligned(16)));
Complex16 t_dmrs1[1200] __attribute__((section(".shared_data_ddr0_cacheable_bss"), aligned(16)));
#endif

void trx_ul_task(uint32_t p)
{
#if !defined(DEBUG_OPT_OFF)
#pragma opt_level = "O3"
#endif
	
	os_status status;
	lte_trx_t *trx = (lte_trx_t *)p;
	lte_enodeb_t *enodeb = trx->enodeb;
	lte_ul_subframe_t *rx_sf;
	uint32_t first_run = 1;
	int32_t i;
	/*
	 uint32_t u0, u1;
	 uint32_t v0, v1;
	 */
	uint32_t Msc_RS = 12;
	uint32_t Msc_RS_idx;
	uint32_t first_prb;
	uint32_t num_prb;
	fapi_ul_config_request_body_t *ul_cfg_req;
	int32_t n_req;
	lte_ul_pusch_descr_t *pusch_descr, *prev_pusch_descr;
	Complex16 *dmrs0, *dmrs1;
	uint32_t modulation;
	int32_t mod_idx;
	uint32_t rv_idx;
	static uint32_t pucch_dump_results = 0;
	//static uint32_t pusch_dump_results = 0;
	lte_ul_pusch_descr_t *pusch_descr_done;
	int32_t ant_rx;
	int32_t ant_no;
	
	/* Идентификатора безопасности */
	secure_id_t sec_id = get_secure_id();
	int32_t sec_time_count;
	uint32_t p8_dump_flags = 0;
	int32_t p8_limit;
	uint32_t ul_has_bad_crc = 0;
	uint32_t ul_has_cqi = 0;
	uint32_t ul_has_ack = 0;
	uint32_t ul_has_nack = 0;
	
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
			osEventQueueReset(trx->evq_pusch_ready, 0);
			osEventQueueReset(trx->evq_tvpe_ready, 0);
			osEventQueueReset(trx->evq_pusch_ready, 0);
			
			trx->harq_tti_counter = 0;
			
			sec_id = get_secure_id();
			
			/* Лимит сообщений P8 */
			p8_limit = g_fapi_ipc_cfg.cfg_v1.p8_dump_limit;
			if(p8_limit == 0)
				p8_limit = INT32_MAX;
			
			//pusch_dump_results = 0;
			first_run = 0;
		}

		/* Ожидание сабфрейма */
		status = osEventQueuePend(trx->evq_ul_pufft_ready, (uint32_t *) &rx_sf, 2);

		if (status == OS_ERR_EVENT_QUEUE_TIMEOUT)
		{
			/* Если произошел таймаут - перезапуск внешнего цикла,
			 * т.к. могло измениться изменение состояния работы L1
			 */
			continue;
		}

		OS_ASSERT_COND(status == OS_SUCCESS);	
		LOG_EVENT_SF(LOGEVT_UL_TASK_BEGIN, rx_sf->frame_no, rx_sf->subframe_no, 0);
		//INFO(DTRX, "UL task %i:%i\n", rx_sf->frame_no, rx_sf->subframe_no);
		
		/* Контроль когерентности */
		DBAR_IBLL();
		
		// Увеличение таймера TTI для отслеживания времени жизни HARQ-процессов
		trx->harq_tti_counter++;
		
#if 0
		/* Slot 0 */
		u0 = enodeb_inst.refsigs->grouphop_pusch[0 + (rx_sf->subframe_no << 1)];
		v0 = enodeb_inst.refsigs->seqhop_pusch[0 + (rx_sf->subframe_no << 1)];

		/* Slot 1 */
		u1 = enodeb_inst.refsigs->grouphop_pusch[1 + (rx_sf->subframe_no << 1)];
		v1 = enodeb_inst.refsigs->seqhop_pusch[1 + (rx_sf->subframe_no << 1)];
#endif

		ul_cfg_req = trx_get_ul_config(enodeb, rx_sf);

		if (ul_cfg_req == NULL)// || ul_cfg_req->number_of_pdus == 0)
		{
			/* Это нормальная ситуация,
			 * просто нет запросов ULSCH для этого сбафреймов 
			 */
			LOG_EVENT_SF(LOGEVT_UL_TASK_END, rx_sf->frame_no, rx_sf->subframe_no, 0);
			
			continue;
		}
		
		//INFO(DTRX, "UL task %i:%i npdu %i\n", rx_sf->frame_no, rx_sf->subframe_no, ul_cfg_req->number_of_pdus);
		
		sweep_cache((uint32_t)rx_sf, sizeof(lte_ul_subframe_t), CACHE_FLUSH);
		
		p8_dump_flags = 0;
		ul_has_bad_crc = 0;
		ul_has_ack = 0;
		ul_has_nack = 0;
		ul_has_cqi = 0;

		prev_pusch_descr = NULL;
		pusch_descr_done = NULL;
		//pusch_dump_results = 0;
		pucch_dump_results = 0;

		fapi_rx_indication_t *rx_ind_msg = NULL;
		fapi_harq_indication_t *harq_ind_msg = NULL;
		fapi_cqi_indication_t *cqi_ind_msg = NULL;
		fapi_crc_indication_t *crc_ind_msg = NULL;
		fapi_sr_indication_t *sr_ind_msg = NULL;
		
		fapi_ipc_msg_t *harq_ind_ipc_msg = NULL; 
		fapi_ipc_msg_t *sr_ind_ipc_msg = NULL;
		fapi_ipc_msg_t *cqi_ind_ipc_msg = NULL;
		fapi_ipc_msg_t *crc_ind_ipc_msg = NULL;
		fapi_ipc_msg_t *rx_ind_ipc_msg = fapi_alloc_send_msg(FAPI_CHANNEL_P7_IND, &rx_ind_msg);

		rx_ind_msg->frame = rx_sf->frame_no;
		rx_ind_msg->subframe = rx_sf->subframe_no;
		rx_ind_msg->header.message_id = FAPI_RX_ULSCH_INDICATION;
		rx_ind_msg->rx_indication_body.number_of_pdus = 0;
		
		harq_ind_ipc_msg = fapi_alloc_send_msg(FAPI_CHANNEL_P7_IND, &harq_ind_msg);
		harq_ind_msg->frame = rx_sf->frame_no;
		harq_ind_msg->subframe = rx_sf->subframe_no;
		harq_ind_msg->harq_indication_body.number_of_harqs = 0;
		
		sr_ind_ipc_msg = fapi_alloc_send_msg(FAPI_CHANNEL_P7_IND, &sr_ind_msg);
		sr_ind_msg->frame = rx_sf->frame_no;
		sr_ind_msg->subframe = rx_sf->subframe_no;
		sr_ind_msg->sr_indication_body.number_of_srs = 0;
		
		cqi_ind_ipc_msg = fapi_alloc_send_msg(FAPI_CHANNEL_P7_IND, &cqi_ind_msg);
		cqi_ind_msg->frame = rx_sf->frame_no;
		cqi_ind_msg->subframe = rx_sf->subframe_no;
		cqi_ind_msg->cqi_indication_body.number_of_cqis = 0;

		crc_ind_ipc_msg = fapi_alloc_send_msg(FAPI_CHANNEL_P7_IND, &crc_ind_msg);
		crc_ind_msg->frame = rx_sf->frame_no;
		crc_ind_msg->subframe = rx_sf->subframe_no;
		crc_ind_msg->crc_indication_body.number_of_crcs = 0;
		
		/* Обработка запроса P8 и подготовка P8.ind */
		fapi_p8_indication_t *p8_ind = NULL;
		fapi_ipc_msg_t *p8_ind_ipc_msg = NULL;
		uint32_t p8_data_offset = 0;
		
		/* Установка флагов дампа из конфигурации IPC */
		p8_dump_flags = g_fapi_ipc_cfg.cfg_v1.p8_dump_flags;
		
		/* Сброс флагов, доступных только для SEC_ID_FULL */ 
		if(sec_id != SECURE_ID_FULL)
			p8_dump_flags = p8_dump_flags & 0x0000ffff;
		
		/* Подготовка сообщения P8.ind на случай если будет отправка */ 
		if (p8_dump_flags != 0)
		{
			p8_ind_ipc_msg = fapi_alloc_p8_ind_msg(rx_sf->subframe_no, &p8_ind);
			
			p8_ind->header.message_id = FAPI_P8_INDICATION;
			p8_ind->frame = rx_sf->frame_no;
			p8_ind->subframe = rx_sf->subframe_no;
			p8_ind->number_of_pdus = 0;
			p8_data_offset = 0;
		}
		else
		{
			p8_limit = 0;
			p8_ind = NULL;
		}

		uint32_t msg_len_aligned = ALIGN_SIZE(FAPI_GET_MSG_PTR_VAR_SIZE(rx_ind_msg, rx_indication_body.rx_pdu_list, rx_ind_msg->rx_indication_body.number_of_pdus), 64);
		uint32_t rx_ind_data_ptr = 0;
		//uint8_t *rx_ind_data_buf = (uint8_t *) rx_ind_msg + msg_len_aligned;
		uint8_t *rx_ind_data_buf = fapi_alloc_rx_data_buf();
		
		//uint32_t num_jobs_prepared = 0;
		int32_t has_harq = 0;
		int32_t has_sr = 0;
		int32_t has_ri = 0;
		int32_t has_cqi = 0;
		fapi_ul_config_ulsch_pdu *ulsch_pdu;
		fapi_ul_config_ulsch_harq_information *ulsch_harq_info;
		fapi_ul_config_cqi_ri_information *ulsch_cqi_ri_info;
		fapi_ul_config_initial_transmission_parameters *ulsch_initial_tx_params;
		uint8_t *cqi_raw_data;
		uint32_t cqi_raw_data_off;
		
		ant_rx = enodeb->fp.LTE_N_PHYS_ANTENNAS_RX;
		
		if(sec_id == SECURE_ID_FULL)
		{
			/* Нормальная работа */
		}
		else
		{
			if(sec_id == SECURE_ID_EVAL)
			{
				/* Ограничение на количество антенн */
				if(ant_rx > 2)
					ant_rx = 2;
			}
			else if(sec_id == SECURE_ID_TIME)
			{
				/* Ограничение по времени работы */
				if(sec_time_count > 0)
				{
					sec_time_count--;
				}
				else
				{
					ul_cfg_req->number_of_pdus = 0;
				}
				
				/* Ограничение на количество антенн */
				if(ant_rx > 1)
					ant_rx = 1;
			}
		}
		
		int32_t align_scale = trx_align_pufft(enodeb, rx_sf);
		
		// При выравнивании скейлов, они все равны 0, приведение делать не надо
		//liblte_calc_pucch_scale_vector(rx_sf, pucch_scale_vector);
		
#define LLR_ABS 16
#define S_SCALE_ABS 16
		
		static int32_t llr_check = -LLR_ABS;
		static int32_t s_scale = -S_SCALE_ABS;
		
		if(p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_LLR_CHECK)
		{
			/* Проверка LLR_ALIGN */
			if(rx_sf->subframe_no == 0)
			{
				llr_check++;
				if(llr_check > LLR_ABS)
				{
					llr_check = -LLR_ABS;
					s_scale++;
					if(s_scale > S_SCALE_ABS)
						s_scale = -S_SCALE_ABS;
				}
		
				INFO(DTRX, "llr_check %i s_scale %i\n", llr_check, s_scale);
			}
		}
		
		for(i=0; i<ant_rx; i++)
		{
			/* Работало */
#ifdef WORK1
			SCALE_vector[ant_rx*16 + i] -= scale_mean - 3;
			SCALE_vector[ant_rx*17 + i] -= scale_mean - 3;
#endif
			
			SCALE_vector[ant_rx*16 + i] = 0; //scale_mean; // - 1;
			SCALE_vector[ant_rx*17 + i] = 0; //scale_mean; // - 1;

			//SCALE_vector[ant_rx*16 + i] = 1;
			//SCALE_vector[ant_rx*17 + i] = 1;

			SCALE_vector[ant_rx * 0 + i] = 0;//(int8_t) (rx_sf->pufft_job[0].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 1 + i] = 0;//(int8_t) (rx_sf->pufft_job[1].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 2 + i] = 0;//(int8_t) (rx_sf->pufft_job[2].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 3 + i] = 0;//(int8_t) (rx_sf->pufft_job[4].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 4 + i] = 0;//(int8_t) (rx_sf->pufft_job[5].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 5 + i] = 0;//(int8_t) (rx_sf->pufft_job[6].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 6 + i] = 0;//(int8_t) (rx_sf->pufft_job[7].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 7 + i] = 0;//(int8_t) (rx_sf->pufft_job[8].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 8 + i] = 0;//(int8_t) (rx_sf->pufft_job[9].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 9 + i] = 0;//(int8_t) (rx_sf->pufft_job[11].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 10 + i] = 0;//(int8_t) (rx_sf->pufft_job[12].status[i] & 0xff) - scale_mean;
			SCALE_vector[ant_rx * 11 + i] = 0;//(int8_t) (rx_sf->pufft_job[13].status[i] & 0xff) - scale_mean;
			
			/* S_SCALE, смещение 2 (адрес = 16*S_SCALE_OFFSET */
			if(p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_LLR_CHECK)
			{
				SCALE_vector[ant_rx * 32 + i] = (int8_t)s_scale;
				SCALE_vector[ant_rx * 33 + i] = (int8_t)s_scale;
				
				if(ul_cfg_req->number_of_pdus > 0)
				{
					INFO(DTRX, "scales: %i %i %i %i %i %i %i %i %i %i %i %i H %i %i S %i A %i\n",
						SCALE_vector[ant_rx * 0 + i], SCALE_vector[ant_rx * 1 + i], SCALE_vector[ant_rx * 2 + i],
						SCALE_vector[ant_rx * 3 + i], SCALE_vector[ant_rx * 4 + i], SCALE_vector[ant_rx * 5 + i],
						SCALE_vector[ant_rx * 6 + i], SCALE_vector[ant_rx * 7 + i], SCALE_vector[ant_rx * 8 + i],
						SCALE_vector[ant_rx * 9 + i], SCALE_vector[ant_rx * 10 + i], SCALE_vector[ant_rx * 11 + i],
						SCALE_vector[ant_rx * 16 + i], SCALE_vector[ant_rx * 17 + i],
						SCALE_vector[ant_rx * 32 + i],
						align_scale);
				}
			}
			else
			{
				SCALE_vector[ant_rx * 32 + i] = 7 + align_scale;
				//SCALE_vector[ant_rx * 32 + i] = 0;
				//SCALE_vector[ant_rx * 32 + i] = align_scale;
				
				/* Значения S_scale при использовании S_COL == 1 */
				/*
				SCALE_vector[ant_rx * 33 + i] = 0 + SCALE_vector[ant_rx * 1 + i];
				SCALE_vector[ant_rx * 34 + i] = 0 + SCALE_vector[ant_rx * 2 + i];
				SCALE_vector[ant_rx * 35 + i] = 0 + SCALE_vector[ant_rx * 3 + i];
				SCALE_vector[ant_rx * 36 + i] = 0 + SCALE_vector[ant_rx * 4 + i];
				SCALE_vector[ant_rx * 37 + i] = 0 + SCALE_vector[ant_rx * 5 + i];
				SCALE_vector[ant_rx * 38 + i] = 0 + SCALE_vector[ant_rx * 6 + i];
				SCALE_vector[ant_rx * 39 + i] = 0 + SCALE_vector[ant_rx * 7 + i];
				SCALE_vector[ant_rx * 40 + i] = 0 + SCALE_vector[ant_rx * 8 + i];
				SCALE_vector[ant_rx * 41 + i] = 0 + SCALE_vector[ant_rx * 9 + i];
				SCALE_vector[ant_rx * 42 + i] = 0 + SCALE_vector[ant_rx * 10 + i];
				SCALE_vector[ant_rx * 43 + i] = 0 + SCALE_vector[ant_rx * 11 + i];
				*/
				
				if(ul_cfg_req->number_of_pdus > 0)
				{
					/*
					INFO(DTRX, "scales: %i %i %i %i %i %i %i %i %i %i %i %i H %i %i S %i A %i\n",
						SCALE_vector[ant_rx * 0 + i], SCALE_vector[ant_rx * 1 + i], SCALE_vector[ant_rx * 2 + i],
						SCALE_vector[ant_rx * 3 + i], SCALE_vector[ant_rx * 4 + i], SCALE_vector[ant_rx * 5 + i],
						SCALE_vector[ant_rx * 6 + i], SCALE_vector[ant_rx * 7 + i], SCALE_vector[ant_rx * 8 + i],
						SCALE_vector[ant_rx * 9 + i], SCALE_vector[ant_rx * 10 + i], SCALE_vector[ant_rx * 11 + i],
						SCALE_vector[ant_rx * 16 + i], SCALE_vector[ant_rx * 17 + i],
						SCALE_vector[ant_rx * 32 + i],
						align_scale);
					*/
					DBG(DTRX, "scales: S %i A %i\n", SCALE_vector[ant_rx * 32 + i], align_scale);
				}
			}
		}

		/**
		 * Обработка запросов UL_CONFIG.req PUCCH
		 */
		/* Смещение CQI RAW PDU от начала структуры (поля Number of PDUs) */
		cqi_raw_data_off = ALIGN_SIZE(sizeof(fapi_cqi_indication_pdu_t)*ul_cfg_req->number_of_pdus + 4, ARCH_CACHE_LINE_SIZE);  
		
		for (n_req = 0; n_req < ul_cfg_req->number_of_pdus; n_req++)
		{
			fapi_ul_config_request_pdu_t *fapi_pdu = &ul_cfg_req->ul_config_pdu_list[n_req];
			
			/* Обнуление флагов передачи UCI */
			has_harq = 0;
			has_sr = 0;
			has_ri = 0;
			has_cqi = 0;
			ulsch_pdu = NULL;
			ulsch_harq_info = NULL;
			ulsch_initial_tx_params = NULL;
			uint32_t *dump_buf, dump_buf_len;
			fapi_p8_ind_pdu_t *p8_ind_pdu;

			pucch_dump_results = 0;
			dump_buf_len = 0;
			dump_buf = NULL;
						
			if(ul_cfg_req->ul_config_pdu_list[n_req].pdu_type == FAPI_UL_CONFIG_UCI_HARQ_PDU_TYPE ||
					ul_cfg_req->ul_config_pdu_list[n_req].pdu_type == FAPI_UL_CONFIG_UCI_SR_PDU_TYPE ||
					ul_cfg_req->ul_config_pdu_list[n_req].pdu_type == FAPI_UL_CONFIG_UCI_SR_HARQ_PDU_TYPE ||
					ul_cfg_req->ul_config_pdu_list[n_req].pdu_type == FAPI_UL_CONFIG_UCI_CQI_PDU_TYPE
			)
			{
				// PUCCH processing
				switch(ul_cfg_req->ul_config_pdu_list[n_req].pdu_type)
				{
					case FAPI_UL_CONFIG_UCI_HARQ_PDU_TYPE:
						has_harq = 1;
						break;
						
					case FAPI_UL_CONFIG_UCI_SR_PDU_TYPE:
						has_sr = 1;
						break;
						
					case FAPI_UL_CONFIG_UCI_SR_HARQ_PDU_TYPE:
						has_harq = 1;
						has_sr = 1;
						break;
						
					case FAPI_UL_CONFIG_UCI_CQI_PDU_TYPE:
						has_cqi = 1;
						break;
						
					default:
				}
				
				fapi_harq_indication_pdu_t *harq_pdu = NULL;
				fapi_sr_indication_pdu_t *sr_pdu = NULL;
				fapi_cqi_indication_pdu_t *cqi_pdu = NULL;
				
				if(has_harq)
				{	
					harq_pdu = &harq_ind_msg->harq_indication_body.harq_pdu_list[harq_ind_msg->harq_indication_body.number_of_harqs++];
				}
				
				if(has_sr)
				{
					sr_pdu = &sr_ind_msg->sr_indication_body.sr_pdu_list[sr_ind_msg->sr_indication_body.number_of_srs];
				}
				
				if(has_cqi)
				{
					cqi_pdu = &cqi_ind_msg->cqi_indication_body.cqi_pdu_list[cqi_ind_msg->cqi_indication_body.number_of_cqis++];
				}
				
				int32_t sr_detected = 0;
				
				//pucch_dump_results = (has_cqi) ? 1 : 0;
				
				pucch_dump_results = 0;
				
				if (sec_id == SECURE_ID_FULL && 
						p8_ind != NULL && 
						((p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUCCH)) &&
						(p8_limit > 0)
					)
				{
					p8_ind_pdu = &p8_ind->pdus[p8_ind->number_of_pdus];
					p8_ind_pdu->offset = p8_data_offset;						
					p8_ind_pdu->type = FAPI_P8_DUMP_FLAG_UL_PUCCH;
					
					dump_buf = (uint32_t *)&p8_ind->pdu_data[p8_data_offset];
					
					uint32_t *tbuf_ul_req = (uint32_t *)fapi_pdu;
				
					if(tbuf_ul_req)
					{
						for(i=0; i<(sizeof(fapi_ul_config_request_pdu_t) + 3) / 4; i++)
							dump_buf[i] = tbuf_ul_req[i];
					}
					
					dump_buf_len += (sizeof(fapi_ul_config_request_pdu_t) + 3) / 4;
					
					pucch_dump_results = 1;
				}

				/* Смещение данных CQI */
				cqi_raw_data = (uint8_t*)(&cqi_ind_msg->cqi_indication_body) + cqi_raw_data_off;
				
				trx_ul_process_uci_pdu(enodeb, ant_rx, &ul_cfg_req->ul_config_pdu_list[n_req], ul_cfg_req->srs_present, rx_sf,
						pucch_scale_vector, harq_pdu, sr_pdu, cqi_pdu, cqi_raw_data, 
						&sr_detected, pucch_dump_results, p8_dump_flags, dump_buf, &dump_buf_len);
				
				if(cqi_pdu != NULL)
				{
					cqi_pdu->cqi_indication_rel8.data_offset = cqi_raw_data_off;
					
					// Расчет нового значения cqi_raw_data
					cqi_raw_data_off += cqi_pdu->cqi_indication_rel8.length;
				}
				
				if(has_sr && sr_detected)
				{	
					sr_ind_msg->sr_indication_body.number_of_srs++;
				}
				
				if(pucch_dump_results && dump_buf_len > 0)
				{
					p8_ind_pdu->length = ALIGN_SIZE(dump_buf_len * 4, ARCH_CACHE_LINE_SIZE);
					p8_data_offset += ALIGN_SIZE(p8_data_offset + p8_ind_pdu->length, ARCH_CACHE_LINE_SIZE);
					p8_ind->number_of_pdus++;
				}
			}
			else if(ul_cfg_req->ul_config_pdu_list[n_req].pdu_type > FAPI_UL_CONFIG_ULSCH_CQI_HARQ_RI_PDU_TYPE)
			{
				ERROR(DTRX, "Possible invalid ULSCH PDU type %i\n", ul_cfg_req->ul_config_pdu_list[n_req].pdu_type);
			}
		}
		
		/**
		 * Обработка запросов UL_CONFIG.req PUSCH с помощью MAPLE-B3
		 */
		
		// Выбор дескриптора PUSCH BD (1 дескриптор на 1 сектор)
		pusch_descr = trx_get_pusch_descr(enodeb);

		pusch_descr->frame = rx_sf->frame_no;
		pusch_descr->subframe = rx_sf->subframe_no;
		
		for (n_req = 0; n_req < ul_cfg_req->number_of_pdus; n_req++)
		{
			/* Обнуление флагов передачи UCI */
			has_harq = 0;
			has_sr = 0;
			has_ri = 0;
			has_cqi = 0;
			ulsch_pdu = NULL;
			ulsch_harq_info = NULL;
			ulsch_initial_tx_params = NULL;
			
			fapi_ul_config_request_pdu_t *ul_config_req_pdu = &ul_cfg_req->ul_config_pdu_list[n_req];
			
			/*
			INFO(DTRX, "Got UL_CONFIG.ulsch pdu type %i for %i:%i\n", ul_cfg_req->ul_config_pdu_list[n_req].pdu_type, 
					pusch_descr->frame, pusch_descr->subframe);
			*/
			if (ul_cfg_req->ul_config_pdu_list[n_req].pdu_type == FAPI_UL_CONFIG_ULSCH_PDU_TYPE)
			{
				ulsch_pdu = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_pdu; 
			}
			else if (ul_cfg_req->ul_config_pdu_list[n_req].pdu_type == FAPI_UL_CONFIG_ULSCH_HARQ_PDU_TYPE)
			{
				ulsch_pdu = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_harq_pdu.ulsch_pdu;
				ulsch_harq_info = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_harq_pdu.harq_information;
				ulsch_initial_tx_params = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_harq_pdu.initial_transmission_parameters;
				has_harq = 1;
			}
			else if(ul_cfg_req->ul_config_pdu_list[n_req].pdu_type == FAPI_UL_CONFIG_ULSCH_CQI_RI_PDU_TYPE)
			{
				ulsch_pdu = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_cqi_ri_pdu.ulsch_pdu;
				ulsch_cqi_ri_info = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_cqi_ri_pdu.cqi_ri_information;
				ulsch_initial_tx_params = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_cqi_ri_pdu.initial_transmission_parameters;
				has_cqi = 1;
			}
			else if (ul_cfg_req->ul_config_pdu_list[n_req].pdu_type == FAPI_UL_CONFIG_ULSCH_CQI_HARQ_RI_PDU_TYPE)
			{
				ulsch_pdu = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_cqi_harq_ri_pdu.ulsch_pdu;
				ulsch_cqi_ri_info = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_cqi_harq_ri_pdu.cqi_ri_information;
				ulsch_harq_info = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_cqi_harq_ri_pdu.harq_information;
				ulsch_initial_tx_params = &ul_cfg_req->ul_config_pdu_list[n_req].ulsch_cqi_harq_ri_pdu.initial_transmission_parameters;
				has_cqi = 1;
				has_harq = 1;
			}
			else
			{
				// Non-ULSCH PDU, skip it				
				pucch_dump_results = 0;
				continue;
			}
			
			INFO(DTRX, "Processing UL_CONFIG.ulsch for %i:%i\n", pusch_descr->frame, pusch_descr->subframe);
			
			/* Проверки индексов пользователей и сегментов */
			if(pusch_descr->n_users >= TRX_NUM_OF_PUSCH_UPH)
			{
				ERROR(DTRX, "Maximum no %i of ULSCH users reached, skipping user\n", TRX_NUM_OF_PUSCH_UPH);
				continue;
			}

			if(pusch_descr->n_segs >= TRX_NUM_OF_PUSCH_SH)
			{
				ERROR(DTRX, "Maximum no %i of ULSCH segments reached, skipping user\n", TRX_NUM_OF_PUSCH_SH);
				continue;
			}

			/*
			 * Ниже идет заполнение дескрипторов MAPLE-B3 (UPH и SH)
			 */
			lte_ul_pusch_user_descr_t *user_descr = &pusch_descr->users[pusch_descr->n_users];
			memset(user_descr, 0, sizeof(lte_ul_pusch_user_descr_t));
			
			user_descr->index = pusch_descr->n_users;
			pusch_descr->n_users++;
			
			pusch_descr->status[user_descr->index].tb0_crc = 1;
			pusch_descr->status[user_descr->index].tb1_crc = 1;
			user_descr->has_harq = 0;
			user_descr->has_ri = 0;
			user_descr->has_cqi = 0;
			user_descr->ul_config_req_pdu = ul_config_req_pdu;
			
			maple_pusch_uph_t *uph = &pusch_descr->pusch_uph[user_descr->index];
			memset(uph, 0, sizeof(maple_pusch_uph_t));

			// ULSCH PDU
			first_prb = ulsch_pdu->ulsch_pdu_rel8.resource_block_start;
			num_prb = ulsch_pdu->ulsch_pdu_rel8.number_of_resource_blocks;
			user_descr->num_prb = num_prb;

			/* Сброс признака валидности данных ULSCH в RX_ULSCH.indication*/
			// Old L2
			//rx_ind_msg->rx_indication_body.rx_pdu_list[user_descr->index].rx_indication_rel8.offset = 0;

			/* Определение типа и индекса модуляции PUSCH */
			mod_idx = -1;
			switch (ulsch_pdu->ulsch_pdu_rel8.modulation_type)
			{
				case 2: /* QPSK */
					modulation = PUSCH_USER_HEADER_CW_MODE_QPSK;
					mod_idx = 0;
					break;
				case 4: /* QAM16*/
					modulation = PUSCH_USER_HEADER_CW_MODE_16QAM;
					mod_idx = 1;
					break;
				case 6: /*QAM64 */
					modulation = PUSCH_USER_HEADER_CW_MODE_64QAM;
					mod_idx = 2;
					break;

				default:
					ERROR(DTRX, "Invalid PUSCH modulation: %i\n", ulsch_pdu->ulsch_pdu_rel8.modulation_type);
					break;
			}

			if (mod_idx < 0)
			{
				/* Неизвестный вид модуляции, переход к следующему PDU */
				continue;
			}

			switch (ulsch_pdu->ulsch_pdu_rel8.redundancy_version)
			{
				case 0:
					rv_idx = PUSCH_USER_HEADER_RV_IDX_0;
					break;
				case 1:
					rv_idx = PUSCH_USER_HEADER_RV_IDX_1;
					break;
				case 2:
					rv_idx = PUSCH_USER_HEADER_RV_IDX_2;
					break;
				case 3:
					rv_idx = PUSCH_USER_HEADER_RV_IDX_3;
					break;
			}
			
			user_descr->rnti = ulsch_pdu->ulsch_pdu_rel8.rnti;
			user_descr->handle = ulsch_pdu->ulsch_pdu_rel8.handle;
			user_descr->rballoc = (ulsch_pdu->ulsch_pdu_rel8.resource_block_start << 16)
					| ulsch_pdu->ulsch_pdu_rel8.number_of_resource_blocks;
			
			user_descr->h_pid = ulsch_pdu->ulsch_pdu_rel8.harq_process_number;

			Msc_RS_idx = liblte_get_dft_by_nrb(num_prb);

			dmrs0 = enodeb->refsigs->pusch_dmrs0[rx_sf->subframe_no][Msc_RS_idx];
			dmrs1 = enodeb->refsigs->pusch_dmrs1[rx_sf->subframe_no][Msc_RS_idx];
			
			user_descr->dmrs0 = dmrs0;
			user_descr->dmrs1 = dmrs1;
			
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{

				/* Расчет зарактеристики канала Hest для двух референсных символов */
				user_descr->ref_symb0[ant_no] = &rx_sf->a0_sym3_aligned[ant_no * enodeb->fp.LTE_N_RE_UL + first_prb * 12];
				user_descr->ref_symb1[ant_no] = &rx_sf->a0_sym10_aligned[ant_no * enodeb->fp.LTE_N_RE_UL + first_prb * 12];
			
#ifdef USE_TEST_DATA
				/* Тестовые DMRS */
				for(i=0; i<num_prb*12; i++)
				{
					dmrs0[i] = TO_COMPLEX16(32767, 0);
					dmrs1[i] = TO_COMPLEX16(32767, 0);
					SCALE_vector[16] = 16;
					SCALE_vector[17] = 16;
					SCALE_vector[32] = 0;
				}
#endif
				
				/* Для вычилсения Hest можно использовать только умножение числителя, т.к. знаменатель == 1 */	
#ifdef COMPLEX_DIV_
				//sc3850_complex_div_16x16_c(user_descr->ref_symb0[ant_no], dmrs0, &user_descr->d.hest0[ant_no * num_prb * 12], num_prb * 12);
				//sc3850_complex_div_16x16_c(user_descr->ref_symb1[ant_no], dmrs1, &user_descr->d.hest1[ant_no * num_prb * 12], num_prb * 12);
				
				//Complex16 t_hest0[300], t_hest1[300];
				sc3850_complex_div_16x16_fc(user_descr->ref_symb0[ant_no], dmrs0, &user_descr->d.hest0[ant_no * num_prb * 12], num_prb * 12);
				sc3850_complex_div_16x16_fc(user_descr->ref_symb1[ant_no], dmrs1, &user_descr->d.hest1[ant_no * num_prb * 12], num_prb * 12);
				/*
				sc3850_complex_div_16x16_c(dmrs0, user_descr->ref_symb0[ant_no], &user_descr->d.hest0[ant_no * num_prb * 12], num_prb * 12);
				sc3850_complex_div_16x16_c(dmrs1, user_descr->ref_symb1[ant_no], &user_descr->d.hest1[ant_no * num_prb * 12], num_prb * 12);
				
				
				for(int inv_i=0; inv_i<num_prb * 12; inv_i++)
				{
					user_descr->d.hest0[inv_i] = TO_COMPLEX16(creal16(user_descr->d.hest0[inv_i]), -cimag16(user_descr->d.hest0[inv_i]));
					user_descr->d.hest1[inv_i] = TO_COMPLEX16(creal16(user_descr->d.hest1[inv_i]), -cimag16(user_descr->d.hest1[inv_i]));;
				}
				*/

#elif(1)
				/* 
				 * Т.к. амплитуда референсных DMRS == 1, то знаменатель при комплексном делении == 1,
				 * поэтому можно использовать только вычисление числителя
				 * 
				 * H = Y / X = ((a+bi)/(c+di) = (a+bi)*(c-di) / (c^2+d^2)
				 * (c^2 + d^2) == 1 =>
				 * H = (a+bi)*(c - di)
				 * 
				 * В матлабе:
				 * hest0 = ref_symb0 .* conj(dmrs0)
				 * hest1 = ref_symb1 .* conj(dmrs1)
				 * 
				 */
				for (i = 0; i < num_prb * 12; i++)
				{
					/*
					user_descr->d.hest0[ant_no * num_prb * 12 + i] = V_pack_2fr(L_mpyd(user_descr->ref_symb0[ant_no][i], dmrs0[i]),
							L_mpycim(user_descr->ref_symb0[ant_no][i], dmrs0[i]));
					user_descr->d.hest1[ant_no * num_prb * 12 + i] = V_pack_2fr(L_mpyd(user_descr->ref_symb1[ant_no][i], dmrs1[i]),
							L_mpycim(user_descr->ref_symb1[ant_no][i], dmrs1[i]));
					*/
					user_descr->d.hest0[ant_no * num_prb * 12 + i] = __mpycx_c_isr_2w(dmrs0[i], user_descr->ref_symb0[ant_no][i]);
					user_descr->d.hest1[ant_no * num_prb * 12 + i] = __mpycx_c_isr_2w(dmrs1[i], user_descr->ref_symb1[ant_no][i]);
					
					/* Увеличение скейлы Ha и Hb на 1, т.к. не производится деление на 2 */
					//SCALE_vector[ant_no*16 + i] += 1;
					//SCALE_vector[ant_no*17 + i] += 1;

					/*
					user_descr->d.hest0[ant_no * num_prb * 12 + i] = V_pack_2fr(L_mpyd(dmrs0[i], user_descr->ref_symb0[ant_no][i]),
							L_mpyim(dmrs0[i], user_descr->ref_symb0[ant_no][i]));
					user_descr->d.hest1[ant_no * num_prb * 12 + i] = V_pack_2fr(L_mpyd(dmrs1[i], user_descr->ref_symb1[ant_no][i]),
							L_mpyim(dmrs1[i], user_descr->ref_symb1[ant_no][i]));
					*/

//					user_descr->hest0[ant_no * num_prb * 12 + i] = V_pack_2fr(L_mpyd(ref_symb0[i], dmrs0[i]), L_mpycim(ref_symb0[i], dmrs0[i]));
//					user_descr->hest1[ant_no * num_prb * 12 + i] = V_pack_2fr(L_mpyd(ref_symb1[i], dmrs1[i]), L_mpycim(ref_symb1[i], dmrs1[i]));
				}
				
#else
				/* 
				 * Фиксированная амплитуда Hest = 1.0 + 0i (0x3fff + 0x0000i)
				 */
				for (i = 0; i < num_prb * 12; i++)
				{
					user_descr->d.hest0[ant_no * num_prb * 12 + i] = TO_COMPLEX16(0x7fff, 0x0000);
					user_descr->d.hest1[ant_no * num_prb * 12 + i] = TO_COMPLEX16(0x7fff, 0x0000);
				}			
#endif
			}

#ifdef H_INTERP
			for(int32_t sym_no = 0; sym_no < LTE_NSYMB_PER_SUBFRAME - 2; sym_no++)
				for(i=0; i<user_descr->num_prb*12; i++)
				{
					int32_t w0 = W_matrix[sym_no * 2 + 0];
					int32_t w1 = W_matrix[sym_no * 2 + 1];
					Complex16 h0 = user_descr->d.hest0[i];
					Complex16 h1 = user_descr->d.hest1[i];
					int32_t re = (creal16(h0) * w0 + creal16(h1) * w1) >> 15;
					int32_t im = (cimag16(h0) * w0 + cimag16(h1) * w1) >> 15;
					OS_ASSERT_COND(re > -32768 && re < 32767);
					OS_ASSERT_COND(im > -32768 && im < 32767);
					h_interp[sym_no][i] = TO_COMPLEX16(re, im);
				}
#endif
			
			pusch_descr->pusch_bd.y_ptr = rx_sf->a0_aligned;
			pusch_descr->pusch_bd.y_gap = enodeb->fp.LTE_N_RE_UL;// * enodeb->fp.LTE_N_PHYS_ANTENNAS_RX;
			pusch_descr->pusch_bd.y_scl_ptr = SCALE_vector;

			/*
			 * Заполнение Segment Headers
			 * Сейчас SEG_ID = USER_ID, т.к. используется 1 слой, SISO
			 */
			
			/* Проверка n_segs не нужна, т.к. делается выше */
			maple_pusch_sh_t *sh = &pusch_descr->pusch_sh[pusch_descr->n_segs++];
			memset(sh, 0, sizeof(maple_pusch_sh_t));
			
			sh->seg_parms0 = (user_descr->index & 0x7f) << 24; // SEG_ID
			sh->seg_parms0 |= PUSCH_SEGMENT_HEADER_USERS_DONE_0;
			sh->seg_parms0 |= PUSCH_SEGMENT_HEADER_COLS_12; // 12 columnds
			sh->seg_parms0 |= ((num_prb * 12 - 1) & 0x3ff);
			
			sh->seg_parms1 = PUSCH_SEGMENT_RB_START(first_prb);
			sh->seg_parms1 |= PUSCH_SEGMENT_HEADER_LV0;
			
			sh->seg_parms1 |= PUSCH_SEGMENT_HEADER_C_EN;
            
			/* 0x08 */
#ifdef H_INTERP
			sh->w_s_params = PUSCH_SEGMENT_HEADER_INTRP_EXTERNAL;
#else
			sh->w_s_params = PUSCH_SEGMENT_HEADER_W_COL_1;
					// | PUSCH_SEGMENT_HEADER_S_COL_1;
					//PUSCH_SEGMENT_HEADER_S_SCL_TYPE_DIFFERENT;
#endif
			/* 0x0C */
			sh->rr0_1 = 0; // no reduction
            /* 0x10 */
			sh->rr2_3 = 0; // no reduction
			/* 0x14 */
			sh->user_rb_amount_layers = PUSCH_SEGMENT_HEADER_USER_RB_AMOUNT(0, num_prb);
			/* 0x18 */
			sh->user_rb_start_layers = PUSCH_SEGMENT_HEADER_USER_RB_START(0, first_prb);
	        /* 0x1C */
			sh->user_id_layers = PUSCH_SEGMENT_HEADER_USER_ID_IN_LAYER(0, user_descr->index);
		    
			/* Used only in joint eq mode, from here */
			/* 0x20 */
			sh->user_grps_mask0_3 = 0; // no groups
		    /* 0x24 */
			sh->user_grps_mask4_7 = 0; // no groups
	        /* 0x28 */
			sh->y_scl_param = 0; // Y scale
			/* Used only in joint eq mode, to here */
			
			/* 0x2C */
			sh->users_type = PUSCH_SEGMENT_HEADER_USER_TYPE(0, 0, 0, 0); // regular users
            /* 0x48 */
			//sh->y_offset = 0;
			sh->y_offset = (first_prb * 12) << 18;
			
            /* 0x4C - 0x54 */
#ifdef H_INTERP
			sh->h_ptr[0] = &h_interp;
			sh->h_ptr[1] = &h_interp;
            /* 0x54 */
			//sh->h_params = ((enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * num_prb * 12) << 2);
			sh->h_params = ((enodeb->fp.LTE_N_RE * 12) << 2);
#else
			sh->h_ptr[0] = user_descr->d.hest0;
			sh->h_ptr[1] = user_descr->d.hest1;
            /* 0x54 */
			//sh->h_params = ((enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * num_prb * 12) << 2);
			sh->h_params = ((num_prb * 12) << 2);
#endif
            
			/* 0x58 */
			/* Определение Эрмитовой матрицы в зависимости от количества приемных антенн
			 * Матрица д.б. представлена в упаковонном виде (п. 1.4.4.3.3.4)
			 */
			switch(enodeb->fp.LTE_N_PHYS_ANTENNAS_RX)
			{
				case 1:
					sh->s_ptr = &S_matrix_1t1r;
					break;
				case 2:
					sh->s_ptr = &S_matrix_2t2r;
					break;
				case 4:
				case 8:
				default:
					sh->s_ptr = &S_matrix_1t1r;
					break;
			}

            /* 0x5C */
			sh->w_ptr = &W_matrix;
            /* 0x60 */
			sh->w_params = (1 << 2); // w_gap = 1 (4 bytes)
            /* 0x64 */
			sh->f_ptr = 0;
            /* 0x68 */
			sh->f_parms  = 0;
            /* 0x70 */
			sh->c_ptr = &C_matrix;
            /* 0x74 */
			sh->scl_ba_ptr = &SCALE_vector;
	        /* 0x78 */
			// FIXME: S SCL_OFFSET, H_SCL_OFFSET
			//sh->s_scl_offset = 2; // BSC9132
			sh->s_scl_offset = 32 * enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; // 2 * 16 (BSC9132 logic)
	        /* 0x7A */
			//sh->h_scl_offset = 1; // BSC9132
			sh->h_scl_offset = 16 * enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; // 1 * 16 (BSC9132 logic)
		    /* 0x7C - 0x84 */
			sh->f_scl_offset[0] = 0;
			sh->f_scl_offset[1] = 0;
			sh->f_scl_offset[2] = 0;
			sh->f_scl_offset[3] = 0;
			
			uint32_t sumKr8 = 0;
			uint32_t maple_ab_table_idx = 0;
			user_descr->Qprime_ACK = 0;
			
			/* Расчет параметров кодирования при наличии HARQ, RI, CQI */
			if(has_harq || has_ri || has_cqi)
			{
				uint32_t B, C, Cplus, Cminus, Kplus, Kminus, F, r, Kr;
				uint32_t sumKr = 0;
			
				B = (ulsch_pdu->ulsch_pdu_rel8.size << 3) + 24;

				liblte_lte_segmentation(B, &C, &Cplus, &Cminus, &Kplus, &Kminus, &F);

				sumKr = 0;

				for (r = 0; r < C; r++)
				{
					if (r < Cminus)
						Kr = Kminus;
					else
						Kr = Kplus;

					sumKr += Kr;
				}

				if (sumKr == 0)
				{
					ERROR(DTRX, "FATAL sumKr is 0!\n");
					continue;
				}
				
				sumKr8 = sumKr << 3;
				INFO(DTRX, "sumKr=%i sumKr8=%i B=%i C=%i Cp=%i Cm=%i Kp=%i Km=%i\n", sumKr, sumKr8,
						B, C, Cplus, Cminus, Kplus, Kminus);
			}
			
			/* Заполнение дескриптора для декодирования HARQ */
			if (has_harq)
			{
				uint32_t Qprime;

				// Compute Q_ack
				Qprime = ulsch_harq_info->harq_information_rel10.harq_size
						* ulsch_initial_tx_params->initial_transmission_parameters_rel8.initial_number_of_resource_blocks
						* LTE_N_SC_RB
						* (12 - (LTE_CP << 1) - ulsch_initial_tx_params->initial_transmission_parameters_rel8.n_srs_initial)
						* beta_harq_times8[ulsch_harq_info->harq_information_rel10.delta_offset_harq];

				if (Qprime > 0)
				{
					if ((Qprime % (sumKr8)) > 0)
						Qprime = 1 + (Qprime / (sumKr8));
					else
						Qprime = Qprime / (sumKr8);

					if (Qprime > (4 * ulsch_pdu->ulsch_pdu_rel8.number_of_resource_blocks * LTE_N_SC_RB))
						Qprime = 4 * ulsch_pdu->ulsch_pdu_rel8.number_of_resource_blocks * LTE_N_SC_RB;
				}

				//  Q_ACK = Qprime * Q_m;
				user_descr->Qprime_ACK = Qprime;
				
				maple_ab_table_idx = ulsch_harq_info->harq_information_rel10.harq_size;
				
				user_descr->has_harq = 1;
			}
			
			user_descr->dtx_thr = maple_a_table[mod_idx][maple_ab_table_idx] * user_descr->Qprime_ACK - maple_b_table[mod_idx][maple_ab_table_idx];
			
			user_descr->Qm = ulsch_pdu->ulsch_pdu_rel8.modulation_type;
			
			if(has_ri)
			{
				
			}
			
			if(has_cqi)
			{
				/* Заполнение полей для CQI */
				uint32_t Qprime, L;
				// Compute Q_cqi
				if (ulsch_cqi_ri_info->cqi_ri_information_rel8.dl_cqi_pmi_size_rank_1 < 12)
					L=0;
				else
					L=8;
				
				// NOTE: we have to handle the case where we have a very small number of bits (condition on pg. 26 36.212)
				if (ulsch_cqi_ri_info->cqi_ri_information_rel8.dl_cqi_pmi_size_rank_1 > 0)
					Qprime = (ulsch_cqi_ri_info->cqi_ri_information_rel8.dl_cqi_pmi_size_rank_1 + L)
						* (LTE_N_SC_RB * ulsch_initial_tx_params->initial_transmission_parameters_rel8.initial_number_of_resource_blocks)
						* (12 - (LTE_CP << 1) - ulsch_initial_tx_params->initial_transmission_parameters_rel8.n_srs_initial)
						* beta_cqi_times8[ulsch_cqi_ri_info->cqi_ri_information_rel8.delta_offset_cqi];
				else
					Qprime=0;
				
				if (Qprime > 0) 
				{
					// check if ceiling is larger than floor in Q' expression
					if ((Qprime % (sumKr8)) > 0)
						Qprime = 1 + (Qprime / (sumKr8));
					else
						Qprime = Qprime/(sumKr8);
				}
				
				//Q_CQI = Q_m * Qprime;
				//pusch_descr->Qprime_CQI = Qprime * ulsch_pdu->ulsch_pdu_rel8.modulation_type;
				user_descr->Qprime_CQI = Qprime;
				user_descr->Qm = ulsch_pdu->ulsch_pdu_rel8.modulation_type;
				
				/* B4860 use internal Qprime_CQI calculation */
				user_descr->d.cqi_amount[0][0] = ulsch_cqi_ri_info->cqi_ri_information_rel8.dl_cqi_pmi_size_rank_1;
				user_descr->cqi_n_carrier = 0;
				user_descr->cqi_i_cqi_offset = ulsch_cqi_ri_info->cqi_ri_information_rel8.delta_offset_cqi;
				user_descr->cqi_cw = 0;
				user_descr->cqi_n_sym_pusch_initial = (12 - (LTE_CP << 1) - ulsch_initial_tx_params->initial_transmission_parameters_rel8.n_srs_initial);
				user_descr->cqi_m_sc_pusch_initial = (LTE_N_SC_RB * ulsch_initial_tx_params->initial_transmission_parameters_rel8.initial_number_of_resource_blocks);
				
				user_descr->has_cqi = 1;
				
				/* Для декодирования CQI размероностью больше 11 бит используется TVPE */
				int32_t cqi_bs = user_descr->d.cqi_amount[0][0];

				if(cqi_bs > 11)
				{
					cqi_bs += 8;
					
					lte_ul_pusch_tvpe_descr_t *tvpe_prev = (pusch_descr->n_tvpe > 0) ? &pusch_descr->pusch_tvpe[pusch_descr->n_tvpe - 1] : NULL;		
					lte_ul_pusch_tvpe_descr_t *tvpe = &pusch_descr->pusch_tvpe[pusch_descr->n_tvpe];
					memset(tvpe, 0, sizeof(lte_ul_pusch_tvpe_descr_t));
					
					if(tvpe_prev)
						tvpe_prev->cop_job.next = &tvpe->cop_job;
					
					tvpe->user = user_descr;
					tvpe->cop_job.job_id = pusch_descr->cop_job.job_id;
					tvpe->cop_job.device_specific = &tvpe->tvpe_job;
					tvpe->cop_job.next = NULL;
					
					tvpe->tvpe_job.bd_index = pusch_descr->n_tvpe;
					
					tvpe->tvpe_job.first_flags = TVPE_VITERBI_K_7 | TVPE_RATE_1_3 | TVPE_VITERBI_JOB | TVPE_PUNC_SCHEME_0;
					tvpe->tvpe_job.second_flags = TVPE_CRC24_POLY0 | TVPE_BLOCK_SIZE(cqi_bs);
					tvpe->tvpe_job.third_flags = TVPE_HOE | TVPE_VIT_SET_1 | TVPE_BUF_SIZE(cqi_bs * 3);
					tvpe->tvpe_job.fourth_flags = TVPE_DOBSY | TVPE_DOBSI | TVPE_POLARITY;
					
					tvpe->tvpe_job.inputs = &user_descr->d.cqi_output[0];
					tvpe->tvpe_job.hard_output_addr = &user_descr->d.cqi_output_hard[0];
					
					pusch_descr->n_tvpe++;
				}
			}
			else
			{
				user_descr->d.cqi_amount[0][0] = 0;
			}
			
			/* q = 0 -- 1 codeword */
			uint32_t q = 0;
			uint32_t c_init = (ulsch_pdu->ulsch_pdu_rel8.rnti << 14) + (q << 13) + (rx_sf->subframe_no << 9)
					+ enodeb->N_id_cell;

			user_descr->hard_out = &rx_ind_data_buf[rx_ind_data_ptr];
			user_descr->tbs_bytes = (ulsch_pdu->ulsch_pdu_rel8.size);
			
			/* Заполнение UPH */
			/* 0x00 */
			//uph->header0 = PUSCH_USER_HEADER_DONE_INT_EN | PUSCH_USER_HEADER_LAYER_0 |
			uph->header0 = PUSCH_USER_HEADER_LAYER_0 |
					PUSCH_USER_HEADER_DISCARD_CW1 | (user_descr->index & 0xff);
			
			/* 0x04 */
			uph->alloc_params = (0x0000 << 16) | ((num_prb << 8) | (first_prb));
			/* 0x08 */
			uph->fc_ptr_shiftted = 0;
			uph->user_type = 0; // Normal user
			//uph->user_type = 1; // EQPE TERM
			
			/* 0x0C */
			uph->nv_beta_ptr = &nvi_beta_table[num_prb][mod_idx];
			/* 0x10 */
			uph->cqi_ptr = &user_descr->d.cqi_output;
			/* 0x14 */
			uph->cqi_amount_ptr = &user_descr->d.cqi_amount;
			
			/* 0x18 */
			/* CQI parameters */
			if(user_descr->has_cqi)
			{
				uph->header1 = (user_descr->cqi_n_carrier << 20) | (user_descr->cqi_i_cqi_offset << 16) |
						(user_descr->cqi_cw << 15) | (user_descr->cqi_n_sym_pusch_initial << 11) | 
						(user_descr->cqi_m_sc_pusch_initial);
			}
			else
			{
				uph->header1 = 0;
			}
			
			/* 0x1C */
			uph->cqi_max_out_en = 0;
			uph->ri_amount = 0;
			
			uph->ack_amount = user_descr->Qprime_ACK;
			
			/* 0x20 */
			if(user_descr->has_harq)
			{
				uph->header2 = ((ulsch_harq_info->harq_information_rel10.harq_size - 1) & 0x1f) << 26;
			}
			else
			{
				uph->header2 = 0;
			}
			
			/* 0x24 */
			/* Порог DTX для NV_BETA_SRC = 0 */
			uph->dtx_thresh = user_descr->dtx_thr;
			
			/* Массив DTX для NV_BETA_SRC = 1 */
			uph->dtx_thresh_ptr = dtx_thr_arr;
			
			/* 0x28 - 0x6C */
			/* 0x28 + 0x24*cw_idx */
#ifdef EQPE_NVI_INTERNAL
			// Расчет NVI/BETA производится в EQPE
			// Т.к. входные скейлы приведены к 0, то вроде как оптимальным является LLR=-4 (0xfc) 
			//uph->cw_parms[0].parms0 = modulation | (0xFC << 16) | (user_descr->tbs_bytes & 0xffff);
			
			
			/* С LLR 4 сейчас не работает */
			uint32_t llr_align = 0;
			int8_t llr_align_qam16[] = {
					10, 9, 9, 8,
					8, 7, 7, 6,
					6, 5, 5, 4,
					4, 3, 3, 2,
					2, 1, 1, 0,
					0, -1, -1, -2,
					-2
			};
			
			switch(modulation)
			{
				case PUSCH_USER_HEADER_CW_MODE_QPSK:
					llr_align = 0xf4; //было 0xf4, (0xf0 - работает)
					break;
					
				case PUSCH_USER_HEADER_CW_MODE_16QAM:
					llr_align = ((uint32_t)llr_align_qam16[num_prb]) & 0xff; //f0
					break;
					
				case PUSCH_USER_HEADER_CW_MODE_64QAM:
					llr_align = 0xff;
					break;
			}
			
			/* Значение LLR_ALIGN
			 * Работает с несколькими значениями при выравнивании скейлов:
			 * S = 8 LLR = 0
			 * S = 8 LLR = 3 - вроде работает лучше
			 * возвращено на 0, с 3 много ошибок для S=7
			 * После установки C_EN=1 и C_matrix={0} возвращено на 3
			 * C_EN = 0, LLR_ALIGN = 0
			 */
			llr_align = 3;
			
			if(p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_LLR_CHECK)
				llr_align = ((uint32_t)llr_check) & 0x000000ff;
			
			uph->cw_parms[0].parms0 = modulation | (llr_align << 16) | (user_descr->tbs_bytes & 0xffff);

#else
			// LLR ALIGN = 4
			uint32_t llr_align = 4;
#ifdef LLR_CHECK
			llr_align = ((uint32_t)llr_check) & 0x000000ff;
#endif
			uph->cw_parms[0].parms0 = modulation | (llr_align << 16) | (user_descr->tbs_bytes & 0xffff);
#endif
			
			/* 0x2C + 0x24*cw_idx */
			uph->cw_parms[0].c_init = c_init;
			
			/* 0x30 + 0x24*cw_idx */
			uph->cw_parms[0].cbx_skip = 0;

			/* 0x34 + 0x24*cw_idx */
			uph->cw_parms[0].parms1 = /*PUSCH_USER_HEADER_DONE_INT_EN | PUSCH_USER_HEADER_CTRL_INT_EN |*/ 
					PUSCH_USER_HEADER_HOE_CW_EN |
					PUSCH_USER_HEADER_MAX_ITER_16 | PUSCH_USER_HEADER_MIN_ITER_1 | rv_idx;
			
			/* 0x38 + 0x24*cw_idx */
			uph->cw_parms[0].parms2 = 0;

			// Получение буфера UL HARQ
			user_descr->hbd = trx_alloc_ul_harq(trx, user_descr->rnti, user_descr->h_pid, first_prb, num_prb);
			
			if(user_descr->hbd != NULL && user_descr->hbd->ptr != NULL)
			{
				if(rv_idx == PUSCH_USER_HEADER_RV_IDX_0)
				{
					// Начальная передача, инициализация буфера HARQ
					user_descr->hbd->n_retx = 0;
					uph->cw_parms[0].parms1 |= (PUSCH_USER_HEADER_FTH_CW_EN | PUSCH_USER_HEADER_HAOE_CW_EN);
				}
				else
				{
					// Продолжение передачи
					uph->cw_parms[0].parms1 |= (PUSCH_USER_HEADER_HAOE_CW_EN);
					user_descr->hbd->n_retx++;
				}
				
				/* 0x3C + 0x24*cw_idx */
				uph->cw_parms[0].harq_in_ptr = user_descr->hbd->ptr;
				/* 0x40 + 0x24*cw_idx */			
				uph->cw_parms[0].harq_out_ptr = user_descr->hbd->ptr;				
			}
			else
			{
				// Буфера HARQ нет, считаем, что всегда начальная передача
				
				uph->cw_parms[0].parms1 |= PUSCH_USER_HEADER_FTH_CW_EN;
				/* 0x3C + 0x24*cw_idx */
				uph->cw_parms[0].harq_in_ptr = NULL;
				/* 0x40 + 0x24*cw_idx */			
				uph->cw_parms[0].harq_out_ptr = NULL;
			}
			
			/* 0x44 + 0x24*cw_idx */
			uph->cw_parms[0].hard_out_ptr = user_descr->hard_out;
			/* 0x48 + 0x24*cw_idx */
			uph->cw_parms[0].soft_out_ptr = NULL;
			/* 0x28 + 0x24*cw_idx */
			uph->cw_parms[1].parms0 = 0;
			/* 0x2C + 0x24*cw_idx */
			uph->cw_parms[1].c_init = 0;
			/* 0x30 + 0x24*cw_idx */
			uph->cw_parms[1].cbx_skip = 0;
			/* 0x34 + 0x24*cw_idx */
			uph->cw_parms[1].parms1 = 0;
			/* 0x38 + 0x24*cw_idx */
			uph->cw_parms[1].parms2 = 0;
			/* 0x3C + 0x24*cw_idx */
			uph->cw_parms[1].harq_in_ptr = 0;
			/* 0x40 + 0x24*cw_idx */			
			uph->cw_parms[1].harq_out_ptr = 0;
			/* 0x44 + 0x24*cw_idx */
			uph->cw_parms[1].hard_out_ptr = 0;
			/* 0x48 + 0x24*cw_idx */
			uph->cw_parms[1].soft_out_ptr = 0;
			
			/* 0x70 */
			user_descr->decouple_ptr = &pusch_descr->decouple_shared[pusch_descr->decouple_offset];
			uph->decouple_ptr = user_descr->decouple_ptr;
			
			/* Расчет смещения для следующего decoupled buffer */
			pusch_descr->decouple_offset += ALIGN_SIZE(user_descr->Qm * (user_descr->num_prb + 1) * LTE_N_SC_RB * LTE_NSYMB_PER_SUBFRAME, ARCH_CACHE_LINE_SIZE);
			
			/* 0x74 */
			if(uph->user_type == 0)
			{
				// Normal user
				uph->eq_ft_rslt_ptr_shifted = 0;
				uph->eqpe_dbg = 0;
			}
			else if(uph->user_type == 1)
			{
				// EQPE TERM
				// TODO: разобраться со смещением буфера
				DEBUG();
				uph->eq_ft_rslt_ptr_shifted = 0; //(((uint32_t)(&user_descr->decouple)) >> 4);
				uph->eqpe_dbg = 1;
			}
			
#ifdef PUSCH_FTPE_DBG
			uph->eq_ft_rslt_ptr_shifted = (((uint32_t)(&pusch_ftpe_eqpe_dbg_buf[0])) >> 4);
			uph->ftpe_dbg = 1;
#else
			uph->ftpe_dbg = 0;
#endif

#ifdef PUSCH_EQPE_DBG
			uph->eq_ft_rslt_ptr_shifted = (((uint32_t)(&pusch_ftpe_eqpe_dbg_buf[0])) >> 4);
			uph->eqpe_dbg = 1;
#else
			uph->ftpe_dbg = 0;
#endif

			/* 0x78 */
			uph->cqi_max_out = sizeof(user_descr->d.cqi_output);
			uph->ext_cqi_bits = 0;
			
			/* 0x80 */
			uph->wh_rbs = 0; /**< Neighbor LLRs weight coefficient for CW0. */
			uph->w_nei_llr_en_cw1 = 0; /**< Neighbor LLRs weights for CW1 enable bit. */
			uph->w_nei_llr_en_cw0 = 0; /**< Neighbor LLRs weights for CW0 enable bit. */
			uph->w_nei_llr_cw1 = 0; /**< Neighbor LLRs weight coefficient for CW1. */
			uph->w_nei_llr_cw0 = 0; /**< Neighbor LLRs weight coefficient for CW0. */
			
			/* 0x84 - 0x90 */
			/**< General purpose pointer:
				 #0: Valid in case user is of type EQ_COMB or LLR_COMB else it should be cleared.
					  In case of EQ-COMB user type - Pointer to neighbor cell post equalization data which will be used for combining.
					  In case of LLR-COMB user type - Pointer to neighbor cell user LLR (post PUPE2) data which will be used for combining.
				 #1: Valid in case user is of type EQ_COMB else it should be cleared.
					 A multi-purpose pointer which points to external data structures requested for PUSCH2_EDF combining process
				 #2: Valid in case of POST_MULT_EN flag is set, else it should be cleared to zero.
					 A multi purpose pointer which points to external data structures requested for PUSCH2_EDF in order to fetch the post multiplication coefficient buffer
			  */
			uph->gen_ptrs[0] = 0;   
			uph->gen_ptrs[1] = 0;
			uph->gen_ptrs[2] = 0;
			
			/* Заполнение RX_ULSCH PDU */
			rx_ind_msg->rx_indication_body.rx_pdu_list[user_descr->index].rx_ue_information.handle = ulsch_pdu->ulsch_pdu_rel8.handle;

			rx_ind_msg->rx_indication_body.rx_pdu_list[user_descr->index].rx_ue_information.rnti = ulsch_pdu->ulsch_pdu_rel8.rnti;

			rx_ind_msg->rx_indication_body.rx_pdu_list[user_descr->index].rx_indication_rel8.offset = 0;
			rx_ind_msg->rx_indication_body.rx_pdu_list[user_descr->index].data_ptr = (void *) &rx_ind_data_buf[rx_ind_data_ptr];
			rx_ind_msg->rx_indication_body.rx_pdu_list[user_descr->index].rx_indication_rel8.length = ulsch_pdu->ulsch_pdu_rel8.size;
			rx_ind_msg->rx_indication_body.rx_pdu_list[user_descr->index].rx_indication_rel8.timing_advance = user_descr->ta;
			
			
			/* Заполнение CRC indication */
			crc_ind_msg->crc_indication_body.crc_pdu_list[user_descr->index].rx_ue_information.rnti = ulsch_pdu->ulsch_pdu_rel8.rnti;
			crc_ind_msg->crc_indication_body.crc_pdu_list[user_descr->index].rx_ue_information.handle = ulsch_pdu->ulsch_pdu_rel8.handle;
			crc_ind_msg->crc_indication_body.crc_pdu_list[user_descr->index].crc_indication_rel8.crc_flag = 1; // 1: CRC_ERROR by default

			/* Адрес следующего блока данных */
			rx_ind_data_ptr += ALIGN_SIZE(ulsch_pdu->ulsch_pdu_rel8.size, ARCH_CACHE_LINE_SIZE);
			
			/* Включение прерывания обработки UE */
			maplePuschJobUserDoneInterruptEnable(uph, &pusch_descr->pusch_bd, user_descr->index);
			if(user_descr->has_cqi)
			{
				maplePuschJobUserCtrlEnable(uph, &pusch_descr->pusch_bd, user_descr->index);
			}
		}

		//rx_ind_msg->header.message_length = msg_len_aligned + rx_ind_data_ptr;
		rx_ind_msg->header.message_length = msg_len_aligned;// + rx_ind_data_ptr;
		rx_ind_msg->rx_indication_body.number_of_pdus = pusch_descr->n_users;
		// Количество CRC совпадает с количеством PDU
		crc_ind_msg->crc_indication_body.number_of_crcs = pusch_descr->n_users;
		
		// Заполнение PUSCH BD header0/1
		pusch_descr->pusch_bd.flags = PUSCH_BD_ALL_USERS_DONE_INT_EN | (((uint32_t)pusch_descr->cop_job.job_id) & 0xff)
				| PUSCH_BD_ALL_CTRL_INT_EN;
		
		switch(enodeb->fp.LTE_N_PHYS_ANTENNAS_RX)
		{
			case 1:
				pusch_descr->pusch_bd.rx = RX_ANTENNAS_1;
				break;
			case 2:
				pusch_descr->pusch_bd.rx = RX_ANTENNAS_2;
				break;
			case 4:
				pusch_descr->pusch_bd.rx = RX_ANTENNAS_4;
				break;
			case 8:
				pusch_descr->pusch_bd.rx = RX_ANTENNAS_8;
				break;
			default:
				ERROR(DTRX,"Invalid number of Rx antennas: %i\n",enodeb->fp.LTE_N_PHYS_ANTENNAS_RX);
		}

		//pusch_descr->pusch_bd.rx = RX_ANTENNAS_1;
#ifdef EQPE_NVI_INTERNAL
		// NVI/Beta расчтиывается блоком EQPE2
		pusch_descr->pusch_bd.nv_beta_src = 1;
#else
		// NVI/Beta расчтиывается хостом
		pusch_descr->pusch_bd.nv_beta_src = 0;
#endif

#if 0
		// Enable decoupled mode when UCI information present
		if(pusch_descr->n_tvpe > 0)
			pusch_descr->pusch_bd.decouple = 1;
		else
			pusch_descr->pusch_bd.decouple = 0;
#else
		// Decoupled буфер включен всегда
		pusch_descr->pusch_bd.decouple = 1;
#endif
		
#ifdef PUSCH_EQPE_DBG
		pusch_descr->pusch_bd.flags |= PUSCH_NV_BETA_OUT_EN;
#endif
		
		if(pusch_descr->n_users != pusch_descr->n_segs)
		{
			invalid_descr++;
		}
		
		if(pusch_descr->n_users > 0)
		{
			if(pusch_descr->n_segs > 0)
			{
				// Valid values: 0 to 199 (corresponds to 1 - 200 segments)
				// Вычитаем 1 из количества сегметов
				pusch_descr->pusch_bd.seg = (pusch_descr->n_segs - 1);
			}
			else
			{
				// Количество сегментов равно 0, этого быть не должно
				ERROR(DTRX, "Zero segments number\n");
				OS_ASSERT;
			}
		}
		
		pusch_descr->pusch_bd.users = pusch_descr->n_users;
		//pusch_descr->pusch_bd.num_eq_term = pusch_descr->n_users;
		
	
		/* Запуск MAPLE PUSCH производится только если количество запросов != 0	 */
		pusch_descr_done = NULL;
		
		if (pusch_descr->n_users > 0)
		{
			osEventQueueReset(trx->evq_pusch_ctrl_ready, 0);
			
			extern volatile uint32_t ctrl_proc;
			extern volatile uint32_t users_proc;
			ctrl_proc = 0;
			users_proc = 0;
			
			DBARS_IBSS_L12();
			DBAR_HWSYNC();
			
			do
			{
				int32_t num_jobs = 1;
				
				status = osCopChannelDispatch(&pusch_ch_handle[0], &pusch_descr->cop_job, &num_jobs);
				if (status != OS_SUCCESS)
				{
					//OS_ASSERT;
				}
			}
			while (status != OS_SUCCESS);

			/* Здесь вызываются процедуры параллельно с работой MAPLE */
			for(int32_t u_no=0; u_no < pusch_descr->n_users; u_no++)
			{
				lte_ul_pusch_user_descr_t *ud = &pusch_descr->users[u_no];
				
				/* Оценка TA и FO */
				pusch_estimate(enodeb, ud, ud->num_prb * 12);
			}
			
			/* Ожидание готовности CQI
			 * Т.к. включен режим DECOUPLED, то сначала демультиплексируются данные CQI,
			 * затем произсодит прерывание ALL_CTRL_DONE, в котором утсанавливается событие готовности CQI,
			 * и только потом запускается процесс декодирования ULSCH
			 * Поэтому сначала ждем готовности CQI и запускаем процесс декодирования на TVPE
			 */
			if(pusch_descr->n_tvpe > 0)
			{
				void *msg = NULL;
				//status = osEventQueuePend(trx->evq_pusch_ctrl_ready, (uint32_t *) &msg, 2);
				//OS_ASSERT_COND(status == OS_SUCCESS);
				
				/* Ожидание готовности CQI или готовности обработки всех пользователей
				 * По какой-то причине иногда не происходит прерывание Control Information Done (флаг ctrl_proc), 
				 * поэтому добавлено ожидание флага user_proc
				 */
				while((ctrl_proc != pusch_descr->n_tvpe) && (users_proc == 0))
					;
			}
			
			if(pusch_descr->n_tvpe > 0)
			{			
				int8_t tw[256];
				
				/* De-interleaving & rate-matching */
				for(int n_tvpe = 0; n_tvpe < pusch_descr->n_tvpe; n_tvpe++)
				{
					lte_ul_pusch_tvpe_descr_t *tvpe = &pusch_descr->pusch_tvpe[n_tvpe];
					int32_t n_cqi_llrs = pusch_descr->status[tvpe->user->index].cqi_amount * tvpe->user->Qm;
					
					lte_rate_matching_cc_rx(enodeb->trx.dw_26_O_RCC, n_cqi_llrs, &tw[92],
							enodeb->trx.dw26, pusch_descr->pusch_tvpe[n_tvpe].user->d.cqi_output);
					
					sub_block_deinterleaving_cc(26, pusch_descr->pusch_tvpe[n_tvpe].user->d.cqi_output, &tw[92]);	
				}
				
				pusch_descr->pusch_tvpe[pusch_descr->n_tvpe - 1].tvpe_job.first_flags |= TVPE_INT_EN;
				
				do
				{
					int32_t num_jobs = pusch_descr->n_tvpe;
					
					status = osCopChannelDispatch(&tvpe_ch_handle[0], &pusch_descr->pusch_tvpe[0].cop_job, &num_jobs);
					if (status != OS_SUCCESS)
					{
						//OS_ASSERT;
					}
				}
				while (status != OS_SUCCESS);
			}

			/* 
			 * Дожидаемся обработки MAPLE-B3
			 */
			status = osEventQueuePend(trx->evq_pusch_ready, (uint32_t *) &pusch_descr_done, 2);
			if (status != OS_SUCCESS || pusch_descr_done == NULL)
			{
				/* Ошибка обработки дескриптора
				 * TODO: надо подумать что тут делать 
				 */
				OS_ASSERT_COND(status == OS_SUCCESS);
			}
			
			int32_t user_done_no = 0;
			
			DBARS_IBSS_L12();
			DBAR_IBSL();
			
			for (user_done_no = 0; user_done_no < pusch_descr->n_users; user_done_no++)
			{
				lte_ul_pusch_user_descr_t *user_descr_done = &pusch_descr_done->users[user_done_no];
				
				//uint8_t *tb = (uint8_t *)rx_ind_msg->rx_indication_body.rx_pdu_list[descr_done_no].data_ptr;

				if (pusch_descr_done->status[user_done_no].tb0_crc == 0)
				{
					uint32_t *hard_buf = (uint32_t *)user_descr_done->hard_out;
					//sweep_cache((uint32_t)hard_buf, ARCH_CACHE_LINE_SIZE, CACHE_INVALIDATE);
					
					// Old L2
					//rx_ind_msg->rx_indication_body.rx_pdu_list[user_done_no].rx_indication_rel8.offset = 1;
					
					// New L2
					// 0: CRC_CORRECT, 1: CRC_ERROR
					crc_ind_msg->crc_indication_body.crc_pdu_list[user_done_no].crc_indication_rel8.crc_flag = 0;
					
					rx_ind_msg->rx_indication_body.rx_pdu_list[user_done_no].rx_indication_rel8.ul_cqi =128 + ((enodeb->fp.LTE_N_PHYS_ANTENNAS_RX == 1) ? 
							user_descr_done->pusch_snr_db[0] / 5 : (user_descr_done->pusch_snr_db[0]  + user_descr_done->pusch_snr_db[1]) / 10);
					
					// Передача прошла успешно, освобождаем буфер HARQ, если он был
					if(user_descr_done->hbd != NULL)
					{
						trx_free_ul_harq(trx, user_descr_done->hbd);
					}
					if(enodeb->fp.LTE_N_PHYS_ANTENNAS_RX == 1)
					{
						//INFO(DTRX, "Got MAC SDU at %i:%i RNTI 0x%04x ptr 0x%08x len %i retx %i ta0 %i ta1 %i ta %i sig %i no %i snr %i %idB\n",
						INFO(DTRX, "Got MAC SDU at %i:%i RNTI 0x%04x ptr 0x%08x len %i retx %i rb_start %i nb_rb %i Qm %i Q_cqi %i ta %i snr0 %i/%i/%idB/%i\n",
								pusch_descr_done->frame, pusch_descr_done->subframe, user_descr_done->rnti,
								hard_buf,
								user_descr_done->tbs_bytes,
								(user_descr_done->hbd != NULL) ? user_descr_done->hbd->n_retx : -1,
								user_descr_done->rballoc >> 16, 
								user_descr_done->rballoc & 0x0000ffff,
								user_descr_done->Qm,
								pusch_descr_done->status[user_done_no].cqi_amount,
								//user_descr_done->ta0[0][0], user_descr_done->ta0[1][0],
								user_descr_done->ta,
								user_descr_done->pusch_sig[0], user_descr_done->pusch_noise[0],
								user_descr_done->pusch_snr_db[0],
								rx_ind_msg->rx_indication_body.rx_pdu_list[user_done_no].rx_indication_rel8.ul_cqi
								);
					}
					else if(enodeb->fp.LTE_N_PHYS_ANTENNAS_RX == 2)
					{
						INFO(DTRX, "Got MAC SDU at %i:%i RNTI 0x%04x ptr 0x%08x len %i retx %i rb_start %i nb_rb %i Qm %i Q_cqi %i ta %i snr0 %i/%i/%idB snr1 %i/%i/%idB %i %i/%i %i/%i\n",
								pusch_descr_done->frame, pusch_descr_done->subframe, user_descr_done->rnti,
								hard_buf,
								user_descr_done->tbs_bytes,
								(user_descr_done->hbd != NULL) ? user_descr_done->hbd->n_retx : -1,
								user_descr_done->rballoc >> 16, 
								user_descr_done->rballoc & 0x0000ffff,
								user_descr_done->Qm,
								pusch_descr_done->status[user_done_no].cqi_amount,
								//user_descr_done->ta0[0][0], user_descr_done->ta0[1][0],
								user_descr_done->ta,
								user_descr_done->pusch_sig[0], user_descr_done->pusch_noise[0],
								user_descr_done->pusch_snr_db[0],
								user_descr_done->pusch_sig[1], user_descr_done->pusch_noise[1],
								user_descr_done->pusch_snr_db[1],
								rx_ind_msg->rx_indication_body.rx_pdu_list[user_done_no].rx_indication_rel8.ul_cqi,
								SCALE_vector[ant_rx * 16 + 0], SCALE_vector[ant_rx * 16 + 1],
								SCALE_vector[ant_rx * 17 + 0], SCALE_vector[ant_rx * 17 + 1]);
					}
				}
				else
				{
					// Установка флага BAD CRC для последующего дампа
					ul_has_bad_crc = 1;
					
					if(enodeb->fp.LTE_N_PHYS_ANTENNAS_RX == 1)
					{
						INFO(DTRX, "Got invalid CRC at %i:%i RNTI 0x%04x rb_start %i nb_rb %i Qm %i Q_cqi %i ta %i snr0 %i/%i/%idB/%i\n",
								pusch_descr_done->frame, pusch_descr_done->subframe,
								user_descr_done->rnti, user_descr_done->rballoc >> 16, 
								user_descr_done->rballoc & 0x0000ffff,
								user_descr_done->Qm,
								pusch_descr_done->status[user_done_no].cqi_amount,
								//user_descr_done->ta0[0][0], user_descr_done->ta0[1][0],
								user_descr_done->ta,
								user_descr_done->pusch_sig[0], user_descr_done->pusch_noise[0],
								user_descr_done->pusch_snr_db[0],
								rx_ind_msg->rx_indication_body.rx_pdu_list[user_done_no].rx_indication_rel8.ul_cqi);
								//user_descr_done->Qprime_CQI);
					}
					else if(enodeb->fp.LTE_N_PHYS_ANTENNAS_RX == 2)
					{
						INFO(DTRX, "Got invalid CRC at %i:%i RNTI 0x%04x rb_start %i nb_rb %i Qm %i Q_cqi %i ta %i snr0 %i/%i/%i/%idB snr1 %i/%i/%i/%idB %i/%i %i/%i\n",
								pusch_descr_done->frame, pusch_descr_done->subframe,
								user_descr_done->rnti, user_descr_done->rballoc >> 16, 
								user_descr_done->rballoc & 0x0000ffff,
								user_descr_done->Qm,
								pusch_descr_done->status[user_done_no].cqi_amount,
								//user_descr_done->ta0[0][0], user_descr_done->ta0[1][0],
								user_descr_done->ta,
								user_descr_done->pusch_sig[0], user_descr_done->pusch_noise[0],
								user_descr_done->pusch_snr[0], user_descr_done->pusch_snr_db[0],
								user_descr_done->pusch_sig[1], user_descr_done->pusch_noise[1],
								user_descr_done->pusch_snr[1], user_descr_done->pusch_snr_db[1],
								SCALE_vector[ant_rx * 16 + 0], SCALE_vector[ant_rx * 16 + 1],
								SCALE_vector[ant_rx * 17 + 0], SCALE_vector[ant_rx * 17 + 1]);
					}
					// Old L2
					//rx_ind_msg->rx_indication_body.rx_pdu_list[user_done_no].rx_indication_rel8.offset = 0;
					
					// New L2
					// 0: CRC_CORRECT, 1: CRC_ERROR
					crc_ind_msg->crc_indication_body.crc_pdu_list[user_done_no].crc_indication_rel8.crc_flag = 1;
					
					/* FIXME: поправить смещение для соответствие FAPI */
					rx_ind_msg->rx_indication_body.rx_pdu_list[user_done_no].rx_indication_rel8.ul_cqi =128 + ((enodeb->fp.LTE_N_PHYS_ANTENNAS_RX == 1) ? 
							user_descr_done->pusch_snr_db[0] / 5 : (user_descr_done->pusch_snr_db[0]  + user_descr_done->pusch_snr_db[1]) / 10);

					// Передача не прошла, проверяем количество перепередач и освобождаем буфер HARQ, если оно превышено
					if(user_descr_done->hbd != NULL && user_descr_done->hbd->n_retx >= 3)
					{
						trx_free_ul_harq(trx, user_descr_done->hbd);
					}
				}
				
				if(user_descr_done->has_harq)
				{
					fapi_harq_indication_pdu_t *harq_ind = &harq_ind_msg->harq_indication_body.harq_pdu_list[harq_ind_msg->harq_indication_body.number_of_harqs++];
					
					harq_ind->rx_ue_information.rnti = user_descr_done->rnti;
					harq_ind->rx_ue_information.handle = user_descr_done->handle;
					
					harq_ind->harq_indication_fdd_rel8.harq_tb1 = (pusch_descr_done->status[user_done_no].ack_d) ? 
							((pusch_descr_done->status[user_done_no].ack_val & 0x01) ? 1 : 0) : 0;
					
					harq_ind->harq_indication_fdd_rel8.harq_tb2 = (pusch_descr_done->status[user_done_no].ack_d) ? 
							((pusch_descr_done->status[user_done_no].ack_val & 0x02) ? 1 : 0) : 0;
					
					harq_ind->ul_cqi_information.channel = 1; // PUSCH
					harq_ind->ul_cqi_information.ul_cqi = 128 + ((enodeb->fp.LTE_N_PHYS_ANTENNAS_RX == 1) ? 
							user_descr_done->pusch_snr_db[0] / 5 : (user_descr_done->pusch_snr_db[0]  + user_descr_done->pusch_snr_db[1]) / 10);
											
					ul_has_ack = (harq_ind->harq_indication_fdd_rel8.harq_tb1 != 0);
					ul_has_nack = (harq_ind->harq_indication_fdd_rel8.harq_tb1 == 0);
					
					INFO(DTRX, "Got HARQ at %i:%i RNTI 0x%04x TB1=%i det %i val %i metric %i/%i Q_ack %i Q_cqi %i\n",
							pusch_descr_done->frame, pusch_descr_done->subframe,
							user_descr_done->rnti,
							harq_ind->harq_indication_fdd_rel8.harq_tb1,
							pusch_descr_done->status[user_done_no].ack_d,
							pusch_descr_done->status[user_done_no].ack_val,
							pusch_descr_done->status[user_done_no].ack_mp_metric[0],
							user_descr_done->dtx_thr,
							user_descr_done->Qprime_ACK,
							pusch_descr_done->status[user_done_no].cqi_amount);
							//user_descr_done->Qprime_CQI);
				}
			}
		}
		
		/* Отправка сообщений FAPI P7 */
		if(rx_ind_msg->rx_indication_body.number_of_pdus > 0)
		{
			fapi_p7_send_rx_indication(rx_ind_ipc_msg);
		}
		
		if(crc_ind_msg->crc_indication_body.number_of_crcs > 0)
		{
			crc_ind_msg->header.message_length = FAPI_GET_MSG_PTR_VAR_SIZE(crc_ind_msg, crc_indication_body.crc_pdu_list,
					crc_ind_msg->crc_indication_body.number_of_crcs);
			
			fapi_p7_send_crc_indication(crc_ind_ipc_msg);
		}
		
		if(harq_ind_msg->harq_indication_body.number_of_harqs > 0)
		{
			harq_ind_msg->header.message_length = FAPI_GET_MSG_PTR_VAR_SIZE(harq_ind_msg, harq_indication_body.harq_pdu_list,
					harq_ind_msg->harq_indication_body.number_of_harqs);
			
			fapi_p7_send_harq_indication(harq_ind_ipc_msg);
		}
		
		if(sr_ind_msg->sr_indication_body.number_of_srs > 0)
		{
			sr_ind_msg->header.message_length = FAPI_GET_MSG_PTR_VAR_SIZE(sr_ind_msg, sr_indication_body.sr_pdu_list,
					sr_ind_msg->sr_indication_body.number_of_srs);

			fapi_p7_send_sr_indication(sr_ind_ipc_msg);
		}
		
		/* Обработка CQI */
		if (pusch_descr_done != NULL && pusch_descr_done->n_users > 0)
		{
			if(pusch_descr_done->n_tvpe > 0)
			{
				/* Дожидаемся обработки TVPE */
				void *tvpe_descr_done = NULL;
				
				status = osEventQueuePend(trx->evq_tvpe_ready, (uint32_t *) &tvpe_descr_done, 2);
				if (status != OS_SUCCESS || tvpe_descr_done == NULL)
				{
					/* Ошибка обработки дескриптора
					 * TODO: надо подумать что тут делать 
					 */
					OS_ASSERT_COND(status == OS_SUCCESS);
				}
			}
			
			for (int32_t user_done_no = 0; user_done_no < pusch_descr_done->n_users; user_done_no++)
			{
				lte_ul_pusch_user_descr_t *user_descr_done = &pusch_descr_done->users[user_done_no];
				if(user_descr_done->has_cqi)
				{
					fapi_cqi_indication_pdu_t *cqi_ind = &cqi_ind_msg->cqi_indication_body.cqi_pdu_list[cqi_ind_msg->cqi_indication_body.number_of_cqis++];
					
					ul_has_cqi = 1;
					
					uint32_t cqi_pusch = 0;
					uint32_t cqi_bit_mask = 0;
					uint32_t cqi_corr_idx = 0;
					uint32_t cqi_crc = 0;
					
					cqi_ind->rx_ue_information.rnti = user_descr_done->rnti;
					cqi_ind->rx_ue_information.handle = user_descr_done->handle;
					
					if(user_descr_done->d.cqi_amount[0][0] < 12)
					{
						uint32_t cqi_mask = 0x80000000;
						uint32_t cqi_corr, cqi_corr_max = 0xffffffff;
						uint32_t n_cqi_bits = 0;
						
						// Декодирование CQI
						/* Новая версия */
						n_cqi_bits = pusch_descr_done->status[user_descr_done->index].cqi_amount * user_descr_done->Qm;
						int64_t *cqi_64b = (int64_t *)&user_descr_done->d.cqi_output[0];
						
						cqi_pusch = __bit_colpsh_b_hh(cqi_pusch, cqi_64b[0]);
						if(n_cqi_bits > 8)
							cqi_pusch = __bit_colpsh_b_hl(cqi_pusch, cqi_64b[1]);
						if(n_cqi_bits > 16)
							cqi_pusch = __bit_colpsh_b_lh(cqi_pusch, cqi_64b[2]);
						if(n_cqi_bits > 24)
							cqi_pusch = __bit_colpsh_b_ll(cqi_pusch, cqi_64b[3]);
		
						// Определние значения CQI
						cqi_bit_mask = ~((1 << (32 - min(n_cqi_bits, 32))) - 1);
						
						for(i=0; i<16; i++)
						{
							cqi_corr = _cob((cqi_precalc_pusch_wideband[i] & cqi_bit_mask) ^ cqi_pusch);
							if(cqi_corr < cqi_corr_max)
							{
								cqi_corr_max = cqi_corr;
								cqi_corr_idx = i;
							}
						}
						
						cqi_ind->cqi_indication_rel8.length = (n_cqi_bits + 7) / 8;
						cqi_ind->cqi_indication_rel8.ul_cqi = cqi_corr_idx;
						cqi_ind->cqi_indication_rel8.data_offset = cqi_raw_data_off;
						cqi_raw_data = (uint8_t*)(&cqi_ind_msg->cqi_indication_body) + cqi_raw_data_off;
						cqi_raw_data_off = ALIGN_SIZE(cqi_raw_data_off + cqi_ind->cqi_indication_rel8.length, ARCH_CACHE_LINE_SIZE);
						
						*cqi_raw_data = cqi_corr_idx & 0xff;
						
						cqi_ind->ul_cqi_information.channel = 1; // PUSCH
						cqi_ind->ul_cqi_information.ul_cqi = 128 + ((enodeb->fp.LTE_N_PHYS_ANTENNAS_RX == 1) ? 
								user_descr_done->pusch_snr_db[0] / 5 : (user_descr_done->pusch_snr_db[0]  + user_descr_done->pusch_snr_db[1]) / 10);

					}
					else
					{
						int32_t n_cqi_bits = user_descr_done->d.cqi_amount[0][0];
						/* Копирование декодированных бит CQI из буфера TVPE */
						cqi_ind->cqi_indication_rel8.length = (n_cqi_bits + 8 + 7) / 8;
						
						/* CQI RAW PDU */
						cqi_ind->cqi_indication_rel8.data_offset = cqi_raw_data_off;
						cqi_raw_data = (uint8_t*)(&cqi_ind_msg->cqi_indication_body) + cqi_raw_data_off;
						cqi_pusch = 0;
						
						cqi_crc = (crc8(user_descr_done->d.cqi_output_hard, n_cqi_bits)) >> 24;
								
						for(int32_t cqi_byte=0; cqi_byte < cqi_ind->cqi_indication_rel8.length; cqi_byte++)
						{
							cqi_pusch = ((cqi_pusch << 8) | user_descr_done->d.cqi_output_hard[cqi_byte]);
							cqi_raw_data[cqi_byte] = user_descr_done->d.cqi_output_hard[cqi_byte];
						}
								
						cqi_raw_data_off = ALIGN_SIZE(cqi_raw_data_off + cqi_ind->cqi_indication_rel8.length, ARCH_CACHE_LINE_SIZE);
								
						cqi_ind->cqi_indication_rel8.ul_cqi = 0;
						
						cqi_ind->ul_cqi_information.channel = 1; // PUSCH
						cqi_ind->ul_cqi_information.ul_cqi = 128 + ((enodeb->fp.LTE_N_PHYS_ANTENNAS_RX == 1) ? 
								user_descr_done->pusch_snr_db[0] / 5 : (user_descr_done->pusch_snr_db[0]  + user_descr_done->pusch_snr_db[1]) / 10);

					}
					
					INFO(DTRX, "Got CQI at %i:%i RNTI 0x%04x Q_ack %i Q_cqi %i val 0x%08x crc %02x CQI %i mask %08x\n",
							pusch_descr_done->frame, pusch_descr_done->subframe,
							user_descr_done->rnti,
							user_descr_done->Qprime_ACK,
							pusch_descr_done->status[user_descr_done->index].cqi_amount,
							//user_descr_done->Qprime_CQI,
							cqi_pusch,
							cqi_crc,
							cqi_corr_idx,
							cqi_bit_mask);
				}				
			}
		}
		
		if(cqi_ind_msg->cqi_indication_body.number_of_cqis > 0)
		{
			cqi_ind_msg->header.message_length = FAPI_GET_MSG_PTR_VAR_SIZE(cqi_ind_msg, cqi_indication_body.cqi_pdu_list,
					cqi_ind_msg->cqi_indication_body.number_of_cqis) + cqi_raw_data_off;
			
			fapi_p7_send_rx_cqi_indication(cqi_ind_ipc_msg);
		}
				
		// Очистка зависших буферов HARQ
		trx_cleanup_ul_harq(trx);
		
		/*
		 * 
		 * 
		 * Отладочный буфер
		 * 
		 * 
		 */
		
		/*
		// Дамп всех непустых сабфреймов
		if(ul_cfg_req->number_of_pdus > 0)
		{
			pusch_dump_results = 1;
			//pusch_descr_done = pusch_descr;
		}
		*/

		if(sec_id != SECURE_ID_FULL)
		{
			/* Отключение дампа */
			//pusch_dump_results = 0;
		}
		
		/* MAPLE-B3 registers */
#define PUPE_BASE_ADDR                        (0x280000)
#define PUPE_IN_BUFF_ADDR                     (PUPE_BASE_ADDR + 0x00000)
#define PUPE_IN_SCL_BUFF_ADDR                 (PUPE_BASE_ADDR + 0x60000)
#define PUPE_OUT0_BUFF_ADDR                   (PUPE_BASE_ADDR + 0x61000)
#define PUPE_OUT1_BUFF_ADDR                   (PUPE_BASE_ADDR + 0x63000)
#define PUPE_REGISTER_BASE_ADDR               (PUPE_BASE_ADDR + 0x7FC00)
		
#ifdef DEBUG_PRACH_DDS		
		//pusch_dump_results = 0;
#endif
		
		if (p8_ind != NULL && pusch_descr_done != NULL &&
				(p8_limit > 0) &&
				((p8_dump_flags & FAPI_P8_DUMP_FLAG_UL) ||
				((p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_ON_BAD_CRC) && ul_has_bad_crc) ||
				((p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUSCH_CQI) && ul_has_cqi) ||
				((p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUSCH_ACK) && ul_has_ack) ||
				((p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUSCH_NACK) && ul_has_nack)
				))
		{
			fapi_p8_ind_pdu_t *p8_ind_pdu = &p8_ind->pdus[p8_ind->number_of_pdus++];
			
			p8_ind_pdu->offset = p8_data_offset;
			p8_ind_pdu->type = FAPI_P8_DUMP_FLAG_UL;
			
			if(ul_has_ack)
				p8_ind_pdu->type |= FAPI_P8_DUMP_FLAG_UL_PUSCH_ACK;
			if(ul_has_nack)
				p8_ind_pdu->type |= FAPI_P8_DUMP_FLAG_UL_PUSCH_NACK;
			if(ul_has_cqi)
				p8_ind_pdu->type |= FAPI_P8_DUMP_FLAG_UL_PUSCH_CQI;
			if(ul_has_bad_crc)
				p8_ind_pdu->type |= FAPI_P8_DUMP_FLAG_UL_ON_BAD_CRC;
			
			uint8_t *p8_ind_data = &p8_ind->pdu_data[p8_data_offset];
			Complex16 *dbg_buf = (Complex16 *)p8_ind_data;
			
			int32_t dbg_sym_no = 0;
			int32_t sym_ptr = 0;
			int32_t out_i;
		
			//dbg_addr_mark = 8416;
			
			INFO(DTRX,"Dumping PUSCH %i:%i addr=0x%08x off=%i\n", rx_sf->frame_no, rx_sf->subframe_no, &p8_ind->pdu_data, 
					p8_data_offset);

			out_i = 0;
			
			if(pusch_descr_done)
			{
				uint32_t *tbuf_ul_req = (uint32_t *)pusch_descr->users[0].ul_config_req_pdu;
			
				if(tbuf_ul_req)
				{
					for(i=0; i<(sizeof(fapi_ul_config_request_pdu_t) + 3) / 4; i++)
					{
						dbg_buf[out_i++] = tbuf_ul_req[i];
					}
				}
				
				num_prb = pusch_descr_done->users[0].num_prb;
			}
			
			out_i += (sizeof(fapi_ul_config_request_pdu_t) + 3) / 4;
			
			if(pusch_descr_done)
			{
				int32_t szbd;
				memcpy(&dbg_buf[out_i], &pusch_descr_done->pusch_bd, sizeof(maple_pusch_job_t));
				szbd = sizeof(maple_pusch_job_t);
				out_i += (sizeof(maple_pusch_job_t) + 3)/ 4;
				
				memcpy(&dbg_buf[out_i], &pusch_descr_done->pusch_sh[0], sizeof(maple_pusch_sh_t));
				out_i += (sizeof(maple_pusch_sh_t) + 3)/ 4;

				memcpy(&dbg_buf[out_i], &pusch_descr_done->pusch_uph[0], sizeof(maple_pusch_uph_t));
				out_i += (sizeof(maple_pusch_uph_t) + 3)/ 4;
				
				memcpy(&dbg_buf[out_i], &pusch_descr_done->status[0], sizeof(maple_pusch_uss_t));
				out_i += (sizeof(maple_pusch_uss_t) + 3)/ 4;

			}
			
			/* Offset 222 samples = 222 * 4 bytes */
			/* Дамп буфера PUFFT */
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
#ifndef USE_SOFT_EQ
				fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->pufft_buf_a0[ant_no * 3 * enodeb->fp.LTE_N_RE], 3 * enodeb->fp.LTE_N_RE * 4);
#else
				fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->a0_aligned[ant_no * 3 * enodeb->fp.LTE_N_RE], 3 * enodeb->fp.LTE_N_RE * 4);
#endif
				out_i += 3 * enodeb->fp.LTE_N_RE;
			}
			
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
#ifndef USE_SOFT_EQ
				fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->pufft_buf_a0_sym3[ant_no * enodeb->fp.LTE_N_RE], enodeb->fp.LTE_N_RE * 4);
#else
				fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->a0_sym3_aligned[ant_no * enodeb->fp.LTE_N_RE], enodeb->fp.LTE_N_RE * 4);
#endif
				out_i += enodeb->fp.LTE_N_RE;
			}
			
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
#ifndef USE_SOFT_EQ
				fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->pufft_buf_a0[ant_no * 3 * enodeb->fp.LTE_N_RE], 6 * enodeb->fp.LTE_N_RE * 4);
#else
				fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->a0_aligned[ant_no * 3 * enodeb->fp.LTE_N_RE], 6 * enodeb->fp.LTE_N_RE * 4);
#endif
				out_i += 6 * enodeb->fp.LTE_N_RE;
			}
			
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
#ifndef USE_SOFT_EQ
				fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->pufft_buf_a0_sym10[ant_no * enodeb->fp.LTE_N_RE], enodeb->fp.LTE_N_RE * 4);
#else
				fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->a0_sym10_aligned[ant_no * enodeb->fp.LTE_N_RE], enodeb->fp.LTE_N_RE * 4);
#endif
				out_i += enodeb->fp.LTE_N_RE;
			}
			
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
#ifndef USE_SOFT_EQ
				fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->pufft_buf_a0[ant_no * 12 * enodeb->fp.LTE_N_RE], 3 * enodeb->fp.LTE_N_RE * 4);
#else
				fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->a0_aligned[ant_no * 12 * enodeb->fp.LTE_N_RE], 3 * enodeb->fp.LTE_N_RE * 4);
#endif
				out_i += 3 * enodeb->fp.LTE_N_RE;
			}
	
			/* Номер фрейма-сабфрейма */
			dbg_buf[enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * 8400] = (TO_COMPLEX16(rx_sf->frame_no, rx_sf->subframe_no));
			dbg_buf[enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * 8401] = (num_prb);
				
			if(pusch_descr_done)
			{
				dbg_buf[enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * 8402] = pusch_descr_done->status[0].tb0_crc == 0 ? 0xff7fff7f : 0x00000000;
			}
				
			/* Скейлы */
			out_i = enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * 9216;			
			DBG_MARK_OUT_I
			for(i=0; i< LTE_NSYMB_PER_SUBFRAME; i++)
			{
				for(ant_no=0; ant_no<enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
				{
					dbg_buf[out_i++] = (rx_sf->pufft_job[i].status[ant_no] & 0x00ff);
				}
			}

			/* Референсные символы */
			out_i = enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * 9232;
			DBG_MARK_OUT_I
			for(i=0; i<num_prb * 12; i++)
			{
				dbg_buf[out_i++] = (dmrs0[i]);
			}

			out_i = ALIGN_ADDRESS(out_i, 16);
			DBG_MARK_OUT_I
			for(i=0; i<num_prb * 12; i++)
			{
				dbg_buf[out_i++] = (dmrs1[i]);
			}
			
			out_i = ALIGN_ADDRESS(out_i, 16);
			DBG_MARK_OUT_I
			if(pusch_descr_done)
			{
				for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
				{
					for(i=0; i<num_prb*12; i++)
					{
						dbg_buf[out_i++] = (pusch_descr_done->users[0].d.hest0[ant_no * num_prb * 12 + i]);
					}
				}
			}
			else
			{
				out_i += num_prb * 12;
			}
			
			out_i = ALIGN_ADDRESS(out_i, 16);
			DBG_MARK_OUT_I
			if(pusch_descr_done)
			{
				for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
				{
					for(i=0; i<num_prb*12; i++)
					{
						dbg_buf[out_i++] = (pusch_descr_done->users[0].d.hest1[ant_no * num_prb * 12 + i]);
					}
				}
			}
			else
			{
				out_i += num_prb * 12;
			}

			out_i = ALIGN_ADDRESS(out_i, 16);
			DBG_MARK_OUT_I
			uint32_t cyclic_shift0 = (enodeb->fapi_config.uplink_reference_signal_config.cyclic_shift_1_for_drms + 0 + //eNB->ulsch[UE_id]->harq_processes[harq_pid]->n_DMRS2 +
					enodeb->refsigs->nPRS[(rx_sf->subframe_no << 1) + 0]) % 12;

			uint32_t cyclic_shift1 = (enodeb->fapi_config.uplink_reference_signal_config.cyclic_shift_1_for_drms + 0 + //eNB->ulsch[UE_id]->harq_processes[harq_pid]->n_DMRS2 +
					enodeb->refsigs->nPRS[(rx_sf->subframe_no << 1) + 1]) % 12;

			dbg_buf[out_i++] = (Complex16) (cyclic_shift0);
			dbg_buf[out_i++] = (Complex16) (cyclic_shift1);
	
			/* Выход IFFT */
			//Complex16 *fft_out = (Complex16 *)0xC10C0000;
			Complex16 *fft_out = (Complex16 *)(SOC_DSP_MAPLE_MBUS_DEFAULT + PUPE_IN_BUFF_ADDR);
			
			out_i = ALIGN_ADDRESS(out_i, 16);
			DBG_MARK_OUT_I
			
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
				for(i=0; i<num_prb * 12 * (LTE_NSYMB_PER_SUBFRAME - 2); i++)
				{
					dbg_buf[out_i++] = (fft_out[ant_no * num_prb * 12 * (LTE_NSYMB_PER_SUBFRAME - 2) + i]);
				}
			}
			
#ifdef PUSCH_FTPE_DBG
			out_i = ALIGN_ADDRESS(out_i, 16);
			DBG_MARK_OUT_I
			//Complex16 *ftpe_dbg_buf = (Complex16 *)pusch_descr_done->users[0].decouple;
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{			
				for(i=0; i<num_prb * 12 * (LTE_NSYMB_PER_SUBFRAME - 2); i++)
				{
					dbg_buf[out_i++] = (pusch_ftpe_eqpe_dbg_buf[i]);
				}
			}
			
			if(pusch_descr_done->pusch_bd.decouple)
			{
				out_i = ALIGN_ADDRESS(out_i, 16);
				DBG_MARK_OUT_I
				for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
				{			
					for(i=0; i<num_prb * 12 * (LTE_NSYMB_PER_SUBFRAME - 2); i++)
					{
						dbg_buf[out_i++] = pusch_descr_done->users[0].decouple[i];
					}
				}
			}
#endif
			
			out_i = ALIGN_ADDRESS(out_i, 16);
			DBG_MARK_OUT_I
			
			//int8_t *fft_scale = (int8_t *)0xC10E0000;
			int8_t *fft_scale = (int8_t *)(SOC_DSP_MAPLE_MBUS_DEFAULT + PUPE_IN_SCL_BUFF_ADDR);
			
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
				for(i=0; i<(LTE_NSYMB_PER_SUBFRAME - 2); i++)
				{
					uint32_t t = (uint32_t)fft_scale[ant_no * (LTE_NSYMB_PER_SUBFRAME - 2) + i];
					dbg_buf[out_i++] = (t);
				}
			}
			/* Выход PUPE */
			//uint8_t *pupe_out = (uint8_t *)(0xC10c0000 + 0x21000);
			uint8_t *pupe_out = (uint8_t *)(SOC_DSP_MAPLE_MBUS_DEFAULT + PUPE_OUT1_BUFF_ADDR);
			
			out_i = ALIGN_ADDRESS(out_i, 16);
			
			uint8_t *pupe_dbg_buf = (uint8_t *)&dbg_buf[out_i]; 

			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
				for(i=0; i<num_prb * 12 * (LTE_NSYMB_PER_SUBFRAME - 2) * 2; i++)
				{
					pupe_dbg_buf[i] = pupe_out[ant_no * num_prb * 12 * (LTE_NSYMB_PER_SUBFRAME - 2) + i];
				}
			}
			
			out_i += enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * num_prb * 12 * (LTE_NSYMB_PER_SUBFRAME - 2) / 4 * 2;
			
#ifdef PUSCH_EQPE_DBG
			if(pusch_descr_done)
			{
				out_i = ALIGN_ADDRESS(out_i, 16);
				DBG_MARK_OUT_I
				int32_t dbg_ptr = 0;
				int8_t *eqpe_scl_buf_in = (int8_t *)pusch_ftpe_eqpe_dbg_buf;
				for(i=0; i<pusch_descr_done->users[0].num_prb * (LTE_NSYMB_PER_SUBFRAME - 2); i++)
				{
					dbg_buf[out_i++] = (TO_COMPLEX16(eqpe_scl_buf_in[i], 0));
				}
				
				dbg_ptr += ALIGN_ADDRESS(pusch_descr_done->users[0].num_prb * (LTE_NSYMB_PER_SUBFRAME - 2), 16) / 4;
				
				for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
				{			
					for(i=0; i<pusch_descr_done->users[0].num_prb * LTE_N_SC_RB * (LTE_NSYMB_PER_SUBFRAME - 2); i++)
					{
						dbg_buf[out_i++] = (pusch_ftpe_eqpe_dbg_buf[dbg_ptr++]);
					}
				}
				
				if(pusch_descr_done->pusch_bd.flags & PUSCH_NV_BETA_OUT_EN)
				{
					for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
					{			
						for(i=0; i<16/4; i++)
						{
							dbg_buf[out_i++] = (pusch_ftpe_eqpe_dbg_buf[dbg_ptr++]);
						}
					}
				}
			}
			
			out_i += 16;
#endif
			/* Decoupled buffer */
			out_i = ALIGN_ADDRESS(out_i, 16);
			DBG_MARK_OUT_I
			
			if(pusch_descr_done->pusch_bd.decouple)
			{
				out_i = ALIGN_ADDRESS(out_i, 16);
				DBG_MARK_OUT_I
				
				uint8_t *t_dec = (uint8_t *)&dbg_buf[out_i];
				int32_t t_dec_ptr = 0;
				
				for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
				{			
					for(i=0; i<pusch_descr_done->users[0].num_prb * LTE_N_SC_RB * (LTE_NSYMB_PER_SUBFRAME - 2); i++)
					{
						//dbg_buf[out_i++] = pusch_descr_done->users[0].decouple_ptr[i];
						t_dec[t_dec_ptr++] = pusch_descr_done->users[0].decouple_ptr[i];
					}
				}
				
				out_i += (t_dec_ptr + 3) / 4;
				
				out_i = ALIGN_ADDRESS(out_i, 16);
				DBG_MARK_OUT_I
			}
			

			uint8_t *sdu_buf = (uint8_t *) &dbg_buf[out_i];
			if(pusch_descr->n_users > 0)
			{
				uint8_t *tb = (uint8_t *)pusch_descr_done->users[0].hard_out;
				for (i = 0; i < pusch_descr_done->users[0].tbs_bytes; i++)
				{
					sdu_buf[i] = tb[i];
				}
				out_i += pusch_descr_done->users[0].tbs_bytes / 4;
			}
			
			//sweep_cache_async((uint32_t) dbg_buf, (14 * 1104 * sizeof(Complex16)), CACHE_FLUSH);
#if 0
			
			out_i = (((out_i + 999) / 1000) * 1000);
			
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
				/* Дамп буфера CPRI, первым идет обработанный буфер, вторым идет активный буфер */
				for (i = 0; i < enodeb->fp.CPRI_SAMPLES_PER_SUBFRAME; i++)
				{
					dbg_buf[out_i++] = (trx->iq_rx_buffer[ant_no][trx->cpri_rx_process_buffer_no][i]);
				}
			}
#endif
#if 0
			for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
				for (i = 0; i < enodeb->fp.CPRI_SAMPLES_PER_SUBFRAME; i++)
				{
					dbg_buf[out_i++] = (trx->iq_rx_buffer[ant_no][trx->cpri_rx_active_buffer_no][i]);;
				}
			}
#endif
			
			//p8_ind_pdu->length = (((out_i + 9999) / 10000) * 10000);
			p8_ind_pdu->length = ALIGN_SIZE(out_i * 4, ARCH_CACHE_LINE_SIZE);
			
			p8_data_offset += ALIGN_SIZE(p8_data_offset + p8_ind_pdu->length, ARCH_CACHE_LINE_SIZE);			
		}
		
		/* Передача P8.ind на PPC, если была индикация */
		if(sec_id == SECURE_ID_FULL && 
				p8_ind != NULL && 
				p8_ind_ipc_msg != NULL && 
				p8_ind->number_of_pdus > 0 &&
				p8_limit > 0)
		{
			fapi_p8_send_indication(p8_ind_ipc_msg);
			p8_limit--;
		}
		
		LOG_EVENT_SF(LOGEVT_UL_TASK_END, rx_sf->frame_no, rx_sf->subframe_no, 0);
	}
}

static float atan2_approximation1(float y, float x)
{
    //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
    //Volkan SALMA

    const float ONEQTR_PI = M_PI / 4.0;
	const float THRQTR_PI = 3.0 * M_PI / 4.0;
	float r, angle;
	float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}
	angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return( -angle );     // negate if in quad III or IV
	else
		return( angle );
}

/* Оценка SNR PUSCH */
#define PUSCH_AVG_FILTER_LEN 3
static int32_t pusch_average_hest(Complex16 *hest, Complex16 *hest_avg, int32_t M)
{
	/*/Коэффициенты усредняющего фильтра по четырем точкам == 0.25 */
	const int32_t avg_filter[PUSCH_AVG_FILTER_LEN] = {TO_COMPLEX16(10922, 0), TO_COMPLEX16(10922, 0), TO_COMPLEX16(10922, 0)};

	int i,k,m;
	Word40 sum_imag0, sum_real0;
	Complex16 *ixi;
	

	//for (i = 0,k=0; k < nr; k++,i += 8)
	for (i = 0,k=0; k < M; k++,i++)
	{
		sum_imag0 = __l_to_x(0);
		sum_real0 = __l_to_x(0);
		ixi = hest + i;

		/*
		for (int j = 0,m=0; m < PUSCH_AVG_FILTER_LEN;m++, j++)
		{
			Complex16 h = avg_filter[j];
			//Complex16 x = ixi[-j-1];
			Complex16 x = ixi[j - PUSCH_AVG_FILTER_LEN / 2];
			//Complex16 x = ixi[-j];
			
			__maccx_2x(x, h, &sum_real0, &sum_imag0);
		}
		*/
		
		Complex16 h;
		Complex16 x;
		
		h = avg_filter[0];
		x = ixi[0 - PUSCH_AVG_FILTER_LEN / 2];
		__maccx_2x(x, h, &sum_real0, &sum_imag0);

		h = avg_filter[1];
		x = ixi[1 - PUSCH_AVG_FILTER_LEN / 2];
		__maccx_2x(x, h, &sum_real0, &sum_imag0);

		h = avg_filter[2];
		x = ixi[2 - PUSCH_AVG_FILTER_LEN / 2];
		__maccx_2x(x, h, &sum_real0, &sum_imag0);

		//__st_srs_8f((void *)&y[i+0],sum_real0,sum_imag0,sum_real1,sum_imag1,sum_real2,sum_imag2,sum_real3,sum_imag3);
		__st_srs_2f((void *)&hest_avg[i],sum_real0,sum_imag0);
	}
	/*
	fir_complex_16x16((const Word16 *) hest, (const Word16 *)avg_filter, (Word16 *) hest_avg,
			M, PUSCH_AVG_FILTER_LEN);
	*/
	return 0;
}

static int32_t pusch_estimate_snr_pusch(lte_enodeb_t *enodeb, lte_ul_pusch_user_descr_t *user_descr, int32_t Msc)
{
	int32_t i;
	int32_t noise_mean = 0;
	int32_t sig_mean = 0;
	int32_t ant_no;
	
	for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
	{
		Complex16 *hest0, *hest1;
		
		hest0 = &user_descr->d.hest0[ant_no * Msc];
		hest1 = &user_descr->d.hest1[ant_no * Msc];
	
		/* Усреднение Hest */
		pusch_average_hest(&hest0[PUSCH_AVG_FILTER_LEN], user_descr->d.hest0_avg, Msc);
		pusch_average_hest(&hest1[PUSCH_AVG_FILTER_LEN], user_descr->d.hest1_avg, Msc);
		
		sig_mean = 0;
		noise_mean = 0;
		
		/* Расчет суммарного значения SNR */
		for(i=0; i<Msc - 2 * PUSCH_AVG_FILTER_LEN; i++)
		{
			Complex16 noise_diff;
			Word40 noise_norm, sig_norm;
			uint32_t noise_norm_round, sig_norm_round;
			
			/* Hest0 */
			sig_norm =  X_mpyd(user_descr->d.hest0_avg[i], user_descr->d.hest0_avg[i]);
			sig_norm_round = X_round(sig_norm);
			//sig_mean += sqrt_table[sig_norm_round >> 2];
			sig_mean += sig_norm_round;
			
			noise_diff = __l_sub_2t(user_descr->d.hest0_avg[i], hest0[i + PUSCH_AVG_FILTER_LEN]);
			noise_norm = X_mpyd(noise_diff, noise_diff);
			noise_norm_round = X_round(noise_norm);
			//noise_mean += sqrt_table[noise_norm_round >> 2];
			noise_mean += noise_norm_round;
			
			/* Hest1 */
			sig_norm =  X_mpyd(user_descr->d.hest1_avg[i], user_descr->d.hest1_avg[i]);
			sig_norm_round = X_round(sig_norm);
			//sig_mean += sqrt_table[sig_norm_round >> 2];
			sig_mean += sig_norm_round;
			
			noise_diff = __l_sub_2t(user_descr->d.hest1_avg[i], hest1[i + PUSCH_AVG_FILTER_LEN]);
			noise_norm = X_mpyd(noise_diff, noise_diff);
			noise_norm_round = X_round(noise_norm);
			//noise_mean += sqrt_table[noise_norm_round >> 2];
			noise_mean += noise_norm_round;
		}
			
		// Calibrated for filter length 3
		/*
		float w = 0.33;
		float a = 7.419 * w * w + 0.1117 * w - 0.005387;
		a = ‭0.854494591‬ * 0.8 = 0.6835956728‬
		k = 1/a = 1.462853
		k_fp = 1.462853 * 1024 = 1498
		 *
		 */
		int32_t k_fp = 1498;
		
		user_descr->pusch_noise[ant_no] = (noise_mean * k_fp) >> 10;
		user_descr->pusch_sig[ant_no] = sig_mean;
		
		if(user_descr->pusch_noise[ant_no] == 0)
			user_descr->pusch_noise[ant_no] = 1;
#if 0
		float snr = (float)(user_descr->pusch_sig[ant_no]) / (float)(user_descr->pusch_noise[ant_no]);
		//user_descr->pusch_snr[ant_no] =(int)((float)snr * 1.58044f);// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		user_descr->pusch_snr[ant_no] = snr;// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		user_descr->pusch_snr_db[ant_no] = dB_fixed_times10(user_descr->pusch_snr[ant_no]);
#else
		int32_t sig_db10 = dB_fixed_times10(user_descr->pusch_sig[ant_no]);
		int32_t noise_db10 = dB_fixed_times10(user_descr->pusch_noise[ant_no]);
		
		user_descr->pusch_snr[ant_no] = 0;// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		user_descr->pusch_snr_db[ant_no] = sig_db10 - noise_db10;
#endif
	}
	
	return 0;
}

static int32_t pusch_estimate_snr_pusch_new(lte_enodeb_t *enodeb, lte_ul_pusch_user_descr_t *user_descr, int32_t Msc)
{
	int32_t i;
	int32_t noise_mean = 0;
	int32_t sig_mean = 0;
	int32_t ant_no;
	int32_t hest0_avg_re=0, hest0_avg_im=0, hest1_avg_re=0, hest1_avg_im=0;
	Complex16 hest0_avg, hest1_avg;
	Word40 noise_norm, sig_norm;
	
	for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
	{
		Complex16 *hest0, *hest1;
		
		hest0 = &user_descr->d.hest0[ant_no * Msc];
		hest1 = &user_descr->d.hest1[ant_no * Msc];
	
		/* Усреднение Hest */
		for(i=0; i<Msc; i++)
		{
			hest0_avg_re = hest0_avg_re + creal16(hest0[i]);
			hest0_avg_im = hest0_avg_im + cimag16(hest0[i]);
			hest1_avg_re = hest1_avg_re + creal16(hest1[i]);
			hest1_avg_im = hest1_avg_im + cimag16(hest1[i]);
		}
		
		hest0_avg_re /= Msc;
		hest0_avg_im /= Msc;
		hest1_avg_re /= Msc;
		hest1_avg_im /= Msc;
		
		hest0_avg = V_pack_2w(hest0_avg_re, hest0_avg_im);
		hest1_avg = V_pack_2w(hest1_avg_re, hest1_avg_im);
		
		sig_mean = 0;
		noise_mean = 0;
		
		sig_norm = X_mpyd(hest0_avg, hest0_avg);
		sig_norm = X_round(sig_norm);
		sig_mean = sig_norm * Msc;

		sig_norm = X_mpyd(hest1_avg, hest1_avg);
		sig_norm = X_round(sig_norm);
		sig_mean += sig_norm * Msc;

		/* Расчет суммарного значения SNR */
		for(i=0; i<Msc; i++)
		{
			Complex16 noise_diff;
			uint32_t noise_norm_round, sig_norm_round;
			
			/* Hest0 */
			noise_diff = __l_sub_2t(hest0_avg, hest0[i]);
			noise_norm = X_mpyd(noise_diff, noise_diff);
			noise_norm_round = X_round(noise_norm);
			//noise_mean += sqrt_table[noise_norm_round >> 2];
			noise_mean += noise_norm_round;
			
			/* Hest1 */
			noise_diff = __l_sub_2t(hest1_avg, hest1[i]);
			noise_norm = X_mpyd(noise_diff, noise_diff);
			noise_norm_round = X_round(noise_norm);
			//noise_mean += sqrt_table[noise_norm_round >> 2];
			noise_mean += noise_norm_round;
		}
		
		// Calibrated for filter length 3
		/*
		float w = 0.33;
		float a = 7.419 * w * w + 0.1117 * w - 0.005387;
		a = ‭0.854494591‬ * 0.8 = 0.6835956728‬
		k = 1/a = 1.462853
		k_fp = 1.462853 * 1024 = 1498
		 *
		 */
		int32_t k_fp = 1498;
		
		user_descr->pusch_noise[ant_no] = (noise_mean * k_fp) >> 10;
		user_descr->pusch_sig[ant_no] = sig_mean;
		
		if(user_descr->pusch_noise[ant_no] == 0)
			user_descr->pusch_noise[ant_no] = 1;
#if 0
		float snr = (float)(user_descr->pusch_sig[ant_no]) / (float)(user_descr->pusch_noise[ant_no]);
		//user_descr->pusch_snr[ant_no] =(int)((float)snr * 1.58044f);// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		user_descr->pusch_snr[ant_no] = snr;// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		user_descr->pusch_snr_db[ant_no] = dB_fixed_times10(user_descr->pusch_snr[ant_no]);
#else
		int32_t sig_db10 = dB_fixed_times10(user_descr->pusch_sig[ant_no]);
		int32_t noise_db10 = dB_fixed_times10(user_descr->pusch_noise[ant_no]);
		
		user_descr->pusch_snr[ant_no] = 0;// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		user_descr->pusch_snr_db[ant_no] = sig_db10 - noise_db10;
#endif
	}
	
	return 0;
}

static int32_t pusch_estimate_snr_pucch(lte_enodeb_t *enodeb, Complex16 *hest)
{
	int32_t i;
	int32_t noise_mean = 0;
	int32_t sig_mean = 0;
	int32_t ant_no;
	int32_t noise[LTE_N_ANTENNAS_MAX], sig[LTE_N_ANTENNAS_MAX], snr_db10[LTE_N_ANTENNAS_MAX];
	
	//for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
	for(ant_no = 0; ant_no < 1; ant_no++)
	{
		Complex16 *hest0, *hest1;
		
		hest0 = &hest[2 * ant_no * LTE_N_SC_RB];
		hest1 = &hest[2 * ant_no * LTE_N_SC_RB + LTE_N_SC_RB];
	
		/* Усреднение Hest */
		pusch_average_hest(&hest0[PUSCH_AVG_FILTER_LEN], hest_avg0, LTE_N_SC_RB);
		pusch_average_hest(&hest1[PUSCH_AVG_FILTER_LEN], hest_avg1, LTE_N_SC_RB);
		
		sig_mean = 0;
		noise_mean = 0;
		
		/* Расчет суммарного значения SNR */
		for(i=0; i<LTE_N_SC_RB - 2 * PUSCH_AVG_FILTER_LEN; i++)
		{
			Complex16 noise_diff;
			Word40 noise_norm, sig_norm;
			uint32_t noise_norm_round, sig_norm_round;
			
			/* Hest0 */
			sig_norm =  X_mpyd(hest_avg0[i], hest_avg0[i]);
			sig_norm_round = X_round(sig_norm);
			//sig_mean += sqrt_table[sig_norm_round >> 2];
			sig_mean += sig_norm_round;
			
			noise_diff = __l_sub_2t(hest_avg0[i], hest0[i + PUSCH_AVG_FILTER_LEN]);
			noise_norm = X_mpyd(noise_diff, noise_diff);
			noise_norm_round = X_round(noise_norm);
			//noise_mean += sqrt_table[noise_norm_round >> 2];
			noise_mean += noise_norm_round;
			
			/* Hest1 */
			sig_norm =  X_mpyd(hest_avg1[i], hest_avg1[i]);
			sig_norm_round = X_round(sig_norm);
			//sig_mean += sqrt_table[sig_norm_round >> 2];
			sig_mean += sig_norm_round;
			
			noise_diff = __l_sub_2t(hest_avg1[i], hest1[i + PUSCH_AVG_FILTER_LEN]);
			noise_norm = X_mpyd(noise_diff, noise_diff);
			noise_norm_round = X_round(noise_norm);
			//noise_mean += sqrt_table[noise_norm_round >> 2];
			noise_mean += noise_norm_round;
		}
			
		// Calibrated for filter length 3
		/*
		float w = 0.33;
		float a = 7.419 * w * w + 0.1117 * w - 0.005387;
		a = ‭0.854494591‬ * 0.8 = 0.6835956728‬
		k = 1/a = 1.462853
		k_fp = 1.462853 * 1024 = 1498
		 *
		 */
		int32_t k_fp = 1498;
		
		noise[ant_no] = (noise_mean * k_fp) >> 10;
		sig[ant_no] = sig_mean;
		
		if(noise[ant_no] == 0)
			noise[ant_no] = 1;
#if 0
		float snr = (float)(user_descr->pusch_sig[ant_no]) / (float)(user_descr->pusch_noise[ant_no]);
		//user_descr->pusch_snr[ant_no] =(int)((float)snr * 1.58044f);// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		user_descr->pusch_snr[ant_no] = snr;// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		user_descr->pusch_snr_db[ant_no] = dB_fixed_times10(user_descr->pusch_snr[ant_no]);
#else
		int32_t sig_db10 = dB_fixed_times10(sig[ant_no]);
		int32_t noise_db10 = dB_fixed_times10(noise[ant_no]);
		
		//snr[ant_no] = 0;// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		snr_db10[ant_no] = sig_db10 - noise_db10;
#endif
	}
	
	return snr_db10[0];
}

static int32_t pusch_estimate_snr_pucch_new(lte_enodeb_t *enodeb, Complex16 *hest)
{
	int32_t i;
	int32_t noise_mean = 0;
	int32_t sig_mean = 0;
	int32_t ant_no;
	int32_t noise[LTE_N_ANTENNAS_MAX], sig[LTE_N_ANTENNAS_MAX], snr_db10[LTE_N_ANTENNAS_MAX];
	int32_t hest0_avg_re=0, hest0_avg_im=0, hest1_avg_re=0, hest1_avg_im=0;
	Complex16 hest0_avg, hest1_avg;
	Word40 noise_norm, sig_norm;
	
	//for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
	for(ant_no = 0; ant_no < 1; ant_no++)
	{
		Complex16 *hest0, *hest1;
		
		hest0 = &hest[2 * ant_no * LTE_N_SC_RB];
		hest1 = &hest[2 * ant_no * LTE_N_SC_RB + LTE_N_SC_RB];
	
		/* Усреднение Hest */
		for(i=0; i<LTE_N_SC_RB; i++)
		{
			hest0_avg_re = hest0_avg_re + creal16(hest0[i]);
			hest0_avg_im = hest0_avg_im + cimag16(hest0[i]);
			hest1_avg_re = hest1_avg_re + creal16(hest1[i]);
			hest1_avg_im = hest1_avg_im + cimag16(hest1[i]);
		}
		
		hest0_avg_re /= LTE_N_SC_RB;
		hest0_avg_im /= LTE_N_SC_RB;
		hest1_avg_re /= LTE_N_SC_RB;
		hest1_avg_im /= LTE_N_SC_RB;
		
		hest0_avg = V_pack_2w(hest0_avg_re, hest0_avg_im);
		hest1_avg = V_pack_2w(hest1_avg_re, hest1_avg_im);
		
		sig_mean = 0;
		noise_mean = 0;
		
		sig_norm = X_mpyd(hest0_avg, hest0_avg);
		sig_norm = X_round(sig_norm);
		sig_mean = sig_norm * LTE_N_SC_RB;

		sig_norm = X_mpyd(hest1_avg, hest1_avg);
		sig_norm = X_round(sig_norm);
		sig_mean += sig_norm * LTE_N_SC_RB;
	
		/* Расчет суммарного значения SNR */
		for(i=0; i<LTE_N_SC_RB; i++)
		{
			Complex16 noise_diff;
			uint32_t noise_norm_round, sig_norm_round;
			
			/* Hest0 */
			noise_diff = __l_sub_2t(hest0_avg, hest0[i]);
			noise_norm = X_mpyd(noise_diff, noise_diff);
			noise_norm_round = X_round(noise_norm);
			//noise_mean += sqrt_table[noise_norm_round >> 2];
			noise_mean += noise_norm_round;
			
			/* Hest1 */
			noise_diff = __l_sub_2t(hest1_avg, hest1[i]);
			noise_norm = X_mpyd(noise_diff, noise_diff);
			noise_norm_round = X_round(noise_norm);
			//noise_mean += sqrt_table[noise_norm_round >> 2];
			noise_mean += noise_norm_round;
		}
			
		// Calibrated for filter length 3
		/*
		float w = 0.33;
		float a = 7.419 * w * w + 0.1117 * w - 0.005387;
		a = ‭0.854494591‬ * 0.8 = 0.6835956728‬
		k = 1/a = 1.462853
		k_fp = 1.462853 * 1024 = 1498
		 *
		 */
		int32_t k_fp = 1498;
		
		noise[ant_no] = (noise_mean * k_fp) >> 10;
		sig[ant_no] = sig_mean;
		
		if(noise[ant_no] == 0)
			noise[ant_no] = 1;
#if 0
		float snr = (float)(user_descr->pusch_sig[ant_no]) / (float)(user_descr->pusch_noise[ant_no]);
		//user_descr->pusch_snr[ant_no] =(int)((float)snr * 1.58044f);// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		user_descr->pusch_snr[ant_no] = snr;// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		user_descr->pusch_snr_db[ant_no] = dB_fixed_times10(user_descr->pusch_snr[ant_no]);
#else
		int32_t sig_db10 = dB_fixed_times10(sig[ant_no]);
		int32_t noise_db10 = dB_fixed_times10(noise[ant_no]);
		
		//snr[ant_no] = 0;// (2 * (Msc - 2 * PUSCH_AVG_FILTER_LEN));
		snr_db10[ant_no] = sig_db10 - noise_db10;
#endif
	}
	
	return snr_db10[0];
}

/* Оценка PUSCH TA */
static int32_t pusch_estimate_ta(lte_enodeb_t *enodeb, lte_ul_pusch_user_descr_t *user_descr, int32_t Msc)
{
	/* Оценка TA */
	/*
	 * hlen = length(h);
	 * r = zeros(2, hlen);
	 * for i=1:m:length(h)-m
	 *     r(:, i) = (h(:, i)) .* conj(h(:, i+m));
	 * end
	 * phi = atan2((imag(r(1,:))), (real(r(1,:))));
	 * dphi = atan2(sum(imag(r(1,2:hlen-2))), sum(real(r(1,2:hlen-2))));
	 * tau = dphi / (2*pi*m)*fftsize
	 */
	int32_t ant_no, i;
	float tauf[2][LTE_N_ANTENNAS_MAX];
	int32_t TA_Ts = 0;
	
	/* Параметр оценки TA - шаг по поднесущим */
	int32_t m = 1;
	
	Word32 sumRi[2], sumRr[2];
	
	for(ant_no=0; ant_no < 1 /*enodeb->fp.LTE_N_PHYS_ANTENNAS_RX*/; ant_no++)
	{
		Word40 sum_r[2], sum_i[2];

		sum_r[0] = X_extend(0);
		sum_i[0] = X_extend(0);
		sum_r[1] = X_extend(0);
		sum_i[1] = X_extend(0);
		
		/* Отступ по 2 поднесущие в начале и в конце (защитные интервалы) */
		//for(i=0; i<Msc - m; i+=m)
		for(i=2; i<Msc - m - 2; i+=m)
		{
			Complex16 x,y;

			x = user_descr->d.hest0[i + ant_no * Msc];
			y = user_descr->d.hest0[i + m + ant_no * Msc];
			sum_r[0] = X_macd(sum_r[0],x,y);
			sum_i[0] = X_maccim(sum_i[0],x,y);
			
			x = user_descr->d.hest1[i + ant_no * Msc];
			y = user_descr->d.hest1[i + m + ant_no * Msc];
			sum_r[1] = X_macd(sum_r[1],x,y);
			sum_i[1] = X_maccim(sum_i[1],x,y);
		}
		
		

		sumRr[0] = X_sat(__ash_rgt_x(11, sum_r[0]));
		sumRi[0] = X_sat(__ash_rgt_x(11, sum_i[0]));
		sumRr[1] = X_sat(__ash_rgt_x(11, sum_r[1]));
		sumRi[1] = X_sat(__ash_rgt_x(11, sum_i[1]));
		
		float sumRrf[2], sumRif[2];
		//__fix2flt_l_2sp(Word32 __Da, Word32 __Db, Word32 __Dc, float* __Dd, float* __De)
		__fix2flt_l_2sp(16, sumRr[0], sumRi[0], &sumRrf[0], &sumRif[0]);
		__fix2flt_l_2sp(16, sumRr[1], sumRi[1], &sumRrf[1], &sumRif[1]);
		
		float dphif[2];
		dphif[0] = atan2_approximation1(sumRif[0], sumRrf[0]);
		dphif[1] = atan2_approximation1(sumRif[1], sumRrf[1]);
		
		tauf[0][ant_no] = dphif[0] / (2.0f*M_PI*m)*enodeb->fp.LTE_SYMBOL_LEN;
		tauf[1][ant_no] = dphif[1] / (2.0f*M_PI*m)*enodeb->fp.LTE_SYMBOL_LEN;
		
		//void __l_flt2fix_sp_2x(Word32 __Da, float __Db, float __Dc, Word32* __Dd, Word32* __De)
		__l_flt2fix_sp_2x(16, tauf[0][ant_no], tauf[1][ant_no], &user_descr->ta0[0][ant_no], &user_descr->ta0[1][ant_no]);
		
		/* Вычисление среднего значения TA_Ts в отсчетах */
		int32_t ta0 = (user_descr->ta0[0][ant_no] + user_descr->ta0[1][ant_no]) / 2;
		TA_Ts += ta0;
	}

	/* Вычисление TA = TA_Ts / 16
	 * Приведение значения TA к диапазону 0..64
	 * 0...31...63
	 * -31...0...31
	 */ 
	int32_t ta = 31 + _round(((TA_Ts / ant_no) * enodeb->fp.LTE_PUSCH_TA_Ts_FACTOR) / 16);
	
	if(ta < 0)
		ta = 0;
	else if(ta > 63)
		ta = 63;
	
	user_descr->ta = ta;
		
	return 0;
}

/* Оценка PUSCh FO */
static int32_t pusch_estimate_fo(lte_enodeb_t *enodeb, lte_ul_pusch_user_descr_t *user_descr, int32_t Msc)
{
	return 0;
}

/* Оценка канала pUSCH */
static int32_t pusch_estimate(lte_enodeb_t *enodeb, lte_ul_pusch_user_descr_t *user_descr, int32_t Msc)
{
	setnosat();
	//setSRbit(0x10); //Set "1-bit down scaling" to ON

	/* Оценка SNR */
	pusch_estimate_snr_pusch(enodeb, user_descr, Msc);
	
	/* Оценка TA */
	pusch_estimate_ta(enodeb, user_descr, Msc);
	
	/* Оценка FO */
	pusch_estimate_fo(enodeb, user_descr, Msc);
	
	return 0;
}

static lte_status_t trx_ul_pucch_ce(lte_enodeb_t *enodeb, lte_ul_subframe_t *rx_sf, PUCCH_FMT_t format, uint32_t n_pucch,
		Complex16 *pucch_dmrs, uint32_t pucch_dmrs_syms, Complex16 *ce, int32_t *snr)
{
	uint32_t i, j;
	Word32 sum_re = 0;
	Word32 sum_im = 0;
	Complex16 *dmrs;
	/* Расчет характеристики канала */
	
	if(format < pucch_format2)
	{
		dmrs = enodeb->refsigs->r_pucch_1[rx_sf->subframe_no][n_pucch];
	}
	else
	{
		dmrs = enodeb->refsigs->r_pucch_2[rx_sf->subframe_no][n_pucch];
	}
	
#ifdef COMPLEX_DIV_
	/* Использование функции комплексного деления */
	sc3850_complex_div_16x16_c(pucch_dmrs, dmrs, ce, pucch_dmrs_syms * LTE_N_SC_RB);
#else
	for(i=0; i<pucch_dmrs_syms * LTE_N_SC_RB; i++)
	{
		/* 
		 * hest0 = ref_symb0 .* conj(dmrs0)
		 */
		//ce[i] = V_pack_2fr(L_mpyd(pucch_dmrs[i], dmrs[i]), L_mpycim(pucch_dmrs[i], dmrs[i]));
		ce[i] = __mpycx_c_isr_2w(dmrs[i], pucch_dmrs[i]);
	}
#endif
	/* расчет множителя для уреднения характеристики канала 
	 * 32767 / (6 >> 1) = 10922
	 */
	Word16 sum_mult = 32767 / (pucch_dmrs_syms >> 1);
	
	for(i=0; i<LTE_N_SC_RB; i++)
	{
		sum_re = 0;
		sum_im = 0;
		
		for(j = 0; j<pucch_dmrs_syms >> 1; j++)
		{
			sum_re = L_mac(sum_re, (Word32)creal16(ce[j * LTE_N_SC_RB + i]), sum_mult); 
			sum_im = L_mac(sum_im, (Word32)cimag16(ce[j * LTE_N_SC_RB + i]), sum_mult);
		}
		
		ce[i] = V_pack_2fr(sum_re, sum_im);
		
		sum_re = 0;
		sum_im = 0;
		
		for(j = pucch_dmrs_syms >> 1; j<pucch_dmrs_syms; j++)
		{
			sum_re = L_mac(sum_re, (Word32)creal16(ce[j * LTE_N_SC_RB + i]), sum_mult); 
			sum_im = L_mac(sum_im, (Word32)cimag16(ce[j * LTE_N_SC_RB + i]), sum_mult);
		}
		
		ce[i + LTE_N_SC_RB] = V_pack_2fr(sum_re, sum_im);

	}
	
	/* Расчет SNR по CE */
	if(snr)
	{
		*snr = pusch_estimate_snr_pucch(enodeb, ce);
	}
	
	return LTE_SUCCESS;
}

static lte_status_t trx_ul_get_pucch(lte_enodeb_t *enodeb, int32_t ant_no, lte_ul_subframe_t *rx_sf, PUCCH_FMT_t format, uint32_t n_pucch,
		uint32_t srs_present, int8_t *scale_vector, int32_t shift_right, Complex16 *ce,
		Complex16 *pucch_sym, uint32_t *pucch_n_syms_out)
{
	uint32_t i;
	uint32_t pucch_n_syms;
	
	/* Выбор RE канала PUCCH */
	if(liblte_pucch_get(enodeb, ant_no, format, n_pucch, rx_sf, scale_vector, srs_present, shift_right, 
			pucch_sym, &pucch_n_syms) != LTE_SUCCESS)
	{
		return LTE_FAIL;
	}
#ifdef COMPLEX_DIV
	/* Корректировка символа PUCCH по ce[] */
	Complex16 *inout = pucch_sym;
	
	for(i = 0; i < pucch_n_syms; i++)
	{
		/* 
		 * res0 = symb0 ./ (ce0)
		 */
		sc3850_complex_div_16x16_c(inout, i < (pucch_n_syms >> 1) ? ce : ce + LTE_N_SC_RB, inout, LTE_N_SC_RB);
		inout += LTE_N_SC_RB;	
	}

#else
	/* Корректировка символа PUCCH по ce[] */
	for(i=0; i<LTE_N_SC_RB; i++)
	{
		for(j=0; j<pucch_n_syms >> 1; j++)
		{
			/* 
			 * res0 = symb0 .* conj(ce0)
			 */
			pucch_sym[j*LTE_N_SC_RB + i] = V_pack_2fr(L_mpyd(pucch_sym[j*LTE_N_SC_RB + i], ce[i]), 
					L_mpycim(pucch_sym[j*LTE_N_SC_RB + i], ce[i]));
		}
		
		for(j=pucch_n_syms >> 1; j<pucch_n_syms; j++)
		{
			/* 
			 * res0 = symb0 ./ conj(ce0)
			 */
			pucch_sym[j*LTE_N_SC_RB + i] = V_pack_2fr(L_mpyd(pucch_sym[j*LTE_N_SC_RB + i], ce[i + LTE_N_SC_RB]), 
					L_mpycim(pucch_sym[j*LTE_N_SC_RB + i], ce[i + LTE_N_SC_RB]));
		}
	}
#endif
	*pucch_n_syms_out = pucch_n_syms;
	return LTE_SUCCESS;
}

static lte_status_t trx_ul_process_uci_pdu(lte_enodeb_t *enodeb, int32_t ant_num, fapi_ul_config_request_pdu_t *ul_req_pdu, uint32_t srs_present,
		lte_ul_subframe_t *rx_sf, int8_t *scale_vector, fapi_harq_indication_pdu_t *harq_pdu,
		fapi_sr_indication_pdu_t *sr_pdu,
		fapi_cqi_indication_pdu_t *cqi_pdu,
		uint8_t *cqi_raw_data,
		int32_t *sr_detected,
		uint32_t allow_dump,
		uint32_t p8_dump_flags,
		uint32_t *dump_buf,
		uint32_t *dump_len)
{
	int32_t ack_len = 0, cqi_len = 0, sr = 0;
	int32_t n_pucch_1_0 = 0;
	int32_t sr_pucch_index = 0;
	int32_t cqi_pucch_index = 0;
	uint32_t pucch_dmrs_syms = 0;
	uint32_t pucch_n_syms;
	uint32_t nCCE = 0;
	PUCCH_FMT_t format;
	int32_t i,j, k;
	uint32_t rnti = 0, handle = 0;
	int32_t refsig_table_idx;
	uint32_t dump_results = 0;
	
	Complex16 *ref_sr, *ref_ack, *ref_nack;
	
	int32_t snr_sr = 0, snr_harq = 0, snr_cqi = 0;
	
	ref_sr = NULL;
	ref_ack = NULL;
	ref_nack = NULL;

	
	*dump_len = 0;
	
	switch(ul_req_pdu->pdu_type)
	{
		case FAPI_UL_CONFIG_UCI_HARQ_PDU_TYPE:
			ack_len = ul_req_pdu->uci_harq_pdu.harq_information.harq_information_rel8_fdd.harq_size;
			n_pucch_1_0 = ul_req_pdu->uci_harq_pdu.harq_information.harq_information_rel8_fdd.n_pucch_1_0;
			
			refsig_table_idx = n_pucch_1_0 % 36;
			
			nCCE = n_pucch_1_0 - enodeb->fapi_config.pucch_config.n1_pucch_an;
			rnti = ul_req_pdu->uci_harq_pdu.ue_information.ue_information_rel8.rnti;
			handle = ul_req_pdu->uci_harq_pdu.ue_information.ue_information_rel8.handle;
			break;
			
		case FAPI_UL_CONFIG_UCI_SR_PDU_TYPE:
			sr = 1;
			sr_pucch_index = ul_req_pdu->uci_sr_pdu.sr_information.sr_information_rel8.pucch_index;
			
			refsig_table_idx = sr_pucch_index % 36;
			
			rnti = ul_req_pdu->uci_sr_pdu.ue_information.ue_information_rel8.rnti;
			handle = ul_req_pdu->uci_sr_pdu.ue_information.ue_information_rel8.handle;
			break;
			
		case FAPI_UL_CONFIG_UCI_SR_HARQ_PDU_TYPE:
			sr = 1;
			ack_len = ul_req_pdu->uci_sr_harq_pdu.harq_information.harq_information_rel8_fdd.harq_size;
			n_pucch_1_0 = ul_req_pdu->uci_sr_harq_pdu.harq_information.harq_information_rel8_fdd.n_pucch_1_0;
			sr_pucch_index = ul_req_pdu->uci_sr_harq_pdu.sr_information.sr_information_rel8.pucch_index;
			
			refsig_table_idx = n_pucch_1_0 % 36;
			
			nCCE = n_pucch_1_0 - enodeb->fapi_config.pucch_config.n1_pucch_an;
			
			rnti = ul_req_pdu->uci_sr_harq_pdu.ue_information.ue_information_rel8.rnti;
			handle = ul_req_pdu->uci_sr_harq_pdu.ue_information.ue_information_rel8.handle;
			break;
			
		case FAPI_UL_CONFIG_UCI_CQI_PDU_TYPE:
			sr = 0;
			ack_len = 0;
			
			cqi_pucch_index = ul_req_pdu->uci_cqi_pdu.cqi_information.cqi_information_rel8.pucch_index;
			cqi_len = ul_req_pdu->uci_cqi_pdu.cqi_information.cqi_information_rel8.dl_cqi_pmi_size;
			
			refsig_table_idx = cqi_pucch_index % 36;
			
			rnti = ul_req_pdu->uci_cqi_pdu.ue_information.ue_information_rel8.rnti;
			handle = ul_req_pdu->uci_cqi_pdu.ue_information.ue_information_rel8.handle;
			break;
			
		default:
			ERROR(DTRX,"Unsupported UL_CONFIG UCI PDU type %i\n", ul_req_pdu->pdu_type);
			return LTE_FAIL;
	}
	
	format = liblte_pucch_get_format(ack_len, cqi_len, sr, LTE_CP);
	
	if(format == pucch_format_error)
	{
		ERROR(DTRX,"Unable to determine PUCCH format ack_len %i cqi_len %i sr %i cp %i\n",
				ack_len, cqi_len, sr, LTE_CP);
		return LTE_FAIL;
	}
	
	if(nCCE >= 100)
	{
		ERROR(DTRX, "Invalid nCCE value %i\n", nCCE);
	}
	
	/* Детектирование PUCCH */
	switch(format)
	{
		case pucch_format1:
		{
			Word32 corr_ant[2];
			Word32 corr_ant_new[2];
			int32_t corr_total = 0;
			int32_t ant_no;
			
			/* Расчет характеристики канала для каждой антенны */
			
			for(ant_no=0; ant_no < ant_num; ant_no++)
			{
			
				if(liblte_refsignal_dmrs_pucch_get(enodeb, ant_no, format, sr_pucch_index,
						rx_sf, scale_vector, pucch_dmrs, &pucch_dmrs_syms) != LTE_SUCCESS)
				{
					return LTE_FAIL;
				}
				
				//trx_ul_pucch_ce(rx_sf, format, sr_pucch_index, pucch_dmrs, pucch_dmrs_syms, ce);
				trx_ul_pucch_ce(enodeb, rx_sf, format, refsig_table_idx, pucch_dmrs, pucch_dmrs_syms, ce, &snr_sr);
				
				trx_ul_get_pucch(enodeb, ant_no, rx_sf, format, sr_pucch_index, srs_present,
						scale_vector, 1, ce, pucch_sym, &pucch_n_syms);
				
				//Complex16 *ref_sr = enodeb_inst.refsigs->r_pucch_sr[rx_sf->subframe_no][sr_pucch_index];
				ref_sr = enodeb->refsigs->r_pucch_sr[rx_sf->subframe_no][refsig_table_idx];
				
				sc3900_vector_dot_product_16x16_c(pucch_sym, ref_sr, pucch_n_syms * LTE_N_SC_RB, corr_ant);
				
				corr_total += _round(corr_ant[0]);
			}
					
			/* FIXME: здесь должен быть вычисляемый порог SR */
			int32_t res = corr_total > enodeb->fp.LTE_SR_THRESHOLD;
			
			if(sr_detected != NULL)
			{
				if((p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUCCH_SR_ALL) ||
						(res && (p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUCCH_SR))
					)
				{
					dump_results = 1;
					INFO(DTRX, "SR rnti %04x n_pucch_sr %i val %i res %i\n", rnti, sr_pucch_index, corr_total, res);
				}

				*sr_detected = res;
				if(sr_pdu)
				{
					sr_pdu->rx_ue_information.rnti = rnti;
					sr_pdu->rx_ue_information.handle = handle;
					/* Заполнение оценки SNR */ 
					sr_pdu->ul_cqi_information.channel = 0;
					sr_pdu->ul_cqi_information.ul_cqi = 128 + snr_sr / 5;
				}
			}
		}
			break;
			
		case pucch_format1a:
		{
			Word32 corr_ack_ant[2], corr_nack_ant[2], corr_ack_sr_ant[2], corr_nack_sr_ant[2];
			int32_t corr_ack_total = 0, corr_nack_total = 0, corr_ack_sr_total = 0, corr_nack_sr_total = 0;
			int32_t sig_thresh_sr = 0, sig_thresh_harq = 0;
			int32_t ant_no;
			int32_t ack = 2;
			int32_t sr_res = 0;
			
			//ref_ack = enodeb_inst.refsigs->r_pucch_ack[rx_sf->subframe_no][n_pucch_1_0];
			//ref_nack = enodeb_inst.refsigs->r_pucch_nack[rx_sf->subframe_no][n_pucch_1_0];
			ref_ack = enodeb->refsigs->r_pucch_ack[rx_sf->subframe_no][refsig_table_idx];
			ref_nack = enodeb->refsigs->r_pucch_nack[rx_sf->subframe_no][refsig_table_idx];
			
			for(ant_no=0; ant_no<ant_num; ant_no++)
			{
			
				if(sr)
				{
					ref_ack = enodeb->refsigs->r_pucch_ack[rx_sf->subframe_no][sr_pucch_index % 36];
					ref_nack = enodeb->refsigs->r_pucch_nack[rx_sf->subframe_no][sr_pucch_index % 36];
					/* Расчет характеристики канала для sr_index_pucch */
					if(liblte_refsignal_dmrs_pucch_get(enodeb, ant_no, format, sr_pucch_index,
							rx_sf, scale_vector, pucch_dmrs, &pucch_dmrs_syms) != LTE_SUCCESS)
					{
						return LTE_FAIL;
					}
					
					//trx_ul_pucch_ce(rx_sf, format, sr_pucch_index, pucch_dmrs, pucch_dmrs_syms, ce);
					/* !!!!!!!!!!!!!!!!!!! */
					
					trx_ul_pucch_ce(enodeb, rx_sf, format, sr_pucch_index % 36, pucch_dmrs, pucch_dmrs_syms, ce, &snr_sr);
					//trx_ul_pucch_ce(rx_sf, format, refsig_table_idx, pucch_dmrs, pucch_dmrs_syms, ce);
					
					trx_ul_get_pucch(enodeb, ant_no, rx_sf, format, sr_pucch_index, srs_present,
										scale_vector, 1, ce, pucch_sym, &pucch_n_syms);
					
					//sc3850_vector_dot_product_16x16_c((Word16 *)pucch_sym, (Word16 *)ref_ack, pucch_n_syms * LTE_N_SC_RB, 2, 2, corr_ack_sr_ant);
					//sc3850_vector_dot_product_16x16_c((Word16 *)pucch_sym, (Word16 *)ref_nack, pucch_n_syms * LTE_N_SC_RB, 2, 2, corr_nack_sr_ant);
					sc3900_vector_dot_product_16x16_c(pucch_sym, ref_ack, pucch_n_syms * LTE_N_SC_RB, corr_ack_sr_ant);
					sc3900_vector_dot_product_16x16_c(pucch_sym, ref_nack, pucch_n_syms * LTE_N_SC_RB, corr_nack_sr_ant);

					corr_ack_sr_total += _round(corr_ack_sr_ant[0]);
					corr_nack_sr_total += _round(corr_nack_sr_ant[0]);					
				}
				else
				{
					corr_ack_sr_total = 0;
					corr_nack_sr_total = 0;
				}
				
				//ref_ack = enodeb_inst.refsigs->r_pucch_ack[rx_sf->subframe_no][n_pucch_1_0];
				//ref_nack = enodeb_inst.refsigs->r_pucch_nack[rx_sf->subframe_no][n_pucch_1_0];
				ref_ack = enodeb->refsigs->r_pucch_ack[rx_sf->subframe_no][refsig_table_idx];
				ref_nack = enodeb->refsigs->r_pucch_nack[rx_sf->subframe_no][refsig_table_idx];
				
				/* Расчет характеристики канала для n_pucch_1_0 */
				if(liblte_refsignal_dmrs_pucch_get(enodeb, ant_no, format, n_pucch_1_0,
						rx_sf, scale_vector, pucch_dmrs, &pucch_dmrs_syms) != LTE_SUCCESS)
				{
					return LTE_FAIL;
				}
							
				//trx_ul_pucch_ce(rx_sf, format, n_pucch_1_0, pucch_dmrs, pucch_dmrs_syms, ce);
				trx_ul_pucch_ce(enodeb, rx_sf, format, refsig_table_idx, pucch_dmrs, pucch_dmrs_syms, ce, &snr_harq);
				
				trx_ul_get_pucch(enodeb, ant_no, rx_sf, format, n_pucch_1_0, srs_present,
									scale_vector, 1, ce, pucch_sym, &pucch_n_syms);
				
				//sc3850_vector_dot_product_16x16_c((Word16 *)pucch_sym, (Word16 *)ref_ack, pucch_n_syms * LTE_N_SC_RB, 2, 2, corr_ack_ant);
				//sc3850_vector_dot_product_16x16_c((Word16 *)pucch_sym, (Word16 *)ref_nack, pucch_n_syms * LTE_N_SC_RB, 2, 2, corr_nack_ant);
				sc3900_vector_dot_product_16x16_c(pucch_sym, ref_ack, pucch_n_syms * LTE_N_SC_RB, corr_ack_ant);
				sc3900_vector_dot_product_16x16_c(pucch_sym, ref_nack, pucch_n_syms * LTE_N_SC_RB, corr_nack_ant);

				corr_ack_total += _round(corr_ack_ant[0]);
				corr_nack_total += _round(corr_nack_ant[0]);
			}
			
			//sig_thresh_harq = _round(sig_mean) * enodeb->fp.LTE_SR_THRESHOLD / (pucch_n_syms * LTE_N_SC_RB);
			sig_thresh_harq = enodeb->fp.LTE_SR_THRESHOLD;
			sig_thresh_sr = enodeb->fp.LTE_SR_THRESHOLD;
			
			/* ACK == 2 - это означает, что ACK/NACK не обнаружен 
			 * Если порог корреляции меньше уровня LTE_SR_THRESHOLD, то значение 2 будет передано по FAPI
			 */
			ack = 2;
			sr_res = 0;
			
			/* Определение варианта SR, SR+ACK, SR_NACK, ACK, NACK, undef */
			if(corr_ack_sr_total > sig_thresh_sr &&
					corr_ack_sr_total > corr_nack_sr_total &&  corr_ack_sr_total > corr_ack_total && corr_ack_sr_total > corr_nack_total)
			{
				/* SR + ACK */
				
				if(p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUCCH_SR)
				{
					dump_results = 1;
					INFO(DTRX, "SR+ACK %i %i %i %i\n", corr_ack_sr_total, corr_nack_sr_total, corr_ack_total, corr_nack_total);
				}
				ack = 1;
				
				if(corr_ack_sr_total > sig_thresh_sr)
					sr_res = 1;
			}
			else if(corr_nack_sr_total > sig_thresh_sr &&
					corr_nack_sr_total > corr_ack_total && corr_nack_sr_total > corr_nack_total)
			{
				/* SR + NACK */
				if(p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUCCH_SR)
				{
					dump_results = 1;
					INFO(DTRX, "SR + NACK %i %i %i %i\n", corr_ack_sr_total, corr_nack_sr_total, corr_ack_total, corr_nack_total);
				}
				ack = 0;
				
				if(corr_nack_sr_total > sig_thresh_sr)
					sr_res = 1;
			}
			else if(corr_ack_total > sig_thresh_harq &&
					corr_ack_total > corr_nack_total)
			{
				/* ACK */
				if(p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUCCH_ACK)
				{
					dump_results = 1;
					INFO(DTRX, "ACK %i %i %i %i\n", corr_ack_sr_total, corr_nack_sr_total, corr_ack_total, corr_nack_total);
				}
				ack = 1;
				sr_res = 0;
			}
			else if(corr_nack_total > sig_thresh_harq)
			{
				/* NACK */
				if(p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUCCH_NACK)
				{
					dump_results = 1;
					INFO(DTRX, "NACK %i %i %i %i\n", corr_ack_sr_total, corr_nack_sr_total, corr_ack_total, corr_nack_total);
				}
				ack = 0;
				sr_res = 0;				
			}
			else
			{
				/* not detected */
				if(p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUCCH_NOT)
				{
					dump_results = 1;
				
					INFO(DTRX, "NOT %i %i %i %i th %i %i\n", corr_ack_sr_total, corr_nack_sr_total, corr_ack_total,
							corr_nack_total, sig_thresh_sr, sig_thresh_harq);
				}
				ack = 0;
				sr_res = 0;
			}

			if(sr_detected)
			{
				*sr_detected = sr_res;
				if(sr_pdu)
				{
					sr_pdu->rx_ue_information.rnti = rnti;
					sr_pdu->rx_ue_information.handle = handle;
					
					/* Заполнение оценки SNR */ 
					sr_pdu->ul_cqi_information.channel = 0;
					sr_pdu->ul_cqi_information.ul_cqi = 128 + snr_sr / 5;
				}
			}
			
			if(harq_pdu != NULL)
			{
				harq_pdu->rx_ue_information.rnti = rnti;
				harq_pdu->rx_ue_information.handle = handle;
				harq_pdu->harq_indication_fdd_rel8.harq_tb1 = ack;
				
				/* Заполнение оценки SNR */ 
				harq_pdu->ul_cqi_information.channel = 0;
				harq_pdu->ul_cqi_information.ul_cqi = 128 + snr_harq / 5;
			}
		}
			break;
			
		case pucch_format2:
		{
			int32_t re_total[16] = {0};
			int32_t im_total[16] = {0};
			int32_t ant_no;
				
			for(ant_no=0; ant_no < ant_num; ant_no++)
			{
				/* Расчет характеристики канала */
				if(liblte_refsignal_dmrs_pucch_get(enodeb, ant_no, format, cqi_pucch_index,
						rx_sf, scale_vector, pucch_dmrs, &pucch_dmrs_syms) != LTE_SUCCESS)
				{
					return LTE_FAIL;
				}
				
				trx_ul_pucch_ce(enodeb, rx_sf, format, refsig_table_idx, pucch_dmrs, pucch_dmrs_syms, ce, &snr_cqi);
				
				trx_ul_get_pucch(enodeb, ant_no, rx_sf, format, cqi_pucch_index, srs_present,
						scale_vector, 1, ce, pucch_sym_tmp, &pucch_n_syms);
				
				/* Перемножение с комплексно-сопряженным референсом */
				sc3850_vector_complex_mult_conj_sc3900(pucch_sym_tmp, 
						enodeb->refsigs->r_pucch_fmt2[rx_sf->subframe_no][refsig_table_idx], pucch_sym, pucch_n_syms * LTE_N_SC_RB);
				
				/* Усреднение по 12 точкам */
				k = 0;
				
				for(i=0; i<pucch_n_syms; i++)
				{
					Complex16 mean = 0;
					for(j=0; j<LTE_N_SC_RB; j++)
					{
						mean = V_add2(mean, pucch_sym[k++]);
					}
					
					re_total[i] += creal16(mean);
					im_total[i] += cimag16(mean);
				}
			}
			
			uint32_t pr = enodeb->refsigs->scrambling_pucch2[rx_sf->subframe_no][rnti];
			uint32_t pr_mask = 1;
			int32_t cqi_descr;
			uint32_t cqi_bits = 0;
			uint32_t cqi_bits_nopr = 0;
			uint32_t cqi_mask = (1 << 19);
			uint32_t cqi_mask_nopr = (1 << 19);
			cqi_descr = i;
			
			for(i = 0; i < 10; i++)
			{
				int16_t s0, s1;
				
				//s0 = creal16(pucch_sym_total[i]);
				//s1 = cimag16(pucch_sym_total[i]);

				s0 = re_total[i];
				s1 = im_total[i];

				cqi_bits_nopr |= (s0 < 0) ? cqi_mask_nopr : 0;
				cqi_mask_nopr = cqi_mask_nopr >> 1;
				
				cqi_bits_nopr |= (s1 < 0) ? cqi_mask_nopr : 0;
				cqi_mask_nopr = cqi_mask_nopr >> 1;
				
				s0 = (pr & pr_mask) ? -s0 : s0;
				pr_mask = pr_mask << 1;
				
				s1 = (pr & pr_mask) ? -s1 : s1;
				pr_mask = pr_mask << 1;
				
				cqi_bits |= (s0 < 0) ? cqi_mask : 0;
				cqi_mask = cqi_mask >> 1;
				
				cqi_bits |= (s1 < 0) ? cqi_mask : 0;
				cqi_mask = cqi_mask >> 1;
				
				pucch_sym[cqi_descr++] = TO_COMPLEX16(s0, 0);
				pucch_sym[cqi_descr++] = TO_COMPLEX16(s1, 0);
			}
			
			// Определение результата CQI
			uint32_t cqi_corr;
			uint32_t cqi_corr_max = 0x0;
			uint32_t cqi_corr_idx = 0;
			
			for(i=0; i<16; i++)
			{
				cqi_corr = _cob((cqi_bits ^ (~cqi_precalc_pucch_wideband[i]) & 0x000fffff));
				if(cqi_corr > cqi_corr_max)
				{
					cqi_corr_max = cqi_corr;
					cqi_corr_idx = i;
				}
			}
			
			if(p8_dump_flags & FAPI_P8_DUMP_FLAG_UL_PUCCH_CQI)
			{
				dump_results = 1;
				INFO(DTRX, "pr %08x b_nopr %08x cqi_bits 0x%08x cqi_corr_max %i cqi_corr_idx %i\n",
					pr, cqi_bits_nopr, cqi_bits, cqi_corr_max, cqi_corr_idx);
			}
			
			if(cqi_pdu != NULL)
			{
				cqi_pdu->rx_ue_information.rnti = rnti;
				cqi_pdu->rx_ue_information.handle = handle;
				cqi_pdu->ul_cqi_information.ul_cqi = 128 + snr_cqi;
				cqi_pdu->ul_cqi_information.channel = 0; // PUCCH
				cqi_pdu->cqi_indication_rel8.length = (cqi_len + 7) / 8;
				cqi_pdu->cqi_indication_rel8.ul_cqi = cqi_corr_idx;
				
				if(cqi_raw_data != NULL)
				{
					uint32_t *cqi_raw_data_32 = (uint32_t *)cqi_raw_data;
					*cqi_raw_data_32 = cqi_bits;
				}
			}
		}
			break;
			
		default:
			ERROR(DTRX,"Unsupported detector for PUCCH format %i\n", format);
	}

	if (allow_dump && dump_results && dump_buf != NULL)
	{
		Complex16 *dbg_buf = (Complex16 *)dump_buf;
		int32_t dbg_sym_no = 0;
		int32_t sym_ptr = 0;
		int32_t out_i;
		int32_t ant_no;

		DBG(DTRX,"Dumping PUCCH %i:%i addr=0x%08x off=%i\n", rx_sf->frame_no, rx_sf->subframe_no,
							debug_ul_buffer, debug_ul_buffer_ptr);
		
		out_i = 222;

		/* Offset 222 samples = 222 * 4 bytes */
		/* Дамп буфера PUFFT */
		for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
		{
#ifndef USE_FAST_MEMCPY_FOR_DEBUG
			for (i = 0; i < 3 * enodeb->fp.LTE_N_RE; i++)
			{
#ifndef USE_SOFT_EQ
				dbg_buf[out_i++] = (rx_sf->pufft_buf_a0[ant_no * 3 * enodeb->fp.LTE_N_RE + i]);
#else
				dbg_buf[out_i++] = (rx_sf->a0_aligned[ant_no * 3 * enodeb->fp.LTE_N_RE + i]);
#endif
			}
#else
#ifndef USE_SOFT_EQ
			fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->pufft_buf_a0[ant_no * 3 * enodeb->fp.LTE_N_RE], 3 * enodeb->fp.LTE_N_RE * 4);
#else
			fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->a0_aligned[ant_no * 3 * enodeb->fp.LTE_N_RE], 3 * enodeb->fp.LTE_N_RE * 4);
#endif
			out_i += 3 * enodeb->fp.LTE_N_RE;
#endif
		}
		
		for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
		{
#ifndef USE_FAST_MEMCPY_FOR_DEBUG
			for (i = 0; i < enodeb->fp.LTE_N_RE; i++)
			{
#ifndef USE_SOFT_EQ
				dbg_buf[out_i++] = (rx_sf->pufft_buf_a0_sym3[ant_no * enodeb->fp.LTE_N_RE + i]);
#else
				dbg_buf[out_i++] = (rx_sf->a0_sym3_aligned[ant_no * enodeb->fp.LTE_N_RE + i]);
#endif
			}
#else
#ifndef USE_SOFT_EQ
			fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->pufft_buf_a0_sym3[ant_no * enodeb->fp.LTE_N_RE], enodeb->fp.LTE_N_RE * 4);
#else
			fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->a0_sym3_aligned[ant_no * enodeb->fp.LTE_N_RE], enodeb->fp.LTE_N_RE * 4);
#endif
			out_i += enodeb->fp.LTE_N_RE;
#endif
		}
		
		for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
		{
#ifndef USE_FAST_MEMCPY_FOR_DEBUG
			for (i = 3 * enodeb->fp.LTE_N_RE; i < 9 * enodeb->fp.LTE_N_RE; i++)
			{
#ifndef USE_SOFT_EQ
				dbg_buf[out_i++] = (rx_sf->pufft_buf_a0[ant_no * 9 * enodeb->fp.LTE_N_RE + i]);
#else
				dbg_buf[out_i++] = (rx_sf->a0_aligned[ant_no * 9 * enodeb->fp.LTE_N_RE + i]);
#endif
			}
#else
#ifndef USE_SOFT_EQ
			fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->pufft_buf_a0[ant_no * 3 * enodeb->fp.LTE_N_RE], 6 * enodeb->fp.LTE_N_RE * 4);
#else
			fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->a0_aligned[ant_no * 3 * enodeb->fp.LTE_N_RE], 6 * enodeb->fp.LTE_N_RE * 4);
#endif
			out_i += 6 * enodeb->fp.LTE_N_RE;
#endif
		}
		
		for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
		{
#ifndef USE_FAST_MEMCPY_FOR_DEBUG
			for (i = 0; i < enodeb->fp.LTE_N_RE; i++)
			{
#ifndef USE_SOFT_EQ
				dbg_buf[out_i++] = (rx_sf->pufft_buf_a0_sym10[ant_no * enodeb->fp.LTE_N_RE + i]);
#else
				dbg_buf[out_i++] = (rx_sf->a0_sym10_aligned[ant_no * enodeb->fp.LTE_N_RE + i]);
#endif
			}
#else
#ifndef USE_SOFT_EQ
			fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->pufft_buf_a0_sym10[ant_no * enodeb->fp.LTE_N_RE], enodeb->fp.LTE_N_RE * 4);
#else
			fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->a0_sym10_aligned[ant_no * enodeb->fp.LTE_N_RE], enodeb->fp.LTE_N_RE * 4);
#endif
			out_i += enodeb->fp.LTE_N_RE;
#endif
		}
		
		for(ant_no = 0; ant_no < enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
		{
#ifndef USE_FAST_MEMCPY_FOR_DEBUG
			for (i = 9 * enodeb->fp.LTE_N_RE; i < 12 * enodeb->fp.LTE_N_RE; i++)
			{
#ifndef USE_SOFT_EQ
				dbg_buf[out_i++] = (rx_sf->pufft_buf_a0[ant_no * 12 * enodeb->fp.LTE_N_RE + i]);
#else
				dbg_buf[out_i++] = (rx_sf->a0_aligned[ant_no * 12 * enodeb->fp.LTE_N_RE + i]);
#endif
			}
#else
#ifndef USE_SOFT_EQ
			fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->pufft_buf_a0[ant_no * 12 * enodeb->fp.LTE_N_RE], 3 * enodeb->fp.LTE_N_RE * 4);
#else
			fast_memcpy_align16(&dbg_buf[out_i], &rx_sf->a0_aligned[ant_no * 12 * enodeb->fp.LTE_N_RE], 3 * enodeb->fp.LTE_N_RE * 4);
#endif
			out_i += 3 * enodeb->fp.LTE_N_RE;
#endif
		}
		
		/* Номер фрейма-сабфрейма */
		dbg_buf[enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * 8400] = (TO_COMPLEX16(rx_sf->frame_no, rx_sf->subframe_no));
			
		/* Скейлы */
		out_i = enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * 9216;			
		for(i=0; i< LTE_NSYMB_PER_SUBFRAME; i++)
		{
			for(ant_no=0; ant_no<enodeb->fp.LTE_N_PHYS_ANTENNAS_RX; ant_no++)
			{
				dbg_buf[out_i++] = (rx_sf->pufft_job[i].status[ant_no] & 0x00ff);
			}
		}
		
		/* Референсные символы */
		out_i = enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * 9232;
		/* Сохранение channel estimate */
		for(i = 0; i < pucch_dmrs_syms * LTE_N_SC_RB; i++)
		{
			dbg_buf[out_i++] = (ce[i]);
		}

		/* Сохранение скорректированных символов PUCCH */
		for(i=0; i<pucch_n_syms * LTE_N_SC_RB; i++)
		{
			dbg_buf[out_i++] = (pucch_sym[i]);
		}
		
		out_i = ALIGN_ADDRESS(out_i, 16);
		if(ref_sr)
		{
			for(i = 0; i < pucch_n_syms * LTE_N_SC_RB; i++)
				dbg_buf[out_i++] = (ref_sr[i]);
		}

		out_i = ALIGN_ADDRESS(out_i, 16);
		if(ref_ack)
		{
			for(i = 0; i < pucch_n_syms * LTE_N_SC_RB; i++)
				dbg_buf[out_i++] = (ref_ack[i]);
		}

		out_i = ALIGN_ADDRESS(out_i, 16);
		if(ref_nack)
		{
			for(i = 0; i < pucch_n_syms * LTE_N_SC_RB; i++)
				dbg_buf[out_i++] = (ref_nack[i]);
		}

		out_i = ALIGN_ADDRESS(out_i, 16);
		
		*dump_len = out_i;
	}
	
	return LTE_SUCCESS;
}

/**
 * Очистка буферов UL HARQ
 * @param trx
 * @return
 */
os_status trx_clear_ul_harq_buffers(lte_trx_t *trx)
{
	int32_t i;
	
	memset(trx->ul_harq_descr, 0, sizeof(lte_ul_harq_descr_t) * 65536);
	
	for(i=0; i<65536; i++)
	{
		trx->ul_harq_descr[i].rnti = i;
	}
	
	return OS_SUCCESS;
}

lte_ul_harq_buf_descr_t *trx_alloc_ul_harq(lte_trx_t *trx, uint32_t rnti, uint32_t h_pid, uint32_t first_rb, uint32_t n_rb)
{
	lte_ul_harq_buf_descr_t *hbd;
	
	OS_ASSERT_COND(trx != NULL);
	OS_ASSERT_COND(rnti >= 0 && rnti <= 65536);
	OS_ASSERT_COND(h_pid >= 0 && h_pid < TRX_NUM_OF_UL_HARQ_BUFFERS);
	
	lte_ul_harq_descr_t *hd = &trx->ul_harq_descr[rnti]; 
	
	hbd = &hd->hd[h_pid];
	
	// Проверка на устаревший дескриптор
	if(hbd->active)
	{
		if((hbd->timer + TRX_UL_HARQ_TIMER_MAX < trx->harq_tti_counter) ||
				hbd->first_rb != first_rb ||
				hbd->n_rb != n_rb)
		{
			// Дескриптор устарел или был с другими параметрами, сброс флага активности
			hbd->active = 0;
		}
	}
	
	if(!hbd->active)
	{
		// Новый дескриптор, заполнение полей, выделение места в буфере
		hbd->timer = trx->harq_tti_counter;
		hbd->first_rb = first_rb;
		hbd->n_rb = n_rb;
		hbd->n_retx = 0;
		
		/*
		 * Расчет смещения в буфере HARQ
		 * Расположение данных в буфере HARQ:
		 * max_mod - бит/символ для QAM64 == 6
		 * | 0 | 1 | n_rb * N_sc * N_syms * max_mod * 3 - 1 | PAD to 64 bytes | first_rb * N_sc * N_syms * max_mod * 3 aligned to 64 bytes | ...
		 */
		hbd->ptr = &trx->ul_harq_buf[h_pid][ALIGN_SIZE(first_rb * LTE_N_SC_RB * LTE_NSYMB_PER_SUBFRAME * 6 * 3, 64)];
	}
	else
	{
		// Активный HARQ, изменяем только время активности и количество перепередач
		// Указатель на буфер остается предыдущий
		hbd->timer = trx->harq_tti_counter;
		//hbd->n_retx++;
	}
	
	hbd->active = 1;
	
	return hbd;
}

void trx_free_ul_harq(lte_trx_t *trx, lte_ul_harq_buf_descr_t *hbd)
{
	OS_ASSERT_COND(trx != NULL);
	OS_ASSERT_COND(hbd != NULL);

	hbd->active = 0;
	hbd->timer = 0;
	hbd->ptr = 0;
}

/**
 * Очистка старых (зависших) HARQ буферов
 * Должна вызываться 1 раз в сабфрейм
 * @param trx
 */
void trx_cleanup_ul_harq(lte_trx_t *trx)
{
	int32_t i;
	
	OS_ASSERT_COND(trx != NULL);
	
	return;
}
