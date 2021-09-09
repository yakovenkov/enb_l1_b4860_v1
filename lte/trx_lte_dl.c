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

// Запись PDSCH в отладочный буфер
//#define DEBUG_TX_CPRI
#define DEBUG_TX_CPRI_ON_DEMAND
#define DEBUG_TX_CPRI_ON_DEMAND_QAM64

//#define DEBUG_PDPE_OUTPUT
//#define DEBUG_PDPE_OUTPUT_QAM64

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

int32_t lte_trx_is_rach_msg3(lte_enodeb_t *enodeb, lte_ul_subframe_t *sf);

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
int sc3850_vector_complex_mult_conj_sc3900(short *restrict input, short *restrict coef, short *restrict result, int N);
os_status maple_fft_ifft(Complex16 *iq_in, Complex16 *iq_out, int32_t len, int32_t inverse);

extern uint32_t dftsizes[33];
extern Complex16 ul_ref_sigs_rx[30][2][33][1200];

/*********************************************************************
 Name: liblte_phy_map_pss

 Description: Maps the Primary Synchronization Signal to the
 subframe.

 Document Reference: 3GPP TS 36.211 v10.1.0 section 6.11.1
 *********************************************************************/
static lte_status_t liblte_phy_map_pss(lte_trx_t *trx, lte_subframe_t *lte_sf)
{
	lte_enodeb_t *enodeb;

	OS_ASSERT_COND(trx != NULL);

	enodeb = trx->enodeb;

	OS_ASSERT_COND(enodeb != NULL);
	OS_ASSERT_COND(lte_sf != NULL);

	/* Проверка на правильность номера слота */
	//OS_ASSERT_COND(lte_sf->subframe_no == 0 || lte_sf->subframe_no == 5);

	lte_sf->job.pdsch_job.third_flags |= PDSCH_BD_PSS_EN;
	lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_PSS_DATA_ADDRESS] = &trx->enodeb->refsigs->pss;
	lte_sf->job.pdsch_job.hdr_config[PDSCH_PSS_GAIN_CONFIG] = PDSCH_BD_GAIN_EXPONENT(trx->enodeb->fp.LTE_PSS_GAIN_E) |
			PDSCH_BD_GAIN_MANTISSA(trx->enodeb->fp.LTE_PSS_GAIN_M);

	return LTE_SUCCESS;
}

/*********************************************************************
 Name: liblte_phy_map_sss

 Description: Maps the Secondary Synchronization Signal to the
 subframe.

 Document Reference: 3GPP TS 36.211 v10.1.0 section 6.11.2
 *********************************************************************/
lte_status_t liblte_phy_map_sss(lte_trx_t *trx, lte_subframe_t *lte_sf)
{
	lte_enodeb_t *enodeb;

	OS_ASSERT_COND(trx != NULL);
	OS_ASSERT_COND(lte_sf != NULL);

	enodeb = trx->enodeb;

	OS_ASSERT_COND(enodeb != NULL);
	//OS_ASSERT_COND(lte_sf->subframe_no == 0 || lte_sf->subframe_no == 5);
	//OS_ASSERT_COND(enodeb->sss0 != NULL);
	//OS_ASSERT_COND(enodeb->sss5 != NULL);

	if (lte_sf->subframe_no == 0)
	{
		lte_sf->job.pdsch_job.third_flags |= PDSCH_BD_SSS_EN;
		lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_SSS_DATA_ADDRESS] = &trx->enodeb->refsigs->sss0;
	}
	else if (lte_sf->subframe_no == 5)
	{
		lte_sf->job.pdsch_job.third_flags |= PDSCH_BD_SSS_EN;
		lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_SSS_DATA_ADDRESS] = &trx->enodeb->refsigs->sss5;
	}
	
	lte_sf->job.pdsch_job.hdr_config[PDSCH_SSS_GAIN_CONFIG] = PDSCH_BD_GAIN_EXPONENT(trx->enodeb->fp.LTE_SSS_GAIN_E) |
			PDSCH_BD_GAIN_MANTISSA(trx->enodeb->fp.LTE_SSS_GAIN_M);

	return (LTE_SUCCESS);
}

lte_status_t lte_fill_dci_1a_alloc_10MHz_FDD(lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	/* SIB1 */
	DCI1A_10MHz_FDD_t *dci_1a = (DCI1A_10MHz_FDD_t *) &dci->dci_pdu[0];
	
	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI1A_10MHz_FDD_t;
	dci->L = req_pdu->dci_dl_pdu_rel8.aggregation_level;
	dci->rnti = req_pdu->dci_dl_pdu_rel8.rnti;
	dci->format = format1A;
	dci->ra_flag = req_pdu->dci_dl_pdu_rel8.allocate_prach_flag;
	dci->first_CCE = req_pdu->dci_dl_pdu_rel8.cce_idx;

	/* Заполнение DCI PDU */
	dci_1a->type = 1;
	dci_1a->vrb_type = req_pdu->dci_dl_pdu_rel8.virtual_resource_block_assignment_flag;

	dci_1a->rballoc = req_pdu->dci_dl_pdu_rel8.resource_block_coding;
	dci_1a->TPC = req_pdu->dci_dl_pdu_rel8.tpc;
	dci_1a->harq_pid = req_pdu->dci_dl_pdu_rel8.harq_process;
	dci_1a->mcs = req_pdu->dci_dl_pdu_rel8.mcs_1;
	dci_1a->ndi = req_pdu->dci_dl_pdu_rel8.new_data_indicator_1;
	dci_1a->rv = req_pdu->dci_dl_pdu_rel8.redundancy_version_1;
	
#if 0
	/*
	 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 * Определение сабфрейма с ответом Msg3 (использовалось для тестирования)
	 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 */
	if (req_pdu->dci_dl_pdu_rel8.allocate_prach_flag)
	{
		uint32_t rach_msg3_frame = lte_sf->frame_no;
		uint32_t rach_msg3_subframe = lte_sf->subframe_no + 6;

		INFO(
				DTRX,
				"Got DCI_1A at %i:%i mcs=%i rballoc=%i rnti=%04x\n",
				lte_sf->frame_no, lte_sf->subframe_no, req_pdu->dci_dl_pdu_rel8.mcs_1, req_pdu->dci_dl_pdu_rel8.resource_block_coding, req_pdu->dci_dl_pdu_rel8.rnti);

		if (rach_msg3_subframe > 9)
		{
			rach_msg3_frame++;
			rach_msg3_subframe -= 10;
		}

		enodeb_inst.trx.rach_msg3_frame_no = rach_msg3_frame;
		enodeb_inst.trx.rach_msg3_subframe_no = rach_msg3_subframe;
		enodeb_inst.trx.rach_msg3_active = 1;
	}
#endif
	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_1a_alloc_3MHz_FDD(lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	/* SIB1 */
	DCI1A_3MHz_FDD_t *dci_1a = (DCI1A_3MHz_FDD_t *) &dci->dci_pdu[0];
	
	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI1A_3MHz_FDD_t;
	dci->L = req_pdu->dci_dl_pdu_rel8.aggregation_level;
	dci->rnti = req_pdu->dci_dl_pdu_rel8.rnti;
	dci->format = format1A;
	dci->ra_flag = req_pdu->dci_dl_pdu_rel8.allocate_prach_flag;
	dci->first_CCE = req_pdu->dci_dl_pdu_rel8.cce_idx;

	/* Заполнение DCI PDU */
	dci_1a->type = 1;
	dci_1a->vrb_type = req_pdu->dci_dl_pdu_rel8.virtual_resource_block_assignment_flag;

	dci_1a->rballoc = req_pdu->dci_dl_pdu_rel8.resource_block_coding;
	dci_1a->TPC = req_pdu->dci_dl_pdu_rel8.tpc;
	dci_1a->harq_pid = req_pdu->dci_dl_pdu_rel8.harq_process;
	dci_1a->mcs = req_pdu->dci_dl_pdu_rel8.mcs_1;
	dci_1a->ndi = req_pdu->dci_dl_pdu_rel8.new_data_indicator_1;
	dci_1a->rv = req_pdu->dci_dl_pdu_rel8.redundancy_version_1;
	
	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_1a_alloc_5MHz_FDD(lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	/* SIB1 */
	DCI1A_5MHz_FDD_t *dci_1a = (DCI1A_5MHz_FDD_t *) &dci->dci_pdu[0];
	
	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI1A_5MHz_FDD_t;
	dci->L = req_pdu->dci_dl_pdu_rel8.aggregation_level;
	dci->rnti = req_pdu->dci_dl_pdu_rel8.rnti;
	dci->format = format1A;
	dci->ra_flag = req_pdu->dci_dl_pdu_rel8.allocate_prach_flag;
	dci->first_CCE = req_pdu->dci_dl_pdu_rel8.cce_idx;

	/* Заполнение DCI PDU */
	dci_1a->type = 1;
	dci_1a->vrb_type = req_pdu->dci_dl_pdu_rel8.virtual_resource_block_assignment_flag;

	dci_1a->rballoc = req_pdu->dci_dl_pdu_rel8.resource_block_coding;
	dci_1a->TPC = req_pdu->dci_dl_pdu_rel8.tpc;
	dci_1a->harq_pid = req_pdu->dci_dl_pdu_rel8.harq_process;
	dci_1a->mcs = req_pdu->dci_dl_pdu_rel8.mcs_1;
	dci_1a->ndi = req_pdu->dci_dl_pdu_rel8.new_data_indicator_1;
	dci_1a->rv = req_pdu->dci_dl_pdu_rel8.redundancy_version_1;
	
	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_1a_alloc_10MHz_TDD(lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	/* SIB1 */
	DCI1A_10MHz_TDD_1_6_t *dci_1a = (DCI1A_10MHz_TDD_1_6_t *) &dci->dci_pdu[0];
	
	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI1A_10MHz_TDD_1_6_t;
	dci->L = req_pdu->dci_dl_pdu_rel8.aggregation_level;
	dci->rnti = req_pdu->dci_dl_pdu_rel8.rnti;
	dci->format = format1A;
	dci->ra_flag = req_pdu->dci_dl_pdu_rel8.allocate_prach_flag;
	dci->first_CCE = req_pdu->dci_dl_pdu_rel8.cce_idx;

	/* Заполнение DCI PDU */
	dci_1a->type = 1;
	dci_1a->vrb_type = req_pdu->dci_dl_pdu_rel8.virtual_resource_block_assignment_flag;

	dci_1a->rballoc = req_pdu->dci_dl_pdu_rel8.resource_block_coding;
	dci_1a->TPC = req_pdu->dci_dl_pdu_rel8.tpc;
	dci_1a->harq_pid = req_pdu->dci_dl_pdu_rel8.harq_process;
	dci_1a->mcs = req_pdu->dci_dl_pdu_rel8.mcs_1;
	dci_1a->ndi = req_pdu->dci_dl_pdu_rel8.new_data_indicator_1;
	dci_1a->rv = req_pdu->dci_dl_pdu_rel8.redundancy_version_1;
	
	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_1a_alloc_5MHz_TDD(lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	/* SIB1 */
	DCI1A_5MHz_TDD_1_6_t *dci_1a = (DCI1A_5MHz_TDD_1_6_t *) &dci->dci_pdu[0];
	
	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI1A_5MHz_TDD_1_6_t;
	dci->L = req_pdu->dci_dl_pdu_rel8.aggregation_level;
	dci->rnti = req_pdu->dci_dl_pdu_rel8.rnti;
	dci->format = format1A;
	dci->ra_flag = req_pdu->dci_dl_pdu_rel8.allocate_prach_flag;
	dci->first_CCE = req_pdu->dci_dl_pdu_rel8.cce_idx;

	/* Заполнение DCI PDU */
	dci_1a->type = 1;
	dci_1a->vrb_type = req_pdu->dci_dl_pdu_rel8.virtual_resource_block_assignment_flag;

	dci_1a->rballoc = req_pdu->dci_dl_pdu_rel8.resource_block_coding;
	dci_1a->TPC = req_pdu->dci_dl_pdu_rel8.tpc;
	dci_1a->harq_pid = req_pdu->dci_dl_pdu_rel8.harq_process;
	dci_1a->mcs = req_pdu->dci_dl_pdu_rel8.mcs_1;
	dci_1a->ndi = req_pdu->dci_dl_pdu_rel8.new_data_indicator_1;
	dci_1a->rv = req_pdu->dci_dl_pdu_rel8.redundancy_version_1;
	
	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_1_alloc_10MHz_FDD(lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	/* SIB1 */
	DCI1_10MHz_FDD_t *dci_1 = (DCI1_10MHz_FDD_t *) &dci->dci_pdu[0];

	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI1_10MHz_FDD_t;
	dci->L = req_pdu->dci_dl_pdu_rel8.aggregation_level;
	dci->rnti = req_pdu->dci_dl_pdu_rel8.rnti;
	dci->format = format1;
	dci->ra_flag = req_pdu->dci_dl_pdu_rel8.allocate_prach_flag;
	dci->first_CCE = req_pdu->dci_dl_pdu_rel8.cce_idx;

	/* Заполнение DCI PDU */
	dci_1->rah = req_pdu->dci_dl_pdu_rel8.resource_allocation_type;
	dci_1->rballoc = req_pdu->dci_dl_pdu_rel8.resource_block_coding;
	dci_1->TPC = req_pdu->dci_dl_pdu_rel8.tpc;
	dci_1->harq_pid = req_pdu->dci_dl_pdu_rel8.harq_process;
	dci_1->mcs = req_pdu->dci_dl_pdu_rel8.mcs_1;
	dci_1->ndi = req_pdu->dci_dl_pdu_rel8.new_data_indicator_1;
	dci_1->rv = req_pdu->dci_dl_pdu_rel8.redundancy_version_1;

	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_1_alloc_3MHz_FDD(lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	/* SIB1 */
	DCI1_3MHz_FDD_t *dci_1 = (DCI1_3MHz_FDD_t *) &dci->dci_pdu[0];

	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI1_3MHz_FDD_t;
	dci->L = req_pdu->dci_dl_pdu_rel8.aggregation_level;
	dci->rnti = req_pdu->dci_dl_pdu_rel8.rnti;
	dci->format = format1;
	dci->ra_flag = req_pdu->dci_dl_pdu_rel8.allocate_prach_flag;
	dci->first_CCE = req_pdu->dci_dl_pdu_rel8.cce_idx;

	/* Заполнение DCI PDU */
	dci_1->rah = req_pdu->dci_dl_pdu_rel8.resource_allocation_type;
	dci_1->rballoc = req_pdu->dci_dl_pdu_rel8.resource_block_coding;
	dci_1->TPC = req_pdu->dci_dl_pdu_rel8.tpc;
	dci_1->harq_pid = req_pdu->dci_dl_pdu_rel8.harq_process;
	dci_1->mcs = req_pdu->dci_dl_pdu_rel8.mcs_1;
	dci_1->ndi = req_pdu->dci_dl_pdu_rel8.new_data_indicator_1;
	dci_1->rv = req_pdu->dci_dl_pdu_rel8.redundancy_version_1;

	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_1_alloc_5MHz_FDD(lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	/* SIB1 */
	DCI1_5MHz_FDD_t *dci_1 = (DCI1_5MHz_FDD_t *) &dci->dci_pdu[0];

	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI1_5MHz_FDD_t;
	dci->L = req_pdu->dci_dl_pdu_rel8.aggregation_level;
	dci->rnti = req_pdu->dci_dl_pdu_rel8.rnti;
	dci->format = format1;
	dci->ra_flag = req_pdu->dci_dl_pdu_rel8.allocate_prach_flag;
	dci->first_CCE = req_pdu->dci_dl_pdu_rel8.cce_idx;

	/* Заполнение DCI PDU */
	dci_1->rah = req_pdu->dci_dl_pdu_rel8.resource_allocation_type;
	dci_1->rballoc = req_pdu->dci_dl_pdu_rel8.resource_block_coding;
	dci_1->TPC = req_pdu->dci_dl_pdu_rel8.tpc;
	dci_1->harq_pid = req_pdu->dci_dl_pdu_rel8.harq_process;
	dci_1->mcs = req_pdu->dci_dl_pdu_rel8.mcs_1;
	dci_1->ndi = req_pdu->dci_dl_pdu_rel8.new_data_indicator_1;
	dci_1->rv = req_pdu->dci_dl_pdu_rel8.redundancy_version_1;

	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_1_alloc_10MHz_TDD(lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	/* SIB1 */
	DCI1_10MHz_TDD_t *dci_1 = (DCI1_10MHz_TDD_t *) &dci->dci_pdu[0];

	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI1_10MHz_TDD_t;
	dci->L = req_pdu->dci_dl_pdu_rel8.aggregation_level;
	dci->rnti = req_pdu->dci_dl_pdu_rel8.rnti;
	dci->format = format1;
	dci->ra_flag = req_pdu->dci_dl_pdu_rel8.allocate_prach_flag;
	dci->first_CCE = req_pdu->dci_dl_pdu_rel8.cce_idx;

	/* Заполнение DCI PDU */
	dci_1->rah = req_pdu->dci_dl_pdu_rel8.resource_allocation_type;
	dci_1->rballoc = req_pdu->dci_dl_pdu_rel8.resource_block_coding;
	dci_1->TPC = req_pdu->dci_dl_pdu_rel8.tpc;
	dci_1->harq_pid = req_pdu->dci_dl_pdu_rel8.harq_process;
	dci_1->mcs = req_pdu->dci_dl_pdu_rel8.mcs_1;
	dci_1->ndi = req_pdu->dci_dl_pdu_rel8.new_data_indicator_1;
	dci_1->rv = req_pdu->dci_dl_pdu_rel8.redundancy_version_1;

	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_1_alloc_5MHz_TDD(lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	/* SIB1 */
	DCI1_5MHz_TDD_t *dci_1 = (DCI1_5MHz_TDD_t *) &dci->dci_pdu[0];

	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI1_5MHz_TDD_t;
	dci->L = req_pdu->dci_dl_pdu_rel8.aggregation_level;
	dci->rnti = req_pdu->dci_dl_pdu_rel8.rnti;
	dci->format = format1;
	dci->ra_flag = req_pdu->dci_dl_pdu_rel8.allocate_prach_flag;
	dci->first_CCE = req_pdu->dci_dl_pdu_rel8.cce_idx;

	/* Заполнение DCI PDU */
	dci_1->rah = req_pdu->dci_dl_pdu_rel8.resource_allocation_type;
	dci_1->rballoc = req_pdu->dci_dl_pdu_rel8.resource_block_coding;
	dci_1->TPC = req_pdu->dci_dl_pdu_rel8.tpc;
	dci_1->harq_pid = req_pdu->dci_dl_pdu_rel8.harq_process;
	dci_1->mcs = req_pdu->dci_dl_pdu_rel8.mcs_1;
	dci_1->ndi = req_pdu->dci_dl_pdu_rel8.new_data_indicator_1;
	dci_1->rv = req_pdu->dci_dl_pdu_rel8.redundancy_version_1;

	return LTE_SUCCESS;
}

static DCI_ALLOC_t last_dci0;

lte_status_t lte_fill_dci_0_alloc_10MHz_FDD(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_hi_dci0_dci_pdu *pdu)
{
	DCI0_10MHz_FDD_t *dci_0 = (DCI0_10MHz_FDD_t *) &dci->dci_pdu[0];

	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI0_10MHz_FDD_t;
	dci->L = pdu->dci_pdu_rel8.aggregation_level;
	dci->rnti = pdu->dci_pdu_rel8.rnti;
	dci->format = format0;
	dci->first_CCE = pdu->dci_pdu_rel8.cce_index;

	/* Заполнение DCI PDU */
	dci_0->type = 0;
	dci_0->hopping = pdu->dci_pdu_rel8.frequency_hopping_enabled_flag;
	dci_0->rballoc = computeRIV(enodeb->fp.LTE_N_RB_UL, pdu->dci_pdu_rel8.resource_block_start, pdu->dci_pdu_rel8.number_of_resource_block);
	dci_0->mcs = pdu->dci_pdu_rel8.mcs_1;
	dci_0->ndi = pdu->dci_pdu_rel8.new_data_indication_1;
	dci_0->TPC = pdu->dci_pdu_rel8.tpc;
	dci_0->cshift = pdu->dci_pdu_rel8.cyclic_shift_2_for_drms;
	dci_0->cqi_req = pdu->dci_pdu_rel8.cqi_csi_request;
	
	// Copy last DCI0 allocation for debug
	//memcpy(&last_dci0, dci, sizeof(DCI_ALLOC_t));

	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_0_alloc_3MHz_FDD(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_hi_dci0_dci_pdu *pdu)
{
	DCI0_3MHz_FDD_t *dci_0 = (DCI0_3MHz_FDD_t *) &dci->dci_pdu[0];

	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI0_3MHz_FDD_t;
	dci->L = pdu->dci_pdu_rel8.aggregation_level;
	dci->rnti = pdu->dci_pdu_rel8.rnti;
	dci->format = format0;
	dci->first_CCE = pdu->dci_pdu_rel8.cce_index;

	/* Заполнение DCI PDU */
	dci_0->type = 0;
	dci_0->hopping = pdu->dci_pdu_rel8.frequency_hopping_enabled_flag;
	dci_0->rballoc = computeRIV(enodeb->fp.LTE_N_RB_UL, pdu->dci_pdu_rel8.resource_block_start, pdu->dci_pdu_rel8.number_of_resource_block);
	dci_0->mcs = pdu->dci_pdu_rel8.mcs_1;
	dci_0->ndi = pdu->dci_pdu_rel8.new_data_indication_1;
	dci_0->TPC = pdu->dci_pdu_rel8.tpc;
	dci_0->cshift = pdu->dci_pdu_rel8.cyclic_shift_2_for_drms;
	dci_0->cqi_req = pdu->dci_pdu_rel8.cqi_csi_request;
	
	// Copy last DCI0 allocation for debug
	//memcpy(&last_dci0, dci, sizeof(DCI_ALLOC_t));

	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_0_alloc_5MHz_FDD(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_hi_dci0_dci_pdu *pdu)
{
	DCI0_5MHz_FDD_t *dci_0 = (DCI0_5MHz_FDD_t *) &dci->dci_pdu[0];

	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI0_5MHz_FDD_t;
	dci->L = pdu->dci_pdu_rel8.aggregation_level;
	dci->rnti = pdu->dci_pdu_rel8.rnti;
	dci->format = format0;
	dci->first_CCE = pdu->dci_pdu_rel8.cce_index;

	/* Заполнение DCI PDU */
	dci_0->type = 0;
	dci_0->hopping = pdu->dci_pdu_rel8.frequency_hopping_enabled_flag;
	dci_0->rballoc = computeRIV(enodeb->fp.LTE_N_RB_UL, pdu->dci_pdu_rel8.resource_block_start, pdu->dci_pdu_rel8.number_of_resource_block);
	dci_0->mcs = pdu->dci_pdu_rel8.mcs_1;
	dci_0->ndi = pdu->dci_pdu_rel8.new_data_indication_1;
	dci_0->TPC = pdu->dci_pdu_rel8.tpc;
	//dci_0->TPC = 2;
	dci_0->cshift = pdu->dci_pdu_rel8.cyclic_shift_2_for_drms;
	dci_0->cqi_req = pdu->dci_pdu_rel8.cqi_csi_request;
	
	// Copy last DCI0 allocation for debug
	//memcpy(&last_dci0, dci, sizeof(DCI_ALLOC_t));

	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_0_alloc_10MHz_TDD(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_hi_dci0_dci_pdu *pdu)
{
	DCI0_10MHz_TDD_1_6_t *dci_0 = (DCI0_10MHz_TDD_1_6_t *) &dci->dci_pdu[0];

	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI0_10MHz_TDD_1_6_t;
	dci->L = pdu->dci_pdu_rel8.aggregation_level;
	dci->rnti = pdu->dci_pdu_rel8.rnti;
	dci->format = format0;
	dci->first_CCE = pdu->dci_pdu_rel8.cce_index;

	/* Заполнение DCI PDU */
	dci_0->type = 0;
	dci_0->hopping = pdu->dci_pdu_rel8.frequency_hopping_enabled_flag;
	dci_0->rballoc = computeRIV(enodeb->fp.LTE_N_RB_UL, pdu->dci_pdu_rel8.resource_block_start, pdu->dci_pdu_rel8.number_of_resource_block);
	dci_0->mcs = pdu->dci_pdu_rel8.mcs_1;
	dci_0->ndi = pdu->dci_pdu_rel8.new_data_indication_1;
	dci_0->TPC = pdu->dci_pdu_rel8.tpc;
	dci_0->cshift = pdu->dci_pdu_rel8.cyclic_shift_2_for_drms;
	dci_0->cqi_req = pdu->dci_pdu_rel8.cqi_csi_request;
	
	// Copy last DCI0 allocation for debug
	//memcpy(&last_dci0, dci, sizeof(DCI_ALLOC_t));

	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_0_alloc_5MHz_TDD(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_hi_dci0_dci_pdu *pdu)
{
	DCI0_5MHz_TDD_1_6_t *dci_0 = (DCI0_5MHz_TDD_1_6_t *) &dci->dci_pdu[0];

	/* Заполнение структуры DCI */
	dci->dci_length = sizeof_DCI0_5MHz_TDD_1_6_t;
	dci->L = pdu->dci_pdu_rel8.aggregation_level;
	dci->rnti = pdu->dci_pdu_rel8.rnti;
	dci->format = format0;
	dci->first_CCE = pdu->dci_pdu_rel8.cce_index;

	/* Заполнение DCI PDU */
	dci_0->type = 0;
	dci_0->hopping = pdu->dci_pdu_rel8.frequency_hopping_enabled_flag;
	dci_0->rballoc = computeRIV(enodeb->fp.LTE_N_RB_UL, pdu->dci_pdu_rel8.resource_block_start, pdu->dci_pdu_rel8.number_of_resource_block);
	dci_0->mcs = pdu->dci_pdu_rel8.mcs_1;
	dci_0->ndi = pdu->dci_pdu_rel8.new_data_indication_1;
	dci_0->TPC = pdu->dci_pdu_rel8.tpc;
	dci_0->cshift = pdu->dci_pdu_rel8.cyclic_shift_2_for_drms;
	dci_0->cqi_req = pdu->dci_pdu_rel8.cqi_csi_request;
	
	// Copy last DCI0 allocation for debug
	//memcpy(&last_dci0, dci, sizeof(DCI_ALLOC_t));

	return LTE_SUCCESS;
}

lte_status_t lte_fill_dci_0_alloc(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_hi_dci0_dci_pdu *pdu)
{
	if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_FDD)
	{
		switch(enodeb->fp.LTE_N_RB_UL)
		{
			case 15:
				return lte_fill_dci_0_alloc_3MHz_FDD(enodeb, lte_sf, dci, pdu);
				break;
			
			case 25:
				return lte_fill_dci_0_alloc_5MHz_FDD(enodeb, lte_sf, dci, pdu);
				break;
				
			case 50:
				return lte_fill_dci_0_alloc_10MHz_FDD(enodeb, lte_sf, dci, pdu);
				break;

			default:
				ERROR(DTRX, "Skipping unsupported DCI0 allocation FDD %i RB\n", enodeb->fp.LTE_N_RB_UL);
		}
	}
	else if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_TDD)
	{
		switch(enodeb->fp.LTE_N_RB_UL)
		{
			case 25:
				return lte_fill_dci_0_alloc_5MHz_TDD(enodeb, lte_sf, dci, pdu);
				break;
				
			case 50:
				return lte_fill_dci_0_alloc_10MHz_TDD(enodeb, lte_sf, dci, pdu);
				break;

			default:
				ERROR(DTRX, "Skipping unsupported DCI0 allocation TDD %i RB\n", enodeb->fp.LTE_N_RB_UL);
		}
	}
	
	return LTE_ERR_UNSUPPORTED_DCI_PDU_FORMAT;
}

lte_status_t lte_fill_dci_1_alloc(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *pdu)
{
	if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_FDD)
	{
		switch(enodeb->fp.LTE_N_RB_DL)
		{
			case 15:
				return lte_fill_dci_1_alloc_3MHz_FDD(lte_sf, dci, pdu);
				break;
				
			case 25:
				return lte_fill_dci_1_alloc_5MHz_FDD(lte_sf, dci, pdu);
				break;
				
			case 50:
				return lte_fill_dci_1_alloc_10MHz_FDD(lte_sf, dci, pdu);
				break;

			default:
				ERROR(DTRX, "Skipping unsupported DCI1 allocation FDD %i RB\n", enodeb->fp.LTE_N_RB_DL);
		}
	}
	else if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_TDD)
	{
		switch(enodeb->fp.LTE_N_RB_DL)
		{
			case 25:
				return lte_fill_dci_1_alloc_5MHz_TDD(lte_sf, dci, pdu);
				break;
				
			case 50:
				return lte_fill_dci_1_alloc_10MHz_TDD(lte_sf, dci, pdu);
				break;

			default:
				ERROR(DTRX, "Skipping unsupported DCI1 allocation TDD %i RB\n", enodeb->fp.LTE_N_RB_DL);
		}
	}
	
	return LTE_ERR_UNSUPPORTED_DCI_PDU_FORMAT;
}

lte_status_t lte_fill_dci_1a_alloc(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *pdu)
{
	if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_FDD)
	{
		switch(enodeb->fp.LTE_N_RB_DL)
		{
			case 15:
				return lte_fill_dci_1a_alloc_3MHz_FDD(lte_sf, dci, pdu);
				break;
			
			case 25:
				return lte_fill_dci_1a_alloc_5MHz_FDD(lte_sf, dci, pdu);
				break;
				
			case 50:
				return lte_fill_dci_1a_alloc_10MHz_FDD(lte_sf, dci, pdu);
				break;

			default:
				ERROR(DTRX, "Skipping unsupported DCI1A allocation FDD %i RB\n", enodeb->fp.LTE_N_RB_DL);
		}
	}
	else if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_TDD)
	{
		switch(enodeb->fp.LTE_N_RB_DL)
		{
			case 25:
				return lte_fill_dci_1a_alloc_5MHz_TDD(lte_sf, dci, pdu);
				break;
				
			case 50:
				return lte_fill_dci_1a_alloc_10MHz_TDD(lte_sf, dci, pdu);
				break;

			default:
				ERROR(DTRX, "Skipping unsupported DCI1A allocation TDD %i RB\n", enodeb->fp.LTE_N_RB_DL);
		}
	}
	
	return LTE_ERR_UNSUPPORTED_DCI_PDU_FORMAT;
}

/**
 * Заполнение DCI из сообщения FAPI
 */
lte_status_t lte_fill_dci_alloc(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_dl_config_dci_dl_pdu *req_pdu)
{
	lte_status_t lte_status = LTE_SUCCESS;
	
	dci->dci_pdu[0] = dci->dci_pdu[1] = dci->dci_pdu[2] = dci->dci_pdu[3] = 
			dci->dci_pdu[4] = dci->dci_pdu[5] = dci->dci_pdu[6] = dci->dci_pdu[7] = 0;


	switch (req_pdu->dci_dl_pdu_rel8.dci_format)
	{
		case FAPI_DL_DCI_FORMAT_1:
			lte_status = lte_fill_dci_1_alloc(enodeb, lte_sf, dci, req_pdu);
			break;

		case FAPI_DL_DCI_FORMAT_1A:
			lte_status = lte_fill_dci_1a_alloc(enodeb, lte_sf, dci, req_pdu);
			break;

		default:
			ERROR(DTRX, "Skipping unsupported DCI format type: %i\n", req_pdu->dci_dl_pdu_rel8.dci_format);
			return LTE_ERR_UNSUPPORTED_DCI_PDU_FORMAT;
	}

	return lte_status;
}

lte_status_t lte_fill_ul_dci0_alloc(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, DCI_ALLOC_t *dci, fapi_hi_dci0_dci_pdu *pdu)
{
	lte_status_t lte_status = LTE_SUCCESS;
	
	dci->dci_pdu[0] = dci->dci_pdu[1] = dci->dci_pdu[2] = dci->dci_pdu[3] = 
				dci->dci_pdu[4] = dci->dci_pdu[5] = dci->dci_pdu[6] = dci->dci_pdu[7] = 0;
	
	switch (pdu->dci_pdu_rel8.dci_format)
	{
		case FAPI_UL_DCI_FORMAT_0:
			lte_status = lte_fill_dci_0_alloc(enodeb, lte_sf, dci, pdu);
			break;

		default:
			ERROR(DTRX, "Skipping unsupported UL DCI format type: %i\n", pdu->dci_pdu_rel8.dci_format);
			return LTE_ERR_UNSUPPORTED_DCI_PDU_FORMAT;
	}

	return lte_status;
}

#ifdef B4860

/**
 * Инициализация указателей на на карту RB для слотов 0 и 1 в зависимости от полосы
 */
static lte_status_t lte_init_rb_map_pointers(lte_enodeb_t *enodeb, lte_subframe_t *sf)
{
	switch(enodeb->fp.LTE_N_RB_DL)
	{
		case 100:
			sf->rb_map_slot0 = sf->rb_map_raw.rb_20.slot0_rbn;
			sf->rb_map_slot1 = sf->rb_map_raw.rb_20.slot1_rbn;
			break;
			
		case 75:
			sf->rb_map_slot0 = sf->rb_map_raw.rb_15.slot0_rbn;
			sf->rb_map_slot1 = sf->rb_map_raw.rb_15.slot1_rbn;
			break;
			
		case 50:
			sf->rb_map_slot0 = sf->rb_map_raw.rb_10.slot0_rbn;
			sf->rb_map_slot1 = sf->rb_map_raw.rb_10.slot1_rbn;
			break;
			
		case 25:
			sf->rb_map_slot0 = sf->rb_map_raw.rb_5.slot0_rbn;
			sf->rb_map_slot1 = sf->rb_map_raw.rb_5.slot1_rbn;
			break;
			
		case 15:
			sf->rb_map_slot0 = sf->rb_map_raw.rb_3.slot0_rbn;
			sf->rb_map_slot1 = sf->rb_map_raw.rb_3.slot1_rbn;
			break;
			
		case 6:
			sf->rb_map_slot0 = sf->rb_map_raw.rb_1_4.slot0_rbn;
			sf->rb_map_slot1 = sf->rb_map_raw.rb_1_4.slot1_rbn;			
			break;
			
		default:
			ERROR(DTRX, "Invalid number of RB: %i\n", enodeb->fp.LTE_N_RB_DL);
			return LTE_ERR_L1_INVALID_N_RB_DL;
	}
	
	return LTE_SUCCESS;
}

static lte_status_t lte_set_rb_map(lte_subframe_t *sf, uint32_t log_rb, uint32_t phys_rb)
{
	// TODO: no RB hopping between slots
	sf->rb_map_slot0[log_rb] = phys_rb;
	sf->rb_map_slot1[log_rb] = phys_rb;

	return LTE_SUCCESS;
}
#else
lte_status_t lte_set_rb_map(maple_pdsch_rb_map_t *rbmap, uint32_t log_rb, uint32_t phys_rb)
{
	uint8_t *rb_map = (uint8_t *) rbmap;
	uint32_t slot;
	uint32_t group;
	uint32_t idx;

	group = (log_rb >> 4);
	idx = (15 - (log_rb & 0x0000000f));

	rb_map[0 * 112 + (group << 4) + idx] = phys_rb;
	rb_map[1 * 112 + (group << 4) + idx] = phys_rb;

	return LTE_SUCCESS;
}
#endif
lte_status_t lte_fill_rb_map(lte_enodeb_t *enodeb, lte_subframe_t *sf, fapi_dl_config_dlsch_pdu *req_pdu, uint32_t *pRB_start, uint32_t *pnRB, uint32_t *pG)
{
	uint32_t rb_ptr = sf->rb_ptr;
	uint32_t RB_start, nRB, G;
	uint32_t rb_alloc = req_pdu->dlsch_pdu_rel8.resource_block_coding;
	uint32_t rb_mask;
	uint32_t i;

	G = 0;
	RB_start = 0;
	nRB = 0;

	/* Заполнение таблицы rb_map в соответствии с типом аалокации */
	switch (req_pdu->dlsch_pdu_rel8.resource_allocation_type)
	{
		case 0:
			/* Тип аллокации 0 */
			rb_mask = (1 << (enodeb->fp.LTE_RB_TYPE0_BITMASK_LENGTH - 1));
			
			for (i = 0; i < enodeb->fp.LTE_RB_TYPE0_BITMASK_LENGTH; i++)
			{
				if (rb_alloc & rb_mask)
				{
					/* Группа RB попадает в аллкоацию */
					uint32_t pRB;
					/* Условие pRB < LTE_N_RB_DL (проверка выхода за диапазон LTE_B_RB_DL) необходимо для
					 * корректной обработки случаев, когда количество RB не кратно размеру LTE_RBG_SIZE
					 * Например, для полосы 10МГц nRB = 50, LTE_RBG_SIZE = 3, nRBG = 7,
					 * но в последнем RBG содержится 2 RB, а не 3
					 */

					for(pRB = i * enodeb->fp.LTE_RBG_SIZE; (pRB < (i+1)*enodeb->fp.LTE_RBG_SIZE) && (pRB < enodeb->fp.LTE_N_RB_DL); pRB++)
					{
						lte_set_rb_map(sf, rb_ptr, pRB);
						G += get_num_bits_in_prb(sf->subframe_no, sf->num_pdcch_symbols, pRB, enodeb->fp.LTE_N_RB_DL,
								// FIXME: режим передачи и количество антенн должны браться из FAPI DL PDU
								enodeb->fp.LTE_N_PHYS_ANTENNAS_TX,//1,
								req_pdu->dlsch_pdu_rel8.modulation);
						rb_ptr++;
						nRB++;
					}
				}

				rb_mask >>= 1;
			}
			DBG(DTRX, "Alloc 0 rb_alloc=0x%08x G=%i n_pdcch=%i nRB=%i\n", rb_alloc, G, sf->num_pdcch_symbols, nRB);
			break;

//		case 1:
//			break;

		case 2:
			/* Тип аллокации 2, DCI 1A, 1B, 1C */
			RB_start = rb_alloc % enodeb->fapi_config.rf_config.dl_channel_bandwidth;
			nRB = (rb_alloc / enodeb->fapi_config.rf_config.dl_channel_bandwidth) + 1;

			DBG(DTRX, "Alloc 2 RB_start=%i, nRB=%i\n", RB_start, nRB);
			/* Расчет емкости блока RB и заполнение таблицы соответствия RB MAP */
			for (i = RB_start; i < RB_start + nRB; i++)
			{
				lte_set_rb_map(sf, rb_ptr, i);
				rb_ptr++;

				G += get_num_bits_in_prb(sf->subframe_no, sf->num_pdcch_symbols, i, enodeb->fp.LTE_N_RB_DL,
						enodeb->fp.LTE_N_PHYS_ANTENNAS_TX,//1,
						req_pdu->dlsch_pdu_rel8.modulation);
			}
			
			DBG(DTRX, "Alloc 2, RB_start=%i, nRB=%i G=%i\n", RB_start, nRB, G);

			break;

		default:
			ERROR(DTRX, "Unsupported DLSCH RB allocation type: %i\n", req_pdu->dlsch_pdu_rel8.resource_allocation_type);
			*pRB_start = 0;
			*pnRB = 0;
			*pG = 0;
			return LTE_ERR_UNSUPPORTED_RB_ALLOC_TYPE;
	}

	*pRB_start = sf->rb_ptr;
	*pnRB = nRB;
	*pG = G;

	sf->rb_ptr = rb_ptr;

	return LTE_SUCCESS;
}

/*
 * Заполнение транспортных блоков данными 
 */
lte_status_t lte_fill_dlsch_alloc(lte_enodeb_t *enodeb, lte_subframe_t *lte_sf, fapi_dl_config_dlsch_pdu *req_pdu, fapi_tx_request_pdu_t *tx_req_pdu,
		maple_pdsch_user_header_t *user_map,
		maple_pdsch_cw_header_t *cw_map, int32_t tb_idx)
{
//#pragma opt_level = "O0"
	
	lte_status_t lte_status = LTE_SUCCESS;
	
	// TB mapping

	/* Nir 36.212 Sec 5.1.4.1.2
	 * Kmimo = 1 (SISO mode)
	 * Kc = 1 (UE cat 1)
	 */
	uint32_t Kmimo = 1;
	uint32_t Nir = get_nsoftbits_by_ue_cat(req_pdu->dlsch_pdu_rel8.ue_category_capacity) / Kmimo / cmin(8, LTE_M_DL_HARQ);

	/* Расчет числа бит после кодирования 
	 * 36.212 Sec 5.1.4.1.2
	 * 
	 * SIB1:
	 * G = число бит в ресурсных блоках
	 * N_l = 1
	 * Q_m = 2 (BPSK == 1, QPSK == 2, QAM16 == 4, QAM64 == 6)
	 * C - количество codeblocks (при размере <= 6144, C = 1, при > 6144 - должно расчитываться 
	 */
	uint32_t G = 0;
	uint32_t tb_mod_mask = 0, cw_mod_mask = 0;

	switch (req_pdu->dlsch_pdu_rel8.modulation)
	{
		case 2:
			tb_mod_mask = PDSCH_USER_TB0_MODULATION_QPSK;
			cw_mod_mask = 0;//PDSCH_CW_MODULATION_QPSK;
			break;
		case 4:
			tb_mod_mask = PDSCH_USER_TB0_MODULATION_16QAM;
			cw_mod_mask = 1;//PDSCH_CW_MODULATION_16QAM;
			break;
		case 6:
			tb_mod_mask = PDSCH_USER_TB0_MODULATION_64QAM;
			cw_mod_mask = 2;//PDSCH_CW_MODULATION_64QAM;
			break;

		default:
			ERROR(DTRX, "Unsupported modulation %i\n", req_pdu->dlsch_pdu_rel8.modulation);
			return LTE_ERR_UNSUPPORTED_MODULATION;

	}
	
#if defined(DEBUG_PDPE_OUTPUT_QAM64)
	// Дамп сабфреймов с модуляцией QAM16/QAM64
	//if(req_pdu->dlsch_pdu_rel8.modulation == 4 || req_pdu->dlsch_pdu_rel8.modulation == 6)
	//if(req_pdu->dlsch_pdu_rel8.rnti != SI_RNTI)
	if(req_pdu->dlsch_pdu_rel8.modulation == 6)
	{
		pdsch_pdpe_dump = 1;
	}
#endif
#if defined(DEBUG_PDPE_OUTPUT_QAM16)
	if(req_pdu->dlsch_pdu_rel8.modulation == 4)
	{
		pdsch_pdpe_dump = 1;
	}
#endif	
#if defined(DEBUG_PDPE_OUTPUT_QPSK)
	if(req_pdu->dlsch_pdu_rel8.modulation == 2)
	{
		pdsch_pdpe_dump = 1;
	}
#endif
	
#ifdef DEBUG_TX_CPRI_ON_DEMAND_QAM64
	// Дамп сабфреймов с модуляцией QAM64
	if(req_pdu->dlsch_pdu_rel8.modulation == 6)
	{
		pdsch_dump_cpri_tx = 1;
		pdsch_dump_cpri_tx_count++;
	}
#endif
#ifdef DEBUG_TX_CPRI_ON_DEMAND_QAM16
	// Дамп сабфреймов с модуляцией QAM16
	if(req_pdu->dlsch_pdu_rel8.modulation == 4)
	{
		pdsch_dump_cpri_tx = 1;
		pdsch_dump_cpri_tx_count++;
	}
#endif
#ifdef DEBUG_TX_CPRI_ON_DEMAND_QPSK
	// Дамп сабфреймов с модуляцией QPSK
	if(req_pdu->dlsch_pdu_rel8.modulation == 2)
	{
		pdsch_dump_cpri_tx = 1;
		pdsch_dump_cpri_tx_count++;
	}
#endif

	
	uint32_t RB_start;
	uint32_t nRB;

	lte_status = lte_fill_rb_map(enodeb, lte_sf, req_pdu, &RB_start, &nRB, &G);

	if (lte_status == LTE_SUCCESS)
	{
		if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 1)
		{
			// SISO
			user_map->first_flags = PDSCH_USER_ANT_EN(0x01)  | PDSCH_USER_RB_START(RB_start) |  PDSCH_USER_NUM_PHYS_RB(nRB);
			user_map->second_flags = PDSCH_USER_1_CW | PDSCH_USER_TRANSMIT_SINGLE_ANT;
			user_map->third_flags =  PDSCH_USER_NIR(Nir) | 
			        PDSCH_USER_RV_IDX_TB0(req_pdu->dlsch_pdu_rel8.redundancy_version) |
			        tb_mod_mask |
			        PDSCH_USER_TB0_NL_1;
		}
		else if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2)
		{
			// 2 антенны
			user_map->first_flags = PDSCH_USER_ANT_EN(0x03) | PDSCH_USER_RB_START(RB_start) |  PDSCH_USER_NUM_PHYS_RB(nRB);
			user_map->second_flags = PDSCH_USER_1_CW | PDSCH_USER_TRANSMIT_DIVERSITY | PDSCH_USER_2_ANT_PORTS_TRANS_DIVERSITY;
			user_map->third_flags = PDSCH_USER_NIR(Nir) | 
			        PDSCH_USER_RV_IDX_TB0(req_pdu->dlsch_pdu_rel8.redundancy_version) |
			        tb_mod_mask |
			        PDSCH_USER_TB0_NL_1;
		}
		else if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 4)
		{
			// 2 антенны
			user_map->first_flags = PDSCH_USER_ANT_EN(0x0f) | PDSCH_USER_RB_START(RB_start) |  PDSCH_USER_NUM_PHYS_RB(nRB);;
			user_map->second_flags = PDSCH_USER_1_CW | PDSCH_USER_TRANSMIT_DIVERSITY | PDSCH_USER_4_ANT_PORTS_TRANS_DIVERSITY;
			user_map->third_flags = PDSCH_USER_NIR(Nir) | 
			        PDSCH_USER_RV_IDX_TB0(req_pdu->dlsch_pdu_rel8.redundancy_version) |
			        tb_mod_mask |
			        PDSCH_USER_TB0_NL_1;
		}
		
		/* Маска символов в слотах для применения амплитуды A (бит == 0) или B (бит == 1) */
		/* TODO: добавить поддержку EXT_CP, MBSF 
		 * Сейчас поддерживается только 1 режим в обоих слотах:
		 * 1, 2, 3, 5, 6 - Gain RhoA
		 * 0, 4 - Gain RhoB (маска 0x11) 
		 */
		user_map->second_flags |= PDSCH_USER_OSPMS(0x11) | PDSCH_USER_OSPMF(0x11);
		
		int32_t first_slot_symbols_mask = lte_sf->num_pdcch_symbols > 0 ? ((1 << lte_sf->num_pdcch_symbols) - 1) : 0;
		
		if (tx_req_pdu->tbs[0].tb_tag)
		{
			os_virt_ptr tb_data_virt;
			
			user_map->tb_size[0] = tx_req_pdu->tbs[0].tb_length;
			osMmuDataPhysToVirtManual((os_const_phys_ptr)tx_req_pdu->tbs[0].tb_data, &tb_data_virt);
			user_map->tb_params[0].tb_input = tb_data_virt;
			user_map->tb_params[0].num_out_bits = PDSCH_SOSD(first_slot_symbols_mask) | G;
			//osMmuDataVirtToPhys(&sib1_test, (void **) &tb_map->tb_input);
		}

		/* Заполнение дескриптора CW */
		/* Расчет c_init */
		/* this is c_init in 36.211 Sec 6.3.1 */
		// x2 = (dlsch->rnti<<14) + (q<<13) + ((Ns>>1)<<9) + frame_parms->Nid_cell;
		uint32_t q = 0;
		uint32_t c_init = (req_pdu->dlsch_pdu_rel8.rnti << 14) + (q << 13) + (lte_sf->subframe_no << 9) + enodeb->N_id_cell;

		cw_map[0]->cw_init = c_init;
		cw_map[0]->cw_mod_type    =  cw_mod_mask;
		cw_map[0]->cw_trgt_layer  =  0x01;

		cw_map[1]->cw_init = c_init;
		cw_map[1]->cw_mod_type    =  cw_mod_mask;
		cw_map[1]->cw_trgt_layer  =  0x01;

#if 0
		// Old code
		cw_map[0]->cw_valid = 1;

		cw_map[0].rb_size = nRB;
		cw_map->rb_ref = RB_start;

		cw_map->cw_flags = PDSCH_SINGLE_ANTENNA | cw_mod_mask;
		cw_map->ant_flags = PDSCH_CW_ANTA_EN | CW_WB_EVEN_1 | CW_WB_ODD_1 | CW_WA_EVEN_1 | CW_WA_ODD_1;
		cw_map->ofdm_map_slots = 0;//PDSCH_CW_OSPM_FIRST(0x3f) | PDSCH_CW_OSPM_SECOND(0x3f);
#endif	
		/* Определние усиления в зависимости от типа модуляции
		 * Это необходимо, т.к. модулированные символы для разных видов модуляции
		 * имеют различные значения (в формате 4q12)
		 * 
		 * QPSK: -1, 1
		 * QAM16: -3, -1, 1, 3
		 * QAM64: -7, -5, -3, -1, 1, 3, 5, 7
		 * 
		 * Здесь же производится коррекция с учетом параметров P-A, P-B
		 */
		
		/* Проверка параметра P-A в PDU на всякий случай */
		if(req_pdu->dlsch_pdu_rel8.pa > 7)
			req_pdu->dlsch_pdu_rel8.pa = 7;
		/* Линейная таблица P-A */
		/*double pa_values[8]={-6.0,-4.77,-3.0,-1.77,0.0,1.0,2.0,3.0}; */
		static int32_t pa_lin_m[8] = { 16422, 18920, 23197, 26726, 32767, 36765, 41251, 46284};
		static int32_t pa_lin_e[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
		/* double ratioPB[2][4]={{ 1.0,4.0/5.0,3.0/5.0,2.0/5.0},
          { 5.0/4.0,1.0,3.0/4.0,1.0/2.0}}; */
		/*double ratioPB[2][4]={{ 0.00000,  -0.96910,  -2.21849,  -3.97940}, //in db
                      { 0.96910,   0.00000,  -1.24939,  -3.01030}};*/
		static int32_t pb_lin_m[2][4] = {{ 32767, 29307, 25381, 20723 }, { 36634, 32767, 28377, 23169 }};
		static int32_t pb_lin_e[2][4] = {{ 0, 0, 0, 0}, {0, 0, 0, 0}};
		
		int32_t pb_e = (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 1) ? pb_lin_e[0][enodeb->fapi_config.subframe_config.pb] :
				pb_lin_e[1][enodeb->fapi_config.subframe_config.pb];

		int32_t pb_m = (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 1) ? pb_lin_m[0][enodeb->fapi_config.subframe_config.pb] :
				pb_lin_m[1][enodeb->fapi_config.subframe_config.pb];
		
		int32_t mod_m, mod_e;
		
		switch (req_pdu->dlsch_pdu_rel8.modulation)
		{
			case 4:
				/* QAM 16 */
				mod_e = enodeb->fp.LTE_CW_GAIN_E_QAM16;
				mod_m = enodeb->fp.LTE_CW_GAIN_M_QAM16;
				break;
			case 6:
				/* QAM64 */
				mod_e = enodeb->fp.LTE_CW_GAIN_E_QAM64;
				mod_m = enodeb->fp.LTE_CW_GAIN_M_QAM64;
				break;
			default:
			case 2:
				/* QPSK */
				mod_e = enodeb->fp.LTE_CW_GAIN_E_QPSK;
				mod_m = enodeb->fp.LTE_CW_GAIN_M_QPSK;
				break;				
		}

		cw_map[0]->gain_a_exp = mod_e + pa_lin_e[req_pdu->dlsch_pdu_rel8.pa];
		cw_map[0]->gain_a_mantisa = (mod_m * pa_lin_m[req_pdu->dlsch_pdu_rel8.pa]) >> 15;
		cw_map[0]->gain_b_exp = cw_map[0]->gain_a_exp + pb_e;
		cw_map[0]->gain_b_mantisa = (cw_map[0]->gain_a_mantisa * pb_m) >> 15;
		
		/*
		DBG(DTRX, "ga_m %i ga_e %i gb_m %i gb_e %i\n", cw_map[0]->gain_a_mantisa, cw_map[0]->gain_a_exp,
				cw_map[0]->gain_b_mantisa, cw_map[0]->gain_b_exp);
		*/
	}

	return lte_status;
}

fapi_tx_request_pdu_t *lte_get_tx_req_pdu_by_index(fapi_tx_request_body_t *tx_req, uint32_t pdu_index)
{
	int32_t i;

	for (i = 0; i < tx_req->number_of_pdus; i++)
	{
		if (tx_req->tx_pdu_list[i].pdu_index == pdu_index)
			return &tx_req->tx_pdu_list[i];
	}

	return NULL;
}

/**
 * Процедура заполнения слота данными, режим FDD
 * @param trx
 * @param lte_slot
 */

static lte_status_t lte_trx_map_slot_fdd(lte_trx_t *trx, lte_subframe_t *lte_sf, fapi_dl_config_request_body_t *dl_cfg, fapi_tx_request_body_t *tx_req,
		fapi_hi_dci0_request_body_t *hi_dci0)
{
	lte_enodeb_t *enodeb;
//	uint8_t *pbch_pdu;
	uint32_t pdcch_alloc;
	lte_status_t lte_status;
	uint32_t tb_alloc = 0;

	OS_ASSERT_COND(trx != NULL);
	OS_ASSERT_COND(lte_sf != NULL);

	enodeb = trx->enodeb;
	OS_ASSERT_COND(enodeb != NULL);

//	pbch_pdu = &enodeb->pbch_pdu[0];

	/* Заполнение 0-го символа сигналами CSRS, антенна 0 */
	memcpy(&lte_sf->pdcch_syms[0], enodeb->refsigs->lte_csrs_sym0[lte_sf->subframe_no][0], 
			enodeb->fp.LTE_N_RE * sizeof(Complex16));
	
	if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2)
	{
		/* Заполнение CSRS, антенна 1
		 * Символы PDCCH идут в порядке 
		 * PDDCH_0_ANT_0 (offset 0)
		 * PDCCH_0_ANT_1 (offset LTE_N_RE)
		 * ...
		 * PDCCH0_ANT_x
		 * PDCCH1_ANT_0
		 * ...
		 */
		memcpy(&lte_sf->pdcch_syms[enodeb->fp.LTE_N_RE], enodeb->refsigs->lte_csrs_sym0[lte_sf->subframe_no][1], 
				enodeb->fp.LTE_N_RE * sizeof(Complex16));
	}

	/* 
	 * Заполнение дескриптора CRS
	 */
	lte_sf->job.pdsch_job.third_flags |= PDSCH_BD_CS_EN;
	
	// FIXME: MAPLE-B3
	//lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_CS_RS_HEADER_ADDRESS] = &enodeb->refsigs->csrs;

	/*
	 * Заполнение PSS/SSS в сабфреймах 0 и 5
	 */
	if (lte_sf->subframe_no == 0 || lte_sf->subframe_no == 5)
	{
		liblte_phy_map_pss(trx, lte_sf);
		liblte_phy_map_sss(trx, lte_sf);
	}
#if 0
	/* Обработка сабфрейма 0 */
	if (lte_sf->subframe_no == 0)
	{
		/*
		 * Формирование данных канала PBCH
		 */

		/*
		 * Заполнение MIB производится с периодом 40мс (фреймы 0, 4, 8, ...)
		 */
		if ((lte_sf->frame_no & 3) == 0)
		{
			pbch_pdu[0] = 0;
			
			switch (enodeb->fp.LTE_N_RB_DL)
			{
				case 6:
					pbch_pdu[0] = (0 << 5);
					break;
				case 15:
					pbch_pdu[0] = (1 << 5);
					break;
				case 25:
					pbch_pdu[0] = (2 << 5);
					break;
				case 50:
					pbch_pdu[0] = (3 << 5);
					break;
				case 100:
					pbch_pdu[0] = (4 << 5);
					break;
				default:
					pbch_pdu[0] = (2 << 5);
					break;
			}

			pbch_pdu[0] |= ((enodeb->fapi_config.phich_config.phich_duration << 4) & 0x10);

			switch (enodeb->fapi_config.phich_config.phich_resource)
			{
				case LTE_PHICH_RESOURCE_ONESIXTH:
					pbch_pdu[0] |= (0 << 2);
					break;
				case LTE_PHICH_RESOURCE_HALF:
					pbch_pdu[0] |= (1 << 2);
					break;
				case LTE_PHICH_RESOURCE_ONE:
					pbch_pdu[0] |= (2 << 2);
					break;
				case LTE_PHICH_RESOURCE_TWO:
					pbch_pdu[0] |= (3 << 2);
					break;
				default:
					pbch_pdu[0] |= (0 << 2);
			}

			pbch_pdu[0] |= ((lte_sf->frame_no >> 8) & 0x3);
			pbch_pdu[1] = lte_sf->frame_no & 0xfc;
			pbch_pdu[2] = 0;
		}
#ifdef DEBUG_PBCH
		INFO(DTRX, "PBCH PDU: %02x %02x %02x\n", pbch_pdu[0], pbch_pdu[1], pbch_pdu[2]);
#endif
		liblte_generate_pbch(trx, lte_sf, LTE_PBCH_AMP);

		// Установка флага PBCH
		// Структура PBCH находится в user_map[0] и cw_map[0]
		lte_sf->job.pdsch_job.third_flags |= PDSCH_BD_PBCH_EN;
		
		// Номер блока PBCH
		switch(lte_sf->frame_no & 3)
		{
			case 0:
				lte_sf->job.pdsch_job.second_flags |= PDSCH_PBCH_FIRST_Q;
				break;
			case 1:
				lte_sf->job.pdsch_job.second_flags |= PDSCH_PBCH_SECOND_Q;
				break;
			case 2:
				lte_sf->job.pdsch_job.second_flags |= PDSCH_PBCH_THIRD_Q;
				break;
			case 3:
				lte_sf->job.pdsch_job.second_flags |= PDSCH_PBCH_FOURTH_Q;
				break;
		}
		
		tb_alloc++;
	}
#else
	/* Обработка сабфрейма 0 */
	if (lte_sf->subframe_no == 0)
	{
		/*
		 * Заполнение MIB производится с периодом 40мс (фреймы 0, 4, 8, ...)
		 */
		if ((lte_sf->frame_no & 3) == 0)
		{
			/* Поиск BCH PDU */
			if (dl_cfg != NULL)
			{
				int32_t dl_cfg_i;
				
				/* Сначал обработка BCH, т.к. это должен быть TB0 */
				for (dl_cfg_i = dl_cfg->number_dci; dl_cfg_i < dl_cfg->number_dci + dl_cfg->number_pdu; dl_cfg_i++)
				{
					fapi_dl_config_request_pdu_t *req_pdu = &dl_cfg->dl_config_pdu_list[dl_cfg_i];

					switch (req_pdu->pdu_type)
					{
						case FAPI_DL_CONFIG_BCH_PDU_TYPE:
						{
							fapi_dl_config_bch_pdu *bch_pdu = &req_pdu->bch_pdu;
							fapi_tx_request_pdu_t *tx_req_pdu = lte_get_tx_req_pdu_by_index(tx_req, bch_pdu->bch_pdu_rel8.pdu_index);
							
							if (tx_req_pdu != NULL)
							{
								// Сохранение новго значения BCH PDU и запуск кодирования
								os_virt_ptr tb_data_virt;								
								osMmuDataPhysToVirtManual((os_const_phys_ptr)tx_req_pdu->tbs[0].tb_data, &tb_data_virt);

								uint8_t *pbch_pdu = (uint8_t *)tb_data_virt;
								enodeb->pbch_pdu[0] = pbch_pdu[0];
								enodeb->pbch_pdu[1] = pbch_pdu[1];
								enodeb->pbch_pdu[2] = pbch_pdu[2];
								
								// Установка флага корректного PBCH PDU
								enodeb->pbch_pdu_ok = 1;
#ifdef DEBUG_PBCH
								INFO(DTRX, "Got new PBCH PDU: %02x %02x %02x\n", pbch_pdu[0], pbch_pdu[1], pbch_pdu[2]);
#endif
							}
							else
							{
								ERROR(DTRX, "Unable to find corresponding TX.request %i\n", bch_pdu->bch_pdu_rel8.pdu_index);
							}
						}
							break;
					}
					
					// Если найден BCH PDU, то прерываем поиск
					if(enodeb->pbch_pdu_ok)
						break;
				}
			}
		}
		
		/* Формирование BCH */
		if(enodeb->pbch_pdu_ok)
		{
			liblte_generate_pbch(trx, lte_sf);
			tb_alloc++;
			
			if((lte_sf->frame_no & 3) == 3)
			{
				// Сброс флага корректного BCH
				enodeb->pbch_pdu_ok = 0;
			}
		}
	}
#endif
	
	/* PDCCH & PDSCH allocation */
	pdcch_alloc = 0;

	/* Формирование DCI PDCCH */
	if (dl_cfg != NULL)
	{
		int32_t dl_cfg_i;

		for (dl_cfg_i = 0; dl_cfg_i < dl_cfg->number_dci; dl_cfg_i++)
		{
			fapi_dl_config_request_pdu_t *req_pdu = &dl_cfg->dl_config_pdu_list[dl_cfg_i];

			switch (req_pdu->pdu_type)
			{
				case FAPI_DL_CONFIG_DCI_DL_PDU_TYPE:
					lte_status = lte_fill_dci_alloc(enodeb, lte_sf, &g_dci_pdu.dci_alloc[pdcch_alloc], &req_pdu->dci_dl_pdu);
					if (lte_status == LTE_SUCCESS)
					{
						pdcch_alloc++;
					}
					break;

				default:
					ERROR(DTRX, "Skipping unsupported DCI PDU type: %i\n", req_pdu->pdu_type);
					break;
			}

			if (pdcch_alloc >= NUM_DCI_MAX)
			{
				ERROR(DTRX, "Maximum number of DCI PDUs reached %i (requested %i)\n", NUM_DCI_MAX, dl_cfg->number_dci);
				break;
			}
		}
		
		/* Обработка соответствующего TX.req */
		if (tx_req != NULL)
		{			
			for (dl_cfg_i = dl_cfg->number_dci; dl_cfg_i < dl_cfg->number_dci + dl_cfg->number_pdu; dl_cfg_i++)
			{
				fapi_dl_config_request_pdu_t *req_pdu = &dl_cfg->dl_config_pdu_list[dl_cfg_i];

				switch (req_pdu->pdu_type)
				{
					case FAPI_DL_CONFIG_DLSCH_PDU_TYPE:
					{
						fapi_dl_config_dlsch_pdu *dlsch_pdu = &req_pdu->dlsch_pdu;
						fapi_tx_request_pdu_t *tx_req_pdu = lte_get_tx_req_pdu_by_index(tx_req, dlsch_pdu->dlsch_pdu_rel8.pdu_index);

						if (tx_req_pdu != NULL)
						{
							maple_pdsch_user_header_t *user_map = &lte_sf->user_map[tb_alloc];
							maple_pdsch_cw_header_t *cw_map = &lte_sf->cw_map[tb_alloc];

							lte_status = lte_fill_dlsch_alloc(enodeb, lte_sf, dlsch_pdu, tx_req_pdu, user_map, cw_map, tb_alloc);

							if (lte_status == LTE_SUCCESS)
							{
								tb_alloc++;
							}
						}
						else
						{
							ERROR(DTRX, "Unable to find corresponding TX.request %i\n", dlsch_pdu->dlsch_pdu_rel8.pdu_index);
						}
					}
						break;
						
					case FAPI_DL_CONFIG_BCH_PDU_TYPE:
						// Пропускаем BCH PDU
						break;
						
					default:
						ERROR(DTRX, "Skipping unsupported PDU type: %i\n", req_pdu->pdu_type);
						break;
				}

				if (tb_alloc >= NUM_DCI_MAX)
				{
					ERROR(DTRX, "Maximum number of TB PDUs reached %i (requested %i)\n", NUM_DCI_MAX, dl_cfg->number_dci);
					break;
				}
			}
		}
		else
		{
			/*
			 * Нет TX.request
			 * Ситуация вполен себе нормальная
			 */
		}
	}
	else 
	{
		/* Вообще-то это ошибка
		 * DL_CONFIG.request ДОЛЖЕН приходить на каждый сабфрейм
		 * 
		 * Но пока ничего не делаем, подумаем
		 */
	}
	
	// pdcch_alloc может быть меньше tb_alloc, т.к. в tb_alloc может передаваться PBCH
	// А вот меньше быть не может никогда
	if(pdcch_alloc > tb_alloc)
	{
		//ERROR(DTRX, "pdcch_alloc %i greater from tb_alloc %i\n", pdcch_alloc, tb_alloc);
	}

	/* Заполнение DCI0 */
	if (hi_dci0 != NULL)
	{
		int32_t hi_dci0_i;

		for (hi_dci0_i = 0; hi_dci0_i < hi_dci0->number_of_dci; hi_dci0_i++)
		{
			fapi_hi_dci0_request_pdu_t *dci0_pdu = &hi_dci0->hi_dci0_pdu_list[hi_dci0_i];

			switch (dci0_pdu->pdu_type)
			{
				case FAPI_HI_DCI0_DCI_PDU_TYPE:
					
//					pdsch_dump_cpri_tx = 1;
					
					lte_status = lte_fill_ul_dci0_alloc(enodeb, lte_sf, &g_dci_pdu.dci_alloc[pdcch_alloc], &dci0_pdu->dci_pdu);
					if (lte_status == LTE_SUCCESS)
					{
						pdcch_alloc++;
					}
					break;

				default:
					ERROR(DTRX, "Skipping unsupported UL DCI PDU type: %i\n", dci0_pdu->pdu_type);
					break;
			}

			if (pdcch_alloc >= NUM_DCI_MAX)
			{
				ERROR(DTRX, "Maximum number of DCI PDUs reached %i (UL DCI requested %i)\n", NUM_DCI_MAX, hi_dci0->number_of_dci);
				break;
			}
		}
	}

	/* Формирование PHICH */
	if (hi_dci0 != NULL)
	{
		liblte_generate_phich_top(enodeb, lte_sf, hi_dci0);
	}
	
	/* Формирование PCFICH */
	if(lte_sf->num_pdcch_symbols > 0)
	{
		liblte_generate_pcfich(lte_sf->num_pdcch_symbols, enodeb->fp.LTE_PCFICH_GAIN, enodeb, lte_sf);
	}

	/* Формирование PDCCH
	 * Происходит всегда, не зависимо от значения pdcch_alloc
	 */
	if (pdcch_alloc == 0)
	{
		//liblte_generate_dci_top(0, NULL, 0, LTE_PDCCH_AMP, enodeb, lte_sf);

		//liblte_generate_pcfich(num_pdcch_symbols, 32760, enodeb, lte_slot);
	}
	else if (pdcch_alloc > 0)
	{
		liblte_generate_dci_top(pdcch_alloc, (DCI_ALLOC_t *) &g_dci_pdu.dci_alloc, 0, enodeb->fp.LTE_PDCCH_GAIN, enodeb, lte_sf);

		//liblte_generate_pcfich(num_pdcch_symbols, 32760, enodeb, lte_slot);
	}
	
	lte_sf->num_common_dci = pdcch_alloc;
	lte_sf->num_ue_dci = 0;
	lte_sf->num_tb_alloc = tb_alloc;

	return LTE_SUCCESS;
}

/**
 * Процедура заполнения слота данными, режим TDD
 * @param trx
 * @param lte_slot
 */

static lte_status_t lte_trx_map_slot_tdd(lte_trx_t *trx, lte_subframe_t *lte_sf, fapi_dl_config_request_body_t *dl_cfg, fapi_tx_request_body_t *tx_req,
		fapi_hi_dci0_request_body_t *hi_dci0)
{
	lte_enodeb_t *enodeb;
	uint8_t *pbch_pdu;
	uint32_t pdcch_alloc;
	lte_status_t lte_status;

	OS_ASSERT_COND(trx != NULL);
	OS_ASSERT_COND(lte_sf != NULL);

	enodeb = trx->enodeb;
	OS_ASSERT_COND(enodeb != NULL);

	/* Заполнение 0-го символа сигналами CSRS */
	if(liblte_is_dl_subframe(enodeb, lte_sf->subframe_no) || liblte_is_spec_subframe(enodeb, lte_sf->subframe_no))
	{
		memcpy(lte_sf->pdcch_syms, enodeb->refsigs->lte_csrs_sym0[lte_sf->subframe_no], sizeof(Complex16) * enodeb->fp.LTE_N_RE);
	}
	else
	{
		memset(lte_sf->pdcch_syms, 0, sizeof(Complex16) * enodeb->fp.LTE_N_RE);
		
		lte_sf->num_common_dci = 0;
		lte_sf->num_ue_dci = 0;
		lte_sf->num_tb_alloc = 0;
		
		return LTE_SUCCESS;
	}

	/* 
	 * Заполнение дескриптора CRS
	 */
	lte_sf->job.pdsch_job.third_flags |= PDSCH_BD_CS_EN;
	
	// FIXME: MAPLE-B3
	//lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_CS_RS_HEADER_ADDRESS] = &enodeb->refsigs->csrs;

	/*
	 * Заполнение PSS в сабфреймах 1 и 6
	 */
	if (lte_sf->subframe_no == 1 || lte_sf->subframe_no == 6)
	{
		liblte_phy_map_pss(trx, lte_sf);
	}

	/*
	 * Заполнение SSS в сабфреймах 0 и 5
	 */
	if (lte_sf->subframe_no == 0 || lte_sf->subframe_no == 5)
	{
		liblte_phy_map_sss(trx, lte_sf);
	}

	/* Обработка сабфрейма 0 */
	if (lte_sf->subframe_no == 0)
	{
		/*
		 * Формирование данных канала PBCH
		 */

		/*
		 * Заполнение MIB производится с периодом 40мс (фреймы 0, 4, 8, ...)
		 */
		if ((lte_sf->frame_no & 3) == 0)
		{
			pbch_pdu = &enodeb->pbch_pdu[0];
			
			((uint8_t*) pbch_pdu)[2] = 0;
			switch (enodeb->fp.LTE_N_RB_DL)
			{
				case 6:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0x1f) | (0 << 5);
					break;
				case 15:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0x1f) | (1 << 5);
					break;
				case 25:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0x1f) | (2 << 5);
					break;
				case 50:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0x1f) | (3 << 5);
					break;
				case 100:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0x1f) | (4 << 5);
					break;
				default:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0x1f) | (2 << 5);
					break;
			}

			((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0xef) | ((enodeb->fapi_config.phich_config.phich_duration << 4) & 0x10);

			switch (enodeb->fapi_config.phich_config.phich_resource)
			{
				case LTE_PHICH_RESOURCE_ONESIXTH:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0xf3) | (0 << 2);
					break;
				case LTE_PHICH_RESOURCE_HALF:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0xf3) | (1 << 2);
					break;
				case LTE_PHICH_RESOURCE_ONE:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0xf3) | (2 << 2);
					break;
				case LTE_PHICH_RESOURCE_TWO:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0xf3) | (3 << 2);
					break;
				default:
					((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0xf3) | (0 << 2);

			}

			((uint8_t*) pbch_pdu)[2] = (((uint8_t*) pbch_pdu)[2] & 0xfc) | ((lte_sf->frame_no >> 8) & 0x3);
			((uint8_t*) pbch_pdu)[1] = lte_sf->frame_no & 0xfc;
			((uint8_t*) pbch_pdu)[0] = 0;
		}
#ifdef DEBUG_PBCH
		DBG(DBTS, "PBCH PDU: %02x %02x %02x\n", pbch_pdu[2], pbch_pdu[1], pbch_pdu[0]);
#endif
		liblte_generate_pbch(trx, lte_sf);

		// FIXME: MAPLE-B3
		//sweep_cache((uint32_t) &enodeb->refsigs->pbch_syms, sizeof(enodeb->refsigs->pbch_syms), CACHE_FLUSH);
#if 1
		lte_sf->job.pdsch_job.third_flags |= PDSCH_BD_PBCH_EN;
		// FIXME: MAPLE-B3
		//lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_PBCH_DATA_ADDRESS] = &enodeb->refsigs->pbch_syms;
#endif
	}

	/* PDCCH & PDSCH allocation */
	pdcch_alloc = 0;
	uint32_t tb_alloc = 0;

	/* Формирование DCI PDCCH */
	if (dl_cfg != NULL)
	{
		int32_t dl_cfg_i;

		for (dl_cfg_i = 0; dl_cfg_i < dl_cfg->number_dci; dl_cfg_i++)
		{
			fapi_dl_config_request_pdu_t *req_pdu = &dl_cfg->dl_config_pdu_list[dl_cfg_i];

			switch (req_pdu->pdu_type)
			{
				case FAPI_DL_CONFIG_DCI_DL_PDU_TYPE:
					lte_status = lte_fill_dci_alloc(enodeb, lte_sf, &g_dci_pdu.dci_alloc[pdcch_alloc], &req_pdu->dci_dl_pdu);
					if (lte_status == LTE_SUCCESS)
					{
						pdcch_alloc++;
					}
					break;

				default:
					ERROR(DTRX, "Skipping unsupported DCI PDU type: %i\n", req_pdu->pdu_type);
					break;
			}

			if (pdcch_alloc >= NUM_DCI_MAX)
			{
				ERROR(DTRX, "Maximum number of DCI PDUs reached %i (requested %i)\n", NUM_DCI_MAX, dl_cfg->number_dci);
				break;
			}
		}

		/* Обработка соответствующего TX.req */
		if (tx_req != NULL)
		{

			for (dl_cfg_i = dl_cfg->number_dci; dl_cfg_i < dl_cfg->number_dci + dl_cfg->number_pdu; dl_cfg_i++)
			{
				fapi_dl_config_request_pdu_t *req_pdu = &dl_cfg->dl_config_pdu_list[dl_cfg_i];

				switch (req_pdu->pdu_type)
				{
					case FAPI_DL_CONFIG_DLSCH_PDU_TYPE:
					{
						fapi_dl_config_dlsch_pdu *dlsch_pdu = &req_pdu->dlsch_pdu;
						fapi_tx_request_pdu_t *tx_req_pdu = lte_get_tx_req_pdu_by_index(tx_req, dlsch_pdu->dlsch_pdu_rel8.pdu_index);

						if (tx_req_pdu != NULL)
						{
							maple_pdsch_user_header_t *user_map = &lte_sf->user_map[tb_alloc];
							maple_pdsch_cw_header_t *cw_map = &lte_sf->cw_map[tb_alloc];

							lte_status = lte_fill_dlsch_alloc(enodeb, lte_sf, dlsch_pdu, tx_req_pdu, user_map, cw_map, tb_alloc);

							if (lte_status == LTE_SUCCESS)
							{
								tb_alloc++;
							}
						}
						else
						{
							ERROR(DTRX, "Unable to find corresponding TX.request %i\n", dlsch_pdu->dlsch_pdu_rel8.pdu_index);
						}
					}
						break;

					default:
						ERROR(DTRX, "Skipping unsupported PDU type: %i\n", req_pdu->pdu_type);
						break;
				}

				if (tb_alloc >= NUM_DCI_MAX)
				{
					ERROR(DTRX, "Maximum number of TB PDUs reached %i (requested %i)\n", NUM_DCI_MAX, dl_cfg->number_dci);
					break;
				}
			}
		}
		else
		{
			/*
			 * Нет TX.request
			 * Ситуация вполен себе номральная
			 */
		}
	}
	else 
	{
		/* Вообще-то это ошибка
		 * DL_CONFIG.request ДОЛЖЕН приходить на каждый сабфрейм
		 * 
		 * Но пока ничего не делаем, подумаем
		 */
	}
	
	if(pdcch_alloc != tb_alloc)
	{
		ERROR(DTRX, "pdcch_alloc %i differs from tb_alloc %i\n", pdcch_alloc, tb_alloc);
	}

	/* Заполнение DCI0 */
	if (hi_dci0 != NULL)
	{
		int32_t hi_dci0_i;

		for (hi_dci0_i = 0; hi_dci0_i < hi_dci0->number_of_dci; hi_dci0_i++)
		{
			fapi_hi_dci0_request_pdu_t *dci0_pdu = &hi_dci0->hi_dci0_pdu_list[hi_dci0_i];

			switch (dci0_pdu->pdu_type)
			{
				case FAPI_HI_DCI0_DCI_PDU_TYPE:
					lte_status = lte_fill_ul_dci0_alloc(enodeb, lte_sf, &g_dci_pdu.dci_alloc[pdcch_alloc], &dci0_pdu->dci_pdu);
					if (lte_status == LTE_SUCCESS)
					{
						pdcch_alloc++;
					}
					break;

				default:
					ERROR(DTRX, "Skipping unsupported UL DCI PDU type: %i\n", dci0_pdu->pdu_type);
					break;
			}

			if (pdcch_alloc >= NUM_DCI_MAX)
			{
				ERROR(DTRX, "Maximum number of DCI PDUs reached %i (UL DCI requested %i)\n", NUM_DCI_MAX, hi_dci0->number_of_dci);
				break;
			}
		}
	}

	/* Формирование PHICH */
	if (hi_dci0 != NULL)
	{
		liblte_generate_phich_top(enodeb, lte_sf, hi_dci0);
	}

	/* Формирование PDCCH
	 * Происходит всегда, не зависимо от значения pdcch_alloc
	 */
	if (pdcch_alloc == 0)
	{
		liblte_generate_dci_top(0, NULL, 0, enodeb->fp.LTE_PDCCH_GAIN, enodeb, lte_sf);

		//liblte_generate_pcfich(num_pdcch_symbols, 32760, enodeb, lte_slot);
	}
	else if (pdcch_alloc > 0)
	{
		liblte_generate_dci_top(pdcch_alloc, (DCI_ALLOC_t *) &g_dci_pdu.dci_alloc, 0, enodeb->fp.LTE_PDCCH_GAIN, enodeb, lte_sf);

		//liblte_generate_pcfich(num_pdcch_symbols, 32760, enodeb, lte_slot);
	}
	
	lte_sf->num_common_dci = pdcch_alloc;
	lte_sf->num_ue_dci = 0;
	lte_sf->num_tb_alloc = tb_alloc;

	return LTE_SUCCESS;
}

static lte_status_t trx_prepare_subframe_data(lte_enodeb_t *enodeb, lte_subframe_t *sf, fapi_dl_config_request_body_t **dl_cfg_req, fapi_tx_request_body_t **tx_req,
		fapi_hi_dci0_request_body_t **hi_dci0_req)
{
	fapi_dl_config_request_t *t_dl_cfg_req;
	fapi_tx_request_t *t_tx_req;
	fapi_hi_dci0_request_t *t_hi_dci0_req;
	int32_t found = 0;
	uint32_t sfn_sf;
	uint32_t req_frame_no;
	uint32_t req_subframe_no;

	*dl_cfg_req = NULL;
	*tx_req = NULL;
	*hi_dci0_req = NULL;

	sfn_sf = (sf->frame_no << 4) | sf->subframe_no;

	found = 0;
	
	t_dl_cfg_req = enodeb->trx.fapi_dl_config[sf->subframe_no];
	enodeb->trx.fapi_dl_config[sf->subframe_no] = NULL;
	
	if(t_dl_cfg_req != NULL)
	{

		req_frame_no = t_dl_cfg_req->sfn_sf >> 4;
		req_subframe_no = t_dl_cfg_req->sfn_sf & 0x0f;
		
		if (req_frame_no == sf->frame_no && req_subframe_no == sf->subframe_no)
		{
			/* Нашли нужный пакет */
			found = 1;
		}
		else if ((req_frame_no < sf->frame_no) || ((req_frame_no == sf->frame_no) && (req_subframe_no < sf->subframe_no)))
		{
			/* Пакет из прошлого, удаляем из очереди и смотрим следующий пакет */
			WARN(DTRX, "Late DL_CONFIG.req for %i:%i at %i:%i\n", req_frame_no, req_subframe_no, sf->frame_no, sf->subframe_no);
		}
		else
		{
			/* Добрались до будущего пакета, выходим */
			WARN(DTRX, "Future DL_CONFIG.req for %i:%i at %i:%i\n", req_frame_no, req_subframe_no, sf->frame_no, sf->subframe_no);
		}
	
		if (found)
			*dl_cfg_req = &t_dl_cfg_req->dl_config_request_body;
		
	}
	found = 0;
	
	t_tx_req = enodeb->trx.fapi_tx_req[sf->subframe_no];
	enodeb->trx.fapi_tx_req[sf->subframe_no] = NULL;
	
	if(t_tx_req != NULL)
	{
		req_frame_no = t_tx_req->sfn_sf >> 4;
		req_subframe_no = t_tx_req->sfn_sf & 0x0f;
	
		if (req_frame_no == sf->frame_no && req_subframe_no == sf->subframe_no)
		{
			/* Нашли нужный пакет */
			found = 1;
		}
		else if ((req_frame_no < sf->frame_no) || ((req_frame_no == sf->frame_no) && (req_subframe_no < sf->subframe_no)))
		{
			/* Пакет из прошлого, удаляем из очереди и смотрим следующий пакет */
			WARN(DTRX, "Late TX.req for %i:%i at %i:%i\n", req_frame_no, req_subframe_no, sf->frame_no, sf->subframe_no);
		}
		else
		{
			/* Добрались до будущего пакета, выходим */
			WARN(DTRX, "Future TX.req for %i:%i at %i:%i\n", req_frame_no, req_subframe_no, sf->frame_no, sf->subframe_no);
		}
	
		if (found)
			*tx_req = &t_tx_req->tx_request_body;
	}
	found = 0;
	
	t_hi_dci0_req = enodeb->trx.fapi_hi_dci0[sf->subframe_no];
	enodeb->trx.fapi_hi_dci0[sf->subframe_no] = NULL;
	
	if(t_hi_dci0_req != NULL)
	{
		req_frame_no = t_hi_dci0_req->sfn_sf >> 4;
		req_subframe_no = t_hi_dci0_req->sfn_sf & 0x0f;
	
		if (req_frame_no == sf->frame_no && req_subframe_no == sf->subframe_no)
		{
			/* Нашли нужный пакет */
			found = 1;
		}
		else if ((req_frame_no < sf->frame_no) || ((req_frame_no == sf->frame_no) && (req_subframe_no < sf->subframe_no)))
		{
			/* Пакет из прошлого, удаляем из очереди и смотрим следующий пакет */
			WARN(DTRX, "Late HI_DCI0.req for %i:%i at %i:%i\n", req_frame_no, req_subframe_no, sf->frame_no, sf->subframe_no);
		}
		else
		{
			/* Добрались до будущего пакета, выходим */
			WARN(DTRX, "Future HI_DCI0.req for %i:%i at %i:%i\n", req_frame_no, req_subframe_no, sf->frame_no, sf->subframe_no);
		}
	
		if (found)
			*hi_dci0_req = &t_hi_dci0_req->hi_dci0_request_body;
	}
	
	return LTE_SUCCESS;
}

void trx_dl_task(uint32_t p)
{
	os_status status;
	lte_trx_t *trx = (lte_trx_t *) p;
	lte_enodeb_t *enodeb;
	uint32_t rx_fn_cnt;
	uint32_t rx_tn_cnt;
	lte_subframe_t *lte_sf;
	int32_t i, j;
	volatile uint64_t loop_start, loop_end;
	uint32_t loop_counter = 0;
	uint32_t tx_initial_round = 0; // was 2;
	fapi_dl_config_request_body_t *dl_cfg_req = NULL;
	fapi_tx_request_body_t *tx_req = NULL;
	fapi_hi_dci0_request_body_t *hi_dci0_req = NULL;

	uint32_t tti_sync;
	lte_status_t lte_status;
	/* Признак первого цикла при запуске
	 * Нужен для правильной генерации сабфреймов, запуска AIC, генерации TTI
	 */
	uint32_t first_run_cycle;
	uint32_t first_run_maple;

	/* Флаги ошибок в цикле передачи */
	uint32_t tx_round_error_flags;

	/* Системные фрейм и сабфрейм (msb и lsb соответственно) */
	uint32_t system_sfn_sf;
	/* Номера подготавливаемых  фрейма и сабфрейма,
	 * системный сабфрейм + 1 
	 */
	uint32_t tx_frame_no, tx_subframe_no;

	/* Номера индицируемых MAC фрейма и сабфрейма,
	 * системный сабфрейм + 2 
	 */
	uint32_t ind_frame, ind_subframe;

	/* Счетчик таймаутов приема DL_CONFIG.req
	 * Используется для статистики и определения правильности работы L2/L3
	 */
	uint32_t dl_cfg_req_recv_timeout;
	
	/* Идентификатора безопасности */
	secure_id_t sec_id = get_secure_id();
	int32_t sec_time_count;

	OS_ASSERT_COND(trx != NULL);
	OS_ASSERT_COND(trx->enodeb != NULL);

	enodeb = trx->enodeb;

	rx_fn_cnt = 0;
	rx_tn_cnt = 0;

	while (1)
	{
		if (enodeb->state != L1_RUNNING)
		{
			tx_initial_round = 0; // was 2;
			first_run_cycle = 1;
			first_run_maple = 1;
			tti_sync = 0;
			tx_round_error_flags = 0;
			system_sfn_sf = 0;
			dl_cfg_req_recv_timeout = 0;

#if defined(DEBUG_TX_CPRI) || defined(DEBUG_PDPE_OUTPUT) || defined(DEBUG_TX_CPRI_ON_DEMAND)
			debug_tx_buffer_ptr = 0;
#endif
#ifdef DEBUG_TX_CPRI_ON_DEMAND
			pdsch_dump_cpri_tx = 0;
#endif
			sec_id = get_secure_id();
			sec_time_count = SECURE_EVAL_COUNT;
					
			osTaskDelay(1);
			continue;
		}

		if (tx_round_error_flags & 0xffff0000 || enodeb->flag_stop == 1)
		{
			/* Во время цикла произошла фатальная ошибка, завершаем работу */
			osTaskDelay(5);
			lte_enodeb_stop(enodeb);
			osTaskDelay(1);
			continue;
		}
#ifdef B4860
		/* Синхронизация с TTI при первом запуске 
		 * использутеся в варианте B4860 
		 */
		if(first_run_cycle)
		{
//#ifdef DEBUG
			cpri_print_counters(enodeb);
//#endif
			/* Сброс счетчиков IQ */
			trx->raw_iq_ts_rx = 0;
			trx->raw_iq_ts_tx = 0;

			osEventSemaphoreReset(trx->evt_subframe_ind, 0);

			/* Ожидание первого перерывания в течение 5мс 
			 * Теоретически, оно может произойти несинхронно
			 */
			status = osEventSemaphorePend(trx->evt_subframe_ind, 200);
			/* Здесь должна быть нормальная обработка таймаута таймера */
			if(status == OS_SUCCESS)
			{
				osEventSemaphoreReset(trx->evt_subframe_ind, 0);

				/* Ожидание второго прерывания TTI */
				status = osEventSemaphorePend(trx->evt_subframe_ind, 200);
				if(status == OS_SUCCESS)
				{
					/* Установка флага синхронизации по TTI */
					tti_sync = 1;
				}
			}

			if(tti_sync == 0)
			{
				ERROR(DTRX, "Error synchronizing TTI\n");
				continue;
			}

			/* Сброс очереди сабфреймов для модуляции */
			osQueueReset(trx->evq_tx_mod_subframe);
			
			enodeb->system_subframe_no = 0;
			
#ifdef DEBUG_PDPE_OUTPUT
		// Сброс флага дампа выхода PDPE
		pdsch_pdpe_dump = 0;
		pdsch_pdpe_dump_cpri = 0;
#endif
						
			first_run_cycle = 0;
		}
#else
		/* Синхронизация с TTI при первом запуске
		 * Используется в варинате BSC9132 (AIC) 
		 */
		if (first_run_cycle)
		{
			osEventSemaphoreReset(trx->evt_subframe_ind, 0);

			/* Сброс очереди сабфреймов для модуляции */
			osQueueReset(trx->evq_tx_mod_subframe);

			enodeb_inst.system_frame_no = 0;
			enodeb_inst.system_subframe_no = 0;
			
			first_run_cycle = 0;
		}
#endif
		/* Ожидание события начала TTI 
		 * За время ожидания L2 должен подготовить данные для фрейма
		 * 
		 * Для первых двух циклов ожидание не требуется
		 */
		if (tx_initial_round == 0)
		{
			status = osEventSemaphorePend(trx->evt_subframe_ind, 5);
			if (status == OS_ERR_EVENT_SEMA_TIMEOUT)
			{
				/* Таймаут приема индикатора сабфрейма
				 * Это означает, что AIC остановлен
				 * Следовательно, можно перезапускать основной цикл, т.к. 
				 * никакие ресурсы до этого момента не выделялись
				 */

				continue;
			}OS_ASSERT_COND(status == OS_SUCCESS);
		}
		else
		{
			tx_initial_round--;
		}
		
#ifdef DEBUG_PDPE_OUTPUT
		// Сброс флага дампа выхода PDPE
		pdsch_pdpe_dump = 0;
#endif
		
#ifdef DEBUG_TX_CPRI_ON_DEMAND
		//pdsch_dump_cpri_tx = 0;
#endif
		
#if 0
		/* Переехало в CPRI_TX_IQ_CB */
		/* Сохранение предыдущего системного счетчика
		 * Эти значения используются в прерываниях приема, т.к.
		 * текущее прерывание соответствуюет приему предыдущего сабфрейма
		 */
		
		osHwiSwiftDisable();
		enodeb->prev_system_frame_no = enodeb->system_frame_no;
		enodeb->prev_system_subframe_no = enodeb->system_subframe_no;

		/* Установка нового системного счетчика */
		enodeb->system_subframe_no = enodeb->system_subframe_no + 1;
		if (enodeb->system_subframe_no == 10)
		{
			enodeb->system_frame_no = (enodeb->system_frame_no + 1) & 1023;
			enodeb->system_subframe_no = 0;
		}
		osHwiSwiftEnable();
#endif
		LOG_EVENT_SF(LOGEVT_TX_SF_SFN, enodeb->system_frame_no, enodeb->system_subframe_no, 0);

		/*
		 * Номер текущего системного фрейма/сабфрейма был принят в сообщении system_sfn_sf
		 * Он уже идет.
		 * 
		 * Подготовка данных должна производиться для следующего сабфрейма
		 */

		tx_subframe_no = enodeb->system_subframe_no + 1;
		tx_frame_no = enodeb->system_frame_no;

		if (tx_subframe_no >= 10)
		{
			tx_frame_no = (tx_frame_no + 1) & 1023;
			tx_subframe_no = 0;
		}
		
		ind_subframe = (tx_subframe_no + 1);
		ind_frame = tx_frame_no;

		if (ind_subframe >= 10)
		{
			ind_frame = (ind_frame + 1) & 1023;
			ind_subframe = 0;
		}
				
		/* Нормальный режим работы */
		/* Передача нового SUBFRAME.indication */
		if (tx_initial_round == 0)
		{
			fapi_p7_send_subframe_indication(ind_frame, ind_subframe);
		}
		
		/* Событие TTI генерируется в обработчике прерывания MPALE PDSCH,
		 * поэтому для правильной работы MAPLE должен работать всегда.
		 * Если текущий сабфрейм не содержит downlink-информации, то необходимо запускать MAPLE с пустыми данными,
		 * чтобы не происходило ошибки AIC underrun.
		 */

#if 1
		/* Если текущий сабфрейм не DL, то ждем следующего */
		if(!liblte_is_dl_subframe(enodeb, tx_subframe_no) && !liblte_is_spec_subframe(enodeb, tx_subframe_no))
		{
			continue;
		}
#endif	
		lte_sf = trx->lte_dl_subframe_pool[trx->lte_dl_subframe_pool_ptr];

		trx->lte_dl_subframe_pool_ptr++;
		if (trx->lte_dl_subframe_pool_ptr >= TRX_NUM_OF_SLOTS)
			trx->lte_dl_subframe_pool_ptr = 0;

		LOG_EVENT_SF(LOGEVT_DL_TASK_BEGIN, tx_frame_no, tx_subframe_no, (uint32_t) lte_sf);

		memset(&lte_sf->job, 0, sizeof(lte_pdsch_job_t));
		memset(&lte_sf->pdcch_syms, 0, PDCCH_NUM_MAX * enodeb->fp.LTE_N_RE * enodeb->fp.LTE_N_PHYS_ANTENNAS_TX * sizeof(Complex16));
		memset(lte_sf->user_map, 0, 64 * sizeof(maple_pdsch_user_header_t));
		memset(lte_sf->cw_map, 0, 64 * sizeof(maple_pdsch_cw_header_t));
		
		lte_sf->timestamp_create = 0;
		lte_sf->timestamp_map = 0;
		lte_sf->timestamp_maple_complete = 0;
		lte_sf->timestamp_maple_ext_sym_start = 0;
		lte_sf->timestamp_maple_start = 0;
		
		lte_init_rb_map_pointers(enodeb, lte_sf);

		loop_start = log_get_timer_value();
		lte_sf->timestamp_create = loop_start;

		/* Заполнение базовых полей lte_subframe_t */
		lte_sf->frame_no = tx_frame_no;
		lte_sf->subframe_no = tx_subframe_no;
		lte_sf->trx = trx;

		lte_sf->job.cop_job.job_id = lte_sf;
		lte_sf->job.cop_job.device_specific = &lte_sf->job.pdsch_job;
		lte_sf->job.cop_job.next = NULL;
		
		//// START MAPLE-B3
		if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_FDD)
		{
			/* Заполнение общих флагов FDD */
			lte_sf->job.pdsch_job.second_flags = PDSCH_LTE_FRAME_STR1 | PDSCH_NORMAL_CP | PDSCH_BD_SF_NUM(lte_sf->subframe_no) |
					PDSCH_BD_EXT_SYM_IN_SCALE(0);

			//lte_sf->job.pdsch_job.third_flags = PDSCH_BD_CS_EN | PDSCH_BD_NUM_SYMS(14) | PDSCH_BD_ADP_OVA_SCL(4);
			//lte_sf->job.pdsch_job.third_flags = PDSCH_BD_CS_EN | PDSCH_BD_NUM_SYMS(14) | PDSCH_BD_ADP_OVA_SCL(9) |
			//lte_sf->job.pdsch_job.third_flags = PDSCH_BD_CS_EN | PDSCH_BD_NUM_SYMS(14) | PDSCH_BD_ADP_OVA_SCL(8) |
			
			lte_sf->job.pdsch_job.third_flags = PDSCH_BD_CS_EN | PDSCH_BD_NUM_SYMS(14) | PDSCH_BD_ADP_OVA_SCL(5) |
					PDSCH_SF_SUBCARRIER_15KHZ;
			
			/* Мощность сигналов CSRS */
			lte_sf->job.pdsch_job.hdr_config[PDSCH_CS_RS_GAIN_CONFIG] = PDSCH_BD_GAIN_EXPONENT(enodeb->fp.LTE_CSRS_GAIN_E) |
					PDSCH_BD_GAIN_MANTISSA(enodeb->fp.LTE_CSRS_GAIN_M);
		}
		else if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_TDD)
		{
			// FIXME: MAPLE-B3
#if 0
			/* Заполнение общих флагов TDD */
			lte_sf->job.pdsch_job.second_flags = PDSCH_LTE_FRAME_STR2 | PDSCH_BD_N_UE_SPECIFIC_RS(0) | PDSCH_BD_EXT_SYM_IN_SCALE(0) | //PDSCH_BD_EXT_SYM_IN_SCALE(13) | 
					PDSCH_LTE_FRAME_STR1 | PDSCH_NORMAL_CP | PDSCH_BD_SF_NUM(lte_sf->subframe_no);

			//lte_sf->job.pdsch_job.third_flags = PDSCH_BD_CS_EN | PDSCH_BD_NUM_SYMS(14) | PDSCH_BD_ADP_OVA_SCL(9);
			lte_sf->job.pdsch_job.third_flags = PDSCH_BD_CS_EN |
					PDSCH_BD_NUM_SYMS(liblte_tdd_num_syms_in_dl_subframe(enodeb, lte_sf->subframe_no)) |
					PDSCH_BD_ADP_OVA_SCL(9);
#endif
		}
		
		lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_EXT_OFDM_SYM_ADDRESS] = lte_sf->pdcch_syms;
		

		// Драйвер MAPLE-B3 проверяет заполненность полей всегда, поэтому обнулять их здесь нельзя
		lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_USER_HEADER_ADDRESS] = lte_sf->user_map;
		lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_CW_HEADER_ADDRESS] = lte_sf->cw_map;
		lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_RB_MAP_TABLE_ADDRESS] = lte_sf->rb_map_slot0;
		
		//// END MAPLE-B3
		
		/* Сброс указателя таблицы соответствия RB */
		lte_sf->rb_ptr = 0;

#ifdef DEBUG_TX_BUFFER_TEST
		status = osMmuDataVirtToPhys((void *)debug_tx_buffer, (void **)&lte_sf->job->pdsch_job.hdr_data_ptrs[PDSCH_ANT0_DATA_ADDRESS]);
		OS_ASSERT_COND(status == OS_SUCCESS);
#endif

#ifdef DEBUG_MAPLE_OUTPUT
		// Старый вариант (BSC9132)
		status = osMmuDataVirtToPhys((void *)&trx_maple_debug_buffer[trx_maple_debug_buffer_ptr],
				(void **)&lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_ANT0_DATA_ADDRESS]);
		OS_ASSERT_COND(status == OS_SUCCESS);

		trx_maple_debug_buffer_ptr += LTE_SAMPLES_PER_SUBFRAME;

		if(trx_maple_debug_buffer_ptr >= TRX_MAPLE_DEBUG_BUFFER_SIZE)
		{
			/* Ожидание завершения обрабокти буферов */
			while(trx->num_disp_jobs != trx->num_reaped_jobs)
				osTaskDelay(1);

			/* Преборазование BE -> LE */
			for(i=0; i<TRX_MAPLE_DEBUG_BUFFER_SIZE; i++)
			{
				trx_maple_debug_buffer[i] = V_swapb2(trx_maple_debug_buffer[i]);
			}

			/* Отладочный буфер заполнен, 
			 * Ставим флаг и на этом прекращаем работу
			 */
			tx_round_error_flags |= 0x00040000;
			continue;
			//OS_ASSERT;
		}
#endif

		if (tx_initial_round == 0/* && liblte_is_dl_subframe(enodeb, lte_sf->subframe_no) */)
		{
			/* Выбор даннх сабфрейма из очередей DL_CONFIG и TX_REQ */
			lte_status = trx_prepare_subframe_data(enodeb, lte_sf, &dl_cfg_req, &tx_req, &hi_dci0_req);

			/* Счетчик таймаутов приема DL_CONFIG.req
			 * Используется дальше для определиния остановки L2/L3 
			 */
			if (dl_cfg_req == NULL)
			{
				dl_cfg_req_recv_timeout++;

				ERROR(DTRX, "Got NULL dl_cfg tx_req=0x%08x, cnt=%i\n", dl_cfg_req, dl_cfg_req_recv_timeout);
			}
			else
				dl_cfg_req_recv_timeout = 0;
		}
		else
		{
			dl_cfg_req = NULL;
			tx_req = NULL;
			hi_dci0_req = NULL;
		}
		
		/* Вообще-то dl_cfg_req->number_pdcch_ofdm_symbols должен быть всегда >0,
		 * если dl_cfg_req != NULL
		 * Но такая ситуация может происходить из-за специфики работы L2, 
		 * для нескольких первых сабфреймов
		 */
		if (dl_cfg_req != NULL && dl_cfg_req->number_pdcch_ofdm_symbols > 0)
		{
			lte_sf->num_pdcch_symbols = dl_cfg_req->number_pdcch_ofdm_symbols;

			/* Заполнение дескриптора сабфрейма в соответствии с BFN/SFN */
			if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_FDD)
			{
				lte_trx_map_slot_fdd(trx, lte_sf, dl_cfg_req, tx_req, hi_dci0_req);
			}
			else if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_TDD)
			{
				lte_trx_map_slot_tdd(trx, lte_sf, dl_cfg_req, tx_req, hi_dci0_req);
			}
		}
		else
		{
			lte_sf->num_pdcch_symbols = 0;
		}

		lte_sf->timestamp_map = log_get_timer_value();
		LOG_EVENT_SF(LOGEVT_DL_SF_MAP, lte_sf->frame_no, lte_sf->subframe_no, (uint32_t) lte_sf);

		/* Заполнение количества символов PDCCH */
		if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_TDD &&
				liblte_tdd_num_syms_in_dl_subframe(enodeb, lte_sf->subframe_no) == 3)
		{
			/* 3 символа в сабфрейме, значит это SSF, необходимо добавить PSS, т.к. автоматически он не добавляется
			 * (описание PDSCH BD, поля CFI, NUM_SYMS)
			 */
			lte_sf->job.pdsch_job.third_flags |= PDSCH_CFI3;
			j = enodeb->fp.LTE_N_RE * 2 + enodeb->fp.LTE_FIRST_SS_CARRIER;
			for(i=0; i<72; i++)
			{
				lte_sf->pdcch_syms[j + i] = TO_COMPLEX16(enodeb->refsigs->pss.signal_input[i].data_re, enodeb->refsigs->pss.signal_input[i].data_img);				
			}
		}
		else
		{
			if (lte_sf->num_pdcch_symbols == 1)
			{
				lte_sf->job.pdsch_job.third_flags |= PDSCH_CFI1;
			}
			else if (lte_sf->num_pdcch_symbols == 2)
			{
				lte_sf->job.pdsch_job.third_flags |= PDSCH_CFI2;
			}
			else if (lte_sf->num_pdcch_symbols == 3)
			{
				lte_sf->job.pdsch_job.third_flags |= PDSCH_CFI3;
			}
			else
			{
				/* Некорректное значение, обнуляем CFI, будет передан place holder */
				lte_sf->num_pdcch_symbols = 0;
			}
		}
		
		if(sec_id == SECURE_ID_FULL)
		{
			/* Нормальная работа */
		}
		else if(sec_id == SECURE_ID_EVAL)
		{
			/* Ограничение на количество пользователей */
			if(lte_sf->num_tb_alloc > 4)
				lte_sf->num_tb_alloc = 4;
			
			/* И на количество антенн */
			if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX > 2)
				lte_sf->num_pdcch_symbols = 0;
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
				lte_sf->num_pdcch_symbols = 0;
			}
			
			/* Ограничение на количество пользователей */
			if(lte_sf->num_tb_alloc > 4)
				lte_sf->num_tb_alloc = 4;
		}

		// Установка количества активных пользователей
		if(lte_sf->num_pdcch_symbols == 0)
		{
			// Place holder
			lte_sf->job.pdsch_job.first_flags = PDSCH_BD_INT_EN | PDSCH_BD_PH_EN;
		}
		else
		{
			lte_sf->job.pdsch_job.first_flags = PDSCH_BD_N_USERS(lte_sf->num_tb_alloc) | PDSCH_BD_INT_EN;
		}

		
		/* Установка флага TB_LAST на последнем блоке данных */
		if (lte_sf->num_tb_alloc > 0)
		{
			// FIXME: MAPLE-B3
			//lte_sf->tb_map[lte_sf->num_tb_alloc - 1].first_flags |= PDSCH_TB_LAST;

			/* Сброс данных TB и CW из кэша в память */
			sweep_cache((uint32_t) &lte_sf->user_map, sizeof(maple_pdsch_user_header_t) * lte_sf->num_tb_alloc, CACHE_FLUSH);
			sweep_cache((uint32_t) &lte_sf->cw_map, sizeof(maple_pdsch_cw_header_t) * lte_sf->num_tb_alloc, CACHE_FLUSH);
			sweep_cache((uint32_t) &lte_sf->rb_map_slot0, PDSCH_RB_MAP_TABLE_SIZE_20MHZ, CACHE_FLUSH);
		}

		sweep_cache((uint32_t) lte_sf->pdcch_syms, enodeb->fp.LTE_N_RE * lte_sf->num_pdcch_symbols * sizeof(Complex16), CACHE_FLUSH);

		/*
		 * Адреса буферов антенн
		 */
#ifdef DEBUG_TX_CPRI
		// Используется отладочный буфер для выходных отсчетов
		if(debug_tx_buffer_ptr < DEBUG_TX_BUFFER_SIZE)
		{
			// Используем отладочный буфер пока он не заполнится
			for(i = 0; i < enodeb->fp.LTE_N_PHYS_ANTENNAS_TX; i++)
			{
				lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_ANT0_DATA_ADDRESS + i] = &debug_tx_buffer[debug_tx_buffer_ptr];
				debug_tx_buffer_ptr += (enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * sizeof(Complex16));
				
				if(debug_tx_buffer_ptr >= DEBUG_TX_BUFFER_SIZE)
					break;
			}
		}
		else
		{
			// Отладочный буфер заполнен, используем адрес физического буфера CPRI
			for(i = 0; i < enodeb->fp.LTE_N_PHYS_ANTENNAS_TX; i++)
			{
				lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_ANT0_DATA_ADDRESS + i] = trx->iq_tx_buffer[i][trx->cpri_tx_prepare_buffer_no];
			}
		}
#else
		// Адрес физического буфера CPRI для всех активных антенн
		for(i = 0; i < enodeb->fp.LTE_N_PHYS_ANTENNAS_TX; i++)
		{
			lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_ANT0_DATA_ADDRESS + i] = trx->iq_tx_buffer[i][trx->cpri_tx_prepare_buffer_no];
		}
#endif
		
		/* Передача подготовленного дескриптора сабфрейма в очередь обработки MAPLE 
		 * Обработка производится в отдельном потоке, который отслеживает занятость MAPLE,
		 * наличие новых дескрипторов, и т.д.
		 */
#ifdef DEBUG_PDPE_OUTPUT
		/* 
		 * Запись сформированного сабфрейма в буфер (до IFFT)
		 */
		
		if(pdsch_pdpe_dump_cpri && debug_tx_buffer_ptr < DEBUG_TX_BUFFER_SIZE)
		{
			Complex16 *dbg_buf_c16 = (Complex16 *)&debug_tx_buffer[debug_tx_buffer_ptr]; 
							
			// Копирование буфера CPRI, антенна 0
			memcpy(dbg_buf_c16, trx->iq_tx_buffer[0][trx->cpri_tx_active_buffer_no], 
					enodeb->fp.LTE_SAMPLES_PER_SUBFRAME*sizeof(int32_t));
			debug_tx_buffer_ptr += (enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * sizeof(Complex16));
			
			if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2)
			{
				// Копирование буфера CPRI, антенна 1
				dbg_buf_c16 = (Complex16 *)&debug_tx_buffer[debug_tx_buffer_ptr]; 
				memcpy(dbg_buf_c16, trx->iq_tx_buffer[1][trx->cpri_tx_active_buffer_no], 
						enodeb->fp.LTE_SAMPLES_PER_SUBFRAME*sizeof(int32_t));
				
				debug_tx_buffer_ptr += (enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * sizeof(Complex16));
			}

			pdsch_pdpe_dump_cpri = 0;
		}
		
		if(pdsch_pdpe_dump && debug_tx_buffer_ptr < DEBUG_TX_BUFFER_SIZE)
		{
			Complex16 *dbg_buf_c16 = (Complex16 *)&debug_tx_buffer[debug_tx_buffer_ptr];
			
			INFO(DTRX, "Dumping TX at offset %i\n", debug_tx_buffer_ptr / sizeof(int32_t));
			
			// Копирование символов PDCCH
			memcpy(dbg_buf_c16, lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_EXT_OFDM_SYM_ADDRESS], 
					enodeb->fp.LTE_N_PHYS_ANTENNAS_TX * lte_sf->num_pdcch_symbols * enodeb->fp.LTE_N_RE*sizeof(int32_t));
			
			// Установка бита POD и буфера для выхода PDPE 
			lte_sf->job.pdsch_job.third_flags |= 0x00200000;
			lte_sf->job.pdsch_job.hdr_data_ptrs[PDSCH_PDPE_OUTPUT_DUMP_ADDRESS] = 
					&dbg_buf_c16[enodeb->fp.LTE_N_PHYS_ANTENNAS_TX * lte_sf->num_pdcch_symbols * enodeb->fp.LTE_N_RE];
			
			dbg_buf_c16[enodeb->fp.LTE_N_PHYS_ANTENNAS_TX * LTE_NSYMB_PER_SUBFRAME * enodeb->fp.LTE_N_RE+16] = V_swapb2(lte_sf->frame_no);
			dbg_buf_c16[enodeb->fp.LTE_N_PHYS_ANTENNAS_TX * LTE_NSYMB_PER_SUBFRAME * enodeb->fp.LTE_N_RE+17] = V_swapb2(lte_sf->subframe_no);
			
			debug_tx_buffer_ptr += (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX * enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * sizeof(Complex16));
			
			pdsch_pdpe_dump_cpri = 1;
		}
#endif
		
		/* На последнем раунде инициализации необходимо запустить MAPLE
		 * При этом все запросы будут уже в очереди 
		 */
		lte_status = trx_maple_pdsch_direct(lte_sf, first_run_maple);
		
		if (lte_status != LTE_SUCCESS)
		{
			/* Очеред MAPLE переполнена
			 * Это фатальная ошибка, значит он, скорее всего, остановился
			 * Ставим флаг и на этом прекращаем работу
			 */
			tx_round_error_flags |= 0x00010000;
		}

		/* Сброс флага первого запуска MAPLE */
		first_run_maple = 0;

#ifdef DEBUG_TX_CPRI
		/* В режиме отладки дожидаемся обработки задания */
		if(debug_tx_buffer_ptr < DEBUG_TX_BUFFER_SIZE)
		{
			Complex16 *dbg_chunk = (Complex16 *)&debug_tx_buffer[debug_tx_buffer_ptr -
			                                                     enodeb->fp.LTE_SAMPLES_PER_SUBFRAME * sizeof(Complex16)];
			
			while(trx->num_disp_jobs != trx->num_reaped_jobs)
				;
		
			// Big endian -> Little endian
			for(i=0; i<enodeb->fp.LTE_SAMPLES_PER_SUBFRAME; i++)
			{
				dbg_chunk[i] = V_swapb2(dbg_chunk[i]);
			}
		}
#endif
		
#ifdef DEBUG_TX_CPRI_ON_DEMAND
		//if(pdsch_dump_cpri_tx_count > 100 && debug_tx_buffer_ptr < DEBUG_TX_BUFFER_SIZE)
		//if(lte_sf->dump_cpri_tx && debug_tx_buffer_ptr < DEBUG_TX_BUFFER_SIZE)
		//if(debug_tx_buffer_ptr < DEBUG_TX_BUFFER_SIZE)
		if(pdsch_dump_cpri_tx && debug_tx_buffer_ptr < DEBUG_TX_BUFFER_SIZE)
		{
			Complex16 *dbg_buf_c16 = (Complex16 *)&debug_tx_buffer[debug_tx_buffer_ptr];
			
			// Ждем завершения работы MAPLE
			while(trx->num_disp_jobs != trx->num_reaped_jobs && enodeb->flag_stop == 0)
				;
			
			DBG(DTRX, "Dumping TX at offset %i\n", debug_tx_buffer_ptr / sizeof(int32_t));

			// Копирование буфера CPRI, антенна 0
			memcpy(dbg_buf_c16, trx->iq_tx_buffer[0][trx->cpri_tx_prepare_buffer_no], 
					enodeb->fp.CPRI_SAMPLES_PER_SUBFRAME*sizeof(Complex16));
			debug_tx_buffer_ptr += (enodeb->fp.CPRI_SAMPLES_PER_SUBFRAME * sizeof(Complex16));
			
			if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2)
			{
				// Копирование буфера CPRI, антенна 1
				dbg_buf_c16 = (Complex16 *)&debug_tx_buffer[debug_tx_buffer_ptr]; 
				memcpy(dbg_buf_c16, trx->iq_tx_buffer[1][trx->cpri_tx_prepare_buffer_no], 
						enodeb->fp.CPRI_SAMPLES_PER_SUBFRAME*sizeof(int32_t));
				
				debug_tx_buffer_ptr += (enodeb->fp.CPRI_SAMPLES_PER_SUBFRAME * sizeof(Complex16));
			}

			//pdsch_dump_cpri_tx = 0;
		}
#endif
		
		/* Если слишком много таймаутов приема данных от L2/L3,
		 * то ставим флаг ошибки
		 */
		if (dl_cfg_req_recv_timeout >= MAX_DL_CFG_REQ_TIMEOUTS)
		{
			tx_round_error_flags |= 0x00020000;
		}

		if (tx_round_error_flags & 0xffff0000)
		{
			/* Во время цикла произошла фатальная ошибка, ждем завершения работы */
			ERROR(DTRX, "MAP cycle fatal error detected: 0x%08x\n", tx_round_error_flags);
		}

		loop_end = log_get_timer_value();
		trx_cycle_counters[TRX_CYCLE_DO_DLSH][loop_counter] = loop_end - loop_start;
		loop_counter = (loop_counter + 1) % TRX_CYCLE_NUM_LOOPS;

		LOG_EVENT_SF(LOGEVT_DL_TASK_END, tx_frame_no, tx_subframe_no, (uint32_t) lte_sf);
	}
}

static void trx_maple_pdsch(uint32_t p)
{
	os_status status;
	lte_trx_t *trx = (lte_trx_t *) p;
	lte_enodeb_t *enodeb;

	uint32_t iter, pcr = 0;
	int num_jobs = 1;
	lte_subframe_t *cur_sf;

	lte_pdsch_job_t *job;

	OS_ASSERT_COND(trx != NULL);
	OS_ASSERT_COND(trx->enodeb != NULL);

	enodeb = trx->enodeb;

	//for (iter = 0; iter < TEST_ITERATIONS_NUM; iter++)
	iter = 0;

#if 0
	/*
	 In order to invalidate all buffers - perform global flush. This comes at the
	 expense of now refreshing all caches, however for demo purposes it is better than
	 having involuntary thrashes at unexpected times.
	 */
	status = osCacheDataSweepGlobal(CACHE_FLUSH);
	OS_ASSERT_COND(status == OS_SUCCESS);
	status = osCacheL2UnifiedSweepGlobal(CACHE_FLUSH);
	OS_ASSERT_COND(status == OS_SUCCESS);

	/* Wait for data to reach physical memory */
	osCacheDataWaitSweep();
#endif
	
	while (1)
	{
		status = osQueueDequeue(trx->evq_tx_mod_subframe, (uint32_t *) &cur_sf);
		//status = osEventQueuePend(trx->evq_tx_mod_subframe, (uint32_t *) &cur_sf, 0);

		OS_ASSERT_COND(status == OS_SUCCESS);

		if (status == OS_ERR_Q_EMPTY)
		{
			/* Underrun */
			continue;
		}
		else if (status != OS_SUCCESS)
		{
			/* Неожиданная ошибка */
			OS_ASSERT;
		}

		//OS_ASSERT_COND(cur_sf != NULL);
		if (cur_sf == NULL)
			continue;

#if (USING_SYM_INTERRUPT == TRUE)
		cur_sf->job.pdsch_job.first_flags |= PDSCH_BD_SYM_INT_EN;
#endif
#if (USING_INTERRUPT == TRUE)
		/* Add interrupt bit to last job in batch */
		cur_sf->job.pdsch_job.first_flags |= PDSCH_BD_INT_EN;
#endif

		job = &cur_sf->job;
//		memcpy(&job->pdsch_job, &job_test->pdsch_job, sizeof(maple_pdsch_job_t));
		job->cop_job.job_id = (void *) trx->maple_job_id;
		trx->maple_jobs[trx->maple_job_id] = cur_sf;
		trx->maple_job_id++;

		sweep_cache((uint32_t) job, sizeof(lte_pdsch_job_t), CACHE_FLUSH);

		//status = osEventSemaphorePend(trx->evt_subframe_tx, 0);

		do
		{
			num_jobs = 1;
			status = osCopChannelDispatch(&pdsch_ch_handle[0], &job->cop_job, &num_jobs);

			if (status == OS_SUCCESS)
				trx->num_disp_jobs += num_jobs;
		}
		while (status != OS_SUCCESS);

		if (iter == 0)
		{
			//FIXME: MAPLE-B3
#if BSC9132
#if 0
			/* PARSE PCR */
			opcode = MAPLE_PARSE_PDSCH_EDF_BD;
			status = osCopDeviceCtrl(maple_handle, MAPLE_CMD_PCR_ACTIVATE_WITH_POLL, &opcode);
			OS_ASSERT_COND(status == OS_SUCCESS);
			//OS_WAIT(APP_ANT_DELAY);
//			OS_WAIT(0x1400);
#endif
			/* Enough time to be sure that first BD is processed; */
			/* PCR MAPLE_PDSCH_EXT_SYM_START - the data for first BD is ready */
			opcode = MAPLE_PDSCH_EXT_SYM_START;
			//status = osCopDeviceCtrl(maple_handle, MAPLE_CMD_PCR_ACTIVATE_WITH_POLL, &opcode);
			status = osCopDeviceCtrl(maple_handle, MAPLE_CMD_PCR_ACTIVATE_WITH_POLL, &opcode);
			OS_ASSERT_COND(status == OS_SUCCESS);

			OS_WAIT(APP_ANT_DELAY);
			OS_WAIT(APP_ANT_DELAY);
			
			osSioDeviceCtrl(trx->aic_adi_lane, SIO_DEVICE_RX_TX_ENABLE, NULL);
#endif
		}

		iter = 1;
	} /* for iter */
}

extern lte_pdsch_job_t* jobs_list_create(int num_jobs, uint32_t id);

static lte_status_t trx_maple_pdsch_direct(lte_subframe_t *cur_sf, uint32_t first_run)
{
	os_status status;
	lte_trx_t *trx = cur_sf->trx;
	lte_enodeb_t *enodeb;

	uint32_t pcr = 0;
	int num_jobs = 1;
	lte_pdsch_job_t *job;

	maple_pcr_opcodes_t opcode;

	OS_ASSERT_COND(trx != NULL);

	OS_ASSERT_COND(trx->enodeb != NULL);

	enodeb = trx->enodeb;

	LOG_EVENT_SF(LOGEVT_DL_SF_MAPLE, cur_sf->frame_no, cur_sf->subframe_no, (uint32_t) cur_sf);
	
	cur_sf->timestamp_maple_start = log_get_timer_value();
	osQueueReset(trx->evq_tx_mod_subframe);

	//status = osEventSemaphorePend(trx->evt_subframe_tx, 0);

	//job_test = jobs_list_create(1, 0);
	job = &cur_sf->job;
	job->cop_job.job_id = (void *) trx->maple_job_id;
	job->cop_job.next = NULL;
	trx->maple_jobs[trx->maple_job_id] = cur_sf;
	trx->maple_job_id++;

	//job = job_test;
	do
	{
		num_jobs = 1;
#if (USING_SYM_INTERRUPT == TRUE)
		//cur_sf->job.pdsch_job.first_flags |= PDSCH_BD_SYM_INT_EN;
		job->pdsch_job.first_flags |= PDSCH_BD_SYM_INT_EN;
#endif
#if (USING_INTERRUPT == TRUE)
		//if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_FDD)
		{
			/* Add interrupt bit to last job in batch in FDD mode (subframes queue processing) */
			job->pdsch_job.first_flags |= PDSCH_BD_INT_EN;
		}
#endif
		//sweep_cache((uint32_t) job, sizeof(lte_pdsch_job_t), CACHE_FLUSH);
		
		DBARS_IBSS_L12();

		status = osCopChannelDispatch(&pdsch_ch_handle[0], &job->cop_job, &num_jobs);

		//OS_ASSERT_COND(status == OS_SUCCESS);

		if (status == OS_SUCCESS)
			trx->num_disp_jobs += num_jobs;

#if (USING_INTERRUPT == FALSE)
		if (status != OS_SUCCESS)
		osCopChannelCtrl(&pdsch_ch_handle[0], MAPLE_PDSCH_CMD_RX_POLL, NULL);
#endif /* USING_INTERRUPT == FALSE */
	}
	while (status != OS_SUCCESS);

#if 1
	/* PARSE PCR */
	opcode = MAPLE_PARSE_PDSCH_EDF_BD;
	status = osCopDeviceCtrl(maple_handle[0], MAPLE_CMD_PCR_ACTIVATE_WITH_POLL, &opcode);
	OS_ASSERT_COND(status == OS_SUCCESS);
#endif
	
	if(cur_sf->num_pdcch_symbols > 0)
	{
		// Дожидаемся обработки BD
		while (job->pdsch_job.first_flags & 0x20000000 == 0)
			;

		//OS_WAIT(APP_ANT_DELAY);
		//OS_WAIT(0x1400);

		/* Enough time to be sure that first BD is processed; */
		/* PCR MAPLE_PDSCH_EXT_SYM_START - the data for first BD is ready */
		opcode = MAPLE_PDSCH_EXT_SYM_START;
		//status = osCopDeviceCtrl(maple_handle, MAPLE_CMD_PCR_ACTIVATE_WITH_POLL, &opcode);
		status = osCopDeviceCtrl(maple_handle[0], MAPLE_CMD_PCR_ACTIVATE_WITH_POLL, &opcode);
		OS_ASSERT_COND(status == OS_SUCCESS);

		cur_sf->timestamp_maple_ext_sym_start = log_get_timer_value();
	}

	/* При нормальной работе второй и последующие задания помещаются в очередь */

	return LTE_SUCCESS;
}
