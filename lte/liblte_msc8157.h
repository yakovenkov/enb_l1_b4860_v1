/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __LIBLTE_MSC8157_H_
#define __LIBLTE_MSC8157_H_

#include <smartdsp_os.h>
#include <prototype.h>
#include <trans.h>
#include <lte_enodeb.h>
#include <dci.h>

#define cmax(x, y) (((x) > (y)) ? x : y)
#define cmin(a,b) ((a)<(b) ? (a) : (b))
/* Для FDD всегда равно 1 */
//#define get_mi(...)	1
/*
 * Вспомогательные функции для битового кодирования-декодирования
 */
void ccodelte_init(void);
void ccodelte_encode(uint32_t numbits, uint8_t add_crc, uint8_t *inPtr, uint8_t *outPtr, uint16_t rnti);

uint32_t lte_gold_generic(uint32_t *x1, uint32_t *x2, uint32_t reset);

uint32_t sub_block_interleaving_cc(uint32_t D, uint8_t *d, uint8_t *w);
void sub_block_deinterleaving_cc(uint32_t D, int8_t *d, int8_t *w);
uint32_t lte_rate_matching_cc(uint32_t RCC, uint16_t E, uint8_t *w, uint8_t *e);
void lte_rate_matching_cc_rx(uint32_t RCC, uint16_t E, int8_t *w, uint8_t *dummy_w, int8_t *soft_input);

void crcTableInit(void);
uint32_t crc24a(uint8_t * inptr, int32_t bitlen);
uint32_t crc24b(uint8_t * inptr, int32_t bitlen);
uint32_t crc16(uint8_t * inptr, int32_t bitlen);
uint32_t crc12(uint8_t *inptr, int32_t bitlen);
uint32_t crc8(uint8_t * inptr, int32_t bitlen);

uint32_t get_nsoftbits_by_ue_cat(uint32_t ue_cat);

/*
 * Формирование пакета на канале PBCH
 */

lte_status_t liblte_generate_pbch(lte_trx_t *trx, lte_subframe_t *lte_subframe);

/*
 * Формирование канала PCFICH
 */
lte_status_t liblte_generate_pcfich_reg_mapping(lte_enodeb_t *enodeb);
lte_status_t liblte_generate_pcfich(uint32_t num_pdcch_symbols, int16_t amp, lte_enodeb_t *enodeb,
		lte_subframe_t *lte_subframe);

lte_status_t liblte_generate_phich_reg_mapping(lte_enodeb_t *enodeb);
void liblte_generate_phich_top(lte_enodeb_t *enodeb, lte_subframe_t *sf, fapi_hi_dci0_request_body_t *hi_pdu_list);

/* DCI */
uint8_t liblte_generate_dci_top(uint32_t num_dci, DCI_ALLOC_t *dci_alloc, uint32_t n_rnti, int16_t amp,
		lte_enodeb_t *enodeb, lte_subframe_t *lte_subframe);

uint16_t computeRIV(uint16_t N_RB_DL, uint16_t RBstart, uint16_t Lcrbs);

uint32_t get_num_bits_in_prb(uint32_t N_subframe, uint32_t N_ctrl_symbs, uint32_t prb, uint32_t N_rb_dl, uint8_t N_ant,
		uint32_t mod_order);

int32_t liblte_gen_dl_cell_spec(lte_enodeb_t *enodeb, Complex16 *output, int16_t amp, uint8_t Ns, uint8_t l, uint8_t p);

void liblte_generate_dmrs_pusch(lte_enodeb_t *enodeb, uint32_t N_subfr, uint32_t cyclic_shift_dci, uint32_t N_prb,
		uint32_t layer, Complex16 *dmrs_0, Complex16 *dmrs_1);

uint32_t liblte_get_dft_by_nrb(uint32_t nrb);

lte_status_t liblte_generate_pucch_dmrs(lte_enodeb_t *enodeb, PUCCH_FMT_t format, int32_t sf_idx, int32_t n_pucch,
		Complex16 *drs);
lte_status_t liblte_generate_pucch_sr(lte_enodeb_t *enodeb, int32_t sf_idx, int32_t n_pucch, Complex16 *sr_nosrs,
		Complex16 *sr_srs);
lte_status_t liblte_generate_pucch_ack(lte_enodeb_t *enodeb, int32_t sf_idx, int32_t n_pucch, Complex16 *ack_nosrs,
		Complex16 *ack_srs);
lte_status_t liblte_generate_pucch_nack(lte_enodeb_t *enodeb, int32_t sf_idx, int32_t n_pucch, Complex16 *nack_nosrs,
		Complex16 *nack_srs);
lte_status_t liblte_generate_pucch_fmt2(lte_enodeb_t *enodeb, int32_t sf_idx, int32_t n_pucch, Complex16 *fmt2_nosrs,
		Complex16 *fmt2_srs);
PUCCH_FMT_t liblte_pucch_get_format(int32_t ack_len, int32_t cqi_len, int32_t sr, int32_t cp);
lte_status_t liblte_refsignal_dmrs_pucch_get(lte_enodeb_t *enodeb, int32_t ant_no, PUCCH_FMT_t format, uint32_t n_pucch,
		lte_ul_subframe_t *rx_sf, int8_t *scale_vector, Complex16 *dest, uint32_t *pucch_n_dmrs_syms);
lte_status_t liblte_pucch_get(lte_enodeb_t *enodeb, int32_t ant_no, PUCCH_FMT_t format, uint32_t n_pucch,
		lte_ul_subframe_t *rx_fs, int8_t *scale_vector, bool shortened, int32_t shift_right, Complex16 *dest,
		uint32_t *pucch_syms_re);
void liblte_calc_pucch_scale_vector(lte_ul_subframe_t *rx_sf, int8_t *scale_vector);
int32_t liblte_lte_segmentation(uint32_t B, uint32_t *C, uint32_t *Cplus, uint32_t *Cminus, uint32_t *Kplus,
		uint32_t *Kminus, uint32_t *F);

int32_t liblte_is_dl_subframe(lte_enodeb_t *enodeb, int32_t subframe);
int32_t liblte_is_spec_subframe(lte_enodeb_t *enodeb, int32_t subframe);
int32_t liblte_is_ul_subframe(lte_enodeb_t *enodeb, int32_t subframe);
int32_t liblte_tdd_num_syms_in_dl_subframe(lte_enodeb_t *enodeb, int32_t subframe);

/*
 * Инициализация внутренних таблиц
 */
lte_status_t liblte_init();

#endif
