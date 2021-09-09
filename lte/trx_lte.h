/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __TRX_LTE_H_
#define __TRX_LTE_H_

#include <smartdsp_os.h>
#include <lte_enodeb.h>

/* Отладочный буфер для PUSCH/PUCCH */
#define TRX_DEBUG_BUFFER

//#define PUSCH_TEST
#define PUSCH_DUMP_SDU 0
#define COMPLEX_DIV

// Настройки для PUSCH
#define SCALE_MEAN // использовать усреднение входных скейлов
#define EQPE_NVI_INTERNAL // использовать расчет NVI/BETA в EQPE


// Запись PDSCH в отладочный буфер
//#define DEBUG_TX_CPRI
#define DEBUG_TX_CPRI_ON_DEMAND
//#define DEBUG_PDPE_OUTPUT

// Debug PRACH DDS output
//#define DEBUG_PRACH_DDS
//#define DEBUG_PRACH_DETECT_DDS_ON_DETECT

//#define SCALE_MEAN

// SC3900 workaround
#define creal16(x) extract_h(x)
#define cimag16(x) extract_l(x)

enum
{
	TRX_CYCLE_DO_DLSH = 0, TRX_CYCLE_RX_PRACH, TRX_CYCLES_MAX
};
#define TRX_CYCLE_NUM_LOOPS 16

extern uint64_t trx_cycle_counters[TRX_CYCLES_MAX][TRX_CYCLE_NUM_LOOPS];
extern lte_prach_t rx_prach_buffer[LTE_PRACH_RX_SUBFRAMES] __attribute__((section(".local_data_ddr0_cacheable_bss"), aligned(16)));

#if defined(DEBUG_TX_CPRI) || defined(DEBUG_PDPE_OUTPUT) || defined(DEBUG_TX_CPRI_ON_DEMAND)
/* Отладочный буфер для выходных отсчетов */
/* Количество сэмплов для отладки */
#define DEBUG_TX_BUFFER_SIZE (30720 * 100 * sizeof(Complex16))
extern uint8_t debug_tx_buffer[DEBUG_TX_BUFFER_SIZE] __attribute__((section(".shared_data_ddr0_cacheable_bss"), aligned(ARCH_CACHE_LINE_SIZE)));
extern uint32_t debug_tx_buffer_ptr;

#ifdef DEBUG_PDPE_OUTPUT
extern int32_t pdsch_pdpe_dump;
extern int32_t pdsch_pdpe_dump_cpri;
#endif

#ifdef DEBUG_TX_CPRI_ON_DEMAND
extern int32_t pdsch_dump_cpri_tx;
extern int32_t pdsch_dump_cpri_tx_count;
#endif
#endif

os_status lte_trx_init(lte_enodeb_t *enodeb);
os_status lte_trx_ctrl_enqueue(lte_trx_t *trx, uint8_t *data, uint32_t len);
os_status lte_trx_data_enqueue(lte_trx_t *trx, uint8_t *data, uint32_t len);
os_status lte_trx_send_tx_slot_ind(lte_trx_t *trx, uint32_t bfn, uint32_t slot_no);
int32_t lte_trx_is_prach_subframe(lte_trx_t *trx);
os_status trx_clear_ul_harq_buffers(lte_trx_t *trx);

lte_ul_harq_buf_descr_t *trx_alloc_ul_harq(lte_trx_t *trx, uint32_t rnti, uint32_t h_pid, uint32_t first_rb, uint32_t n_rb);
void trx_free_ul_harq(lte_trx_t *trx, lte_ul_harq_buf_descr_t *hbd);
void trx_cleanup_ul_harq(lte_trx_t *trx);

void trx_dl_task(uint32_t p);
void trx_ul_task(uint32_t p);
void trx_prach_task(uint32_t p);

#ifdef TRX_DEBUG_BUFFER
void trx_init_debug_buffer();
#endif

#endif
