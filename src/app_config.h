/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#pragma gcc_extensions on

#include "smartdsp_os.h"
#include "maple_pe_init.h"
#include "maple.h"
#include "maple_init.h"
#include "maple_pdsch_init.h"
#include "maple_pdsch.h"
#include "maple_memmap.h"
#include "maple_init_common.h"

//#define PUSCH_MMSE_EN ON

#if (PUSCH_MMSE_EN == ON)
#include "maple_eqpe.h"
#endif //PUSCH_MMSE_EN == ON


// FIXME: SC3900 workaround
#define CACHE_FLUSH 0

//#define DEBUG_TRX_LTE_BD
//#define DEBUG_TX_BUFFER_TEST
//#define DEBUG_MAPLE_OUTPUT
//#define DEBUG_AIC_NO_TX_ACK
//#define DEBUG_AIC_NO_RX_ACK

#ifdef DEBUG_MAPLE_OUTPUT
/* Количество сэмплов для отладки */
#define DEBUG_MAPLE_SUBFRAMES	100
#define LTE_SAMPLES_PER_SUBFRAME_DEBUG 15360
#define TRX_MAPLE_DEBUG_BUFFER_SIZE (LTE_SAMPLES_PER_SUBFRAME_DEBUG * DEBUG_MAPLE_SUBFRAMES)
#endif

//#define DEBUG_PRACH_DETECT
//#define DEBUG_OPT_OFF

/* PRACH debug macros */
//#define DEBUG_PRACH_DETECT_DDS
//#define DEBUG_PRACH_DETECT_DDS_ON_DETECT
//#define DEBUG_PRACH_DETECT_HB1
//#define DEBUG_PRACH_DETECT_HB2
//#define DEBUG_PRACH_DETECT_FIR3
//#define DEBUG_PRACH_DETECT_FFT
//#define DEBUG_PRACH_DETECT_CORR_IFFT
#define DEBUG_PRACH_DETECT_CORR_PEAKS

//#define DEBUG_AIC_RX
//#define DEBUG_AIC_RX_SUBFRAMES (2*1000)

#define ENABLE_EVENTS_LOG

/* Пироритеты задач ЦОС, в порядке убывания */
#define TASK_PRIORITY_FAPI_P7		OS_TASK_PRIORITY_09
#define TASK_PRIORITY_DL_TASK		OS_TASK_PRIORITY_10
#define TASK_PRIORITY_UL_TASK		OS_TASK_PRIORITY_11
#define TASK_PRIORITY_PRACH_TASK	OS_TASK_PRIORITY_20
#define TASK_PRIORITY_CPRI_ETH_OUT	OS_TASK_PRIORITY_21
#define TASK_PRIORITY_CPRI_ETH_IN	OS_TASK_PRIORITY_22
#define TASK_PRIORITY_FAPI_P5		OS_TASK_PRIORITY_23
#define TASK_PRIORITY_IPC_LOG		OS_TASK_PRIORITY_29

#define DONT_FLIP_DCI
//#define  USE_PDSCH_QUEUE

#define MAX_NUM_OF_ANTENNAS         2
#ifdef AIC_MULTI_LANE_DEMO
#define NUM_OF_LANES                2
#else
#define NUM_OF_LANES                1
#endif //AIC_MULTI_LANE_DEMO
#define NUM_OF_ANTENNAS             1

#define RX_IQ_MBUS_TRANSACTION_SIZE_IN_REG          AIC_IQ_TRANSACTION_256_BYTES
#define TX_IQ_MBUS_TRANSACTION_SIZE_IN_REG          AIC_IQ_TRANSACTION_256_BYTES
#define RX_IQ_MBUS_TRANSACTION_SIZE_IN_BYTES        256
#define TX_IQ_MBUS_TRANSACTION_SIZE_IN_BYTES        256

#define RX_IQ_MBUS_PRIORITY_LEVEL1  0           // Disabled
#define RX_IQ_MBUS_PRIORITY_LEVEL23 0           // Disabled

#define TX_IQ_MBUS_PRIORITY_LEVEL1  0           // Disabled
#define TX_IQ_MBUS_PRIORITY_LEVEL23 0           // Disabled

/* Number of Samples in 1 OFDMA Symbol */
#define     NUM_OF_SAMPLES1(e)     (e->fp.LTE_SYMBOL_LEN + e->fp.LTE_CP0_LEN)  // Number of Samples in First Symbol
#define     NUM_OF_SAMPLES1_MAX     (2048 + 160)  // Number of Samples in First Symbol for 20MHz BW
#define     NUM_OF_SAMPLES2(e)     (e->fp.LTE_SYMBOL_LEN + e->fp.LTE_CPx_LEN)  // Number of Samples in Other Symbols

/* Size of 1 OFDMA Symbol */
#define     MAXIMUM_SYMBOL_SIZE (NUM_OF_SAMPLES1_MAX * 4)   // In Bytes
//#define     MAXIMUM_SYMBOL_SIZE_MAX (NUM_OF_SAMPLES1_MAX * 4)   // In Bytes
#define     SYMBOL_SIZE         (NUM_OF_SAMPLES2 * 4)   // In bytes

/* Tx and Rx IQ Buffer Size in System Memory */
//#define TX_BUFFER_SIZE              (MAXIMUM_SYMBOL_SIZE * LTE_NSYMB_PER_SUBFRAME)    // Size in Bytes for 1 Subframe
//#define RX_BUFFER_SIZE              (MAXIMUM_SYMBOL_SIZE * LTE_NSYMB_PER_SUBFRAME)    // Size in Bytes for 1 Subframe

#ifdef OS_B4_REV1
#define MAX_BD_IN_DISPATCH          4        /**< Number of BD to dispatch each call to osCopChannelDispatch() */
#else
//#define MAX_BD_IN_DISPATCH          6        /**< Number of BD to dispatch each call to osCopChannelDispatch() */
//#define PUFFT_MAX_BD_RING_SIZE		PUFFT_MAX_NUM_BD_FOR_DISPACTH /* 14 symbols */ 
#endif
#define USING_INTERRUPT             TRUE     /**< Should the driver use interrupts or polling */
#define USING_MAPLE_MMU             TRUE     /**< Enable or disable Maple MMU */
#define USING_GO_FUNCTION			TRUE

#define NUM_MAPLES_USED         (MAPLE_0 + MAPLE_1 + MAPLE_2)
#define NUM_PUFFTS_USED         (MAPLE_0_PUFFT + MAPLE_1_PUFFT + MAPLE_2_PUFFT)
#define NUM_PDSCH_USED			(MAPLE_0_PDSCH + MAPLE_1_PDSCH)
#define NUM_PUSCH_USED			(MAPLE_0_PUSCH + MAPLE_1_PUSCH)
#define NUM_TVPE_USED			(MAPLE_0_TVPE + MAPLE_1_TVPE)

#define CPRI_PUFFT_TIMER
#define SOC_TIMER_CPRI_RX	SOC_TIMER32_0

INLINE uint32_t MAX_SYM_SIZE(uint32_t S_RB, bool ext_cp, bool first_in_slot)
{
	uint32_t samples;
	
	switch (S_RB){
		case 6:
			samples = (ext_cp)? 160 : ((first_in_slot)? 138 : 137);
			break;
		case 15:
			samples = (ext_cp)? 320 : ((first_in_slot)? 276 : 274);
			break;
		case 25:
			samples = (ext_cp)? 640 : ((first_in_slot)? 552 : 548);
			break;
		case 50:
			samples = (ext_cp)? 1280 : ((first_in_slot)? 1104 : 1196);
			break;
		case 75:
			samples = (ext_cp)? 1920 : ((first_in_slot)? 1656 : 1644);
			break;
		case 100:
			samples = (ext_cp)? 2560 : ((first_in_slot)? 2208 : 2192);
			break;
		default:
			OS_ASSERT;
			samples = 0;
			break;
	}
	return samples * 4;
}

#define MAX_SYM_0_AND_7_SIZE(S_BW)               MAX_SYM_SIZE(S_BW, FALSE, TRUE)  /**< Symbol 0 and 7 in BYTES */
#define MAX_SYM_1_6_AND_8_13_SIZE(S_BW)          MAX_SYM_SIZE(S_BW, FALSE, FALSE) /**< Symbol 1-6 8-13 in BYTES */
#define MAX_EXT_CP_SYM_0_AND_7_SIZE(S_BW)        MAX_SYM_SIZE(S_BW, TRUE, TRUE)   /**< Symbol 0 and 7 Extended CP in BYTES */
#define MAX_EXT_CP_SYM_1_6_AND_8_11_SIZE(S_BW)   MAX_SYM_SIZE(S_BW, TRUE, FALSE)  /**< Symbol 1-6 8-11 Extended CP in BYTES */
#define MAX_ANT_DATA_SIZE(S_RB)             ((2*MAX_SYM_0_AND_7_SIZE(S_RB) + 12*MAX_SYM_1_6_AND_8_13_SIZE(S_RB)))

#define MAX_EXT_CP_ANT_DATA_SIZE(S_BW)       ((2*MAX_EXT_CP_SYM_0_AND_7_SIZE(S_BW) + 10*MAX_EXT_CP_SYM_1_6_AND_8_11_SIZE(S_BW)))
#if (MAX_ANT_DATA_SIZE != MAX_EXT_CP_ANT_DATA_SIZE)
#error " MAX_ANT_DATA_SIZE != MAX_EXT_CP_ANT_DATA_SIZE "
#endif
/**< Maximum antenna output size in BYTES */

// Helper to get DTX size
enum
{
	DTX_SIZE_CFI1_1_4MHZ = 88,
	DTX_SIZE_CFI2_1_4MHZ = 180,
	DTX_SIZE_CFI1_3MHZ   = 176,
	DTX_SIZE_CFI2_3MHZ   = 360,
	DTX_SIZE_CFI1_5MHZ   = 352,
	DTX_SIZE_CFI2_5MHZ   = 720,
	DTX_SIZE_CFI1_10MHZ  = 708,
	DTX_SIZE_CFI2_10MHZ  = 1440,
	DTX_SIZE_CFI1_15MHZ  = 1506,
	DTX_SIZE_CFI2_15MHZ  = 2160,
	DTX_SIZE_CFI1_20MHZ  = 1408,
	DTX_SIZE_CFI2_20MHZ  = 2880
};
/***************************************************************************/

/***************************************************************************/
/* Memory managers used by the runtime                                     */
/***************************************************************************/
typedef enum
{
    JOBS_MEM_MNGR,
    ANT_DATA_MEM_MNGR,            // manages 2 antennas output buffers
    TB_HDR_MEM_MNGR,
    CW_HDR_MEM_MNGR,
    RB_MAP_TABLE_MEM_MNGR,
    UE_RS_HDR_MEM_MNGR,
    CS_MBSFN_HDR_MEM_MNGR,        // manages 2 headers buffers CS_RS, MBSFN_RS
    POS_RS_HDR_MEM_MNGR,       // manages POS_RS
    EXT_OFDM_SYM_MEM_MNGR,
    PSS_SSS_DATA_MEM_MNGR,        // manages 2 data buffers PSS, SSS
    PBCH_DATA_MEM_MNGR,
    NUM_MEM_MNGR                  // must remain last
} app_mem_mngr_t;

/***************************************************************************/
/* Memory manager structure                                   */
/***************************************************************************/
typedef struct
{
    os_mem_part_t *pool;        /**< Holds the pool for managing the memory */
    uint8_t       *space;       /**< Pool of buffers being managed (MEM_PART_DATA_SIZE) */
    uint8_t       *mem_manager; /**< Memory manager (MEM_PART_SIZE) */
} mem_manager_t;

/* Отладочные дефайны */
//#define DEBUG_DIRECT_DEMODULATION	1
//#define DEBUG_DUMP_SLOT_TX 1
#define DEBUG_DUMP_SLOT_BFN_NUM 100
//#define DEBUG_PBCH 1
//#define DEBUG_TEST_CPRI	1

/* LTE configuration */
/**
 * Количество реализованных eNodeB
 */
//#define NUM_ENODEB	1

/**
 * Количетсво TRX на каждой eNodeB
 */
//#define NUM_TRX	1

/**
 * Тип CP
 */
#define LTE_CP_NORMAL 0
#define LTE_CP_EXTENDED 1

#define LTE_CP	LTE_CP_NORMAL

#if (LTE_CP == LTE_CP_NORMAL)
#define LTE_NSYMB_PER_SUBFRAME	14
#elif (LTE_CP == LTE_CP_EXTENDED)
#define LTE_NSYMB_PER_SUBFRAME	12
#else
#error "Invalid LTE CP type!"
#endif
/**
 * PHICH duration 
 */
#define LTE_PHICH_DURATION_NORMAL	0
#define LTE_PHICH_DURATON_EXTENDED	1

//#define LTE_PHICH_DURATION	LTE_PHICH_DURATION_NORMAL

/**
 * PHICH resource
 */
#define LTE_PHICH_RESOURCE_ONESIXTH	1
#define LTE_PHICH_RESOURCE_HALF		3
#define LTE_PHICH_RESOURCE_ONE		6
#define LTE_PHICH_RESOURCE_TWO		12

//#define LTE_PHICH_RESOURCE	LTE_PHICH_RESOURCE_ONE

/**
 * SISO mode
 */
#define LTE_MODE1_FLAG	1
#define LTE_N_ANTENNAS_MAX	(MAPLE_NUM_ANT)

/* Максимальное количество процессов HARQ
 * 36.213 Sec 7
 */
#define LTE_M_DL_HARQ	8

/* UE capabilities
 * 36.306 Sec 4.1
 */
#define LTE_UE_CAT1_N_SOFT	250368
#define LTE_UE_CAT2_N_SOFT	1237248

/* Количество таймаутов приема DL_CONFIG.req до остановки L1 (в сабфреймах) */
#define MAX_DL_CFG_REQ_TIMEOUTS	20

#define LTE_PRACH_SEQ_LEN_MAX    24576
#define LTE_PRACH_SEQ_CP_MAX     3168
#define LTE_PRACH_SEQ_CP_LEN_MAX (LTE_PRACH_SEQ_LEN_MAX + LTE_PRACH_SEQ_CP_MAX)
#define LTE_SAMPLES_PER_SUBFRAME_MAX 30720
#define LTE_PRACH_DETECT_MAX	4
#define LTE_PRACH_N_PRE_MAX		64
#define LTE_FIRST_CARRIER_OFFSET 0
#define LTE_N_SC_RB	12

/*
#define GAIN_1 (32753 * 512 / 600)
#define GAIN_SQRT_2 (23160 * 512 / 600)
#define GAIN_SQRT_10 (10357 * 512 / 600)
#define GAIN_SQRT_42 (5054 * 512 / 600)
*/

/* Уровни сигналов
 * 
 * Все расчеты нормированы к 1
 * Pmax = 0.95 (0.05 - запас)
 * Nre - количество ресурсных элементо
 * Nfft - размерность FFT
 * 
 * EPRE = (Pmax  * Nrfft / Nre) / Nre = Pmax * Nfft / (Nre^2) 
 * 
 * EPRE(5MHz) = 0.95 * 512 / (300^2) = 0.005404444 = 0.6917688 * 2^-7 = (22667 / 2^15) * 2^-7 
 * Mant = 0.6917688 = 22667 / 2^15
 * Exp = -7
 * 
 */
#define GAIN_1 (32753)// * 512 / LTE_N_RE)
#if 0
#define GAIN_SQRT_2 (23160 * 512 / LTE_N_RE)
#define GAIN_SQRT_10 (10357 * 512 / LTE_N_RE)
#define GAIN_SQRT_42 (5054 * 512 / LTE_N_RE)
#else
//#define GAIN_MOD	(16383)
#define GAIN_MOD	(6538) // 16383 * (1.08 / (1 / sqrt(2)))

#define GAIN_SQRT_2 (((23160) * GAIN_MOD) >> 15)
#define GAIN_SQRT_10 (((10357) * GAIN_MOD) >> 15)
#define GAIN_SQRT_42 (((5054) * GAIN_MOD) >> 15)
#endif

////#define LTE_PDCCH_AMP GAIN_1

#define LTE_IQ_PER_SLOT (LTE_SYMBOL_LEN * 7 + LTE_CP0_LEN + LTE_CPx_LEN * 6)

/***************************************************************************/
INLINE void app_store_barrier()
{
#if (L2CACHE_ENABLE == ON)
            /* Store-to-store barrier, between two cacheable stores.
             * This barrier ensures that the first store is updated in the coherency domain before the second one updates the L2 cache.
             * On the way it clears the SGB from all previous entries.
             * This barrier does not stall the core. */
            DBARS_IBSS_L12();
#elif (OS_SGB_ENABLE == ON)
            /* Store-to-store barrier. The barrier ensures that the two stores will not be packed into a single access in the SGB,
             * and it also drains all prior entries in the SGB.
             * This barrier does not stall the core */
            DBARS_IBSS_L1();
#else
            /* The second access is held until the fabric reports that the first access
             * had reached its final destination */
            DBAR_HWSYNC();
#endif
}

#endif /* APP_CONFIG_H */
