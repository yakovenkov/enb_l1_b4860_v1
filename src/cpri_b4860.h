/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __CPRI_B4860_H_
#define __CPRI_B4860_H_

#include "cpri.h"
#include "cpri_init.h"
//#include <lte_enodeb.h>

#define NUM_OF_USED_CPRI_UNITS                              1

#if 0
#define NUM_OF_CHANNELS_IN_TEST                             1                   /* Number of channels in test  */
#define NUM_OF_CHANNELS_IN_TEST_AFTER_RECONFIGURATION       3                   /* Number of channels in test  */
#if NUM_OF_CHANNELS_IN_TEST > NUM_OF_CHANNELS_IN_TEST_AFTER_RECONFIGURATION
#define MAX_NUM_OF_CHANNELS_IN_TEST   NUM_OF_CHANNELS_IN_TEST
#else
#define MAX_NUM_OF_CHANNELS_IN_TEST   NUM_OF_CHANNELS_IN_TEST_AFTER_RECONFIGURATION
#endif
#endif

/* Размеры буферов CPRI в байтах для различных полос */
#define CPRI_LTE_SUBFRAME_SIZE_20MHZ	(30720 * 4)
#define CPRI_LTE_SUBFRAME_SIZE_10MHZ	(15360 * 4)
#define CPRI_LTE_SUBFRAME_SIZE_5MHZ		(7680 * 4)
#define CPRI_LTE_SUBFRAME_SIZE_3MHZ		(3840 * 4)

#define IQ_TX_BUFFER_SIZE_DEFAULT CPRI_LTE_SUBFRAME_SIZE_5MHZ
#define IQ_RX_BUFFER_SIZE_DEFAULT CPRI_LTE_SUBFRAME_SIZE_5MHZ

#define IQ_TX_BUFFER_SIZE_MAX CPRI_LTE_SUBFRAME_SIZE_20MHZ
#define IQ_RX_BUFFER_SIZE_MAX CPRI_LTE_SUBFRAME_SIZE_20MHZ

/* Количество используемых буферов для каждой антенны 
 * 2 - режим Ping-Pong
 */
#define NUM_CPRI_BUFFERS	2

/* Порог срабатывания прерывания буфера приема/передачи */
//#define IQ_TX_THRESHOLD_SIZE IQ_TX_BUFFER_SIZE/2 // one buffer
//#define IQ_RX_THRESHOLD_SIZE IQ_RX_BUFFER_SIZE/2 // one bueffer
//#define IQ_RX_THRESHOLD_SIZE 256 // 256 bytes rx threshold used for CPRI timer counting for PUFFT
//#define NUM_OF_BUFFERS                                      2                  /* Number of buffers   */
#if 0
#define VSS_TX_BUFFER_SIZE           ((16 * 5 * 150) * 2)               /* Size of each tx buffer */
#define VSS_RX_BUFFER_SIZE           ((16 * 5 * 150) * 2)               /* Size of each rx buffer */
#define VSS_TX_THRESHOLD_SIZE        VSS_TX_BUFFER_SIZE/2
#define VSS_RX_THRESHOLD_SIZE        VSS_RX_BUFFER_SIZE/2
#endif

#define VSS_TX_BUFFER_SIZE           ((64) * 2)               /* Size of each tx buffer */
#define VSS_RX_BUFFER_SIZE           ((64) * 2)               /* Size of each rx buffer */
#define VSS_TX_THRESHOLD_SIZE        VSS_TX_BUFFER_SIZE/2
#define VSS_RX_THRESHOLD_SIZE        VSS_RX_BUFFER_SIZE/2

#define VSS_TX_THRESHOLD_SIZE_AFTER_RECONFIGURATION     VSS_TX_BUFFER_SIZE_AFTER_RECONFIGURATION/2
#define VSS_RX_THRESHOLD_SIZE_AFTER_RECONFIGURATION     VSS_RX_BUFFER_SIZE_AFTER_RECONFIGURATION/2

#if VSS_TX_BUFFER_SIZE_AFTER_RECONFIGURATION > VSS_TX_BUFFER_SIZE
#define MAX_VSS_TX_BUFFER_SIZE      VSS_TX_BUFFER_SIZE_AFTER_RECONFIGURATION
#else
#define MAX_VSS_TX_BUFFER_SIZE      VSS_TX_BUFFER_SIZE
#endif

#if VSS_RX_BUFFER_SIZE_AFTER_RECONFIGURATION > VSS_RX_BUFFER_SIZE
#define MAX_VSS_RX_BUFFER_SIZE      VSS_RX_BUFFER_SIZE_AFTER_RECONFIGURATION
#else
#define MAX_VSS_RX_BUFFER_SIZE      VSS_RX_BUFFER_SIZE
#endif

#define VSS_RX_TRANSACTION_SIZE		CPRI_VSS_TRANSACTION_64_BYTES
#define VSS_TX_TRANSACTION_SIZE		CPRI_VSS_TRANSACTION_64_BYTES

/* Размер транзакции CPRI DMA в байтах */
//#define RX_TRANSACTION_SIZE                                 CPRI_IQ_TRANSACTION_512_BYTES
//#define TX_TRANSACTION_SIZE                                 CPRI_IQ_TRANSACTION_512_BYTES
#define RX_TRANSACTION_SIZE                                 CPRI_IQ_TRANSACTION_256_BYTES
#define TX_TRANSACTION_SIZE                                 CPRI_IQ_TRANSACTION_256_BYTES

#define ETHERNET_BD_RING_SIZE            8
#define ETHERNET_NUM_OF_BUFS        (NUM_OF_USED_CPRI_UNITS * (ETHERNET_BD_RING_SIZE * 4))
#define ETHERNET_CRC_SIZE                4
#define ETHERNET_DATA_SIZE          0x600 /* must be multiply of 0x100 */
#define ETHERNET_RX_BUFFER_SIZE          0x4000      /* Size of each rx buffer */
#define ETHERNET_NUM_OF_TX_FRAMES   20
#define ETHERNET_FRAMES_IN_POOL     (NUM_OF_USED_CPRI_UNITS * ((ETHERNET_NUM_OF_TX_FRAMES * 2) + 4))

#if defined(B4860) && !defined(TEST_RESET_REQUEST)
#define RECONFIGURATION_LEVEL1_APPLY    // if defined - apply reconfiguration level1 and run another session
#endif //B4860

typedef struct lte_enodeb_t lte_enodeb_t;

os_status cpri_init_top(lte_enodeb_t *enodeb);
os_status cpri_reinit(lte_enodeb_t *enodeb);
void cpri_setup_buffers(lte_enodeb_t *enodeb);
void cpri_start(lte_enodeb_t *enodeb);
void cpri_stop(lte_enodeb_t *enodeb);
void cpri_eth_tx(lte_enodeb_t *enodeb, void *buf, int32_t len);
void cpri_print_counters(lte_enodeb_t *enodeb);
#endif
