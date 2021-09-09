/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "smartdsp_os.h"
#include "os_config.h"
#include "os_runtime.h"
#include "app_config.h"
#include "cpri_b4860.h"

#pragma data_seg_name ".shared_data_ddr0_cacheable"
#pragma bss_seg_name  ".shared_data_ddr0_cacheable_bss"

uint8_t  iq_rx_buffer [NUM_OF_USED_CPRI_UNITS][LTE_N_ANTENNAS_MAX][IQ_RX_BUFFER_SIZE_DEFAULT * NUM_CPRI_BUFFERS] __attribute__ ((aligned (RX_TRANSACTION_SIZE*64))); /* Rx Buffers */
uint8_t  iq_tx_buffer [NUM_OF_USED_CPRI_UNITS][LTE_N_ANTENNAS_MAX][IQ_TX_BUFFER_SIZE_DEFAULT * NUM_CPRI_BUFFERS] __attribute__ ((aligned (TX_TRANSACTION_SIZE*64))); /* Tx Buffers */

uint8_t  vss_rx_buffer [NUM_OF_USED_CPRI_UNITS][MAX_VSS_RX_BUFFER_SIZE] __attribute__ ((aligned (VSS_RX_TRANSACTION_SIZE*64))); /* Rx Buffers */
uint8_t  vss_tx_buffer [NUM_OF_USED_CPRI_UNITS][MAX_VSS_TX_BUFFER_SIZE] __attribute__ ((aligned (VSS_TX_TRANSACTION_SIZE*64))); /* Tx Buffers */

uint8_t ethernet_buffers_space[MEM_PART_DATA_SIZE(ETHERNET_NUM_OF_BUFS, ETHERNET_DATA_SIZE, ALIGNED_64_BYTES)];
