/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __FAPI_H_
#define __FAPI_H_

#include "smartdsp_os.h"
#include "os_config.h"
//#include "os_cache.h"
//#include "sc3x00_cache.h"
#include "sc39xx_cache.h"
/*
#include "psc9x3x_debug_print.h"
#include "psc9x3x_debug_print_init.h"
*/

#include "app_config.h"

#include <lte_enodeb.h>
#include <log.h>

#include <fapi_interface.h>
#include <fapi_b4860.h>

os_status fapi_init();
fapi_ipc_msg_t *fapi_alloc_send_msg(uint32_t id, void *body_addr);
fapi_ipc_msg_t *fapi_alloc_rx_ind_msg(uint32_t id, void *body_addr);
fapi_ipc_msg_t *fapi_alloc_cpri_eth_msg(uint32_t id, void *body_addr);
fapi_ipc_msg_t *fapi_alloc_cpri_iq_msg(uint32_t id, void *body_addr);
fapi_ipc_msg_t *fapi_alloc_p8_ind_msg(int32_t subframe, void *body_addr);
void fapi_clear_p8_ind_msg(int32_t subframe);
void *fapi_alloc_rx_data_buf();
os_status fapi_p7_send_subframe_indication(uint32_t frame, uint32_t subframe);
os_status fapi_p7_send_rach_indication(uint32_t sf, uint32_t sfn, int32_t n_preambles, int32_t *rnti, int32_t *preamble, int32_t *ta, int32_t *value);
os_status fapi_p7_send_rx_indication(fapi_ipc_msg_t *ipc_msg);
os_status fapi_p7_send_harq_indication(fapi_ipc_msg_t *ipc_msg);
os_status fapi_p7_send_crc_indication(fapi_ipc_msg_t *ipc_msg);
os_status fapi_p7_send_rx_cqi_indication(fapi_ipc_msg_t *ipc_msg);
os_status fapi_p7_send_sr_indication(fapi_ipc_msg_t *ipc_msg);
void fapi_ipc_cb(void *ch, void *data, uint32_t length);
void fapi_cpri_eth_send(fapi_ipc_msg_t *msg);
void fapi_cpri_iq_send(fapi_ipc_msg_t *msg);
os_status fapi_p8_send_indication(fapi_ipc_msg_t *ipc_msg);
uint32_t * fapi_get_iq_data(int32_t sf);

#endif
