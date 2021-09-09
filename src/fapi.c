/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include "smartdsp_os.h"
#include "os_config.h"
//#include "os_cache.h"
#include "sc39xx_cache.h"
/*
#include "psc9x3x_debug_print.h"
#include "psc9x3x_debug_print_init.h"
*/
#include "b486x_heterogeneous.h"
#include "b486x_heterogeneous_ipc.h"

#include "app_config.h"

#include <lte_enodeb.h>
#include <log.h>

#include <string.h>

#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_OPT_OFF
#pragma opt_level = "O3"
#endif

#define FAPI_P5_TASK_STACK_SIZE (16*1024)
#define FAPI_P7_TASK_STACK_SIZE (16*1024)
#define FAPI_CPRI_IN_TASK_STACK_SIZE (4096) // Min stack size for MMU checks
#define FAPI_CPRI_OUT_TASK_STACK_SIZE (4096) // Min stack size for MMU checks

ARCH_DECLARE_STATIC_STACK(fapi_p5_task_stack, FAPI_P5_TASK_STACK_SIZE);// __attribute__((section(".local_data_ddr0_cacheable_bss")));
//static uint8_t fapi_p5_task_stack[FAPI_P5_TASK_STACK_SIZE] __attribute__((section(".local_data_ddr0_cacheable_bss")));
//#pragma align fapi_p5_task_stack ARCH_CACHE_LINE_SIZE

ARCH_DECLARE_STATIC_STACK(fapi_p7_task_stack, FAPI_P7_TASK_STACK_SIZE);// __attribute__((section(".local_data_ddr0_cacheable_bss")));
//static uint8_t fapi_p7_task_stack[FAPI_P7_TASK_STACK_SIZE] __attribute__((section(".local_data_ddr0_cacheable_bss")));
//#pragma align fapi_p7_task_stack ARCH_CACHE_LINE_SIZE

ARCH_DECLARE_STATIC_STACK(fapi_cpri_eth_in_task_stack, FAPI_CPRI_IN_TASK_STACK_SIZE);
ARCH_DECLARE_STATIC_STACK(fapi_cpri_eth_out_task_stack, FAPI_CPRI_OUT_TASK_STACK_SIZE);

os_task_handle fapi_p5_handle;
os_task_handle fapi_cpri_eth_in_handle;
os_task_handle fapi_cpri_eth_out_handle;

/* Message buffers for IPC */
static uint8_t *fapi_send_msgs_buf;
static uint32_t fapi_send_msgs_ptr = 0;

/* Message buffers for RX.indication */
static uint8_t *fapi_rx_ind_msgs_buf;
static uint32_t fapi_rx_ind_msgs_ptr = 0;

/* Message buffers for CPRI ethernet */
static uint8_t *fapi_cpri_eth_msgs_buf;
static uint32_t fapi_cpri_eth_msgs_ptr = 0;

/* Message buffers for P8.ind */
static uint8_t *fapi_p8_ind_msgs_buf;
static uint32_t fapi_p8_ind_msgs_ptr = 0;


static os_event_handle fapi_p5_evq_msgs;
static os_event_handle fapi_p7_evq_msgs;
static os_event_handle fapi_cpri_eth_out_msgs;
static os_event_handle fapi_cpri_eth_in_msgs;

static void fapi_p5_task(uint32_t p);
static void fapi_cpri_eth_in_task(uint32_t p);
static void fapi_cpri_eth_out_task(uint32_t p);

// Phys-Virt translation offset, calculated at IPC initialization
// Translation: VIRT = (PHYS - ipc_virt_to_phys)
// PHYS = (VIRT + ipc_virt_to_phys)
//int64_t                   ipc_virt_to_phys;//used for translation between heterogeneous IPC structure virtual address and physical address and vice versa at ipc level

os_status fapi_init()
{
	os_status status;
	os_task_init_param_t task_init_params;
	
	/* Indication messages pool */
	fapi_send_msgs_buf = osAlignedMalloc(FAPI_IPC_MAX_SC2PA_MSGS_NUM * FAPI_IPC_MAX_SC2PA_MSG_SIZE,
				OS_MEM_HET_DDR0_SHARED_CACHEABLE, ARCH_CACHE_LINE_SIZE);
	OS_ASSERT_COND(fapi_send_msgs_buf != NULL);
	fapi_send_msgs_ptr = 0;
	
	/* RX.indication messages pool */
	fapi_rx_ind_msgs_buf = osAlignedMalloc(FAPI_IPC_RX_IND_SC2PA_MSGS_NUM * FAPI_IPC_RX_IND_SC2PA_MSG_SIZE,
				OS_MEM_HET_DDR0_SHARED_CACHEABLE, ARCH_CACHE_LINE_SIZE);
	OS_ASSERT_COND(fapi_rx_ind_msgs_buf != NULL);
	fapi_rx_ind_msgs_ptr = 0;

	/* CPRI ethernet messages pool */
	fapi_cpri_eth_msgs_buf = osAlignedMalloc(FAPI_IPC_CPRI_ETH_SC2PA_MSGS_NUM * FAPI_IPC_CPRI_ETH_SC2PA_MSG_SIZE,
				OS_MEM_HET_DDR0_SHARED_CACHEABLE, ARCH_CACHE_LINE_SIZE);
	OS_ASSERT_COND(fapi_cpri_eth_msgs_buf != NULL);
	fapi_cpri_eth_msgs_ptr = 0;

	/* P8 ind messages pool */
	fapi_p8_ind_msgs_buf = osAlignedMalloc(FAPI_IPC_P8_IND_SC2PA_MSGS_NUM * FAPI_IPC_P8_IND_SC2PA_MSGS_SIZE,
				OS_MEM_HET_DDR0_SHARED_CACHEABLE, ARCH_CACHE_LINE_SIZE);
	OS_ASSERT_COND(fapi_p8_ind_msgs_buf != NULL);
	fapi_p8_ind_msgs_ptr = 0;

	/* P5 & P7 messages event queues */
	status = osEventQueueFind(&fapi_p5_evq_msgs);
	OS_ASSERT_COND(status == OS_SUCCESS);
	status = osEventQueueCreate(fapi_p5_evq_msgs, FAPI_IPC_PA2SC_BD_RING_SIZE);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventQueueFind(&fapi_p7_evq_msgs);
	OS_ASSERT_COND(status == OS_SUCCESS);
	
	status = osEventQueueCreate(fapi_p7_evq_msgs, FAPI_IPC_PA2SC_BD_RING_SIZE);
	OS_ASSERT_COND(status == OS_SUCCESS);

	/* FAPI P5 messages handler task */
	status = osTaskFind(&fapi_p5_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	task_init_params.task_function = fapi_p5_task;
	task_init_params.task_arg = 0;
	task_init_params.top_of_stack = (uint32_t) fapi_p5_task_stack;
	task_init_params.stack_size = FAPI_P5_TASK_STACK_SIZE;
	task_init_params.task_priority = TASK_PRIORITY_FAPI_P5;
	task_init_params.task_name = "FAPI P5 handler task";
	task_init_params.private_data = 0;

	status = osTaskCreate(fapi_p5_handle, &task_init_params);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osTaskActivate(fapi_p5_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	/* CPRI ethernet messages event queues */
	status = osEventQueueFind(&fapi_cpri_eth_in_msgs);
	OS_ASSERT_COND(status == OS_SUCCESS);
	status = osEventQueueCreate(fapi_cpri_eth_in_msgs, FAPI_IPC_PA2SC_BD_RING_SIZE);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osEventQueueFind(&fapi_cpri_eth_out_msgs);
	OS_ASSERT_COND(status == OS_SUCCESS);
	status = osEventQueueCreate(fapi_cpri_eth_out_msgs, FAPI_IPC_PA2SC_BD_RING_SIZE);
	OS_ASSERT_COND(status == OS_SUCCESS);
	
	status = osTaskFind(&fapi_cpri_eth_in_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);
	
	task_init_params.task_function = fapi_cpri_eth_in_task;
	task_init_params.task_arg = 0;
	task_init_params.top_of_stack = (uint32_t) fapi_cpri_eth_in_task_stack;
	task_init_params.stack_size = FAPI_CPRI_IN_TASK_STACK_SIZE;
	task_init_params.task_priority = TASK_PRIORITY_CPRI_ETH_IN;
	task_init_params.task_name = "FAPI CPRI IN handler task";
	task_init_params.private_data = 0;

	status = osTaskCreate(fapi_cpri_eth_in_handle, &task_init_params);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osTaskActivate(fapi_cpri_eth_in_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);
	
	status = osTaskFind(&fapi_cpri_eth_out_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);
	
	task_init_params.task_function = fapi_cpri_eth_out_task;
	task_init_params.task_arg = 0;
	task_init_params.top_of_stack = (uint32_t) fapi_cpri_eth_out_task_stack;
	task_init_params.stack_size = FAPI_CPRI_OUT_TASK_STACK_SIZE;
	task_init_params.task_priority = TASK_PRIORITY_CPRI_ETH_OUT;
	task_init_params.task_name = "FAPI CPRI OUT handler task";
	task_init_params.private_data = 0;

	status = osTaskCreate(fapi_cpri_eth_out_handle, &task_init_params);
	OS_ASSERT_COND(status == OS_SUCCESS);

	status = osTaskActivate(fapi_cpri_eth_out_handle);
	OS_ASSERT_COND(status == OS_SUCCESS);

	return OS_SUCCESS;
}

fapi_ipc_msg_t *fapi_alloc_send_msg(uint32_t id, void *body_addr)
{
	fapi_ipc_msg_t *ptr;
	
	osHwiSwiftDisable();
	
	ptr = (fapi_ipc_msg_t *)&fapi_send_msgs_buf[fapi_send_msgs_ptr];
	ptr->channel_id = id;
	ptr->body_addr = (uint32_t)((uint8_t*)ptr + sizeof(fapi_ipc_msg_t));
	
	fapi_send_msgs_ptr = (fapi_send_msgs_ptr + FAPI_IPC_MAX_SC2PA_MSG_SIZE) & 
			(FAPI_IPC_MAX_SC2PA_MSGS_NUM*FAPI_IPC_MAX_SC2PA_MSG_SIZE-1); 
	
	osHwiSwiftEnable();
	
	if(body_addr)
		*(uint32_t*)body_addr = (uint32_t)ptr->body_addr;
	
	return ptr;
}

fapi_ipc_msg_t *fapi_alloc_cpri_eth_msg(uint32_t id, void *body_addr)
{
	fapi_ipc_msg_t *ptr;
	
	osHwiSwiftDisable();
	
	ptr = (fapi_ipc_msg_t *)&fapi_cpri_eth_msgs_buf[fapi_cpri_eth_msgs_ptr];
	ptr->channel_id = id;
	ptr->body_addr = (uint32_t)((uint8_t*)ptr + sizeof(fapi_ipc_msg_t));
	
	fapi_cpri_eth_msgs_ptr = (fapi_cpri_eth_msgs_ptr + FAPI_IPC_CPRI_ETH_SC2PA_MSG_SIZE) & 
			(FAPI_IPC_CPRI_ETH_SC2PA_MSGS_NUM*FAPI_IPC_CPRI_ETH_SC2PA_MSG_SIZE-1); 
	
	osHwiSwiftEnable();
	
	if(body_addr)
		*(uint32_t*)body_addr = (uint32_t)ptr->body_addr;
	
	return ptr;
}

void *fapi_alloc_rx_data_buf()
{
	void *ptr;
	
	osHwiSwiftDisable();
	
	ptr = &fapi_rx_ind_msgs_buf[fapi_rx_ind_msgs_ptr];
	
	fapi_rx_ind_msgs_ptr = (fapi_rx_ind_msgs_ptr + FAPI_IPC_RX_IND_SC2PA_MSG_SIZE) & 
			(FAPI_IPC_RX_IND_SC2PA_MSGS_NUM * FAPI_IPC_RX_IND_SC2PA_MSG_SIZE-1); 
	
	osHwiSwiftEnable();
	
	return ptr;
}

fapi_ipc_msg_t *fapi_alloc_p8_ind_msg(int32_t subframe, void *body_addr)
{
	fapi_ipc_msg_t *ptr;
	
	osHwiSwiftDisable();
	
	fapi_p8_ind_msgs_ptr = subframe * FAPI_IPC_P8_IND_SC2PA_MSGS_SIZE;
	
	ptr = (fapi_ipc_msg_t *)&fapi_p8_ind_msgs_buf[fapi_p8_ind_msgs_ptr];
	ptr->channel_id = FAPI_CHANNEL_P8_IND;
	ptr->body_addr = (uint32_t)((uint8_t*)ptr + sizeof(fapi_ipc_msg_t));
/*	
	fapi_p8_ind_msgs_ptr = (fapi_p8_ind_msgs_ptr + FAPI_IPC_P8_IND_SC2PA_MSGS_SIZE) & 
			(FAPI_IPC_P8_IND_SC2PA_MSGS_NUM * FAPI_IPC_P8_IND_SC2PA_MSGS_SIZE-1); 
	*/
	osHwiSwiftEnable();
	
	if(body_addr)
		*(uint32_t*)body_addr = (uint32_t)ptr->body_addr;
#if 0
	if(pdu_data_offset)
	{
		fapi_p8_indication_t *p8_ind = (fapi_p8_indication_t *) ptr->body_addr;
		int32_t off = 0;
		
		if(p8_ind->number_of_pdus > 0)
		{
			fapi_p8_ind_pdu_t *ind_pdu = &p8_ind->pdus[p8_ind->number_of_pdus - 1];
			off = ALIGN_SIZE(ind_pdu->offset + ind_pdu->length, ARCH_CACHE_LINE_SIZE);
		}
		
		*pdu_data_offset = off;
	}
#endif
	return ptr;
}

void fapi_clear_p8_ind_msg(int32_t subframe)
{
	fapi_p8_indication_t *p8_ind;
	
	fapi_alloc_p8_ind_msg(subframe, &p8_ind);
	p8_ind->number_of_pdus = 0;
	p8_ind->header.message_length = 0;
}

os_status fapi_p7_send_subframe_indication(uint32_t frame, uint32_t subframe)
{
	fapi_ipc_msg_t *ipc_msg;
	fapi_subframe_indication_t *msg;

	ipc_msg = fapi_alloc_send_msg(FAPI_CHANNEL_P7_IND, &msg);
		
	if(msg == NULL)
	{
		ERROR(DFAPI, "Error allocating P7 message\n");
		return OS_FAIL;
	}
	
	msg->header.message_id = FAPI_SUBFRAME_INDICATION;
	msg->header.message_length = sizeof(fapi_subframe_indication_t);
	msg->frame = frame;
	msg->subframe = subframe;
	
	//sweep_cache((uint32_t)msg, sizeof(fapi_subframe_indication_t), CACHE_FLUSH);
	
	FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, msg);
	
	ipc_send_msg(ipc_msg);
	
	return OS_SUCCESS;
}

os_status fapi_p7_send_rach_indication(uint32_t frame, uint32_t subframe, int32_t n_preambles, 
		int32_t *rnti, int32_t *preamble, int32_t *ta, int32_t *value)
{
	fapi_ipc_msg_t *ipc_msg;
	fapi_rach_indication_t *msg;
	int32_t i;

	ipc_msg = fapi_alloc_send_msg(FAPI_CHANNEL_P7_IND, &msg);
	
	if(ipc_msg == NULL)
	{
		ERROR(DFAPI, "Error allocating FAPI message\n");
		return OS_FAIL;
	}
	
	msg = (fapi_rach_indication_t *)ipc_msg->body_addr;

	msg->header.message_id = FAPI_RACH_INDICATION;
	
	msg->frame = frame;
	msg->subframe = subframe;
	msg->rach_indication_body.number_of_preambles = n_preambles;
	
	msg->header.message_length = FAPI_GET_MSG_PTR_VAR_SIZE(msg, rach_indication_body.preamble_list, 
				msg->rach_indication_body.number_of_preambles);
	
	for(i=0; i<n_preambles; i++)
	{
		msg->rach_indication_body.preamble_list[i].r8.rnti = rnti[i];
		msg->rach_indication_body.preamble_list[i].r8.preamble = preamble[i];
		msg->rach_indication_body.preamble_list[i].r8.timing_advance = ta[i];
		msg->rach_indication_body.preamble_list[i].r8.power = (uint32_t)value[i];
	}
	
	//sweep_cache((uint32_t)msg, sizeof(fapi_rach_indication_t), CACHE_FLUSH);
	FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, msg);
	
	ipc_send_msg(ipc_msg);
	
	return OS_SUCCESS;
}

os_status fapi_p7_send_rx_indication(fapi_ipc_msg_t *ipc_msg)
{
	int32_t i;
	fapi_rx_indication_t *l1_msg = (fapi_rx_indication_t *)ipc_msg->body_addr;
	
	l1_msg->header.message_id = FAPI_RX_ULSCH_INDICATION;

	/* Преобразование виртуального адреса данных в физический
	 * TODO: переделать без использования osMmuDataVirtToPhys (через смещение virt->phys)
	 */

	for(i=0; i< l1_msg->rx_indication_body.number_of_pdus; i++)
	{
		int64_t phys_addr;
				
		osMmuDataVirtToPhys((void *)l1_msg->rx_indication_body.rx_pdu_list[i].data_ptr, &phys_addr);
		l1_msg->rx_indication_body.rx_pdu_list[i].data_ptr = (void *)phys_addr;
	}
	
	FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, l1_msg);
	
	ipc_send_msg(ipc_msg);
	
	return OS_SUCCESS;
}

os_status fapi_p7_send_harq_indication(fapi_ipc_msg_t *ipc_msg)
{
	fapi_harq_indication_t *l1_msg = (fapi_harq_indication_t *)ipc_msg->body_addr;
	int32_t i;
	
	l1_msg->header.message_id = FAPI_HARQ_INDICATION;
	
	for(i=0; i<l1_msg->harq_indication_body.number_of_harqs; i++)
	{
		if(l1_msg->harq_indication_body.harq_pdu_list[i].harq_indication_fdd_rel8.harq_tb1 == 0)
			l1_msg->harq_indication_body.harq_pdu_list[i].harq_indication_fdd_rel8.harq_tb1 = 2;
		
		if(l1_msg->harq_indication_body.harq_pdu_list[i].harq_indication_fdd_rel8.harq_tb2 == 0)
			l1_msg->harq_indication_body.harq_pdu_list[i].harq_indication_fdd_rel8.harq_tb2 = 2;
	}
	
	FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, l1_msg);
	
	ipc_send_msg(ipc_msg);
	
	return OS_SUCCESS;
}

os_status fapi_p7_send_crc_indication(fapi_ipc_msg_t *ipc_msg)
{
	fapi_crc_indication_t *l1_msg = (fapi_crc_indication_t *)ipc_msg->body_addr;
	
	l1_msg->header.message_id = FAPI_CRC_INDICATION;
	
	FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, l1_msg);
	
	ipc_send_msg(ipc_msg);
	
	return OS_SUCCESS;
}

os_status fapi_p7_send_rx_cqi_indication(fapi_ipc_msg_t *ipc_msg)
{
	fapi_cqi_indication_t *l1_msg = (fapi_cqi_indication_t *)ipc_msg->body_addr;
	
	l1_msg->header.message_id = FAPI_RX_CQI_INDICATION;
	
	FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, l1_msg);
	
	ipc_send_msg(ipc_msg);
	
	return OS_SUCCESS;
}

os_status fapi_p7_send_sr_indication(fapi_ipc_msg_t *ipc_msg)
{
	fapi_sr_indication_t *l1_msg = (fapi_sr_indication_t *)ipc_msg->body_addr;
	
	l1_msg->header.message_id = FAPI_RX_SR_INDICATION;
	
	FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, l1_msg);
	
	ipc_send_msg(ipc_msg);
	
	return OS_SUCCESS;
}

void fapi_p5_task(uint32_t p)
{
	fapi_ipc_msg_t *ipc_msg;
	fapi_l1_message_header_t *l1_msg;
	os_status status;
	lte_status_t lte_status;
	fapi_ipc_msg_t *resp_msg;
	
	while(1)
	{
		status = osEventQueuePend(fapi_p5_evq_msgs, (uint32_t *) &ipc_msg, 0);
		OS_ASSERT_COND(status == OS_SUCCESS);
		
		// Адрес ipc_msg->body_addr физический, перед работой его необходимо привести к адресации DSP
		os_virt_ptr body_addr_virt;
		OS_ASSERT_COND(osMmuDataPhysToVirtManual(ipc_msg->body_addr, &body_addr_virt))
		
		osCacheLoadBarrier(L1_L2_CACHED);
		
		__dcm_inval_l12((Word32 *)body_addr_virt);
		
		l1_msg = (fapi_l1_message_header_t *)(body_addr_virt);
		
		switch(l1_msg->message_id)
		{
			case FAPI_CONFIG_REQUEST:
				INFO(DFAPI, "P5 CONFIG.request received\n");
				fapi_config_t *new_config = (fapi_config_t *)l1_msg;
				// FIXME: динамический номер eNodeB
				lte_status = lte_enodeb_configure(&g_enodeb_inst[0], new_config);
				
				if(lte_status != LTE_SUCCESS)
				{
					ERROR(DFAPI, "P5 invalid lte_enodeb_configure() status %i\n", lte_status);
				}
				
				fapi_config_response_t *cfg_response;
				resp_msg = fapi_alloc_send_msg(FAPI_CHANNEL_P5_IND, &cfg_response);
				
				cfg_response->header.message_id = FAPI_CONFIG_RESPONSE;
				cfg_response->header.message_length = sizeof(fapi_config_response_t);
				cfg_response->error_code = (lte_status == LTE_SUCCESS) ? FAPI_MSG_OK : FAPI_MSG_INVALID_CONFIG;
				
				FAPI_IPC_MSG_SET_LENGTH_FROM_L1(resp_msg, cfg_response);
				
				ipc_send_msg(resp_msg);
			
				break;
				
			case FAPI_START_REQUEST:
				INFO(DFAPI, "P5 START.request received\n");
				// FIXME: динамический номер eNodeB
				lte_status = lte_enodeb_start(&g_enodeb_inst[0]);
				if(lte_status != LTE_SUCCESS)
				{
					ERROR(DFAPI, "P5 invalid lte_enodeb_start() status %i\n", lte_status);
				}

				break;
				
			case FAPI_STOP_REQUEST:
				INFO(DFAPI, "P5 STOP.request received\n");
				// FIXME: динамический номер eNodeB
				g_enodeb_inst[0].flag_stop = 1;
				/*
				lte_status = lte_enodeb_stop();
				if(lte_status != LTE_SUCCESS)
				{
					ERROR(DFAPI, "P5 invalid lte_enodeb_stop() status %i\n", lte_status);
				}
				*/
				
				break;
				
			default:
				ERROR(DFAPI, "P5 invalid message ID: 0x%02x", l1_msg->message_id);
				break;
		}
	}
}

void fapi_p7_task_direct(fapi_ipc_msg_t *ipc_msg)
{
	fapi_l1_message_header_t *l1_msg;
	
	// Адрес ipc_msg->body_addr физический, перед работой его необходимо привести к адресации DSP
	os_virt_ptr body_addr_virt;
	
	OS_ASSERT_COND(osMmuDataPhysToVirtManual(ipc_msg->body_addr, &body_addr_virt))	
	DBAR_IBSL();
	
	l1_msg = (fapi_l1_message_header_t *)(body_addr_virt);
	
	//INFO(DTRX, "R %08x %08x t %i\n", ipc_msg, l1_msg, l1_msg->message_id);

	switch(l1_msg->message_id)
	{
		case FAPI_DL_CONFIG_REQUEST:
		{
			fapi_dl_config_request_t *dl_cfg_req = (fapi_dl_config_request_t *)l1_msg;
			DBG(DFAPI, "P7 DL_CONFIG.request %i:%i\n", dl_cfg_req->sfn_sf >> 4, dl_cfg_req->sfn_sf & 0x0f);
			// FIXME: динамический номер eNodeB
			g_enodeb_inst[0].trx.fapi_dl_config[dl_cfg_req->sfn_sf & 0x0f] = dl_cfg_req;
			
		}
			break;
			
		case FAPI_TX_REQUEST:
		{
			//DBG(DFAPI, "P7 TX_REQUEST\n");
			fapi_tx_request_t *tx_req = (fapi_tx_request_t *)l1_msg;
			DBG(DFAPI, "P7 TX_REQUEST %i:%i\n", tx_req->sfn_sf >> 4, tx_req->sfn_sf & 0x0f);
			// FIXME: динамический номер eNodeB
			g_enodeb_inst[0].trx.fapi_tx_req[tx_req->sfn_sf & 0x0f] = tx_req;
		}
			break;
			
		case FAPI_UL_CONFIG_REQUEST:
		{
			fapi_ul_config_request_t *ul_cfg_req = (fapi_ul_config_request_t *)l1_msg;
			DBG(DFAPI, "P7 UL_CONFIG.request %i:%i\n", ul_cfg_req->sfn_sf >> 4, ul_cfg_req->sfn_sf & 0x0f);
			// FIXME: динамический номер eNodeB
			g_enodeb_inst[0].trx.fapi_ul_config[ul_cfg_req->sfn_sf & 0x0f] = ul_cfg_req;
		}
			break;

		case FAPI_HI_DCI0_REQUEST:
		{
			fapi_hi_dci0_request_t *hi_dci0_req = (fapi_hi_dci0_request_t *)l1_msg;
			DBG(DFAPI, "P7 HI_DCI0.request %i:%i\n", hi_dci0_req->sfn_sf >> 4, hi_dci0_req->sfn_sf & 0x0f);
			// FIXME: динамический номер eNodeB
			g_enodeb_inst[0].trx.fapi_hi_dci0[hi_dci0_req->sfn_sf & 0x0f] = hi_dci0_req;
		}
			break;

		default:
			ERROR(DFAPI, "P7 invalid message ID: 0x%02x\n", l1_msg->message_id);
			break;
	}
}

/* Обработчик канала P5 FAPI */
void fapi_ipc_cb(void* ch_ptr, void* data, uint32_t length)
{
	// FIXME: сделать определение текущей eNodeB по номеру IPC канала
	lte_enodeb_t *enodeb = &g_enodeb_inst[0];
	lte_trx_t *trx = &enodeb->trx;
	
	fapi_ipc_msg_t *msg;
	OS_ASSERT_COND(data != NULL);

	//msg_data = data;
	
	//while(msg_data != NULL)
	//{
	
		/* Цикл вычитывания всех сообщений из кольца
		 * 
		 * Теоретически, их может оказаться больше одного на 1 прерывание
		 */
		//msg = (fapi_ipc_msg_t *)msg_data;
		msg = (fapi_ipc_msg_t *)data;
		// FIXME: SC3900
		//sweep_cache((uint32_t)msg->body_addr, msg->length, /*CACHE_FLUSH*/CACHE_INVALIDATE);
		
		LOG_EVENT(LOGEVT_IPC_RECV, msg->channel_id);
		
		// Адрес msg->body_addr физический, перед отправкой обработчику его неоходимо привести к адресному пространству DSP
		os_virt_ptr body_addr_virt;
		
		if(msg->body_addr == 0)
			return;
		
		OS_ASSERT_COND(osMmuDataPhysToVirtManual(msg->body_addr, &body_addr_virt) == OS_SUCCESS);
		
		osCacheLoadBarrier(L1_L2_CACHED);
		
		switch(msg->channel_id)
		{
			case FAPI_CHANNEL_P5_REQ:
				osEventQueuePost(fapi_p5_evq_msgs, (uint32_t)(body_addr_virt), NULL);
				break;
				
			case FAPI_CHANNEL_P7_REQ:
				//INFO(DFAPI, "FAPI P7\n");
				fapi_p7_task_direct((fapi_ipc_msg_t *)body_addr_virt);			
				break;
				
			case FAPI_CHANNEL_CPRI_ETH:
				//osEventQueuePost(fapi_cpri_eth_in_msgs, (uint32_t)(body_addr_virt), NULL);
				if(enodeb->state == L1_RUNNING)
				{
					fapi_ipc_msg_t *cpri_ipc_msg = (fapi_ipc_msg_t *)body_addr_virt;
					if(cpri_ipc_msg->length > 0)
					{
						// Адрес ipc_msg->body_addr физический, перед работой его необходимо привести к адресации DSP
						os_virt_ptr body_addr_virt;
						OS_ASSERT_COND(osMmuDataPhysToVirtManual(cpri_ipc_msg->body_addr, &body_addr_virt))
								
						cpri_eth_tx(enodeb, body_addr_virt, cpri_ipc_msg->length - sizeof(fapi_ipc_msg_t));
					}
				}
				break;
			
			default:
				/* Неизвестный канал, дропаем сообщение */
				ERROR(DFAPI, "Unknown FAPI channel %i\n", msg->channel_id);
				break;
		}
		
		//msg_data = ipc_recv_msg();
	//}
}

/**
 * CPRI ethernet IN (from PPC) processing 
 */
void fapi_cpri_eth_in_task(uint32_t p)
{
	fapi_ipc_msg_t *ipc_msg;
	os_status status;
	
	while(1)
	{
		status = osEventQueuePend(fapi_cpri_eth_in_msgs, (uint32_t *) &ipc_msg, 0);
		OS_ASSERT_COND(status == OS_SUCCESS);
		
		if(ipc_msg->length > 0)
		{
			// Адрес ipc_msg->body_addr физический, перед работой его необходимо привести к адресации DSP
			os_virt_ptr body_addr_virt;
			OS_ASSERT_COND(osMmuDataPhysToVirtManual(ipc_msg->body_addr, &body_addr_virt))
			
			osCacheLoadBarrier(L1_L2_CACHED);
			
			// Перенесено в обработчик FAPI IPC
			//cpri_eth_tx(body_addr_virt, ipc_msg->length - sizeof(fapi_ipc_msg_t));
		}
	}
}

/**
 * CPRI ethernet OUT (to PPC) processing 
 */
void fapi_cpri_eth_send(fapi_ipc_msg_t *msg)
{
	msg->channel_id = FAPI_CHANNEL_CPRI_ETH;
	//osEventQueuePost(fapi_cpri_eth_out_msgs, (uint32_t)msg, NULL);
	ipc_send_msg(msg);
}

/**
 * P8.ind (to PPC) processing 
 */
os_status fapi_p8_send_indication(fapi_ipc_msg_t *ipc_msg)
{
	int32_t i;
	fapi_p8_indication_t *l1_msg = (fapi_p8_indication_t *)ipc_msg->body_addr;
	
	l1_msg->header.message_length = 0;
	
	for(i=0; i<l1_msg->number_of_pdus; i++)
	{
		l1_msg->header.message_length += l1_msg->pdus[i].length;
	}
	
	FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, l1_msg);
	
	ipc_send_msg(ipc_msg);
	
	return OS_SUCCESS;
}

void fapi_cpri_eth_out_task(uint32_t p)
{
	fapi_ipc_msg_t *ipc_msg;
	os_status status;
	
	while(1)
	{
		status = osEventQueuePend(fapi_cpri_eth_out_msgs, (uint32_t *) &ipc_msg, 0);
		OS_ASSERT_COND(status == OS_SUCCESS);
		
		if(ipc_msg->length > 0)
		{
			ipc_send_msg(ipc_msg);
		}
	}
}
