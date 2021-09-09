/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */


#ifndef _FAPI_B4860_H_
#define _FAPI_B4860_H_

#include <stdint.h>

#define FAPI_TX_REQ_BUF_SIZE FAPI_IPC_MAX_PA2SC_PTR_SIZE
#define FAPI_TX_REQ_BUF_NUM FAPI_IPC_MAX_PA2SC_PTR_NUM
#define FAPI_P5_MAX_QUEUE_LEN 16
#define FAPI_CPRI_ETH_BUF_SIZE 4096
#define FAPI_CPRI_ETH_BUF_NUM 1024

#define FAPI_IPC_PA2SC_BD_RING_SIZE 16
#define FAPI_IPC_MAX_SC2PA_MSG_SIZE 16384
#define FAPI_IPC_MAX_PA2SC_MSG_SIZE 1020
#define FAPI_IPC_MAX_PA2SC_PTR_SIZE 65536

#define FAPI_IPC_MAX_SC2PA_MSGS_NUM 64
#define FAPI_IPC_MAX_PA2SC_MSGS_NUM 256
#define FAPI_IPC_MAX_PA2SC_PTR_NUM 16

#define FAPI_IPC_RX_IND_SC2PA_MSGS_NUM 16
#define FAPI_IPC_RX_IND_SC2PA_MSG_SIZE (128 * 1024)

#define FAPI_IPC_CPRI_IQ_SC2PA_MSGS_NUM		8
#define FAPI_IPC_CPRI_IQ_SC2PA_MSG_SIZE		(2048 * 1024) // 30720 * 8 * 8 = 1966080 -> 2MB

#define FAPI_IPC_CPRI_ETH_SC2PA_MSGS_NUM	1024
#define FAPI_IPC_CPRI_ETH_SC2PA_MSG_SIZE	4096

#define FAPI_IPC_P8_IND_SC2PA_MSGS_NUM		10
#define FAPI_IPC_P8_IND_SC2PA_MSGS_SIZE		(2048 * 1024)

typedef enum
{
	IPC_DEDICATED_CH_ID = 0,
	IPC_RESERVED1,
	IPC_MSG_CH_ID,
	IPC_RESERVED3,
	IPC_PA2SC_CH_ID,
	IPC_SC2PA_CH_ID,
	NUM_IPC_CH
} ipc_channels_e;

typedef enum
{
	FAPI_CHANNEL_P5_REQ = 0,
	FAPI_CHANNEL_P5_IND,
	FAPI_CHANNEL_P7_REQ,
	FAPI_CHANNEL_P7_IND,
	FAPI_CHANNEL_P7_TX_REQ,
	FAPI_CHANNEL_LOG,
	FAPI_CHANNEL_IQDATA,
	FAPI_CHANNEL_CPRI_ETH,
	FAPI_CHANNEL_P8_REQ,
	FAPI_CHANNEL_P8_IND,

	FAPI_CHANNELS_NUM
} fapi_channels_e;

typedef struct
{
	uint32_t channel_id;
	uint32_t length;
	uint32_t body_addr;
	union {
		uint32_t reserved;
		uint32_t phys_addr;
	};
} fapi_ipc_msg_t;

#define FAPI_IPC_CFG_MAGIC 0x6a45fe67

typedef struct
{
	uint32_t cpri_port;
	uint32_t speed;
} fapi_ipc_rru_cfg_t;

typedef struct
{
	uint8_t license[64];
	uint32_t loglevel;
	uint32_t p8_dump_flags;
	uint32_t p8_dump_limit;
	uint32_t n_rrus;
	fapi_ipc_rru_cfg_t rrus[4];
} fapi_ipc_cfg_v1_t;

typedef struct
{
	uint32_t magic;
	uint32_t version;
	fapi_ipc_cfg_v1_t cfg_v1;
} fapi_ipc_cfg_t;

#if 0
#ifdef __powerpc__
/*
 * Со стороны PA сообщения fapi_ipc_msg_t отправляются по каналу MSG
 * Следовательно, в буфере не надо учитывать их длину
 */
#define FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, l1_msg)                                                               \
	{                                                                                                                  \
		ipc_msg->length = l1_msg->header.message_length;                                                               \
	}
#else
/*
 * Со стороны SC сообщения fapi_ipc_msg_t отправляются через общий буфер
 * Следовательно, их длина должна учитываться
 */
#define FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, l1_msg)                                                               \
	{                                                                                                                  \
		ipc_msg->length = sizeof(fapi_ipc_msg_t) + l1_msg->header.message_length;                                      \
	}
#endif
#else
#define FAPI_IPC_MSG_SET_LENGTH_FROM_L1(ipc_msg, l1_msg)                                                               \
	{                                                                                                                  \
		ipc_msg->length = sizeof(fapi_ipc_msg_t) + l1_msg->header.message_length;                                      \
	}
#endif

typedef unsigned long phys_addr_t;
int32_t fapi_b4860_fapi_cpri_eth_send(void *data, int32_t len);

#endif /* _FAPI_B4860_H_ */
