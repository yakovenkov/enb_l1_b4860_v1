/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __LOG_EVENT_H_
#define __LOG_EVENT_H_

#include <stdint.h>
#ifdef SC3X50
#include <app_config.h>
#endif

#ifdef ENABLE_EVENTS_LOG
void log_event_reset();
void log_event(uint32_t id, uint32_t param);
#endif

typedef struct __attribute__((packed))
{
	uint64_t tick;
	uint32_t id;
	uint32_t param;
} log_event_t;

/* Events */
/* Генерал пЁпез */
#define LOGEVT_L1_START		0x00000001
#define LOGEVT_L1_STOP		0x00000002
#define LOGEVT_L1_CONFIGURE	0x00000003
#define LOGEVT_IPC_RECV		0x00000010
#define LOGEVT_IPC_SEND		0x00000011
//#define LOGEVT_FAPI_P7_REQ	0x00000012
//#define LOGEVT_FAPI_P7_IND	0x00000013
#define LOGEVT_RX_SF_SFN	0x00000040
#define LOGEVT_TX_SF_SFN	0x00000041

/* TTI, мап слота, модуляция */
#define LOGEVT_DL_TASK_BEGIN	0x00000100
#define LOGEVT_DL_TASK_END	0x00000101
//#define LOGEVT_DL_SF_EVENT	0x00000102
#define LOGEVT_DL_SF_MAP		0x00000103
#define LOGEVT_DL_SF_MAPLE		0x00000104
#define LOGEVT_DL_SF_MAPLE_DISPATCH	0x00000105

/* Обработка аплинка */
#define LOGEVT_RACH_TASK_BEGIN	0x00000200
#define LOGEVT_RACH_DETECT		0x00000201
#define LOGEVT_RACH_TASK_END	0x00000202
#define LOGEVT_RACH_TASK_EVENT	0x00000203

#define LOGEVT_UL_TASK_BEGIN	0x00000301
#define LOGEVT_UL_TASK_END		0x00000302
#define LOGEVT_UL_PUFFT_SYM		0x00000303
#define LOGEVT_UL_PUFFT_SUBFRAME	0x00000304
#define LOGEVT_UL_PUSCH_READY	0x00000305

#define LOGEVT_HAS_SF_INFO	0x80000000
#define LOGEVT_HAS_P2_INFO	0x40000000

#ifdef ENABLE_EVENTS_LOG
#define LOG_EVENT(x, param) log_event(((x) & 0x0fff) << 16, param)
#define LOG_EVENT_P2(x, param1, param) log_event(LOGEVT_HAS_P2_INFO | ((x & 0x0fff) << 16) | (param1), param)
#define LOG_EVENT_SF(x, sf, sfn, param) log_event(LOGEVT_HAS_SF_INFO | ((x & 0x0fff) << 16) | ((sf) << 4) | (sfn), param)
#else
#define LOG_EVENT(x, y)
#define LOG_EVENT_P2(x, param1, param)
#define LOG_EVENT_SF(x, sf, sfn, param)
#endif

#endif
