/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __LTE_DEFS_H_
#define __LTE_DEFS_H_

#include "dci.h"

#define NSOFT 1827072

#define MAX_NUM_DLSCH_SEGMENTS 16
#define MAX_NUM_ULSCH_SEGMENTS MAX_NUM_DLSCH_SEGMENTS
#define MAX_DLSCH_PAYLOAD_BYTES (MAX_NUM_DLSCH_SEGMENTS*768)
#define MAX_ULSCH_PAYLOAD_BYTES (MAX_NUM_ULSCH_SEGMENTS*768)

#define MAX_NUM_CHANNEL_BITS (14*1200*6)  // 14 symbols, 1200 REs, 12 bits/RE
#define MAX_NUM_RE (14*1200)

typedef enum
{
	format0,
	format1,
	format1A,
	format1A_RA,
	format1B,
	format1C,
	format1D,
	format1E_2A_M10PRB,
	format2_2A_L10PRB,
	format2_2A_M10PRB,
	format2_4A_L10PRB,
	format2_4A_M10PRB,
	format2A_2A_L10PRB,
	format2A_2A_M10PRB,
	format2A_4A_L10PRB,
	format2A_4A_M10PRB,
	format3
} DCI_format_t;

typedef enum
{
	pucch_format1 = 0, pucch_format1a, pucch_format1b, pucch_format2, pucch_format2a, pucch_format2b, pucch_format_error
} PUCCH_FMT_t;

typedef struct
{
	/// Length of DCI in bits
	uint8_t dci_length;
	/// Aggregation level 
	uint8_t L;
	/// Position of first CCE of the dci
	uint32_t first_CCE;
	/// flag to indicate that this is a RA response
	uint8_t ra_flag;
	/// rnti
	uint16_t rnti;
	/// Format
	DCI_format_t format;
	/// DCI pdu
	uint8_t dci_pdu[8];
} DCI_ALLOC_t;

#endif
