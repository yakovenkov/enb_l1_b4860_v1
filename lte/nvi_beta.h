/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __NVI_BETA_H_
#define __NVI_BETA_H_

#include <stdint.h>

typedef struct __attribute__((packed))
{
	uint16_t beta0_m;
	uint16_t beta1_m;
	int8_t beta0_e;
	int8_t beta1_e;
	uint16_t nvi0_m;
	uint16_t nvi1_m;
	int8_t nvi0_e;
	int8_t nvi1_e;
	uint32_t reserved;
} nvi_beta_t;

/* Таблица NVI/BETA для 100 RB и 3-х видов модуляции:
 * 0 - QPSK
 * 1 - QAM16
 * 2 - QAM64
 */
extern nvi_beta_t nvi_beta_table[100][3][4];

void generate_nvi_beta_tables();

#endif
