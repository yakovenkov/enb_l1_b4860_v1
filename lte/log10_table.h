/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef _LOG10_TABLE_H
#define _LOG10_TABLE_H

#include <stdint.h>

#define N_LOG10_TABLE_ITEMS 10001

extern int16_t log10_table[N_LOG10_TABLE_ITEMS];

/* 
 * Перевод дБ в линейное значение
 * Формат 4q12
 */
#define __fapi_dB_2_lin(x) ((x < 0 || x > N_LOG10_TABLE_ITEMS) ? 4096 : log10_table[x])

/* 
 * Скейл X на значение FAPI dB с приведением к 1q15  
 */
#define scale_by_fapi_dB(x, y) ((((int32_t)x) * ((int32_t)__fapi_dB_2_lin(y))) >> 12)

#endif
