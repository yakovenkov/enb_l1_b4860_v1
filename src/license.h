/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __LICENSE_H_
#define __LICENSE_H_

#include <stdint.h>

/* Структура лицензии
 * Размер 64 байта
 */
typedef struct __attribute__((packed))
{
	uint32_t salt;
	/* Флаги лицензии */
	uint32_t features;
	/* Количество приемных антенн */
	uint8_t n_ant_tx;
	/* Количество передающих антенн */
	uint8_t n_ant_rx;
	/* Количество пользователей DL */
	uint8_t n_users_dl;
	/* Количество пользователей UL */
	uint8_t n_users_ul;
	/* Время работы для timelimit */
	uint16_t n_timelimit_secs;
	
	uint8_t reserved[48];
	
	uint16_t crc16;
} l1_license_t;

#define FEATURE_FULL		0x80000000
#define FEATURE_TIMELIMIT	0x40000000

#endif
