/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __SECURE_ID_H_
#define __SECURE_ID_H_

#include <license.h>

/* Счетчик циклов на 5 минут (5 * 60 * 1000) */
#define SECURE_EVAL_COUNT 300000

typedef enum secure_id_e
{
	SECURE_ID_FULL = 15,
	SECURE_ID_TIME,
	SECURE_ID_EVAL,
	SECURE_ID_NONE
} secure_id_t;

extern l1_license_t g_l1_license;
extern secure_id_t g_secure_id;

secure_id_t get_secure_id();
int32_t process_license(uint8_t *buf);

#endif
