/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <stdint.h>
#include <string.h>
#include <secure_id.h>

l1_license_t g_l1_license;
secure_id_t g_secure_id;

uint32_t crc16(uint8_t * inptr, int32_t bitlen);

#define ID_XOR_VAL	0x12345678
#define MAKE_ID(x) (x ^ ID_XOR_VAL)

static uint32_t get_chip_id()
{
	uint32_t *chip_id_ptr = (uint32_t *)0x7f0e8270;
	return *chip_id_ptr;
}

#ifdef DEBUG_BUILD
uint32_t get_chip_id_ext()
{
	uint32_t *chip_id_ptr = (uint32_t *)0x7f0e8270;
	return *chip_id_ptr;
}
#else
uint32_t get_chip_id_ext()
{
	return 0;
}
#endif

static void fill_license_full(l1_license_t *lic)
{
	lic->features = FEATURE_FULL;
	lic->n_ant_rx = 255;
	lic->n_ant_tx = 255;
	lic->n_users_dl = 255;
	lic->n_users_ul = 255;
	lic->n_timelimit_secs = 0;
}

static void fill_license_eval(l1_license_t *lic)
{
	lic->features = 0;
	lic->n_ant_rx = 2;
	lic->n_ant_tx = 2;
	lic->n_users_dl = 2;
	lic->n_users_ul = 2;
	lic->n_timelimit_secs = 0;
}

static void fill_license_timelimit(l1_license_t *lic)
{
	lic->features = FEATURE_TIMELIMIT;
	lic->n_ant_rx = 2;
	lic->n_ant_tx = 2;
	lic->n_users_dl = 2;
	lic->n_users_ul = 2;
	lic->n_timelimit_secs = 600;
}

static void fill_license_none(l1_license_t *lic)
{
	lic->features = FEATURE_TIMELIMIT;
	lic->n_ant_rx = 1;
	lic->n_ant_tx = 1;
	lic->n_users_dl = 2;
	lic->n_users_ul = 0;
	lic->n_timelimit_secs = 600;
}

secure_id_t get_secure_id()
{
	return SECURE_ID_FULL;
}

int32_t process_license(uint8_t *buf)
{
	l1_license_t *lic = (l1_license_t *)buf;
	uint32_t crc;
	
	/* Расшифровка лицензии in-place */
	uint32_t id = get_chip_id();
	uint32_t *lic_words = (uint32_t *)lic;
	uint32_t prev_word = 0x12345678;
	int32_t i;
	
	g_secure_id = get_secure_id();
	
	for(i=0; i<sizeof(l1_license_t)/sizeof(uint32_t); i++)
	{
		lic_words[i] = lic_words[i] ^ id ^ prev_word;
		prev_word = lic_words[i];
	}
	
	crc = (crc16((uint8_t *)lic, (64 - 2) * 8) >> 16);
	
	if(crc != lic->crc16)
	{
		memset(lic, 0, sizeof(l1_license_t));
		
		switch(g_secure_id)
		{
			case SECURE_ID_FULL:
				fill_license_full(lic);
				break;
				
			case SECURE_ID_EVAL:
				fill_license_eval(lic);
				break;
				
			case SECURE_ID_TIME:
				fill_license_timelimit(lic);
				break;
			
			default:
				fill_license_none(lic);
				break;
		}
		
		return 1;
	}
	
	/* TODO: обработка лицензии */
	
	return 0;
}
