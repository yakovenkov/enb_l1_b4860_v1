/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <prototype.h>

Complex16 liblte_sss_mod[3][168][63][2] __attribute__((section(".local_data_ddr0_cacheable"), aligned(8))) =
{
		{
#include "sss_0.c_inc"
		},
		{
#include "sss_1.c_inc"
		},
		{
#include "sss_2.c_inc"
		}
};
