/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

#include "liblte_msc8157.h"

#ifndef DEBUG_TX_BUFFER_TEST
#pragma opt_level = "O3"
#endif

uint32_t lte_gold_generic(uint32_t *x1, uint32_t *x2, uint32_t reset)
{
	int32_t n;
	if (reset)
	{
		*x1 = 1 + (1 << 31);
		*x2 = *x2 ^ ((*x2 ^ (*x2 >> 1) ^ (*x2 >> 2) ^ (*x2 >> 3)) << 31);
		for (n = 1; n < 50; n++)
		{
			*x1 = (*x1 >> 1) ^ (*x1 >> 4);
			*x1 = *x1 ^ (*x1 << 31) ^ (*x1 << 28);
			*x2 = (*x2 >> 1) ^ (*x2 >> 2) ^ (*x2 >> 3) ^ (*x2 >> 4);
			*x2 = *x2 ^ (*x2 << 31) ^ (*x2 << 30) ^ (*x2 << 29) ^ (*x2 << 28);
		}
	}
	*x1 = (*x1 >> 1) ^ (*x1 >> 4);
	*x1 = *x1 ^ (*x1 << 31) ^ (*x1 << 28);
	*x2 = (*x2 >> 1) ^ (*x2 >> 2) ^ (*x2 >> 3) ^ (*x2 >> 4);
	*x2 = *x2 ^ (*x2 << 31) ^ (*x2 << 30) ^ (*x2 << 29) ^ (*x2 << 28);
	return (*x1 ^ *x2);
}
