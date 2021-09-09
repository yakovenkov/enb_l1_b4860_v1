/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#pragma opt_level = "O3"

#include "dsp_kernels.h"

void sc3900_vector_dot_product_16x16_c(Complex16 * restrict __attribute__((aligned(8))) vec1,
		Complex16 * restrict __attribute__((aligned(8))) vec2, Word32 length, Word32 res[2])
{
	int i;

	Word40 sum_r, sum_i;

	sum_r = X_extend(0);
	sum_i = X_extend(0);

	cw_assert(length >= 1);
	for (i = 0; i < length; i += 2)
		__maccxd_cpp_2x(vec2[i], vec2[i + 1], vec1[i], vec1[i + 1], &sum_r, &sum_i);

	__sat_x_2l(sum_r, sum_i, &res[0], &res[1]);
}
