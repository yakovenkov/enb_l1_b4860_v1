/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <smartdsp_os.h>
//#include <complex.h>
#include <lte_enodeb.h>
#include <log.h>
#include "liblte_msc8157.h"

#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_OPT_OFF
#pragma opt_level = "O3"
#endif

#define M_1_2PI_F  ((float)(M_1_PI / 2.0f))
extern Complex16 ul_ref_sigs_rx[30][2][33][1200];

// UL reference signal prime numbers less than 2048 from 3GPP TS 36.211 v10.1.0 section 5.5.1.1
#define UL_RS_PRIMES_LESS_THAN_2048_LEN 309
uint32_t UL_RS_PRIMES_LESS_THAN_2048[UL_RS_PRIMES_LESS_THAN_2048_LEN] =
	{ 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107,
		109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229,
		233, 239, 241, 251, 257, 263, 269, 271, 277, 281, 283, 293, 307, 311, 313, 317, 331, 337, 347, 349, 353, 359,
		367, 373, 379, 383, 389, 397, 401, 409, 419, 421, 431, 433, 439, 443, 449, 457, 461, 463, 467, 479, 487, 491,
		499, 503, 509, 521, 523, 541, 547, 557, 563, 569, 571, 577, 587, 593, 599, 601, 607, 613, 617, 619, 631, 641,
		643, 647, 653, 659, 661, 673, 677, 683, 691, 701, 709, 719, 727, 733, 739, 743, 751, 757, 761, 769, 773, 787,
		797, 809, 811, 821, 823, 827, 829, 839, 853, 857, 859, 863, 877, 881, 883, 887, 907, 911, 919, 929, 937, 941,
		947, 953, 967, 971, 977, 983, 991, 997, 1009, 1013, 1019, 1021, 1031, 1033, 1039, 1049, 1051, 1061, 1063, 1069,
		1087, 1091, 1093, 1097, 1103, 1109, 1117, 1123, 1129, 1151, 1153, 1163, 1171, 1181, 1187, 1193, 1201, 1213,
		1217, 1223, 1229, 1231, 1237, 1249, 1259, 1277, 1279, 1283, 1289, 1291, 1297, 1301, 1303, 1307, 1319, 1321,
		1327, 1361, 1367, 1373, 1381, 1399, 1409, 1423, 1427, 1429, 1433, 1439, 1447, 1451, 1453, 1459, 1471, 1481,
		1483, 1487, 1489, 1493, 1499, 1511, 1523, 1531, 1543, 1549, 1553, 1559, 1567, 1571, 1579, 1583, 1597, 1601,
		1607, 1609, 1613, 1619, 1621, 1627, 1637, 1657, 1663, 1667, 1669, 1693, 1697, 1699, 1709, 1721, 1723, 1733,
		1741, 1747, 1753, 1759, 1777, 1783, 1787, 1789, 1801, 1811, 1823, 1831, 1847, 1861, 1867, 1871, 1873, 1877,
		1879, 1889, 1901, 1907, 1913, 1931, 1933, 1949, 1951, 1973, 1979, 1987, 1993, 1997, 1999, 2003, 2011, 2017,
		2027, 2029, 2039 };

// N_1_DMRS table from 3GPP TS 36.211 v10.1.0 table 5.5.2.1.1-2
uint32_t N_1_DMRS_5_5_2_1_1_2[8] =
	{ 0, 2, 3, 4, 6, 8, 9, 10 };

// N_2_DMRS_LAMBDA table from 3GPP TS 36.211 v10.1.0 table 5.5.2.1.1-1
uint32_t N_2_DMRS_LAMBDA_5_5_2_1_1_1[8][4] =
	{
		{ 0, 6, 3, 9 },
		{ 6, 0, 9, 3 },
		{ 3, 9, 6, 0 },
		{ 4, 10, 7, 1 },
		{ 2, 8, 5, 11 },
		{ 8, 2, 11, 5 },
		{ 10, 4, 1, 7 },
		{ 9, 3, 0, 6 } };

// W table from 3GPP TS 36.211 v10.1.0 table 5.5.2.1.1-1
int32_t W_1_5_5_2_1_1_1[8][4] =
	{
		{ 1, 1, -1, -1 },
		{ -1, -1, 1, 1 },
		{ -1, -1, 1, 1 },
		{ 1, 1, 1, 1 },
		{ 1, 1, 1, 1 },
		{ -1, -1, -1, -1 },
		{ -1, -1, -1, -1 },
		{ 1, 1, -1, -1 } };

// UL reference signal phi value for M_sc_rs = N_sc_rb_ul from 3GPP TS 36.211 v10.1.0 table 5.5.1.2-1
int32_t UL_RS_5_5_1_2_1[30][12] =
	{
		{ -1, 1, 3, -3, 3, 3, 1, 1, 3, 1, -3, 3 },
		{ 1, 1, 3, 3, 3, -1, 1, -3, -3, 1, -3, 3 },
		{ 1, 1, -3, -3, -3, -1, -3, -3, 1, -3, 1, -1 },
		{ -1, 1, 1, 1, 1, -1, -3, -3, 1, -3, 3, -1 },
		{ -1, 3, 1, -1, 1, -1, -3, -1, 1, -1, 1, 3 },
		{ 1, -3, 3, -1, -1, 1, 1, -1, -1, 3, -3, 1 },
		{ -1, 3, -3, -3, -3, 3, 1, -1, 3, 3, -3, 1 },
		{ -3, -1, -1, -1, 1, -3, 3, -1, 1, -3, 3, 1 },
		{ 1, -3, 3, 1, -1, -1, -1, 1, 1, 3, -1, 1 },
		{ 1, -3, -1, 3, 3, -1, -3, 1, 1, 1, 1, 1 },
		{ -1, 3, -1, 1, 1, -3, -3, -1, -3, -3, 3, -1 },
		{ 3, 1, -1, -1, 3, 3, -3, 1, 3, 1, 3, 3 },
		{ 1, -3, 1, 1, -3, 1, 1, 1, -3, -3, -3, 1 },
		{ 3, 3, -3, 3, -3, 1, 1, 3, -1, -3, 3, 3 },
		{ -3, 1, -1, -3, -1, 3, 1, 3, 3, 3, -1, 1 },
		{ 3, -1, 1, -3, -1, -1, 1, 1, 3, 1, -1, -3 },
		{ 1, 3, 1, -1, 1, 3, 3, 3, -1, -1, 3, -1 },
		{ -3, 1, 1, 3, -3, 3, -3, -3, 3, 1, 3, -1 },
		{ -3, 3, 1, 1, -3, 1, -3, -3, -1, -1, 1, -3 },
		{ -1, 3, 1, 3, 1, -1, -1, 3, -3, -1, -3, -1 },
		{ -1, -3, 1, 1, 1, 1, 3, 1, -1, 1, -3, -1 },
		{ -1, 3, -1, 1, -3, -3, -3, -3, -3, 1, -1, -3 },
		{ 1, 1, -3, -3, -3, -3, -1, 3, -3, 1, -3, 3 },
		{ 1, 1, -1, -3, -1, -3, 1, -1, 1, 3, -1, 1 },
		{ 1, 1, 3, 1, 3, 3, -1, 1, -1, -3, -3, 1 },
		{ 1, -3, 3, 3, 1, 3, 3, 1, -3, -1, -1, 3 },
		{ 1, 3, -3, -3, 3, -3, 1, -1, -1, 3, -1, -3 },
		{ -3, -1, -3, -1, -3, 3, 1, -1, 1, 3, -3, -3 },
		{ -1, 3, -3, 3, -1, 3, 3, -3, 3, 3, -1, -1 },
		{ 3, -3, -3, -1, -1, -3, -1, 3, -3, 3, 1, -1 } };

// UL reference signal phi value for M_sc_rs = 2*N_sc_rb_ul from 3GPP TS 36.211 v10.1.0 table 5.5.1.2-2
int32_t UL_RS_5_5_1_2_2[30][24] =
	{
		{ -1, 3, 1, -3, 3, -1, 1, 3, -3, 3, 1, 3, -3, 3, 1, 1, -1, 1, 3, -3, 3, -3, -1, -3 },
		{ -3, 3, -3, -3, -3, 1, -3, -3, 3, -1, 1, 1, 1, 3, 1, -1, 3, -3, -3, 1, 3, 1, 1, -3 },
		{ 3, -1, 3, 3, 1, 1, -3, 3, 3, 3, 3, 1, -1, 3, -1, 1, 1, -1, -3, -1, -1, 1, 3, 3 },
		{ -1, -3, 1, 1, 3, -3, 1, 1, -3, -1, -1, 1, 3, 1, 3, 1, -1, 3, 1, 1, -3, -1, -3, -1 },
		{ -1, -1, -1, -3, -3, -1, 1, 1, 3, 3, -1, 3, -1, 1, -1, -3, 1, -1, -3, -3, 1, -3, -1, -1 },
		{ -3, 1, 1, 3, -1, 1, 3, 1, -3, 1, -3, 1, 1, -1, -1, 3, -1, -3, 3, -3, -3, -3, 1, 1 },
		{ 1, 1, -1, -1, 3, -3, -3, 3, -3, 1, -1, -1, 1, -1, 1, 1, -1, -3, -1, 1, -1, 3, -1, -3 },
		{ -3, 3, 3, -1, -1, -3, -1, 3, 1, 3, 1, 3, 1, 1, -1, 3, 1, -1, 1, 3, -3, -1, -1, 1 },
		{ -3, 1, 3, -3, 1, -1, -3, 3, -3, 3, -1, -1, -1, -1, 1, -3, -3, -3, 1, -3, -3, -3, 1, -3 },
		{ 1, 1, -3, 3, 3, -1, -3, -1, 3, -3, 3, 3, 3, -1, 1, 1, -3, 1, -1, 1, 1, -3, 1, 1 },
		{ -1, 1, -3, -3, 3, -1, 3, -1, -1, -3, -3, -3, -1, -3, -3, 1, -1, 1, 3, 3, -1, 1, -1, 3 },
		{ 1, 3, 3, -3, -3, 1, 3, 1, -1, -3, -3, -3, 3, 3, -3, 3, 3, -1, -3, 3, -1, 1, -3, 1 },
		{ 1, 3, 3, 1, 1, 1, -1, -1, 1, -3, 3, -1, 1, 1, -3, 3, 3, -1, -3, 3, -3, -1, -3, -1 },
		{ 3, -1, -1, -1, -1, -3, -1, 3, 3, 1, -1, 1, 3, 3, 3, -1, 1, 1, -3, 1, 3, -1, -3, 3 },
		{ -3, -3, 3, 1, 3, 1, -3, 3, 1, 3, 1, 1, 3, 3, -1, -1, -3, 1, -3, -1, 3, 1, 1, 3 },
		{ -1, -1, 1, -3, 1, 3, -3, 1, -1, -3, -1, 3, 1, 3, 1, -1, -3, -3, -1, -1, -3, -3, -3, -1 },
		{ -1, -3, 3, -1, -1, -1, -1, 1, 1, -3, 3, 1, 3, 3, 1, -1, 1, -3, 1, -3, 1, 1, -3, -1 },
		{ 1, 3, -1, 3, 3, -1, -3, 1, -1, -3, 3, 3, 3, -1, 1, 1, 3, -1, -3, -1, 3, -1, -1, -1 },
		{ 1, 1, 1, 1, 1, -1, 3, -1, -3, 1, 1, 3, -3, 1, -3, -1, 1, 1, -3, -3, 3, 1, 1, -3 },
		{ 1, 3, 3, 1, -1, -3, 3, -1, 3, 3, 3, -3, 1, -1, 1, -1, -3, -1, 1, 3, -1, 3, -3, -3 },
		{ -1, -3, 3, -3, -3, -3, -1, -1, -3, -1, -3, 3, 1, 3, -3, -1, 3, -1, 1, -1, 3, -3, 1, -1 },
		{ -3, -3, 1, 1, -1, 1, -1, 1, -1, 3, 1, -3, -1, 1, -1, 1, -1, -1, 3, 3, -3, -1, 1, -3 },
		{ -3, -1, -3, 3, 1, -1, -3, -1, -3, -3, 3, -3, 3, -3, -1, 1, 3, 1, -3, 1, 3, 3, -1, -3 },
		{ -1, -1, -1, -1, 3, 3, 3, 1, 3, 3, -3, 1, 3, -1, 3, -1, 3, 3, -3, 3, 1, -1, 3, 3 },
		{ 1, -1, 3, 3, -1, -3, 3, -3, -1, -1, 3, -1, 3, -1, -1, 1, 1, 1, 1, -1, -1, -3, -1, 3 },
		{ 1, -1, 1, -1, 3, -1, 3, 1, 1, -1, -1, -3, 1, 1, -3, 1, 3, -3, 1, 1, -3, -3, -1, -1 },
		{ -3, -1, 1, 3, 1, 1, -3, -1, -1, -3, 3, -3, 3, 1, -3, 3, -3, 1, -1, 1, -3, 1, 1, 1 },
		{ -1, -3, 3, 3, 1, 1, 3, -1, -3, -1, -1, -1, 3, 1, -3, -3, -1, 3, -3, -1, -3, -1, -3, -1 },
		{ -1, -3, -1, -1, 1, -3, -1, -1, 1, -1, -3, 1, 1, -3, 1, -3, -3, 3, 1, 1, -1, 3, -1, -1 },
		{ 1, 1, -1, -1, -3, -1, 3, -1, 3, -1, 1, 3, 1, -1, 3, 1, 3, -3, -3, 1, -1, -1, 1, 3 } };

/* Table 5.5.2.2.2-1: Demodulation reference signal location for different PUCCH formats. 36.211 */
uint32_t pucch_dmrs_symbol_format1_cpnorm[3] =
	{ 2, 3, 4 };
uint32_t pucch_dmrs_symbol_format1_cpext[2] =
	{ 2, 3 };
uint32_t pucch_dmrs_symbol_format2_cpnorm[2] =
	{ 1, 5 };
uint32_t pucch_dmrs_symbol_format2_cpext[1] =
	{ 3 };

uint32_t pucch_symbol_format1_cpnorm[4] =
	{ 0, 1, 5, 6 };
uint32_t pucch_symbol_format1_cpext[4] =
	{ 0, 1, 4, 5 };
uint32_t pucch_symbol_format2_cpnorm[5] =
	{ 0, 2, 3, 4, 6 };
uint32_t pucch_symbol_format2_cpext[5] =
	{ 0, 1, 2, 4, 5 };

#define M_PI_FLOAT		3.14159265358979323846
float w_n_oc[2][3][4] =
	{
	// Table 5.4.1-2 Orthogonal sequences w for N_sf=4 (complex argument)
			{
				{ 0, 0, 0, 0 },
				{ 0, M_PI_FLOAT, 0, M_PI_FLOAT },
				{ 0, M_PI_FLOAT, M_PI_FLOAT, 0 } },
		// Table 5.4.1-3 Orthogonal sequences w for N_sf=3
			{
				{ 0, 0, 0, 0 },
				{ 0, 2 * M_PI_FLOAT / 3, 4 * M_PI_FLOAT / 3, 0 },
				{ 0, 4 * M_PI_FLOAT / 3, 2 * M_PI_FLOAT / 3, 0 } },

	};

/* Orthogonal sequences for PUCCH formats 1a, 1b and 1c. Table 5.5.2.2.1-2 
 */
float w_arg_pucch_format1_cpnorm[3][3] =
	{
		{ 0, 0, 0 },
		{ 0, 2.0f * M_PI_FLOAT / 3.0f, 4.0f * M_PI_FLOAT / 3.0f },
		{ 0, 4.0f * M_PI_FLOAT / 3.0f, 2.0f * M_PI_FLOAT / 3.0f } };

float w_arg_pucch_format1_cpext[3][2] =
	{
		{ 0, 0 },
		{ 0, M_PI_FLOAT },
		{ 0, 0 } };

float w_arg_pucch_format2_cpnorm[2] =
	{ 0, 0 };
float w_arg_pucch_format2_cpext[1] =
	{ 0 };

/* Таблица соответствия номера символа символу в буфере lte_ul_subframe_t->pufft_a0 
 * Символы 3 и 10 записаны в свои буферы
 */
uint32_t l2pufft_table_normcp[2][7] = {
		{0, 1, 2, 0, 3, 4, 5},
		{6, 7, 8, 0, 9, 10, 11}
};

/* Таблицы SIN/COS */
#define SINCOS_TABLESIZE (360*10)
static int16_t cos_tab[SINCOS_TABLESIZE] __attribute__((section(".local_data_ddr0_cacheable_bss"), aligned(ARCH_CACHE_LINE_SIZE)));
static int16_t sin_tab[SINCOS_TABLESIZE] __attribute__((section(".local_data_ddr0_cacheable_bss"), aligned(ARCH_CACHE_LINE_SIZE)));
static int32_t sin_cos_initialized = 0;

/** compute sin via lookup table. */
static Word16 liblte_sin_lookup(float x)
{
	float argT;
	int16_t argI;
	argT = x * ((float) SINCOS_TABLESIZE) * M_1_2PI_F;
	argI = ((int16_t) round(argT)) % SINCOS_TABLESIZE;

	return sin_tab[argI];
}

/** compute cos via lookup table. */
static Word16 liblte_cos_lookup(float x)
{
	float argT;
	int16_t argI;
	argT = x * ((float) SINCOS_TABLESIZE) * M_1_2PI_F;
	argI = ((int16_t) round(argT)) % SINCOS_TABLESIZE;

	return cos_tab[argI];
}

uint16_t computeRIV(uint16_t N_RB_DL, uint16_t RBstart, uint16_t Lcrbs)
{

	uint16_t RIV;

	if (Lcrbs <= (1 + (N_RB_DL >> 1)))
		RIV = (N_RB_DL * (Lcrbs - 1)) + RBstart;
	else
		RIV = (N_RB_DL * (N_RB_DL + 1 - Lcrbs)) + (N_RB_DL - 1 - RBstart);

	return (RIV);
}

uint32_t get_nsoftbits_by_ue_cat(uint32_t ue_cat)
{
	switch (ue_cat)
	{
		case 1:
			return 250368;
		case 2:
			return 1237248;
		case 3:
			return 1237248;
		case 4:
			return 1827072;

		default:
			// Return UE cat 1 by default
			return 250368;
			//ERROR(DTRX, "Unsupported UE cat %i\n", ue_cat);
	}

	return 0;
}

static void pbch_scrambling(lte_enodeb_t *enodeb, uint8_t *pbch_e, uint32_t length)
{
	int32_t i;
	uint8_t reset;
	uint32_t x1, x2, s = 0;

	reset = 1;

	x2 = enodeb->N_id_cell;

	for (i = 0; i < length; i++)
	{
		if ((i & 0x1f) == 0)
		{
			s = lte_gold_generic(&x1, &x2, reset);
			reset = 0;
		}

		pbch_e[i] = (pbch_e[i] & 1) ^ ((s >> (i & 0x1f)) & 1);

	}
}

/**
 * Формирование канала PBCH
 * @param trx - дескриптор TRX
 * @param lte_slot - формируемый слот LTE
 * @param amp - множитель амплитуды
 * @return код статуса
 */
lte_status_t liblte_generate_pbch(lte_trx_t *trx, lte_subframe_t *lte_sf)
{

	int32_t i;

	uint32_t pbch_D, pbch_E, pbch_per_sf; //,pbch_coded_bytes;
	uint8_t pbch_a[LTE_PBCH_A >> 3];
	uint8_t RCC;

	uint32_t nsymb = (LTE_CP == LTE_CP_NORMAL) ? 14 : 12;
	uint32_t second_pilot = (LTE_CP == LTE_CP_NORMAL) ? 4 : 3;
	uint32_t jj = 0;
	uint32_t re_allocated = 0;
	uint32_t re_offset, symbol_offset;
	uint16_t amask = 0;
	uint32_t frame_mod4;
	int32_t idx_data;
	lte_enodeb_t *enodeb;

	OS_ASSERT_COND(trx != NULL);
	OS_ASSERT_COND(lte_sf != NULL);

	enodeb = trx->enodeb;

	pbch_D = 16 + LTE_PBCH_A;

	pbch_E = (LTE_CP == LTE_CP_NORMAL) ? 1920 : 1728; //RE/RB * #RB * bits/RB (QPSK)
	pbch_per_sf = (LTE_CP == LTE_CP_NORMAL) ? (1920 / 4) : (1728 / 4);

	frame_mod4 = (lte_sf->frame_no & 3);

	if (frame_mod4 == 0)
	{
		/* Формирование канала PBCH производится с периодом 40мс (каждый 4-й фрейм) */
		memset(&pbch_a, 0, LTE_PBCH_A >> 3);
		memset(&enodeb->refsigs->pbch.pbch_e, 0, pbch_E);
		memset(&enodeb->refsigs->pbch.pbch_d, LTE_NULL, 96);

		// Encode data

		// Fix byte endian of PBCH (bit 23 goes in first)
		// Not needed now
		/* for (i = 0; i < (LTE_PBCH_A >> 3); i++)
			pbch_a[(LTE_PBCH_A >> 3) - i - 1] = enodeb->pbch_pdu[i];
		*/
		pbch_a[0] = enodeb->pbch_pdu[0];
		pbch_a[1] = enodeb->pbch_pdu[1];
		pbch_a[2] = enodeb->pbch_pdu[2];
		
		if (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 1)
		{
			amask = 0x0000;
		}
		else if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2)
		{
			amask = 0xffff;
		}
		else if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX >= 4)
		{
			amask = 0x5555;
		}

		ccodelte_encode(LTE_PBCH_A, 2, pbch_a, enodeb->refsigs->pbch.pbch_d + 96, amask);

		RCC = sub_block_interleaving_cc(pbch_D, enodeb->refsigs->pbch.pbch_d + 96, enodeb->refsigs->pbch.pbch_w);

		lte_rate_matching_cc(RCC, pbch_E, enodeb->refsigs->pbch.pbch_w, enodeb->refsigs->pbch.pbch_e);

		// Приведение к битовому представлению
		for(i=0; i<pbch_E / 8; i++)
		{
			uint64_t *da = (uint64_t *)&enodeb->refsigs->pbch.pbch_e[i * 8];
			
			// Упаковка по младших разрядов 8 байт в 1 байт 
			enodeb->refsigs->pbch.pbch_e_bits[i] = __bit_colpsl_b_ll(0, *da);
		}	
	}

	// Скремблирование и дальнейшая обработка производится в MAPLE-B3
	if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 1)
	{
		// SISO
		lte_sf->user_map[0].first_flags = PDSCH_USER_ANT_EN(0x01);
		lte_sf->user_map[0].second_flags = PDSCH_USER_1_CW | PDSCH_USER_TRANSMIT_SINGLE_ANT;
		lte_sf->user_map[0].third_flags =  PDSCH_USER_TB0_MODULATION_QPSK | PDSCH_USER_TB0_NL_1;
	}
	else if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2)
	{
		// 2 антенны
		lte_sf->user_map[0].first_flags = PDSCH_USER_ANT_EN(0x03);
		lte_sf->user_map[0].second_flags = PDSCH_USER_1_CW | PDSCH_USER_TRANSMIT_DIVERSITY | PDSCH_USER_2_ANT_PORTS_TRANS_DIVERSITY;
		lte_sf->user_map[0].third_flags =  PDSCH_USER_TB0_MODULATION_QPSK | PDSCH_USER_TB0_NL_2;
	}
	else if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 4)
	{
		// 2 антенны
		lte_sf->user_map[0].first_flags = PDSCH_USER_ANT_EN(0x0f);
		lte_sf->user_map[0].second_flags = PDSCH_USER_1_CW | PDSCH_USER_TRANSMIT_DIVERSITY | PDSCH_USER_4_ANT_PORTS_TRANS_DIVERSITY;
		lte_sf->user_map[0].third_flags =  PDSCH_USER_TB0_MODULATION_QPSK | PDSCH_USER_TB0_NL_2;
	}
	
	/* Остальные параметры одинаковые для различных конифигураций антенн */
	lte_sf->user_map[0].tb_size[0] = (LTE_CP == LTE_CP_NORMAL ? 60 : 54);
	lte_sf->user_map[0].tb_size[1] = 0;
	lte_sf->user_map[0].tb_params[0].tb_input = (os_virt_ptr)enodeb->refsigs->pbch.pbch_e_bits;
	lte_sf->user_map[0].tb_params[1].tb_input = (os_virt_ptr)enodeb->refsigs->pbch.pbch_e_bits;
	lte_sf->user_map[0].tb_params[0].num_out_bits = (LTE_CP == LTE_CP_NORMAL ? 480 : 432);
	lte_sf->user_map[0].tb_params[1].num_out_bits = 0;
	
	lte_sf->cw_map[0][0].cw_init        =  enodeb->N_id_cell;
	lte_sf->cw_map[0][0].cw_mod_type    =  0; // QPSK
	lte_sf->cw_map[0][0].cw_trgt_layer  =  0x01;
	lte_sf->cw_map[0][0].gain_a_exp     =  enodeb->fp.LTE_PBCH_GAIN_E;
	lte_sf->cw_map[0][0].gain_a_mantisa =  enodeb->fp.LTE_PBCH_GAIN_M;
	lte_sf->cw_map[0][0].gain_b_exp     =  enodeb->fp.LTE_PBCH_GAIN_E;
	lte_sf->cw_map[0][0].gain_b_mantisa =  enodeb->fp.LTE_PBCH_GAIN_M;
	lte_sf->cw_map[0][1].cw_init        =  0x00000000;
	lte_sf->cw_map[0][1].cw_mod_type    =  0;
	lte_sf->cw_map[0][1].cw_trgt_layer  =  0x00;
	lte_sf->cw_map[0][1].gain_a_exp     =  0;
	lte_sf->cw_map[0][1].gain_a_mantisa =  0;
	lte_sf->cw_map[0][1].gain_b_exp     =  0;
	lte_sf->cw_map[0][1].gain_b_mantisa =  0;		
	
	// Установка флага PBCH
	// Структура PBCH находится в user_map[0] и cw_map[0]
	lte_sf->job.pdsch_job.third_flags |= PDSCH_BD_PBCH_EN;
	
	// Номер блока PBCH
	switch(lte_sf->frame_no & 3)
	{
		case 0:
			lte_sf->job.pdsch_job.second_flags |= PDSCH_PBCH_FIRST_Q;
			break;
		case 1:
			lte_sf->job.pdsch_job.second_flags |= PDSCH_PBCH_SECOND_Q;
			break;
		case 2:
			lte_sf->job.pdsch_job.second_flags |= PDSCH_PBCH_THIRD_Q;
			break;
		case 3:
			lte_sf->job.pdsch_job.second_flags |= PDSCH_PBCH_FOURTH_Q;
			break;
	}

	return LTE_SUCCESS;
}

/**
 * Генератор таблицы распределения PCFICH
 * @param frame_parms
 */
lte_status_t liblte_generate_pcfich_reg_mapping(lte_enodeb_t *enodeb)
{

	uint16_t kbar = 6 * (enodeb->N_id_cell % (2 * enodeb->fp.LTE_N_RB_DL));
	uint16_t first_reg;
	uint32_t *pcfich_reg = enodeb->pcfich_reg;

	pcfich_reg[0] = kbar / 6;
	first_reg = pcfich_reg[0];

	enodeb->pcfich_first_reg_idx = 0;

	pcfich_reg[1] = ((kbar + (enodeb->fp.LTE_N_RB_DL >> 1) * 6) % (enodeb->fp.LTE_N_RB_DL * 12)) / 6;
	if (pcfich_reg[1] < pcfich_reg[0])
	{
		enodeb->pcfich_first_reg_idx = 1;
		first_reg = pcfich_reg[1];
	}
	pcfich_reg[2] = ((kbar + (enodeb->fp.LTE_N_RB_DL) * 6) % (enodeb->fp.LTE_N_RB_DL * 12)) / 6;
	if (pcfich_reg[2] < first_reg)
	{
		enodeb->pcfich_first_reg_idx = 2;
		first_reg = pcfich_reg[2];
	}
	pcfich_reg[3] = ((kbar + ((3 * enodeb->fp.LTE_N_RB_DL) >> 1) * 6) % (enodeb->fp.LTE_N_RB_DL * 12)) / 6;
	if (pcfich_reg[3] < first_reg)
	{
		enodeb->pcfich_first_reg_idx = 3;
		first_reg = pcfich_reg[3];
	}

	return LTE_SUCCESS;
}

/**
 * генератор таблицы распределения PHICH
 * @param frame_parms
 */
lte_status_t liblte_generate_phich_reg_mapping(lte_enodeb_t *enodeb)
{
	unsigned short n0 = (enodeb->fp.LTE_N_RB_DL * 2) - 4; // 2 REG per RB less the 4 used by PCFICH in first symbol
	unsigned short n1 = (enodeb->fp.LTE_N_RB_DL * 3); // 3 REG per RB in second and third symbol
	unsigned short n2 = n1;
	unsigned short mprime = 0;
	unsigned short Ngroup_PHICH;
	uint32_t *pcfich_reg = enodeb->pcfich_reg;

	// compute Ngroup_PHICH (see formula at beginning of Section 6.9 in 36-211
	Ngroup_PHICH = (enodeb->fapi_config.phich_config.phich_resource * enodeb->fp.LTE_N_RB_DL) / 48;
	if (((enodeb->fapi_config.phich_config.phich_resource * enodeb->fp.LTE_N_RB_DL) % 48) > 0)
		Ngroup_PHICH++;

	// check if Extended prefix
#if LTE_CP == LTE_CP_EXTENDED
	Ngroup_PHICH <<= 1;
#endif

	if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_TDD &&
			enodeb->fapi_config.tdd_frame_structure_config.subframe_assignment == 0)
	{
		/* Handle TDD configuration 0 for subframes 0 and 5 */
		Ngroup_PHICH *= 2;
	}

	// This is the algorithm from Section 6.9.3 in 36-211
	for (mprime = 0; mprime < ((LTE_CP == LTE_CP_NORMAL) ? Ngroup_PHICH : (Ngroup_PHICH >> 1)); mprime++)
	{

#if LTE_CP == LTE_CP_NORMAL
		// normal prefix

		enodeb->phich_reg[mprime][0] = (enodeb->N_id_cell + mprime) % n0;

		if (enodeb->phich_reg[mprime][0] >= pcfich_reg[enodeb->pcfich_first_reg_idx])
			enodeb->phich_reg[mprime][0]++;
		if (enodeb->phich_reg[mprime][0] >= pcfich_reg[(enodeb->pcfich_first_reg_idx + 1) & 3])
			enodeb->phich_reg[mprime][0]++;
		if (enodeb->phich_reg[mprime][0] >= pcfich_reg[(enodeb->pcfich_first_reg_idx + 2) & 3])
			enodeb->phich_reg[mprime][0]++;
		if (enodeb->phich_reg[mprime][0] >= pcfich_reg[(enodeb->pcfich_first_reg_idx + 3) & 3])
			enodeb->phich_reg[mprime][0]++;

		enodeb->phich_reg[mprime][1] = (enodeb->N_id_cell + mprime + (n0 / 3)) % n0;

		if (enodeb->phich_reg[mprime][1] >= pcfich_reg[enodeb->pcfich_first_reg_idx])
			enodeb->phich_reg[mprime][1]++;
		if (enodeb->phich_reg[mprime][1] >= pcfich_reg[(enodeb->pcfich_first_reg_idx + 1) & 3])
			enodeb->phich_reg[mprime][1]++;
		if (enodeb->phich_reg[mprime][1] >= pcfich_reg[(enodeb->pcfich_first_reg_idx + 2) & 3])
			enodeb->phich_reg[mprime][1]++;
		if (enodeb->phich_reg[mprime][1] >= pcfich_reg[(enodeb->pcfich_first_reg_idx + 3) & 3])
			enodeb->phich_reg[mprime][1]++;

		enodeb->phich_reg[mprime][2] = (enodeb->N_id_cell + mprime + (2 * n0 / 3)) % n0;
		if (enodeb->phich_reg[mprime][2] >= pcfich_reg[enodeb->pcfich_first_reg_idx])
			enodeb->phich_reg[mprime][2]++;
		if (enodeb->phich_reg[mprime][2] >= pcfich_reg[(enodeb->pcfich_first_reg_idx + 1) & 3])
			enodeb->phich_reg[mprime][2]++;
		if (enodeb->phich_reg[mprime][2] >= pcfich_reg[(enodeb->pcfich_first_reg_idx + 2) & 3])
			enodeb->phich_reg[mprime][2]++;
		if (enodeb->phich_reg[mprime][2] >= pcfich_reg[(enodeb->pcfich_first_reg_idx + 3) & 3])
			enodeb->phich_reg[mprime][2]++;
#else
#error "Extended CP not supported!"
#endif
	} // mprime loop

	return LTE_SUCCESS;
}

int32_t alam_bpsk_perm1[4] =
	{ 2, 1, 4, 3 }; // -conj(x) 1 (-1-j) -> 2 (1-j), 2->1, 3 (-1+j) -> (4) 1+j, 4->3
int32_t alam_bpsk_perm2[4] =
	{ 3, 4, 2, 1 }; // conj(x) 1 (-1-j) -> 3 (-1+j), 3->1, 2 (1-j) -> 4 (1+j), 4->2

// Generate PHICH
void liblte_generate_phich(lte_enodeb_t *enodeb, lte_subframe_t *sf, int16_t amp, uint8_t nseq_PHICH, uint8_t ngroup_PHICH, uint8_t HI)
{
	int16_t d[24], *dp;
	//  unsigned int i,aa;
	uint32_t re_offset;
	int16_t y0_16[8], y1_16[8];
	Complex16 *y0, *y1;
	// scrambling
	uint32_t x1, x2, s = 0;
	uint8_t reset = 1;
	int16_t cs[12];
	uint32_t i, i2, i3, m, j;
	int16_t gain_lin_QPSK;
	//uint32_t subframe_offset=0;
	Complex16 *y = sf->pdcch_syms;
	uint32_t subframe = sf->subframe_no;
	int32_t nushiftmod3 = enodeb->lte_enodeb_params.Vshift % 3;

	memset(&d, 0, 24 * sizeof(int16_t));

	gain_lin_QPSK = (int16_t) (((int32_t) amp * GAIN_SQRT_2) >> 15);

	// BPSK modulation of HI input (to be repeated 3 times, 36-212 Section 5.3.5, p. 56 in v8.6)
	if (HI > 0)
		HI = 1;

	x2 = (((subframe + 1) * ((enodeb->N_id_cell << 1) + 1)) << 9) + enodeb->N_id_cell;

	s = lte_gold_generic(&x1, &x2, reset);

	// compute scrambling sequence
	for (i = 0; i < 12; i++)
	{
		cs[i] = (uint8_t) ((s >> (i & 0x1f)) & 1);
		cs[i] = (cs[i] == 0) ? (1 - (HI << 1)) : ((HI << 1) - 1);
	}

	if (LTE_CP == LTE_CP_NORMAL)
	{
		// Normal Cyclic Prefix
		for (i = 0, i2 = 0, i3 = 0; i < 3; i++, i2 += 4, i3 += 8)
		{
			switch (nseq_PHICH)
			{
				case 0: // +1 +1 +1 +1
					d[i3] = cs[i2];
					d[1 + i3] = cs[i2];
					d[2 + i3] = cs[1 + i2];
					d[3 + i3] = cs[1 + i2];
					d[4 + i3] = cs[2 + i2];
					d[5 + i3] = cs[2 + i2];
					d[6 + i3] = cs[3 + i2];
					d[7 + i3] = cs[3 + i2];
					break;

				case 1: // +1 -1 +1 -1
					d[i3] = cs[i2];
					d[1 + i3] = cs[i2];
					d[2 + i3] = -cs[1 + i2];
					d[3 + i3] = -cs[1 + i2];
					d[4 + i3] = cs[2 + i2];
					d[5 + i3] = cs[2 + i2];
					d[6 + i3] = -cs[3 + i2];
					d[7 + i3] = -cs[3 + i2];
					break;

				case 2: // +1 +1 -1 -1
					d[i3] = cs[i2];
					d[1 + i3] = cs[i2];
					d[2 + i3] = cs[1 + i2];
					d[3 + i3] = cs[1 + i2];
					d[4 + i3] = -cs[2 + i2];
					d[5 + i3] = -cs[2 + i2];
					d[6 + i3] = -cs[3 + i2];
					d[7 + i3] = -cs[3 + i2];
					break;

				case 3: // +1 -1 -1 +1
					d[i3] = cs[i2];
					d[1 + i3] = cs[i2];
					d[2 + i3] = -cs[1 + i2];
					d[3 + i3] = -cs[1 + i2];
					d[4 + i3] = -cs[2 + i2];
					d[5 + i3] = -cs[2 + i2];
					d[6 + i3] = cs[3 + i2];
					d[7 + i3] = cs[3 + i2];
					break;

				case 4: // +j +j +j +j
					d[i3] = -cs[i2];
					d[1 + i3] = cs[i2];
					d[2 + i3] = -cs[1 + i2];
					d[3 + i3] = cs[1 + i2];
					d[4 + i3] = -cs[2 + i2];
					d[5 + i3] = cs[2 + i2];
					d[6 + i3] = -cs[3 + i2];
					d[7 + i3] = cs[3 + i2];
					break;

				case 5: // +j -j +j -j
					d[1 + i3] = cs[i2];
					d[3 + i3] = -cs[1 + i2];
					d[5 + i3] = cs[2 + i2];
					d[7 + i3] = -cs[3 + i2];
					d[i3] = -cs[i2];
					d[2 + i3] = cs[1 + i2];
					d[4 + i3] = -cs[2 + i2];
					d[6 + i3] = cs[3 + i2];
					break;

				case 6: // +j +j -j -j
					d[1 + i3] = cs[i2];
					d[3 + i3] = cs[1 + i2];
					d[5 + i3] = -cs[2 + i2];
					d[7 + i3] = -cs[3 + i2];
					d[i3] = -cs[i2];
					d[2 + i3] = -cs[1 + i2];
					d[4 + i3] = cs[2 + i2];
					d[6 + i3] = cs[3 + i2];
					break;

				case 7: // +j -j -j +j
					d[1 + i3] = cs[i2];
					d[3 + i3] = -cs[1 + i2];
					d[5 + i3] = -cs[2 + i2];
					d[7 + i3] = cs[3 + i2];
					d[i3] = -cs[i2];
					d[2 + i3] = cs[1 + i2];
					d[4 + i3] = cs[2 + i2];
					d[6 + i3] = -cs[3 + i2];
					break;

				default:
					ERROR(DTRX, "Illegal nseq_PHICH: %i\n", nseq_PHICH);
			}
		}

		// modulation here
		if (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX > 1)
		{
			// Alamouti precoding
			gain_lin_QPSK = (int16_t) (((int32_t) amp * GAIN_SQRT_2) >> 15);

			// Symbol 0
			re_offset = 0 + (enodeb->phich_reg[ngroup_PHICH][0]*6);

			if (re_offset > enodeb->fp.LTE_N_RE)
				re_offset -= (enodeb->fp.LTE_N_RE - 1);

			// Буфер антенны 0
			y0 = &y[re_offset];
			// Буфер антенны 1
			y1 = &y[re_offset + enodeb->fp.LTE_N_RE];

			// first antenna position n -> x0
			y0_16[0] = d[0]*gain_lin_QPSK;
			y0_16[1] = d[1]*gain_lin_QPSK;
			// second antenna position n -> -x1*
			y1_16[0] = -d[2]*gain_lin_QPSK;
			y1_16[1] = d[3]*gain_lin_QPSK;
			// fill in the rest of the ALAMOUTI precoding
			y0_16[2] = -y1_16[0];
			y0_16[3] = y1_16[1];
			y1_16[2] = y0_16[0];
			y1_16[3] = -y0_16[1];

			// first antenna position n -> x0
			y0_16[4] = d[4]*gain_lin_QPSK;
			y0_16[5] = d[5]*gain_lin_QPSK;
			// second antenna position n -> -x1*
			y1_16[4] = -d[6]*gain_lin_QPSK;
			y1_16[5] = d[7]*gain_lin_QPSK;
			// fill in the rest of the ALAMOUTI precoding
			y0_16[6] = -y1_16[4];
			y0_16[7] = y1_16[5];
			y1_16[6] = y0_16[4];
			y1_16[7] = -y0_16[5];

			for (i = 0, j = 0, m = 0; i < 6; i++, j++)
			{
				if ((i != nushiftmod3) && (i != (nushiftmod3 + 3)))
				{
					int16_t re0, re1, im0, im1;
					
					re0 = y0_16[m];
					re1 = y1_16[m++];
					im0 = y0_16[m];
					im1 = y1_16[m++];
					
					y0[j] = V_add2(y0[j], TO_COMPLEX16(re0, im0));
					y1[j] = V_add2(y1[j], TO_COMPLEX16(re1, im1));
				}
			}

			// Symbol 1
			re_offset = 0 + (enodeb->phich_reg[ngroup_PHICH][1]*6);

			if (re_offset > enodeb->fp.LTE_N_RE)
				re_offset -= (enodeb->fp.LTE_N_RE - 1);

			// Буфер антенны 0
			y0 = &y[re_offset];
			// Буфер антенны 1
			y1 = &y[re_offset + enodeb->fp.LTE_N_RE];

			// first antenna position n -> x0
			y0_16[0] = d[8]*gain_lin_QPSK;
			y0_16[1] = d[9]*gain_lin_QPSK;
			// second antenna position n -> -x1*
			y1_16[0] = -d[10]*gain_lin_QPSK;
			y1_16[1] = d[11]*gain_lin_QPSK;
			// fill in the rest of the ALAMOUTI precoding
			y0_16[2] = -y1_16[0];
			y0_16[3] = y1_16[1];
			y1_16[2] = y0_16[0];
			y1_16[3] = -y0_16[1];

			// first antenna position n -> x0
			y0_16[4] = d[12]*gain_lin_QPSK;
			y0_16[5] = d[13]*gain_lin_QPSK;
			// second antenna position n -> -x1*
			y1_16[4] = -d[14]*gain_lin_QPSK;
			y1_16[5] = d[15]*gain_lin_QPSK;
			// fill in the rest of the ALAMOUTI precoding
			y0_16[6] = -y1_16[4];
			y0_16[7] = y1_16[5];
			y1_16[6] = y0_16[4];
			y1_16[7] = -y0_16[5];

			for (i = 0, j = 0, m = 0; i < 6; i++, j++)
			{
				if ((i != (nushiftmod3)) && (i != (nushiftmod3 + 3)))
				{
					int16_t re0, re1, im0, im1;
					
					re0 = y0_16[m];
					re1 = y1_16[m++];
					im0 = y0_16[m];
					im1 = y1_16[m++];
					
					y0[j] = V_add2(y0[j], TO_COMPLEX16(re0, im0));
					y1[j] = V_add2(y1[j], TO_COMPLEX16(re1, im1));
				}
			}

			// Symbol 2
			re_offset = 0 + (enodeb->phich_reg[ngroup_PHICH][2]*6);

			if (re_offset > enodeb->fp.LTE_N_RE)
				re_offset -= (enodeb->fp.LTE_N_RE - 1);

			y0 = &y[re_offset];
			y1 = &y[re_offset + enodeb->fp.LTE_N_RE];

			// first antenna position n -> x0
			y0_16[0] = d[16]*gain_lin_QPSK;
			y0_16[1] = d[17]*gain_lin_QPSK;
			// second antenna position n -> -x1*
			y1_16[0] = -d[18]*gain_lin_QPSK;
			y1_16[1] = d[19]*gain_lin_QPSK;
			// fill in the rest of the ALAMOUTI precoding
			y0_16[2] = -y1_16[0];
			y0_16[3] = y1_16[1];
			y1_16[2] = y0_16[0];
			y1_16[3] = -y0_16[1];

			// first antenna position n -> x0
			y0_16[4] = d[20]*gain_lin_QPSK;
			y0_16[5] = d[21]*gain_lin_QPSK;
			// second antenna position n -> -x1*
			y1_16[4] = -d[22]*gain_lin_QPSK;
			y1_16[5] = d[23]*gain_lin_QPSK;
			// fill in the rest of the ALAMOUTI precoding
			y0_16[6] = -y1_16[4];
			y0_16[7] = y1_16[5];
			y1_16[6] = y0_16[4];
			y1_16[7] = -y0_16[5];

			for (i = 0, j = 0, m = 0; i < 6; i++, j++)
			{
				if ((i != (nushiftmod3)) && (i != (nushiftmod3 + 3)))
				{
					int16_t re0, re1, im0, im1;
					
					re0 = y0_16[m];
					re1 = y1_16[m++];
					im0 = y0_16[m];
					im1 = y1_16[m++];
					
					y0[j] = V_add2(y0[j], TO_COMPLEX16(re0, im0));
					y1[j] = V_add2(y1[j], TO_COMPLEX16(re1, im1));
				}
			}

		} // Alamouti
		else
		{
			// SISO
			// Symbol 0
			re_offset = 0 + (enodeb->phich_reg[ngroup_PHICH][0] * 6);

			if (re_offset > enodeb->fp.LTE_N_RE)
				re_offset -= enodeb->fp.LTE_N_RE - 1;

			y0 = &y[re_offset];

			y0_16[0] = d[0] * gain_lin_QPSK;
			y0_16[1] = d[1] * gain_lin_QPSK;
			y0_16[2] = d[2] * gain_lin_QPSK;
			y0_16[3] = d[3] * gain_lin_QPSK;
			y0_16[4] = d[4] * gain_lin_QPSK;
			y0_16[5] = d[5] * gain_lin_QPSK;
			y0_16[6] = d[6] * gain_lin_QPSK;
			y0_16[7] = d[7] * gain_lin_QPSK;

			for (i = 0, j = 0, m = 0; i < 6; i++, j++)
			{
				if ((i != nushiftmod3) && (i != (nushiftmod3 + 3)))
				{
					int16_t re, im;
					re = y0_16[m++];
					im = y0_16[m++];

					y0[j] = V_add2(y0[j], TO_COMPLEX16(re, im));
				}
			}

			// Symbol 1
			re_offset = 0 + (enodeb->phich_reg[ngroup_PHICH][1] * 6);

			if (re_offset > enodeb->fp.LTE_N_RE)
				re_offset -= (enodeb->fp.LTE_N_RE - 1);

			y0 = &y[re_offset];

			y0_16[0] = d[8] * gain_lin_QPSK;
			y0_16[1] = d[9] * gain_lin_QPSK;
			y0_16[2] = d[10] * gain_lin_QPSK;
			y0_16[3] = d[11] * gain_lin_QPSK;
			y0_16[4] = d[12] * gain_lin_QPSK;
			y0_16[5] = d[13] * gain_lin_QPSK;
			y0_16[6] = d[14] * gain_lin_QPSK;
			y0_16[7] = d[15] * gain_lin_QPSK;

			for (i = 0, j = 0, m = 0; i < 6; i++, j++)
			{
				if ((i != nushiftmod3) && (i != (nushiftmod3 + 3)))
				{
					int16_t re, im;
					re = y0_16[m++];
					im = y0_16[m++];
					y0[j] = V_add2(y0[j], TO_COMPLEX16(re, im));
				}
			}

			// Symbol 2
			re_offset = 0 + (enodeb->phich_reg[ngroup_PHICH][2] * 6);

			if (re_offset > enodeb->fp.LTE_N_RE)
				re_offset -= (enodeb->fp.LTE_N_RE - 1);

			y0 = &y[re_offset];

			y0_16[0] = d[16] * gain_lin_QPSK;
			y0_16[1] = d[17] * gain_lin_QPSK;
			y0_16[2] = d[18] * gain_lin_QPSK;
			y0_16[3] = d[19] * gain_lin_QPSK;
			y0_16[4] = d[20] * gain_lin_QPSK;
			y0_16[5] = d[21] * gain_lin_QPSK;
			y0_16[6] = d[22] * gain_lin_QPSK;
			y0_16[7] = d[23] * gain_lin_QPSK;

			for (i = 0, j = 0, m = 0; i < 6; i++, j++)
			{
				if ((i != nushiftmod3) && (i != (nushiftmod3 + 3)))
				{

					int16_t re, im;
					re = y0_16[m++];
					im = y0_16[m++];
					y0[j] = V_add2(y0[j], TO_COMPLEX16(re, im));

				}
			}
		}
	}
	else
	{
		ERROR(DTRX, "Extended CP not supported!\n");
	}
}

void liblte_generate_phich_top(lte_enodeb_t *enodeb, lte_subframe_t *sf, fapi_hi_dci0_request_body_t *hi_dci0)
{
	uint8_t Ngroup_PHICH, ngroup_PHICH, nseq_PHICH;
	uint8_t NSF_PHICH = 4;
	uint32_t i;

	/* compute Ngroup_PHICH (6.9 in 36-211 */
	Ngroup_PHICH = (enodeb->fapi_config.phich_config.phich_resource * enodeb->fp.LTE_N_RB_DL) / 48;
	if (((enodeb->fapi_config.phich_config.phich_resource * enodeb->fp.LTE_N_RB_DL) % 48) > 0)
		Ngroup_PHICH++;

	if (LTE_CP == LTE_CP_EXTENDED)
		NSF_PHICH = 2;

	for (i = hi_dci0->number_of_dci; i < hi_dci0->number_of_dci + hi_dci0->number_of_hi; i++)
	{
		ngroup_PHICH = (hi_dci0->hi_dci0_pdu_list[i].hi_pdu.hi_pdu_rel8.resource_block_start
				+ hi_dci0->hi_dci0_pdu_list[i].hi_pdu.hi_pdu_rel8.cyclic_shift_2_for_drms) % Ngroup_PHICH;

		if (hi_dci0->hi_dci0_pdu_list[i].hi_pdu.hi_pdu_rel8.i_phich == 1)
			ngroup_PHICH += Ngroup_PHICH;

		nseq_PHICH = ((hi_dci0->hi_dci0_pdu_list[i].hi_pdu.hi_pdu_rel8.resource_block_start / Ngroup_PHICH)
				+ hi_dci0->hi_dci0_pdu_list[i].hi_pdu.hi_pdu_rel8.cyclic_shift_2_for_drms) % (2 * NSF_PHICH);

		liblte_generate_phich(enodeb, sf, hi_dci0->hi_dci0_pdu_list[i].hi_pdu.hi_pdu_rel8.transmission_power, nseq_PHICH,
				ngroup_PHICH, hi_dci0->hi_dci0_pdu_list[i].hi_pdu.hi_pdu_rel8.hi_value);
	}
}

/**
 * PFICH scrambling
 * @param enodeb
 * @param subframe
 * @param b
 * @param bt
 */
static void liblte_pcfich_scrambling(lte_enodeb_t *enodeb, uint32_t subframe, uint8_t *b, uint8_t *bt)
{

	uint32_t i;
	uint32_t reset;
	uint32_t x1, x2, s = 0;

	reset = 1;

	x2 = ((((2 * enodeb->N_id_cell) + 1) * (1 + subframe)) << 9) + enodeb->N_id_cell; 
	for (i = 0; i < 32; i++)
	{
		if ((i & 0x1f) == 0)
		{
			s = lte_gold_generic(&x1, &x2, reset);
			reset = 0;
		}

		bt[i] = (b[i] & 1) ^ ((s >> (i & 0x1f)) & 1);
	}
}

/**
 * Значение CFI для различных вариантов передачи PDCCH
 */
static uint8_t pcfich_b[4][32] =
	{
		{ 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1 },
		{ 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0 },
		{ 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

lte_status_t liblte_generate_pcfich(uint32_t num_pdcch_symbols, int16_t amp, lte_enodeb_t *enodeb,
		lte_subframe_t *lte_sf)
{

	uint8_t pcfich_bt[32] =
		{ 0 }, nsymb, pcfich_quad;
	Complex16 pcfich_d[2][16];
	uint32_t i;
	uint32_t symbol_offset, m, re_offset, reg_offset;
	uint32_t *pcfich_reg;
	int32_t nushiftmod3;

	OS_ASSERT_COND(enodeb != NULL);
	OS_ASSERT_COND(lte_sf != NULL);
	OS_ASSERT_COND((num_pdcch_symbols > 0) && (num_pdcch_symbols < 4));

	nushiftmod3 = enodeb->lte_enodeb_params.Vshift % 3;
	pcfich_reg = enodeb->pcfich_reg;

	if ((num_pdcch_symbols > 0) && (num_pdcch_symbols < 4))
		liblte_pcfich_scrambling(enodeb, lte_sf->subframe_no, pcfich_b[num_pdcch_symbols - 1], pcfich_bt);


	if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 1)
	{
		/* SISO */	
		for (i = 0; i < 16; i++)
		{
			int16_t iq_i, iq_q;
			iq_i = ((pcfich_bt[2 * i] == 1) ? -amp : amp);
			iq_q = ((pcfich_bt[2 * i + 1] == 1) ? -amp : amp);
	
			pcfich_d[0][i] = TO_COMPLEX16(iq_i, iq_q);
			pcfich_d[1][i] = TO_COMPLEX16(iq_i, iq_q);
		}
	}
	else if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2)
	{
		/* ALAMOUTI */	
		for (i = 0; i < 16; i+=2)
		{
			int16_t iq0_i, iq0_q, iq1_i, iq1_q;
			
			iq0_i = ((pcfich_bt[2 * i] == 1) ? -amp : amp);
			iq0_q = ((pcfich_bt[2 * i + 1] == 1) ? -amp : amp);
			iq1_i = ((pcfich_bt[2 * i + 2] == 1) ? -amp : amp);
			iq1_q = ((pcfich_bt[2 * i + 3] == 1) ? -amp : amp);
			
			pcfich_d[0][i] = TO_COMPLEX16(iq0_i, iq0_q);
			pcfich_d[1][i] = TO_COMPLEX16(-iq1_i, iq1_q);
			pcfich_d[0][i + 1] = TO_COMPLEX16(iq1_i, iq1_q);
			pcfich_d[1][i + 1] = TO_COMPLEX16(iq0_i, -iq0_q);
		}
	}

	/* mapping */
	nsymb = LTE_NSYMB_PER_SUBFRAME;

	re_offset = LTE_FIRST_CARRIER_OFFSET;

	/* lookup REGs */
	m = 0;
	for (pcfich_quad = 0; pcfich_quad < 4; pcfich_quad++)
	{
		reg_offset = re_offset + ((uint16_t) pcfich_reg[pcfich_quad] * 6);

		if (reg_offset >= enodeb->fp.LTE_SYMBOL_LEN)
			reg_offset = 1 + reg_offset - enodeb->fp.LTE_SYMBOL_LEN;

		for (i = 0; i < 6; i++)
		{
			if ((i != nushiftmod3) && (i != (nushiftmod3 + 3)))
			{
				lte_sf->pdcch_syms[0 + reg_offset + i] = pcfich_d[0][m];
			
				if (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2)
				{
					// Заполнение PCFICH для второй антенны
					lte_sf->pdcch_syms[enodeb->fp.LTE_N_RE + reg_offset + i] = pcfich_d[1][m];
				}
				
				m++;
			}
		}
	}

	return LTE_SUCCESS;
}

uint32_t get_num_bits_in_prb(uint32_t N_subframe, uint32_t N_ctrl_symbs, uint32_t prb, uint32_t N_rb_dl, uint8_t N_ant,
		uint32_t mod_order)
{
	uint32_t first_prb;
	uint32_t last_prb;
	uint32_t N_REs;
	uint32_t N_bits;
	int32_t partial_prb = 0;

	// Determine first and last PBCH, PSS, and SSS PRBs
	if (N_rb_dl == 6)
	{
		partial_prb = 0;
		first_prb = 0;
		last_prb = 5;
	}
	else if (N_rb_dl == 15)
	{
		partial_prb = 1;
		first_prb = 4;
		last_prb = 10;
	}
	else if (N_rb_dl == 25)
	{
		partial_prb = 1;
		first_prb = 9;
		last_prb = 15;
	}
	else if (N_rb_dl == 50)
	{
		partial_prb = 0;
		first_prb = 22;
		last_prb = 27;
	}
	else if (N_rb_dl == 75)
	{
		partial_prb = 1;
		first_prb = 34;
		last_prb = 40;
	}
	else
	{
		partial_prb = 0;
		first_prb = 47;
		last_prb = 52;
	}

	if (N_ant == 1)
	{
		// Start with raw number of resource elements
		N_REs = (14 - 4) * 12 + 4 * 10;

		// Remove control symbols
		N_REs -= ((N_ctrl_symbs - 1) * 12 + 10);

		// Remove PBCH, PSS, and SSS
		if (prb >= first_prb && prb <= last_prb)
		{
			if (partial_prb && (prb == first_prb || prb == last_prb))
			{
				if (N_subframe == 0)
					N_REs -= (5 * 6 + 5);
				else if (N_subframe == 5)
					N_REs -= 2 * 6;
			}
			else
			{
				if (N_subframe == 0)
					N_REs -= (5 * 12 + 10);
				else if (N_subframe == 5)
					N_REs -= 2 * 12;
			}
		}
	}
	else if (N_ant == 2)
	{
		// Start with raw number of resource elements
		N_REs = (14 - 4) * 12 + 4 * 8;

		// Remove control symbols
		N_REs -= ((N_ctrl_symbs - 1) * 12 + 8);

		// Remove PBCH, PSS, and SSS
		if (prb >= first_prb && prb <= last_prb)
		{
			if (partial_prb && (prb == first_prb || prb == last_prb))
			{
				if (N_subframe == 0)
					N_REs -= (5 * 6 + 4);
				else if (N_subframe == 5)
					N_REs -= 2 * 6;
			}
			else
			{
				if (N_subframe == 0)
					N_REs -= (5 * 12 + 8);
				else if (N_subframe == 5)
					N_REs -= 2 * 12;
			}
		}
	}
	else
	{ // N_ant == 4
	  // Start with raw number of resource elements
		N_REs = (14 - 6) * 12 + 6 * 8;

		// Remove control symbols
		if (1 == N_ctrl_symbs)
			N_REs -= 8;
		else
			N_REs -= ((N_ctrl_symbs - 2) * 12 + 2 * 8);

		// Remove PBCH, PSS, and SSS
		if (prb >= first_prb && prb <= last_prb)
		{
			if (partial_prb && (prb == first_prb || prb == last_prb))
			{
				if (N_subframe == 0)
					N_REs -= (4 * 6 + 2 * 4);
				else if (N_subframe == 5)
					N_REs -= 2 * 6;
			}
			else
			{
				if (N_subframe == 0)
					N_REs -= (4 * 12 + 2 * 8);
				else if (N_subframe == 5)
					N_REs -= 2 * 12;
			}
		}
	}

	N_bits = N_REs * mod_order;
	
	return (N_bits);
}

static void init_pucch_w_tables()
{
	w_arg_pucch_format1_cpnorm[0][0] = 0;
	w_arg_pucch_format1_cpnorm[0][1] = 0;
	w_arg_pucch_format1_cpnorm[0][2] = 0;
	w_arg_pucch_format1_cpnorm[1][0] = 0;
	w_arg_pucch_format1_cpnorm[1][1] = 2.0f * M_PI / 3.0f;
	w_arg_pucch_format1_cpnorm[1][2] = 4.0f * M_PI / 3.0f;
	w_arg_pucch_format1_cpnorm[2][0] = 0;
	w_arg_pucch_format1_cpnorm[2][1] = 4.0f * M_PI / 3.0f;
	w_arg_pucch_format1_cpnorm[2][2] = 2.0f * M_PI / 3.0f;

	w_arg_pucch_format1_cpext[0][0] = 0;
	w_arg_pucch_format1_cpext[0][1] = 0;
	w_arg_pucch_format1_cpext[1][0] = 0;
	w_arg_pucch_format1_cpext[1][1] = M_PI;
	w_arg_pucch_format1_cpext[2][0] = 0;
	w_arg_pucch_format1_cpext[2][1] = 0;

	w_n_oc[0][0][0] = 0;
	w_n_oc[0][0][1] = 0;
	w_n_oc[0][0][2] = 0;
	w_n_oc[0][0][3] = 0;
	w_n_oc[0][1][0] = 0;
	w_n_oc[0][1][1] = M_PI;
	w_n_oc[0][1][2] = 0;
	w_n_oc[0][1][3] = M_PI;
	w_n_oc[0][2][0] = 0;
	w_n_oc[0][2][1] = M_PI;
	w_n_oc[0][2][2] = M_PI;
	w_n_oc[0][2][3] = 0;

	w_n_oc[1][0][0] = 0;
	w_n_oc[1][0][1] = 0;
	w_n_oc[1][0][2] = 0;
	w_n_oc[1][0][3] = 0;
	w_n_oc[1][1][0] = 0;
	w_n_oc[1][1][1] = 2.0f * M_PI / 3.0f;
	w_n_oc[1][1][2] = 4.0f * M_PI / 3.0f;
	w_n_oc[1][1][3] = 0;
	w_n_oc[1][2][0] = 0;
	w_n_oc[1][2][1] = 4.0f * M_PI / 3.0f;
	w_n_oc[1][2][2] = 2.0f * M_PI / 3.0f;
	w_n_oc[1][2][3] = 0;
}

lte_status_t liblte_init()
{
	int32_t i;
	
	ccodelte_init();

	init_pucch_w_tables();

	crcTableInit();
	
	/* Инициализация таблиц sin/cos */
	for (i = 0; i < SINCOS_TABLESIZE; i++)
	{
		cos_tab[i] = cosf(2.0 * M_PI * (float)i / (float) SINCOS_TABLESIZE) * MAX_16;
		sin_tab[i] = sinf(2.0 * M_PI * (float)i / (float) SINCOS_TABLESIZE) * MAX_16;
	}

	return LTE_SUCCESS;
}

int32_t liblte_gen_dl_cell_spec(lte_enodeb_t *enodeb, Complex16 *output, int16_t amp, uint8_t Ns, uint8_t l, uint8_t p)
{

	unsigned char nu, mprime, mprime_dword, mprime_qpsk_symb, m;
	unsigned short k, a;
	Complex16 qpsk[4];

	a = amp;
	qpsk[0] = TO_COMPLEX16(a, a);
	qpsk[1] = TO_COMPLEX16(-a, a);
	qpsk[2] = TO_COMPLEX16(a, -a);
	qpsk[3] = TO_COMPLEX16(-a, -a);

	if ((p == 0) && (l == 0))
		nu = 0;
	else if ((p == 0) && (l > 0))
		nu = 3;
	else if ((p == 1) && (l == 0))
		nu = 3;
	else if ((p == 1) && (l > 0))
		nu = 0;
	else
		return (-1);

	mprime = 110 - enodeb->fp.LTE_N_RB_DL;

	k = (nu + enodeb->lte_enodeb_params.Vshift);

	if (k > 5)
		k -= 6;

	for (m = 0; m < enodeb->fp.LTE_N_RB_DL << 1; m++)
	{

		mprime_dword = mprime >> 4;
		mprime_qpsk_symb = mprime & 0xf;

		// r_mprime 3GPP 36-211 6.10.1.2
		output[k] = qpsk[(enodeb->refsigs->lte_gold_table[Ns][l][mprime_dword] >> (2 * mprime_qpsk_symb)) & 3];

		mprime++;
		k += 6;

		if (k >= enodeb->fp.LTE_N_RE)
		{
			k -= enodeb->fp.LTE_N_RE;
		}
	}

	return LTE_SUCCESS;
}

void liblte_generate_prs_c(uint32_t c_init, uint32_t len, uint32_t *c)
{
	uint32_t i;
	uint32_t x1;
	uint32_t x2;
	uint8_t new_bit1;
	uint8_t new_bit2;

	// Initialize the 2nd m-sequence
	x2 = c_init;

	// Advance the 2nd m-sequence
	for (i = 0; i < (1600 - 31); i++)
	{
		new_bit2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 0x1;

		x2 = (x2 >> 1) | (new_bit2 << 30);
	}

	// Initialize the 1st m-sequence
	x1 = 0x54D21B24; // This is the result of advancing the initial value of 0x00000001

	// Generate c
	for (i = 0; i < len; i++)
	{
		new_bit1 = ((x1 >> 3) ^ x1) & 0x1;
		new_bit2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 0x1;

		x1 = (x1 >> 1) | (new_bit1 << 30);
		x2 = (x2 >> 1) | (new_bit2 << 30);

		c[i] = new_bit1 ^ new_bit2;
	}
}

void liblte_generate_ul_rs(lte_enodeb_t *enodeb, uint32_t N_slot, uint32_t N_prb, float alpha, Complex16 *ul_rs)
{
	float q_bar;
	uint32_t M_sc_rs;
	uint32_t N_zc_rs;
	uint32_t i;
	uint32_t f_ss;
	uint32_t f_gh;
	uint32_t v;
	uint32_t u;
	int32_t q;
	bool group_hopping_enabled = (enodeb->fapi_config.uplink_reference_signal_config.uplink_rs_hopping == 1);
	bool sequence_hopping_enabled = (enodeb->fapi_config.uplink_reference_signal_config.uplink_rs_hopping == 2);
	uint32_t delta_ss = enodeb->fapi_config.uplink_reference_signal_config.group_assignment;

	// Calculate M_sc_rs
	M_sc_rs = N_prb * LTE_N_SC_RB;

	// Determine N_zc_rs
	for (i = UL_RS_PRIMES_LESS_THAN_2048_LEN - 1; i > 0; i--)
	{
		if (UL_RS_PRIMES_LESS_THAN_2048[i] < M_sc_rs)
		{
			N_zc_rs = UL_RS_PRIMES_LESS_THAN_2048[i];
			break;
		}
	}

	// Determine f_ss
	f_ss = ((enodeb->N_id_cell % 30) + delta_ss) % 30;

	// Determine u
	if (group_hopping_enabled)
	{
		liblte_generate_prs_c(enodeb->N_id_cell / 30, 160, (uint32_t *) &enodeb->refsigs->ulrs_c);
		f_gh = 0;
		for (i = 0; i < 8; i++)
		{
			f_gh += enodeb->refsigs->ulrs_c[8 * N_slot + i] << i;
		}
		f_gh %= 30;
		u = (f_gh + f_ss) % 30;
	}
	else
	{
		u = f_ss % 30;
	}

	// Determine v
	if (M_sc_rs < 6 * LTE_N_SC_RB)
	{
		v = 0;
	}
	else
	{
		if (!group_hopping_enabled && sequence_hopping_enabled)
		{
			liblte_generate_prs_c(((enodeb->N_id_cell / 30) << 5) + f_ss, 20, (uint32_t *) &enodeb->refsigs->ulrs_c);
			v = enodeb->refsigs->ulrs_c[N_slot];
		}
		else
		{
			v = 0;
		}
	}

	// Determine r_bar_u_v
	if (M_sc_rs >= 3 * LTE_N_SC_RB)
	{
		q_bar = (float) N_zc_rs * (float) (u + 1) / (float) 31;

		if ((((uint32_t) (2 * q_bar)) % 2) == 0)
			q = (uint32_t) (q_bar + 0.5) + v;
		else
			q = (uint32_t) (q_bar + 0.5) - v;

		for (i = 0; i < N_zc_rs; i++)
		{
			enodeb->refsigs->ulrs_x_q_re[i] = cosf(-M_PI * q * i * (i + 1) / N_zc_rs);
			enodeb->refsigs->ulrs_x_q_im[i] = sinf(-M_PI * q * i * (i + 1) / N_zc_rs);
		}
		for (i = 0; i < M_sc_rs; i++)
		{
			enodeb->refsigs->ulrs_r_bar_u_v_re[i] = enodeb->refsigs->ulrs_x_q_re[i % N_zc_rs];
			enodeb->refsigs->ulrs_r_bar_u_v_im[i] = enodeb->refsigs->ulrs_x_q_im[i % N_zc_rs];
		}
	}
	else if (M_sc_rs == LTE_N_SC_RB)
	{
		for (i = 0; i < M_sc_rs; i++)
		{
			enodeb->refsigs->ulrs_r_bar_u_v_re[i] = cosf(UL_RS_5_5_1_2_1[u][i] * M_PI / 4);
			enodeb->refsigs->ulrs_r_bar_u_v_im[i] = sinf(UL_RS_5_5_1_2_1[u][i] * M_PI / 4);
		}
	}
	else
	{
		for (i = 0; i < M_sc_rs; i++)
		{
			enodeb->refsigs->ulrs_r_bar_u_v_re[i] = cosf(UL_RS_5_5_1_2_2[u][i] * M_PI / 4);
			enodeb->refsigs->ulrs_r_bar_u_v_im[i] = sinf(UL_RS_5_5_1_2_2[u][i] * M_PI / 4);
		}
	}

	for (i = 0; i < M_sc_rs; i++)
	{
		float re, im;

		re = 32760.0f * (cosf(alpha*i)*enodeb->refsigs->ulrs_r_bar_u_v_re[i] - sinf(alpha*i)*enodeb->refsigs->ulrs_r_bar_u_v_im[i]);
		im = 32760.0f * (sinf(alpha*i)*enodeb->refsigs->ulrs_r_bar_u_v_re[i] + cosf(alpha*i)*enodeb->refsigs->ulrs_r_bar_u_v_im[i]);
		
		ul_rs[i] = TO_COMPLEX16(floorf(re), floorf(im));
	}
}

/* 36.211 5.5.2 */
void liblte_generate_dmrs_pusch(lte_enodeb_t *enodeb, uint32_t N_subfr, uint32_t cyclic_shift_dci, uint32_t N_prb,
		uint32_t layer, Complex16 *dmrs_0, Complex16 *dmrs_1)
{
	uint32_t N_slot;
	uint32_t lambda;
	uint32_t f_ss_pusch;
	uint32_t n_pn_ns_1;
	uint32_t n_pn_ns_2;
	uint32_t n_1_dmrs;
	uint32_t n_2_dmrs_lambda;
	uint32_t n_cs_lambda_1;
	uint32_t n_cs_lambda_2;
	uint32_t M_sc_rb = N_prb * LTE_N_SC_RB;
	uint32_t N_ul_symb = 7;
	uint32_t i;
	float alpha_lambda_1;
	float alpha_lambda_2;
	Complex16 r_u_v_alpha_lambda[2][110 * 12];
	int32_t w_vector[2];

	// Calculate N_slot
	N_slot = N_subfr * 2;

	// Set lambda
	lambda = layer;

	// Calculate f_ss_pusch
	f_ss_pusch = ((enodeb->N_id_cell % 30) + enodeb->fapi_config.uplink_reference_signal_config.group_assignment) % 30;

	// Generate c
	liblte_generate_prs_c(((enodeb->N_id_cell / 30) << 5) + f_ss_pusch, 8 * N_ul_symb * 20,
			enodeb->refsigs->pusch_dmrs_c);

	// Calculate n_pn_ns
	n_pn_ns_1 = 0;
	n_pn_ns_2 = 0;
	for (i = 0; i < 8; i++)
	{
		n_pn_ns_1 += enodeb->refsigs->pusch_dmrs_c[8 * N_ul_symb * N_slot + i] << i;
		n_pn_ns_2 += enodeb->refsigs->pusch_dmrs_c[8 * N_ul_symb * (N_slot + 1) + i] << i;
	}

	// Determine n_1_dmrs
	n_1_dmrs = N_1_DMRS_5_5_2_1_1_2[enodeb->fapi_config.uplink_reference_signal_config.cyclic_shift_1_for_drms];

	// Determine n_2_dmrs_lambda
	n_2_dmrs_lambda = N_2_DMRS_LAMBDA_5_5_2_1_1_1[cyclic_shift_dci][lambda];

	// Calculate n_cs_lambda
	n_cs_lambda_1 = (n_1_dmrs + n_2_dmrs_lambda + n_pn_ns_1) % 12;
	n_cs_lambda_2 = (n_1_dmrs + n_2_dmrs_lambda + n_pn_ns_2) % 12;

	// Calculate alpha_lambda
	alpha_lambda_1 = 2 * M_PI * n_cs_lambda_1 / 12;
	alpha_lambda_2 = 2 * M_PI * n_cs_lambda_2 / 12;

	// Generate the base reference signal
	liblte_generate_ul_rs(enodeb, N_slot, N_prb, alpha_lambda_1, r_u_v_alpha_lambda[0]);

	liblte_generate_ul_rs(enodeb, N_slot + 1, N_prb, alpha_lambda_2, r_u_v_alpha_lambda[1]);

	// Determine w vector
	// FIXME: Handle fixed values
	w_vector[0] = TO_COMPLEX16(1 * 32767, 0);
	w_vector[1] = TO_COMPLEX16(W_1_5_5_2_1_1_1[cyclic_shift_dci][lambda] * 32767, 0);

	//w_vector[0] = TO_COMPLEX16(1 * 128, 0);
	//w_vector[1] = TO_COMPLEX16(W_1_5_5_2_1_1_1[cyclic_shift_dci][lambda] * 128, 0);

	//w_vector[0] = TO_COMPLEX16(1 * 8190, 0);
	//w_vector[1] = TO_COMPLEX16(W_1_5_5_2_1_1_1[cyclic_shift_dci][lambda] * 8190, 0);

	//w_vector[0] = TO_COMPLEX16(1 * 23160 / 1, 0);
	//w_vector[1] = TO_COMPLEX16(W_1_5_5_2_1_1_1[cyclic_shift_dci][lambda] * 23160 / 1, 0);

	// Generate the PUSCH demodulation reference signal sequence
	for (i = 0; i < M_sc_rb; i++)
	{
		Complex32 t;
		t = C_L_mpy(w_vector[0], r_u_v_alpha_lambda[0][i]);
		dmrs_0[i] = V_pack_2fr(creal32(t), cimag32(t));

		t = C_L_mpy(w_vector[1], r_u_v_alpha_lambda[1][i]);
		dmrs_1[i] = V_pack_2fr(creal32(t), cimag32(t));
	}

	// FIXME: Add precoding
}

uint32_t dftsizes[33] =
	{ 12, 24, 36, 48, 60, 72, 96, 108, 120, 144, 180, 192, 216, 240, 288, 300, 324, 360, 384, 432, 480, 540, 576, 600,
		648, 720, 864, 900, 960, 972, 1080, 1152, 1200 };

static uint32_t ref_primes[33] __attribute__((section(".local_data_ddr0_cacheable"))) =
	{ 11, 23, 31, 47, 59, 71, 89, 107, 113, 139, 179, 191, 211, 239, 283, 293, 317, 359, 383, 431, 479, 523, 571, 599,
		647, 719, 863, 887, 953, 971, 1069, 1151, 1193 };

/* 36.211 table 5.5.1.2-1 */
static int32_t ref12[360] __attribute__((section(".local_data_ddr0_cacheable"))) =
	{ -1, 1, 3, -3, 3, 3, 1, 1, 3, 1, -3, 3, 1, 1, 3, 3, 3, -1, 1, -3, -3, 1, -3, 3, 1, 1, -3, -3, -3, -1, -3, -3, 1,
		-3, 1, -1, -1, 1, 1, 1, 1, -1, -3, -3, 1, -3, 3, -1, -1, 3, 1, -1, 1, -1, -3, -1, 1, -1, 1, 3, 1, -3, 3, -1, -1,
		1, 1, -1, -1, 3, -3, 1, -1, 3, -3, -3, -3, 3, 1, -1, 3, 3, -3, 1, -3, -1, -1, -1, 1, -3, 3, -1, 1, -3, 3, 1, 1,
		-3, 3, 1, -1, -1, -1, 1, 1, 3, -1, 1, 1, -3, -1, 3, 3, -1, -3, 1, 1, 1, 1, 1, -1, 3, -1, 1, 1, -3, -3, -1, -3,
		-3, 3, -1, 3, 1, -1, -1, 3, 3, -3, 1, 3, 1, 3, 3, 1, -3, 1, 1, -3, 1, 1, 1, -3, -3, -3, 1, 3, 3, -3, 3, -3, 1,
		1, 3, -1, -3, 3, 3, -3, 1, -1, -3, -1, 3, 1, 3, 3, 3, -1, 1, 3, -1, 1, -3, -1, -1, 1, 1, 3, 1, -1, -3, 1, 3, 1,
		-1, 1, 3, 3, 3, -1, -1, 3, -1, -3, 1, 1, 3, -3, 3, -3, -3, 3, 1, 3, -1, -3, 3, 1, 1, -3, 1, -3, -3, -1, -1, 1,
		-3, -1, 3, 1, 3, 1, -1, -1, 3, -3, -1, -3, -1, -1, -3, 1, 1, 1, 1, 3, 1, -1, 1, -3, -1, -1, 3, -1, 1, -3, -3,
		-3, -3, -3, 1, -1, -3, 1, 1, -3, -3, -3, -3, -1, 3, -3, 1, -3, 3, 1, 1, -1, -3, -1, -3, 1, -1, 1, 3, -1, 1, 1,
		1, 3, 1, 3, 3, -1, 1, -1, -3, -3, 1, 1, -3, 3, 3, 1, 3, 3, 1, -3, -1, -1, 3, 1, 3, -3, -3, 3, -3, 1, -1, -1, 3,
		-1, -3, -3, -1, -3, -1, -3, 3, 1, -1, 1, 3, -3, -3, -1, 3, -3, 3, -1, 3, 3, -3, 3, 3, -1, -1, 3, -3, -3, -1, -1,
		-3, -1, 3, -3, 3, 1, -1 };

/* 36.211 table 5.5.1.2-2 */
static int32_t ref24[720] __attribute__((section(".local_data_ddr0_cacheable"))) =
	{ -1, 3, 1, -3, 3, -1, 1, 3, -3, 3, 1, 3, -3, 3, 1, 1, -1, 1, 3, -3, 3, -3, -1, -3, -3, 3, -3, -3, -3, 1, -3, -3, 3,
		-1, 1, 1, 1, 3, 1, -1, 3, -3, -3, 1, 3, 1, 1, -3, 3, -1, 3, 3, 1, 1, -3, 3, 3, 3, 3, 1, -1, 3, -1, 1, 1, -1, -3,
		-1, -1, 1, 3, 3, -1, -3, 1, 1, 3, -3, 1, 1, -3, -1, -1, 1, 3, 1, 3, 1, -1, 3, 1, 1, -3, -1, -3, -1, -1, -1, -1,
		-3, -3, -1, 1, 1, 3, 3, -1, 3, -1, 1, -1, -3, 1, -1, -3, -3, 1, -3, -1, -1, -3, 1, 1, 3, -1, 1, 3, 1, -3, 1, -3,
		1, 1, -1, -1, 3, -1, -3, 3, -3, -3, -3, 1, 1, 1, 1, -1, -1, 3, -3, -3, 3, -3, 1, -1, -1, 1, -1, 1, 1, -1, -3,
		-1, 1, -1, 3, -1, -3, -3, 3, 3, -1, -1, -3, -1, 3, 1, 3, 1, 3, 1, 1, -1, 3, 1, -1, 1, 3, -3, -1, -1, 1, -3, 1,
		3, -3, 1, -1, -3, 3, -3, 3, -1, -1, -1, -1, 1, -3, -3, -3, 1, -3, -3, -3, 1, -3, 1, 1, -3, 3, 3, -1, -3, -1, 3,
		-3, 3, 3, 3, -1, 1, 1, -3, 1, -1, 1, 1, -3, 1, 1, -1, 1, -3, -3, 3, -1, 3, -1, -1, -3, -3, -3, -1, -3, -3, 1,
		-1, 1, 3, 3, -1, 1, -1, 3, 1, 3, 3, -3, -3, 1, 3, 1, -1, -3, -3, -3, 3, 3, -3, 3, 3, -1, -3, 3, -1, 1, -3, 1, 1,
		3, 3, 1, 1, 1, -1, -1, 1, -3, 3, -1, 1, 1, -3, 3, 3, -1, -3, 3, -3, -1, -3, -1, 3, -1, -1, -1, -1, -3, -1, 3, 3,
		1, -1, 1, 3, 3, 3, -1, 1, 1, -3, 1, 3, -1, -3, 3, -3, -3, 3, 1, 3, 1, -3, 3, 1, 3, 1, 1, 3, 3, -1, -1, -3, 1,
		-3, -1, 3, 1, 1, 3, -1, -1, 1, -3, 1, 3, -3, 1, -1, -3, -1, 3, 1, 3, 1, -1, -3, -3, -1, -1, -3, -3, -3, -1, -1,
		-3, 3, -1, -1, -1, -1, 1, 1, -3, 3, 1, 3, 3, 1, -1, 1, -3, 1, -3, 1, 1, -3, -1, 1, 3, -1, 3, 3, -1, -3, 1, -1,
		-3, 3, 3, 3, -1, 1, 1, 3, -1, -3, -1, 3, -1, -1, -1, 1, 1, 1, 1, 1, -1, 3, -1, -3, 1, 1, 3, -3, 1, -3, -1, 1, 1,
		-3, -3, 3, 1, 1, -3, 1, 3, 3, 1, -1, -3, 3, -1, 3, 3, 3, -3, 1, -1, 1, -1, -3, -1, 1, 3, -1, 3, -3, -3, -1, -3,
		3, -3, -3, -3, -1, -1, -3, -1, -3, 3, 1, 3, -3, -1, 3, -1, 1, -1, 3, -3, 1, -1, -3, -3, 1, 1, -1, 1, -1, 1, -1,
		3, 1, -3, -1, 1, -1, 1, -1, -1, 3, 3, -3, -1, 1, -3, -3, -1, -3, 3, 1, -1, -3, -1, -3, -3, 3, -3, 3, -3, -1, 1,
		3, 1, -3, 1, 3, 3, -1, -3, -1, -1, -1, -1, 3, 3, 3, 1, 3, 3, -3, 1, 3, -1, 3, -1, 3, 3, -3, 3, 1, -1, 3, 3, 1,
		-1, 3, 3, -1, -3, 3, -3, -1, -1, 3, -1, 3, -1, -1, 1, 1, 1, 1, -1, -1, -3, -1, 3, 1, -1, 1, -1, 3, -1, 3, 1, 1,
		-1, -1, -3, 1, 1, -3, 1, 3, -3, 1, 1, -3, -3, -1, -1, -3, -1, 1, 3, 1, 1, -3, -1, -1, -3, 3, -3, 3, 1, -3, 3,
		-3, 1, -1, 1, -3, 1, 1, 1, -1, -3, 3, 3, 1, 1, 3, -1, -3, -1, -1, -1, 3, 1, -3, -3, -1, 3, -3, -1, -3, -1, -3,
		-1, -1, -3, -1, -1, 1, -3, -1, -1, 1, -1, -3, 1, 1, -3, 1, -3, -3, 3, 1, 1, -1, 3, -1, -1, 1, 1, -1, -1, -3, -1,
		3, -1, 3, -1, 1, 3, 1, -1, 3, 1, 3, -3, -3, 1, -1, -1, 1, 3 };

uint32_t liblte_get_dft_by_nre(uint32_t nre)
{
	switch (nre)
	{
		case 12:
			return 0;
		case 24:
			return 1;
		case 36:
			return 2;
		case 48:
			return 3;
		case 60:
			return 4;
		case 72:
			return 5;
		case 96:
			return 6;
		case 108:
			return 7;
		case 120:
			return 8;
		case 144:
			return 9;
		case 180:
			return 10;
		case 192:
			return 11;
		case 216:
			return 12;
		case 240:
			return 13;
		case 288:
			return 14;
		case 300:
			return 15;
		case 324:
			return 16;
		case 360:
			return 17;
		case 384:
			return 18;
		case 432:
			return 19;
		case 480:
			return 20;
		case 540:
			return 21;
		case 576:
			return 22;
		case 600:
			return 23;
		case 648:
			return 24;
		case 720:
			return 25;
		case 864:
			return 26;
		case 900:
			return 27;
		case 960:
			return 28;
		case 972:
			return 29;
		case 1080:
			return 30;
		case 1152:
			return 31;
		case 1200:
			return 32;
		default:
			OS_ASSERT;
	}

	OS_ASSERT;

	return 0;
}

uint32_t liblte_get_dft_by_nrb(uint32_t nre)
{
	switch (nre)
	{
		case 1:
			return 0;
		case 2:
			return 1;
		case 3:
			return 2;
		case 4:
			return 3;
		case 5:
			return 4;
		case 6:
			return 5;
		case 8:
			return 6;
		case 9:
			return 7;
		case 10:
			return 8;
		case 12:
			return 9;
		case 15:
			return 10;
		case 16:
			return 11;
		case 18:
			return 12;
		case 20:
			return 13;
		case 24:
			return 14;
		case 25:
			return 15;
		case 27:
			return 16;
		case 30:
			return 17;
		case 32:
			return 18;
		case 36:
			return 19;
		case 40:
			return 20;
		case 45:
			return 21;
		case 48:
			return 22;
		case 50:
			return 23;
		case 54:
			return 24;
		case 60:
			return 25;
		case 72:
			return 26;
		case 75:
			return 27;
		case 80:
			return 28;
		case 81:
			return 29;
		case 90:
			return 30;
		case 96:
			return 31;
		case 100:
			return 32;
		default:
			OS_ASSERT;
	}

	OS_ASSERT;

	return 0;
}

/* Number of PUCCH demodulation reference symbols per slot N_rs_pucch Table 5.5.2.2.1-1 36.211 */
uint32_t liblte_refsignal_dmrs_N_rs(PUCCH_FMT_t format, int32_t cp)
{
	switch (format)
	{
		case pucch_format1:
		case pucch_format1a:
		case pucch_format1b:
			if (cp == LTE_CP_NORMAL)
				return 3;
			else
				return 2;
		case pucch_format2:
			if (cp == LTE_CP_NORMAL)
				return 2;
			else
				return 1;
		case pucch_format2a:
		case pucch_format2b:
			return 2;
		default:
			ERROR(DTRX, "Unsupported format %d\n", format);
			return 0;
	}
	return 0;
}

void liblte_refsignal_r_uv_arg_1prb(float *arg, uint32_t u)
{
	int i;
	for (i = 0; i < LTE_N_SC_RB; i++)
		arg[i] = UL_RS_5_5_1_2_1[u][i] * M_PI / 4;
}

uint32_t liblte_refsignal_dmrs_pucch_symbol(uint32_t m, PUCCH_FMT_t format, int32_t cp)
{
	switch (format)
	{
		case pucch_format1:
		case pucch_format1a:
		case pucch_format1b:
			if (cp == LTE_CP_NORMAL)
			{
				if (m < 3)
					return pucch_dmrs_symbol_format1_cpnorm[m];
			}
			else
			{
				if (m < 2)
					return pucch_dmrs_symbol_format1_cpext[m];
			}
			break;
		case pucch_format2:
			if (cp == LTE_CP_NORMAL)
			{
				if (m < 2)
					return pucch_dmrs_symbol_format2_cpnorm[m];
			}
			else
			{
				if (m < 1)
					return pucch_dmrs_symbol_format2_cpext[m];
			}
			break;
		case pucch_format2a:
		case pucch_format2b:
			if (m < 2)
				return pucch_dmrs_symbol_format2_cpnorm[m];
			break;
		default:
			ERROR(DTRX, "Unsupported format %d\n", format);
			return 0;
	}
	return 0;
}

/* Calculates alpha for format 1/a/b according to 5.5.2.2.2 (is_dmrs=true) or 5.4.1 (is_dmrs=false) of 36.211 */
float liblte_pucch_alpha_format1(lte_enodeb_t *enodeb, uint32_t n_pucch, int32_t cp, bool is_dmrs, uint32_t ns,
		uint32_t l, uint32_t *n_oc_ptr, uint32_t *n_prime_ns)
{
	uint32_t c = (cp == LTE_CP_NORMAL) ? 3 : 2;
	uint32_t N_prime =
			(n_pucch < c * enodeb->fapi_config.pucch_config.n_an_cs / enodeb->fapi_config.pucch_config.delta_pucch_shift) ?
					enodeb->fapi_config.pucch_config.n_an_cs : LTE_N_SC_RB;

	uint32_t n_prime = n_pucch;

	if (n_pucch >= c * enodeb->fapi_config.pucch_config.n_an_cs / enodeb->fapi_config.pucch_config.delta_pucch_shift)
		n_prime = (n_pucch
				- c * enodeb->fapi_config.pucch_config.n_an_cs / enodeb->fapi_config.pucch_config.delta_pucch_shift)
				% (c * LTE_N_SC_RB / enodeb->fapi_config.pucch_config.delta_pucch_shift);
	
	if (ns % 2)
	{
		if (n_pucch
				>= c * enodeb->fapi_config.pucch_config.n_an_cs / enodeb->fapi_config.pucch_config.delta_pucch_shift)
		{
			n_prime = (c * (n_prime + 1)) % (c * LTE_N_SC_RB / enodeb->fapi_config.pucch_config.delta_pucch_shift + 1)
					- 1;
		}
		else
		{
			uint32_t d = (cp == LTE_CP_NORMAL) ? 2 : 0;
			uint32_t h = (n_prime + d) % (c * N_prime / enodeb->fapi_config.pucch_config.delta_pucch_shift);
			n_prime = (h / c) + (h % c) * N_prime / enodeb->fapi_config.pucch_config.delta_pucch_shift;
		}
	}

	if (n_prime_ns)
		*n_prime_ns = n_prime;

	uint32_t n_oc_div = (!is_dmrs && (cp == LTE_CP_EXTENDED)) ? 2 : 1;

	uint32_t n_oc = n_prime * enodeb->fapi_config.pucch_config.delta_pucch_shift / N_prime;

	if (!is_dmrs && (cp == LTE_CP_EXTENDED))
		n_oc *= 2;
	
	if (n_oc_ptr)
		*n_oc_ptr = n_oc;

	uint32_t n_cs = 0;
	if ((cp == LTE_CP_NORMAL))
		n_cs = (enodeb->refsigs->n_cs_cell[ns][l]
				+ (n_prime * enodeb->fapi_config.pucch_config.delta_pucch_shift
						+ (n_oc % enodeb->fapi_config.pucch_config.delta_pucch_shift)) % N_prime) % LTE_N_SC_RB;
	else
		n_cs = (enodeb->refsigs->n_cs_cell[ns][l]
				+ (n_prime * enodeb->fapi_config.pucch_config.delta_pucch_shift + n_oc / n_oc_div) % N_prime)
				% LTE_N_SC_RB;
	
	return 2 * M_PI * (n_cs) / LTE_N_SC_RB;
}

/* Calculates alpha for format 2/a/b according to 5.4.2 of 36.211 */
float liblte_pucch_alpha_format2(lte_enodeb_t *enodeb, uint32_t n_pucch, uint32_t ns, uint32_t l)
{
	uint32_t n_prime = n_pucch % LTE_N_SC_RB;
	
	if (n_pucch >= LTE_N_SC_RB * enodeb->fapi_config.pucch_config.n_cqi_rb)
		n_prime = (n_pucch + enodeb->fapi_config.pucch_config.n_an_cs + 1) % LTE_N_SC_RB;

	if (ns % 2)
	{
		n_prime = (LTE_N_SC_RB * (n_prime + 1)) % (LTE_N_SC_RB + 1) - 1;
		if (n_pucch >= LTE_N_SC_RB * enodeb->fapi_config.pucch_config.n_cqi_rb)
		{
			int x = (LTE_N_SC_RB - 2 - (int) n_pucch) % LTE_N_SC_RB;
			if (x >= 0)
				n_prime = (uint32_t) x;
			else
				n_prime = LTE_N_SC_RB + x;
		}
	}
	uint32_t n_cs = (enodeb->refsigs->n_cs_cell[ns][l] + n_prime) % LTE_N_SC_RB;
	float alpha = 2 * M_PI * (n_cs) / LTE_N_SC_RB;

	return alpha;
}

/* Generates n_cs_cell according to Sec 5.4 of 36.211 */
void liblte_init_ncs_cell(lte_enodeb_t *enodeb)
{
	uint8_t ns, l, reset = 1, i, N_UL_symb;
	uint32_t x1, x2, j = 0, s = 0;

	N_UL_symb = (LTE_CP == LTE_CP_NORMAL) ? 7 : 6;
	x2 = enodeb->N_id_cell;

	for (ns = 0; ns < 20; ns++)
	{

		for (l = 0; l < N_UL_symb; l++)
		{
			enodeb->refsigs->n_cs_cell[ns][l] = 0;

			for (i = 0; i < 8; i++)
			{
				if ((j % 32) == 0)
				{
					s = lte_gold_generic(&x1, &x2, reset);
					reset = 0;
				}

				if (((s >> (j % 32)) & 1) == 1)
					enodeb->refsigs->n_cs_cell[ns][l] += (1 << i);

				j++;
			}
		}
	}
}

/* Generates DMRS for PUCCH according to 5.5.2.2 in 36.211 */
lte_status_t liblte_refsignal_dmrs_pucch_gen(lte_enodeb_t *enodeb, PUCCH_FMT_t format, uint32_t n_pucch,
		uint32_t sf_idx, uint8_t pucch_bits[2], Complex16 *r_pucch)
{
	lte_status_t ret = LTE_FAIL;
	float tmp_arg[12];

	if (r_pucch)
	{
		ret = LTE_FAIL;

		uint32_t N_rs = liblte_refsignal_dmrs_N_rs(format, LTE_CP);

		Complex16 z_m_1 = TO_COMPLEX16(32767, 0);

		if (format == pucch_format2a || format == pucch_format2b)
			ERROR(DTRX, "Format 2a/b not supported yet\n");

		for (uint32_t ns = 2 * sf_idx; ns < 2 * (sf_idx + 1); ns++)
		{
			// Get group hopping number u 
			uint32_t f_gh = 0;

			if (enodeb->fapi_config.pusch_config.hopping_mode == 1)
				f_gh = enodeb->refsigs->grouphop_pucch[ns];
			
			uint32_t u = (f_gh + (enodeb->N_id_cell % 30)) % 30;

			liblte_refsignal_r_uv_arg_1prb(tmp_arg, u);

			for (uint32_t m = 0; m < N_rs; m++)
			{
				uint32_t n_oc = 0;

				uint32_t l = liblte_refsignal_dmrs_pucch_symbol(m, format, LTE_CP);

				// Add cyclic prefix alpha
				float alpha = 0.0;
				if (format < pucch_format2)
					alpha = liblte_pucch_alpha_format1(enodeb, n_pucch, LTE_CP, 1, ns, l, &n_oc, NULL);
				else
					alpha = liblte_pucch_alpha_format2(enodeb, n_pucch, ns, l);

				// Choose number of symbols and orthogonal sequence from Tables 5.5.2.2.1-1 to -3 
				float *w = NULL;
				switch (format)
				{
					case pucch_format1:
					case pucch_format1a:
					case pucch_format1b:
						if (LTE_CP == LTE_CP_NORMAL)
							w = w_arg_pucch_format1_cpnorm[n_oc];
						else
							w = w_arg_pucch_format1_cpext[n_oc];
						break;
					case pucch_format2:
						if (LTE_CP == LTE_CP_NORMAL)
							w = w_arg_pucch_format2_cpnorm;
						else
							w = w_arg_pucch_format2_cpext;
						break;
					case pucch_format2a:
					case pucch_format2b:
						w = w_arg_pucch_format2_cpnorm;
						break;
					default:
						ERROR(DTRX, "Unsupported format %d\n", format);
						return LTE_FAIL;
				}

				Complex16 z_m = TO_COMPLEX16(32767, 0);

				if (m == 1)
					z_m = z_m_1;
				
				for (uint32_t n = 0; n < LTE_N_SC_RB; n++)
				{
					float img = (w[m] + tmp_arg[n] + alpha * n);
					float exp_re = floorf(1.0 * cosf(img) * 32760.0f);
					float exp_im = floorf(1.0 * sinf(img) * 32760.0f);
					
					r_pucch[(ns % 2) * LTE_N_SC_RB * N_rs + m * LTE_N_SC_RB + n] = TO_COMPLEX16((int16_t)exp_re, (int16_t)exp_im);					
				}
			}
		}
		ret = LTE_SUCCESS;
	}
	return ret;
}

lte_status_t liblte_generate_pucch_dmrs(lte_enodeb_t *enodeb, PUCCH_FMT_t format, int32_t sf_idx, int32_t n_pucch, Complex16 *drs)
{
	lte_status_t status = LTE_FAIL;

	int n_rs = liblte_refsignal_dmrs_N_rs(format, LTE_CP);
	if (!n_rs)
	{
		ERROR(DTRX, "Error computing N_rs\n");
		return LIBLTE_ERROR_PUCCH_DRS;
	}

	int nrefs_sf = LTE_N_SC_RB * n_rs * 2;

	/* Generate known pilots */
	uint8_t pucch2_bits[2] = { 0, 0 };

	if (format == pucch_format2a || format == pucch_format2b)
		ERROR(DTRX, "PUCCH DMRS format 2A/B not supported yet\n");
	else
		status = liblte_refsignal_dmrs_pucch_gen(enodeb, format, n_pucch, sf_idx, pucch2_bits, drs);

	return status;
}

static Complex16 liblte_uci_encode_format1()
{
	return TO_COMPLEX16(32767, 0);
}

static Complex16 liblte_uci_encode_format1a(uint8_t bit)
{
	return bit ? TO_COMPLEX16(-32767, 0) : TO_COMPLEX16(32767, 0);
}

static Complex16 liblte_uci_encode_format1b(uint8_t bits[2])
{
	if (bits[0] == 0)
	{
		if (bits[1] == 0)
			return TO_COMPLEX16(32767, 0); // 1.0
		else
			return TO_COMPLEX16(0, -32767); // -I
	}
	else
	{
		if (bits[1] == 0)
			return TO_COMPLEX16(0, 32767); // I 
		else
			return TO_COMPLEX16(-32767, 0); // -1.0 
	}
}

static int liblte_uci_mod_bits(lte_enodeb_t *enodeb, PUCCH_FMT_t format, uint8_t *bits, uint32_t sf_idx, uint16_t rnti,
		Complex16 *d)
{
	uint8_t tmp[2];

	switch (format)
	{
		case pucch_format1:
			d[0] = liblte_uci_encode_format1();
			break;
		case pucch_format1a:
			d[0] = liblte_uci_encode_format1a(bits[0]);
			break;
		case pucch_format1b:
			tmp[0] = bits[0];
			tmp[1] = bits[1];
			d[0] = liblte_uci_encode_format1b(tmp);
			break;
		case pucch_format2:
		case pucch_format2a:
		case pucch_format2b:
			ERROR(DTRX, "PUCCH formats 2x not supported yet\n");
			break;
		default:
			ERROR(DTRX, "PUCCH format %i not supported\n", format);
			return LTE_FAIL;
	}
	return LTE_SUCCESS;
}

uint32_t liblte_get_N_sf(PUCCH_FMT_t format, uint32_t slot_idx, bool shortened)
{
	switch (format)
	{
		case pucch_format1:
		case pucch_format1a:
		case pucch_format1b:
			if (!slot_idx)
				return 4;
			else
				return shortened ? 3 : 4;
		case pucch_format2:
		case pucch_format2a:
		case pucch_format2b:
			return 5;
		default:
			return 0;
	}
	return 0;
}

uint32_t liblte_get_pucch_symbol(uint32_t m, PUCCH_FMT_t format, int32_t cp)
{
	switch (format)
	{
		case pucch_format1:
		case pucch_format1a:
		case pucch_format1b:
			if (m < 4)
			{
				if (cp == LTE_CP_NORMAL)
					return pucch_symbol_format1_cpnorm[m];
				else
					return pucch_symbol_format1_cpext[m];
			}
			break;
		case pucch_format2:
		case pucch_format2a:
		case pucch_format2b:
			if (m < 5)
			{
				if (cp == LTE_CP_NORMAL)
					return pucch_symbol_format2_cpnorm[m];
				else
					return pucch_symbol_format2_cpext[m];
			}
			break;
		default:
			return 0;
	}
	return 0;
}

/* Choose PUCCH format based on pending transmission as described in 10.1 of 36.213 */
PUCCH_FMT_t liblte_pucch_get_format(int32_t ack_len, int32_t cqi_len, int32_t sr, int32_t cp)
{
	PUCCH_FMT_t format = pucch_format_error;

	// No CQI data
	if (cqi_len == 0)
	{
		// 1-bit ACK + optional SR
		if (ack_len == 1)
			format = pucch_format1a;
		// 2-bit ACK + optional SR
		else if (ack_len == 2)
			format = pucch_format1a;
		// SR only 
		else if (sr)
			format = pucch_format1;
	}
	// CQI data
	else
	{
		// CQI and no ack
		if (ack_len == 0)
			format = pucch_format2;
		// CQI + 1-bit ACK
		else if (ack_len == 1 && (cp == LTE_CP_NORMAL))
			format = pucch_format2a;
		// CQI + 2-bit ACK 
		else if (ack_len == 2)
			format = pucch_format2b;
		// CQI + 2-bit ACK + cyclic prefix 
		else if (ack_len == 1 && (cp == LTE_CP_EXTENDED))
			format = pucch_format2b;
	}
	return format;
}

static int liblte_pucch_encode(lte_enodeb_t* enodeb, PUCCH_FMT_t format, int32_t sf_idx, int32_t n_pucch, uint16_t rnti,
		uint8_t *bits, Complex16 *z, bool shortened, bool signal_only)
{
	Complex16 d[64];
	float tmp_arg[12];
	float gain = floor((32767.0f / 4.0f) / (float)((liblte_get_N_sf(format, 0, shortened) + liblte_get_N_sf(format, 1, shortened)) * LTE_N_SC_RB));

	if (!signal_only)
	{
		if (liblte_uci_mod_bits(enodeb, format, bits, sf_idx, rnti, d) != LTE_SUCCESS)
		{
			ERROR(DTRX, "Error encoding PUCCH bits\n");
			return LTE_FAIL;
		}
	}
	else
	{
		for (int i = 0; i < 64 / 2; i++)
			d[i] = TO_COMPLEX16(32767, 0);
	}

	uint32_t N_sf_0 = liblte_get_N_sf(format, 0, shortened);
	
	for (uint32_t ns = 2 * sf_idx; ns < 2 * (sf_idx + 1); ns++)
	{
		uint32_t N_sf = liblte_get_N_sf(format, ns % 2, shortened);

		// Get group hopping number u 
		uint32_t f_gh = 0;
		if (enodeb->fapi_config.pusch_config.hopping_mode == 1)
			f_gh = enodeb->refsigs->grouphop_pucch[ns];
		
		uint32_t u = (f_gh + (enodeb->N_id_cell % 30)) % 30;

		liblte_refsignal_r_uv_arg_1prb(tmp_arg, u);
		uint32_t N_sf_widx = N_sf == 3 ? 1 : 0;
		for (uint32_t m = 0; m < N_sf; m++)
		{
			uint32_t l = liblte_get_pucch_symbol(m, format, LTE_CP);
			float alpha = 0;
			if (format >= pucch_format2)
			{
				/* Усиление = 32767 * (LTE_N_SC_RB * N_CQI_BITS) */
				gain = floor(32767.0f / (12.0f * 20.0f));
				alpha = liblte_pucch_alpha_format2(enodeb, n_pucch, ns, l);
				
				for (uint32_t n = 0; n < LTE_N_SC_RB; n++)
				{
					float img = (tmp_arg[n] + alpha * n);
					float exp_re = (1.0 * cosf(img) * gain);
					float exp_im = (1.0 * sinf(img) * gain);
					Complex16 expc = TO_COMPLEX16((int16_t) exp_re, (int16_t) exp_im);

					Complex32 t;
					t = C_L_mpy(d[(ns % 2) * N_sf + m], expc);
					z[(ns % 2) * N_sf * LTE_N_SC_RB + m * LTE_N_SC_RB + n] = 
							V_pack_2fr(creal32(t), cimag32(t));
				}
			}
			else
			{
				uint32_t n_prime_ns = 0;
				uint32_t n_oc = 0;
				alpha = liblte_pucch_alpha_format1(enodeb, n_pucch, LTE_CP, 1, ns, l, &n_oc, &n_prime_ns);
				float S_ns = 0;
				if (n_prime_ns % 2)
				{
					S_ns = M_PI / 2;
				}
				
				for (uint32_t n = 0; n < LTE_N_SC_RB; n++)
				{
					float img = (w_n_oc[N_sf_widx][n_oc % 3][m] + tmp_arg[n] + alpha * n + S_ns);
					float exp_re =(1.0 * cosf(img) * gain);
					float exp_im = (1.0 * sinf(img) * gain);
					Complex16 expc = TO_COMPLEX16((int16_t) exp_re, (int16_t) exp_im);

					Complex32 t;
					t = C_L_mpy(d[0], expc);

					z[(ns % 2) * N_sf_0 * LTE_N_SC_RB + m * LTE_N_SC_RB + n] = 
							V_pack_2fr(creal32(t), cimag32(t));
				}
			}
		}
	}
	return LTE_SUCCESS;
}

// Compute m according to Section 5.4.3 of 36.211
static uint32_t liblte_pucch_m(lte_enodeb_t *enodeb, PUCCH_FMT_t format, uint32_t n_pucch, int32_t cp)
{
	uint32_t m = 0;
	switch (format)
	{
		case pucch_format1:
		case pucch_format1a:
		case pucch_format1b:
			m = enodeb->fapi_config.pucch_config.n_cqi_rb;

			uint32_t c = (cp == LTE_CP_NORMAL) ? 3 : 2;

			if (n_pucch
					>= c * enodeb->fapi_config.pucch_config.n_an_cs / enodeb->fapi_config.pucch_config.delta_pucch_shift)
			{
				m = (n_pucch
						- c * enodeb->fapi_config.pucch_config.n_an_cs
								/ enodeb->fapi_config.pucch_config.delta_pucch_shift)
						/ (c * LTE_N_SC_RB / enodeb->fapi_config.pucch_config.delta_pucch_shift)
						+ enodeb->fapi_config.pucch_config.n_cqi_rb
						+ ((enodeb->fapi_config.pucch_config.n_an_cs + 7) >> 3);
			}
			break;
		case pucch_format2:
		case pucch_format2a:
		case pucch_format2b:
			m = n_pucch / LTE_N_SC_RB;
			break;
		default:
			m = 0;
			break;
	}
	return m;
}

static uint32_t liblte_pucch_n_prb(lte_enodeb_t *enodeb, PUCCH_FMT_t format, uint32_t n_pucch,
		uint32_t nof_prb, int32_t cp, uint32_t ns)
{
	uint32_t m = liblte_pucch_m(enodeb, format, n_pucch, cp);
	
	// Determine n_prb 
	uint32_t n_prb = m / 2;
	if ((m + ns) % 2)
	{
		n_prb = nof_prb - 1 - m / 2;
	}
	return n_prb;
}

/* Gets PUCCH DMRS from the physical resources as defined in 5.5.2.2.2 in 36.211 */
lte_status_t liblte_refsignal_dmrs_pucch_get(lte_enodeb_t *enodeb, int32_t ant_no, PUCCH_FMT_t format, uint32_t n_pucch,
		lte_ul_subframe_t *rx_sf, int8_t *scale_vector, Complex16 *dest, uint32_t *pucch_n_dmrs_syms)
{
	lte_status_t ret = LTE_FAIL;
	int32_t re;
	int8_t scale = 0;
	Complex16 *dest_ptr = dest;

	if (LTE_CP == LTE_CP_EXTENDED)
		return ret;

	uint32_t nsymbols = LTE_NSYMB_PER_SUBFRAME >> 1; // Nsymbols per slot
	uint32_t N_rs = liblte_refsignal_dmrs_N_rs(format, LTE_CP);

	*pucch_n_dmrs_syms = N_rs << 1;

	for (uint32_t ns = 0; ns < 2; ns++)
	{
		// Determine n_prb
		uint32_t n_prb = liblte_pucch_n_prb(enodeb, format, n_pucch, enodeb->fapi_config.rf_config.ul_channel_bandwidth,
				LTE_CP, ns);

		for (uint32_t i = 0; i < N_rs; i++)
		{
			uint32_t l = liblte_refsignal_dmrs_pucch_symbol(i, format, LTE_CP);

			// N_rs - номер референсного символа символа
			// l - номер референсного символа в слоте
			// n_prb - смещение PRB PUCCH
			if (LTE_CP == LTE_CP_NORMAL)
			{
				Complex16 *src_sym;
				if(l == 3)
				{
					if(ns == 0)
					{
						//src_sym = &rx_sf->pufft_buf_a0_sym3[ant_no * enodeb->fp.LTE_N_RE_UL];
						src_sym = &rx_sf->a0_sym3_aligned[ant_no * enodeb->fp.LTE_N_RE_UL];
						scale = 0;//scale_vector[3];
					}
					else
					{
						//src_sym = &rx_sf->pufft_buf_a0_sym10[ant_no * enodeb->fp.LTE_N_RE_UL];
						src_sym = &rx_sf->a0_sym10_aligned[ant_no * enodeb->fp.LTE_N_RE_UL];
						scale = 0;//scale_vector[10];
					}
				}
				else
				{
					//src_sym = &rx_sf->pufft_buf_a0[ant_no * enodeb->fp.LTE_N_RE_UL + enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * l2pufft_table_normcp[ns][l] * enodeb->fp.LTE_N_RE_UL];
					src_sym = &rx_sf->a0_aligned[ant_no * enodeb->fp.LTE_N_RE_UL + enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * l2pufft_table_normcp[ns][l] * enodeb->fp.LTE_N_RE_UL];
					scale = 0; //scale_vector[((ns == 1) ? 7 : 0) + l];
				}

				for(re = 0; re < LTE_N_SC_RB; re++)
				{
#pragma loop_unroll 12
					dest_ptr[re] =	V_asrr2(src_sym[n_prb * LTE_N_SC_RB + re], scale);
				}
				
				dest_ptr += LTE_N_SC_RB;

			}
			else
			{
				ERROR(DTRX, "PUCCH extract for ext CP not implemented!\n");
			}
		}
	}

	ret = LTE_SUCCESS;

	return ret;
}

void liblte_calc_pucch_scale_vector(lte_ul_subframe_t *rx_sf, int8_t *scale_vector)
{
	int32_t i;
	int8_t max_scale = -127;
	
	for(i=0; i<LTE_NSYMB_PER_SUBFRAME; i++)
	{
		//scale_vector[i] = (int8_t)rx_sf->pufft_job[i].status[0];
		scale_vector[i] = (int8_t)rx_sf->a0_scales_aligned[i];
	}
		
	for(i=0; i<LTE_NSYMB_PER_SUBFRAME; i++)
	{
		if(scale_vector[i] > max_scale)
			max_scale = scale_vector[i];
	}
	
	for(i=0; i<LTE_NSYMB_PER_SUBFRAME; i++)
	{
		scale_vector[i] = scale_vector[i] - max_scale;
	}
}

/* Get PUCCH symbols to physical resources according to 5.4.3 in 36.211 */
lte_status_t liblte_pucch_get(lte_enodeb_t *enodeb, int32_t ant_no, PUCCH_FMT_t format, uint32_t n_pucch, lte_ul_subframe_t *rx_sf,
		int8_t *scale_vector, bool shortened, int32_t shift_right, Complex16 *dest, uint32_t *pucch_n_syms)
{
//#pragma opt_level = "O0"
	lte_status_t ret = LTE_FAIL;
	int32_t re;
	Complex16 * restrict dest_ptr = dest;

	uint32_t nsymbols = LTE_NSYMB_PER_SUBFRAME >> 1; // Nsymbols per slot

	uint32_t n_syms = 0;
	uint32_t N_sf_0 = liblte_get_N_sf(format, 0, shortened);
	
	for (uint32_t ns = 0; ns < 2; ns++)
	{
		uint32_t N_sf = liblte_get_N_sf(format, ns % 2, shortened);

		// Determine n_prb
		uint32_t n_prb = liblte_pucch_n_prb(enodeb, format, n_pucch, enodeb->fapi_config.rf_config.ul_channel_bandwidth, LTE_CP, ns);
		
		if (n_prb < enodeb->fapi_config.rf_config.ul_channel_bandwidth)
		{
			for (uint32_t i = 0; i < N_sf; i++)
			{
				uint32_t l = liblte_get_pucch_symbol(i, format, LTE_CP);
				
				// N_rs - номер референсного символа символа
				// l - номер референсного символа в слоте
				// n_prb - смещение PRB PUCCH
				if (LTE_CP == LTE_CP_NORMAL)
				{
					Complex16 * restrict src_sym;
					
					if(l == 3)
					{
						if(ns == 0)
						{
							//src_sym = &rx_sf->pufft_buf_a0_sym3[ant_no * enodeb->fp.LTE_N_RE_UL];
							src_sym = &rx_sf->a0_sym3_aligned[ant_no * enodeb->fp.LTE_N_RE_UL];
							//scale = scale_vector[3];
						}
						else
						{
							src_sym = &rx_sf->a0_sym10_aligned[ant_no * enodeb->fp.LTE_N_RE_UL];
							//scale = scale_vector[10];
						}
					}
					else
					{
						//src_sym = &rx_sf->pufft_buf_a0[ant_no * enodeb->fp.LTE_N_RE_UL + enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * l2pufft_table_normcp[ns][l] * enodeb->fp.LTE_N_RE_UL];
						src_sym = &rx_sf->a0_aligned[ant_no * enodeb->fp.LTE_N_RE_UL + enodeb->fp.LTE_N_PHYS_ANTENNAS_RX * l2pufft_table_normcp[ns][l] * enodeb->fp.LTE_N_RE_UL];
						//scale = scale_vector[((ns == 1) ? 7 : 0) + l];
					}

					for(re = 0; re < LTE_N_SC_RB; re++)
					{
#pragma loop_unroll 12
						dest_ptr[re] = V_asrr2(src_sym[n_prb * LTE_N_SC_RB + re], shift_right);
					}
					dest_ptr += LTE_N_SC_RB;

				}
				else
				{
					ERROR(DTRX, "PUCCH extract for ext CP not implemented!\n");
				}
				
				n_syms++;
			}
		}
		else
		{
			return LTE_FAIL;
		}
	}
	*pucch_n_syms = n_syms;

	return LTE_SUCCESS;
}

lte_status_t liblte_generate_pucch_sr(lte_enodeb_t *enodeb, int32_t sf_idx, int32_t n_pucch, Complex16 *sr_nosrs, Complex16 *sr_srs)
{
	uint8_t bits[2] =
		{ 0, 0 };

	liblte_pucch_encode(enodeb, pucch_format1, sf_idx, n_pucch, 0, bits, sr_nosrs, FALSE, FALSE);
	liblte_pucch_encode(enodeb, pucch_format1, sf_idx, n_pucch, 0, bits, sr_srs, TRUE, FALSE);

	return LTE_SUCCESS;
}

lte_status_t liblte_generate_pucch_ack(lte_enodeb_t *enodeb, int32_t sf_idx, int32_t n_pucch, Complex16 *ack_nosrs, Complex16 *ack_srs)
{
	uint8_t bits[2] = { 1, 0 };

	liblte_pucch_encode(enodeb, pucch_format1a, sf_idx, n_pucch, 0, bits, ack_nosrs, FALSE, FALSE);
	liblte_pucch_encode(enodeb, pucch_format1a, sf_idx, n_pucch, 0, bits, ack_srs, TRUE, FALSE);
	
	return LTE_SUCCESS;
}

lte_status_t liblte_generate_pucch_nack(lte_enodeb_t *enodeb, int32_t sf_idx, int32_t n_pucch, Complex16 *nack_nosrs, Complex16 *nack_srs)
{
	uint8_t bits[2] = { 0, 0 };

	liblte_pucch_encode(enodeb, pucch_format1a, sf_idx, n_pucch, 0, bits, nack_nosrs, FALSE, FALSE);
	liblte_pucch_encode(enodeb, pucch_format1a, sf_idx, n_pucch, 0, bits, nack_srs, TRUE, FALSE);

	return LTE_SUCCESS;
}

lte_status_t liblte_generate_pucch_fmt2(lte_enodeb_t *enodeb, int32_t sf_idx, int32_t n_pucch, Complex16 *fmt2_nosrs, Complex16 *fmt2_srs)
{

	liblte_pucch_encode(enodeb, pucch_format2, sf_idx, n_pucch, 0, NULL, fmt2_nosrs, FALSE, TRUE);
	liblte_pucch_encode(enodeb, pucch_format2, sf_idx, n_pucch, 0, NULL, fmt2_srs, TRUE, TRUE);

	return LTE_SUCCESS;
}


void liblte_generate_grouphop(lte_enodeb_t *enodeb)
{

	uint8_t ns;
	uint8_t reset = 1;
	uint32_t x1, x2, s = 0;
	uint32_t fss_pusch = enodeb->N_id_cell + enodeb->fapi_config.uplink_reference_signal_config.group_assignment;
	uint32_t fss_pucch = enodeb->N_id_cell;

	x2 = enodeb->N_id_cell / 30;

	for (ns = 0; ns < 20; ns++)
	{
		if (enodeb->fapi_config.uplink_reference_signal_config.uplink_rs_hopping != 1)
		{
			/* Group hopping disabled */
			enodeb->refsigs->grouphop_pusch[ns] = fss_pusch % 30;
			enodeb->refsigs->grouphop_pucch[ns] = fss_pucch % 30;
		}
		else
		{
			/* Group hopping enabled */
			if ((ns & 3) == 0)
			{
				s = lte_gold_generic(&x1, &x2, reset);
				reset = 0;
			}

			enodeb->refsigs->grouphop_pusch[ns] = (((uint8_t*) &s)[ns & 3] + fss_pusch) % 30;
			enodeb->refsigs->grouphop_pucch[ns] = (((uint8_t*) &s)[ns & 3] + fss_pucch) % 30;
		}
	}
}

void liblte_generate_seqhop(lte_enodeb_t *enodeb)
{

	uint8_t ns, reset = 1;
	uint32_t x1, x2, s = 0;
	uint32_t fss_pusch = enodeb->N_id_cell + enodeb->fapi_config.uplink_reference_signal_config.group_assignment;

	x2 = (32 * (enodeb->N_id_cell / 30) + fss_pusch) % 30;

	s = lte_gold_generic(&x1, &x2, reset);

	for (ns = 0; ns < 20; ns++)
	{
		if (enodeb->fapi_config.uplink_reference_signal_config.uplink_rs_hopping == 2)
			enodeb->refsigs->seqhop_pusch[ns] = (s >> (ns & 0x1f)) & 1;
		else
			enodeb->refsigs->seqhop_pusch[ns] = 0;
	}
}

void liblte_generate_nPRS(lte_enodeb_t *enodeb)
{

	uint16_t n = 0;
	uint8_t reset = 1;
	uint32_t x1, x2, s = 0;
	uint8_t Nsymb_UL = (enodeb->fapi_config.subframe_config.ul_cyclic_prefix_type == FAPI_CP_NORMAL) ? 7 : 6;
	uint16_t next = 0;
	uint8_t ns = 0;

	uint32_t fss_pucch = (enodeb->N_id_cell) % 30;
	uint32_t fss_pusch = (fss_pucch + enodeb->fapi_config.uplink_reference_signal_config.group_assignment) % 30;

	x2 = (32 * (uint32_t) (enodeb->N_id_cell / 30)) + fss_pusch;

	for (n = 0; n < (20 * Nsymb_UL); n++)
	{
		if ((n & 3) == 0)
		{
			s = lte_gold_generic(&x1, &x2, reset);
			reset = 0;
		}

		if (n == next)
		{
			enodeb->refsigs->nPRS[ns] = ((uint8_t*) &s)[next & 3];
			ns++;
			next += Nsymb_UL;
		}
	}
}

int32_t liblte_is_dl_subframe(lte_enodeb_t *enodeb, int32_t subframe)
{
	/* По-умолчанию считаем, что передача Downlink разрешена */
	int32_t res = 1;

	if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_TDD)
	{
		switch(enodeb->fapi_config.tdd_frame_structure_config.subframe_assignment)
		{
			case 0:
				if(subframe == 0 || subframe == 5)
					res = 1;
				else 
					res = 0;
				break;
			case 1:
				if(subframe == 0 || subframe == 4 || subframe == 5 || subframe == 9)
					res = 1;
				else 
					res = 0;
				break;
			case 2:
				if(subframe == 0 || subframe == 3 || subframe == 4 || subframe == 5 || subframe == 8 || subframe == 9)
					res = 1;
				else
					res = 0;
				break;
			case 3:
				if(subframe == 0 || subframe == 5 || subframe == 6 || subframe == 7 || subframe == 8 || subframe == 9)
					res = 1;
				else
					res = 0;
				break;
			case 4:
				if(subframe == 0 || subframe == 4 || subframe == 5 || subframe == 6 || subframe == 7 || subframe == 8 || subframe == 9)
					res = 1;
				else
					res = 0;
				break;
			case 5:
				if(subframe == 0 || subframe == 3 || subframe == 4 || subframe == 5 || subframe == 6 || subframe == 7 || subframe == 8 || subframe == 9)
					res = 1;
				else
					res = 0;
				break;
			case 6:
				if(subframe == 0 || subframe == 5 || subframe == 9)
					res = 1;
				else
					res = 0;
				break;
		
			default:
				res = 1;
				break;
		}
	}
	
	return res;
}

int32_t liblte_is_spec_subframe(lte_enodeb_t *enodeb, int32_t subframe)
{
	/* По-умолчанию считаем, что передача Downlink разрешена */
	int32_t res = 0;

	if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_TDD)
	{
		switch(enodeb->fapi_config.tdd_frame_structure_config.subframe_assignment)
		{
			case 0:
			case 1:
			case 2:
			case 6:
				if(subframe == 1 || subframe == 6)
					res = 1;
				else 
					res = 0;
				break;
				
			case 3:
			case 4:
			case 5:
				if(subframe == 1)
					res = 1;
				else
					res = 0;
				break;
		
			default:
				res = 0;
				break;
		}
	}
	
	return res;
}

int32_t liblte_is_ul_subframe(lte_enodeb_t *enodeb, int32_t subframe)
{
	/* По-умолчанию считаем, что передача Uplink разрешена */
	int32_t res = 1;

	if(enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_TDD)
	{
		switch(enodeb->fapi_config.tdd_frame_structure_config.subframe_assignment)
		{
			case 0:
				if(subframe == 2 || subframe == 3 || subframe == 4 || subframe == 7 || subframe == 8 || subframe == 9)
					res = 1;
				else 
					res = 0;
				break;
				
			case 1:
				if(subframe == 2 || subframe == 3 || subframe == 7 || subframe == 8)
					res = 1;
				else 
					res = 0;
				break;
				
			case 2:
				if(subframe == 2 || subframe == 7)
					res = 1;
				else
					res = 0;
				break;
				
			case 3:
				if(subframe == 2 || subframe == 3 || subframe == 4)
					res = 1;
				else
					res = 0;
				break;
				
			case 4:
				if(subframe == 2 || subframe == 3)
					res = 1;
				else
					res = 0;
				break;
				
			case 5:
				if(subframe == 2)
					res = 1;
				else
					res = 0;
				break;
				
			case 6:
				if(subframe == 2 || subframe == 3 || subframe == 4 || subframe == 7 || subframe == 8)
					res = 1;
				else
					res = 0;
				break;
		
			default:
				res = 1;
				break;
		}
	}
	
	return res;
}

int32_t liblte_tdd_num_syms_in_dl_subframe(lte_enodeb_t *enodeb, int32_t subframe)
{
	/* По-умолчанию 0 символов */
	int32_t res = 0;
	
	if(liblte_is_dl_subframe(enodeb, subframe))
	{
		res = 14;
	}
	else if(liblte_is_spec_subframe(enodeb, subframe))
	{
		switch(enodeb->fapi_config.tdd_frame_structure_config.special_subframe_patterns)
		{
			case 0:
			case 5:
				res = 3;
				break;
			case 1:
			case 6:
				res = 9;
				break;
			case 2:
			case 7:
				res = 10;
				break;
			case 3:
			case 8:
				res = 11;
				break;
			case 4:
				res = 12;
				break;
				
			default:
				res = 0;
				break;
		}
	}
	
	return res;
}
