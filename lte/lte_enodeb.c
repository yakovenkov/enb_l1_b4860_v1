/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <smartdsp_os.h>
#include <app_config.h>
#include <lte_enodeb.h>
#include <liblte_msc8157.h>
#include <log.h>
#include <complex.h>
#include <math.h>
#include <string.h>
#include <trans.h>
#include <trx_lte.h>

#include <secure_id.h>

//#define USE_PUFFT_OVA_SCL
// SC3900 workaround
#define creal16(x) extract_h(x)
#define cimag16(x) extract_l(x)

#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_OPT_OFF
#pragma opt_level = "O3"
#endif

lte_enodeb_t g_enodeb_inst[NUM_ENODEB] __attribute__((section(".local_data_ddr0_cacheable_bss")));

lte_enodeb_refsigs_t lte_enodeb_refsigs_inst __attribute__((section(".shared_data_ddr0_cacheable_bss")));
#pragma align lte_enodeb_refsigs_inst ARCH_CACHE_LINE_SIZE

/* Количество секторов */
uint32_t LTE_N_SECTORS;
/* Количество антенн в секторе*/
//uint32_t LTE_N_ANTENNAS[MAPLE_NUM_SECTORS];
#define NCS_1024_839_COEFF ((1024 * 65536) / 839)
#define NCS_839_1024_COEFF ((839 * 65536) / 1024)
#define LTE_PRACH_GUARD_SAMPLES 3

///uint32_t LTE_SYMBOL_LEN;
///uint32_t LTE_SAMPLERATE;
///uint32_t LTE_SAMPLES_PER_SUBFRAME;
///uint32_t LTE_PRACH_SEQ_LEN;
///uint32_t LTE_PRACH_SEQ_CP;
//uint32_t LTE_PRACH_Ncs_125x;
///int32_t LTE_PRACH_Ncs;
///uint32_t LTE_PRACH_Nx;
//uint32_t LTE_PRACH_DETECT_MAX;
///uint32_t LTE_PRACH_SEQ_CP_LEN;
///uint32_t LTE_N_RB_DL;
///uint32_t LTE_RB_TYPE0_BITMASK_LENGTH;
///uint32_t LTE_RBG_SIZE;
///uint32_t LTE_N_RB_UL;
///uint32_t LTE_N_RE;
///uint32_t LTE_N_RE_UL;
///uint32_t LTE_FIRST_SS_CARRIER;
///int32_t LTE_SR_THRESHOLD;
///uint32_t LTE_CP0_LEN;
///uint32_t LTE_CPx_LEN;
///uint32_t LTE_N_SC_DL_HALF;

#include "git_version.h"
extern const char *l1_version_full;
extern cpri_global_init_params_t *cpri_global_params;

#define N_SUPPORTED_UL_NRB	33
uint32_t supported_ul_nrb[N_SUPPORTED_UL_NRB] =
	{ 1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 15, 16, 18, 20, 24, 25, 27, 30, 32, 36, 40, 45, 48, 50, 54, 60, 64, 72, 75, 80, 81, 90, 96 };

#define SQRT_TABLE_SIZE 65536
int32_t sqrt_table[65536] __attribute__((section(".shared_data_ddr0_cacheable_bss"), aligned(16)));

/* Таблица DDS для смещения 7.5kHz
 * Расположена в DDR для доступа из MAPLE-B3
 */
static Complex16 ul_shift_dds_ddr[2048] __attribute__((section(".shared_data_ddr0_cacheable_bss"), aligned(ARCH_CACHE_LINE_SIZE)));

os_status lte_enodeb_init_trxs(lte_enodeb_t *enodeb)
{
	uint32_t i;

	OS_ASSERT_COND(enodeb != NULL);

	lte_trx_init(enodeb);

	return OS_SUCCESS;
}

/* c(n) = x1(n+Nc) + x2(n+Nc) mod 2
 x1(n+31) = (x1(n+3)+x1(n))mod 2
 x2(n+31) = (x2(n+3)+x2(n+2)+x2(n+1)+x2(n))mod 2
 x1(0)=1,x1(n)=0,n=1...30
 x2 <=> cinit = sum_{i=0}^{30} x2(i)2^i

 equivalent
 x1(n) = x1(n-28) + x1(n-31)
 x2(n) = x2(n-28) + x2(n-29) + x2(n-30) + x2(n-31)
 x1(0)=1,x1(1)=0,...x1(30)=0,x1(31)=1
 x2 <=> cinit, x2(31) = x2(3)+x2(2)+x2(1)+x2(0)

 N_RB^{max,DL}=110
 c_init = 2^1 * (7(n_s + 1) + l + 1)*(2N_{ID}^{cell} + 1) + 2*N_{ID}^{cell} + N_CP
 N_{ID}^{cell = 0..503
 */

//uint32_t lte_gold_table[3][20][2][14];  // need 55 bytes for sequence
// slot index x pilot within slot x sequence
static void lte_init_gold_tables(lte_enodeb_t *enodeb, uint32_t fp_Ncp)
//,uint32_t lte_gold_table[20][2][14],uint16_t Nid_cell) {
{

	uint8_t ns, l, Ncp = 1 - fp_Ncp;
	uint32_t n, x1, x2; //,x1tmp,x2tmp;
	uint32_t N_id_cell;
	N_id_cell = enodeb->N_id_cell;

	for (ns = 0; ns < 20; ns++)
	{

		for (l = 0; l < 2; l++)
		{

			x2 = Ncp + (N_id_cell << 1) + (((1 + (N_id_cell << 1)) * (1 + (((fp_Ncp == 0) ? 4 : 3) * l) + (7 * (1 + ns)))) << 10); //cinit
			//x2 = frame_parms->Ncp + (Nid_cell<<1) + (1+(Nid_cell<<1))*(1 + (3*l) + (7*(1+ns))); //cinit
			//n = 0
			//      printf("cinit (ns %d, l %d) => %d\n",ns,l,x2);
			x1 = 1 + (1 << 31);
			x2 = x2 ^ ((x2 ^ (x2 >> 1) ^ (x2 >> 2) ^ (x2 >> 3)) << 31);
			// skip first 50 double words (1600 bits)
			//printf("n=0 : x1 %x, x2 %x\n",x1,x2);
			for (n = 1; n < 50; n++)
			{
				x1 = (x1 >> 1) ^ (x1 >> 4);
				x1 = x1 ^ (x1 << 31) ^ (x1 << 28);
				x2 = (x2 >> 1) ^ (x2 >> 2) ^ (x2 >> 3) ^ (x2 >> 4);
				x2 = x2 ^ (x2 << 31) ^ (x2 << 30) ^ (x2 << 29) ^ (x2 << 28);
				//	printf("x1 : %x, x2 : %x\n",x1,x2);
			}
			for (n = 0; n < 14; n++)
			{
				x1 = (x1 >> 1) ^ (x1 >> 4);
				x1 = x1 ^ (x1 << 31) ^ (x1 << 28);
				x2 = (x2 >> 1) ^ (x2 >> 2) ^ (x2 >> 3) ^ (x2 >> 4);
				x2 = x2 ^ (x2 << 31) ^ (x2 << 30) ^ (x2 << 29) ^ (x2 << 28);

				enodeb->refsigs->lte_gold_table[ns][l][n] = x1 ^ x2;
				//	printf("n=%d : c %x\n",n,x1^x2);	
			}

		}

	}
}

/* 
 * Инициализация таблиц для детектора PRACH
 */
#include "twiddle839.h"

int32_t table_5_7_2_2_urs[]  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16))) =
	{ 0, 13, 15, 18, 22, 26, 32, 38, 46, 59, 76, 93, 119, 167, 279, 419 };
int32_t table_5_7_2_2_rs[]  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16)))=
	{ 15, 18, 22, 26, 32, 38, 46, 55, 68, 82, 100, 128, 158, 202, 237 };
int32_t table_5_7_2_3[]  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16)))=
	{ 2, 4, 6, 8, 10, 12, 15 };
int32_t table_5_7_2_4[]  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16)))=
	{ 129, 710, 140, 699, 120, 719, 210, 629, 168, 671, 84, 755, 105, 734, 93, 746, 70, 769, 60, 779, 2, 837, 1, 838, 56, 783, 112, 727, 148, 691, 80, 759, 42,
		797, 40, 799, 35, 804, 73, 766, 146, 693, 31, 808, 28, 811, 30, 809, 27, 812, 29, 810, 24, 815, 48, 791, 68, 771, 74, 765, 178, 661, 136, 703, 86, 753,
		78, 761, 43, 796, 39, 800, 20, 819, 21, 818, 95, 744, 202, 637, 190, 649, 181, 658, 137, 702, 125, 714, 151, 688, 217, 622, 128, 711, 142, 697, 122,
		717, 203, 636, 118, 721, 110, 729, 89, 750, 103, 736, 61, 778, 55, 784, 15, 824, 14, 825, 12, 827, 23, 816, 34, 805, 37, 802, 46, 793, 207, 632, 179,
		660, 145, 694, 130, 709, 223, 616, 228, 611, 227, 612, 132, 707, 133, 706, 143, 696, 135, 704, 161, 678, 201, 638, 173, 666, 106, 733, 83, 756, 91, 748,
		66, 773, 53, 786, 10, 829, 9, 830, 7, 832, 8, 831, 16, 823, 47, 792, 64, 775, 57, 782, 104, 735, 101, 738, 108, 731, 208, 631, 184, 655, 197, 642, 191,
		648, 121, 718, 141, 698, 149, 690, 216, 623, 218, 621, 152, 687, 144, 695, 134, 705, 138, 701, 199, 640, 162, 677, 176, 663, 119, 720, 158, 681, 164,
		675, 174, 665, 171, 668, 170, 669, 87, 752, 169, 670, 88, 751, 107, 732, 81, 758, 82, 757, 100, 739, 98, 741, 71, 768, 59, 780, 65, 774, 50, 789, 49,
		790, 26, 813, 17, 822, 13, 826, 6, 833, 5, 834, 33, 806, 51, 788, 75, 764, 99, 740, 96, 743, 97, 742, 166, 673, 172, 667, 175, 664, 187, 652, 163, 676,
		185, 654, 200, 639, 114, 725, 189, 650, 115, 724, 194, 645, 195, 644, 192, 647, 182, 657, 157, 682, 156, 683, 211, 628, 154, 685, 123, 716, 139, 700,
		212, 627, 153, 686, 213, 626, 215, 624, 150, 689, 225, 614, 224, 615, 221, 618, 220, 619, 127, 712, 147, 692, 124, 715, 193, 646, 205, 634, 206, 633,
		116, 723, 160, 679, 186, 653, 167, 672, 79, 760, 85, 754, 77, 762, 92, 747, 58, 781, 62, 777, 69, 770, 54, 785, 36, 803, 32, 807, 25, 814, 18, 821, 11,
		828, 4, 835, 3, 836, 19, 820, 22, 817, 41, 798, 38, 801, 44, 795, 52, 787, 45, 794, 63, 776, 67, 772, 72, 767, 76, 763, 94, 745, 102, 737, 90, 749, 109,
		730, 165, 674, 111, 728, 209, 630, 204, 635, 117, 722, 188, 651, 159, 680, 198, 641, 113, 726, 183, 656, 180, 659, 177, 662, 196, 643, 155, 684, 214,
		625, 126, 713, 131, 708, 219, 620, 222, 617, 226, 613, 230, 609, 232, 607, 262, 577, 252, 587, 418, 421, 416, 423, 413, 426, 411, 428, 376, 463, 395,
		444, 283, 556, 285, 554, 379, 460, 390, 449, 363, 476, 384, 455, 388, 451, 386, 453, 361, 478, 387, 452, 360, 479, 310, 529, 354, 485, 328, 511, 315,
		524, 337, 502, 349, 490, 335, 504, 324, 515, 323, 516, 320, 519, 334, 505, 359, 480, 295, 544, 385, 454, 292, 547, 291, 548, 381, 458, 399, 440, 380,
		459, 397, 442, 369, 470, 377, 462, 410, 429, 407, 432, 281, 558, 414, 425, 247, 592, 277, 562, 271, 568, 272, 567, 264, 575, 259, 580, 237, 602, 239,
		600, 244, 595, 243, 596, 275, 564, 278, 561, 250, 589, 246, 593, 417, 422, 248, 591, 394, 445, 393, 446, 370, 469, 365, 474, 300, 539, 299, 540, 364,
		475, 362, 477, 298, 541, 312, 527, 313, 526, 314, 525, 353, 486, 352, 487, 343, 496, 327, 512, 350, 489, 326, 513, 319, 520, 332, 507, 333, 506, 348,
		491, 347, 492, 322, 517, 330, 509, 338, 501, 341, 498, 340, 499, 342, 497, 301, 538, 366, 473, 401, 438, 371, 468, 408, 431, 375, 464, 249, 590, 269,
		570, 238, 601, 234, 605, 257, 582, 273, 566, 255, 584, 254, 585, 245, 594, 251, 588, 412, 427, 372, 467, 282, 557, 403, 436, 396, 443, 392, 447, 391,
		448, 382, 457, 389, 450, 294, 545, 297, 542, 311, 528, 344, 495, 345, 494, 318, 521, 331, 508, 325, 514, 321, 518, 346, 493, 339, 500, 351, 488, 306,
		533, 289, 550, 400, 439, 378, 461, 374, 465, 415, 424, 270, 569, 241, 598, 231, 608, 260, 579, 268, 571, 276, 563, 409, 430, 398, 441, 290, 549, 304,
		535, 308, 531, 358, 481, 316, 523, 293, 546, 288, 551, 284, 555, 368, 471, 253, 586, 256, 583, 263, 576, 242, 597, 274, 565, 402, 437, 383, 456, 357,
		482, 329, 510, 317, 522, 307, 532, 286, 553, 287, 552, 266, 573, 261, 578, 236, 603, 303, 536, 356, 483, 355, 484, 405, 434, 404, 435, 406, 433, 235,
		604, 267, 572, 302, 537, 309, 530, 265, 574, 233, 606, 367, 472, 296, 543, 336, 503, 305, 534, 373, 466, 280, 559, 279, 560, 419, 420, 240, 599, 258,
		581, 229, 610 };
int32_t table_5_7_2_5[]  __attribute__((section(".shared_data_ddr0_cacheable"), aligned(16)))=
	{ 1, 138, 2, 137, 3, 136, 4, 135, 5, 134, 6, 133, 7, 132, 8, 131, 9, 130, 10, 129, 11, 128, 12, 127, 13, 126, 14, 125, 15, 124, 16, 123, 17, 122, 18, 121,
		19, 120, 20, 119, 21, 118, 22, 117, 23, 116, 24, 115, 25, 114, 26, 113, 27, 112, 28, 111, 29, 110, 30, 109, 31, 108, 32, 107, 33, 106, 34, 105, 35, 104,
		36, 103, 37, 102, 38, 101, 39, 100, 40, 99, 41, 98, 42, 97, 43, 96, 44, 95, 45, 94, 46, 93, 47, 92, 48, 91, 49, 90, 50, 89, 51, 88, 52, 87, 53, 86, 54,
		85, 55, 84, 56, 83, 57, 82, 58, 81, 59, 80, 60, 79, 61, 78, 62, 77, 63, 76, 64, 75, 65, 74, 66, 73, 67, 72, 68, 71, 69, 70 };

void gen_x_u(lte_enodeb_t *enodeb, int32_t root_seq_idx, int32_t pre_format, int32_t zczc, int32_t hs_flag)
{
	int32_t u;
	int32_t N_loop = 0;
	int32_t N_zc;
	int32_t N_cs, N_x;
	int32_t N_gen_pre = 0;
	float _Complex *x_u;
	
	while(N_gen_pre < 64)
	{
		x_u = enodeb->prach_x_u_f[N_loop];
		
		// Determine u
		if (pre_format >= 0 && pre_format <= 3)
			u = table_5_7_2_4[(root_seq_idx + N_loop) % (sizeof(table_5_7_2_4) / sizeof(int32_t))];
		else
			u = table_5_7_2_5[(root_seq_idx + N_loop) % (sizeof(table_5_7_2_5) / sizeof(int32_t))];
	
		//% Determine N_zc
		if (pre_format >= 0 && pre_format <= 3)
			N_zc = 839;
		else
			N_zc = 139;
	
		//% Generate x_u
		for (int n = 0; n < N_zc; n++)
		{
			float phase = -M_PI * u * n * (n + 1) / (float) N_zc;
			x_u[n] = cosf(phase) + 1*I*sinf(phase);
		}
		
		//% Determine v_max and N_cs
		if(pre_format == 4)
			N_cs = table_5_7_2_3[zczc];
		
		if(hs_flag == 0)
		{
			if(pre_format >= 0 && pre_format <= 3)
				N_cs = table_5_7_2_2_urs[zczc];
			
			//% Unrestricted set
			if(N_cs == 0)
			{
				/* 1 PRACH на 1 последовательность */
				N_x = 1;
			}
			else
				N_x = floor(N_zc/N_cs);
		}
		else
		{
			ERROR(DCONFIG, "High-speed PRACH not supported\n");
#if 0
		if(pre_format >= 0 && pre_format <= 3)
			N_cs = table_5_7_2_2_rs[zczc];
		//% Determine d_u
		for(int32_t p=0; p<N_zc; p++)
		{
			if(p*u % N_zc == 1)
				break;
		}
		
		if(p >= 0 && p < N_zc/2)
			d_u = p;
		else
			d_u = N_zc - p;
		
		//% Determine N_RA_shift, d_start, N_RA_group, and N_neg_RA_shift
		if(d_u >= N_cs && d_u < N_zc/3)
		{
			N_RA_shift     = floor(d_u/N_cs);
			d_start        = 2*d_u + N_RA_shift*N_cs;
			N_RA_group     = floor(N_zc/d_start);
			N_neg_RA_shift = max(floor((N_zc - 2*d_u - N_RA_group*d_start)/N_cs), 0);
		}
		else
		{
			N_RA_shift     = floor((N_zc - 2*d_u)/N_cs);
			d_start        = N_zc - 2*d_u + N_RA_shift*N_cs;
			N_RA_group     = floor(d_u/d_start);
			N_neg_RA_shift = min(max(floor((d_u - N_RA_group*d_start)/N_cs), 0), N_RA_shift);
		}
		
		//% Restricted set
		v_max = N_RA_shift*N_RA_group + N_neg_RA_shift - 1;
#endif
		}
		
		N_gen_pre += N_x;
		N_loop++;
	}
	
	enodeb->fp.LTE_PRACH_Ncs = N_cs;
	enodeb->fp.LTE_PRACH_Nx = N_x;
	enodeb->n_prach_x_u = N_loop;
	
	DBG(DCONFIG, "Generated %i PRACH sequences Ncs %i Nx %i\n", enodeb->n_prach_x_u, enodeb->fp.LTE_PRACH_Ncs, enodeb->fp.LTE_PRACH_Nx);
}

static float _Complex X_res[1024];

void dft_twiddle(int N, float _Complex *X_in, Complex16 *X_out, Complex16 *X_out_u)
{
	int n, k;
	float _Complex W_conj;
	float g = sqrt(N) / 32767;

	for (k = 0; k < N; k++)
	{
		X_res[k] = 0;
		
		int a = 0, b = 0;

		for (n = 0; n < N; n++)
		{
			W_conj = Wcos[a % N] - 1*I*Wsin[a % N];
			X_res[k] += X_in[n] * W_conj;
			
			a += k;
			b += k;
		}
	}

	for (k = 0; k < N; k++)
	{
		X_out[k] = TO_COMPLEX16((uint16_t)(crealf(X_res[k]) / g), (uint16_t)(cimagf(X_res[k]) / g));
		X_out_u[k] = TO_COMPLEX16((uint16_t)(crealf(X_res[k]) * 32760.0f), (uint16_t)(cimagf(X_res[k]) * 32760.0f));
	}
}

lte_status_t lte_init_prach_tables(lte_enodeb_t *enodeb)
{
	uint32_t i;

	float prach_freq_offset = enodeb->fapi_config.prach_config.frequency_offset;

	/* Расчет центральной частоты канала PRACH */
	//float Fprach = -(50.0f * 12.0f / 2.0f - (prach_freq_offset + 3.0f) * 12.0f) * 15000.0f; // 10MHz BW
	float Fprach = -(((float) enodeb->fapi_config.rf_config.dl_channel_bandwidth) * 12.0f / 2.0f - (prach_freq_offset + 3.0f) * 12.0f) * 15000.0f;

	/* Генерация таблицы DDS */
	for (i = 0; i < enodeb->fp.CPRI_PRACH_SEQ_LEN; i++)
	{
		enodeb->prach_dds[i] = TO_COMPLEX16((uint16_t)(floor(cos(2.0f * M_PI * i * Fprach / enodeb->fp.CPRI_SAMPLERATE) * 32767.0f)),
				(uint16_t)(floor(-1.0f * sin(2.0f * M_PI * i * Fprach / enodeb->fp.CPRI_SAMPLERATE) * 32767.0f)));
	}
	
	/* Генерация PRACH */
	gen_x_u(enodeb, enodeb->fapi_config.prach_config.root_sequence_index, 0, enodeb->fapi_config.prach_config.zero_correlation_zone_configuration,
			enodeb->fapi_config.prach_config.high_speed_flag);
	
	for(i=0; i<enodeb->n_prach_x_u; i++)
	{
		dft_twiddle(839, enodeb->prach_x_u_f[i], enodeb->prach_x_u_fft[i], enodeb->prach_x_u[i]);
	}
	
	return OS_SUCCESS;
}

/* 
 * Инициализация таблиц для обработки UL
 */
lte_status_t lte_init_ul_tables(lte_enodeb_t *enodeb)
{
	uint32_t i, j, sf_idx;
	uint32_t n_pucch;

	/* Сдвиг на половину частотного канала - 7.5кГц */
	float ul_freq_offset = 7500;

	/* Генерация таблицы DDS */
#ifdef B4860
	/* В B4860 это адрес таблицы, сам массив находится в DDR */
	enodeb->ul_shift_dds = ul_shift_dds_ddr;
#endif

	for (i = 0; i < enodeb->fp.CPRI_SYMBOL_LEN; i++)
	{
		enodeb->ul_shift_dds[i] = TO_COMPLEX16((uint16_t)(floor(cos(2.0f * M_PI * i * ul_freq_offset / enodeb->fp.CPRI_SAMPLERATE) * 32767.0f)),
				(uint16_t)(floor(-1.0f * sin(2.0f * M_PI * i * ul_freq_offset / enodeb->fp.CPRI_SAMPLERATE) * 32767.0f)));
	}

	/*liblte_generate_ul_ref_sigs(enodeb);
	 liblte_generate_ul_ref_sigs_rx(enodeb);
	 liblte_generate_grouphop(enodeb);
	 liblte_generate_seqhop(enodeb);
	 liblte_generate_nPRS(enodeb);
	 */
	/* Расчет таблиц DMRS UL */
	for (i = 0; i < 10; i++)
	{
		for (j = 0; j < N_SUPPORTED_UL_NRB; j++)
		{
			/*void liblte_generate_dmrs_pusch(lte_enodeb_t *enodeb,
			 uint32_t             N_subfr,
			 uint32_t             cyclic_shift_dci,
			 uint32_t             N_prb,
			 uint32_t             layer,
			 Complex16             *dmrs_0,
			 Complex16             *dmrs_1)*/
			liblte_generate_dmrs_pusch(enodeb, i, 0, // TODO: Cyclic shift DCI
					supported_ul_nrb[j], 0, // LAYER
					enodeb->refsigs->pusch_dmrs0[i][j], enodeb->refsigs->pusch_dmrs1[i][j]);
		}
	}

	/* Таблицы NVI/BETA для демапера */
	generate_nvi_beta_tables();

	INFO(DTRX, "Generating PUCCH DMRS\n");

	liblte_init_ncs_cell(enodeb);

#if 0
	/* Расчет таблич DRS PUCCH */
	for(sf_idx = 0; sf_idx < 10; sf_idx++)
	{
		/* Генрация эталонных сигналов DRS, ACK, NACK и SR в диапазоне индексов 0...N1_PUCCH_AN + nCCE_max
		 * nCCE_max - максимальное количество CCE для 3-х символов PDCCH
		 */
		for(n_pucch=0; n_pucch < enodeb->fapi_config.pucch_config.n1_pucch_an + get_nCCE(3, enodeb, get_mi(enodeb)); n_pucch++)
		{
			/* Расчет сигналов DMRS
			 * Производится для всех значений индексов 0...N1_PUCCH_AN+nCCE_max
			 * Это необходимо, т.к. индексы SR лежат в диапазоне 0...N1_PUCCH_AN-1,
			 * а индексы HARQ лежат в диапазоне N1_PUCCH_AN...N1_PUCCH_AN + nCCE_max
			 * Передача может вестись в любом индексе (для format1, format1a)
			 */
			if(liblte_generate_pucch_dmrs(enodeb, pucch_format1, sf_idx, n_pucch,
							enodeb->refsigs->r_pucch_1[sf_idx][n_pucch]) != LTE_SUCCESS)
			{
				ERROR(DTRX, "Error generating PUCCH DMRS sf_idx %i n_pucch %i\n", sf_idx, n_pucch);
			}

			/* Расчет сигналов SR
			 * Производится для значений индексов 0...N1_PUCCH_AN-1
			 */
			if(n_pucch < enodeb->fapi_config.pucch_config.n1_pucch_an)
			{
				if(liblte_generate_pucch_sr(enodeb, sf_idx, n_pucch, enodeb->refsigs->r_pucch_sr[sf_idx][n_pucch],
								enodeb->refsigs->r_pucch_sr_srs[sf_idx][n_pucch]) != LTE_SUCCESS)
				{
					ERROR(DTRX, "Error generating PUCCH SR sf_idx %i n_pucch %i\n", sf_idx, n_pucch);
				}
			}
#if 0
			else
			{
				/* Расчет сигналов ACK/NACK
				 * Производится для индексов N1_PUCCH_AN...N1_PUCCH_AN+nCCE_max
				 * n_pucch = N1_PUCCH_AN + nCCE -> nCCE = n_pucch - N1_PUCCH_AN
				 */
				int32_t nCCE = n_pucch - enodeb->fapi_config.pucch_config.n1_pucch_an;

				if(liblte_generate_pucch_ack(enodeb, sf_idx, n_pucch, enodeb->refsigs->r_pucch_ack[sf_idx][nCCE],
								enodeb->refsigs->r_pucch_ack_srs[sf_idx][nCCE]) != LTE_SUCCESS)
				{
					ERROR(DTRX, "Error generating PUCCH ACK sf_idx %i n_pucch %i\n", sf_idx, i);
				}

				if(liblte_generate_pucch_nack(enodeb, sf_idx, n_pucch, enodeb->refsigs->r_pucch_nack[sf_idx][nCCE],
								enodeb->refsigs->r_pucch_nack_srs[sf_idx][nCCE]) != LTE_SUCCESS)
				{
					ERROR(DTRX, "Error generating PUCCH NACK sf_idx %i n_pucch %i\n", sf_idx, n_pucch);
				}
			}
#else
			/* Расчет сигналов ACK/NACK
			 * Производится для индексов N1_PUCCH_AN...N1_PUCCH_AN+nCCE_max
			 * n_pucch = N1_PUCCH_AN + nCCE -> nCCE = n_pucch - N1_PUCCH_AN
			 */
			int32_t nCCE = n_pucch - enodeb->fapi_config.pucch_config.n1_pucch_an;

			if(liblte_generate_pucch_ack(enodeb, sf_idx, n_pucch, enodeb->refsigs->r_pucch_ack[sf_idx][n_pucch],
							enodeb->refsigs->r_pucch_ack_srs[sf_idx][n_pucch]) != LTE_SUCCESS)
			{
				ERROR(DTRX, "Error generating PUCCH ACK sf_idx %i n_pucch %i\n", sf_idx, i);
			}

			if(liblte_generate_pucch_nack(enodeb, sf_idx, n_pucch, enodeb->refsigs->r_pucch_nack[sf_idx][n_pucch],
							enodeb->refsigs->r_pucch_nack_srs[sf_idx][n_pucch]) != LTE_SUCCESS)
			{
				ERROR(DTRX, "Error generating PUCCH NACK sf_idx %i n_pucch %i\n", sf_idx, n_pucch);
			}

#endif
		}
	}
#else
	/* Расчет таблиц DRS PUCCH 
	 * Расчет производится для 36 значений, т.к. всего возможно максимум 36 вариантов
	 * */
	for (sf_idx = 0; sf_idx < 10; sf_idx++)
	{
		/* Генрация эталонных сигналов DRS, ACK, NACK и SR в диапазоне индексов 0...N1_PUCCH_AN + nCCE_max
		 * nCCE_max - максимальное количество CCE для 3-х символов PDCCH
		 */
		for (n_pucch = 0; n_pucch < 36; n_pucch++)
		{
			/* Расчет сигналов DMRS
			 * Производится для всех значений индексов 0...N1_PUCCH_AN+nCCE_max
			 * Это необходимо, т.к. индексы SR лежат в диапазоне 0...N1_PUCCH_AN-1,
			 * а индексы HARQ лежат в диапазоне N1_PUCCH_AN...N1_PUCCH_AN + nCCE_max
			 * Передача может вестись в любом индексе (для format1, format1a)
			 */
			if (liblte_generate_pucch_dmrs(enodeb, pucch_format1, sf_idx, n_pucch, enodeb->refsigs->r_pucch_1[sf_idx][n_pucch]) != LTE_SUCCESS)
			{
				ERROR(DTRX, "Error generating PUCCH DMRS sf_idx %i n_pucch %i\n", sf_idx, n_pucch);
			}

			if (liblte_generate_pucch_dmrs(enodeb, pucch_format2, sf_idx, n_pucch, enodeb->refsigs->r_pucch_2[sf_idx][n_pucch]) != LTE_SUCCESS)
			{
				ERROR(DTRX, "Error generating PUCCH DMRS sf_idx %i n_pucch %i\n", sf_idx, n_pucch);
			}

			/* Расчет сигналов SR
			 * Производится для значений индексов 0...N1_PUCCH_AN-1
			 */
			if (liblte_generate_pucch_sr(enodeb, sf_idx, n_pucch, enodeb->refsigs->r_pucch_sr[sf_idx][n_pucch],
					enodeb->refsigs->r_pucch_sr_srs[sf_idx][n_pucch]) != LTE_SUCCESS)
			{
				ERROR(DTRX, "Error generating PUCCH SR sf_idx %i n_pucch %i\n", sf_idx, n_pucch);
			}

			/* Расчет сигналов ACK/NACK
			 * Производится для индексов N1_PUCCH_AN...N1_PUCCH_AN+nCCE_max
			 * n_pucch = N1_PUCCH_AN + nCCE -> nCCE = n_pucch - N1_PUCCH_AN
			 */

			if (liblte_generate_pucch_ack(enodeb, sf_idx, n_pucch, enodeb->refsigs->r_pucch_ack[sf_idx][n_pucch],
					enodeb->refsigs->r_pucch_ack_srs[sf_idx][n_pucch]) != LTE_SUCCESS)
			{
				ERROR(DTRX, "Error generating PUCCH ACK sf_idx %i n_pucch %i\n", sf_idx, i);
			}

			if (liblte_generate_pucch_nack(enodeb, sf_idx, n_pucch, enodeb->refsigs->r_pucch_nack[sf_idx][n_pucch],
					enodeb->refsigs->r_pucch_nack_srs[sf_idx][n_pucch]) != LTE_SUCCESS)
			{
				ERROR(DTRX, "Error generating PUCCH NACK sf_idx %i n_pucch %i\n", sf_idx, n_pucch);
			}

			if (liblte_generate_pucch_fmt2(enodeb, sf_idx, n_pucch, enodeb->refsigs->r_pucch_fmt2[sf_idx][n_pucch],
					enodeb->refsigs->r_pucch_fmt2_srs[sf_idx][n_pucch]) != LTE_SUCCESS)
			{
				ERROR(DTRX, "Error generating PUCCH FORMAT 2 sf_idx %i n_pucch %i\n", sf_idx, n_pucch);
			}

		}
	}
#endif

	return OS_SUCCESS;
}


static uint32_t bitrev_cc[32] =
	{ 1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31, 0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30 };

uint32_t generate_dummy_w_cc(uint32_t D, int8_t *w)
{
	uint32_t RCC = (D >> 5), ND;
	uint32_t col, Kpi, index;
	int32_t k;

	if ((D & 0x1f) > 0)
		RCC++;

	Kpi = (RCC << 5);
	//  Kpi3 = Kpi*3;
	ND = Kpi - D;
	//  ND3 = ND*3;
	// copy d02 to dD2 (for mod Kpi operation from clause (4), p.16 of 36.212
	k = 0;

	for (col = 0; col < 32; col++)
	{
		index = bitrev_cc[col];

		if (index < ND)
		{
			w[k] = LTE_NULL;
			w[Kpi + k] = LTE_NULL;
			w[(Kpi << 1) + k] = LTE_NULL;
		}

		k += RCC;
	}
	return (RCC);
}

/* 
 * Инициализация вектора W для CC rate-matching (PUSCH CQI)
 */
lte_status_t lte_init_ul_dummy_w(lte_enodeb_t *enodeb)
{
	if (enodeb->fp.LTE_N_RB_UL == 25)
	{
		enodeb->trx.dw_26_O_RCC = generate_dummy_w_cc(18 + 8, enodeb->trx.dw26);
	}
	
	return OS_SUCCESS;
}

/* Конфигурация eNodeB */
lte_status_t lte_enodeb_configure(lte_enodeb_t *enodeb, fapi_config_t *new_config)
{
	int32_t i, j, k;
	int32_t sym_no;
	int32_t re_offset;
	os_status status;
	os_status cpri_init_status = OS_SUCCESS;

	OS_ASSERT_COND(new_config != NULL);

	/* Сброс буфера событий */
	log_event_reset();

	if (enodeb->state != L1_IDLE && enodeb->state != L1_CONFIGURED)
		return LTE_ERR_L1_INVALID_STATE;

	/*
	 if (enodeb->state == L1_CONFIGURED && memcmp(&enodeb->fapi_config, new_config, sizeof(fapi_config_t)) == 0)
	 {
	 return LTE_SUCCESS;
	 }
	 */

	enodeb->secure_id = get_secure_id();

	// TODO: сделать количество секторов настраиваемым
	// 1 сектор
	LTE_N_SECTORS = 1;

	memcpy(&enodeb->fapi_config, new_config, sizeof(fapi_config_t));

	// Конфигурация антенных портов
	//enodeb->fp.LTE_N_LOG_ANTENNAS_RX = enodeb->fapi_config.rf_config.rx_antenna_ports;
	//enodeb->fp.LTE_N_LOG_ANTENNAS_TX = enodeb->fapi_config.rf_config.tx_antenna_ports;
	enodeb->fp.LTE_N_PHYS_ANTENNAS_RX = enodeb->fapi_config.rf_config.rx_antenna_ports;
	enodeb->fp.LTE_N_PHYS_ANTENNAS_TX = enodeb->fapi_config.rf_config.tx_antenna_ports;
	
	INFO(DCONFIG, "4G L1 B4860 version " GIT_CURRENT_SHA1);
	
#ifdef DEBUG_BUILD
	/* Вывод ID чипа в отладочной версии */
	uint32_t get_chip_id_ext();
	INFO(DCONFIG, "ID 0x%08x\n", get_chip_id_ext());
#endif

	INFO(DCONFIG, "Configuring %iT%iR antennas\n", enodeb->fp.LTE_N_PHYS_ANTENNAS_TX, enodeb->fp.LTE_N_PHYS_ANTENNAS_RX);
	
	INFO(DCONFIG, "Free mem: LC %i SC %i DLC %i DSC %i HDSC %i\n", osGetFreeMemSize(OS_MEM_LOCAL_CACHEABLE),
			osGetFreeMemSize(OS_MEM_SHARED_CACHEABLE),
			osGetFreeMemSize(OS_MEM_DDR0_LOCAL_CACHEABLE),
			osGetFreeMemSize(OS_MEM_DDR0_SHARED_CACHEABLE),
			osGetFreeMemSize(OS_MEM_HET_DDR0_SHARED_CACHEABLE)
			);

	enodeb->N_id_cell = enodeb->fapi_config.sch_config.physical_cell_id;
	INFO(DCONFIG, "Configuring N_id_cell=%i\n", enodeb->N_id_cell);

	enodeb->fp.LTE_N_RB_DL = enodeb->fapi_config.rf_config.dl_channel_bandwidth;
	enodeb->fp.LTE_N_RB_UL = enodeb->fapi_config.rf_config.ul_channel_bandwidth;
	INFO(DCONFIG, "Configuring L1 %i RB\n", enodeb->fp.LTE_N_RB_DL);

	switch (enodeb->fp.LTE_N_RB_DL)
	{
		case 15:
			enodeb->fp.LTE_SYMBOL_LEN = 256;
			enodeb->fp.LTE_CP0_LEN = 20;
			enodeb->fp.LTE_CPx_LEN = 18;
			enodeb->fp.LTE_SAMPLERATE = 3840000;
			enodeb->fp.LTE_SAMPLES_PER_SUBFRAME = 3840;
			
			enodeb->fp.CPRI_SAMPLERATE = 7680000;
			enodeb->fp.CPRI_SYMBOL_LEN = 512;
			enodeb->fp.CPRI_CP0_LEN = 40;
			enodeb->fp.CPRI_CPx_LEN = 36;
			enodeb->fp.CPRI_SAMPLES_PER_SUBFRAME = 7680;
			enodeb->fp.CPRI_PRACH_SEQ_LEN = 6144;
			enodeb->fp.CPRI_PRACH_SEQ_CP = 792;

			enodeb->fp.LTE_PRACH_SEQ_LEN = 3072;
			enodeb->fp.LTE_PRACH_SEQ_CP = 396;
			
			/* Здесь присваиваются значения по-умолчанию для параметров Ncs и Nx
			 * Окончательное их заполнение производится в функции lte_init_prach_tables()
			 */
			enodeb->fp.LTE_PRACH_Ncs = 13;
			enodeb->fp.LTE_PRACH_Nx = 4;
			
			//LTE_PRACH_Ncs_125x = 13 * 1024 / 839;
			//LTE_PRACH_NCS_FFT_TO_ZC_COEFF = ((512 * 65536) / 839);
			//LTE_PRACH_NCS_ZC_TO_FFT_COEFF = ((839 * 65536) / 512);
			//LTE_PRACH_GUARD_SAMPLES  = 0;

			enodeb->fp.LTE_RBG_SIZE = 2;
			enodeb->fp.LTE_RB_TYPE0_BITMASK_LENGTH = 8;

			enodeb->fp.LTE_SR_THRESHOLD = 24576; //(0.8 / 6), 6 DMRS символов PUCCH

			break;

		case 25:
			enodeb->fp.LTE_SYMBOL_LEN = 512;
			enodeb->fp.LTE_CP0_LEN = 40;
			enodeb->fp.LTE_CPx_LEN = 36;

			enodeb->fp.LTE_SAMPLERATE = 7680000;
			enodeb->fp.LTE_SAMPLES_PER_SUBFRAME = 7680;
			
			enodeb->fp.CPRI_SAMPLERATE = 7680000;
			enodeb->fp.CPRI_SYMBOL_LEN = 512;
			enodeb->fp.CPRI_CP0_LEN = 40;
			enodeb->fp.CPRI_CPx_LEN = 36;
			enodeb->fp.CPRI_SAMPLES_PER_SUBFRAME = 7680;
			enodeb->fp.CPRI_PRACH_SEQ_LEN = 6144;
			enodeb->fp.CPRI_PRACH_SEQ_CP = 792;

			enodeb->fp.LTE_PRACH_SEQ_LEN = 6144;
			enodeb->fp.LTE_PRACH_SEQ_CP = 792;
			enodeb->fp.LTE_PRACH_Ncs = 13;

			//LTE_PRACH_Ncs_125x = 13 * 1024 / 839;
			enodeb->fp.LTE_PRACH_Nx = 4;
			//LTE_PRACH_NCS_FFT_TO_ZC_COEFF = ((512 * 65536) / 839);
			//LTE_PRACH_NCS_ZC_TO_FFT_COEFF = ((839 * 65536) / 512);
			//LTE_PRACH_GUARD_SAMPLES  = 0;

			enodeb->fp.LTE_RBG_SIZE = 2;
			enodeb->fp.LTE_RB_TYPE0_BITMASK_LENGTH = 13;

			enodeb->fp.LTE_SR_THRESHOLD = (32767 / 4) / 4; //24576; //(0.8 / 6), 6 DMRS символов PUCCH
			
			break;

		case 50:
			enodeb->fp.LTE_SYMBOL_LEN = 1024;
			enodeb->fp.LTE_CP0_LEN = 80;
			enodeb->fp.LTE_CPx_LEN = 72;

			enodeb->fp.LTE_SAMPLERATE = 15360000;
			enodeb->fp.LTE_SAMPLES_PER_SUBFRAME = 15360;

			enodeb->fp.LTE_PRACH_SEQ_LEN = 12288;
			enodeb->fp.LTE_PRACH_SEQ_CP = 1584;
			enodeb->fp.LTE_PRACH_Ncs = 13;
			//LTE_PRACH_Ncs_125x = 13 * 1024 / 839;
			enodeb->fp.LTE_PRACH_Nx = 4;
			//LTE_PRACH_NCS_FFT_TO_ZC_COEFF = ((1024 * 65536) / 839);
			//LTE_PRACH_NCS_ZC_TO_FFT_COEFF = ((839 * 65536) / 1024);
			//LTE_PRACH_GUARD_SAMPLES  = 3;

			enodeb->fp.LTE_RBG_SIZE = 3;
			enodeb->fp.LTE_RB_TYPE0_BITMASK_LENGTH = 17;

			enodeb->fp.LTE_SR_THRESHOLD = 24576; //(0.8 / 6), 6 DMRS символов PUCCH
			break;

		case 6:
		case 75:
		case 100:
		default:
			ERROR(DCONFIG, "Invalid downlink N_RB_DL specified: %i\n", enodeb->fp.LTE_N_RB_DL);
			return LTE_ERR_L1_INVALID_N_RB_DL;
	}

	/* расчет других значений параметров фрейма */
	enodeb->fp.LTE_PRACH_SEQ_CP_LEN = (enodeb->fp.LTE_PRACH_SEQ_LEN + enodeb->fp.LTE_PRACH_SEQ_CP);
	enodeb->fp.LTE_N_RE = (enodeb->fp.LTE_N_RB_DL * LTE_N_SC_RB);
	enodeb->fp.LTE_N_RE_UL = (enodeb->fp.LTE_N_RB_UL * LTE_N_SC_RB);
	enodeb->fp.LTE_N_SC_DL_HALF = (enodeb->fp.LTE_N_RB_DL * LTE_N_SC_RB / 2);
	enodeb->fp.LTE_PUSCH_TA_Ts_FACTOR = 30720 / enodeb->fp.LTE_SAMPLES_PER_SUBFRAME;

	enodeb->fp.LTE_FIRST_SS_CARRIER = (enodeb->fp.LTE_N_SC_DL_HALF - 72 / 2);
	/* Конфигурация уровней каналов, в зависимости от количества антенн
	 * По-идее число антенн и уровни должны спускать из L2 по FAPI
	 * TODO: а так ли это?
	 */
	int32_t ant_gain_mult = 32760;
	switch (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX)
	{
		case 1:
			ant_gain_mult = 32760;
			break;

		case 2:
			ant_gain_mult = 23160;
			//ant_gain_mult = 32760;
			break;

		case 4:
			ant_gain_mult = 16380;
			break;

		case 8:
			ant_gain_mult = 11580;
			break;

		default:
			ant_gain_mult = 32760;
	}

	/* Конфигурация  параметров L1 PHY при нормальном режиме работы
	 * В режиме RAW_IQ конфигурация не требуется
	 */
	enodeb->fp.LTE_PSS_GAIN_E = -3;
	//enodeb->fp.LTE_PSS_GAIN_M = (GAIN_MOD / (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX));
	enodeb->fp.LTE_PSS_GAIN_M = (GAIN_MOD * ant_gain_mult) >> 15;

	enodeb->fp.LTE_SSS_GAIN_E = -3;
	enodeb->fp.LTE_SSS_GAIN_M = (GAIN_MOD * ant_gain_mult) >> 15;

	enodeb->fp.LTE_CSRS_GAIN_E = 0;
	enodeb->fp.LTE_CSRS_GAIN_M = GAIN_SQRT_2;

	enodeb->fp.LTE_PBCH_GAIN_E = 0;
	enodeb->fp.LTE_PBCH_GAIN_M = (GAIN_SQRT_2 * ant_gain_mult) >> 15;

	enodeb->fp.LTE_PCFICH_GAIN = (GAIN_SQRT_2 * ant_gain_mult) >> 15;
	enodeb->fp.LTE_PDCCH_GAIN = (GAIN_SQRT_2 * ant_gain_mult) >> 15;

	enodeb->fp.LTE_CW_GAIN_E_QPSK = 0;
	enodeb->fp.LTE_CW_GAIN_M_QPSK = GAIN_SQRT_2;//(GAIN_SQRT_2 * ant_gain_mult) >> 15;;

	enodeb->fp.LTE_CW_GAIN_E_QAM16 = 0;
	enodeb->fp.LTE_CW_GAIN_M_QAM16 = GAIN_SQRT_10;//(GAIN_SQRT_10 * ant_gain_mult) >> 15;;

	enodeb->fp.LTE_CW_GAIN_E_QAM64 = 0;
	enodeb->fp.LTE_CW_GAIN_M_QAM64 = GAIN_SQRT_42;//(GAIN_SQRT_42 * ant_gain_mult) >> 15;

	switch (enodeb->fapi_config.phich_config.phich_resource)
	{
		case 0:
			// 1/6
			enodeb->fapi_config.phich_config.phich_resource = LTE_PHICH_RESOURCE_ONESIXTH;
			break;
		case 1:
			// 1/2
			enodeb->fapi_config.phich_config.phich_resource = LTE_PHICH_RESOURCE_HALF;
			break;
		case 2:
			// 1
			enodeb->fapi_config.phich_config.phich_resource = LTE_PHICH_RESOURCE_ONE;
			break;
		case 3:
			// 2
			enodeb->fapi_config.phich_config.phich_resource = LTE_PHICH_RESOURCE_TWO;
			break;
		default:
			ERROR(DCONFIG, "Invalid PHICH resourse %i\n", enodeb->fapi_config.phich_config.phich_resource);
	}

	INFO(DCONFIG, "PHICH resource: %i\n", enodeb->fapi_config.phich_config.phich_resource);

	enodeb->lte_enodeb_params.N_id_2 = (enodeb->N_id_cell % 3);
	enodeb->lte_enodeb_params.N_id_1 = (enodeb->N_id_cell - enodeb->lte_enodeb_params.N_id_2) / 3;
	enodeb->refsigs = &lte_enodeb_refsigs_inst;

	memset(&enodeb->refsigs->pss, 0, sizeof(maple_pdsch_ss_header_t));
	memset(&enodeb->refsigs->sss0, 0, sizeof(maple_pdsch_ss_header_t));
	memset(&enodeb->refsigs->sss5, 0, sizeof(maple_pdsch_ss_header_t));

	// Заполнение массивов PSS/SSS
	for (i = 0; i < 72; i++)
	{
		enodeb->refsigs->pss.signal_input[i].k0_re = enodeb->fp.LTE_FIRST_SS_CARRIER + i;
		enodeb->refsigs->pss.signal_input[i].k0_img = enodeb->fp.LTE_FIRST_SS_CARRIER + i;

		enodeb->refsigs->sss0.signal_input[i].k0_re = enodeb->fp.LTE_FIRST_SS_CARRIER + i;
		enodeb->refsigs->sss0.signal_input[i].k0_img = enodeb->fp.LTE_FIRST_SS_CARRIER + i;

		enodeb->refsigs->sss5.signal_input[i].k0_re = enodeb->fp.LTE_FIRST_SS_CARRIER + i;
		enodeb->refsigs->sss5.signal_input[i].k0_img = enodeb->fp.LTE_FIRST_SS_CARRIER + i;
	}

#if 1
	// Версия с полной амплитудой 4q12, при этом используются коэффиценты амплитуды при генерации в MAPLE
	for (i = 0; i < 62; i++)
	{
		enodeb->refsigs->pss.signal_input[i + 5].data_re = (uint16_t) creal16(liblte_pss_mod[enodeb->lte_enodeb_params.N_id_2][i]);
		enodeb->refsigs->pss.signal_input[i + 5].data_img = (uint16_t) cimag16(liblte_pss_mod[enodeb->lte_enodeb_params.N_id_2][i]);

		enodeb->refsigs->sss0.signal_input[i + 5].data_re = (uint16_t) creal16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][0]);
		enodeb->refsigs->sss0.signal_input[i + 5].data_img = (uint16_t) cimag16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][0]);

		enodeb->refsigs->sss5.signal_input[i + 5].data_re = (uint16_t) creal16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][1]);
		enodeb->refsigs->sss5.signal_input[i + 5].data_img = (uint16_t) cimag16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][1]);
	}
#else
	// Версия с нормализацией к 2^12 (PSS и SSS записаны в формате 4q12, а не 1q15)

	// 4090 
#define PSS_GAIN (4090)
	// 4090 * sqrt(2)
#define SSS_GAIN (5784) 			
	for(i=0; i<62; i++)
	{
		int32_t r;
		r = (uint16_t)((((int16_t)creal16(liblte_pss_mod[enodeb->lte_enodeb_params.N_id_2][i])) * PSS_GAIN) >> 15);
		enodeb->refsigs->pss.signal_input[i+5].data_re = r;

		r = (uint16_t)((((int16_t)cimag16(liblte_pss_mod[enodeb->lte_enodeb_params.N_id_2][i])) * PSS_GAIN) >> 15);
		enodeb->refsigs->pss.signal_input[i+5].data_img = r;

		r = (uint16_t)((((int16_t)creal16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][0])) * SSS_GAIN) >> 15);
		enodeb->refsigs->sss0.signal_input[i+5].data_re = r; //(uint16_t)creal16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][0]);

		r = (uint16_t)((((int16_t)cimag16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][0])) * SSS_GAIN) >> 15);
		enodeb->refsigs->sss0.signal_input[i+5].data_img = r;//(uint16_t)cimag16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][0]);

		r = (uint16_t)((((int16_t)creal16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][1])) * SSS_GAIN) >> 15);
		enodeb->refsigs->sss5.signal_input[i+5].data_re = r;//(uint16_t)creal16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][1]);

		r = (uint16_t)((((int16_t)cimag16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][1])) * SSS_GAIN) >> 15);
		enodeb->refsigs->sss5.signal_input[i+5].data_img = r;//(uint16_t)cimag16(liblte_sss_mod[enodeb->lte_enodeb_params.N_id_2][enodeb->lte_enodeb_params.N_id_1][i][1]);
	}

#endif	
	enodeb->lte_enodeb_params.Vshift = enodeb->N_id_cell % 6;

	/* Инициализация gold tables */
	lte_init_gold_tables(enodeb, 0);

	/* Инициализация таблицы распределения PCFICH */
	liblte_generate_pcfich_reg_mapping(enodeb);

	/* Инициализация таблицы распределения PHICH */
	liblte_generate_phich_reg_mapping(enodeb);

	/* Инициализация CSRS для 0-го символа */
	for (i = 0; i < 10; i++)
	{
		memset(&enodeb->refsigs->lte_csrs_sym0[i], 0, sizeof(Complex16) * enodeb->fp.LTE_N_RE);
		//liblte_gen_dl_cell_spec(enodeb, enodeb->refsigs->lte_csrs_sym0[i], 8192, i << 1, 0, 0);
		liblte_gen_dl_cell_spec(enodeb, enodeb->refsigs->lte_csrs_sym0[i][0], enodeb->fp.LTE_CSRS_GAIN_M, i << 1, 0, 0);

		if (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2)
		{
			liblte_gen_dl_cell_spec(enodeb, enodeb->refsigs->lte_csrs_sym0[i][1], enodeb->fp.LTE_CSRS_GAIN_M, i << 1, 0, 1);
		}
	}

	/* Параметры SIB2 */
	lte_init_prach_tables(enodeb);

	lte_init_ul_tables(enodeb);

	/* Скремблирующая последовательность PUCCH2 (CQI)
	 * 36.211 5.4.2
	 */
	for (i = 0; i < 10; i++)
	{
		uint32_t rnti;
		uint32_t c_init;
		uint32_t pr;
		uint32_t x1;

		for (rnti = 0; rnti < 65536; rnti++)
		{
			c_init = (((i + 1) * (2 * enodeb->N_id_cell + 1)) << 16) + rnti;

			pr = lte_gold_generic(&x1, &c_init, 1);

			enodeb->refsigs->scrambling_pucch2[i][rnti] = pr;
		}
	}
	
	/* Генерация вектора W для CC rate-matching (PUSHC CQI) */
	lte_init_ul_dummy_w(enodeb);

#if OLD_CODE
	/* Инициализация случайного заполнения слота */
	for(i=0; i<7; i++)
	memset(&enodeb->rand_slot.syms[i], 0, LTE_SYMBOL_LEN * sizeof(Complex16));
#endif

	
	/* Инициализация таблицы квадратного корня */
	for(i=0; i<SQRT_TABLE_SIZE; i++)
	{
		sqrt_table[i] = (int32_t)sqrtf(i*4);
	}
	
	/* Инициализация буферов CPRI, выполняется во всех режимах работы */
	cpri_setup_buffers(enodeb);

	/* Инициализация интерфейса CPRI */ 
	INFO(DCONFIG, "L1 initializing CPRI link\n");
	
	/* Init CPRI link */
	/* Номер логического интерфейса CPRI равен номер eNodeb */ 
	//enodeb->trx.cpri_no = enodeb->no;
	if(cpri_global_params->cpri_num_of_used_units > 0)
	{
		cpri_init_status = cpri_init_top(enodeb);
	
		if(cpri_init_status != OS_SUCCESS)
			ERROR(DCONFIG, "Error initializing CPRI link!\n");
		else
		{
			/* Переинициализация интерфейса CPRI, т.к. могла быть изменена конфигурация антенн, и т.п. */
			cpri_init_status = cpri_reinit(enodeb);
		}
	}

	if(cpri_init_status == OS_SUCCESS)
	{
		INFO(DCONFIG, "L1 configuration done, going to L1_CONFIGURED state\n");

		/* Переход в состояние L1_CONFIGURED */
		enodeb->state = L1_CONFIGURED;
	}
	else
	{
		INFO(DCONFIG, "L1 configuration fail, remaining in L1_IDLE state\n");

		/* Ошибка конфигурации, остаемся в состоянии L1_IDLE */
		enodeb->state = L1_IDLE;		
	}
	
	return (enodeb->state == L1_CONFIGURED) ? LTE_SUCCESS : LTE_FAIL;
}

/* Инициализация едскрипторов аплинка */
#ifdef B4860
static int32_t get_maple_fft_size_code(int32_t fft_len)
{
	int32_t fft_size_code = 0;

	switch (fft_len)
	{
		case 128:
			fft_size_code = 0;
			break;

		case 256:
			fft_size_code = 1;
			break;

		case 512:
			fft_size_code = 2;
			break;

		case 1024:
			fft_size_code = 3;
			break;

		case 2048:
			fft_size_code = 5;
			break;

		default:
			OS_ASSERT;
	}

	return fft_size_code;
}
#else
static int32_t get_maple_fft_size_code(int32_t fft_len)
{
	int32_t fft_size_code = 0;

	switch (fft_len)
	{
		case 128:
		fft_size_code = 35;
		break;

		case 256:
		fft_size_code = 36;
		break;

		case 512:
		fft_size_code = 37;
		break;

		case 1024:
		fft_size_code = 38;
		break;

		case 2048:
		fft_size_code = 39;
		break;

		default:
		OS_ASSERT;
	}

	return fft_size_code;
}
#endif

int32_t trx_fill_pufft_jobs(lte_enodeb_t *enodeb)
{
	int32_t i, job_no;
	lte_trx_t *trx = &enodeb->trx;
	int32_t fft_size_code = get_maple_fft_size_code(enodeb->fp.LTE_SYMBOL_LEN);
	int32_t cpri_fft_size_code = get_maple_fft_size_code(enodeb->fp.CPRI_SYMBOL_LEN);
	
#define OVA_SCALING ((uint32_t)(63))
	
	trx->pufft_ova_scl = 4;

	// FIXME: переделать для MAPLE-B3
	for (i = 0; i < TRX_NUM_OF_UL_JOBS; i++)
	{
		lte_ul_subframe_t *sf = trx->lte_ul_subframe_pool[i];
		int32_t n_pusch_sym = 0;

		for (job_no = 0; job_no < LTE_NSYMB_PER_SUBFRAME; job_no++)
		{
			lte_pufft_job_t *job = &sf->pufft_job[job_no];
			job->cop_job.job_id = (cop_job_id) i;
			job->cop_job.device_specific = &job->pufft_job;

			// Guard removal is enabled by PUFFT2_EDF by default (s. 1.4.14.5)
			//job->pufft_job.first_flags = PUFFT_ANT_MASK(0x01)  | PUFFT_ACK_INT_EN | fft_size_code;// |	PUFFT_OVA_SCL;
			job->pufft_job.first_flags = PUFFT_ANT_MASK(((1 << enodeb->fp.LTE_N_PHYS_ANTENNAS_RX) - 1)) | 
					PUFFT_ACK_INT_EN | fft_size_code;// | PUFFT_OVA_SCL;
			
			// Pre-mult buffer update on 1-st symbol inly
			if (job_no == 0)
			{
				job->pufft_job.first_flags |= PUFFT_PRE_MULTI_EN_AND_UPDATE;
			}
			else
			{
				//job->pufft_job.first_flags |= PUFFT_PRE_MULTI_EN_AND_UPDATE;
				job->pufft_job.first_flags |= PUFFT_PRE_MULTI_EN_NO_UPDATE;
			}

			// Pre-multiplier vector specified at device open
			//job_sym0->pufft_job.pre_multiplier_input = &enodeb_inst.ul_shift_dds;
#ifdef USE_PUFFT_OVA_SCL
			job->pufft_job.first_flags |= PUFFT_OVA_SCL;
			job->pufft_job.second_flags = PUFFT_ADP_OVA_SCL(trx->pufft_ova_scl);
#else
			job->pufft_job.second_flags = PUFFT_ADP_OVA_SCL(0);
#endif
			
			
			/* Использование альтернативного размера FFT если длина символа CPRI != LTE */
			if(fft_size_code < cpri_fft_size_code)
			{
				job->pufft_job.second_flags |= (0x8 | cpri_fft_size_code);
			}

			// Заполнение адреса входного буфера производится в обработчике прерываний
			// TODO: переделать на прямую работу с CPRI
			job->pufft_job.antx_input[0] = 0;

			job->pufft_job.status_ptr = &job->status;

			if (job_no == 3)
			{
				job->pufft_job.output = &sf->pufft_buf_a0_sym3[0];
				job->pufft_job.output_gap = enodeb->fp.LTE_N_RE_UL * sizeof(Complex16);
			}
			else if (job_no == 10)
			{
				job->pufft_job.output = &sf->pufft_buf_a0_sym10[0];
				job->pufft_job.output_gap = enodeb->fp.LTE_N_RE_UL * sizeof(Complex16);
			}
			else
			{
				job->pufft_job.output = &sf->pufft_buf_a0[enodeb->fp.LTE_N_RE * n_pusch_sym * enodeb->fp.LTE_N_PHYS_ANTENNAS_RX];
				job->pufft_job.output_gap = enodeb->fp.LTE_N_RE_UL * sizeof(Complex16);
				n_pusch_sym++;
			}

			if (job_no == LTE_NSYMB_PER_SUBFRAME - 1)
			{
				// Last symbol job, enable int and finish processing
				job->pufft_job.first_flags |= PUFFT_INT_EN;
				job->cop_job.next = NULL;
			}
			else
			{
				// Link to next job
				job->cop_job.next = &sf->pufft_job[job_no + 1].cop_job;
			}

			// Flush cache
			sweep_cache((uint32_t) job, sizeof(lte_pufft_job_t), CACHE_FLUSH);
		}

		// Число дескрипторов в списке, проверяется драйвером при запуске
		sf->n_pufft_jobs = job_no;
	}

	return 0;
}

/* Запуск L1 */
lte_status_t lte_enodeb_start(lte_enodeb_t *enodeb)
{
	os_status status;
	int32_t i;

	if (enodeb->state != L1_CONFIGURED)
		return LTE_ERR_L1_INVALID_STATE;

	/* Инициализация MAPLE и AIC */
	status = lte_maple_init(enodeb);
	OS_ASSERT_COND(OS_SUCCESS == status);

	//enodeb_inst.system_frame_no = 0;
	//enodeb_inst.system_subframe_no = 0;
	
	enodeb->system_frame_no = 0;
	enodeb->system_subframe_no = 0;
	enodeb->prev_system_frame_no = 0;
	enodeb->prev_system_subframe_no = 0;

	enodeb->flag_stop = 0;

	//osEventQueueReset(enodeb_inst.trx.evq_subframe_ind, 0);
	osEventQueueReset(enodeb->trx.evq_prach_rx, 0);

	for (i = 0; i < 10; i++)
	{
		enodeb->trx.fapi_dl_config[i] = NULL;
		enodeb->trx.fapi_ul_config[i] = NULL;
		enodeb->trx.fapi_tx_req[i] = NULL;
		enodeb->trx.fapi_hi_dci0[i] = NULL;
	}

	osEventQueueReset(enodeb->trx.evq_ul_pufft_ready, 0);

	enodeb->trx.frame_no_rx = 0;
	enodeb->trx.subframe_no_rx = 0;
	//enodeb_inst.trx.sym_no_rx = 0;

	enodeb->trx.rx_prach_no = 0;

	enodeb->trx.cpri_rx_process_buffer_no = 1;
	enodeb->trx.cpri_rx_active_buffer_no = 0;

	enodeb->trx.cpri_tx_active_buffer_no = 0;
	enodeb->trx.cpri_tx_prepare_buffer_no = 1;

	trx_fill_pufft_jobs(enodeb);

	trx_clear_ul_harq_buffers(&enodeb->trx);

	/* Запуск очереди обработки PUFFT */
	/* Для B4860 запуск идет в обработчике прерывания от CPRI или непосредственно блоком CPRI */
#ifdef CPRI_PUFFT_TIMER
	status = lte_maple_pufft_start(enodeb, enodeb->trx.lte_ul_subframe_pool[0]);
	OS_ASSERT_COND(status == OS_SUCCESS);
#endif

#ifdef TRX_DEBUG_BUFFER
	trx_init_debug_buffer();
#endif
	
	/* Очистка P8 */
	for(i=0; i<10; i++)
	{
		enodeb->trx.fapi_p8_back_indication[i] = NULL;
	}

	osCacheL2GlobalFlush();
	
	/*
	 *  Запуск CPRI IQ, таймеров
	 *  Производится после подготовки задач PUFFT	 
	 */
	if(cpri_global_params->cpri_num_of_used_units > 0)
		cpri_start(enodeb);

	INFO(DTRX, "L1 started\n");
	enodeb->state = L1_RUNNING;

	return LTE_SUCCESS;
}

/* Останов L1 */
lte_status_t lte_enodeb_stop(lte_enodeb_t *enodeb)
{
	if (enodeb->state != L1_RUNNING && enodeb->state != L1_CONFIGURED)
		return LTE_ERR_L1_INVALID_STATE;

	if (enodeb->state == L1_RUNNING)
	{
		enodeb->state = L1_CONFIGURED;
		
		/* Остановка CPRI */
		if(cpri_global_params->cpri_num_of_used_units > 0)
			cpri_stop(enodeb);
		
		osTaskDelay(20);

		/* Остановка MAPLE */
		lte_maple_close(enodeb);
	}

	INFO(DTRX, "L1 stopped\n");

	//enodeb->state = L1_CONFIGURED;

	osCacheL2GlobalFlush();

	return LTE_SUCCESS;
}

/* Инициализация BTS */
lte_enodeb_t *lte_enodeb_init(int32_t enodeb_no)
{
	lte_enodeb_t *enodeb;
	os_status status;

	enodeb = &g_enodeb_inst[enodeb_no];
	OS_ASSERT_COND(enodeb != NULL);

	memset(enodeb, 0, sizeof(lte_enodeb_t));

	enodeb->no = enodeb_no;
	enodeb->state = L1_IDLE;

	enodeb->sync_clock_tx = 0;
#if 0
	status = lte_enodeb_init_sockets(enodeb);
	OS_ASSERT_COND(status == OS_SUCCESS);
#endif

	status = lte_enodeb_init_trxs(enodeb);
	OS_ASSERT_COND(status == OS_SUCCESS);

	enodeb->state = L1_IDLE;

	return enodeb;
}
