#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <lte_enodeb.h>
#include <liblte_msc8157.h>
#include <log.h>

#if !defined DEBUG_TX_BUFFER_TEST && !defined  DEBUG_OPT_OFF
#pragma opt_level = "O3"
#endif

static uint32_t Y;

#define CCEBITS 72
#define CCEPERSYMBOL 33  // This is for 1200 RE
#define CCEPERSYMBOL0 22  // This is for 1200 RE
#define DCI_BITS_MAX ((2*CCEPERSYMBOL+CCEPERSYMBOL0)*CCEBITS)
#define Msymb (DCI_BITS_MAX/2)

static uint32_t bitrev_cc_dci[32] =
	{ 1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31, 0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30 };

static Complex16 wtemp[2][Msymb] __attribute__((section(".local_data_ddr0_cacheable_bss")));

static uint8_t d[3 * (MAX_DCI_SIZE_BITS + 16) + 96];// __attribute__((section(".local_data_ddr1_bss")));
static uint8_t w[3 * 3 * (MAX_DCI_SIZE_BITS + 16)];// __attribute__((section(".local_data_ddr1_bss")));

static uint8_t e[DCI_BITS_MAX];// __attribute__((section(".local_data_ddr1_bss")));
static Complex16 yseq0[Msymb];// __attribute__((section(".local_data_ddr1_bss")));
static Complex16 yseq1[Msymb];// __attribute__((section(".local_data_ddr1_bss")));
static Complex16 wbar0[Msymb];// __attribute__((section(".local_data_ddr1_bss")));
static Complex16 wbar1[Msymb];// __attribute__((section(".local_data_ddr1_bss")));
static Complex16 wbar2[Msymb];// __attribute__((section(".local_data_ddr1_bss")));
static Complex16 wbar3[Msymb];// __attribute__((section(".local_data_ddr1_bss")));

void pdcch_scrambling(lte_enodeb_t *enodeb, uint8_t subframe, uint8_t *e, uint32_t length);

uint32_t check_phich_reg(lte_enodeb_t *enodeb, uint32_t kprime, uint8_t lprime, uint8_t mi)
{
	uint32_t i;
	uint32_t Ngroup_PHICH = (enodeb->fapi_config.phich_config.phich_resource * enodeb->fp.LTE_N_RB_DL) / 48;
	uint32_t mprime;
	uint32_t *pcfich_reg = enodeb->pcfich_reg;

	if ((lprime > 0) && (LTE_CP == LTE_CP_NORMAL))
		return (0);

	// compute REG based on symbol
	if ((lprime == 0) || ((lprime == 1) && (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 4)))
	{
		mprime = kprime / 6;
	}
	else
	{
		mprime = kprime >> 2;
	}

	/* check if PCFICH uses mprime */
	if ((lprime == 0) && ((mprime == pcfich_reg[0]) || (mprime == pcfich_reg[1]) || (mprime == pcfich_reg[2]) || (mprime == pcfich_reg[3])))
	{
		return (1);
	}

	/* TODO: handle SSF for TDD */
	if (mi > 0)
	{
		if (((enodeb->fapi_config.phich_config.phich_resource * enodeb->fp.LTE_N_RB_DL) % 48) > 0)
			Ngroup_PHICH++;

		if (LTE_CP == LTE_CP_EXTENDED)
		{
			Ngroup_PHICH <<= 1;
		}

		Ngroup_PHICH = Ngroup_PHICH * mi;
		
		for (i = 0; i < Ngroup_PHICH; i++)
		{
			if ((mprime == enodeb->phich_reg[i][0]) || (mprime == enodeb->phich_reg[i][1]) || (mprime == enodeb->phich_reg[i][2]))
			{
				return (1);
			}
		}
	}
	return (0);
}

void dci_encoding(uint8_t *a, uint8_t A, uint16_t E, uint8_t *e, uint16_t rnti)
{

	uint8_t D = (A + 16);
	uint32_t RCC;

	// encode dci 
	memset((void *) &d, LTE_NULL, 96);

	ccodelte_encode(A, 2, a, d + 96, rnti);
	RCC = sub_block_interleaving_cc(D, d + 96, w);
	
	lte_rate_matching_cc(RCC, E, w, e);

}

uint8_t *generate_dci0(uint8_t *dci, uint8_t *e, uint8_t DCI_LENGTH, uint8_t aggregation_level, uint16_t rnti)
{

	uint16_t coded_bits;

	coded_bits = 72 * aggregation_level;

	dci_encoding(dci, DCI_LENGTH, coded_bits, e, rnti);

	return (e + coded_bits);
}

uint16_t get_nquad(uint8_t num_pdcch_symbols, lte_enodeb_t *enodeb, uint8_t mi)
{

	uint16_t Nreg = 0;
	uint8_t Ngroup_PHICH = (enodeb->fapi_config.phich_config.phich_resource * enodeb->fp.LTE_N_RB_DL) / 48;

	if (((enodeb->fapi_config.phich_config.phich_resource * enodeb->fp.LTE_N_RB_DL) % 48) > 0)
		Ngroup_PHICH++;

#if (LTE_CP == LTE_CP_EXTENDED)
		Ngroup_PHICH <<= 1;
#endif

	Ngroup_PHICH *= mi;

	if ((num_pdcch_symbols > 0) && (num_pdcch_symbols < 4))
	{
		switch (enodeb->fp.LTE_N_RB_DL)
		{
			case 6:
				Nreg = 12 + (num_pdcch_symbols - 1) * 18;
				break;
			case 15:
				Nreg = 30 + (num_pdcch_symbols - 1) * 45;
				break;
			case 25:
				Nreg = 50 + (num_pdcch_symbols - 1) * 75;
				break;
			case 50:
				Nreg = 100 + (num_pdcch_symbols - 1) * 150;
				break;
			case 100:
				Nreg = 200 + (num_pdcch_symbols - 1) * 300;
				break;
			default:

				return (0);
		}
	}
	
	return (Nreg - 4 - (3 * Ngroup_PHICH));
}

uint16_t get_nCCE(uint8_t num_pdcch_symbols, lte_enodeb_t *enodeb, uint8_t mi)
{
	return(get_nquad(num_pdcch_symbols, enodeb, mi)/9);
}

void pdcch_interleaving(lte_enodeb_t *enodeb, Complex16 **z, Complex16 **wbar, uint8_t n_symbols_pdcch, uint8_t mi)
{

	Complex16 *wptr, *wptr2, *zptr;
	uint32_t Mquad = get_nquad(n_symbols_pdcch, enodeb, mi);
	uint32_t RCC = (Mquad >> 5), ND;
	uint32_t row, col, Kpi, index;
	int32_t i, k, a;

	if ((Mquad & 0x1f) > 0)
		RCC++;
	
	Kpi = (RCC << 5);
	ND = Kpi - Mquad;

	k = 0;
	for (col = 0; col < 32; col++)
	{
		index = bitrev_cc_dci[col];

		for (row = 0; row < RCC; row++)
		{
			if (index >= ND)
			{
				for (a = 0; a < enodeb->fp.LTE_N_PHYS_ANTENNAS_TX; a++)
				{
					wptr = &wtemp[a][k << 2];
					zptr = &z[a][(index - ND) << 2];

					wptr[0] = zptr[0];
					wptr[1] = zptr[1];
					wptr[2] = zptr[2];
					wptr[3] = zptr[3];
				}
				k++;
			}
			index += 32;
		}
	}

	// permutation
	for (i = 0; i < Mquad; i++)
	{

		for (a = 0; a < enodeb->fp.LTE_N_PHYS_ANTENNAS_TX; a++)
		{
			wptr = &wtemp[a][((i + enodeb->N_id_cell) % Mquad) << 2];
			wptr2 = &wbar[a][i << 2];
			wptr2[0] = wptr[0];
			wptr2[1] = wptr[1];
			wptr2[2] = wptr[2];
			wptr2[3] = wptr[3];
		}
	}
}

void pdcch_scrambling(lte_enodeb_t *enodeb, uint8_t subframe, uint8_t *e, uint32_t length)
{
	int32_t i;
	uint8_t reset;
	uint32_t x1, x2, s = 0;

	reset = 1;
	// x1 is set in lte_gold_generic

	x2 = (subframe << 9) + enodeb->N_id_cell;

	for (i = 0; i < length; i++)
	{
		if ((i & 0x1f) == 0)
		{
			s = lte_gold_generic(&x1, &x2, reset);
			reset = 0;
		}
		if (e[i] != 2) // <NIL> element is 2
			e[i] = (e[i] & 1) ^ ((s >> (i & 0x1f)) & 1);
	}
}

static uint32_t get_mi(lte_enodeb_t *enodeb, uint32_t subframe)
{

	// for FDD
	if (enodeb->fapi_config.subframe_config.duplex_mode == FAPI_DUPLEXING_MODE_FDD)
		return 1;

	// for TDD
	switch (enodeb->fapi_config.tdd_frame_structure_config.subframe_assignment)
	{

		case 0:
			if ((subframe == 0) || (subframe == 5))
				return (2);
			else
				return (1);

			break;

		case 1:
			if ((subframe == 0) || (subframe == 5))
				return (0);
			else
				return (1);

			break;

		case 2:
			if ((subframe == 3) || (subframe == 8))
				return (1);
			else
				return (0);

			break;

		case 3:
			if ((subframe == 0) || (subframe == 8) || (subframe == 9))
				return (1);
			else
				return (0);

			break;

		case 4:
			if ((subframe == 8) || (subframe == 9))
				return (1);
			else
				return (0);

			break;

		case 5:
			if (subframe == 8)
				return (1);
			else
				return (0);

			break;

		case 6:
			return (1);
			break;

		default:
			return (0);
	}
}

uint8_t liblte_generate_dci_top(uint32_t num_dci, DCI_ALLOC_t *dci_alloc,
		uint32_t n_rnti, int16_t amp, lte_enodeb_t *enodeb,
		lte_subframe_t *lte_subframe)
{
	uint8_t *e_ptr, num_pdcch_symbols;
	int8_t L;
	uint32_t i, lprime;
	uint32_t kprime, kprime_mod12, mprime, nsymb, symbol_offset, tti_offset;
	int16_t gain_lin_QPSK;
	int16_t re_offset;
	uint32_t mi = get_mi(enodeb, lte_subframe->subframe_no);

	Complex16 *y[2];
	Complex16 *wbar[4];

	int32_t nushiftmod3 = enodeb->lte_enodeb_params.Vshift % 3;

	int32_t Msymb2;
	int32_t split_flag = 0;
	uint32_t subframe;

	subframe = lte_subframe->subframe_no;

	if (enodeb->fp.LTE_N_RB_DL == 100)
		Msymb2 = Msymb;
	else if (enodeb->fp.LTE_N_RB_DL == 75)
		Msymb2 = 3 * Msymb / 4;
	else if (enodeb->fp.LTE_N_RB_DL == 50)
		Msymb2 = Msymb >> 1;
	else if (enodeb->fp.LTE_N_RB_DL == 25)
		Msymb2 = Msymb >> 2;
	else if (enodeb->fp.LTE_N_RB_DL == 15)
		Msymb2 = Msymb * 15 / 100;
	else if (enodeb->fp.LTE_N_RB_DL == 6)
		Msymb2 = Msymb * 6 / 100;
	else
		Msymb2 = Msymb >> 2;

	num_pdcch_symbols = lte_subframe->num_pdcch_symbols;

	wbar[0] = &wbar0[0];
	wbar[1] = &wbar1[0];
	wbar[2] = &wbar2[0];
	wbar[3] = &wbar3[0];
	y[0] = &yseq0[0];
	y[1] = &yseq1[0];

	// reset all bits to <NIL>, here we set <NIL> elements as 2
	memset(&e, 2, DCI_BITS_MAX);
	
	e_ptr = (uint8_t*) &e;
	
	// generate DCIs in order of decreasing aggregation level, then common/ue spec
	// MAC is assumed to have ordered the UE spec DCI according to the RNTI-based randomization
	for (L = 3; L >= 0; L--)
	{
		for (i = 0; i < num_dci; i++)
		{
			if (dci_alloc[i].L == (uint8_t) (1 << L))
			{
				e_ptr = generate_dci0(dci_alloc[i].dci_pdu, 
						&e[72 * dci_alloc[i].first_CCE],
						dci_alloc[i].dci_length,
						dci_alloc[i].L,
						dci_alloc[i].rnti);
			}
		}
	}

	// Scrambling
	pdcch_scrambling(enodeb, subframe, e, 8 * get_nquad(num_pdcch_symbols, enodeb, mi));

	// Modulation
	gain_lin_QPSK = (int16_t)((amp * GAIN_SQRT_2) >> 15);
	
	e_ptr = e;

	if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 1)
	{
		// SISO
		gain_lin_QPSK = amp; //(int16_t)((amp * GAIN_SQRT_2) >> 15);
		
		for (i = 0; i < Msymb2; i++)
		{
			int16_t iq_i, iq_q;
	
			iq_i = (*e_ptr == 2) ? 0 : (*e_ptr == 1) ? -gain_lin_QPSK : gain_lin_QPSK;
			e_ptr++;
	
			iq_q = (*e_ptr == 2) ? 0 : (*e_ptr == 1) ? -gain_lin_QPSK : gain_lin_QPSK;
			e_ptr++;
	
			y[0][i] = TO_COMPLEX16(iq_i, iq_q);
			y[1][i] = TO_COMPLEX16(iq_i, iq_q);
		}
	}
	else if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 2)
	{
		//ALAMOUTI
		gain_lin_QPSK = amp;//(int16_t)((amp * GAIN_SQRT_2) >> 15);
		for (i = 0; i < Msymb2; i += 2)
		{	
			int16_t iq0_i, iq0_q, iq1_i, iq1_q;
			
			iq0_i = (*e_ptr == 2) ? 0 : (*e_ptr == 1) ? -gain_lin_QPSK : gain_lin_QPSK;
			e_ptr++;
			iq0_q = (*e_ptr == 2) ? 0 : (*e_ptr == 1) ? -gain_lin_QPSK : gain_lin_QPSK;
			e_ptr++;
			
			iq1_i = (*e_ptr == 2) ? 0 : (*e_ptr == 1) ? -gain_lin_QPSK : gain_lin_QPSK;
			e_ptr++;
			iq1_q = (*e_ptr == 2) ? 0 : (*e_ptr == 1) ? -gain_lin_QPSK : gain_lin_QPSK;
			e_ptr++;

			// first antenna position n -> x0
			y[0][i] = TO_COMPLEX16(iq0_i, iq0_q);
			// second antenna position n -> -x1*
			y[1][i] = TO_COMPLEX16(-iq1_i, iq1_q);

			// fill in the rest of the ALAMOUTI precoding
			y[0][i + 1] = TO_COMPLEX16(iq1_i, iq1_q);
			y[1][i + 1] = TO_COMPLEX16(iq0_i, -iq0_q);			
		}
	}

	// This is the interleaving procedure defined in 36-211, first part of Section 6.8.5
	pdcch_interleaving(enodeb, &y[0], &wbar[0], num_pdcch_symbols, mi);

	mprime = 0;

	nsymb = LTE_NSYMB_PER_SUBFRAME;
	re_offset = 0;

	/* REG allocation algorithm from 36-211, 6.8.5 */
	for (kprime = 0; kprime < enodeb->fp.LTE_N_RE; kprime++)
	{
		for (lprime = 0; lprime < num_pdcch_symbols; lprime++)
		{
			symbol_offset = lprime * enodeb->fp.LTE_N_RE * enodeb->fp.LTE_N_PHYS_ANTENNAS_TX;

			tti_offset = re_offset;

			if (re_offset == enodeb->fp.LTE_N_RE - 2)
				split_flag = 1;
			else
				split_flag = 0;

			if (!check_phich_reg(enodeb, kprime, lprime, mi) == 1)
			{
				// Copy REG to TX buffer      
				if ((lprime == 0) || ((lprime == 1) && (enodeb->fp.LTE_N_PHYS_ANTENNAS_TX == 4)))
				{
					// first symbol, or second symbol+4 TX antennas skip pilots
					kprime_mod12 = kprime % 12;
					if ((kprime_mod12 == 0) || (kprime_mod12 == 6))
					{
						// kprime represents REG	    
						for (i = 0; i < 6; i++)
						{
							if ((i != (nushiftmod3)) && (i != (nushiftmod3 + 3)))
							{
								lte_subframe->pdcch_syms[symbol_offset + tti_offset + i] = wbar[0][mprime];
								
								if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX > 1)
								{
									lte_subframe->pdcch_syms[symbol_offset + tti_offset + i + enodeb->fp.LTE_N_RE] = wbar[1][mprime];
								}
								mprime++;
							}
						}
					}
				}
				else
				{ // no pilots in this symbol
					kprime_mod12 = kprime % 12;
					if ((kprime_mod12 == 0) || (kprime_mod12 == 4) || (kprime_mod12 == 8))
					{
						// kprime represents REG
						if (split_flag == 0)
						{
							for (i = 0; i < 4; i++)
							{
								lte_subframe->pdcch_syms[symbol_offset + tti_offset + i] = wbar[0][mprime];
								
								if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX > 1)
									lte_subframe->pdcch_syms[symbol_offset + tti_offset + i + enodeb->fp.LTE_N_RE] = wbar[1][mprime];
								
								mprime++;
							}
						}
						else
						{
							lte_subframe->pdcch_syms[symbol_offset + tti_offset + 0] = wbar[0][mprime];
							
							if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX > 1)
								lte_subframe->pdcch_syms[symbol_offset + tti_offset + 0 + enodeb->fp.LTE_N_RE] = wbar[1][mprime];
							
							mprime++;

							lte_subframe->pdcch_syms[symbol_offset + tti_offset + 1] = wbar[0][mprime];
							
							if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX > 1)
								lte_subframe->pdcch_syms[symbol_offset + tti_offset + 1 + enodeb->fp.LTE_N_RE] = wbar[1][mprime];

							mprime++;

							lte_subframe->pdcch_syms[symbol_offset + tti_offset - enodeb->fp.LTE_N_RE + 3] = wbar[0][mprime];
							
							if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX > 1)
								lte_subframe->pdcch_syms[symbol_offset + tti_offset + 3 - enodeb->fp.LTE_N_RE + enodeb->fp.LTE_N_RE] = wbar[1][mprime];

							mprime++;

							lte_subframe->pdcch_syms[symbol_offset + tti_offset - enodeb->fp.LTE_N_RE + 4] = wbar[0][mprime];
							if(enodeb->fp.LTE_N_PHYS_ANTENNAS_TX > 1)
								lte_subframe->pdcch_syms[symbol_offset + tti_offset + 4 - enodeb->fp.LTE_N_RE + enodeb->fp.LTE_N_RE] = wbar[1][mprime];

							mprime++;

						}
					}
				}
				if (mprime >= Msymb2)
					return (num_pdcch_symbols);
			}

		}

		re_offset++;

		if (re_offset == enodeb->fp.LTE_N_RE)
			re_offset = 0;

	}

	return (num_pdcch_symbols);
}

