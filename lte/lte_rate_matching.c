#include <string.h>
#include "liblte_msc8157.h"

//#if !defined DEBUG_TX_BUFFER_TEST && !defined DEBUG_OPT_OFF
#pragma opt_level = "O3"
//#endif

static uint32_t bitrev[32] =
	{ 0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30, 1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31 };
static uint32_t bitrev_cc[32] =
	{ 1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31, 0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30 };

uint32_t sub_block_interleaving_cc(uint32_t D, uint8_t *d, uint8_t *w)
{

	uint32_t RCC = (D >> 5), ND, ND3;
	uint32_t row, col, Kpi, index;
	uint32_t index3, k;

	if ((D & 0x1f) > 0)
		RCC++;
	Kpi = (RCC << 5);
	ND = Kpi - D;
	
	ND3 = ND * 3;

	k = 0;
	for (col = 0; col < 32; col++)
	{
		index = bitrev_cc[col];
		index3 = 3 * index;
		for (row = 0; row < RCC; row++)
		{
			w[k] = d[index3 - ND3];
			w[Kpi + k] = d[index3 - ND3 + 1];
			w[(Kpi << 1) + k] = d[index3 - ND3 + 2];
			
			index3 += 96;
			index += 32;
			k++;
		}
	}
	
	return (RCC);
}

void sub_block_deinterleaving_cc(uint32_t D, int8_t *d, int8_t *w)
{

	uint32_t RCC = (D >> 5), ND, ND3;
	uint32_t row, col, Kpi, index;
	uint32_t index3, k;

	if ((D & 0x1f) > 0)
		RCC++;
	Kpi = (RCC << 5);
	ND = Kpi - D;
	ND3 = ND * 3;

	k = 0;
	for (col = 0; col < 32; col++)
	{
		index = bitrev_cc[col];
		index3 = 3 * index;
		for (row = 0; row < RCC; row++)
		{
			d[index3 - ND3] = w[k];
			d[index3 - ND3 + 1] = w[Kpi + k];
			d[index3 - ND3 + 2] = w[(Kpi << 1) + k];
			
			index3 += 96;
			index += 32;
			k++;
		}
	}

}

uint32_t lte_rate_matching_cc(uint32_t RCC, uint16_t E, uint8_t *w, uint8_t *e)
{
	uint32_t ind = 0, k;
	uint16_t Kw = 3 * (RCC << 5);

	for (k = 0; k < E; k++)
	{
		while (w[ind] == LTE_NULL)
		{
			ind++;
			if (ind == Kw)
				ind = 0;
		}

		e[k] = w[ind];
		ind++;
		if (ind == Kw)
			ind = 0;
	}
	return (E);
}

void lte_rate_matching_cc_rx(uint32_t RCC, uint16_t E, int8_t *w, uint8_t *dummy_w, int8_t *soft_input)
{
	uint32_t ind = 0, k;
	uint16_t Kw = 3 * (RCC << 5);
	uint32_t acc = 1;
	int16_t w16[Kw];

	memset(w, 0, Kw);
	memset(w16, 0, Kw * sizeof(int16_t));

	for (k = 0; k < E; k++)
	{
		while (dummy_w[ind] == LTE_NULL)
		{
			ind++;

			if (ind == Kw)
				ind = 0;
		}

		w16[ind] += soft_input[k];
		ind++;

		if (ind == Kw)
		{
			ind = 0;
			acc++;
		}
	}

	// rescale
	for (ind = 0; ind < Kw; ind++)
	{
		if (w16[ind] > 127)
			w[ind] = 127;
		else if (w16[ind] < -128)
			w[ind] = -128;
		else
			w[ind] = (int8_t) w16[ind];
	}
}
