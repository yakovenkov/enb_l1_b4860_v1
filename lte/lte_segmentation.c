#include <stdint.h>
#include <lte_defs.h>
#include <liblte_msc8157.h>

int32_t liblte_lte_segmentation(uint32_t B, uint32_t *C, uint32_t *Cplus, uint32_t *Cminus, uint32_t *Kplus,
		uint32_t *Kminus, uint32_t *F)
{
	uint32_t L, Bprime, Bprime_by_C, r, Kr, k, s, crc;

	if (B <= 6144)
	{
		L = 0;
		*C = 1;
		Bprime = B;
	}
	else
	{
		L = 24;
		*C = B / (6144 - L);

		if ((6144 - L) * (*C) < B)
			*C = *C + 1;

		Bprime = B + ((*C) * L);
	}

	if ((*C) > MAX_NUM_DLSCH_SEGMENTS)
	{
		//ERROR(TRX, "too many segments %d, B %d, L %d, Bprime %d\n", *C, B, L, Bprime);
		return (-1);
	}

	// Find K+
	Bprime_by_C = Bprime / (*C);

	if (Bprime_by_C <= 40)
	{
		*Kplus = 40;
		*Kminus = 0;
	}
	else if (Bprime_by_C <= 512)
	{ // increase by 1 byte til here
		*Kplus = (Bprime_by_C >> 3) << 3;
		*Kminus = Bprime_by_C - 8;
	}
	else if (Bprime_by_C <= 1024)
	{ // increase by 2 bytes til here
		*Kplus = (Bprime_by_C >> 4) << 4;

		if (*Kplus < Bprime_by_C)
			*Kplus = *Kplus + 16;

		*Kminus = (*Kplus - 16);
	}
	else if (Bprime_by_C <= 2048)
	{ // increase by 4 bytes til here
		*Kplus = (Bprime_by_C >> 5) << 5;

		if (*Kplus < Bprime_by_C)
			*Kplus = *Kplus + 32;

		*Kminus = (*Kplus - 32);
	}
	else if (Bprime_by_C <= 6144)
	{ // increase by 8 bytes til here

		*Kplus = (Bprime_by_C >> 6) << 6;

		if (*Kplus < Bprime_by_C)
			*Kplus = *Kplus + 64;

		*Kminus = (*Kplus - 64);
	}
	else
	{
		//ERROR(TRX, "Illegal codeword size !\n");
		return (-1);
	}

	if (*C == 1)
	{
		*Cplus = *C;
		*Kminus = 0;
		*Cminus = 0;
	}
	else
	{
		*Cminus = ((*C) * (*Kplus) - (Bprime)) / ((*Kplus) - (*Kminus));
		*Cplus = (*C) - (*Cminus);
	}

	*F = ((*Cplus) * (*Kplus) + (*Cminus) * (*Kminus) - (Bprime));

	return (0);
}
