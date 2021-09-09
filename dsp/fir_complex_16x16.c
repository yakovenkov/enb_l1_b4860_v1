#pragma opt_level = "O4"

#include "dsp_kernels.h"

void fir_complex_16x16(Word16 *x, Word16 *h, Word16 *y, Word16 nr, Word16 nh)
{
	int __SR__;
	int i, j,k,m;
	Word40 sum_imag0, sum_real0;
	Word40 sum_imag1, sum_real1;
	Word40 sum_imag2, sum_real2;
	Word40 sum_imag3, sum_real3;
	Word32 ih0,ih1,ix0,ix1,ix2,ix3,ix4,ix5,ix6,ix7;
	Word32 * ixi;
	Word32 * ih;
	__SR__ = readSR();
	__setscdown1(); //Set "1-bit down scaling" to ON
	
	cw_assert(nh%2==0 && nh >= 2);
	cw_assert(nr%4==0 && nr >= 4);
	nr = (2*nr)>>3;
	nh = nh>>1;
	cw_assert(nr>=1);
	for (i = 0,k=0; k < nr; k++,i += 8)
	{
		sum_imag0 = __l_to_x(0);
		sum_imag1 = __l_to_x(0);
		sum_imag2 = __l_to_x(0);
		sum_imag3 = __l_to_x(0);
		sum_real0 = __l_to_x(0);
		sum_real1 = __l_to_x(0);
		sum_real2 = __l_to_x(0);
		sum_real3 = __l_to_x(0);
		ixi = (Word32 *)((Word16 *)x+i);
		ih = (Word32 *)h;
		cw_assert(nh>=1);
		for (j = 0,m=0; m < nh;m++, j+=2)
		{
			__ld_2l(&ih[j+0],&ih0,&ih1);
			__ld_8l(&ixi[-j-1],&ix4,&ix0,&ix1,&ix2,&ix3,&ix5,&ix6,&ix7);
			__maccxd_ppx_2x(ix4,ix0,ih0,ih1,&sum_real0,&sum_imag0);
			__maccxd_ppx_2x(ix0,ix1,ih0,ih1,&sum_real1,&sum_imag1);
			__maccxd_ppx_2x(ix1,ix2,ih0,ih1,&sum_real2,&sum_imag2);
			__maccxd_ppx_2x(ix2,ix3,ih0,ih1,&sum_real3,&sum_imag3);
		}
		__st_srs_8f((void *)&y[i+0],sum_real0,sum_imag0,sum_real1,sum_imag1,sum_real2,sum_imag2,sum_real3,sum_imag3);
	}
	writeSR(__SR__);
}
