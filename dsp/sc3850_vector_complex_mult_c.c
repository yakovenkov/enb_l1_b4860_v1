/****************************************************************************
* SC3850 Core Libraries
* Freescale Semiconductor
* Copyright (C) 2008 All rights reserved
******************************************************************************
* File name: 	 sc3850_vector_complex_mult_c.c
* Function name: sc3850_vector_complex_mult_c
* Author name:   Duberly and Yongqing Liang 
*
* Description:   complex multiply
* int sc3850_vector_complex_mult_c(short *restrict coef, short *restrict input, short *restrict result, int N)
*
* Target Processor: StarCore SC3850
*
* Input:
*			x: pointer at input data, x[0/2/4/6..] are real, 
*			                          x[1/3/5/7..] are image, 
*			y: pointer at input data, y[0/2/4/6..] are real, 
*			                          y[1/3/5/7..] are image,
*			N: Length of input vectors.
*
* Output:
*			z: pointer at the product  z[0/2/4/6..] are real, 
*                                      z[1/3/5/7..] are image,
* Restritions:  N>0 && N%4==0
*
* Implement notes:
*
******************************************************************************
* Revision History
******************************************************************************
* $Log: sc3850_vector_complex_mult_c.c,v $
* Revision 1.1  2008/10/23 20:11:35  b14270
* initial version
*
*
****************************************************************************/

#pragma opt_level = "O3"

#include "dsp_kernels.h"

int sc3850_vector_complex_mult_c(short *restrict coef, short *restrict input, short *restrict result, int N)
{
#pragma noinline
	#pragma align *coef 8
	#pragma align *input 8
	#pragma align *result 8

	int *restrict coef_int = (int * restrict)coef;
	int *restrict input_int = (int * restrict)input;
	int *restrict result_int1 = (int * restrict)result;
	int *restrict result_int2 = (int * restrict)(result+4);
	
	#pragma align *coef_int 8
	#pragma align *input_int 8
	#pragma align *result_int1 8
	#pragma align *result_int2 8

    int i;
    int tempI1,tempQ1;
    int tempI2,tempQ2;
    int tempI3,tempQ3;
    int tempI4,tempQ4;
    
    int input0, input1, input2, input3;
    int coef0, coef1, coef2, coef3;
    
    input0 = input_int[0];
    input1 = input_int[1];
    coef0  = coef_int[0];
    coef1  = coef_int[1];

    cw_assert(N>0 && N%4==0);
	for(i=0;i<N;i=i+4)
	{
        tempI1 = L_mpyre(input0, coef0);
        tempQ1 = L_mpyim(input0, coef0);
        tempI2 = L_mpyre(input1, coef1);
	    tempQ2 = L_mpyim(input1, coef1);
	
        input2 = input_int[i+2];
        input3 = input_int[i+3];
        coef2  = coef_int[i+2];
        coef3  = coef_int[i+3];

	    tempI3 = L_mpyre(input2, coef2);
	    tempQ3 = L_mpyim(input2, coef2);
        tempI4 = L_mpyre(input3, coef3);
	    tempQ4 = L_mpyim(input3, coef3);

        input0 = input_int[i+4];
        input1 = input_int[i+5];
        coef0  = coef_int[i+4];
        coef1  = coef_int[i+5];
    
        writer_4f((short*)&result_int1[i], tempI1, tempQ1, tempI2, tempQ2);
        writer_4f((short*)&result_int2[i], tempI3, tempQ3, tempI4, tempQ4);
	    
	}
	
	return 0;
}

int sc3850_vector_complex_mult_asm(short *restrict coef, short *restrict input, short *restrict result, int N)
{
#pragma noinline
	#pragma align *coef 8
	#pragma align *input 8
	#pragma align *result 8

	int *restrict coef_int = (int * restrict)coef;
	int *restrict input_int = (int * restrict)input;
	int *restrict result_int1 = (int * restrict)result;
	int *restrict result_int2 = (int * restrict)(result+4);
	
	#pragma align *coef_int 8
	#pragma align *input_int 8
	#pragma align *result_int1 8
	#pragma align *result_int2 8

    int i;
    int tempI1,tempQ1;
    int tempI2,tempQ2;
    int tempI3,tempQ3;
    int tempI4,tempQ4;
    
    int input0, input1, input2, input3;
    int coef0, coef1, coef2, coef3;
    
    input0 = input_int[0];
    input1 = input_int[1];
    coef0  = coef_int[0];
    coef1  = coef_int[1];

    cw_assert(N>0 && N%4==0);
	for(i=0;i<N;i=i+4)
	{
        tempI1 = L_mpyre(input0, coef0);
        tempQ1 = L_mpyim(input0, coef0);
        tempI2 = L_mpyre(input1, coef1);
	    tempQ2 = L_mpyim(input1, coef1);
	
        input2 = input_int[i+2];
        input3 = input_int[i+3];
        coef2  = coef_int[i+2];
        coef3  = coef_int[i+3];

	    tempI3 = L_mpyre(input2, coef2);
	    tempQ3 = L_mpyim(input2, coef2);
        tempI4 = L_mpyre(input3, coef3);
	    tempQ4 = L_mpyim(input3, coef3);

        input0 = input_int[i+4];
        input1 = input_int[i+5];
        coef0  = coef_int[i+4];
        coef1  = coef_int[i+5];
    
        writer_4f((short*)&result_int1[i], tempI1, tempQ1, tempI2, tempQ2);
        writer_4f((short*)&result_int2[i], tempI3, tempQ3, tempI4, tempQ4);
	    
	}
	
	return 0;
}

int sc3850_vector_complex_mult_conj_sc3900(short *restrict input, short *restrict coef, short *restrict result, int N)
{
#pragma noinline
	#pragma align *coef 8
	#pragma align *input 8
	#pragma align *result 8

	int *restrict coef_int = (int * restrict)coef;
	int *restrict input_int = (int * restrict)input;
	int *restrict result_int1 = (int * restrict)result;
	int i;
	
	#pragma align *coef_int 8
	#pragma align *input_int 8
	#pragma align *result_int1 8
	#pragma align *result_int2 8

    cw_assert(N>0 && N%4==0);
	for(i=0;i<N;i=i+4)
	{
		/*
		result_int1[i+0] = __mpycx_c_sr_2w(coef_int[i+0], input_int[i+0]);
		result_int1[i+1] = __mpycx_c_sr_2w(coef_int[i+1], input_int[i+1]);
		result_int1[i+2] = __mpycx_c_sr_2w(coef_int[i+2], input_int[i+2]);
		result_int1[i+3] = __mpycx_c_sr_2w(coef_int[i+3], input_int[i+3]);
		*/
		
		__mpycx_c_sr_4w(coef_int[i+0], coef_int[i+1], input_int[i+0], input_int[i+1], &result_int1[i+0], &result_int1[i+1]);
		__mpycx_c_sr_4w(coef_int[i+2], coef_int[i+3], input_int[i+2], input_int[i+3], &result_int1[i+2], &result_int1[i+3]);
	}
	
	return 0;
}

int sc3850_vector_complex_mult_conj_asm(short *restrict coef, short *restrict input, short *restrict result, int N)
{
#pragma noinline
	#pragma align *coef 8
	#pragma align *input 8
	#pragma align *result 8

	int *restrict coef_int = (int * restrict)coef;
	int *restrict input_int = (int * restrict)input;
	int *restrict result_int1 = (int * restrict)result;
	int *restrict result_int2 = (int * restrict)(result+4);
	
	#pragma align *coef_int 8
	#pragma align *input_int 8
	#pragma align *result_int1 8
	#pragma align *result_int2 8

    int i;
    int tempI1,tempQ1;
    int tempI2,tempQ2;
    int tempI3,tempQ3;
    int tempI4,tempQ4;
    
    int input0, input1, input2, input3;
    int coef0, coef1, coef2, coef3;
    
    input0 = input_int[0];
    input1 = input_int[1];
    coef0  = coef_int[0];
    coef1  = coef_int[1];

    cw_assert(N>0 && N%4==0);
	for(i=0;i<N;i=i+4)
	{
        tempI1 = L_mpyre(input0, coef0);
        tempQ1 = L_mpycim(input0, coef0);
        tempI2 = L_mpyre(input1, coef1);
	    tempQ2 = L_mpycim(input1, coef1);
	
        input2 = input_int[i+2];
        input3 = input_int[i+3];
        coef2  = coef_int[i+2];
        coef3  = coef_int[i+3];

	    tempI3 = L_mpyre(input2, coef2);
	    tempQ3 = L_mpycim(input2, coef2);
        tempI4 = L_mpyre(input3, coef3);
	    tempQ4 = L_mpycim(input3, coef3);

        input0 = input_int[i+4];
        input1 = input_int[i+5];
        coef0  = coef_int[i+4];
        coef1  = coef_int[i+5];
    
        writer_4f((short*)&result_int1[i], tempI1, tempQ1, tempI2, tempQ2);
        writer_4f((short*)&result_int2[i], tempI3, tempQ3, tempI4, tempQ4);
	    
	}
	
	return 0;
}

