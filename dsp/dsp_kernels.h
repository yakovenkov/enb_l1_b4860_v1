/****************************************************************************
* SC3850 Core Libraries
* Freescale Semiconductor
* Copyright (C) 2008 All rights reserved
******************************************************************************
* File name: 	 sc3850_kernels_api.h
* Author name:   Yongqing Liang
*
* Description :  Header file for the sc3850 kernel lib
*
******************************************************************************
* Revision History
******************************************************************************
* $Log: sc3850_kernels_api.h,v $
* Revision 1.9  2010/08/04 20:14:11  b14270
* remove 256 point FFT
*
* Revision 1.8  2010/02/04 17:10:07  b14270
* Added Cholesky decomposition kernel
*
* Revision 1.7  2009/11/10 15:22:41  b14270
* Added vector dot product with direct vector input
*
* Revision 1.6  2009/06/22 21:05:54  b14270
* added Ln
*
* Revision 1.5  2009/06/19 20:02:54  b14270
* Added complex div
*
* Revision 1.4  2009/05/28 22:16:54  b14270
* Changed digital reverse array
*
* Revision 1.3  2009/05/18 20:36:17  b14270
* rebuild
*
* Revision 1.2  2009/04/30 21:41:25  b14270
* Added radix-2-4 FFT
*
* Revision 1.1  2009/04/30 16:09:01  b14270
* initial version
*
* Revision 1.26  2009/04/02 22:08:08  b14270
* Added interleave and 2x2 matrix inversion
*
* Revision 1.25  2009/04/02 15:24:54  b14270
* add 4x4 matrix scale C
*
* Revision 1.24  2009/04/02 15:09:06  b14270
* Added idft
*
* Revision 1.23  2009/03/23 21:28:32  b14270
* Defined new parameters for matrix operation
*
* Revision 1.22  2009/03/10 20:51:26  b14270
* Added QRD kernel
*
* Revision 1.21  2009/02/25 15:40:19  b14270
* change prototype of viterbi
*
* Revision 1.20  2009/02/24 22:04:27  b14270
* changed kernel names
*
* Revision 1.19  2009/01/26 23:30:00  b14270
* Added 4x4 matrix inverse (asm) and matrix mult (c and asm)
*
* Revision 1.18  2009/01/15 23:51:16  b14270
* Added DFT DIT ASM
*
* Revision 1.17  2008/12/25 04:38:47  r64909
* Added matrix lu decomposition, lu solve, inverse
*
* Revision 1.16  2008/12/03 10:09:41  r64909
* updated for real fft and ifft asm
*
* Revision 1.15  2008/11/26 16:16:28  b14270
* added viterbi asm
*
* Revision 1.14  2008/11/10 07:57:54  r64909
* Added real number16X16 radix-4 fft ifft
*
* Revision 1.13  2008/10/23 20:12:06  b14270
* added vector_complex mult
*
* Revision 1.12  2008/09/03 17:17:48  b14270
* Added fft_radix4 16x16 and 32x16 kernels
*
* Revision 1.11  2008/07/16 20:35:17  b14270
* Changed API of sc3850_freq_rot_c
*
* Revision 1.10  2008/07/02 22:33:28  b14270
* Changed API of fir complex 16x16
*
* Revision 1.9  2008/06/11 16:40:16  b14270
* Added APIs
*
* Revision 1.8  2008/05/28 16:04:24  b14270
* Added iir and freq_rot
*
* Revision 1.7  2008/05/19 15:41:12  b14270
* Added APIs
*
* Revision 1.6  2008/04/28 22:41:56  b14270
* Added header
*
****************************************************************************/

#ifndef __SC3850_KERNELS_API_H
#define __SC3850_KERNELS_API_H

#include "prototype.h"

/* FIR */
void sc3850_fir_real_16x16_asm(Word16 x[], Word16 h[], Word16 y[], Word16 nr, Word16 nh);
void sc3850_fir_real_32x32_asm(Word32 x[], Word32 h[], Word32 y[], Word16 Nr, Word16 Nh);
void sc3850_fir_real_8x16_asm(Word8 x[], Word16 h[], Word8 y[], Word16 nr, Word16 nh);
void sc3850_fir_real_16x16_c(Word16 x[], Word16 h[], Word16 y[], Word16 nr, Word16 nh);
void sc3850_fir_real_32x32_c(Word32 x[], Word32 h[], Word32 y[], Word16 Nr, Word16 Nh);
void sc3850_fir_real_8x16_c(Word8 x[], Word16 h[], Word8 y[], Word16 nr, Word16 nh);
void sc3850_fir_complex_16x16_c(Word32 x[], Word32 h[], Word16 y[], Word16 nr, Word16 nh);
void sc3850_fir_complex_16x16_asm(Word32 x[], Word32 h[], Word16 y[], Word16 nr, Word16 nh);
void fir_complex_16x16(Word16 *x, Word16 *h, Word16 *y, Word16 nr, Word16 nh);
/* CRC */
void sc3850_crc_enc_asm(unsigned short*,UWord32*,unsigned short,unsigned short);
void sc3850_crc_enc_c(unsigned short*,UWord32*,unsigned short,unsigned short);

/* FFT  */
void sc3850_fft_radix4_complex_32x16_asm(Word32 data_buffer[], Word16 wbtwiddles[], Word16 wctwiddles[], Word16 wdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_fft_radix4_complex_32x16_c(  Word32 data_buffer[], Word16 wbtwiddles[], Word16 wctwiddles[], Word16 wdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_ifft_radix4_complex_32x16_asm(Word32 data_buffer[], Word16 wbtwiddles[], Word16 wctwiddles[], Word16 wdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_ifft_radix4_complex_32x16_c(  Word32 data_buffer[], Word16 wbtwiddles[], Word16 wctwiddles[], Word16 wdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_fft_radix4_complex_16x16_asm(Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_fft_radix4_complex_16x16_c(  Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_ifft_radix4_complex_16x16_asm(Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_ifft_radix4_complex_16x16_c(  Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_fft_radix4_real_16x16_c(  Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_ifft_radix4_real_16x16_c(  Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_fft_radix4_real_16x16_asm(  Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_ifft_radix4_real_16x16_asm(  Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_fft_radix_2_4_complex_16x16_asm(Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_fft_radix_2_4_complex_16x16_c(  Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_ifft_radix_2_4_complex_16x16_asm(Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);
void sc3850_ifft_radix_2_4_complex_16x16_c(  Word16 data_buffer[], Word16 wctwiddles[], Word16 wbdtwiddles[], Word16 n, Word16 ln, Word16 Shift_down);


/* lookup */
Word16 sc3850_lookup_asm (const Word16* clb_in, Word16* clb_out, Word16 clb_length, const Word16* lookup_table);
Word16 sc3850_lookup_c (const Word16* clb_in, Word16* clb_out, Word16 clb_length, const Word16* lookup_table);

/* DIV */
// 16 / 16 -> 16
typedef struct div_arg_16x16_t{
	Word16	a;
	Word16	b;
}div_arg_16x16;
Word16 sc3850_div_16x16_c(div_arg_16x16 *arg);
Word16 sc3850_div_16x16_asm(div_arg_16x16 *arg);
// 32 / 32 -> 32
typedef struct div_arg_32x32_t
{
	Word32	a;
	Word32	b;
}div_arg_32x32;
Word32 sc3850_div_32x32_c(div_arg_32x32 *arg);
Word32 sc3850_div_32x32_asm(div_arg_32x32 *arg);
Word32 sc3850_div_32x32_lte_asm(div_arg_32x32 *arg);
Word32 sc3850_div_32x32_lte_c(div_arg_32x32 *arg);
// 32 / 16 -> 32
typedef struct div_arg_32x16_t
{
	Word32	a;
	Word16	b;
}div_arg_32x16;
Word32 sc3850_div_32x16_c(div_arg_32x16 *arg);
Word32 sc3850_div_32x16_asm(div_arg_32x16 *arg);
// 16 / 32 -> 32
typedef struct div_arg_16x32_t
{
	Word16	a;
	Word32	b;
}div_arg_16x32;
Word32 sc3850_div_16x32_c(div_arg_16x32 *arg);
Word32 sc3850_div_16x32_asm(div_arg_16x32 *arg);
//complex div 16/16 -> 16
void sc3850_complex_div_16x16_c(Complex16 *restrict in, Complex16 *restrict H, Complex16 *restrict out, int equalizedsbc);

/* square root */
Word32 sc3850_sqrt32_c(Word32 arg);
Word32 sc3850_sqrt32_asm(Word32 arg);
Word16 sc3850_sqrt16_c(Word16 arg);
Word16 sc3850_sqrt16_asm(Word16 arg);

/* frequency rotation */
void sc3850_freq_rot_c(Word32 x[], Word16 y[], Word16 g, Word16 Nr);
void sc3850_freq_rot_asm(Word32 x[], Word16 y[], Word16 g, Word16 Nr);

/* IIR */
typedef struct iir_1st_arg_t
{
	Word16	*y;
	Word16	*x;
	Word16	*c;
	Word16	*s;
	unsigned short M;
} iir_1st_arg;
void  sc3850_iir_1st_c( iir_1st_arg *pt);
void  sc3850_iir_1st_asm( iir_1st_arg *pt);

/* vector dot product */
//input vectors are taken out from matrices,
//which give more flexibility with the cost of more cycles.
void sc3850_vector_dot_product_16x16_c(Word16 *vec1, Word16 *vec2, Word32 length, Word32 incr1, Word32 incr2, Word32 res[2]);
void sc3850_vector_dot_product_16x16_asm(Word16 *vec1, Word16 *vec2, Word32 length, Word32 incr1, Word32 incr2, Word32 res[2]);
void sc3850_vector_dot_product_16x32_c(Word16 *vec1, Word32 *vec2, Word32 length, Word32 incr1, Word32 incr2, Word32 res[2]);
void sc3850_vector_dot_product_16x32_asm(Word16 *vec1, Word32 *vec2, Word32 length, Word32 incr1, Word32 incr2, Word32 res[2]);
void sc3900_vector_dot_product_16x16_c(Complex16 * restrict __attribute__((aligned(8))) vec1,
		Complex16 * restrict __attribute__((aligned(8))) vec2, Word32 length, Word32 res[2]);
//two direct input vectors, give min cycles
void sc3850_vector_dot_product_16x16_direct_asm(Word16 *vec1, Word16 *vec2, Word32 length, Word32 res[2]);
void sc3850_vector_dot_product_16x16_direct_c(Word16 *vec1, Word16 *vec2, Word32 length, Word32 res[2]);

/* vector complex multiply */
int sc3850_vector_complex_mult_c(short *restrict coef, short *restrict input, short *restrict result, int N);
int sc3850_vector_complex_mult_asm(short *restrict coef, short *restrict input, short *restrict result, int N);

// Viterbi decoder
void sc3850_viterbi_k9r2_dec_asm(unsigned long *, int);

/* matrix lu decompose */
#define MAX_ROW  10
#define MAX_COL  10
#define MIN_ROW  2
#define MIN_COL  2
Word16 sc3850_matrix_lu_decompose_c(Word32*, Word16*, Word16);
Word16 sc3850_matrix_lu_decompose_asm(Word32*, Word16*, Word16);

/* matrix lu solve */
void sc3850_matrix_lu_solve_c(Word32*, Word32*, Word16*, Word16);
void sc3850_matrix_lu_solve_asm(Word32*, Word32*, Word16*, Word16);

/* lu decomposition based matrix inverse */
Word16 sc3850_matrix_lu_inverse_c(Word32*, Word32*, Word16);
Word16 sc3850_matrix_lu_inverse_asm(Word32*, Word32*, Word16);

/* DFT  */
//-------------------------------------------------------------------------------
// Structure for DFT/IDFT processing
//-------------------------------------------------------------------------------
typedef struct fft_arg_t
{
   short *psiIn;             // Pointer to Input Buffer
   short *psiOut;            // Pointer to Output Buffer
   short *psiNumRadix;       // Pointer to the array of radix every stage
   short *psiNumButterfly;   // Pointer to the array of number of butterflies every subgroup
   short *psiNumSubgroup;    // Pointer to the array of number of subgroups every stage
   short *psiNumRadixOffset; // Pointer to the array of number of each radix in/out offset
   long  *psiDigitReversedAddress_absolute_address; // Pointer to the Absolute Digit Reversed Address
   short *psiWb;             // Pointer to the array of twiddle factor Wb
   short *psiWc;             // Pointer to the array of twiddle factor Wc
   short *psiWd;             // Pointer to the array of twiddle factor Wd
   short *psiWe;             // Pointer to the array of twiddle factor We
   short *psiDFTpoint;       // Pointer to the DFT point
   short *psiScale;          // Pointer to the scaling factor every butterfly stage
} fft_arg;
//-------------------------------------------------------------------------------
// Structure for radix butterfly calculation in C
//-------------------------------------------------------------------------------
typedef struct radix_arg_t
{
   short *psiIn;             // Pointer to Input Buffer
   short *psiOut;            // Pointer to Output Buffer
   short siNumRadix;         // Number of radix every stage
   short siNumButterfly;     // Number of butterflies every subgroup
   short siNumSubgroup;      // Number of subgroups every stage
   short siNumRadixOffset;   // Number of each radix in/out offset
   long  *psiDigitReversedAddress_absolute_address; // Pointer to the Absolute Digit Reversed Address
   short *psiWb;             // Pointer to the array of twiddle factor Wb
   short *psiWc;             // Pointer to the array of twiddle factor Wc
   short *psiWd;             // Pointer to the array of twiddle factor Wd
   short *psiWe;             // Pointer to the array of twiddle factor We
   short siNorm;             // Normalization value
   short *psiScale;          // Pointer to the scaling factor every butterfly stage
} radix_arg;
//-------------------------------------------------------------------------------
// Structure for determining normalization factor in C
//-------------------------------------------------------------------------------
typedef struct norm_arg_t
{
   short *psiIn;             // Pointer to Input Buffer
   short siDftPoint;         // DFT point
   short siNorm;             // Normalization value
} norm_arg;
void sc3850_dft_dit_complex_16x16_auto_scale_asm( fft_arg *psFit );
void sc3850_idft_dit_complex_16x16_auto_scale_asm( fft_arg *psFit );
//extern void sc3850_dft_complex_16x16_auto_scale_c( fft_arg *psFft );
//extern void sc3850_idft_complex_16x16_auto_scale_c( fft_arg *psFft );

/* 4x4 Matrix mult single precision */
void sc3850_cplx_matrix_mult_sp_4x4_c(Word32 *in1, Word32 *in2, Word16 *out);
void sc3850_cplx_matrix_mult_sp_4x4_asm(Word32 *in1, Word32 *in2, Word16 *out);

/* 4x4 Matrix inverse 16-bit input 32-bit output */
#define MATRIX_INPUT_SHIFT 3
void sc3850_matrix_inv_complex_4x4_c(const Complex16 *source, Word32 *output);
void sc3850_matrix_inv_complex_4x4_asm(const Complex16 * source, Word32 * output);
void sc3850_matrix_scale_c(const Complex16 * restrict pmatrix, Complex16 * restrict poutmatrix, Word32 shift);
Word32 sc3850_matrix_inv_complex_4x4_scale_c(const Complex16 * source, Word32 * output, Word16 * sf, Word32 detmin, Word32 input_shift);
Word32 sc3850_matrix_inv_complex_4x4_scale_asm(const Complex16 * source, Word32 * output, Word16 * sf, Word32 detmin, Word32 input_shift);

/* 2x2 Matrix inverse 16-bit input 16-bit output */
Complex16 sc3850_matrix_inverse_2x2_complex16_C(const Complex16 * input, Complex16 * output, Word16 * output_shift_left);
Complex16 sc3850_matrix_inverse_2x2_complex16_ASM(const Complex16 * input, Complex16 * output, Word16 * output_shift_left);
typedef struct matrix_inverse_2x2_complex16_vec2_t
{
	const Complex16 *input;
	Complex16 *output;
	Word32 output_shift_left;
	Complex16 detA_scaled;
}matrix_inverse_2x2_complex16_vec2_s;
void sc3850_matrix_inverse_2x2_complex16_vec2_C(matrix_inverse_2x2_complex16_vec2_s *arg);
void sc3850_matrix_inverse_2x2_complex16_vec2_ASM(matrix_inverse_2x2_complex16_vec2_s *arg);


/* 4x4 Matrix QRD */
#define ROW  4                         // For ROW equals row
#define COLUMN 4
typedef struct
{
    Word32 real;
	Word32 imag;
} X32_complex , *p_X32_complex;
void sc3850_matrix_QRD_c(X32_complex DataIn[ROW][COLUMN],X32_complex DataOut_R[ROW][COLUMN],X32_complex DataOut_Q[ROW][COLUMN]);
void sc3850_matrix_QRD_asm(X32_complex DataIn[ROW][COLUMN],X32_complex DataOut_R[ROW][COLUMN],X32_complex DataOut_Q[ROW][COLUMN]);

// 3_WAY_DEINTERLEAVE and INTERLEAVE
void sc3850_3_WAY_DEINTERLEAVE_C( const UWord8 *Input, UWord8 *Output_A, UWord8 *Output_B, UWord8 *Output_C, Word32 Length);
void sc3850_3_WAY_DEINTERLEAVE_ASM(const UWord8 *Input, UWord8 *Output_A, UWord8 *Output_B, UWord8 *Output_C, Word32 Length);
void sc3850_3_WAY_INTERLEAVE_C(const UWord8 *Input_A, const UWord8 *Input_B, const UWord8 *Input_C, UWord8 *Output, Word32 Length);
void sc3850_3_WAY_INTERLEAVE_ASM(const UWord8 *Input_A, const UWord8 *Input_B, const UWord8 *Input_C, UWord8 *Output, Word32 Length);

//Ln
typedef struct ln_arg_t{
	long	*X;
	long	*Y;
	unsigned short	n;
}ln_arg;
void sc3850_ln_c(ln_arg *data_in);

// Cholesky decomposition
int sc3850_cholesky_c(Word32 *P,
              Word16 *P_sf,
              Word16 *k,
              Word16 *k_sf,
              Word16 *L,
              Word32 *d_inv,
              Word16 *d_inv_sf,
              Word32 *detP,
              Word16 *det_P_sf,
              Word16 *x,
              Word16 *x_sf,
              Word16 *Pinv,
              Word16 *Pinv_sf,
              Word16 dim1,
              Word16 mode);


#define fast_memcpy_align16(dst, src, len) \
		{ \
			uint32_t * restrict src32 = (uint32_t *)src; \
			uint32_t * restrict dst32 = (uint32_t *)dst; \
			\
			for(int32_t i=0; i< (len) / (16 * 4); i++) \
			{ \
				Word32 dc[16]; \
				__ld_16l(src32, &dc[0], &dc[1], &dc[2], &dc[3], &dc[4], &dc[5], &dc[6], &dc[7], \
						&dc[8], &dc[9], &dc[10], &dc[11], &dc[12], &dc[13], &dc[14], &dc[15]); \
				__st_16l(dst32, dc[0], dc[1], dc[2], dc[3], dc[4], dc[5], dc[6], dc[7], \
						dc[8], dc[9], dc[10], dc[11], dc[12], dc[13], dc[14], dc[15]); \
			 \
			 	 src32 += 16; \
			 	 dst32 += 16; \
			} \
		}

#endif // __SC3850_KERNELS_API_H
