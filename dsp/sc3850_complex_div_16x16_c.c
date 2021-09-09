/****************************************************************************
* SC3850 Core Libraries
* Freescale Semiconductor
* Copyright (C) 2009 All rights reserved
******************************************************************************
* File name: 	 sc3850_complex_div_16x16_c.c
* Function name: sc3850_complex_div_16x16_c
* Author name:   Avi Gal
* Target Processor: StarCore SC3850
*
* Description : Complex division, Complex 16-bit signed inputs and output.
*
* Input: 	
*			Complex16 in[]		// input y
*			Complex16 H[]		// input H
* 			int equalizedsbc 	// Vector length
* Output:	
*			Complex16 out		// output x - complex
*
* CYCLES:		
*			8N + 32  
* Restritions:  
*			Big endian (Little endian not checked).
* 			The intention of the code is to solve the equation in = h*x (in < h).
* 			From DSP point of view they want to solve the equation y = h*x 
* 			for x (y,h,x are complex samples). Whereas 'y' is a received modulation 
* 			symbol and 'h' is a channel coefficient coming from the channel estimation.
* 
*
* Assumptions:
*			N is a multiply of 4
*			in, H and out are 8 bytes aligned
* 
******************************************************************************
* Revision History
******************************************************************************
* $Log: sc3850_complex_div_16x16_c.c,v $
* Revision 1.2  2009/07/08 16:41:59  b14270
* added comments
*
* Revision 1.1  2009/06/19 20:02:42  b14270
* Added complex div
*
* Revision 1.1  2009/06/09 22:34:53  b09074
* initial version
*
****************************************************************************/
#pragma opt_level = "O4"
#include <prototype.h> 
void sc3850_complex_div_16x16_c(Complex16 *restrict in, Complex16 *restrict H, Complex16 *restrict out, int equalizedsbc);

void sc3850_complex_div_16x16_c(Complex16 *restrict in, Complex16 *restrict H, Complex16 *restrict out, int equalizedsbc)
{
	//Complex16 x, H00;
	//Word32 determinant, num[4], inv_det2, yr, yi;
	//Word16 *y, nlb[4], det16[4], inv_det[4];
	//int j, k, p, sbc;
	uint32_t __SR__;
	uint32_t newSR;
	Complex16 x0, x1, x2, x3, h0, h1, h2, h3;
	Word40 h20, h21, h22, h23, y0r, y0i, y1r, y1i, y2r, y2i, y3r, y3i; 
	Word32 num0, num1, num2, num3;
	Word16 *y, nlb0, nlb1, nlb2, nlb3;
	int sbc, p;
	
	//writeSR(0x00e400A8);			// upscaling by 1 ON, SR=0, SR2=1, two's-complement rounding, W20-bits mode OFF
	
	// SC3850 version
	//asm("	bmclr #$1ff7,sr.l	;// upscaling by 1 ON, SR=0, SR2=1, two's-complement rounding, W20-bits mode OFF");
	//asm("	bmset #$00a8,sr.l	;// upscaling by 1 ON, SR=0, SR2=1, two's-complement rounding, W20-bits mode OFF");

	// SC3900 version
	__SR__ = readSR();
	newSR = (__SR__ & (~0x00001ff7)) | 0x000000a8;
	writeSR(newSR);

	cw_assert((int)in % 8 == 0);
	cw_assert((int)H % 8 == 0);	
	cw_assert(equalizedsbc >= 4 && (equalizedsbc % 4) == 0);
	y = (Word16 *) out;
	for(sbc=0;sbc<equalizedsbc;sbc+=4)
	{      
        h0 = H[sbc];
        h1 = H[sbc+1];
        h2 = H[sbc+2];
        h3 = H[sbc+3];
        h20 = X_mpyd(h0, h0);
        h21 = X_mpyd(h1, h1);
        h22 = X_mpyd(h2, h2);
        h23 = X_mpyd(h3, h3);
        
        nlb0 = X_neg_norm(h20);
        nlb1 = X_neg_norm(h21);
        nlb2 = X_neg_norm(h22);
        nlb3 = X_neg_norm(h23);
        
        h20 = X_shr(h20,nlb0);
        h21 = X_shr(h21,nlb1);
        h22 = X_shr(h22,nlb2);
        h23 = X_shr(h23,nlb3);
        
        num0 = 0x3FFF0000;
        num1 = 0x3FFF0000;
        num2 = 0x3FFF0000;
        num3 = 0x3FFF0000;
        for(p=0;p<16;p++)
        {
            #pragma loop_unroll 16
            num0 = _divp0(num0,X_extract_h(h20));
            num1 = _divp1(num1,X_extract_h(h21));
            num2 = _divp2(num2,X_extract_h(h22));
            num3 = _divp3(num3,X_extract_h(h23));
        }
        
        num0 = num0 & 0x00007FFF;
        num1 = num1 & 0x00007FFF;
        num2 = num2 & 0x00007FFF;
        num3 = num3 & 0x00007FFF;
       	
       	num0 = V_tfrw_lh(num0,num0);
       	num1 = V_tfrw_lh(num1,num1);
       	num2 = V_tfrw_lh(num2,num2);
       	num3 = V_tfrw_lh(num3,num3);
       	
       	h0 = V_L_mpy2(num0, h0);
      	h1 = V_L_mpy2(num1, h1);
       	h2 = V_L_mpy2(num2, h2);
      	h3 = V_L_mpy2(num3, h3);
       	x0 = in[sbc];
       	x1 = in[sbc+1];
       	
        y0r = X_mpyd(x0, h0);
        y0i = X_mpycim(x0, h0);
        y1r = X_mpyd(x1, h1);
        y1i = X_mpycim(x1, h1);        
        
      	y0r = X_shr(y0r, nlb0);
     	y0i = X_shr(y0i, nlb0);
      	y1r = X_shr(y1r, nlb1);
     	y1i = X_shr(y1i, nlb1);
        x2 = in[sbc+2];
       	x3 = in[sbc+3];
       	
       	//SC3850
     	//writer_4fw(y, y0r, y0i, y1r, y1i);
       	
       	//SC3900
     	__st_srs_4f(y, y0r, y0i, y1r, y1i);
     	
     	//very old code 
     	//writer_4f(y, X_trunc(y0r), X_trunc(y0i), X_trunc(y1r), X_trunc(y1i));
     	y+=4;
        y2r = X_mpyd(x2, h2);
        y2i = X_mpycim(x2, h2);
        y3r = X_mpyd(x3, h3);
        y3i = X_mpycim(x3, h3);
 
      	y2r = X_shr(y2r, nlb2);
     	y2i = X_shr(y2i, nlb2);
      	y3r = X_shr(y3r, nlb3);
     	y3i = X_shr(y3i, nlb3);

     	//SC3850
     	//writer_4fw(y, y2r, y2i, y3r, y3i);
     	
     	//SC3900
     	__st_srs_4f(y, y2r, y2i, y3r, y3i);
     	
     	// very old code
     	//writer_4f(y, X_trunc(y2r), X_trunc(y2i), X_trunc(y3r), X_trunc(y3i));
     	
     	y+=4;       
    }
	
	// Restore original SR
	writeSR(__SR__);
}


