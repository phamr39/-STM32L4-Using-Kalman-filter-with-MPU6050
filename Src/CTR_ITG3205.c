/**************************************************************************//**
 * @file     CTR_ITG3205.c
 * @Deverloper	lochoang360			    
 *           
 * @version  V0.1
 * @date     14. Oct 2016
 *
 * @note 	support for GY85 Module
 *
 ******************************************************************************/
/* Copyright (c) 2016 LINH KIEN 3M*/
#include "CTR_ITG3205.h"
unsigned char BUF[10];       		 
short T_X,T_Y,T_Z,T_T;	

void CTR_Init_ITG3205(void)
{
   CTR_Single_Write(CTR_ITG3205_Addr,CTR_PWR_M, 0x80);   
   CTR_Single_Write(CTR_ITG3205_Addr,CTR_SMPL, 0x07);    
   CTR_Single_Write(CTR_ITG3205_Addr,CTR_DLPF, 0x1E);    
   CTR_Single_Write(CTR_ITG3205_Addr,CTR_INT_C, 0x00 );  
   CTR_Single_Write(CTR_ITG3205_Addr,CTR_PWR_M, 0x00);   
}

short * CTR_READ_ITG3205(void)
{
	static short value[4];
   BUF[0]=CTR_Single_Read(CTR_ITG3205_Addr,CTR_GX_L); 
   BUF[1]=CTR_Single_Read(CTR_ITG3205_Addr,CTR_GX_H);
   T_X=	(BUF[1]<<8)|BUF[0];
   value[0]=T_X/14.375; 						   

   BUF[2]=CTR_Single_Read(CTR_ITG3205_Addr,CTR_GY_L);
   BUF[3]=CTR_Single_Read(CTR_ITG3205_Addr,CTR_GY_H);
   T_Y=	(BUF[3]<<8)|BUF[2];
   value[1]=T_Y/14.375; 						   
   BUF[4]=CTR_Single_Read(CTR_ITG3205_Addr,CTR_GZ_L);
   BUF[5]=CTR_Single_Read(CTR_ITG3205_Addr,CTR_GZ_H);
   T_Z=	(BUF[5]<<8)|BUF[4];
   value[2]=T_Z/14.375; 					       

   BUF[6]=CTR_Single_Read(CTR_ITG3205_Addr,CTR_TMP_L); 
   BUF[7]=CTR_Single_Read(CTR_ITG3205_Addr,CTR_TMP_H); 
   T_T=(BUF[7]<<8)|BUF[6];
   value[3] = (short)(35+ ((double) (T_T + 13200)) / 280);
	return value;
}
