/**************************************************************************//**
 * @file     CTR_ADXL345.c
 * @Deverloper	lochoang360			    
 *           
 * @version  V0.1
 * @date     14. Oct 2016
 *
 * @note 	support for GY85 Module
 *
 ******************************************************************************/
/* Copyright (c) 2016 LINH KIEN 3M*/
#include "CTR_ADXL345.h"
unsigned char CTR_ADXL345_BUF[8];

void  CTR_Init_ADXL345(void)
{
   CTR_Single_Write(CTR_ADXL345_Addr,0x31,0x0B);    
   CTR_Single_Write(CTR_ADXL345_Addr,0x2D,0x08);   
   CTR_Single_Write(CTR_ADXL345_Addr,0x2E,0x80); 	
}

int * CTR_Read_ADXL345(void)
{
	static int value[3];
	CTR_ADXL345_BUF[0]=CTR_Single_Read(CTR_ADXL345_Addr,0x32);
	CTR_ADXL345_BUF[1]=CTR_Single_Read(CTR_ADXL345_Addr,0x33);

	CTR_ADXL345_BUF[2]=CTR_Single_Read(CTR_ADXL345_Addr,0x34);
	CTR_ADXL345_BUF[3]=CTR_Single_Read(CTR_ADXL345_Addr,0x35);

	CTR_ADXL345_BUF[4]=CTR_Single_Read(CTR_ADXL345_Addr,0x36);
	CTR_ADXL345_BUF[5]=CTR_Single_Read(CTR_ADXL345_Addr,0x37);

	value[0]=(CTR_ADXL345_BUF[1]<<8)+CTR_ADXL345_BUF[0];  
	value[1]=(CTR_ADXL345_BUF[3]<<8)+CTR_ADXL345_BUF[2];  
	value[2]=(CTR_ADXL345_BUF[5]<<8)+CTR_ADXL345_BUF[4];  
	return value;
}
