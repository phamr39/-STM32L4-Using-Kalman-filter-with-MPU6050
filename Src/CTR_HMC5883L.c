/**************************************************************************//**
 * @file     CTR_HMC5883L.c
 * @Deverloper	lochoang360			    
 *           
 * @version  V0.1
 * @date     14. Oct 2016
 *
 * @note 	support for GY85 Module
 *
 ******************************************************************************/
/* Copyright (c) 2016 LINH KIEN 3M*/
#include "CTR_HMC5883L.h"

uint8_t CTR_HMC5883L_BUF[10];
int   x,y;

void CTR_Init_HMC5883L(void)
{
   CTR_Single_Write(CTR_HMC5883L_Addr,0x00,0x14);   
   CTR_Single_Write(CTR_HMC5883L_Addr,0x02,0x00);   
}

float CTR_Read_hmc5883l(void)
{
	static float angle;
       CTR_Single_Write(CTR_HMC5883L_Addr,0x00,0x14);   
       CTR_Single_Write(CTR_HMC5883L_Addr,0x02,0x00);   
  	   HAL_Delay(10);

       CTR_HMC5883L_BUF[1]=CTR_Single_Read(CTR_HMC5883L_Addr,0x03);
       CTR_HMC5883L_BUF[2]=CTR_Single_Read(CTR_HMC5883L_Addr,0x04); // x
			 CTR_HMC5883L_BUF[5]=CTR_Single_Read(CTR_HMC5883L_Addr,0x05);
       CTR_HMC5883L_BUF[6]=CTR_Single_Read(CTR_HMC5883L_Addr,0x06); // z
			 CTR_HMC5883L_BUF[3]=CTR_Single_Read(CTR_HMC5883L_Addr,0x07);
       CTR_HMC5883L_BUF[4]=CTR_Single_Read(CTR_HMC5883L_Addr,0x08); // y
	

       x=(CTR_HMC5883L_BUF[1] << 8) | CTR_HMC5883L_BUF[2]; 
       y=(CTR_HMC5883L_BUF[3] << 8) | CTR_HMC5883L_BUF[4]; 

       if(x>0x7fff)x-=0xffff;	  
       if(y>0x7fff)y-=0xffff;	  
       angle= (float)(atan2((double)y,(double)x) * (180 / 3.14159265) + 180); 
	return angle;
}
