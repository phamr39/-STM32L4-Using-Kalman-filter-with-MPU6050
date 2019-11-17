/**************************************************************************//**
 * @file     CTR_ADXL345.h
 * @Deverloper	lochoang360			    
 *           
 * @version  V0.1
 * @date     14. Oct 2016
 *
 * @note 	support for GY85 Module
 *
 ******************************************************************************/
/* Copyright (c) 2016 LINH KIEN 3M*/

#ifndef _CTR_ADXL345_
#define _CTR_ADXL345_
#include "CTR_I2C.h"
#define	CTR_ADXL345_Addr    0xA6
void  CTR_Init_ADXL345(void);
int * CTR_Read_ADXL345(void);
#endif
