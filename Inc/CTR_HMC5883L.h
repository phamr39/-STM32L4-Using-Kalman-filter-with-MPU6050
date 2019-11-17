/**************************************************************************//**
 * @file     CTR_HMC5883L.h
 * @Deverloper	lochoang360			    
 *           
 * @version  V0.1
 * @date     14. Oct 2016
 *
 * @note 	support for GY85 Module
 *
 ******************************************************************************/
/* Copyright (c) 2016 LINH KIEN 3M*/

#ifndef _CTR_HMC5883L_
#define _CTR_HMC5883L_
#include "CTR_I2C.h"
#include "math.h"
#define	CTR_HMC5883L_Addr   0x3C
void CTR_Init_HMC5883L(void);
float CTR_Read_hmc5883l(void);
#endif
