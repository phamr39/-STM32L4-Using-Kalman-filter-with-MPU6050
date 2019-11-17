/**************************************************************************//**
 * @file     CTR_MPU6050.h
 * @Deverloper	lochoang360			    
 *           
 * @version  V0.1
 * @date     14. Oct 2016
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2016 LINH KIEN 3M*/
#ifndef _CTR_MPU6050_
#define _CTR_MPU6050_
#include "CTR_I2C.h"
#include "math.h"
#define CTR_WHO	    0x00
#define	CTR_SMPL	0x15
#define CTR_DLPF	0x16
#define CTR_INT_C	0x17
#define CTR_INT_S	0x1A
#define	CTR_TMP_H	0x1B
#define	CTR_TMP_L	0x1C
#define	CTR_GX_H	0x1D
#define	CTR_GX_L	0x1E
#define	CTR_GY_H	0x1F
#define	CTR_GY_L	0x20
#define CTR_GZ_H	0x21
#define CTR_GZ_L	0x22
#define CTR_PWR_M	0x3E

#define	CTR_MPU3050_Addr   0xD0	  
void CTR_Init_MPU3050(void);
float * CTR_READ_MPU3050(void);

#endif