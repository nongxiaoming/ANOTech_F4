#ifndef _AK8975_H_
#define _AK8975_H_
#include "stm32f4xx.h"

u8 AK8975_Run(void);
u8 AK8975_Check1(void);
u8 AK8975_Check2(void);
u8 AK8975_Read(void);
void AK8975_Cal(void);
//void AK8975_GetValue(T_int16_xyz *mag);

#endif
