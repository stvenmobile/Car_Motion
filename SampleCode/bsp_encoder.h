#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include "stdint.h"


// One full turn of the wheel, the number of pulses picked up by the coder: 55*11*2*2
#define ENCODER_CIRCLE           (2420)

void Encoder_Init(void);
void Encoder_Update_Count(void);
int Encoder_Get_Count_Now(uint8_t Motor_id);
void Encoder_Get_ALL(int* Encoder_all);

#endif

