#ifndef __BSP_RGB_H
#define __BSP_RGB_H

#include "main.h"

void RGB_Init(void);
void RGB_Test_Blink(void);
void RGB_Clear(void);
void RGB_Set_All(uint8_t r, uint8_t g, uint8_t b);

#endif