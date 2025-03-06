#ifndef __BSP_H__
#define __BSP_H__

/* Import HAL related library  导入HAL相关库 */
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "stm32f1xx_hal.h"
#include "stm32f103xe.h"
#include "bsp_beep.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include "bsp_motion.h"
#include "bsp_pid.h"
#include "stdio.h"


/* DEFINE */
#define LED_ON()         HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET)
#define LED_OFF()        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET)
#define LED_TOGGLE()     HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)

// Global variable for UART data reception
extern uint8_t RxTemp;

// Set to 1 to enable debug prints, 0 to disable

/* functions */
void Bsp_Init(void);
void Bsp_Loop(void);


#endif /* __BSP_H__ */
