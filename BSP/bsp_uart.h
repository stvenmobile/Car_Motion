#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "stdint.h"
#include "bsp.h"
#include "bsp_motion.h"
#include "bsp_pid.h"

/* --- External Hardware Handles --- */
// This tells the compiler that huart1 is defined in usart.c
extern UART_HandleTypeDef huart1;

/* --- Function Prototypes --- */
void USART1_Init(void);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);

void ProcessCommandLine(char *cmd);
void Command_Handler(void);

/* Information Query Handlers */
void Handle_Info_Encoders(void);
void Handle_Info_Battery(void);
void Handle_Info_PID(motor_data_t* motor);

/* Data Accessors */
int16_t GetLatestVelocityX(void);

#endif /* __BSP_UART_H */