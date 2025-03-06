/*
 * bsp_uart.h
 *
 *  Created on: Mar 4, 2022
 *      Author: Administrator
 */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "stdint.h"

// Function Prototypes
// bsp_uart.h
#undef DEBUG          // Undefine any previous definition
#define DEBUG 0       // Change to 1 to enable debug prints

void UART_StartReception(void);
void USART1_Init(void);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
void ProcessCommandLine(char *cmd);
void Command_Handler(void);
void Handle_Info_Encoders();
void Handle_Info_Battery();
void Debug_Print(const char *format, ...);



#endif /* __BSP_UART_H */

