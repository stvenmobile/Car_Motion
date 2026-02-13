#include "bsp.h"
#include <stdlib.h>
#include "bsp_uart.h"
#include "bsp_beep.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

char cmd_buffer[64];
uint8_t cmd_index = 0;
uint8_t RxTemp = 0;
extern motor_data_t motor_data;



/* --- INITIALIZATION --- */
void USART1_Init(void)
{
    // Start interrupt-based reception
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
}

/* --- UART INTERRUPT CALLBACK --- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (RxTemp != '\r' && RxTemp != '\n')
        {
            if (cmd_index < sizeof(cmd_buffer) - 1)
            {
                cmd_buffer[cmd_index++] = RxTemp;
            }
        }
        else if (cmd_index > 0)
        {
            cmd_buffer[cmd_index] = '\0';
            ProcessCommandLine(cmd_buffer);
            cmd_index = 0;
        }
        
        RxTemp = 0;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
    }
}

/* --- COMMAND PROCESSOR --- */
void ProcessCommandLine(char *cmd)
{
    char token1[8], token2[8], token3[8], token4[8];
    
    int count = sscanf(cmd, "%7s %7s %7s %7s", token1, token2, token3, token4);
    if (count < 1) return;

    if (strcmp(token1, "V") == 0 && count >= 4)
    {
        Motion_Ctrl(atoi(token2), atoi(token3), atoi(token4));
    }
    else if (strcmp(token1, "I") == 0 && count >= 2)
    {
        if (strcmp(token2, "ENC") == 0) Handle_Info_Encoders();
        else if (strcmp(token2, "RESET") == 0) Handle_Info_ResetEncoders();
        else if (strcmp(token2, "BAT") == 0) Handle_Info_Battery();
    }
    // ... add T and D handlers if needed ...
}

/* Helper for printf redirection if you ever use standard printf() */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

/* Required for bsp_pid.c and linker compatibility */
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
    if (Length > 0)
    {
        HAL_UART_Transmit(&huart1, BufferPtr, Length, 1000);
    }
}
