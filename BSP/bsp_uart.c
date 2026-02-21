#include "main.h"     // Essential for HAL and basic types
#include "bsp.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include "bsp_uart.h"
#include "bsp_beep.h"
#include "usart.h"
#include "bsp_icm20948.h"
#include "bsp_motion.h"

// Prototype to clear IDE warnings
void ProcessCommandLine(char *cmd);

// External variables from other modules
extern uint8_t g_start_ctrl; 
extern motor_data_t motor_data;
extern float g_last_loop_time;
extern uint32_t g_last_cmd_tick;

char cmd_buffer[64];
uint8_t cmd_index = 0;
uint8_t RxTemp = 0;

/* --- INITIALIZATION --- */
void USART1_Init(void)
{
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
}

/* --- ROBUST UART INTERRUPT CALLBACK --- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (RxTemp == '\r' || RxTemp == '\n')
        {
            if (cmd_index > 0) 
            {
                cmd_buffer[cmd_index] = '\0';
                ProcessCommandLine(cmd_buffer);
                cmd_index = 0; 
            }
        }
        else
        {
            if (cmd_index < sizeof(cmd_buffer) - 1)
            {
                cmd_buffer[cmd_index++] = RxTemp;
            }
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
        if (strcmp(token2, "ENC") == 0)
        {
            Handle_Info_Encoders();
        }
        else if (strcmp(token2, "IMU") == 0)
        {
            printf("IMU: A(%.2f, %.2f, %.2f) G(%.2f, %.2f, %.2f) Yaw: %.2f\r\n",
                   g_imu_accel.x, g_imu_accel.y, g_imu_accel.z,
                   g_imu_gyro.x, g_imu_gyro.y, g_imu_gyro.z,
                   g_current_yaw);
        }
        else if (strcmp(token2, "TEST") == 0)
        {
            // Print current Kp to verify PID_Param_Init worked
            printf("DEBUG: PID Kp is %.2f (Should be 1.80)\r\n", pid_motor[0].Kp);
            printf("NON-BLOCKING TEST START: 500 mm/s\r\n");

            g_start_ctrl = 1;                // Unlock the PID logic
            g_last_cmd_tick = HAL_GetTick(); // Reset watchdog timer

            Motion_Ctrl(500, 0, 0);          // Set the target
        }
    }
}

/* Redirection for printf support */
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

void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
    if (Length > 0)
    {
        HAL_UART_Transmit(&huart1, BufferPtr, Length, 1000);
    }
}
