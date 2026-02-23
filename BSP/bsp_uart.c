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
#include "bsp_motor.h"

// Prototype to clear IDE warnings
void ProcessCommandLine(char *cmd);

// External variables from other modules
extern uint8_t g_start_ctrl; 
extern motor_data_t motor_data;
extern float g_last_loop_time;
extern uint32_t g_last_cmd_tick;

char cmd_buffer[64];
static char cmd_pending[64];
static volatile uint8_t cmd_ready = 0;
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
                if (!cmd_ready)
                {
                    // Safe handoff: copy complete command and set flag for main-loop processing.
                    // ProcessCommandLine must NOT be called here — this is ISR context and
                    // USART1 is at priority 0, so HAL_Delay / printf inside it would deadlock.
                    cmd_buffer[cmd_index] = '\0';
                    memcpy(cmd_pending, cmd_buffer, cmd_index + 1);
                    cmd_ready = 1;
                }
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

    // Handle Speed Commands (e.g., V 100 0 0)
    if (strcmp(token1, "V") == 0 && count >= 4)
    {
        Motion_Ctrl(atoi(token2), atoi(token3), atoi(token4));
    }
    // Handle Information and Test Commands
    else if (strcmp(token1, "I") == 0 && count >= 2)
    {
        if (strcmp(token2, "ENC") == 0)
        {
            Handle_Info_Encoders();
        }
        else if (strcmp(token2, "RESET") == 0)
        {
            Handle_Info_ResetEncoders();
            printf("ACK: Encoders Reset to Zero\r\n");
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
            printf("DEBUG: PID Kp is %.2f (Should be 1.80)\r\n", pid_motor[0].Kp);
            printf("NON-BLOCKING TEST START: 500 mm/s\r\n");

            g_start_ctrl = 1;
            g_last_cmd_tick = HAL_GetTick();

            Motion_Ctrl(500, 0, 0);
        }
        else if (strcmp(token2, "DIAG") == 0)
        {
            Motor_Diagnostic_Sequence();
        }
        //*  **The Isolation Test**:
        //* Unplug all 4 motor cables from the expansion board.
        //* Send the `DIAG` command.
        //* If the garbage still appears, the issue is internal to the MCU configuration (Timer/Pin conflict).
        //* If the garbage disappears, plug in the motors **one at a time** and repeat the test.
    }
    // Handle Live PID Tuning Commands (e.g., P 1.5 0.5 0.05)
    else if (strcmp(token1, "P") == 0 && count >= 4)
    {
        float new_kp = (float)atof(token2);
        float new_ki = (float)atof(token3);
        float new_kd = (float)atof(token4);

        // Update all 4 motors and reset their integral sums to prevent jumps
        for(int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].Kp = new_kp;
            pid_motor[i].Ki = new_ki;
            pid_motor[i].Kd = new_kd;
            pid_motor[i].integral = 0.0f;
        }
        printf("ACK: PID updated - KP:%.2f KI:%.2f KD:%.2f\r\n", new_kp, new_ki, new_kd);
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

/**
 * @brief  Poll for a pending serial command and dispatch it.
 * @note   Must be called from the main loop (non-ISR context) so that
 *         printf, HAL_Delay, and other blocking calls inside handlers are safe.
 */
void Command_Handler(void)
{
    if (cmd_ready)
    {
        cmd_ready = 0;
        ProcessCommandLine(cmd_pending);
    }
}

void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
    if (Length > 0)
    {
        HAL_UART_Transmit(&huart1, BufferPtr, Length, 1000);
    }
}
