#include <ctype.h>    // For isspace
#include <stdarg.h>   // For va_list, va_start, va_end
#include <string.h>   // For strlen, strcmp, etc.
#include <stdio.h>    // For vsnprintf
#include <stdlib.h>   // For atoi
#include <stdint.h>   // For uint8_t

#include "main.h"     // Declares huart1
#include "bsp.h"      // For Debug_Print macro
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "bsp_motion.h"
#include "bsp_encoder.h"

// Extern declarations for hardware handles
extern UART_HandleTypeDef huart1;

#define CMD_BUFFER_SIZE 128
static char uart_cmd_buffer[CMD_BUFFER_SIZE];
static volatile uint8_t uart_cmd_index = 0;
uint8_t RxTemp = 0;

// This function is called by Debug_Print macro if DEBUG=1
void Debug_Print(const char* format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Example init function for USART1
void USART1_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &RxTemp, 1);
}


void ProcessCommandLine(char *cmd)
{
    // Let’s read up to 5 tokens: e.g. "D FWD 1200"
    char token1[8], token2[8], token3[8], token4[8], token5[8];
    memset(token1, 0, sizeof(token1));
    memset(token2, 0, sizeof(token2));
    memset(token3, 0, sizeof(token3));
    memset(token4, 0, sizeof(token4));
    memset(token5, 0, sizeof(token5));

    // Example: V 200 200 200 200
    // will parse into token1="V", token2="200", token3="200", ...
    int count = sscanf(cmd, "%7s %7s %7s %7s %7s",
                       token1, token2, token3, token4, token5);

    if (count < 1) {
        Debug_Print("Invalid command: %s\r\n", cmd);
        return;
    }

    if (strcmp(token1, "V") == 0)
    {
        // V <vx> <vy> <vz>
        if (count < 4) {
            Debug_Print("Not enough args for velocity command\r\n");
            return;
        }
        int vx = atoi(token2);
        int vy = atoi(token3);
        int vz = atoi(token4);

        Debug_Print("Calling Motion_Ctrl with vx=%d, vy=%d, vz=%d\r\n", vx, vy, vz);
        Motion_Ctrl(vx, vy, vz);
    }
    else if (strcmp(token1, "D") == 0)
    {
        // D <direction> <speed>
        if (count < 3) {
            Debug_Print("Not enough args for direction command\r\n");
            return;
        }
        char *direction = token2;  // e.g. "FWD"
        int speed = atoi(token3);  // e.g. 1200
        Handle_Directional(direction, speed);
    }
    else if (strcmp(token1, "I") == 0)
    {
        // I <query>
        if (count < 2) {
            Debug_Print("Not enough args for info command\r\n");
            return;
        }
        if (strcmp(token2, "ENC") == 0) {
            Handle_Info_Encoders();
        } else if (strcmp(token2, "BAT") == 0) {
            Handle_Info_Battery();
        } else {
            Debug_Print("Unknown info query: %s\r\n", token2);
        }
    }
    else
    {
        Debug_Print("Unknown command type '%s'\r\n", token1);
    }
}


// UART RX callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Debug_Print("UART Rx: 0x%02X ('%c')\r\n",
                    RxTemp, (RxTemp >= 32 && RxTemp <= 126) ? RxTemp : '.');

        if (RxTemp == '\n' || RxTemp == '\r') {
            uart_cmd_buffer[uart_cmd_index] = '\0';
            ProcessCommandLine(uart_cmd_buffer);
            uart_cmd_index = 0;
        } else {
            if (uart_cmd_index < CMD_BUFFER_SIZE - 1) {
                uart_cmd_buffer[uart_cmd_index++] = RxTemp;
            } else {
                // Optional: handle overflow
                uart_cmd_index = 0;
            }
        }
        HAL_UART_Receive_IT(&huart1, &RxTemp, 1);
    }
}
