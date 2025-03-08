#include "bsp.h"
#include <stdlib.h>
#include "bsp_uart.h"
#include "bsp_beep.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>


#define ENABLE_UART_DMA    0
#define PRINTF_BUFFER_SIZE 256

// Command buffer and index (Global Scope)
char cmd_buffer[64];
uint8_t cmd_index = 0;
static bool backup_beep_enabled = true;  // Default: Beep is ON

extern motor_data_t motor_data;  // Ensure motor_data is declared globally



// Debug Print Function
void Debug_Print(const char *format, ...)
{
    #if DEBUG
    va_list args;
    va_start(args, format);

    // Format the string
    vsnprintf(printf_buffer, PRINTF_BUFFER_SIZE, format, args);

    // Transmit the formatted string over USART1
    HAL_UART_Transmit(&huart1, (uint8_t *)printf_buffer, strlen(printf_buffer), HAL_MAX_DELAY);

    va_end(args);
    #endif
}


uint8_t RxTemp = 0;

// Initialize USART1
void USART1_Init(void)
{
    // Enable UART Interrupt reception
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
}


// The serial port sends one byte
void USART1_Send_U8(uint8_t ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
}

// The serial port sends a string of data
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
    #if ENABLE_UART_DMA
    HAL_UART_Transmit_IT(&huart1, BufferPtr, Length);
    #else
    while (Length--)
    {
        USART1_Send_U8(*BufferPtr);
        BufferPtr++;
    }
    #endif
}

// The serial port receiving is interrupted. Procedure
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    // Directly store the received byte in the buffer
    if (RxTemp != '\r' && RxTemp != '\n')
    {
        cmd_buffer[cmd_index++] = RxTemp;

        // Prevent buffer overflow
        if (cmd_index >= sizeof(cmd_buffer) - 1)
        {
            Debug_Print("Command buffer overflow, resetting.\r\n");
            cmd_index = 0;
        }
    }
    else
    {
        // End of command, process it
        cmd_buffer[cmd_index] = '\0'; // Null-terminate the string
        ProcessCommandLine(cmd_buffer); // Call the command processor
        cmd_index = 0; // Reset index for next command
    }

    // Clear the received byte and restart reception for the next byte
    RxTemp = 0;
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
}



void Command_Handler(void)
{
    // Store the received byte
    if (RxTemp != '\r' && RxTemp != '\n')
    {
        // Add to command buffer
        cmd_buffer[cmd_index++] = RxTemp;

        // Prevent buffer overflow
        if (cmd_index >= sizeof(cmd_buffer) - 1)
        {
            Debug_Print("Command buffer overflow, resetting.\r\n");
            cmd_index = 0;
        }
    }
    else
    {
        // End of command, process it
        cmd_buffer[cmd_index] = '\0'; // Null-terminate the string
        ProcessCommandLine(cmd_buffer); // Call the command processor
        cmd_index = 0; // Reset index for next command
    }

    // Clear the received byte
    RxTemp = 0;

    // Restart reception for the next byte
    Debug_Print("Received byte: %c\r\n", RxTemp);
    Debug_Print("Current buffer: %s\r\n", cmd_buffer);

    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
}

// Global variable to store latest vx value
static int latest_vx_value = 0;

int GetLatestVelocityX()
{
    return latest_vx_value;
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

    Debug_Print("Processing command: '%s'\r\n", cmd);


    int count = sscanf(cmd, "%7s %7s %7s %7s %7s",
                       token1, token2, token3, token4, token5);

    if (count < 2) {
        Debug_Print("Invalid command: %s\r\n", cmd);
        return;
    }


    if (strcmp(token1, "V") == 0)
    {
        // V <vx> <vy> <vz>
        if (count < 4) {
            Debug_Print("Not enough arguments for velocity command\r\n");
            return;
        }

        int vx = atoi(token2);
        int vy = atoi(token3);
        int vz = atoi(token4);

        // Check for Hard Brake Command: V 9 9 9
        if (vx == 9 && vy == 9 && vz == 9)
        {
            Debug_Print("Hard Brake Command Received: V 9 9 9\r\n");
            Motor_Hard_Brake();
        }
        else
        {
            Motion_Ctrl(vx, vy, vz);
        }

        // 🚨 **Backup Beep Loop** 🚨
        static uint32_t last_beep_time = 0;  // Timestamp for beep control
        static bool beep_is_on = false;      // Track beep state

        if (backup_beep_enabled)  // Check if beeping is enabled
        {
            if (vx < 0)  // Moving backward
            {
                uint32_t current_time = HAL_GetTick();  // Get current time in ms

                if (current_time - last_beep_time >= 500)  // Toggle every 500ms
                {
                    if (!beep_is_on)
                    {
                        Beep_On_Time(500);  // Beep for 500ms
                        Debug_Print("🚨 Backup Beep ON 🚨\r\n");
                    }
                    else
                    {
                        Beep_Off();  // Silence for 500ms
                        Debug_Print("🔇 Backup Beep OFF 🔇\r\n");
                    }

                    beep_is_on = !beep_is_on;  // Toggle state
                    last_beep_time = current_time;  // Update timestamp
                }
            }
            else  // If vx >= 0, stop beeping completely
            {
                if (beep_is_on)
                {
                    Debug_Print("🔇 Stopping backup beep completely.\r\n");
                    Beep_Off();
                    beep_is_on = false;
                }
            }
        }


        else
        {
            Debug_Print("🔇 Stopping backup beep.\r\n");
        }
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
        }
        else if (strcmp(token2, "RESET") == 0) {
            Handle_Info_ResetEncoders();
        }
        else if (strcmp(token2, "BAT") == 0) {
            Handle_Info_Battery();
        }
        else {
            Debug_Print("Unknown info query: %s\r\n", token2);
        }
    }

    else
    {
        Debug_Print("Unknown command type '%s'\r\n", token1);
    }
}



#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

