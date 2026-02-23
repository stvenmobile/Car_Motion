#include "bsp.h"
#include "bsp_uart.h"
#include "bsp_icm20948.h"
#include <string.h>
#include "bsp_rgb.h"

// The LED heartbeat for the board
static void Bsp_Led_Show_State_Handle(void)
{
    static uint8_t led_count = 0;
    led_count++;
    if (led_count > 20)
    {
        led_count = 0;
        LED_TOGGLE();
    }
}

void Bsp_Init(void)
{
    USART1_Init();
    HAL_Delay(1000);

    RGB_Init();
    RGB_Clear();

    ICM20948_init();
    AK09916_init();
    ICM20948_gyro_calibration();

    Motor_Init();
    Encoder_Init();
    PID_Param_Init(); // <--- ADD THIS LINE TO LOAD PID GAINS

    printf("IMU initialization complete...\r\n");
    printf("Mecca Robot initialization complete...\r\n");
    Beep_On_Time(50);
}

void Bsp_Loop(void)
{
    static uint32_t last_rgb_tick = 0;

    // 1. Time-Critical Locomotion & Housekeeping
    Command_Handler();       // Process any pending serial command (safe main-loop context)
    Bsp_Led_Show_State_Handle();
    Beep_Timeout_Close_Handle();
    Backup_Beep_Handle();
    Encoder_Update_Count();
    Motion_Handle();
    ICM20948_Read_Data_Handle();

    // 2. RGB Heartbeat Test (Every 1 second)
    if (HAL_GetTick() - last_rgb_tick > 1000) {
        static uint8_t toggle = 0;
        if (toggle) {
            RGB_Set_All(20, 0, 0); // Dim Red
        } else {
            RGB_Clear();
        }
        toggle = !toggle;
        last_rgb_tick = HAL_GetTick();
    }

    HAL_Delay(10); // Maintains the 10ms base loop frequency
}
