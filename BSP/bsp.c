#include "bsp.h"
#include "bsp_uart.h"
#include <string.h>


// The LED displays the current operating status, which is invoked every 10 milliseconds, and the LED blinks every 200 milliseconds.  
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
    // 1. Initialize the UART listener.

    USART1_Init();
    HAL_Delay(3000); // 3s delay
    printf("Mecca Robot initialization complete...\r\n");
    Beep_On_Time(50);

}


int car_state = 0;
// This function is called in a loop in main.c to avoid multiple modifications to the main.c file


void Bsp_Loop(void)
{
    // 1. Time-Critical Locomotion (Every 10ms)
    Bsp_Led_Show_State_Handle();       // Keep the LED heartbeat
    Beep_Timeout_Close_Handle();       // Manage buzzer duration
    Backup_Beep_Handle();              // 🏁 Existing: Check reverse beep
    Encoder_Update_Count();            // 🏁 Existing: Update wheel positions
    Motion_Handle();                   // 🏁 Existing: PID and motor control

    // 3. System Yield
    HAL_Delay(10); // Maintains the 10ms base loop frequency
}
