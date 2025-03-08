#include "bsp.h"
#include "bsp_uart.h"


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


// The peripheral device is initialized
void Bsp_Init(void)
{
	Beep_On_Time(50);
	Motor_Init();
	Encoder_Init();
	PID_Param_Init();
}

int car_state = 0;
// This function is called in a loop in main.c to avoid multiple modifications to the main.c file



void Bsp_Loop(void)
{
	Bsp_Led_Show_State_Handle();
	Beep_Timeout_Close_Handle();
	Encoder_Update_Count();
	Motion_Handle();
	//Command_Handler();
	HAL_Delay(10);
}
