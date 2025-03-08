#include "bsp.h"
#include "bsp_beep.h"


uint16_t beep_on_time = 0;
uint8_t beep_state = 0;


// Refreshes the buzzer time
static void Beep_Set_Time(uint16_t time)
{
	beep_on_time = time;
}


// Gets the remaining time of the current buzzer on
static uint16_t Beep_Get_Time(void)
{
	return beep_on_time;
}


// Refreshes the buzzer status
static void Beep_Set_State(uint8_t state)
{
	beep_state = state;
}


// Gets the status of the buzzer
static uint8_t Beep_Get_State(void)
{
	return beep_state;
}


// Set the buzzer start time. The buzzer is disabled when time is 0
// Keeps ringing when time is 1, and automatically shuts down after time>=10
void Beep_On_Time(uint16_t time)
{
	if (time == BEEP_STATE_ON_ALWAYS)
	{
		Beep_Set_State(BEEP_STATE_ON_ALWAYS);
		Beep_Set_Time(0);
		BEEP_ON();
	}
	else if (time == BEEP_STATE_OFF)
	{
		Beep_Set_State(BEEP_STATE_OFF);
		Beep_Set_Time(0);
		BEEP_OFF();
	}
	else
	{
		if (time >= 10)
		{
			Beep_Set_State(BEEP_STATE_ON_DELAY);
			Beep_Set_Time(time / 10);
			BEEP_ON();
		}
	}
}

void Beep_Off()
{
    BEEP_OFF();  // Calls the macro to turn the buzzer off
    Beep_Set_State(BEEP_STATE_OFF);  // Ensure the state is properly updated
}


// Buzzer timeout automatically shut down the program, 10 milliseconds to call once
void Beep_Timeout_Close_Handle(void)
{
	if (Beep_Get_State() == BEEP_STATE_ON_DELAY)
	{
		if (Beep_Get_Time())
		{
			beep_on_time--;
		}
		else
		{
			BEEP_OFF();
			Beep_Set_State(BEEP_STATE_OFF);
		}
	}
}


