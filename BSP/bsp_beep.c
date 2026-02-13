#include "bsp.h"
#include "bsp_beep.h"
#include "bsp_uart.h" // For GetLatestVelocityX


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

// Handle Backup Beep (Reverse Warning)
void Backup_Beep_Handle(void)
{
    static uint32_t last_beep_time = 0;
    static uint8_t beep_is_active = 0;
    int vx = GetLatestVelocityX();

    // Check if reversing (vx is negative)
    // Using a small threshold to avoid noise
    if (vx < -50)
    {
        uint32_t current_time = HAL_GetTick();
        
        // Toggle every 500ms
        if (current_time - last_beep_time >= 500)
        {
            if (beep_is_active == 0)
            {
                // Turn Beep ON
                Beep_On_Time(BEEP_STATE_ON_ALWAYS);
                beep_is_active = 1;
            }
            else
            {
                // Turn Beep OFF
                Beep_Off();
                beep_is_active = 0;
            }
            last_beep_time = current_time;
        }
    }
    else
    {
        // Not reversing, ensure beep is off if it was active
        if (beep_is_active)
        {
            Beep_Off();
            beep_is_active = 0;
        }
    }
}
