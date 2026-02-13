#include "bsp.h"
#include "bsp_motion.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "bsp_pid.h"

#include "main.h"    // For GPIO Port and Pin definitions
#include "tim.h"     // For Timer handles
#include <string.h>
#include <stdio.h>
#include <stdlib.h> // for abs()
#include <stdbool.h>


int32_t g_Encoder_All_Now[MAX_MOTOR] = {0};
int g_Encoder_All_Last[MAX_MOTOR] = {0};
int g_Encoder_All_Offset[MAX_MOTOR] = {0};

uint8_t g_start_ctrl = 0;

car_data_t car_data;
motion_dist_t move_dist = {0}; // Initialize to zero


// Control car movement, Motor_X=[-3600, 3600], beyond the range is invalid. 
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4)
{
    int16_t max_value = MOTOR_MAX_PULSE - MOTOR_IGNORE_PULSE;
    if (Motor_1 >= -max_value && Motor_1 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M1, Motor_1);
    }
    if (Motor_2 >= -max_value && Motor_2 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M2, Motor_2);
    }
    if (Motor_3 >= -max_value && Motor_3 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M3, Motor_3);
    }
    if (Motor_4 >= -max_value && Motor_4 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M4, Motor_4);
    }
}

// The car stopped
void Motion_Stop(uint8_t brake)
{
    Motion_Set_Speed(0, 0, 0, 0);
    PID_Clear_Motor(MAX_MOTOR);
    Motor_Stop(brake);
    g_start_ctrl = 0;
    
    // Clear distance mode
    move_dist.active = 0;
    move_dist.target_dist_mm = 0;
    move_dist.current_dist_mm = 0;
}



// Set speed speed mX=[-1000, 1000], unit: mm/s
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4)
{
    // Local buffer for immediate transmission
    char speed_buffer[128]; 

    // Formatting the string locally
    int len = snprintf(speed_buffer, sizeof(speed_buffer), 
                       "SPEED_SET: M1:%d M2:%d M3:%d M4:%d\r\n", 
                       speed_m1, speed_m2, speed_m3, speed_m4);

    // Using the hardware transmit directly, just like "I ENC"
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)speed_buffer, (uint16_t)len, 100);
    }

    g_start_ctrl = 1;
    motor_data.speed_set[0] = speed_m1;
    motor_data.speed_set[1] = speed_m2;
    motor_data.speed_set[2] = speed_m3;
    motor_data.speed_set[3] = speed_m4;

    for (uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        PID_Set_Motor_Target(i, motor_data.speed_set[i]*1.0);
    }
}



// Read the current speed of each wheel from the encoder in mm/s
void Motion_Get_Speed(car_data_t* car)
{
    Motion_Get_Encoder();

    float circle_mm = Motion_Get_Circle_MM();

    float speed_m1 = (g_Encoder_All_Offset[0]) * 100 * circle_mm / (float)ENCODER_CIRCLE;
    float speed_m2 = (g_Encoder_All_Offset[1]) * 100 * circle_mm / (float)ENCODER_CIRCLE;
    float speed_m3 = (g_Encoder_All_Offset[2]) * 100 * circle_mm / (float)ENCODER_CIRCLE;
    float speed_m4 = (g_Encoder_All_Offset[3]) * 100 * circle_mm / (float)ENCODER_CIRCLE;
    float robot_APB = Motion_Get_APB();

    car->Vx = (speed_m1 + speed_m2 + speed_m3 + speed_m4) / 4;
    car->Vy = -(speed_m1 - speed_m2 - speed_m3 + speed_m4) / 4;
    car->Vz = -(speed_m1 + speed_m2 - speed_m3 - speed_m4) / 4.0f / robot_APB * 1000;

    if (g_start_ctrl)
    {
        // ✅ Update actual speeds for PID calculations
        motor_data.speed_mm_s[0] = speed_m1;
        motor_data.speed_mm_s[1] = speed_m2;
        motor_data.speed_mm_s[2] = speed_m3;
        motor_data.speed_mm_s[3] = speed_m4;

        // ✅ Ensure PID target values are set correctly
        for (int i = 0; i < 4; i++)
        {
            pid_motor[i].target_val = motor_data.speed_set[i];
        }

        PID_Calc_Motor(&motor_data);
    }
}


// Returns half of the sum of the current cart wheel axles
float Motion_Get_APB(void)
{
    float apb_value = 150.0;  // Ensure this is defined
    return apb_value;
}


// Returns the number of millimeters at which the current wheel has been turned
float Motion_Get_Circle_MM(void)
{
    return MECANUM_MINI_CIRCLE_MM;
}


// Obtain encoder data and calculate the number of deviation pulses
void Motion_Get_Encoder(void)
{
    Encoder_Update_Count();
    Encoder_Get_ALL(g_Encoder_All_Now);

    for(uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        g_Encoder_All_Offset[i] = g_Encoder_All_Now[i] - g_Encoder_All_Last[i];
	    g_Encoder_All_Last[i] = g_Encoder_All_Now[i];
    }

}

// Control car movement
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z)
{
	//Debug_Print("Motion_Ctrl called: V_x=%d, V_y=%d, V_z=%d\r\n", V_x, V_y, V_z);
    
    // If setting a manual velocity, we should cancel any distance task
    // UNLESS this is the internal call from Motion_Handle for the distance task itself.
    // However, Motion_Ctrl is usually called from UART parser.
    // If called from UART parser, it overwrites distance mode.
    // We will assume this function is the "Manual Velocity" entry point.
    // (Note: Motion_Handle calls Motion_Get_Speed which calls PID_Calc, but Motion_Ctrl sets targets)

	float robot_APB = Motion_Get_APB();
	if (robot_APB == 0) {
	    robot_APB = 1.0f;  // Use a default value to avoid division by zero
	}

    float speed_lr = (float)(-V_y);
    float speed_fb = (float)(V_x);

    float speed_spin = -V_z / 250.0f * robot_APB;

    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motion_Stop(STOP_BRAKE);
        return;
    }
    
    // If we are setting a velocity, we are technically "starting control"
    // BUT we should verify if we want to cancel distance mode here?
    // Yes, a manual velocity command overrides a distance command.
    // We'll set active=0 IF this function is called from outside. 
    // Since we don't have a parameter for "source", we'll just check if we are 
    // NOT inside the loop... but actually, simpler is:
    // If the user sends "V ...", we disable "T ...".
    // This is handled in UART parser logic or here. 
    // Let's rely on the UART parser calling Motion_Stop() or similar before setting new mode if needed.
    // OR we just clear it here safely.
    // However, we need to be careful if we call Motion_Ctrl INSIDE the distance loop. 
    // We will NOT call Motion_Ctrl inside Motion_Handle. We will use Motion_Set_Speed directly.

    // So if this is called, it's manual.
    move_dist.active = 0; 

    int speed_L1_setup = speed_fb + speed_lr + speed_spin;
    int speed_L2_setup = speed_fb - speed_lr + speed_spin;
    int speed_R1_setup = speed_fb - speed_lr - speed_spin;
    int speed_R2_setup = speed_fb + speed_lr - speed_spin;

    // Clamp
    if (speed_L1_setup > 3000) speed_L1_setup = 3000;
    if (speed_L1_setup < -3000) speed_L1_setup = -3000;
    if (speed_L2_setup > 3000) speed_L2_setup = 3000;
    if (speed_L2_setup < -3000) speed_L2_setup = -3000;
    if (speed_R1_setup > 3000) speed_R1_setup = 3000;
    if (speed_R1_setup < -3000) speed_R1_setup = -3000;
    if (speed_R2_setup > 3000) speed_R2_setup = 3000;
    if (speed_R2_setup < -3000) speed_R2_setup = -3000;
    
    Motion_Set_Speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
}


// Start a distance movement
void Motion_Set_Distance(int32_t distance_mm, int16_t speed_mm_s)
{
    Motion_Stop(STOP_BRAKE); // Reset everything first
    
    move_dist.active = 1;
    move_dist.target_dist_mm = distance_mm;
    move_dist.speed_mm_s = abs(speed_mm_s); // Speed is magnitude
    move_dist.current_dist_mm = 0.0f;

    //Debug_Print("Distance Mode: Target=%ld mm, Speed=%d mm/s\r\n", distance_mm, speed_mm_s);

    // Initial movement kick-off
    // If distance is positive -> Move Forward (Positive Vx)
    // If distance is negative -> Move Backward (Negative Vx)
    int16_t initial_speed = (distance_mm >= 0) ? move_dist.speed_mm_s : -move_dist.speed_mm_s;
    
    // We use Motion_Set_Speed to set wheel targets directly for simple FWD/BWD
    // For FWD/BWD, all wheels turn same direction (roughly).
    // FWD: + + + +
    // BWD: - - - -
    Motion_Set_Speed(initial_speed, initial_speed, initial_speed, initial_speed);
}



// Motion control handle, called every 10ms
void Motion_Handle(void)
{
    Motion_Get_Speed(&car_data);

    // Distance Control Logic
    if (move_dist.active)
    {
        // Calculate distance traveled in this interval (10ms)
        // car_data.Vx is in mm/s
        // distance = speed * time
        // time = 0.01 s
        float dist_step = (float)car_data.Vx * 0.01f;
        
        move_dist.current_dist_mm += dist_step;

        // Check for completion
        bool finished = false;
        if (move_dist.target_dist_mm > 0)
        {
            if (move_dist.current_dist_mm >= move_dist.target_dist_mm) finished = true;
        }
        else // Target is negative
        {
            if (move_dist.current_dist_mm <= move_dist.target_dist_mm) finished = true;
        }

        if (finished)
        {
            //Debug_Print("Distance Reached: %.1f mm\r\n", move_dist.current_dist_mm);
            Motion_Stop(STOP_BRAKE);
            // Beep to confirm? Optional.
        }
    }


    if (g_start_ctrl)
    {
        Motion_Set_Pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1],
        		motor_data.speed_pwm[2], motor_data.speed_pwm[3]);
    }
}

void Handle_Velocity_4(int m1, int m2, int m3, int m4)
{
    move_dist.active = 0; // Disable distance mode
    Motor_Set_Pwm(MOTOR_ID_M1, -m1); // if needed
    Motor_Set_Pwm(MOTOR_ID_M2, -m2);
    Motor_Set_Pwm(MOTOR_ID_M3, m3);
    Motor_Set_Pwm(MOTOR_ID_M4, m4);
}



//------------------------------------------------------------------------------
// Handle_Directional
//------------------------------------------------------------------------------
void Handle_Directional(const char* direction, int speed)
{
    move_dist.active = 0; // Disable distance mode manually
    
    int w1 = 0, w2 = 0, w3 = 0, w4 = 0;  // Wheel speeds

    // Optional debug
    // Debug_Print("Handle_Directional: %s | SPEED=%d\r\n", direction, speed);

    if (strcmp(direction, "FWD") == 0)
    {
        // Forward: all wheels at +speed
        w1 = speed; w2 = speed; w3 = speed; w4 = speed;
    }
    else if (strcmp(direction, "BWD") == 0)
    {
        // Backward: all wheels at -speed
        w1 = -speed; w2 = -speed; w3 = -speed; w4 = -speed;
    }
    else if (strcmp(direction, "LFT") == 0)
    {
        // Left: M1 & M4 = -speed, M2 & M3 = +speed
        w1 = -speed; w2 = speed;  w3 = speed;  w4 = -speed;
    }
    else if (strcmp(direction, "RGT") == 0)
    {
        // Right: M1 & M4 = +speed, M2 & M3 = -speed
        w1 = speed;  w2 = -speed; w3 = -speed; w4 = speed;
    }
    else if (strcmp(direction, "FDL") == 0)
    {
        // Forward-Diagonal-Left: M2 & M3 = +2×speed, M1 & M4 = 0
        w2 = 2 * speed;
        w3 = 2 * speed;
    }
    else if (strcmp(direction, "FDR") == 0)
    {
        // Forward-Diagonal-Right: M1 & M4 = +2×speed, M2 & M3 = 0
        w1 = 2 * speed;
        w4 = 2 * speed;
    }
    else if (strcmp(direction, "BDL") == 0)
    {
        // Backward-Diagonal-Left: M2 & M3 = -2×speed, M1 & M4 = 0
        w2 = -2 * speed;
        w3 = -2 * speed;
    }
    else if (strcmp(direction, "BDR") == 0)
    {
        // Backward-Diagonal-Right: M1 & M4 = -2×speed, M2 & M3 = 0
        w1 = -2 * speed;
        w4 = -2 * speed;
    }
    else if (strcmp(direction, "STP") == 0 || strcmp(direction, "BRK") == 0)
    {
        // Stop or Brake
        // Debug_Print("Handle_Directional: Hard stop\r\n");
        Motion_Stop(STOP_BRAKE); // This already clears distance mode
        return;
    }
    else
    {
        //Debug_Print("Unknown directional command: '%s'\r\n", direction);
        return;
    }

    // Now apply the speeds to each wheel
    Motion_Set_Speed(w1, w2, w3, w4);
}


void Handle_Info_Encoders()
{
    char local_buf[128];
    int32_t enc[4] = {0};
    Encoder_Get_ALL(enc);

    // Formulate the EXACT string expected by your serial monitor
    int len = snprintf(local_buf, sizeof(local_buf),
                       "I ENC %ld %ld %ld %ld\r\n",
                       enc[0], enc[1], enc[2], enc[3]);

    // Use a blocking transmit to ensure the hardware doesn't "give up"
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)local_buf, len, 5000);
    }
}

void Handle_Info_ResetEncoders()
{
    //Debug_Print("Handle_Information: Reset encoder values to 0.\r\n");

    // Reset all hardware timers for encoders
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Timer for Encoder 1
    __HAL_TIM_SET_COUNTER(&htim3, 0);  // Timer for Encoder 2
    __HAL_TIM_SET_COUNTER(&htim4, 0);  // Timer for Encoder 3
    __HAL_TIM_SET_COUNTER(&htim5, 0);  // Timer for Encoder 4

    // Also reset the software variables
    for (int i = 0; i < 4; i++) {
        g_Encoder_All_Now[i] = 0;
        g_Encoder_All_Offset[i] = 0;
        g_Encoder_All_Last[i] = 0;
    }

    //Debug_Print("Encoders reset complete.\r\n");
}

int GetLatestVelocityX(void)
{
    // For now, return 0 so the backup beep doesn't trigger accidentally
    return 0;
}

void Handle_Info_Battery()
{
	//Debug_Print("Handle_Information: Get battery status.\r\n");
	// Get the current state of the battery
	//Debug_Print("Battery status: nn%\r\n");
}
