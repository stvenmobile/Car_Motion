#include "bsp_motion.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "bsp.h"

#include "main.h"    // For GPIO Port and Pin definitions
#include "tim.h"     // For Timer handles
#include <string.h>
#include <stdio.h>




int32_t g_Encoder_All_Now[MAX_MOTOR] = {0};
int g_Encoder_All_Last[MAX_MOTOR] = {0};

int g_Encoder_All_Offset[MAX_MOTOR] = {0};

uint8_t g_start_ctrl = 0;

car_data_t car_data;
motor_data_t motor_data;



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
}



// Set speed speed mX=[-1000, 1000], unit: mm/s
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4)
{
    static uint32_t loop_counter = 0;
    loop_counter++;

	//Debug_Print("Motion_Set_Speed called: M1=%d, M2=%d, M3=%d, M4=%d\r\n", speed_m1, speed_m2, speed_m3, speed_m4);
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
        motor_data.speed_mm_s[0] = speed_m1;
        motor_data.speed_mm_s[1] = speed_m2;
        motor_data.speed_mm_s[2] = speed_m3;
        motor_data.speed_mm_s[3] = speed_m4;
        PID_Calc_Motor(&motor_data);
    }

    static uint32_t loop_counter = 0;
    if (loop_counter >= 0)
    {
        //Debug_Print("Encoder Offsets: M1=%ld, M2=%ld, M3=%ld, M4=%ld\r\n",
           // g_Encoder_All_Offset[0],
           // g_Encoder_All_Offset[1],
           // g_Encoder_All_Offset[2],
           // g_Encoder_All_Offset[3]);

        //Debug_Print("Speed Calculations: M1=%.2f mm/s, M2=%.2f mm/s, M3=%.2f mm/s, M4=%.2f mm/s\r\n",
           // speed_m1,
           // speed_m2,
           // speed_m3,
           // speed_m4);
        loop_counter = 0;
    }

}

// Returns half of the sum of the current cart wheel axles
float Motion_Get_APB(void)
{
    float apb_value = 150.0;  // Ensure this is defined
    //Debug_Print("1 Motion_Get_APB called: APB = %.1f\r\n", apb_value);

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

	float robot_APB = Motion_Get_APB();
	if (robot_APB == 0) {
	    robot_APB = 1.0f;  // Use a default value to avoid division by zero
	    //Debug_Print("Warning: Motion_Get_APB() returned 0. Using fallback value 1.0f\r\n");
	}

    //Debug_Print("robot_APB = %.1f\r\n", robot_APB);

    float speed_lr = (float)(-V_y);
    float speed_fb = (float)(V_x);

    if (robot_APB == 0) {
        robot_APB = 1.0f;  // Prevent division by zero
        Debug_Print("Warning: robot_APB is 0. Using fallback value 1.0f\r\n");
    }
    float speed_spin = -V_z / 1000.0f * robot_APB;

    //Debug_Print(" Motion_Ctrl => speed_lr=%.1f, speed_fb=%.1f, speed_spin=%.1f\r\n", speed_lr, speed_fb, speed_spin);

    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motion_Stop(STOP_BRAKE);
        return;
    }

    int speed_L1_setup = speed_fb + speed_lr + speed_spin;
    int speed_L2_setup = speed_fb - speed_lr + speed_spin;
    int speed_R1_setup = speed_fb - speed_lr - speed_spin;
    int speed_R2_setup = speed_fb + speed_lr - speed_spin;

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



// Motion control handle, called every 10ms, mainly processing speed related data
void Motion_Handle(void)
{
    Motion_Get_Speed(&car_data);

    if (g_start_ctrl)
    {
        Motion_Set_Pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1],
        		motor_data.speed_pwm[2], motor_data.speed_pwm[3]);
    }
}

void Handle_Velocity_4(int m1, int m2, int m3, int m4)
{
    Motor_Set_Pwm(MOTOR_ID_M1, -m1); // if needed
    Motor_Set_Pwm(MOTOR_ID_M2, -m2);
    Motor_Set_Pwm(MOTOR_ID_M3, m3);
    Motor_Set_Pwm(MOTOR_ID_M4, m4);
}



//------------------------------------------------------------------------------
// Handle_Directional: Sets 4 wheel speeds for Mecanum directions.
//  - "FWD" => All wheels = +speed
//  - "BWD" => All wheels = -speed
//  - "LFT" => M1 & M4 = -speed, M2 & M3 = +speed
//  - "RGT" => M1 & M4 = +speed,  M2 & M3 = -speed
//  - "FDL" => M2 & M3 = +2×speed, M1 & M4 = 0
//  - "FDR" => M1 & M4 = +2×speed, M2 & M3 = 0
//  - "BDL" => M2 & M3 = -2×speed, M1 & M4 = 0
//  - "BDR" => M1 & M4 = -2×speed, M2 & M3 = 0
//  - "STP" => All wheels = 0
//
// speed is the "base speed" (e.g., 1200). Diagonals double that for only 2 wheels.
//------------------------------------------------------------------------------
void Handle_Directional(const char* direction, int speed)
{
    int w1 = 0, w2 = 0, w3 = 0, w4 = 0;  // Wheel speeds

    // Optional debug
    Debug_Print("Handle_Directional: %s | SPEED=%d\r\n", direction, speed);

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
        // If you want a "hard brake," you might call Motor_Stop(1),
        // otherwise just set speeds to zero:
        Debug_Print("Handle_Directional: Hard stop\r\n");
        Motor_Stop(1);
        return;
    }
    else
    {
        Debug_Print("Unknown directional command: '%s'\r\n", direction);
        return;
    }

    // Now apply the speeds to each wheel
    Motor_Set_Pwm(MOTOR_ID_M1, w1);
    Motor_Set_Pwm(MOTOR_ID_M2, w2);
    Motor_Set_Pwm(MOTOR_ID_M3, w3);
    Motor_Set_Pwm(MOTOR_ID_M4, w4);
}


void Handle_Info_Encoders()
{

#define PRINTF_BUFFER_SIZE 128  // Adjust size as needed

char printf_buffer[PRINTF_BUFFER_SIZE];  // Declare this at the top of the function
    int32_t enc[4] = {0};
    Encoder_Get_ALL(enc);

    snprintf(printf_buffer, PRINTF_BUFFER_SIZE, "Encoder Values: M1=%ld, M2=%ld, M3=%ld, M4=%ld\r\n",
                enc[0], enc[1], enc[2], enc[3]);
    HAL_UART_Transmit(&huart1, (uint8_t *)printf_buffer, strlen(printf_buffer), HAL_MAX_DELAY);

}

void Handle_Info_ResetEncoders()
{
    Debug_Print("Handle_Information: Reset encoder values to 0.\r\n");

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

    Debug_Print("Encoders reset complete.\r\n");
}



void Handle_Info_Battery()
{
	Debug_Print("Handle_Information: Get battery status.\r\n");
	// Get the current state of the battery
	Debug_Print("Battery status: nn%\r\n");
}





