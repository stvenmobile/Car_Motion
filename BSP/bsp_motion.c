#include "main.h"
#include "bsp.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "bsp_motion.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "bsp_pid.h"
#include "tim.h"

// Corrected to 440 based on the 10x error analysis
#ifndef ENCODER_CIRCLE
#define ENCODER_CIRCLE  220.0f
#endif

int32_t g_Encoder_All_Now[MAX_MOTOR] = {0};
int g_Encoder_All_Last[MAX_MOTOR] = {0};
int g_Encoder_All_Offset[MAX_MOTOR] = {0};
uint8_t g_start_ctrl = 0;
uint32_t g_last_cmd_tick = 0;

car_data_t car_data;
motion_dist_t move_dist = {0};

// Shared with bsp_encoder.c
extern int32_t g_Encoder_M1_Now, g_Encoder_M2_Now, g_Encoder_M3_Now, g_Encoder_M4_Now;

// Local tracking for speed calculation
static int32_t g_Encoder_Last_Stored[MAX_MOTOR] = {0};

/* --- Motor and Motion Control --- */

void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4)
{
    Motor_Set_Pwm(MOTOR_ID_M1, Motor_1);
    Motor_Set_Pwm(MOTOR_ID_M2, Motor_2);
    Motor_Set_Pwm(MOTOR_ID_M3, Motor_3);
    Motor_Set_Pwm(MOTOR_ID_M4, Motor_4);
}

void Motion_Stop(uint8_t brake)
{
    PID_Clear_Motor(MAX_MOTOR);
    Motor_Stop(brake);
    g_start_ctrl = 0;
    move_dist.active = 0;
}

void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4)
{
    g_start_ctrl = 1;
    motor_data.speed_set[0] = speed_m1;
    motor_data.speed_set[1] = speed_m2;
    motor_data.speed_set[2] = speed_m3;
    motor_data.speed_set[3] = speed_m4;

    for (uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        PID_Set_Motor_Target(i, (float)motor_data.speed_set[i]);
    }
}

void Motion_Handle(void)
{
    Motion_Get_Speed(&car_data);

    // --- SOFTWARE WATCHDOG CHECK ---
    if (g_start_ctrl && !move_dist.active)
    {
        // Increased to 2000ms for easier manual testing over Serial
        if (HAL_GetTick() - g_last_cmd_tick > 2000)
        {
            Motion_Stop(STOP_BRAKE);
        }
    }

    if (g_start_ctrl)
    {
        Motion_Set_Pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1],
                       motor_data.speed_pwm[2], motor_data.speed_pwm[3]);
    }
}

void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z)
{
    g_last_cmd_tick = HAL_GetTick();
    
    car_data.Vx = V_x;
    car_data.Vy = V_y;
    car_data.Vz = V_z;

    float robot_APB = 500.0f;

    int speed_L1 = V_x - V_y - (V_z * robot_APB / 1000.0f);
    int speed_L2 = V_x + V_y - (V_z * robot_APB / 1000.0f);
    int speed_R1 = V_x + V_y + (V_z * robot_APB / 1000.0f);
    int speed_R2 = V_x - V_y + (V_z * robot_APB / 1000.0f);

    Motion_Set_Speed(speed_L1, speed_L2, speed_R1, speed_R2);
}

/* --- Encoder and Speed Handling --- */

void Motion_Get_Speed(car_data_t* car)
{
    Motion_Get_Encoder();
    if (g_start_ctrl) { PID_Calc_Motor(&motor_data); }
}

void Motion_Get_Encoder(void)
{
    int32_t current_totals[MAX_MOTOR];
    current_totals[0] = Encoder_Get_Total_Count(MOTOR_ID_M1);
    current_totals[1] = Encoder_Get_Total_Count(MOTOR_ID_M2);
    current_totals[2] = Encoder_Get_Total_Count(MOTOR_ID_M3);
    current_totals[3] = Encoder_Get_Total_Count(MOTOR_ID_M4);

    float circle_mm = Motion_Get_Circle_MM(); 

    for(uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        // Calculate the "Delta" using the persistent totals
        g_Encoder_All_Offset[i] = current_totals[i] - g_Encoder_Last_Stored[i];
        g_Encoder_Last_Stored[i] = current_totals[i];
        
        // Speed in mm/s: (pulses * 100Hz loop) * (mm_per_turn / pulses_per_turn)
        motor_data.speed_mm_s[i] = (float)g_Encoder_All_Offset[i] * 100.0f * circle_mm / (float)ENCODER_CIRCLE;
    }
}

/* --- Data Accessors and Linker Fixes --- */

float Motion_Get_Circle_MM(void)
{
    return MECANUM_MINI_CIRCLE_MM;
}

int16_t GetLatestVelocityX(void)
{
    return car_data.Vx;
}

void Handle_Info_Encoders(void)
{
    printf("I ENC %ld %ld %ld %ld\r\n", 
            Encoder_Get_Total_Count(MOTOR_ID_M1), 
            Encoder_Get_Total_Count(MOTOR_ID_M2), 
            Encoder_Get_Total_Count(MOTOR_ID_M3), 
            Encoder_Get_Total_Count(MOTOR_ID_M4));
}

void Handle_Info_ResetEncoders(void)
{
    for(int i=0; i<MAX_MOTOR; i++) {
        g_Encoder_All_Now[i] = 0;
        g_Encoder_All_Last[i] = 0;
        g_Encoder_All_Offset[i] = 0;
        g_Encoder_Last_Stored[i] = 0;
    }
}
