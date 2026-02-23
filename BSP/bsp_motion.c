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

/**
 * @brief Encoder resolution (ticks per revolution).
 * @note  Matches the JGB37-520 hardware specification.
 */
#ifndef ENCODER_CIRCLE
#define ENCODER_CIRCLE  2450.0f
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

/**
 * @brief Sets the PWM for the four logical motor controllers.
 * @note  Maps logical IDs directly to the motor driver ports.
 */
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4)
{
    Motor_Set_Pwm(MOTOR_ID_M1, Motor_1);
    Motor_Set_Pwm(MOTOR_ID_M2, Motor_2);
    Motor_Set_Pwm(MOTOR_ID_M3, Motor_3);
    Motor_Set_Pwm(MOTOR_ID_M4, Motor_4);
}

/**
 * @brief Immediately stops the robot and clears PID states.
 */
void Motion_Stop(uint8_t brake)
{
    g_start_ctrl = 0;
    PID_Clear_Motor(MAX_MOTOR);
    Motor_Stop(brake);
    move_dist.active = 0;

    car_data.Vx = 0;
    car_data.Vy = 0;
    car_data.Vz = 0;
}

/**
 * @brief Internal function to set motor target speeds for PID.
 */
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4)
{
    motor_data.speed_set[0] = speed_m1;
    motor_data.speed_set[1] = speed_m2;
    motor_data.speed_set[2] = speed_m3;
    motor_data.speed_set[3] = speed_m4;

    for (uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        PID_Set_Motor_Target(i, (float)motor_data.speed_set[i]);
    }

    g_start_ctrl = 1;
}

/**
 * @brief Prints target vs actual speeds for all motors for tuning feedback.
 * @param motor Pointer to motor data structure.
 */
void Handle_Info_PID(motor_data_t* motor)
{
    // Output format: I PID T1 A1 T2 A2 T3 A3 T4 A4
    printf("I PID %.0f %.0f %.0f %.0f %.0f %.0f %.0f %.0f\r\n",
           pid_motor[0].target_val, motor->speed_mm_s[0],
           pid_motor[1].target_val, motor->speed_mm_s[1],
           pid_motor[2].target_val, motor->speed_mm_s[2],
           pid_motor[3].target_val, motor->speed_mm_s[3]);
}

/**
 * @brief Main motion handling loop (called from the main background loop).
 */
void Motion_Handle(void)
{
    // --- SAFETY WATCHDOG ---
    // Stops motors if no command is received within 500ms
    if (g_start_ctrl)
    {
        uint32_t current_tick = HAL_GetTick();
        if (current_tick - g_last_cmd_tick > CMD_WATCHDOG_TIMEOUT_MS)
        {
            Motion_Stop(STOP_BRAKE);
            return;
        }
    }

    // Update feedback and calculate PID outputs
    Motion_Get_Speed(&car_data);

    if (g_start_ctrl)
    {
        // Apply calculated PID PWM values to the motor driver
        Motion_Set_Pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1],
                       motor_data.speed_pwm[2], motor_data.speed_pwm[3]);
    }
}

/**
 * @brief  Updates motor targets based on kinematic mix for mecanum wheels.
 * @note   Formula:
 * LF = Vx - Vy - Vz*(L+W)
 * LR = Vx + Vy - Vz*(L+W)
 * RF = Vx + Vy + Vz*(L+W)
 * RR = Vx - Vy + Vz*(L+W)
 */
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z)
{
    g_last_cmd_tick = HAL_GetTick();

    car_data.Vx = V_x;
    car_data.Vy = V_y;
    car_data.Vz = V_z;

    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motion_Stop(STOP_BRAKE);
        return;
    }

    // Kinematic factor based on wheel separation
    float robot_APB = 1200.0f;
    float rot_factor = (float)V_z * robot_APB / 1000.0f;

    // Kinematic mapping: M1=LF, M2=LR, M3=RF, M4=RR
    int16_t speed_L1 = (int16_t)((float)V_x - (float)V_y - rot_factor); // LF (M1)
    int16_t speed_L2 = (int16_t)((float)V_x + (float)V_y - rot_factor); // LR (M2)
    int16_t speed_R1 = (int16_t)((float)V_x + (float)V_y + rot_factor); // RF (M3)
    int16_t speed_R2 = (int16_t)((float)V_x - (float)V_y + rot_factor); // RR (M4)

    Motion_Set_Speed(speed_L1, speed_L2, speed_R1, speed_R2);
}

/* --- Encoder and Speed Handling --- */

/**
 * @brief Orchestrates encoder polling and PID calculation.
 */
void Motion_Get_Speed(car_data_t* car)
{
    Motion_Get_Encoder();
    if (g_start_ctrl) { PID_Calc_Motor(&motor_data); }
}

/**
 * @brief Reads raw encoder counts and calculates instantaneous speed in mm/s.
 * @note  Uses a measured inter-call dt instead of an assumed 100 Hz rate.
 *        The actual loop period is ~12-15ms (10ms HAL_Delay + ICM20948 SPI + overhead),
 *        so a hardcoded 100.0f multiplier would underestimate speed by up to 50%.
 */
void Motion_Get_Encoder(void)
{
    // --- Measure actual elapsed time since last call ---
    static uint32_t last_encoder_tick = 0;
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - last_encoder_tick;
    last_encoder_tick = now;

    // Guard: on the very first call (last_encoder_tick was 0) dt_ms is huge;
    // also reject zero or unreasonably long gaps. Fall back to 100 Hz in those cases.
    float freq_hz = (dt_ms > 0 && dt_ms <= 500) ? (1000.0f / (float)dt_ms) : 100.0f;

    int32_t current_totals[MAX_MOTOR];
    current_totals[0] = Encoder_Get_Total_Count(MOTOR_ID_M1);
    current_totals[1] = Encoder_Get_Total_Count(MOTOR_ID_M2);
    current_totals[2] = Encoder_Get_Total_Count(MOTOR_ID_M3);
    current_totals[3] = Encoder_Get_Total_Count(MOTOR_ID_M4);

    float circle_mm = Motion_Get_Circle_MM();

    for(uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        g_Encoder_All_Offset[i] = current_totals[i] - g_Encoder_Last_Stored[i];
        g_Encoder_Last_Stored[i] = current_totals[i];

        // Speed mm/s = (Pulses * MeasuredFrequency * WheelCircumference) / Resolution
        motor_data.speed_mm_s[i] = (float)g_Encoder_All_Offset[i] * freq_hz * circle_mm / (float)ENCODER_CIRCLE;
    }
}

float Motion_Get_Circle_MM(void)
{
    return MECANUM_MINI_CIRCLE_MM;
}

int16_t GetLatestVelocityX(void)
{
    return car_data.Vx;
}

/**
 * @brief Prints current raw encoder values to the serial port.
 */
void Handle_Info_Encoders(void)
{
    printf("I ENC %ld %ld %ld %ld\r\n", 
            Encoder_Get_Total_Count(MOTOR_ID_M1), 
            Encoder_Get_Total_Count(MOTOR_ID_M2), 
            Encoder_Get_Total_Count(MOTOR_ID_M3), 
            Encoder_Get_Total_Count(MOTOR_ID_M4));
}

/**
 * @brief Resets all encoder counts and internal speed offsets.
 */
void Handle_Info_ResetEncoders(void)
{
    for(int i = 0; i < MAX_MOTOR; i++) {
        g_Encoder_All_Now[i] = 0;
        g_Encoder_All_Last[i] = 0;
        g_Encoder_All_Offset[i] = 0;
        g_Encoder_Last_Stored[i] = 0;
    }
}
