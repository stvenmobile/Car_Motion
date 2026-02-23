#include "bsp_pid.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
#include "bsp_motion.h"
#include <string.h>
#include <stdio.h>

/* Global motor and PID data structures */
motor_data_t motor_data;
motor_pid_t pid_motor[MAX_MOTOR] = {0};

// Number of 10ms PID cycles over which the dead-zone floor is linearly ramped
// from 0 to MOTOR_IGNORE_PULSE on motor startup (20 cycles = ~200ms).
// This staggers the effective inrush current and prevents supply brownouts.
#define STARTUP_RAMP_CYCLES  20

/**
  * @brief  Initializes PID parameters for all motors using constants from bsp_pid.h
  */
void PID_Param_Init(void)
{
    for (int i = 0; i < MAX_MOTOR; i++)
    {
        pid_motor[i].Kp = PID_KP;
        pid_motor[i].Ki = PID_KI;
        pid_motor[i].Kd = PID_KD;
        pid_motor[i].integral = 0.0f;
        pid_motor[i].err_last = 0.0f;
        pid_motor[i].target_val = 0.0f;
    }
}

/**
  * @brief  Core Positional PID calculation
  * @param  pid: Pointer to the motor's PID structure
  * @param  actual_val: Current speed (A) from encoders
  * @retval Calculated PWM adjustment
  */
float PID_Positional_Calc(motor_pid_t *pid, float actual_val)
{
    // Calculate current error
    pid->err = pid->target_val - actual_val;

    // Accumulate integral (I)
    pid->integral += pid->err;

    // Anti-Windup: Limit the integral term to prevent runaway if the wheel is stalled
    // Set to 2000.0f to provide enough "reach" for the 3200 max pulse
    if (pid->integral > 2000.0f) pid->integral = 2000.0f;
    if (pid->integral < -2000.0f) pid->integral = -2000.0f;

    // Calculate derivative (D)
    float derivative = pid->err - pid->err_last;

    // Sum the terms
    pid->pwm_output = (pid->Kp * pid->err) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Save error for next cycle
    pid->err_last = pid->err;

    // Hard safety clamp to defined MOTOR_MAX_PULSE
    if (pid->pwm_output > MOTOR_MAX_PULSE)  pid->pwm_output = (float)MOTOR_MAX_PULSE;
    if (pid->pwm_output < -MOTOR_MAX_PULSE) pid->pwm_output = (float)-MOTOR_MAX_PULSE;

    return pid->pwm_output;
}

/**
  * @brief  Calculates PID for all motors and applies Dead Zone compensation
  * @param  motor: Pointer to the shared motor data structure
  */
void PID_Calc_Motor(motor_data_t* motor)
{
    for (int i = 0; i < MAX_MOTOR; i++)
    {
        // Calculate raw PID adjustment
        float raw_pwm = PID_Positional_Calc(&pid_motor[i], motor->speed_mm_s[i]);

        // APPLY THE FLOOR WITH SOFT-START RAMP:
        // Instead of jumping to MOTOR_IGNORE_PULSE immediately (which causes a large
        // inrush current surge across all 4 motors), linearly scale the dead-zone
        // floor from 0 up to MOTOR_IGNORE_PULSE over STARTUP_RAMP_CYCLES iterations.
        // Each motor ramps independently; the counter resets when the motor stops.
        int16_t raw_i16 = (int16_t)raw_pwm;
        int16_t final_pwm;

        if (raw_i16 > 5 || raw_i16 < -5)
        {
            // Pre-clamp: keep raw PID output within ±(MOTOR_MAX_PULSE - MOTOR_IGNORE_PULSE).
            // Without this, PID_Positional_Calc can return up to ±MOTOR_MAX_PULSE (3200),
            // and adding MOTOR_IGNORE_PULSE (1250) would give ±4450 — which Motor_Set_Pwm
            // then silently re-clamps to 3200.  That makes the PID's control range appear
            // to saturate at raw=1950 while the integral keeps winding up past that point.
            // With this clamp the full [raw + floor] range maps exactly to [0, MOTOR_MAX_PULSE].
            const int16_t max_raw = (int16_t)(MOTOR_MAX_PULSE - MOTOR_IGNORE_PULSE); // 1950
            if (raw_i16 >  max_raw) raw_i16 =  max_raw;
            if (raw_i16 < -max_raw) raw_i16 = -max_raw;

            uint8_t ramp = pid_motor[i].startup_ramp_cnt;
            if (ramp < STARTUP_RAMP_CYCLES) pid_motor[i].startup_ramp_cnt++;
            int16_t scaled_floor = (int16_t)((float)MOTOR_IGNORE_PULSE * ramp / STARTUP_RAMP_CYCLES);
            final_pwm = (raw_i16 > 0) ? (raw_i16 + scaled_floor) : (raw_i16 - scaled_floor);
        }
        else
        {
            // Output is in the dead band — hold ramp at zero so it restarts cleanly.
            pid_motor[i].startup_ramp_cnt = 0;
            final_pwm = 0;
        }

        motor->speed_pwm[i] = (float)final_pwm;
    }
}

/**
  * @brief  Sets target speed for a specific motor or all motors
  */
void PID_Set_Motor_Target(uint8_t motor_id, float target)
{
    if (motor_id >= MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++) pid_motor[i].target_val = target;
    }
    else
    {
        pid_motor[motor_id].target_val = target;
    }
}

/**
  * @brief  Resets PID variables to zero to prevent jumps when restarting movement
  */
void PID_Clear_Motor(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].pwm_output = 0.0f;
            pid_motor[i].err = 0.0f;
            pid_motor[i].err_last = 0.0f;
            pid_motor[i].integral = 0.0f;
            pid_motor[i].target_val = 0.0f;
            pid_motor[i].startup_ramp_cnt = 0;
        }
    }
    else
    {
        pid_motor[motor_id].pwm_output = 0.0f;
        pid_motor[motor_id].err = 0.0f;
        pid_motor[motor_id].err_last = 0.0f;
        pid_motor[motor_id].integral = 0.0f;
        pid_motor[motor_id].target_val = 0.0f;
        pid_motor[motor_id].startup_ramp_cnt = 0;
    }
}

/**
  * @brief  Dynamically update PID gains
  */
void PID_Set_Motor_Parm(uint8_t motor_id, float Kp, float Ki, float Kd)
{
    if (motor_id < MAX_MOTOR)
    {
        pid_motor[motor_id].Kp = Kp;
        pid_motor[motor_id].Ki = Ki;
        pid_motor[motor_id].Kd = Kd;
    }
}
