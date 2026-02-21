#include "bsp_pid.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
#include "bsp_motion.h"
#include <string.h>
#include <stdio.h>

/* Global motor and PID data structures */
motor_data_t motor_data;
motor_pid_t pid_motor[MAX_MOTOR] = {0};

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

    // Anti-Windup: Prevent the integral from growing infinitely if the robot is stuck
    if (pid->integral > 1500.0f) pid->integral = 1500.0f;
    if (pid->integral < -1500.0f) pid->integral = -1500.0f;

    // Calculate derivative (D)
    float derivative = pid->err - pid->err_last;

    // Sum the terms
    pid->pwm_output = (pid->Kp * pid->err) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Save error for next cycle
    pid->err_last = pid->err;

    // Hard safety clamp to your defined MOTOR_MAX_PULSE (e.g., 2200)
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
    static uint32_t log_counter = 0;
    log_counter++;

    for (int i = 0; i < MAX_MOTOR; i++)
    {
        // Calculate raw PID adjustment
        float raw_pwm = PID_Positional_Calc(&pid_motor[i], motor->speed_mm_s[i]);

        // APPLY THE FLOOR: This jumps the 1450 PWM gap to overcome physical friction
        int16_t final_pwm = Motor_Ignore_Dead_Zone((int16_t)raw_pwm);
        motor->speed_pwm[i] = (float)final_pwm;
        
        // Log output every 50 cycles (~500ms) for serial debugging
        if (log_counter >= 50)
        {
            char msg[80];
            snprintf(msg, sizeof(msg), "M%d: T:%.1f A:%.1f OUT:%d\r\n",
                     i, pid_motor[i].target_val, motor->speed_mm_s[i], final_pwm);
            USART1_Send_ArrayU8((uint8_t *)msg, strlen(msg));
        }
    }
    if (log_counter >= 50) log_counter = 0;
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
        }
    }
    else
    {
        pid_motor[motor_id].pwm_output = 0.0f;
        pid_motor[motor_id].err = 0.0f;
        pid_motor[motor_id].err_last = 0.0f;
        pid_motor[motor_id].integral = 0.0f;
        pid_motor[motor_id].target_val = 0.0f;
    }
}

/**
  * @brief  Dynamically update PID gains (Useful for ROS2 teleop tuning)
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
