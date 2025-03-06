#include "bsp_pid.h"
#include "bsp_uart.h"
#include "bsp.h"


#define PI      (3.1415926f)

motor_pid_t pid_motor[4];


// Example Initialize PID parameters 初始化PID参数
void PID_Param_Init(void)
{
    for (int i = 0; i < MAX_MOTOR; i++)
    {
        pid_motor[i].target_val = 0.0;
        pid_motor[i].pwm_output = 0.0;
        pid_motor[i].err = 0.0;
        pid_motor[i].err_last = 0.0;
        pid_motor[i].err_next = 0.0;
        pid_motor[i].integral = 0.0;

        pid_motor[i].Kp = PID_DEF_KP;
        pid_motor[i].Ki = PID_DEF_KI;
        pid_motor[i].Kd = PID_DEF_KD;
    }
}

// Incremental PID calculation formula
float PID_Incre_Calc(motor_pid_t *pid, float actual_val, int motor_index)

{
    pid->err = pid->target_val - actual_val;
    pid->pwm_output += pid->Kp * (pid->err - pid->err_next)
                    + pid->Ki * pid->err
                    + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;

    // Limit PWM output within safe range
    if (pid->pwm_output > MOTOR_MAX_PULSE)  pid->pwm_output = MOTOR_MAX_PULSE;
    if (pid->pwm_output < -MOTOR_MAX_PULSE) pid->pwm_output = -MOTOR_MAX_PULSE;

    // Debug print to monitor PID calculations
    static uint32_t loop_counter = 0;
    loop_counter++;
    if (loop_counter >= 10)  // Adjust this value for desired frequency
    {
        Debug_Print("PID Debug: Motor=%d, Target=%.2f, Actual=%.2f, Error=%.2f, PWM=%.2f\r\n",
                    motor_index,
                    pid->target_val,
                    actual_val,
                    pid->err,
                    pid->pwm_output);
        loop_counter = 0;
    }

    // Ensure correct type casting to int32_t for motor control
    return (int32_t)pid->pwm_output;
}




// PID Calculates the output value
void PID_Calc_Motor(motor_data_t* motor)
{
    for (int i = 0; i < MAX_MOTOR; i++)
    {
        // Calculate PWM output as float and pass motor index
        float pwm = PID_Incre_Calc(&pid_motor[i], motor->speed_mm_s[i], i);

        // Clamp the output before converting to int32_t
        if (pwm > MOTOR_MAX_PULSE) pwm = MOTOR_MAX_PULSE;
        if (pwm < -MOTOR_MAX_PULSE) pwm = -MOTOR_MAX_PULSE;

        // Convert to int32_t for motor driver
        motor->speed_pwm[i] = (int32_t)pwm;

        static uint32_t loop_counter = 0;
        loop_counter++;
        if (loop_counter >= 10)
        {
            Debug_Print("Calling PID_Calc_Motor: Motor %d, Speed mm/s: %ld \r\n", i, motor->speed_pwm[i]);
            loop_counter = 0;
        }
    }
}




// Set PID parameters, motor_id=4 set all, =0123 Set PID parameters of the corresponding motor
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd)
{
	static uint32_t loop_counter = 0;
	loop_counter++;
	if (loop_counter >= 10) {
		Debug_Print("PID_Set_Motor_Parm called: motor_id=%d, kp=%.2f, ki=%.2f, kd=%.2f\r\n",
				                motor_id, kp, ki, kd);
	    loop_counter = 0;
	}

    if (motor_id > MAX_MOTOR) return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].Kp = kp;
            pid_motor[i].Ki = ki;
            pid_motor[i].Kd = kd;
            Debug_Print(" -> motor[%d] Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n",
                                                i, pid_motor[i].Kp, pid_motor[i].Ki, pid_motor[i].Kd);
        }
    }
    else
    {
        pid_motor[motor_id].Kp = kp;
        pid_motor[motor_id].Ki = ki;
        pid_motor[motor_id].Kd = kd;
        Debug_Print(" -> motor[%d] Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n",
                                    motor_id,
                                    pid_motor[motor_id].Kp,
                                    pid_motor[motor_id].Ki,
                                    pid_motor[motor_id].Kd);
    }
}

// Clearing PID Data  清除PID数据
void PID_Clear_Motor(uint8_t motor_id)
{
    if (motor_id > MAX_MOTOR) return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].pwm_output = 0.0;
            pid_motor[i].err = 0.0;
            pid_motor[i].err_last = 0.0;
            pid_motor[i].err_next = 0.0;
            pid_motor[i].integral = 0.0;
        }
    }
    else
    {
        pid_motor[motor_id].pwm_output = 0.0;
        pid_motor[motor_id].err = 0.0;
        pid_motor[motor_id].err_last = 0.0;
        pid_motor[motor_id].err_next = 0.0;
        pid_motor[motor_id].integral = 0.0;
    }
}

// Set PID target speed, unit: mm/s
void PID_Set_Motor_Target(uint8_t motor_id, float target)
{
    if (motor_id > MAX_MOTOR) return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].target_val = target;
        }
    }
    else
    {
        pid_motor[motor_id].target_val = target;
    }
}
