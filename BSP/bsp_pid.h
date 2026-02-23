#ifndef __BSP_PID_H
#define __BSP_PID_H

#include "bsp.h"
#include "bsp_motor.h"
#include "stdint.h"

// GENTLE GAINS FOR GRANULAR CONTROL
#define PID_KP  0.4f   // Lowered to stretch the range
#define PID_KI  0.15f  // Smooth buildup
#define PID_KD  0.1f

typedef struct _motor_data_t {
    float speed_mm_s[4];
    float speed_pwm[4];
    int16_t speed_set[4];
} motor_data_t;

extern motor_data_t motor_data;

typedef struct {
    float target_val;
    float pwm_output;
    float Kp, Ki, Kd;
    float err, err_last;
    float integral;
    uint8_t startup_ramp_cnt;  // Soft-start: ramps dead-zone floor from 0→full over STARTUP_RAMP_CYCLES
} motor_pid_t;

extern motor_pid_t pid_motor[MAX_MOTOR];

void PID_Param_Init(void);
float PID_Positional_Calc(motor_pid_t *pid, float actual_val);
void PID_Calc_Motor(motor_data_t *motor);
void PID_Set_Motor_Parm(uint8_t motor_id, float Kp, float Ki, float Kd);
void PID_Clear_Motor(uint8_t motor_id);
void PID_Set_Motor_Target(uint8_t motor_id, float target);

#endif
