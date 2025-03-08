#ifndef __BSP_PID_H
#define __BSP_PID_H

#include "bsp.h"
#include "bsp_motor.h"
#include "stdint.h"

#define PID_KP  1.8   // Increased for faster response
#define PID_KI  0.10  // Reduced to prevent slow buildup
#define PID_KD  0.35  // Increased for sharper corrections



typedef struct _motor_data_t {
    float speed_mm_s[4];        // Encoder-calculated speed
    float speed_pwm[4];         // PID-calculated PWM
    int16_t speed_set[4];       // Target speed
} motor_data_t;

extern motor_data_t motor_data;

typedef struct {
    float target_val;
    float pwm_output;
    float Kp, Ki, Kd;
    float err, err_last, err_next;
    float integral;
} motor_pid_t;

extern motor_pid_t pid_motor[MAX_MOTOR];

// Function declarations
void PID_Param_Init();
float PID_Incre_Calc(motor_pid_t *pid, float actual_val, int motor_index);
void PID_Calc_Motor(motor_data_t *motor);
void PID_Set_Motor_Parm(uint8_t motor_id, float Kp, float Ki, float Kd);
void PID_Clear_Motor(uint8_t motor_id);
void PID_Set_Motor_Target(uint8_t motor_id, float target);



#endif /* __BSP_PID_H */
