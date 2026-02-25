#ifndef BSP_PID_H_
#define BSP_PID_H_

#include "stdint.h"

#define PID_DEF_KP      (0.8f)
#define PID_DEF_KI      (0.06f)
#define PID_DEF_KD      (0.5f)

typedef struct _pid_t
{
    float target_val;
    float pwm_output;
    float Kp,Ki,Kd;
    float err;
    float err_last;

    float err_next;
    float integral;
} motor_pid_t;

typedef struct _motor_data_t
{
    float speed_mm_s[4];        // Input value, encoder calculation speed
    float speed_pwm[4];         // Output value, PID calculates PWM value
    int16_t speed_set[4];       // Speed setting value
} motor_data_t;


void PID_Param_Init(void);
void PID_Calc_Motor(motor_data_t* motor);
void PID_Set_Motor_Target(uint8_t motor_id, float target);
void PID_Clear_Motor(uint8_t motor_id);
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd);


#endif /* BSP_PID_H_ */
