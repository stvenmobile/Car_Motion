#include "bsp.h"
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "main.h"

// Motor 1 GPIO
#define MOTOR_IN1_GPIO_Port GPIOC
#define MOTOR_IN1_Pin GPIO_PIN_6
#define MOTOR_IN2_GPIO_Port GPIOC
#define MOTOR_IN2_Pin GPIO_PIN_7

// Motor 2 GPIO
#define MOTOR_IN3_GPIO_Port GPIOC
#define MOTOR_IN3_Pin GPIO_PIN_8
#define MOTOR_IN4_GPIO_Port GPIOC
#define MOTOR_IN4_Pin GPIO_PIN_9

// Motor 3 GPIO
#define MOTOR_IN5_GPIO_Port GPIOA
#define MOTOR_IN5_Pin GPIO_PIN_11
#define MOTOR_IN6_GPIO_Port GPIOA
#define MOTOR_IN6_Pin GPIO_PIN_8

// Motor 4 GPIO
#define MOTOR_IN7_GPIO_Port GPIOB
#define MOTOR_IN7_Pin GPIO_PIN_0
#define MOTOR_IN8_GPIO_Port GPIOB
#define MOTOR_IN8_Pin GPIO_PIN_1

/**
  * @brief  Jumps the PWM over the physical friction deadband.
  * @param  pulse: Raw PID output
  * @retval Boosted PWM value
  */
int16_t Motor_Ignore_Dead_Zone(int16_t pulse)
{
    // Only jump the gap if the PID request is significant
    // This stops the "shuddering" when the robot is near its target speed.
    if (pulse > 5)  return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < -5) return pulse - MOTOR_IGNORE_PULSE;
    return 0;
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    // Advanced Timers (TIM1/TIM8) require Main Output Enable
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim8);
}

void Motor_Stop(uint8_t brake)
{
    if (brake != 0) brake = 1;
    uint32_t val = brake * MOTOR_MAX_PULSE;

    PWM_M1_A = PWM_M1_B = val;
    PWM_M2_A = PWM_M2_B = val;
    PWM_M3_A = PWM_M3_B = val;
    PWM_M4_A = PWM_M4_B = val;
}

void Motor_Set_Pwm(uint8_t id, int16_t speed)
{
    // Note: Dead zone is now applied in PID_Calc_Motor for closed-loop control.
    // We keep it here as a safety for any direct PWM calls.
    int16_t pulse = speed;

    if (pulse >= MOTOR_MAX_PULSE)  pulse = MOTOR_MAX_PULSE;
    if (pulse <= -MOTOR_MAX_PULSE) pulse = -MOTOR_MAX_PULSE;

    switch (id)
    {
        case MOTOR_ID_M1:
            pulse = -pulse;
            if (pulse >= 0) { PWM_M1_A = pulse; PWM_M1_B = 0; }
            else { PWM_M1_A = 0; PWM_M1_B = -pulse; }
            break;
        case MOTOR_ID_M2:
            pulse = -pulse;
            if (pulse >= 0) { PWM_M2_A = pulse; PWM_M2_B = 0; }
            else { PWM_M2_A = 0; PWM_M2_B = -pulse; }
            break;
        case MOTOR_ID_M3:
            if (pulse >= 0) { PWM_M3_A = pulse; PWM_M3_B = 0; }
            else { PWM_M3_A = 0; PWM_M3_B = -pulse; }
            break;
        case MOTOR_ID_M4:
            if (pulse >= 0) { PWM_M4_A = pulse; PWM_M4_B = 0; }
            else { PWM_M4_A = 0; PWM_M4_B = -pulse; }
            break;
        default: break;
    }
}

void Motor_Hard_Brake(void)
{
    PWM_M1_A = PWM_M1_B = PWM_M2_A = PWM_M2_B = 0;
    PWM_M3_A = PWM_M3_B = PWM_M4_A = PWM_M4_B = 0;

    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_SET);
}
