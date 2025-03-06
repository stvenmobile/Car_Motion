#include "bsp_uart.h"
#include "bsp_motor.h"
#include "bsp.h"
#include "main.h"

// Motor 1
#define MOTOR_IN1_GPIO_Port GPIOC
#define MOTOR_IN1_Pin GPIO_PIN_6
#define MOTOR_IN2_GPIO_Port GPIOC
#define MOTOR_IN2_Pin GPIO_PIN_7

// Motor 2
#define MOTOR_IN3_GPIO_Port GPIOC
#define MOTOR_IN3_Pin GPIO_PIN_8
#define MOTOR_IN4_GPIO_Port GPIOC
#define MOTOR_IN4_Pin GPIO_PIN_9

// Motor 3
#define MOTOR_IN5_GPIO_Port GPIOA
#define MOTOR_IN5_Pin GPIO_PIN_11
#define MOTOR_IN6_GPIO_Port GPIOA
#define MOTOR_IN6_Pin GPIO_PIN_8

// Motor 4
#define MOTOR_IN7_GPIO_Port GPIOB
#define MOTOR_IN7_Pin GPIO_PIN_0
#define MOTOR_IN8_GPIO_Port GPIOB
#define MOTOR_IN8_Pin GPIO_PIN_1


// Ignore PWM dead band
static int16_t Motor_Ignore_Dead_Zone(int16_t pulse)
{
    if (pulse > 0) return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < 0) return pulse - MOTOR_IGNORE_PULSE;
    return 0;
}

// The PWM port of the motor is initialized
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
}

// All motors stopped
void Motor_Stop(uint8_t brake)
{
    if (brake != 0) brake = 1;
    PWM_M1_A = brake * MOTOR_MAX_PULSE;
    PWM_M1_B = brake * MOTOR_MAX_PULSE;
    PWM_M2_A = brake * MOTOR_MAX_PULSE;
    PWM_M2_B = brake * MOTOR_MAX_PULSE;
    PWM_M3_A = brake * MOTOR_MAX_PULSE;
    PWM_M3_B = brake * MOTOR_MAX_PULSE;
    PWM_M4_A = brake * MOTOR_MAX_PULSE;
    PWM_M4_B = brake * MOTOR_MAX_PULSE;
}

// Set motor speed, speed:± (3600-MOTOR_IGNORE_PULSE), 0 indicates stop
void Motor_Set_Pwm(uint8_t id, int16_t speed)
{
	speed = -speed;     // to account for reverse wiring of the motor.

    int16_t pulse = Motor_Ignore_Dead_Zone(speed);
    // Limit input
    if (pulse >= MOTOR_MAX_PULSE)
        pulse = MOTOR_MAX_PULSE;
    if (pulse <= -MOTOR_MAX_PULSE)
        pulse = -MOTOR_MAX_PULSE;

    switch (id)
    {
    case MOTOR_ID_M1:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            PWM_M1_A = pulse;
            PWM_M1_B = 0;
        }
        else
        {
            PWM_M1_A = 0;
            PWM_M1_B = -pulse;
        }
        break;
    }
    case MOTOR_ID_M2:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            PWM_M2_A = pulse;
            PWM_M2_B = 0;
        }
        else
        {
            PWM_M2_A = 0;
            PWM_M2_B = -pulse;
        }
        break;
    }

    case MOTOR_ID_M3:
    {
        if (pulse >= 0)
        {
            PWM_M3_A = pulse;
            PWM_M3_B = 0;
        }
        else
        {
            PWM_M3_A = 0;
            PWM_M3_B = -pulse;
        }
        break;
    }
    case MOTOR_ID_M4:
    {
        if (pulse >= 0)
        {
            PWM_M4_A = pulse;
            PWM_M4_B = 0;
        }
        else
        {
            PWM_M4_A = 0;
            PWM_M4_B = -pulse;
        }
        break;
    }

    default:
        break;
    }
}


void Motor_Hard_Brake(void)
{
    // Stop PWM output on all channels
    PWM_M1_A = 0;
    PWM_M1_B = 0;
    PWM_M2_A = 0;
    PWM_M2_B = 0;
    PWM_M3_A = 0;
    PWM_M3_B = 0;
    PWM_M4_A = 0;
    PWM_M4_B = 0;

    // Engage braking by shorting motor terminals
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_SET);

    Debug_Print("Hard Brake Engaged\r\n");
}
