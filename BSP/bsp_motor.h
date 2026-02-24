#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include "main.h"

/* PWM Register Mapping:
 * Port M1: TIM8 CH1/CH2   (main channels, PA8/PA11 via TIM8 remapped to PC6-PC9)
 * Port M2: TIM8 CH3/CH4   (main channels)
 * Port M3: TIM1 CH4/CH1   (main channels → PA11/PA8)
 * Port M4: TIM1 CH2N/CH3N (COMPLEMENTARY N-channels → PB0/PB1, NOT main CH2/CH3)
 *          The N-channels are inverted: pin HIGH when CNT >= CCR.
 *          Use TIM1_ARR to invert: duty% on pin = (TIM1_ARR - CCR) / TIM1_ARR.
 *          Motor_Set_Pwm and Motor_Stop handle this inversion for MOTOR_ID_M4.
 */
#define TIM1_ARR    3599u   /* TIM1 auto-reload value (= Period in MX_TIM1_Init) */

#define PWM_M1_A  TIM8->CCR1
#define PWM_M1_B  TIM8->CCR2

#define PWM_M2_A  TIM8->CCR3
#define PWM_M2_B  TIM8->CCR4

#define PWM_M3_A  TIM1->CCR4
#define PWM_M3_B  TIM1->CCR1

/* M4 N-channel CCR registers — written directly only by Motor_Init/Stop/HardBrake.
 * All other code must go through Motor_Set_Pwm(MOTOR_ID_M4, ...) to get inversion. */
#define PWM_M4_AN  TIM1->CCR2   /* CH2N = PB0 */
#define PWM_M4_BN  TIM1->CCR3   /* CH3N = PB1 */

/* Vendor Standard Power Constants */
#define MOTOR_IGNORE_PULSE  (1250)
#define MOTOR_MAX_PULSE     (3200)
#define MAX_MOTOR            4

typedef enum {
    MOTOR_ID_M1 = 0, // Logic Left Front
    MOTOR_ID_M2,     // Logic Left Rear
    MOTOR_ID_M3,     // Logic Right Front
    MOTOR_ID_M4,     // Logic Right Rear
} Motor_ID;

int16_t Motor_Ignore_Dead_Zone(int16_t pulse);
void Motor_Init(void);
void Motor_Set_Pwm(uint8_t id, int16_t speed);
void Motor_Stop(uint8_t brake);
void Motor_Hard_Brake(void);
void Motor_Diagnostic_Sequence(void);

#endif
