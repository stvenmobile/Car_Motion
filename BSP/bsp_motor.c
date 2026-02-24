#include "bsp.h"
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "main.h"
#include <stdio.h>

/**
 * @brief  Adds a deadzone compensation to the PWM signal.
 * @param  pulse The raw PWM value from the PID controller.
 * @return The compensated PWM value.
 */
int16_t Motor_Ignore_Dead_Zone(int16_t pulse)
{
    if (pulse > 5)  return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < -5) return pulse - MOTOR_IGNORE_PULSE;
    return 0;
}

/**
 * @brief  Initializes the PWM timers for motor control.
 * @note   Timer 1 and Timer 8 are advanced control timers requiring MOE.
 *         Port M3 uses TIM1 main channels CH1/CH4 (PA8, PA11).
 *         Port M4 uses TIM1 COMPLEMENTARY N-channels CH2N/CH3N (PB0, PB1) —
 *         these require HAL_TIMEx_PWMN_Start, not HAL_TIM_PWM_Start.
 */
void Motor_Init(void)
{
    // TIM1: Port M3 uses main channels CH1 and CH4
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // TIM1: Port M4 uses COMPLEMENTARY N-channels CH2N (PB0) and CH3N (PB1).
    // The GPIO is wired to the N-channel outputs, so we must start the N-channels.
    // Initialise CCR to TIM1_ARR so the N-channel pins start LOW (0% duty).
    TIM1->CCR2 = TIM1_ARR;
    TIM1->CCR3 = TIM1_ARR;
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    // TIM8: Ports M1 and M2 (all main channels)
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    // Essential for Advanced Timers: Main Output Enable
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim8);
}

/**
 * @brief  Stops all motors.
 * @param  brake If non-zero, applies electronic braking.
 * @note   M1/M2/M3 use main channels: CCR=MOTOR_MAX_PULSE brakes, CCR=0 coasts.
 *         M4 uses N-channels (inverted): CCR=0 brakes (100% duty), CCR=TIM1_ARR coasts (0% duty).
 */
void Motor_Stop(uint8_t brake)
{
    uint32_t val = (brake != 0) ? MOTOR_MAX_PULSE : 0;

    // M1, M2, M3: main channels — CCR directly proportional to duty
    PWM_M1_A = PWM_M1_B = val;
    PWM_M2_A = PWM_M2_B = val;
    PWM_M3_A = PWM_M3_B = val;

    // M4: N-channels (inverted) — duty% = (TIM1_ARR - CCR) / TIM1_ARR
    //   brake → both leads high (100% duty) → CCR = 0
    //   coast → both leads low  (0%   duty) → CCR = TIM1_ARR
    uint32_t m4_val = (brake != 0) ? 0u : TIM1_ARR;
    PWM_M4_AN = m4_val;
    PWM_M4_BN = m4_val;
}

/**
 * @brief  Sets the PWM duty cycle for a specific logical motor.
 * @param  id The logical motor ID (M1-M4).
 * @param  speed The target speed/PWM value (-MOTOR_MAX_PULSE to MOTOR_MAX_PULSE).
 * @note   CLEAN 1-to-1 MAPPING (Rewired State):
 * M1 (Logic LF) -> Port M1
 * M2 (Logic LR) -> Port M2
 * M3 (Logic RF) -> Port M3
 * M4 (Logic RR) -> Port M4
 * * POLARITY LOGIC:
 * Left side (M1/M2) uses A=0, B=pulse for forward.
 * Right side (M3/M4) uses A=pulse, B=0 for forward (mechanically mirrored).
 */
void Motor_Set_Pwm(uint8_t id, int16_t speed)
{
    int16_t pulse = speed;

    // Clamp to hardware limits defined in bsp_motor.h
    if (pulse >= MOTOR_MAX_PULSE)  pulse = MOTOR_MAX_PULSE;
    if (pulse <= -MOTOR_MAX_PULSE) pulse = -MOTOR_MAX_PULSE;

    switch (id)
    {
        case MOTOR_ID_M1: // Port M1 (LF)
            if (pulse >= 0) { PWM_M1_A = 0; PWM_M1_B = pulse; }
            else { PWM_M1_A = -pulse; PWM_M1_B = 0; }
            break;

        case MOTOR_ID_M2: // Port M2 (LR)
            if (pulse >= 0) { PWM_M2_A = 0; PWM_M2_B = pulse; }
            else { PWM_M2_A = -pulse; PWM_M2_B = 0; }
            break;

        case MOTOR_ID_M3: // Port M3 (RF)
            if (pulse >= 0) { PWM_M3_A = pulse; PWM_M3_B = 0; }
            else { PWM_M3_A = 0; PWM_M3_B = -pulse; }
            break;

        case MOTOR_ID_M4: // Port M4 (RR) — N-channels CH2N/CH3N (inverted)
            // Pin HIGH when CNT >= CCR, so effective duty = (TIM1_ARR - CCR) / TIM1_ARR.
            // To drive forward at +pulse: A = pulse duty → CCR2 = TIM1_ARR - pulse; B = 0% → CCR3 = TIM1_ARR.
            // To drive reverse at -pulse: A = 0%         → CCR2 = TIM1_ARR;         B = |pulse| duty → CCR3 = TIM1_ARR - |pulse|.
            if (pulse >= 0) { PWM_M4_AN = TIM1_ARR - pulse; PWM_M4_BN = TIM1_ARR; }
            else            { PWM_M4_AN = TIM1_ARR;         PWM_M4_BN = TIM1_ARR + pulse; }
            break;

        default:
            break;
    }
}

/**
 * @brief  Diagnostic tool to test motors one by one with serial feedback.
 * @note   Uses vendor power constants to ensure physical movement.
 */
void Motor_Diagnostic_Sequence(void)
{
    printf("\r\n--- STARTING SEQUENTIAL MOTOR DIAGNOSTIC ---\r\n");
    HAL_Delay(1000);

    const char* names[] = {"M1 (LF)", "M2 (LR)", "M3 (RF)", "M4 (RR)"};

    for (int i = 0; i < 4; i++) {
        printf("Testing %s... ", names[i]);
        HAL_Delay(200);

        // Pulse the motor at a PWM level that overcomes static friction.
        // Motor_Set_Pwm writes directly to the CCR register — the PID dead-zone
        // path is not involved here — so we must supply the full value explicitly:
        // MOTOR_IGNORE_PULSE (1250) as the stiction floor + 200 supplemental = 1450.
        Motor_Set_Pwm(i, MOTOR_IGNORE_PULSE + 200);
        HAL_Delay(1000);
        Motor_Stop(STOP_BRAKE);

        printf("DONE\r\n");
        HAL_Delay(500);
    }
    printf("--- DIAGNOSTIC COMPLETE ---\r\n");
}

/**
 * @brief  Forcefully shorts all motor leads to max potential for maximum braking.
 * @note   M4 N-channels: CCR=0 puts pins at 100% duty = both leads shorted high.
 */
void Motor_Hard_Brake(void)
{
    PWM_M1_A = PWM_M1_B = MOTOR_MAX_PULSE;
    PWM_M2_A = PWM_M2_B = MOTOR_MAX_PULSE;
    PWM_M3_A = PWM_M3_B = MOTOR_MAX_PULSE;
    PWM_M4_AN = PWM_M4_BN = 0;   // N-channel: CCR=0 → 100% duty → brake
}
