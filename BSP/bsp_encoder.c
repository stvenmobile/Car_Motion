#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp.h"

int32_t g_Encoder_M1_Now = 0;
int32_t g_Encoder_M2_Now = 0;
int32_t g_Encoder_M3_Now = 0;
int32_t g_Encoder_M4_Now = 0;

static int32_t encoder_offset[4] = {0, 0, 0, 0};

/**
 * @brief Read raw timer count and reset it
 */
static int16_t Encoder_Read_CNT(uint8_t Motor_id)
{
    int16_t Encoder_TIM = 0;
    switch(Motor_id)
    {
        case MOTOR_ID_M1:  Encoder_TIM = (short)TIM2 -> CNT; TIM2 -> CNT = 0; break;
        case MOTOR_ID_M2:  Encoder_TIM = (short)TIM4 -> CNT; TIM4 -> CNT = 0; break;
        case MOTOR_ID_M3:  Encoder_TIM = (short)TIM5 -> CNT; TIM5 -> CNT = 0; break;
        case MOTOR_ID_M4:  Encoder_TIM = (short)TIM3 -> CNT; TIM3 -> CNT = 0; break;
        default:  break;
    }
    return Encoder_TIM;
}

/**
  * @brief  Updates global encoder counts.
  * @note   Signs flipped to ensure Forward Motion = Positive Count.
  * Based on manual test where forward push reported negative values.
  */
void Encoder_Update_Count(void)
{
    // Polarity adjusted so forward rotation increments the count
    g_Encoder_M1_Now -= Encoder_Read_CNT(MOTOR_ID_M1);
    g_Encoder_M2_Now += Encoder_Read_CNT(MOTOR_ID_M2);
    g_Encoder_M3_Now += Encoder_Read_CNT(MOTOR_ID_M3);
    g_Encoder_M4_Now -= Encoder_Read_CNT(MOTOR_ID_M4);

    static uint32_t reset_counter = 0;
    if (++reset_counter >= 10000)
    {
        encoder_offset[0] += g_Encoder_M1_Now;
        encoder_offset[1] += g_Encoder_M2_Now;
        encoder_offset[2] += g_Encoder_M3_Now;
        encoder_offset[3] += g_Encoder_M4_Now;
        g_Encoder_M1_Now = g_Encoder_M2_Now = g_Encoder_M3_Now = g_Encoder_M4_Now = 0;
        reset_counter = 0;
    }
}

/**
 * @brief Returns persistent total count for a motor
 */
int32_t Encoder_Get_Total_Count(uint8_t Motor_id)
{
    if (Motor_id == MOTOR_ID_M1) return encoder_offset[0] + g_Encoder_M1_Now;
    if (Motor_id == MOTOR_ID_M2) return encoder_offset[1] + g_Encoder_M2_Now;
    if (Motor_id == MOTOR_ID_M3) return encoder_offset[2] + g_Encoder_M3_Now;
    if (Motor_id == MOTOR_ID_M4) return encoder_offset[3] + g_Encoder_M4_Now;
    return 0;
}

/**
 * @brief Start timer encoder modes
 */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);
}
