#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp.h"

int32_t g_Encoder_M1_Now = 0;
int32_t g_Encoder_M2_Now = 0;
int32_t g_Encoder_M3_Now = 0;
int32_t g_Encoder_M4_Now = 0;

static int32_t encoder_offset[4] = {0, 0, 0, 0};

/**
 * @brief Reads the hardware timer counter.
 * Each motor ID reads the encoder timer on its own board port.
 * Port-to-timer mapping (from schematic):
 *   Port M1 (MOTOR_ID_M1 / LF) -> TIM2
 *   Port M2 (MOTOR_ID_M2 / LR) -> TIM4
 *   Port M3 (MOTOR_ID_M3 / RF) -> TIM5
 *   Port M4 (MOTOR_ID_M4 / RR) -> TIM3
 */
static int16_t Encoder_Read_CNT(uint8_t Motor_id)
{
    int16_t Encoder_TIM = 0;
    switch(Motor_id)
    {
        case MOTOR_ID_M1:  Encoder_TIM = (short)TIM2->CNT; TIM2->CNT = 0; break; // LF (Port M1)
        case MOTOR_ID_M2:  Encoder_TIM = (short)TIM4->CNT; TIM4->CNT = 0; break; // LR (Port M2)
        case MOTOR_ID_M3:  Encoder_TIM = (short)TIM5->CNT; TIM5->CNT = 0; break; // RF (Port M3)
        case MOTOR_ID_M4:  Encoder_TIM = (short)TIM3->CNT; TIM3->CNT = 0; break; // RR (Port M4)
        default:  break;
    }
    return Encoder_TIM;
}

void Encoder_Update_Count(void)
{
    /* Signs confirmed by manual ~1/4-turn forward rotation on each motor:
     *   M1 (FL, TIM2): hardware reads +678 for forward  → += yields positive ✓
     *   M2 (RL, TIM4): hardware reads -685 for forward  → -= yields positive ✓
     *   M3 (FR, TIM5): hardware reads -733 for forward  → -= yields positive ✓
     *   M4 (RR, TIM3): hardware reads +789 for forward  → += yields positive ✓
     * FL/RR share one mounting orientation (+), RL/FR share the mirrored orientation (-).
     */
    g_Encoder_M1_Now += Encoder_Read_CNT(MOTOR_ID_M1); // FL: hardware + for forward
    g_Encoder_M2_Now -= Encoder_Read_CNT(MOTOR_ID_M2); // RL: hardware - for forward
    g_Encoder_M3_Now -= Encoder_Read_CNT(MOTOR_ID_M3); // FR: hardware - for forward
    g_Encoder_M4_Now += Encoder_Read_CNT(MOTOR_ID_M4); // RR: hardware + for forward

    static uint32_t reset_counter = 0;
    if (++reset_counter >= 10000)
    {
        for(int i=0; i<4; i++) encoder_offset[i] += (&g_Encoder_M1_Now)[i];
        g_Encoder_M1_Now = g_Encoder_M2_Now = g_Encoder_M3_Now = g_Encoder_M4_Now = 0;
        reset_counter = 0;
    }
}

int32_t Encoder_Get_Total_Count(uint8_t Motor_id)
{
    if (Motor_id == MOTOR_ID_M1) return encoder_offset[0] + g_Encoder_M1_Now;
    if (Motor_id == MOTOR_ID_M2) return encoder_offset[1] + g_Encoder_M2_Now;
    if (Motor_id == MOTOR_ID_M3) return encoder_offset[2] + g_Encoder_M3_Now;
    if (Motor_id == MOTOR_ID_M4) return encoder_offset[3] + g_Encoder_M4_Now;
    return 0;
}

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);
}
