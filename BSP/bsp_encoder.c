#include "bsp_encoder.h"
#include "bsp.h"
#include "bsp_uart.h"


int32_t g_Encoder_M1_Now = 0;
int32_t g_Encoder_M2_Now = 0;
int32_t g_Encoder_M3_Now = 0;
int32_t g_Encoder_M4_Now = 0;

static int32_t encoder_offset[4] = {0, 0, 0, 0};



// Read the encoder count, call every 10 milliseconds

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



// Returns the total count of encoders from boot up to now (single channel)
// Returns the total count including offset
int32_t Encoder_Get_Count_Now(uint8_t Motor_id)
{
    return Encoder_Get_Total_Count(Motor_id);
}


// 获取开机到现在总共的四路编码器计数。
// Get the total four - way encoder count up to now

void Encoder_Get_ALL(int32_t* Encoder_all)
{
	static uint32_t loop_count = 0;
	Encoder_all[0] = g_Encoder_M1_Now;
	Encoder_all[1] = g_Encoder_M2_Now;
	Encoder_all[2] = g_Encoder_M3_Now;
	Encoder_all[3] = g_Encoder_M4_Now;

	loop_count++;
	if (loop_count > 50)
	{
		//Debug_Print("Encoder_Get_ALL called: M1=%d, M2=%d, M3=%d, M4=%d\r\n",
		//	    	                g_Encoder_M1_Now, g_Encoder_M2_Now,
		//	    	                g_Encoder_M3_Now, g_Encoder_M4_Now);
		loop_count = 0;
	}

}

// 更新编码器的计数总值。需每10毫秒调用一次
// Update the count value of the encoder. call every 10 milliseconds
// Update the count value of the encoder. Call every 10 milliseconds
void Encoder_Update_Count(void)
{
    g_Encoder_M1_Now -= Encoder_Read_CNT(MOTOR_ID_M1);
    g_Encoder_M2_Now += Encoder_Read_CNT(MOTOR_ID_M2);
    g_Encoder_M3_Now += Encoder_Read_CNT(MOTOR_ID_M3);
    g_Encoder_M4_Now -= Encoder_Read_CNT(MOTOR_ID_M4);

    // Periodic Reset with Offset Tracking
    static uint32_t reset_counter = 0;
    reset_counter++;
    if (reset_counter >= 10000)  // Every 10 seconds (assuming 10ms call interval)
    {
        // Store accumulated counts in offset
        encoder_offset[0] += g_Encoder_M1_Now;
        encoder_offset[1] += g_Encoder_M2_Now;
        encoder_offset[2] += g_Encoder_M3_Now;
        encoder_offset[3] += g_Encoder_M4_Now;

        // Reset current counts to avoid overflow
        g_Encoder_M1_Now = 0;
        g_Encoder_M2_Now = 0;
        g_Encoder_M3_Now = 0;
        g_Encoder_M4_Now = 0;

        reset_counter = 0;
    }
}

// Get the total encoder count including the offset
int32_t Encoder_Get_Total_Count(uint8_t Motor_id)
{
    if (Motor_id == MOTOR_ID_M1) return encoder_offset[0] + g_Encoder_M1_Now;
    if (Motor_id == MOTOR_ID_M2) return encoder_offset[1] + g_Encoder_M2_Now;
    if (Motor_id == MOTOR_ID_M3) return encoder_offset[2] + g_Encoder_M3_Now;
    if (Motor_id == MOTOR_ID_M4) return encoder_offset[3] + g_Encoder_M4_Now;
    return 0;
}



// Initializing timer  初始化定时器
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);
}

