#include "bsp_key.h"
#include "bsp.h"


// Determine if the key is pressed, press to return KEY_PRESS, release to return KEY_RELEASE  
static uint8_t Key1_is_Press(void)
{
	if (!HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin))
	{
		return KEY_PRESS; // 如果按键被按下，则返回KEY_PRESS
	}
	return KEY_RELEASE;   // 如果按键是松开状态，则返回KEY_RELEASE
}


// Read the state of key K1, press down to return KEY_PRESS, release to return key_release. 
// mode: setting mode, 0: press down to return KEY_PRESS;  1: KEY_PRESS is returned only once  
uint8_t Key1_State(uint8_t mode)
{
	static uint16_t key1_state = 0;

	if (Key1_is_Press() == KEY_PRESS)
	{
		if (key1_state < (mode + 1) * 2)
		{
			key1_state++;
		}
	}
	else
	{
		key1_state = 0;
	}
	if (key1_state == 2)
	{
		return KEY_PRESS;
	}
	return KEY_RELEASE;
}


/*********************************************END OF FILE**********************/
