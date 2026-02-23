#ifndef BSP_MOTION_H_
#define BSP_MOTION_H_

#include <stdint.h>
#include <stdbool.h>
#include "bsp_pid.h"

#define ENABLE_REAL_WHEEL    (0)
#define YAW_ADJUST           (1)
#define YAW_NO_ADJUST        (0)

// Safety watchdog: stop motors if no V command is received within this many ms.
// 1000ms gives a 1-second window for manual serial-monitor testing while still
// providing a safe cut-off if the ROS2 serial bridge drops.
#define CMD_WATCHDOG_TIMEOUT_MS  1000

// Update to match the 96mm wheels
#define MECANUM_MINI_CIRCLE_MM       (301.6f)
#define MECANUM_MINI_APB             (150.0f)

typedef enum _stop_mode {
    STOP_FREE = 0,
    STOP_BRAKE
} stop_mode_t;

extern motor_data_t motor_data;
extern uint8_t g_start_ctrl;

typedef struct _car_data_t
{
    int16_t Vx;
    int16_t Vy;
    int16_t Vz;
} car_data_t;

typedef struct {
    uint8_t  active;           
    int32_t  target_dist_mm;   
    float    current_dist_mm;  
    int16_t  speed_mm_s;       
} motion_dist_t;

/* Function Prototypes */
void Motion_Stop(uint8_t brake);
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4);
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z);
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4);
void Motion_Handle(void);
void Motion_Get_Speed(car_data_t* car);
void Motion_Get_Encoder(void);

/* Data Accessors */
float Motion_Get_Circle_MM(void);
int16_t GetLatestVelocityX(void);
void Handle_Info_Encoders(void);
void Handle_Info_ResetEncoders(void);

#endif /* BSP_MOTION_H_ */
