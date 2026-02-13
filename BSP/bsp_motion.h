#ifndef BSP_MOTION_H_
#define BSP_MOTION_H_

#include "bsp_pid.h"
#include "stdint.h"


#define ENABLE_REAL_WHEEL    (0)
#define YAW_ADJUST           (1)
#define YAW_NO_ADJUST        (0)


// The displacement of a wheel in one complete turn, Unit: mm 
#define MECANUM_MINI_CIRCLE_MM       (298.0f)
// 底盘电机间距之和的一半
// Half of the sum of the chassis motor spacing
#define MECANUM_MINI_APB             (150.0f)


// Stop mode, STOP_FREE: stop freely, STOP_BRAKE: brake
typedef enum _stop_mode {
    STOP_FREE = 0,
    STOP_BRAKE
} stop_mode_t;


extern motor_data_t motor_data;  // Declare as external


// The speed structure of the car
typedef struct _car_data_t
{
    int16_t Vx;
    int16_t Vy;
    int16_t Vz;
} car_data_t;

// Distance Control Structure
typedef struct {
    uint8_t  active;           // 1 = Move Distance Mode Active
    int32_t  target_dist_mm;   // Target distance in mm (signed)
    float    current_dist_mm;  // Accumulator for distance traveled
    int16_t  speed_mm_s;       // Speed to travel at
} motion_dist_t;



void Motion_Stop(uint8_t brake);
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4);
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z);
void Motion_Set_Distance(int32_t distance_mm, int16_t speed_mm_s); // New prototype

void Motion_Get_Encoder(void);
void Handle_Info_ResetEncoders();

void Motion_Handle(void);

void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4);
void Motion_Get_Speed(car_data_t* car);
float Motion_Get_Circle_MM(void);
float Motion_Get_APB(void);

void Handle_Velocity_4(int m1, int m2, int m3, int m4);
void Handle_Directional(const char* direction, int speed);

#endif /* BSP_MOTION_H_ */
