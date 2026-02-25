#include "bsp_motion.h"
#include "bsp_encoder.h"
#include "bsp_pid.h"
#include "bsp_motor.h"
#include "string.h"




int g_Encoder_All_Now[MAX_MOTOR] = {0};
int g_Encoder_All_Last[MAX_MOTOR] = {0};

int g_Encoder_All_Offset[MAX_MOTOR] = {0};

uint8_t g_start_ctrl = 0;

car_data_t car_data;
motor_data_t motor_data;




// Example maximum linear speed in mm/s for 100% command.
// Adjust to your preference.
#define DIRECTION_MAX_SPEED_MM_S (1000.0f)

/**
 * @brief  Control the robot in one of the eight directions, plus STOP.
 * @param  dir: Direction command string (FWD, BWD, LFT, RGT, FDL, FDR, BDL, BDR, STP)
 * @param  speedPercent: Speed in [1..100], or 0 if STP.
 */
void Motion_Ctrl_Direction(const char* dir, int speedPercent)
{
    // 1) Clamp speed percentage to safe range
    if (speedPercent < 0)   speedPercent = 0;
    if (speedPercent > 100) speedPercent = 100;

    // 2) Convert percentage to mm/s
    float speed = (speedPercent / 100.0f) * DIRECTION_MAX_SPEED_MM_S;

    // 3) If this is a STOP command, just call Motion_Stop and return
    if (strcmp(dir, "STP") == 0)
    {
        Motion_Stop(STOP_BRAKE);
        return;
    }

    // 4) Decide Vx, Vy, and (optionally) Vz. Here we do 0 rotation for directions.
    //    Adjust sign convention as needed (some setups do Vx forward, Vy left, etc.)
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;  // No rotation in these simple commands

    if (strcmp(dir, "FWD") == 0)
    {
        // Forward
        vx = +speed;
        vy = 0.0f;
    }
    else if (strcmp(dir, "BWD") == 0)
    {
        // Backward
        vx = -speed;
        vy = 0.0f;
    }
    else if (strcmp(dir, "LFT") == 0)
    {
        // Left strafe
        vx = 0.0f;
        vy = +speed;  // Check sign is correct for your robot
    }
    else if (strcmp(dir, "RGT") == 0)
    {
        // Right strafe
        vx = 0.0f;
        vy = -speed;
    }
    else if (strcmp(dir, "FDL") == 0)
    {
        // Forward Diagonal Left
        // If you want actual diagonal speed to match FWD speed, you can multiply by 0.707
        // For now, let's keep it simple:
        vx = +speed;
        vy = +speed;
    }
    else if (strcmp(dir, "FDR") == 0)
    {
        vx = +speed;
        vy = -speed;
    }
    else if (strcmp(dir, "BDL") == 0)
    {
        vx = -speed;
        vy = +speed;
    }
    else if (strcmp(dir, "BDR") == 0)
    {
        vx = -speed;
        vy = -speed;
    }
    else
    {
        // Unknown direction
        // You might do nothing or just stop the motors
        Motion_Stop(STOP_BRAKE);
        return;
    }

    // 5) Invoke your existing motion control function
    //    (which does Mecanum kinematics and PID)
    Motion_Ctrl(vx, vy, vz);
}











// Control car movement, Motor_X=[-3600, 3600], beyond the range is invalid. 
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4)
{
    int16_t max_value = MOTOR_MAX_PULSE - MOTOR_IGNORE_PULSE;
    if (Motor_1 >= -max_value && Motor_1 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M1, Motor_1);
    }
    if (Motor_2 >= -max_value && Motor_2 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M2, Motor_2);
    }
    if (Motor_3 >= -max_value && Motor_3 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M3, Motor_3);
    }
    if (Motor_4 >= -max_value && Motor_4 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M4, Motor_4);
    }
}

// The car stopped
void Motion_Stop(uint8_t brake)
{
    Motion_Set_Speed(0, 0, 0, 0);
    PID_Clear_Motor(MAX_MOTOR);
    Motor_Stop(brake);
    g_start_ctrl = 0;
}


// Set speed speed mX=[-1000, 1000], unit: mm/s
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4)
{
    g_start_ctrl = 1;
    motor_data.speed_set[0] = speed_m1;
    motor_data.speed_set[1] = speed_m2;
    motor_data.speed_set[2] = speed_m3;
    motor_data.speed_set[3] = speed_m4;
    for (uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        PID_Set_Motor_Target(i, motor_data.speed_set[i]*1.0);
    }
}



// Read the current speed of each wheel from the encoder in mm/s
void Motion_Get_Speed(car_data_t* car)
{
    Motion_Get_Encoder();

    float circle_mm = Motion_Get_Circle_MM();

    float speed_m1 = (g_Encoder_All_Offset[0]) * 100 * circle_mm / (float)ENCODER_CIRCLE;
    float speed_m2 = (g_Encoder_All_Offset[1]) * 100 * circle_mm / (float)ENCODER_CIRCLE;
    float speed_m3 = (g_Encoder_All_Offset[2]) * 100 * circle_mm / (float)ENCODER_CIRCLE;
    float speed_m4 = (g_Encoder_All_Offset[3]) * 100 * circle_mm / (float)ENCODER_CIRCLE;
    float robot_APB = Motion_Get_APB();

    car->Vx = (speed_m1 + speed_m2 + speed_m3 + speed_m4) / 4;
    car->Vy = -(speed_m1 - speed_m2 - speed_m3 + speed_m4) / 4;
    car->Vz = -(speed_m1 + speed_m2 - speed_m3 - speed_m4) / 4.0f / robot_APB * 1000;

    if (g_start_ctrl)
    {
        motor_data.speed_mm_s[0] = speed_m1;
        motor_data.speed_mm_s[1] = speed_m2;
        motor_data.speed_mm_s[2] = speed_m3;
        motor_data.speed_mm_s[3] = speed_m4;
        PID_Calc_Motor(&motor_data);
    }
}

// Returns half of the sum of the current cart wheel axles
float Motion_Get_APB(void)
{
    return MECANUM_MINI_APB;
}

// Returns the number of millimeters at which the current wheel has been turned
float Motion_Get_Circle_MM(void)
{
    return MECANUM_MINI_CIRCLE_MM;
}


// Obtain encoder data and calculate the number of deviation pulses
void Motion_Get_Encoder(void)
{
    Encoder_Update_Count();
    Encoder_Get_ALL(g_Encoder_All_Now);

    for(uint8_t i = 0; i < MAX_MOTOR; i++)
    {
        g_Encoder_All_Offset[i] = g_Encoder_All_Now[i] - g_Encoder_All_Last[i];
	    g_Encoder_All_Last[i] = g_Encoder_All_Now[i];
    }
}

// Control car movement
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z)
{
    float robot_APB = Motion_Get_APB();
    float speed_lr = -V_y;
    float speed_fb = V_x;
    float speed_spin = -V_z / 1000.0f * robot_APB;
    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motion_Stop(STOP_BRAKE);
        return;
    }

    int speed_L1_setup = speed_fb + speed_lr + speed_spin;
    int speed_L2_setup = speed_fb - speed_lr + speed_spin;
    int speed_R1_setup = speed_fb - speed_lr - speed_spin;
    int speed_R2_setup = speed_fb + speed_lr - speed_spin;

    if (speed_L1_setup > 1000) speed_L1_setup = 1000;
    if (speed_L1_setup < -1000) speed_L1_setup = -1000;
    if (speed_L2_setup > 1000) speed_L2_setup = 1000;
    if (speed_L2_setup < -1000) speed_L2_setup = -1000;
    if (speed_R1_setup > 1000) speed_R1_setup = 1000;
    if (speed_R1_setup < -1000) speed_R1_setup = -1000;
    if (speed_R2_setup > 1000) speed_R2_setup = 1000;
    if (speed_R2_setup < -1000) speed_R2_setup = -1000;
    Motion_Set_Speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
}



// Motion control handle, called every 10ms, mainly processing speed related data
void Motion_Handle(void)
{
    Motion_Get_Speed(&car_data);

    if (g_start_ctrl)
    {
        Motion_Set_Pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1],
        		motor_data.speed_pwm[2], motor_data.speed_pwm[3]);
    }
}

