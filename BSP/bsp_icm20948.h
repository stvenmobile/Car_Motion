#ifndef __BSP_ICM20948_H
#define __BSP_ICM20948_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>


/* --- Custom Data Structures --- */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} raw_data_t;

typedef struct {
    float x;
    float y;
    float z;
} axises_t;

typedef enum {
    ub_0 = 0,
    ub_1,
    ub_2,
    ub_3
} userbank_t;

/* --- Expose Calibration Data --- */
extern axises_t g_imu_accel;
extern axises_t g_imu_gyro;
extern float g_current_yaw;
extern float g_last_loop_time;

/* --- Function Prototypes --- */
void ICM20948_init(void);
void AK09916_init(void);
void ICM20948_gyro_calibration(void);
void ICM20948_accel_read(raw_data_t* data);
void ICM20948_gyro_read(raw_data_t* data);
bool AK09916_mag_read(raw_data_t* data);


void ICM20948_init();
void AK09916_init();

// read raw data.
void ICM20948_gyro_read(raw_data_t* data);
void ICM20948_accel_read(raw_data_t* data);
bool AK09916_mag_read(raw_data_t* data);

// Convert 16 bits ADC value to their unit.
void ICM20948_gyro_read_dps(axises_t* data);
void ICM20948_accel_read_g(axises_t* data);

bool AK09916_mag_read_uT(axises_t* data);
bool ICM20948_who_am_i();
bool AK09916_who_am_i();

void ICM20948_Read_Data_Handle(void);

/* --- Expose Data to UART --- */
extern raw_data_t g_raw_accel;
extern raw_data_t g_raw_gyro;
extern raw_data_t g_raw_mag;

#endif
