/* =============================================================================
 * FILE : MPU6050.h
 * ORIGINAL AUTHOR : Controllerstech / community STM32 HAL port
 * MODIFIED BY : EEE4113F Group 23, 2026
 *
 * CHANGE FROM ORIGINAL:
 *   stm32f1xx_hal.h  →  stm32l4xx_hal.h
 *   The original library targeted the STM32F1 family.
 *   Our board is the NUCLEO-L4R5ZI-P (STM32L4R5ZI), which belongs to the
 *   STM32L4 family. Changing the HAL header is the only modification needed —
 *   all register addresses and I2C logic are identical across chip families.
 *
 * WHAT THIS HEADER EXPOSES:
 *   Three functions are all you need to interact with the MPU-6050:
 *     MPU6050_Init()           — wake sensor, configure ranges, verify WHO_AM_I
 *     MPU6050_Read_Accel()     — read 6 accel bytes, return calibrated g values
 *     MPU6050_Read_Gyro()      — read 6 gyro bytes, return °/s values
 *
 * I2C DEPENDENCY:
 *   MPU6050.c uses 'extern I2C_HandleTypeDef hi2c1' — it expects CubeIDE's
 *   generated hi2c1 handle to exist in main.c. No other setup is needed.
 * =============================================================================
 */

#ifndef MPU6050_H
#define MPU6050_H

/* Changed from stm32f1xx_hal.h to stm32l4xx_hal.h for NUCLEO-L4R5ZI-P */
#include "stm32l4xx_hal.h"

/*
 * MPU6050_Init
 * ─────────────
 * Wakes the MPU-6050 from power-on sleep and configures it for wave sensing.
 * Must be called once before any Read function.
 * Internally:
 *   - Reads WHO_AM_I register (0x75) and checks for 0x68 — confirms I2C works
 *   - Writes 0x00 to PWR_MGMT_1 (0x6B) — clears SLEEP bit, wakes sensor
 *   - Writes to SMPLRT_DIV, ACCEL_CONFIG, GYRO_CONFIG — sets ranges
 */
void MPU6050_Init(void);

/*
 * MPU6050_Read_Accel
 * ───────────────────
 * Reads accelerometer X, Y, Z axes in one 6-byte I2C burst.
 * Stores results (in g) at the float addresses passed in.
 *
 * Parameters:
 *   Ax, Ay, Az — pointers to float variables that receive the results
 *
 * Usage:
 *   float ax, ay, az;
 *   MPU6050_Read_Accel(&ax, &ay, &az);
 *   // ax is now acceleration in g (e.g. 0.012 for near-zero X motion)
 */
void MPU6050_Read_Accel(float *Ax, float *Ay, float *Az);

/*
 * MPU6050_Read_Gyro
 * ──────────────────
 * Reads gyroscope X, Y, Z axes in one 6-byte I2C burst.
 * Stores results (in degrees per second) at the float addresses passed in.
 *
 * Parameters:
 *   Gx, Gy, Gz — pointers to float variables that receive the results
 *
 * Usage:
 *   float gx, gy, gz;
 *   MPU6050_Read_Gyro(&gx, &gy, &gz);
 *   // gx is now angular rate in °/s (e.g. 0.21 when nearly stationary)
 */
void MPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz);

#endif /* MPU6050_H */
