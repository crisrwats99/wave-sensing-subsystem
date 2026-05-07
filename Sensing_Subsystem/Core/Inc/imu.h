/* =============================================================================
 * FILE  : imu.h
 * BOARD : STM32 NUCLEO-L4R5ZI-P
 * SENSOR: GY-87  (MPU6050 + HMC5883L)
 * =============================================================================
 */

#ifndef IMU_H
#define IMU_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

/* ── Change these to adjust behaviour ───────────────────────────────────── */
#define IMU_SAMPLE_RATE_HZ         100          /* Hz — 80 is standard      */
#define IMU_PERIOD_MS              (1000 / IMU_SAMPLE_RATE_HZ)
#define IMU_SESSION_DURATION_MIN   30           /* minutes per session       */
#define IMU_SESSION_MS             ((uint32_t)(IMU_SESSION_DURATION_MIN) * 60UL * 1000UL)
#define IMU_SAMPLES_PER_SESSION    ((uint32_t)(IMU_SAMPLE_RATE_HZ) * (uint32_t)(IMU_SESSION_DURATION_MIN) * 60UL)
#define IMU_SESSION_INTERVAL_HOURS 24           /* hours between sessions    */
#define IMU_SESSION_INTERVAL_MS    ((uint32_t)(IMU_SESSION_INTERVAL_HOURS) * 3600UL * 1000UL)
#define IMU_BATCH_SIZE             100          /* samples before SD write   */
#define IMU_CALIB_SAMPLES          500          /* samples for calibration   */
#define IMU_DEBUG_PRINT            1            /* 1=print to serial, 0=off  */
/* ─────────────────────────────────────────────────────────────────────────── */

/* One sample — one CSV row */
typedef struct {
    uint32_t timestamp_ms;
    float ax, ay, az;       /* acceleration in g           */
    float gx, gy, gz;       /* angular rate in deg/s       */
    float mx, my, mz;       /* magnetic field in microtesla*/
    float heading_deg;      /* compass heading 0-360 deg   */
    float roll_deg;         /* complementary filter roll   */
    float pitch_deg;        /* complementary filter pitch  */
} IMU_Sample_t;

/* Shared with SD card module */
extern IMU_Sample_t imu_batch[IMU_BATCH_SIZE];
extern uint16_t     imu_batch_count;
extern uint8_t      imu_batch_ready;

/* Public functions */

uint8_t IMU_Init(void);       /* call once at startup                        */
uint8_t IMU_Calibrate(void);  /* call once after Init, sensor must be still  */
void    IMU_Sleep(void);      /* call between sessions                       */
void    IMU_Wake(void);       /* call before each session                    */
void    IMU_Tick(void);       /* call from main loop every iteration         */
void    IMU_ClearBatch(void); /* call after SD card writes the batch         */

#endif
