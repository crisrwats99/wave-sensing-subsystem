/* =============================================================================
 * FILE     : gps.h
 * BOARD    : STM32 NUCLEO-L4R5ZI-P
 * MODULE   : TEL0132 GPS (AT6558 chip)
 * PROTOCOL : UART (9600 baud NMEA sentences)
 * Author   : Batsirai Chris Rwatirera
 * =============================================================================
 */

#ifndef GPS_H
#define GPS_H

#include <stdint.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * CONFIGURATION
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define GPS_SAMPLE_RATE_HZ      8      /* 8 Hz sampling (125 ms period)      */
#define GPS_PERIOD_MS           (1000 / GPS_SAMPLE_RATE_HZ)
#define GPS_BATCH_SIZE          80     /* 10 seconds worth at 8 Hz           */

#define GPS_DEBUG_PRINT         0      /* 1 = print to serial every 1s       */


/* ─────────────────────────────────────────────────────────────────────────────
 * GPS SAMPLE STRUCTURE
 * One sample = one timestamp + latest GPS data
 * ─────────────────────────────────────────────────────────────────────────────
 */
typedef struct {
    uint32_t timestamp_ms;      /* HAL_GetTick() when sample was taken       */
    uint8_t  valid;             /* 1 if GPS has fix, 0 if no fix             */
    int      utc_h;             /* UTC hour (0-23)                           */
    int      utc_m;             /* UTC minute (0-59)                         */
    int      utc_s;             /* UTC second (0-59)                         */
    float    lat_deg;           /* Latitude in decimal degrees (+ = North)   */
    float    lon_deg;           /* Longitude in decimal degrees (+ = East)   */
    float    speed_knots;       /* Speed over ground in knots                */
} GPS_Sample_t;


/* ─────────────────────────────────────────────────────────────────────────────
 * BATCH BUFFER (same pattern as IMU)
 * Main code reads these when gps_batch_ready == 1
 * ─────────────────────────────────────────────────────────────────────────────
 */
extern GPS_Sample_t gps_batch[GPS_BATCH_SIZE];
extern uint16_t     gps_batch_count;
extern uint8_t      gps_batch_ready;


/* ─────────────────────────────────────────────────────────────────────────────
 * PUBLIC FUNCTIONS (same pattern as IMU)
 * ─────────────────────────────────────────────────────────────────────────────
 */

/**
 * @brief Initialize GPS module and start UART reception
 * @return 1 on success, 0 on failure
 */
uint8_t GPS_Init(void);

/**
 * @brief Call from main loop — non-blocking, reads GPS when ready
 * Fills gps_batch[] and sets gps_batch_ready when batch full
 */
void GPS_Tick(void);

/**
 * @brief Clear batch after SD write is done
 */
void GPS_ClearBatch(void);

/**
 * @brief Put GPS into low-power sleep mode
 */
void GPS_Sleep(void);

/**
 * @brief Wake GPS from sleep
 */
void GPS_Wake(void);

#endif /* GPS_H */
