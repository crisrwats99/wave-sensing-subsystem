/* =============================================================================
 * FILE     : storage.h
 * BOARD    : STM32 NUCLEO-L4R5ZI-P
 * MODULE   : SD Card logging via SPI + FatFs
 * Author   : Batsirai Chris Rwatirera
 *
 * PURPOSE  : Log IMU and GPS batch data to SD card in CSV format
 *            **POWER-LOSS RESILIENT** via f_sync() after every write
 * =============================================================================
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include "imu.h"
#include "gps.h"

/* ─────────────────────────────────────────────────────────────────────────────
 * PUBLIC FUNCTIONS
 * ─────────────────────────────────────────────────────────────────────────────
 */

/**
 * @brief  Initialize SD card and FatFs filesystem
 * @retval 1 = success, 0 = failure
 *
 * Call this once during startup after MX_SPI1_Init() and MX_FATFS_Init().
 * Tests write access by creating a test file.
 */
uint8_t Storage_Init(void);

/**
 * @brief  Open a new session on the SD card
 * @param  session_num: Session number (e.g. 1, 2, 3...)
 * @retval 1 = success, 0 = failure
 *
 * Creates two new CSV files:
 *   - S001_IMU.CSV
 *   - S001_GPS.CSV
 *
 * Writes CSV headers to both files.
 * Call this when a recording session starts (after IMU_Wake, GPS_Wake).
 */
uint8_t Storage_OpenSession(uint16_t session_num);

/**
 * @brief  Write IMU batch to SD card (POWER-LOSS SAFE)
 * @param  batch: Pointer to IMU_Sample_t array
 * @param  count: Number of samples in batch (typically IMU_BATCH_SIZE = 100)
 * @retval 1 = success, 0 = failure
 *
 * Appends batch data to S001_IMU.CSV in CSV format.
 * Calls f_sync() after writing to ensure data is flushed to physical media.
 * **Max data loss on power cut: 1 batch (1 second of IMU data)**
 *
 * Call this when imu_batch_ready flag is set in main loop.
 */
uint8_t Storage_WriteIMUBatch(IMU_Sample_t *batch, uint16_t count);

/**
 * @brief  Write GPS batch to SD card (POWER-LOSS SAFE)
 * @param  batch: Pointer to GPS_Sample_t array
 * @param  count: Number of samples in batch (typically GPS_BATCH_SIZE = 80)
 * @retval 1 = success, 0 = failure
 *
 * Appends batch data to S001_GPS.CSV in CSV format.
 * Calls f_sync() after writing to ensure data is flushed to physical media.
 * **Max data loss on power cut: 1 batch (10 seconds of GPS data)**
 *
 * Call this when gps_batch_ready flag is set in main loop.
 */
uint8_t Storage_WriteGPSBatch(GPS_Sample_t *batch, uint16_t count);

/**
 * @brief  Close current session files
 * @retval 1 = success, 0 = failure
 *
 * Flushes and closes both IMU and GPS CSV files.
 * Call this at end of recording session (before IMU_Sleep, GPS_Sleep).
 */
uint8_t Storage_CloseSession(void);

/**
 * @brief  Get SD card status string for debugging
 * @retval Pointer to status string
 */
const char* Storage_GetStatus(void);

#endif /* STORAGE_H */
