/* =============================================================================
 * FILE     : sd_card.h
 * BOARD    : STM32 NUCLEO-L4R5ZI-P
 * MODULE   : SD Card logging via SDMMC1 + FatFs
 * Author   : Batsirai Chris Rwatirera
 *
 * PURPOSE  : Log IMU and GPS batch data to SD card in CSV format
 *            Integrates with imu.c and gps.c batch buffers
 * =============================================================================
 */

#ifndef SD_CARD_H
#define SD_CARD_H

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
 * Call this once during startup after MX_FATFS_Init().
 * Creates root directory if needed, tests write access.
 */
uint8_t SD_Init(void);

/**
 * @brief  Open a new session on the SD card
 * @param  session_num: Session number (e.g. 1, 2, 3...)
 * @retval 1 = success, 0 = failure
 * 
 * Creates two new CSV files:
 *   - SESSION_XXX_IMU.CSV
 *   - SESSION_XXX_GPS.CSV
 * 
 * Writes CSV headers to both files.
 * Call this when a recording session starts (after IMU_Wake, GPS_Wake).
 */
uint8_t SD_OpenSession(uint16_t session_num);

/**
 * @brief  Write IMU batch to SD card
 * @param  batch: Pointer to IMU_Sample_t array
 * @param  count: Number of samples in batch (typically IMU_BATCH_SIZE = 100)
 * @retval 1 = success, 0 = failure
 * 
 * Appends batch data to SESSION_XXX_IMU.CSV in CSV format.
 * Call this when imu_batch_ready flag is set in main loop.
 */
uint8_t SD_WriteIMUBatch(IMU_Sample_t *batch, uint16_t count);

/**
 * @brief  Write GPS batch to SD card
 * @param  batch: Pointer to GPS_Sample_t array
 * @param  count: Number of samples in batch (typically GPS_BATCH_SIZE = 100)
 * @retval 1 = success, 0 = failure
 * 
 * Appends batch data to SESSION_XXX_GPS.CSV in CSV format.
 * Call this when gps_batch_ready flag is set in main loop.
 */
uint8_t SD_WriteGPSBatch(GPS_Sample_t *batch, uint16_t count);

/**
 * @brief  Close current session files
 * @retval 1 = success, 0 = failure
 * 
 * Flushes and closes both IMU and GPS CSV files.
 * Call this at end of recording session (before IMU_Sleep, GPS_Sleep).
 */
uint8_t SD_CloseSession(void);

/**
 * @brief  Get SD card status string for debugging
 * @retval Pointer to status string
 */
const char* SD_GetStatus(void);

#endif /* SD_CARD_H */
