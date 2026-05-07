/* =============================================================================
 * FILE     : sd_card.c
 * BOARD    : STM32 NUCLEO-L4R5ZI-P
 * MODULE   : SD Card logging via SDMMC1 + FatFs
 * Author   : Batsirai Chris Rwatirera
 *
 * HOW IT WORKS:
 * ─────────────────────────────────────────────────────────────────────────────
 * 1. SD_Init() - mount SD card, verify write access
 * 2. SD_OpenSession(n) - create SESSION_001_IMU.CSV and SESSION_001_GPS.CSV
 * 3. SD_WriteIMUBatch() - append 100 IMU samples to IMU CSV
 * 4. SD_WriteGPSBatch() - append 100 GPS samples to GPS CSV
 * 5. SD_CloseSession() - close files, flush buffers
 *
 * INTEGRATION WITH MAIN.C:
 * ─────────────────────────────────────────────────────────────────────────────
 * - Call SD_Init() once at startup
 * - Call SD_OpenSession(session_number) when session starts
 * - In main loop: when imu_batch_ready == 1, call SD_WriteIMUBatch()
 * - In main loop: when gps_batch_ready == 1, call SD_WriteGPSBatch()
 * - Call SD_CloseSession() when session ends
 *
 * CSV FORMAT:
 * ─────────────────────────────────────────────────────────────────────────────
 * IMU CSV columns:
 *   timestamp_ms, ax, ay, az, gx, gy, gz, mx, my, mz, heading, roll, pitch
 * 
 * GPS CSV columns:
 *   timestamp_ms, valid, utc_h, utc_m, utc_s, lat_deg, lon_deg, speed_knots
 *
 * =============================================================================
 */

#include "sd_card.h"
#include "fatfs.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * FATFS HANDLES
 * ─────────────────────────────────────────────────────────────────────────────
 */
static FATFS   fs;              /* Filesystem object */
static FIL     imu_file;        /* IMU CSV file handle */
static FIL     gps_file;        /* GPS CSV file handle */
static uint8_t fs_mounted = 0;  /* Flag: is SD card mounted? */
static uint8_t session_open = 0; /* Flag: are session files open? */

/* File path buffers */
static char imu_path[32];  /* e.g. "SESSION_001_IMU.CSV" */
static char gps_path[32];  /* e.g. "SESSION_001_GPS.CSV" */

/* Status message buffer for debugging */
static char status_msg[100];

/* ─────────────────────────────────────────────────────────────────────────────
 * UART DEBUG (same as IMU and GPS modules)
 * ─────────────────────────────────────────────────────────────────────────────
 */
extern UART_HandleTypeDef hlpuart1;

static void dbg(const char *msg)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);
}


/* =============================================================================
 * SD_Init - Mount SD card and verify write access
 * =============================================================================
 */
uint8_t SD_Init(void)
{
    FRESULT fres;
    
    dbg("[SD] Initializing SD card...\r\n");
    
    /* Mount the SD card */
    fres = f_mount(&fs, "", 1);  /* "" = default drive, 1 = mount immediately */
    
    if (fres != FR_OK)
    {
        snprintf(status_msg, sizeof(status_msg), 
                 "[SD] ERROR: Mount failed (code %d)\r\n", fres);
        dbg(status_msg);
        
        /* Common error codes:
         * FR_NO_FILESYSTEM (13) = card not formatted as FAT
         * FR_DISK_ERR (1) = hardware error, card not inserted
         * FR_NOT_READY (3) = card not ready
         */
        if (fres == FR_NO_FILESYSTEM)
        {
            dbg("[SD] Card needs formatting. Use PC to format as FAT32.\r\n");
        }
        return 0;
    }
    
    fs_mounted = 1;
    dbg("[SD] Filesystem mounted.\r\n");
    
    /* Test write access by creating a test file */
    FIL test_file;
    fres = f_open(&test_file, "TEST.TXT", FA_CREATE_ALWAYS | FA_WRITE);
    
    if (fres != FR_OK)
    {
        snprintf(status_msg, sizeof(status_msg),
                 "[SD] ERROR: Cannot create test file (code %d)\r\n", fres);
        dbg(status_msg);
        return 0;
    }
    
    /* Write test string */
    UINT bytes_written;
    const char *test_str = "EEE4113F Group 23 - SD Card Test\r\n";
    fres = f_write(&test_file, test_str, strlen(test_str), &bytes_written);
    f_close(&test_file);
    
    if (fres != FR_OK || bytes_written != strlen(test_str))
    {
        dbg("[SD] ERROR: Test write failed.\r\n");
        return 0;
    }
    
    dbg("[SD] Write test OK. Card ready.\r\n");
    return 1;
}


/* =============================================================================
 * SD_OpenSession - Create new CSV files for this session
 * =============================================================================
 */
uint8_t SD_OpenSession(uint16_t session_num)
{
    FRESULT fres;
    UINT bytes_written;
    
    if (!fs_mounted)
    {
        dbg("[SD] ERROR: Filesystem not mounted. Call SD_Init() first.\r\n");
        return 0;
    }
    
    if (session_open)
    {
        dbg("[SD] WARNING: Session already open. Closing previous session.\r\n");
        SD_CloseSession();
    }
    
    snprintf(status_msg, sizeof(status_msg),
             "[SD] Opening session %u...\r\n", session_num);
    dbg(status_msg);
    
    /* Create filenames: SESSION_001_IMU.CSV, SESSION_001_GPS.CSV */
    snprintf(imu_path, sizeof(imu_path), "S%03u_IMU.CSV", session_num);
    snprintf(gps_path, sizeof(gps_path), "S%03u_GPS.CSV", session_num);
    
    /* ── Open IMU file ──────────────────────────────────────────────────── */
    fres = f_open(&imu_file, imu_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        snprintf(status_msg, sizeof(status_msg),
                 "[SD] ERROR: Cannot create %s (code %d)\r\n", imu_path, fres);
        dbg(status_msg);
        return 0;
    }
    
    /* Write IMU CSV header */
    const char *imu_header = 
        "timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,heading_deg,roll_deg,pitch_deg\r\n";
    fres = f_write(&imu_file, imu_header, strlen(imu_header), &bytes_written);
    
    if (fres != FR_OK)
    {
        dbg("[SD] ERROR: Failed to write IMU header.\r\n");
        f_close(&imu_file);
        return 0;
    }
    
    snprintf(status_msg, sizeof(status_msg), "[SD] Created %s\r\n", imu_path);
    dbg(status_msg);
    
    /* ── Open GPS file ──────────────────────────────────────────────────── */
    fres = f_open(&gps_file, gps_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        snprintf(status_msg, sizeof(status_msg),
                 "[SD] ERROR: Cannot create %s (code %d)\r\n", gps_path, fres);
        dbg(status_msg);
        f_close(&imu_file);  /* Close IMU file if GPS fails */
        return 0;
    }
    
    /* Write GPS CSV header */
    const char *gps_header = 
        "timestamp_ms,valid,utc_h,utc_m,utc_s,lat_deg,lon_deg,speed_knots\r\n";
    fres = f_write(&gps_file, gps_header, strlen(gps_header), &bytes_written);
    
    if (fres != FR_OK)
    {
        dbg("[SD] ERROR: Failed to write GPS header.\r\n");
        f_close(&imu_file);
        f_close(&gps_file);
        return 0;
    }
    
    snprintf(status_msg, sizeof(status_msg), "[SD] Created %s\r\n", gps_path);
    dbg(status_msg);
    
    session_open = 1;
    dbg("[SD] Session files ready.\r\n");
    return 1;
}


/* =============================================================================
 * SD_WriteIMUBatch - Write batch of IMU samples to CSV
 * =============================================================================
 */
uint8_t SD_WriteIMUBatch(IMU_Sample_t *batch, uint16_t count)
{
	if (batch == NULL || count == 0)
	{
	    dbg("[SD] ERROR: Invalid IMU batch.\r\n");
	    return 0;
	}

    if (!session_open)
    {
        dbg("[SD] ERROR: No session open. Call SD_OpenSession() first.\r\n");
        return 0;
    }
    
    FRESULT fres;
    UINT bytes_written;
    char line[200];  /* Buffer for one CSV line */
    
    /* Write each sample as a CSV row */
    for (uint16_t i = 0; i < count; i++)
    {
        IMU_Sample_t *s = &batch[i];
        
        /* Format: timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,heading,roll,pitch */
        snprintf(line, sizeof(line),
                 "%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                 (unsigned long)s->timestamp_ms,
                 s->ax, s->ay, s->az,
                 s->gx, s->gy, s->gz,
                 s->mx, s->my, s->mz,
                 s->heading_deg, s->roll_deg, s->pitch_deg);
        
        fres = f_write(&imu_file, line, strlen(line), &bytes_written);
        
        if (fres != FR_OK || bytes_written != strlen(line))
        {
            snprintf(status_msg, sizeof(status_msg),
                     "[SD] ERROR: IMU write failed at sample %u (code %d)\r\n", i, fres);
            dbg(status_msg);
            return 0;
        }
    }
    
    /* Flush to ensure data is written to card */
    if (f_sync(&imu_file) != FR_OK)
    {
        dbg("[SD] ERROR: IMU sync failed.\r\n");
        return 0;
    }
    
    snprintf(status_msg, sizeof(status_msg),
             "[SD] Wrote %u IMU samples\r\n", count);
    dbg(status_msg);
    
    return 1;
}


/* =============================================================================
 * SD_WriteGPSBatch - Write batch of GPS samples to CSV
 * =============================================================================
 */
uint8_t SD_WriteGPSBatch(GPS_Sample_t *batch, uint16_t count)
{
	if (batch == NULL || count == 0)
	{
	    dbg("[SD] ERROR: Invalid GPS batch.\r\n");
	    return 0;
	}

    if (!session_open)
    {
        dbg("[SD] ERROR: No session open. Call SD_OpenSession() first.\r\n");
        return 0;
    }
    
    FRESULT fres;
    UINT bytes_written;
    char line[150];  /* Buffer for one CSV line */
    
    /* Write each sample as a CSV row */
    for (uint16_t i = 0; i < count; i++)
    {
        GPS_Sample_t *s = &batch[i];
        
        /* Format: timestamp,valid,h,m,s,lat,lon,speed */
        snprintf(line, sizeof(line),
                 "%lu,%u,%d,%d,%d,%.6f,%.6f,%.2f\r\n",
                 (unsigned long)s->timestamp_ms,
                 s->valid,
                 s->utc_h, s->utc_m, s->utc_s,
                 s->lat_deg, s->lon_deg,
                 s->speed_knots);
        
        fres = f_write(&gps_file, line, strlen(line), &bytes_written);
        
        if (fres != FR_OK || bytes_written != strlen(line))
        {
            snprintf(status_msg, sizeof(status_msg),
                     "[SD] ERROR: GPS write failed at sample %u (code %d)\r\n", i, fres);
            dbg(status_msg);
            return 0;
        }
    }
    
    /* Flush to ensure data is written to card */
    if (f_sync(&gps_file) != FR_OK)
    {
        dbg("[SD] ERROR: GPS sync failed.\r\n");
        return 0;
    }
    
    snprintf(status_msg, sizeof(status_msg),
             "[SD] Wrote %u GPS samples\r\n", count);
    dbg(status_msg);
    
    return 1;
}


/* =============================================================================
 * SD_CloseSession - Close and flush session files
 * =============================================================================
 */
uint8_t SD_CloseSession(void)
{
    if (!session_open)
    {
        dbg("[SD] No session to close.\r\n");
        return 1;  /* Not an error */
    }
    
    dbg("[SD] Closing session files...\r\n");
    
    /* Close both files */
    f_close(&imu_file);
    f_close(&gps_file);
    
    session_open = 0;
    
    snprintf(status_msg, sizeof(status_msg),
             "[SD] Session closed. Files: %s, %s\r\n", imu_path, gps_path);
    dbg(status_msg);
    
    return 1;
}


/* =============================================================================
 * SD_GetStatus - Return current status string
 * =============================================================================
 */
const char* SD_GetStatus(void)
{
    if (!fs_mounted)
        return "[SD] Not mounted";
    if (!session_open)
        return "[SD] Ready (no session open)";
    
    return status_msg;
}
