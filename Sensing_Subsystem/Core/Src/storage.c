/* =============================================================================
 * FILE     : storage.c
 * BOARD    : STM32 NUCLEO-L4R5ZI-P
 * MODULE   : SD Card logging via SPI + FatFs
 * Author   : Batsirai Chris Rwatirera
 *
 * PURPOSE  : Log IMU and GPS batch data to SD card in CSV format
 *            **POWER-LOSS RESILIENT** via f_sync() after every write
 *
 * UPDATED  : Cleaned up for working SD driver (slow SPI, no workarounds needed)
 * =============================================================================
 */

#include "storage.h"
#include "fatfs.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * PRIVATE STATE
 * ─────────────────────────────────────────────────────────────────────────────
 */
static FATFS   fs;
static FIL     imu_file;
static FIL     gps_file;
static uint8_t fs_mounted = 0;
static uint8_t session_open = 0;

static char imu_path[32];
static char gps_path[32];
static char status_msg[100];

extern UART_HandleTypeDef hlpuart1;

static void dbg(const char *msg)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);
}


/* ─────────────────────────────────────────────────────────────────────────────
 * Storage_Init - Mount SD card filesystem
 * ─────────────────────────────────────────────────────────────────────────────
 */
uint8_t Storage_Init(void)
{
    FRESULT fres;
    char msg[100];

    dbg("\r\n[STOR] Initializing SD card...\r\n");

    /* Mount filesystem - the slow SPI speed (125 kHz) handles all timing */
    dbg("[STOR] Attempting mount...\r\n");
    fres = f_mount(&fs, "0:", 1);

    if (fres != FR_OK) {
        snprintf(msg, sizeof(msg), "[STOR] ERROR: Mount failed (code %d)\r\n", fres);
        dbg(msg);
        return 0;
    }

    dbg("[STOR] Filesystem mounted.\r\n");
    fs_mounted = 1;

    return 1;
}


/* ─────────────────────────────────────────────────────────────────────────────
 * Storage_OpenSession - Create new session files with headers
 * ─────────────────────────────────────────────────────────────────────────────
 */
uint8_t Storage_OpenSession(uint16_t session_num)
{
    FRESULT fres;
    UINT bytes_written;

    if (!fs_mounted)
    {
        dbg("[STOR] ERROR: Filesystem not mounted.\r\n");
        return 0;
    }

    if (session_open)
    {
        dbg("[STOR] WARNING: Session already open. Closing.\r\n");
        Storage_CloseSession();
    }

    snprintf(status_msg, sizeof(status_msg),
             "[STOR] Opening session %u...\r\n", session_num);
    dbg(status_msg);

    /* Build file paths */
    snprintf(imu_path, sizeof(imu_path), "0:S%03u_IMU.CSV", session_num);
    snprintf(gps_path, sizeof(gps_path), "0:S%03u_GPS.CSV", session_num);

    /* ── CREATE IMU FILE ────────────────────────────────────────────────── */
    fres = f_open(&imu_file, imu_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        snprintf(status_msg, sizeof(status_msg),
                 "[STOR] ERROR: Cannot create %s (code %d)\r\n", imu_path, fres);
        dbg(status_msg);
        return 0;
    }

    /* Write IMU header */
    const char *imu_header =
        "timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,heading_deg,roll_deg,pitch_deg\r\n";

    fres = f_write(&imu_file, imu_header, strlen(imu_header), &bytes_written);
    if (fres != FR_OK || bytes_written != strlen(imu_header))
    {
        snprintf(status_msg, sizeof(status_msg),
                 "[STOR] ERROR: IMU header write failed (code %d, wrote %u bytes)\r\n",
                 fres, bytes_written);
        dbg(status_msg);
        f_close(&imu_file);
        return 0;
    }

    f_sync(&imu_file);

    snprintf(status_msg, sizeof(status_msg), "[STOR] Created %s\r\n", imu_path);
    dbg(status_msg);

    /* ── CREATE GPS FILE ────────────────────────────────────────────────── */
    fres = f_open(&gps_file, gps_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        snprintf(status_msg, sizeof(status_msg),
                 "[STOR] ERROR: Cannot create %s (code %d)\r\n", gps_path, fres);
        dbg(status_msg);
        f_close(&imu_file);
        return 0;
    }

    /* Write GPS header */
    const char *gps_header =
        "timestamp_ms,valid,utc_h,utc_m,utc_s,lat_deg,lon_deg,speed_knots\r\n";

    fres = f_write(&gps_file, gps_header, strlen(gps_header), &bytes_written);
    if (fres != FR_OK || bytes_written != strlen(gps_header))
    {
        snprintf(status_msg, sizeof(status_msg),
                 "[STOR] ERROR: GPS header write failed (code %d, wrote %u bytes)\r\n",
                 fres, bytes_written);
        dbg(status_msg);
        f_close(&imu_file);
        f_close(&gps_file);
        return 0;
    }

    f_sync(&gps_file);

    snprintf(status_msg, sizeof(status_msg), "[STOR] Created %s\r\n", gps_path);
    dbg(status_msg);

    session_open = 1;
    dbg("[STOR] Session files ready.\r\n");
    return 1;
}


/* ─────────────────────────────────────────────────────────────────────────────
 * Storage_WriteIMUBatch - Write 100 IMU samples to SD card
 * ─────────────────────────────────────────────────────────────────────────────
 */
uint8_t Storage_WriteIMUBatch(IMU_Sample_t *batch, uint16_t count)
{
    if (batch == NULL || count == 0 || !session_open) return 0;

    FRESULT fres;
    UINT bytes_written;

    /* Build entire batch as single string */
    static char batch_buffer[20000];  /* ~200 bytes per line × 100 lines */
    char line[200];
    batch_buffer[0] = '\0';

    for (uint16_t i = 0; i < count; i++)
    {
        IMU_Sample_t *s = &batch[i];

        snprintf(line, sizeof(line),
                 "%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                 (unsigned long)s->timestamp_ms,
                 s->ax, s->ay, s->az,
                 s->gx, s->gy, s->gz,
                 s->mx, s->my, s->mz,
                 s->heading_deg, s->roll_deg, s->pitch_deg);

        strcat(batch_buffer, line);
    }

    /* Write entire batch in ONE operation */
    fres = f_write(&imu_file, batch_buffer, strlen(batch_buffer), &bytes_written);
    if (fres != FR_OK) {
        return 0;
    }

    if (bytes_written != strlen(batch_buffer)) {
        return 0;
    }

    /* POWER-LOSS PROTECTION: sync after every batch */
    fres = f_sync(&imu_file);
    if (fres != FR_OK) {
        return 0;
    }

    return 1;
}


/* ─────────────────────────────────────────────────────────────────────────────
 * Storage_WriteGPSBatch - Write 80 GPS samples to SD card
 * ─────────────────────────────────────────────────────────────────────────────
 */
uint8_t Storage_WriteGPSBatch(GPS_Sample_t *batch, uint16_t count)
{
    if (batch == NULL || count == 0 || !session_open) return 0;

    FRESULT fres;
    UINT bytes_written;

    /* Build entire batch as single string */
    static char batch_buffer[12000];  /* ~150 bytes per line × 80 lines */
    char line[150];
    batch_buffer[0] = '\0';

    for (uint16_t i = 0; i < count; i++)
    {
        GPS_Sample_t *s = &batch[i];

        snprintf(line, sizeof(line),
                 "%lu,%u,%d,%d,%d,%.6f,%.6f,%.2f\r\n",
                 (unsigned long)s->timestamp_ms,
                 s->valid,
                 s->utc_h, s->utc_m, s->utc_s,
                 s->lat_deg, s->lon_deg,
                 s->speed_knots);

        strcat(batch_buffer, line);
    }

    /* Write entire batch in ONE operation */
    fres = f_write(&gps_file, batch_buffer, strlen(batch_buffer), &bytes_written);
    if (fres != FR_OK || bytes_written != strlen(batch_buffer)) {
        return 0;
    }

    /* POWER-LOSS PROTECTION: sync after every batch */
    fres = f_sync(&gps_file);
    if (fres != FR_OK) {
        return 0;
    }

    return 1;
}


/* ─────────────────────────────────────────────────────────────────────────────
 * Storage_CloseSession - Close files at end of recording session
 * ─────────────────────────────────────────────────────────────────────────────
 */
uint8_t Storage_CloseSession(void)
{
    if (!session_open) return 1;

    dbg("[STOR] Closing session files...\r\n");

    /* Final sync before closing */
    f_sync(&imu_file);
    f_sync(&gps_file);

    f_close(&imu_file);
    f_close(&gps_file);

    session_open = 0;

    snprintf(status_msg, sizeof(status_msg),
             "[STOR] Session closed. Files: %s, %s\r\n", imu_path, gps_path);
    dbg(status_msg);

    return 1;
}


/* ─────────────────────────────────────────────────────────────────────────────
 * Storage_GetStatus - For debugging
 * ─────────────────────────────────────────────────────────────────────────────
 */
const char* Storage_GetStatus(void)
{
    if (!fs_mounted)
        return "[STOR] Not mounted";
    if (!session_open)
        return "[STOR] Ready (no session open)";

    return status_msg;
}
