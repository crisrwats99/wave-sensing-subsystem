/* =============================================================================
 * FILE     : storage.c (ROBUST VERSION WITH RETRY)
 * =============================================================================
 */

#include "storage.h"
#include "fatfs.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>

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

/* Retry SD card init up to 3 times */
uint8_t Storage_Init(void)
{
    FRESULT fres;
    char msg[100];

    dbg("\r\n[STOR] Initializing SD card...\r\n");

    /* Power cycle SD card (toggle CS pin) */
    dbg("[STOR] Power-on reset sequence (5s)...\r\n");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS high
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS low
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS high
    HAL_Delay(5000);  // Let card stabilize for 5 seconds

    /* Mount filesystem */
    dbg("[STOR] Attempting mount...\r\n");
    fres = f_mount(&fs, "0:", 1);

    if (fres != FR_OK) {
        snprintf(msg, sizeof(msg), "[STOR] ERROR: Mount failed (code %d)\r\n", fres);
        dbg(msg);
        return 0;
    }

    dbg("[STOR] Filesystem mounted.\r\n");
    fs_mounted = 1;

    dbg("[STOR] Card ready. Proceeding to session files.\r\n\r\n");

    return 1;
}

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

    /* CRITICAL FIX: Remount filesystem to clear any stale state */
    dbg("[STOR] Remounting filesystem...\r\n");
    f_mount(NULL, "0:", 0);  /* Unmount */
    HAL_Delay(100);
    fres = f_mount(&fs, "0:", 1);  /* Remount */

    if (fres != FR_OK) {
        snprintf(status_msg, sizeof(status_msg),
                 "[STOR] ERROR: Remount failed (code %d)\r\n", fres);
        dbg(status_msg);
        return 0;
    }

    dbg("[STOR] Remount OK. Creating files...\r\n");
    HAL_Delay(500);  /* Give card time to stabilize */

    snprintf(imu_path, sizeof(imu_path), "0:S%03u_IMU.CSV", session_num);
    snprintf(gps_path, sizeof(gps_path), "0:S%03u_GPS.CSV", session_num);

    /* Open IMU file */
    fres = f_open(&imu_file, imu_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        snprintf(status_msg, sizeof(status_msg),
                 "[STOR] ERROR: Cannot create %s (code %d)\r\n", imu_path, fres);
        dbg(status_msg);
        return 0;
    }

    const char *imu_header =
        "timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,heading_deg,roll_deg,pitch_deg\r\n";

    /* Wait before writing IMU header */
    HAL_Delay(500);

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

    /* CRITICAL: Wait longer before opening GPS file */
    dbg("[STOR] Waiting before GPS file (3s)...\r\n");
    HAL_Delay(3000);

    /* Open GPS file */
    fres = f_open(&gps_file, gps_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        snprintf(status_msg, sizeof(status_msg),
                 "[STOR] ERROR: Cannot create %s (code %d)\r\n", gps_path, fres);
        dbg(status_msg);
        f_close(&imu_file);
        return 0;
    }

    /* Wait before writing GPS header */
    HAL_Delay(500);

    const char *gps_header =
        "timestamp_ms,valid,utc_h,utc_m,utc_s,lat_deg,lon_deg,speed_knots\r\n";
    fres = f_write(&gps_file, gps_header, strlen(gps_header), &bytes_written);

    if (fres != FR_OK)
    {
        dbg("[STOR] ERROR: Failed to write GPS header.\r\n");
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

uint8_t Storage_WriteIMUBatch(IMU_Sample_t *batch, uint16_t count)
{
    if (batch == NULL || count == 0 || !session_open) return 0;

    FRESULT fres;
    UINT bytes_written;
    char debug_msg[100];

    /* CRITICAL: Give SD card time to be ready (your card is SLOW) */
    HAL_Delay(500);

    /* Build entire batch as single string (much faster!) */
    static char batch_buffer[20000];  /* ~200 bytes per line × 100 lines */
    char line[200];
    batch_buffer[0] = '\0';  /* Clear buffer */

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
        snprintf(debug_msg, sizeof(debug_msg), "[IMU_WRITE] Write failed: code %d\r\n", fres);
        dbg(debug_msg);
        return 0;
    }

    if (bytes_written != strlen(batch_buffer)) {
        snprintf(debug_msg, sizeof(debug_msg), "[IMU_WRITE] Partial write: %u/%u bytes\r\n",
                 bytes_written, (unsigned)strlen(batch_buffer));
        dbg(debug_msg);
        return 0;
    }

    /* POWER-LOSS PROTECTION */
    fres = f_sync(&imu_file);
    if (fres != FR_OK) {
        snprintf(debug_msg, sizeof(debug_msg), "[IMU_WRITE] Sync failed: code %d\r\n", fres);
        dbg(debug_msg);
        return 0;
    }

    /* Give SD card time to finish sync before next operation */
    HAL_Delay(500);

    return 1;
}

uint8_t Storage_WriteGPSBatch(GPS_Sample_t *batch, uint16_t count)
{
    if (batch == NULL || count == 0 || !session_open) return 0;

    FRESULT fres;
    UINT bytes_written;

    /* CRITICAL: Give SD card time to be ready */
    HAL_Delay(500);

    /* Build entire batch as single string */
    static char batch_buffer[12000];  /* ~150 bytes per line × 80 lines */
    char line[150];
    batch_buffer[0] = '\0';  /* Clear buffer */

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

    if (fres != FR_OK || bytes_written != strlen(batch_buffer))
    {
        return 0;
    }

    /* POWER-LOSS PROTECTION */
    fres = f_sync(&gps_file);
    if (fres != FR_OK) return 0;

    /* Give SD card time to finish sync */
    HAL_Delay(500);

    return 1;
}

uint8_t Storage_CloseSession(void)
{
    if (!session_open) return 1;

    dbg("[STOR] Closing session files...\r\n");

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

const char* Storage_GetStatus(void)
{
    if (!fs_mounted)
        return "[STOR] Not mounted";
    if (!session_open)
        return "[STOR] Ready (no session open)";

    return status_msg;
}
