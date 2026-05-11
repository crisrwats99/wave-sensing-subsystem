/* =============================================================================
 * FILE     : gps.c
 * BOARD    : STM32 NUCLEO-L4R5ZI-P
 * MODULE   : TEL0132 GPS (AT6558 chip) via LPUART1 at 9600 baud
 * Author   : Batsirai Chris Rwatirera
 *
 * =============================================================================
 */

#include "gps.h"
#include "minmea.h"
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * UART RX BUFFER - stores incoming NMEA bytes from interrupt
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define RX_BUF_SIZE  512
static uint8_t  rx_buffer[RX_BUF_SIZE];
static uint16_t rx_write = 0;    /* ISR writes here */
static uint16_t rx_read  = 0;    /* GPS_Tick reads here */
static uint8_t  rx_byte;         /* Single byte for HAL_UART_Receive_IT */

/* ─────────────────────────────────────────────────────────────────────────────
 * NMEA LINE BUFFER - one complete sentence
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define LINE_SIZE  100
static char    line[LINE_SIZE];
static uint8_t line_idx = 0;

/* ─────────────────────────────────────────────────────────────────────────────
 * BATCH BUFFER (extern in gps.h, main.c reads this)
 * ─────────────────────────────────────────────────────────────────────────────
 */
GPS_Sample_t gps_batch[GPS_BATCH_SIZE];
uint16_t     gps_batch_count = 0;
uint8_t      gps_batch_ready = 0;

/* ─────────────────────────────────────────────────────────────────────────────
 * PRIVATE STATE
 * ─────────────────────────────────────────────────────────────────────────────
 */
static uint32_t next_tick = 0;
static uint8_t  tick_started = 0;

/* Latest valid GPS data (cached between samples) */
static uint8_t last_valid = 0;
static int     last_h = 0, last_m = 0, last_s = 0;
static float   last_lat = 0.0f, last_lon = 0.0f, last_speed = 0.0f;

#if GPS_DEBUG_PRINT
static uint32_t last_debug_ms = 0;
#endif

/* HAL handles from main.c */
extern UART_HandleTypeDef huart2;  /* GPS connected to USART2 */


/* ─────────────────────────────────────────────────────────────────────────────
 * DEBUG PRINT (same as IMU)
 * ─────────────────────────────────────────────────────────────────────────────
 */
static void dbg(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);
}


/* =============================================================================
 * UART RX CALLBACK - called when one byte arrives
 * Stores byte in ring buffer, re-arms UART for next byte
 * =============================================================================
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        rx_buffer[rx_write] = rx_byte;
        rx_write = (rx_write + 1) % RX_BUF_SIZE;
        
        /* Buffer overflow check */
        if (rx_write == rx_read) {
            rx_read = (rx_read + 1) % RX_BUF_SIZE;  /* Drop oldest byte */
        }
        
        /* Re-arm for next byte */
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}


/* =============================================================================
 * EXTRACT ONE NMEA LINE from ring buffer
 * Returns 1 if complete line extracted, 0 otherwise
 * Line starts with '$' and ends with '\n'
 * =============================================================================
 */
static uint8_t get_line(void)
{
    while (rx_read != rx_write)
    {
        uint8_t c = rx_buffer[rx_read];
        rx_read = (rx_read + 1) % RX_BUF_SIZE;
        
        if (c == '$')  /* Start of sentence */
        {
            line_idx = 0;
            line[line_idx++] = c;
        }
        else if (line_idx > 0)
        {
            if (line_idx < LINE_SIZE - 1)
            {
                line[line_idx++] = c;
                
                if (c == '\n')  /* End of sentence */
                {
                    line[line_idx] = '\0';
                    line_idx = 0;
                    return 1;
                }
            }
            else
            {
                line_idx = 0;  /* Line too long, discard */
            }
        }
    }
    return 0;
}


/* =============================================================================
 * GPS_Init - Start UART reception
 * =============================================================================
 */
uint8_t GPS_Init(void)
{
    dbg("[GPS] Initializing TEL0132...\r\n");
    
    /* Start UART interrupt reception */
    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1) != HAL_OK)
    {
        dbg("[GPS] ERROR: UART start failed\r\n");
        return 0;
    }
    
    dbg("[GPS] UART started at 9600 baud\r\n");
    dbg("[GPS] Waiting 5s for GPS module to boot...\r\n");
    HAL_Delay(5000);
    
    dbg("[GPS] Ready. 8Hz sampling.\r\n");
    return 1;
}


/* =============================================================================
 * GPS_Tick - NON-BLOCKING, call from main loop
 * 
 * Same pattern as IMU_Tick():
 *   - Check if sample deadline reached
 *   - Read latest GPS data
 *   - Store in batch
 *   - Set batch_ready when full
 * =============================================================================
 */
void GPS_Tick(void)
{
    /* Don't sample if batch is full and waiting for SD write */
    if (gps_batch_ready) return;
    
    /* Initialize tick on first call */
    if (!tick_started)
    {
        next_tick = HAL_GetTick();
        tick_started = 1;
    }
    
    /* Process incoming NMEA data while waiting for next sample time */
    if (get_line())
    {
        /* Parse RMC sentence (has time, position, speed) */
        if (minmea_sentence_id(line, false) == MINMEA_SENTENCE_RMC)
        {
            struct minmea_sentence_rmc rmc;
            if (minmea_parse_rmc(&rmc, line))
            {
                /* Cache latest GPS data */
                last_valid = rmc.valid ? 1 : 0;
                last_h = rmc.time.hours;
                last_m = rmc.time.minutes;
                last_s = rmc.time.seconds;
                last_lat = minmea_tocoord(&rmc.latitude);
                last_lon = minmea_tocoord(&rmc.longitude);
                last_speed = minmea_tofloat(&rmc.speed);
            }
        }
    }
    
    /* Check if sample deadline reached */
    if ((int32_t)(HAL_GetTick() - next_tick) < 0) return;
    
    next_tick += GPS_PERIOD_MS;  /* Schedule next sample 125 ms later */
    
    /* ── SAMPLE: store current GPS data in batch ───────────────────────── */
    GPS_Sample_t *s = &gps_batch[gps_batch_count];
    s->timestamp_ms = HAL_GetTick();
    s->valid = last_valid;
    s->utc_h = last_h;
    s->utc_m = last_m;
    s->utc_s = last_s;
    s->lat_deg = last_lat;
    s->lon_deg = last_lon;
    s->speed_knots = last_speed;
    
    gps_batch_count++;
    
    if (gps_batch_count >= GPS_BATCH_SIZE)
    {
        gps_batch_ready = 1;  /* Signal to main: batch is full, write it */
    }
    
    /* ── DEBUG MONITOR (same as IMU) ─────────────────────────────────── */
#if GPS_DEBUG_PRINT
    if (HAL_GetTick() - last_debug_ms >= 1000)
    {
        last_debug_ms = HAL_GetTick();
        char buf[120];
        snprintf(buf, sizeof(buf),
            "GPS: %02d:%02d:%02d | %.6f %.6f | %.2f kn | fix=%s\r\n",
            last_h, last_m, last_s,
            last_lat, last_lon, last_speed,
            last_valid ? "YES" : "NO");
        dbg(buf);
    }
#endif
}


/* =============================================================================
 * GPS_ClearBatch - called by main after SD write finishes
 * =============================================================================
 */
void GPS_ClearBatch(void)
{
    gps_batch_count = 0;
    gps_batch_ready = 0;
}


/* =============================================================================
 * GPS_Sleep - power down GPS module
 * No specific sleep mode
 * We just stop sampling. Module stays on but draws less power when idle.
 * =============================================================================
 */
void GPS_Sleep(void)
{
    tick_started = 0;
    dbg("[GPS] Sleeping.\r\n");
}


/* =============================================================================
 * GPS_Wake - resume sampling
 * =============================================================================
 */
void GPS_Wake(void)
{
    tick_started = 0;
    dbg("[GPS] Awake.\r\n");
}
