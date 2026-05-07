/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file          : main.c
  * @brief         : Main program body
  * PROJECT        : EEE4113F Group 23 — Wave Direction, 2026
  * Authur         : Batsirai Chris Rwatirera
  * BOARD          : STM32 NUCLEO-L4R5ZI-P
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "comp.h"
#include "fatfs.h"
#include "i2c.h"
#include "usart.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu.h"             /* IMU module — MPU6050 + HMC5883L              */
#include "gps.h"             /* GPS module — TEL0132                         */
#include "sd_card.h"         /* SD card logging module                       */
#include <string.h>          /* For strlen() in log_msg                      */
#include <stdio.h>           /* For snprintf()                               */

/* ═════════════════════════════════════════════════════════════════════════════
 * DEBUG MODE FLAGS — Comment/uncomment these to enable debug modes
 * ═════════════════════════════════════════════════════════════════════════════
 * When DEBUG_IMU_PRINT or DEBUG_GPS_PRINT is defined (uncommented):
 *   - Real-time sensor readings are printed to serial terminal
 *   - SD card writing is DISABLED for that sensor
 *   - Useful for testing sensors individually
 * 
 * When both are commented out:
 *   - Normal operation: sensors write to SD card
 *   - No real-time printing (minimal serial output)
 * ═════════════════════════════════════════════════════════════════════════════
 */

//#define DEBUG_IMU_PRINT      /* Uncomment to print IMU readings in real-time */
//#define DEBUG_GPS_PRINT      /* Uncomment to print GPS readings in real-time */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t session_number = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
static void log_msg(const char *msg);
static void wait_ms(uint32_t duration_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =============================================================================
 * HELPER: debug print to serial terminal
 * Open terminal at 115200 baud on the NUCLEO COM port to see these messages.
 * =============================================================================
 */
static void log_msg(const char *msg)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg,
                      (uint16_t)strlen(msg), HAL_MAX_DELAY);
}


/* =============================================================================
 * HELPER: wait for a number of milliseconds while printing a heartbeat
 * Used during the long idle period between sessions.
 * Blinks LD2 (PB7) every 60 seconds so you can see the board is alive.
 * =============================================================================
 */
static void wait_ms(uint32_t duration_ms)
{
    uint32_t start      = HAL_GetTick();
    uint32_t last_blink = start;

    while (HAL_GetTick() - start < duration_ms)
    {
        /* Blink LED and print heartbeat every 60 seconds */
        if (HAL_GetTick() - last_blink >= 60000)
        {
            last_blink = HAL_GetTick();

            uint32_t remaining_min = (duration_ms - (HAL_GetTick() - start)) / 60000;
            char msg[60];
            snprintf(msg, sizeof(msg),
                "[MAIN] Sleeping. Next session in ~%lu min\r\n",
                (unsigned long)remaining_min);
            log_msg(msg);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
            HAL_Delay(300);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Enable FPU before HAL_Init() to avoid FPU warning */
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_I2C1_Init();
  MX_I2C2_SMBUS_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SAI1_Init();
  MX_SAI2_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* ── Startup banner ───────────────────────────────────────────────────── */
  log_msg("\r\n=========================================\r\n");
  log_msg("  EEE4113F Group 23 — Wave Direction\r\n");
  log_msg("=========================================\r\n");

  char cfg[200];
  snprintf(cfg, sizeof(cfg),
      "  Sample rate : %d Hz\r\n"
      "  Session     : %d min  (%lu samples)\r\n"
      "  Interval    : %d hours\r\n"
      "  Batch write : every %d samples\r\n"
      "=========================================\r\n",
      IMU_SAMPLE_RATE_HZ,
      IMU_SESSION_DURATION_MIN,
      (unsigned long)IMU_SAMPLES_PER_SESSION,
      IMU_SESSION_INTERVAL_HOURS,
      IMU_BATCH_SIZE);
  log_msg(cfg);


  /* ── ONE-TIME STARTUP INITIALISATION ─────────────────────────────────── */
  /* Each module is initialised once here and never again.
   * If any init fails, Error_Handler() halts with a blinking LED.        */

  /* IMU — MPU6050 + HMC5883L */
  log_msg("[MAIN] Initialising IMU...\r\n");
  if (!IMU_Init())
  {
      log_msg("[MAIN] IMU init FAILED. Halting.\r\n");
      Error_Handler();
  }

  /* Calibration — sensor must be still and flat for 5 seconds */
  log_msg("[MAIN] Calibrating IMU...\r\n");
  if (!IMU_Calibrate())
  {
      log_msg("[MAIN] IMU calibration FAILED. Halting.\r\n");
      Error_Handler();
  }

  /* GPS initialisation */
  log_msg("[MAIN] Initialising GPS...\r\n");
  if (!GPS_Init())
  {
      log_msg("[MAIN] GPS init FAILED. Halting.\r\n");
      Error_Handler();
  }

  /* SD card initialisation - SKIP if in debug print mode */
#if !defined(DEBUG_IMU_PRINT) && !defined(DEBUG_GPS_PRINT)
  log_msg("[MAIN] Initialising SD card...\r\n");
  if (!SD_Init())
  {
      log_msg("[MAIN] SD card init FAILED. Halting.\r\n");
      Error_Handler();
  }
#else
  log_msg("[MAIN] *** DEBUG MODE: SD card DISABLED ***\r\n");
  #ifdef DEBUG_IMU_PRINT
  log_msg("[MAIN] *** IMU real-time printing ENABLED ***\r\n");
  #endif
  #ifdef DEBUG_GPS_PRINT
  log_msg("[MAIN] *** GPS real-time printing ENABLED ***\r\n");
  #endif
#endif

  log_msg("[MAIN] All modules ready. Starting session loop.\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ══════════════════════════════════════════════════════════════════════
     * DAILY SESSION LOOP
     * Runs forever:
     *   1. Wake all sensors
     *   2. Record for IMU_SESSION_DURATION_MIN minutes
     *   3. Sleep all sensors
     *   4. Wait for the rest of the 24-hour period
     *   5. Repeat
     * ══════════════════════════════════════════════════════════════════════
     */

    /* ── WAKE ─────────────────────────────────────────────────────────── */
    session_number++;
    char sess_msg[60];
    snprintf(sess_msg, sizeof(sess_msg),
        "[MAIN] Session %u starting (%d min @ %d Hz)\r\n",
        session_number, IMU_SESSION_DURATION_MIN, IMU_SAMPLE_RATE_HZ);
    log_msg(sess_msg);

    IMU_Wake();
    GPS_Wake();

    /* Open new session files on SD card - SKIP in debug mode */
#if !defined(DEBUG_IMU_PRINT) && !defined(DEBUG_GPS_PRINT)
    if (!SD_OpenSession(session_number))
    {
        log_msg("[MAIN] ERROR: Failed to open SD session. Halting.\r\n");
        Error_Handler();
    }
#endif


    /* ── RECORD SESSION ──────────────────────────────────────────────── */
    uint32_t session_start = HAL_GetTick();
    uint32_t samples_written = 0;

    while (HAL_GetTick() - session_start < IMU_SESSION_MS)
    {
        /* ── IMU tick: non-blocking, reads sensor when deadline reached  */
        IMU_Tick();

        /* ── GPS tick: non-blocking, reads GPS when deadline reached     */
        GPS_Tick();

        /* ══════════════════════════════════════════════════════════════
         * IMU DATA HANDLING
         * ══════════════════════════════════════════════════════════════
         */
        if (imu_batch_ready)
        {
#ifdef DEBUG_IMU_PRINT
            /* ── DEBUG MODE: Print IMU readings in real-time ─────────── */
            log_msg("\r\n=== IMU BATCH (100 samples) ===\r\n");
            for (uint16_t i = 0; i < imu_batch_count; i++)
            {
                IMU_Sample_t *s = &imu_batch[i];
                char buf[200];
                snprintf(buf, sizeof(buf),
                    "[%lu] ax=%.3f ay=%.3f az=%.3f | gx=%.2f gy=%.2f gz=%.2f | "
                    "mx=%.1f my=%.1f mz=%.1f | hdg=%.1f roll=%.1f pitch=%.1f\r\n",
                    (unsigned long)s->timestamp_ms,
                    s->ax, s->ay, s->az,
                    s->gx, s->gy, s->gz,
                    s->mx, s->my, s->mz,
                    s->heading_deg, s->roll_deg, s->pitch_deg);
                log_msg(buf);
            }
            log_msg("=================================\r\n\r\n");
#else
            /* ── NORMAL MODE: Write to SD card ───────────────────────── */
            if (!SD_WriteIMUBatch(imu_batch, imu_batch_count))
            {
                log_msg("[MAIN] WARNING: IMU SD write failed.\r\n");
            }
#endif
            samples_written += imu_batch_count;
            IMU_ClearBatch();
        }

        /* ══════════════════════════════════════════════════════════════
         * GPS DATA HANDLING
         * ══════════════════════════════════════════════════════════════
         */
        if (gps_batch_ready)
        {
#ifdef DEBUG_GPS_PRINT
            /* ── DEBUG MODE: Print GPS readings in real-time ─────────── */
            log_msg("\r\n=== GPS BATCH (100 samples) ===\r\n");
            for (uint16_t i = 0; i < gps_batch_count; i++)
            {
                GPS_Sample_t *s = &gps_batch[i];
                char buf[150];
                snprintf(buf, sizeof(buf),
                    "[%lu] %02d:%02d:%02d | %.6f, %.6f | %.2f kn | fix=%s\r\n",
                    (unsigned long)s->timestamp_ms,
                    s->utc_h, s->utc_m, s->utc_s,
                    s->lat_deg, s->lon_deg,
                    s->speed_knots,
                    s->valid ? "YES" : "NO");
                log_msg(buf);
            }
            log_msg("================================\r\n\r\n");
#else
            /* ── NORMAL MODE: Write to SD card ───────────────────────── */
            if (!SD_WriteGPSBatch(gps_batch, gps_batch_count))
            {
                log_msg("[MAIN] WARNING: GPS SD write failed.\r\n");
            }
#endif
            GPS_ClearBatch();
        }
    }
    /* ── END OF SESSION ─────────────────────────────────────────────── */

    char end_msg[80];
    snprintf(end_msg, sizeof(end_msg),
        "[MAIN] Session %u complete. %lu samples written.\r\n",
        session_number, (unsigned long)samples_written);
    log_msg(end_msg);


    /* ── SLEEP ────────────────────────────────────────────────────────── */
    IMU_Sleep();
    GPS_Sleep();

    /* Close SD card session files - SKIP in debug mode */
#if !defined(DEBUG_IMU_PRINT) && !defined(DEBUG_GPS_PRINT)
    SD_CloseSession();
#endif


    /* ── WAIT until next session ─────────────────────────────────────── */
    /* Total interval = IMU_SESSION_INTERVAL_MS (e.g. 24 hours)
     * Time already used = session duration
     * Wait = interval - session duration = e.g. 23h 30min             */
    uint32_t elapsed   = HAL_GetTick() - session_start;
    uint32_t wait_time = 0;

    if (IMU_SESSION_INTERVAL_MS > elapsed)
    {
        wait_time = IMU_SESSION_INTERVAL_MS - elapsed;
    }

    char wait_msg[70];
    snprintf(wait_msg, sizeof(wait_msg),
        "[MAIN] Sleeping for %.1f hours until next session.\r\n",
        (float)wait_time / 3600000.0f);
    log_msg(wait_msg);

    wait_ms(wait_time);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK
                              |RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    /* Blink LD2 fast to indicate error */
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    HAL_Delay(150);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
