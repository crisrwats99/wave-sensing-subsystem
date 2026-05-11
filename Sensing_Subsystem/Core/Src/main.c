/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file          : main.c
  * @brief         : Main program body - IMU Test
  * PROJECT        : EEE4113F Group 23 — Wave Direction, 2026
  * Author         : Batsirai Chris Rwatirera
  * BOARD          : STM32 NUCLEO-L4R5ZI-P
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
#include <string.h>
#include <stdio.h>
#include "imu.h"
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
extern UART_HandleTypeDef hlpuart1;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Enable FPU before HAL_Init() */
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // LED setup via direct register access
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;  // Enable GPIOB clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;  // Enable GPIOC clock

  // Configure PB7 (Blue LED), PB14 (Red LED) as outputs
  GPIOB->MODER &= ~(3UL << (7*2));
  GPIOB->MODER |= (1UL << (7*2));
  GPIOB->MODER &= ~(3UL << (14*2));
  GPIOB->MODER |= (1UL << (14*2));

  // Configure PC7 (Green LED) as output
  GPIOC->MODER &= ~(3UL << (7*2));
  GPIOC->MODER |= (1UL << (7*2));

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

    // Initialize peripherals
    MX_GPIO_Init();
    MX_LPUART1_UART_Init();
    MX_I2C1_Init();

    // Helper function to print messages
    void print_msg(const char* msg) {
      HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 1000);
    }

    // Print startup banner
    print_msg("\r\n========================================\r\n");
    print_msg("  EEE4113F Group 23 — Wave Direction\r\n");
    print_msg("=========================================\r\n");

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
    print_msg(cfg);

    /* ── ONE-TIME STARTUP INITIALISATION ───────────────────────────────── */

    // Initialize IMU module
    print_msg("[MAIN] Initialising IMU...\r\n");
    if (!IMU_Init())
    {
        print_msg("[MAIN] IMU init FAILED. Halting.\r\n");
        while(1) {
          GPIOB->BSRR = GPIO_PIN_14;  // Red LED blink = error
          HAL_Delay(200);
          GPIOB->BSRR = GPIO_PIN_14 << 16;
          HAL_Delay(200);
        }
    }

    // Calibrate IMU (sensor must be still and flat for 5 seconds)
    print_msg("[MAIN] Calibrating IMU (5 sec - keep still)...\r\n");
    if (!IMU_Calibrate())
    {
        print_msg("[MAIN] IMU calibration FAILED. Halting.\r\n");
        while(1) {
          GPIOB->BSRR = GPIO_PIN_14;  // Red LED blink = error
          HAL_Delay(200);
          GPIOB->BSRR = GPIO_PIN_14 << 16;
          HAL_Delay(200);
        }
    }

    print_msg("[MAIN] All modules ready. Starting session loop.\r\n\r\n");

    GPIOC->BSRR = GPIO_PIN_7;  // Green LED ON = initialization complete

    /* ── DAILY SESSION LOOP ──────────────────────────────────────────── */

    uint16_t session_number = 0;

    while(1)
    {
      /* ── START SESSION ───────────────────────────────────────────────── */
      session_number++;
      char sess_msg[80];
      snprintf(sess_msg, sizeof(sess_msg),
          "[MAIN] Session %u starting (%d min @ %d Hz)\r\n",
          session_number, IMU_SESSION_DURATION_MIN, IMU_SAMPLE_RATE_HZ);
      print_msg(sess_msg);

      IMU_Wake();

      // TODO: GPS_Wake();
      // TODO: SD_OpenSession(session_number);

      /* ── RECORD FOR 30 MINUTES AT 100 Hz ──────────────────────────── */
      uint32_t session_start = HAL_GetTick();
      uint32_t samples_written = 0;

      while (HAL_GetTick() - session_start < IMU_SESSION_MS)
      {
          GPIOB->BSRR = GPIO_PIN_7;  // Blue LED ON during sampling

          // Non-blocking tick - reads sensors when 100 Hz deadline reached
          IMU_Tick();

          // Check if batch is ready (every 100 samples)
          if (imu_batch_ready)
          {
              // TODO: Write to SD card
              // SD_WriteIMUBatch(imu_batch, imu_batch_count);

              samples_written += imu_batch_count;
              IMU_ClearBatch();

              GPIOB->BSRR = GPIO_PIN_7 << 16;  // Blue LED OFF after write
          }
      }

      /* ── END SESSION ──────────────────────────────────────────────────── */
      char end_msg[80];
      snprintf(end_msg, sizeof(end_msg),
          "[MAIN] Session %u complete. %lu samples recorded.\r\n",
          session_number, (unsigned long)samples_written);
      print_msg(end_msg);

      IMU_Sleep();
      // TODO: GPS_Sleep();
      // TODO: SD_CloseSession();

      /* ── WAIT 23.5 HOURS UNTIL NEXT SESSION ────────────────────────── */
      uint32_t elapsed = HAL_GetTick() - session_start;
      uint32_t wait_time = 0;

      if (IMU_SESSION_INTERVAL_MS > elapsed)
      {
          wait_time = IMU_SESSION_INTERVAL_MS - elapsed;
      }

      char wait_msg[70];
      snprintf(wait_msg, sizeof(wait_msg),
          "[MAIN] Sleeping for %.1f hours until next session.\r\n\r\n",
          (float)wait_time / 3600000.0f);
      print_msg(wait_msg);

      HAL_Delay(wait_time);  // Wait remainder of 24 hours
    }

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

  // This code never runs because we're stuck in the while(1) above

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

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

  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  __disable_irq();
  while (1)
  {
    GPIOB->BSRR = GPIO_PIN_7;  // Blue LED blink fast = error
    for(volatile uint32_t i=0; i<300000; i++);
    GPIOB->BSRR = GPIO_PIN_7 << 16;
    for(volatile uint32_t i=0; i<300000; i++);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
