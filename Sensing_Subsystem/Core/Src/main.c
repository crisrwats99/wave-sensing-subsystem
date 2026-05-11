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

  // Initialize GPIO, UART, and I2C
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();

  // Helper function to print messages
  void print_msg(const char* msg) {
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 1000);
  }

  print_msg("\r\n========================================\r\n");
  print_msg("  IMU TEST - GY-87 Module\r\n");
  print_msg("========================================\r\n");

  // Test I2C bus - scan for devices
  print_msg("[TEST] Scanning I2C bus...\r\n");
  uint8_t found_devices = 0;
  for(uint8_t addr = 1; addr < 128; addr++) {
    if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
      char buf[50];
      sprintf(buf, "[TEST] Found device at address 0x%02X\r\n", addr);
      print_msg(buf);
      found_devices++;
    }
  }

  if(found_devices == 0) {
    print_msg("[ERROR] No I2C devices found! Check wiring.\r\n");
    while(1) {
      GPIOB->BSRR = GPIO_PIN_14;  // Red LED ON (error)
      for(volatile uint32_t i=0; i<500000; i++);
      GPIOB->BSRR = GPIO_PIN_14 << 16;  // Red LED OFF
      for(volatile uint32_t i=0; i<500000; i++);
    }
  }

  char found_msg[50];
  sprintf(found_msg, "[TEST] Found %d I2C device(s)\r\n\r\n", found_devices);
  print_msg(found_msg);

  // Expected devices:
  // 0x68 or 0x69 = MPU6050 (accelerometer + gyroscope)
  // 0x1E = HMC5883L (magnetometer)

  print_msg("[TEST] Attempting to read MPU6050 WHO_AM_I...\r\n");
  uint8_t mpu_addr = 0x68 << 1;  // MPU6050 default address
  uint8_t who_am_i_reg = 0x75;
  uint8_t who_am_i_val = 0;

  if(HAL_I2C_Mem_Read(&hi2c1, mpu_addr, who_am_i_reg, 1, &who_am_i_val, 1, 100) == HAL_OK) {
    char buf[60];
    sprintf(buf, "[SUCCESS] MPU6050 WHO_AM_I = 0x%02X (expected 0x68)\r\n", who_am_i_val);
    print_msg(buf);

    if(who_am_i_val == 0x68) {
      print_msg("[SUCCESS] MPU6050 responding correctly!\r\n");
      GPIOC->BSRR = GPIO_PIN_7;  // Green LED ON = success
    } else {
      print_msg("[WARNING] Unexpected WHO_AM_I value\r\n");
    }
  } else {
    print_msg("[ERROR] Failed to read MPU6050\r\n");
  }

  print_msg("\r\n========================================\r\n");
  print_msg("IMU Test Complete. Check results above.\r\n");
  print_msg("========================================\r\n\r\n");

  // Blink blue LED to show test is done
  while(1) {
    GPIOB->BSRR = GPIO_PIN_7;  // Blue LED ON
    for(volatile uint32_t i=0; i<2000000; i++);
    GPIOB->BSRR = GPIO_PIN_7 << 16;  // Blue LED OFF
    for(volatile uint32_t i=0; i<2000000; i++);
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
