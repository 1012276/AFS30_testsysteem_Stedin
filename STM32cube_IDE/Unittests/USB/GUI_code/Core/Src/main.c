/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"


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
#define GPIO_PORT_VBUS GPIOA   // De GPIO-poort voor VBUS (bijv. GPIOA)
#define GPIO_PIN_VBUS  GPIO_PIN_9   // De specifieke pin (bijv. pin 9 op GPIOA)
//#define BUFFER_SIZE 1024   // Vergroot de buffer naar 512 karakters
//#define WAITING_FOR_SETTINGS "STATUS: Wachten op instellingen\r\n"
//#define READY_STATUS "STATUS: Gereed\r\n"
//#define TEST_RUNNING "STATUS: Bezig met testprocedure\r\n"
//#define TEST_PAUSED "STATUS: Gepauzeerd\r\n"
//#define TEST_COMPLETED "STATUS: Voltooid\r\n"

#define BUFFER_SIZE 1024   // Vergroot de buffer naar 512 karakters
#define WAITING_FOR_SETTINGS "WACHT_OP_INSTELLINGEN"
#define READY_STATUS "GEREED"
#define TEST_RUNNING "BEZIG_MET_TEST"
#define TEST_PAUSED "GEPAUZEERD"
#define TEST_COMPLETED "VOLTOOID"
#define TEST_STOPPED "GESTOPT"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

char *data= "Hello world, USB is working \r\n";
uint8_t buffer [64];
char receivedData[BUFFER_SIZE];  // Buffer to store received data
uint32_t receivedLength = 0;  // Houd de lengte van de ontvangen data bij
float stroom_s1, stroom_s2, stroom_s3;

// Globale variabelen voor THD scenario 1
float thd_s1_rms;  // RMS stroom voor THD scenario 1
int thd_s1_3h, thd_s1_5h, thd_s1_7h, thd_s1_9h, thd_s1_11h, thd_s1_13h;  // Oneven harmonischen voor THD scenario 1

// Globale variabelen voor THD scenario 2
float thd_s2_rms;  // RMS stroom voor THD scenario 2
int thd_s2_3h, thd_s2_5h, thd_s2_7h, thd_s2_9h, thd_s2_11h, thd_s2_13h;  // Oneven harmonischen voor THD scenario 2

// Globale variabelen voor THD scenario 3
float thd_s3_rms;  // RMS stroom voor THD scenario 3
int thd_s3_3h, thd_s3_5h, thd_s3_7h, thd_s3_9h, thd_s3_11h, thd_s3_13h;  // Oneven harmonischen voor THD scenario 3
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void CheckReceivedData(void)
//{
//  /* Check if the received data contains the target string */
//  if (strstr(receivedData, "Hello world, USB is working") != NULL) {
//
//    /* Clear the buffer after matching */
//    memset(receivedData, '\0', sizeof(receivedData));
//    receivedLength = 0;
//  }
//
//}
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 4 */

// Functie om status door te geven via USB

void non_blocking_delay(uint32_t delay_ms) {
    uint32_t startTick = HAL_GetTick();
    while((HAL_GetTick() - startTick) < delay_ms) {
        // Tijdens het wachten kunnen USB-interrupts worden afgehandeld
    }
}

void send_status_to_gui(char* status_message) {
    CDC_Transmit_FS((uint8_t *)status_message, strlen(status_message));
}

// Functie om te checken of de knoppen voor starten, pauzeren of stoppen zijn ingedrukt


// Simuleer de ontvangst van instellingen en stuur statusupdates naar de GUI
void receive_settings_and_update_status(void) {
    // Wachten op instellingen
    send_status_to_gui(WAITING_FOR_SETTINGS);

    // Simuleer een vertraging voor het ontvangen van instellingen

    // Zodra instellingen zijn ontvangen, stuur de status "Gereed"

}

// Functie om de volledige testprocedure te beheren
void run_test_procedure(void) {
    // Start de testprocedure
    send_status_to_gui(TEST_RUNNING);

    // Simuleer het uitvoeren van de test

}
void CheckReceivedData(void)
{
  // Simuleer het ontvangen van data (normaliter via USB/UART)
  if (receivedLength > 0) {
    parse_received_data(receivedData);  // Verwerk de ontvangen gegevens
    memset(receivedData, 0, BUFFER_SIZE);  // Clear de buffer na verwerking
    receivedLength = 0;
  }
}

// Functie om de ontvangen data te parsen
void parse_received_data(char* data)
{
    // Verwacht dataformaat: "STROOM_S1=100.000;STROOM_S2=200.000;THD_S1_RMS=50.000;..."
    // Verwacht dataformaat:
    // "STROOM_S1=100.000;STROOM_S2=200.000;STROOM_S3=300.000;THD_S1_RMS=50.000;THD_S1_3H=15;...THD_S3_13H=10;"

    sscanf(data,
           "STROOM_S1=%f;STROOM_S2=%f;STROOM_S3=%f;THD_S1_RMS=%f;THD_S1_3H=%d;THD_S1_5H=%d;"
           "THD_S1_7H=%d;THD_S1_9H=%d;THD_S1_11H=%d;THD_S1_13H=%d;"
           "THD_S2_RMS=%f;THD_S2_3H=%d;THD_S2_5H=%d;THD_S2_7H=%d;THD_S2_9H=%d;THD_S2_11H=%d;THD_S2_13H=%d;"
           "THD_S3_RMS=%f;THD_S3_3H=%d;THD_S3_5H=%d;THD_S3_7H=%d;THD_S3_9H=%d;THD_S3_11H=%d;THD_S3_13H=%d;",
           &stroom_s1, &stroom_s2, &stroom_s3,
           &thd_s1_rms, &thd_s1_3h, &thd_s1_5h, &thd_s1_7h, &thd_s1_9h, &thd_s1_11h, &thd_s1_13h,
           &thd_s2_rms, &thd_s2_3h, &thd_s2_5h, &thd_s2_7h, &thd_s2_9h, &thd_s2_11h, &thd_s2_13h,
           &thd_s3_rms, &thd_s3_3h, &thd_s3_5h, &thd_s3_7h, &thd_s3_9h, &thd_s3_11h, &thd_s3_13h);


	send_status_to_gui(READY_STATUS);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_I2S1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */





  /////// KLOK FREQEUNTIE OP 12 MHZ zetten////
  /////// KLOK FREQEUNTIE OP 12 MHZ zetten////
  /////// KLOK FREQEUNTIE OP 12 MHZ zetten////
  /////// KLOK FREQEUNTIE OP 12 MHZ zetten////


  /////// KLOK FREQEUNTIE OP 8 MHZ zetten////
  /////// KLOK FREQEUNTIE OP 8 MHZ zetten voor nucleo board////

  while (!(HAL_GPIO_ReadPin(GPIO_PORT_VBUS, GPIO_PIN_VBUS) == GPIO_PIN_SET));
//      // VBUS is present, USB can be initialized

 // simulatie tesprocedure met alle statussen
  HAL_Delay(2000);
  receive_settings_and_update_status();
//  HAL_Delay(6000);
//  send_status_to_gui(READY_STATUS);
//  HAL_Delay(5000);
//  run_test_procedure();
//  HAL_Delay(5000);
//  send_status_to_gui(TEST_PAUSED);
//  HAL_Delay(5000);
//  run_test_procedure();
//  HAL_Delay(2000);
//  send_status_to_gui(TEST_STOPPED);
//  HAL_Delay(5000);
//  send_status_to_gui(TEST_COMPLETED);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1|RCC_PERIPHCLK_I2S_APB2;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 75;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 6;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLI2S;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC_RESET_GPIO_Port, DAC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, RODE_LED_Pin|GROENE_LED_Pin|BLAUWE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DE_OUTPUT_Pin|RE_OUTPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DAC_RESET_Pin */
  GPIO_InitStruct.Pin = DAC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DAC_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : START_KNOP_Pin PAUZEER_KNOP_Pin STOP_KNOP_Pin */
  GPIO_InitStruct.Pin = START_KNOP_Pin|PAUZEER_KNOP_Pin|STOP_KNOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RODE_LED_Pin GROENE_LED_Pin BLAUWE_LED_Pin */
  GPIO_InitStruct.Pin = RODE_LED_Pin|GROENE_LED_Pin|BLAUWE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : DE_OUTPUT_Pin RE_OUTPUT_Pin */
  GPIO_InitStruct.Pin = DE_OUTPUT_Pin|RE_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
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
