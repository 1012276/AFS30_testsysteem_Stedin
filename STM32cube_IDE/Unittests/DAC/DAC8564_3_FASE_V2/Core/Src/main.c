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
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_TIMEOUT 1000
#define DAC8564_SEQUENTIANALY_WRITE_UPDATE 0x10
#define VREF 2.5
//#define AMPLITUDE 0.7
#define PI 3.14159265
#define SAMPLE_RATE 48000
#define FREQUENCY 50
#define NUM_SAMPLES (SAMPLE_RATE / FREQUENCY)
#define PHASE_SHIFT_120_DEG (2.0 * PI / 3.0)  // 120-degree phase shift
#define PHASE_SHIFT_240_DEG (4.0 * PI / 3.0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


volatile uint16_t sample_index = 0;
volatile uint32_t seconds_elapsed = 0;
volatile uint8_t dac_update_flag = 0;
volatile uint32_t sample_counter = 0;
volatile uint8_t current_scenario = 1;

float AMPLITUDE;

float HARMONIC_3, HARMONIC_5, HARMONIC_7, HARMONIC_9, HARMONIC_11, HARMONIC_13;


uint16_t sine_wave_A_scenario[6][NUM_SAMPLES];
uint16_t sine_wave_B_scenario[6][NUM_SAMPLES];
uint16_t sine_wave_C_scenario[6][NUM_SAMPLES];

uint16_t* current_sine_wave_A;
uint16_t* current_sine_wave_B;
uint16_t* current_sine_wave_C;





float stroom_s1 = 0.0;
float stroom_s2 = 0.0;
float stroom_s3 = 0.0;

// Globale variabelen voor THD scenario's
float thd_s1_rms = 0.0, thd_s2_rms = 0.0, thd_s3_rms = 0.0;
int thd_s1_3h = 0, thd_s1_5h = 0, thd_s1_7h = 0, thd_s1_9h = 0,
    thd_s1_11h = 0, thd_s1_13h = 0;

int thd_s2_3h = 0, thd_s2_5h = 0, thd_s2_7h = 0, thd_s2_9h = 0,
    thd_s2_11h = 0, thd_s2_13h = 0;

int thd_s3_3h = 0, thd_s3_5h = 0, thd_s3_7h = 0, thd_s3_9h = 0,
    thd_s3_11h = 0, thd_s3_13h = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2S1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void calculate_three_phase_sine_wave_samples(uint16_t* wave_A, uint16_t* wave_B, uint16_t* wave_C) {
    for (int i = 0; i < NUM_SAMPLES; i++) {
        double t = (double)i / SAMPLE_RATE;

        // Phase A with harmonic distortion
        double sample_A = AMPLITUDE * sin(2 * PI * FREQUENCY * t);
        sample_A += HARMONIC_3 * AMPLITUDE * sin(2 * PI * 3 * FREQUENCY * t);
        sample_A += HARMONIC_5 * AMPLITUDE * sin(2 * PI * 5 * FREQUENCY * t);
        sample_A += HARMONIC_7 * AMPLITUDE * sin(2 * PI * 7 * FREQUENCY * t);
        sample_A += HARMONIC_9 * AMPLITUDE * sin(2 * PI * 9 * FREQUENCY * t);
        sample_A += HARMONIC_11 * AMPLITUDE * sin(2 * PI * 11 * FREQUENCY * t);
        sample_A += HARMONIC_13 * AMPLITUDE * sin(2 * PI * 13 * FREQUENCY * t);

        if (sample_A >= 2.5)
        {
        	sample_A= 2.5;
        }
        else if (sample_A <= -2.5)
		{
			sample_A= -2.5;
		}

        wave_A[i] = (uint16_t)((65536 / (2 * VREF)) * (sample_A + VREF));

        // Phase B with harmonic distortion (120 degrees shifted)

        double sample_B = AMPLITUDE * sin(2 * PI * FREQUENCY * t + PHASE_SHIFT_120_DEG);
        sample_B += HARMONIC_3 * AMPLITUDE * sin(2 * PI * 3 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
        sample_B += HARMONIC_5 * AMPLITUDE * sin(2 * PI * 5 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
        sample_B += HARMONIC_7 * AMPLITUDE * sin(2 * PI * 7 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
        sample_B += HARMONIC_9 * AMPLITUDE * sin(2 * PI * 9 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
        sample_B += HARMONIC_11 * AMPLITUDE * sin(2 * PI * 11 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
        sample_B += HARMONIC_13 * AMPLITUDE * sin(2 * PI * 13 * FREQUENCY * t + PHASE_SHIFT_120_DEG);

        if (sample_B >= 2.5)
        {
        	sample_B= 2.5;
        }
        else if (sample_B <= -2.5)
		{
			sample_B= -2.5;
		}

        wave_B[i] = (uint16_t)((65536 / (2 * VREF)) * (sample_B + VREF));


        // Phase C with harmonic distortion (240 degrees shifted)
        double sample_C = AMPLITUDE * sin(2 * PI * FREQUENCY * t + PHASE_SHIFT_240_DEG);
        sample_C += HARMONIC_3 * AMPLITUDE * sin(2 * PI * 3 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
        sample_C += HARMONIC_5 * AMPLITUDE * sin(2 * PI * 5 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
        sample_C += HARMONIC_7 * AMPLITUDE * sin(2 * PI * 7 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
        sample_C += HARMONIC_9 * AMPLITUDE * sin(2 * PI * 9 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
        sample_C += HARMONIC_11 * AMPLITUDE * sin(2 * PI * 11 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
        sample_C += HARMONIC_13 * AMPLITUDE * sin(2 * PI * 13 * FREQUENCY * t + PHASE_SHIFT_240_DEG);

        if (sample_C >= 2.5)
        {
        	sample_C= 2.5;
        }
        else if (sample_C <= -2.5)
		{
			sample_C= -2.5;
		}
        wave_C[i] = (uint16_t)((65536 / (2 * VREF)) * (sample_C + VREF));
    }
}

////// Function to select the test scenario and calculate its samples
void select_test_scenario(int scenario_number) {
    switch (scenario_number) {
		case 1:  // Stroom scenario 1
			AMPLITUDE = (stroom_s1 / 10000) *sqrt(2.0);
			HARMONIC_3 = 0; HARMONIC_5 = 0; HARMONIC_7 = 0; HARMONIC_9 = 0;
			HARMONIC_11 = 0; HARMONIC_13 = 0;
			calculate_three_phase_sine_wave_samples(sine_wave_A_scenario[0], sine_wave_B_scenario[0], sine_wave_C_scenario[0]);
			break;
		case 2:  // Stroom scenario 2
			AMPLITUDE = (stroom_s2 / 10000) *sqrt(2.0);
			HARMONIC_3 = 0; HARMONIC_5 = 0; HARMONIC_7 = 0; HARMONIC_9 = 0;
			HARMONIC_11 = 0; HARMONIC_13 = 0;
			calculate_three_phase_sine_wave_samples(sine_wave_A_scenario[1], sine_wave_B_scenario[1], sine_wave_C_scenario[1]);
			break;
        case 3:  // Stroom scenario 3
            AMPLITUDE = (stroom_s3 / 10000) *sqrt(2.0);
            HARMONIC_3 = 0; HARMONIC_5 = 0; HARMONIC_7 = 0; HARMONIC_9 = 0;
            HARMONIC_11 = 0; HARMONIC_13 = 0;
            calculate_three_phase_sine_wave_samples(sine_wave_A_scenario[2], sine_wave_B_scenario[2], sine_wave_C_scenario[2]);
            break;
        case 4:  // THD scenario 1
            AMPLITUDE = (thd_s1_rms / 10000) *sqrt(2.0);
            HARMONIC_3 = thd_s1_3h / 100.0;
            HARMONIC_5 = thd_s1_5h / 100.0;
            HARMONIC_7 = thd_s1_7h / 100.0;
            HARMONIC_9 = thd_s1_9h / 100.0;
            HARMONIC_11 = thd_s1_11h / 100.0;
            HARMONIC_13 = thd_s1_13h / 100.0;
            calculate_three_phase_sine_wave_samples(sine_wave_A_scenario[3], sine_wave_B_scenario[3], sine_wave_C_scenario[3]);
            break;
        case 5:  // THD scenario 2
            AMPLITUDE = (thd_s2_rms / 10000) *sqrt(2.0);
            HARMONIC_3 = thd_s2_3h / 100.0;
            HARMONIC_5 = thd_s2_5h / 100.0;
            HARMONIC_7 = thd_s2_7h / 100.0;
            HARMONIC_9 = thd_s2_9h / 100.0;
            HARMONIC_11 = thd_s2_11h / 100.0;
            HARMONIC_13 = thd_s2_13h / 100.0;
            calculate_three_phase_sine_wave_samples(sine_wave_A_scenario[4], sine_wave_B_scenario[4], sine_wave_C_scenario[4]);
            break;
        case 6:  // THD scenario 3
            AMPLITUDE = (thd_s3_rms / 10000) *sqrt(2.0);
            HARMONIC_3 = thd_s3_3h / 100.0;
            HARMONIC_5 = thd_s3_5h / 100.0;
            HARMONIC_7 = thd_s3_7h / 100.0;
            HARMONIC_9 = thd_s3_9h / 100.0;
            HARMONIC_11 = thd_s3_11h / 100.0;
            HARMONIC_13 = thd_s3_13h / 100.0;
            calculate_three_phase_sine_wave_samples(sine_wave_A_scenario[5], sine_wave_B_scenario[5], sine_wave_C_scenario[5]);
            break;
    }
}


void DAC8564_Write(uint16_t channel, uint16_t value) {
    uint8_t data[3];
    data[0] = DAC8564_SEQUENTIANALY_WRITE_UPDATE | (channel << 1);  // Set command and channel
    data[1] = (value >> 8) & 0xFF;  // MSB
    data[2] = value & 0xFF;  // LSB

    HAL_GPIO_WritePin(GPIOB, SPI_SYNC_Pin, GPIO_PIN_RESET);  // CS Low
    HAL_SPI_Transmit(&hspi2, data, 3, SPI_TIMEOUT);
    HAL_GPIO_WritePin(GPIOB, SPI_SYNC_Pin, GPIO_PIN_SET);  // CS High
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
    	sample_counter++;
    	if (sample_counter== 48000) {
    	        seconds_elapsed++;
                sample_counter=0;
    	        // Wissel scenario na 120 seconden
    	        if (seconds_elapsed >=15) {
    	            seconds_elapsed = 0;

//    	             Ga door naar het volgende scenario
    	            current_scenario++;
    	            if (current_scenario > 6) {
    	                current_scenario = 6;

    	            }else {

    					// Wissel de actieve buffers naar de nieuwe berekende golfvormen
    					current_sine_wave_A = sine_wave_A_scenario[current_scenario - 1];
    					current_sine_wave_B = sine_wave_B_scenario[current_scenario - 1];
    					current_sine_wave_C = sine_wave_C_scenario[current_scenario - 1];
    					sample_index =0;
    	            }

    	        }



    	}

        DAC8564_Write(0, current_sine_wave_A[sample_index]);  // Phase A on DAC A
        DAC8564_Write(1, current_sine_wave_B[sample_index]);  // Phase B on DAC B
        DAC8564_Write(2, current_sine_wave_C[sample_index]);  // Phase C on DAC C

        // Verhoog de sample index
        sample_index = (sample_index + 1) % NUM_SAMPLES;

        // Verhoog de sample index


    }
}

void SET_refference() {
    uint8_t data[3];
    data[0] = 0x01;
    data[1] = 0x10;
    data[2] = 0x00;

    HAL_GPIO_WritePin(GPIOB, SPI_SYNC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, data, 3, SPI_TIMEOUT);
    HAL_GPIO_WritePin(GPIOB, SPI_SYNC_Pin, GPIO_PIN_SET);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_I2S1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  stroom_s1 = 10000.0;
  stroom_s2 = 7500.0;
  stroom_s3 = 5000.0;

 // Globale variabelen voor THD scenario's
 thd_s1_rms = 4000.0, thd_s2_rms =3000.0, thd_s3_rms =2000.0;

 for (int i = 1; i <= 6; i++) {
     select_test_scenario(i);
 }

 // Zet het eerste scenario klaar

 current_sine_wave_A = sine_wave_A_scenario[0];
 current_sine_wave_B = sine_wave_B_scenario[0];
 current_sine_wave_C = sine_wave_C_scenario[0];
 current_scenario = 1;




  HAL_Delay(150);
  HAL_GPIO_WritePin(GPIOB, SPI_SYNC_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  SET_refference();

  // Calculate samples for three-phase sine waves


  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
      Error_Handler();
  }

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
  RCC_OscInitStruct.PLL.PLLM = 6;
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
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_8K;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1749;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Blauwe_LED_Pin|Groene_LED_Pin|Rode_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_SYNC_Pin|RE_tranceiver_Pin|DE_tranceiver_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, RODE_LED_Pin|GROENE_LED_Pin|BLAUWE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Start_knop_Pin Pauzeer_knop_Pin Stop_knop_Pin */
  GPIO_InitStruct.Pin = Start_knop_Pin|Pauzeer_knop_Pin|Stop_knop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Blauwe_LED_Pin Groene_LED_Pin Rode_LED_Pin */
  GPIO_InitStruct.Pin = Blauwe_LED_Pin|Groene_LED_Pin|Rode_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_SYNC_Pin RE_tranceiver_Pin DE_tranceiver_Pin */
  GPIO_InitStruct.Pin = SPI_SYNC_Pin|RE_tranceiver_Pin|DE_tranceiver_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LDAC_Pin */
  GPIO_InitStruct.Pin = LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LDAC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RODE_LED_Pin GROENE_LED_Pin BLAUWE_LED_Pin */
  GPIO_InitStruct.Pin = RODE_LED_Pin|GROENE_LED_Pin|BLAUWE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
