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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	STATUS_IDLE,
    STATUS_WACHTEN_OP_INSTELLINGEN,
    STATUS_GEREED,
    STATUS_TEST_GESTART,
    STATUS_TEST_GEPAUZEERD,
	STATUS_TEST_GESTOPT,
    STATUS_TEST_VOLTOOID
} TestStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SPI_TIMEOUT 1000
#define DAC8564_SEQUENTIANALY_WRITE_UPDATE 0x10  // Command to write and update DAC
#define VREF 2.5  // Reference voltage

#define OFFSET (VREF / 2)  // Midpoint offset for unipolar output
#define PI 3.14159265
#define SAMPLE_RATE 48000  // Sample rate in Hz
#define FREQUENCY 50  // Frequency of the sine wave in Hz
#define NUM_SAMPLES (SAMPLE_RATE / FREQUENCY)  // Number of samples in one cycle
#define DAC_REFERENCE_ALWAYS_POWERED_UP    0x11000

#define PHASE_SHIFT_120_DEG (2.0 * PI / 3.0)  // 120-degree phase shift
#define PHASE_SHIFT_240_DEG (4.0 * PI / 3.0)

#define BLAUWE_LED_PIN GPIO_PIN_1
#define BLAUWE_LED_PORT GPIOF

#define GROENE_LED_PIN GPIO_PIN_2
#define GROENE_LED_PORT GPIOF

#define RODE_LED_PIN GPIO_PIN_3
#define RODE_LED_PORT GPIOF

#define START_KNOP_PIN GPIO_PIN_13
#define START_KNOP_PORT GPIOC

#define PAUZEER_KNOP_PIN GPIO_PIN_14
#define PAUZEER_KNOP_PORT GPIOC

#define STOP_KNOP_PIN GPIO_PIN_15
#define STOP_KNOP_PORT GPIOC

#define GPIO_PORT_VBUS GPIOA   // De GPIO-poort voor VBUS (bijv. GPIOA)
#define GPIO_PIN_VBUS  GPIO_PIN_9   // De specifieke pin (bijv. pin 9 op GPIOA)


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

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KnopTASK */
osThreadId_t KnopTASKHandle;
const osThreadAttr_t KnopTASK_attributes = {
  .name = "KnopTASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedTASK */
osThreadId_t LedTASKHandle;
const osThreadAttr_t LedTASK_attributes = {
  .name = "LedTASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Dac_TASK */
osThreadId_t Dac_TASKHandle;
const osThreadAttr_t Dac_TASK_attributes = {
  .name = "Dac_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
char receivedData[BUFFER_SIZE];  // Buffer to store received data
uint32_t receivedLength = 0;  // Houd de lengte van de ontvangen data bij
float stroom_s1, stroom_s2, stroom_s3;

float AMPLITUDE;

float HARMONIC_3, HARMONIC_5, HARMONIC_7, HARMONIC_9, HARMONIC_11, HARMONIC_13;


uint16_t sine_wave_A[NUM_SAMPLES];
uint16_t sine_wave_B[NUM_SAMPLES];
uint16_t sine_wave_C[NUM_SAMPLES];
volatile uint16_t sample_index = 0;

float stroom_s1 = 0.0;
float stroom_s2 = 0.0;
float stroom_s3 = 0.0;

// Globale variabelen voor THD scenario's
float thd_s1_rms = 0.0, thd_s2_rms = 0.0, thd_s3_rms = 0.0;
int thd_s1_3h, thd_s1_5h, thd_s1_7h, thd_s1_9h, thd_s1_11h, thd_s1_13h;
int thd_s2_3h, thd_s2_5h, thd_s2_7h, thd_s2_9h, thd_s2_11h, thd_s2_13h;
int thd_s3_3h, thd_s3_5h, thd_s3_7h, thd_s3_9h, thd_s3_11h, thd_s3_13h;

TestStatus huidig_status = STATUS_IDLE;
volatile bool usb_busy = false;  // Vlag om aan te geven of USB-communicatie bezig i


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2S1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void StartKnopThread(void *argument);
void StartLEDThread(void *argument);
void Simulate_DAC(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*  Functies DAC */
/* DAC CODE BEGIN*/
void calculate_three_phase_sine_wave_samples() {
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
    	        sine_wave_A[i] = (uint16_t)((65536 / (2 * VREF)) * (sample_A + OFFSET));

    	        // Phase B with harmonic distortion (120 degrees shifted)
    	        double sample_B = AMPLITUDE * sin(2 * PI * FREQUENCY * t + PHASE_SHIFT_120_DEG);
    	        sample_B += HARMONIC_3 * AMPLITUDE * sin(2 * PI * 3 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
    	        sample_B += HARMONIC_5 * AMPLITUDE * sin(2 * PI * 5 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
    	        sample_B += HARMONIC_7 * AMPLITUDE * sin(2 * PI * 7 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
    	        sample_B += HARMONIC_9 * AMPLITUDE * sin(2 * PI * 9 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
    	        sample_B += HARMONIC_11 * AMPLITUDE * sin(2 * PI * 11 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
    	        sample_B += HARMONIC_13 * AMPLITUDE * sin(2 * PI * 13 * FREQUENCY * t + PHASE_SHIFT_120_DEG);
    	        sine_wave_B[i] = (uint16_t)((65536 / (2 * VREF)) * (sample_B + OFFSET));

    	        // Phase C with harmonic distortion (240 degrees shifted)
    	        double sample_C = AMPLITUDE * sin(2 * PI * FREQUENCY * t + PHASE_SHIFT_240_DEG);
    	        sample_C += HARMONIC_3 * AMPLITUDE * sin(2 * PI * 3 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
    	        sample_C += HARMONIC_5 * AMPLITUDE * sin(2 * PI * 5 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
    	        sample_C += HARMONIC_7 * AMPLITUDE * sin(2 * PI * 7 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
    	        sample_C += HARMONIC_9 * AMPLITUDE * sin(2 * PI * 9 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
    	        sample_C += HARMONIC_11 * AMPLITUDE * sin(2 * PI * 11 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
    	        sample_C += HARMONIC_13 * AMPLITUDE * sin(2 * PI * 13 * FREQUENCY * t + PHASE_SHIFT_240_DEG);
    	        sine_wave_C[i] = (uint16_t)((65536 / (2 * VREF)) * (sample_C + OFFSET));
    }
}

void select_test_scenario(int scenario_number) {
    switch (scenario_number) {
        case 1:  // Stroom scenario 1
            AMPLITUDE = stroom_s1 / 10000;  // Stroom gedeeld door 10.000
            HARMONIC_3 = 0; HARMONIC_5 = 0; HARMONIC_7 = 0; HARMONIC_9 = 0;
            HARMONIC_11 = 0; HARMONIC_13 = 0;
            break;
        case 2:  // Stroom scenario 2
            AMPLITUDE = stroom_s2 / 10000;
            HARMONIC_3 = 0; HARMONIC_5 = 0; HARMONIC_7 = 0; HARMONIC_9 = 0;
            HARMONIC_11 = 0; HARMONIC_13 = 0;
            break;
        case 3:  // Stroom scenario 3
            AMPLITUDE = stroom_s3 / 10000;
            HARMONIC_3 = 0; HARMONIC_5 = 0; HARMONIC_7 = 0; HARMONIC_9 = 0;
            HARMONIC_11 = 0; HARMONIC_13 = 0;
            break;
        case 4:  // THD scenario 1
            AMPLITUDE = thd_s1_rms / 10000;  // Grondharmonische amplitude
            HARMONIC_3 = thd_s1_3h / 100.0;  // Percentages omzetten naar amplitudes
            HARMONIC_5 = thd_s1_5h / 100.0;
            HARMONIC_7 = thd_s1_7h / 100.0;
            HARMONIC_9 = thd_s1_9h / 100.0;
            HARMONIC_11 = thd_s1_11h / 100.0;
            HARMONIC_13 = thd_s1_13h / 100.0;
            break;
        case 5:  // THD scenario 2
            AMPLITUDE = thd_s2_rms / 10000;
            HARMONIC_3 = thd_s2_3h / 100.0;
            HARMONIC_5 = thd_s2_5h / 100.0;
            HARMONIC_7 = thd_s2_7h / 100.0;
            HARMONIC_9 = thd_s2_9h / 100.0;
            HARMONIC_11 = thd_s2_11h / 100.0;
            HARMONIC_13 = thd_s2_13h / 100.0;
            break;
        case 6:  // THD scenario 3
            AMPLITUDE = thd_s3_rms / 10000;
            HARMONIC_3 = thd_s3_3h / 100.0;
            HARMONIC_5 = thd_s3_5h / 100.0;
            HARMONIC_7 = thd_s3_7h / 100.0;
            HARMONIC_9 = thd_s3_9h / 100.0;
            HARMONIC_11 = thd_s3_11h / 100.0;
            HARMONIC_13 = thd_s3_13h / 100.0;
            break;
    }
    calculate_three_phase_sine_wave_samples();
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
        // Output the current sample for each phase to respective DAC channels
        DAC8564_Write(0, sine_wave_A[sample_index]);  // Phase A on DAC A
        DAC8564_Write(1, sine_wave_B[sample_index]);  // Phase B on DAC B
        DAC8564_Write(2, sine_wave_C[sample_index]);  // Phase C on DAC C

        // Increment the sample index
        sample_index = (sample_index + 1) % NUM_SAMPLES;
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

/* DAC CODE END*/

/*  Functies voor user interface */
/* Userinterface CODE BEGIN*/
void zet_blauw_licht_aan() {
    HAL_GPIO_WritePin(BLAUWE_LED_PORT, BLAUWE_LED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GROENE_LED_PORT, GROENE_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RODE_LED_PORT, RODE_LED_PIN, GPIO_PIN_RESET);
}

void zet_groen_licht_aan() {
    HAL_GPIO_WritePin(BLAUWE_LED_PORT, BLAUWE_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GROENE_LED_PORT, GROENE_LED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RODE_LED_PORT, RODE_LED_PIN, GPIO_PIN_RESET);
}

void zet_rood_licht_aan() {
    HAL_GPIO_WritePin(BLAUWE_LED_PORT, BLAUWE_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GROENE_LED_PORT, GROENE_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RODE_LED_PORT, RODE_LED_PIN, GPIO_PIN_SET);
}

void groen_knipperend() {
	HAL_GPIO_WritePin(BLAUWE_LED_PORT, BLAUWE_LED_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RODE_LED_PORT, RODE_LED_PIN, GPIO_PIN_RESET);
	HAL_GPIO_TogglePin(GROENE_LED_PORT, GROENE_LED_PIN);


}

void blauw_knipperend() {
    HAL_GPIO_WritePin(GROENE_LED_PORT, GROENE_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RODE_LED_PORT, RODE_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_TogglePin(BLAUWE_LED_PORT, BLAUWE_LED_PIN);
}

void update_status(TestStatus nieuwe_status) {
    huidig_status = nieuwe_status;
}

/* Userinterface CODE END*/

/* USB_GUI_ CODE BEGIN*/
void send_active_scenario_to_gui(int scenario_number) {
    char scenario_message[64];
    sprintf(scenario_message, "ACTIEF_SCENARIO=%d\n", scenario_number);
    send_status_to_gui(scenario_message);  // Verstuur het scenario


}


void send_status_to_gui(char* status_message) {
    CDC_Transmit_FS((uint8_t *)status_message, strlen(status_message));
}

// Functie om te checken of de knoppen voor starten, pauzeren of stoppen zijn ingedrukt


// Simuleer de ontvangst van instellingen en stuur statusupdates naar de GUI
void receive_settings_and_update_status(void) {
    // Wachten op instellingen
	update_status (STATUS_WACHTEN_OP_INSTELLINGEN);


}

// Functie om de volledige testprocedure te beheren

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

    update_status(STATUS_GEREED);


}
/* USB_GUI_ CODE END*/
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
  MX_TIM2_Init();
  MX_I2S1_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  while (!(HAL_GPIO_ReadPin(GPIO_PORT_VBUS, GPIO_PIN_VBUS) == GPIO_PIN_SET));
  receive_settings_and_update_status();
  HAL_Delay (3000);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of KnopTASK */
  KnopTASKHandle = osThreadNew(StartKnopThread, NULL, &KnopTASK_attributes);

  /* creation of LedTASK */
  LedTASKHandle = osThreadNew(StartLEDThread, NULL, &LedTASK_attributes);

  /* creation of Dac_TASK */
  Dac_TASKHandle = osThreadNew(Simulate_DAC, NULL, &Dac_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
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
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Blauwe_LED_Pin|Groene_LED_Pin|Rode_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_SYNC_Pin|RE_tranceiver_Pin|DE_tranceiver_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_RESET);

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartKnopThread */
/**
* @brief Function implementing the KnopTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKnopThread */
void StartKnopThread(void *argument)
{
  /* USER CODE BEGIN StartKnopThread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartKnopThread */
}

/* USER CODE BEGIN Header_StartLEDThread */
/**
* @brief Function implementing the LedTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDThread */
void StartLEDThread(void *argument)
{
  /* USER CODE BEGIN StartLEDThread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLEDThread */
}

/* USER CODE BEGIN Header_Simulate_DAC */
/**
* @brief Function implementing the Dac_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Simulate_DAC */
void Simulate_DAC(void *argument)
{
  /* USER CODE BEGIN Simulate_DAC */
	int scenario=1;
  /* Infinite loop */

     for(;;)
	  {




		    while ((huidig_status == STATUS_TEST_GESTART && usb_busy == false)  || huidig_status == STATUS_TEST_GEPAUZEERD) {
				   // Deze Delay is noodzakkelijk anders voert hij de send_status_to_gui(TEST_RUNNING);niet uit in de status thread.
		    	   osDelay(10);


				  // Stuur het actieve scenario naar de GUI
					usb_busy = true;
					send_active_scenario_to_gui(scenario);
					usb_busy = false;
					// Zet de vlag weer op false zodat andere threads de USB kunnen gebruiken


				  // Simuleer een vertraging tijdens het draaien van dit scenario
				  for (int i = 0; i < 5000; i++) { // Delay van 5 seconden
				   // Controleer tijdens de delay of de test gepauzeerd is
						if (huidig_status == STATUS_TEST_GEPAUZEERD) {
						  // Als de test gepauzeerd is, blijf in deze lus totdat de status verandert
							  while (huidig_status == STATUS_TEST_GEPAUZEERD) {
								osDelay(1); // Wacht kleine intervallen en check status continu
							  }
						}
						osDelay(1);  // Dit vervangt de HAL_Delay met osDelay
				  }

				  // Verhoog het scenario
				  scenario++;
				  if (scenario > 6) {
					// Als alle scenario's zijn afgerond, stel de status in op voltooid
					huidig_status = STATUS_TEST_VOLTOOID;
					scenario = 1;
					break;  // Breek de while-loop
				  }

			    // Eindig met het versturen van de voltooid-status


		    }


		// Kleine vertraging om CPU te besparen
		osDelay(1);
	  }
  /* USER CODE END Simulate_DAC */
}

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
