/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdint.h>
#include <stdlib.h>
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
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;

/* USER CODE BEGIN PV */
#define MAX_LED 133
// Global array for noise thresholds
uint16_t vu_thresholds[MAX_LED];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */
// Helper function that processes audio
void process_audio_chunk(int16_t* buffer_half, uint16_t size);
// Helper function that draws the LEDs
void update_vu_meter(int16_t peak);

void generate_vu_thresholds(uint16_t min_val, uint16_t max_val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void generate_vu_thresholds(uint16_t min_val, uint16_t max_val) {
    // Formula for geometric progression:
    // Value = Min * (Factor ^ Index)
    // Where Factor = (Max / Min)^(1 / (N - 1))

    double factor = pow((double)max_val / min_val, 1.0 / (MAX_LED - 1));
    double current_val = min_val;

    for (int i = 0; i < MAX_LED; i++) {
        vu_thresholds[i] = (uint16_t)current_val;
        current_val *= factor;
    }
}

// Buffer for I2S DMA
// Array of 256 samples filled by DMA in continuous mode
#define I2S_BUFFER_SIZE 256
int16_t i2s_dma_buffer[I2S_BUFFER_SIZE]; // 16-bit Data



// The "volume variable"
// Stores the absolute maximum value (volume peak) found
// in the last processed audio data block. This is the value the update_vu_meter function uses to plot the LEDs.
volatile int16_t audio_peak_level = 0;


uint8_t LED_Data[MAX_LED][4];


int datasentflag=0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_1);
	datasentflag=1;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}





uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;



	for (int i= 0; i<MAX_LED; i++)
	{
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 67;  // 2/3 of 100
			}

			else pwmData[indx] = 33;  // 1/3 of 100

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}

void Reset_LED (void)
{
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Data[i][0] = i;
		LED_Data[i][1] = 0;
		LED_Data[i][2] = 0;
		LED_Data[i][3] = 0;
	}
}



/**
 * @brief Draws the VU meter on the LED strip
 * @param peak The read peak level (from 0 to 32767)
 */
void update_vu_meter(int16_t peak)
{
    uint8_t leds_to_light = 0;

    // 1. Find how many LEDs to light up by comparing the peak with the calculated array
    for (int i = 0; i < MAX_LED; i++) {
        if (peak > vu_thresholds[i]) {
            leds_to_light = i + 1;
        } else {
            // Optimization: if the peak is lower than this threshold,
            // it will also be lower than the subsequent ones (which are increasing).
            break;
        }
    }

    // 2. Draw colors based on position (Percentages)
    // Green: first 40% | Yellow: from 40% to 70% | Red: last 30%
    int green_zone = MAX_LED * 0.40;
    int yellow_zone = MAX_LED * 0.70;

    for (int i = 0; i < MAX_LED; i++) {
        if (i < leds_to_light) {
            if (i < green_zone) {
                Set_LED(i, 0, 150, 0);   // Green (Medium brightness)
            } else if (i < yellow_zone) {
                Set_LED(i, 200, 60, 0); // Yellow/Orange
            } else {
                Set_LED(i, 255, 0, 0);   // Red (Peak)
            }
        } else {
            // Turn off LEDs above the volume level
            Set_LED(i, 0, 0, 0);
        }
    }

    WS2812_Send();
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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */
  // --- START LISTENING ---
  // Starts DMA in circular mode.
  // From this moment, 'i2s_dma_buffer' will fill automatically.
  if (HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*)i2s_dma_buffer, I2S_BUFFER_SIZE) != HAL_OK) {
      Error_Handler(); // Error
  }

  // Variable for non-blocking LED update
  uint32_t last_vu_update = 0;
  Reset_LED();
  WS2812_Send();
  HAL_Delay(50);
  // CALCULATE LOGARITHMIC THRESHOLDS
  // This fills the vu_thresholds array with 133 perfect values.
  generate_vu_thresholds(70, 700);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	      uint32_t current_time = HAL_GetTick();
	      if (current_time - last_vu_update > 20)
	      {
	          last_vu_update = current_time;

	          // Read the peak (calculated by DMA) and draw the LEDs
	          update_vu_meter(audio_peak_level);

	          // Reset the peak for the next "frame"
	          // This gives a reactive "peak meter" effect
	          audio_peak_level = 0;

	      }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief This function is called by DMA
 * when the FIRST HALF of the buffer is full.
 */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  // Process the first half of the buffer (from index 0)
  process_audio_chunk(&i2s_dma_buffer[0], I2S_BUFFER_SIZE / 2);
}

/**
 * @brief This function is called by DMA
 * when the buffer is COMPLETELY full.
 */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  // Process the second half of the buffer
  process_audio_chunk(&i2s_dma_buffer[I2S_BUFFER_SIZE / 2], I2S_BUFFER_SIZE / 2);
}

/**
 * @brief Analyzes an audio block and finds the volume peak.
 */
void process_audio_chunk(int16_t* buffer_half, uint16_t size)
{
    int16_t max_val = 0;

    for (uint16_t i = 0; i < size; i += 2) // NOTE: i += 2
    {
        // 1. Get the audio sample
        // I2S data is signed (ranges from -32768 to +32767)
        int16_t sample = buffer_half[i];

        // 2. Rectify it (we only care about volume, not phase)
        int16_t abs_sample = abs(sample);

        // 3. Is it the highest we've seen in this block?
        if (abs_sample > max_val) {
            max_val = abs_sample;
        }

        // NOTE: Why "i += 2"?
        // The INMP441 is MONO but I2S is a STEREO standard.
        // The microphone writes only to one channel (Left OR Right).
        // The other channel will be all zeros.
        // By reading only EVEN (or only ODD) samples,
        // we take only the correct channel and avoid zeros.
    }

    // 4. Update the global peak variable
    // If the peak of this block is higher than the one
    // we have stored, update it.
    if (max_val > audio_peak_level) {
        audio_peak_level = max_val;
    }
}
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
