/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* ADC sampling frequency 1458.33 Hz */
/* USER CODE END header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#define FFT_BUFFER_LENGTH (1024)
#define FFT_LENGTH (64)
#define FFT_SLIDING (8)
#define FLOAT_PRECISION (6)
#define CHIRP_LENGTH (64)
#define CORRELATION_THRESHOLD (0.9)
#define FREQ_ONE_BIN (11)
#define FREQ_ZERO_BIN (10)
#define FSK_MESSAGE_BYTE_SIZE (4) //receiving 4 bytes
#define FSK_MESSAGE_BIT_SIZE (FSK_MESSAGE_BYTE_SIZE*8)
//#elements * ( decimal digits + 3 integer digits + 1 dot + 1 sign + 1 space ) + \r\n
#define UART_BUFFER_LENGTH (FFT_LENGTH * (FLOAT_PRECISION + 6)  + 2)
#include "math.h"
#include "arm_math.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float32_t valueBuffer[FFT_BUFFER_LENGTH] =
		{ [0 ... FFT_BUFFER_LENGTH - 1] = 0.0 };
float32_t tempValueBuffer[FFT_LENGTH];
volatile uint16_t valueBufferIndex = 0;
float32_t fftBuffer[FFT_LENGTH] = { [0 ... FFT_LENGTH - 1] = 0.0 };
float32_t magnitudes[FFT_LENGTH / 2];
//TODO: should be changed to a more effective chirp signal (search on a signal processing book?)
const float32_t chirpBuffer[CHIRP_LENGTH] = { 0.1142, 0.0818, 0.0345, -0.0191,
		-0.0692, -0.1067, -0.1247, -0.1199, -0.0931, -0.0494, 0.0034, 0.0555,
		0.0975, 0.1217, 0.1236, 0.1030, 0.0635, 0.0124, -0.0410, -0.0868,
		-0.1168, -0.1255, -0.1112, -0.0766, -0.0280, 0.0258, 0.0748, 0.1101,
		0.1253, 0.1176, 0.0885, 0.0431, -0.0101, -0.0615, -0.1017, -0.1232,
		-0.1222, -0.0989, -0.0575, -0.0056, 0.0473, 0.0916, 0.1192, 0.1249,
		0.1079, 0.0711, 0.0213, -0.0323, -0.0801, -0.1132, -0.1256, -0.1151,
		-0.0835, -0.0367, 0.0169, 0.0673, 0.1055, 0.1244, 0.1205, 0.0946,
		0.0514, -0.0011, -0.0535, -0.0961 };
float32_t receivedChirpBuffer[CHIRP_LENGTH] = { [0 ... CHIRP_LENGTH - 1] = 0.0 };
float32_t receivedTempChirpBuffer[CHIRP_LENGTH] = { [0 ... CHIRP_LENGTH - 1
		] = 0.0 };
float32_t chirpFftBuffer[CHIRP_LENGTH];
float32_t chirpFftMagnitude[CHIRP_LENGTH / 2];
float32_t chirpOffsetValue[CHIRP_LENGTH];
float32_t chirpCorrelation[2 * CHIRP_LENGTH - 1];
volatile float32_t receivedChirpNorm;
volatile float32_t receivedChirpOffset;
arm_rfft_fast_instance_f32 fftInstance, chirpFftInstance;
volatile uint8_t computeFFT = 0;
volatile uint8_t receivingData = 0;
volatile uint16_t receivedBitSize = 0;
volatile uint16_t computeFFTLastIndex = 0;
volatile uint16_t savedComputeFFTLastIndex;
char uartBuffer[UART_BUFFER_LENGTH] = { 0 };
uint8_t receivedData[FSK_MESSAGE_BYTE_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//convert ADC values and copy into tempValueBuffer
static void convertLastADCValueBatch(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
//	size_t uartStringLen = 0;
	float32_t maxValue;
	int maxValueIndex;

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
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_IT(&hadc1);
//  HAL_ADC_Start_DMA(hadc, pData, Length)
	while (ARM_MATH_SUCCESS != arm_rfft_fast_init_f32(&fftInstance, FFT_LENGTH))
		;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (computeFFT != 0) {
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);

			convertLastADCValueBatch();
			//TODO: note that sensor value conversion is useless, since conversion doesn't affect FFT
			//if we need to compute FFT with values at both ends of array...
			arm_rfft_fast_f32(&fftInstance, tempValueBuffer, fftBuffer, 0);
			arm_cmplx_mag_f32(fftBuffer, magnitudes, FFT_LENGTH / 2);
			maxValue = magnitudes[0];
			maxValueIndex = 0;

			for (int i = 1; i < FFT_LENGTH / 2 - 1; i++) {
				if (magnitudes[i] > maxValue) {
					maxValue = magnitudes[i];
					maxValueIndex = i;
				}
			}

			if(maxValueIndex == FREQ_ONE_BIN)
				receivedData[(receivedBitSize-1)/8] |= 1 << (7 - ((receivedBitSize-1) % 8));
			else
				receivedData[(receivedBitSize-1)/8] &= ~(1 << (7 - ((receivedBitSize-1) % 8)));
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
			computeFFT = 0;
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PC10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//if receiving samples for data
	if (receivingData) {
		valueBuffer[valueBufferIndex] = HAL_ADC_GetValue(&hadc1);
		valueBufferIndex = (valueBufferIndex + 1) % FFT_BUFFER_LENGTH;
		//TODO: this check shouldn't be required, since fft computation should be much faster than ADC ...
		if (computeFFT == 0) {
			if (valueBufferIndex % FFT_LENGTH == 0) {
				computeFFTLastIndex = valueBufferIndex;
				computeFFT = 1;
				receivedBitSize++;
			}
		}
		if (receivedBitSize == FSK_MESSAGE_BIT_SIZE)
			receivingData = 0;

	} else {
		//trying to receive chirp
		receivedChirpOffset = 0.0;
		//inserting new ADC value in last (chirp length) buffer position
		//after sliding previous values
		for (int i = 0; i < CHIRP_LENGTH - 1; i++) {
			receivedChirpBuffer[i] = receivedChirpBuffer[i + 1];
			receivedChirpOffset += receivedChirpBuffer[i];
		}
		receivedChirpBuffer[CHIRP_LENGTH - 1] = HAL_ADC_GetValue(&hadc1);
		receivedChirpOffset += receivedChirpBuffer[CHIRP_LENGTH - 1];
		receivedChirpOffset = receivedChirpOffset / CHIRP_LENGTH;
		for (int i = 0; i < CHIRP_LENGTH; i++) {
			chirpOffsetValue[i] = receivedChirpOffset;
		}
		memcpy(receivedTempChirpBuffer, receivedChirpBuffer, CHIRP_LENGTH);
		//remove offset
		arm_sub_f32(receivedTempChirpBuffer, chirpOffsetValue,
				receivedTempChirpBuffer, CHIRP_LENGTH);
		//normalize and correlate
		receivedChirpNorm = 0.0;
		for (int i = 0; i < CHIRP_LENGTH; i++) {
			receivedChirpNorm = receivedChirpNorm
					+ receivedTempChirpBuffer[i] * receivedTempChirpBuffer[i];
		}
		receivedChirpNorm = sqrt(receivedChirpNorm);
		for (int i = 0; i < CHIRP_LENGTH; i++) {
			receivedTempChirpBuffer[i] = receivedTempChirpBuffer[i]
					/ receivedChirpNorm;
		}
		//detect chirp by correlation
		arm_correlate_f32(receivedTempChirpBuffer, CHIRP_LENGTH, chirpBuffer,
		CHIRP_LENGTH, chirpCorrelation);
		//TODO: evaluate correlation threshold!
		if (chirpCorrelation[CHIRP_LENGTH - 1] > CORRELATION_THRESHOLD) {
			receivedBitSize = 0;
			receivingData = 1;
		}

	}
	/*If continuousconversion mode is DISABLED uncomment below*/
	HAL_ADC_Start_IT(&hadc1);
}

static void convertLastADCValueBatch() {
	if (computeFFTLastIndex < FFT_LENGTH) {
		for (int i = computeFFTLastIndex - FFT_LENGTH + FFT_BUFFER_LENGTH,
				j = 0; i < FFT_BUFFER_LENGTH; i++, j++) {
			//ADC value mapping
			valueBuffer[i] *= (3300.0 / 4500.0);
			//sensor conversion
			valueBuffer[i] = valueBuffer[i] - 1680;
			valueBuffer[i] = valueBuffer[i] / (4 * 12.2 * 100);
			tempValueBuffer[j] = valueBuffer[i];
		}

		for (int i = 0, j = FFT_LENGTH - computeFFTLastIndex;
				i < computeFFTLastIndex; i++, j++) {
			valueBuffer[i] *= (3300.0 / 4500.0);
			//sensor conversion
			valueBuffer[i] = valueBuffer[i] - 1680;
			valueBuffer[i] = valueBuffer[i] / (4 * 12.2 * 100);
			tempValueBuffer[j] = valueBuffer[i];

		}
	} else {
		for (int i = 0; i < FFT_LENGTH; i++) {
			valueBuffer[i + computeFFTLastIndex - FFT_LENGTH] *= (3300.0
					/ 4500.0);
			//sensor conversion
			valueBuffer[i + computeFFTLastIndex - FFT_LENGTH] = valueBuffer[i
					+ computeFFTLastIndex - FFT_LENGTH] - 1680;
			valueBuffer[i + computeFFTLastIndex - FFT_LENGTH] = valueBuffer[i
					+ computeFFTLastIndex - FFT_LENGTH] / (4 * 12.2 * 100);

			tempValueBuffer[i] = valueBuffer[i + computeFFTLastIndex
					- FFT_LENGTH];
		}
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
