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
#include "nRF24L01.h"
#include "RF24_STM32.h"
#include "ina219.h"
#include "string.h"
#include "stdio.h"

RF24_HandleTypeDef hRF24;
INA219_t ina219;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const uint8_t rx_address[5] = { 0x30, 0x30, 0x30, 0x30, 0x31 };
const uint8_t tx_address[5] = { 0x30, 0x30, 0x30, 0x30, 0x32 };
char rxData[32];
uint32_t last_rx = 0;
const uint32_t timeout = 500;

uint32_t last_ina_read = 0;
const uint32_t ina_interval = 1000;

volatile uint32_t pwm = 700;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void stop(void);
void tien(void);
void trai(void);
void phai(void);
void lui(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t rx_address[5] = { 0x12, 0x34, 0x56, 0x78, 0x90 };
/* ----------------------------------- */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */

	__HAL_TIM_MOE_ENABLE(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	INA219_Init(&ina219, &hi2c1, INA219_ADDRESS);
	if (INA219_Begin(&ina219)) {
		printf("Ket noi thanh cong INA");
	} else {
		//Error_Handler();
	}
	printf("System Initializing...\r\n");

	RF24_Init(&hRF24, &hspi1, NRF_CE_GPIO_Port, NRF_CE_Pin, NRF_CSN_GPIO_Port,
	NRF_CSN_Pin);
	if (!RF24_begin(&hRF24)) {
		while (1) {
			printf("Error: NRF24 not connected!\r\n");
			HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
			HAL_Delay(100);
		}
	}
	printf("NRF24 Connected! Mode: RX\r\n");

	RF24_setPALevel(&hRF24, RF24_PA_LOW);

	RF24_setDataRate(&hRF24, RF24_250KBPS);

	RF24_setChannel(&hRF24, 76);
	RF24_setPayloadSize(&hRF24, 32);

	RF24_openWritingPipe(&hRF24, tx_address);
	RF24_openReadingPipe(&hRF24, 1, rx_address);
	RF24_startListening(&hRF24);

	HAL_Delay(100);
	printf("Flushing old data...\r\n");
	while (RF24_available(&hRF24)) {
		char trash[32];
		RF24_read(&hRF24, trash, 32);
		printf("Discarded: %s\r\n", trash);
	}
	last_rx = HAL_GetTick();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (RF24_available(&hRF24)) {
			memset(rxData, 0, sizeof(rxData));

			RF24_read(&hRF24, rxData, 32);

			last_rx = HAL_GetTick();

			HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);

			printf("Cmd: %s\r\n", rxData);

			if (strcmp(rxData, "tien") == 0) {
				tien();
				HAL_Delay(100);
			} else if (strcmp(rxData, "lui") == 0) {
				lui();
				HAL_Delay(100);
			} else if (strcmp(rxData, "trai") == 0) {
				trai();
				HAL_Delay(100);
			} else if (strcmp(rxData, "phai") == 0) {
				phai();
				HAL_Delay(100);
			} else if (strcmp(rxData, "tang") == 0) {
				pwm += 100;
			} else if (strcmp(rxData, "giam") == 0) {
				pwm -= 100;
			} else {
				stop();
				if ((HAL_GetTick() - last_ina_read) >= ina_interval) {
					last_ina_read = HAL_GetTick();

					float busVolt = INA219_getBusVoltage_V(&ina219);
					if (ina219.hi2c->ErrorCode != HAL_I2C_ERROR_NONE) {
						printf("I2C Err: %lu\r\n", ina219.hi2c->ErrorCode);
						HAL_I2C_DeInit(&hi2c1);
						HAL_Delay(5);
						HAL_I2C_Init(&hi2c1);
						INA219_setCalibration_32V_2A(&ina219);
					} else {
						printf("V_Pin: %.2f V\r\n", busVolt);
					}

					RF24_stopListening(&hRF24);
					bool report = RF24_write(&hRF24, &busVolt, sizeof(float));
					RF24_startListening(&hRF24);

					if (report)
						printf("Sent: %f\r\n", busVolt);
				}
			}
		}

		if ((HAL_GetTick() - last_rx) > timeout) {
			stop();

			HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
			HAL_Delay(1000);
			printf("Connection Lost!\r\n");
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 7;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | NRF_CE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	ENA_Pin | IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin | ENB_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_ERR_Pin */
	GPIO_InitStruct.Pin = LED_ERR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_ERR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : NRF_CSN_Pin NRF_CE_Pin */
	GPIO_InitStruct.Pin = NRF_CSN_Pin | NRF_CE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ENA_Pin IN1_Pin IN2_Pin IN3_Pin
	 IN4_Pin ENB_Pin */
	GPIO_InitStruct.Pin = ENA_Pin | IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin
			| ENB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void pwm_set(void) {
	if (pwm > 999)
		pwm = 999;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
}

void stop(void) {

	HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ENB_Pin, GPIO_PIN_RESET);
	pwm_set();

	HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET);
}

void tien(void) {

	HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ENB_Pin, GPIO_PIN_SET);

	pwm_set();
	// IN1=1, IN2=0 (Trái tiến)
	HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);
	// IN3=1, IN4=0 (Phải tiến)
	HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET);
}

void lui(void) {
	HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ENB_Pin, GPIO_PIN_SET);

	pwm_set();
	// IN1=0, IN2=1 (Trái lùi)
	HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_SET);

	// IN3=0, IN4=1 (Phải lùi)
	HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_SET);
}

void trai(void) {
	HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ENB_Pin, GPIO_PIN_SET);

	pwm_set();
	// IN1=1, IN2=0 (Trái tiến)
	HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);

	// IN3=0, IN4=1 (Phải lùi)
	HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_SET);
}

void phai(void) {
	HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ENB_Pin, GPIO_PIN_SET);

	pwm_set();
	// IN1=0, IN2=1 (Trái lùi)
	HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_SET);

	// IN3=1, IN4=0 (Phải tiến)
	HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET);
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
#ifdef USE_FULL_ASSERT
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
