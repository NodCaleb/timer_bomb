/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "digits.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define spi_latch_pin	LL_GPIO_PIN_4
#define input_port		GPIOB
#define play_pause_pin	GPIO_PIN_3
#define stop_pin		GPIO_PIN_4
#define rt_clk_pin		GPIO_PIN_5
#define rt_dt_pin		LL_GPIO_PIN_6
#define debounse_time	100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint16_t millis = 0;
uint16_t cool_down_millis = 0;
uint8_t led_state = 0;
uint8_t spi_data[2]; //0 - index, 1 - value
uint8_t *p_data = spi_data;
uint16_t initial_seconds = 30;
uint16_t current_seconds = 0;
uint8_t armed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void UART_Init(void);
void One_Second_Tick(void);
uint8_t Seconds_Step(uint16_t value);
void Set_Digit_Value(uint8_t digit);
void Set_Digit_Index(uint8_t index);
void Transmit_SPI(void);
void Display_Digits(uint8_t digit_0, uint8_t digit_1, uint8_t digit_2, uint8_t digit_3);
void Display_Time(uint16_t seconds);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  UART_Init();
  HAL_TIM_Base_Start_IT(&htim1); // запуск таймера

  USART1->DR = 0x53; //Start MCU debug signal

  current_seconds = initial_seconds;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  Display_Digits(1,2,3,4);
	  Display_Time(current_seconds);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; //Lower value will be too fast for 74HC595
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  //SPI latch config
	LL_GPIO_SetPinMode(GPIOA, spi_latch_pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOA, spi_latch_pin, LL_GPIO_OUTPUT_PUSHPULL);

	/*Configure GPIO pins : PB5 PB6 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	LL_GPIO_SetPinMode(input_port, rt_dt_pin, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(input_port, rt_dt_pin, LL_GPIO_PULL_DOWN);
}

/* USER CODE BEGIN 4 */
static void UART_Init(void){
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // включаем тактирование UART1
	// настройка вывода PA9 (TX1) на режим альтернативной функции с активным выходом
	// Биты CNF = 10, ,биты MODE = X1
	GPIOA->CRH &= (~GPIO_CRH_CNF9_0);
	GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);

	// настройка вывода PA10 (RX1) на режим входа с подтягивающим резистором
	// Биты CNF = 10, ,биты MODE = 00, ODR = 1
	GPIOA->CRH &= (~GPIO_CRH_CNF10_0);
	GPIOA->CRH |= GPIO_CRH_CNF10_1;
	GPIOA->CRH &= (~(GPIO_CRH_MODE10));
	GPIOA->BSRR |= GPIO_ODR_ODR10;

	// конфигурация UART1
	USART1->CR1 = USART_CR1_UE; // разрешаем USART1, сбрасываем остальные биты
	USART1->BRR = 7500; // скорость 9600 бод
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // разрешаем приемник, передатчик и прерывание по приему
	USART1->CR2 = 0;
	USART1->CR3 = 0;

	// разрешения прерывания UART1 в контроллере прерываний
	NVIC_EnableIRQ (USART1_IRQn);
}

//External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (cool_down_millis == 0){
		cool_down_millis = debounse_time;

		if(GPIO_Pin == GPIO_PIN_5) // Play/pause button
		{
			USART1->DR = 0x30; //Debug signal
			if (armed == 0 && current_seconds > 0){
				armed = 1;
				millis = 0; //To make first second to pass after 1 second after button push
			}
			else if (armed == 1){
				armed = 0;
			}
		}

		if(GPIO_Pin == stop_pin) // Play/pause button
		{
			USART1->DR = 0x31; //Debug signal
			current_seconds = initial_seconds;
			armed = 0;
		}

		if(GPIO_Pin == rt_clk_pin) // Play/pause button
		{
			if( LL_GPIO_IsInputPinSet(input_port, rt_dt_pin) != 0 ) {
				USART1->DR = 0x3E; //Debug signal
				current_seconds++;// += Seconds_Step(current_seconds);
			}
			else {
				USART1->DR = 0x3C; //Debug signal
				current_seconds--;// += Seconds_Step(current_seconds-1);
			}

			if(current_seconds < 0) current_seconds = 0;

			initial_seconds = current_seconds;
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim == &htim1){ //Check if it's TIM1
		millis++;
		if (millis == 1000){
			millis = 0;
			One_Second_Tick();
		}

		if(cool_down_millis > 0) cool_down_millis--;
	}
}

//Ticks once per second
void One_Second_Tick(void){

	//USART1->DR = 0x53; Send char 'S' to UART (to test that UART and timer work)

	if (armed == 1){
		current_seconds--;
		if (current_seconds == 0){
			armed = 0;
		}
	}

}

uint8_t Seconds_Step(uint16_t value){
  if (value < 0) return 0; //To prevent negative values
  if (value < 30) return 5;
  if (value < 120) return 10;
  if (value < 300) return 30;
  return 60;
}

void Set_Digit_Value(uint8_t digit){
	switch (digit){
	case 0: spi_data[1] = DIGIT_0;	break;
	case 1: spi_data[1] = DIGIT_1;	break;
	case 2: spi_data[1] = DIGIT_2;	break;
	case 3: spi_data[1] = DIGIT_3;	break;
	case 4: spi_data[1] = DIGIT_4;	break;
	case 5: spi_data[1] = DIGIT_5;	break;
	case 6: spi_data[1] = DIGIT_6;	break;
	case 7: spi_data[1] = DIGIT_7;	break;
	case 8: spi_data[1] = DIGIT_8;	break;
	case 9: spi_data[1] = DIGIT_9;	break;
	default: spi_data[1] = 0x00; break;
	}
}

void Set_Digit_Index(uint8_t index){
	switch (index){
	case 0: spi_data[0] = SHOW_0;	break;
	case 1: spi_data[0] = SHOW_1;	break;
	case 2: spi_data[0] = SHOW_2;	break;
	case 3: spi_data[0] = SHOW_3;	break;
	default: spi_data[0] = 0x00; break;
	}
}

void Transmit_SPI(void){
	LL_GPIO_ResetOutputPin(GPIOA, spi_latch_pin);
	HAL_SPI_Transmit(&hspi1, p_data, 2, 0xFFFF);
	LL_GPIO_SetOutputPin(GPIOA, spi_latch_pin);
}

void Display_Digits(uint8_t digit_0, uint8_t digit_1, uint8_t digit_2, uint8_t digit_3){

	Set_Digit_Value(digit_0);
	Set_Digit_Index(0);
	Transmit_SPI();

	spi_data[0] = 0x00;
	spi_data[1] = 0x0F;
	Transmit_SPI();

	Set_Digit_Value(digit_1);
	Set_Digit_Index(1);
	spi_data[1] |= 0x80; //Display dots
	Transmit_SPI();

	spi_data[0] = 0x00;
	spi_data[1] = 0x0F;
	Transmit_SPI();

	Set_Digit_Value(digit_2);
	Set_Digit_Index(2);
	Transmit_SPI();

	spi_data[0] = 0x00;
	spi_data[1] = 0x0F;
	Transmit_SPI();

	Set_Digit_Value(digit_3);
	Set_Digit_Index(3);
	Transmit_SPI();

	spi_data[0] = 0x00;
	spi_data[1] = 0x0F;
	Transmit_SPI();

}

void Display_Time(uint16_t seconds){

	uint8_t minutesToDisplay = seconds / 60;
	uint8_t secondsToDisplay = seconds % 60;

	uint8_t digit0 = minutesToDisplay / 10;
	uint8_t digit1 = minutesToDisplay % 10;
	uint8_t digit2 = secondsToDisplay / 10;
	uint8_t digit3 = secondsToDisplay % 10;

	Display_Digits(digit0, digit1, digit2, digit3);
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
