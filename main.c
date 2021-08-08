
#include "main.h"

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);

#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1                                                          \
  0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x80      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

uint32_t value0;
uint32_t value1;
int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	begin();
	setPWMFreq(50);
	while (1)
	{
		//read first Potentiometer 
		ADC_ChannelConfTypeDef sConfig = {0};
	  	sConfig.Channel = ADC_CHANNEL_0;
	  	sConfig.Rank = ADC_REGULAR_RANK_1;
	  	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	  	HAL_ADC_Start (&hadc1);
	  	HAL_ADC_PollForConversion (&hadc1, 1000);
	  	value0 = HAL_ADC_GetValue (&hadc1);
	  	HAL_ADC_Stop (&hadc1);

		//read second Potentiometer 
	  	sConfig.Channel = ADC_CHANNEL_1;
	  	sConfig.Rank = ADC_REGULAR_RANK_1;
	  	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	  	HAL_ADC_Start (&hadc1);
	  	HAL_ADC_PollForConversion (&hadc1, 1000);
	  	value1 = HAL_ADC_GetValue (&hadc1);
	  	HAL_ADC_Stop (&hadc1);

	  	printf("value %d, %d\n", value0, value1);
	  	set_pwm(&hi2c1, PCA9685_I2C_ADDRESS, 0, 0, (uint16_t)(value0/16)+30);	//set PCA9685 channel 0 with reading from first Potentiometer 
	  	set_pwm(&hi2c1, PCA9685_I2C_ADDRESS, 1, 0, (uint16_t)(value1/16)+30);	//set PCA9685 channel 1 with reading from second Potentiometer 
	  	HAL_Delay(20);
	}
}

void setPWMFreq(float freq)
{

	// Range output modulation frequency is dependant on oscillator
	if (freq < 1)
		freq = 1;
	if (freq > 3500)
		freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

	float prescaleval = ((FREQUENCY_OSCILLATOR / (freq * 4096.0)) + 0.5) - 1;
	if (prescaleval < PCA9685_PRESCALE_MIN)
		prescaleval = PCA9685_PRESCALE_MIN;
	if (prescaleval > PCA9685_PRESCALE_MAX)
		prescaleval = PCA9685_PRESCALE_MAX;
	uint8_t prescale = (uint8_t)prescaleval;

	uint8_t oldmode;
	HAL_I2C_Mem_Read(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &oldmode, 1, 10000);
	uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &newmode, 1, 10000);
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_PRESCALE, I2C_MEMADD_SIZE_8BIT, &prescale, 1, 10000);
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &oldmode, 1, 10000);
	HAL_Delay(5);
	// This sets the MODE1 register to turn on auto increment.
	uint8_t finalmode = oldmode | MODE1_RESTART | MODE1_AI;
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &finalmode, 1, 10000);
}

void begin()
{
	uint8_t oldmode = MODE1_RESTART;
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, I2C_MEMADD_SIZE_8BIT, &oldmode, 1, 10000);
	HAL_Delay(10);
	setPWMFreq(1000);
}

void set_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off)
{
	HAL_StatusTypeDef ret;
	uint8_t data0 = on;
	uint8_t data1 = (on >> 8);
	uint8_t data2 = off;
	uint8_t data3 = (off >> 8);
	printf("set pwm %d, %d\n", (uint32_t)data0+(uint32_t)data1*256, (uint32_t)data2+(uint32_t)data3*256);
	printf("this %d, %d, %d, %d\n", PCA9685_LED0_ON_L + 4 * num, PCA9685_LED0_ON_H + 4 * num, PCA9685_LED0_OFF_L + 4 * num, PCA9685_LED0_OFF_H + 4 * num);
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_LED0_ON_L + 4 * num, I2C_MEMADD_SIZE_8BIT, &data0, 1, 100);
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_LED0_ON_H + 4 * num, I2C_MEMADD_SIZE_8BIT, &data1, 1, 100);
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_LED0_OFF_L + 4 * num, I2C_MEMADD_SIZE_8BIT, &data2, 1, 100);
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_LED0_OFF_H + 4 * num, I2C_MEMADD_SIZE_8BIT, &data3, 1, 100);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
