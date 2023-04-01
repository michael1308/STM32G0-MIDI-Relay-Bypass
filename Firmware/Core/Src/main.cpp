/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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
#include <stdbool.h>
#include "MIDI.h"
#include "serialMIDI.h"

/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HAL_FLASH_MODULE_ENABLED
#define MIDI_CHANNEL 1
#define MIDI_CC_CHANNEL_1 80
#define MIDI_CC_CHANNEL_2 81
#define ADDR_FLASH_PAGE_31    ((uint32_t)0x0800F800)
#define MIDI_CHANNEL_DEFAULT 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx; 


/* USER CODE BEGIN PV */
bool setChannel = 0;
uint8_t UART1_rxBuffer[1] = {0};
uint8_t state1 = 0;
uint8_t state2 = 0;
uint8_t bTurnOn1 = 0;
uint8_t bTurnOff1 = 0;
uint8_t bTurnOn2 = 0;
uint8_t bTurnOff2 = 0;
uint8_t isCC = 0;
uint8_t isCorrectCCChannel1 = 0;
uint8_t isCorrectCCChannel2 = 0;
uint64_t midi_channel;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
bool btnState(GPIO_TypeDef*, uint16_t);
void displayChannel(int, GPIO_TypeDef*, uint16_t);
void turnOn(GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t);
void turnOff(GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t);
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
  uint16_t lastPressMillis = 0;
    /* Erase Flash Page 31*/
  FLASH_EraseInitTypeDef FLASH_EraseInitStruct = {0};
  FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;    //Erase type set to erase pages( Available other type is mass erase)
  FLASH_EraseInitStruct.Page = 31;            //Starting address of flash page (0x0800 0000 - 0x0801 FC00)
  FLASH_EraseInitStruct.NbPages = 1;                    //The number of pages to be erased                  
  uint32_t  errorStatus = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Write Channel to Flash*/
  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&FLASH_EraseInitStruct,&errorStatus);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,ADDR_FLASH_PAGE_31, MIDI_CHANNEL_DEFAULT);
  HAL_FLASH_Lock();

  /* Read MIDI Channel from Flash*/
  uint64_t * RDAddr = (uint64_t *)  ADDR_FLASH_PAGE_31;
  midi_channel = *RDAddr;
  
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
  {
    HAL_Delay(3000);
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
      setChannel = 1;
    HAL_Delay(3000);
    displayChannel(midi_channel, LED1_GPIO_Port, LED1_Pin);
  }
  while(setChannel)
  {
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
    {
      midi_channel++;
      if(midi_channel > 16)
      {
        midi_channel = 1;
      }
      displayChannel(midi_channel, LED1_GPIO_Port, LED1_Pin);
      lastPressMillis = HAL_GetTick();
    }
    if (HAL_GetTick() - lastPressMillis > 20000)
    {
      displayChannel(3, LED1_GPIO_Port, LED1_Pin);

      HAL_FLASH_Unlock();
      HAL_FLASHEx_Erase(&FLASH_EraseInitStruct,&errorStatus);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,ADDR_FLASH_PAGE_31, midi_channel);
      HAL_FLASH_Lock();

      setChannel = 0;
    }
  }
  
  HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 1);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(btnState(SW1_GPIO_Port, SW1_Pin))
    {
      if(state1)
      {
        turnOff(LED1_GPIO_Port, LED1_Pin, FET1_GPIO_Port, FET1_Pin, RELAY1_GPIO_Port, RELAY1_Pin);
        state1 = 0;
      }
      else
      {
        turnOn(LED1_GPIO_Port, LED1_Pin, FET1_GPIO_Port, FET1_Pin, RELAY1_GPIO_Port, RELAY1_Pin);
        state1 = 1;
      }
      bTurnOff1 = 0;
      bTurnOn1 = 0;
    }
    if(bTurnOff1)
    {
      if(state1)
        {
          turnOff(LED1_GPIO_Port, LED1_Pin, FET1_GPIO_Port, FET1_Pin, RELAY1_GPIO_Port, RELAY1_Pin);
          state1 = 0;
        }
      bTurnOff1 = 0;
    }
    if(bTurnOn1)
    {
      if(!state1)
        {
          turnOn(LED1_GPIO_Port, LED1_Pin, FET1_GPIO_Port, FET1_Pin, RELAY1_GPIO_Port, RELAY1_Pin);
          state1 = 1;
        }
      bTurnOn1 = 0;
    }

    if(btnState(SW2_GPIO_Port, SW2_Pin))
    {
      if(state2)
      {
        turnOff(LED2_GPIO_Port, LED2_Pin, FET2_GPIO_Port, FET2_Pin, RELAY2_GPIO_Port, RELAY2_Pin);
        state2 = 0;
      }
      else
      {
        turnOn(LED2_GPIO_Port, LED2_Pin, FET2_GPIO_Port, FET2_Pin, RELAY2_GPIO_Port, RELAY2_Pin);
        state2 = 1;
      }
      bTurnOff2 = 0;
      bTurnOn2 = 0;
    }
    if(bTurnOff2)
    {
      if(state2)
        {
          turnOff(LED2_GPIO_Port, LED2_Pin, FET2_GPIO_Port, FET2_Pin, RELAY2_GPIO_Port, RELAY2_Pin);
          state2 = 0;
        }
      bTurnOff2 = 0;
    }
    if(bTurnOn2)
    {
      if(!state2)
        {
          turnOn(LED2_GPIO_Port, LED2_Pin, FET2_GPIO_Port, FET2_Pin, RELAY2_GPIO_Port, RELAY2_Pin);
          state2 = 1;
        }
      bTurnOn2 = 0;
    }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 31250;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY2_Pin|LED1_Pin|LED2_Pin|FET1_Pin
                          |FET2_Pin|RELAY1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RELAY2_Pin LED1_Pin LED2_Pin FET1_Pin
                           FET2_Pin RELAY1_Pin */
  GPIO_InitStruct.Pin = RELAY2_Pin|LED1_Pin|LED2_Pin|FET1_Pin
                          |FET2_Pin|RELAY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void turnOn(GPIO_TypeDef *LED_GPIO_Port, uint16_t LED_Pin, GPIO_TypeDef *FET_GPIO_Port, uint16_t FET_Pin, GPIO_TypeDef *RELAY_GPIO_Port, uint16_t RELAY_Pin)
{
  HAL_GPIO_WritePin(FET_GPIO_Port, FET_Pin, GPIO_PIN_SET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(FET_GPIO_Port, FET_Pin, GPIO_PIN_RESET);
}

void turnOff(GPIO_TypeDef *LED_GPIO_Port, uint16_t LED_Pin, GPIO_TypeDef *FET_GPIO_Port, uint16_t FET_Pin, GPIO_TypeDef *RELAY_GPIO_Port, uint16_t RELAY_Pin)
{
  HAL_GPIO_WritePin(FET_GPIO_Port, FET_Pin, GPIO_PIN_SET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(FET_GPIO_Port, FET_Pin, GPIO_PIN_RESET);
}

void displayChannel(int channel, GPIO_TypeDef *LED_GPIO_Port, uint16_t LED_Pin)
{
  for (int i = 0; i < channel; i++)
  {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(250);
  }
}

bool btnState(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  static uint16_t state = 0;
  state = (state<<1) | (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) | 0xfe00;
  return (state == 0xff00);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // HAL_UART_Transmit(&huart1, UART1_rxBuffer, 12, 100);
  if(UART1_rxBuffer[0] == 0xB0)
    isCC = 1;
  else if(isCC == 1 && UART1_rxBuffer[0] == MIDI_CC_CHANNEL_1)
    isCorrectCCChannel1 = 1;
  else if(isCC == 1 && UART1_rxBuffer[0] == MIDI_CC_CHANNEL_2)
    isCorrectCCChannel2 = 1;
  else if(isCorrectCCChannel1 && UART1_rxBuffer[0] >= 0x40)
  {
    bTurnOn1 = 1;
    isCC = 0;
    isCorrectCCChannel1 = 0;
  }
  else if(isCorrectCCChannel2 && UART1_rxBuffer[0] >= 0x40)
  {
    bTurnOn2 = 1;
    isCC = 0;
    isCorrectCCChannel2 = 0;
  }
  else if(isCorrectCCChannel1 && UART1_rxBuffer[0] < 0x40)
  {
    bTurnOff1 = 1;
    isCC = 0;
    isCorrectCCChannel1 = 0;
  }
  else if(isCorrectCCChannel2 && UART1_rxBuffer[0] < 0x40)
  {
    bTurnOff2 = 1;
    isCC = 0;
    isCorrectCCChannel2 = 0;
  }
  else
  {
    isCC = 0;
    isCorrectCCChannel1 = 0;
    isCorrectCCChannel2 = 0;
  }
    
  HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 1);
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
