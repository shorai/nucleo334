/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "stm32f3xx_hal_opamp.h"


// Combinations (steps, prescaler) that work ..... (400,100)
// want 12800 steps per rev
#define SPEED 32
//#define stepsPerRev (400*4*SPEED)
#define stepsPerRev 12800
//#define PRESCALER_VALUE (128/SPEED)
#define PRESCALER_VALUE (64/SPEED)

/**
 * The pushbutton is used in EXTI mode
 * Had to put a handler in the -it.c file and a callback here
 * Not sure how to handle multiple pins in the IRW for pins 10-15
 *
 * The polled version is not fully switchable from this define (yet)
 */

//#define POLLED_PB

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

OPAMP_HandleTypeDef hopamp2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_OPAMP2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */

//*	@TODO:  Could not get pushbutton on STM32F334 nucleo working on EXTI

#define POLLED_PB
#ifdef POLLED_PB
void MAIN_pushbutton(void) ;
#endif
static void MAIN_rxChar(char ch);
void MAIN_startMotor(uint32_t frames);
void MAIN_stopMotor(void);

/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
 * Main movie capture
 * ==================
 *
 * This process drives the system
 *
 * 1) The ADC reads the gap sensor tied to the frame advance mechanism
 *        On Interrupt (signal going low through 50%)
 *           The command 'Snap' is sent via USART
 *           The movie is advanced one frame
 *    The Gap sensor is read by DMA (half and full complete)
 *         If it transitioned a threshold, a snap is requested
 *         The motor does not stop
 *     It would be better to use the Analog watchdog
 * 2) Pushing blue button also sends snap
 *
 * 3) user can send one char commands
 *
 * 'S' command (Start) sets the motor going
 * 'E' stops capturing or rewinding
 * 'R' rewinds (second motor)
 *
 * '0' Ends capture or rewind
 * '1'..'9' Frame .... takes one to 9 frames
 *
 *  @TODO:  Accelerate up to a faster speed, decelerate to stop
 *  @TODO:  Wind takeup motor @ speed / 10(start) speed/30 (end)
 *  @TODO: Rewind
 *
 *
 *  @TODO:  Commands for 5 minute, 10 minute, 20 minute reels (ABC)
 *  @TODO:  Best timing for shutter, may have to step beyond Gap sensor
 *
 *
 *	@DONE:  ? help command
 *
 *  OPENCV Capture device
 *  --------------------
 *
 *  The openCV capture device can be pretty simple
 *
 *  1) When it receives 'snap\r\n' it captures a frame
 *  2) Can pass through single character commands
 *  3) Can check for broken film (frame == white) and stop
 *
 *  clock division 4, period 1000, pulse 500
 * htim1.Init.Prescaler = 70; //70;  good // 100; good // 400; very slow  // 50 skipping // 60 noisy
 *
 *
 */

static const char *helpString = "* Main movie capture\r\n"\
		 "* ==================\r\n"\
		 "*\r\n"\
		 "* This process drives the system\r\n"\
		 "*\r\n"\
		 "* 1) The ADC reads the gap sensor tied to the frame advance mechanism\r\n"\
		 "*        On Interrupt (signal going low through 50%)\r\n"\
		 "*           The command 'Snap' is sent via USART\r\n"\
		 "*           The movie is advanced one frame\r\n"\
		 "*    The Gap sensor is read by DMA (half and full complete)\r\n"\
		 "*         If it transitioned a threshold, a snap is requested\r\n"\
		 "*         The motor does not stop\r\n"\
		 "*     It would be better to use the Analog watchdog\r\n"\
		 "* 2) Pushing blue button also sends snap\r\n"\
		 "*\r\n"\
		 "* 3) user can send one char commands\r\n"\
		 "*\r\n"\
		 "* 'S' command (Start) sets the motor going\r\n"\
		 "* 'E' stops capturing or rewinding\r\n"\
		 "* 'R' rewinds (second motor)\r\n"\
		 "*\r\n"\
		 "* '0' Ends capture or rewind\r\n"\
		 "* '1'..'9' Frame .... takes one to 9 frames\r\n"\
		 "*\r\n"\
		 "*  @TODO:  Accelerate up to a faster speed, decelerate to stop\r\n"\
		 "*  @TODO:  Wind takeup motor @ speed / 10(start) speed/30 (end)\r\n"\
		 "*\r\n"\
		 "*  @TODO:  Commands for 5 minute, 10 minute, 20 minute reels (ABC)\r\n"\
		 "*  @DONE:  ? help command\r\n"\
		 "*  @TODO:  BEst timing for shutter\r\n"\
		 "*\r\n"\
		 "*  OPENCV Cpture device\r\n"\
		 "*  --------------------\r\n"\
		 "*\r\n"\
		 "*  The openCV capture device can be pretty simple\r\n"\
		 "*\r\n"\
		 "*  1) When it receives 'snap\r\n' it captures a frame\r\n"\
		 "*  2) Can pass through single character commands\r\n"\
		 "*  3) Can check for broken film (frame == white) and stop\r\n"\
		 "";

/**
 * Only two ADC values are buffered by DMA in circular mode.
 *
 * The Half and Full complete IRQ can process
 *
 * Ideally we get rid of these and use the ADC watchdog to look for a
 *      H->L transition
 */
#define ADC_LEN 100
uint16_t ADC_buffer[ADC_LEN];
char rxChar = 0;

/**
 * Likewise the Rx buffer is just 2 characters,
 *    enabling the reception and processing of individual keystrokes
 *    most are handled in the IRQ itself
 */

/** RUnning has values .. <0 == run continuously
*                          0 == stopped
*							>0 = snap N frames
*/
int32_t MAIN_running = 0;

uint8_t MAIN_rewinding = 0;
uint8_t MAIN_snap = 0;



char UART_RxBuffer[2];

 //	@TODO:  STM libs don't stop USART DMA transfers

void My_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
	//while ((HAL_UART_GetState(&huart2) & HAL_UART_STATE_BUSY_TX )== HAL_UART_STATE_BUSY_TX )
	if (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
		    	 HAL_Delay(100);

	HAL_Delay(10);
	huart2.State = HAL_UART_STATE_READY;  // Hal libs 1/3/2016 aren't clearing this on end of DMA Transmit


	HAL_UART_Transmit_DMA(huart, pData, Size);
}
void MAIN_enable(GPIO_PinState val) {
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,val);  // main motor
	if (val== GPIO_PIN_SET) {
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);  // takeup motor
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);  // rewind motor

	} else {
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);  // takeup motor
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);  // rewind motor
	}
}

void MAIN_direction(GPIO_PinState val) {
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,val); // main motor
	if (val== GPIO_PIN_SET) {
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);// takeup motor
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);// rewind motor
	} else {
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
	}

}

char * SnapMsgx = "snap                             \r\n";
uint8_t snapADC = 0;
uint8_t snapADCx = 0;

uint8_t snapPoll = 0;
uint8_t snapWatch = 0;

char SnapMsg[35];

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	MAIN_running = 0;
	MAIN_rewinding = 0;
	MAIN_snap = 0;


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_OPAMP2_Init();

  /* USER CODE BEGIN 2 */


  //htim1.Instance->PSC = PRESCALER_VALUE;

  HAL_ADC_Start_DMA(&hadc2,(uint32_t * )&ADC_buffer,ADC_LEN);
  HAL_UART_Receive_DMA(&huart2,(uint8_t *) &UART_RxBuffer,2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // __HAL_GPIO_EXTI_INIT(PORTC,13);  // pushbutton
  //HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
  // HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));

  //My_UART_Transmit_DMA(&huart2,(uint8_t *) "Starting Moviecap\r\n",19);


#ifndef POLLED_PB
  HAL_NVIC_SetPriority(EXTI15_10_IRQn,0xf,0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

#endif

  //if(HAL_WWDG_Start(&hwwdg) != HAL_OK)
  //{
   // while(-1);
  //}

  while (1)
  {
	  strcpy(SnapMsg,SnapMsgx);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
#ifdef POLLED_PB
	  MAIN_pushbutton();
#endif
	//  if(HAL_WWDG_Refresh(&hwwdg, 127) != HAL_OK)
	//    {
	//      while(-1);
	//    }

	  if (MAIN_snap) {
		if ((HAL_UART_GetState(&huart2) & HAL_UART_STATE_BUSY_TX )== HAL_UART_STATE_BUSY_TX )
	    	 HAL_Delay(10);

		huart2.State = HAL_UART_STATE_READY;  // Hal libs 1/3/2016 aren't clearing this on end of DMA Transmit

	    MAIN_snap = 0;

	    if (MAIN_running > 0)
	    	MAIN_running--;



	    if (snapWatch!=0) {
	    	snapWatch=0;
	    	SnapMsg[5] = 'W';
	    }

	    if (snapPoll!=0) {
	    	SnapMsg[6] = 'P';
	    	snapPoll = 0;
	    }

	    if (snapADC!=0) {
	    	SnapMsg[7] = 'A';
	    	snapADC = 0;
	    }


	    if (snapADCx!=0) {
	    	SnapMsg[7] = 'X';
	    	snapADCx = 0;
	    }


	  }
		My_UART_Transmit_DMA(&huart2,(uint8_t *) SnapMsg,strlen(SnapMsg));




	//	MAIN_running--;
		if (MAIN_running == 0) {
			MAIN_stopMotor();
			MAIN_running--;
		} //else //if (MAIN_running > 0) {
			//MAIN_running--;
		//}
		if (rxChar != 0)
			MAIN_rxChar(rxChar);
		//HAL_Delay(10);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 2);
}

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc2.Init.Resolution = ADC_RESOLUTION12b;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc2);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

}

/* OPAMP2 init function */
void MX_OPAMP2_Init(void)
{

  hopamp2.Instance = OPAMP2;
  hopamp2.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_VP0;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  HAL_OPAMP_Init(&hopamp2);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 6;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 60;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_MspPostInit(&htim3);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
// ***********************************************   UART Rx ......
static void MAIN_rxChar(char ch) {
	uint32_t i,k;
	//volatile uint32_t j;
	switch (ch) {


	case 'E':
	case 'e':
	case '0':// MAIN_running = 0;
				MAIN_stopMotor();
			  break;


	case 'O':
	case 'o':
		for (k=0; k < 100; k++) {
			for(i=0; i < 1600; i++) {
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
				HAL_Delay(10);
				//for(j=0; j < 20000; j++);
			}
		//for (j=0; j < 0; j++);
			HAL_Delay(1000);
		}

	break;

	case 'R':
	case 'r': MAIN_rewinding = 1;
			break;

	case 'S' :
	case 's' : //if (MAIN_running == 0) {
				//MAIN_running = -1;
				MAIN_startMotor(5*60*18);  // 5 minutes of frames
				//}
			break;

	case '?': My_UART_Transmit_DMA(&huart2,(uint8_t *) helpString, strlen(helpString));
		break;

	case '1' :  // these may be better as minutes of capture
	case '2' :
	case '3' :
	case '4' :
	case '5' :
	case '6' :
	case '7' :
	case '8' :
	case '9' :
		if (MAIN_running <= 0) {
					//MAIN_running = ch - '0';
					MAIN_startMotor(ch-'0');  // 5 minutes of frames
					}
		break;

	}
	rxChar = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//MAIN_rxChar(UART_RxBuffer[1]);
	rxChar = UART_RxBuffer[1];
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	//MAIN_rxChar(UART_RxBuffer[0]);
	rxChar = UART_RxBuffer[0];
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){}

//***********************************************************************************
// ************************************************************* ADC handling
// really like to do this with a watchdog

// @TODO Analog watchdog not working


static 	uint16_t ltMinADC=4096;
static uint16_t ltMaxADC=0;

void itoar(uint16_t val, char *str) {
	while(val) {
		*str-- = '0' + val%10;
		val /=10;
	}
}

#define ADC_LOW 1500
#define ADC_HIGH 2500

void MAIN_ADC_handler(uint16_t index) {
	static uint16_t MAIN_lastADC=0;
	    uint16_t minADC;
		uint16_t maxADC;

		//uint16_t x;

	uint16_t x;// =  ADC_buffer[index];
	minADC = maxADC = ADC_buffer[index];

	uint16_t cnt = 0;
	for (cnt=0; cnt < ADC_LEN/2; cnt++) {
		x = ADC_buffer[index+cnt];

		if (x > ADC_HIGH)
			if (MAIN_lastADC < ADC_LOW) {
			//HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				MAIN_snap = 1;
				snapADC = 1;
		}
		if (x < minADC) minADC = x;
		if (x > maxADC) maxADC = x;
	}
//	if ((minADC < ADC_LOW) && (maxADC > ADC_HIGH)) {
	if (maxADC > ADC_HIGH) {
		MAIN_snap = 1;
		snapADCx = 1;
	}

	MAIN_lastADC = x;
	if (ltMinADC > minADC) ltMinADC = minADC;
	if (ltMaxADC < maxADC) ltMaxADC = maxADC;

	itoar(minADC,&SnapMsg[15]);
	itoar(maxADC,&SnapMsg[25]);

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	MAIN_ADC_handler(0);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	MAIN_ADC_handler(ADC_LEN/2);
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef * hadc) {
	static uint8_t blocked = 0;
	//@TODO:   how do we know which transition occurred?
	if ((hadc->Instance->DR < 2000) && (blocked == 0)) {// going negative
		MAIN_snap = 1;
		snapWatch = 1;

	    blocked =1;
	} else {
		//snapWatch=0;
		if (hadc->Instance->DR > 2200) // must go positive before we take another pic
		blocked = 0;

	}
}
#ifdef USE_WWDG
void HAL_WWDG_Callback(WWDG_HandleTypeDef *wwdg) {
	uint32_t x = wwdg->Instance->CR;

}
#endif

// hard fault handler in C,
// with stack frame location as input parameter
// called from HardFault_Handler in file xxx.s
char errors[16][80];
void hard_fault_handler_c (unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  sprintf (errors[0],"\n\n[Hard fault handler - all numbers in hex]\n");
  sprintf (errors[1],"R0 = %x\n", stacked_r0);
  sprintf (errors[2],"R1 = %x\n", stacked_r1);
  sprintf (errors[3],"R2 = %x\n", stacked_r2);
  sprintf (errors[4],"R3 = %x\n", stacked_r3);
  sprintf (errors[5],"R12 = %x\n", stacked_r12);
  sprintf (errors[6],"LR [R14] = %x  subroutine call return address\n", stacked_lr);
  sprintf (errors[7],"PC [R15] = %x  program counter\n", stacked_pc);
  sprintf (errors[8],"PSR = %x\n", stacked_psr);
  sprintf (errors[9],"BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
  sprintf (errors[10],"CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
  sprintf (errors[11],"HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
  sprintf (errors[12],"DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
  sprintf (errors[13],"AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
  sprintf (errors[14],"SCB_SHCSR = %x\n",(unsigned int) SCB->SHCSR);
  sprintf (errors[15],"minADC %d   maxADC %d\n",ltMinADC, ltMaxADC);

  while (1);
}



void WWDG_IRQHandlerx(void) {
	while(-1);
}


void UsageFault_Handlerx(void) {
	while(-1);
}


void BusFault_Handler(void) {
	  __asm volatile (
	    " movs r0,#4       \n"
	    " movs r1, lr      \n"
	    " tst r0, r1       \n"
	    " beq _MSP         \n"
	    " mrs r0, psp      \n"
	    " b _HALT          \n"
	  "_MSP:               \n"
	    " mrs r0, msp      \n"
		" mrs r3, psr      \n"
	  "_HALT:              \n"
	    " ldr r1,[r0,#20]  \n"
	    " bkpt #0          \n"
	  );
	while(-1);
}

void MemMang_Handler(void) {
	  __asm volatile (
	    " movs r0,#4       \n"
	    " movs r1, lr      \n"
	    " tst r0, r1       \n"
	    " beq _MSP1         \n"
	    " mrs r0, psp      \n"
	    " b _HALT1          \n"
	  "_MSP1:               \n"
	    " mrs r0, msp      \n"
		" mrs r3, psr      \n"
	  "_HALT1:              \n"
	    " ldr r1,[r0,#20]  \n"
	    " bkpt #0          \n"
	  );
	while(-1);
}

void HardFault_Handler(void) {
#ifdef NEVER
	/* Load the address of the interrupt control register into r3. */
	  asm("ldr r3, IPSR"); //NVIC_INT_CTRL_CONST
	  /* Load the value of the interrupt control register into r2 from the
	  address held in r3. */
	  asm("ldr r2, [r3, #0]");
	  /* The interrupt number is in the least significant byte - clear all
	  other bits. */
	  asm("uxtb r2, r2");

DefaultHandler:
      movs r0,#4
	     movs r1, lr
	     tst r0, r1
	     beq _MSP
	     mrs r0, psp
	     b _HALT
	     _MSP:
	     mrs r0, msp
	     mrs r3,ipsr	// number of IRQ handler being serviced
	     _HALT:
	     ldr r1,[r0,#20]
	     bkpt #0

	  __asm volatile (
	    " movs r0,#4       \n"
	    " movs r1, lr      \n"
	    " tst r0, r1       \n"
	    " beq _MSP2         \n"
	    " mrs r0, psp      \n"
	    " b _HALT2          \n"
	  "_MSP2:               \n"
	    " mrs r0, msp      \n"
		" mrs r3, psr      \n"
	  "_HALT2:              \n"
	    " ldr r1,[r0,#20]  \n"
	    " bkpt #0          \n"
	  );
#endif
	     __asm volatile (
	       " TST LR, #4 \n"
	       " ITE EQ \n"
	       " MRSEQ R0, MSP \n"
	       " MRSNE R0, PSP \n"
	       " B hard_fault_handler_c \n"
		   );
}



//void DMA1_Channel7_IRQHandler(void) {
//	while(-1);
//}

void ADC1_2_IRQHandler(void) {
	while(-1);
}
// **********************************************************************
// ******************************************************** Pushbutton
//  Debounce   we poll 10 times for low in a row
#ifdef POLLED_PB
uint8_t MAIN_PBState = 0;

void MAIN_pushbutton(void) {

	if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == GPIO_PIN_RESET) {
		MAIN_PBState++;
		if (MAIN_PBState >=10) {
			//MAIN_snap = 1;
			MAIN_startMotor(1);
			//MAIN_running = 1;
		}
	} else {
		MAIN_PBState = 0;
	}
}
#else
/**
 * To get this to work, initialise the pin in EXTI mode, Rising / Falling
 *
 * This has been done for us already
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
		MAIN_snap = 1;
		MAIN_startMotor(1);
		MAIN_running = 1;
	}
}
#endif
// **********************************************************************
// Stepper controls
/**
 * The stepper must know how to do exactly one rev
 *
 * Most of the time though, it will run continuously
 */

/**
 * IRQ on GAP sensor
 */
void MAIN_GAP_handler(void) {
	MAIN_snap = 1;
	if (MAIN_running > 0) {
		MAIN_snap = 1;
		snapPoll = 1;
	} else {
		MAIN_stopMotor();
	};

}



void MAIN_startMotor(uint32_t frames) {
	MAIN_direction(GPIO_PIN_RESET);
	MAIN_enable(GPIO_PIN_SET);




	MAIN_running = frames;
	htim1.Instance->RCR = frames * stepsPerRev;

	//HAL_TIM_Base_Start(&htim1);
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK) {
	//		My_UART_Transmit_DMA(&huart2,(uint8_t *) "Frame Motor Started ",15);
	       /* PWM Generation Error */
	     } else {
	//    	My_UART_Transmit_DMA(&huart2,(uint8_t *) "Frame Motor Did not Start",15);

	     }

	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) == HAL_OK) {
	//		My_UART_Transmit_DMA(&huart2,(uint8_t *) ",Takeup  Started\r\n",15);
	       /* PWM Generation Error */
	     } else {
	//    	My_UART_Transmit_DMA(&huart2,(uint8_t *) ",Takeup Motor Did not Start\r\n",15);

	     }

	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) == HAL_OK) {
	//			My_UART_Transmit_DMA(&huart2,(uint8_t *) ",Rewind  Started\r\n",15);
		       /* PWM Generation Error */
		     } else {
	//	    	My_UART_Transmit_DMA(&huart2,(uint8_t *) ",Rewind Motor Did not Start\r\n",15);



		     }
}

void MAIN_stopMotor() {
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);

	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);

	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
	//HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);


	MAIN_enable(GPIO_PIN_RESET);
	//htim1.Instance->CNT = 0;
	htim1.Instance->RCR = 0;
	MAIN_running = 0;
	//My_UART_Transmit_DMA(&huart2,(uint8_t *) "Motors Stopped\r\n",15);
}

//HAL_TIM_PWM_PulseFinishedCallback();
// HAL_TIM_PeriodElapsedCallback();


/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
