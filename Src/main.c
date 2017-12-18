/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t InputVals[2];
float OutputVals[4];
float sum1;
float wyk;

uint8_t inputLayer;
uint8_t hiddenLayer;
uint8_t outputLayer;

float inputWeights[3][8];
float hiddenWeights[9][4];

float inputNeurons[2];
float hiddenNeurons[8];
float outputNeurons[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

float tansig(float value) {
	wyk = (-2.0) * value;
	return (2.0 / (1.0 + expf(wyk))) -1.0;
}

void feedForward(uint8_t inL, uint8_t hidL, uint8_t outL, float inW[3][8], float hidW[9][4], float inN[2], float hidN[2], float outN[2], uint32_t input[], float output[]) {
	//uint8_t j, k, l;

	inN[0] = (1.0+1.0)*(input[0] - 1000.0) / (1900.0 - 1000.0) - 1.0;
	inN[1] = (1.0+1.0)*(input[1] - 1000.0) / (1900.0 - 1000.0) - 1.0;

	sum1 = inN[0] * inW[0][0] + inN[1] * inW[1][0] + inW[2][0];
	hidN[0] = tansig(sum1);
	sum1 = inN[0] * inW[0][1] + inN[1] * inW[1][1] + inW[2][1];
	hidN[1] = tansig(sum1);
	sum1 = inN[0] * inW[0][2] + inN[1] * inW[1][2] + inW[2][2];
	hidN[2] = tansig(sum1);
	sum1 = inN[0] * inW[0][3] + inN[1] * inW[1][3] + inW[2][3];
	hidN[3] = tansig(sum1);
	sum1 = inN[0] * inW[0][4] + inN[1] * inW[1][4] + inW[2][4];
	hidN[4] = tansig(sum1);
	sum1 = inN[0] * inW[0][5] + inN[1] * inW[1][5] + inW[2][5];
	hidN[5] = tansig(sum1);
	sum1 = inN[0] * inW[0][6] + inN[1] * inW[1][6] + inW[2][6];
	hidN[6] = tansig(sum1);
	sum1 = inN[0] * inW[0][7] + inN[1] * inW[1][7] + inW[2][7];
	hidN[7] = tansig(sum1);

	outN[0] = hidN[0] * hidW[0][0] + hidN[1] * hidW[1][0] + hidN[2] * hidW[2][0] + hidN[3] * hidW[3][0] + hidN[4] * hidW[4][0] + hidN[5] * hidW[5][0]
						+ hidN[6] * hidW[6][0] + hidN[7] * hidW[7][0] + hidW[8][0];
	outN[1] = hidN[0] * hidW[0][1] + hidN[1] * hidW[1][1] + hidN[2] * hidW[2][1] + hidN[3] * hidW[3][1] + hidN[4] * hidW[4][1] + hidN[5] * hidW[5][1]
						+ hidN[6] * hidW[6][1] + hidN[7] * hidW[7][1] + hidW[8][1];
	outN[2] = hidN[0] * hidW[0][2] + hidN[1] * hidW[1][2] + hidN[2] * hidW[2][2] + hidN[3] * hidW[3][2] + hidN[4] * hidW[4][2] + hidN[5] * hidW[5][2]
						+ hidN[6] * hidW[6][2] + hidN[7] * hidW[7][2] + hidW[8][2];
	outN[3] = hidN[0] * hidW[0][3] + hidN[1] * hidW[1][3] + hidN[2] * hidW[2][3] + hidN[3] * hidW[3][3] + hidN[4] * hidW[4][3] + hidN[5] * hidW[5][3]
						+ hidN[6] * hidW[6][3] + hidN[7] * hidW[7][3] + hidW[8][3];

	output[0] = (uint32_t)round((10.0 - 5.0) * (outN[0] + 1.0) / (1.0 + 1.0) + 5.0);
	output[1] = (uint32_t)round((10.0 - 5.0) * (outN[1] + 1.0) / (1.0 + 1.0) + 5.0);
	output[2] = ((1.0 + 1.0) * (outN[2] + 1.0) / (1.0 + 1.0) - 1.0);
	output[3] = ((1.0 + 1.0) * (outN[3] + 1.0) / (1.0 + 1.0) - 1.0);
/*
	for (l = 0; l < layers.hidden; l++) {
		sum1 = 0;
		for (j = 0; j < layers.input; j++) { sum1 += neurons.input[j] * weights.input[j][l]; }
		sum1 += 1 * weights.input[j+1][l];
		neurons.hidden[l] = tansig(sum1);
		checkHidden[l] = neurons.hidden[l];
	}
	*/
/*
	for (k = 0; k < layers.output; k++) {
		sum2 = 0;
		for (j = 0; j < layers.hidden; j++) { sum2 += neurons.hidden[j] * weights.hidden[j][k]; }
		sum2 += 1 * weights.hidden[j+1][k];

		checkOutput[k] = neurons.output[k];
		if (k == 0 || k == 1) { output[k] = (uint32_t)((50.0 - 10.0) * (neurons.output[k] + 1.0) / (1.0 + 1.0) + 10.0); }
		if (k == 2 || k == 3) { output[k] = (uint32_t)((1.0 + 1.0) * (neurons.output[k] + 1.0) / (1.0 + 1.0) - 1.0); }
	}
	*/
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)InputVals, 2);

  //	Budowa sieci
  inputLayer = 2;
  hiddenLayer = 8;
  outputLayer = 4;

  //	Wagi
  inputWeights[0][0] =   3.97681621216933;
  inputWeights[0][1] =  -3.43496298788826;
  inputWeights[0][2] =   0.51806207013445;
  inputWeights[0][3] =   0.82171043263078;
  inputWeights[0][4] =   0.71482567750189;
  inputWeights[0][5] =   3.13587278595063;
  inputWeights[0][6] =  -1.13347989568598;
  inputWeights[0][7] =   2.75103671091913;

  inputWeights[1][0] =  -1.12567585837059;
  inputWeights[1][1] =  -5.60379820912743;
  inputWeights[1][2] =  -1.67260335217006;
  inputWeights[1][3] =  -0.59083546705418;
  inputWeights[1][4] =   2.04527923010639;
  inputWeights[1][5] =   2.53399406174777;
  inputWeights[1][6] =  -4.57620518303079;
  inputWeights[1][7] =   1.37198169402332;

  inputWeights[2][0] =  -3.51380189962795;
  inputWeights[2][1] =   5.82953064721858;
  inputWeights[2][2] =   0.13605362831278;
  inputWeights[2][3] =   0.29849016329210;
  inputWeights[2][4] =  -1.17434690710991;
  inputWeights[2][5] =   1.96714436336079;
  inputWeights[2][6] =  -1.62541860897994;
  inputWeights[2][7] =   2.79152826743692;

  hiddenWeights[0][0] =  -0.374539869053815;
  hiddenWeights[0][1] =   0.005209587497091;
  hiddenWeights[0][2] =   1.450415974891660;
  hiddenWeights[0][3] =   1.450512795351500;

  hiddenWeights[1][0] =   0.045091367937266;
  hiddenWeights[1][1] =   0.105106685428046;
  hiddenWeights[1][2] =  -0.730598594275543;
  hiddenWeights[1][3] =  -0.730520130391922;

  hiddenWeights[2][0] =   0.163770796582358;
  hiddenWeights[2][1] =  -0.199147228985666;
  hiddenWeights[2][2] =   3.122170139602460;
  hiddenWeights[2][3] =   3.123815518815370;

  hiddenWeights[3][0] =  -1.270754271837920;
  hiddenWeights[3][1] =   0.710953002540675;
  hiddenWeights[3][2] =  -3.389839440958070;
  hiddenWeights[3][3] =  -3.391039100880490;

  hiddenWeights[4][0] =  -0.242669899517994;
  hiddenWeights[4][1] =  -0.646304722160610;
  hiddenWeights[4][2] =   2.148537956459040;
  hiddenWeights[4][3] =   2.149746437647770;

  hiddenWeights[5][0] =  -0.158782162500928;
  hiddenWeights[5][1] =  -0.146909674635699;
  hiddenWeights[5][2] =  -0.009617146295958;
  hiddenWeights[5][3] =  -0.008957030265926;

  hiddenWeights[6][0] =   0.120545026126452;
  hiddenWeights[6][1] =   0.269719021014171;
  hiddenWeights[6][2] =  -0.403459385194824;
  hiddenWeights[6][3] =  -0.403554762692929;

  hiddenWeights[7][0] =   0.221448389101420;
  hiddenWeights[7][1] =   0.086036007781647;
  hiddenWeights[7][2] =   0.826170858432722;
  hiddenWeights[7][3] =   0.823929019418342;

  hiddenWeights[8][0] =  -0.145356096680302;
  hiddenWeights[8][1] =  -0.346164769000769;
  hiddenWeights[8][2] =   2.12361827932047;
  hiddenWeights[8][3] =   2.12594115099956;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  feedForward(inputLayer, hiddenLayer, outputLayer, inputWeights, hiddenWeights, inputNeurons, hiddenNeurons, outputNeurons, InputVals, OutputVals);
	  /*
	   * Zamienic zakresy steruj¹ce predkoscia timerow i PWM
	   * Zakres 1-5, gdzie poziomy wynosz¹ odpowiednio: 20%, 40%, 60%, 80% i 100%, a 0 to 0%.
	   */

	  TIM1->CCR1 = OutputVals[0];
	  TIM1->CCR2 = OutputVals[1];
	  //if (OutputVals[0] >= 10) { TIM1->CCR1 = 16; TIM1->CCR2 = 2; } else { TIM1->CCR1 = 2; TIM1->CCR2 = 16; }

	  if (OutputVals[2] > 0) {
		  HAL_GPIO_WritePin(GPIO_Out_Motor1_F_GPIO_Port, GPIO_Out_Motor1_F_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIO_Out_Motor1_R_GPIO_Port, GPIO_Out_Motor1_R_Pin, GPIO_PIN_RESET);
	  }
	  else {
		  HAL_GPIO_WritePin(GPIO_Out_Motor1_F_GPIO_Port, GPIO_Out_Motor1_F_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIO_Out_Motor1_R_GPIO_Port, GPIO_Out_Motor1_R_Pin, GPIO_PIN_SET);
	  }

	  if (OutputVals[3] > 0) {
		  HAL_GPIO_WritePin(GPIO_Out_Motor2_F_GPIO_Port, GPIO_Out_Motor2_F_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIO_Out_Motor2_R_GPIO_Port, GPIO_Out_Motor2_R_Pin, GPIO_PIN_RESET);
	  }
	  else {
		  HAL_GPIO_WritePin(GPIO_Out_Motor2_F_GPIO_Port, GPIO_Out_Motor2_F_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIO_Out_Motor2_R_GPIO_Port, GPIO_Out_Motor2_R_Pin, GPIO_PIN_SET);
	  }

	  HAL_Delay(100);

	  /*
	  if (OutputVals[0] <= 0.5) { TIM1->CCR1 = 100; left = 0; }
	  else if (OutputVals[0] > 0.5 && OutputVals[0] <= 1.5) { TIM1->CCR1 = 300; left = 1; }
	  else if (OutputVals[0] > 1.5) { TIM1->CCR1 = 500; left = 2; }

	  if (OutputVals[1] <= 0.5) { TIM1->CCR2 = 100; right = 0; }
	  else if (OutputVals[1] > 0.5 && OutputVals[2] <= 1.5) { TIM1->CCR2 = 300; right = 1; }
	  else if (OutputVals[1] > 1.5) { TIM1->CCR2 = 500; right = 2; }

	  if (OutputVals[2] <= 0.0) { HAL_GPIO_WritePin(GPIO_Out_Motor1_GPIO_Port, GPIO_Out_Motor1_Pin, GPIO_PIN_SET); leftDir = 1; }
	  else if (OutputVals[2] > 0.0) { HAL_GPIO_WritePin(GPIO_Out_Motor1_GPIO_Port, GPIO_Out_Motor1_Pin, GPIO_PIN_RESET); leftDir = 0; }

	  if (OutputVals[3] <= 0.0) { HAL_GPIO_WritePin(GPIO_Out_Motor2_GPIO_Port, GPIO_Out_Motor2_Pin, GPIO_PIN_SET); rightDir = 1; }
	  else if (OutputVals[3] > 0.0 ) { HAL_GPIO_WritePin(GPIO_Out_Motor2_GPIO_Port, GPIO_Out_Motor2_Pin, GPIO_PIN_RESET); rightDir = 0; }
	  */
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3599;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_Out_Motor1_R_Pin|GPIO_Out_Motor1_F_Pin|GPIO_Out_Motor2_R_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_Out_Motor2_F_GPIO_Port, GPIO_Out_Motor2_F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_Out_Motor1_R_Pin GPIO_Out_Motor1_F_Pin GPIO_Out_Motor2_R_Pin LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_Out_Motor1_R_Pin|GPIO_Out_Motor1_F_Pin|GPIO_Out_Motor2_R_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_Out_Motor2_F_Pin */
  GPIO_InitStruct.Pin = GPIO_Out_Motor2_F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_Out_Motor2_F_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

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
