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
#include "stm32f7xx_hal.h"
#include <math.h>
#include <stdbool.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
unsigned char RXData[15];																//Array for Bluetooth reception
unsigned char X_Char;																		//Stores received X as Char
unsigned char Y_Char;																		//Stores received Y as Char
unsigned char R_Char;																		//Stores received R as Char
int X_Vect =0;																					//Stores received X as Int
int Y_Vect =0;																					//Stores received Y as Int
int R_Vect =0;																					//Stores received R as Int
int legAngles[15];																			//Stores the desired angle for each limb
int legAngles_id[] = {1, 2, 3, 4, 5,										//Stores the ID of each limb for sorting
											6, 7, 8, 9, 10,
											11, 12, 13, 14, 15};
int servoOffset[] = {300,315,300,305,295,								//Array for storing the offset of each servo
											430,460,450,440,442,
											-110,-112,-105,-78,-98};		
int servoMultiplier[] = {	-360,-380,-363,-365,-370,			//Array for storing the multiplier of each servo
													-400,-400,-400,-400,-400,
													480,545,490,460,500};		
int servoCount = 0;																			//Counter used for keeping track of servo interrupts
char servoCountChar[2];
int BTCount = 0;																				//Counter for BlueTooth LED
int newPeriod = 0;

static TIM_HandleTypeDef ServoTimer;
static TIM_HandleTypeDef ServoTimer2;

unsigned char BTRewrite[] = {'X','Y','R','\r'};					//String used for debugging

struct servos {
	int theta;
	int phi;
	int alpha;
};

struct coordinates{
	double x;
	double y;
	double z;
};

double rad2deg = 57.29577951308232;
double A_length = 50;
double B_length = 106;
double C_length = 130;
double z_body = 30;
double robotRadius = 90;   

bool resetStatus[5];																		//Array used to store reset status of each leg

double X_Vector = 0;																		//Global movement vectors
double Y_Vector = 0;
double R_Vector = 0;	

double destination[2][5];																//2D array that stores X([0]) and Y([1]) values 
																												//for the destination of each foot
double currentPosition[2][5];														//2D array that stores X and Y values for the 
																												//current position of each foot
double translateLeg[2][5];															//Stores coordinates of joint 1 for each leg.
																												//Replaces TranslateLeg()
//Peripheral pin & port difinitions
#define BTPort 					GPIOE
#define BTLEDPin 				GPIO_PIN_0
#define redLEDPort 			GPIOB
#define redLEDPin 			GPIO_PIN_9
#define greenLEDPort 		GPIOB
#define greenLEDPin 		GPIO_PIN_8
#define servoPowerPort	GPIOC
#define servoPowerPin		GPIO_PIN_9

//Servo pin & port definitions
#define servo1Pin 		GPIO_PIN_12
#define servo1Port 		GPIOB
#define servo2Pin 		GPIO_PIN_13
#define servo2Port 		GPIOB
#define servo3Pin 		GPIO_PIN_14
#define servo3Port 		GPIOB
#define servo4Pin 		GPIO_PIN_15
#define servo4Port 		GPIOB
#define servo5Pin 		GPIO_PIN_8
#define servo5Port 		GPIOD
#define servo6Pin 		GPIO_PIN_9
#define servo6Port 		GPIOD
#define servo7Pin 		GPIO_PIN_10
#define servo7Port 		GPIOD
#define servo8Pin 		GPIO_PIN_11
#define servo8Port 		GPIOD
#define servo9Pin 		GPIO_PIN_12
#define servo9Port 		GPIOD
#define servo10Pin 		GPIO_PIN_13
#define servo10Port 	GPIOD
#define servo11Pin 		GPIO_PIN_14
#define servo11Port 	GPIOD
#define servo12Pin 		GPIO_PIN_15
#define servo12Port 	GPIOD
#define servo13Pin 		GPIO_PIN_6
#define servo13Port 	GPIOC
#define servo14Pin 		GPIO_PIN_7
#define servo14Port 	GPIOC
#define servo15Pin 		GPIO_PIN_8
#define servo15Port 	GPIOC

uint16_t servoPinArray[] ={	servo1Pin, servo2Pin, servo3Pin,
														servo4Pin, servo5Pin, servo6Pin,
														servo7Pin, servo8Pin, servo9Pin,
														servo10Pin, servo11Pin, servo12Pin,
														servo13Pin, servo14Pin, servo15Pin};

GPIO_TypeDef * servoPortArray[] = {	servo1Port, servo2Port, servo3Port,
																		servo4Port, servo5Port, servo6Port,
																		servo7Port, servo8Port, servo9Port,
																		servo10Port, servo11Port, servo12Port,
																		servo13Port, servo14Port, servo15Port};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Debug(char *Array, int count);														//Sends something to UART1 for debugging
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);				//Sends error message with Debug()
void SetupTimers(void);																				//Configure timers 6 and 7 for use in servo control
void Sort(void);																							//Sorts timer values into ascending order
void SetServo(int servo, int degrees);												//Sets the timer value for a servo from a degree value
struct servos IK( struct coordinates input);									//Inverse Kinematics function
void CheckBounds(void);																				//Check if legs need to be reset
struct coordinates Rotate(double x, double y, double angle);	//Rotate coordinates about an angle
void TranslateLeg(void);																			//Generates constants at beginning of program
struct coordinates Vector(double x, double y, int leg, bool offset);
void StartPosition(void);																			//Resets robot to default position

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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
	SetupTimers();
	StartPosition();

	TranslateLeg();

	Debug("Init done",9);
	//Turn off all LEDs except green
	HAL_GPIO_WritePin(BTPort,BTLEDPin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(greenLEDPort,greenLEDPin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(redLEDPort,redLEDPin,GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	Debug("Entering main loop",18);
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);					//Turn blue LED off
		HAL_UART_Receive_IT(&huart3,RXData,15);											//Receive commands via bluetooth
		for(int i = 0;i<7;i++)
		{
			//Is this a valid message?
			if ((RXData[i] == 'X')&& (RXData[i+2] == 'Y')&&(RXData[i+4]=='R')&&(RXData[i+6]=='*'))
			{
				//Turn Blue LED on
				HAL_GPIO_WritePin(BTPort,BTLEDPin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(servoPowerPort,servoPowerPin,GPIO_PIN_SET);
				BTCount = 0;
				//Extract the necessary commands
				X_Char = RXData[i+1];
				Y_Char = RXData[i+3];
				R_Char = RXData[i+5];
				//Clear the array
				for(int j = 0;j<14;j++)
				{
					RXData[j]=0;
				}
				//Calculate correct vectors
				X_Vect = X_Char - '5';
				Y_Vect = Y_Char - '5';
				R_Vect = R_Char - '5';
				//Debug
				HAL_UART_Transmit(&huart1,&X_Char,1,10);
				HAL_UART_Transmit(&huart1,&Y_Char,1,10);
				HAL_UART_Transmit(&huart1,&R_Char,1,10);
				HAL_UART_Transmit(&huart1,BTRewrite,4,10);
				}
		}
		
		//Toggle pin to check for stuck program
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1); 
		
		
		//Do leg stuff
		struct coordinates leg1 = Vector(-X_Vect*10,-Y_Vect*10,1,true);
		struct servos leg1_servo = IK(leg1);
		SetServo(1,leg1_servo.theta);
		SetServo(6,leg1_servo.phi);
		SetServo(11,leg1_servo.alpha);
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
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

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC6 PC7 PC8 
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB12 PB13 PB14 
                           PB15 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 
                           PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  This function sends a string to UART1 to be received in a terminal
	* @param  *Array is a pointer to the string
						count is an int equal to the number of characters in the string
  * @retval None
  */
void Debug(char *Array, int count)
{
	uint8_t TXBuffer[count+1];
	for(int i = 0; i!=count;i++)
	{
		TXBuffer[i] = Array[i];
	}		
	
	TXBuffer[count] ='\r';
	TXBuffer[count+1] ='\n';
	HAL_UART_Transmit(&huart1,TXBuffer,count+2,10);
}

/**
  * @brief  This function is executed in the case of an error with the UART modules
	* @param  *huart is a handle of the UART module in question
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	Debug("Bluetooth Error",15);
}

/**
  * @brief  This function configures the timers used in generating the servo signals
	* @param  None
  * @retval None
  */
void SetupTimers(void)
{
	//Timer 6
	//50Hz Interrupt
	ServoTimer.Instance =TIM6;
	ServoTimer.Init.Prescaler =40000;
	ServoTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
	ServoTimer.Init.Period =54;
	ServoTimer.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&ServoTimer);
	HAL_TIM_Base_Start_IT(&ServoTimer);
	
	//Timer 7
	//Variable frequency interrupt
	htim7.Instance =TIM7;
	htim7.Init.Prescaler =1000;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period =160;
	htim7.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&htim7);
	HAL_TIM_Base_Start_IT(&htim7);
}
	

/** @brief Function for sorting the desired leg angles into ascending order 
						to make timer functions simpler
  * @param None
  * @retval None */
void Sort(void)
{
	for(int i = 0;i<15;i++)
	{
		for(int j = i+1;j<15;j++)
		{
			if(legAngles[i] > legAngles[j])
			{
				//swop the angles
				int a = legAngles[i];
				legAngles[i] = legAngles[j];
				legAngles[j] = a;
				//now swop the index
				a = legAngles_id[i];
				legAngles_id[i] = legAngles_id[j];
				legAngles_id[j] = a;
			}
		}
	}
}

/**
  * @brief  This function executes when a timer interrupts. It is used to generate servo signals
	* @param  *htim is a handle of the timer module that triggered the interrupt
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		
		if(htim == &htim7){
			//Turn off
			++servoCount;
			
			if (servoCount < 15)
			{
				newPeriod = (legAngles[servoCount]- legAngles[servoCount-1]);
				
				while (newPeriod == 0)
				{
					HAL_GPIO_WritePin(servoPortArray[legAngles_id[servoCount-1]-1] ,servoPinArray[legAngles_id[servoCount-1]-1],GPIO_PIN_RESET);
					++servoCount;
					newPeriod = (legAngles[servoCount]- legAngles[servoCount-1]);
				}
				//TIM7->CNT = 0;
				if (newPeriod>1)
				{
					--newPeriod;
				}
				TIM7->ARR = (newPeriod);
				HAL_GPIO_WritePin(servoPortArray[legAngles_id[servoCount-1]-1] ,servoPinArray[legAngles_id[servoCount-1]-1],GPIO_PIN_RESET);
			}else{
				//Turn off servo 15
				HAL_GPIO_WritePin(servoPortArray[legAngles_id[14]-1] ,servoPinArray[legAngles_id[14]-1],GPIO_PIN_RESET);
				//Stop the timer for the rest of the cycle
				HAL_TIM_Base_Stop(&htim7);
				servoCount = 0;
			}
		}
		if (htim == &htim6)
		{
			//50Hz Interrupt
			//HAL_GPIO_TogglePin(redLEDPort,redLEDPin);
			//Turn all on
			HAL_GPIO_WritePin(servo1Port,servo1Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo2Port,servo2Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo3Port,servo3Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo4Port,servo4Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo5Port,servo5Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo6Port,servo6Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo7Port,servo7Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo8Port,servo8Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo9Port,servo9Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo10Port,servo10Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo11Port,servo11Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo12Port,servo12Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo13Port,servo13Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo14Port,servo14Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(servo15Port,servo15Pin,GPIO_PIN_SET);
			
			//Sort the servo times in ascending order
			Sort();
			//Set period for 1st servo
			newPeriod = legAngles[servoCount];
			while (newPeriod == 0)
			{
				++servoCount;
				newPeriod = legAngles[servoCount];
			}
				
			TIM7->ARR = newPeriod;
			//Start the timer
			HAL_TIM_Base_Start(&htim7);
			
			//BT LED
			++BTCount;
			if (BTCount > 120) //20ms * 120 = 2.4s
			{
				//Turn LED off
				HAL_GPIO_WritePin(BTPort,BTLEDPin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(servoPowerPort,servoPowerPin,GPIO_PIN_RESET);
				BTCount = 0;
				HAL_GPIO_TogglePin(greenLEDPort,greenLEDPin);
				HAL_GPIO_TogglePin(redLEDPort,redLEDPin);
			}
		}
	}


/**
  * @brief  This function calculates the required timer values for each of the servos
	* @param  servo is an int in the range 1:15 corresponding to a specific joint
						degrees is an int in the range 0:180 for the required position of the specific servo
  * @retval None
  */	
void SetServo(int servo, int degrees)
{
	//Set the servo with the correct index in the array 
	for (int i = 0; i<15; i++)
	{
		if (servo == legAngles_id[i])
		{
			legAngles[i] =servoOffset[servo-1] + (degrees*(servoMultiplier[servo-1])/180);
		}
	}
}

/**
  * @brief  This function does the inverse kinematic calculations for a specific leg
	* @param  input is a struct of type coordinates with members x, y and z
							corresponding to the required position of the foot relative to the hip.
  * @retval servos is a struct with members theta, phi and alpha corresponding with the 
							resulting angles required.
  */
struct servos IK( struct coordinates input)
{
	//make the return struct
	struct servos result;
	//Calculate theta
	double theta = atan2(input.x,input.y);
	//Calculate coordinates of joint 2
	double x = A_length * sin(theta);
	double y = A_length * cos(theta);
	//horizontal distance from joint 1 to foot
	double reach = sqrt(pow(input.x-x,2)+pow(input.y-y,2));
	//Absolute distance between joint 2 and foot
	double twoToFoot = sqrt(pow(input.x-x,2)+pow(input.y-y,2) +pow(input.z-z_body,2));
	//perform Cosine rule to get alpha
	double f =((pow(B_length ,2) + pow(C_length,2) - pow(twoToFoot ,2))/(2*B_length*C_length));
	double d = acos(f);
	double alpha =(rad2deg * d) + 0.5;		//+0.5 for rounding
	result.alpha = (int)alpha;
	//use sine rule to get phi
	double c = asin(C_length * sin(d) / twoToFoot);
	double e = atan2(reach ,z_body-input.z);
	double phi = 270 - c*rad2deg - e*rad2deg + 0.5; // +0.5 for rounding
	result.phi = (int)phi;
	//convert theta to degrees. Add 0.5 for rounding
	theta = (theta * rad2deg) + 0.5;
	result.theta = (int)theta;
	
	return result;
}

/**
  * @brief  This function checks that all legs are within the boundaries specified and
							writes the results in a global array
	* @param  None
  * @retval None
  */
void CheckBounds(void)
{
	for(int i = 0 ; i<5 ; i++)
	{
		//
		if ( legAngles[i+1] < 50 || legAngles[i+1] > 120		// 50 < theta < 120
			|| legAngles[i+6] < 90 || legAngles[i+6] > 170		// 90 < phi   < 170
			|| legAngles[i+11]< 70 || legAngles[i+11]> 130)		// 70 < alpha < 130
		{
			resetStatus[i] = true;				
		}else{
			resetStatus[i] = false;
		}
	}			
}

/**
  * @brief  This function rotates the given coordinates about the origin through a given angle 
	* @param  x,y are doubles corresponding to the coordinates
						angle is a double in degrees
  * @retval a coordinates struct with members x, y and z. z is not used in this case.
  */
struct coordinates Rotate(double x, double y, double angle)
{
	//Create output struct
	struct coordinates result;
	//convert angle to radians
	angle = angle/rad2deg;
	//calculate return values
	result.x = x * cos(angle) - y * sin(angle);
	result.y = x * sin(angle) + y * cos(angle);
	
	return result;
}

/**
  * @brief  This function calculates the normalized position of a leg from the 
							given movement vectors
	* @param	x is a double with the X vector
						y iss a double with the Y vector
						leg is an int in the range 1:5
						offset is a bool indicating whether the coordinates should be
							denormalized
  * @retval a struct of type coordinates. The z member is unused
  */
struct coordinates Vector(double x, double y, int leg, bool offset)
{
	//Create return struct
	struct coordinates result;
	//Make the trajectory polar with a magnitude and angle
	double trajectory = atan2(y,x);
	double magnitude = sqrt(pow(x,2)+pow(y,2));
	//Match the trajectory to the leg
	struct coordinates neutral;
	//Translation components
	if (offset)
	{
		neutral = Rotate(270,0,(leg-1)*72);
		struct coordinates offset= Rotate(robotRadius,0,(leg-1)*72);
		neutral.x = neutral.x - offset.x;
		neutral.y = neutral.y - offset.y;
	}else{
		neutral.x = 0;
		neutral.y = 0;
	}
	result.x = neutral.x + magnitude*cos(trajectory) + translateLeg[0][leg-1];
	result.y = neutral.y + magnitude*sin(trajectory) + translateLeg[1][leg-1];
	//+ Rotation
	struct coordinates temp = Rotate(result.x,result.y,-2*R_Vector);
	result.x = temp.x - translateLeg[0][leg-1];
	result.y = temp.y - translateLeg[1][leg-1];
	if (offset)
	{
		destination[0][leg-1] = result.x;
		destination[1][leg-1] = result.y;
	}
	return result;
}

/**
  * @brief  This function 
	* @param  None
  * @retval None
  */	
void TranslateLeg(void)
{
	for(int leg = 0; leg < 5; leg++)
	{
		translateLeg[0][leg] = robotRadius * cos(leg*72/rad2deg);
		translateLeg[1][leg] = robotRadius * sin(leg*72/rad2deg);
	}
}

/**
  * @brief  This function resets a leg if it is out of bounds
	* @param  leg is an int in the range 1:5
  * @retval None
  */
void ResetLeg(int leg)
{
	//Lift up the leg where it is currently
	struct coordinates lift;
	lift.x = destination[0][leg-1];
	lift.y = destination[1][leg-1];
	lift.z = 30;
	struct servos leg_up = IK(lift);
	//Send this to servos and wait
	//Now move the leg over to the new position
	struct coordinates reset = Vector(X_Vector,Y_Vector,leg,true);
	reset.z = 30;
	struct servos leg_reset = IK(reset);
	//Send this to servos and wait
	//Now put down the leg where it is
	reset.z = 0;
	leg_reset = IK(reset);
	//Send this to the servos and wait
}

void StartPosition(void)
{
	//Set servo begin positions
	SetServo(1,90);
	SetServo(2,90);
	SetServo(3,90);
	SetServo(4,90);
	SetServo(5,90);
	SetServo(6,100);
	SetServo(7,100);
	SetServo(8,100);
	SetServo(9,100);
	SetServo(10,100);
	SetServo(11,90);
	SetServo(12,90);
	SetServo(13,90);
	SetServo(14,90);
	SetServo(15,90);
}
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
