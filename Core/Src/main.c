/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <oled.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId robotCommandTasHandle;
osThreadId leftEncoderTaskHandle;
osThreadId rightEncoderTasHandle;
osThreadId stopBotTaskHandle;
osThreadId motorTaskHandle;
osThreadId OLEDTaskHandle;
osThreadId gyroTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void const * argument);
void robotCommand(void const * argument);
void leftEncoder(void const * argument);
void rightEncoder(void const * argument);
void stopBot(void const * argument);
void motor(void const * argument);
void OLEDShow(void const * argument);
void gyroTask1(void const * argument);

/* USER CODE BEGIN PFP */
void stopMovement(void);
void moveFrontStraight(void);
void moveBackStraight(void);
void moveForward(char dir[10], int distance);
void moveBackward(char dir[10], int distance);
float Left_PID_control (float setpoint, float measure);
float Right_PID_control (float setpoint, float measure);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[20];
uint16_t leftIR[4096]; // PC2 is left IR
uint16_t rightIR[4096]; // PC1 is right IR
int rightEncoderVal = 0, leftEncoderVal = 0;
uint16_t rightPWMval = 0, leftPWMval = 0;
int motorDir = 00;
int angle = 90;
int totalLeftEncoder=0;
int totalRightEncoder=0;
uint8_t buff[20];

//PID variables
float Kp=7, Ki=10, Kd=0.01;
float T=5; /*Sample Period*/
float ruk, ruk_1=0, rek_1=0, rek_2=0, rek=0;
float luk, luk_1=0, lek_1=0, lek_2=0, lek=0;
float umax=3500;/*Constant*/
float umin=1500; /*Constant*/
char temp[6] = "test\0";
uint8_t ICMAddr = 0x68;

double Aint = 0;

uint32_t Difference = 0;
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

double total_angle=0;
double angle_offset=0;

void readByte(uint8_t addr, uint8_t* data){
	buff[0] = addr;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr<<1, buff, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, ICMAddr<<1, data, 2, 20);

}

void writeByte(uint8_t addr, uint8_t data){
	buff[0] = addr;
	buff[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr << 1, buff, 2, 20);
}

void correctDirection(double target_angle, int dir){

	int pidVal;

	// calibrate these for PID and gyro
	pidVal = (int)(147 + (dir*(total_angle - target_angle)*2.0 + 0.0007*Aint));
	if(pidVal <= 100){
		pidVal = 100;
	}

	if(pidVal >= 200){
		pidVal = 200;
	}
	Aint += dir*(total_angle - target_angle);
	htim1.Instance -> CCR4 = pidVal;

}

void gyroInit(){

	writeByte(0x06, 0x00);
	osDelay(10);
	writeByte(0x03, 0x80);
	osDelay(10);
	writeByte(0x07, 0x07);
	osDelay(10);
	writeByte(0x06, 0x01);
	osDelay(10);
	writeByte(0x7F, 0x20);
	osDelay(10);
	writeByte(0x01, 0x2F);
	osDelay(10);
	writeByte(0x0, 0x00);
	osDelay(10);
	writeByte(0x7F, 0x00);
	osDelay(10);
	writeByte(0x07, 0x00);
	osDelay(10);

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
	MX_TIM8_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */
	OLED_Init();
	HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer, 5);
	/* USER CODE END 2 */

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
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of robotCommandTas */
	osThreadDef(robotCommandTas, robotCommand, osPriorityIdle, 0, 128);
	robotCommandTasHandle = osThreadCreate(osThread(robotCommandTas), NULL);

	/* definition and creation of leftEncoderTask */
	osThreadDef(leftEncoderTask, leftEncoder, osPriorityIdle, 0, 128);
	leftEncoderTaskHandle = osThreadCreate(osThread(leftEncoderTask), NULL);

	/* definition and creation of rightEncoderTas */
	osThreadDef(rightEncoderTas, rightEncoder, osPriorityIdle, 0, 128);
	rightEncoderTasHandle = osThreadCreate(osThread(rightEncoderTas), NULL);

	/* definition and creation of stopBotTask */
	osThreadDef(stopBotTask, stopBot, osPriorityIdle, 0, 128);
	stopBotTaskHandle = osThreadCreate(osThread(stopBotTask), NULL);

	/* definition and creation of motorTask */
	osThreadDef(motorTask, motor, osPriorityIdle, 0, 128);
	motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

	/* definition and creation of OLEDTask */
	osThreadDef(OLEDTask, OLEDShow, osPriorityIdle, 0, 128);
	OLEDTaskHandle = osThreadCreate(osThread(OLEDTask), NULL);

	/* definition and creation of gyroTask */
	osThreadDef(gyroTask, gyroTask1, osPriorityIdle, 0, 128);
	gyroTaskHandle = osThreadCreate(osThread(gyroTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
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

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 160;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 16;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 7199;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
			|TRIG_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin|MotorA_IN1_Pin|MotorB_IN1_Pin|MotorB_IN2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin */
	GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : MotorA_IN2_Pin MotorA_IN1_Pin MotorB_IN1_Pin MotorB_IN2_Pin */
	GPIO_InitStruct.Pin = MotorA_IN2_Pin|MotorA_IN1_Pin|MotorB_IN1_Pin|MotorB_IN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : TRIG_Pin */
	GPIO_InitStruct.Pin = TRIG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* to prevent unused argument(s) compilation warning */
	UNUSED(huart);
	//	HAL_UART_Transmit(&huart3, (uint8_t *) aRxBuffer, 10, 0xFFFF);
	//motorDir = aRxBuffer[0];
	sprintf(temp,"%s\0",aRxBuffer);
	int value = atoi(temp);
	motorDir = value/1000;
	angle = value % 1000;
	if (!motorDir){
		stopMovement();
		HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
	}
	HAL_UART_Receive_IT (&huart3, aRxBuffer, 5);
}

void delay(uint16_t duration){

	__HAL_TIM_SET_COUNTER(&htim4,0);
	while(__HAL_TIM_GET_COUNTER(&htim4) < duration);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Distance+=4;  //ultrasonic sensor is off by 4cm
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim4,TIM_IT_CC1);
		}
	}
}

void ultrasonic_read(void){
	//code for ultrasound
	HAL_GPIO_WritePin(GPIOE, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	osDelay(10);  // wait for 10 us
	HAL_GPIO_WritePin(GPIOE, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
}



void stopMovement(void){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	//	htim1.Instance->CCR4 = 146;
	//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	//forward - Motor A
	HAL_GPIO_WritePin(GPIOA,MotorA_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorA_IN2_Pin,GPIO_PIN_RESET); // low

	//forward - Motor B
	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin,GPIO_PIN_RESET); // low
}

void moveForward(char dir[10], int distance){
	stopMovement();
	osDelay(100);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	if(strcmp(dir, "Left") == 0)
		htim1.Instance->CCR4 = 101;
	else if(strcmp(dir, "Right") == 0)
		htim1.Instance->CCR4 = 220;
	else
		htim1.Instance->CCR4 = 147;

	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);

	// forward left and right wheels CLEARED CHECKLIST @ left: 2650; right: 2000
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500); //left wheel was 1650
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500); //right wheel was 1400

	//	osDelay(2250);   //latest
	// forward movement delay CLEARED CHECKLIST @ 2725/100
	int t = 0;
	//HAL_GetTick()
	uint32_t PreviousWakeTime = osKernelSysTick();
	int tick = HAL_GetTick();
	uint32_t delay = angle * (12800/10);
	//original value before PID: (2875/100) ; 5th Oct 4pm
	//prev value (12500/10) ; 6th Oct 1pm

	totalLeftEncoder = 0;
	total_angle = 0;

	Aint = 0;


	// for forward movement, adjust the denominator (1500) to be more if the distance traversed is not enough
	while(totalLeftEncoder/1300.0 * 21.04 < distance){


		correctDirection(0, 1);
		osDelay(1);

	}
	stopMovement();
	motorDir = 0;
}

void moveBackward(char dir[10], int distance){
	uint32_t delay = 0;
	stopMovement();
	osDelay(100);
	uint32_t target_angle = 90;
	total_angle = 0;

	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	if(strcmp(dir, "Left") == 0){
		htim1.Instance->CCR4 = 101;

		HAL_GPIO_WritePin(GPIOA,MotorA_IN1_Pin, GPIO_PIN_RESET);  // high
		HAL_GPIO_WritePin(GPIOA,MotorA_IN2_Pin, GPIO_PIN_SET); // low

		// back
		HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_RESET);  // high
		HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_SET); // low

		osDelay(250);

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500); //motor A; 1000 at 14 oct 245am
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500); //motor B;

		osDelay(10);
		while(total_angle*-1  <= target_angle - 3){//was 3
			osDelay(10);
		}

		stopMovement();

		osDelay(10);
		stopMovement();
		htim1.Instance->CCR4 = 147;
		osDelay(200);
		moveForward("Straight", 7);
		osDelay(200);
		stopMovement();
		motorDir = 0;
	}
	else if(strcmp(dir, "Right") == 0){
		htim1.Instance->CCR4 = 220;

		HAL_GPIO_WritePin(GPIOA,MotorA_IN1_Pin, GPIO_PIN_RESET);  // high
		HAL_GPIO_WritePin(GPIOA,MotorA_IN2_Pin, GPIO_PIN_SET); // low

		//forward
		HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_RESET);  // high
		HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_SET); // low

		//back
		//	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_RESET);  // low
		//	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_SET); // high

		osDelay(250);

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500); //motor A
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500); //motor B; 1000 at 14 oct 245am

		osDelay(10);
		while(total_angle  <= target_angle + 1){
			osDelay(10);
		}

		stopMovement();

		osDelay(10);
		stopMovement();
		htim1.Instance->CCR4 = 147;
		osDelay(200);
		moveForward("Straight", 7);
		osDelay(200);
		stopMovement();
		motorDir = 0;

	}
	else{
		htim1.Instance->CCR4 = 147;
		delay = angle * (2600/100);// was 2510 for 100 (not accurate) // 251 for 10cm slight short

		// backward left and right wheels CLEARED CHECKLIST @ left: 2550; right: 2250

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);// was 2550
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);// was 2250

		totalLeftEncoder = 0;
		total_angle = 0;

		Aint = 0;

		while(totalLeftEncoder/1500.0 * -21.04 < distance){

			correctDirection(0, -1);
			//osDelayUntil(&PreviousWakeTime,5);
			osDelay(1);

		}
		stopMovement();
		motorDir = 0;
	}

}

void motorRight(int target_angle){

	total_angle = 0;
	//target_angle = angle;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = 220;

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorA_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorA_IN2_Pin, GPIO_PIN_RESET); // low

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_RESET); // low

	//back
	//	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_RESET);  // low
	//	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_SET); // high

	osDelay(250);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500); //motor A; 1000 at 14 oct 245am
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500); //motor B;

	osDelay(10);
	total_angle = total_angle * (44/45);
	while(total_angle*-1  + 2<= target_angle){
		osDelay(10);
	}

	stopMovement();

	double Aint = 0;

	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

	osDelay(10);

	osDelay(10);
	stopMovement();
	htim1.Instance->CCR4 = 147;
	motorDir = 0;

	osDelay(1000);

}
void motorLeft(int target_angle){

	total_angle = 0;
	//target_angle = angle;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = 101;

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorA_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorA_IN2_Pin, GPIO_PIN_RESET); // low

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_RESET); // low

	//back
	//	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_RESET);  // low
	//	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_SET); // high

	osDelay(250);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500); //motor A
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500); //motor B; 1000 at 14 oct 245am

	osDelay(10);

	total_angle = total_angle - (total_angle / 45);
	while(total_angle  <= target_angle - 2){
		osDelay(10);
	}

	stopMovement();

	double Aint = 0;

	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

	osDelay(10);

	osDelay(10);
	stopMovement();
	htim1.Instance->CCR4 = 147;

	motorDir = 0;

	osDelay(1000);
}

float Left_PID_control (float setpoint, float measure)
{
	lek_2=lek_1;
	lek_1=lek;
	lek=setpoint-measure;
	luk_1=luk;
	luk=luk_1+Kp*(lek-lek_1) +Ki*(T/2)*(lek+lek_1)+ (Kd/T)*(lek-2*lek_1+lek_2);
	//	if (luk>umax) luk=umax;
	//	if (luk<umin) luk=umin;
	return (luk);
}

float Right_PID_control (float setpoint, float measure)
{
	rek_2=rek_1;
	rek_1=rek;
	rek=setpoint-measure;
	ruk_1=ruk;
	ruk=ruk_1+Kp*(rek-rek_1) +Ki*(T/2)*(rek+rek_1)+ (Kd/T)*(rek-2*rek_1+rek_2);
	//	if (ruk>umax) ruk=umax;
	//	if (ruk<umin) ruk=umin;
	return (ruk);
}

// turns on motor in fwd direction
void motorForward(int pwm){
	total_angle = 0;
	// these 4 lines control direction of the motors
	// HAL is a system library that has a function "GPIO write pin"
	// GPIOA is the port (port A)
	// MotorA_IN_Pin is the pin number (PA3)
	// set is 1, reset is 0
	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);

	//
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,  pwm + 50);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);

}

void motorReverse(int pwm){
	total_angle = 0;
	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	HAL_ADC_Start_DMA(&hadc1, rightIR, 4096); // PC1 is right IR
	HAL_ADC_Start_DMA(&hadc2,  leftIR, 4096); // PC2 is left IR
	htim1.Instance->CCR4 = 147;
	for(;;)
	{
		ultrasonic_read();
		osDelay(200);
	}
	/* USER CODE END 5 */
}
void thirtyCMFromObstacle(){
	gyroInit();
	osDelay(1000);
	motorForward(1300); // 1500 too fast; 13th Oct 6pm
	while(Distance > 30){
		correctDirection(0, 1);
		osDelay(1);
	}
	stopMovement();
	motorReverse(1000);
	while(Distance < 30){
		osDelay(1);
	}
	stopMovement();
	motorDir = 0;
}

void twentyCMFromObstacle(){
	gyroInit();
	osDelay(1000);
	motorForward(1300); // 1500 too fast; 13th Oct 6pm
	while(Distance > 20){
		correctDirection(0, 1);
		osDelay(1);
	}
	stopMovement();
	motorReverse(1000);
	while(Distance < 20){
		osDelay(1);
	}
	stopMovement();
	motorDir = 0;
}

/* USER CODE BEGIN Header_robotCommand */
/**
 * @brief Function implementing the robotCommandTas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_robotCommand */

int obs_flag=0;
int distance_between_obs1_obs2 = 0;
int distance_counter = 3;
int total_distance_travelled = 0;
void robotCommand(void const * argument)
{
	/* USER CODE BEGIN robotCommand */
	/* Infinite loop */
	/*
	 * 5bit Instruction
	 *  |--------------------------------------------------------------------------------|
	 *  |Instruction		|			Action											 |
	 *  |--------------------------------------------------------------------------------|
	 *  |  00xxx			|	Stop Movement											 |
	 *  |  01xxx			|	Move forward for xxx distance (Straight Line)			 |
	 *  |  02xxx			|	Turn Left for xxx angle (forward)						 |
	 *  |  03xxx			|	Turn Right for xxx angle (forward)						 |
	 *  |  11xxx			|	Move backward for xxx distance (Straight Line)			 |
	 *  |  12xxx			|	Turn Left for xxx angle (Backward)						 |
	 *  |  13xxx			|	Turn Right for xxx angle (Backward)						 |
	 *  |--------------------------------------------------------------------------------|
	 *
	 */

	int turnDegree=0;
	double target_angle;
	double Aint;


	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);

	for(;;){
		target_angle = 0;
		switch(motorDir){
		case 0:
			stopMovement();
			break;
		case 1:

			//move forward
			//				HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
			//				HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
			//				HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
			//				HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);
			//
			//				// forward left and right wheels CLEARED CHECKLIST @ left: 2650; right: 2000
			//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000); //left wheel was 1650
			//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000); //right wheel was 1400


			Aint = 0;

			gyroInit();
			osDelay(1000);

			turnDegree = 0;
			moveForward("Straight", angle);
			HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
			break;
		case 2:

			Aint = 0;
			gyroInit();
			osDelay(1000);

			//motorLeft(angle);

			motorLeft(angle);
			osDelay(1);
			//				moveBackward("Straight", 10);
			//				osDelay(1);
			//				motorLeft(40);
			//				moveBackward("Straight", 10);

			HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
			break;

		case 3: // turn right

			Aint = 0;
			gyroInit();
			osDelay(1000);

			//motorRight(angle);

			motorRight(angle);
			osDelay(1);
			//				moveBackward("Straight", 20);
			//				osDelay(1);
			//				motorRight(35);
			//				moveBackward("Straight", 7);
			HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
			break;

		case 4: // go fwd, stop 30cm from obstacle

			Aint = 0;
			gyroInit();
			osDelay(1000);

			motorForward(1300); // 1500 too fast; 13th Oct 6pm
			while(Distance > 30){
				correctDirection(0, 1);
				osDelay(1);
			}
			stopMovement();


			motorReverse(1000);
			while(Distance < 30){
				osDelay(1);
			}
			stopMovement();
			motorDir = 0;
			//HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);

			break;

		case 5: // go around the first obstacle from the left side
			if(obs_flag==0){
				Aint = 0;
				gyroInit();
				osDelay(1000);
				total_angle = 0;

				motorLeft(43); // was 45
				total_angle = 0;
				moveForward("Straight", 8);
				total_angle = 0;
				osDelay(500);

				motorRight(45); //was 45, 46
				total_angle = 0;

				moveForward("Straight", 5);

				motorRight(45);
				total_angle = 0;

				moveForward("Straight", 8); // was 15, 17, 20, 19, 18 ,10
				total_angle = 0;

				motorLeft(45); // was 45, 43, 41, 42, 43, 41, 43, 42, 43
				total_angle = 0;

				moveBackward("Straight", 10);
				total_angle = 0;

				osDelay(2000);
				distance_between_obs1_obs2=Distance;

				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
				obs_flag=1;
				osDelay(1000);

				Aint = 0;
				gyroInit();
				//osDelay(1000);

				//				motorForward(1300); // 1500 too fast; 13th Oct 6pm
				while (Distance > 30) {
					moveForward("Straight", 10);
					distance_counter++;
					correctDirection(0, 1);
					osDelay(1);
				}
				total_distance_travelled = distance_counter * 10;
				//				while(Distance > 30){
				//					correctDirection(0, 1);
				//					osDelay(1);
				//				}
				total_angle = 0;
				stopMovement();


				motorReverse(1300);
				while(Distance < 30){
					osDelay(1);
				}
				stopMovement();
				total_angle = 0;
				motorDir = 0;

				break;
			}
			else{
				//Go left around second obstacle
				Aint = 0;
				gyroInit();
				osDelay(1000);
				total_angle = 0;

				motorLeft(90); // was 90
				total_angle = 0;

				//				moveForward("Straight", 5);
				//				stopMovement();
				//				total_angle = 0;

				motorRight(90);
				total_angle = 0;

				moveForward("Straight", 5); // was 10
				total_angle = 0;

				motorRight(90); // was 90
				total_angle = 0;
				stopMovement();
				moveForward("Straight", 53); // was 70, 55, 57
				motorRight(90);
				total_angle = 0;
				//Robot at the edge of second obstacle
				if (total_distance_travelled > distance_between_obs1_obs2){
					distance_between_obs1_obs2 = total_distance_travelled;
				}
				moveForward("Straight", distance_between_obs1_obs2 + 40 + 35); // was 40 + 50, 40 + 45, was 40 + 40
				total_angle = 0;

				motorRight(90);
				total_angle = 0;
				moveBackward("Straight", 3); // was 5
				motorLeft(90);
				total_angle = 0;
				//Robot at the center of obstacle 2
				twentyCMFromObstacle();

			}
			break;

		case 6: // go around the first obstacle from the right side
			if(obs_flag==0){
				Aint = 0;
				gyroInit();
				osDelay(1000);
				total_angle = 0;

				motorRight(45);
				total_angle = 0;
				moveForward("Straight", 10); // was 15, was 8, was 9
				total_angle = 0;
				osDelay(500);

				motorLeft(45); // was 45, 47, 46, 45, 43. 41
				total_angle = 0;

				moveForward("Straight", 5); // was 3
				total_angle = 0;

				motorLeft(45); // was 43
				total_angle = 0;

				moveForward("Straight", 8); // was 18, 16, 13
				total_angle = 0;

				motorRight(48); // was 45, 47, 49, 46, 45, 47
				total_angle = 0;

				moveBackward("Straight", 10);
				total_angle = 0;

				osDelay(2000);
				distance_between_obs1_obs2=Distance;

				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
				obs_flag=1;
				osDelay(1000);

				Aint = 0;
				gyroInit();
				//osDelay(1000);

				//				motorForward(1300); // 1500 too fast; 13th Oct 6pm
				//				while(Distance > 30){
				//					correctDirection(0, 1);
				//					osDelay(1);
				//				}
				while (Distance > 30) {
					moveForward("Straight", 10);
					distance_counter++;
					correctDirection(0, 1);
					osDelay(1);
				}
				total_distance_travelled = distance_counter * 10;
				total_angle = 0;
				stopMovement();


				motorReverse(1300);
				while(Distance < 30){
					osDelay(1);
				}
				stopMovement();
				total_angle = 0;
				motorDir = 0;
				break;

			}
			else if(obs_flag==1){
				// go right around second obstacle
				Aint = 0;
				gyroInit();
				osDelay(1000);
				total_angle = 0;

				motorRight(90);
				total_angle = 0;

				//				moveForward("Straight", 5);
				//				stopMovement();
				//				total_angle = 0;

				motorLeft(88); // was 90
				total_angle = 0;

				moveForward("Straight", 5); // was 10
				total_angle = 0;

				motorLeft(88); // was 90
				total_angle = 0;
				stopMovement();
				moveForward("Straight", 62); // was 70, 63, 55, 60
				total_angle = 0;
				motorLeft(88); // was 90
				total_angle = 0;
				//Robot at the edge of second obstacle
				if (total_distance_travelled > distance_between_obs1_obs2){
					distance_between_obs1_obs2 = total_distance_travelled;
				}
				moveForward("Straight", distance_between_obs1_obs2 + 40 + 35); // was 40 + 50
				total_angle = 0;

				motorLeft(88); // was 90
				total_angle = 0;
				//				moveBackward("Straight", 5);
				motorRight(90);
				total_angle = 0;

				twentyCMFromObstacle();
			}

			break;

		case 11:
			Aint = 0;
			gyroInit();
			osDelay(1000);
			moveBackward("Straight", angle);
			HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
			break;
		case 12:
			Aint = 0;
			gyroInit();
			moveBackward("Left", angle);
			HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
			break;
		case 13:
			Aint = 0;
			gyroInit();
			moveBackward("Right", angle);
			HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
			break;
		case 20:
			NVIC_SystemReset();
			while(1);
			break;

		default:
			stopMovement();
			HAL_UART_Transmit(&huart3, (uint8_t *) "Err", 5, 0xFFFF);
			break;
		}

		osDelay(1);
	}
	/* USER CODE END robotCommand */
}

/* USER CODE BEGIN Header_leftEncoder */
/**
 * @brief Function implementing the leftEncoderTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_leftEncoder */
void leftEncoder(void const * argument)
{
	/* USER CODE BEGIN leftEncoder */
	//OLED_Refresh_Gram();
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

	int cntL1, cntL2;

	int cnt2;
	int dirL = 1;
	int diff;
	uint32_t tick;
	cntL1 = __HAL_TIM_GET_COUNTER(&htim2);

	tick = HAL_GetTick();
	//uint8_t left[30] = "Test \0";
	/* Infinite loop */
	//	float speed = 0;
	//	int delta_count = 0;
	for(;;)
	{
		if(HAL_GetTick()-tick > 100L)
		{

			cnt2 = __HAL_TIM_GET_COUNTER(&htim2);

			if(cnt2 > 32000){
				dirL = 1;

				diff = (65536 - cnt2);


			} else {
				dirL = -1;




				diff = cnt2;

			}




			if(dirL == 1){
				totalLeftEncoder += diff;
			} else {
				totalLeftEncoder -= diff;
			}

			__HAL_TIM_SET_COUNTER(&htim2, 0);

			tick = HAL_GetTick();
		}
		osDelay(1);
	}
	/* USER CODE END leftEncoder */
}

/* USER CODE BEGIN Header_rightEncoder */
/**
 * @brief Function implementing the rightEncoderTas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_rightEncoder */
void rightEncoder(void const * argument)
{
	/* USER CODE BEGIN rightEncoder */
	//OLED_Refresh_Gram();

	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

	int cntR1, cntR2;
	uint32_t tick;
	cntR1 = __HAL_TIM_GET_COUNTER(&htim3);
	tick = HAL_GetTick();
	//uint8_t right[30] = "Test\0";
	/* Infinite loop */
	for(;;)
	{
		if(HAL_GetTick()-tick > 1000L)
		{
			cntR2 = __HAL_TIM_GET_COUNTER(&htim3);

			// for left encoder
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
			{
				if(cntR2<cntR1)
				{
					rightEncoderVal = cntR1 - cntR2;
				}
				else
					rightEncoderVal = (65535-cntR2) + cntR1;

			}
			else
			{
				if(cntR2>cntR1)
				{
					rightEncoderVal = cntR2 - cntR1;
				}
				else
					rightEncoderVal = (65535-cntR1) + cntR2;
			}
			//rightPWMval = Right_PID_control (1500, rightEncoderVal);

			cntR1 = __HAL_TIM_GET_COUNTER(&htim3);
			tick = HAL_GetTick();
		}
		osDelay(1);
	}
	/* USER CODE END rightEncoder */
}

/* USER CODE BEGIN Header_stopBot */
/**
 * @brief Function implementing the stopBotTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_stopBot */
void stopBot(void const * argument)
{
	/* USER CODE BEGIN stopBot */
	/* Infinite loop */
	for(;;)
	{
		if(motorDir == 0)
			stopMovement();
	}
	/* USER CODE END stopBot */
}

/* USER CODE BEGIN Header_motor */
/**
 * @brief Function implementing the motorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_motor */
void motor(void const * argument)
{
	/* USER CODE BEGIN motor */
	/* Infinite loop */
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	for(;;)
	{
		//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, rightPWMval);
		//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, leftPWMval);
		osDelay(1);
	}
	/* USER CODE END motor */
}

/* USER CODE BEGIN Header_OLEDShow */
/**
 * @brief Function implementing the OLEDTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_OLEDShow */
void OLEDShow(void const * argument)
{
	/* USER CODE BEGIN OLEDShow */
	/* Infinite loop */
	for(;;)
	{
		uint8_t oledText[70] = "oled \0";
		//sprintf(oledText,"Right is %5d, %5d \n Left is %5d, %5d \0",rightEncoderVal, rightPWMval, leftEncoderVal, leftPWMval);
		sprintf(oledText,"angle: %5d", (int)total_angle);
		OLED_ShowString(10,10, oledText);
		sprintf(oledText, "distance: %5d", (int)Distance);
		OLED_ShowString(10,20, oledText);
		sprintf(oledText, "IR %4d %4d", (int)leftIR[0], (int)rightIR[0]);
		OLED_ShowString(10,30, oledText);
		OLED_Refresh_Gram();
		osDelay(200);
	}
	/* USER CODE END OLEDShow */
}


/* USER CODE BEGIN Header_gyroTask1 */
/**
 * @brief Function implementing the gyroTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_gyroTask1 */

void gyroTask1(void const * argument)
{
	/* USER CODE BEGIN gyroTask1 */
	/* Infinite loop */
	uint8_t val[2] = {0,0};

	char hello[20];
	int16_t angular_speed = 0;

	uint32_t tick = 0;
	gyroInit();
	int dir;
	//	int16_t offset = 0;

	tick = HAL_GetTick();
	osDelay(5);

	//		for(int i = 0; i < 3000; i++){
	//			readByte(0x37, val);
	//			angular_speed = val[0] << 8 | val[1];
	//			offset += angular_speed;
	//			//osDelayUntil(1);
	//		}
	//		offset = offset/3000.0f;

	for(;;)
	{
		osDelay(5);

		if(HAL_GetTick() - tick >= 100){
			readByte(0x37, val);
			//osDelayUntil(1);
			//angular_speed = ((int16_t)((int8_t)val[0]))*256 +  (uint16_t)val[1];
			angular_speed = (val[0] << 8) | val[1];


			// for gyro drift removal, calibration needs to be added to angular speed
			total_angle +=(double)(angular_speed - 4.7)*((HAL_GetTick() - tick)/16400.0);


			//prevSpeed = angular_speed;
			if(total_angle >= 720){
				total_angle = 0;
			}
			if(total_angle <= -720){
				total_angle = 0;
			}


			tick = HAL_GetTick();
		}
	}
	/* USER CODE END gyroTask1 */
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
