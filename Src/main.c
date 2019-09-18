/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sd_hal_mpu6050.h"
#define INTERVAL 3000 //interval ms
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
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

typedef struct{

	uint8_t mod;
	uint8_t armco;
	uint16_t desired_angle;
	uint16_t valueJoyStick_X_1;
	uint16_t valueJoyStick_Y_1;
	uint16_t valueJoyStick_X_2;
	uint16_t valueJoyStick_Y_2;

}Master;

Master control;
SD_MPU6050 mpu1 = {0};
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

uint16_t on_deger, arka_deger, onsa_deger, onso_deger, arsa_deger, arso_deger;

uint32_t last_time;

uint16_t adc_buffer[2];
uint8_t pin;
uint8_t armed = 1;
uint16_t i=0;
uint16_t pwmLeft = 1500;
uint16_t pwmRight = 1500;

char in[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Battery();
void uartPrintln();
void uartPrint();
void arming();
uint16_t parseData(uint8_t bit8data);

float gyroXoffset, gyroYoffset, gyroZoffset;

float elapsedTime, time, timePrev;
float rad_to_deg = 180/3.141592654;

float PID = 0;
float error = 0;
float previous_error = 0;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

double kp=3.55;//3.55
double ki=0.03;//0.003
double kd=2.05;//2.05

float desired_angle = 0;
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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

    last_time = HAL_GetTick(); // this give us the current

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim3);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 2);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);


    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;


    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }

    TxHeader.StdId = 0x321;
    TxHeader.ExtId = 0x01;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    time = HAL_GetTick();

    SD_MPU6050_Init(&hi2c2,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_500s,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    	timePrev = time;
    	time = HAL_GetTick();
        elapsedTime = (time - timePrev) * 0.001;

        pin=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

	    control.valueJoyStick_X_1 = parseData(RxData[0]);
		control.valueJoyStick_Y_1 = parseData(RxData[1]);
		control.valueJoyStick_X_2= parseData(RxData[2]);
		control.valueJoyStick_Y_2 = parseData(RxData[3]);
		control.desired_angle = parseData(RxData[4]);
		control.armco = RxData[5];
		control.mod = RxData[6];

        if(pin == 0 || control.mod!=1){
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		HAL_Delay(1000);
		HAL_NVIC_SystemReset();
        }

        arming();

    	char tx[250];

    	SD_MPU6050_ReadAngles(&hi2c2, &mpu1);

		//float AngleX = mpu1.GyroAngles_X;
		//float AngleY = mpu1.GyroAngles_Y;
		//float AngleZ = mpu1.GyroAngles_Z;
		float Temp = mpu1.Temperature;
		float AngleX = mpu1.Angels_X/2.0;
		float AngleY = mpu1.Angels_Y/2.0;
		float AngleZ = mpu1.Angels_Z/2.0;

		error = AngleZ - desired_angle;

		pid_p = kp*error;

		//if(-3.0 >= error && error <=3.0)
		pid_i = pid_i+(ki*error)*elapsedTime;

		pid_d = kd*((error - previous_error)/elapsedTime);

		PID = pid_p + pid_i + pid_d;

		if(PID <= -1000)
		  PID=-1000;
		if(PID >= 1000)
		  PID=1000;


        if(control.valueJoyStick_Y_1<1000 || control.valueJoyStick_Y_1>2000 || control.valueJoyStick_X_1<1000 || control.valueJoyStick_X_1>2000
        || control.valueJoyStick_Y_2<1000 || control.valueJoyStick_Y_2>2000 || control.valueJoyStick_X_2<1000 || control.valueJoyStick_X_2>2000)
        {
        	control.valueJoyStick_Y_1 = 1500;
        	control.valueJoyStick_X_1 = 1500;
        	control.valueJoyStick_Y_2 = 1500;
        	control.valueJoyStick_X_2 = 1500;
        }
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, control.valueJoyStick_Y_1);//yukarı
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, control.valueJoyStick_Y_1);//control.valueJoyStick_Y_1
/*
        pwmLeft + (int)PID
		pwmRight - (int)PID
		pwmRight - (int)PID
		pwmLeft + (int)PID
		*/
    	onsa_deger =  1500 + (int)PID  - (control.valueJoyStick_Y_2 - 1500) + (control.valueJoyStick_X_2 - 1500) + (control.valueJoyStick_X_1 - 1500);
        onso_deger =  1500 - (int)PID  - (control.valueJoyStick_Y_2 - 1500) - (control.valueJoyStick_X_2 - 1500) - (control.valueJoyStick_X_1 - 1500);
        arsa_deger =  1500 - (int)PID  + (control.valueJoyStick_Y_2 - 1500) + (control.valueJoyStick_X_2 - 1500) - (control.valueJoyStick_X_1 - 1500);
        arso_deger =  1500 + (int)PID + (control.valueJoyStick_Y_2 - 1500) - (control.valueJoyStick_X_2 - 1500) + (control.valueJoyStick_X_1 - 1500);

        if (onsa_deger >= 2000)
            onsa_deger = 2000;
        else if (onsa_deger <= 1000)
            onsa_deger = 1000;
        if (arsa_deger >= 2000)
            arsa_deger = 2000;
        else if (arsa_deger <= 1000)
            arsa_deger = 1000;
        if (onso_deger >= 2000)
            onso_deger = 2000;
        else if (onso_deger <= 1000)
            onso_deger = 1000;
        if (arso_deger >= 2000)
            arso_deger = 2000;
        else if (arso_deger <= 1000)
            arso_deger = 1000;

        if(control.desired_angle == 1){
        	desired_angle = control.desired_angle*90;
        }

        //desired_angle = 90*control.desired_angle;

        if(control.valueJoyStick_X_1 != 1500)
        desired_angle = AngleZ;

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arsa_deger);//arsa_deger
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arso_deger);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, onso_deger);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, onsa_deger);

        if(control.armco == 1)
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        else
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        //HAL_UART_Transmit(&huart1, (uint8_t*)tx, sprintf(tx, "X_1: %d Y_1: %d X_2: %d Y_2: %d\n", arsa_deger,arso_deger,onso_deger,onsa_deger), 500);
        //HAL_UART_Transmit(&huart1, (uint8_t*)tx, sprintf(tx, "X_1: %d Y_1: %d X_2: %d Y_2: %d angle: %d arm: %d mod: %d\n", control.valueJoyStick_X_1,control.valueJoyStick_Y_1,control.valueJoyStick_X_2,control.valueJoyStick_Y_2,control.desired_angle,control.armco,control.mod), 500);

        //HAL_UART_Transmit(&huart1, (uint8_t*)tx, sprintf(tx, "AngleX: %f AngleY: %f AngelZ: %f desired_angle: %f error: %f\n", AngleX,AngleY,AngleZ,desired_angle,error), 500);
/*
       	 if(HAL_GetTick() - last_time > INTERVAL)
       	 {
       		TxData[0] = Battery();
       		last_time = HAL_GetTick();

       		HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

       		//HAL_UART_Transmit(&huart1, (uint16_t*)tx, sprintf(tx, "Batt: %d\n", TxData[0]), 500);
       	 }
*/

        //SD_MPU6050_ReadTemperature(&hi2c2, &mpu1);

        //float temper = mpu1.Temperature;
        //temp = (char)temper;
        /*
        SD_MPU6050_ReadGyroscope(&hi2c2, &mpu1);
        int16_t g_x = mpu1.Gyroscope_X;
        int16_t g_y = mpu1.Gyroscope_Y;
        int16_t g_z = mpu1.Gyroscope_Z;

        SD_MPU6050_ReadAccelerometer(&hi2c2, &mpu1);
        int16_t a_x = mpu1.Accelerometer_X;
        int16_t a_y = mpu1.Accelerometer_Y;
        int16_t a_z = mpu1.Accelerometer_Z;
        */



       // HAL_UART_Transmit(&huart1, (uint8_t*)tx, sprintf(tx, "pid_p: %f pid_i: %f pid_d: %f desired_angle: %f error: %f\n", pid_p,pid_i,pid_d,desired_angle,error), 500);
        //HAL_UART_Transmit(&huart1, (uint8_t*)tx, sprintf(tx, "pwmRight: %d pwmLeft: %d\n", pwmRight,pwmLeft), 500);

        HAL_UART_Transmit(&huart1, (uint8_t*)tx, sprintf(tx, "dataXaxis: %f dataYaxis: %f dataZaxis: %f\n PWML: %d", AngleX, AngleY, AngleZ,pwmLeft), 500);

        previous_error = error;


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
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
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
float Battery()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 2);

    float refV = 12.6;
    float level = (adc_buffer[0] * refV / 1314)*100;
    level=(int)level;

    return level;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
void uartPrint(UART_HandleTypeDef *huart, char _out[]){
    HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
}
void uartPrintln(UART_HandleTypeDef *huart, char _out[]){
    HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
    char newline[2] = "\r\n";
    HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}
void arming(){
if(armed == 1){
/*	int i;
	for(i=2100; i<=1000; i--){
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);

				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, i);

				HAL_Delay(10);
			}*/

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);

		HAL_Delay(2000);

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 2000);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);

		HAL_Delay(500);


		armed = 0;
		}
}

uint16_t parseData(uint8_t bit8data)
{
	return (bit8data*4)+1000;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
