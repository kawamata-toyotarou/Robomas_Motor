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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	volatile uint16_t CANID;
	volatile uint8_t motorID;
	volatile int16_t actVel;
	volatile int16_t actangle;
	volatile int16_t actCurrent;
  volatile int16_t cu;
}motor;

typedef struct
{
  uint16_t CANID;
  uint8_t motorID;
  float trgVel;
}hiradora;

typedef float float32_t;
typedef uint16_t float16_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_MACRO_kaetekudasai 0x7FE // example CAN ID macro for testing
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_TxHeaderTypeDef TxHeader_motor;

motor robomas[8] = {
  {0x201, 1, 0, 0, 0, 0},
  {0x202, 2, 0, 0, 0, 0},
  {0x203, 3, 0, 0, 0, 0},
  {0x204, 4, 0, 0, 0, 0},
  {0x205, 5, 0, 0, 0, 0},
  {0x206, 6, 0, 0, 0, 0},
  {0x207, 7, 0, 0, 0, 0},
  {0x208, 8, 0, 0, 0, 0},
};

hiradora Hiradora[8] = {
  {0x401, 1, 0},
  {0x402, 2, 0},
  {0x403, 3, 0},
  {0x404, 4, 0},
  {0x405, 5, 0},
  {0x406, 6, 0},
  {0x407, 7, 0},
  {0x408, 8, 0},
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN3_Init(void);
/* USER CODE BEGIN PFP */
void interboard_comms_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings);
void interboard_comms_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef interboard_comms_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef CAN_SEND(uint32_t CANID, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader);

void motor_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings);
void motor_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader);

void move_motor(motor *motor_typedef);

float16_t convert_f32_f16(float32_t);
float32_t convert_f16_f32(float16_t);

void move_hiradora(hiradora *Hiradora_handler);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if (RESET != (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {

    /* Retrieve Rx messages from RX FIFO1 */
		uint8_t RxData[64] = {};
    FDCAN_RxHeaderTypeDef RxHeader;
		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData)) {
			printf("fdcan_getrxmessage is error\r\n");
			Error_Handler();
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

    /* Retrieve Rx messages from RX FIFO0 */
		uint8_t RxData_motor[8] = {};
    FDCAN_RxHeaderTypeDef RxHeader_motor;
		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader_motor, RxData_motor)) {
			printf("fdcan_getrxmessage_motor is error\r\n");
			Error_Handler();
		}
		/*receive robomas's status*/
		for (int i=0; i < 8; i++){
			if (RxHeader_motor.Identifier == (robomas[i].CANID)) {
				robomas[i].actangle = (int16_t)((RxData_motor[0] << 8) | RxData_motor[1]);
				robomas[i].actVel = (int16_t)((RxData_motor[2] << 8) | RxData_motor[3]);
				robomas[i].actCurrent = (int16_t)((RxData_motor[4] << 8) | RxData_motor[5]);
			}
		}
	}
}

int _write(int file,char *ptr,int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 10);
  return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  setbuf(stdout, NULL);
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
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_FDCAN3_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_OK != interboard_comms_CAN_RxTxSettings_init(&TxHeader)) Error_Handler();
  HAL_TIM_Base_Start_IT(&htim6);
  if (HAL_OK != motor_CAN_RxTxSettings_init(&TxHeader_motor)) Error_Handler();
  /* USER CODE END 2 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 2;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 15;
  hfdcan3.Init.DataTimeSeg2 = 4;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Board_LED_Pin */
  GPIO_InitStruct.Pin = Board_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void interboard_comms_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings)
{
  Hfdcan_Filter_Settings->IdType = FDCAN_STANDARD_ID;
  Hfdcan_Filter_Settings->FilterIndex = 0;
  Hfdcan_Filter_Settings->FilterType = FDCAN_FILTER_RANGE;
  Hfdcan_Filter_Settings->FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  Hfdcan_Filter_Settings->FilterID1 = 0x00;
  Hfdcan_Filter_Settings->FilterID2 = 0x7ff;
}

void interboard_comms_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  Htxheader->Identifier = 0x00;
  Htxheader->IdType = FDCAN_STANDARD_ID;
  Htxheader->TxFrameType = FDCAN_DATA_FRAME;
  Htxheader->DataLength = FDCAN_DLC_BYTES_8;
  Htxheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  Htxheader->FDFormat = FDCAN_FD_CAN;
  Htxheader->BitRateSwitch = FDCAN_BRS_ON;
  Htxheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  Htxheader->MessageMarker = 0;
}

HAL_StatusTypeDef interboard_comms_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  FDCAN_FilterTypeDef FDCAN_Filter_settings;
  interboard_comms_CAN_filter_init(&FDCAN_Filter_settings);
  interboard_comms_CAN_txheader_init(Htxheader);
  if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter_settings))
  {
    printf("fdcan_configfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
  {
    printf("fdcan_configglobalfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_Start(&hfdcan1))
  {
    printf("fdcan_start is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0))
  {
    printf("fdcan_activatenotification is error\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

// CAN_SEND(0x7FE, TxData, &hfdcan1, &TxHeader); // example usage
HAL_StatusTypeDef CAN_SEND(uint32_t CANID, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader)
{
  htxheader->Identifier = CANID;
  if (HAL_OK != HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, htxheader, txdata))
  {
    printf("addmessage error\r\n");
    return HAL_ERROR;
  }
  return HAL_OK;
}


void motor_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings)
{
  Hfdcan_Filter_Settings->IdType = FDCAN_STANDARD_ID;
  Hfdcan_Filter_Settings->FilterIndex = 0;
  Hfdcan_Filter_Settings->FilterType = FDCAN_FILTER_RANGE;
  Hfdcan_Filter_Settings->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  Hfdcan_Filter_Settings->FilterID1 = 0x200;
  Hfdcan_Filter_Settings->FilterID2 = 0x410;
}

void motor_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  Htxheader->Identifier = 0x200;
  Htxheader->IdType = FDCAN_STANDARD_ID;
  Htxheader->TxFrameType = FDCAN_DATA_FRAME;
  Htxheader->DataLength = FDCAN_DLC_BYTES_8;
  Htxheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  Htxheader->FDFormat = FDCAN_CLASSIC_CAN;
  Htxheader->BitRateSwitch = FDCAN_BRS_ON;
  Htxheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  Htxheader->MessageMarker = 0;
}

HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  FDCAN_FilterTypeDef FDCAN_Filter_settings;
  motor_CAN_filter_init(&FDCAN_Filter_settings);
  motor_CAN_txheader_init(Htxheader);
  if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter_settings))
  {
    printf("fdcan_configfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
  {
    printf("fdcan_configglobalfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_Start(&hfdcan3))
  {
    printf("fdcan_start is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0))
  {
    printf("fdcan_activatenotification is error\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

void move_motor(motor *motor_typedef)
{
  uint8_t TxData[8] = {};
  for (int i = 0; i < 4; i++)
  {
    TxData[i*2] = (uint8_t)(motor_typedef->cu >> 8);
    TxData[i*2 + 1] = (uint8_t)(motor_typedef->cu & 0xff);
  }
  CAN_SEND(0x200, TxData, &hfdcan3, &TxHeader_motor);
}

float16_t convert_f32_f16(float32_t fp32){
    uint32_t fp32_bits = *(uint32_t*)&fp32;
    uint16_t sign = (fp32_bits >> 31) & 1;
    uint16_t exponent = (fp32_bits >> 23) & 0xff;
    uint32_t mantissa = fp32_bits & 0x7fffff;
    uint16_t fp16_bits;

    if(exponent == 0xFF){
        if(mantissa == 0){
            //NaN
            fp16_bits = 0x1F << 10 | 1;
        }else{
            //+inf/-inf
            fp16_bits = sign << 15 | 0x1F << 10;
        }
    }else{
        if(exponent > 16 + 127){
            //Overflow
            fp16_bits = sign << 15 | 0x1F << 10;
        }else if(exponent < - 15 + 127){
            //Underflow
            return sign << 15;
        }else{
            fp16_bits = sign << 15 | (exponent + 15 - 127) << 10 | mantissa >> 14;
        }
    }
    return *(float16_t*)&fp16_bits;
}

float32_t convert_f16_f32(float16_t fp16){
    uint16_t fp16_bits = *(uint16_t*)&fp16;
    uint32_t sign = (fp16_bits >> 15) & 1;
    uint32_t exponent = (fp16_bits >> 10) & 0x1F;
    uint32_t mantissa = fp16_bits & 0x3FF;
    uint32_t fp32_bits = 0;

    if(exponent == 0x1F){
        if(mantissa == 0){
            //NaN
            fp32_bits = 0xFF << 23 | 1;
        }else{
            //+inf/-inf
            fp32_bits = sign << 31 | 0xFF << 23;
        }
    }else{
        if(exponent == 0){
            fp32_bits = sign << 31;
        }else{
            fp32_bits = sign << 31 | (exponent - 15 + 127) << 23 | mantissa << 14;
        }
    }

    return *(float32_t*)&fp32_bits;
}

void move_hiradora(hiradora *Hiradora_handler)
{
  uint8_t TxData[8] = {0};
  for (int i = 0; i < 3; i++)
  {
    float16_t trgVel_f16 = convert_f32_f16((float32_t)Hiradora_handler[i].trgVel);
    TxData[i*2] = (uint8_t)(trgVel_f16 >> 8);
    TxData[i*2 + 1] = (uint8_t)(trgVel_f16 & 0xff);
  }
  CAN_SEND(0x400, TxData, &hfdcan3, &TxHeader_motor);
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
