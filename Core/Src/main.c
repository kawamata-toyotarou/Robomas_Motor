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
#include <memory.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
    volatile float speed_total_difference;
    volatile float speed_last_time_difference;   
    float Kp, Ki, Kd, Ka;
    volatile uint16_t angle_data;   
    volatile int16_t speed;
    volatile uint16_t can_id; 
    int16_t speed_target;
    int16_t current_target;
    volatile uint16_t angle;
    volatile float current;
    volatile float lowpass_difference;
    volatile float angle_target;
    volatile float last_time_angle;
    volatile float angle_total_difference;
    volatile float angle_last_time_difference; 
    volatile float angle_lowpass_difference;
    volatile int16_t rotate_last_time_angle;
    volatile int32_t rotate_total_angle; 
    volatile int16_t rotate_now_angle;
    int init_flag;
    int16_t now_current; 
    int16_t to_send_current;
    volatile int32_t can_now_time;
} Motor;

typedef struct
{
  uint16_t CANID;
  uint8_t motorID;
  float trgVel;
} hiradora;

typedef float float32_t;
typedef uint16_t float16_t;

typedef struct
{
  int32_t position;  // Current position (rad)
  int16_t velocity;  // Current velocity (rad/s)
  int16_t torque;    // Current torque (Nm)
  uint8_t mode;      // Current mode
  uint8_t error;
}motor_status;

typedef struct
{
  int16_t p_des;     // Target position (rad)
  int16_t v_des;     // Target velocity (rad/s)
  uint16_t kp;       // Position gain
  uint16_t kd;       // Velocity gain
  int16_t t_ff;      // Feed-forward torque (Nm)
}mit_command;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_MACRO_kaetekudasai 0x7FE // example CAN ID macro for testing

#define MAIN_CANID 0xFE

// 何の値を変換するかを指定する
enum
{
  TARGET_ANGLE,
  TARGET_ANGULAR_VELOCITY,
  K_P,
  K_D
};
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

// --- PID制御 & 位置復元用変数 (Input 2より) ---
volatile int32_t saved_total_ecd[4] __attribute__((section(".noinit"))); // リセットまたぎ用
volatile uint32_t magic_flag __attribute__((section(".noinit")));        // 起動判定フラグ

int mode[4] ={1,1,1,1};  // 0:位置制御, 1:速度制御, 2:カスケード制御, 3:電流制御
Motor motors[4]; // PID制御対象のモーター(ID 0x201~0x204想定)
uint8_t TxData[8] = {0};
uint8_t TxData2[2] = {0};
uint16_t kaitensuu = (uint16_t)(1.4 * 60);
uint8_t cutoff = 8;
uint16_t gravity = 0;

float syuusokuryoku = 0;
uint8_t TxData200[8] = {0}; // ID 1-4 (0x200で送信)用
uint8_t TxData1FF[8] = {0}; // ID 5-8 (0x1FFで送信)用
int16_t karentobaryu = 0;

uint8_t get_id = 0;
uint8_t motor_state=0;
int64_t Elapsed_time;
int len;
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

void motor_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings);
void motor_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef CAN_SEND(uint32_t CANID, uint32_t DataLength, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader);

void move_hiradora(hiradora *Hiradora_handler);
void move_motor(Motor *motor_typedef);
void update_total_angle(Motor *m);
int pid(float v, int mokuhyou, int p, int i, int d, volatile float *gosagoukei, volatile float *lowpastgosa, float gravity, int cutoff, volatile float *maenogosa, int h);
mit_command robosutoraido;
float16_t convert_f32_f16(float32_t);
float32_t convert_f16_f32(float16_t);
void u8_to_int(uint8_t *req, int32_t *des, uint32_t uint8_len);
void u8_to_float(uint8_t *req, float *des, uint32_t uint8_len);
void float_to_u8(float *req, uint8_t *des, uint32_t float_len);
void int_to_u8(int32_t *req, uint8_t *des, uint32_t int_len);

void send_robstride(uint8_t Motor_canid, uint8_t mode, uint8_t *data, uint8_t *data2);

void mit_to_u8(mit_command *cmd, uint8_t *txdata, uint8_t *txdata2);
void move_robstride(mit_command *cmd, uint8_t motor_id);

uint16_t map_robstride(float x, int mode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if((kokoichi_Pin==GPIO_Pin)){
    if(motor_state==0){
      motor_state=1;
      Elapsed_time=HAL_GetTick();
    }
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if (RESET != (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {

    /* Retrieve Rx messages from RX FIFO1 */
		uint8_t RxData[64] = {};
    FDCAN_RxHeaderTypeDef RxHeader;
		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData)) {
			printf("fdcan_getrxmessage is error\r\n");
			Error_Handler();
		}

    len = 0;
    switch (RxHeader.DataLength)
    {
    case FDCAN_DLC_BYTES_0:
      len = 0;
      break;
    case FDCAN_DLC_BYTES_1:
      len = 1;
      break;
    case FDCAN_DLC_BYTES_2:
      len = 2;
      break;
    case FDCAN_DLC_BYTES_3:
      len = 3;
      break;
    case FDCAN_DLC_BYTES_4:
      len = 4;
      break;
    case FDCAN_DLC_BYTES_5:
      len = 5;
      break;
    case FDCAN_DLC_BYTES_6:
      len = 6;
      break;
    case FDCAN_DLC_BYTES_7:
      len = 7;
      break;
    case FDCAN_DLC_BYTES_8:
      len = 8;
      break;
    case FDCAN_DLC_BYTES_12:
      len = 12;
      break;
    case FDCAN_DLC_BYTES_16:
      len = 16;
      break;
    case FDCAN_DLC_BYTES_20:
      len = 20;
      break;
    case FDCAN_DLC_BYTES_24:
      len = 24;
      break;
    case FDCAN_DLC_BYTES_32:
      len = 32;
      break;
    case FDCAN_DLC_BYTES_48:
      len = 48;
      break;
    case FDCAN_DLC_BYTES_64:
      len = 64;
      break;
    default:
      break;
    }

    switch (RxHeader.Identifier)
    {
      case CAN_ID_MACRO_kaetekudasai: // change this value for testing. Reccommend to use an ID with privateDefined macro
        /* code */
        break;
      default:
        // printf("unknown CAN ID received: 0x%03lX\r\n", RxHeader.Identifier); // printf should be commented out within Callback
        break;
    }
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

  uint8_t RxData[8] = {0};
  FDCAN_RxHeaderTypeDef RxHeader;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader, RxData)) {
      printf("fdcan_getrxmessage_motor is error\r\n");
      Error_Handler();
    }
    
    if (FDCAN_STANDARD_ID == RxHeader.IdType)
    {
      // モーター情報の更新 (Input 2のロジックを採用)
      for (int i = 0; i < 4; i++) {
        if (RxHeader.Identifier == motors[i].can_id) {
          motors[i].angle_data = (uint16_t)((RxData[0] << 8) | RxData[1]);
          motors[i].speed = (int16_t)((RxData[2] << 8) | RxData[3]);
          motors[i].now_current = (int16_t)((RxData[4] << 8) | RxData[5]); // トルク電流
          motors[i].init_flag = 1;
          motors[i].can_now_time = HAL_GetTick();
        }
      }
    }
    else if (FDCAN_EXTENDED_ID == RxHeader.IdType)
    {
      if ((RxHeader.Identifier >> 24) == 0x00)
      {
        get_id = (uint16_t)((RxHeader.Identifier >> 8) & 0xFFFFu);
      }
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  uint32_t now = HAL_GetTick();
    // モーター0が一度でも通信できていて、かつ最後の通信から500ms以上経過したら電源が切れたと判断
    if (motors[0].init_flag == 1 && (now - motors[0].can_now_time) > 500) {
        NVIC_SystemReset(); // マイコン自身を強制的に再起動（初期化）する
    }

  if (htim == &htim6) {
    int send200_flag = 0;
    int send1FF_flag = 0;

    for (int h = 0; h < 4; h++) {
      motors[h].rotate_now_angle = motors[h].angle_data;
      update_total_angle(&motors[h]);

      if (mode[h] == 0) { // 位置制御
        motors[h].Kp = 1;
        motors[h].Ki = 0.03;
        motors[h].Kd = 80.0;
        karentobaryu = (int)pid((float)motors[h].rotate_total_angle, motors[h].angle_target, motors[h].Kp, motors[h].Ki, motors[h].Kd, &motors[h].angle_total_difference, &motors[h].angle_lowpass_difference, gravity, cutoff, &motors[h].angle_last_time_difference, h);
        motors[h].last_time_angle = motors[h].angle;
        int error_abs = motors[h].angle_target - motors[h].rotate_total_angle;
        if (error_abs < 0) error_abs = -error_abs;
        if (error_abs < 10) {
          karentobaryu = 0;
          motors[h].angle_total_difference = 0;
        }
      }

      if (mode[h] == 1) { // 速度制御
        if(motor_state==0){
            motors[h].speed_target=-300;
        }
        else{
            motors[h].speed_target=300;
        }
        motors[h].Kp = 15.0;
        motors[h].Ki = 8.0;
        motors[h].Kd = 10.0;
        karentobaryu = (int)(pid(motors[h].speed, motors[h].speed_target, motors[h].Kp, motors[h].Ki, motors[h].Kd, &motors[h].speed_total_difference, &motors[h].lowpass_difference, gravity, cutoff, &motors[h].speed_last_time_difference, h));
      }

      if (mode[h] == 2) { // カスケード制御
        motors[h].Kp = 1;
        motors[h].Ki = 0.0;
        motors[h].Kd = 0.0;
        motors[h].speed_target = (int)pid((float)motors[h].rotate_total_angle, motors[h].angle_target, motors[h].Kp, motors[h].Ki, motors[h].Kd, &motors[h].angle_total_difference, &motors[h].angle_lowpass_difference, gravity, cutoff, &motors[h].angle_last_time_difference, h);
        
        motors[h].Kp = 15.0;
        motors[h].Ki = 8.0;
        motors[h].Kd = 10.0;
        karentobaryu = (int)(pid(motors[h].speed, motors[h].speed_target, motors[h].Kp, motors[h].Ki, motors[h].Kd, &motors[h].speed_total_difference, &motors[h].lowpass_difference, gravity, cutoff, &motors[h].speed_last_time_difference, h));
        int error_abs = motors[h].angle_target - motors[h].rotate_total_angle;
        if (error_abs < 0) error_abs = -error_abs;
        if (error_abs < 20) {
          karentobaryu = 0;
          motors[h].angle_total_difference = 0;
        }
      }

      if (mode[h] == 3) { // 電流制御
        motors[h].current_target=1000;
        // motors[h].Kp = 15.0;
        // motors[h].Ki = 8.0;
        // motors[h].Kd = 0.0;
        //karentobaryu=(int)(pid(motors[h].c_current,motors[h].cmokuhyou,motors[h].Kp,motors[h].Ki,motors[h].Kd,&motors[h].gosagoukei,&motors[h].lowpastgosa,gravity,cutoff,&motors[h].maenogosa)); 
        karentobaryu = motors[h].current_target;
      }
// --- 時間経過による安全停止ロジック ---
      if (motor_state == 1) { // ボタンが押された後
          uint32_t diff_time = now - Elapsed_time;
          
          if (diff_time < 1000) {
              // ボタンを押して1秒未満は停止
              karentobaryu = 0;
          } else if (diff_time > 6000) {
              // ボタンを押して6秒以上経ったら停止
              karentobaryu = 0;
          } else {
              // 1秒〜6秒の間は、PIDで計算されたkarentobaryu
          }
      } else {
          // motor_state == 0 (起動直後、ボタンが押される前) の動作
      }
      saved_total_ecd[h] = motors[h].rotate_total_angle;

      // 送信データの作成
      if (motors[h].can_id >= 0x201 && motors[h].can_id <= 0x204) {
        int idx = (motors[h].can_id - 0x201) * 2;
        TxData200[idx] = (karentobaryu >> 8) & 0xFF;
        TxData200[idx + 1] = karentobaryu & 0xFF;
        send200_flag = 1;
      } else if (motors[h].can_id >= 0x205 && motors[h].can_id <= 0x208) {
        int idx = (motors[h].can_id - 0x205) * 2;
        TxData1FF[idx] = (karentobaryu >> 8) & 0xFF;
        TxData1FF[idx + 1] = karentobaryu & 0xFF;
        send1FF_flag = 1;
      }

      motors[h].angle = motors[h].angle_data * 360 / 8192;
    }

    if (send200_flag) {
      TxHeader_motor.Identifier = 0x200;
      TxHeader_motor.DataLength = FDCAN_DLC_BYTES_8; // 念のため
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_motor, TxData200) != HAL_OK) {
          // Error Handling
      }
    }

    if (send1FF_flag) {
      TxHeader_motor.Identifier = 0x1FF;
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_motor, TxData1FF) != HAL_OK) {
          // Error Handling
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

  __HAL_RCC_CLEAR_RESET_FLAGS();

  if (HAL_OK != interboard_comms_CAN_RxTxSettings_init(&TxHeader)) Error_Handler();
  HAL_TIM_Base_Start_IT(&htim6);
  if (HAL_OK != motor_CAN_RxTxSettings_init(&TxHeader_motor)) Error_Handler();

  robosutoraido.p_des=0; robosutoraido.v_des=1000; robosutoraido.kd=3000; robosutoraido.kp=3000; robosutoraido.t_ff=1000;

  for (int i = 0; i < 4; i++) {
    motors[i].Kp = 1.0;     // 位置制御用ゲイン (調整してください)
    motors[i].Ki = 0.0; 
    motors[i].Kd = 0.0;     // 必要に応じて入れる
    motors[i].can_id = 0x201 + i;
    motors[i].init_flag = 0;
    
    motors[i].speed_total_difference = 0;
    motors[i].speed_last_time_difference = 0;
    motors[i].speed = 0;
    motors[i].angle_data = 0;
    motors[i].rotate_total_angle = 0;      // とりあえず0
    motors[i].lowpass_difference = 0;
  }


  printf("start!\r\n");
  printf("Waiting for motor data to set HOME position...\r\n");

  // --- 1. まずモーターからの通信が来るのを待つ ---
  // これがないと、初期位置(last_raw_ecd)が0のまま計算が始まり、
  // 起動直後に巨大な角度変化として誤検知されて暴走する。
  while (1) {
    int ready_count = 0;
    for (int i = 0; i < 4; i++) {
      if (motors[i].init_flag == 1) {
        ready_count++;
      }
    }
    // とりあえず1個でも通信できたら次へ（全モーター繋いでいるなら == 4 にする）
    if (ready_count >= 1) {
      HAL_Delay(100); // データ安定待ち
      break;
    }
    HAL_Delay(10);
  }

  printf("Motor data received. Setting current position as ZERO.\r\n");


  // --- 2. 現在の位置を「0」としてリセットする (ここが重要) ---
  float one_degree_val = 8192.0f / 360.0f; 
  
  float target_move_angle = 30.0f; // ★ここで「起動後に動かしたい角度」を指定

  for (int i = 0; i < 4; i++) {
    motors[i].rotate_last_time_angle = motors[i].angle_data; 
    motors[i].rotate_total_angle = 0; 
    motors[i].angle_target = (int)(target_move_angle * one_degree_val); 
    motors[i].angle_total_difference = 0;
    motors[i].angle_last_time_difference = 0;
    motors[i].angle_lowpass_difference = 0;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    uint8_t data[8] = {0};
    uint8_t data2[2] = {0, MAIN_CANID};
    send_robstride(0x7f,0x3,data, data2);
    move_robstride(&robosutoraido, 0x7f);
    HAL_Delay(2);
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
  hfdcan3.Init.ExtFiltersNbr = 1;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Board_LED_Pin */
  GPIO_InitStruct.Pin = Board_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
HAL_StatusTypeDef CAN_SEND(uint32_t CANID, uint32_t DataLength, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader)
{
  htxheader->DataLength = DataLength;
  htxheader->Identifier = CANID;
  if (HAL_OK != HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, htxheader, txdata))
  {
    printf("addmessage error\r\n");
    return HAL_ERROR;
  }
  return HAL_OK;
}

void u8_to_float(uint8_t *req, float *des, uint32_t uint8_len)
{
  union IntAndFloat {
    uint32_t ival;
    float fval;
  };
  for(int i = 0; i < uint8_len/4; i++){
    uint32_t f32_u32 = ((req[i*4] << 24) | (req[i*4+1] << 16) | (req[i*4+2] << 8) | (req[i*4+3]));
    union IntAndFloat target;
    target.ival = f32_u32;
    des[i] = target.fval;
  }
}

void u8_to_int(uint8_t *req, int32_t *des, uint32_t uint8_len)
{
  for(int i = 0; i < uint8_len/4; i++){
    uint32_t u32 = ((req[i*4] << 24) | (req[i*4+1] << 16) | (req[i*4+2] << 8) | (req[i*4+3]));
    des[i] = (int32_t)u32;
  }
}

void float_to_u8(float *req, uint8_t *des, uint32_t float_len)
{
  union IntAndFloat {
    uint32_t ival;
    float fval;
  };
  for (int i = 0; i < float_len; i++)
  {
    union IntAndFloat target;
    target.fval = req[i];
    uint32_t val = target.ival;
    des[i*4    ] = (uint8_t)((val >> 24) & 0xff);
    des[i*4 + 1] = (uint8_t)((val >> 16) & 0xff);
    des[i*4 + 2] = (uint8_t)((val >>  8) & 0xff);
    des[i*4 + 3] = (uint8_t)((val      ) & 0xff);
  }
}

void int_to_u8(int32_t *req, uint8_t *des, uint32_t int_len)
{
  for (int i = 0; i < int_len; i++)
  {
    uint32_t val = (uint32_t)req[i];
    des[i*4    ] = (uint8_t)((val >> 24) & 0xff);
    des[i*4 + 1] = (uint8_t)((val >> 16) & 0xff);
    des[i*4 + 2] = (uint8_t)((val >>  8) & 0xff);
    des[i*4 + 3] = (uint8_t)((val      ) & 0xff);
  }
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
  Htxheader->BitRateSwitch = FDCAN_BRS_OFF;
  Htxheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  Htxheader->MessageMarker = 0;
}

void robstride_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings)
{
  Hfdcan_Filter_Settings->IdType = FDCAN_EXTENDED_ID;
  Hfdcan_Filter_Settings->FilterIndex = 0;
  Hfdcan_Filter_Settings->FilterType = FDCAN_FILTER_RANGE;
  Hfdcan_Filter_Settings->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  Hfdcan_Filter_Settings->FilterID1 = 0x00;
  Hfdcan_Filter_Settings->FilterID2 = 0x2000000;
}

HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  FDCAN_FilterTypeDef FDCAN_Filter_settings;
  FDCAN_FilterTypeDef FDCAN_Filter_settings_robstride;
  motor_CAN_filter_init(&FDCAN_Filter_settings);
  robstride_CAN_filter_init(&FDCAN_Filter_settings_robstride);
  motor_CAN_txheader_init(Htxheader);
  if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter_settings))
  {
    printf("fdcan_configfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter_settings_robstride))
  {
    printf("fdcan_configfilter robstride is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
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

void move_motor(Motor *motor_typedef)
{
  uint8_t TxData[8] = {};
  for (int i = 0; i < 4; i++)
  {
    TxData[i*2] = (uint8_t)(motor_typedef[i].to_send_current >> 8);
    TxData[i*2 + 1] = (uint8_t)(motor_typedef[i].to_send_current & 0xff);
  }
  CAN_SEND(0x200, FDCAN_DLC_BYTES_8, TxData, &hfdcan3, &TxHeader_motor);
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
  for (int i = 0; i < 4; i++)
  {
    float16_t trgVel_f16 = convert_f32_f16((float32_t)Hiradora_handler[i].trgVel);
    TxData[i*2] = (uint8_t)(trgVel_f16 >> 8);
    TxData[i*2 + 1] = (uint8_t)(trgVel_f16 & 0xff);
  }
  CAN_SEND(0x400, FDCAN_DLC_BYTES_8, TxData, &hfdcan3, &TxHeader_motor);
}

void update_total_angle(Motor *m) {
    int16_t diff = m->angle_data - m->rotate_last_time_angle;
    if (diff > 4096) {
        diff -= 8192;
    }
    if (diff < -4096) {
        diff += 8192;
    }
    m->rotate_total_angle += diff;
    m->rotate_last_time_angle = m->angle_data;
}

int pid(float v, int mokuhyou, int p, int i, int d, volatile float *gosagoukei, volatile float *lowpastgosa, float gravity, int cutoff, volatile float *maenogosa, int h) {
    float current = (float)v;
    float gosa = (mokuhyou - current);
    
    *gosagoukei += gosa * 0.001;
    if (*gosagoukei > 10000) *gosagoukei = 10000;
    if (*gosagoukei < -10000) *gosagoukei = -10000;
    
    float derivative = (gosa - *maenogosa);
    *lowpastgosa += (derivative - *lowpastgosa) / cutoff;
    if (*lowpastgosa > 100) *lowpastgosa = 100;
    if (*lowpastgosa < -100) *lowpastgosa = -100;
    
    if (mode[h] == 2) {
        syuusokuryoku = p * gosa;
    } else {
        syuusokuryoku = p * gosa + i * (*gosagoukei) + d * (*lowpastgosa) + gravity;
    }
    
    *maenogosa = gosa;
    
    if (syuusokuryoku > 16384.0f) {
        syuusokuryoku = 16384.0f;
    } else if (syuusokuryoku < -16384.0f) {
        syuusokuryoku = -16384.0f;
    }
    
    return (int16_t)syuusokuryoku;
}

void send_robstride(uint8_t Motor_canid, uint8_t mode, uint8_t *data, uint8_t *data2)
{
  FDCAN_TxHeaderTypeDef TxHeader_robstride = TxHeader_motor;
  TxHeader_robstride.IdType = FDCAN_EXTENDED_ID;
  CAN_SEND((uint32_t)(((mode & 0x1F) << 24) | ((data2[1] << 8 | data2[0]) << 8) | (Motor_canid & 0xFF)), FDCAN_DLC_BYTES_8, data, &hfdcan3, &TxHeader_robstride);
}

void mit_to_u8(mit_command *cmd, uint8_t *txdata, uint8_t *txdata2)
{
  txdata[0] = (uint8_t)(((uint16_t)cmd->p_des >> 8) & 0xFF);
  txdata[1] = (uint8_t)((uint16_t)cmd->p_des & 0xFF);
  txdata[2] = (uint8_t)(((uint16_t)cmd->v_des >> 8) & 0xFF);
  txdata[3] = (uint8_t)((uint16_t)cmd->v_des & 0xFF);
  txdata[4] = (uint8_t)(((uint16_t)cmd->kp >> 8) & 0xFF);
  txdata[5] = (uint8_t)((uint16_t)cmd->kp & 0xFF);
  txdata[6] = (uint8_t)(((uint16_t)cmd->kd >> 8) & 0xFF);
  txdata[7] = (uint8_t)((uint16_t)cmd->kd & 0xFF);

  txdata2[0] = (uint8_t)(((uint16_t)cmd->t_ff >> 8) & 0xFF);
  txdata2[1] = (uint8_t)((uint16_t)cmd->t_ff & 0xFF);
}

void move_robstride(mit_command *cmd, uint8_t motor_id)
{
  uint8_t TxData[8] = {0};
  uint8_t TxData2[2] = {0};

  mit_to_u8(cmd, TxData, TxData2);

  send_robstride(motor_id, 0x1, TxData, TxData2);
}
uint16_t map_robstride(float x, int mode)
{
    float before_min, before_max, after_min, after_max;
    switch(mode)
    {
        case TARGET_ANGLE:
            before_min = -4 * M_PI;
            before_max = 4 * M_PI;
            break;
        case TARGET_ANGULAR_VELOCITY:
            before_min = -44.0;
            before_max = 44.0;
            break;
        case K_P:
            before_min = 0.0;
            before_max = 500.0;
            break;
        case K_D:
            before_min = 0.0;
            before_max = 5.0;
            break;
    }
    after_min = 0.0;
    after_max = 65535.0;

    return after_min + (x - before_min) * (after_max - after_min) / (before_max - before_min);
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
