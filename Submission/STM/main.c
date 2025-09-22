/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "ICM20948.h"
#include "oled.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"

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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FWTask */
osThreadId_t FWTaskHandle;
const osThreadAttr_t FWTask_attributes = {
  .name = "FWTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BWTask */
osThreadId_t BWTaskHandle;
const osThreadAttr_t BWTask_attributes = {
  .name = "BWTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FLTask */
osThreadId_t FLTaskHandle;
const osThreadAttr_t FLTask_attributes = {
  .name = "FLTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FRTask */
osThreadId_t FRTaskHandle;
const osThreadAttr_t FRTask_attributes = {
  .name = "FRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BLTask */
osThreadId_t BLTaskHandle;
const osThreadAttr_t BLTask_attributes = {
  .name = "BLTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BRTask */
osThreadId_t BRTaskHandle;
const osThreadAttr_t BRTask_attributes = {
  .name = "BRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cmdTask */
osThreadId_t cmdTaskHandle;
const osThreadAttr_t cmdTask_attributes = {
  .name = "cmdTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for moveDistObsTask */
osThreadId_t moveDistObsTaskHandle;
const osThreadAttr_t moveDistObsTask_attributes = {
  .name = "moveDistObsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for turnATask */
osThreadId_t turnATaskHandle;
const osThreadAttr_t turnATask_attributes = {
  .name = "turnATask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for turnBTask */
osThreadId_t turnBTaskHandle;
const osThreadAttr_t turnBTask_attributes = {
  .name = "turnBTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GHTask */
osThreadId_t GHTaskHandle;
const osThreadAttr_t GHTask_attributes = {
  .name = "GHTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PPTask */
osThreadId_t PPTaskHandle;
const osThreadAttr_t PPTask_attributes = {
  .name = "PPTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DZTask */
osThreadId_t DZTaskHandle;
const osThreadAttr_t DZTask_attributes = {
  .name = "DZTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

uint8_t RX_BUFFER_SIZE = 5;
uint8_t aRxBuffer[10];

typedef struct _command
{
  uint8_t index;
  uint16_t val;
} Command;

uint8_t CMD_BUFFER_SIZE = 12;
typedef struct _commandQueue
{
  uint8_t head;
  uint8_t tail;
  uint8_t size;
  Command buffer[12];
} CommandQueue;

CommandQueue cmdQ;

Command curCmd;
uint8_t rxMsg[16];

uint8_t manualMode = 0;

// imu
float targetAngle = 0;
float angleNow = 0;
float angleTemp = 0;
uint8_t readGyroZData[2];
int16_t gyroZ = 0;
int16_t readGyroData[3];

// angle - Task 2
//  For Task 2 Turn A
uint8_t FCOFFSET = 0; //3.5
uint8_t FAOFFSET = 1.5; //4

// motor
uint16_t newDutyL, newDutyR;
uint32_t last_curTask_tick = 0;

// distance
int curSpeed = 0;
float targetDist = 0;
uint16_t curDistTick = 0;
uint16_t targetDistTick = 0;
uint16_t dist_dL = 0;
uint16_t lastDistTick_L = 0;

typedef struct _pidConfig
{
  float Kp;
  float Ki;
  float Kd;
  float ek1;
  float ekSum;
} PIDConfig;

PIDConfig pidSlow, pidTSlow, pidFast, pidZoomZoom;

typedef struct _commandConfig
{
  uint16_t leftDuty;
  uint16_t rightDuty;
  float servoTurnVal;
  float targetAngle;
  uint8_t direction;
} CmdConfig;

// command value used for Command struct
#define CONFIG_FL00 7
#define CONFIG_FR00 8
#define CONFIG_BL00 9
#define CONFIG_BR00 10

#define CONFIG_FL30 11
#define CONFIG_FR30 12
#define CONFIG_BL30 13
#define CONFIG_BR30 14

//toedit
CmdConfig cfgs[19] = {
    {0, 0, SERVO_CENTER, 0, DIR_FORWARD},        // STOP
    {1200, 1200, SERVO_CENTER, 0, DIR_FORWARD},  // FW00
    {1200, 1200, SERVO_CENTER, 0, DIR_BACKWARD}, // BW00

    {800, 1200, SERVO_LEFT_MAX, 90, DIR_FORWARD},   // FL--
    {800, 1200, SERVO_LEFT_MAX, -90, DIR_BACKWARD}, // BL--
    {1200, 800, SERVO_RIGHT_MAX, -90, DIR_FORWARD}, // FR--
    {1200, 800, SERVO_RIGHT_MAX, 90, DIR_BACKWARD}, // BR--

    // indoor 3x2
    {800, 2400, SERVO_LEFT_MAX, 86, DIR_FORWARD},   // FL00 87
    {2650, 600, SERVO_RIGHT_MAX, -85.5, DIR_FORWARD},  // FR00 -85.5
    {800, 2400, SERVO_LEFT_MAX, -83.5, DIR_BACKWARD}, // BL00 -83.5
    {2550, 650, SERVO_RIGHT_MAX, 90, DIR_BACKWARD},  // BR00 90

    // outdoor 3x2
    {800, 2400, SERVO_LEFT_MAX, 87.5, DIR_FORWARD},   // FL30 87
    {2650, 600, SERVO_RIGHT_MAX, -86, DIR_FORWARD},  // FR30 -85.5
    {800, 2400, SERVO_LEFT_MAX, -83.5, DIR_BACKWARD}, // BL30 -83.5
    {2550, 650, SERVO_RIGHT_MAX, 90, DIR_BACKWARD},  // BR30 90

};

enum TASK_TYPE
{
  TASK_MOVE_FORWARD,
  TASK_MOVE_BACKWARD,
  TASK_FL,
  TASK_FR,
  TASK_BL,
  TASK_BR,
  TASK_MOVE_DIST,
  TASK_TURN_ANGLE,
  TASK_ADC,
  TASK_MOVE_OBS,
  TASK_MOVE_OBS_ZOOMZOOM,
  TASK_TURN_A,
  TASK_TURN_B,
  TASK_TURN_IR,
  TASK_TURN_IR_CLOSE,
  TASK_GO_HOME,
  TASK_PP, // Just a cup of NIE canteen avocado milkshake, basically.
  TASK_NONE
};
enum TASK_TYPE curTask = TASK_NONE, prevTask = TASK_NONE;

enum MOVE_MODE
{
  SLOW,
  FAST
};
enum MOVE_MODE moveMode = FAST;
float speedScale = 1;

// IR
uint16_t obsTick_IR_R = 0;
uint16_t obsTick_IR_L = 0;
uint8_t obsDist_IR_L = 0, obsDist_IR_R = 0; // left/right IR
uint16_t dataPoint_R = 0;
uint16_t dataPoint_L = 0;
uint32_t IR_data_raw_acc_R = 0;
uint32_t IR_data_raw_acc_L = 0;

// ultrasonic
float obsDist_Ultrasonic = 0;
float obsDist_B = 1000; // saved distance after Turn A - measured by ultrasonic sensor --Task 2

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void runEncoder(void *argument);
void runOledTask(void *argument);
void runFWTask(void *argument);
void runBWTask(void *argument);
void runFLTask(void *argument);
void runFRTask(void *argument);
void runBLTask(void *argument);
void runBRTask(void *argument);
void runCmdTask(void *argument);
void runMoveDistObsTask(void *argument);
void runTurnATask(void *argument);
void runTurnBTask(void *argument);
void runGHTask(void *argument);
void runPPTask(void *argument);
void runDZTask(void *argument);

/* USER CODE BEGIN PFP */

// PID controller
void PIDConfigInit(PIDConfig *cfg, const float Kp, const float Ki, const float Kd);
void PIDConfigReset(PIDConfig *cfg);
// Straight line movement
void StraightMovement(const uint8_t speedMode);
void StraightMovementWithSpeedScale(const uint8_t speedMode, float *speedScale);
void RobotMoveDistance(float *targetDist, const uint8_t dir, const uint8_t speedMode);
void RobotMoveDistanceObstacle(float *targetDist, const uint8_t speedMode);
// Robot turn
void RobotTurn(float *targetAngle);

// For Task 2
// Turn A
void RobotTurnFA45();
void RobotTurnFC45();
void RobotTurnFA90();
void RobotTurnFC90();
void RobotTurnFA180();
void RobotTurnFC180();
// Turn B - Outdoor
void RobotTurnFR30();
void RobotTurnFL30();
// Turn B - Indoor
void RobotTurnFR00();
void RobotTurnFL00();
// UltraSonic sensor read
void HCSR04_Read(void);
// IR sensor read and movement
void RobotMoveUntilIROvershoot(int isIR_R);
void RobotMoveUntilIRCloseDist(int isIR_R);

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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  // oled
  OLED_Init();

  // gyro
  ICM20948_init(&hi2c1, 0, GYRO_FULL_SCALE_2000DPS, ACCEL_FULL_SCALE_2G);

  // servo

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // motor
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  // encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // uart
  HAL_UART_Receive_IT(&huart3, aRxBuffer, RX_BUFFER_SIZE);

  // ultrasonic
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  // IR
  // HAL_ADC_Start(&hadc1);
  // HAL_ADC_Start(&hadc2);

  __RESET_SERVO_TURN(&htim1);

  // command queue initialization
  curCmd.index = 100;
  curCmd.val = 10;

  cmdQ.head = 0;
  cmdQ.tail = 0;
  cmdQ.size = CMD_BUFFER_SIZE;
  for (int i = 0; i < CMD_BUFFER_SIZE; i++)
  {
    Command cmd;
    cmd.index = 100;
    cmd.val = 0;
    cmdQ.buffer[i] = cmd;
  }
  //PIDConfigInit(&pidTSlow, 2.1, 0.045, 0.8);
  //PIDConfigInit(&pidSlow, 2.1, 0.045, 0.8);
  //PIDConfigInit(&pidFast, 1.1, 0.05, 0.3);
  //PIDConfigInit(&pidZoomZoom, 0.6, 0.05, 0.2);

  PIDConfigInit(&pidTSlow, 2.4, 0.06, 0.65);  // Kd: 0.7 → 0.65
  PIDConfigInit(&pidSlow, 2.4, 0.08, 0.65);   // Kd: 0.7 → 0.65
  PIDConfigInit(&pidFast, 1.3, 0.065, 0.2);   // Kd: 0.25 → 0.2
  PIDConfigInit(&pidZoomZoom, 0.75, 0.065, 0.15);// Kd: 0.18 → 0.15

  // UART Rx
  HAL_UART_Receive_IT(&huart3, aRxBuffer, RX_BUFFER_SIZE);

  // adjust steering
  __RESET_SERVO_TURN(&htim1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of encoderTask */
  encoderTaskHandle = osThreadNew(runEncoder, NULL, &encoderTask_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(runOledTask, NULL, &OledTask_attributes);

  /* creation of FWTask */
  FWTaskHandle = osThreadNew(runFWTask, NULL, &FWTask_attributes);

  /* creation of BWTask */
  BWTaskHandle = osThreadNew(runBWTask, NULL, &BWTask_attributes);

  /* creation of FLTask */
  FLTaskHandle = osThreadNew(runFLTask, NULL, &FLTask_attributes);

  /* creation of FRTask */
  FRTaskHandle = osThreadNew(runFRTask, NULL, &FRTask_attributes);

  /* creation of BLTask */
  BLTaskHandle = osThreadNew(runBLTask, NULL, &BLTask_attributes);

  /* creation of BRTask */
  BRTaskHandle = osThreadNew(runBRTask, NULL, &BRTask_attributes);

  /* creation of cmdTask */
  cmdTaskHandle = osThreadNew(runCmdTask, NULL, &cmdTask_attributes);

  /* creation of moveDistObsTask */
  moveDistObsTaskHandle = osThreadNew(runMoveDistObsTask, NULL, &moveDistObsTask_attributes);

  /* creation of turnATask */
  turnATaskHandle = osThreadNew(runTurnATask, NULL, &turnATask_attributes);

  /* creation of turnBTask */
  turnBTaskHandle = osThreadNew(runTurnBTask, NULL, &turnBTask_attributes);

  /* creation of GHTask */
  GHTaskHandle = osThreadNew(runGHTask, NULL, &GHTask_attributes);

  /* creation of PPTask */
  PPTaskHandle = osThreadNew(runPPTask, NULL, &PPTask_attributes);

  /* creation of DZTask */
  DZTaskHandle = osThreadNew(runDZTask, NULL, &DZTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim6.Init.Prescaler = 16-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|US_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin US_Trig_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|US_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t US_diff = 0;
uint8_t Is_First_Captured = 0; // is the first value captured

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // if the interrupt source is channel 2
  {
    if (Is_First_Captured == 0) // if the first value is not captured
    {
      IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
      Is_First_Captured = 1;                                    // set the first captured as true
      // Now change the polarity to falling edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
    }

    else if (Is_First_Captured == 1) // if the first is already captured
    {
      IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read second value
      __HAL_TIM_SET_COUNTER(htim, 0);                           // reset the counter

      if (IC_Val2 > IC_Val1)
      {
        obsDist_Ultrasonic = IC_Val2 - IC_Val1;
      }

      else if (IC_Val1 > IC_Val2)
      {
        obsDist_Ultrasonic = (0xffff - IC_Val1) + IC_Val2;
      }

      obsDist_Ultrasonic = obsDist_Ultrasonic * .034 / 2;
      Is_First_Captured = 0; // set it back to false

      // set polarity to rising edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
      __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC2);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //  prevent unused argument(s) compilation warning
  UNUSED(huart);
  int val;

  val = (aRxBuffer[2] - 48) * 10 + (aRxBuffer[3] - 48);
  if (aRxBuffer[4] >= '0' && aRxBuffer[4] <= '9')
    // val += (aRxBuffer[4] - 48) * 100;
    val = val * 10 + (aRxBuffer[4] - 48);

  manualMode = 0;

  if (aRxBuffer[0] == 'S' && aRxBuffer[1] == 'T')
  { // only STOP can preempt any greedy task
    //__ADD_COMMAND(cmdQ, 0, 0); // stop
    __ON_TASK_END(&htim8, prevTask, curTask);
    angleNow = 0;
    gyroZ = 0; // reset angle for PID
    PIDConfigReset(&pidTSlow);
    PIDConfigReset(&pidSlow);
    PIDConfigReset(&pidFast);
    PIDConfigReset(&pidZoomZoom);
    curDistTick = 0;
    if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
    {
      __CLEAR_CURCMD(curCmd);
      __ACK_TASK_DONE(&huart3, rxMsg);
    }
    else
    {
      __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
  }
  else if (aRxBuffer[0] == 'R' && aRxBuffer[1] == 'S')
  {
    __ON_TASK_END(&htim8, prevTask, curTask);
    angleNow = 0;
    gyroZ = 0; // reset angle for PID
    __RESET_SERVO_TURN(&htim1);
    PIDConfigReset(&pidTSlow);
    PIDConfigReset(&pidSlow);
    PIDConfigReset(&pidFast);
    PIDConfigReset(&pidZoomZoom);
    curDistTick = 0;
    if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
    {
      __CLEAR_CURCMD(curCmd);
      __ACK_TASK_DONE(&huart3, rxMsg);
    }
    else
    {
      __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
  }
  else if (aRxBuffer[0] == 'F' && (aRxBuffer[1] == 'W'))
  { // FW
    manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
    moveMode = FAST;
    __ADD_COMMAND(cmdQ, 1, val);
  }
  else if (aRxBuffer[0] == 'B' && (aRxBuffer[1] == 'W'))
  { // BW
    manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
    moveMode = FAST;
    __ADD_COMMAND(cmdQ, 2, val);
  }

  else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'L')
  { // FL
    manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
    __ADD_COMMAND(cmdQ, 3 + (manualMode ? 0 : 4), val);
  }
  else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'R')
  { // FR
    manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
    __ADD_COMMAND(cmdQ, 4 + (manualMode ? 0 : 4), val);
  }
  else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'L')
  { // BL
    manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
    __ADD_COMMAND(cmdQ, 5 + (manualMode ? 0 : 4), val);
  }
  else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'R')
  { // BR
    manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
    __ADD_COMMAND(cmdQ, 6 + (manualMode ? 0 : 4), val);
  }
  else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'L')
    __ADD_COMMAND(cmdQ, 11, val); // TL turn left max
  else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'R')
    __ADD_COMMAND(cmdQ, 12, val); // TR turn right max
  else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'T')
    __ADD_COMMAND(cmdQ, 14, val); // DT move until specifie d distance from obstacle
  else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'Z')
    __ADD_COMMAND(cmdQ, 15, val); // DZ move until specified distance from obstacle with faster speed
  else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'A')
    __ADD_COMMAND(cmdQ, 88, val); // forward anti-clockwise rotation with variable
  else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'C')
    __ADD_COMMAND(cmdQ, 89, val); // forward clockwise rotation with variable
  else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'A')
    __ADD_COMMAND(cmdQ, 90, val); // backward anti-clockwise rotation with variable
  else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'C')
    __ADD_COMMAND(cmdQ, 91, val); // backward clockwise rotation with variable
  else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'A')
    __ADD_COMMAND(cmdQ, 92, val); // Turn A - Task 2 1st obstacle
  else if (aRxBuffer[0] == 'I' && aRxBuffer[1] == 'R')
    __ADD_COMMAND(cmdQ, 93, val); // Robot move until IR overshoot - for debugging only
  else if (aRxBuffer[0] == 'I' && aRxBuffer[1] == 'C')
    __ADD_COMMAND(cmdQ, 94, val); // Robot move until IR detect close distance obstacle - for debugging only
  else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'B')
    __ADD_COMMAND(cmdQ, 95, val); // Turn B - Task 2 2nd obstacle
  else if (aRxBuffer[0] == 'G' && aRxBuffer[1] == 'H')
    __ADD_COMMAND(cmdQ, 96, val); // Go Home - Task 2 go back to carpark
  else if (aRxBuffer[0] == 'P' && aRxBuffer[1] == 'P')
    __ADD_COMMAND(cmdQ, 97, val); //Pre-pare
  if (!__COMMAND_QUEUE_IS_EMPTY(cmdQ))
  {
    __READ_COMMAND(cmdQ, curCmd, rxMsg);
  }

  // clear aRx buffer
  __HAL_UART_FLUSH_DRREGISTER(&huart3);
  HAL_UART_Receive_IT(&huart3, aRxBuffer, RX_BUFFER_SIZE);
}

// pid
void PIDConfigInit(PIDConfig *cfg, const float Kp, const float Ki, const float Kd)
{
  cfg->Kp = Kp;
  cfg->Ki = Ki;
  cfg->Kd = Kd;
  cfg->ek1 = 0;
  cfg->ekSum = 0;
}

void PIDConfigReset(PIDConfig *cfg)
{
  cfg->ek1 = 0;
  cfg->ekSum = 0;
}

void HCSR04_Read(void)
{
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
  __delay_us(&htim6, 50);
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_SET);   // pull the TRIG pin HIGH
  __delay_us(&htim6, 10);                                            // wait for 10 us
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET); // pull the TRIG pin LOW
  __delay_us(&htim6, 50);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);
}

int8_t dir = 1;
int correction = 0;

void StraightMovement(const uint8_t speedMode)
{

  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ); // polling
  dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
  angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;s

  if (speedMode == SPEED_MODE_T)
    __PID_SPEED_T(pidTSlow, angleNow, correction, dir, newDutyL, newDutyR);
  else if (speedMode == SPEED_MODE_3)
    __PID_SPEED_3(pidZoomZoom, angleNow, correction, dir, newDutyL, newDutyR);
  else if (speedMode == SPEED_MODE_2)
    __PID_SPEED_2(pidFast, angleNow, correction, dir, newDutyL, newDutyR);
  else if (speedMode == SPEED_MODE_1)
    __PID_SPEED_1(pidSlow, angleNow, correction, dir, newDutyL, newDutyR);

  __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
}

/**
 * @brief Moves the robot a certain distance in a specified direction and speed mode.
 *
 * @param targetDist Pointer to the target distance to move.
 * @param dir The direction to move the robot in.
 * @param speedMode The speed mode to use for the movement.
 */
void RobotMoveDistance(float *targetDist, const uint8_t dir, const uint8_t speedMode)
{
  angleNow = 0;
  gyroZ = 0; // reset angle for PID
  PIDConfigReset(&pidTSlow);
  PIDConfigReset(&pidSlow);
  PIDConfigReset(&pidFast);
  PIDConfigReset(&pidZoomZoom);
  curDistTick = 0;
  dist_dL = 0;
  __GET_TARGETTICK(*targetDist, targetDistTick); //targetDist +5 for better acc

  last_curTask_tick = HAL_GetTick();
  __SET_MOTOR_DIRECTION(dir);
  __SET_ENCODER_LAST_TICK(&htim2, lastDistTick_L);
  do
  {

    __GET_ENCODER_TICK_DELTA(&htim2, lastDistTick_L, dist_dL);
    curDistTick += dist_dL;
    if (curDistTick >= targetDistTick)
      break;

    if (HAL_GetTick() - last_curTask_tick >= 10)
    {
      if (speedMode == SPEED_MODE_T)
      {
        StraightMovement(SPEED_MODE_T);
      }
      else
      {
        speedScale = abs(curDistTick - targetDistTick) / 990; // start to slow down at last 990 ticks (15cm)
        if (speedMode == SPEED_MODE_1)
          speedScale = speedScale > 1 ? 1 : (speedScale < 0.75 ? 0.75 : speedScale); //0.75
        else if (speedMode == SPEED_MODE_2)
          speedScale = speedScale > 1 ? 1 : (speedScale < 0.4 ? 0.4 : speedScale); //0.4
        else if (speedMode == SPEED_MODE_3)
          speedScale = speedScale > 1 ? 1 : (speedScale < 0.3 ? 0.3 : speedScale); //0.3
        StraightMovementWithSpeedScale(speedMode, &speedScale);
      }

      last_curTask_tick = HAL_GetTick();
    }
  } while (1);
  __SET_MOTOR_DUTY(&htim8, 0, 0);
}


void StraightMovementWithSpeedScale(const uint8_t speedMode, float *speedScale)
{
  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);            // polling
  dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1 : -1;  // use only one of the wheel to determine car direction
  angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
  if (speedMode == SPEED_MODE_1)
    __PID_SPEED_1(pidSlow, angleNow, correction, dir, newDutyL, newDutyR);
  else if (speedMode == SPEED_MODE_2)
    __PID_SPEED_2(pidFast, angleNow, correction, dir, newDutyL, newDutyR);
  else if (speedMode == SPEED_MODE_3)
    __PID_SPEED_3(pidZoomZoom, angleNow, correction, dir, newDutyL, newDutyR);

  __SET_MOTOR_DUTY(&htim8, newDutyL * (*speedScale), newDutyR * (*speedScale));
}

void RobotTurn(float *targetAngle)
{
  angleNow = 0;
  gyroZ = 0;
  last_curTask_tick = HAL_GetTick();
  do
  {
    if (HAL_GetTick() - last_curTask_tick >= 10)
    { // sample gyro every 5ms
      __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
      angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
      if (abs(angleNow - *targetAngle) < 0.01)
        break;
      last_curTask_tick = HAL_GetTick();
    }
  } while (1);
  __SET_MOTOR_DUTY(&htim8, 0, 0);
  __RESET_SERVO_TURN(&htim1);
}

// RobotMoveDistanceObstacle must be called within a task(eg. runFastestPath) and not within an interrupt(eg. UART, EXTI)
// else osDelay won't work and TRI's timer interrupt can't be given chance to update obsDist_US
void RobotMoveDistanceObstacle(float *targetDist, const uint8_t speedMode)
{
  angleNow = 0;
  gyroZ = 0;
  PIDConfigReset(&pidTSlow);
  PIDConfigReset(&pidSlow);
  PIDConfigReset(&pidFast);
  PIDConfigReset(&pidZoomZoom);
  obsDist_Ultrasonic = 1000;
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); // Ultrasonic sensor start
  last_curTask_tick = HAL_GetTick();
  do
  {
    HCSR04_Read();
    osDelay(10); // give timer interrupt chance to update obsDist_US value
    if (abs(*targetDist - obsDist_Ultrasonic) <= 1)
      break;
    __SET_MOTOR_DIRECTION(obsDist_Ultrasonic >= *targetDist);
    if (HAL_GetTick() - last_curTask_tick >= 20)
    {
      if (speedMode == SPEED_MODE_1)
      {
        speedScale = abs(obsDist_Ultrasonic - *targetDist) / 15; // slow down at 15cm
        speedScale = speedScale > 1 ? 1 : (speedScale < 0.75 ? 0.75 : speedScale);
        if (abs(*targetDist - obsDist_Ultrasonic) <= 2) // slow down when the distance between obstacle and robot is less than 2cm
          speedScale *= 0.5;
        else if ((abs(*targetDist - obsDist_Ultrasonic) > 2) && (abs(*targetDist - obsDist_Ultrasonic) <= 5)) // slow down when the distance between obstacle and robot is less than 5cm
          speedScale *= 0.75;
        StraightMovementWithSpeedScale(SPEED_MODE_1, &speedScale);

        /*if (counter > 5)
          break;
        if (speedScale > oldspeed && speedScale > 0)
        {
          oldspeed = speedScale;
          counter++;
        }
        else if (speedScale < 0 && oldspeed > speedScale){
          counter++;
          oldspeed = speedScale;
        }*/
      }
      else if (speedMode == SPEED_MODE_2)
      {
        speedScale = abs(obsDist_Ultrasonic - *targetDist) / 15; // slow down at 15cm
        speedScale = speedScale > 1 ? 1 : (speedScale < 0.4 ? 0.4 : speedScale);
        if (abs(*targetDist - obsDist_Ultrasonic) <= 2) // slow down when the distance between obstacle and robot is less than 5cm
          speedScale *= 0.5;
        else if ((abs(*targetDist - obsDist_Ultrasonic) > 2) && (abs(*targetDist - obsDist_Ultrasonic) <= 5)) // slow down when the distance between obstacle and robot is less than 5cm
          speedScale *= 0.75;
        StraightMovementWithSpeedScale(SPEED_MODE_2, &speedScale);
      }
      else if (speedMode == SPEED_MODE_3)
      {
        speedScale = abs(obsDist_Ultrasonic - *targetDist) / 15; // slow down at 15cm
        speedScale = speedScale > 1 ? 1 : (speedScale < 0.4 ? 0.4 : speedScale);
        if (abs(*targetDist - obsDist_Ultrasonic) <= 2) // slow down when the distance between obstacle and robot is less than 5cm
          speedScale *= 0.5;
        else if ((abs(*targetDist - obsDist_Ultrasonic) > 2) && (abs(*targetDist - obsDist_Ultrasonic) <= 5)) // slow down when the distance between obstacle and robot is less than 5cm
          speedScale *= 0.75;
        StraightMovementWithSpeedScale(SPEED_MODE_3, &speedScale);
      }

      last_curTask_tick = HAL_GetTick();
    }

  } while (1);

  __SET_MOTOR_DUTY(&htim8, 0, 0);
  HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
}

/**
 * @brief Moves the robot until it overshoots the IR sensor reading.
 *
 * @param isIR_R Determines whether to use the right or left IR sensor.
 *                1 for right, 0 for left.
 */
void RobotMoveUntilIROvershoot(int isIR_R)
{
  PIDConfigReset(&pidTSlow);
  PIDConfigReset(&pidSlow);
  PIDConfigReset(&pidFast);
  PIDConfigReset(&pidZoomZoom);
  obsDist_IR_R = 0;
  obsDist_IR_L = 0;
  angleNow = 0;
  gyroZ = 0;
  last_curTask_tick = HAL_GetTick();
  if (isIR_R)
  {
    do
    {
      __ADC_Read_Dist_R(&hadc1, dataPoint_R, IR_data_raw_acc_R, obsDist_IR_R, obsTick_IR_R);
      osDelay(20);
      if (obsDist_IR_R > 35)
        break;
      if (HAL_GetTick() - last_curTask_tick >= 10)
      {
        OLED_ShowNumber(0, 0, obsDist_IR_R, 5, 12);
        __SET_MOTOR_DIRECTION(DIR_FORWARD);
        StraightMovement(SPEED_MODE_2);
        last_curTask_tick = HAL_GetTick();
      }

    } while (1);
    __SET_MOTOR_DUTY(&htim8, 0, 0);
    HAL_ADC_Stop(&hadc1);
  }

  else
  {
    do
    {
      __ADC_Read_Dist_L(&hadc2, dataPoint_L, IR_data_raw_acc_L, obsDist_IR_L, obsTick_IR_L);
      osDelay(20);
      if (obsDist_IR_L > 35)
        break;
      if (HAL_GetTick() - last_curTask_tick >= 10)
      {
        OLED_ShowNumber(0, 0, obsDist_IR_L, 5, 12);
        __SET_MOTOR_DIRECTION(DIR_FORWARD);
        StraightMovement(SPEED_MODE_2);
        last_curTask_tick = HAL_GetTick();
      }

    } while (1);
    __SET_MOTOR_DUTY(&htim8, 0, 0);
    HAL_ADC_Stop(&hadc2);
  }
}

/**
 * @brief Moves the robot until the IR sensor detects an obstacle at a close distance (25cm).
 *
 * @param isIR_R Flag to indicate if the right IR sensor is being used.
 *                1 if right IR sensor is being used, 0 if left IR sensor is being used.
 */
void RobotMoveUntilIRCloseDist(int isIR_R)
{
  PIDConfigReset(&pidTSlow);
  PIDConfigReset(&pidSlow);
  PIDConfigReset(&pidSlow);
  PIDConfigReset(&pidZoomZoom);
  obsDist_IR_R = 0xFF;
  obsDist_IR_L = 0xFF;
  angleNow = 0;
  gyroZ = 0;
  last_curTask_tick = HAL_GetTick();

  if (isIR_R)
  {
    do
    {
      __ADC_Read_Dist_R(&hadc1, dataPoint_R, IR_data_raw_acc_R, obsDist_IR_R, obsTick_IR_R);
      // OLED_ShowNumber(0, 0, obsDist_IR_R, 5, 12);
      if (HAL_GetTick() - last_curTask_tick >= 10)
      {
        __SET_MOTOR_DIRECTION(DIR_FORWARD);
        StraightMovement(SPEED_MODE_1);
        last_curTask_tick = HAL_GetTick();
      }

    } while (obsDist_IR_R >= 25);
    __SET_MOTOR_DUTY(&htim8, 0, 0);
    HAL_ADC_Stop(&hadc1);
  }

  else
  {
    do
    {
      __ADC_Read_Dist_L(&hadc2, dataPoint_L, IR_data_raw_acc_L, obsDist_IR_L, obsTick_IR_L);
      // OLED_ShowNumber(0, 0, obsDist_IR_L, 5, 12);
      if (HAL_GetTick() - last_curTask_tick >= 10)
      {
        __SET_MOTOR_DIRECTION(DIR_FORWARD);
        StraightMovement(SPEED_MODE_1);
        last_curTask_tick = HAL_GetTick();
      }

    } while (obsDist_IR_L >= 25);
    __SET_MOTOR_DUTY(&htim8, 0, 0);
    HAL_ADC_Stop(&hadc2);
  }
}

void RobotTurnFC45()
{
  //  FC45
  targetAngle = -(45 - FCOFFSET);
  __SET_MOTOR_DUTY(&htim8, 2000, 1333);
  __SET_SERVO_TURN_MAX(&htim1, 1);
  __SET_MOTOR_DIRECTION(DIR_FORWARD);
  RobotTurn(&targetAngle);
  osDelay(300); // reset wheel
}
void RobotTurnFC60()
{
  //  FC60
  targetAngle = -(60 - FCOFFSET);
  __SET_MOTOR_DUTY(&htim8, 2000, 1333);
  __SET_SERVO_TURN_MAX(&htim1, 1);
  __SET_MOTOR_DIRECTION(DIR_FORWARD);
  RobotTurn(&targetAngle);
  osDelay(100); // reset wheel
}
void RobotTurnFC90()
{
  //  FC90
  targetAngle = -(90 - FCOFFSET);
  __SET_MOTOR_DUTY(&htim8, 2000, 1333);
  __SET_SERVO_TURN_MAX(&htim1, 1);
  __SET_MOTOR_DIRECTION(DIR_FORWARD);
  RobotTurn(&targetAngle);
  osDelay(300); // reset wheel
}
void RobotTurnFC180()
{
  //  FC180
  targetAngle = -(180 - FCOFFSET);
  __SET_MOTOR_DUTY(&htim8, 2000, 1333);
  __SET_SERVO_TURN_MAX(&htim1, 1);
  __SET_MOTOR_DIRECTION(DIR_FORWARD);
  RobotTurn(&targetAngle);
  osDelay(300); // reset wheel
}

void RobotTurnFA45()
{
  // FA45
  targetAngle = (45 + FAOFFSET);
  __SET_MOTOR_DUTY(&htim8, 1333, 2000);
  __SET_SERVO_TURN(&htim1, 90);
  __SET_MOTOR_DIRECTION(DIR_FORWARD);
  RobotTurn(&targetAngle);
  osDelay(300); // reset wheel
}
void RobotTurnFA60()
{
  // FA45
  targetAngle = (60 + FAOFFSET);
  __SET_MOTOR_DUTY(&htim8, 1333, 2000);
  __SET_SERVO_TURN(&htim1, 90);
  __SET_MOTOR_DIRECTION(DIR_FORWARD);
  RobotTurn(&targetAngle);
  osDelay(100); // reset wheel
}
void RobotTurnFA90()
{
  // FA90
  targetAngle = (90 + FAOFFSET);
  __SET_MOTOR_DUTY(&htim8, 1333, 2000);
  __SET_SERVO_TURN(&htim1, 90);
  __SET_MOTOR_DIRECTION(DIR_FORWARD);
  RobotTurn(&targetAngle);
  osDelay(300); // reset wheel
}
void RobotTurnFA180()
{
  // FA180
  targetAngle = (180 + FAOFFSET);
  __SET_MOTOR_DUTY(&htim8, 1333, 2000);
  __SET_SERVO_TURN(&htim1, 90);
  __SET_MOTOR_DIRECTION(DIR_FORWARD);
  RobotTurn(&targetAngle);
  osDelay(300); // reset wheel
}

// For Task 2 Turn B - outdoor
void RobotTurnFR30()
{
  targetDist = 2; //7
  RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
  __SET_CMD_CONFIG(cfgs[CONFIG_FR30], &htim8, &htim1, targetAngle);
  RobotTurn(&targetAngle);
  targetDist = 1; //2
  RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
}
void RobotTurnFL30()
{
  targetDist = 7; //3.5
  RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
  __SET_CMD_CONFIG(cfgs[CONFIG_FL30], &htim8, &htim1, targetAngle);
  RobotTurn(&targetAngle);
  targetDist = 3; //4
  RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
}

// For Task 2 Turn B - indoor
void RobotTurnFL00()
{
  targetDist = 6; //8
  RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
  __SET_CMD_CONFIG(cfgs[CONFIG_FL00], &htim8, &htim1, targetAngle);
  RobotTurn(&targetAngle);
  targetDist = 3; //2
  RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
}

void RobotTurnFR00()
{
  targetDist = 2.5; //3.5
  RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
  __SET_CMD_CONFIG(cfgs[CONFIG_FR00], &htim8, &htim1, targetAngle);
  RobotTurn(&targetAngle);
  targetDist = 1.5; //4
  RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_runEncoder */
/**
 * @brief  Function implementing the encoderTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_runEncoder */
void runEncoder(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(100);
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_runOledTask */
/**
 * @brief Function implementing the OledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runOledTask */
void runOledTask(void *argument)
{
  /* USER CODE BEGIN runOledTask */
  /* Infinite loop */

  for (;;)
  {

    // angle debugging
    //  angleTemp = angleNow / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
    //  snprintf(ch, sizeof(ch), "angle:%-4d", (int)angleTemp);
    //  OLED_ShowString(0, 40, (char *)ch);

    // uart received cmd debugging
    // __ACK_TASK_DONE(&huart3, rxMsg);
    // HAL_UART_Transmit(&huart3, aRxBuffer, RX_BUFFER_SIZE, 0xFFFF);
    // IR_data_raw_acc_R = HAL_ADC_GetValue(&hadc1);
    // IR_data_raw_acc_L = HAL_ADC_GetValue(&hadc2);

    // ir debugging
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc2, 20);
    HAL_ADC_PollForConversion(&hadc1, 20);
    // IR_data_raw_acc_R = HAL_ADC_GetValue(&hadc1);
    // OLED_ShowNumber(0, 10, IR_data_raw_acc_R, 20, 12);
    // HAL_UART_Transmit(&huart3, (uint8_t *)IR_data_raw_acc_R, 4, 0xFFFF);
    // HAL_ADC_Stop(&hadc1);
    // HAL_ADC_Start(&hadc2);
    // HAL_ADC_PollForConversion(&hadc2, 20);
    // IR_data_raw_acc_L = HAL_ADC_GetValue(&hadc2);
    OLED_ShowNumber(0, 20, HAL_ADC_GetValue(&hadc2), 5, 12);
    OLED_ShowNumber(60, 20, HAL_ADC_GetValue(&hadc1), 5, 12);

    // char temp[10];
    // snprintf((char *)temp, sizeof(temp) - 1, "%d\n", HAL_ADC_GetValue(&hadc2));
    // HAL_UART_Transmit(&huart3, (uint8_t *)temp, strlen(temp), 0xFFFF);

    HAL_ADC_Stop(&hadc2);
    HAL_ADC_Stop(&hadc1);

    // __ADC_Read_Dist_R(&hadc1, dataPoint_R, IR_data_raw_acc_R, obsDist_IR_R, obsTick_IR_R);
    // OLED_ShowNumber(0, 30, obsDist_IR_R, 20, 12);
    // HAL_ADC_Stop(&hadc1);

    // HAL_ADC_Stop(&hadc2);

    // us debugging
    HCSR04_Read();
    OLED_ShowNumber(0, 0, obsDist_Ultrasonic, 5, 12);

    // display current command
    OLED_ShowString(0, 40, (char *)aRxBuffer);

    OLED_Refresh_Gram();
    osDelay(100);
  }
  /* USER CODE END runOledTask */
}

/* USER CODE BEGIN Header_runFWTask */
/**
 * @brief Function implementing the FWTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runFWTask */
void runFWTask(void *argument)
{
  /* USER CODE BEGIN runFWTask */
  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_MOVE_FORWARD)
      osDelay(1000);
    else
    {
      targetDist = 0;
      targetDistTick = 0;
      if (manualMode)
      {

        angleNow = 0;
        gyroZ = 0; // reset angle for PID
        PIDConfigReset(&pidTSlow);
        PIDConfigReset(&pidSlow);
        PIDConfigReset(&pidFast);
        PIDConfigReset(&pidZoomZoom);

        __SET_MOTOR_DIRECTION(DIR_FORWARD);




        __ON_TASK_END(&htim8, prevTask, curTask);

        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);

        last_curTask_tick = HAL_GetTick();
        do
        {
          if (!manualMode)
            break;
          if (HAL_GetTick() - last_curTask_tick >= 10)
          {

            StraightMovement(SPEED_MODE_T);
            last_curTask_tick = HAL_GetTick();
          }

        } while (1);
      }
      else
      {

        targetDist = (float)curCmd.val;
        // for target distance lesser than 10, move mode must be forced to SLOW
        if (targetDist <= 15)
          moveMode = SLOW;

        //if (targetDist >= 100)
          //targetDist -= 2;

        if (moveMode == SLOW)
        {
          RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        }
        else
        {
          RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_2);
        }

        __ON_TASK_END(&htim8, prevTask, curTask);

        if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
        {
          __CLEAR_CURCMD(curCmd);
          __ACK_TASK_DONE(&huart3, rxMsg);
        }
        else
        {
        	__ACK_TASK_DONE(&huart3, rxMsg);
        	__READ_COMMAND(cmdQ, curCmd, rxMsg);
        }

      }
    }
  }

  /* USER CODE END runFWTask */
}

/* USER CODE BEGIN Header_runBWTask */
/**
 * @brief Function implementing the BWTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runBWTask */
void runBWTask(void *argument)
{
  /* USER CODE BEGIN runBWTask */

  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_MOVE_BACKWARD)
      osDelay(1000);
    else
    {
      targetDist = 0;
      targetDistTick = 0;
      if (manualMode)
      {

        angleNow = 0;
        gyroZ = 0; // reset angle for PID
        PIDConfigReset(&pidTSlow);
        PIDConfigReset(&pidSlow);
        PIDConfigReset(&pidFast);
        PIDConfigReset(&pidZoomZoom);

        __SET_MOTOR_DIRECTION(DIR_BACKWARD);

        __ON_TASK_END(&htim8, prevTask, curTask);

        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);

        last_curTask_tick = HAL_GetTick();
        do
        {
          if (!manualMode)
            break;
          if (HAL_GetTick() - last_curTask_tick >= 10)
          {
            StraightMovement(SPEED_MODE_T);
            last_curTask_tick = HAL_GetTick();
          }

        } while (1);
      }
      else
      {

        targetDist = (float)curCmd.val; //(float)curCmd.val+1;

        //if(targetDist > 20)
        	//targetDist -= 1;
        // for target distance lesser than 15, move mode must be forced to SLOW
        if (targetDist <= 15)
          moveMode = SLOW;
        if (moveMode == SLOW)
        {
          RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        }
        else
        {
          RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_2);
        }

        __ON_TASK_END(&htim8, prevTask, curTask);

        if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
        {
          __CLEAR_CURCMD(curCmd);
          //added for missing delay
          //if(targetDist <= 30)
          //{
        	  //osDelay(100);
          //}
          __ACK_TASK_DONE(&huart3, rxMsg);
        }
        else
        {
        	__ACK_TASK_DONE(&huart3, rxMsg);
        	__READ_COMMAND(cmdQ, curCmd, rxMsg);
        }

      }
    }
  }
  /* USER CODE END runBWTask */
}

/* USER CODE BEGIN Header_runFLTask */
/**
 * @brief Function implementing the FLTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runFLTask */
void runFLTask(void *argument)
{
  /* USER CODE BEGIN runFLTask */
	//toedit
  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_FL)
      osDelay(1000);
    else
    {

      switch (curCmd.val)
      {
      case 30: // FL30 (outdoor 3x2)
        targetDist = 7; //8
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        __SET_CMD_CONFIG(cfgs[CONFIG_FL30], &htim8, &htim1, targetAngle);
        RobotTurn(&targetAngle);
        targetDist = 3; //3
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        break;
      case 00: // FL00 (indoor 3x2)
        targetDist = 6;
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        __SET_CMD_CONFIG(cfgs[CONFIG_FL00], &htim8, &htim1, targetAngle);
        RobotTurn(&targetAngle);
        targetDist = 3;
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        break;
      }
      prevTask = curTask;
      curTask = TASK_NONE;
      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
  }
  /* USER CODE END runFLTask */
}

/* USER CODE BEGIN Header_runFRTask */
/**
 * @brief Function implementing the FRTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runFRTask */
void runFRTask(void *argument)
{
  /* USER CODE BEGIN runFRTask */
	//toedit
  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_FR)
      osDelay(1000);
    else
    {
      switch (curCmd.val)
      {
      case 30: // FR30 (outdoor 3x2)
        targetDist = 2; //4
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        __SET_CMD_CONFIG(cfgs[CONFIG_FR30], &htim8, &htim1, targetAngle);
        RobotTurn(&targetAngle);
        targetDist = 1; // if too little change back to 1.5 1
        RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        break;
      case 00: // FR00 (indoor 3x2)
        targetDist = 2.5;
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        __SET_CMD_CONFIG(cfgs[CONFIG_FR00], &htim8, &htim1, targetAngle);
        RobotTurn(&targetAngle);
        targetDist = 1.5;
        RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        break;
      }

      prevTask = curTask;
      curTask = TASK_NONE;
      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
  }
  /* USER CODE END runFRTask */
}

/* USER CODE BEGIN Header_runBLTask */
/**
 * @brief Function implementing the BLTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runBLTask */
void runBLTask(void *argument)
{
  /* USER CODE BEGIN runBLTask */
	//toedit
  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_BL)
      osDelay(1000);
    else
    {

      switch (curCmd.val)
      {
      case 30: // BL30 (outdoor 3x2)
        targetDist = 4; //3.5
        RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        __SET_CMD_CONFIG(cfgs[CONFIG_BL30], &htim8, &htim1, targetAngle);
        RobotTurn(&targetAngle);
        targetDist = 8; //7
        RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        break;
      case 00: // BL00 (indoor 3x2)
        targetDist = 2.5;
        RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        __SET_CMD_CONFIG(cfgs[CONFIG_BL00], &htim8, &htim1, targetAngle);
        RobotTurn(&targetAngle);
        targetDist = 8.5;
        RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        break;
      }
      prevTask = curTask;
      curTask = TASK_NONE;
      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
  }
  /* USER CODE END runBLTask */
}

/* USER CODE BEGIN Header_runBRTask */
/**
 * @brief Function implementing the BRTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runBRTask */
void runBRTask(void *argument)
{
  /* USER CODE BEGIN runBRTask */
	//toedit
  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_BR)
      osDelay(1000);
    else
    {

      switch (curCmd.val)
      {
      case 30: // BR30 (outdoor 3x2)
        targetDist = 1.5; //2
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        __SET_CMD_CONFIG(cfgs[CONFIG_BR30], &htim8, &htim1, targetAngle);
        RobotTurn(&targetAngle);
        targetDist = 1.5; //4
        RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        break;
      case 00: // BR00 (indoor 3x2)
        targetDist = 3.5;
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        __SET_CMD_CONFIG(cfgs[CONFIG_BR00], &htim8, &htim1, targetAngle);
        RobotTurn(&targetAngle);
        targetDist = 2;
        RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        break;
      }
      prevTask = curTask;
      curTask = TASK_NONE;
      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
  }
  /* USER CODE END runBRTask */
}

/* USER CODE BEGIN Header_runCmdTask */
/**
 * @brief Function implementing the cmdTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runCmdTask */
void runCmdTask(void *argument)
{
  /* USER CODE BEGIN runCmdTask */
  /* Infinite loop */
  for (;;)
  {
    switch (curCmd.index)
    {
    //case 0: // STOP handled in UART IRQ directly
    //  break;
    case 1: // FW
      curTask = TASK_MOVE_FORWARD;
      __PEND_CURCMD(curCmd);
      break;
    case 2: // BW
      curTask = TASK_MOVE_BACKWARD;
      __PEND_CURCMD(curCmd);
      break;
    case 3: // FL manual
    case 4: // FR manual
    case 5: // BL manual
    case 6: // BR manual
      __SET_CMD_CONFIG(cfgs[curCmd.index], &htim8, &htim1, targetAngle);
      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
      __PEND_CURCMD(curCmd);
      break;
    case 7: // FL
      curTask = TASK_FL;
      __PEND_CURCMD(curCmd);
      break;
    case 8: // FR
      curTask = TASK_FR;
      __PEND_CURCMD(curCmd);
      break;
    case 9: // BL
      curTask = TASK_BL;
      __PEND_CURCMD(curCmd);
      break;
    case 10: // BR
      curTask = TASK_BR;
      __PEND_CURCMD(curCmd);
      break;
    case 11: // TL
    case 12: // TR
      __SET_SERVO_TURN_MAX(&htim1, curCmd.index - 11 ? 1 : 0);
      __CLEAR_CURCMD(curCmd);
      __ACK_TASK_DONE(&huart3, rxMsg);
      break;
    case 14: // DT move until specified distance from obstacle
      curTask = TASK_MOVE_OBS;
      __PEND_CURCMD(curCmd);
      break;
    case 15: // DZ move until specified distance from obstacle, but with zoomzoom config
      curTask = TASK_MOVE_OBS_ZOOMZOOM;
      __PEND_CURCMD(curCmd);
      break;
    case 88: // FAxxx, forward rotate left by xxx degree
    case 89: // FCxxx, forward rotate right by xxx degree
      __SET_MOTOR_DIRECTION(DIR_FORWARD);
      if (curCmd.index == 88)
      {
        __SET_SERVO_TURN(&htim1, 90);
        if (curCmd.val >= 10)
          targetAngle = (curCmd.val + FAOFFSET);
        else
          targetAngle = curCmd.val;
        __SET_MOTOR_DUTY(&htim8, 1333, 2000); // 900, 3000
      }
      else
      {
        __SET_SERVO_TURN(&htim1, 265);
        if (curCmd.val >= 10)
          targetAngle = -(curCmd.val - FCOFFSET);
        else
          targetAngle = -curCmd.val;
        __SET_MOTOR_DUTY(&htim8, 2000, 1333); //2000, 1333
      }
      __PEND_CURCMD(curCmd);
      RobotTurn(&targetAngle);
      /*if(curCmd.index == 88)
      {
    	  __SET_SERVO_TURN(&htim1, 155);
      }
      else
      {
    	  __SET_SERVO_TURN(&htim1, 145);
      }*/
      __CLEAR_CURCMD(curCmd);
      __ACK_TASK_DONE(&huart3, rxMsg);
      break;
    case 90: // BAxxx, backward rotate right by xxx degree
    case 91: // BCxxx, backward rotate left by xxx degree
      __SET_SERVO_TURN_MAX(&htim1, (int)(!(curCmd.index - 90)));
      __SET_MOTOR_DIRECTION(DIR_BACKWARD);
      if (curCmd.index == 90)
      {

        targetAngle = curCmd.val;
        __SET_MOTOR_DUTY(&htim8, 2000, 1333);
      }
      else
      {
        targetAngle = -curCmd.val;
        __SET_MOTOR_DUTY(&htim8, 1333, 2000);
      }
      __PEND_CURCMD(curCmd);
      RobotTurn(&targetAngle);
      __CLEAR_CURCMD(curCmd);
      __ACK_TASK_DONE(&huart3, rxMsg);
      break;
    case 92: // TAxx, 01 turn right, 02 turn left --TASK 2
      curTask = TASK_TURN_A;
      __PEND_CURCMD(curCmd);
      break;
    case 93: // IR move until overshoot - for debugging only
      curTask = TASK_TURN_IR;
      __PEND_CURCMD(curCmd);
      break;
    case 94: // IR move until close to obstacle - for debugging only
      curTask = TASK_TURN_IR_CLOSE;
      __PEND_CURCMD(curCmd);
      break;
    case 95: // TBxx, 01 turn right, 02 turn left --TASK 2
      curTask = TASK_TURN_B;
      __PEND_CURCMD(curCmd);
      break;
    case 96: // GHxx, 01 from left (after TB01), 02 from right (after TB02),  --TASK 2
      curTask = TASK_GO_HOME;
      __PEND_CURCMD(curCmd);
      break;
    case 97: // PPxx, 01 from left (after TA01), 02 from right (after TA02),  --TASK 2
      curTask = TASK_PP;
      __PEND_CURCMD(curCmd);
      break;
    case 99:
      break;
    case 100:
      break;
    default:
      //		 curCmd.index = 99;
      break;
    }
    osDelay(1);
  }
  /* USER CODE END runCmdTask */
}

/* USER CODE BEGIN Header_runMoveDistObsTask */
/**
 * @brief Function implementing the moveDistObsTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runMoveDistObsTask */
void runMoveDistObsTask(void *argument)
{
  /* USER CODE BEGIN runMoveDistObsTask */
  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_MOVE_OBS)
      osDelay(1000);
    else
    {
      targetDist = (float)curCmd.val;
      RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_2); //originally SPEED_MODE_2

      __ON_TASK_END(&htim8, prevTask, curTask);

      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
    osDelay(1);
  }

  /* USER CODE END runMoveDistObsTask */
}

/* USER CODE BEGIN Header_runTurnATask */
/**
 * @brief Function implementing the turnATask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runTurnATask */
void runTurnATask(void *argument)
{
  /* USER CODE BEGIN runTurnATask */
  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_TURN_A)
      osDelay(1000);
    else
    {

      switch (curCmd.val)
      {
      case 01: // Turn A right outdoor:
        // DT25
        //targetDist = 27;
        //RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_3);
        //osDelay(100);
        // FC45
        RobotTurnFC45();
        osDelay(50);
        // FA45, parallel to obstacle
        RobotTurnFA45();
        //osDelay(50);
        // BW05
        //targetDist = 5;
        //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        break;

      case 02: // Turn A left outdoor:
        // DT25
        //targetDist = 27;
        //RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_3);
        //osDelay(50);
        // FA45
        RobotTurnFA45();
        osDelay(50);
        // FC45, parallel to obstacle
        RobotTurnFC45();
        //osDelay(50);
        // BW05
        //targetDist = 5;
        //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        break;

      case 03: // Turn A right indoor:
        // DT25
        //targetDist = 27;
        //RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_3);
        //osDelay(50);
        // FC45
        RobotTurnFC45();
        osDelay(50);
        // FA45, parallel to obstacle
        RobotTurnFA45();
        //osDelay(50);
        // BW05
        //targetDist = 5;
        //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        break;

      case 04: // Turn A left indoor:
        // DT25
        //targetDist = 27;
        //RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_3);
        //osDelay(50);
        // FA45
        RobotTurnFA45();
        osDelay(50);
        // FC45, parallel to obstacle
        RobotTurnFC45();
        //osDelay(50);
        // BW05
        //targetDist = 5;
        //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        break;
      }
      prevTask = curTask;
      curTask = TASK_NONE;
      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
    osDelay(1);
  }
  /* USER CODE END runTurnATask */
}

/* USER CODE BEGIN Header_runTurnBTask */
/**
 * @brief Function implementing the turnBTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runTurnBTask */
void runTurnBTask(void *argument)
{
  /* USER CODE BEGIN runTurnBTask */
  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_TURN_B)
      osDelay(1000);
    else
    {
      switch (curCmd.val)
      {
      case 01: // Turn B right outdoor:
          // DT30
          //targetDist = 27;
          //RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_3);
          //osDelay(100);
          // FR00
          RobotTurnFR30();
          //RobotTurnFC45();
          //RobotTurnFA45();
          osDelay(50);
          // IR01 (left IR, follow wall 1st)
          RobotMoveUntilIROvershoot(0);
          osDelay(50);
          //might need to move back
          // FL00
          //RobotTurnFA45();
          //RobotTurnFC45();
          RobotTurnFL30();
          osDelay(50);
          // 1st turn after sideway of obstacle
          // FW07 - avoidance of obstacle
          //targetDist = 7;
          //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
          //osDelay(50);
          // out of obstacle zone
          // move until obstacle detected
          RobotMoveUntilIROvershoot(0);
          osDelay(50);
          // FL00
          //RobotTurnFA45();
          //RobotTurnFA45();
          RobotTurnFL30();
          osDelay(50);
          // move until obstacle detected
          RobotMoveUntilIRCloseDist(0);
          osDelay(50);
          // IR01 (left IR)
          RobotMoveUntilIROvershoot(0);
          osDelay(50);
          // FW15
          //targetDist = 8;
          //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_1);
          //osDelay(50);
          // FL00
          RobotTurnFL30();
          //RobotTurnFA45();
          //RobotTurnFA45();
          osDelay(50);
        break;

      case 02: // Turn B left outdoor:
          // DT30
          //targetDist = 27;
          //RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_3);
          //osDelay(100);
          // FL00
          RobotTurnFL30();
          osDelay(50);
          // IR02 (right IR, follow wall 1st)
          RobotMoveUntilIROvershoot(1);
          osDelay(50);
          // FR00
          RobotTurnFR30();
          //RobotTurnFC45();
          //RobotTurnFC45();
          osDelay(50);
          // 1st turn after sideway of obstacle
          // FW07 - avoidance of obstacle
          //targetDist = 7;
          //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
          //osDelay(50);
          // out of obstacle zone
          // move until obstacle detected
          RobotMoveUntilIROvershoot(1);
          osDelay(50);
          // FR00
          RobotTurnFR30();
          osDelay(50);
          // move until obstacle detected
          RobotMoveUntilIRCloseDist(1);
          osDelay(50);
          // IR02 (right IR)
          RobotMoveUntilIROvershoot(1);
          osDelay(50);
          // FW15
          //targetDist = 8;
          //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_1);
          //osDelay(50);
          // FR00
          RobotTurnFR30();
          //RobotTurnFC45();
          //RobotTurnFC45();
          osDelay(50);
          break;

      case 03: // Turn B right indoor:
        // DT30
        //targetDist = 27;
        //RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_3);
        //osDelay(100);
        // FR00
        RobotTurnFR00();
        //RobotTurnFC45();
        //RobotTurnFA45();
        osDelay(50);
        // IR01 (left IR, follow wall 1st)
        RobotMoveUntilIROvershoot(0);
        osDelay(50);
        // FL00
        //RobotTurnFA45();
        //RobotTurnFC45();
        RobotTurnFL00();
        osDelay(50);
        // 1st turn after sideway of obstacle
        // FW07 - avoidance of obstacle
        //targetDist = 7;
        //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        //osDelay(50);
        // out of obstacle zone
        // move until obstacle detected
        RobotMoveUntilIROvershoot(0);
        osDelay(50);
        // FL00
        //RobotTurnFA45();
        //RobotTurnFA45();
        RobotTurnFL00();
        osDelay(50);
        // move until obstacle detected
        RobotMoveUntilIRCloseDist(0);
        osDelay(50);
        // IR01 (left IR)
        RobotMoveUntilIROvershoot(0);
        osDelay(50);
        // FW15
        //targetDist = 8;
        //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_1);
        //osDelay(50);
        // FL00
        RobotTurnFL00();
        //RobotTurnFA45();
        //RobotTurnFA45();
        osDelay(50);
        break;

      case 04: // Turn B left indoor:
/*			//testing
    	  //FC90
    	  RobotTurnFA90();
		  osDelay(50);
		  // IR02 (right IR, follow wall 1st)
		  RobotMoveUntilIROvershoot(1);
		  osDelay(50);
		  RobotTurnFC180();
		  osDelay(50);
		  RobotMoveUntilIRCloseDist(1);
		  RobotMoveUntilIROvershoot(1);
		  osDelay(50);
		  RobotTurnFC90();
*/
    	 //Old Code
        // DT30
        //targetDist = 27;
        //RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_3);
        //osDelay(100);
        // FL00
        RobotTurnFL00();
        osDelay(50);
        // IR02 (right IR, follow wall 1st)
        RobotMoveUntilIROvershoot(1);
        osDelay(50);
        // FR00
        RobotTurnFR00();
        //RobotTurnFC45();
        //RobotTurnFC45();
        osDelay(50);
        // 1st turn after sideway of obstacle
        // FW07 - avoidance of obstacle
        //targetDist = 7;
        //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        //osDelay(50);
        // out of obstacle zone
        // move until obstacle detected
        RobotMoveUntilIROvershoot(1);
        osDelay(50);
        // FR00
        RobotTurnFR00();
        osDelay(50);
        // move until obstacle detected
        RobotMoveUntilIRCloseDist(1);
        osDelay(50);
        // IR02 (right IR)
        RobotMoveUntilIROvershoot(1);
        osDelay(50);
        // FW15
        //targetDist = 8;
        //RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_1);
        //osDelay(50);
        // FR00
        RobotTurnFR00();
        //RobotTurnFC45();
        //RobotTurnFC45();
        osDelay(50);
        break;
      }
      prevTask = curTask;
      curTask = TASK_NONE;
      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
    osDelay(1);
  }
  /* USER CODE END runTurnBTask */
}

/* USER CODE BEGIN Header_runGHTask */
/**
 * @brief Function implementing the GHTask thread.
 * Need to run Turn A before GH to memorize obstacle B distance
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runGHTask */
void runGHTask(void *argument)
{
  /* USER CODE BEGIN runGHTask */
  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_GO_HOME)
      osDelay(1000);
    else
    {
      obsDist_B += 75; //80
      if (obsDist_B < 1000)
      {
        switch (curCmd.val)
        {
        case 01: // Move after Turn B right outdoor
          // move to obs A location +75 cm
          RobotMoveDistance(&obsDist_B, DIR_FORWARD, SPEED_MODE_3);
          osDelay(50);
          // FL30
          RobotTurnFL30();
          osDelay(50);
          // stop when IR detects obs
          RobotMoveUntilIRCloseDist(0);
          // BW15
          targetDist = 15;
          RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
          // FR30
          RobotTurnFR30();
          targetDist = 10;
          RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_2);
          break;

        case 02: // Move after Turn B left outdoor
          // move to obs A location +75 cm
          RobotMoveDistance(&obsDist_B, DIR_FORWARD, SPEED_MODE_3);
          osDelay(50);
          // FR30
          RobotTurnFR30();
          osDelay(50);
          // stop when IR detects obs
          RobotMoveUntilIRCloseDist(1);
          // BW15
          targetDist = 15;
          RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
          // FL30
          RobotTurnFL30();
          targetDist = 10;
          RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_2);
          break;

        case 03: // Move after Turn B right indoor
          // move to obs A location +75 cm
          RobotMoveDistance(&obsDist_B, DIR_FORWARD, SPEED_MODE_3);
          osDelay(50);
          // FL00
          RobotTurnFL00();
          osDelay(50);
          // stop when IR detects obs
          RobotMoveUntilIRCloseDist(0);
          // BW15
          targetDist = 15;
          RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
          // FR00
          RobotTurnFR00();
          targetDist = 10;
          RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_2);
          break;

        case 04: // Move after Turn B left indoor
          // move to obs A location +75 cm
          RobotMoveDistance(&obsDist_B, DIR_FORWARD, SPEED_MODE_3);
          osDelay(50);
          // FR00
          RobotTurnFR00();
          osDelay(50);
          // stop when IR detects obs
          RobotMoveUntilIRCloseDist(1);
          // BW15
          targetDist = 15;
          RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
          // FL00
          RobotTurnFL00();
          targetDist = 10;
          RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_3);
          break;
        }
      }

      prevTask = curTask;
      curTask = TASK_NONE;
      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
    osDelay(1);
  }
  /* USER CODE END runGHTask */
}

/* USER CODE BEGIN Header_runPPTask */
/**
 * @brief Function implementing the PPTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runPPTask */
void runPPTask(void *argument)
{
  /* USER CODE BEGIN runPPTask */
  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_PP)
      osDelay(1000);
    else
    {

      switch (curCmd.val)
      {
      case 01: // PP right outdoor - use right after Turn A right outdoor:
        // FW5
        targetDist = 6; //5
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        //  FA60
        RobotTurnFA45();
        osDelay(50);
        // FC60
        RobotTurnFC45();
        osDelay(50);
        //BW15
        //targetDist = 15;
        //RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        // save obstacle B distance for go home (GH) command
        obsDist_Ultrasonic = 1000;
        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
        HCSR04_Read();
        osDelay(100);
        if (obsDist_Ultrasonic < 27 || obsDist_Ultrasonic > 500){
        	//BW15
        	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
        	HCSR04_Read();
        	osDelay(100);
        	targetDist = 20;
        	RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_1);

        }
        //osDelay(100);
        obsDist_B = obsDist_Ultrasonic;
        OLED_ShowNumber(0, 50, obsDist_Ultrasonic, 5, 12);
        HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
        break;

      case 02: // PP left outdoor - use right after Turn A left outdoor:
        // FW5
        targetDist = 6; //5
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        // FC45
        RobotTurnFC45();
        osDelay(50);
        // FA45
        RobotTurnFA45();
        osDelay(50);
        //BW15
        //targetDist = 15;
        //RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        // save obstacle B distance for go home (GH) command
        obsDist_Ultrasonic = 1000;
        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
        HCSR04_Read();
        osDelay(100);
        if (obsDist_Ultrasonic < 27 || obsDist_Ultrasonic > 500){
        	//BW15
        	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
        	HCSR04_Read();
        	osDelay(100);
        	targetDist = 20;
        	RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_1);
        }
        osDelay(100);
        obsDist_B = obsDist_Ultrasonic;
        OLED_ShowNumber(0, 50, obsDist_Ultrasonic, 5, 12);
        HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
        break;

      case 03: // PP right indoor - use right after Turn A right indoor:
        // FW5
        targetDist = 5;
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        //  FA60
        RobotTurnFA45();
        osDelay(50);
        // FC60
        RobotTurnFC45();
        osDelay(50);
        // BW15
        //targetDist = 15;
        //RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
        // save obstacle B distance for go home (GH) command
        obsDist_Ultrasonic = 1000;
        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
        HCSR04_Read();
        osDelay(100);
        if (obsDist_Ultrasonic < 27 || obsDist_Ultrasonic > 500){
        	//BW15
        	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
        	HCSR04_Read();
        	osDelay(100);
        	targetDist = 20;
        	RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_1);

        }
        obsDist_B = obsDist_Ultrasonic;
        OLED_ShowNumber(0, 50, obsDist_Ultrasonic, 5, 12);
        HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
        break;

      case 04: // PP left indoor - use right after Turn A left indoor:
        // FW5
        targetDist = 5;
        RobotMoveDistance(&targetDist, DIR_FORWARD, SPEED_MODE_T);
        // FC45
        RobotTurnFC45();
        osDelay(50);
        // FA45
        RobotTurnFA45();
        osDelay(50);
        //BW15
        //targetDist = 20;
        //RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_1);
        // save obstacle B distance for go home (GH) command
        obsDist_Ultrasonic = 1000;
        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
        HCSR04_Read();
        osDelay(100);
        if (obsDist_Ultrasonic < 27 || obsDist_Ultrasonic > 500){
        	//BW15
        	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
        	HCSR04_Read();
        	osDelay(100);
        	targetDist = 20;
        	RobotMoveDistance(&targetDist, DIR_BACKWARD, SPEED_MODE_1);

        }

        obsDist_B = obsDist_Ultrasonic;
        OLED_ShowNumber(0, 50, obsDist_Ultrasonic, 5, 12);
        HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
        break;
      }

      prevTask = curTask;
      curTask = TASK_NONE;
      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
    osDelay(1);
  }
  /* USER CODE END runPPTask */
}

/* USER CODE BEGIN Header_runDZTask */
/**
 * @brief Function implementing the DZTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runDZTask */
void runDZTask(void *argument)
{
  /* USER CODE BEGIN runDZTask */
  for (;;)
  {
    if (curTask != TASK_MOVE_OBS_ZOOMZOOM)
      osDelay(1000);
    else
    {
      targetDist = (float)curCmd.val;
      RobotMoveDistanceObstacle(&targetDist, SPEED_MODE_3);

      __ON_TASK_END(&htim8, prevTask, curTask);

      if (__COMMAND_QUEUE_IS_EMPTY(cmdQ))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cmdQ, curCmd, rxMsg);
    }
    osDelay(1);
  }
  /* USER CODE END runDZTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\0", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
