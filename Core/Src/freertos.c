/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUZZER_REST_HZ         0U
#define BUZZER_TIMER_HZ        1000000U
#define BUZZER_DUTY_PERCENT    35U
#define BUZZER_GAP_MS          35U
#define PLAYBACK_SPEED_PERCENT 150U
#define PITCH_SHIFT_PERCENT    400U
#define GPIO_TEST_TICK_MS      200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for StartKeyboardTa */
osThreadId_t StartKeyboardTaHandle;
const osThreadAttr_t StartKeyboardTa_attributes = {
  .name = "StartKeyboardTa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void CAN_TestStart(void);
static void CAN_SendTestFrame(void);
static void RS485_Send(const uint8_t *data, uint16_t length);
static void RS485_SendTestFrame(void);
#if 0
static void Buzzer_PlayNote(uint32_t freqHz, uint32_t durationMs);
#endif
static void Buzzer_Off(void);

/* USER CODE END FunctionPrototypes */

void KeyboardTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of StartKeyboardTa */
  StartKeyboardTaHandle = osThreadNew(KeyboardTask, NULL, &StartKeyboardTa_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_KeyboardTask */
/**
  * @brief  Function implementing the StartKeyboardTa thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_KeyboardTask */
__weak void KeyboardTask(void *argument)
{
  /* USER CODE BEGIN KeyboardTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END KeyboardTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void CAN_TestStart(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterBank = 0;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  (void)HAL_CAN_ConfigFilter(&hcan, &filter);
  (void)HAL_CAN_Start(&hcan);
}

static void CAN_SendTestFrame(void)
{
  static uint8_t decimalCounter[8] = {0U};
  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t data[8];
  uint32_t txMailbox;

  if (HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_READY &&
      HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_LISTENING)
  {
    return;
  }

  txHeader.StdId = 0x123U;
  txHeader.ExtId = 0x00000000U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8U;
  txHeader.TransmitGlobalTime = DISABLE;

  for (uint32_t i = 0U; i < sizeof(data); ++i)
  {
    data[i] = decimalCounter[i];
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0U)
  {
    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox) == HAL_OK)
    {
      for (int32_t i = (int32_t)sizeof(decimalCounter) - 1; i >= 0; --i)
      {
        decimalCounter[i]++;
        if (decimalCounter[i] <= 9U)
        {
          break;
        }

        decimalCounter[i] = 0U;
      }
    }
  }
}

static void RS485_Send(const uint8_t *data, uint16_t length)
{
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, length, 100U);
  while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET)
  {
  }
}

static void RS485_SendTestFrame(void)
{
  static const uint8_t frame[] = {
    0x55U, 0x55U, 0x55U, 0x55U,
    0x55U, 0x55U, 0x55U, 0x55U
  };

  RS485_Send(frame, (uint16_t)sizeof(frame));
}

#if 0
static void Buzzer_PlaySong(const Note_t *song)
{
  uint32_t i = 0U;

  while (song[i].freqHz != MUSIC_NOTE_END)
  {
    Buzzer_PlayNote(song[i].freqHz, song[i].durationMs);
    osDelay(BUZZER_GAP_MS);
    ++i;
  }
}
static void Buzzer_PlayNote(uint32_t freqHz, uint32_t durationMs)
{
  uint32_t scaledFreqHz;
  uint32_t period;
  uint32_t pulse;
  uint32_t scaledDurationMs;

  scaledDurationMs = (durationMs * 100U) / PLAYBACK_SPEED_PERCENT;

  if (freqHz == BUZZER_REST_HZ)
  {
    Buzzer_Off();
    osDelay(scaledDurationMs);
    return;
  }

  scaledFreqHz = (freqHz * PITCH_SHIFT_PERCENT) / 100U;
  period = BUZZER_TIMER_HZ / scaledFreqHz;
  pulse = (period * BUZZER_DUTY_PERCENT) / 100U;

  if (pulse == 0U)
  {
    pulse = 1U;
  }

  __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
  osDelay(scaledDurationMs);
  Buzzer_Off();
}
#endif

static void Buzzer_Off(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
}

/* USER CODE END Application */

