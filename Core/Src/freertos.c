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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "tim.h"
#include "usart.h"
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
/* Definitions for StartMachineTas */
osThreadId_t StartMachineTasHandle;
const osThreadAttr_t StartMachineTas_attributes = {
  .name = "StartMachineTas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for StartLCDTask */
osThreadId_t StartLCDTaskHandle;
const osThreadAttr_t StartLCDTask_attributes = {
  .name = "StartLCDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for StartMachineCMD */
osThreadId_t StartMachineCMDHandle;
const osThreadAttr_t StartMachineCMD_attributes = {
  .name = "StartMachineCMD",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for StartModuleTask */
osThreadId_t StartModuleTaskHandle;
const osThreadAttr_t StartModuleTask_attributes = {
  .name = "StartModuleTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void KeyboardTask(void *argument);
void MachineTask(void *argument);
void LCDTask(void *argument);
void MachineCMDTask(void *argument);
void ModuleTask(void *argument);

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

  /* creation of StartMachineTas */
  StartMachineTasHandle = osThreadNew(MachineTask, NULL, &StartMachineTas_attributes);

  /* creation of StartLCDTask */
  StartLCDTaskHandle = osThreadNew(LCDTask, NULL, &StartLCDTask_attributes);

  /* creation of StartMachineCMD */
  StartMachineCMDHandle = osThreadNew(MachineCMDTask, NULL, &StartMachineCMD_attributes);

  /* creation of StartModuleTask */
  StartModuleTaskHandle = osThreadNew(ModuleTask, NULL, &StartModuleTask_attributes);

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

/* USER CODE BEGIN Header_MachineTask */
/**
* @brief Function implementing the StartMachineTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MachineTask */
__weak void MachineTask(void *argument)
{
  /* USER CODE BEGIN MachineTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MachineTask */
}

/* USER CODE BEGIN Header_LCDTask */
/**
* @brief Function implementing the StartLCDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCDTask */
__weak void LCDTask(void *argument)
{
  /* USER CODE BEGIN LCDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LCDTask */
}

/* USER CODE BEGIN Header_MachineCMDTask */
/**
* @brief Function implementing the StartMachineCMD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MachineCMDTask */
__weak void MachineCMDTask(void *argument)
{
  /* USER CODE BEGIN MachineCMDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MachineCMDTask */
}

/* USER CODE BEGIN Header_ModuleTask */
/**
* @brief Function implementing the StartModuleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModuleTask */
__weak void ModuleTask(void *argument)
{
  /* USER CODE BEGIN ModuleTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ModuleTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

/* USER CODE END Application */

