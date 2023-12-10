/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       Task_Safe.c/h
  * @brief      安全任务
  * @note       合并版本
  * @history
  *
  @verbatim   v1.0
  ==============================================================================

  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
  */
#ifndef __TASK_SAFE_H // 如果未定义
#define __TASK_SAFE_H // 那么定义
#include "main.h"
#include "Init_Task.h"
#include "safe_check.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/*OS控制任务周期以及启动时间*/
#define LED_TASK_INIT_TIME 5
#define LED_CONTROL_TIME 5

void Safe_Task(void const *argument);
#endif
