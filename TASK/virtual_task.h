#ifndef __TASK_VIRTUAL_H  //如果未定义
#define __TASK_VIRTUAL_H  //那么定义

#include "gimbal_struct_variables.h"

extern void Virtual_Task(void const * argument);
void gimbal_clear_virtual_recive(void);
const gimbal_auto_control_t **get_auto_control_point(void);
#endif


