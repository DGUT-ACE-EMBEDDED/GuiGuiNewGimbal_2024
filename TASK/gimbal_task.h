#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "gimbal_struct_variables.h"
#include "gimbal_config.h"

void Gimbal_Task(void const *argument);
gimbal_behaviour_e *get_gimbal_behaviour_point(void);
gimbal_pitch_control_t *get_gimbal_pitch_point(void);
void Gimbal_pitch_positon360_update(void);
#endif
