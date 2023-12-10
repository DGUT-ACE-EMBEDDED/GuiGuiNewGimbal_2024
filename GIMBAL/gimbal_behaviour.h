#ifndef __GIMBAL_BEHAVIOUR_H
#define __GIMBAL_BEHAVIOUR_H

#include "gimbal_struct_variables.h"

void gimbal_behaviour_choose(gimbal_control_t *gimbal_behaviour_choose_f);
void gimbal_behaviour_react(gimbal_control_t *gimbal_behaviour_react_f);
void gimbal_pid_calculate(gimbal_control_t *gimbal_pid_calculate_f);

float* get_Gimbal_pitch_point(void);
float* get_Gimbal_yaw_point(void);
#endif
