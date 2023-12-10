#ifndef __CAN1_RECEIVE_H
#define __CAN1_RECEIVE_H

#include "can.h"
#include "gimbal_struct_variables.h"

MI_Motor_t *get_pitch_motor_measure_point(void);
motor_measure_t *get_right_motor_measure_point(void);
motor_measure_t *get_left_motor_measure_point(void);
motor_measure_t *get_fire_motor_measure_point(void);
extern void CAN1_filter_config(void);
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
extern void chassis_can1_callback(CAN_HandleTypeDef *hcan);

#endif
