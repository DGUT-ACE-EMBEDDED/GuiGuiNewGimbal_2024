#ifndef __CAN2_RECEIVE_H
#define __CAN2_RECEIVE_H


#include "can1_receive.h"
//#include "rc.h"



//void (*can2_callback)(CAN_HandleTypeDef *);
//const chassis_receive_gimbal_t *get_yaw_receive_measure_point(void);
//motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
//uint32_t gimbal_rc_lost_time(void);

motor_measure_t *get_yaw_motor_measure_point(void);

extern void CAN2_filter_config(void);
extern void CAN2_Data_Receive(CAN_HandleTypeDef *hcan);
extern void chassis_can2_callback(CAN_HandleTypeDef *hcan);
extern void gimbal_can2_callback(CAN_HandleTypeDef *hcan);
extern uint16_t re_can2_shooter_heat0_speed_limit(void);
extern float re_chassis_gimbal_angel(void);
extern float re_gimbal_pitch_angle(void);
extern int re_gimbal_behaviour(void);
extern uint8_t re_robot_red_or_blue(void);


#endif 

