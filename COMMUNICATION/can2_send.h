#ifndef __CAN2_SEND_H
#define __CAN2_SEND_H

#include "gimbal_struct_variables.h"

//extern void can2_gimbal_to_chassis(void);
extern void can2_gimbal_setmsg_to_yaw(int16_t yaw);
extern void can2_gimbal_to_chassis(void);
#endif
