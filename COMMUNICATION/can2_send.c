#include "can2_send.h"
#include "can.h"
#include "gimbal_config.h"
static CAN_TxHeaderTypeDef Txmessage; //发送的信息
extern gimbal_control_t Gimbal_Control;

void can2_gimbal_setmsg_to_yaw(int16_t yaw)
{
    uint32_t send_mail_box;
    uint8_t Data[8]; //发送数据的数组

    Txmessage.StdId = 0x1FF;
    Txmessage.IDE = CAN_ID_STD;
    Txmessage.RTR = CAN_RTR_DATA;
    Txmessage.DLC = 0x08;
    Data[0] = 0;
    Data[1] = 0;
    Data[2] = yaw >> 8;
    Data[3] = yaw;
    Data[4] = 0;
    Data[5] = 0;
    Data[6] = 0;
    Data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box);
}
/**
 * @brief		云台发送
 * @param		none
 *	@retval		none
 */
void can2_gimbal_to_chassis(void)
{
	uint32_t send_mail_box; //发送邮箱
	uint8_t Data[5];		//发送数据的数组
	
	Txmessage.StdId = 0x501;	  //
	Txmessage.IDE = CAN_ID_STD;	  //指定将要传输的消息的标识符的类型
	Txmessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
	Txmessage.DLC = 2;

	#ifdef FIRE_WORK
	Data[0] = (*Gimbal_Control.fire_c)->replenish_flag;
	Data[1] = (Gimbal_Control.gimbal_behaviour);
	#else
	Data[0] = 0;
	Data[1] = 0;
	#endif

	while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan2))==0);
	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box); //将一段数据通过 CAN 总线发送
}
