/*--------------------- CONTROL --------------------*/
#include "gimbal_auto.h"
#include "gimbal_struct_variables.h"

/*--------------------- FIRMWARE --------------------*/
#include "usbd_cdc_if.h"


/**
  * @brief      发送数据给视觉
  * @param[in]  data: 敌方装甲板  红：0 | 蓝：1
  * @param[in]  mode: 模式选择    0：装甲模式  1：能量机关模式  2：陀螺模式
  * @param[in]  shoot_speed: 射速，目前是平均实际速度，其实就是当前上限射速-2
  * @retval     none
  * @attention  协议：帧头0xFF  数据  帧尾0xFE
  */
void visual_send_data(uint8_t data, uint8_t mode, uint8_t shoot_speed,float yaw,float pitch)
{
    uint8_t SendBuff[29];
	
    SendBuff[0] = 0xFF;
    SendBuff[1] = data;
    SendBuff[2] = mode;
    SendBuff[3] = shoot_speed;
		{
			float register *Register1 = INS.q;
			uint8_t register *Register2 = &SendBuff[4];
			int register Register3;
			 __asm__ volatile
			{
				LDR Register3 , [Register1],#4
				STR	Register3 , [Register2],#4
				LDR Register3 , [Register1],#4
				STR	Register3 , [Register2],#4
				LDR Register3 , [Register1],#4
				STR	Register3 , [Register2],#4
				LDR Register3 , [Register1],#4
				STR	Register3 , [Register2],#4
				LDR Register3 , [&yaw]
				STR Register3 , [Register2],#4
				LDR Register3 , [&pitch]
				STR Register3 , [Register2]
			}
		}
    SendBuff[28] = 0xFE;
		CDC_Transmit_FS(SendBuff, sizeof(SendBuff));
}

