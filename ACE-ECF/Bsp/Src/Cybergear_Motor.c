
/*************************** Dongguan-University of Technology -ACE**************************
 * @file    cybergear.c
 * @author  zhengNannnn
 * @version V1.0
 * @date    2023/10/1
 * @brief
 ******************************************************************************
 * @verbatim
 *  �׹����������,֧���Զ���ѡ��can����,֧�ֶ���ͬʱʹ��
 *  ʹ�÷�����
 *      �ȴ���һ���׹�����ṹ��
 *      ��ʼ����
 *            ʹ��MI_motor_init�������ݽṹ�壬ע���������ú���ʹ�õ�can����
 *            ʹ��MI_motor_get_ID(MI_Motor);//��ȡ���idֵ
 *            ʹ��changeID(MI_Motor,num);//����ID
 *      ʹ�ã���������ģʽ,�������̲ο��û��ֲ�
 *  demo��
 *       extern MI_Motor_t MI_Motor;//����һ���׹�����ṹ��,ʵ������can�������.c����������
 *       MI_Motor = MI_motor_init(hcan1);//������can1
 *       MI_motor_get_ID(MI_Motor);//��ȡ���idֵ,���浽�ṹ��
 *       MI_motor_changeID(MI_Motor,127);//����IDΪ127
 *       MI_motor_enable(MI_Motor)��//���ʹ��
 *       MI_motor_controlmode(MI_Motor, torque, MechPosition , speed , kp , kd);//�˿�ģʽ
 *       void MI_motor_stop(MI_Motor_t* hmotor);//���ʧ��
 * @attention
 *      ��ȷ��CAN�Ѿ�start
 *      �����жϻ��ڴ��ͨ�Ż���С��ͨ��,��MI_motor_recive_callback�������޸ĺ궨��
 *      ÿ����һ���׹�����ṹ��,�Ͷ�һ������������Ӧ��can�ж�����,����
 *      MI_Motor_t YAW_Motor;
 *      MI_Motor_t PITCH_Motor;
 *      MI_motor_recive_callback(YAW_Motor, Rxmessage.ExtId, Rx_Data)�����жϴ�����
 *      MI_motor_recive_callback(PITCH_Motor, Rxmessage.ExtId, Rx_Data)�����жϴ�����
 * @version
 * v1.0   �����汾
 * v1.1   ��ӽ��մ�����
 ************************** Dongguan-University of Technology -ACE***************************/
#include "Cybergear_Motor.h"
#include <string.h>
#include <stdlib.h>

//������floatת��uint8_t
union FloatAndBytes_t {
    float floatValue;
    uint8_t byteValue[4];
}FloatAndBytes;

bool get_can_id = false;
uint8_t MI_id = 0;
/**
  * @brief         ������ת4�ֽ�
  * @param         ������
  * @return        4�ֽ�����
  * @description   IEEE 754 Э��
  */
uint8_t byte[4];
uint8_t* Float_to_Byte(float f)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;       
	byte[0] = (longdata & 0xFF000000) >> 24;
	byte[1] = (longdata & 0x00FF0000) >> 16;
	byte[2] = (longdata & 0x0000FF00) >> 8;
	byte[3] = (longdata & 0x000000FF);
	return byte;
}
/**
  * @brief         4�ֽ�ת������
  * @param         4�ֽ�����
  * @return        ������
  * @description   IEEE 754 Э��
  */
float Byte_to_Float(uint8_t* byte)
{
    unsigned long longdata = 0;

    // ���ֽڰ�λ��װ��һ���޷��ų�����
    longdata |= (unsigned long)byte[0] << 24;
    longdata |= (unsigned long)byte[1] << 16;
    longdata |= (unsigned long)byte[2] << 8;
    longdata |= (unsigned long)byte[3];

    // �����������½���Ϊ������
    float result = *(float*)&longdata;

    return result;
}

/**
  * @brief  floatתint�����ݴ����
  * @param  x float��ֵ
  * @param  x_min float��ֵ����Сֵ
  * @param  x_max float��ֵ�����ֵ
  * @param  bits  int������λ��
  * @retval null
  */
int float_to_uint(float x, float x_min, float x_max, int bits) 
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x=x_max;
    else if(x < x_min) x= x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
/**
  * @brief  С�׵������16λ����ת����
  * @param  16λ���� 
  * @param  ��Ӧ�������� 
  * @param  ��Ӧ�������� 
  * @param  ����λ��
  * @return ������Ӧ������
  */
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

/**
  * @brief  С�׵��CANͨ�ŷ���
  * @param  hmotor ����ṹ��
  * @retval null
  */
CAN_TxHeaderTypeDef CAN_TxHeader_MI;
void MI_Motor_CanTx(MI_Motor_t* MI_motor) 
{
    CAN_TxHeader_MI.DLC = 8;
    CAN_TxHeader_MI.IDE = CAN_ID_EXT;
    CAN_TxHeader_MI.RTR = CAN_RTR_DATA;
    CAN_TxHeader_MI.ExtId = *((uint32_t*)&(MI_motor->txMsg.ExtId));
	/*CAN_TxHeader_MI.ExtId = hmotor->EXT_ID.motor_id<<24 | hmotor->EXT_ID.data << 8 | hmotor->EXT_ID.mode << 5;*/
    uint32_t mailbox;
    
    uint32_t ret = HAL_CAN_AddTxMessage(MI_motor->phcan, &CAN_TxHeader_MI, MI_motor->txMsg.Data, &mailbox);
//    if (ret != HAL_OK) {
//        /* Transmission request Error */
//        while(1);
//    }
    while (ret != HAL_OK) {
        /* Transmission request Error */
        ret = HAL_CAN_AddTxMessage(MI_motor->phcan, &CAN_TxHeader_MI, MI_motor->txMsg.Data, &mailbox);
    }
}

/**
  * @brief  ����С�׵���ṹ�岢��ʼ��
  * @param  phcan can���߾��
  * @retval �����С�׵���ṹ��ָ��
  */
MI_Motor_t* MI_motor_init(CAN_HandleTypeDef *phcan)
{
    MI_Motor_t *MI_motor = (MI_Motor_t *)malloc(sizeof(MI_Motor_t));
    memset(MI_motor, 0, sizeof(MI_Motor_t));
    
    MI_motor->master_id = MASTER_ID;
    MI_motor->motor_id = MOTOR_ID;
    MI_motor->phcan = phcan;
    
    return MI_motor;
}

/**
  * @brief  С�׵��ʹ��
  * @param  hmotor ����ṹ��
  * @param  id ���id
  * @retval null
  */
void MI_motor_enable(MI_Motor_t* MI_motor)
{
    MI_motor->txMsg.ExtId.mode = 3;
    MI_motor->txMsg.ExtId.motor_id = MI_motor->motor_id;
    MI_motor->txMsg.ExtId.data = MI_motor->master_id;
    MI_motor->txMsg.ExtId.res = 0;
    for(uint8_t i=0; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  ��ȡ�豸ID ��ͨ������0�������ڵ��ʹ��ǰʹ��
  * @param  hmotor ����ṹ��
  * @retval null 
  */
void MI_motor_get_ID(MI_Motor_t* MI_motor)
{
    MI_motor->txMsg.ExtId.mode = 0;
    MI_motor->txMsg.ExtId.data = 0;
    MI_motor->txMsg.ExtId.motor_id = 0;
    MI_motor->txMsg.ExtId.res = 0;
    
    get_can_id = true;
    for(uint8_t i=0; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
    while(get_can_id);//�ȴ�can�жϽ��յ����Ŀǰid
    MI_motor->motor_id = MI_id;
    
}

/**
  * @brief  �˿�ģʽ�������ָ�ͨ������1��
  * @param  hmotor ����ṹ��
  * @param  torque ����
  * @param  MechPosition Ŀ��λ��
  * @param  speed ת��
  * @param  kp 
  * @param  kd 
  * @retval null
  */
void MI_motor_controlmode(MI_Motor_t* hmotor, float torque, float MechPosition , float speed , float kp , float kd)
{
    hmotor->txMsg.ExtId.mode = 1;
    hmotor->txMsg.ExtId.data = float_to_uint(torque,T_MIN,T_MAX,16);
    hmotor->txMsg.ExtId.res = 0;
 
    hmotor->txMsg.Data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
    hmotor->txMsg.Data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
    hmotor->txMsg.Data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
    hmotor->txMsg.Data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
    hmotor->txMsg.Data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    hmotor->txMsg.Data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
    hmotor->txMsg.Data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    hmotor->txMsg.Data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);
    MI_Motor_CanTx(hmotor);
}

/**
  * @brief  ���ֹͣ����֡��ͨ������4��
  * @param  hmotor ����ṹ��
  * @retval null
  */
void MI_motor_stop(MI_Motor_t* MI_motor)
{
    MI_motor->txMsg.ExtId.mode = 4;
    MI_motor->txMsg.ExtId.data = MI_motor->master_id;
    MI_motor->txMsg.ExtId.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  ���õ����е��λ��ͨ������6����ѵ�ǰ���λ����Ϊ��е��λ�����綪ʧ��
  * @param  hmotor ����ṹ��
  * @retval null
  */
void MI_motor_setMechPosition2Zero(MI_Motor_t* MI_motor)
{
    MI_motor->txMsg.ExtId.mode = 6;
    MI_motor->txMsg.ExtId.data = MASTER_ID;
    MI_motor->txMsg.ExtId.res = 0;
    MI_motor->txMsg.Data[0]=1;
 
    for(uint8_t i=1; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  ���õ��CAN_ID��ͨ������7�����ĵ�ǰ���CAN_ID , ������Ч�����ڵ��ʹ��ǰʹ��
  * @param  MI_motor ����ṹ��
  * @param  Target_ID ��Ҫ�ĳɵĵ��ID
  * @retval null
  */
void MI_motor_changeID(MI_Motor_t* MI_motor,uint8_t Target_ID)
{
    MI_motor->txMsg.ExtId.mode = 7;	
    MI_motor->txMsg.ExtId.motor_id = MI_motor->motor_id;
    MI_motor->txMsg.ExtId.data = Target_ID << 8 | MI_motor->master_id;
    MI_motor->txMsg.ExtId.res = 0;
    
    MI_motor->motor_id = Target_ID;//���ı����canID
 
    for(uint8_t i=0; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  ����������ȡ��ͨ������17��
  * @param  MI_motor ����ṹ��ָ��
  * @param  parameter_index_e ��ȡ�Ĳ���
  * @retval null
  * @note   Ŀǰ�������մ���Сbug
  */
void MI_motor_Read_One_Para(MI_Motor_t* MI_motor, parameter_index_e index)
{
    MI_motor->txMsg.ExtId.mode = 17;
    MI_motor->txMsg.ExtId.data = MASTER_ID;
    MI_motor->txMsg.ExtId.res = 0;
    MI_motor->txMsg.Data[0]=index;
    memcpy(&MI_motor->txMsg.Data[0],&index,2);
    for(uint8_t i=2; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  ��������д�루ͨ������18�� �����綪ʧ��
  * @param  ����ṹ��ָ��
  * @param  д��Ĳ���
  * @param  д���ֵ
  * @retval null
  * @note   ���� λ�� �ٶ�ģʽ��һ��д���޷�Ӧ��
  */
void MI_motor_Write_One_Para(MI_Motor_t* MI_motor, parameter_index_e index , float Value)
{
    MI_motor->txMsg.ExtId.mode = 0x12;
    MI_motor->txMsg.ExtId.data = MI_motor->master_id;
    MI_motor->txMsg.ExtId.res = 0;
    for(uint8_t i=2; i<8; i++)
    {
        MI_motor->txMsg.Data[i] = 0;
    }

    memcpy(&MI_motor->txMsg.Data[0],&index,2);
//    memcpy(&MI_motor->txMsg.Data[4],data, 4);
    if (index == run_mode)
    {
        MI_motor->txMsg.Data[4]=(uint8_t)Value;
		MI_motor->txMsg.Data[5]=0x00;
		MI_motor->txMsg.Data[6]=0x00;
		MI_motor->txMsg.Data[7]=0x00;	
    }
    else
    {
		Float_to_Byte(Value);
		MI_motor->txMsg.Data[4]=byte[3];
		MI_motor->txMsg.Data[5]=byte[2];
		MI_motor->txMsg.Data[6]=byte[1];
		MI_motor->txMsg.Data[7]=byte[0];		
	}
    MI_Motor_CanTx(MI_motor);
}

/**
  * @function     : ���õ������ģʽ
  * @param        : �������ģʽ
  * @return       : null
  */
void Set_Mode(MI_Motor_t *MI_motor,Mode_rum_mode_t Mode)
{
	MI_motor_Write_One_Para(MI_motor,run_mode,Mode);
}


/**
  * @brief  С�׵�����մ�����
  * @param  MI_motor ����ṹ��ָ��
  * @param  receive_ExtId ���յ�can��չ��ʶ��
  * @param  Rx_Data[8] ���յ�8�ֽ�����
  * @retval null
  * @note   �жϵ����ݽṹ���ڱ����id��Ҫ�Լ��ֶ�����
  */
void MI_motor_recive_callback(MI_Motor_t* MI_motor, uint32_t receive_ExtId, uint8_t Rx_Data[8])
{
//    uint16_t temp;
    MI_motor->rxMsg.ExtId = *((rxExtId_t*)&receive_ExtId);
    if (MI_motor->rxMsg.ExtId.motor_id != MI_motor->master_id)       return;//�Ǳ����
    if (MI_motor->rxMsg.ExtId.motor_now_id != MI_motor->motor_id)    return;//�Ǳ�����
    //���ϲ���������can�����ϴ��ض���׹���������,ÿʹ��һ���׹�����Ͷ�һ�������can�����жϴ�������,���յ���ʶ�������idֵ�Զ�����
    
    switch (MI_motor->rxMsg.ExtId.mode)
    {
        case 0:{//��ȡ�豸��ID��64λMCUΨһ��ʶ��(ͨ������0)
            if (MI_motor->rxMsg.ExtId.motor_id == 0xFE){
                get_can_id = true;//MI_motor_get_ID�õõ�
                MI_id = MI_motor->rxMsg.ExtId.motor_id;
            };     //�������64λMCUΨһ��ʶ��,�ݲ����
            break;
        }
        case 2:{//�����������֡,���������������������״̬(ͨ������2)
            
            //��չ֡bit16~23��Ϣ
            MI_motor->motor_parameter_list.Mode_rum_mode = (Mode_rum_mode_t)MI_motor->rxMsg.ExtId.Mode_status;
            MI_motor->motor_error_list.Encoder_not_calibrated_fault = MI_motor->rxMsg.ExtId.Uncalibrated;
            MI_motor->motor_error_list.HALL_encoding_fault = MI_motor->rxMsg.ExtId.HALL_encoding_failt;
            MI_motor->motor_error_list.Magnetic_encoding_fault = MI_motor->rxMsg.ExtId.Magnetic_encoding_fault;
            MI_motor->motor_error_list.Over_temperature_fault = MI_motor->rxMsg.ExtId.Over_temperature;
            MI_motor->motor_error_list.Over_current_fault = MI_motor->rxMsg.ExtId.Over_current;
            MI_motor->motor_error_list.Under_voltage_fault = MI_motor->rxMsg.ExtId.Under_voltage_fault;
            
//            temp = (Rx_Data[0]<<8| Rx_Data[1]);
//            MI_motor->postion     = (float)temp/32768*4*3.1415926f;
//            
//            temp = (Rx_Data[2]<<8| Rx_Data[3]);
//            MI_motor->speed     = (float)temp/32768*30;
//            
//            temp = (Rx_Data[4]<<8| Rx_Data[5]);
//            MI_motor->torque     = (float)temp/32768*12.0f;
//            
//            temp = (Rx_Data[6]<<8| Rx_Data[7]);
//            MI_motor->temperture     = (float)temp/10.0f;//�ش����¶��ǵ�ǰ���϶�*10
            MI_motor->postion=uint16_to_float(Rx_Data[0]<<8|Rx_Data[1],MIN_P,MAX_P,16);

            MI_motor->speed=uint16_to_float(Rx_Data[2]<<8|Rx_Data[3],MIN_S,MAX_S,16);			

            MI_motor->torque=uint16_to_float(Rx_Data[4]<<8|Rx_Data[5],MIN_T,MAX_T,16);				

            MI_motor->temperture=(Rx_Data[6]<<8|Rx_Data[7])*Temp_Gain;

            break;
        }
        case 17:{//���Ͷ�ȡ����������Ӧ��֡,����index�����±��жϻش���������(ͨ������17)
            parameter_index_e index = (parameter_index_e)(Rx_Data[1]<<8| Rx_Data[0]);//

            //������4��u8תfloat
            FloatAndBytes.byteValue[0] = Rx_Data[4];
            FloatAndBytes.byteValue[1] = Rx_Data[5];
            FloatAndBytes.byteValue[2] = Rx_Data[6];
            FloatAndBytes.byteValue[3] = Rx_Data[7];

            switch (index){
                case run_mode:
                    MI_motor->motor_parameter_list.Mode_rum_mode = (Mode_rum_mode_t) Rx_Data[4];//
                    break;
                case iq_ref:
                    MI_motor->motor_parameter_list.iq_ref = FloatAndBytes.floatValue;
                    break;
                case spd_ref:
                    MI_motor->motor_parameter_list.spd_ref = FloatAndBytes.floatValue;
                    break;
                case imit_torque:
                    MI_motor->motor_parameter_list.imit_torque = FloatAndBytes.floatValue;
                    break;
                case cur_kp:
                    MI_motor->motor_parameter_list.cur_kp = FloatAndBytes.floatValue;
                    break;
                case cur_ki:
                    MI_motor->motor_parameter_list.cur_ki = FloatAndBytes.floatValue;
                    break;
                case cur_filt_gain:
                    MI_motor->motor_parameter_list.cur_filt_gain = FloatAndBytes.floatValue;
                    break;
                case loc_ref:
                    MI_motor->motor_parameter_list.loc_ref = FloatAndBytes.floatValue;
                    break;
                case limit_spd:
                    MI_motor->motor_parameter_list.limit_spd = FloatAndBytes.floatValue;
                    break;
                case limit_cur:
                    MI_motor->motor_parameter_list.limit_cur = FloatAndBytes.floatValue;
                    break;
                default :
                    break;
            }
        }
        case 21:{//���Ϸ���֡
            //Byte0
            MI_motor->motor_error_list.Over_temperature_fault = Rx_Data[0] & 0x01;//ȡbit0
            MI_motor->motor_error_list.Drive_core_fault = (Rx_Data[0]>>1) & 0x01;//ȡbit1
            MI_motor->motor_error_list.Under_voltage_fault = (Rx_Data[0]>>2) & 0x01;//ȡbit2
            MI_motor->motor_error_list.Over_voltage_fault =  (Rx_Data[0]>>3) & 0x01;//ȡbit3
            MI_motor->motor_error_list.B_phase_current_sampling_overcurrent = (Rx_Data[0]>>4) & 0x01;//ȡbit4
            MI_motor->motor_error_list.C_phase_current_sampling_overcurrent = (Rx_Data[0]>>5) & 0x01;//ȡbit5
            MI_motor->motor_error_list.Magnetic_encoding_fault = (Rx_Data[0]>>7) ;//ȡbit7
            
            uint16_t Byte1_2 = ((uint16_t)Rx_Data[1] << 8) | Rx_Data[2];//ȡbit8~bit15
            if (Byte1_2 != 0)   MI_motor->motor_error_list.Overload_fault = true;
            else                MI_motor->motor_error_list.Overload_fault = false;
            
            MI_motor->motor_error_list.A_phase_current_sampling_overcurrent = Rx_Data[3] & 0x01;//ȡbit16
        }
    }
}
