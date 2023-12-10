#ifndef CYBERGEAR_H
#define CYBERGEAR_H

#include "stm32f4xx_hal.h"
#include <struct_typedef.h>
#include <stdbool.h>


/*---IDֵ�궨��---*/
#define MASTER_ID 0
#define MOTOR_ID 127 //pitch���127��

/*---д�������ַ�궨��---*/
#define SET_MODE_INDEX 0x7005
#define POSTION_MODE_SET_POSTION_INDEX 0x7016
#define POSTION_MODE_SET_SPEED_INDEX 0x7017
#define SPEED_MODE_SET_CURRENT_INDEX 0x7018

/*---CAN��չ��ʶ�������---*/
#define txCanIdEx (((struct ExtId)&(txMsg.tx_extid)))
#define rxCanIdEx (((struct ExtId)&(rxMsg.rx_efid))) //����չ֡id����Ϊ�������ݽṹ
/*�����������*/
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define Temp_Gain 0.1
#define MAX_P 720
#define MIN_P -720
#define MAX_S 30
#define MIN_S -30
#define MAX_T 12
#define MIN_T -12
    
typedef enum{
    RESET_MODE = 0,//Reset[ģʽ]
    CALI_MODE  = 1,//Cali ģʽ[�궨]
    RUN_MODE   = 2 //Motorģʽ[����]
} motor_mode_e;//���ģʽ

typedef enum{
    control = 0,//�˿�ģʽ
    postion = 1,//λ��ģʽ
    speed   = 2,//�ٶ�ģʽ
    current = 3 //����ģʽ
}Mode_rum_mode_t;//ģʽ״̬

//��������б�
typedef struct{
    Mode_rum_mode_t Mode_rum_mode;
    float iq_ref;
    float spd_ref;
    float imit_torque;
    float cur_kp;
    float cur_ki;
    float cur_filt_gain;
    float loc_ref;
    float limit_spd;
    float limit_cur;
}motor_parameter_list_t;

//������ϼ������б�
typedef struct{
    bool Over_temperature_warning;  //���¾���
    
    bool Over_temperature_fault;    //���¹���
    bool Over_voltage_fault;        //��ѹ����
    bool Over_current_fault;        //��������
    bool Overload_fault;            //���ع���
    bool Under_voltage_fault;       //Ƿѹ����
    bool Encoder_not_calibrated_fault;//������δ�궨����
    bool Magnetic_encoding_fault;   //�ű������
    bool HALL_encoding_fault;       //HALL�������
    bool Drive_core_fault;          //����оƬ����
    bool A_phase_current_sampling_overcurrent;//A��������ù���
    bool B_phase_current_sampling_overcurrent;//B��������ù���
    bool C_phase_current_sampling_overcurrent;//C��������ù���   
}motor_error_list_t;//������ϼ������б�

typedef enum{
    run_mode        = 0x7005,
    iq_ref          = 0x7006,
    spd_ref         = 0x700A,
    imit_torque     = 0x700B,
    cur_kp          = 0x7010,
    cur_ki          = 0x7011,
    cur_filt_gain   = 0x7014,
    loc_ref         = 0x7016,
    limit_spd       = 0x7017,
    limit_cur       = 0x7018
}parameter_index_e;//��������±�

//CAN��չ��ʶ���ṹ��  motor_id��data��mode��29λ
typedef struct {
    uint32_t motor_id:8;    //bit7~0        ���canID
    uint32_t data:16;       //bit15~23       ��Ϣλ
    uint32_t mode:5;        //bit28~24      ͨ������
    uint32_t res:3;         //ռ��λ������Ϣ
}txExtId_t;
 
typedef struct {
    uint32_t motor_id:8;                //bit7~0    ���canID
    uint32_t motor_now_id:8;            //bit15~8   ��ǰ���canID
    uint32_t Under_voltage_fault:1;     //bit16     Ƿѹ����
    uint32_t Over_current:1;            //bit17     ����
    uint32_t Over_temperature:1;        //bit18     ����
    uint32_t Magnetic_encoding_fault:1; //bit19     �ű������
    uint32_t HALL_encoding_failt:1;     //bit20     HALL�������
    uint32_t Uncalibrated:1;            //bit21     δ�궨
    uint32_t Mode_status:2;             //bit22~23  ģʽ״̬
    uint32_t mode:5;                    //bit28~24  ͨ������
    uint32_t res:3;                     //ռ��λ������Ϣ
}rxExtId_t;

//CANͨ�Žṹ�� ���ذ�->���
typedef struct {
    txExtId_t     ExtId;//CAN 29λ��չ��ʶ��
    uint8_t       Data[8];//��������
}txMsg_t;

//CANͨ�Žṹ�� ���->���ذ�
typedef struct {
    rxExtId_t     ExtId;//CAN 29λ��չ��ʶ��
    uint8_t       Data[8];//��������
}rxMsg_t;

typedef struct{
    uint8_t master_id;
    uint8_t motor_id;
    uint64_t MCU;
    fp32 postion;
    fp32 speed;     
    fp32 torque;    //����
    fp32 temperture;//�¶�
    bool    error_falg;     //���ϱ�־λ
    
    motor_parameter_list_t motor_parameter_list;//�����б�
    motor_error_list_t    motor_error_list;     //״̬�б�
    
    CAN_HandleTypeDef *phcan;//���ע��ʹ�õ�CAN����
    txMsg_t            txMsg;//����
    rxMsg_t            rxMsg;//����
    
}MI_Motor_t;

void MI_motor_get_ID(MI_Motor_t* hmotor);
MI_Motor_t* MI_motor_init(CAN_HandleTypeDef *phcan);
void MI_motor_enable(MI_Motor_t *hmotor);
void MI_motor_controlmode(MI_Motor_t* hmotor, float torque, float MechPosition , float speed , float kp , float kd);
void MI_motor_stop(MI_Motor_t *hmotor);
void MI_motor_setMechPosition2Zero(MI_Motor_t *hmotor);
void MI_motor_changeID(MI_Motor_t* hmotor,uint8_t Target_ID);
void MI_motor_recive_callback(MI_Motor_t* MI_motor, uint32_t receive_ExtId, uint8_t Rx_Data[8]);
void MI_motor_Write_One_Para(MI_Motor_t* MI_motor, parameter_index_e index , float Value);
void MI_motor_Read_One_Para(MI_Motor_t* MI_motor,parameter_index_e index);
void Set_Mode(MI_Motor_t *MI_motor,Mode_rum_mode_t Mode);
#endif
