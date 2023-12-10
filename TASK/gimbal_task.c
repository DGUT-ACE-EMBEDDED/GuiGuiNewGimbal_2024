/*--------------------- FIRMWARE --------------------*/
#include "string.h"
#include "usbd_cdc_if.h"

/*--------------------- TASK --------------------*/
#include "gimbal_task.h"
#include "fire_Task.h"
#include "virtual_task.h"
/*--------------------- COMMUINICATE --------------------*/
#include "can1_receive.h"
#include "can2_receive.h"
#include "can1_send.h"
#include "can2_send.h"

/*--------------------- CONTROL --------------------*/
#include "gimbal_behaviour.h"
#include "gimbal_struct_variables.h"
#include "gimbal_auto.h"
#include "gimbal_config.h"
/*--------------------- ALGORITHM --------------------*/
#include "pid.h"
#include "maths.h"
#include "filter.h"
#include "lqr.h"
/*--------------------- BSP --------------------*/
#include "bsp_dr16.h"
#include "bsp_Motor_Encoder.h"
#include "bsp_referee.h"

extern TIM_HandleTypeDef htim8;

gimbal_control_t Gimbal_Control;

static void Gimbal_Work(gimbal_control_t *Gimbal_Work_f);
static void Gimbal_Init(gimbal_control_t *Gimbal_Init_f);
#if(PITCH_CONTROLER == PITCH_USE_LQR)
float k_pitch_lqr[2] = {-5.94183025734807, -0.3};
#endif
#if(YAW_CONTROLER == YAW_USE_LQR)
float k_yaw_lqr[2] = {-7.96227766016838, -0.50467065322830};
//float k_yaw_lqr[2] = {-12.96227766016838, -0.66667065322830};
#endif
/**
 * @brief  云台主任务
 * @param
 * @retval void
 */
void Gimbal_Task(void const *argument)
{
//    uint32_t currentTime;

    Gimbal_Init(&Gimbal_Control);
//    currentTime = xTaskGetTickCount();
    while (1)
    {
			if(xTaskGetTickCount() > chassis_rc_lost_time() )
			{
				RC_DataReload();
			}
			can2_gimbal_to_chassis();
			taskENTER_CRITICAL();         // 进入临界区
			Gimbal_Work(&Gimbal_Control); // 云台状态控制    //云台工作
			taskEXIT_CRITICAL();              // 退出临界区
			MI_motor_controlmode(Gimbal_Control.Pitch_c.pitch_motor, Gimbal_Control.Pitch_c.Torque_val, 0, 0, 0, 0);
			can2_gimbal_setmsg_to_yaw(Gimbal_Control.Yaw_c.yaw_motor.set_voltage);
		
      vTaskDelay(1); // 绝对延时//vTaskDelay(2);
    }
}

void Gimbal_Init(gimbal_control_t *Gimbal_Init_f)
{
    memset(Gimbal_Init_f, 0, sizeof(gimbal_control_t));
    /*--------------------获取指针--------------------*/
    // 获取遥控器指针(数据)
    Gimbal_Init_f->Gimbal_RC = RC_Get_RC_Pointer();
    // 获取云台指针
    Gimbal_Init_f->Pitch_c.pitch_motor = get_pitch_motor_measure_point();
    Gimbal_Init_f->Yaw_c.yaw_motor.motor_measure = get_yaw_motor_measure_point();
    // 获取火控指针
    Gimbal_Init_f->fire_c = get_fire_control_point();
		//获取自瞄指针
		Gimbal_Init_f->auto_c = get_auto_control_point();
    // 获取陀螺仪指针
    Gimbal_Init_f->Imu_c = get_imu_control_point();
		// 获取裁判系统指针
		Gimbal_Init_f->referee = Get_referee_Address();
    /*--------------------初始化编码器--------------------*/
    Gimbal_Init_f->Yaw_c.yaw_motor_encoder = Encoder_Init(GM6020, 2);

		#if(PITCH_CONTROLER == PITCH_USE_LQR)
		PidInit(&Gimbal_Init_f->Pitch_c.qitch_lqr_only_i_pid, 0, 25.0f, 0, Integral_Limit | ChangingIntegrationRate | Separated_Integral);
    PidInitMode(&Gimbal_Init_f->Pitch_c.qitch_lqr_only_i_pid, Integral_Limit, 350, 0);
		PidInitMode(&Gimbal_Init_f->Pitch_c.qitch_lqr_only_i_pid, ChangingIntegrationRate, 3, 0);
		PidInitMode(&Gimbal_Init_f->Pitch_c.qitch_lqr_only_i_pid, Separated_Integral, 3.0f, -3.0f);
		
		LQR_Init(&Gimbal_Init_f->Pitch_c.motor_lqr, 2, 1, k_pitch_lqr);
		#elif(PITCH_CONTROLER == PITCH_USE_PID)
    /*--------------------pid--------------------*/
    // P轴
    PidInit(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, GIMBAL_PITCH_S_P, GIMBAL_PITCH_S_I, GIMBAL_PITCH_S_D, Integral_Limit | Output_Limit);
    PidInit(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, GIMBAL_PITCH_P_P, GIMBAL_PITCH_P_I, GIMBAL_PITCH_P_D, Integral_Limit | Output_Limit | StepIn | ChangingIntegrationRate);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, Output_Limit, 10, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_speed_pid, Integral_Limit, 0, 0);
        
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, ChangingIntegrationRate   , 4, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, Integral_Limit, 1000, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, Output_Limit, 10, 0);
    PidInitMode(&Gimbal_Init_f->Pitch_c.pitch_motor_position_pid, StepIn, 1000, 0);
		#endif
		#if(YAW_CONTROLER == YAW_USE_LQR)
		PidInit(&Gimbal_Init_f->Yaw_c.yaw_lqr_only_i_pid, 0, 15, 0, Integral_Limit | ChangingIntegrationRate | Separated_Integral);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_lqr_only_i_pid, Integral_Limit, 350, 0);
		PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_lqr_only_i_pid, ChangingIntegrationRate, 4, 0);
		PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_lqr_only_i_pid, Separated_Integral, 4.0f, -4.0f);
		
		LQR_Init(&Gimbal_Init_f->Yaw_c.motor_lqr, 2, 1, k_yaw_lqr);
		#elif(YAW_CONTROLER == YAW_USE_PID)
    // Y轴
    PidInit(&Gimbal_Init_f->Yaw_c.yaw_motor_speed_pid, GIMBAL_YAW_S_P, GIMBAL_YAW_S_I, GIMBAL_YAW_S_D, Output_Limit | Integral_Limit);
    PidInit(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, GIMBAL_YAW_P_P,GIMBAL_YAW_P_I, GIMBAL_YAW_P_D, Integral_Limit | Output_Limit | StepIn);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_speed_pid, Output_Limit, 30000, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_speed_pid, Integral_Limit, 10000, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, StepIn, 60, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, Integral_Limit, 300, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_position_pid, Output_Limit, 30000, 0);
		
		//自瞄
		PidInit(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_speed_pid, GIMBAL_YAW_visual_S_P, GIMBAL_YAW_visual_S_I, GIMBAL_YAW_visual_S_D, Output_Limit | Integral_Limit );
    PidInit(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_position_pid, GIMBAL_YAW_visual_P_P, GIMBAL_YAW_visual_P_I, GIMBAL_YAW_visual_P_D, Integral_Limit | Output_Limit );		
		PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_speed_pid, Output_Limit, 30000, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_speed_pid, Integral_Limit, 10000, 0);
    
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_position_pid, Integral_Limit, 5, 0);
    PidInitMode(&Gimbal_Init_f->Yaw_c.yaw_motor_visual_position_pid, Output_Limit, 30000, 0);
		#endif
		
		/*--------------------filter--------------------*/
		#if(PITCH_CONTROLER == PITCH_USE_LQR)
		sliding_mean_filter_init(&Gimbal_Init_f->Pitch_c.motor_filter);
		#endif
		#if(YAW_CONTROLER == YAW_USE_LQR)
		sliding_mean_filter_init(&Gimbal_Init_f->Yaw_c.motor_filter);
		#endif
		first_order_filter_init(&Gimbal_Init_f->Pitch_c.visual_pitch__first_order_filter,0.03);
		
    EncoderValZero(Gimbal_Init_f->Yaw_c.yaw_motor_encoder);
    Gimbal_Init_f->chassis_gimbal_angel = 0;
}

void Gimbal_Work(gimbal_control_t *Gimbal_Work_f)
{
    gimbal_behaviour_choose(Gimbal_Work_f);

    gimbal_behaviour_react(Gimbal_Work_f);

    gimbal_pid_calculate(Gimbal_Work_f);
}
void Gimbal_pitch_positon360_update(void)
{
	//Gimbal_Control.Pitch_c.pitch_motor.actPositon_360 = ((float)Gimbal_Control.Pitch_c.pitch_motor_encoder->Encode_Actual_Val * 360.0f / 8192.0f - PITCH_ZERO_OFFSET);
}
gimbal_behaviour_e *get_gimbal_behaviour_point(void)
{
    return &Gimbal_Control.gimbal_behaviour;
}
gimbal_pitch_control_t *get_gimbal_pitch_point(void)
{
    return &Gimbal_Control.Pitch_c;
}
