#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "maths.h"
#include "gimbal_config.h"
#include "virtual_task.h"
#include "filter.h"
#include "lqr.h"

float Gimbal_pitch = 0.0f;
float Gimbal_yaw = 0.0f;

static void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f);
static void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f);
static void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f);
static float torque_to_voltage_6020(float torque);

float *get_Gimbal_pitch_point(void)
{
    return &Gimbal_pitch;
}

float *get_Gimbal_yaw_point(void)
{
    return &Gimbal_yaw;
}

void gimbal_behaviour_choose(gimbal_control_t *gimbal_behaviour_choose_f)
{
    gimbal_behaviour_e last_behaviour;
    static gimbal_behaviour_e rc_behaviour = GIMBAL_MANUAL;
    static gimbal_behaviour_e kb_behaviour = GIMBAL_MANUAL;

    // 手柄
    last_behaviour = rc_behaviour;
    switch (gimbal_behaviour_choose_f->Gimbal_RC->rc.s2)
    {
    case RC_SW_UP:
        rc_behaviour = GIMBAL_AUTOATTACK;
        break;
    case RC_SW_MID:
        rc_behaviour = GIMBAL_MANUAL;
        break;
    case RC_SW_DOWN:
        rc_behaviour = GIMBAL_SMALL_AUTOBUFF;
        break;
    default:
			rc_behaviour = GIMBAL_MANUAL;
        break;
    }
    // 如果挡位发生改变，设置对应的模式
    if (last_behaviour != rc_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = rc_behaviour;
			if(gimbal_behaviour_choose_f->gimbal_behaviour == GIMBAL_BIG_AUTOBUFF || gimbal_behaviour_choose_f->gimbal_behaviour == GIMBAL_SMALL_AUTOBUFF|| gimbal_behaviour_choose_f->gimbal_behaviour == GIMBAL_AUTOATTACK )
			{
					//TODO:注意安全
					float *k = gimbal_behaviour_choose_f->Yaw_c.motor_lqr.k;
					*k = -12.96227766016838;
					k++;
					*k = -0.66667065322830;
					k = gimbal_behaviour_choose_f->Pitch_c.motor_lqr.k;
					*k = -25.0;
					k++;
					*k = -1.5;
			}
			else
			{
					//TODO:注意安全
					float *k = gimbal_behaviour_choose_f->Yaw_c.motor_lqr.k;
					*k = -7.96227766016838;
					k++;
					*k = -0.50467065322830;
					k = gimbal_behaviour_choose_f->Pitch_c.motor_lqr.k;
					*k = -6.0;
					k++;
					*k = -0.3;
			}
    }

    // 键鼠
    last_behaviour = kb_behaviour;
    //**c
//    if(gimbal_behaviour_choose_f->Gimbal_RC->kb.bit.C)
//    {
//        kb_behaviour = GIMBAL_MANUAL;
//    }
		if(gimbal_behaviour_choose_f->Gimbal_RC->mouse.press_r)
    {
         kb_behaviour = GIMBAL_AUTOATTACK;
				if(gimbal_behaviour_choose_f->Gimbal_RC->kb.bit.V)
				{
					kb_behaviour = GIMBAL_AUTOATTACK;
				}
				else if(gimbal_behaviour_choose_f->Gimbal_RC->kb.bit.B)
				{
					kb_behaviour = GIMBAL_SMALL_AUTOBUFF;
				}
				else if(gimbal_behaviour_choose_f->Gimbal_RC->kb.bit.G)
				{
					kb_behaviour = GIMBAL_BIG_AUTOBUFF;
				}
    }
		else
		{
				kb_behaviour = GIMBAL_MANUAL;
		}
    // 如果模式发生改变，设置对应的模式
    if (last_behaviour != kb_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = kb_behaviour;
			if(gimbal_behaviour_choose_f->gimbal_behaviour == GIMBAL_BIG_AUTOBUFF || gimbal_behaviour_choose_f->gimbal_behaviour == GIMBAL_SMALL_AUTOBUFF|| gimbal_behaviour_choose_f->gimbal_behaviour == GIMBAL_AUTOATTACK )
			{
					//TODO:注意安全
					float *k = gimbal_behaviour_choose_f->Yaw_c.motor_lqr.k;
					*k = -12.96227766016838;
					k++;
					*k = -0.66667065322830;
									k = gimbal_behaviour_choose_f->Pitch_c.motor_lqr.k;
					*k = -25.0;
					k++;
					*k = -1.5;
			}
			else
			{
					//TODO:注意安全
					float *k = gimbal_behaviour_choose_f->Yaw_c.motor_lqr.k;
					*k = -7.96227766016838;
					k++;
					*k = -0.50467065322830;
									k = gimbal_behaviour_choose_f->Pitch_c.motor_lqr.k;
					*k = -6.0;
					k++;
					*k = -0.3;
			}
    }
}

void gimbal_behaviour_react(gimbal_control_t *gimbal_behaviour_react_f)
{
		Gimbal_pitch -= gimbal_behaviour_react_f->Gimbal_RC->mouse.y * MOUSE_PITCH_SPEED;
    Gimbal_pitch += gimbal_behaviour_react_f->Gimbal_RC->rc.ch[1] * RC_PITCH_SPEED;

    Gimbal_yaw -= gimbal_behaviour_react_f->Gimbal_RC->mouse.x * MOUSE_YAW_SPEED;
    Gimbal_yaw -= gimbal_behaviour_react_f->Gimbal_RC->rc.ch[0] * RC_YAW_SPEED;

    switch (gimbal_behaviour_react_f->gimbal_behaviour)
    {
    case GIMBAL_MANUAL:
        f_GIMBAL_MANUAL(gimbal_behaviour_react_f);
        break;
    case GIMBAL_AUTOATTACK:
        f_GIMBAL_AUTOATTACK(gimbal_behaviour_react_f);
        break;
    case GIMBAL_SMALL_AUTOBUFF:
        f_GIMBAL_AUTOBUFF(gimbal_behaviour_react_f);
        break;
		case GIMBAL_BIG_AUTOBUFF:
        f_GIMBAL_AUTOBUFF(gimbal_behaviour_react_f);
        break;
    default:
        break;
    }
		
		value_limit(Gimbal_pitch, PITCH_ANGLE_LIMIT_DOWN, PITCH_ANGLE_LIMIT_UP);
		#if(YAW_CONTROLER == YAW_USE_PID)
				Gimbal_yaw = loop_fp32_constrain(Gimbal_yaw, 0.0f, 360.0f);
		#elif(YAW_CONTROLER == YAW_USE_LQR)
				Gimbal_yaw = loop_fp32_constrain(Gimbal_yaw, -180.0f, 180.0f);
		#endif
}

void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f)
{
	
}
void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f)
{
	if(((*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw != 0) || ((*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch != 0))
	{
		#ifdef VIRTUAL_DELAY_COMPENSATE
					Gimbal_pitch = (*f_GIMBAL_AUTOATTACK_f->auto_c)->gimbal_use_control[0];
					Gimbal_yaw = (*f_GIMBAL_AUTOATTACK_f->auto_c)->gimbal_use_control[1];
					Gimbal_pitch = first_order_filter(&f_GIMBAL_AUTOATTACK_f->Pitch_c.visual_pitch__first_order_filter,Gimbal_pitch);
					gimbal_clear_virtual_recive();
		#else
					#if(PITCH_ANGLE_SENSOR == PITCH_USE_ENCODER)
					f_GIMBAL_AUTOATTACK_f->Pitch_c.pitch_motor.actPositon_360 = ((float)f_GIMBAL_AUTOATTACK_f->Pitch_c.pitch_motor_encoder->Encode_Actual_Val * 360.0f / 8192.0f - PITCH_ZERO_OFFSET);
					Gimbal_pitch = f_GIMBAL_AUTOATTACK_f->Pitch_c.pitch_motor.actPositon_360 + (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch;
					#elif(PITCH_ANGLE_SENSOR == PITCH_USE_IMU)
					Gimbal_pitch = f_GIMBAL_AUTOATTACK_f->Imu_c->Roll + (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch;
					#endif
					Gimbal_yaw = f_GIMBAL_AUTOATTACK_f->Imu_c->Yaw + (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw;
					Gimbal_pitch = first_order_filter(&f_GIMBAL_AUTOATTACK_f->Pitch_c.visual_pitch__first_order_filter,Gimbal_pitch);
					gimbal_clear_virtual_recive();
		#endif
	}
}
void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f)
{
		if(((*f_GIMBAL_AUTOBUFF_f->auto_c)->auto_yaw != 0) || ((*f_GIMBAL_AUTOBUFF_f->auto_c)->auto_pitch != 0))
	{
		#ifdef VIRTUAL_DELAY_COMPENSATE
					Gimbal_pitch = (*f_GIMBAL_AUTOATTACK_f->auto_c)->gimbal_use_control[0];
					Gimbal_yaw = (*f_GIMBAL_AUTOATTACK_f->auto_c)->gimbal_use_control[1];
					Gimbal_pitch = first_order_filter(&f_GIMBAL_AUTOATTACK_f->Pitch_c.visual_pitch__first_order_filter,Gimbal_pitch);
					gimbal_clear_virtual_recive();
		#else
					#if(PITCH_ANGLE_SENSOR == PITCH_USE_ENCODER)
					f_GIMBAL_AUTOATTACK_f->Pitch_c.pitch_motor.actPositon_360 = ((float)f_GIMBAL_AUTOATTACK_f->Pitch_c.pitch_motor_encoder->Encode_Actual_Val * 360.0f / 8192.0f - PITCH_ZERO_OFFSET);
					Gimbal_pitch = f_GIMBAL_AUTOATTACK_f->Pitch_c.pitch_motor.actPositon_360 + (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch;
					#elif(PITCH_ANGLE_SENSOR == PITCH_USE_IMU)
					Gimbal_pitch = f_GIMBAL_AUTOBUFF_f->Imu_c->Roll + (*f_GIMBAL_AUTOBUFF_f->auto_c)->auto_pitch;
					#endif
					Gimbal_yaw = f_GIMBAL_AUTOBUFF_f->Imu_c->Yaw + (*f_GIMBAL_AUTOBUFF_f->auto_c)->auto_yaw;
					Gimbal_pitch = first_order_filter(&f_GIMBAL_AUTOBUFF_f->Pitch_c.visual_pitch__first_order_filter,Gimbal_pitch);
					gimbal_clear_virtual_recive();
		#endif
	}
}

void gimbal_pid_calculate(gimbal_control_t *gimbal_pid_calculate_f)
{
	 /*-----------------------PITCH--------------------------*/
		#if(PITCH_ANGLE_SENSOR == PITCH_USE_ENCODER)
				gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360 = ((float)gimbal_pid_calculate_f->Pitch_c.pitch_motor_encoder->Encode_Actual_Val * 360.0f / 8192.0f - PITCH_ZERO_OFFSET);
		#endif
		#if(PITCH_CONTROLER == PITCH_USE_PID)
				#if(PITCH_ANGLE_SENSOR == PITCH_USE_ENCODER)
						gimbal_pid_calculate_f->Pitch_c.pitch_motor.set_voltage = motor_position_speed_control(&gimbal_pid_calculate_f->Pitch_c.pitch_motor_speed_pid,
																																																 &gimbal_pid_calculate_f->Pitch_c.pitch_motor_position_pid,
																																																 Gimbal_pitch,
																																																 gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360,
																																																 gimbal_pid_calculate_f->Pitch_c.pitch_motor.motor_measure->speed);
				#elif(PITCH_ANGLE_SENSOR == PITCH_USE_IMU)
						gimbal_pid_calculate_f->Pitch_c.Torque_val = motor_position_speed_control(&gimbal_pid_calculate_f->Pitch_c.pitch_motor_speed_pid, 
                                                                                            &gimbal_pid_calculate_f->Pitch_c.pitch_motor_position_pid, 
                                                                                            Gimbal_pitch,
                                                                                            gimbal_pid_calculate_f->Imu_c->Roll,
                                                                                            gimbal_pid_calculate_f->Pitch_c.pitch_motor->speed);
                        if (gimbal_pid_calculate_f->Imu_c->Roll >= 24 || gimbal_pid_calculate_f->Imu_c->Roll <= -41)
                        {//临界值 电机失能
                            MI_motor_stop(gimbal_pid_calculate_f->Pitch_c.pitch_motor);
                            pid_clear(&gimbal_pid_calculate_f->Pitch_c.pitch_motor_speed_pid);
                            pid_clear(&gimbal_pid_calculate_f->Pitch_c.pitch_motor_speed_pid);
                        }
                        else //P轴电机使能
                        {
                            MI_motor_Write_One_Para(gimbal_pid_calculate_f->Pitch_c.pitch_motor, imit_torque, 10);
                            MI_motor_enable(gimbal_pid_calculate_f->Pitch_c.pitch_motor);
                        }
                            
				#endif
		#elif(PITCH_CONTROLER == PITCH_USE_LQR)
				#if(PITCH_ANGLE_SENSOR == PITCH_USE_ENCODER)
						gimbal_pid_calculate_f->Pitch_c.motor_target = float_min_distance(Gimbal_pitch,gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360, -180, 180);
				#elif(PITCH_ANGLE_SENSOR == PITCH_USE_IMU)
						gimbal_pid_calculate_f->Pitch_c.motor_target = float_min_distance(Gimbal_pitch,gimbal_pid_calculate_f->Imu_c->Roll, -180.0f, 180.0f);
				#endif
			
						float pitch_system_state[2] = {((-gimbal_pid_calculate_f->Pitch_c.motor_target) / 57.295779513f), gimbal_pid_calculate_f->Imu_c->Gyro[1]};
						LQR_Data_Update(&gimbal_pid_calculate_f->Pitch_c.motor_lqr, pitch_system_state);
						if(gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_AUTOATTACK || gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_SMALL_AUTOBUFF || gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_BIG_AUTOBUFF)//自瞄时使用i
						{

				#if(PITCH_ANGLE_SENSOR == PITCH_USE_ENCODER)
							PidCalculate(&gimbal_pid_calculate_f->Pitch_c.qitch_lqr_only_i_pid, Gimbal_pitch, gimbal_pid_calculate_f->Pitch_c.pitch_motor.actPositon_360 );
						}
				#elif(PITCH_ANGLE_SENSOR == PITCH_USE_IMU)
							PidCalculate(&gimbal_pid_calculate_f->Pitch_c.qitch_lqr_only_i_pid, Gimbal_pitch, gimbal_pid_calculate_f->Imu_c->Roll );
						}
						else
						{
							pid_clear(&gimbal_pid_calculate_f->Pitch_c.qitch_lqr_only_i_pid);
						}
						LQR_Calculate(&gimbal_pid_calculate_f->Pitch_c.motor_lqr);
				#endif
						gimbal_pid_calculate_f->Pitch_c.motor_lqr.Output[0] = sliding_mean_filter(&gimbal_pid_calculate_f->Pitch_c.motor_filter, gimbal_pid_calculate_f->Pitch_c.motor_lqr.Output[0],10);
						gimbal_pid_calculate_f->Pitch_c.motor_output = (gimbal_pid_calculate_f->Pitch_c.motor_lqr.Output[0]) + gimbal_pid_calculate_f->Pitch_c.qitch_lqr_only_i_pid.out;
						gimbal_pid_calculate_f->Pitch_c.Torque_val = abs_limit(gimbal_pid_calculate_f->Pitch_c.motor_output, 10);
		#endif
		
		/*-----------------------YAW--------------------------*/
		#if(YAW_CONTROLER == YAW_USE_PID)
				gimbal_pid_calculate_f->Yaw_c.yaw_motor.set_voltage = motor_position_speed_control(&gimbal_pid_calculate_f->Yaw_c.yaw_motor_visual_speed_pid,
																																													 &gimbal_pid_calculate_f->Yaw_c.yaw_motor_visual_position_pid,
																																													 // 视当前位置为0，寻目标的劣弧角度即为控制量
																																													 user_abs(Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) > 180 ? ((Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) > 0 ? ((Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) - 360.0f) : (360.0f - (Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw))) : (Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw),
																																													 0,
																																													 gimbal_pid_calculate_f->Yaw_c.yaw_motor.motor_measure->speed);
		#elif(YAW_CONTROLER == YAW_USE_LQR)
				gimbal_pid_calculate_f->Yaw_c.motor_target = float_min_distance(Gimbal_yaw,gimbal_pid_calculate_f->Imu_c->Yaw, -180.0f, 180.0f);
				float Yaw_system_state[2] = {((-gimbal_pid_calculate_f->Yaw_c.motor_target) / 57.295779513f), gimbal_pid_calculate_f->Imu_c->Gyro[2]};
				
				
				LQR_Data_Update(&gimbal_pid_calculate_f->Yaw_c.motor_lqr, Yaw_system_state);
				LQR_Calculate(&gimbal_pid_calculate_f->Yaw_c.motor_lqr);
						if(gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_AUTOATTACK || gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_SMALL_AUTOBUFF || gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_BIG_AUTOBUFF)//自瞄时使用i
				{
					PidCalculate(&gimbal_pid_calculate_f->Yaw_c.yaw_lqr_only_i_pid, Gimbal_yaw, gimbal_pid_calculate_f->Imu_c->Yaw );
				}
				else
				{
					pid_clear(&gimbal_pid_calculate_f->Yaw_c.yaw_lqr_only_i_pid);
				}
				gimbal_pid_calculate_f->Yaw_c.motor_lqr.Output[0] = sliding_mean_filter(&gimbal_pid_calculate_f->Yaw_c.motor_filter, gimbal_pid_calculate_f->Yaw_c.motor_lqr.Output[0],10);
				gimbal_pid_calculate_f->Yaw_c.motor_output = torque_to_voltage_6020(gimbal_pid_calculate_f->Yaw_c.motor_lqr.Output[0]) + gimbal_pid_calculate_f->Yaw_c.yaw_lqr_only_i_pid.out;
				gimbal_pid_calculate_f->Yaw_c.yaw_motor.set_voltage = abs_limit(gimbal_pid_calculate_f->Yaw_c.motor_output, 25000);
		#endif
}

float torque_to_voltage_6020(float torque)
{
	float voltage = 0.0f;
		
	float current = torque / 0.741f * 10000;
	voltage = (current - 128.3507f) / 0.7778f;
	voltage = abs_limit(voltage,25000);
	
	return voltage;
		
}
