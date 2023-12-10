/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       Task_OutOfC.c/h
  * @brief      失控任务
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/
#ifndef __TASK_IMU_H  //如果未定义
#define __TASK_IMU_H  //那么定义
#include "main.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "cmsis_os.h"


#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include "bsp_pwm.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];
    float Accel[3];
    float MotionAccel_b[3];
    float MotionAccel_n[3];

    float AccelLPF;

    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern INS_t INS;

extern void INS_Init(void);
extern void INS_Task(void);
extern void IMU_Temperature_Ctrl(void);

extern void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
extern void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
extern void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
extern void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
extern void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
extern const INS_t *get_imu_control_point(void);
extern void imu_app_init(void);
extern void imu_Task(void const * argument);
//extern TaskHandle_t *get_imu_task_handle(void);

#endif




