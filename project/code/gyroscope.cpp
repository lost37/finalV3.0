#include "zf_common_headfile.h"
#include <time.h>
#include <math.h>

float angle = 0;


/*陀螺仪*/
int ZeroDrift_gyro_y=0;
int ZeroDrift_gyro_z=0;

int GptSpeed = 0;

uint8 Gyro_first=0;
int gyro_x;
int gyro_y;
float gyro_z=0;

int Gyro_times=0;
int Gyro_sum_z = 0;
int Gyro_sum_y = 0;


uint16 count_y=0;
uint16 count_z=0;

int16 FJ_gyro_z = 0,FJ_gyro_y = 0;
float FJ_Angle = 0;//最后引出的变量//航角

float pitch = 0;


volatile float FJ_Pitch = 0;//最后引出的变量//仰角
volatile float FJ_PitchSpeed = 0,FJ_LastPitchSpeed = 0;
volatile float FJ_LastAngleSpeed = 0,FJ_AngleSpeed = 0;
volatile float FJ_Angle_Max =  360;
volatile float FJ_Angle_Min = -360;
volatile float AngleSpeed = 0;
#define   GYRO_SENS             1/16.4
#define   ACCE_SENS             90.0/4096
#define   DT                    0.0121  //  0.01

int zero_point_y_accu=0;
int zero_point_z_accu=0;



//----------------------------------------------------------------------------------------------------------------
// 函数名称 get_gyro
// 函数简介 读取陀螺仪数据
// 参数说明
// 返回参数
// 使用示例 get_gyro();
// 备注信息
//----------------------------------------------------------------------------------------------------------------

void get_gyro() {
    imu660ra_get_gyro();
    imu660ra_get_acc();
    if(l_land_flag != 0) 
    {
        if (imu660ra_acc_y < AY_EXIT_THRESHOLD && imu660ra_acc_y > -AY_EXIT_THRESHOLD)
            imu_ring_exit_counter++;
        else
            imu_ring_exit_counter = 0;
    }
    if(barrier_flag != 0)
    {
        pitch = calculate_pitch_angle();
    }
}


float calculate_pitch_angle()
{
    float ax = imu660ra_acc_x;
    float ay = imu660ra_acc_y;
    float az = imu660ra_acc_z;

    // 转换成弧度，再转角度
    float pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));
    float pitch_deg = pitch_rad * 180.0f / M_PI;

    return pitch_deg;
}
//----------------------------------------------------------------------------------------------------------------
// 函数名称 Zero_Point_Detect
// 函数简介 零点检测
// 参数说明
// 返回参数
// 使用示例 Zero_Point_Detect();
// 备注信息
//----------------------------------------------------------------------------------------------------------------

void Zero_Point_Detect() {
    uint8 i;

    for (i = 0; i < 100; i++) { // 积累100次，求取平均值，获取当前零飘
        imu660ra_get_gyro();
        zero_point_y_accu += imu660ra_gyro_y;
        zero_point_z_accu += imu660ra_gyro_z;
    }
    ZeroDrift_gyro_y = zero_point_y_accu / 100.0;
    ZeroDrift_gyro_z = zero_point_z_accu / 100.0;
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 Gyroscope_GetData
// 函数简介 获取陀螺仪数据
// 参数说明
// 返回参数
// 使用示例 Gyroscope_GetData();
// 备注信息
//----------------------------------------------------------------------------------------------------------------

void Gyroscope_GetData() {
    int16 gyro_z = 0;
    imu660ra_get_gyro();
    gyro_z = imu660ra_gyro_z;
    AngleSpeed = ((gyro_z - ZeroDrift_gyro_z) * GYRO_SENS) * DT;
    //printf("a:%d",(int)AngleSpeed);
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 Get_Gyroscope_Angle
// 函数简介 获取陀螺仪航偏角
// 参数说明
// 返回参数
// 使用示例 Get_Gyroscope_Angle();
// 备注信息
//----------------------------------------------------------------------------------------------------------------

void Get_Gyroscope_Angle() {
    float K = 0.75;
    FJ_gyro_z = imu660ra_gyro_z;
    FJ_LastAngleSpeed = FJ_AngleSpeed;
    FJ_AngleSpeed += ((FJ_gyro_z - ZeroDrift_gyro_z) * GYRO_SENS) * DT;
    FJ_Angle = FJ_AngleSpeed * K + FJ_LastAngleSpeed * (1 - K);
    FJ_Angle = FJ_Angle > FJ_Angle_Max ? FJ_Angle_Max : FJ_Angle;
    FJ_Angle = FJ_Angle < FJ_Angle_Min ? FJ_Angle_Min : FJ_Angle;
    FJ_Angle = -FJ_Angle;
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 Get_Gyroscope_Pitch
// 函数简介 获取陀螺仪俯仰角
// 参数说明
// 返回参数
// 使用示例 Get_Gyroscope_Pitch();
// 备注信息
//----------------------------------------------------------------------------------------------------------------

void Get_Gyroscope_Pitch() {
    float K = 0.7;
    FJ_LastPitchSpeed = FJ_PitchSpeed;
    FJ_gyro_y = imu660ra_gyro_y;
    FJ_PitchSpeed += ((FJ_gyro_y - ZeroDrift_gyro_y) * GYRO_SENS) * DT;
    FJ_Pitch = FJ_PitchSpeed * K + FJ_LastPitchSpeed * (1 - K);

    FJ_Pitch = FJ_Pitch > 40 ? 40 : FJ_Pitch;
    FJ_Pitch = FJ_Pitch < -40 ? -40 : FJ_Pitch;
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 Clear_Gyroscope_Pitch
// 函数简介 清算俯仰角
// 参数说明
// 返回参数
// 使用示例 Clear_Gyroscope_Pitch();
// 备注信息
//----------------------------------------------------------------------------------------------------------------

void Clear_Gyroscope_Pitch() {
    FJ_Pitch = 0;
    FJ_gyro_y = 0;
    FJ_PitchSpeed = 0;
    FJ_LastPitchSpeed = 0;
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 Clear_Gyroscope_Angle
// 函数简介 清算航偏角
// 参数说明
// 返回参数
// 使用示例 Clear_Gyroscope_Angle();
// 备注信息
//----------------------------------------------------------------------------------------------------------------

void Clear_Gyroscope_Angle() {
    FJ_Angle = 0;
    FJ_gyro_z = 0;
    FJ_AngleSpeed = 0;
    FJ_LastAngleSpeed = 0;
}