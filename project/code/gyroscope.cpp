//陀螺仪
#include "zf_common_headfile.h"
#include <math.h>



#define GYRO_SENS (1.0f / 16.4f)
#define GYRO_DT 0.0121f
#define GYRO_ZERO_SAMPLE_COUNT 100
#define GYRO_ZERO_SAMPLE_DELAY_MS 2
#define GYRO_WARMUP_COUNT 20

float angle = 0.0f;

int ZeroDrift_gyro_y = 0;
int ZeroDrift_gyro_z = 0;

int GptSpeed = 0;

uint8 Gyro_first = 0;
int gyro_x = 0;
int gyro_y = 0;
float gyro_z = 0.0f;

int Gyro_times = 0;
int Gyro_sum_z = 0;
int Gyro_sum_y = 0;

uint16 count_y = 0;
uint16 count_z = 0;

int16 FJ_gyro_z = 0;
int16 FJ_gyro_y = 0;
float FJ_Angle = 0.0f;
float pitch = 0.0f;

volatile float FJ_Pitch = 0.0f;
volatile float FJ_PitchSpeed = 0.0f;
volatile float FJ_LastPitchSpeed = 0.0f;
volatile float FJ_LastAngleSpeed = 0.0f;
volatile float FJ_AngleSpeed = 0.0f;
volatile float FJ_Angle_Max = 360.0f;
volatile float FJ_Angle_Min = -360.0f;
volatile float AngleSpeed = 0.0f;

static uint8 gyroscope_inited = 0;

static void imu660ra_publish_measurement(void)
{
    imu_acc_x = imu660ra_acc_x;
    imu_acc_y = imu660ra_acc_y;
    imu_acc_z = imu660ra_acc_z;
    imu_gyro_x = imu660ra_gyro_x;
    imu_gyro_y = imu660ra_gyro_y;
    imu_gyro_z = imu660ra_gyro_z;
}

void gyroscope_reset_runtime(void)
{
    angle = 0.0f;
    gyro_x = 0;
    gyro_y = 0;
    gyro_z = 0.0f;
    pitch = 0.0f;
    AngleSpeed = 0.0f;
    FJ_gyro_z = 0;
    FJ_gyro_y = 0;
    FJ_Angle = 0.0f;
    FJ_Pitch = 0.0f;
    FJ_PitchSpeed = 0.0f;
    FJ_LastPitchSpeed = 0.0f;
    FJ_LastAngleSpeed = 0.0f;
    FJ_AngleSpeed = 0.0f;
}
//----------------------------------------------------------------------------------------------------------------
// 函数名称 Zero_Point_Detect
// 函数简介 零点检测
// 参数说明
// 返回参数
// 使用示例 Zero_Point_Detect();
// 备注信息
//----------------------------------------------------------------------------------------------------------------


void Zero_Point_Detect(void)
{
    int zero_point_y_accu = 0;
    int zero_point_z_accu = 0;

    for(int i = 0; i < GYRO_ZERO_SAMPLE_COUNT; i++)
    {
        imu660ra_get_gyro();
        zero_point_y_accu += imu660ra_gyro_y;
        zero_point_z_accu += imu660ra_gyro_z;
        system_delay_ms(GYRO_ZERO_SAMPLE_DELAY_MS);
    }

    ZeroDrift_gyro_y = zero_point_y_accu / GYRO_ZERO_SAMPLE_COUNT;
    ZeroDrift_gyro_z = zero_point_z_accu / GYRO_ZERO_SAMPLE_COUNT;
}

void gyroscope_init(void)
{
    imu_type = DEV_IMU660RA;

    for(int i = 0; i < GYRO_WARMUP_COUNT; i++)
    {
        imu660ra_get_gyro();
        imu660ra_get_acc();
        system_delay_ms(GYRO_ZERO_SAMPLE_DELAY_MS);
    }

    Zero_Point_Detect();
    gyroscope_reset_runtime();
    get_gyro();
    gyroscope_inited = 1;
}

float calculate_pitch_angle(void)
{
    const float ax = (float)imu_acc_x;
    const float ay = (float)imu_acc_y;
    const float az = (float)imu_acc_z;

    const float pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));
    return pitch_rad * 180.0f / (float)M_PI;
}

void Gyroscope_GetData(void)
{
    AngleSpeed = ((float)(imu_gyro_z - ZeroDrift_gyro_z) * GYRO_SENS) * GYRO_DT;
}

void Get_Gyroscope_Angle(void)
{
    const float K = 0.75f;

    FJ_gyro_z = imu_gyro_z;
    FJ_LastAngleSpeed = FJ_AngleSpeed;
    FJ_AngleSpeed += ((float)(FJ_gyro_z - ZeroDrift_gyro_z) * GYRO_SENS) * GYRO_DT;
    FJ_Angle = FJ_AngleSpeed * K + FJ_LastAngleSpeed * (1.0f - K);
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

void Get_Gyroscope_Pitch(void)
{
    const float K = 0.7f;

    FJ_LastPitchSpeed = FJ_PitchSpeed;
    FJ_gyro_y = imu_gyro_y;
    FJ_PitchSpeed += ((float)(FJ_gyro_y - ZeroDrift_gyro_y) * GYRO_SENS) * GYRO_DT;
    FJ_Pitch = FJ_PitchSpeed * K + FJ_LastPitchSpeed * (1.0f - K);
    FJ_Pitch = FJ_Pitch > 40.0f ? 40.0f : FJ_Pitch;
    FJ_Pitch = FJ_Pitch < -40.0f ? -40.0f : FJ_Pitch;
}
//----------------------------------------------------------------------------------------------------------------
// 函数名称 Clear_Gyroscope_Pitch
// 函数简介 清算俯仰角
// 参数说明
// 返回参数
// 使用示例 Clear_Gyroscope_Pitch();
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void Clear_Gyroscope_Pitch(void)
{
    FJ_Pitch = 0.0f;
    FJ_gyro_y = 0;
    FJ_PitchSpeed = 0.0f;
    FJ_LastPitchSpeed = 0.0f;
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 Clear_Gyroscope_Angle
// 函数简介 清算航偏角
// 参数说明
// 返回参数
// 使用示例 Clear_Gyroscope_Angle();
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void Clear_Gyroscope_Angle(void)
{
    FJ_Angle = 0.0f;
    FJ_gyro_z = 0;
    FJ_AngleSpeed = 0.0f;
    FJ_LastAngleSpeed = 0.0f;
}

void get_gyro(void)
{
    imu660ra_get_gyro();
    imu660ra_get_acc();
    imu660ra_publish_measurement();

    gyro_x = imu_gyro_x;
    gyro_y = imu_gyro_y;
    gyro_z = (float)(imu_gyro_z - ZeroDrift_gyro_z);

    if(gyroscope_inited != 0)
    {
        Gyroscope_GetData();
        Get_Gyroscope_Angle();
    }

    if(l_land_flag != 0)
    {
        if(imu_acc_y < AY_EXIT_THRESHOLD && imu_acc_y > -AY_EXIT_THRESHOLD)
        {
            imu_ring_exit_counter++;
        }
        else
        {
            imu_ring_exit_counter = 0;
        }
    }

    if(barrier_flag != 0)
    {
        pitch = calculate_pitch_angle();
    }
}

void gyroscope_interrupt_handler(void)
{
    get_gyro();
}

void call_gyroscope_thread(void)
{
    get_gyro();
}
