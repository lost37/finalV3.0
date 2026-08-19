//陀螺仪
// 陀螺仪处理流程。
// 参考：E:\Virtual Machines\temp\Abandoned-main\project\code\IMU963R.cpp
// 核心思路相同：先预热传感器，再静止采样求零偏，之后周期采样时减去零偏。
// 当前差异：这里只校准 Y/Z 轴，并对 gyro_z 做一阶低通滤波；没有温漂补偿或运行时零偏更新。
#include "zf_common_headfile.h"
#include <math.h>



// IMU660RA 在常用 +/-2000dps 量程下的陀螺仪比例系数。
// 原始陀螺仪差值 * GYRO_SENS -> deg/s；再乘 GYRO_DT -> 单周期角度增量。
#define GYRO_SENS (1.0f / 16.4f)
#define GYRO_DT 0.0121f
// 上电零偏校准：小车静止时采样 130 次，每次间隔 5ms。
#define GYRO_ZERO_SAMPLE_COUNT 220
#define GYRO_ZERO_SAMPLE_DELAY_MS 5
// 零偏校准前先丢弃若干次刚启动时不稳定的传感器读数。
#define GYRO_WARMUP_COUNT 20
// gyro_z 一阶低通滤波截止频率。参考 filt.cpp 中 LPF_1(20, 5.0e-3, ...) 的滤波强度。
#define GYRO_Z_LPF_CUTOFF_HZ 20.0f
// Z 轴运行时零漂补偿。使用原始陀螺仪单位，不改变 gyro_z/GKD 的现有调参尺度。
#define GYRO_ONLINE_BIAS_ACC_1G 4096.0f
#define GYRO_ONLINE_BIAS_ACC_TOL 300.0f
#define GYRO_ONLINE_BIAS_GYRO_LIMIT 25.0f
#define GYRO_ONLINE_BIAS_START_COUNT 50
#define GYRO_ONLINE_BIAS_LEARN_RATE 0.001f
#define GYRO_ONLINE_BIAS_LIMIT 20.0f

float angle = 0.0f;

int ZeroDrift_gyro_y = 0;
int ZeroDrift_gyro_z = 0;

int GptSpeed = 0;

uint8 Gyro_first = 0;
int gyro_x = 0;
int gyro_y = 0;
float gyro_z = 0.0f;
static uint8 gyro_z_filter_ready = 0;
static float gyro_z_online_bias = 0.0f;
static uint16 gyro_z_online_static_count = 0;

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

static float clamp_float(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

// 一阶低通滤波器。hz 为截止频率，time 为调用周期，out 保存上次滤波结果。
static void LPF_1(float hz, float time, float in, float *out)
{
    const float alpha = 1.0f / (1.0f + 1.0f / (hz * 6.28f * time));
    *out += alpha * (in - *out);
}

// 在线 Z 轴零漂补偿：只有在接近静止/直行低角速度时慢速学习，弯道大角速度时不更新。
static void update_gyro_z_online_bias(float gyro_z_static_raw)
{
    const float acc_x = (float)imu_acc_x;
    const float acc_y = (float)imu_acc_y;
    const float acc_z = (float)imu_acc_z;
    const float acc_norm = sqrtf(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
    const float gyro_y_static_raw = (float)(imu_gyro_y - ZeroDrift_gyro_y);
    const uint8 acc_near_1g = (fabsf(acc_norm - GYRO_ONLINE_BIAS_ACC_1G) < GYRO_ONLINE_BIAS_ACC_TOL);
    const uint8 gyro_near_zero = (fabsf(gyro_z_static_raw) < GYRO_ONLINE_BIAS_GYRO_LIMIT
                                  && fabsf(gyro_y_static_raw) < GYRO_ONLINE_BIAS_GYRO_LIMIT);

    if(acc_near_1g != 0 && gyro_near_zero != 0)
    {
        if(gyro_z_online_static_count < GYRO_ONLINE_BIAS_START_COUNT)
        {
            gyro_z_online_static_count++;
            return;
        }

        const float residual = gyro_z_static_raw - gyro_z_online_bias;
        gyro_z_online_bias += residual * GYRO_ONLINE_BIAS_LEARN_RATE;
        gyro_z_online_bias = clamp_float(gyro_z_online_bias, -GYRO_ONLINE_BIAS_LIMIT, GYRO_ONLINE_BIAS_LIMIT);
    }
    else
    {
        gyro_z_online_static_count = 0;
    }
}

static float get_gyro_z_compensated_raw(void)
{
    return (float)(imu_gyro_z - ZeroDrift_gyro_z) - gyro_z_online_bias;
}

// 将 IMU660RA 驱动层读数发布到项目统一 IMU 全局变量。
// 其他模块尽量读取这些统一变量，避免直接混用驱动层专用变量名。
static void imu660ra_publish_measurement(void)
{
    imu_acc_x = imu660ra_acc_x;
    imu_acc_y = imu660ra_acc_y;
    imu_acc_z = imu660ra_acc_z;
    imu_gyro_x = imu660ra_gyro_x;
    imu_gyro_y = imu660ra_gyro_y;
    imu_gyro_z = imu660ra_gyro_z;
}

// 只清空运行时积分/滤波状态。
// 不清空 ZeroDrift_gyro_y / ZeroDrift_gyro_z，因此上电校准得到的零偏仍然有效。
void gyroscope_reset_runtime(void)
{
    angle = 0.0f;
    gyro_x = 0;
    gyro_y = 0;
    gyro_z = 0.0f;
    gyro_z_filter_ready = 0;
    gyro_z_online_bias = 0.0f;
    gyro_z_online_static_count = 0;
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

// 静止零偏校准。
// 对应参考代码中的 calibrate_offsets()：小车静止时连续采样并求平均。
// 本项目保存 Y/Z 轴原始零偏，后续在 get_gyro() 和角度积分中再减去。
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

// 初始化流程：
// 1. 固定使用 IMU660RA。
// 2. 预热读取，丢弃启动初期不稳定数据。
// 3. 静止采样，计算零偏。
// 4. 清空运行时积分状态。
// 5. 控制开始前先发布一帧新数据。
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

// 通过加速度计估算俯仰角，主要用于坡道/障碍相关逻辑，不用于循迹位置环陀螺抑制。
float calculate_pitch_angle(void)
{
    const float ax = (float)imu_acc_x;
    const float ay = (float)imu_acc_y;
    const float az = (float)imu_acc_z;

    const float pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));
    return pitch_rad * 180.0f / (float)M_PI;
}

// 计算减去上电零偏后的 Z 轴单周期角度增量。
void Gyroscope_GetData(void)
{
    AngleSpeed = (get_gyro_z_compensated_raw() * GYRO_SENS) * GYRO_DT;
}

// 将 Z 轴陀螺仪积分成类似航向角的量，并做简单一阶平滑。
// 当前已加入慢速在线零漂补偿，但没有磁力计/视觉闭环，长时间积分仍可能累计漂移。
void Get_Gyroscope_Angle(void)
{
    const float K = 0.75f;

    FJ_gyro_z = imu_gyro_z;
    FJ_LastAngleSpeed = FJ_AngleSpeed;
    FJ_AngleSpeed += (get_gyro_z_compensated_raw() * GYRO_SENS) * GYRO_DT;
    FJ_Angle = FJ_AngleSpeed * K + FJ_LastAngleSpeed * (1.0f - K);
    FJ_Angle = FJ_Angle > FJ_Angle_Max ? FJ_Angle_Max : FJ_Angle;
    FJ_Angle = FJ_Angle < FJ_Angle_Min ? FJ_Angle_Min : FJ_Angle;
    FJ_Angle = -FJ_Angle;
}

// 对 Y 轴陀螺仪积分，用于俯仰相关状态。
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

void Clear_Gyroscope_Pitch(void)
{
    FJ_Pitch = 0.0f;
    FJ_gyro_y = 0;
    FJ_PitchSpeed = 0.0f;
    FJ_LastPitchSpeed = 0.0f;
}

void Clear_Gyroscope_Angle(void)
{
    FJ_Angle = 0.0f;
    FJ_gyro_z = 0;
    FJ_AngleSpeed = 0.0f;
    FJ_LastAngleSpeed = 0.0f;
}

// 周期 IMU 更新入口。
// 和参考代码 update() 相比，这里会读取 gyro/acc、减去零偏，并对 gyro_z 做一阶低通滤波。
// gyro_z 是位置环 GKD 使用的车身旋转反馈，滤波后可减少瞬时噪声造成的左右抖动。
void get_gyro(void)
{
    imu660ra_get_gyro();
    imu660ra_get_acc();
    imu660ra_publish_measurement();

    gyro_x = imu_gyro_x;
    gyro_y = imu_gyro_y;
    const float gyro_z_static_raw = (float)(imu_gyro_z - ZeroDrift_gyro_z);
    if(gyroscope_inited != 0)
    {
        update_gyro_z_online_bias(gyro_z_static_raw);
    }

    const float gyro_z_raw = get_gyro_z_compensated_raw();
    if(gyro_z_filter_ready == 0)
    {
        gyro_z = gyro_z_raw;
        gyro_z_filter_ready = 1;
    }
    else
    {
        LPF_1(GYRO_Z_LPF_CUTOFF_HZ, GYRO_DT, gyro_z_raw, &gyro_z);
    }

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
