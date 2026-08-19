
#include "zf_common_headfile.h"
#include "gyroscope.h"

#define ServoMID  545
//电机 初始化参数
struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;

volatile int32_t l_out = 0, r_out = 0;
int left_pwm_out = 0, right_pwm_out = 0;
//编码器 初始化参数
int16 speed_left = 0;
int16 speed_right = 0;
int16 actual_speed = 0;
int16_t enconder_left = 0;
int16_t enconder_right = 0;
volatile int32_t encoder_acc_left = 0;
volatile int32_t encoder_acc_right = 0;
volatile int32_t encoder_acc_avg = 0;
//标志位
uint8 pwm0_flag = 0;
//参数
float P_R_island = 200.55f;
float I_R_island = 60.5f;

float P_L = 200.55f;
float I_L = 70.5f;
float P_R = 200.55f;
float I_R = 70.5f;

float P_L_island = 200.55f;
float I_L_island = 60.5f;

// 调参：位置环线性比例项。增大转向响应更强，过大容易左右摆。
volatile float servo_pid_kp = 3.0f;
// 调参：位置环二次比例项。增大后大误差时更激进，当前先保持 0。
volatile float servo_pid_kp2 = 0.0f;
// 调参：图像误差差分项。增大可抑制过冲，过大可能转向迟钝。
volatile float servo_pid_kd = 0.01f;
// 调参：陀螺仪 Z 轴修正项。增大可抑制车身旋转，过大可能压制正常转弯。
volatile float servo_pid_gkd = 0.015f;

// 调参：左轮速度内环。kp/ki 越大响应越强，A 越大输出滤波越弱。
volatile float motor_l_kp = 13.0f;//13.6
volatile float motor_l_ki = 2.0f;
volatile float motor_l_filter_a = 0.95f;
// 调参：右轮速度内环。左右轮机械差异较大时允许单独调。
volatile float motor_r_kp = 13.0f;//13.6
volatile float motor_r_ki = 2.0f;
volatile float motor_r_filter_a = 0.95f;

volatile int motor_limit = 5000;

void motor_init()
{
    pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);
}

int32_t l_pid(int set_speed, int speed)
{
    static int32_t out = 0;
    static int32_t out_increment = 0;
    static int32_t ek = 0, ek1 = 0;
    static float out_last = 0.0f;

    const float kp = motor_l_kp;
    const float ki = motor_l_ki;
    const float A = motor_l_filter_a;

    ek1 = ek;
    ek = set_speed - speed;
    out_increment = (int32_t)(kp * (ek - ek1) + ki * ek);
    out += out_increment;

    const float filtered_out = out * A + out_last * (1.0f - A);
    out_last = filtered_out;
    out = (int32_t)filtered_out;

    if(out >= motor_limit)
    {
        out = motor_limit;
    }
    else if(out <= -motor_limit)
    {
        out = -motor_limit;
    }
    return out;
}

int32_t r_pid(int set_speed, int speed)
{
    static int32_t out = 0;
    static int32_t out_increment = 0;
    static int32_t ek = 0, ek1 = 0;
    static float out_last = 0.0f;

    const float kp = motor_r_kp;
    const float ki = motor_r_ki;
    const float A = motor_r_filter_a;

    ek1 = ek;
    ek = set_speed - speed;
    out_increment = (int32_t)(kp * (ek - ek1) + ki * ek);
    out += out_increment;

    const float filtered_out = out * A + out_last * (1.0f - A);
    out_last = filtered_out;
    out = (int32_t)filtered_out;

    if(out >= motor_limit)
    {
        out = motor_limit;
    }
    else if(out <= -motor_limit)
    {
        out = -motor_limit;
    }
    return out;
}

int16 Servo_PID(float Image_err)
{
    volatile static int16 out = ServoMID;
    volatile static int out_increment;
    volatile static int16 err_last = 0;
    volatile static int16 err = 0;

    const float Kp = servo_pid_kp;
    const float Kp2 = servo_pid_kp2;
    const float Kd = servo_pid_kd;
    const float GKD = servo_pid_gkd;

    err_last = err;
    err = Image_err;
    out_increment = err * Kp + err * func_abs(err) * Kp2 - (err - err_last) * Kd
           - gyro_z * GKD;
    if(out_increment > 0)
    {
        out_increment = out_increment * 1.0f;
    }
    out = out_increment;
    out = func_limit_ab(out, -2050, 2050);
    return out;
}

void LeftMotor_Ctrl(int32_t dutyL)
{
    if(dutyL >= motor_limit)
    {
        dutyL = motor_limit;
    }
    else if(dutyL <= -motor_limit)
    {
        dutyL = -motor_limit;
    }

    if(dutyL >= 0)
    {
        gpio_set_level(MOTOR2_DIR, 0);
        pwm_set_duty(MOTOR2_PWM, dutyL);
    }
    else
    {
        gpio_set_level(MOTOR2_DIR, 1);
        pwm_set_duty(MOTOR2_PWM, -dutyL);
    }
}

void RightMotor_Ctrl(int32_t dutyR)
{
    if(dutyR >= motor_limit)
    {
        dutyR = motor_limit;
    }
    else if(dutyR <= -motor_limit)
    {
        dutyR = -motor_limit;
    }

    if(dutyR >= 0)
    {
        gpio_set_level(MOTOR1_DIR, 0);
        pwm_set_duty(MOTOR1_PWM, dutyR);
    }
    else
    {
        gpio_set_level(MOTOR1_DIR, 1);
        pwm_set_duty(MOTOR1_PWM, -dutyR);
    }
}

void Motor_Control(int32_t l_duty, int32_t r_duty)
{
    LeftMotor_Ctrl(l_duty);
    RightMotor_Ctrl(r_duty);
}

void Motor_test()
{
    left_pwm_out = 1000;
    right_pwm_out = 1000;

    if(Car_ShouldMotorStop())
    {
        time1++;
        timestop = 0;
        if(time1 >= 50)
        {
            pwm0_flag = 1;
        }
    }
    else
    {
        timestop++;
        time1 = 0;
    }
    if(pwm0_flag == 1)
    {
        left_pwm_out = 0;
        right_pwm_out = 0;
    }
    LeftMotor_Ctrl(left_pwm_out);
    RightMotor_Ctrl(right_pwm_out);
}

void Get_speed(void)
{
    enconder_left = encoder_get_count(ENCODER_1);
    enconder_right = -encoder_get_count(ENCODER_2);
    encoder_acc_left += enconder_left;
    encoder_acc_right += enconder_right;
    encoder_acc_avg = encoder_acc_left / 2 + encoder_acc_right / 2;
    actual_speed = (enconder_left + enconder_right) / 2;
}
