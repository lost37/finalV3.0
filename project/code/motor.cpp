/*
 * 该文件作用：电机调参
 * motor.cpp
 */
#include "zf_common_headfile.h"

#define ServoMID  545        //速度变大，可以减小 545
//#define ServoLMAX ServoMID+640      //7993     78
//#define ServoRMAX ServoMID-640       //6433     75
//电机 初始化参数
struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;

volatile int32_t l_out=0 , r_out=0 ;
int left_pwm_out=0,right_pwm_out=0;

//编码器 初始化参数
int16 speed_left = 0;
int16 speed_right = 0;
int16 actual_speed = 0;
int16_t enconder_left = 0;
int16_t enconder_right = 0;

//标志位
uint8 pwm0_flag=0;

//参数
float P_R_island=200.55;
float I_R_island=60.5;

float P_L=200.55;//80.55      116.55
float I_L=70.5;//             43.5
float P_R=200.55;//80.55      116.55
float I_R=70.5;//             43.5

float P_L_island=200.55;//1.19
float I_L_island=60.5;

volatile int motor_limit = 5000; //电机限幅
//----------------------------------------------------------------------------------------------------------------
// 函数名称 motor_init
// 函数简介 电机初始化
// 参数说明
// 返回参数
// 使用示例 motor_init();
// 备注信息
//----------------------------------------------------------------------------------------------------------------

void motor_init()
{
    // 获取PWM设备信息
    pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);
}
//----------------------------------------------------------------------------------------------------------------
// 函数名称 l_pid
// 函数简介 左电机pid计算
// 使用示例
//----------------------------------------------------------------------------------------------------------------

int32_t l_pid(int set_speed ,int speed)//pid控制左电机转速
{
    static int32_t out=0;
    static int32_t out_increment=0;  //输出增量
    static int32_t ek=0,ek1=0;
    static float out_last = 0;

    float kp=14.25;
    float ki=7.63;
    float A = 0.95;

    ek1 = ek;                   //上一次误差
    ek = set_speed - speed;     //当前误差
    out_increment= (int)(kp*(ek-ek1) + ki*ek);
    out+= out_increment;
    float filtered_out = out * A + out_last * (1.0 - A);
    out_last = filtered_out;
    out = (int32_t)filtered_out;
    // if(l_land_flag||r_land_flag)//
    // {
    //     kp = P_L_island;
    //     ki = I_L_island;
    // }
    // else//发车阶段速度环要硬
    // {
    //     kp = P_L;
    //     ki = I_L;
    // }
    if(out>=motor_limit) out=motor_limit;
    else if(out<=-motor_limit) out=(-motor_limit);
    return  out;
}
//----------------------------------------------------------------------------------------------------------------
// 函数名称 r_pid
// 函数简介 右电机pid计算
// 使用示例
//----------------------------------------------------------------------------------------------------------------

int32_t r_pid(int set_speed ,int speed)//pid控制右电机转速
{
    static int32_t out=0;
    static int32_t out_increment=0;  //输出增量
    static int32_t ek=0,ek1=0;
    static float out_last = 0;

    float kp=14.25;
    float ki=7.63;
    float A = 0.95;

    // float kp=14.2;
    // float ki=7.43;


    ek1 = ek;                   //上一次误差
    ek = set_speed - speed;     //当前误差
    out_increment= (int)(kp*(ek-ek1) + ki*ek);
    out+= out_increment;
    float filtered_out = out * A + out_last * (1.0 - A);
    out_last = filtered_out;
    out = (int32_t)filtered_out;
    // if(l_land_flag||r_land_flag)//
    // {
    //     kp = P_R_island;// 一套pi足矣，速度拨动不会太大
    //     ki = I_R_island;
    // }
    // else//发车阶段速度环要硬
    // {
    //     kp = P_R;//一套pi足矣，速度拨动不会太大  20
    //     ki = I_R;//0.9
    // }
    if(out>=motor_limit) out=motor_limit;
    else if(out<=-motor_limit) out=(-motor_limit);
    return  out;

}



int16 Servo_PID (float Image_err)
{
    volatile static int16 out = ServoMID;
    volatile static int out_increment;  //输出变化量

    volatile static int16 err_last = 0;
    volatile static int16 err = 0;
    float Kp=9.0;
    float Kp2=0;
    float Kd=5.4;
    float GKD=0.02;//imu660ra_gyro_transition(imu660ra_gyro_z)6.71
    // float Kp=7;
    // float Kp2=0;
    // float Kd=5.2;
    // float GKD=6.5;//imu660ra_gyro_transition(imu660ra_gyro_z)
    
    //  Servo.Kp = ??;        //动态p值变化函数
    err_last = err;
    err = Image_err;
    out_increment = err * Kp + err * func_abs(err) * Kp2 - (err - err_last) * Kd
           - imu660ra_gyro_z * GKD;//imu660ra_gyro_transition(imu660ra_gyro_z)
    if(out_increment > 0)
    {
        out_increment = out_increment * 1.0;    //假如左转输出误差值乘以0.9
    }
    out = out_increment;
    out = func_limit_ab(out, -2000, 2000); //限幅
    return out;
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 LeftMotor_Ctrl
// 函数简介 左电机控制
// 参数说明 dutyL左电机占空比
// 返回参数
// 使用示例
// 备注信息
//----------------------------------------------------------------------------------------------------------------

void LeftMotor_Ctrl(int32_t dutyL)  
{
    if(dutyL>=motor_limit)     //电机限幅
    dutyL=motor_limit;
    else if(dutyL<=(-motor_limit))
    dutyL=(-motor_limit);

    if(dutyL >= 0)                                                           // 正转
    {   
        gpio_set_level(MOTOR2_DIR, 0);                                      // DIR输出高电平
        pwm_set_duty(MOTOR2_PWM, dutyL);       // 计算占空比
    }
    else
    {                                             
        gpio_set_level(MOTOR2_DIR, 1);                                      // DIR输出低电平
        pwm_set_duty(MOTOR2_PWM, -dutyL);      // 计算占空比
    }
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 RightMotor_Ctrl
// 函数简介 右电机控制
// 参数说明 dutyR右电机占空比
// 返回参数
// 使用示例
// 备注信息
//----------------------------------------------------------------------------------------------------------------

//右电机控制
void RightMotor_Ctrl(int32_t dutyR)
{
    if(dutyR>=motor_limit)     //电机限幅
    dutyR=motor_limit;
    else if(dutyR<=(-motor_limit))
    dutyR=(-motor_limit);
    
    if(dutyR >= 0)                                                           // 正转
    {
        gpio_set_level(MOTOR1_DIR, 0);                                      // DIR输出高电平
        pwm_set_duty(MOTOR1_PWM, dutyR);       // 计算占空比 
    }
    else
    {
        gpio_set_level(MOTOR1_DIR, 1);                                      // DIR输出低电平
        pwm_set_duty(MOTOR1_PWM, -dutyR);      // 计算占空比
    }
}

void Motor_Control(int32_t l_duty ,int32_t r_duty)
{
    LeftMotor_Ctrl(l_duty);
    RightMotor_Ctrl(r_duty);
}

void Motor_test()  //开环
{
    left_pwm_out=1000;
    right_pwm_out=1000;


    if(Car_ShouldMotorStop())  //停车  || xie_cross_flag > 0 || cross_flag > 0
    {
        time1++;
        timestop=0;
        if(time1>=50)
        {
            pwm0_flag=1;
        }
    }
    else
    {
        timestop++;
        time1=0;
    }
    if(pwm0_flag==1)
    {
        left_pwm_out=0;
        right_pwm_out=0;
    }
    LeftMotor_Ctrl(left_pwm_out);
    RightMotor_Ctrl(right_pwm_out);
}

//获取编码器值 获取速度
void Get_speed(void)
{
  enconder_left  = encoder_get_count(ENCODER_1);
  enconder_right= -encoder_get_count(ENCODER_2);
  actual_speed = (enconder_left + enconder_right) / 2; //实际速度
}
