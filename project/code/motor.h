#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_ 

#include "zf_common_headfile.h"
/*************************************
*
*
*           变量及常数定义
*
*
*************************************/
extern volatile int32_t l_out , r_out ;
extern volatile int32_t dutyR , dutyL ;
#define MAX_DUTY        (3000)   // 最大 MAX_DUTY% 占空比
#define MOTOR1_DIR   "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM   "/dev/zf_device_pwm_motor_1"

#define MOTOR2_DIR   "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM   "/dev/zf_device_pwm_motor_2"

// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX    (motor_1_pwm_info.duty_max)     //右轮  
#define MOTOR2_PWM_DUTY_MAX    (motor_2_pwm_info.duty_max)     //左轮   

#define ENCODER_1           "/dev/zf_encoder_1"
#define ENCODER_2           "/dev/zf_encoder_2"
extern int16_t speed_left;
extern int16_t speed_right;
extern int16_t actual_speed;
extern int16_t enconder_left;
extern int16_t enconder_right;
extern volatile int32_t encoder_acc_left;
extern volatile int32_t encoder_acc_right;
extern volatile int32_t encoder_acc_avg;

extern int left_pwm_out,right_pwm_out;
extern uint8 pwm0_flag;
 extern uint8 l_land_flag;                  //左环岛状态位
 extern uint8 r_land_flag;                  //右环岛状态位

/*************************************
 *
 *
 *            函数声明
 *
 *
 ************************************/
void motor_init(void);
void LeftMotor_Ctrl(int32_t dutyL);
void RightMotor_Ctrl(int32_t dutyR);

int32_t l_pid(int set_speed ,int speed);//pid控制左电机转速
int32_t r_pid(int set_speed ,int speed);//pid控制右电机转速
int16 Servo_PID (float Image_err);

void Motor_Control(int32_t l_duty,int32_t r_duty);  //电机控制
void Motor_test(); //电机测试出界保护
/* 编码器反馈刷新函数
 * 负责读取左右轮编码器并更新统一反馈变量：
 * enconder_left / enconder_right / actual_speed
 */
void Get_speed(void);

#endif
