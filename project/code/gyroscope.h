#ifndef CODE_ADC_H_
#define CODE_ADC_H_ 

#include "zf_common_headfile.h"
/*************************************
*
*
*           变量及常数定义
*
*
*************************************/
extern float pitch;
extern float FJ_Angle;//最后引出的变量//航角
extern volatile float AngleSpeed;
extern float gyro_z;
/*************************************
 *
 *
 *            函数声明
 *
 *
 ************************************/

float calculate_pitch_angle();
void gyroscope_interrupt_handler(void);
void call_gyroscope_thread(void);
void Zero_Point_Detect(void);
void Gyroscope_GetData(void);
void Get_Gyroscope_Angle(void);
void Get_Gyroscope_Pitch(void);
void Clear_Gyroscope_Pitch(void);
void Clear_Gyroscope_Angle(void);
void get_gyro();
 #endif