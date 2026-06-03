#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_
#include "zf_common_headfile.h"
 
 /*************************************
 *
 *
 *           变量及常数定义
 *
 *
 *************************************/



 /*电机*/


#define L 160 //车身的前后轮中心距     毫米
#define B 155 //两后轮中心距         毫米
#define STATE_CHANGE_COOL_DOWN 20 // 状态切换冷却时间（单位：毫秒）
#define NEGATIVE_PRESSURE_DELAY 150

extern int time1,timestop;      //出界停止时间

extern int32_t l_speed, r_speed;  //速度赋值临时量
extern volatile int32_t set_speed; //设置的电机速度

extern int8_t speed_mode; //速度模式


//速度决策状态机
typedef enum {
    STATE_CURVE,        //急弯/正常弯道/慢速
    STATE_GENTLE_CURVE_L2, //普通缓弯
    STATE_GENTLE_CURVE_L1, //稍缓的弯
    STATE_DECEL,        //入弯预备
    STATE_STRAIGHT,     //直道
    STATE_ROUND,        //环岛
    STATE_SLOPE,        //坡道
} RoadState;
extern RoadState current_state;
extern int go_delay_counter;


extern float mid_k;
  /*************************************
  *
  *
  *            函数声明
  *
  *
  ************************************/


 /* 电机控制调度函数
  * 只负责消费已经更新好的反馈值，不再直接读取编码器。
  */
 void dianji_control();

 /* 电机速度控制执行函数
  * 依赖编码器反馈变量已经在外层调度中先被刷新。
  */
 void Speed_control();
 void Gear_Box();
 void Speed_decision();
 void go_init(); //发车初始化
 float residual(uint8 n, int16 *border); //计算中线平均残差平方和
  #endif
