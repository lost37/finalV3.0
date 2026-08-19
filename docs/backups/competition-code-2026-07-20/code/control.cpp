/*
 * 该文件作用：总控制
 * control.cpp
 */
/*************************************
 *             头文件声明
 *************************************/
#include "zf_common_headfile.h"
#include "math.h"
#include "redblock.h"
 /*************************************
 *           变量及常数定义
 *************************************/
 /*共用结构体*/
 RoadState current_state = STATE_CURVE;

 /*电机*/
int time1=0,timestop=0;      //出界停止时间

int32_t l_speed = 0, r_speed = 0;  //速度赋值临时量
volatile int32_t set_speed =320;//设置的电机速度
volatile int land_s = 460; //环岛速度
volatile int po_s = 600;   //坡道速度
volatile int wan_s = 500;   // 弯道速度
volatile int ru_s = 500;    // 入弯速度
volatile int zhi_s = 600;   // 直道速度

//int Speed_dif; //差速


/*速度表*/
int8_t speed_mode = 0; //速度模式: 0慢速 1正常 2快速 3非常快
// SpeedConfig speed_list = { //速度结构体
//     .speed0 = 0, //零速
//     .speed1 = 640,
//     .speed2 = 660,
//     .speed3 = 680,
//     .speed4 = 700,
//     .speed5 = 720,
//     .speed6 = 740,
//     .speed7 = 760,
//     .speed8 = 780,
// };    

int go_delay_counter =0;

void dianji_control() //电机控制 这个放在中断里
{
    /* 这里不再直接读取编码器。
     * 编码器反馈由调度层先统一刷新，再由控制层消费。
     */
    Speed_control();
}

void Speed_control()
{
    if(Car_ShouldMotorStop())  //停车
    {
        time1++;
        timestop=0;
        if(time1>=65)
        {
            pwm0_flag=1;
        }
    }
    else
    {
        timestop++;
        time1=0;
        chasu_calculation(); //单减差速控制
    }
    if(pwm0_flag==1)
    {
        if(RedBlock_IsSlowdownActive())
        {
            pwm0_flag = 0;
            Motor_Control(l_out,r_out);
            return;
        }
        else
        {
            l_out=l_pid(0, enconder_left);
            r_out=r_pid(0, enconder_right);
        }
        Motor_Control(l_out,r_out);
    }
    // else   //差速控制
    // {
    //     l_out = l_pid(l_speed, actual_speed);
    //     r_out = r_pid(r_speed, actual_speed);
    // }
    //Motor_Control(l_out,r_out);
    //chasu_calculation(); //单减差速控制
}
//----------------------------------------------------------------------------------------------------------------
//速度控制决策相关
//----------------------------------------------------------------------------------------------------------------

float residual(uint8 n, uint8_t *border)  //计算中线平均残差平方和
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    float a=0,b=0;

    for (i = 0; i < n; i++)
    {
        xsum += i;
        ysum += border[i];
        xysum += i * (border[i]);
        x2sum += i * i;
    }

    a = (n * xysum - xsum * ysum) / (n * x2sum - xsum * xsum);
    b = (ysum - a * xsum) / n;

    // 计算残差平方和
    float sum_residual = 0.0f;
    for (int i = 0; i < n; i++)
    {
        float x_pred = a *i + b;
        float residual = border[i] - x_pred;
        sum_residual += residual*residual; // 残差平方和
    }

    sum_residual/=n;

    return sum_residual;
}

float mid_k;

void Speed_decision() {
    float err = func_abs(err_new);
    int speed_chose;

    if(RedBlock_IsSlowdownActive())
    {
        set_speed = RedBlock_GetSlowdownSpeedCmd();
        return;
    }

    if(RedBlock_IsModelPending())
    {
        return;
    }

    if(RedBlock_IsBypassActive())
    {
        set_speed = RedBlock_GetBypassSpeedCmd();
        return;
    }

    // 距离与误差门槛参数
    const int straight_upper = 140;
    const int decel_zone     = 130;
    const int straight_lower = 80;
    const int gentle_l1_dist = 75;
    const int gentle_l2_dist = 80;

    // 状态与滤波计数器
    // static RoadState current_state = STATE_CURVE;
    static int straight_cnt = 0;
    const int straight_threshold = 2;
    static bool in_round = false;
    static bool in_slope = false;

    // === 状态判断 ===
    if (l_land_flag != 0 || r_land_flag != 0) {
        // 处于环岛，优先处理
        current_state = STATE_ROUND;
        in_round = true;
        in_slope = false;  // 保证状态互斥
    } else if (in_round && l_land_flag == 0 && r_land_flag == 0) {
        // 离开环岛
        current_state = STATE_CURVE;
        in_round = false;
        straight_cnt = 0;
    } else if (barrier_flag != 0) {
        // 不在环岛，才考虑坡道
        current_state = STATE_SLOPE;
        in_slope = true;
    } else if (in_slope && barrier_flag == 0) {
        // 离开坡道
        current_state = STATE_CURVE;
        in_slope = false;
        straight_cnt = 0;
    } else {
        switch (current_state) {
            case STATE_CURVE:
                if (err < 3 && Farthest_distance > decel_zone) {
                    current_state = STATE_DECEL;
                    straight_cnt = 0;
                } else if (err >= 7.0 && err < 9.8 && Farthest_distance >= gentle_l2_dist) {
                    current_state = STATE_GENTLE_CURVE_L2;
                } else if (err >= 5.0 && err < 7.0 && Farthest_distance >= gentle_l1_dist) {
                    current_state = STATE_GENTLE_CURVE_L1;
                }
                break;

            case STATE_GENTLE_CURVE_L2:
                if (err < 3 && Farthest_distance >= decel_zone) {
                    current_state = STATE_DECEL;
                    straight_cnt = 0;
                } else if (err >= 5.0 && err < 7.0 && Farthest_distance >= gentle_l1_dist) {
                    current_state = STATE_GENTLE_CURVE_L1;
                } else if (err >= 9.8 || Farthest_distance < gentle_l1_dist) {
                    current_state = STATE_CURVE;
                }
                break;

            case STATE_GENTLE_CURVE_L1:
                if (err < 3 && Farthest_distance >= straight_upper) {
                    straight_cnt++;
                    if (straight_cnt >= straight_threshold) {
                        current_state = STATE_STRAIGHT;
                    }
                } else {
                    straight_cnt = 0;
                    if (err >= 7.0 && err < 9.8 && Farthest_distance >= gentle_l2_dist) {
                        current_state = STATE_GENTLE_CURVE_L2;
                    } else if (err >= 9.8 || Farthest_distance < gentle_l1_dist) {
                        current_state = STATE_CURVE;
                    }
                }
                break;

            case STATE_DECEL:
                if (err < 3 && Farthest_distance >= straight_upper) {
                    straight_cnt++;
                    if (straight_cnt >= straight_threshold) {
                        current_state = STATE_STRAIGHT;
                    }
                } else {
                    straight_cnt = 0;
                    if (err >= 9.8 || Farthest_distance < straight_lower) {
                        current_state = STATE_CURVE;
                    } else if (err >= 5.0 && err < 7.0 && Farthest_distance >= gentle_l1_dist) {
                        current_state = STATE_GENTLE_CURVE_L1;
                    } else if (err >= 7.0 && err < 9.8 && Farthest_distance >= gentle_l2_dist) {
                        current_state = STATE_GENTLE_CURVE_L2;
                    }
                }
                break;

            case STATE_STRAIGHT:
                if (err > 6 || Farthest_distance < decel_zone) {
                    current_state = STATE_DECEL;
                    straight_cnt = 0;
                }
                break;

            case STATE_ROUND:
                // do nothing; 保持该状态直到 l_land_flag 清零
                break;

            case STATE_SLOPE:
                // do nothing; 保持该状态直到 barrier_flag 清零
                break;
        }
    }

    // === 状态对应速度设定 ===
    switch (current_state) {
        case STATE_CURVE:            speed_chose = wan_s;        break;
        case STATE_GENTLE_CURVE_L2:  speed_chose = wan_s + 30;  break;
        case STATE_GENTLE_CURVE_L1:  speed_chose = wan_s + 40;  break;
        case STATE_DECEL:            speed_chose = ru_s;         break;
        case STATE_STRAIGHT:         speed_chose = zhi_s;        break;
        case STATE_ROUND:            speed_chose = land_s;       break;
        case STATE_SLOPE:            speed_chose = po_s;         break;
        default:                     speed_chose = wan_s;        break;
    }

    // 一阶低通滤波：平滑状态切换时的速度跳变
    // alpha 越小过渡越慢越平滑，越大越接近原来的硬切换，先试 0.15
    static float set_speed_f = 500.0f;
    const float alpha = 0.4f;//第一次尝试0.2 可以往0.15修改
    set_speed_f = alpha * (float)speed_chose + (1.0f - alpha) * set_speed_f;
    set_speed = (int32_t)set_speed_f;

    // 可选调试输出
    // printf("State: %d, Err: %.1f, Dist: %.1f, Speed: %d, l_land_flag: %d\n", current_state, err, Farthest_distance, speed_chose, l_land_flag);
}



void Gear_Box()  // 变速箱  速度决策函数  最长白列 加 中线平均残差平方和
{
    int speed_chose;

    mid_k = residual(Cut_ROW,Center_point); //计算中线平均残差平方和

    if(mid_k < 170 && white_length_max[0] < 15) //长直道（最大速度）
    {
        speed_chose =620 ;
    }
    if(mid_k > 170 && white_length_max[0] < 15)  //入弯速度（中速）
    {
        speed_chose = 550;
    }
    if(mid_k > 170 && white_length_max[0] > 15)  //弯道速度（最慢速度）
    {
        speed_chose = 500;
    }

    set_speed = speed_chose;
}



void go_init() //发车初始化
{
    l_speed = 0; //左轮速度清零
    r_speed = 0; //右轮速度清零
    gyroscope_reset_runtime();
    time1 = 0;
    timestop = 0;
    pwm0_flag = 0;
    RedBlock_ResetState();
    l_land_flag = 0; //左环岛标志清零
    r_land_flag = 0; //右环岛标志清零
    cross_flag = 0; //十字路口标志清零
    s_wan_flag = 0; //出环岛T字标志清零
    zhang_ai_flag = 0; //障碍物标志清零
    //camera_flag=0;
    go_flag = 0; // 发车标识默认关闭，启动程序后必须按 KEY0 才允许发车。
    //ips200_full(RGB565_BLACK);
}
