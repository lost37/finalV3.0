/*
 * camera.h
 *
 *  Created on: 2025年3月18日
 *      Author: xiaozuo
 */

 #ifndef CODE_CAMERA_H_
 #define CODE_CAMERA_H_

 /*************************************
 *             头文件声明
 *************************************/
 #include "zf_common_headfile.h"
 #include "circle.h"
 #include "cross.h"
 #include "zebra.h"
 #include "redblock.h"

 
 /*************************************
 *            变量及常数定义
 *************************************/
 #define ROW     UVC_WIDTH              //设定图像宽度
 #define COL     UVC_HEIGHT             //设定图像高度
 #define Cut_ROW 90                 //裁减行数
 #define Cut_COL 160                      //裁减列数
 
 #define SEARCH_MAX      Cut_COL-2       //边界搜索右边界
 #define SEARCH_MIN      2               //边界搜索左边界
 #define err_limit 80                   //误差限制
 #define ZEBRA_STRIPE_WIDTH_MIN 3        // 斑马线最小条纹宽度
 #define ZEBRA_STRIPE_WIDTH_MAX 10       // 斑马线最大条纹宽度
 #define ZEBRA_STRIPE_COUNT_MIN 3        // 斑马线最小条纹数量

extern uint16 l_duty1;
extern uint16 r_duty1;
extern uint16 l_duty2;
extern uint16 r_duty2;


 extern uint8 Foresight_left,Foresight_right;
 extern float Farthest_distance;

 extern uint8 Straight_track_width[Cut_ROW];    //标准直道宽度
 extern float err_new ;                          //误差

 /*图像*/
 extern volatile uint8 Image_Use[COL][ROW];                     //图像
 extern volatile uint8 Cut_Image_Use[Cut_ROW][Cut_COL];         //裁减图像
 extern volatile uint8 Cut_rgbImage_Use[Cut_ROW][Cut_COL * 3];  //裁减彩色图像
 extern volatile uint8 Canny_Image_Use[COL][ROW];               //Canny图像
 extern volatile uint8 Canny_Cut_Image_Use[Cut_ROW][Cut_COL];   //Canny裁减图像

/*重要参数*/
 extern uint8 Threshold;                            //阈值
 extern int16 white_length_max[2];                  //存储最长白列，[0][0]:行值;[1][0]:列值   
 extern int16 white_length_max_Num;                 //最长白列的列数
 extern uint8 white_length_start;                   //最长白列搜索起始列
 extern uint8 white_length_end;                     //最长白列搜索结束列
 extern int16 r_effect_num;                         //右边界有效点的个数
 extern int16 l_effect_num;                         //右边界有效点的个数
 extern int16 l_start;                              //左边界的起始行数
 extern int16 r_start;                              //右边界的起始行数
 extern uint8 Center_point[Cut_ROW];
 extern uint8 l_border[Cut_ROW], r_border[Cut_ROW]; //左右边界
 extern int16 Center_err[Cut_ROW];                  //赛道误差中线
 extern uint8 width[Cut_ROW];                       //赛道宽度
 extern uint8 land_line;                            //搜索角点的行位置

/*权重*/
 extern uint8 weight[Cut_ROW];                      //图像权重数组（动态）
volatile extern int8 Search_Stop_Line;                      //搜索停止线

/*S弯*/
 extern uint8 s_wan_flag;                    //S弯标志位

/*弯道*/
 extern uint8 Straight_Flag;                //弯道状态位，0为非弯道，1为弯道

/*停车*/
 extern u_char stop;                        //停车标志位，置1表示停车
 extern uint8 redblock_pause_flag;          //红色色块触发的临时停车标志
 extern uint8 model_request_flag;           //请求开启模型标志
 extern uint8 model_running_flag;           //模型运行标志

 extern int8  error_border_flag;            //错误边线标志位
 extern uint8 barrier_flag;                 //斜坡标志位
 extern uint16 po_sum;                      //特定坡道数量的临时变量
 extern uint8 po_num;                   //坡道数

/*障碍*/
 extern uint8 zhang_ai_flag;                //障碍物标志位
 extern uint8 zhang_ai_num;                 //障碍临时变量

 extern uint8 x1_boundary[UVC_HEIGHT], x2_boundary[UVC_HEIGHT], x3_boundary[UVC_HEIGHT];
 /////////////////////////////////////////////////////////////////////////////////////////////////////////

 #define BEEP "/dev/zf_driver_gpio_beep"

 typedef enum                               //用于函数变量，便于判断左右
 {
     left,
     right,
 }Direction;
 
// 斑马线检测状态
typedef enum {
    ZEBRA_STATE_NONE = 0,     // 初始状态
    ZEBRA_STATE_DETECTED_1 = 1, // 首次检测到斑马线(置1)
    ZEBRA_STATE_PASSED_1 = 2,   // 已通过第一条斑马线(置2)
    ZEBRA_STATE_DETECTED_2 = 3  // 第二次检测到斑马线(置3)
} ZebraState;

/*额外共享变量声明（供子模块使用）*/
extern int l_land_num;
extern uint8 xie_cross_time;

 /*************************************
  *              函数声明
  ************************************/
 /*获取图像*/
 void Get_Use_Image(uint8_t* image, volatile uint8_t end_image[][UVC_WIDTH]);
 void Cut_Use_Image(volatile uint8_t image[][Cut_COL], volatile uint8_t end_image[][Cut_COL]);              //裁减图像
 void Get_Use_rgbImage(uint8_t* image, volatile uint8_t end_image[][UVC_WIDTH * 3]);                        //获取未压缩的图像数据
 void Cut_Use_rgbImage(volatile uint8_t image[][Cut_COL * 3], volatile uint8_t end_image[][Cut_COL * 3]);   //裁减彩色图像

 /*图像二值化*/
 float limit(float x, int32 y);                         //限幅函数,限制幅值在(X,Y)之间
 int16 calc_diff_zebra(int16 x, int16 y);               //差比和函数（斑马线用）

 /*搜索最长白列*/
 void search_longest_white_col(void);

 /*辅助函数*/
 void fill_line(uint8 *array_value, uint8 down_row, uint8 down_col, uint8 up_row, uint8 up_col);
 uint8 compare_border_judge(Direction direction, uint8 start, uint8 end, uint8 col, uint8 require_num);

 /*后期处理*/
 float Err_Get(void);
 void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2); //线性回归函数

 /*差速计算*/
 void chasu_calculation();

 /*速度决策*/
 void Furthest_judge(); //距离判断
 void weight_box(); //动态权重

 /*图像处理全过程*/
 float Camera_Function(void);

void search_center(int row,int col);    //计算中线误差
void protect(void);                     //出界保护
uint8 Car_ShouldPause(void);            //主循环是否应暂停
uint8 Car_ShouldMotorStop(void);        //电机是否应停车

 #endif /* CODE_CAMERA_H_ */
 

