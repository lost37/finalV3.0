#include "zf_common_headfile.h"
#include "math.h"
#include "circle.h"
#include "cross.h"
#include "zebra.h"
#include "redblock.h"
#include "ncnn_infer.h"

 /*************************************
 *           变量及常数定义
 *************************************/
#define THRESHOLD 20                    //这个是阈值比较
#define JUMP_NUM  1                     //对比对计算时，两点间隔距离

    uint16 l_duty1 = 1500;
    uint16 r_duty1 =1400;
    uint16 l_duty2 =0;
    uint16 r_duty2 =0;



//模式切换
volatile int BIG_LAND = 0; //环岛模式 0小环岛 1大环岛
//斑马线模式已移至 zebra.cpp


//图像
volatile uint8 Image_Use[COL][ROW];                     //图像
volatile uint8 Cut_Image_Use[Cut_ROW][Cut_COL];         //裁减图像
//volatile uint8 Cut_rgbImage_Use[Cut_ROW][Cut_COL * 3];  //裁减彩色图像
volatile uint8 Canny_Image_Use[COL][ROW];               //Canny图像
volatile uint8 Canny_Cut_Image_Use[Cut_ROW][Cut_COL];   //Canny裁减图像

 
//重要参数
uint8 Threshold = 125;                  //阈值
int16 white_length_max[2];              //[0]:行值;[1]:列值   最长白列
int16 white_length_max_Num=0;           //最长白列的列数
int16 first_end;                        //扫线结束行，最长白列行值加上一个偏移量计算得到的数
int16 l_start;                          //左边界的起始行数
int16 r_start;                          //右边界的起始行数
int16 l_effect_num;                     //左边界有效点的数量
int16 r_effect_num;                     //右边界有效点的数量
int16 search_l_start[Cut_ROW], search_l_end[Cut_ROW];   //临时变量,用于划分一段区域,在该区域中寻找左边界
int16 search_r_start[Cut_ROW], search_r_end[Cut_ROW];   //临时变量,用于划分一段区域,在该区域中寻找右边界
uint8 l_border[Cut_ROW], r_border[Cut_ROW];             //左右边界
int8  l_effect_flag[Cut_ROW], r_effect_flag[Cut_ROW];   //左右边界有效标志位
uint8 Center_point[Cut_ROW];
int16 Center_err[Cut_ROW];              //中线误差
uint8 white_length_start;               //最长白列搜索起始列
uint8 white_length_end;                 //最长白列搜索结束列
double b;                               //截距
double k;                               //斜率
double c_x1;                                    //截距
double c_y1;                                    //斜率
double c_x2;                                    //截距
double c_y2;                                    //斜率
extern volatile float dif_speed;

//发车
//uint8 go_flag=0;

volatile int w = 47;//前瞻

//弯道
uint8 Straight_Flag=0;                  //弯道状态位

//S弯
uint8 s_wan_flag = 0;                    //S弯标志位
int16 s_Center_point[Cut_ROW];

//坡道
int8  error_border_flag;                //错误边线标志位
uint8 barrier_flag = 0;                 //斜坡标志位
uint16 po_sum = 0;                      //特定坡道数量的临时变量
uint8 po_num = 0;                       //坡道数

//障碍
uint8 zhang_ai_flag = 0;                //障碍物标志位
uint8 zhang_ai_num = 0;                 //障碍临时变量

//距离判断
uint8 distance_flag=0;                  //实际距离标志位
float Farthest_distance = 5.0;
uint8 Foresight = ROW - 1;
uint8 Foresight_left=0;
uint8 Foresight_right=0;

//停车
u_char stop=0;                          //停车标志位，置1表示停车
uint8 redblock_pause_flag = 0;
uint8 model_request_flag = 0;
uint8 model_running_flag = 0;
float SUM = 0;                          //临时变量，用于出界计时

//斑马线变量已移至 zebra.cpp

uint8 x1_boundary[UVC_HEIGHT], x2_boundary[UVC_HEIGHT], x3_boundary[UVC_HEIGHT];

float err_new = 0;
volatile int8 Search_Stop_Line =45;                       //权重线，超过这个行数的权重才会被计算  35
const uint8 weight_key[10]=
{
   2, 4, 6, 10, 11, 11, 10, 6, 4, 2
};

namespace
{
    constexpr int MODEL_CLASS_SUPPLIERS = 0;
    constexpr int MODEL_CLASS_VEHICLE = 1;
    constexpr int MODEL_CLASS_WEAPON = 2;
    constexpr uint8 MODEL_CLASS_COUNT = 3;
    constexpr uint8 MODEL_STABLE_REQUIRED = 2;//连续几帧满足稳定才开始模型流程。越大越稳；越小响应越快
    constexpr uint8 MODEL_DROP_VALID_REQUIRED = 1;//模型刚开始后丢弃几帧有效结果。越大越能避开刚停车时画面抖动；越小越快。
    constexpr uint8 MODEL_VOTE_REQUIRED = 5; //投票次数
    constexpr uint16 MODEL_WAIT_STABLE_TIMEOUT = 180;
    constexpr uint16 MODEL_INFER_TIMEOUT = 120;

    typedef enum
    {
        MODEL_ACTION_NONE = 0,
        MODEL_ACTION_STRAIGHT,
        MODEL_ACTION_LEFT_BYPASS,
        MODEL_ACTION_RIGHT_BYPASS,
    } ModelAction;

    typedef enum
    {
        MODEL_STAGE_IDLE = 0,
        MODEL_STAGE_WAIT_STABLE,
        MODEL_STAGE_DROP_VALID_FRAMES,
        MODEL_STAGE_COLLECT_VOTES,
    } ModelConfirmStage;

    ModelConfirmStage g_model_stage = MODEL_STAGE_IDLE;
    uint8 g_model_stable_count = 0;
    uint8 g_model_drop_valid_count = 0;
    uint8 g_model_vote_valid_count = 0;
    uint8 g_model_vote_count[MODEL_CLASS_COUNT] = {0};
    uint16 g_model_stage_retry_count = 0;

    void Model_Confirm_Reset(void)
    {
        uint8 i = 0;

        g_model_stage = MODEL_STAGE_IDLE;
        g_model_stable_count = 0;
        g_model_drop_valid_count = 0;
        g_model_vote_valid_count = 0;
        g_model_stage_retry_count = 0;
        for(i = 0; i < MODEL_CLASS_COUNT; i++)
        {
            g_model_vote_count[i] = 0;
        }
    }

    void Model_Request_ClearFlags(void)
    {
        model_request_flag = 0;
        model_running_flag = 0;
    }

    void Model_SetStage(ModelConfirmStage stage)
    {
        g_model_stage = stage;
        g_model_stage_retry_count = 0;
    }

    uint8 Model_IsStable(void)
    {
        return (func_abs(enconder_left) < 60 || func_abs(enconder_right) < 60);
    }

    const char *Model_ClassLabel(int coarse_index)
    {
        switch(coarse_index)
        {
            case MODEL_CLASS_SUPPLIERS:
                return "suppliers";

            case MODEL_CLASS_VEHICLE:
                return "vehicle";

            case MODEL_CLASS_WEAPON:
                return "weapon";

            default:
                return "unknown";
        }
    }

    void Model_Vote_Add(int coarse_index)
    {
        if(coarse_index >= 0 && coarse_index < MODEL_CLASS_COUNT)
        {
            g_model_vote_count[coarse_index]++;
            g_model_vote_valid_count++;
        }
    }

    int Model_Vote_GetBestClass(void)
    {
        int best_class_index = -1;
        uint8 best_vote_count = 0;
        uint8 tie_flag = 0;
        uint8 i = 0;

        for(i = 0; i < MODEL_CLASS_COUNT; i++)
        {
            if(g_model_vote_count[i] > best_vote_count)
            {
                best_vote_count = g_model_vote_count[i];
                best_class_index = i;
                tie_flag = 0;
            }
            else if(g_model_vote_count[i] == best_vote_count && g_model_vote_count[i] > 0)
            {
                tie_flag = 1;
            }
        }

        if(best_vote_count == 0 || tie_flag != 0)
        {
            return -1;
        }

        return best_class_index;
    }

    ModelAction Model_DecideAction(const NCNN_Infer_Result &infer_result)
    {
        switch(infer_result.coarse_index)
        {
            case MODEL_CLASS_VEHICLE:
                return MODEL_ACTION_STRAIGHT;

            case MODEL_CLASS_SUPPLIERS:
                return MODEL_ACTION_RIGHT_BYPASS;

            case MODEL_CLASS_WEAPON:
                return MODEL_ACTION_LEFT_BYPASS;

            default:
                return MODEL_ACTION_NONE;
        }
    }

    void Model_ExecuteAction(ModelAction action, const NCNN_Infer_Result &infer_result)
    {
        Model_Request_ClearFlags();
        RedBlock_OnModelConfirmed();

        switch(action)
        {
            case MODEL_ACTION_STRAIGHT:
                printf("Model action: vehicle -> straight pass\n");
                RedBlock_StartBypassMode(RB_BYPASS_MODE_STRAIGHT);
                break;

            case MODEL_ACTION_LEFT_BYPASS:
                printf("Model action: weapon -> left bypass\n");
                RedBlock_StartBypassMode(RB_BYPASS_MODE_LEFT);
                break;

            case MODEL_ACTION_RIGHT_BYPASS:
                printf("Model action: suppliers -> right bypass\n");
                RedBlock_StartBypassMode(RB_BYPASS_MODE_RIGHT);
                break;

            default:
                printf(
                    "Model action: unknown fine=%d coarse=%d label=%s, keep pause\n",
                    infer_result.class_index,
                    infer_result.coarse_index,
                    infer_result.label.c_str()
                );
                break;
        }
    }

    void Model_ApplyConfirmedAction(const NCNN_Infer_Result &infer_result)
    {
        const ModelAction action = Model_DecideAction(infer_result);
        Model_ExecuteAction(action, infer_result);
    }
}


//标准赛道宽度
uint8 Straight_track_width[Cut_ROW] =
{
    15, 16, 17, 19, 20, 21, 22, 24, 25, 26,
    27, 28, 30, 31, 32, 33, 35, 36, 37, 38,
    39, 41, 42, 43, 44, 46, 47, 48, 49, 51,
    52, 53, 54, 55, 57, 58, 59, 60, 62, 63,
    64, 65, 66, 68, 69, 70, 71, 73, 74, 75,
    76, 77, 79, 80, 81, 82, 84, 85, 86, 87,
    88, 90, 91, 92, 93, 95, 96, 97, 98, 100,
    101, 102, 103, 104, 106, 107, 108, 109, 111, 112,
    113, 114, 115, 116, 118, 119, 120, 121, 123, 124
};
//10-35行平均宽度
float po_width = 46.6;

float distance[70]=       
{
   237, 235, 233, 231, 226, 218, 205, 194, 185, 176,       //0-9     240 230
   176, 153, 132, 115,  97,  90,  83,  75,  68,  62,             //10-19   97  176
    62,  60,  58,  55,  52,  49,  46,  43,  40,  37,                 //20-29   62  46
    37,  36,  35,  35,  33,  31,  30,  28,  27,  26,                 //30-39   37  30
    25,  24,  23,  22,  21,  20,  19,  18,  18,  17,                 //40-49   25  21
    17,  16,  16,  15,  15,  14,  14,  13,  13,  12,                 //50-59   18  15
    12,  10,  10,   9,  10,   9,   9,   8,   8,  8                        //60-69   12  10  8
};

//左轮对应列数
uint8 left_edge[70]=
{ //0   1   2   3   4   5   6   7   8   9
    78, 77, 77, 77, 77, 77, 76, 76, 76, 75,                 //0-9
    75, 75, 74, 74, 73, 73, 72, 71, 71, 71,                 //10-19
    70, 70, 69, 69, 68, 68, 67, 67, 66, 66,                 //20-29
    66, 65, 65, 64, 64, 63, 62, 62, 61, 61,                 //30-39
    60, 60, 59, 59, 58, 58, 57, 57, 56, 56,                 //40 -49
    55, 55, 54, 54, 53, 53, 52, 52, 51, 50,                 //50-59
    50, 49, 49, 48, 48, 48, 47, 47, 46, 46                  //60-69
};
//右轮对应列数
uint8 right_edge[70]=
{    // 0    1   2   3   4   5   6   7   8   9
    80, 80, 81, 81, 82, 82, 83, 83, 84, 84,                 //0-9
    85, 85, 86, 86, 87, 87, 88, 88, 88, 89,                 //10-19
    89, 89, 90, 90, 91, 91, 92, 92, 93, 93,                 //20-29
    94, 94, 95, 95, 96, 96, 87, 87, 98, 98,                //30-39
    99, 99, 100, 100, 101, 101, 102, 102, 103, 103,       //40 -49
    104, 104, 105, 105, 106, 106, 107, 107, 108, 108,       //50-59
    108, 109, 109, 110, 110, 111, 111, 112, 112, 113        //60-69
};


//当前帧捕获赛道每个高度的宽度
uint8 width[Cut_ROW]=
{
    0
};



void Get_Use_Image(uint8_t* image, volatile uint8_t end_image[][UVC_WIDTH])
{
  int i = 0,j = 0,row = 0,line = 0;
  int sum = 0;
  for(i = 0; i  < Cut_ROW; i++)  //80，
  {
    for(j = 0;j < Cut_COL; j++)  //160，
    {
        sum = row * Cut_COL + line;
        end_image[i][j] = image[sum];
        line++;
    }
    line = 0;
    row++;
  }
}



//----------------------------------------------------------------------------------------------------------------
// 函数名称     void Cut_Use_Image()
// 函数简介     裁剪图像
// 参数说明
// 返回参数
// 使用示例
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void Cut_Use_Image(volatile uint8_t image[][ROW], volatile uint8_t end_image[][Cut_COL]) {
    int i, j;
    for(i = 0; i < Cut_ROW; i++) {
        for(j = 0; j < Cut_COL; j++) {
            end_image[i][j] = image[i][j]; // 裁剪起始行偏移30
        }
    }
}

void fill_line(uint8 *array_value, uint8 down_row, uint8 down_col, uint8 up_row, uint8 up_col)
{
   down_row = func_limit_ab(down_row, 0, Cut_ROW);
   up_row = func_limit_ab(up_row, 0, Cut_ROW);
   down_col = func_limit_ab(down_col, 0, Cut_COL);
   up_col = func_limit_ab(up_col, 0, Cut_COL);
   float point_1=(float)down_col;
   float point_2=(float)up_col;
   float temp_slope=(point_2-point_1)/(down_row- up_row);

   for(int i=0;i<(down_row -up_row);i++)
   {
       array_value[down_row-i]=(uint8)((int8)(temp_slope *i)+ down_col);//连接线就是设置边界
   }
}


//----------------------------------------------------------------------------------------------------------------
// 函数名称 int16 calc_diff_zebra(int16 x, int16 y)
// 函数简介 差比和函数
// 参数说明
// 返回参数
// 使用示例
// 备注信息
//----------------------------------------------------------------------------------------------------------------
int16 calc_diff_zebra(int16 x, int16 y)       //差比和算法,会在search_white中用到
{
   if (x + y == 0) {
       return 0; // 处理分母为零的情况
   }
   return ( ((x-y)<<7)/(x+y) );
}


//----------------------------------------------------------------------------------------------------------------
// 函数名称 void regression1(int startline, int endline)
// 函数简介 最小二乘法1号
// 参数说明
// 返回参数
// 使用示例
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void regression1(int startline, int endline)
{
    int i = 0, SumX = 0, SumY = 0, SumLines = 0;
    float SumUp = 0, SumDown = 0, avrX = 0, avrY = 0;
    SumLines = endline - startline;   // startline 为开始行， //endline 为结束行 //SumLines 为y轴误差

    for (i = startline; i < endline; i++)
    {
        SumX += i;
        SumY += Center_point[i];    //这里Middle_black为存放中线的数组
    }
    avrX = (float)SumX / SumLines;  //X的平均值
    avrY = (float)SumY / SumLines;  //Y的平均值     
    SumUp = 0;
    SumDown = 0;
    for (i = startline; i < endline; i++)
    {
        SumUp += (Center_point[i] - avrY) * (i - avrX);
        SumDown += (i - avrX) * (i - avrX);
    }
    if (SumDown == 0)
        k = 0;
    else
        k = SumUp / SumDown;     //斜率
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称     void search_longest_white_col(void)
// 函数简介     搜索最长白列函数
// 参数说明
// 返回参数
// 使用示例
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void search_longest_white_col()     //搜索最长白列函数
{
    white_length_start = 159;
    white_length_end = 0;
    int16 i, j;
    uint8 div_x = 2;                    //每隔div_x列搜索一次             
    //int16 temp_value;(当前未使用，可能在环岛寻线时使用)
    white_length_max[0] = Cut_ROW - 1;     //最长白列行数初始化，119代表最低，行数越低代表着白列约长
    //按列搜线，每一列向上搜


    for (int i = 1; i < Cut_COL - 1; i++)
    {
        if (Canny_Cut_Image_Use[Cut_ROW - 1][i] == 255 && i < Cut_COL / 4 * 3 && Cut_Image_Use[Cut_ROW - 1][i - 1] == 0 && white_length_start == 159)
            white_length_start = i;
        if (Canny_Cut_Image_Use[Cut_ROW - 1][i] == 255 && i >= Cut_COL / 4 && Cut_Image_Use[Cut_ROW - 1][i + 1] == 0 && white_length_end == 0)
            white_length_end = i;
        if (white_length_start > white_length_end)
        {
            if (Cut_Image_Use[Cut_ROW - 2][(white_length_start + Cut_COL - 1) / 2] == 0)
                white_length_start = 159;
            else
                white_length_end = 0;
        }
    }
    if (white_length_start == 159)
        for (int i = 0; i < Cut_ROW - 1; i++)
            Canny_Cut_Image_Use[i][0] = 255;
    else
        for (int i = 0; i <= white_length_start; i++)
            Canny_Cut_Image_Use[Cut_ROW - 1][i] = 255;
    if (white_length_end == 0)
        for (int i = 0; i < Cut_ROW - 1; i++)
            Canny_Cut_Image_Use[i][Cut_COL - 1] = 255;
    else
        for (int i = Cut_COL - 1; i >= white_length_end; i--)
            Canny_Cut_Image_Use[Cut_ROW - 1][i] = 255;
    if (white_length_start == 159 && white_length_end == 0 && Cut_Image_Use[Cut_ROW - 1][Cut_COL / 2] == 0)
    {
        for (int i = 0; i < Cut_COL; i++)
            Canny_Cut_Image_Use[Cut_ROW - 1][i] = 255;
        if(RedBlock_ShouldIgnoreBoundaryStop() == 0)
        {
            stop = 1;
            l_land_flag = 0;
            r_land_flag = 0;
            barrier_flag = 0;
        }
    }
    //右环岛特殊处理
    if (r_land_flag == 3 || r_land_flag == 6)
    {
        for (i = 25; i < ((Cut_COL - 5) / div_x - 1); i++)
        {
            if (Canny_Cut_Image_Use[Cut_ROW - 1][i * div_x] == 255)
                continue;
            for (j = Cut_ROW - 1; j > 0; j--)     //从下往上找
            {
                if (Canny_Cut_Image_Use[j][i * div_x] > Threshold)
                    break;
            }
            if (j < white_length_max[0] && (func_abs(i * div_x - white_length_max[1]) < (Cut_COL / 3)))
                //white_length[0][i] < white_length_max[0]=1代表行数更小，func_abs返回值<60代表该最长白列与上一列间隔小于一定列，满足两个条件则进入if
            {
                white_length_max[0] = j;//更新最长白列行值
                white_length_max[1] = i * div_x;//更新最长白列列值
                white_length_max_Num = i;                  //更新最长白列列值white_length_max_Num
            }
        }
    }
    //正常情况下的最长白列搜索
    else
    {
        for (i = 5; i < ((Cut_COL - 5) / div_x - 1); i++)
        {
            if (Canny_Cut_Image_Use[Cut_ROW - 1][i * div_x] == 255)
                continue;
            for (j = Cut_ROW - 1; j > 0; j--)     //从下往上找
            {
                if (Canny_Cut_Image_Use[j][i * div_x] > Threshold)
                    break;
            }
            if (j < white_length_max[0] && (func_abs((i * div_x) - white_length_max[1]) < (Cut_COL / 3)))
                //white_length[0][i] < white_length_max[0]=1代表行数更小，func_abs返回值<60代表该最长白列与上一列间隔小于一定列，满足两个条件则进入if
            {
                white_length_max[0] = j;//更新最长白列行值
                white_length_max[1] = i * div_x;//更新最长白列列值
                white_length_max_Num = i;                  //更新最长白列列值white_length_max_Num
            }
        }
    }
}



//----------------------------------------------------------------------------------------------------------------
// 函数名称     void search_border(void)
// 函数简介     搜索赛道边界函数(裁减后)
// 参数说明
// 返回参数
// 使用示例     search_border()
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void search_border(int row,int col)
{

    int16 i, j;
    int16 temp_value, temp_value1;
    first_end = white_length_max[0];   //最长白列行值     扫线截至行
    l_start = 0;                            //左右边界的起始行数
    r_start = 0;
    l_effect_num = 0;                       //左右边界有效点的数量
    r_effect_num = 0;

    if(first_end > col - 1)   
        first_end = col - 1;//防止超范围
    for(i = 0;i < first_end;i++)
    {
        l_border[i] = 0;
        r_border[i] = row - 1;
        if(i == 89) {
            printf("Row 89 init: l_border[89]=%d, r_border[89]=%d\n", l_border[89], r_border[89]);
        }
        
        
    }
    for(i=col - 1; i >= first_end; i--)
    {
        search_l_start[i] = white_length_max[1];//左右边界从最长列开始     从最长列开始寻找边界，左边向左右边向右
        search_r_start[i] = white_length_max[1];
        search_l_end[i] = SEARCH_MIN;//左边界结束为图像左边界第一列
        search_r_end[i] = SEARCH_MAX;//右边界结束为图像右边界最后一列
        l_border[i] = search_l_end[i];
        r_border[i] = search_r_end[i];

        //搜索左边
        l_effect_flag[i] = 0;        //清除边界有效标志位
        for(j=search_l_start[i]; j>=search_l_end[i]; j--)//左边界从右往左扫描，从最长白列处开始扫描
        {
            l_border[i] = (uint8)j;//赋予左边界列的值
            if (Threshold > Canny_Cut_Image_Use[i][j]) //当前点灰度值直接和当前白点值比较，大于白点值直接判定为白点
                continue;
            else
            {
                l_effect_flag[i] = 1;   //左边寻到黑白跳变标志位
                if(!l_start) 
                    l_start = i;   //若为第一次寻到黑白跳变，将改行设置为寻到左边界的起始行数
                l_effect_num++;         //左边界有效点的数量
                break;
            }
        }
        //搜索右边
        r_effect_flag[i] = 0;        //清除边界有效标志位
        for(j=search_r_start[i]; j<=search_r_end[i]; j++)//右边界从左往右扫描
        {
            r_border[i] = (uint8)j;//赋予右边界列的值
            if (Threshold > Canny_Cut_Image_Use[i][j]) //当前点灰度值直接和当前白点值比较，大于白点值直接判定为白点
                continue;
            else
            {
                r_effect_flag[i] = 1;
                if(!r_start) 
                    r_start = i;
                r_effect_num++;
                break;
            }
        }
        //左右边界限幅，并将边界保存在l_border和r_border数组中
        l_border[i] = func_limit_ab(l_border[i], SEARCH_MIN, SEARCH_MAX);
        r_border[i] = func_limit_ab(r_border[i], SEARCH_MIN, SEARCH_MAX);
    }
    l_effect_num += white_length_max[0];
    r_effect_num += white_length_max[0];
    // 检查最顶端行的偏差，决定是否强制填充边界
    temp_value = first_end + 3;  // 从 first_end 向上取 3 行
    if (temp_value > col - 1) {
        temp_value = col - 1;    // 防止越界
    }

    if (l_land_flag == 0 && r_land_flag == 0)
    {
        // 1. 先判断是否为单边丢失（针对欧米伽弯的关键逻辑）
        // 如果左边有效点多，右边有效点极少（右边丢线）
        if (l_effect_num > 40 && r_effect_num < 15) 
        {
            // 使用左边线 + 赛道宽度 = 中线
            for (int i = 0; i < col - 1; i++)
            {
                // 注意：这里使用你代码里定义的 Straight_track_width 标准宽度数组
                // 如果左边界存在，中线 = 左边界 + 宽度的一半
                if(l_border[i] > 1) 
                    Center_point[i] = l_border[i] + (Straight_track_width[i] / 2);
                else 
                    Center_point[i] = col / 2; // 兜底
                
                // 限制幅值
                if (Center_point[i] > col - 1) Center_point[i] = col - 1;
            }
        }
        // 如果右边有效点多，左边有效点极少（左边丢线）
        else if (r_effect_num > 40 && l_effect_num < 15) 
        {
            for (int i = 0; i < col - 1; i++)
            {
                if(r_border[i] < col - 2)
                    Center_point[i] = r_border[i] - (Straight_track_width[i] / 2);
                else
                    Center_point[i] = col / 2;
                
                if (Center_point[i] < 0) Center_point[i] = 0;
            }
        }
        // 2. 如果两边都比较正常，才考虑是否使用直线拟合
        else 
        {
            regression1(white_length_max[0] + 2, Cut_ROW - 2);
            
            // 只有当斜率非常小（近似直道）时，才允许强行拉直
            // 欧米伽弯时 k 值通常较大，加上 abs(k) < 0.3 的限制可以防止弯道被拉直
            if (k != 0 && func_abs(k) < 0.3 && xie_cross_flag == 0) 
            {
                b = (r_border[white_length_max[0] + 3] + l_border[white_length_max[0] + 3]) / 2 - k * (white_length_max[0] + 3);
                for (int i = white_length_max[0]; i >= 0; i--)
                {
                    Center_point[i] = k * i + b;
                    if (Center_point[i] < 0) Center_point[i] = 0;
                    if (Center_point[i] > Cut_COL - 1) Center_point[i] = Cut_COL - 1;
                }
                // 这里不需要再 fill_line 修改 border 了，直接算出了 Center_point
                // 你的原代码这里去修改 l_border/r_border 反而会破坏后续逻辑
            }
            else
            {
                 
            }
        }
        
    }
    else
    {
        temp_value1 = (l_border[temp_value] + r_border[temp_value]) / 2 - row / 2;

        // 根据偏差决定填充策略
        if (temp_value1 < -(row / 10)) {  // 严重偏左
            fill_line(l_border, temp_value, l_border[temp_value], 0, 1);           // 左边界填充到最左侧
            fill_line(r_border, temp_value, r_border[temp_value], 0, 2); // 右边界填充到中线左侧
            Straight_Flag = 0;  // 标记为弯道
        }
        else if (temp_value1 > (row / 10)) {  // 严重偏右
            fill_line(l_border, temp_value, l_border[temp_value], 0, row - 2); // 左边界填充到中线右侧
            fill_line(r_border, temp_value, r_border[temp_value], 0, row - 1);     // 右边界填充到最右侧
            Straight_Flag = 0;  // 标记为弯道
        }
        else {  // 近似直道
            fill_line(l_border, temp_value, l_border[temp_value], 0, row / 2 - 1); // 左边界填充到中线左侧
            fill_line(r_border, temp_value, r_border[temp_value], 0, row / 2 + 1); // 右边界填充到中线右侧
            Straight_Flag = 1;  // 标记为直道
        }
    }
}


//----------------------------------------------------------------------------------------------------------------
// 函数名称 compare_border_judge
// 函数简介 从下往上左右单边判断自规定范围内行的边界是否小于col，最后如果符合要求的行数大于等于require_num就返回1，否则返回0
// 参数说明 direction           左边或右边
// 参数说明 start               起始行
// 参数说明 end                 结束行
// 参数说明 col                 要求的列
// 参数说明 require_num         达到要求的行数
// 返回参数
// 使用示例
// 备注信息 
//----------------------------------------------------------------------------------------------------------------
uint8 compare_border_judge (Direction direction, uint8 start, uint8 end, uint8 col, uint8 require_num)
{
    int i = 0, j = 0;
    if(end < white_length_max[0]+10)
        end = white_length_max[0]+10;
    if(start > Cut_ROW - 1)
        start = Cut_ROW - 1;
    if(end == 0)
        end = Cut_ROW - 1;
    if (direction == 0)//左边
    {
        for (i = start; i <= end; i++)
        {
            if (l_border[i] <= col)
                j++;//符合要求
        }
    }
    else//右边
    {
        for (i = start; i <= end; i++)
        {
            //if (r_border[i] >= col)
            if (Cut_COL - 1 - r_border[i] < col)
                j++;
        }
    }
    if (j >= require_num)
        return 1;
    else
        return 0;
}


//----------------------------------------------------------------------------------------------------------------
// 函数名称 search_anglepoint()
// 函数简介 模糊搜索四个角点是否存在
// 参数说明 void
// 返回参数 void
// 使用示例
// 备注信息
//----------------------------------------------------------------------------------------------------------------

// search_anglepoint 已移至 cross.cpp

//----------------------------------------------------------------------------------------------------------------
// 函数名称 Cross_judge()
// 函数简介 角点(类十字)补线
// 参数说明 void
// 返回参数 void
// 使用示例
// 备注信息 当上角点存在一个超过一定时间，是初判断为环岛或者十字，但是环岛存在一边不会丢边，而十字两边都会丢边，即使是斜入十字依然如此，
//----------------------------------------------------------------------------------------------------------------

// Cross_judge 已移至 cross.cpp
//----------------------------------------------------------------------------------------------------------------
// 函数名称 center_lianjei
// 函数简介 中线画线
// 参数说明
// 返回参数 err_new   每一行赋予权重赋值后的偏差值
// 使用示例 Camera_Function()
// 备注信息
//----------------------------------------------------------------------------------------------------------------
// void center_lianjei(int startline, int endline, float k, float b)
// {
//     int x1 = startline;
//     int x2 = endline;
//     if (k != 0)
//     {
//         for (int i = x1; i >= x2; i--)
//         {
//             Center_point[i] = k * i + b;
//         }
//     }
// }

void s_judge()
{
    uint8 temp_l = 0;
    uint8 temp_r = 0;
    uint8 num_l = 0;
    uint8 num_r = 0;

    for (int i = 0; i < Cut_ROW; i++)
        s_Center_point[i] = 0;
    if (l_land_flag == 0 && r_land_flag == 0 && cross_flag == 0 && l_start + 1 - l_effect_num < 4 && r_start + 1 - r_effect_num < 4 && l_start + r_start > 110)
    {
        if (s_wan_flag == 0)
        {
            for (int i = Cut_ROW - 3; i > white_length_max[0] + 3; i--)
            {
                if (r_border[i] > r_border[i + 1 + num_r])
                {
                    num_r++;
                    if (num_r == 3)
                        temp_r = 1;
                }
                else
                    num_r = 0;
                if (l_border[i] < l_border[i + 1 + num_l])
                {
                    num_l++;
                    if (num_l == 3)
                        temp_l = 1;
                }
                else
                    num_l = 0;
            }
            if (temp_l == 1 && temp_r == 1)
                s_wan_flag = 1;
            else
                s_wan_flag = 0;
        }
        if (white_length_max[0] > 15)
            s_wan_flag = 0;
    }
    else
        s_wan_flag = 0;
    if(l_land_flag != 0 || r_land_flag != 0)
        s_wan_flag = 0;

}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 zhang_ai_judge()
// 函数简介 障碍
// 参数说明
// 返回参数
// 使用示例 search_center();
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void zhang_ai_judge()
{
    int i = 0;
    // if(zhang_ai_flag > 0)
    //     // gpio_set_level(BEEP,0x01);
    // else
    //     // gpio_set_level(BEEP,0x00);
    if (zhang_ai_flag == 1) {
        for (i = Cut_ROW - 2; i > zhang_ai_num + 3; i--)
            if (l_border[i] == l_border[i - 1] && l_border[i] == l_border[i - 2] && l_border[i] == l_border[i - 3]) {
                fill_line(l_border, Cut_ROW - 1, 40, i, l_border[i]);
                break;
            }
        if(i == zhang_ai_num + 3)
            fill_line(l_border, Cut_ROW - 1, 40, zhang_ai_num, l_border[zhang_ai_num]);
    }
    if (zhang_ai_flag == 2){
        for (i = Cut_ROW - 2; i > zhang_ai_num + 3; i--)
            if (r_border[i] == r_border[i - 1] && r_border[i] == r_border[i - 2] && r_border[i] == r_border[i - 3]) {
                fill_line(r_border, Cut_ROW - 1, 120, i, r_border[i]);
                break;
            }
        if (i == zhang_ai_num + 3)
            fill_line(r_border, Cut_ROW - 1, 120, zhang_ai_num, r_border[zhang_ai_num]);
        }
    if(l_start > 30 && r_start > 30 && zhang_ai_flag == 0)
        for (i = Cut_ROW - 7; i > white_length_max[0] - 5; i--)
        {
            if (l_border[i] - l_border[i + 1] > 4 &&
                l_border[i] == l_border[i - 1] &&
                l_border[i] == l_border[i - 2] &&
                l_border[i] == l_border[i - 3] &&
                r_border[i - 1] - l_border[i - 1] < Straight_track_width[i - 1]) {
                fill_line(l_border, Cut_ROW - 1, 40, i - 1, l_border[i - 1]);
                zhang_ai_flag = 1;
                zhang_ai_num = i;
                break;
            }
            else if (r_border[i + 1] - r_border[i] > 4 &&
                r_border[i] == r_border[i - 1] &&
                r_border[i] == r_border[i - 2] &&
                r_border[i] == r_border[i - 3] &&
                r_border[i - 1] - l_border[i - 1] < Straight_track_width[i - 1]) {
                fill_line(r_border, Cut_ROW - 1, 120, i - 1, r_border[i - 1]);
                zhang_ai_flag = 2;
                zhang_ai_num = i;
                break;
            }
        }
    if (r_border[Cut_ROW - 2] - l_border[Cut_ROW - 2] < 115 || r_land_flag > 0 || r_land_flag > 0 || s_wan_flag > 0)
        zhang_ai_flag = 0;
}



//----------------------------------------------------------------------------------------------------------------
// 函数名称 search_center()
// 函数简介 根据边线计算中线和中线误差
// 参数说明
// 返回参数
// 使用示例 search_center();
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void search_center(int row, int col)
{
    uint8 i;
    for (i = 0; i < col - 1; i++)
    {
        width[i] = r_border[i] - l_border[i];       //记录当前赛道宽度

        //常规赛道中点及误差计算
        Center_point[i] = ((l_border[i] + r_border[i]) >> 1);//右移1位，等效除2

        //Center_err[i] = Cut_COL/2 - Center_point[i];  //这是每一行的中线误差
    }
    
    for (i = 0; i < col - 1; i++)
        Center_err[i] = Cut_COL/2 - Center_point[i];
    //regression(Cut_ROW - 2, white_length_max[0] + 2);
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 po_judge()
// 函数简介 坡道
// 参数说明
// 返回参数
// 使用示例 search_center();
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void po_judge()
{
    po_sum = 0;
    if (white_length_max[0] <= 5)
        for (int i = 10; i < 35; i++)
            po_sum += width[i];
    po_sum /= 25;
    if (po_sum >= 60 && l_start + 1 - l_effect_num < 5 && r_start + 1 - r_effect_num < 5 && l_land_flag == 0 && r_land_flag == 0 && cross_flag == 0 && r_start > 50 && l_start > 50)
        barrier_flag = 1;
    if (barrier_flag != 0)
    {
        switch (barrier_flag)
        {
        case 1: //上坡-坡顶
            for (int i = white_length_max[0]; i < white_length_max[0] + 10; i++)
                po_sum += width[i];
            po_sum /= 10;
            if (po_sum < 25)
                barrier_flag = 2;
            break;
        case 2: //坡顶-下坡
            if (white_length_max[0] <= 5 && l_land_flag == 0 && r_land_flag == 0 && cross_flag == 0)
                for (int i = 10; i < 35; i++)
                    po_sum += width[i];
            po_sum /= 25;
            if (white_length_max[0] < 25)
                barrier_flag = 3;
            break;
        // case 3: //退出坡道状态
        //     if (pitch < 2.0f)
        //     {
        //         po_num++;
        //         barrier_flag = 0; // 进入坡道中段
        //     }
        //    break;
        }
    }
}


//----------------------------------------------------------------------------------------------------------------
// 函数名称 Second_center
// 函数简介 二次扫线函数，防止中线发生突变
// 参数说明
// 返回参数
// 使用示例 Second_center ();
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void Second_center (void)
{
    uint8 i;
    for(i=Cut_ROW - 2;i > 0; i--)       //倒数第二行扫到最顶上
    {
        if((Center_point[i]-Center_point[i-1]) > 1)         //中点在上一个点右边
            Center_point[i] = Center_point[i-1] + 1;
        else if((Center_point[i] - Center_point[i-1]) == 0) //中点在上一个点上面
            Center_point[i] = Center_point[i-1];
        else                                                //中点在上一个点左边
            Center_point[i] = Center_point[i-1]-1;  
    }
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 Furthest_judge
// 函数简介 判断左右轮距离边界距离
// 参数说明
// 返回参数
// 使用示例 Furthest_judge();
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void Furthest_judge()
{
    const int max_distance_index = (int)(sizeof(distance) / sizeof(distance[0])) - 1;
    const int max_left_edge_index = (int)(sizeof(left_edge) / sizeof(left_edge[0])) - 1;
    const int max_right_edge_index = (int)(sizeof(right_edge) / sizeof(right_edge[0])) - 1;
    int scan_start = Cut_ROW - 1;
    int scan_end = white_length_max[0];

    // These lookup tables only cover rows 0-69. Clamp scan range before indexing them.
    if(scan_start > max_distance_index)
    {
        scan_start = max_distance_index;
    }
    if(scan_start > max_left_edge_index)
    {
        scan_start = max_left_edge_index;
    }
    if(scan_start > max_right_edge_index)
    {
        scan_start = max_right_edge_index;
    }
    if(scan_end < 0)
    {
        scan_end = 0;
    }
    if(scan_end > scan_start)
    {
        scan_end = scan_start;
    }

    Foresight = (uint8)scan_start;
    for (int i = scan_start; i >= scan_end; i--)
    {
        Foresight = (uint8)i;      //记录最远行
        if (l_border[i] > left_edge[i])  //左边界的列值 大于了轮胎到边界的位置 就把左边界出界行的位置赋给最远距离
        {
            Foresight_left = (uint8)i;
        }

        if (r_border[i] < right_edge[i])  //右边界的列值 小于了轮胎到边界的位置 就把右边界出界行的位置赋给最远距离
        {
            Foresight_right = (uint8)i;
        }
        if(l_border[i] > left_edge[i] || r_border[i] < right_edge[i]) //两边都一样的话 ，就是直道 直接是最长白列
        {
            break;
        }
        
    }
    Farthest_distance = distance[Foresight];    //把行数换算为实际距离
}

float FMy_Abs(float a, float b)//求两数之差绝对值的浮点数
{

    if ((a - b) > 0)
        return ((float)(a - b));
    else return ((float)(b - a));
}



float Err_Get(void)
{
    float err=0;
    float weight_count=0;
    int i;
    for(i = 0; i < 10; i++)
    {
        err+=Center_err[Search_Stop_Line + i] * weight_key[i];  //要改权重直接改
        weight_count += weight_key[i];
    }
    err = err / weight_count;  //计算误差
    // if (err>err_limit) err=err_limit;
    // if (err<-err_limit) err=-err_limit;

    return err;
}

//***************************************************************
//* 函数名称： void protect()
//* 功能说明： 车如果跑出赛道，停车
//* 函数返回： 
//* 备 注：    0表示黑色 255表示白色，大家自己思考一下怎么写
//***************************************************************
void protect()
{
    if((l_land_flag > 0 || r_land_flag > 0) && RedBlock_ShouldIgnoreBoundaryStop() == 0)
    {
        stop=1;
        l_land_flag = 0;
        r_land_flag = 0;
    }
}

// 斑马线函数已移至 zebra.cpp

void chasu_calculation() //阿克曼速度比分配
{
    // 车轮实测：直径 64mm、宽度 27mm。当前阿克曼速度比只需要轴距和轮距。
    // 调参：前后轴距，单位 mm。请按实车前后轮中心距离测量。
    const float ACK_WHEEL_BASE_MM = 160.0f;
    // 调参：左右轮距，单位 mm。你提供的轮距为 15.5cm，即 155mm。
    const float ACK_TRACK_WIDTH_MM = 155.0f;
    // 调参：最大等效转角，单位度。越大同样误差下转弯越急。
    const float ACK_MAX_STEER_DEG = 51.0f;//51
    // 调参：Servo_PID 输出满量程。越小转向越灵敏；当前先用 200，避免阿克曼差速过小。
    const float ACK_DIF_FULL_SCALE = 90.0f;//
    const float PI = 3.1415926f;

    float steer_ratio = dif_speed / ACK_DIF_FULL_SCALE;
    steer_ratio = func_limit_ab(steer_ratio, -1.0f, 1.0f);

    const float steer_rad = steer_ratio * ACK_MAX_STEER_DEG * PI / 180.0f;
    if(fabs(steer_rad) < 0.001f)
    {
        l_speed = set_speed;
        r_speed = set_speed;
    }
    else
    {
        const float radius = ACK_WHEEL_BASE_MM / tanf(fabs(steer_rad));
        const float inner_speed = (float)set_speed * (radius - ACK_TRACK_WIDTH_MM * 0.5f) / radius;
        const float outer_speed = (float)set_speed * (radius + ACK_TRACK_WIDTH_MM * 0.5f) / radius;

        if(err_new >= 0) // 左转：左轮内侧，右轮外侧
        {
            l_speed = (int32_t)inner_speed;
            r_speed = (int32_t)outer_speed;
        }
        else // 右转：右轮内侧，左轮外侧
        {
            r_speed = (int32_t)inner_speed;
            l_speed = (int32_t)outer_speed;
        }
    }

    r_speed = func_limit_ab(r_speed, -3000, 3000); //限幅
    l_speed = func_limit_ab(l_speed, -3000, 3000); //限幅

    l_out = l_pid(l_speed, enconder_left);
    r_out = r_pid(r_speed, enconder_right);
    Motor_Control(l_out,r_out);
    //Motor_Control()
    //printf("r=%d.\n",(int)r_speed);
    //printf("l=%d.\n",(int)l_speed);
    //printf("d=%d.\n",(int)dif_speed);
}

void weight_box() //动态前瞻
{
    if(l_land_flag !=0 || r_land_flag !=0)
    {
        Search_Stop_Line = land_w;
    }
    else
    {
        Search_Stop_Line = w;
    }
}

uint8 Car_ShouldPause(void)
{
    return (stop || redblock_pause_flag);
}

uint8 Car_ShouldMotorStop(void)
{
    return (zebra_flag == 3 || stop || redblock_pause_flag);
}

static void Model_Request_Process(void)
{
    if(model_request_flag == 0)
    {
        Model_Confirm_Reset();
        return;
    }

    if(g_model_stage == MODEL_STAGE_IDLE)
    {
        Model_SetStage(MODEL_STAGE_WAIT_STABLE);
        printf("Model stage -> wait_stable\n");
    }

    if(model_running_flag != 0)
    {
        return;
    }

    if(g_model_stage == MODEL_STAGE_WAIT_STABLE)
    {
        if(Model_IsStable())
        {
            if(g_model_stable_count < MODEL_STABLE_REQUIRED)
            {
                g_model_stable_count++;
            }

            printf(
                "Model wait stable: left=%d right=%d stable_count=%u/%u\n",
                enconder_left,
                enconder_right,
                g_model_stable_count,
                MODEL_STABLE_REQUIRED
            );

            if(g_model_stable_count >= MODEL_STABLE_REQUIRED)
            {
                Model_SetStage(MODEL_STAGE_DROP_VALID_FRAMES);
                printf(
                    "Model stage -> drop_valid_frames, discard first %u valid frames\n",
                    MODEL_DROP_VALID_REQUIRED
                );
            }
        }
        else
        {
            if(g_model_stable_count != 0)
            {
                printf(
                    "Model wait stable interrupted: left=%d right=%d\n",
                    enconder_left,
                    enconder_right
                );
            }
            g_model_stable_count = 0;
        }

        g_model_stage_retry_count++;
        if(g_model_stage_retry_count >= MODEL_WAIT_STABLE_TIMEOUT)
        {
            printf("Model wait stable timeout, release pause\n");
            Model_Confirm_Reset();
            RedBlock_ReleasePause();
        }
        return;
    }

    RedBlock_OnModelStarted();
    const NCNN_Infer_Result infer_result = ncnn_infer_run_once();
    if(!infer_result.ready)
    {
        printf("Model infer not ready\n");
        Model_Confirm_Reset();
        RedBlock_ReleasePause();
        return;
    }

    if(!infer_result.valid)
    {
        g_model_stage_retry_count++;
        printf(
            "Model infer skipped: roi unavailable, retry=%u/%u\n",
            g_model_stage_retry_count,
            MODEL_INFER_TIMEOUT
        );
        model_running_flag = 0;

        if(g_model_stage_retry_count >= MODEL_INFER_TIMEOUT)
        {
            printf("Model infer timeout, release pause\n");
            Model_Confirm_Reset();
            RedBlock_ReleasePause();
        }
        return;
    }

    g_model_stage_retry_count = 0;

    if(g_model_stage == MODEL_STAGE_DROP_VALID_FRAMES)
    {
        g_model_drop_valid_count++;
        printf(
            "Model discard valid frame: %u/%u fine=%d %s coarse=%d %s confidence=%.4f\n",
            g_model_drop_valid_count,
            MODEL_DROP_VALID_REQUIRED,
            infer_result.class_index,
            infer_result.fine_label.c_str(),
            infer_result.coarse_index,
            infer_result.label.c_str(),
            infer_result.confidence
        );

        if(g_model_drop_valid_count >= MODEL_DROP_VALID_REQUIRED)
        {
            Model_SetStage(MODEL_STAGE_COLLECT_VOTES);
            printf(
                "Model stage -> collect_votes, target valid frames=%u\n",
                MODEL_VOTE_REQUIRED
            );
        }

        model_running_flag = 0;
        return;
    }

    if(g_model_stage == MODEL_STAGE_COLLECT_VOTES)
    {
        Model_Vote_Add(infer_result.coarse_index);
        printf(
            "Model vote: fine=%d %s coarse=%d(%s) label=%s confidence=%.4f total=%u/%u votes=[%u,%u,%u]\n",
            infer_result.class_index,
            infer_result.fine_label.c_str(),
            infer_result.coarse_index,
            Model_ClassLabel(infer_result.coarse_index),
            infer_result.label.c_str(),
            infer_result.confidence,
            g_model_vote_valid_count,
            MODEL_VOTE_REQUIRED,
            g_model_vote_count[MODEL_CLASS_SUPPLIERS],
            g_model_vote_count[MODEL_CLASS_VEHICLE],
            g_model_vote_count[MODEL_CLASS_WEAPON]
        );

        if(g_model_vote_valid_count >= MODEL_VOTE_REQUIRED)
        {
            const int final_coarse_index = Model_Vote_GetBestClass();
            if(final_coarse_index < 0)
            {
                printf("Model vote tie or empty, release pause\n");
                Model_Confirm_Reset();
                RedBlock_ReleasePause();
                return;
            }

            NCNN_Infer_Result final_result = infer_result;
            final_result.coarse_index = final_coarse_index;
            final_result.label = Model_ClassLabel(final_coarse_index);

            printf(
                "Model final result: fine=%d %s coarse=%d label=%s votes=[%u,%u,%u]\n",
                final_result.class_index,
                final_result.fine_label.c_str(),
                final_result.coarse_index,
                final_result.label.c_str(),
                g_model_vote_count[MODEL_CLASS_SUPPLIERS],
                g_model_vote_count[MODEL_CLASS_VEHICLE],
                g_model_vote_count[MODEL_CLASS_WEAPON]
            );

            Model_Confirm_Reset();
            Model_ApplyConfirmedAction(final_result);
            return;
        }
    }

    model_running_flag = 0;
}



//----------------------------------------------------------------------------------------------------------------
// 函数名称 Camera_Function
// 函数简介 图像处理全过程
// 参数说明
// 返回参数 err_new   每一行赋予权重赋值后的偏差值
// 使用示例 Camera_Function()
// 备注信息
//----------------------------------------------------------------------------------------------------------------
float Camera_Function (void)
{
    Get_Use_Image(rgay_image, Cut_Image_Use);  //获取灰度图像
    Get_Use_Image(edge_image, Canny_Cut_Image_Use);  //获取灰度图像
    /*最长白列*/
    if(xie_cross_time == 0 && l_land_flag != 4)
        search_longest_white_col();         //搜索最长白列

    /*搜索边界*/
    // if(barrier_flag == 0)
    //     search_border(Cut_COL, Cut_ROW);     //搜索边界
    // if (error_border_flag == 1 || barrier_flag != 0)
    // {
    //     error_border_flag = 0;
    //     search_border_error(Cut_COL, Cut_ROW);
    // }
    search_border(Cut_COL, Cut_ROW);     //搜索边界

    const uint8 other_element_exclusive = (
        zebra_flag != 0 ||
        cross_flag != 0 ||
        xie_cross_flag != 0 ||
        l_land_flag != 0 ||
        r_land_flag != 0 ||
        s_wan_flag != 0 ||
        barrier_flag != 0
    );

    if(other_element_exclusive == 0)
    {
        RedBlock_Update();
        Model_Request_Process();
    }
    else
    {
        const uint8 redblock_model_only = (model_request_flag != 0 || model_running_flag != 0);
        if(redblock_model_only != 0)
        {
            Model_Request_Process();
        }
        else
        {
            RedBlock_ResetState();
        }
    }

    const uint8 redblock_exclusive = RedBlock_IsElementExclusive();

    if(redblock_exclusive == 0)
    {
        /*斑马线*/
        if(zebra_mode == 1){
            Zebra_Detect(); //状态机
        }else{
            if(go_flag == 1)
            Zebra_Detect_delay(); //延迟检测
        }

        /*十字*/
        search_anglepoint();                //模糊搜索四个角点是否存在
        if(xie_cross_flag == 1)
            search_anglepoint();
        Cross_judge ();                     //如果上方角点不丢失，判断为十字补线，否则cross_flag = 0;

        /*障碍*/
        //zhang_ai_judge();

        /*环岛*/
        l_land_judge();
        // if(l_land_num > 0)
        // l_xie_land_judge();
        r_land_judge();

        /*S弯*/
        s_judge();
    }
    else
    {
        ResetZebraDetection();
        zebra_flag = 0;
        cross_flag = 0;
        xie_cross_flag = 0;
        xie_cross_time = 0;
        left_up_flag = 0;
        left_down_flag = 0;
        right_up_flag = 0;
        right_down_flag = 0;
        left_up = 0;
        left_down = 0;
        right_up = 0;
        right_down = 0;
        l_land_flag = 0;
        r_land_flag = 0;
        s_wan_flag = 0;
        barrier_flag = 0;
    }

    RedBlock_ApplyBypass();

    /*中线处理*/
    search_center(Cut_COL,Cut_ROW);     //根据边界计算中线

    /*动态前瞻*/
    weight_box();

    /*转换距离*/
    Furthest_judge();            //转换实际距离
    
    /*坡道*/
    if(po_num == 0 && redblock_exclusive == 0)
        po_judge();

    //Second_center();                    //二次扫线
    //protect();                  //元素互斥测试

    err_new = Err_Get();         //误差计算
    return err_new;
}
