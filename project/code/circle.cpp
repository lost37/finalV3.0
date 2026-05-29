/*
 * circle.cpp
 * 环岛检测模块
 */

#include "circle.h"
#include "camera.h"
#include "motor.h"
#include <sys/time.h>

uint8 lianxu = 0;
uint8 dizeng = 0;
uint8 l_land_flag = 0;
uint8 r_land_flag = 0;
int l_land_time = 0;
uint8 land_line = 0;
volatile int land_data = 10;
volatile int land_w = 40;
uint8 imu_ring_test_active = 0;
volatile uint8 imu_ring_exit_beeped = 0;
volatile uint8 imu_ring_exit_counter = 0;
int l_land_num = 0;
static uint8 l_land_cooldown = 0;
static uint8 l_land_is_cooling = 0;
static double l_land_cooldown_start_sec = 0.0;
static const double L_LAND_COOLDOWN_SEC = 5.0;
static uint8 l_case23_confirm = 0;
static uint8 l_case10_time = 0;
static uint8 r_land_cooldown = 0;
static uint8 r_case23_confirm = 0;
static uint8 r_case10_time = 0;
static int32_t l_ring_phase_start_encoder = 0;

#define LEFT_RING_ENCODER_SWITCH 0
static const int32_t L_RING_IN_TO_RUNNING_DISTANCE = 300;

static double circle_get_time_sec(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

//----------------------------------------------------------------------------------------------------------------
// 函数名称 l_land_judge()
// 函数简介 左环岛
// 参数说明 void
// 返回参数 void
// 使用示例
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void l_land_judge()

{
    int16 i, j;
    static int8 m = 5;
    land_line = 0;
    if (l_land_is_cooling)
    {
        if (circle_get_time_sec() - l_land_cooldown_start_sec >= L_LAND_COOLDOWN_SEC)
        {
            l_land_is_cooling = 0;
            l_land_cooldown = 0;
        }
    }
    if (l_land_flag == 0 && l_land_is_cooling == 0 && cross_flag == 0 /*&& white_length_max[0] < 10 */&& r_effect_num > 55 && r_land_flag == 0 && left_down) //&& (r_start + 1 - r_effect_num) < 5 && (l_start + 1 - l_effect_num) > 5)
    {// 左方检测直道          没有进入十字        最长白列正常                 左下角点存在   右边界有效数大于115      右方检测直道
        lianxu = 1;
        dizeng = 1;
        
        imu_ring_exit_counter = 0;
        imu_ring_exit_beeped = 0;
        //gpio_set_level(BEEP, 0x01); // 关闭蜂鸣器

        //判断右边界是否连续
        for (int i = Cut_ROW - 2; i > white_length_max[0] + 4; i--)
        {
            if (func_abs(r_border[i+1] - r_border[i]) > 2)    //边界跳变大于2则temp++
                lianxu = 0;
        }
        // for (i = white_length_max[0] + 10; i > Cut_ROW - 1; i--)
        // {
        //     if (r_border[i-1] >= r_border[i])    //判断数组边界单调
        //         dizeng = 0;
        // }
        for (j = 3; j < left_down - 6; j++)
        {
            land_line = 0;  //弧点位置

            if ( l_border[j] >= l_border[j + 3] && l_border[j] >= l_border[j - 3]
                    && l_border[j] >= l_border[j + 2] && l_border[j] >= l_border[j - 2]
                    && l_border[j] >= l_border[j + 1] && l_border[j] >= l_border[j - 1])    //精确搜索弧点

            {
                if(l_border[j]>30)
                    land_line = (uint8)j;   //弧点位置正确

                if (land_line && lianxu == 1 && dizeng == 1 && white_length_max[0] < 25
                && cross_flag == 0 && compare_border_judge(left, land_line, left_down, 3, 7))
                {
                    l_land_flag = 1;
                    // l_land_num++;
                    gpio_set_level(BEEP, 0x01); // 关闭蜂鸣器

                }
            }
        }
    }
    if(l_land_flag != 0)
    {
        //l_land_time++;
        if(l_land_time > 360){
            l_land_flag = 0;
            land_line = 0;
            l_case10_time = 0;
        }
        switch (l_land_flag)
        {
        case 1: //预环岛
            imu_ring_exit_counter = 0;       //  清零 IMU 判断状态
            imu_ring_exit_beeped = 0;
            //gpio_set_level(BEEP, 0x01);      // 关闭蜂鸣器
            for (i = white_length_max[0] + 5; i < left_down - 16; i++)
            {

                if (l_border[i] > l_border[i + 5] && l_border[i] > l_border[i + 4]
                        && l_border[i] >= l_border[i + 3] && l_border[i] >= l_border[i - 3]
                        && l_border[i] >= l_border[i + 2] && l_border[i] >= l_border[i - 2]
                        && l_border[i] >= l_border[i + 1] && l_border[i] >= l_border[i - 1])
                    if (l_border[i] > 30)
                        land_line = i;
            }
            if (land_line)
                fill_line(l_border, left_down, l_border[left_down], land_line, l_border[land_line]);
            if (left_down == 0)
            {
                printf("[LEFT_RING] 1->2 land_line=%d left_up=%d left_down=%d\n", land_line, left_up, left_down);
                l_land_flag = 2;
                l_land_time = 0;
            }
            break;
        case 2: //预环岛
            for (i = white_length_max[0] + 8; i < Cut_ROW - 30; i++)
            {
                if (l_border[i] > l_border[i + 5] && l_border[i] > l_border[i - 5] && l_border[i] >= l_border[i + 4]
                        && l_border[i] >= l_border[i - 4] && l_border[i] >= l_border[i + 3]
                        && l_border[i] >= l_border[i - 3] && l_border[i] >= l_border[i + 2]
                        && l_border[i] >= l_border[i - 2] && l_border[i] >= l_border[i + 1]
                        && l_border[i] >= l_border[i - 1])
                if (l_border[i] > 30)
                    land_line = i;  //弧点
            }
            if (r_border[Cut_ROW - 2] - 158 > 2)
                fill_line(l_border, Cut_ROW - 2, r_border[Cut_ROW - 2] - 158, land_line, l_border[land_line]);
            else
                fill_line(l_border, Cut_ROW - 2, 2, land_line, l_border[land_line]);    //补出环岛丢线部分的左边界
            //当环岛点大于45行 并且 左上角点存在
            if (land_line > 15 && left_up && left_down)
            // if (land_line > 25 && left_up && left_down)
            {
                l_case23_confirm++;
            }
            else
            {
                l_case23_confirm = 0;
            }
            if (l_case23_confirm >= 3)
            {
                printf("[LEFT_RING] 2->3 land_line=%d left_up=%d left_down=%d confirm=%d\n", land_line, left_up, left_down, l_case23_confirm);
                l_land_flag = 3;
                l_land_time = 0;
                l_case23_confirm = 0;
            }
            break;
        case 3: //入环岛
            j = 0;
            for (i = white_length_max[0] + 20; i < Cut_ROW - 40; i++)   //寻找弧点
            {
                if (l_border[i] > l_border[i + 5] && l_border[i] > l_border[i - 5] && l_border[i] >= l_border[i + 4]
                        && l_border[i] >= l_border[i - 4] && l_border[i] >= l_border[i + 3]
                        && l_border[i] >= l_border[i - 3] && l_border[i] >= l_border[i + 2]
                        && l_border[i] >= l_border[i - 2] && l_border[i] >= l_border[i + 1]
                        && l_border[i] >= l_border[i - 1])
                    if (l_border[i] > 30)
                    {
                        land_line = i;
                        j = 1;
                    }
            }
            if (j != 1)
                m--;

            for (i = white_length_max[0]; i < Cut_ROW; i++)
            {
                // int tmp = (int)l_border[i] + (int)Straight_track_width[i]*0.97;
                int tmp = (int)l_border[i] + (int)Straight_track_width[i]*1.02; //1.01
                r_border[i] = (uint8)func_limit_ab(tmp, SEARCH_MIN, SEARCH_MAX);
            }


            if (left_up < 10 && l_start + 1 - l_effect_num > 10 && white_length_max[1] < Cut_COL / 3)  //列值大于 180*3/5=108
            {
                printf("[LEFT_RING] 3->4 left_up=%d l_loss=%d right_up=%d w1=%d\n", left_up, l_start + 1 - l_effect_num, right_up, white_length_max[1]);
                l_land_flag = 4;
                l_land_time = 0;
                l_ring_phase_start_encoder = encoder_acc_avg;
            }
            break;
        case 4: //入环岛
            l_land_time++;
            for (i = white_length_max[0]; i < Cut_ROW; i++)
            {
                int tmp = (int)l_border[i] + (int)Straight_track_width[i]*1.02; //1.04
                r_border[i] = (uint8)func_limit_ab(tmp, SEARCH_MIN, SEARCH_MAX);
            }
#if LEFT_RING_ENCODER_SWITCH
            {
                const int32_t encoder_progress = func_abs(encoder_acc_avg - l_ring_phase_start_encoder);
                const uint8 vision_ready = (
                    right_up == 0 &&
                    r_effect_num > 50 &&
                    r_start > 45 &&
                    l_land_time > 30
                );

                if (encoder_progress >= L_RING_IN_TO_RUNNING_DISTANCE || vision_ready)
                {
                    printf(
                        "[LEFT_RING] 4->6 progress=%d/%d vision=%u time=%d right_up=%d r_effect=%d r_start=%d\n",
                        encoder_progress,
                        L_RING_IN_TO_RUNNING_DISTANCE,
                        vision_ready,
                        l_land_time,
                        right_up,
                        r_effect_num,
                        r_start
                    );
                    l_land_flag = 6;
                    l_land_time = 0;
                }
            }
#else
            // 找到右侧稳定线后切换到绕环阶段，旧视觉切换逻辑保留用于回退
            if (right_up == 0 && r_effect_num > 50 && r_start > 45 && l_land_time > 25)
            {
                printf("[LEFT_RING] 4->6 time=%d right_up=%d r_effect=%d r_start=%d\n", l_land_time, right_up, r_effect_num, r_start);
                l_land_flag = 6;
                l_land_time = 0;
            }
#endif
            break;
        // case 6: //绕环岛
        //     for (i = Cut_ROW - 1; i >= white_length_max[0]; i--)   //寻找弧点
        //     {
        //         if (r_border[i] < r_border[i + 5] && r_border[i] < r_border[i - 5] && r_border[i] <= r_border[i + 4]
        //             && r_border[i] <= r_border[i - 4] && r_border[i] <= r_border[i + 3]
        //             && r_border[i] <= r_border[i - 3] && r_border[i] <= r_border[i + 2]
        //             && r_border[i] <= r_border[i - 2] && r_border[i] <= r_border[i + 1]
        //             && r_border[i] <= r_border[i - 1])
        //         {
        //             fill_line(r_border, i, r_border[i], left_up, l_border[left_up]);
        //             fill_line(l_border, left_up, l_border[left_up], 0, 0);
        //             fill_line(r_border, left_up, l_border[left_up], 0, 0);
        //             break;
        //         }
        //     }
        //     if(r_start + 1 - r_effect_num > 6 && white_length_max[1] > 30)
        //     {
        //         //出环岛的T字
        //         l_land_flag = 8;
        //         l_land_time = 0;
        //     }
        //     break;

        case 6: //绕环岛
            for (i = white_length_max[0]; i < Cut_ROW; i++)
            {
                int tmp = (int)r_border[i] - (int)Straight_track_width[i]*0.86;
                l_border[i] = (uint8)func_limit_ab(tmp, SEARCH_MIN, SEARCH_MAX);
            }

            if (r_start + 1 - r_effect_num > 6 && white_length_max[1] > 30)
            {
                printf("[LEFT_RING] 6->8 r_start=%d r_effect=%d w1=%d\n", r_start, r_effect_num, white_length_max[1]);
                //出环岛的T字
                l_land_flag = 8;
                l_land_time = 0;
            }
            break;
        case 8: // 出T字后准备出环
            for (i = white_length_max[0]; i < Cut_ROW; i++)
            {
                int tmp = (int)l_border[i] + (int)Straight_track_width[i]*1.06;
                r_border[i] = (uint8)func_limit_ab(tmp, SEARCH_MIN, SEARCH_MAX);
            }

            // 加速度判断：车身变正
            if (imu_acc_y < AY_EXIT_THRESHOLD && imu_acc_y > -AY_EXIT_THRESHOLD)
                imu_ring_exit_counter++;
            else
                imu_ring_exit_counter = 0;

            // 记录IMU已判断车身稳定
            if (imu_ring_exit_counter >= EXIT_FRAME_COUNT && !imu_ring_exit_beeped)
            {
                //gpio_set_level(BEEP, 0x01);    // 响蜂鸣器
                imu_ring_exit_beeped = 1;      // 标记已触发
            }

            if (left_up && r_start + 1 - r_effect_num == 0 && r_effect_num > 45)
            {
                printf("[LEFT_RING] 8->9 left_up=%d r_effect=%d imu_counter=%d\n", left_up, r_effect_num, imu_ring_exit_counter);
                l_land_flag = 9;
                l_land_time = 0;
            }
            break;

            case 9: // 出环岛阶段
                fill_line(l_border, Cut_ROW - 1, 1, left_up, l_border[left_up]);

                if(left_up == 0)
                {
                    printf("[LEFT_RING] 9->10 出环缓冲\n");
                    l_land_flag = 10;
                    l_land_time = 0;
                    l_case10_time = 0;
                }

                //search_border(Cut_COL, Cut_ROW);
                break;
            case 10: // 出环缓冲段
                l_case10_time++;
                for (i = white_length_max[0]; i < Cut_ROW; i++)
                {
                    int tmp = (int)r_border[i] - (int)Straight_track_width[i];
                    l_border[i] = (uint8)func_limit_ab(tmp, SEARCH_MIN, SEARCH_MAX);
                }

                if (l_case10_time >= 50)
                {
                    printf("[LEFT_RING] 10->0 出环完成 cooldown=5.0s time=%d\n", l_case10_time);
                    l_land_flag = 0;
                    l_land_time = 0;
                    l_land_num++;
                    l_land_is_cooling = 1;
                    l_land_cooldown_start_sec = circle_get_time_sec();
                    l_land_cooldown = 0;
                    l_case23_confirm = 0;
                    l_case10_time = 0;

                    imu_ring_exit_counter = 0;
                    imu_ring_exit_beeped = 0;
                    gpio_set_level(BEEP, 0x00);
                }

                break;
        }
    }
}

void l_xie_land_judge()
{
    int16 i, j;
    static int8 m = 5;
    land_line = 0;
    if (l_land_flag == 0 && cross_flag == 0 && white_length_max[0] < 10 && r_effect_num > 40 && r_land_flag == 0 && left_up && (r_start + 1 - r_effect_num) < 5 && l_start < 40)
    {
        lianxu = 1;
        dizeng = 1;
        for (int i = Cut_ROW - 2; i > white_length_max[0] + 4; i--)
        {
            if (func_abs(r_border[i+1] - r_border[i]) > 2)
                lianxu = 0;
        }
        for (j = 3; j < l_start; j++)
        {
            land_line = 0;
            if (l_border[j] >= l_border[j + 3] && l_border[j] >= l_border[j - 3]
                    && l_border[j] >= l_border[j + 2] && l_border[j] >= l_border[j - 2]
                    && l_border[j] >= l_border[j + 1] && l_border[j] >= l_border[j - 1])
            {
                if(l_border[j]>30)
                    land_line = (uint8)j;
                if (land_line && lianxu == 1 && dizeng == 1 && white_length_max[0] < 25 && cross_flag == 0)
                {
                    l_land_flag = 2;
                    l_land_num++;
                }
            }
        }
    }
    if(l_land_flag != 0)
    {
        if(l_land_time > 360){
            l_land_flag = 0;
            land_line = 0;
        }
        switch (l_land_flag)
        {
        case 1:
            for (i = white_length_max[0] + 5; i < left_down - 12; i++)
            {
                if (l_border[i] > l_border[i + 5] && l_border[i] > l_border[i + 4]
                        && l_border[i] >= l_border[i + 3] && l_border[i] >= l_border[i - 3]
                        && l_border[i] >= l_border[i + 2] && l_border[i] >= l_border[i - 2]
                        && l_border[i] >= l_border[i + 1] && l_border[i] >= l_border[i - 1])
                    if (l_border[i] > 30)
                        land_line = i;
            }
            if (land_line)
                fill_line(l_border, left_down, l_border[left_down], land_line, l_border[land_line]);
            if (left_down == 0)
            {
                l_land_flag = 2;
                l_land_time = 0;
            }
            break;
        case 2:
            for (i = white_length_max[0] + 8; i < Cut_ROW - 40; i++)
            {
                if (l_border[i] > l_border[i + 5] && l_border[i] > l_border[i - 5] && l_border[i] >= l_border[i + 4]
                        && l_border[i] >= l_border[i - 4] && l_border[i] >= l_border[i + 3]
                        && l_border[i] >= l_border[i - 3] && l_border[i] >= l_border[i + 2]
                        && l_border[i] >= l_border[i - 2] && l_border[i] >= l_border[i + 1]
                        && l_border[i] >= l_border[i - 1])
                if (l_border[i] > 30)
                    land_line = i;
            }
            if (r_border[Cut_ROW - 2] - 158 > 2)
                fill_line(l_border, Cut_ROW - 2, r_border[Cut_ROW - 2] - 158, land_line, l_border[land_line]);
            else
                fill_line(l_border, Cut_ROW - 2, 2, land_line, l_border[land_line]);
            if (land_line > 25 && left_up && left_down)
            {
                l_land_flag = 3;
                l_land_time = 0;
            }
            break;
        case 3:
            j = 0;
            for (i = white_length_max[0] + 20; i < Cut_ROW - 40; i++)
            {
                if (l_border[i] > l_border[i + 5] && l_border[i] > l_border[i - 5] && l_border[i] >= l_border[i + 4]
                        && l_border[i] >= l_border[i - 4] && l_border[i] >= l_border[i + 3]
                        && l_border[i] >= l_border[i - 3] && l_border[i] >= l_border[i + 2]
                        && l_border[i] >= l_border[i - 2] && l_border[i] >= l_border[i + 1]
                        && l_border[i] >= l_border[i - 1])
                    if (l_border[i] > 30)
                    {
                        land_line = i;
                        j = 1;
                    }
            }
            if (j != 1)
                m--;
            if (l_border[left_up] < 30)
            {
                fill_line(r_border, Cut_ROW - 2, r_border[Cut_ROW - 2], left_up, 70 - (Straight_track_width[left_up] / 2));
                fill_line(r_border, left_up, 70 - (Straight_track_width[left_up] / 2), 0, 2);
                fill_line(l_border, left_up + 10, 2, 0, 1);
            }
            else
            {
                fill_line(r_border, Cut_ROW - 2, r_border[Cut_ROW - 2], left_up, l_border[left_up]);
                fill_line(r_border, left_up, l_border[left_up], 0, 2);
                fill_line(l_border, left_up + 10, 2, 0, 1);
            }
            if (left_up == 0 && white_length_max[1] < Cut_COL / 3)
            {
                l_land_flag = 4;
                l_land_time = 0;
            }
            break;
        case 4:
            if(right_up == 0)
                fill_line(r_border, Cut_ROW - 1, Cut_COL - 1, 0, 0);
            else
                fill_line(r_border, Cut_ROW - 1, Cut_COL - 1, right_up, r_border[right_up]);
            if (right_up == 0 && r_effect_num > 50 && r_start > 45)
            {
                l_land_flag = 6;
                l_land_time = 0;
            }
            break;
        case 6:
            l_land_time++;
            for (i = Cut_ROW - 1; i >= white_length_max[0]; i--)
            {
                if (r_border[i] < r_border[i + 5] && r_border[i] < r_border[i - 5] && r_border[i] <= r_border[i + 4]
                    && r_border[i] <= r_border[i - 4] && r_border[i] <= r_border[i + 3]
                    && r_border[i] <= r_border[i - 3] && r_border[i] <= r_border[i + 2]
                    && r_border[i] <= r_border[i - 2] && r_border[i] <= r_border[i + 1]
                    && r_border[i] <= r_border[i - 1])
                {
                    fill_line(r_border, i, r_border[i], left_up, l_border[left_up]);
                    fill_line(l_border, left_up, l_border[left_up], 0, 0);
                    fill_line(r_border, left_up, l_border[left_up], 0, 0);
                    break;
                }
            }
            if(r_start + 1 - r_effect_num > 5 && white_length_max[1] > 30 && l_land_time > 5)
            {
                l_land_flag = 8;
                l_land_time = 0;
            }
            break;
        case 8:
            fill_line(r_border, Cut_ROW - 1, Cut_COL - 1, 1, 1);
            fill_line(l_border, Cut_ROW - 1, 1, 1, 1);
            if(left_up && r_start + 1 - r_effect_num == 0 && r_effect_num > 45)
            {
                l_land_flag = 9;
                l_land_time = 0;
            }
            break;
        case 9:
            fill_line(l_border, Cut_ROW - 1, 1, left_up, l_border[left_up]);
            if(left_up == 0 && (l_start > 60 || l_start == 0))
            {
                l_land_flag = 0;
                l_land_time = 0;
            }
            break;
        }
    }
}
//----------------------------------------------------------------------------------------------------------------
// 函数名称 r_land_judge()
// 函数简介 右环岛
// 参数说明 void
// 返回参数 void
// 使用示例
// 备注信息
//----------------------------------------------------------------------------------------------------------------
void r_land_judge()
{
    int16 i, j;
    static int8 m = 5;
    land_line = 0;
    if (r_land_cooldown > 0)
        r_land_cooldown--;
    if (r_land_flag == 0 && r_land_cooldown == 0 && cross_flag == 0 && l_effect_num > 60 && l_land_flag == 0 && right_down && 0)
    {
        lianxu = 1;
        dizeng = 1;
        //判断右边界是否连续
        imu_ring_exit_counter = 0;
        imu_ring_exit_beeped = 0;

        for (int i = Cut_ROW - 2; i > white_length_max[0] + 4; i--)
        {
            if (func_abs(l_border[i+1] - l_border[i]) > 2)
                lianxu = 0;
        }
        for (j = 0; j < right_down - 6; j++)
        {
            land_line = 0;

            if (r_border[j] <= r_border[j + 3] && r_border[j] <= r_border[j - 3]
                    && r_border[j] <= r_border[j + 2] && r_border[j] <= r_border[j - 2]
                    && r_border[j] <= r_border[j + 1] && r_border[j] <= r_border[j - 1])
            {
                if(r_border[j] < Cut_COL - 31)
                    land_line = (uint8)j;//弧点位置正确

                if (land_line && lianxu == 1 && dizeng == 1 && white_length_max[0] < 25
                && cross_flag == 0 && compare_border_judge(right, land_line, right_down, 3, 7))
                {
                    r_land_flag = 1;
                    gpio_set_level(BEEP, 0x01);
                }
            }
        }
    }
    if(r_land_flag != 0)
    {
        if(l_land_time > 360){
            r_land_flag = 0;
            land_line = 0;
            r_case10_time = 0;
        }
        switch (r_land_flag)
        {
        case 1:
            imu_ring_exit_counter = 0;
            imu_ring_exit_beeped = 0;
            for (i = white_length_max[0] + 5; i < right_down - 12; i++)
            {
                if (r_border[i] < r_border[i + 5] && r_border[i] < r_border[i + 4]
                        && r_border[i] <= r_border[i + 3] && r_border[i] <= r_border[i - 3]
                        && r_border[i] <= r_border[i + 2] && r_border[i] <= r_border[i - 2]
                        && r_border[i] <= r_border[i + 1] && r_border[i] <= r_border[i - 1])
                    if (r_border[i] < Cut_COL - 31)
                        land_line = i;
            }
            if (land_line)
                fill_line(r_border, right_down, r_border[right_down], land_line, r_border[land_line]);
            if (right_down == 0)
            {
                printf("[RIGHT_RING] 1->2 land_line=%d right_up=%d right_down=%d\n", land_line, right_up, right_down);
                r_land_flag = 2;
                l_land_time = 0;
            }
            break;
        case 2:
            for (i = white_length_max[0] + 8; i < Cut_ROW - 30; i++)
            {
                if (r_border[i] < r_border[i + 5] && r_border[i] < r_border[i - 5] && r_border[i] <= r_border[i + 4]
                        && r_border[i] <= r_border[i - 4] && r_border[i] <= r_border[i + 3]
                        && r_border[i] <= r_border[i - 3] && r_border[i] <= r_border[i + 2]
                        && r_border[i] <= r_border[i - 2] && r_border[i] <= r_border[i + 1]
                        && r_border[i] <= r_border[i - 1])
                if (r_border[i] < Cut_COL - 31)
                    land_line = i;
            }
            if (l_border[Cut_ROW - 2] > 2)
                fill_line(r_border, Cut_ROW - 2, 158 - l_border[Cut_ROW - 2], land_line, r_border[land_line]);
            else
                fill_line(r_border, Cut_ROW - 2, Cut_COL - 2, land_line, r_border[land_line]);

            if (land_line > 15 && right_up && right_down)
            {
                r_case23_confirm++;
            }
            else
            {
                r_case23_confirm = 0;
            }
            if (r_case23_confirm >= 3)
            {
                printf("[RIGHT_RING] 2->3 land_line=%d right_up=%d right_down=%d confirm=%d\n", land_line, right_up, right_down, r_case23_confirm);
                r_land_flag = 3;
                l_land_time = 0;
                r_case23_confirm = 0;
            }
            break;
        case 3:
            j = 0;
            for (i = white_length_max[0] + 20; i < Cut_ROW - 40; i++)
            {
                if (r_border[i] < r_border[i + 5] && r_border[i] < r_border[i - 5] && r_border[i] <= r_border[i + 4]
                        && r_border[i] <= r_border[i - 4] && r_border[i] <= r_border[i + 3]
                        && r_border[i] <= r_border[i - 3] && r_border[i] <= r_border[i + 2]
                        && r_border[i] <= r_border[i - 2] && r_border[i] <= r_border[i + 1]
                        && r_border[i] <= r_border[i - 1])
                    if (r_border[i] < Cut_COL - 30)
                    {
                        land_line = i;
                        j = 1;
                    }
            }
            if (j != 1)
                m--;

            for (i = white_length_max[0]; i < Cut_ROW; i++)
            {
                int tmp = (int)r_border[i] - (int)Straight_track_width[i];
                l_border[i] = (uint8)func_limit_ab(tmp, SEARCH_MIN, SEARCH_MAX);
            }

            if (right_up > Cut_COL - 10 && r_start + 1 - r_effect_num > 10 && white_length_max[1] > (Cut_COL * 2) / 3)
            {
                printf("[RIGHT_RING] 3->4 right_up=%d r_loss=%d left_up=%d w1=%d\n", right_up, r_start + 1 - r_effect_num, left_up, white_length_max[1]);
                r_land_flag = 4;
                l_land_time = 0;
            }
            break;
        case 4:
            l_land_time++;
            for (i = white_length_max[0]; i < Cut_ROW; i++)
            {
                int tmp = (int)r_border[i] - (int)Straight_track_width[i]*1.00;
                l_border[i] = (uint8)func_limit_ab(tmp, SEARCH_MIN, SEARCH_MAX);
            }
            if (left_up == 0 && l_effect_num > 50 && l_start > 45 && l_land_time > 5)
            {
                printf("[RIGHT_RING] 4->6 time=%d left_up=%d l_effect=%d l_start=%d\n", l_land_time, left_up, l_effect_num, l_start);
                r_land_flag = 6;
                l_land_time = 0;
            }
            break;
        case 6:
            for (i = white_length_max[0]; i < Cut_ROW; i++)
            {
                int tmp = (int)l_border[i] + (int)Straight_track_width[i]*1.05;
                r_border[i] = (uint8)func_limit_ab(tmp, SEARCH_MIN, SEARCH_MAX);
            }

            if (l_start + 1 - l_effect_num > 6 && white_length_max[1] < (Cut_COL - 30))
            {
                printf("[RIGHT_RING] 6->8 l_start=%d l_effect=%d w1=%d\n", l_start, l_effect_num, white_length_max[1]);
                r_land_flag = 8;
                l_land_time = 0;
            }
            break;
        case 8:
            for (i = white_length_max[0]; i < Cut_ROW; i++)
            {
                int tmp = (int)r_border[i] - (int)Straight_track_width[i];
                l_border[i] = (uint8)func_limit_ab(tmp, SEARCH_MIN, SEARCH_MAX);
            }

            if (imu_acc_y < AY_EXIT_THRESHOLD && imu_acc_y > -AY_EXIT_THRESHOLD)
                imu_ring_exit_counter++;
            else
                imu_ring_exit_counter = 0;

            if (imu_ring_exit_counter >= EXIT_FRAME_COUNT && !imu_ring_exit_beeped)
            {
                imu_ring_exit_beeped = 1;
            }

            if (right_up && l_start + 1 - l_effect_num == 0 && l_effect_num > 45)
            {
                printf("[RIGHT_RING] 8->9 right_up=%d l_effect=%d imu_counter=%d\n", right_up, l_effect_num, imu_ring_exit_counter);
                r_land_flag = 9;
                l_land_time = 0;
            }
            break;
        case 9:
            fill_line(r_border, Cut_ROW - 1, Cut_COL - 2, right_up, r_border[right_up]);

            if(right_up == 0)
            {
                printf("[RIGHT_RING] 9->10 出环缓冲\n");
                r_land_flag = 10;
                l_land_time = 0;
                r_case10_time = 0;
            }
            break;
        case 10:
            r_case10_time++;
            for (i = white_length_max[0]; i < Cut_ROW; i++)
            {
                int tmp = (int)l_border[i] + (int)Straight_track_width[i];
                r_border[i] = (uint8)func_limit_ab(tmp, SEARCH_MIN, SEARCH_MAX);
            }

            if (r_case10_time >= 40)
            {
                printf("[RIGHT_RING] 10->0 出环完成 cooldown=40 time=%d\n", r_case10_time);
                r_land_flag = 0;
                l_land_time = 0;
                r_land_cooldown = 40;
                r_case23_confirm = 0;
                r_case10_time = 0;

                imu_ring_exit_counter = 0;
                imu_ring_exit_beeped = 0;
                gpio_set_level(BEEP, 0x00);
            }
            break;
        }
    }
}
