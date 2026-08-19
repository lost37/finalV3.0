/*
 * cross.cpp
 * 十字路口检测模块
 */

#include "cross.h"
#include "camera.h"

uint8 left_up_flag = 0;
uint8 left_down_flag = 0;
uint8 right_up_flag = 0;
uint8 right_down_flag = 0;
uint8 left_up = 0;
uint8 left_down = 0;
uint8 right_up = 0;
uint8 right_down = 0;
uint8 cross_flag = 0;
uint8 xie_cross_flag = 0;
uint8 xie_cross_time = 0;

void search_anglepoint()
{
    uint8 i;
    left_up_flag = 0;
    right_up_flag = 0;
    left_down_flag = 0;
    right_down_flag = 0;
    left_up = 0;
    left_down = 0;
    right_up = 0;
    right_down = 0;

    //搜左角点
    if(l_border[l_start - 1] <= 7)
        i = l_start - 8;
    else
        i = Cut_ROW - 6;
    for (; i > white_length_max[0] + 2; i--)
    {
        if (compare_border_judge(left, i, i + 8, (l_border[i] / 4), 5) && l_border[i] <= l_border[i - 1])
        {
            left_up = i;
            left_up_flag = 1;
        }
    }
    for (i = Cut_ROW - 2; i > left_up + 6; i--)
    {
        if(compare_border_judge(left, i-15, i, (l_border[i] / 2), 5) && left_down_flag == 0 && l_border[i] >= l_border[i + 1]
                && i > 10)
        {
            left_down = i;
            left_down_flag = 1;
        }
    }

    //搜右角点
    if(r_border[r_start - 1] >= Cut_COL - 8)
        i = r_start - 8;
    else
        i = Cut_ROW - 6;
    for (; i > white_length_max[0] + 2; i--)
    {
        if (compare_border_judge(right, i, i + 8, ((Cut_COL - 1 - r_border[i]) / 4), 5)
                && r_border[i] >= r_border[i - 1])
        {
            right_up = i;
            right_up_flag = 1;
        }
    }
    for (i = Cut_ROW - 2; i > right_up + 6; i--)
    {
        if(compare_border_judge(right, i-15, i, ((Cut_COL - 1 - r_border[i]) / 2), 5) && right_down_flag == 0
                && r_border[i] <= r_border[i + 1] && i > 10)
        {
            right_down = i;
            right_down_flag = 1;
        }
    }

    if(white_length_max[1] - l_border[left_down] < 20 && r_start + 1 - r_effect_num > 15 && white_length_max[0] <= 15 && l_land_flag == 0 && r_land_flag == 0)
    {
        white_length_max[1] = (Cut_COL - 1 + l_border[left_down]) / 2;
        xie_cross_flag = 1;
    }
    if(xie_cross_flag == 1 || xie_cross_time > 0)
    {
        xie_cross_time++;
        if(xie_cross_time >= 5)
            xie_cross_time = 0;
    }
}

void Cross_judge()
{
    static int Cross_end = 0;
    static int Cross_t = 0;
    if(left_up || right_up)
        Cross_t++;
    if(Cross_t >= 5 && left_up_flag && right_up_flag &&
            compare_border_judge(left, left_up, left_up + 15, 3, 5) &&
            compare_border_judge(right, right_up, right_up + 15, 3, 5) &&
            r_land_flag == 0 && l_land_flag == 0 && cross_flag == 0)
        cross_flag = 1;
    if(cross_flag == 1)
    {
        if (left_up_flag == 1 && right_up_flag == 1 && left_down_flag == 1 && right_down_flag == 1)
        {
            fill_line(l_border, left_down, l_border[left_down], left_up, l_border[left_up]);
            fill_line(r_border, right_down, r_border[right_down], right_up, r_border[right_up]);
        }
        else
        {
            if (left_up_flag == 0 || right_up_flag == 0)
            {
                cross_flag = 0;
                Cross_t = 0;
                Cross_end = 0;
                xie_cross_flag = 0;
            }
            else
            {
                if (left_down_flag == 0 && l_start != 0 && right_down_flag == 1 && Cross_end == 0)
                {
                    fill_line(l_border, Cut_ROW - 1, 10, left_up, l_border[left_up]);
                    fill_line(r_border, right_down, r_border[right_down], right_up, r_border[right_up]);
                }
                else if (right_down_flag == 0 && r_start != 0 && left_down_flag == 1 && Cross_end == 0)
                {
                    fill_line(l_border, left_down, l_border[left_down], left_up, l_border[left_up]);
                    fill_line(r_border, Cut_ROW - 1, Cut_COL - 10, right_up, r_border[right_up]);
                }
                else
                {
                    fill_line(l_border, Cut_ROW - 1, 10, left_up, l_border[left_up]);
                    fill_line(r_border, Cut_ROW - 1, Cut_COL - 10, right_up, r_border[right_up]);
                    Cross_end = 1;
                }
            }
        }
    }
}
