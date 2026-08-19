/*
 * zebra.cpp
 * 斑马线检测模块
*/

#include "zebra.h"
#include "camera.h"
#include "motor.h"

volatile int zebra_mode = 1;// 0:关闭斑马线检测 1:开启斑马线检测.比赛模式
// 调参：从斑马线前发车时，1 表示跑一圈后停车，2 表示跑两圈后停车。
volatile int zebra_target_laps = 1;// 1:跑一圈后停车 2:跑两圈后停车,后续需要添加到菜单中
uint8 zebra_flag = 0;

static ZebraState zebra_state = ZEBRA_STATE_NONE;
static int consecutive_non_detections = 0;
static int zebra_detect_count = 0;
static uint8 zebra_wait_pass = 0;
static const int NON_DETECTION_THRESHOLD = 30;
// Tune: encoder distance to keep driving after zebra detection before stopping.
static const int32_t ZEBRA_STOP_DELAY_ENCODER = 40000;
static uint8 zebra_stop_delay_active = 0;
static int32_t zebra_stop_start_encoder = 0;

static int32_t Zebra_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int Zebra_TargetDetectionCount(void)// 斑马线计数：第 1 次是起点线，第 laps+1 次才是目标停车线。
{
    int laps = zebra_target_laps;

    return laps + 1;
}

static uint8 Zebra_FrameDetected(int start_row, int diff_threshold)
{
    int change_row = 0;

    for(int i = start_row; i <= Cut_ROW - 1; i++)
    {
        int change_num = 0;
        for(int j = 2; j <= Cut_COL - 1; j++)
        {
            const int temp = calc_diff_zebra(Cut_Image_Use[i][j], Cut_Image_Use[i][j - 2]);
            if(temp > diff_threshold)
            {
                change_num++;
            }
        }

        if((change_num >= 10) && (width[i] < 30))
        {
            change_row++;
        }
    }

    return (change_row >= 5) ? 1 : 0;
}

static void Zebra_StartDelayedStop(void)
{
    if(zebra_stop_delay_active == 0)
    {
        zebra_stop_delay_active = 1;
        zebra_stop_start_encoder = encoder_acc_avg;
        printf("[ZEBRA] detected -> delay_stop start_enc=%ld target=%ld\n",
               (long)zebra_stop_start_encoder,
               (long)ZEBRA_STOP_DELAY_ENCODER);
        gpio_set_level(BEEP, 0x01);
    }
}

static void Zebra_UpdateDelayedStop(void)
{
    if(zebra_stop_delay_active != 0)
    {
        const int32_t progress = Zebra_Abs32(encoder_acc_avg - zebra_stop_start_encoder);
        if(progress >= ZEBRA_STOP_DELAY_ENCODER)
        {
            zebra_stop_delay_active = 0;
            zebra_flag = 3;
            stop = 1;
            printf("[ZEBRA] delay_stop_done progress=%ld target=%ld\n",
                   (long)progress,
                   (long)ZEBRA_STOP_DELAY_ENCODER);
        }
    }
}

static void Zebra_UpdateLapState(uint8 current_zebra_detected)
{
    if(current_zebra_detected)
    {
        if(zebra_wait_pass == 0)
        {
            zebra_detect_count++;
            consecutive_non_detections = 0;
            zebra_wait_pass = 1;

            if(zebra_detect_count >= Zebra_TargetDetectionCount())
            {
                Zebra_StartDelayedStop();
                zebra_state = ZEBRA_STATE_DETECTED_2;
                l_land_num = 0;
            }
            else
            {
                zebra_state = ZEBRA_STATE_DETECTED_1;
            }
        }
        else
        {
            consecutive_non_detections = 0;
        }
    }
    else if(zebra_wait_pass != 0)
    {
        consecutive_non_detections++;
        if(consecutive_non_detections >= NON_DETECTION_THRESHOLD)
        {
            consecutive_non_detections = 0;
            zebra_wait_pass = 0;
            zebra_state = ZEBRA_STATE_PASSED_1;
        }
    }

    if(zebra_stop_delay_active == 0 && zebra_flag != 3)
    {
        zebra_flag = (zebra_detect_count == 0) ? 0 : 1;
    }
}

void Zebra_Detect()  //竖着的条纹检测
{
    static int t = 0;
    Zebra_UpdateDelayedStop();
    if(zebra_stop_delay_active != 0 || zebra_flag == 3)
    {
        return;
    }

    t++;
    if (t > 10)
    {
        Zebra_UpdateLapState(Zebra_FrameDetected(35, 28));
    }
}

void ResetZebraDetection()
{
    zebra_state = ZEBRA_STATE_NONE;
    consecutive_non_detections = 0;
    zebra_detect_count = 0;
    zebra_wait_pass = 0;
    zebra_stop_delay_active = 0;
}

uint8 Zebra_IsStopDelayActive(void)
{
    return zebra_stop_delay_active;
}

void Zebra_Detect_delay()
{
    int i, j;
    static int t = 0;
    int change_num = 0, temp = 0, change_row = 0;
    Zebra_UpdateDelayedStop();
    if(zebra_stop_delay_active != 0 || zebra_flag == 3)
    {
        return;
    }

    t++;
    if (t > 100)
    {
        for (i = 50; i <= Cut_ROW - 1; i++)
        {
            change_num = 0;
            for (j = 2; j <= Cut_COL-1; j++)
            {
                temp = calc_diff_zebra(Cut_Image_Use[i][j], Cut_Image_Use[i][j - 2]);
                if (temp > 10)
                {
                    change_num++;
                }
            }
            if ((change_num >= 10) && (width[i] < 30))
                change_row++;

            if(change_row >= 5)
            {
                Zebra_StartDelayedStop();
                break;
            }
        }
    }
}
