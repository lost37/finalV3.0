/*
 * zebra.cpp
 * 斑马线检测模块
 */

#include "zebra.h"
#include "camera.h"

volatile int zebra_mode = 0;
uint8 zebra_flag = 0;

static ZebraState zebra_state = ZEBRA_STATE_NONE;
static int consecutive_non_detections = 0;
static const int NON_DETECTION_THRESHOLD = 30;

void Zebra_Detect()
{
    int i, j;
    static int t = 0;
    int change_num = 0, temp = 0, change_row = 0;
    int current_zebra_detected = 0;
    t++;
    if (t > 10)
    {
        for (i = 35; i <= Cut_ROW - 1; i++)
        {
            change_num = 0;
            for (j = 2; j <= Cut_COL-1; j++)
            {
                temp = calc_diff_zebra(Cut_Image_Use[i][j], Cut_Image_Use[i][j - 2]);
                if (temp > 28)
                {
                    change_num++;
                }
            }
            if ((change_num >= 10) && (width[i] < 30))
                change_row++;

            current_zebra_detected = (change_row >= 5);

            switch(zebra_state)
            {
                case ZEBRA_STATE_NONE:
                    if (current_zebra_detected) {
                        zebra_state = ZEBRA_STATE_DETECTED_1;
                    }
                    break;

                case ZEBRA_STATE_DETECTED_1:
                    if (!current_zebra_detected)
                    {
                        consecutive_non_detections++;
                        if (consecutive_non_detections >= NON_DETECTION_THRESHOLD)
                        {
                            zebra_state = ZEBRA_STATE_PASSED_1;
                        }
                    }
                    else
                    {
                        consecutive_non_detections = 0;
                    }
                    break;

                case ZEBRA_STATE_PASSED_1:
                    if (current_zebra_detected) {
                        zebra_state = ZEBRA_STATE_DETECTED_2;
                        l_land_num = 0;
                    }
                    break;

                case ZEBRA_STATE_DETECTED_2:
                    break;
            }

            zebra_flag = (int)zebra_state;
        }
    }
}

void ResetZebraDetection()
{
    zebra_state = ZEBRA_STATE_NONE;
    consecutive_non_detections = 0;
}

void Zebra_Detect_delay()
{
    int i, j;
    static int t = 0;
    int change_num = 0, temp = 0, change_row = 0;
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
                zebra_flag = 3;
                gpio_set_level(BEEP, 0x01);
                break;
            }
        }
    }
}
