//tcp图传

#include "my_image_transmitter.h"

#include "zf_common_headfile.h"
#include "camera.h"

#include <opencv2/imgproc/imgproc.hpp>

static uint8 g_my_image_transmitter_ready = 1;
static uint8 g_my_image_gray[UVC_RAW_HEIGHT][UVC_RAW_WIDTH];

static void my_image_transmitter_send_track_gray(void)
{
    /* 发送当前巡线主算法直接使用的裁剪灰度图。
     * 这是最常用的调试视图，能直接对应中线、边界和偏差计算结果。
     */
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        (uint8 *)Cut_Image_Use,
        Cut_COL,
        Cut_ROW
    );
    seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0, NULL, NULL, NULL, NULL, NULL, NULL);
    seekfree_assistant_camera_send();
}

static void my_image_transmitter_send_edge_gray(void)
{
    /* 发送 Canny 裁剪边缘图。
     * 这个视图更适合看边界提取、断边、噪声和轮廓是否稳定。
     */
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        (uint8 *)Canny_Cut_Image_Use,
        Cut_COL,
        Cut_ROW
    );
    seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0, NULL, NULL, NULL, NULL, NULL, NULL);
    seekfree_assistant_camera_send();
}

static void my_image_transmitter_send_full_gray(void)
{
    if(frame_rgb.empty())
    {
        return;
    }

    cv::Mat frame_gray;

    /* 整图调试模式：
     * 从原始彩色图转成灰度图，再叠加红色块检测相关的辅助框，
     * 更适合联调整体视觉链，而不是只看巡线结果。
     */
    cv::cvtColor(frame_rgb, frame_gray, cv::COLOR_BGR2GRAY);

    int16 search_x = 0;
    int16 search_y = 0;
    uint16 search_width = 0;
    uint16 search_height = 0;
    if(RedBlock_GetSearchRect(&search_x, &search_y, &search_width, &search_height))
    {
        cv::rectangle(
            frame_gray,
            cv::Rect(search_x, search_y, search_width, search_height),
            cv::Scalar(120),
            1
        );
    }

    int16 rect_x = 0;
    int16 rect_y = 0;
    uint16 rect_width = 0;
    uint16 rect_height = 0;
    if(RedBlock_GetRect(&rect_x, &rect_y, &rect_width, &rect_height))
    {
        cv::rectangle(
            frame_gray,
            cv::Rect(rect_x, rect_y, rect_width, rect_height),
            cv::Scalar(255),
            2
        );
    }

    int16 roi_x = 0;
    int16 roi_y = 0;
    uint16 roi_width = 0;
    uint16 roi_height = 0;
    if(RedBlock_GetModelRoi(&roi_x, &roi_y, &roi_width, &roi_height))
    {
        cv::rectangle(
            frame_gray,
            cv::Rect(roi_x, roi_y, roi_width, roi_height),
            cv::Scalar(180),
            1
        );
    }

    memcpy(g_my_image_gray[0], frame_gray.data, UVC_RAW_WIDTH * UVC_RAW_HEIGHT);
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        g_my_image_gray[0],
        UVC_RAW_WIDTH,
        UVC_RAW_HEIGHT
    );
    seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0, NULL, NULL, NULL, NULL, NULL, NULL);
    seekfree_assistant_camera_send();
}

uint8 my_image_transmitter_init(void)
{
#if (1 == MY_IMAGE_TRANSMITTER_ENABLE)
    /* 图传初始化失败只影响调试，不影响主控链继续运行。 */
    if(tcp_client_init(MY_IMAGE_TRANSMITTER_SERVER_IP, MY_IMAGE_TRANSMITTER_PORT) == 0)
    {
        seekfree_assistant_interface_init(tcp_client_send_data, tcp_client_read_data);
        g_my_image_transmitter_ready = 1;

#if (1 == MY_IMAGE_TRANSMITTER_PRINT_STATUS)
        printf("my_image_transmitter connected\r\n");
#endif
    }
    else
    {
        g_my_image_transmitter_ready = 0;

#if (1 == MY_IMAGE_TRANSMITTER_PRINT_STATUS)
        printf("my_image_transmitter connect failed\r\n");
#endif
    }
#else
    g_my_image_transmitter_ready = 0;
#endif

    return g_my_image_transmitter_ready;
}

void my_image_transmitter_send(void)
{
#if (1 == MY_IMAGE_TRANSMITTER_ENABLE)
    if(!g_my_image_transmitter_ready)
    {
        /* 未连接时静默返回，避免把调试逻辑耦合进主循环控制链。 */
        return;
    }

#if (MY_IMAGE_TRANSMITTER_MODE == MY_IMAGE_TRANSMITTER_MODE_FULL_GRAY)
    my_image_transmitter_send_full_gray();
#elif (MY_IMAGE_TRANSMITTER_MODE == MY_IMAGE_TRANSMITTER_MODE_TRACK_GRAY)
    my_image_transmitter_send_track_gray();
#elif (MY_IMAGE_TRANSMITTER_MODE == MY_IMAGE_TRANSMITTER_MODE_EDGE_GRAY)
    my_image_transmitter_send_edge_gray();
#endif
#endif
}

void my_image_transmitter_deinit(void)
{
    g_my_image_transmitter_ready = 0;
}
