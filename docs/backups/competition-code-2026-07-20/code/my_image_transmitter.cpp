//tcp图传

#include "my_image_transmitter.h"

#include "zf_common_headfile.h"
#include "camera.h"

#include <opencv2/imgproc/imgproc.hpp>

static uint8 g_my_image_transmitter_ready = 0;

#if (1 == MY_IMAGE_TRANSMITTER_ENABLE) && (MY_IMAGE_TRANSMITTER_MODE == MY_IMAGE_TRANSMITTER_MODE_FULL_GRAY)
static uint8 g_my_image_gray[UVC_RAW_HEIGHT][UVC_RAW_WIDTH];
#endif

#if (1 == MY_IMAGE_TRANSMITTER_ENABLE) && (MY_IMAGE_TRANSMITTER_MODE == MY_IMAGE_TRANSMITTER_MODE_TRACK_GRAY)
static uint16 g_my_image_track_rgb565[Cut_ROW][Cut_COL];

static uint16 my_image_gray_to_rgb565(uint8 gray)
{
    const uint16 r = (uint16)((gray >> 3) & 0x1F);
    const uint16 g = (uint16)((gray >> 2) & 0x3F);
    const uint16 b = (uint16)((gray >> 3) & 0x1F);
    return (uint16)((r << 11) | (g << 5) | b);
}

static int my_image_get_foresight_row(void)
{
    int foresight_row = (int)w;
    if(foresight_row < 0)
    {
        foresight_row = 0;
    }
    else if(foresight_row >= (int)Cut_ROW)
    {
        foresight_row = (int)Cut_ROW - 1;
    }
    return foresight_row;
}

static void my_image_build_track_rgb565_with_foresight(void)
{
    for(uint16 row = 0; row < Cut_ROW; row++)
    {
        for(uint16 col = 0; col < Cut_COL; col++)
        {
            g_my_image_track_rgb565[row][col] = my_image_gray_to_rgb565(Cut_Image_Use[row][col]);
        }
    }

    // 调试：在上传图像中叠加 w 对应的前瞻黄线，不修改算法使用的 Cut_Image_Use。
    const int foresight_row = my_image_get_foresight_row();
    for(uint16 col = 0; col < Cut_COL; col++)
    {
        g_my_image_track_rgb565[foresight_row][col] = RGB565_YELLOW;
    }
}

static void my_image_transmitter_send_track_gray(void)
{
    /* 发送当前巡线主算法直接使用的裁剪灰度图。
     * 这是最常用的调试视图，能直接对应中线、边界和偏差计算结果。
     */
    my_image_build_track_rgb565_with_foresight();

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_RGB565,
        (uint8 *)g_my_image_track_rgb565,
        Cut_COL,
        Cut_ROW
    );
    seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0, NULL, NULL, NULL, NULL, NULL, NULL);
    seekfree_assistant_camera_send();
}
#endif

#if (1 == MY_IMAGE_TRANSMITTER_ENABLE) && (MY_IMAGE_TRANSMITTER_MODE == MY_IMAGE_TRANSMITTER_MODE_EDGE_GRAY)
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
#endif

#if (1 == MY_IMAGE_TRANSMITTER_ENABLE) && (MY_IMAGE_TRANSMITTER_MODE == MY_IMAGE_TRANSMITTER_MODE_FULL_GRAY)
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
#endif

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
