#include "zf_device_uvc.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <thread>

using namespace cv;

//==================================================== 全局图像缓存 ====================================================

cv::Mat frame_rgb;          // 320x240 原始彩色图，供 NCNN/红块识别使用
cv::Mat frame_line_resize;  // 160x120 缩放图
cv::Mat frame_rgay;         // 160x90 巡线二值灰度图
cv::Mat frame_edge;         // 160x90 边缘图，保持全局生命周期供 edge_image 使用

uint8_t *rgay_image = nullptr; // 巡线二值灰度图数据指针
uint8_t *edge_image = nullptr; // 巡线边缘图数据指针

VideoCapture cap;

//==================================================== 调试图像保存配置 ================================================

#define UVC_DEBUG_VIDEO_ENABLE 0
#define UVC_DEBUG_VIDEO_SKIP   3
#define UVC_DEBUG_FRAME_PATH_FORMAT "./processed_debug_%06u.jpg"

#if UVC_DEBUG_VIDEO_ENABLE
static uint32_t debug_video_frame_count = 0;
static uint32_t debug_video_saved_count = 0;

static void debug_video_draw_point(cv::Mat &image, int row, int col, const cv::Scalar &color)
{
    if(row < 0 || row >= image.rows || col < 0 || col >= image.cols)
    {
        return;
    }

    cv::circle(image, cv::Point(col, row), 1, color, -1);
}
#endif

//==================================================== 内部工具函数 ====================================================

// 等待摄像头输出首帧。部分 UVC 摄像头 open 成功后立即 read 会返回空帧。
static int wait_camera_ready_ms(int timeout_ms)
{
    if(!cap.isOpened())
    {
        return -1;
    }

    const auto start_time = std::chrono::steady_clock::now();
    cv::Mat test_frame;

    while(true)
    {
        cap >> test_frame;
        if(!test_frame.empty())
        {
            frame_rgb = test_frame;
            return 0;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if(elapsed_ms >= timeout_ms)
        {
            return -1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// 将摄像头参数恢复到当前工程记录的默认值。
static void uvc_apply_default_controls()
{
    cap.set(cv::CAP_PROP_BRIGHTNESS, static_cast<double>(UVC_DEFAULT_BRIGHTNESS));
    cap.set(cv::CAP_PROP_CONTRAST, static_cast<double>(UVC_DEFAULT_CONTRAST));
    cap.set(cv::CAP_PROP_SATURATION, static_cast<double>(UVC_DEFAULT_SATURATION));
    cap.set(cv::CAP_PROP_GAIN, static_cast<double>(UVC_DEFAULT_GAIN));
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, static_cast<double>(UVC_AUTO_EXPOSURE_DISABLE));
    cap.set(cv::CAP_PROP_EXPOSURE, static_cast<double>(UVC_DEFAULT_EXPOSURE));
}

//==================================================== 摄像头初始化 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UVC 摄像头初始化
// 参数说明     path 摄像头设备路径，例如 "/dev/video0"
// 返回参数     int8 0-初始化成功  -1-初始化失败
// 使用示例     uvc_camera_init("/dev/video0");
// 备注信息     设置 MJPG、320x240、默认相机参数，并等待首帧可用
//-------------------------------------------------------------------------------------------------------------------
int8 uvc_camera_init(const char *path)
{
    cap.open(path);

    if(!cap.isOpened())
    {
        printf("find uvc camera error.\r\n");
        return -1;
    }

    printf("find uvc camera Successfully.\r\n");

    cap.set(cv::CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, UVC_RAW_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, UVC_RAW_HEIGHT);
    cap.set(CAP_PROP_FPS, UVC_FPS);

    uvc_apply_default_controls();

    const double frame_fps = cap.get(cv::CAP_PROP_FPS);
    printf("fps:%3f\n", frame_fps);
    // 读取摄像头参数，便于调试和验证设置是否生效
    // printf(
    //     "uvc controls: exposure=%.3f gain=%.3f brightness=%.3f contrast=%.3f saturation=%.3f auto=%.3f\n",
    //     cap.get(cv::CAP_PROP_EXPOSURE),
    //     cap.get(cv::CAP_PROP_GAIN),
    //     cap.get(cv::CAP_PROP_BRIGHTNESS),
    //     cap.get(cv::CAP_PROP_CONTRAST),
    //     cap.get(cv::CAP_PROP_SATURATION),
    //     cap.get(cv::CAP_PROP_AUTO_EXPOSURE)
    // );

    if(wait_camera_ready_ms(1200) != 0)
    {
        printf("uvc camera frame not ready in time.\r\n");
        return -1;
    }

    return 0;
}

//==================================================== 原始帧信息 ======================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前原始彩色帧信息
// 参数说明     image 输出图像首地址，可为 nullptr
// 参数说明     width 输出图像宽度，可为 nullptr
// 参数说明     height 输出图像高度，可为 nullptr
// 参数说明     step 输出每行字节跨度，可为 nullptr
// 返回参数     int8 0-获取成功  -1-当前无有效图像
// 使用示例     get_rgb_frame_info(&ptr, &w, &h, &step);
// 备注信息     指针指向 frame_rgb 内部缓存，下一次 wait_image_refresh 后内容会更新
//-------------------------------------------------------------------------------------------------------------------
int8 get_rgb_frame_info(const uint8_t **image, int *width, int *height, int *step)
{
    if(frame_rgb.empty())
    {
        return -1;
    }

    if(image != nullptr)
    {
        *image = frame_rgb.ptr<uint8_t>(0);
    }
    if(width != nullptr)
    {
        *width = frame_rgb.cols;
    }
    if(height != nullptr)
    {
        *height = frame_rgb.rows;
    }
    if(step != nullptr)
    {
        *step = static_cast<int>(frame_rgb.step);
    }

    return 0;
}

//==================================================== 曝光控制接口 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置自动曝光模式
// 参数说明     auto_exposure_mode 曝光模式，建议使用 UVC_AUTO_EXPOSURE_ENABLE/DISABLE
// 返回参数     int8 0-设置成功  -1-摄像头未打开或设置失败
// 使用示例     uvc_set_auto_exposure(UVC_AUTO_EXPOSURE_ENABLE);
// 备注信息     常见 UVC/V4L2 取值：3=自动曝光，1=手动曝光
//-------------------------------------------------------------------------------------------------------------------
int8 uvc_set_auto_exposure(int32 auto_exposure_mode)
{
    if(!cap.isOpened())
    {
        std::cerr << "camera not opened, can not set auto exposure!" << std::endl;
        return -1;
    }

    try
    {
        if(!cap.set(cv::CAP_PROP_AUTO_EXPOSURE, static_cast<double>(auto_exposure_mode)))
        {
            std::cerr << "set auto exposure mode failed!" << std::endl;
            return -1;
        }
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置手动曝光值
// 参数说明     exposure_value 手动曝光值，具体有效范围由摄像头决定
// 返回参数     int8 0-设置成功  -1-摄像头未打开或设置失败
// 使用示例     uvc_set_exposure_value(UVC_EXPOSURE);
// 备注信息     函数内部会先切到手动曝光模式，避免自动曝光覆盖固定曝光值
//-------------------------------------------------------------------------------------------------------------------
int8 uvc_set_exposure_value(int32 exposure_value)
{
    if(!cap.isOpened())
    {
        std::cerr << "camera not opened, can not set exposure value!" << std::endl;
        return -1;
    }

    try
    {
        if(!cap.set(cv::CAP_PROP_AUTO_EXPOSURE, static_cast<double>(UVC_AUTO_EXPOSURE_DISABLE)))
        {
            std::cerr << "set manual exposure mode failed!" << std::endl;
            return -1;
        }

        if(!cap.set(cv::CAP_PROP_EXPOSURE, static_cast<double>(exposure_value)))
        {
            std::cerr << "set exposure value failed!" << std::endl;
            return -1;
        }
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前自动曝光模式
// 返回参数     double 当前模式值，-1.0 表示摄像头未打开或读取失败
// 使用示例     double mode = uvc_get_auto_exposure_mode();
// 备注信息     可用于验证自动/手动曝光设置是否生效
//-------------------------------------------------------------------------------------------------------------------
double uvc_get_auto_exposure_mode()
{
    if(!cap.isOpened())
    {
        std::cerr << "camera not opened, can not get auto exposure mode!" << std::endl;
        return -1.0;
    }

    try
    {
        return cap.get(cv::CAP_PROP_AUTO_EXPOSURE);
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return -1.0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前曝光值
// 返回参数     double 当前曝光值，-1.0 表示摄像头未打开或读取失败
// 使用示例     double exposure = uvc_get_current_exposure();
// 备注信息     自动曝光模式下返回摄像头当前自动调节值，手动模式下返回固定曝光值
//-------------------------------------------------------------------------------------------------------------------
double uvc_get_current_exposure()
{
    if(!cap.isOpened())
    {
        std::cerr << "camera not opened, can not get current exposure value!" << std::endl;
        return -1.0;
    }

    try
    {
        return cap.get(cv::CAP_PROP_EXPOSURE);
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return -1.0;
    }
}

//==================================================== 图像刷新与预处理 ================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     阻塞刷新摄像头图像并生成巡线输入图
// 返回参数     int8 0-刷新成功  -1-刷新失败或帧为空
// 使用示例     wait_image_refresh();
// 备注信息     输出三份数据：
//              1. frame_rgb: 320x240 原始彩色图
//              2. rgay_image: 160x90 OTSU 二值灰度图
//              3. edge_image: 160x90 Canny 边缘图
//-------------------------------------------------------------------------------------------------------------------
int8 wait_image_refresh()
{
    try
    {
        cap >> frame_rgb;
        if(frame_rgb.empty())
        {
            std::cerr << "failed to get valid image frame" << std::endl;
            return -1;
        }
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return -1;
    }

    resize(frame_rgb, frame_line_resize, Size(UVC_WIDTH, UVC_HEIGHT));

    // 原巡线算法只使用 160x90 上半部分图像。
    Rect roi(0, 0, UVC_WIDTH, 90);
    Mat line_bgr = frame_line_resize(roi);
    cvtColor(line_bgr, frame_rgay, COLOR_BGR2GRAY);

    // frame_edge 必须是全局对象，否则 edge_image 指针会悬空。
    frame_edge.create(frame_rgay.size(), CV_8UC1);
    Canny(frame_rgay, frame_edge, 90, 240);//125 350 100 280

    threshold(frame_rgay, frame_rgay, 0, 255, THRESH_BINARY | THRESH_OTSU);
    rgay_image = reinterpret_cast<uint8_t *>(frame_rgay.ptr(0));
    edge_image = reinterpret_cast<uint8_t *>(frame_edge.ptr(0));

    return 0;
}

//==================================================== 调试图像输出 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存带边界/中线/误差文字的调试图像
// 参数说明     gray_image 二值灰度图首地址
// 参数说明     width/height 图像尺寸
// 参数说明     left_border/right_border/center_line 每行边界和中线，可为 nullptr
// 参数说明     row_count 有效行数
// 参数说明     err 当前误差
// 参数说明     redblock_state 红块状态
// 参数说明     redblock_action_phase 红块动作阶段
// 返回参数     无
// 使用示例     uvc_debug_video_write_processed(...)
// 备注信息     默认 UVC_DEBUG_VIDEO_ENABLE=0，不产生文件
//-------------------------------------------------------------------------------------------------------------------
void uvc_debug_video_write_processed(
    const volatile uint8_t *gray_image,
    int width,
    int height,
    const uint8_t *left_border,
    const uint8_t *right_border,
    const uint8_t *center_line,
    int row_count,
    float err,
    uint8 redblock_state,
    uint8 redblock_action_phase
)
{
#if UVC_DEBUG_VIDEO_ENABLE
    if(gray_image == nullptr || width <= 0 || height <= 0)
    {
        return;
    }

    debug_video_frame_count++;
    if((debug_video_frame_count % UVC_DEBUG_VIDEO_SKIP) != 0)
    {
        return;
    }

    cv::Mat gray(height, width, CV_8UC1);
    for(int row = 0; row < height; row++)
    {
        for(int col = 0; col < width; col++)
        {
            gray.at<uint8_t>(row, col) = gray_image[row * width + col];
        }
    }

    cv::Mat view;
    cv::cvtColor(gray, view, cv::COLOR_GRAY2BGR);

    const int rows = row_count < height ? row_count : height;
    for(int row = 0; row < rows; row++)
    {
        if(left_border != nullptr)
        {
            debug_video_draw_point(view, row, left_border[row], cv::Scalar(0, 255, 0));
        }
        if(right_border != nullptr)
        {
            debug_video_draw_point(view, row, right_border[row], cv::Scalar(0, 0, 255));
        }
        if(center_line != nullptr)
        {
            debug_video_draw_point(view, row, center_line[row], cv::Scalar(255, 0, 0));
        }
    }

    char text[128];
    snprintf(
        text,
        sizeof(text),
        "err=%.1f rb=%u act=%u",
        err,
        redblock_state,
        redblock_action_phase
    );
    cv::putText(view, text, cv::Point(4, 14), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);

    debug_video_saved_count++;
    char filename[128];
    snprintf(filename, sizeof(filename), UVC_DEBUG_FRAME_PATH_FORMAT, debug_video_saved_count);
    if(!cv::imwrite(filename, view))
    {
        printf("debug frame write failed: %s\n", filename);
    }
    else if(debug_video_saved_count == 1)
    {
        printf("debug frame recording: %s\n", UVC_DEBUG_FRAME_PATH_FORMAT);
    }
#else
    (void)gray_image;
    (void)width;
    (void)height;
    (void)left_border;
    (void)right_border;
    (void)center_line;
    (void)row_count;
    (void)err;
    (void)redblock_state;
    (void)redblock_action_phase;
#endif
}

//==================================================== 计时工具 ========================================================

int32 get_begin_time()
{
    return static_cast<int32>(cv::getTickCount());
}

double get_duration(int32 start)
{
    return (cv::getTickCount() - start) / cv::getTickFrequency();
}
