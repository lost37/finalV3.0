#include "zf_device_uvc.h"


#include <opencv2/imgproc/imgproc.hpp>  // for cv::cvtColor
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#include <opencv2/opencv.hpp>

#include <iostream> // for std::cerr
#include <fstream>  // for std::ofstream
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstdint>
#include <cstdio>

using namespace cv;

cv::Mat frame_rgb;          // 320x240 原始彩色图，供 NCNN 使用
cv::Mat frame_line_resize;  // 160x120 缩放图
cv::Mat frame_rgay;         // 160x90 巡线灰度图
cv::Mat frame_edge;         // 160x90 edge image with global lifetime for edge_image.

uint8_t *rgay_image;        // 巡线灰度图像数组指针
uint8_t *edge_image;        // 巡线轮廓图像数组指针

VideoCapture cap;

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

double customOtsu(const cv::Mat& image) {
    cv::Mat hist;
    int channels[] = {0};
    int histSize[] = {256};
    float range[] = {0, 256};
    const float* ranges[] = {range};
    cv::calcHist(&image, 1, channels, cv::Mat(), hist, 1, histSize, ranges);

    double total_pixels = image.rows * image.cols;
    double sum = 0, sumB = 0, wB = 0, wF = 0, max_var = 0;
    double threshold = 0;

    for (int t = 0; t < 256; t++) sum += t * hist.at<float>(t);

    for (int t = 0; t < 256; t++) {
        wB += hist.at<float>(t);
        if (wB == 0) continue;
        wF = total_pixels - wB;
        if (wF == 0) break;

        sumB += t * hist.at<float>(t);
        double mB = sumB / wB;
        double mF = (sum - sumB) / wF;
        double var = wB * wF * (mB - mF) * (mB - mF);

        if (var > max_var) {
            max_var = var;
            threshold = t;
        }
    }
    return threshold;
}

int8 uvc_camera_init(const char *path)
{
    cap.open(path);

    if(!cap.isOpened())
    {
        printf("find uvc camera error.\r\n");
        return -1;
    } 
    else 
    {
        printf("find uvc camera Successfully.\r\n");
    }

    // 设置视频流编码器
    cap.set(cv::CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 320);     // 设置摄像头原始宽度
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);   // 设置摄像头原始高度
    cap.set(CAP_PROP_FPS, 120);

    double frame_fps = cap.get(cv::CAP_PROP_FPS);
    printf("fps:%3f\n", frame_fps);
    if(wait_camera_ready_ms(1200) != 0)
    {
        printf("uvc camera frame not ready in time.\r\n");
        return -1;
    }
    return 0;
}
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

int8 wait_image_refresh()
{
    try
    {
        // 阻塞式等待图像刷新
        cap >> frame_rgb;
        // cap.read(frame_rgb);
        if (frame_rgb.empty())
        {
            std::cerr << "未获取到有效图像帧" << std::endl;
            return -1;
        }
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        return -1;
    }

    resize(frame_rgb, frame_line_resize, Size(UVC_WIDTH, UVC_HEIGHT));



    // 裁剪出 160x90 供原巡线流程继续使用
    Rect roi(0, 0, UVC_WIDTH, 90);
    Mat line_bgr = frame_line_resize(roi);
    // rgb转灰度
    cvtColor(line_bgr, frame_rgay, COLOR_BGR2GRAY);

    // Keep frame_edge global so edge_image remains valid after this function returns.
    frame_edge.create(frame_rgay.size(), CV_8UC1);
    Canny(frame_rgay, frame_edge, 125, 350);

    threshold(frame_rgay, frame_rgay, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // Expose OpenCV buffers to the existing C-style image pipeline.
    rgay_image = reinterpret_cast<uint8_t *>(frame_rgay.ptr(0));
    edge_image = reinterpret_cast<uint8_t *>(frame_edge.ptr(0));

    return 0;
}

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


int32 get_begin_time()
{
    const int32 start = cv::getTickCount(); // 获取当前时间
    return start;
}


double get_duration(int64 start)
{
    double duration = (cv::getTickCount() - start) / cv::getTickFrequency(); // 计算时间差
    return duration;
}

