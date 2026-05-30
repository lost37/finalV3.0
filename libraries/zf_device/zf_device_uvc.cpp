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

using namespace cv;

cv::Mat frame_rgb;          // 320x240 原始彩色图，供 NCNN 使用
cv::Mat frame_line_resize;  // 160x120 缩放图
cv::Mat frame_rgay;         // 160x90 巡线灰度图

uint8_t *rgay_image;        // 巡线灰度图像数组指针
uint8_t *edge_image;        // 巡线轮廓图像数组指针

VideoCapture cap;

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
    sleep(1);
    // 获取视频流
    Mat frame;
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
    Mat lunkuo = Mat::zeros(Size(UVC_WIDTH, UVC_HEIGHT), CV_8UC1);
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

    lunkuo = lunkuo(roi);
    Canny(frame_rgay, lunkuo, 125, 350);

    threshold(frame_rgay, frame_rgay, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // cv对象转指针
    rgay_image = reinterpret_cast<uint8_t *>(frame_rgay.ptr(0));
    edge_image = reinterpret_cast<uint8_t *>(lunkuo.ptr(0));

    return 0;
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

