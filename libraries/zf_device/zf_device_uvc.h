#ifndef _zf_driver_uvc_h
#define _zf_driver_uvc_h

#include "zf_common_typedef.h"
#include <opencv2/core.hpp>

// UVC 图像配置
#define UVC_WIDTH       160
#define UVC_HEIGHT      120
#define UVC_FPS         120       // 摄像头默认 FPS 读取常为 0；0 表示初始化时不强制设置
#define UVC_EXPOSURE    190      // 默认手动曝光值，具体有效范围取决于摄像头-5
#define UVC_GAIN        2       // 曝光增益预留参数
#define UVC_RAW_WIDTH   320
#define UVC_RAW_HEIGHT  240
#define UVC_DEFAULT_EXPOSURE     UVC_EXPOSURE
#define UVC_DEFAULT_GAIN          2
#define UVC_DEFAULT_BRIGHTNESS    5 //亮度
#define UVC_DEFAULT_CONTRAST      15
#define UVC_DEFAULT_SATURATION    30  //饱和度31
#define UVC_DEFAULT_AUTO_EXPOSURE -1

// OpenCV/V4L2 常用曝光模式：1=手动曝光，3=自动曝光。
typedef enum
{
    UVC_AUTO_EXPOSURE_DISABLE = 1,
    UVC_AUTO_EXPOSURE_ENABLE  = 3,
} uvc_exposure_type_enum;

// 摄像头初始化：打开设备、设置 MJPG/分辨率/帧率，并等待首帧可用。
int8 uvc_camera_init(const char *path);

// 阻塞刷新一帧图像，并更新巡线灰度图、边缘图和原始彩色图缓存。
int8 wait_image_refresh();

// 获取当前 640x480 原始彩色帧信息，供 NCNN/红块识别使用。
int8 get_rgb_frame_info(const uint8_t **image, int *width, int *height, int *step);

// 设置自动曝光模式。mode 建议使用 UVC_AUTO_EXPOSURE_ENABLE/DISABLE。
int8 uvc_set_auto_exposure(int32 auto_exposure_mode);

// 设置固定曝光值。函数内部会先切到手动曝光模式。
int8 uvc_set_exposure_value(int32 exposure_value);

// 读取当前自动曝光模式，失败返回 -1.0。
double uvc_get_auto_exposure_mode();

// 读取当前曝光值，失败返回 -1.0。
double uvc_get_current_exposure();

// 调试：把处理后的巡线结果写成图片文件。默认编译关闭。
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
);

// 计时工具。
int32 get_begin_time();
double get_duration(int32 start);

// 当前帧缓存指针。
extern uint8_t *rgay_image;
extern uint8_t *edge_image;
extern cv::Mat frame_rgb;

#endif
