#ifndef _zf_driver_uvc_h
#define _zf_driver_uvc_h

#include "zf_common_typedef.h"
#include <opencv2/core.hpp>

#define UVC_WIDTH       160
#define UVC_HEIGHT      120
#define UVC_FPS         120
#define UVC_EXPOSURE    190     //曝光
 #define UVC_GAIN        1      //曝光增益
#define UVC_RAW_WIDTH   320
#define UVC_RAW_HEIGHT  240

int8 uvc_camera_init(const char *path);
int8 wait_image_refresh();
int8 get_rgb_frame_info(const uint8_t **image, int *width, int *height, int *step);
int32 get_begin_time();
double get_duration(int32 start);

extern uint8_t *rgay_image;
extern uint8_t *edge_image;
extern cv::Mat frame_rgb;

#endif
