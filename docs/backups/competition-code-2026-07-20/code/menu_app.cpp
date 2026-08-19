#include "menu_app.h"

#include "camera.h"
#include "circle.h"
#include "control.h"
#include "key.h"
#include "menu_display.h"
#include "menu_settings.h"
#include "motor.h"
#include "redblock.h"
#include "tuning_menu.h"
#include "zf_device_ips200_fb.h"
#include "zf_device_uvc.h"
#include "zf_driver_gpio.h"

#include <cstring>
#include <opencv2/imgproc/imgproc.hpp>

namespace
{
    MenuCore g_menu;

    enum DisplayMode : uint8_t
    {
        DISPLAY_MODE_MENU = 0,
        DISPLAY_MODE_IMAGE = 1,
        DISPLAY_MODE_UNKNOWN = 0xFF,
    };

    enum ImageViewMode : uint8_t
    {
        IMAGE_VIEW_TRACK_GRAY = 0,
        IMAGE_VIEW_FULL_GRAY = 1,
        IMAGE_VIEW_EDGE_GRAY = 2,
        IMAGE_VIEW_EDGE_BOUNDARY = 3,
    };

    // 调参：SWITCH_0 读到该电平时显示菜单；如果实车拨码方向相反，把 0 改成 1。
    const uint8_t SWITCH0_MENU_LEVEL = 0;
    // 调参：160x90 摄像头图像在 IPS200 上的显示位置。
    const uint16_t CAMERA_IMAGE_X = 0;
    const uint16_t CAMERA_IMAGE_Y = 0;
    // 调参：TRACK_GRAY 与调试图传模式一致，显示主算法使用的 Cut_Image_Use。
    const uint16_t CAMERA_IMAGE_WIDTH = Cut_COL;
    const uint16_t CAMERA_IMAGE_HEIGHT = Cut_ROW;
    // 调参：图像侧在 IPS200 上的显示大小。240x135 是 TRACK_GRAY 等比例放大到屏幕宽度。
    const uint16_t CAMERA_DISPLAY_WIDTH = 240;
    const uint16_t CAMERA_DISPLAY_HEIGHT = 135;
    // 调参：图像侧显示方向，1 为旋转 180 度，0 为正常方向；只影响屏幕显示，不影响循迹算法。
    const uint8_t CAMERA_DISPLAY_ROTATE_180 = 1;
    // 调参：FULL_GRAY 显示大小。320x240 占满常见 IPS200 横屏，不改摄像头和算法输入。
    const uint16_t FULL_GRAY_DISPLAY_WIDTH = UVC_RAW_WIDTH;
    const uint16_t FULL_GRAY_DISPLAY_HEIGHT = UVC_RAW_HEIGHT;

    DisplayMode g_last_display_mode = DISPLAY_MODE_UNKNOWN;
    ImageViewMode g_image_view_mode = IMAGE_VIEW_TRACK_GRAY;
    uint8_t g_full_gray_buffer[UVC_RAW_WIDTH * UVC_RAW_HEIGHT];

    DisplayMode ReadDisplayMode(void)
    {
        return (gpio_get_level(SWITCH_0) == SWITCH0_MENU_LEVEL) ? DISPLAY_MODE_MENU : DISPLAY_MODE_IMAGE;
    }

    uint16_t GrayToRgb565(uint8_t gray)
    {
        const uint16_t r = (gray >> 3) & 0x1F;
        const uint16_t g = (gray >> 2) & 0x3F;
        const uint16_t b = (gray >> 3) & 0x1F;
        return (uint16_t)((r << 11) | (g << 5) | b);
    }

    void DrawScaledGrayImage(
        const uint8_t *image,
        uint16_t source_width,
        uint16_t source_height,
        uint16_t display_width,
        uint16_t display_height
    )
    {
        for(uint16_t y = 0; y < display_height; y++)
        {
            uint16_t src_y = (uint16_t)((uint32_t)y * source_height / display_height);
            if(CAMERA_DISPLAY_ROTATE_180)
            {
                src_y = (uint16_t)(source_height - 1 - src_y);
            }
            for(uint16_t x = 0; x < display_width; x++)
            {
                uint16_t src_x = (uint16_t)((uint32_t)x * source_width / display_width);
                if(CAMERA_DISPLAY_ROTATE_180)
                {
                    src_x = (uint16_t)(source_width - 1 - src_x);
                }
                const uint8_t gray = image[src_y * source_width + src_x];
                ips200_draw_point((uint16_t)(CAMERA_IMAGE_X + x), (uint16_t)(CAMERA_IMAGE_Y + y), GrayToRgb565(gray));
            }
        }
    }

    void DrawScaledTrackPoint(uint16_t source_x, uint16_t source_y, uint16_t color)
    {
        if(source_x >= CAMERA_IMAGE_WIDTH || source_y >= CAMERA_IMAGE_HEIGHT)
        {
            return;
        }

        if(CAMERA_DISPLAY_ROTATE_180)
        {
            source_x = (uint16_t)(CAMERA_IMAGE_WIDTH - 1 - source_x);
            source_y = (uint16_t)(CAMERA_IMAGE_HEIGHT - 1 - source_y);
        }

        const uint16_t display_x = (uint16_t)(CAMERA_IMAGE_X + ((uint32_t)source_x * CAMERA_DISPLAY_WIDTH / CAMERA_IMAGE_WIDTH));
        const uint16_t display_y = (uint16_t)(CAMERA_IMAGE_Y + ((uint32_t)source_y * CAMERA_DISPLAY_HEIGHT / CAMERA_IMAGE_HEIGHT));
        ips200_draw_point(display_x, display_y, color);
    }

    void DrawTrackBordersOverlay(uint8_t show_center)
    {
        for(uint16_t row = 0; row < CAMERA_IMAGE_HEIGHT; row++)
        {
            // 左边线绿色，中线黄色，右边线红色；显示叠加层只用于观察，不影响循迹计算。
            DrawScaledTrackPoint(l_border[row], row, RGB565_GREEN);
            if(show_center)
            {
                DrawScaledTrackPoint(Center_point[row], row, RGB565_YELLOW);
            }
            DrawScaledTrackPoint(r_border[row], row, RGB565_RED);
        }
    }

    void DrawForesightLineOverlay(void)
    {
        int foresight_row = (int)w;
        if(foresight_row < 0)
        {
            foresight_row = 0;
        }
        else if(foresight_row >= (int)CAMERA_IMAGE_HEIGHT)
        {
            foresight_row = (int)CAMERA_IMAGE_HEIGHT - 1;
        }

        uint16_t source_y = (uint16_t)foresight_row;
        if(CAMERA_DISPLAY_ROTATE_180)
        {
            source_y = (uint16_t)(CAMERA_IMAGE_HEIGHT - 1 - source_y);
        }

        const uint16_t display_y = (uint16_t)(CAMERA_IMAGE_Y + ((uint32_t)source_y * CAMERA_DISPLAY_HEIGHT / CAMERA_IMAGE_HEIGHT));
        for(uint16_t x = 0; x < CAMERA_DISPLAY_WIDTH; x++)
        {
            ips200_draw_point((uint16_t)(CAMERA_IMAGE_X + x), display_y, RGB565_YELLOW);
        }
    }

    uint8_t BuildFullGrayImage(void)
    {
        if(frame_rgb.empty())
        {
            return 0;
        }

        cv::Mat frame_gray;
        cv::cvtColor(frame_rgb, frame_gray, cv::COLOR_BGR2GRAY);

        int16 search_x = 0;
        int16 search_y = 0;
        uint16 search_width = 0;
        uint16 search_height = 0;
        if(RedBlock_GetSearchRect(&search_x, &search_y, &search_width, &search_height))
        {
            cv::rectangle(frame_gray, cv::Rect(search_x, search_y, search_width, search_height), cv::Scalar(120), 1);
        }

        int16 rect_x = 0;
        int16 rect_y = 0;
        uint16 rect_width = 0;
        uint16 rect_height = 0;
        if(RedBlock_GetRect(&rect_x, &rect_y, &rect_width, &rect_height))
        {
            cv::rectangle(frame_gray, cv::Rect(rect_x, rect_y, rect_width, rect_height), cv::Scalar(255), 2);
        }

        int16 roi_x = 0;
        int16 roi_y = 0;
        uint16 roi_width = 0;
        uint16 roi_height = 0;
        if(RedBlock_GetModelRoi(&roi_x, &roi_y, &roi_width, &roi_height))
        {
            cv::rectangle(frame_gray, cv::Rect(roi_x, roi_y, roi_width, roi_height), cv::Scalar(180), 1);
        }

        memcpy(g_full_gray_buffer, frame_gray.data, sizeof(g_full_gray_buffer));
        return 1;
    }

    void DrawTrackGrayImage(void)
    {
        DrawScaledGrayImage(
            (const uint8_t *)Cut_Image_Use,
            CAMERA_IMAGE_WIDTH,
            CAMERA_IMAGE_HEIGHT,
            CAMERA_DISPLAY_WIDTH,
            CAMERA_DISPLAY_HEIGHT
        );
        DrawTrackBordersOverlay(0);
        DrawForesightLineOverlay();
    }

    void DrawEdgeGrayImage(void)
    {
        DrawScaledGrayImage(
            (const uint8_t *)Canny_Cut_Image_Use,
            CAMERA_IMAGE_WIDTH,
            CAMERA_IMAGE_HEIGHT,
            CAMERA_DISPLAY_WIDTH,
            CAMERA_DISPLAY_HEIGHT
        );
        DrawForesightLineOverlay();
    }

    void DrawEdgeBoundaryImage(void)
    {
        DrawEdgeGrayImage();
        DrawTrackBordersOverlay(1);
    }

    void DrawFullGrayImage(void)
    {
        if(BuildFullGrayImage() == 0)
        {
            ips200_show_string(0, 0, "No FULL_GRAY image");
            return;
        }

        DrawScaledGrayImage(
            g_full_gray_buffer,
            UVC_RAW_WIDTH,
            UVC_RAW_HEIGHT,
            FULL_GRAY_DISPLAY_WIDTH,
            FULL_GRAY_DISPLAY_HEIGHT
        );
    }

    void DrawCameraImage(void)
    {
        if(g_image_view_mode == IMAGE_VIEW_FULL_GRAY)
        {
            DrawFullGrayImage();
        }
        else if(g_image_view_mode == IMAGE_VIEW_EDGE_GRAY)
        {
            DrawEdgeGrayImage();
        }
        else if(g_image_view_mode == IMAGE_VIEW_EDGE_BOUNDARY)
        {
            DrawEdgeBoundaryImage();
        }
        else
        {
            DrawTrackGrayImage();
        }
    }
}

void MenuApp_Init(void)
{
    TuningMenuBindings bindings{};

    MenuSettings_Load();

    bindings.servo_kp = &servo_pid_kp;
    bindings.servo_kp2 = &servo_pid_kp2;
    bindings.servo_kd = &servo_pid_kd;
    bindings.servo_gkd = &servo_pid_gkd;
    bindings.camera_w = &w;
    bindings.land_w = &land_w;
    bindings.set_speed = &set_speed;
    bindings.land_speed = &land_s;
    bindings.ack_dif_full_scale = &ack_dif_full_scale;
    bindings.redblock_detection_enable = &redblock_detection_enable;
    bindings.redblock_visual_return_mode = &redblock_visual_return_mode;

    MenuCore_Init(&g_menu, "Menu");
    TuningMenu_Register(&g_menu, &bindings);
    MenuDisplay_Init();
    MenuDisplay_RequestRefresh();
}

void MenuApp_HandleAction(MenuCoreAction action)
{
    MenuCore_HandleAction(&g_menu, action);
    MenuSettings_Save();
    MenuDisplay_RequestRefresh();
}

void MenuApp_DrawIfNeeded(void)
{
    MenuDisplay_DrawIfNeeded(&g_menu);
}

void MenuApp_DrawActiveDisplay(void)
{
    const DisplayMode mode = ReadDisplayMode();
    if(mode != g_last_display_mode)
    {
        ips200_clear();
        if(mode == DISPLAY_MODE_MENU)
        {
            MenuDisplay_RequestRefresh();
        }
        g_last_display_mode = mode;
    }

    if(mode == DISPLAY_MODE_MENU)
    {
        MenuDisplay_DrawIfNeeded(&g_menu);
    }
    else
    {
        DrawCameraImage();
    }
}

uint8_t MenuApp_IsTuningMode(void)
{
    return ReadDisplayMode() == DISPLAY_MODE_MENU;
}

void MenuApp_SelectTrackImageView(void)
{
    g_image_view_mode = IMAGE_VIEW_EDGE_GRAY;
    ips200_clear();
}

void MenuApp_SelectFullGrayView(void)
{
    g_image_view_mode = IMAGE_VIEW_FULL_GRAY;
    ips200_clear();
}

void MenuApp_SelectEdgeGrayView(void)
{
    g_image_view_mode = IMAGE_VIEW_EDGE_GRAY;
    ips200_clear();
}

void MenuApp_SelectEdgeBoundaryView(void)
{
    g_image_view_mode = IMAGE_VIEW_EDGE_BOUNDARY;
    ips200_clear();
}

MenuCore *MenuApp_GetCore(void)
{
    return &g_menu;
}
