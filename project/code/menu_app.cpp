#include "menu_app.h"

#include "camera.h"
#include "circle.h"
#include "control.h"
#include "key.h"
#include "menu_display.h"
#include "motor.h"
#include "tuning_menu.h"
#include "zf_device_ips200_fb.h"
#include "zf_device_uvc.h"
#include "zf_driver_gpio.h"

namespace
{
    MenuCore g_menu;

    enum DisplayMode : uint8_t
    {
        DISPLAY_MODE_MENU = 0,
        DISPLAY_MODE_IMAGE = 1,
        DISPLAY_MODE_UNKNOWN = 0xFF,
    };

    // 调参：SWITCH_0 读到该电平时显示菜单；如果实车拨码方向相反，把 0 改成 1。
    const uint8_t SWITCH0_MENU_LEVEL = 0;
    // 调参：160x90 摄像头图像在 IPS200 上的显示位置。
    const uint16_t CAMERA_IMAGE_X = 0;
    const uint16_t CAMERA_IMAGE_Y = 0;
    // 调参：当前循迹灰度图尺寸是 160x90，不是 UVC 缩放后的 160x120。
    const uint16_t CAMERA_IMAGE_WIDTH = UVC_WIDTH;
    const uint16_t CAMERA_IMAGE_HEIGHT = 90;
    // 调参：图像侧在 IPS200 上的显示大小。240x135 是 160x90 等比例放大到屏幕宽度。
    const uint16_t CAMERA_DISPLAY_WIDTH = 240;
    const uint16_t CAMERA_DISPLAY_HEIGHT = 135;
    // 调参：图像侧显示方向，1 为旋转 180 度，0 为正常方向；只影响屏幕显示，不影响循迹算法。
    const uint8_t CAMERA_DISPLAY_ROTATE_180 = 1;

    DisplayMode g_last_display_mode = DISPLAY_MODE_UNKNOWN;

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

    void DrawScaledGrayImage(const uint8_t *image)
    {
        for(uint16_t y = 0; y < CAMERA_DISPLAY_HEIGHT; y++)
        {
            uint16_t src_y = (uint16_t)((uint32_t)y * CAMERA_IMAGE_HEIGHT / CAMERA_DISPLAY_HEIGHT);
            if(CAMERA_DISPLAY_ROTATE_180)
            {
                src_y = (uint16_t)(CAMERA_IMAGE_HEIGHT - 1 - src_y);
            }
            for(uint16_t x = 0; x < CAMERA_DISPLAY_WIDTH; x++)
            {
                uint16_t src_x = (uint16_t)((uint32_t)x * CAMERA_IMAGE_WIDTH / CAMERA_DISPLAY_WIDTH);
                if(CAMERA_DISPLAY_ROTATE_180)
                {
                    src_x = (uint16_t)(CAMERA_IMAGE_WIDTH - 1 - src_x);
                }
                const uint8_t gray = image[src_y * CAMERA_IMAGE_WIDTH + src_x];
                ips200_draw_point((uint16_t)(CAMERA_IMAGE_X + x), (uint16_t)(CAMERA_IMAGE_Y + y), GrayToRgb565(gray));
            }
        }
    }

    void DrawCameraImage(void)
    {
        if(rgay_image == nullptr)
        {
            ips200_show_string(0, 0, "No 160x90 image");
            return;
        }

        DrawScaledGrayImage(rgay_image);
    }
}

void MenuApp_Init(void)
{
    TuningMenuBindings bindings{};

    bindings.servo_kp = &servo_pid_kp;
    bindings.servo_kp2 = &servo_pid_kp2;
    bindings.servo_kd = &servo_pid_kd;
    bindings.servo_gkd = &servo_pid_gkd;
    bindings.camera_w = &w;
    bindings.land_w = &land_w;
    bindings.set_speed = &set_speed;
    bindings.land_speed = &land_s;

    MenuCore_Init(&g_menu, "Menu");
    TuningMenu_Register(&g_menu, &bindings);
    MenuDisplay_Init();
    MenuDisplay_RequestRefresh();
}

void MenuApp_HandleAction(MenuCoreAction action)
{
    MenuCore_HandleAction(&g_menu, action);
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

MenuCore *MenuApp_GetCore(void)
{
    return &g_menu;
}
