#include "menu_app.h"

#include "camera.h"
#include "circle.h"
#include "control.h"
#include "menu_display.h"
#include "motor.h"
#include "tuning_menu.h"

namespace
{
    MenuCore g_menu;
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

MenuCore *MenuApp_GetCore(void)
{
    return &g_menu;
}
