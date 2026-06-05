#include <cmath>
#include <cstdlib>
#include <iostream>

#include "menu_core.h"
#include "menu_render.h"
#include "tuning_menu.h"

static int g_failed = 0;

#define CHECK_TRUE(expr) do { \
    if(!(expr)) { \
        std::cerr << "CHECK_TRUE failed at line " << __LINE__ << ": " << #expr << "\n"; \
        g_failed = 1; \
    } \
} while(0)

#define CHECK_EQ(actual, expected) do { \
    auto actual_value = (actual); \
    auto expected_value = (expected); \
    if(actual_value != expected_value) { \
        std::cerr << "CHECK_EQ failed at line " << __LINE__ << ": " \
                  << #actual << "=" << actual_value << " expected " << expected_value << "\n"; \
        g_failed = 1; \
    } \
} while(0)

#define CHECK_FLOAT(actual, expected) do { \
    float actual_value = (actual); \
    float expected_value = (expected); \
    if(std::fabs(actual_value - expected_value) > 0.0001f) { \
        std::cerr << "CHECK_FLOAT failed at line " << __LINE__ << ": " \
                  << #actual << "=" << actual_value << " expected " << expected_value << "\n"; \
        g_failed = 1; \
    } \
} while(0)

static void test_menu_enters_folder_and_clamps_selected_float()
{
    MenuCore menu;
    float kp = 2.5f;

    MenuCore_Init(&menu, "Menu");
    MenuCoreNode *pid = MenuCore_AddFolder(&menu, MenuCore_Root(&menu), "PID");
    MenuCoreNode *kp_node = MenuCore_AddFloat(&menu, pid, "Kp", &kp, 0.1f, 0.0f, 3.0f);

    CHECK_TRUE(pid != nullptr);
    CHECK_TRUE(kp_node != nullptr);
    CHECK_EQ(MenuCore_Current(&menu), pid);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_ENTER);
    CHECK_EQ(MenuCore_Current(&menu), kp_node);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_ENTER);
    CHECK_TRUE(MenuCore_Current(&menu)->selected);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_UP);
    CHECK_FLOAT(kp, 2.6f);

    for(int i = 0; i < 10; i++) {
        MenuCore_HandleAction(&menu, MENU_CORE_ACTION_UP);
    }
    CHECK_FLOAT(kp, 3.0f);

    for(int i = 0; i < 40; i++) {
        MenuCore_HandleAction(&menu, MENU_CORE_ACTION_DOWN);
    }
    CHECK_FLOAT(kp, 0.0f);
}

static void test_menu_wraps_siblings_and_quits_to_parent()
{
    MenuCore menu;
    int speed = 270;
    int land_speed = 450;

    MenuCore_Init(&menu, "Menu");
    MenuCoreNode *speed_folder = MenuCore_AddFolder(&menu, MenuCore_Root(&menu), "Speed");
    MenuCoreNode *base = MenuCore_AddInt(&menu, speed_folder, "base", &speed, 10.0f, 0.0f, 1000.0f);
    MenuCoreNode *ring = MenuCore_AddInt(&menu, speed_folder, "ring", &land_speed, 10.0f, 0.0f, 1000.0f);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_ENTER);
    CHECK_EQ(MenuCore_Current(&menu), base);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_DOWN);
    CHECK_EQ(MenuCore_Current(&menu), ring);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_DOWN);
    CHECK_EQ(MenuCore_Current(&menu), base);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_BACK);
    CHECK_EQ(MenuCore_Current(&menu), speed_folder);
}

static void test_menu_registers_multiple_parameter_groups()
{
    MenuCore menu;
    float kp = 2.5f;
    int lookahead = 47;

    MenuCore_Init(&menu, "Menu");
    MenuCoreNode *pid = MenuCore_AddFolder(&menu, MenuCore_Root(&menu), "PID");
    MenuCore_AddFloat(&menu, pid, "Kp", &kp, 0.1f, 0.0f, 20.0f);
    MenuCoreNode *camera = MenuCore_AddFolder(&menu, MenuCore_Root(&menu), "Camera");
    MenuCore_AddInt(&menu, camera, "w", &lookahead, 1.0f, 20.0f, 80.0f);

    CHECK_EQ(MenuCore_Current(&menu), pid);
    CHECK_EQ(MenuCore_Current(&menu)->next, camera);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_DOWN);
    CHECK_EQ(MenuCore_Current(&menu), camera);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_ENTER);
    CHECK_EQ(MenuCore_Current(&menu)->name, std::string("w"));
}

static void test_tuning_menu_registers_real_tuning_groups()
{
    MenuCore menu;
    TuningMenuBindings bindings{};

    volatile float kp = 2.6f;
    volatile float kp2 = 0.0f;
    volatile float kd = 0.3f;
    volatile float gkd = 0.015f;
    volatile int camera_w = 47;
    volatile int ring_w = 40;
    volatile int32_t base_speed = 270;
    volatile int ring_speed = 450;

    bindings.servo_kp = &kp;
    bindings.servo_kp2 = &kp2;
    bindings.servo_kd = &kd;
    bindings.servo_gkd = &gkd;
    bindings.camera_w = &camera_w;
    bindings.land_w = &ring_w;
    bindings.set_speed = &base_speed;
    bindings.land_speed = &ring_speed;

    MenuCore_Init(&menu, "Menu");
    CHECK_TRUE(TuningMenu_Register(&menu, &bindings) != 0);

    CHECK_EQ(MenuCore_Current(&menu)->name, std::string("PID"));
    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_ENTER);
    CHECK_EQ(MenuCore_Current(&menu)->name, std::string("Kp"));

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_ENTER);
    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_UP);
    CHECK_FLOAT(kp, 2.7f);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_BACK);
    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_BACK);
    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_DOWN);
    CHECK_EQ(MenuCore_Current(&menu)->name, std::string("Camera"));

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_DOWN);
    CHECK_EQ(MenuCore_Current(&menu)->name, std::string("Speed"));
    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_ENTER);
    CHECK_EQ(MenuCore_Current(&menu)->name, std::string("set"));
    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_ENTER);
    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_UP);
    CHECK_EQ(base_speed, 280);
}

static void test_menu_renderer_marks_current_and_selected_value()
{
    MenuCore menu;
    float kp = 2.6f;

    MenuCore_Init(&menu, "Menu");
    MenuCoreNode *pid = MenuCore_AddFolder(&menu, MenuCore_Root(&menu), "PID");
    MenuCore_AddFloat(&menu, pid, "Kp", &kp, 0.1f, 0.0f, 20.0f);

    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_ENTER);
    MenuCore_HandleAction(&menu, MENU_CORE_ACTION_ENTER);

    MenuRenderPage page{};
    CHECK_TRUE(MenuRender_BuildPage(&menu, &page) != 0);

    CHECK_EQ(page.line_count, 2);
    CHECK_EQ(std::string(page.lines[0]), std::string("Menu/PID"));
    CHECK_EQ(std::string(page.lines[1]), std::string("-> Kp        <2.600>"));
}

int main()
{
    test_menu_enters_folder_and_clamps_selected_float();
    test_menu_wraps_siblings_and_quits_to_parent();
    test_menu_registers_multiple_parameter_groups();
    test_tuning_menu_registers_real_tuning_groups();
    test_menu_renderer_marks_current_and_selected_value();

    if(g_failed) {
        return EXIT_FAILURE;
    }

    std::cout << "menu_core tests passed\n";
    return EXIT_SUCCESS;
}
