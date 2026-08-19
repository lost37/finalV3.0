#include "tuning_menu.h"

static float *MutableFloat(volatile float *value)
{
    return const_cast<float *>(value);
}

static int *MutableInt(volatile int *value)
{
    return const_cast<int *>(value);
}

static int32_t *MutableInt32(volatile int32_t *value)
{
    return const_cast<int32_t *>(value);
}

uint8_t TuningMenu_Register(MenuCore *menu, TuningMenuBindings *bindings)
{
    if(menu == nullptr || bindings == nullptr)
    {
        return 0;
    }

    MenuCoreNode *root = MenuCore_Root(menu);
    if(root == nullptr)
    {
        return 0;
    }

    MenuCoreNode *pid = MenuCore_AddFolder(menu, root, "PID");
    MenuCoreNode *camera = MenuCore_AddFolder(menu, root, "Camera");
    MenuCoreNode *speed = MenuCore_AddFolder(menu, root, "Speed");
    MenuCoreNode *ack = MenuCore_AddFolder(menu, root, "Ack");
    MenuCoreNode *redblock = MenuCore_AddFolder(menu, root, "RedBlk");

    if(pid == nullptr || camera == nullptr || speed == nullptr || ack == nullptr || redblock == nullptr)
    {
        return 0;
    }

    MenuCore_AddFloat(menu, pid, "Kp", MutableFloat(bindings->servo_kp), 0.1f, 0.0f, 20.0f);
    MenuCore_AddFloat(menu, pid, "Kp2", MutableFloat(bindings->servo_kp2), 0.01f, 0.0f, 1.0f);
    MenuCore_AddFloat(menu, pid, "Kd", MutableFloat(bindings->servo_kd), 0.05f, 0.0f, 10.0f);
    MenuCore_AddFloat(menu, pid, "GKD", MutableFloat(bindings->servo_gkd), 0.005f, 0.0f, 1.0f);

    MenuCore_AddInt(menu, camera, "w", MutableInt(bindings->camera_w), 1.0f, 20.0f, 80.0f);
    MenuCore_AddInt(menu, camera, "land_w", MutableInt(bindings->land_w), 1.0f, 20.0f, 80.0f);

    MenuCore_AddInt32(menu, speed, "set", MutableInt32(bindings->set_speed), 10.0f, 0.0f, 1000.0f);
    MenuCore_AddInt(menu, speed, "land", MutableInt(bindings->land_speed), 10.0f, 0.0f, 1000.0f);

    MenuCore_AddFloat(menu, ack, "dif_scale", MutableFloat(bindings->ack_dif_full_scale), 1.0f, 20.0f, 200.0f);
    MenuCore_AddInt(menu, redblock, "detect", MutableInt(bindings->redblock_detection_enable), 1.0f, 0.0f, 1.0f);
    MenuCore_AddInt(menu, redblock, "return", MutableInt(bindings->redblock_visual_return_mode), 1.0f, 0.0f, 1.0f);

    return 1;
}
