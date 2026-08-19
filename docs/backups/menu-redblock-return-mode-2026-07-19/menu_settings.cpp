#include "menu_settings.h"

#include "camera.h"
#include "circle.h"
#include "control.h"
#include "motor.h"
#include "redblock.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

namespace
{
    // 调参：菜单参数保存文件。程序重启后会读取这里的值作为新的运行初值。
    const char *MENU_SETTINGS_PATH = "/home/root/smartcar_menu.cfg";

    void ApplySetting(const char *key, float value)
    {
        if(strcmp(key, "servo_kp") == 0)
        {
            servo_pid_kp = value;
        }
        else if(strcmp(key, "servo_kp2") == 0)
        {
            servo_pid_kp2 = value;
        }
        else if(strcmp(key, "servo_kd") == 0)
        {
            servo_pid_kd = value;
        }
        else if(strcmp(key, "servo_gkd") == 0)
        {
            servo_pid_gkd = value;
        }
        else if(strcmp(key, "camera_w") == 0)
        {
            w = (int)value;
        }
        else if(strcmp(key, "land_w") == 0)
        {
            land_w = (int)value;
        }
        else if(strcmp(key, "set_speed") == 0)
        {
            set_speed = (int32_t)value;
        }
        else if(strcmp(key, "land_speed") == 0)
        {
            land_s = (int)value;
        }
        else if(strcmp(key, "ack_dif_full_scale") == 0)
        {
            ack_dif_full_scale = value;
        }
        else if(strcmp(key, "motor_l_kp") == 0)
        {
            motor_l_kp = value;
        }
        else if(strcmp(key, "motor_l_ki") == 0)
        {
            motor_l_ki = value;
        }
        else if(strcmp(key, "motor_l_filter_a") == 0)
        {
            motor_l_filter_a = value;
        }
        else if(strcmp(key, "motor_r_kp") == 0)
        {
            motor_r_kp = value;
        }
        else if(strcmp(key, "motor_r_ki") == 0)
        {
            motor_r_ki = value;
        }
        else if(strcmp(key, "motor_r_filter_a") == 0)
        {
            motor_r_filter_a = value;
        }
        else if(strcmp(key, "redblock_detection_enable") == 0)
        {
            redblock_detection_enable = (value != 0.0f) ? 1 : 0;
        }
    }
}

void MenuSettings_Load(void)
{
    FILE *fp = fopen(MENU_SETTINGS_PATH, "r");
    if(fp == nullptr)
    {
        return;
    }

    char key[32] = {0};
    float value = 0.0f;
    while(fscanf(fp, "%31[^=]=%f\n", key, &value) == 2)
    {
        ApplySetting(key, value);
        key[0] = '\0';
    }

    fclose(fp);
}

uint8_t MenuSettings_Save(void)
{
    FILE *fp = fopen(MENU_SETTINGS_PATH, "w");
    if(fp == nullptr)
    {
        printf("MenuSettings_Save failed: %s\n", MENU_SETTINGS_PATH);
        return 0;
    }

    fprintf(fp, "servo_kp=%.6f\n", (double)servo_pid_kp);
    fprintf(fp, "servo_kp2=%.6f\n", (double)servo_pid_kp2);
    fprintf(fp, "servo_kd=%.6f\n", (double)servo_pid_kd);
    fprintf(fp, "servo_gkd=%.6f\n", (double)servo_pid_gkd);
    fprintf(fp, "camera_w=%d\n", (int)w);
    fprintf(fp, "land_w=%d\n", (int)land_w);
    fprintf(fp, "set_speed=%ld\n", (long)set_speed);
    fprintf(fp, "land_speed=%d\n", (int)land_s);
    fprintf(fp, "ack_dif_full_scale=%.6f\n", (double)ack_dif_full_scale);
    fprintf(fp, "motor_l_kp=%.6f\n", (double)motor_l_kp);
    fprintf(fp, "motor_l_ki=%.6f\n", (double)motor_l_ki);
    fprintf(fp, "motor_l_filter_a=%.6f\n", (double)motor_l_filter_a);
    fprintf(fp, "motor_r_kp=%.6f\n", (double)motor_r_kp);
    fprintf(fp, "motor_r_ki=%.6f\n", (double)motor_r_ki);
    fprintf(fp, "motor_r_filter_a=%.6f\n", (double)motor_r_filter_a);
    fprintf(fp, "redblock_detection_enable=%d\n", redblock_detection_enable != 0 ? 1 : 0);

    fclose(fp);
    return 1;
}
