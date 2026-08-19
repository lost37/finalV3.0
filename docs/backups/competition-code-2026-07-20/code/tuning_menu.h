#ifndef CODE_TUNING_MENU_H_
#define CODE_TUNING_MENU_H_

#include <stdint.h>

#include "menu_core.h"

typedef struct
{
    volatile float *servo_kp;
    volatile float *servo_kp2;
    volatile float *servo_kd;
    volatile float *servo_gkd;
    volatile int *camera_w;
    volatile int *land_w;
    volatile int32_t *set_speed;
    volatile int *land_speed;
    volatile float *ack_dif_full_scale;
    volatile int *redblock_detection_enable;
    volatile int *redblock_visual_return_mode;
} TuningMenuBindings;

uint8_t TuningMenu_Register(MenuCore *menu, TuningMenuBindings *bindings);

#endif /* CODE_TUNING_MENU_H_ */
