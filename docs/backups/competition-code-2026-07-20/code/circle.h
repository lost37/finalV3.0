/*
 * circle.h
 * 环岛检测模块
 */

#ifndef CODE_CIRCLE_H_
#define CODE_CIRCLE_H_

#include "zf_common_headfile.h"

extern uint8 lianxu, dizeng;
extern uint8 l_land_flag;
extern uint8 r_land_flag;
extern int l_land_time;
extern uint8 land_line;
extern volatile int land_data;
extern volatile int land_w;
extern uint8 imu_ring_test_active;
extern volatile uint8 imu_ring_exit_beeped;
extern volatile uint8 imu_ring_exit_counter;
extern int l_land_num;

#define AY_EXIT_THRESHOLD 2500
#define EXIT_FRAME_COUNT 2

void l_land_judge(void);
void l_xie_land_judge(void);
void r_land_judge(void);

#endif /* CODE_CIRCLE_H_ */
