/*
 * cross.h
 * 十字路口检测模块
 */

#ifndef CODE_CROSS_H_
#define CODE_CROSS_H_

#include "zf_common_headfile.h"

extern uint8 left_up_flag, left_down_flag;
extern uint8 right_up_flag, right_down_flag;
extern uint8 left_up, left_down;
extern uint8 right_up, right_down;
extern uint8 cross_flag;
extern uint8 xie_cross_flag;
extern uint8 xie_cross_time;

void search_anglepoint(void);
void Cross_judge(void);

#endif /* CODE_CROSS_H_ */
