/*
 * zebra.h
 * 斑马线检测模块
 */

#ifndef CODE_ZEBRA_H_
#define CODE_ZEBRA_H_

#include "zf_common_headfile.h"

extern volatile int zebra_mode;
extern uint8 zebra_flag;

void Zebra_Detect(void);
void ResetZebraDetection(void);
void Zebra_Detect_delay(void);

#endif /* CODE_ZEBRA_H_ */
