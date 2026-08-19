/*
 * redblock.h
 * 红色色块检测模块
 */

#ifndef CODE_REDBLOCK_H_
#define CODE_REDBLOCK_H_

#include "zf_common_typedef.h"

typedef enum
{
    RB_DEC_IDLE = 0,
    RB_DEC_CONFIRMING = 1,
    RB_DEC_LOW_SPEED_SETTLE = 3,
    RB_DEC_MODEL_RECOGNIZING = 4,
    RB_DEC_MOTION_ACTIVE = 5,
} RedBlockState;

#define RB_IDLE                  RB_DEC_IDLE
#define RB_CONFIRMING            RB_DEC_CONFIRMING
#define RB_SLOWDOWN              RB_DEC_LOW_SPEED_SETTLE
#define RB_PAUSED                RB_DEC_MODEL_RECOGNIZING
#define RB_MODEL_WAIT            RB_DEC_MODEL_RECOGNIZING
#define RB_CONFIRMED             RB_DEC_MOTION_ACTIVE
#define RB_BYPASS                RB_DEC_MOTION_ACTIVE

typedef enum
{
    RB_BYPASS_MODE_NONE = 0,
    RB_BYPASS_MODE_STRAIGHT,
    RB_BYPASS_MODE_RIGHT,
    RB_BYPASS_MODE_LEFT,
} RedBlockBypassMode;

typedef enum
{
    RB_VISUAL_RETURN_LEGACY_DIRECT = 0,
    RB_VISUAL_RETURN_SMOOTH_BLEND = 1,
} RedBlockVisualReturnMode;

typedef enum
{
    RB_BYPASS_PHASE_IDLE = 0,
    RB_BYPASS_PHASE_SEEK_BOUNDARY,
    RB_BYPASS_PHASE_BOUNDARY_HOLD,
    RB_BYPASS_PHASE_BLEND_BACK,
    RB_BYPASS_PHASE_RECOVER,
    RB_BYPASS_PHASE_POST_PASS_HOLD,
} RedBlockBypassPhase;

// 兼容旧名称，phase 数值和外部日志保持不变。
#define RB_BYPASS_PHASE_APPROACH  RB_BYPASS_PHASE_SEEK_BOUNDARY
#define RB_BYPASS_PHASE_COMMIT    RB_BYPASS_PHASE_BOUNDARY_HOLD
#define RB_BYPASS_PHASE_EXIT_HOLD RB_BYPASS_PHASE_BLEND_BACK

extern uint8 redblock_flag;
extern uint8 redblock_state_flag;
extern uint8 redblock_confirm_count;
extern float redblock_area;
extern int16 redblock_x;
extern int16 redblock_y;
extern int16 redblock_center_x;
extern int16 redblock_center_y;
extern uint16 redblock_width;
extern uint16 redblock_height;
extern uint8 redblock_bypass_mode_flag;
extern uint8 redblock_bypass_phase_flag;
extern uint8 redblock_bypass_active_flag;
extern volatile int redblock_detection_enable;
extern volatile int redblock_visual_return_mode;
extern volatile int redblock_cross_fill_enable;
extern volatile int32_t redblock_bypass_speed_cmd;
extern volatile int32_t redblock_slowdown_speed_cmd;

void RedBlock_Detect(void);
void RedBlock_Update(void);
void RedBlock_ReportClassificationSequenceIfStopped(void);
void RedBlock_UpdatePerception(void);
void RedBlock_UpdateDecision(void);
RedBlockState RedBlock_GetState(void);
void RedBlock_RequestPause(void);
void RedBlock_ReleasePause(void);
void RedBlock_ResetState(void);
void RedBlock_OnModelStarted(void);
void RedBlock_OnModelConfirmed(void);
uint8 RedBlock_IsModelPending(void);
uint8 RedBlock_IsSlowdownActive(void);
int32_t RedBlock_GetSlowdownSpeedCmd(void);
void RedBlock_StartBypass(void);
void RedBlock_StartBypassMode(RedBlockBypassMode mode);
void RedBlock_FinishBypass(void);
uint8 RedBlock_IsBypassActive(void);
uint8 RedBlock_IsActive(void);
uint8 RedBlock_ShouldSuppressOtherElements(void);
uint8 RedBlock_ShouldUseLowSpeedHold(void);
int32_t RedBlock_GetMotionSpeedCmd(void);
float RedBlock_GetMotionDifSpeed(void);
uint8 RedBlock_ShouldIgnoreBoundaryStop(void);
uint8 RedBlock_IsElementExclusive(void);
RedBlockBypassMode RedBlock_GetBypassMode(void);
RedBlockBypassPhase RedBlock_GetBypassPhase(void);
uint8 RedBlock_ApplyBypass(void);
uint8 RedBlock_ApplyVisualCenterline(void);
void RedBlock_LogVisualControl(float current_err);
int32_t RedBlock_GetBypassSpeedCmd(void);

uint8 RedBlock_GetRect(int16 *x, int16 *y, uint16 *width, uint16 *height);
uint8 RedBlock_GetSearchRect(int16 *x, int16 *y, uint16 *width, uint16 *height);
uint8 RedBlock_GetModelRoi(int16 *x, int16 *y, uint16 *width, uint16 *height);
uint8 RedBlock_PrepareModelInput(uint8 *output_bgr, uint16 output_width, uint16 output_height);

#endif /* CODE_REDBLOCK_H_ */
