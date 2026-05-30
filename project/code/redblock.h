/*
 * redblock.h
 * 红色色块检测模块
 */

#ifndef CODE_REDBLOCK_H_
#define CODE_REDBLOCK_H_

#include "zf_common_typedef.h"

typedef enum
{
    RB_IDLE = 0,
    RB_CONFIRMING,
    RB_PAUSED,
    RB_MODEL_WAIT,
    RB_CONFIRMED,
    RB_BYPASS,
} RedBlockState;

typedef enum
{
    RB_BYPASS_MODE_NONE = 0,
    RB_BYPASS_MODE_STRAIGHT,
    RB_BYPASS_MODE_RIGHT,
    RB_BYPASS_MODE_LEFT,
} RedBlockBypassMode;

typedef enum
{
    RB_BYPASS_PHASE_IDLE = 0,
    RB_BYPASS_PHASE_APPROACH,
    RB_BYPASS_PHASE_COMMIT,
    RB_BYPASS_PHASE_EXIT_HOLD,
    RB_BYPASS_PHASE_RECOVER,
} RedBlockBypassPhase;

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

void RedBlock_Detect(void);
void RedBlock_Update(void);
RedBlockState RedBlock_GetState(void);
void RedBlock_RequestPause(void);
void RedBlock_ReleasePause(void);
void RedBlock_ResetState(void);
void RedBlock_OnModelStarted(void);
void RedBlock_OnModelConfirmed(void);
void RedBlock_StartBypass(void);
void RedBlock_StartBypassMode(RedBlockBypassMode mode);
void RedBlock_FinishBypass(void);
uint8 RedBlock_IsBypassActive(void);
uint8 RedBlock_ShouldIgnoreBoundaryStop(void);
uint8 RedBlock_IsElementExclusive(void);
RedBlockBypassMode RedBlock_GetBypassMode(void);
RedBlockBypassPhase RedBlock_GetBypassPhase(void);
uint8 RedBlock_ApplyBypass(void);

uint8 RedBlock_GetRect(int16 *x, int16 *y, uint16 *width, uint16 *height);
uint8 RedBlock_GetSearchRect(int16 *x, int16 *y, uint16 *width, uint16 *height);
uint8 RedBlock_GetModelRoi(int16 *x, int16 *y, uint16 *width, uint16 *height);
uint8 RedBlock_PrepareModelInput(uint8 *output_bgr, uint16 output_width, uint16 output_height);

#endif /* CODE_REDBLOCK_H_ */
