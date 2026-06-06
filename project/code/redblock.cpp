#include "redblock.h"

#include <stdio.h>
#include <string.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "camera.h"
#include "motor.h"
#include "cross.h"
#include "gyroscope.h"
#include "ncnn_infer.h"
#include "zf_device_uvc.h"

extern uint8 redblock_pause_flag;
extern uint8 model_request_flag;
extern uint8 model_running_flag;
extern int time1;
extern int timestop;
extern uint8 pwm0_flag;
extern volatile int land_s;
extern volatile int32_t set_speed;
extern volatile int32_t encoder_acc_left;
extern volatile int32_t encoder_acc_right;
extern volatile int32_t encoder_acc_avg;

uint8 redblock_flag = 0;
uint8 redblock_state_flag = RB_IDLE;
uint8 redblock_confirm_count = 0;
float redblock_area = 0;
int16 redblock_x = -1;
int16 redblock_y = -1;
int16 redblock_center_x = -1;
int16 redblock_center_y = -1;
uint16 redblock_width = 0;
uint16 redblock_height = 0;
uint8 redblock_bypass_mode_flag = RB_BYPASS_MODE_NONE;
uint8 redblock_bypass_phase_flag = RB_BYPASS_PHASE_IDLE;
uint8 redblock_bypass_active_flag = 0;
uint8 redblock_brake_ticks = 0;
uint8 redblock_action_phase_flag = RB_ACT_IDLE;
volatile float redblock_bypass_dif_speed = 0.0f;
volatile int32_t redblock_bypass_speed_cmd = 0;
volatile int32_t redblock_slowdown_speed_cmd = 120;  //红块减速速度

namespace
{
    // 调参：红块检测间隔帧数。越小响应越快但更吃算力，越大更稳但触发更慢。
    constexpr uint8 REDBLOCK_DETECT_INTERVAL = 2;

    // 调参：连续命中多少次才确认红块。1 最快，2~3 更抗误检。
    constexpr uint8 REDBLOCK_CONFIRM_REQUIRED = 1;

    // 调参：红色轮廓最小面积，增大可过滤远处小红点，过大可能漏检小红块。
    constexpr int RED_MIN_AREA = 80;

    // 调参：形态学开闭运算核大小。增大更能去噪，过大可能吞掉小目标。
    constexpr int RED_KERNEL_SIZE = 5;

    // 调参：红块外接矩形宽高比范围，用于排除过细或过扁的红色区域。
    constexpr float RED_ASPECT_MIN = 0.7f;
    constexpr float RED_ASPECT_MAX = 5.0f;

    // 调参：轮廓填充率下限。越大越严格，可排除稀疏噪声。
    constexpr float RED_FILL_MIN = 0.50f;

    // 调参：模型 ROI 最小边长。增大能保留更多上下文，减小能让目标占比更大。
    constexpr int RED_MODEL_ROI_MIN_SIDE = 75;
    // 调参：模型 ROI 边缘留白，防止红块贴边裁切。
    constexpr int RED_MODEL_ROI_EDGE_PADDING = 2;
    // 调参：红块底部留白比例。增大可保留红块下缘，过大可能压缩上方目标。
    constexpr float RED_MODEL_ROI_BOTTOM_MARGIN_RATIO = 0.08f;
    // 调参：按红块高度估计上方主体高度。目标更高时增大，背景太多时减小。
    constexpr float RED_MODEL_ROI_OBJECT_HEIGHT_RATIO = 1.05f;
    // 调参：ROI 中心上移比例。增大让上方主体占比更大，过大可能裁掉红块。
    constexpr float RED_MODEL_ROI_CENTER_UP_RATIO = 0.10f;

    // 调参：红块检测四边形区域，坐标为 320x240 图像坐标系。
    // 顺时针组织四个点，缩小区域可减少误检，放大区域可提前发现红块。
    constexpr int RED_SEARCH_POINT_LEFT_TOP_X = 50;
    constexpr int RED_SEARCH_POINT_LEFT_TOP_Y = 0;
    constexpr int RED_SEARCH_POINT_RIGHT_TOP_X = 265;
    constexpr int RED_SEARCH_POINT_RIGHT_TOP_Y = 0;
    constexpr int RED_SEARCH_POINT_RIGHT_BOTTOM_X = 265;
    constexpr int RED_SEARCH_POINT_RIGHT_BOTTOM_Y = 195;
    constexpr int RED_SEARCH_POINT_LEFT_BOTTOM_X = 50;
    constexpr int RED_SEARCH_POINT_LEFT_BOTTOM_Y = 195;

    // 调参：固定反冲周期。越大减速越强，过大可能反拖或抖车。
    constexpr uint8 REDBLOCK_BRAKE_TICKS = 8;
    // 调参：反冲结束后低速保持几帧再开始模型识别。当前需求定为 5 帧。
    constexpr uint8 REDBLOCK_LOW_SPEED_SETTLE_FRAMES = 5;
    // 调参：模型 invalid 允许重试几帧；红块当前帧丢失时不重试，直接左绕。
    constexpr uint8 REDBLOCK_MODEL_INVALID_RETRY_FRAMES = 5;
    // 调参：红块识别期间低速保持速度。
    constexpr int32_t REDBLOCK_SLOWDOWN_SPEED_CMD = 130;
    // 调参：绕行动作 APPROACH 阶段保持帧数，增大可让切出前姿态更稳定。
    constexpr uint8 REDBLOCK_BYPASS_APPROACH_FRAMES = 6;
    // 调参：绕过后继续保持切出方向的帧数，增大可离红块更远。
    constexpr uint8 REDBLOCK_BYPASS_EXIT_HOLD_FRAMES = 8;
    // 调参：恢复阶段保持帧数，增大回线更慢更稳，减小恢复更快。
    constexpr uint8 REDBLOCK_BYPASS_RECOVER_FRAMES = 6;
    // 调参：绕行时允许连续丢失红块的帧数，增大更宽容，减小更敏感。
    constexpr uint8 REDBLOCK_BYPASS_LOST_LIMIT = 4;
    // 调参：投影到赛道图后的红块最小宽高，过滤过小投影。
    constexpr int REDBLOCK_BYPASS_MIN_BLOCK_WIDTH = 8;
    constexpr int REDBLOCK_BYPASS_MIN_BLOCK_HEIGHT = 8;
    // 调参：绕行补线影响的上下行扩展范围，增大可让补线覆盖更长距离。
    constexpr int REDBLOCK_BYPASS_ROW_EXTEND_UP = 6;
    constexpr int REDBLOCK_BYPASS_ROW_EXTEND_DOWN = 12;
    // 调参：绕行补线内外侧余量，影响车身离红块的横向距离。
    constexpr int REDBLOCK_BYPASS_INNER_MARGIN = 6;
    constexpr int REDBLOCK_BYPASS_OUTER_MARGIN = 3;
    // 调参：APPROACH 阶段提前横移量，增大可更早开始绕。
    constexpr int REDBLOCK_BYPASS_APPROACH_SHIFT = 8;
    // 调参：绕行切出目标角度，增大绕行幅度更大。
    constexpr float REDBLOCK_TURN_OUT_ANGLE = 35.0f;
    // 调参：切回角度容差，增大更容易进入直行段，减小姿态更准。
    constexpr float REDBLOCK_TURN_BACK_TOLERANCE = 4.0f;
    // 调参：绕行动作两段通过距离，增大通过更远，减小动作更紧凑。
    constexpr int32_t REDBLOCK_PASS1_DISTANCE = 300;
    constexpr int32_t REDBLOCK_PASS2_DISTANCE = 180;
    // 调参：绕行固定转向差速命令，增大转弯更急。
    constexpr float REDBLOCK_TURN_CMD = 120.0f;
    // 调参：红块绕行专用速度。不要直接使用 land_s，避免从低速识别阶段突然跳到环岛速度导致前冲。
    constexpr int32_t REDBLOCK_BYPASS_SPEED_CMD = 220;
    // 调参：恢复赛道需要连续满足的帧数，增大更稳，减小更快退出绕行。
    constexpr uint8 REDBLOCK_RECOVER_STABLE_REQUIRED = 3;
    // 调参：恢复判定的左右有效边界数量下限。
    constexpr int REDBLOCK_RECOVER_EFFECT_THRESHOLD = 45;
    // 调参：恢复判定的中线误差阈值。
    constexpr float REDBLOCK_RECOVER_ERR_THRESHOLD = 8.0f;
    // 调参：恢复判定的前方可视距离阈值。实测 far 经常只有 8~35，阈值设小避免绕行后长期无法退出。
    constexpr float REDBLOCK_RECOVER_DISTANCE_THRESHOLD = 5.0f;
    constexpr int MODEL_CLASS_SUPPLIERS = 0;
    constexpr int MODEL_CLASS_VEHICLE = 1;
    constexpr int MODEL_CLASS_WEAPON = 2;
    constexpr uint8 MODEL_CLASS_COUNT = 3;
    // 调参：有效模型投票帧数。当前只有 3 个粗类，5 帧投票不会平票。
    constexpr uint8 MODEL_VOTE_REQUIRED = 5;

    RedBlockState redblock_state = RB_IDLE;
    uint8 redblock_detect_frame_counter = 0;
    RedBlockBypassMode redblock_bypass_mode = RB_BYPASS_MODE_NONE;
    RedBlockBypassPhase redblock_bypass_phase = RB_BYPASS_PHASE_IDLE;
    RedBlockActionPhase redblock_action_phase = RB_ACT_IDLE;
    uint8 redblock_bypass_phase_counter = 0;
    uint8 redblock_bypass_lost_counter = 0;
    uint8 redblock_bypass_detect_valid = 0;
    uint8 redblock_bypass_last_row_top = 0;
    uint8 redblock_bypass_last_row_bottom = 0;
    uint8 redblock_bypass_last_col_left = 0;
    uint8 redblock_bypass_last_col_right = 0;
    float redblock_start_yaw = 0.0f;
    int32_t redblock_phase_start_encoder_avg = 0;
    uint8 redblock_recover_ready_count = 0;
    uint8 redblock_recover_diag_div = 0;
    uint8 redblock_slowdown_frame_count = 0;
    uint8 redblock_low_speed_settle_count = 0;
    uint8 redblock_model_invalid_count = 0;
    uint8 redblock_model_vote_valid_count = 0;
    uint8 redblock_model_vote_count[MODEL_CLASS_COUNT] = {0};

    int ClampInt(int value, int min_value, int max_value)
    {
        if(value < min_value)
        {
            return min_value;
        }
        if(value > max_value)
        {
            return max_value;
        }
        return value;
    }

    uint8 ClampRowIndex(int value)
    {
        return (uint8)ClampInt(value, 0, Cut_ROW - 1);
    }

    uint8 ClampColIndex(int value)
    {
        return (uint8)ClampInt(value, SEARCH_MIN, SEARCH_MAX);
    }

    uint8 RedBlock_PrepareModelInputFromCurrentRoi(uint8 *output_bgr, uint16 output_width, uint16 output_height)
    {
        const uint8_t *rgb_frame = nullptr;
        int frame_width = 0;
        int frame_height = 0;
        int frame_step = 0;
        int16 roi_x = 0;
        int16 roi_y = 0;
        uint16 roi_width = 0;
        uint16 roi_height = 0;

        if(output_bgr == nullptr || output_width == 0 || output_height == 0)
        {
            return 0;
        }

        if(RedBlock_GetModelRoi(&roi_x, &roi_y, &roi_width, &roi_height) == 0)
        {
            return 0;
        }

        if(get_rgb_frame_info(&rgb_frame, &frame_width, &frame_height, &frame_step) != 0)
        {
            return 0;
        }

        if(rgb_frame == nullptr || frame_width <= 0 || frame_height <= 0 || frame_step <= 0)
        {
            return 0;
        }

        cv::Mat frame_bgr(frame_height, frame_width, CV_8UC3, const_cast<uint8_t *>(rgb_frame), frame_step);
        cv::Rect roi_rect(roi_x, roi_y, roi_width, roi_height);
        cv::Mat roi = frame_bgr(roi_rect);
        cv::Mat resized;
        cv::resize(roi, resized, cv::Size(output_width, output_height), 0, 0, cv::INTER_LINEAR);

        const size_t output_size = static_cast<size_t>(output_width) * static_cast<size_t>(output_height) * 3;
        memcpy(output_bgr, resized.data, output_size);
        return 1;
    }

    void RedBlock_SetState(RedBlockState state)
    {
        if(redblock_state != state)
        {
            printf("[RB_DEC] %u -> %u\n", (unsigned)redblock_state, (unsigned)state);
        }
        redblock_state = state;
        redblock_state_flag = static_cast<uint8>(state);
    }

    void RedBlock_ResetModelVoting(void)
    {
        uint8 i = 0;
        redblock_model_invalid_count = 0;
        redblock_model_vote_valid_count = 0;
        for(i = 0; i < MODEL_CLASS_COUNT; i++)
        {
            redblock_model_vote_count[i] = 0;
        }
    }

    void RedBlock_AddModelVote(int coarse_index)
    {
        if(coarse_index >= 0 && coarse_index < MODEL_CLASS_COUNT)
        {
            redblock_model_vote_count[coarse_index]++;
            redblock_model_vote_valid_count++;
        }
    }

    int RedBlock_GetBestModelVote(void)
    {
        int best_class = -1;
        uint8 best_count = 0;
        uint8 tie = 0;
        uint8 i = 0;

        for(i = 0; i < MODEL_CLASS_COUNT; i++)
        {
            if(redblock_model_vote_count[i] > best_count)
            {
                best_count = redblock_model_vote_count[i];
                best_class = i;
                tie = 0;
            }
            else if(redblock_model_vote_count[i] == best_count && redblock_model_vote_count[i] > 0)
            {
                tie = 1;
            }
        }

        if(best_count == 0 || tie != 0)
        {
            return -1;
        }
        return best_class;
    }

    const char *RedBlock_ModelClassLabel(int coarse_index)
    {
        switch(coarse_index)
        {
            case MODEL_CLASS_SUPPLIERS:
                return "suppliers";
            case MODEL_CLASS_VEHICLE:
                return "vehicle";
            case MODEL_CLASS_WEAPON:
                return "weapon";
            default:
                return "unknown";
        }
    }

    void RedBlock_SetBypassPhase(RedBlockBypassPhase phase)
    {
        redblock_bypass_phase = phase;
        redblock_bypass_phase_flag = static_cast<uint8>(phase);
        redblock_bypass_phase_counter = 0;
    }

    void RedBlock_SetActionPhase(RedBlockActionPhase phase)
    {
        redblock_action_phase = phase;
        redblock_action_phase_flag = static_cast<uint8>(phase);
    }

    void RedBlock_ClearDetectionResult(void)
    {
        redblock_flag = 0;
        redblock_area = 0;
        redblock_x = -1;
        redblock_y = -1;
        redblock_center_x = -1;
        redblock_center_y = -1;
        redblock_width = 0;
        redblock_height = 0;
    }

    void RedBlock_ResetBypassContext(void)
    {
        redblock_bypass_mode = RB_BYPASS_MODE_NONE;
        redblock_bypass_mode_flag = RB_BYPASS_MODE_NONE;
        redblock_bypass_active_flag = 0;
        redblock_bypass_detect_valid = 0;
        redblock_bypass_lost_counter = 0;
        redblock_bypass_last_row_top = 0;
        redblock_bypass_last_row_bottom = 0;
        redblock_bypass_last_col_left = 0;
        redblock_bypass_last_col_right = 0;
        redblock_start_yaw = 0.0f;
        redblock_phase_start_encoder_avg = 0;
        redblock_recover_ready_count = 0;
        redblock_recover_diag_div = 0;
        redblock_slowdown_frame_count = 0;
        redblock_low_speed_settle_count = 0;
        redblock_bypass_dif_speed = 0.0f;
        redblock_bypass_speed_cmd = 0;
        redblock_slowdown_speed_cmd = REDBLOCK_SLOWDOWN_SPEED_CMD;
        RedBlock_ResetModelVoting();
        RedBlock_SetBypassPhase(RB_BYPASS_PHASE_IDLE);
        RedBlock_SetActionPhase(RB_ACT_IDLE);
    }

    void RedBlock_ClearLocalState(void)
    {
        redblock_confirm_count = 0;
        redblock_detect_frame_counter = 0;
        RedBlock_ClearDetectionResult();
        RedBlock_ResetBypassContext();
        RedBlock_SetState(RB_IDLE);
    }

    void RedBlock_GetSearchPolygon(std::vector<cv::Point> *polygon)
    {
        if(polygon == nullptr)
        {
            return;
        }

        polygon->clear();
        polygon->push_back(cv::Point(RED_SEARCH_POINT_LEFT_TOP_X, RED_SEARCH_POINT_LEFT_TOP_Y));
        polygon->push_back(cv::Point(RED_SEARCH_POINT_RIGHT_TOP_X, RED_SEARCH_POINT_RIGHT_TOP_Y));
        polygon->push_back(cv::Point(RED_SEARCH_POINT_RIGHT_BOTTOM_X, RED_SEARCH_POINT_RIGHT_BOTTOM_Y));
        polygon->push_back(cv::Point(RED_SEARCH_POINT_LEFT_BOTTOM_X, RED_SEARCH_POINT_LEFT_BOTTOM_Y));
    }

    uint8 RedBlock_BuildSearchRect(int frame_width, int frame_height, cv::Rect *search_rect)
    {
        if(search_rect == nullptr || frame_width <= 0 || frame_height <= 0)
        {
            return 0;
        }

        std::vector<cv::Point> polygon;
        RedBlock_GetSearchPolygon(&polygon);
        if(polygon.size() != 4)
        {
            return 0;
        }

        cv::Rect bounding_rect = cv::boundingRect(polygon);
        const int x1 = ClampInt(bounding_rect.x, 0, frame_width - 1);
        const int y1 = ClampInt(bounding_rect.y, 0, frame_height - 1);
        const int x2 = ClampInt(bounding_rect.x + bounding_rect.width, 1, frame_width);
        const int y2 = ClampInt(bounding_rect.y + bounding_rect.height, 1, frame_height);

        if(x2 <= x1 || y2 <= y1)
        {
            return 0;
        }

        *search_rect = cv::Rect(x1, y1, x2 - x1, y2 - y1);
        return 1;
    }

    uint8 RedBlock_ProjectRectToTrack(uint8 *row_top, uint8 *row_bottom, uint8 *col_left, uint8 *col_right)
    {
        const uint8_t *rgb_frame = nullptr;
        int frame_width = 0;
        int frame_height = 0;
        int frame_step = 0;

        if(row_top == nullptr || row_bottom == nullptr || col_left == nullptr || col_right == nullptr)
        {
            return 0;
        }

        if(RedBlock_GetRect(nullptr, nullptr, nullptr, nullptr) == 0)
        {
            return 0;
        }

        if(get_rgb_frame_info(&rgb_frame, &frame_width, &frame_height, &frame_step) != 0)
        {
            return 0;
        }

        if(frame_width <= 0 || frame_height <= 0)
        {
            return 0;
        }

        const int block_left = redblock_x * Cut_COL / frame_width;
        const int block_right = (redblock_x + redblock_width - 1) * Cut_COL / frame_width;
        const int block_top = redblock_y * Cut_ROW / frame_height;
        const int block_bottom = (redblock_y + redblock_height - 1) * Cut_ROW / frame_height;

        *col_left = ClampColIndex(block_left);
        *col_right = ClampColIndex(block_right);
        *row_top = ClampRowIndex(block_top);
        *row_bottom = ClampRowIndex(block_bottom);
        return 1;
    }

    void RedBlock_UpdateProjectedBox(void)
    {
        uint8 row_top = 0;
        uint8 row_bottom = 0;
        uint8 col_left = 0;
        uint8 col_right = 0;

        if(RedBlock_ProjectRectToTrack(&row_top, &row_bottom, &col_left, &col_right) == 0)
        {
            redblock_bypass_detect_valid = 0;
            return;
        }

        if((int)col_right - (int)col_left + 1 < REDBLOCK_BYPASS_MIN_BLOCK_WIDTH ||
           (int)row_bottom - (int)row_top + 1 < REDBLOCK_BYPASS_MIN_BLOCK_HEIGHT)
        {
            redblock_bypass_detect_valid = 0;
            return;
        }

        row_top = ClampRowIndex((int)row_top - REDBLOCK_BYPASS_ROW_EXTEND_UP);
        row_bottom = ClampRowIndex((int)row_bottom + REDBLOCK_BYPASS_ROW_EXTEND_DOWN);
        col_left = ClampColIndex((int)col_left - REDBLOCK_BYPASS_OUTER_MARGIN);
        col_right = ClampColIndex((int)col_right + REDBLOCK_BYPASS_OUTER_MARGIN);

        redblock_bypass_last_row_top = row_top;
        redblock_bypass_last_row_bottom = row_bottom;
        redblock_bypass_last_col_left = col_left;
        redblock_bypass_last_col_right = col_right;
        redblock_bypass_detect_valid = 1;
    }

    uint8 RedBlock_ApplyStraightPass(void)
    {
        uint8 row = 0;

        if(redblock_bypass_detect_valid == 0)
        {
            return 0;
        }

        for(row = redblock_bypass_last_row_top; row <= redblock_bypass_last_row_bottom; row++)
        {
            int left_target = redblock_bypass_last_col_left - Straight_track_width[row] / 2;
            int right_target = redblock_bypass_last_col_right + Straight_track_width[row] / 2;

            left_target = ClampInt(left_target, SEARCH_MIN, redblock_bypass_last_col_left - 1);
            right_target = ClampInt(right_target, redblock_bypass_last_col_right + 1, SEARCH_MAX);

            if(l_border[row] < left_target)
            {
                l_border[row] = (uint8)left_target;
            }
            if(r_border[row] > right_target)
            {
                r_border[row] = (uint8)right_target;
            }
            if(r_border[row] <= l_border[row])
            {
                r_border[row] = ClampColIndex(l_border[row] + 2);
            }
        }
        return 1;
    }

    uint8 RedBlock_ApplyRightBypass(uint8 approach_only)
    {
        uint8 row = 0;
        int shift = 0;

        if(redblock_bypass_detect_valid == 0)
        {
            return 0;
        }

        shift = approach_only ? REDBLOCK_BYPASS_APPROACH_SHIFT : REDBLOCK_BYPASS_INNER_MARGIN;

        for(row = redblock_bypass_last_row_top; row <= redblock_bypass_last_row_bottom; row++)
        {
            int left_target = redblock_bypass_last_col_right + shift;
            left_target = ClampInt(left_target, SEARCH_MIN, SEARCH_MAX - 2);
            l_border[row] = (uint8)left_target;

            {
                int right_target = left_target + Straight_track_width[row];
                r_border[row] = (uint8)ClampInt(right_target, l_border[row] + 2, SEARCH_MAX);
            }
        }
        return 1;
    }

    uint8 RedBlock_ApplyLeftBypass(uint8 approach_only)
    {
        uint8 row = 0;
        int shift = 0;

        if(redblock_bypass_detect_valid == 0)
        {
            return 0;
        }

        shift = approach_only ? REDBLOCK_BYPASS_APPROACH_SHIFT : REDBLOCK_BYPASS_INNER_MARGIN;

        for(row = redblock_bypass_last_row_top; row <= redblock_bypass_last_row_bottom; row++)
        {
            int right_target = redblock_bypass_last_col_left - shift;
            right_target = ClampInt(right_target, SEARCH_MIN + 2, SEARCH_MAX);
            r_border[row] = (uint8)right_target;

            {
                int left_target = right_target - Straight_track_width[row];
                l_border[row] = (uint8)ClampInt(left_target, SEARCH_MIN, r_border[row] - 2);
            }
        }
        return 1;
    }

    uint8 RedBlock_ApplyBypassByMode(uint8 approach_only)
    {
        switch(redblock_bypass_mode)
        {
            case RB_BYPASS_MODE_STRAIGHT:
                return RedBlock_ApplyStraightPass();

            case RB_BYPASS_MODE_RIGHT:
                return RedBlock_ApplyRightBypass(approach_only);

            case RB_BYPASS_MODE_LEFT:
                return RedBlock_ApplyLeftBypass(approach_only);

            default:
                return 0;
        }
    }

    float RedBlock_GetTurnOutProgress(void)
    {
        const float delta = FJ_Angle - redblock_start_yaw;

        if(redblock_bypass_mode == RB_BYPASS_MODE_LEFT)
        {
            return delta;
        }
        if(redblock_bypass_mode == RB_BYPASS_MODE_RIGHT)
        {
            return -delta;
        }
        return REDBLOCK_TURN_OUT_ANGLE;
    }

    uint8 RedBlock_CheckRecoverReady(void)
    {
        if(l_effect_num <= REDBLOCK_RECOVER_EFFECT_THRESHOLD)
        {
            return 0;
        }
        if(r_effect_num <= REDBLOCK_RECOVER_EFFECT_THRESHOLD)
        {
            return 0;
        }
        if(func_abs(err_new) >= REDBLOCK_RECOVER_ERR_THRESHOLD)
        {
            return 0;
        }
        if(Farthest_distance <= REDBLOCK_RECOVER_DISTANCE_THRESHOLD)
        {
            return 0;
        }

        return 1;
    }
}

RedBlockState RedBlock_GetState(void)
{
    return redblock_state;
}

RedBlockBypassMode RedBlock_GetBypassMode(void)
{
    return redblock_bypass_mode;
}

RedBlockBypassPhase RedBlock_GetBypassPhase(void)
{
    return redblock_bypass_phase;
}

RedBlockActionPhase RedBlock_GetActionPhase(void)
{
    return redblock_action_phase;
}

uint8 RedBlock_IsBypassActive(void)
{
    return redblock_bypass_active_flag;
}

float RedBlock_GetBypassDifSpeed(void)
{
    return redblock_bypass_dif_speed;
}

int32_t RedBlock_GetBypassSpeedCmd(void)
{
    return redblock_bypass_speed_cmd;
}

uint8 RedBlock_ShouldIgnoreBoundaryStop(void)
{
    return (redblock_state == RB_DEC_MOTION_ACTIVE && redblock_bypass_active_flag != 0);
}

uint8 RedBlock_IsElementExclusive(void)
{
    return (
        redblock_state == RB_DEC_BRAKING ||
        redblock_state == RB_DEC_LOW_SPEED_SETTLE ||
        redblock_state == RB_DEC_MODEL_RECOGNIZING ||
        (redblock_state == RB_DEC_MOTION_ACTIVE && redblock_bypass_active_flag != 0)
    );
}

uint8 RedBlock_IsModelPending(void)
{
    return (
        redblock_state == RB_DEC_LOW_SPEED_SETTLE ||
        redblock_state == RB_DEC_MODEL_RECOGNIZING ||
        model_request_flag != 0 ||
        model_running_flag != 0
    );
}

uint8 RedBlock_IsSlowdownActive(void)
{
    return (
        redblock_state == RB_DEC_BRAKING ||
        redblock_state == RB_DEC_LOW_SPEED_SETTLE ||
        redblock_state == RB_DEC_MODEL_RECOGNIZING
    );
}

int32_t RedBlock_GetSlowdownSpeedCmd(void)
{
    return redblock_slowdown_speed_cmd;
}

uint8 RedBlock_IsActive(void)
{
    return redblock_state != RB_DEC_IDLE;
}

uint8 RedBlock_ShouldSuppressOtherElements(void)
{
    return (RedBlock_IsActive() || redblock_flag != 0);
}

uint8 RedBlock_ShouldUseLowSpeedHold(void)
{
    return (
        redblock_state == RB_DEC_BRAKING ||
        redblock_state == RB_DEC_LOW_SPEED_SETTLE ||
        redblock_state == RB_DEC_MODEL_RECOGNIZING ||
        (redblock_state == RB_DEC_MOTION_ACTIVE && redblock_bypass_active_flag != 0)
    );
}

int32_t RedBlock_GetMotionSpeedCmd(void)
{
    if(redblock_state == RB_DEC_MOTION_ACTIVE && redblock_bypass_active_flag != 0)
    {
        return redblock_bypass_speed_cmd;
    }
    return redblock_slowdown_speed_cmd;
}

float RedBlock_GetMotionDifSpeed(void)
{
    if(redblock_state == RB_DEC_MOTION_ACTIVE && redblock_bypass_active_flag != 0)
    {
        return redblock_bypass_dif_speed;
    }
    return (float)Servo_PID(err_new);
}

void RedBlock_StartFallbackLeftBypass(const char *reason)
{
    printf("[RB_DEC] recognition_failed reason=%s -> left_bypass\n", reason);
    RedBlock_StartBypassMode(RB_BYPASS_MODE_LEFT);
}

void RedBlock_StartActionForClass(int coarse_index)
{
    switch(coarse_index)
    {
        case MODEL_CLASS_VEHICLE:
            printf("[RB_DEC] voting_result vehicle -> straight_pass\n");
            RedBlock_StartBypassMode(RB_BYPASS_MODE_STRAIGHT);
            break;

        case MODEL_CLASS_SUPPLIERS:
            printf("[RB_DEC] voting_result suppliers -> right_bypass\n");
            RedBlock_StartBypassMode(RB_BYPASS_MODE_RIGHT);
            break;

        case MODEL_CLASS_WEAPON:
            printf("[RB_DEC] voting_result weapon -> left_bypass\n");
            RedBlock_StartBypassMode(RB_BYPASS_MODE_LEFT);
            break;

        default:
            RedBlock_StartFallbackLeftBypass("unknown_class");
            break;
    }
}

void RedBlock_Detect(void)
{
    const uint8_t *rgb_frame = nullptr;
    int frame_width = 0;
    int frame_height = 0;
    int frame_step = 0;

    RedBlock_ClearDetectionResult();

    if(get_rgb_frame_info(&rgb_frame, &frame_width, &frame_height, &frame_step) != 0)
    {
        return;
    }

    if(rgb_frame == nullptr || frame_width <= 0 || frame_height <= 0 || frame_step <= 0)
    {
        return;
    }

    cv::Mat frame_bgr(frame_height, frame_width, CV_8UC3, const_cast<uint8_t *>(rgb_frame), frame_step);
    cv::Rect search_rect(0, 0, frame_width, frame_height);
    if(RedBlock_BuildSearchRect(frame_width, frame_height, &search_rect) != 0)
    {
        frame_bgr = frame_bgr(search_rect);
    }
    else
    {
        search_rect = cv::Rect(0, 0, frame_width, frame_height);
    }

    cv::Mat frame_hsv;
    cv::Mat red_mask_low;
    cv::Mat red_mask_high;
    cv::Mat red_mask;
    std::vector<cv::Point> search_polygon;

    cv::cvtColor(frame_bgr, frame_hsv, cv::COLOR_BGR2HSV);

    cv::Scalar lower_red1(0, 100, 80);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(170, 100, 80);
    cv::Scalar upper_red2(180, 255, 255);

    cv::inRange(frame_hsv, lower_red1, upper_red1, red_mask_low);
    cv::inRange(frame_hsv, lower_red2, upper_red2, red_mask_high);
    cv::bitwise_or(red_mask_low, red_mask_high, red_mask);

    RedBlock_GetSearchPolygon(&search_polygon);
    if(search_polygon.size() >= 3)
    {
        std::vector<std::vector<cv::Point>> polygon_list(1);
        polygon_list[0].reserve(search_polygon.size());
        for(const cv::Point &point : search_polygon)
        {
            polygon_list[0].push_back(cv::Point(point.x - search_rect.x, point.y - search_rect.y));
        }

        cv::Mat polygon_mask = cv::Mat::zeros(red_mask.size(), CV_8UC1);
        cv::fillPoly(polygon_mask, polygon_list, cv::Scalar(255));
        cv::bitwise_and(red_mask, polygon_mask, red_mask);
    }

    const int red_pixel_count = cv::countNonZero(red_mask);
    if(red_pixel_count < 50)
    {
        return;
    }

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(RED_KERNEL_SIZE, RED_KERNEL_SIZE)
    );
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0;
    cv::Rect best_rect;

    for(const auto &contour : contours)
    {
        const double area = cv::contourArea(contour);
        if(area < RED_MIN_AREA)
        {
            continue;
        }

        const cv::Rect rect = cv::boundingRect(contour);
        if(rect.width <= 0 || rect.height <= 0)
        {
            continue;
        }

        const float aspect_ratio = static_cast<float>(rect.width) / static_cast<float>(rect.height);
        if(aspect_ratio < RED_ASPECT_MIN || aspect_ratio > RED_ASPECT_MAX)
        {
            continue;
        }

        const float fill_ratio = static_cast<float>(area) / static_cast<float>(rect.width * rect.height);
        if(fill_ratio < RED_FILL_MIN)
        {
            continue;
        }

        if(area > best_area)
        {
            best_area = area;
            best_rect = rect;
        }
    }

    redblock_area = static_cast<float>(best_area);
    redblock_x = static_cast<int16>(search_rect.x + best_rect.x);
    redblock_y = static_cast<int16>(search_rect.y + best_rect.y);
    redblock_width = static_cast<uint16>(best_rect.width);
    redblock_height = static_cast<uint16>(best_rect.height);

    if(best_rect.width > 0 && best_rect.height > 0)
    {
        redblock_center_x = static_cast<int16>(redblock_x + best_rect.width / 2);
        redblock_center_y = static_cast<int16>(redblock_y + best_rect.height / 2);
    }

    if(best_area > 0)
    {
        redblock_flag = 1;
    }
}

void RedBlock_RequestPause(void)
{
    if(redblock_pause_flag == 0)
    {
        model_request_flag = 1;
        redblock_confirm_count = 0;
        printf("RedBlock low speed reached: area=%.0f, detect model while ultra-low speed\n", redblock_area);
    }
}

void RedBlock_RequestSlowdown(void)
{
    if(redblock_state == RB_SLOWDOWN)
    {
        return;
    }

    redblock_slowdown_frame_count = 0;
    redblock_slowdown_speed_cmd = REDBLOCK_SLOWDOWN_SPEED_CMD;
    redblock_confirm_count = 0;
    pwm0_flag = 1;
    redblock_brake_ticks = REDBLOCK_BRAKE_TICKS;
    printf(
        "RedBlock confirmed: area=%.0f, brake then ultra-low speed before model speed=%ld\n",
        redblock_area,
        (long)redblock_slowdown_speed_cmd
    );
    RedBlock_SetState(RB_SLOWDOWN);
}

void RedBlock_ReleasePause(void)
{
    if(redblock_pause_flag == 1)
    {
        printf("RedBlock_ReleasePause: redblock pause released\n");
    }
    redblock_pause_flag = 0;
    model_request_flag = 0;
    model_running_flag = 0;
    time1 = 0;
    timestop = 0;
    pwm0_flag = 0;
    redblock_brake_ticks = 0;
    RedBlock_ClearLocalState();
}

void RedBlock_ResetState(void)
{
    redblock_pause_flag = 0;
    model_request_flag = 0;
    model_running_flag = 0;
    redblock_brake_ticks = 0;
    redblock_slowdown_frame_count = 0;
    RedBlock_ClearLocalState();
}

void RedBlock_OnModelStarted(void)
{
    model_running_flag = 1;
    if(redblock_state == RB_SLOWDOWN || redblock_state == RB_PAUSED || redblock_state == RB_MODEL_WAIT)
    {
        RedBlock_SetState(RB_MODEL_WAIT);
    }
}

void RedBlock_OnModelConfirmed(void)
{
    if(redblock_state == RB_SLOWDOWN || redblock_state == RB_PAUSED || redblock_state == RB_MODEL_WAIT)
    {
        RedBlock_SetState(RB_CONFIRMED);
    }
}

void RedBlock_StartBypass(void)
{
    RedBlock_StartBypassMode(RB_BYPASS_MODE_STRAIGHT);
}

void RedBlock_StartBypassMode(RedBlockBypassMode mode)
{
    redblock_pause_flag = 0;
    model_request_flag = 0;
    model_running_flag = 0;
    time1 = 0;
    timestop = 0;
    pwm0_flag = 0;
    redblock_brake_ticks = 0;
    redblock_bypass_mode = mode;
    redblock_bypass_mode_flag = static_cast<uint8>(mode);
    redblock_bypass_active_flag = (mode != RB_BYPASS_MODE_NONE) ? 1 : 0;
    redblock_bypass_lost_counter = 0;
    redblock_bypass_detect_valid = 0;
    redblock_start_yaw = FJ_Angle;
    redblock_phase_start_encoder_avg = encoder_acc_avg;
    redblock_recover_ready_count = 0;
    redblock_bypass_speed_cmd = REDBLOCK_BYPASS_SPEED_CMD;
    redblock_bypass_dif_speed = 0.0f;
    RedBlock_SetBypassPhase(RB_BYPASS_PHASE_APPROACH);
    RedBlock_SetActionPhase(RB_ACT_TURN_OUT);
    RedBlock_SetState(RB_DEC_MOTION_ACTIVE);
    printf("[RB_MOT] start mode=%d\n", redblock_bypass_mode_flag);
}

void RedBlock_FinishBypass(void)
{
    if(redblock_pause_flag)
    {
        RedBlock_ReleasePause();
        return;
    }
    printf("[RB_MOT] finish\n");
    RedBlock_ClearLocalState();
}

uint8 RedBlock_ApplyBypass(void)
{
    uint8 applied = 0;
    const int32_t encoder_progress = func_abs(encoder_acc_avg - redblock_phase_start_encoder_avg);

    if(redblock_state != RB_DEC_MOTION_ACTIVE || redblock_bypass_active_flag == 0)
    {
        return 0;
    }

    redblock_bypass_speed_cmd = REDBLOCK_BYPASS_SPEED_CMD;

    switch(redblock_action_phase)
    {
        case RB_ACT_TURN_OUT:
            if(redblock_bypass_mode == RB_BYPASS_MODE_LEFT)
            {
                redblock_bypass_dif_speed = REDBLOCK_TURN_CMD;
            }
            else if(redblock_bypass_mode == RB_BYPASS_MODE_RIGHT)
            {
                redblock_bypass_dif_speed = -REDBLOCK_TURN_CMD;
            }
            else
            {
                redblock_bypass_dif_speed = 0.0f;
            }

            if(RedBlock_GetTurnOutProgress() >= REDBLOCK_TURN_OUT_ANGLE)
            {
                redblock_phase_start_encoder_avg = encoder_acc_avg;
                redblock_bypass_dif_speed = 0.0f;
                RedBlock_SetActionPhase(RB_ACT_PASS_1);
            }
            break;

        case RB_ACT_PASS_1:
            redblock_bypass_dif_speed = 0.0f;
            if(encoder_progress >= REDBLOCK_PASS1_DISTANCE)
            {
                RedBlock_SetActionPhase(RB_ACT_TURN_BACK);
            }
            break;

        case RB_ACT_TURN_BACK:
            if(redblock_bypass_mode == RB_BYPASS_MODE_LEFT)
            {
                redblock_bypass_dif_speed = -REDBLOCK_TURN_CMD;
            }
            else if(redblock_bypass_mode == RB_BYPASS_MODE_RIGHT)
            {
                redblock_bypass_dif_speed = REDBLOCK_TURN_CMD;
            }
            else
            {
                redblock_bypass_dif_speed = 0.0f;
            }

            if(func_abs(FJ_Angle - redblock_start_yaw) <= REDBLOCK_TURN_BACK_TOLERANCE)
            {
                redblock_phase_start_encoder_avg = encoder_acc_avg;
                redblock_bypass_dif_speed = 0.0f;
                RedBlock_SetActionPhase(RB_ACT_PASS_2);
            }
            break;

        case RB_ACT_PASS_2:
            redblock_bypass_dif_speed = 0.0f;
            if(encoder_progress >= REDBLOCK_PASS2_DISTANCE)
            {
                redblock_recover_ready_count = 0;
                RedBlock_SetActionPhase(RB_ACT_RECOVER);
            }
            break;

        case RB_ACT_RECOVER:
            redblock_bypass_dif_speed = (float)Servo_PID(err_new);
            redblock_recover_diag_div++;
            if(redblock_recover_diag_div >= 10)
            {
                redblock_recover_diag_div = 0;
                printf(
                    "[RB_RECOVER] ready_cnt=%u err=%.2f far=%.2f effect(L,R)=(%d,%d) dif=%.2f active=%u phase=%u\n",
                    redblock_recover_ready_count,
                    err_new,
                    Farthest_distance,
                    l_effect_num,
                    r_effect_num,
                    redblock_bypass_dif_speed,
                    redblock_bypass_active_flag,
                    redblock_action_phase_flag
                );
            }
            if(RedBlock_CheckRecoverReady())
            {
                if(redblock_recover_ready_count < 255)
                {
                    redblock_recover_ready_count++;
                }
            }
            else
            {
                redblock_recover_ready_count = 0;
            }

            if(redblock_recover_ready_count >= REDBLOCK_RECOVER_STABLE_REQUIRED)
            {
                RedBlock_FinishBypass();
            }
            break;

        default:
            redblock_bypass_dif_speed = 0.0f;
            break;
    }

    return applied;
}

uint8 RedBlock_GetSearchRect(int16 *x, int16 *y, uint16 *width, uint16 *height)
{
    const uint8_t *rgb_frame = nullptr;
    int frame_width = 0;
    int frame_height = 0;
    int frame_step = 0;
    cv::Rect search_rect;

    if(get_rgb_frame_info(&rgb_frame, &frame_width, &frame_height, &frame_step) != 0)
    {
        return 0;
    }

    if(frame_width <= 0 || frame_height <= 0)
    {
        return 0;
    }

    if(RedBlock_BuildSearchRect(frame_width, frame_height, &search_rect) == 0)
    {
        return 0;
    }

    if(x != nullptr)
    {
        *x = static_cast<int16>(search_rect.x);
    }
    if(y != nullptr)
    {
        *y = static_cast<int16>(search_rect.y);
    }
    if(width != nullptr)
    {
        *width = static_cast<uint16>(search_rect.width);
    }
    if(height != nullptr)
    {
        *height = static_cast<uint16>(search_rect.height);
    }
    return 1;
}

uint8 RedBlock_GetRect(int16 *x, int16 *y, uint16 *width, uint16 *height)
{
    if(redblock_flag == 0 || redblock_width == 0 || redblock_height == 0)
    {
        return 0;
    }

    if(x != nullptr)
    {
        *x = redblock_x;
    }
    if(y != nullptr)
    {
        *y = redblock_y;
    }
    if(width != nullptr)
    {
        *width = redblock_width;
    }
    if(height != nullptr)
    {
        *height = redblock_height;
    }
    return 1;
}

uint8 RedBlock_GetModelRoi(int16 *x, int16 *y, uint16 *width, uint16 *height)
{
    const uint8_t *rgb_frame = nullptr;
    int frame_width = 0;
    int frame_height = 0;
    int frame_step = 0;

    if(RedBlock_GetRect(nullptr, nullptr, nullptr, nullptr) == 0)
    {
        return 0;
    }

    if(get_rgb_frame_info(&rgb_frame, &frame_width, &frame_height, &frame_step) != 0)
    {
        return 0;
    }

    if(frame_width <= 0 || frame_height <= 0)
    {
        return 0;
    }

    const int block_width = redblock_width;
    const int block_height = redblock_height;
    const int block_left = redblock_x;
    const int block_top = redblock_y;
    const int block_right = block_left + block_width;
    const int block_bottom = block_top + block_height - 1;

    if(block_width <= 0 || block_height <= 0 || block_left < 0 || block_top < 0)
    {
        return 0;
    }

    const int estimated_object_height = static_cast<int>(block_height * RED_MODEL_ROI_OBJECT_HEIGHT_RATIO);
    const int total_height = estimated_object_height + block_height + RED_MODEL_ROI_EDGE_PADDING;
    int roi_side = total_height;
    if(roi_side < block_width)
    {
        roi_side = block_width;
    }
    if(roi_side < RED_MODEL_ROI_MIN_SIDE)
    {
        roi_side = RED_MODEL_ROI_MIN_SIDE;
    }

    roi_side = ClampInt(roi_side, 1, frame_width < frame_height ? frame_width : frame_height);

    const int bottom_margin = static_cast<int>(roi_side * RED_MODEL_ROI_BOTTOM_MARGIN_RATIO);
    const int center_up_offset = static_cast<int>(roi_side * RED_MODEL_ROI_CENTER_UP_RATIO);
    const int roi_center_x = (block_left + block_right) / 2;
    int roi_x1 = roi_center_x - roi_side / 2;
    int roi_y2 = block_bottom + RED_MODEL_ROI_EDGE_PADDING + bottom_margin - center_up_offset;
    int roi_y1 = roi_y2 - roi_side;
    int roi_x2 = roi_x1 + roi_side;

    if(roi_x1 < 0)
    {
        roi_x2 -= roi_x1;
        roi_x1 = 0;
    }
    if(roi_x2 > frame_width)
    {
        roi_x1 -= (roi_x2 - frame_width);
        roi_x2 = frame_width;
    }
    if(roi_y1 < 0)
    {
        roi_y2 -= roi_y1;
        roi_y1 = 0;
    }
    if(roi_y2 > frame_height)
    {
        roi_y1 -= (roi_y2 - frame_height);
        roi_y2 = frame_height;
    }

    roi_x1 = ClampInt(roi_x1, 0, frame_width - roi_side);
    roi_y1 = ClampInt(roi_y1, 0, frame_height - roi_side);

    if(width != nullptr)
    {
        *width = static_cast<uint16>(roi_side);
    }
    if(height != nullptr)
    {
        *height = static_cast<uint16>(roi_side);
    }
    if(x != nullptr)
    {
        *x = static_cast<int16>(roi_x1);
    }
    if(y != nullptr)
    {
        *y = static_cast<int16>(roi_y1);
    }

    return 1;
}

uint8 RedBlock_PrepareModelInput(uint8 *output_bgr, uint16 output_width, uint16 output_height)
{
    if(output_bgr == nullptr || output_width == 0 || output_height == 0)
    {
        return 0;
    }

    return RedBlock_PrepareModelInputFromCurrentRoi(output_bgr, output_width, output_height);
}

void RedBlock_UpdatePerception(void)
{
    if(redblock_state != RB_DEC_IDLE)
    {
        RedBlock_Detect();
        redblock_detect_frame_counter = 0;
        return;
    }

    if(redblock_detect_frame_counter != 0)
    {
        redblock_detect_frame_counter = (redblock_detect_frame_counter + 1) % REDBLOCK_DETECT_INTERVAL;
        return;
    }

    RedBlock_Detect();
    redblock_detect_frame_counter = (redblock_detect_frame_counter + 1) % REDBLOCK_DETECT_INTERVAL;
}

void RedBlock_UpdateDecision(void)
{
    switch(redblock_state)
    {
        case RB_DEC_IDLE:
            if(redblock_flag)
            {
                redblock_confirm_count = 1;
                printf("[RB_DEC] redblock_seen area=%.0f confirm=%u/%u\n",
                       redblock_area,
                       redblock_confirm_count,
                       REDBLOCK_CONFIRM_REQUIRED);
                RedBlock_SetState(RB_DEC_CONFIRMING);
            }
            break;

        case RB_DEC_CONFIRMING:
            if(redblock_flag)
            {
                if(redblock_confirm_count < REDBLOCK_CONFIRM_REQUIRED)
                {
                    redblock_confirm_count++;
                }

                if(redblock_confirm_count >= REDBLOCK_CONFIRM_REQUIRED)
                {
                    redblock_slowdown_speed_cmd = REDBLOCK_SLOWDOWN_SPEED_CMD;
                    redblock_low_speed_settle_count = 0;
                    redblock_confirm_count = 0;
                    redblock_brake_ticks = REDBLOCK_BRAKE_TICKS;
                    pwm0_flag = 1;
                    RedBlock_ResetModelVoting();
                    printf("[RB_DEC] confirmed area=%.0f -> braking ticks=%u speed=%ld\n",
                           redblock_area,
                           REDBLOCK_BRAKE_TICKS,
                           (long)redblock_slowdown_speed_cmd);
                    RedBlock_SetState(RB_DEC_BRAKING);
                }
            }
            else
            {
                redblock_confirm_count = 0;
                RedBlock_SetState(RB_DEC_IDLE);
            }
            break;

        case RB_DEC_BRAKING:
            if(redblock_brake_ticks == 0)
            {
                pwm0_flag = 0;
                redblock_low_speed_settle_count = 0;
                printf("[RB_DEC] braking_done enc=(%d,%d) -> low_speed_settle %u frames\n",
                       enconder_left,
                       enconder_right,
                       REDBLOCK_LOW_SPEED_SETTLE_FRAMES);
                RedBlock_SetState(RB_DEC_LOW_SPEED_SETTLE);
            }
            break;

        case RB_DEC_LOW_SPEED_SETTLE:
            if(redblock_low_speed_settle_count < REDBLOCK_LOW_SPEED_SETTLE_FRAMES)
            {
                redblock_low_speed_settle_count++;
                printf("[RB_DEC] low_speed_settle %u/%u enc=(%d,%d)\n",
                       redblock_low_speed_settle_count,
                       REDBLOCK_LOW_SPEED_SETTLE_FRAMES,
                       enconder_left,
                       enconder_right);
            }

            if(redblock_low_speed_settle_count >= REDBLOCK_LOW_SPEED_SETTLE_FRAMES)
            {
                if(redblock_flag == 0)
                {
                    RedBlock_StartFallbackLeftBypass("redblock_lost_after_settle");
                }
                else
                {
                    RedBlock_ResetModelVoting();
                    printf("[RB_DEC] model_recognizing votes=%u invalid_retry=%u\n",
                           MODEL_VOTE_REQUIRED,
                           REDBLOCK_MODEL_INVALID_RETRY_FRAMES);
                    RedBlock_SetState(RB_DEC_MODEL_RECOGNIZING);
                }
            }
            break;

        case RB_DEC_MODEL_RECOGNIZING:
            {
                if(redblock_flag == 0)
                {
                    RedBlock_StartFallbackLeftBypass("redblock_lost_during_model");
                    break;
                }

                model_running_flag = 1;
                const NCNN_Infer_Result infer_result = ncnn_infer_run_once();
                model_running_flag = 0;

                if(!infer_result.ready || !infer_result.valid)
                {
                    redblock_model_invalid_count++;
                    printf("[RB_DEC] model_invalid retry=%u/%u ready=%d valid=%d\n",
                           redblock_model_invalid_count,
                           REDBLOCK_MODEL_INVALID_RETRY_FRAMES,
                           infer_result.ready ? 1 : 0,
                           infer_result.valid ? 1 : 0);
                    if(redblock_model_invalid_count >= REDBLOCK_MODEL_INVALID_RETRY_FRAMES)
                    {
                        RedBlock_StartFallbackLeftBypass("model_invalid_timeout");
                    }
                    break;
                }

                redblock_model_invalid_count = 0;
                RedBlock_AddModelVote(infer_result.coarse_index);
                printf("[RB_DEC] vote %u/%u fine=%d %s coarse=%d %s votes=[%u,%u,%u]\n",
                       redblock_model_vote_valid_count,
                       MODEL_VOTE_REQUIRED,
                       infer_result.class_index,
                       infer_result.fine_label.c_str(),
                       infer_result.coarse_index,
                       infer_result.label.c_str(),
                       redblock_model_vote_count[MODEL_CLASS_SUPPLIERS],
                       redblock_model_vote_count[MODEL_CLASS_VEHICLE],
                       redblock_model_vote_count[MODEL_CLASS_WEAPON]);

                if(redblock_model_vote_valid_count >= MODEL_VOTE_REQUIRED)
                {
                    const int final_class = RedBlock_GetBestModelVote();
                    if(final_class < 0)
                    {
                        RedBlock_StartFallbackLeftBypass("vote_no_result");
                    }
                    else
                    {
                        printf("[RB_DEC] final class=%d %s votes=[%u,%u,%u]\n",
                               final_class,
                               RedBlock_ModelClassLabel(final_class),
                               redblock_model_vote_count[MODEL_CLASS_SUPPLIERS],
                               redblock_model_vote_count[MODEL_CLASS_VEHICLE],
                               redblock_model_vote_count[MODEL_CLASS_WEAPON]);
                        RedBlock_StartActionForClass(final_class);
                    }
                }
            }
            break;

        case RB_DEC_MOTION_ACTIVE:
        default:
            break;
    }
}

void RedBlock_ApplyMotion(void)
{
    RedBlock_ApplyBypass();
}

void RedBlock_Update(void)
{
    RedBlock_UpdatePerception();
    RedBlock_UpdateDecision();
}
