#include "redblock.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "camera.h"
#include "motor.h"
#include "cross.h"
#include "ncnn_infer.h"
#include "zf_device_uvc.h"

extern uint8 redblock_pause_flag;
extern uint8 model_request_flag;
extern uint8 model_running_flag;
extern int time1;
extern int timestop;
extern uint8 pwm0_flag;
extern u_char stop;
extern uint8 zebra_flag;
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
volatile int32_t redblock_bypass_speed_cmd = 0;
volatile int32_t redblock_slowdown_speed_cmd = 90;  //红块减速速度

namespace
{
    // 调试开关：0=完全关闭红块检测、模型识别和绕行，车辆仅执行普通巡线；1=启用红块模块。
    constexpr uint8 REDBLOCK_DETECTION_ENABLE = 1;

    // 调参：红块检测间隔帧数。越小响应越快但更吃算力，越大更稳但触发更慢。
    constexpr uint8 REDBLOCK_DETECT_INTERVAL = 2;

    // 调参：连续命中多少次才确认红块。1 最快，2~3 更抗误检。
    constexpr uint8 REDBLOCK_CONFIRM_REQUIRED = 1;

    // 调参：红色轮廓最小面积，增大可过滤远处小红点，过大可能漏检小红块。
    constexpr int RED_MIN_AREA = 104;
    constexpr int RED_MAX_AREA = 830;  // 调参：红色轮廓最大面积，超过认为不是需要绕行的红色色块，忽略并保持普通巡线。

    // 调参：形态学开闭运算核大小。增大更能去噪，过大可能吞掉小目标。
    constexpr int RED_KERNEL_SIZE = 3;

    // 调参：红块外接矩形宽高比范围，用于排除过细或过扁的红色区域。
    constexpr float RED_ASPECT_MIN = 0.20f;
    constexpr float RED_ASPECT_MAX = 5.0f;

    constexpr float RED_FILL_MIN = 0.40f;
    constexpr float REDBLOCK_SIDE_IGNORE_RATIO = 0.16f;  // 调参：仅赛道最外侧 12% 的红色轮廓才会被视为贴边红砖。
    constexpr int REDBLOCK_SIDE_IGNORE_MIN_LANE_WIDTH = 40;  // 调参：边线间距小于此值时放行红块，避免边线不可靠导致漏检。
    // 调参：窄赛道时，距边线此范围内或位于边线外的红色轮廓仍按侧边红砖忽略。
    constexpr int REDBLOCK_SIDE_IGNORE_NARROW_EDGE_MARGIN = 2;
    // 调参：仅过滤画面上方的窄赛道侧边红砖。红砖靠近时面积会变大，不能以面积作为放行条件。
    // 真实红块进入此行后，即使贴边也必须参与识别；当前手推样本中真实红块最早位于 y=81。
    constexpr int REDBLOCK_SIDE_IGNORE_NARROW_MAX_CENTER_Y = 70;
    constexpr uint8 REDBLOCK_SIDE_IGNORE_LOG_INTERVAL = 8;  // 调参：贴边红砖忽略日志间隔，检测每 2 帧执行一次。
    // 调试：侧边红砖门控放行原因日志间隔。手推验证时保持 1，赛跑时可改为 8。
    constexpr uint8 REDBLOCK_SIDE_GATE_DIAG_INTERVAL = 1;
    constexpr int RED_MODEL_ROI_MIN_SIDE = 42;
    // 调参：模型 ROI 相对红色底座宽度的放大倍率。1.25 为底座保留约 20% 横向余量，匹配当前采集构图。
    constexpr float RED_MODEL_ROI_WIDTH_SCALE = 1.25f;
    constexpr int RED_MODEL_ROI_EDGE_PADDING = 2;
    // 调参：红块底部留白比例。增大可保留红块下缘，过大可能压缩上方目标。
    constexpr float RED_MODEL_ROI_BOTTOM_MARGIN_RATIO = 0.08f;
    // 调参：按红块高度估计上方主体高度。目标更高时增大，背景太多时减小。
    constexpr float RED_MODEL_ROI_OBJECT_HEIGHT_RATIO = 1.05f;
    // 调参：ROI 中心上移比例。增大让上方主体占比更大，过大可能裁掉红块。
    constexpr float RED_MODEL_ROI_CENTER_UP_RATIO = 0.10f;

    // 调参：红块检测四边形区域，坐标为 320x240 图像坐标系。
    // 顺时针组织四个点，缩小区域可减少误检，放大区域可提前发现红块。
    constexpr int RED_SEARCH_POINT_LEFT_TOP_X = 56;
    constexpr int RED_SEARCH_POINT_LEFT_TOP_Y = 1;
    constexpr int RED_SEARCH_POINT_RIGHT_TOP_X = 258;
    constexpr int RED_SEARCH_POINT_RIGHT_TOP_Y = 1;
    constexpr int RED_SEARCH_POINT_RIGHT_BOTTOM_X = 258;
    constexpr int RED_SEARCH_POINT_RIGHT_BOTTOM_Y = 205;
    constexpr int RED_SEARCH_POINT_LEFT_BOTTOM_X = 56;
    constexpr int RED_SEARCH_POINT_LEFT_BOTTOM_Y = 205;

    // 调参：确认红块后低速保持几帧再开始模型识别。增大可让画面更稳，过大响应变慢。
    constexpr uint8 REDBLOCK_LOW_SPEED_SETTLE_FRAMES = 3;
    // 调参：模型 invalid 允许重试几帧；红块当前帧丢失时不重试，直接左绕。
    constexpr uint8 REDBLOCK_MODEL_INVALID_RETRY_FRAMES = 4;
    // 调参：红块识别期间低速保持速度。
    constexpr int32_t REDBLOCK_SLOWDOWN_SPEED_CMD = 40;
    // Visual bypass progresses by encoder distance only. Red-block position is
    // used for recognition, not for the bypass path.
    // Visual bypass directly assigns the selected lane boundary to Center_point.
    constexpr int REDBLOCK_VISUAL_CONTROL_WINDOW = 10;
    constexpr int REDBLOCK_VISUAL_CONTROL_ROW_MIN = 45;
    // 调参：一个控制窗口中至少需要多少个当前帧有效边界点。增大更严格，减小更容易跟线。
    constexpr int REDBLOCK_VISUAL_VALID_ROWS_REQUIRED = 7;
    // 调参：控制窗口每帧最多移动的行数，限制动态前瞻跳变。
    constexpr int REDBLOCK_VISUAL_CONTROL_ROW_STEP = 2;
    // 调参：切向贴边曲线的远端、近端行偏移。远端行已经贴边，近端行仍保持普通中线。
    constexpr int REDBLOCK_VISUAL_TANGENT_FAR_OFFSET = 8;
    constexpr int REDBLOCK_VISUAL_TANGENT_NEAR_OFFSET = 27;//24
    // 调参：切向贴边曲线指数。必须大于 1；越大越早靠边，过大可能横向切入过急。
    constexpr float REDBLOCK_VISUAL_TANGENT_POWER = 14.0f;//12
    constexpr float REDBLOCK_VISUAL_TANGENT_POWER_MIN = 1.1f;
    constexpr float REDBLOCK_VISUAL_TANGENT_POWER_MAX = 16.0f;
    // 调参：控制行的生成目标距离真实边线不超过该像素数，才允许进入贴边保持阶段。
    constexpr int REDBLOCK_VISUAL_TANGENT_READY_GAP = 2;
    constexpr int REDBLOCK_VISUAL_BOUNDARY_MIN_WIDTH = 8;
    // 调参：同一行边界相邻帧允许的最大横向跳变。急转时边线移动很快，过小会把真实边线误判为丢失。
    constexpr int REDBLOCK_VISUAL_BOUNDARY_MAX_FRAME_STEP = 80;
    // 调参：边界短暂丢失时继续使用最近可靠目标的帧数。
    constexpr uint8 REDBLOCK_VISUAL_BOUNDARY_CACHE_FRAMES = 3;
    // 调参：超过缓存期后保持低速；只有从未取得可靠缓存时才停车等待边界恢复。
    constexpr uint8 REDBLOCK_VISUAL_BOUNDARY_LOST_SLOW_FRAMES = 7;
    // 调参：靠边阶段连续失去边线且无缓存达到此帧数时，放弃本次绕行并恢复普通巡线，防止永久停车。
    constexpr uint8 REDBLOCK_VISUAL_SEEK_LOST_EXIT_FRAMES = 15;
    // 调参：边界连续丢失 4~7 帧时的低速命令，降低盲走距离。
    constexpr int32_t REDBLOCK_VISUAL_BOUNDARY_LOST_SPEED_CMD = 90;
    // 调参：边界连续丢失 4~7 帧时的差速限幅，避免沿旧目标继续猛转。
    constexpr float REDBLOCK_VISUAL_BOUNDARY_LOST_DIF_LIMIT = 16.0f;
    // 调参：快速靠边阶段速度。保持较低前进速度，把余量留给横向转向。
    constexpr int32_t REDBLOCK_VISUAL_SEEK_SPEED_CMD = 126;
    // 调参：快速靠边阶段最大差速，增大可更快抵达边线，过大可能过冲。
    constexpr float REDBLOCK_VISUAL_SEEK_DIF_LIMIT = 52.0f;
    // 调参：判断已经贴近边线的误差阈值和连续帧数。
    constexpr float REDBLOCK_VISUAL_SEEK_READY_ERR = 4.0f;
    constexpr uint8 REDBLOCK_VISUAL_SEEK_READY_FRAMES = 2;
    // 调参：靠边距离超过该值仍未贴边时降速继续追边，不强制切换 phase。
    constexpr int32_t REDBLOCK_VISUAL_SEEK_SOFT_DISTANCE = 4500;
    constexpr int32_t REDBLOCK_VISUAL_SEEK_SLOW_SPEED_CMD = 100;
    // 调参：贴边保持阶段速度和差速限幅。
    constexpr int32_t REDBLOCK_VISUAL_HOLD_SPEED_CMD = 160;
    constexpr float REDBLOCK_VISUAL_HOLD_DIF_LIMIT = 28.0f;
    // 调参：确认贴边后沿边线行驶的编码器距离，不包含前面的靠边距离。
    constexpr int32_t REDBLOCK_VISUAL_HOLD_DISTANCE = 7000;
    // 调参：主体通过后继续贴边的编码器距离，用于确保车尾越过障碍后才开始回中线。
    constexpr int32_t REDBLOCK_VISUAL_POST_PASS_HOLD_DISTANCE = 4900;
    // 调参：主体通过后至少保持的帧数。与距离条件同时满足后才允许回中线，43 继承旧版有效策略。
    constexpr uint8 REDBLOCK_VISUAL_POST_PASS_HOLD_MIN_FRAMES = 21;
    constexpr float REDBLOCK_VISUAL_RECOVER_DIF_LIMIT = 25.0f;
    // 调参：从边线平滑回普通中线所用的编码器距离，不承担车尾越障保持功能。
    constexpr int32_t REDBLOCK_VISUAL_EXIT_HOLD_DISTANCE = 1500;
    constexpr uint8 REDBLOCK_VISUAL_EXIT_HOLD_MAX_FRAMES = 42;
    constexpr uint8 REDBLOCK_VISUAL_RECOVER_MAX_FRAMES = 28;
    // 调参：回线恢复阶段速度。低于绕行速度，减少回线时左右摆动。
    constexpr int32_t REDBLOCK_RECOVER_SPEED_CMD = 140;
    // 调参：恢复赛道需要连续满足的帧数，增大更稳，减小更快退出绕行。
    constexpr uint8 REDBLOCK_RECOVER_STABLE_REQUIRED = 3;
    // 调参：恢复判定的左右有效边界数量下限。
    constexpr int REDBLOCK_RECOVER_EFFECT_THRESHOLD = 35;
    // 调参：恢复判定的中线误差阈值。
    constexpr float REDBLOCK_RECOVER_ERR_THRESHOLD = 12.0f;
    // 调参：恢复判定的前方可视距离阈值。实测 far 经常只有 8~35，阈值设小避免绕行后长期无法退出。
    constexpr float REDBLOCK_RECOVER_DISTANCE_THRESHOLD = 5.0f;
    // 调参：绕行结束后的重触发冷却帧数。60fps 下 45 帧约 0.75s，用来避开同一红块仍在视野内的尾帧。
    constexpr uint8 REDBLOCK_COOLDOWN_FRAMES = 60;
    // 调参：绕行结束后继续忽略出界/斑马线停车的帧数，防止刚回线时被保护逻辑立刻刹停。
    constexpr uint8 REDBLOCK_FINISH_BOUNDARY_GRACE_FRAMES = 12;
    constexpr int MODEL_CLASS_SUPPLIERS = 0;
    constexpr int MODEL_CLASS_VEHICLE = 1;
    constexpr int MODEL_CLASS_WEAPON = 2;
    constexpr uint8 MODEL_CLASS_COUNT = 3;
    // 调参：一趟运行最多记录的已确认粗分类数量。超过后保留前面的识别顺序并在停车日志标记截断。
    constexpr uint8 REDBLOCK_CLASS_HISTORY_CAPACITY = 8;
    // 调参：有效模型投票帧数。当前只有 3 个粗类，5 帧投票不会平票。
    constexpr uint8 MODEL_VOTE_REQUIRED = 3;//1

    RedBlockState redblock_state = RB_IDLE;
    uint8 redblock_detect_frame_counter = 0;
    RedBlockBypassMode redblock_bypass_mode = RB_BYPASS_MODE_NONE;
    RedBlockBypassPhase redblock_bypass_phase = RB_BYPASS_PHASE_IDLE;
    uint8 redblock_bypass_phase_counter = 0;
    int32_t redblock_phase_start_encoder_avg = 0;
    uint8 redblock_recover_ready_count = 0;
    uint8 redblock_recover_diag_div = 0;
    uint8 redblock_visual_diag_div = 0;
    uint8 redblock_motion_dif_diag_div = 0;
    uint8 redblock_visual_err_diag_div = 0;
    int8 redblock_visual_control_row = REDBLOCK_VISUAL_CONTROL_ROW_MIN;
    uint8 redblock_visual_control_row_valid = 0;
    uint8 redblock_visual_boundary_cache[Cut_ROW] = {0};
    uint8 redblock_visual_boundary_cache_valid[Cut_ROW] = {0};
    uint8 redblock_visual_boundary_lost_frames = 0;
    uint8 redblock_visual_boundary_valid_rows = 0;
    uint8 redblock_visual_seek_ready_count = 0;
    uint8 redblock_visual_seek_slow_logged = 0;
    uint8 redblock_visual_tangent_ready = 0;
    uint8 redblock_visual_tangent_normal_center = 0;
    uint8 redblock_visual_tangent_boundary_center = 0;
    uint8 redblock_visual_tangent_target_center = 0;
    uint8 redblock_visual_tangent_gap = 0;
    float redblock_visual_tangent_power_applied = REDBLOCK_VISUAL_TANGENT_POWER;

    enum RedBlockVisualTargetSource
    {
        RB_VIS_TARGET_CURRENT = 0,
        RB_VIS_TARGET_CACHE,
        RB_VIS_TARGET_LOST,
    };

    RedBlockVisualTargetSource redblock_visual_target_source = RB_VIS_TARGET_LOST;
    uint8 redblock_slowdown_frame_count = 0;
    uint8 redblock_low_speed_settle_count = 0;
    uint8 redblock_model_invalid_count = 0;
    uint8 redblock_model_vote_valid_count = 0;
    uint8 redblock_model_vote_count[MODEL_CLASS_COUNT] = {0};
    uint8 redblock_class_history[REDBLOCK_CLASS_HISTORY_CAPACITY] = {0};
    uint8 redblock_class_history_count = 0;
    uint8 redblock_class_history_overflow = 0;
    uint8 redblock_class_history_reported = 0;
    int32_t redblock_restore_speed_cmd = 0;
    uint8 redblock_restore_speed_valid = 0;
    uint8 redblock_cooldown_count = 0;
    uint8 redblock_boundary_stop_grace_frames = 0;
    uint8 redblock_side_ignore_log_count = 0;
    uint8 redblock_side_gate_diag_count = 0;

    void RedBlock_ClearLocalState(void);

    void RedBlock_ResetVisualTargetContext(void)
    {
        memset(redblock_visual_boundary_cache, 0, sizeof(redblock_visual_boundary_cache));
        memset(redblock_visual_boundary_cache_valid, 0, sizeof(redblock_visual_boundary_cache_valid));
        redblock_visual_boundary_lost_frames = 0;
        redblock_visual_boundary_valid_rows = 0;
        redblock_visual_seek_ready_count = 0;
        redblock_visual_seek_slow_logged = 0;
        redblock_visual_tangent_ready = 0;
        redblock_visual_tangent_normal_center = 0;
        redblock_visual_tangent_boundary_center = 0;
        redblock_visual_tangent_target_center = 0;
        redblock_visual_tangent_gap = 0;
        redblock_visual_tangent_power_applied = REDBLOCK_VISUAL_TANGENT_POWER;
        redblock_visual_target_source = RB_VIS_TARGET_LOST;
        redblock_visual_control_row = REDBLOCK_VISUAL_CONTROL_ROW_MIN;
        redblock_visual_control_row_valid = 0;
    }

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

    float ClampFloat(float value, float min_value, float max_value)
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

    void RedBlock_LogSideGateDiagnostic(const char *reason,
                                        int mapped_line_row,
                                        int line_x,
                                        int line_row,
                                        int left_border,
                                        int right_border,
                                        float lane_ratio)
    {
        redblock_side_gate_diag_count++;
        if(redblock_side_gate_diag_count < REDBLOCK_SIDE_GATE_DIAG_INTERVAL)
        {
            return;
        }

        redblock_side_gate_diag_count = 0;
        printf("[RB_GATE_DIAG] %s area=%.0f raw=(%d,%d) line_x=%d row=%d->%d lane=(%d,%d) ratio=%.3f\n",
               reason,
               redblock_area,
               redblock_center_x,
               redblock_center_y,
               line_x,
               mapped_line_row,
               line_row,
               left_border,
               right_border,
               lane_ratio);
    }

    uint8 RedBlock_ShouldIgnoreSideBrick(int frame_width, int frame_height)
    {
        // 原始 RGB 为 320x240，巡线边线为缩放后的 160x90。边线或坐标不可靠时一律放行。
        if(frame_width <= 0 || frame_height <= 0 ||
           redblock_center_x < 0 || redblock_center_y < 0)
        {
            return 0;
        }

        const int line_x = redblock_center_x * UVC_WIDTH / frame_width;
        const int mapped_line_row = redblock_center_y * UVC_HEIGHT / frame_height;
        if(line_x < 0 || line_x >= Cut_COL)
        {
            RedBlock_LogSideGateDiagnostic("x_invalid", mapped_line_row, line_x, -1, -1, -1, -1.0f);
            return 0;
        }

        // 原图底部红砖映射到巡线图外时，使用最后一行有效边线进行贴边判定。
        const int line_row = ClampInt(mapped_line_row, 0, Cut_ROW - 1);
        const uint8 row_clamped = (mapped_line_row != line_row);

        if(l_effect_flag[line_row] == 0 || r_effect_flag[line_row] == 0)
        {
            RedBlock_LogSideGateDiagnostic("edge_invalid", mapped_line_row, line_x, line_row, -1, -1, -1.0f);
            return 0;
        }

        const int left_border = l_border[line_row];
        const int right_border = r_border[line_row];
        const int lane_width = right_border - left_border;
        if(lane_width < REDBLOCK_SIDE_IGNORE_MIN_LANE_WIDTH)
        {
            const uint8 is_narrow_side_brick =
                (redblock_center_y <= REDBLOCK_SIDE_IGNORE_NARROW_MAX_CENTER_Y &&
                 (line_x <= left_border + REDBLOCK_SIDE_IGNORE_NARROW_EDGE_MARGIN ||
                  line_x >= right_border - REDBLOCK_SIDE_IGNORE_NARROW_EDGE_MARGIN));
            if(is_narrow_side_brick != 0)
            {
                RedBlock_LogSideGateDiagnostic("lane_narrow_side", mapped_line_row, line_x, line_row,
                                               left_border, right_border, -1.0f);
                return 1;
            }
            RedBlock_LogSideGateDiagnostic("lane_narrow_object", mapped_line_row, line_x, line_row,
                                           left_border, right_border, -1.0f);
            return 0;
        }

        const float lane_ratio =
            static_cast<float>(line_x - left_border) / static_cast<float>(lane_width);
        const uint8 is_side_brick =
            (lane_ratio <= REDBLOCK_SIDE_IGNORE_RATIO ||
             lane_ratio >= (1.0f - REDBLOCK_SIDE_IGNORE_RATIO));
        if(is_side_brick == 0)
        {
            redblock_side_ignore_log_count = 0;
            RedBlock_LogSideGateDiagnostic("not_side", mapped_line_row, line_x, line_row,
                                           left_border, right_border, lane_ratio);
            return 0;
        }

        if(row_clamped != 0)
        {
            RedBlock_LogSideGateDiagnostic("row_clamp", mapped_line_row, line_x, line_row,
                                           left_border, right_border, lane_ratio);
        }

        if(redblock_side_ignore_log_count < REDBLOCK_SIDE_IGNORE_LOG_INTERVAL)
        {
            redblock_side_ignore_log_count++;
        }
        if(redblock_side_ignore_log_count >= REDBLOCK_SIDE_IGNORE_LOG_INTERVAL)
        {
            redblock_side_ignore_log_count = 0;
            printf("[RB_GATE] side_brick_ignore area=%.0f raw=(%d,%d) line=(%d,%d) lane=(%d,%d) ratio=%.3f\n",
                   redblock_area,
                   redblock_center_x,
                   redblock_center_y,
                   line_x,
                   line_row,
                   left_border,
                   right_border,
                   lane_ratio);
        }
        return 1;
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

    void RedBlock_RecordConfirmedClass(int coarse_index)
    {
        if(coarse_index < 0 || coarse_index >= MODEL_CLASS_COUNT)
        {
            return;
        }

        if(redblock_class_history_count >= REDBLOCK_CLASS_HISTORY_CAPACITY)
        {
            redblock_class_history_overflow = 1;
            return;
        }

        redblock_class_history[redblock_class_history_count] = (uint8)coarse_index;
        redblock_class_history_count++;
    }

    void RedBlock_ReportClassificationSequence(void)
    {
        uint8 index = 0;

        if(stop == 0 || redblock_class_history_reported != 0)
        {
            return;
        }

        redblock_class_history_reported = 1;
        printf("[RB_RESULT] coarse_sequence count=%u: ",
               (unsigned)redblock_class_history_count);
        if(redblock_class_history_count == 0)
        {
            printf("none");
        }
        else
        {
            for(index = 0; index < redblock_class_history_count; index++)
            {
                if(index > 0)
                {
                    printf(" -> ");
                }
                printf("%s", RedBlock_ModelClassLabel(redblock_class_history[index]));
            }
        }
        if(redblock_class_history_overflow != 0)
        {
            printf(" (超过%u条，后续未记录)",
                   (unsigned)REDBLOCK_CLASS_HISTORY_CAPACITY);
        }
        printf("\n");
    }

    void RedBlock_SetBypassPhase(RedBlockBypassPhase phase)
    {
        if(redblock_bypass_phase != phase)
        {
            printf("[RB_VIS] phase %u -> %u enc=%ld\n",
                   (unsigned)redblock_bypass_phase,
                   (unsigned)phase,
                   (long)encoder_acc_avg);

        }
        redblock_bypass_phase = phase;
        redblock_bypass_phase_flag = static_cast<uint8>(phase);
        redblock_bypass_phase_counter = 0;
    }

    void RedBlock_SetVisualPhase(RedBlockBypassPhase phase)
    {
        RedBlock_SetBypassPhase(phase);
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
        redblock_phase_start_encoder_avg = 0;
        redblock_recover_ready_count = 0;
        redblock_recover_diag_div = 0;
        redblock_visual_diag_div = 0;
        redblock_motion_dif_diag_div = 0;
        redblock_visual_err_diag_div = 0;
        RedBlock_ResetVisualTargetContext();
        redblock_slowdown_frame_count = 0;
        redblock_low_speed_settle_count = 0;
        redblock_bypass_speed_cmd = 0;
        redblock_slowdown_speed_cmd = REDBLOCK_SLOWDOWN_SPEED_CMD;
        redblock_restore_speed_cmd = 0;
        redblock_restore_speed_valid = 0;
        redblock_cooldown_count = 0;
        redblock_boundary_stop_grace_frames = 0;
        RedBlock_ResetModelVoting();
        RedBlock_SetBypassPhase(RB_BYPASS_PHASE_IDLE);
    }

    void RedBlock_SaveRestoreSpeed(void)
    {
        if(redblock_restore_speed_valid == 0)
        {
            redblock_restore_speed_cmd = set_speed;
            redblock_restore_speed_valid = 1;
            printf("[RB_MOT] save speed=%ld\n", (long)redblock_restore_speed_cmd);
        }
    }

    void RedBlock_RestoreSpeedIfNeeded(void)
    {
        if(redblock_restore_speed_valid != 0)
        {
            set_speed = redblock_restore_speed_cmd;
            printf("[RB_MOT] restore speed=%ld\n", (long)set_speed);
            redblock_restore_speed_valid = 0;
            redblock_restore_speed_cmd = 0;
        }
    }

    void RedBlock_StartCooldown(void)
    {
        redblock_cooldown_count = REDBLOCK_COOLDOWN_FRAMES;
        printf("[RB_DEC] cooldown start frames=%u\n", (unsigned)redblock_cooldown_count);
    }

    void RedBlock_TickCooldown(void)
    {
        if(redblock_cooldown_count > 0)
        {
            redblock_cooldown_count--;
            RedBlock_ClearDetectionResult();
            redblock_confirm_count = 0;
        }
    }

    void RedBlock_StartBoundaryGrace(void)
    {
        redblock_boundary_stop_grace_frames = REDBLOCK_FINISH_BOUNDARY_GRACE_FRAMES;
        printf("[RB_MOT] boundary grace start frames=%u\n",
               (unsigned)redblock_boundary_stop_grace_frames);
    }

    void RedBlock_ReturnToLineFollow(const char *reason)
    {
        printf("[RB_DEC] %s -> line_follow\n", reason);
        RedBlock_RestoreSpeedIfNeeded();
        RedBlock_ClearLocalState();
        RedBlock_StartCooldown();
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

    uint8 RedBlock_IsVisualBypassMode(void)
    {
        return (
            redblock_bypass_mode == RB_BYPASS_MODE_LEFT ||
            redblock_bypass_mode == RB_BYPASS_MODE_RIGHT
        );
    }

    const char *RedBlock_VisualTargetSourceLabel(void)
    {
        switch(redblock_visual_target_source)
        {
            case RB_VIS_TARGET_CURRENT:
                return "current";
            case RB_VIS_TARGET_CACHE:
                return "cache";
            case RB_VIS_TARGET_LOST:
            default:
                return "lost";
        }
    }

    uint8 RedBlock_GetCurrentBoundaryPoint(int row, int *boundary_point)
    {
        int point = 0;
        int lane_width = 0;
        const int first_searched_row = ClampInt(white_length_max[0], 0, Cut_ROW - 1);

        if(boundary_point == nullptr || row < first_searched_row || row >= Cut_ROW)
        {
            return 0;
        }

        lane_width = func_abs((int)r_border[row] - (int)l_border[row]);
        if(lane_width < REDBLOCK_VISUAL_BOUNDARY_MIN_WIDTH)
        {
            return 0;
        }

        if(redblock_bypass_mode == RB_BYPASS_MODE_LEFT)
        {
            if(l_effect_flag[row] == 0)
            {
                return 0;
            }
            point = (int)l_border[row];
        }
        else if(redblock_bypass_mode == RB_BYPASS_MODE_RIGHT)
        {
            if(r_effect_flag[row] == 0)
            {
                return 0;
            }
            point = (int)r_border[row];
        }
        else
        {
            return 0;
        }

        if(redblock_visual_boundary_cache_valid[row] != 0 &&
           func_abs(point - (int)redblock_visual_boundary_cache[row]) >
               REDBLOCK_VISUAL_BOUNDARY_MAX_FRAME_STEP)
        {
            return 0;
        }

        *boundary_point = ClampInt(point, SEARCH_MIN, SEARCH_MAX);
        return 1;
    }

    void RedBlock_UpdateVisualBoundaryCache(void)
    {
        int row = 0;

        for(row = 0; row < Cut_ROW; row++)
        {
            int boundary_point = 0;
            if(RedBlock_GetCurrentBoundaryPoint(row, &boundary_point) != 0)
            {
                redblock_visual_boundary_cache[row] = (uint8)boundary_point;
                redblock_visual_boundary_cache_valid[row] = 1;
            }
        }
    }

    int RedBlock_GetWindowCurrentValidRows(int start_row, int *roughness)
    {
        int i = 0;
        int valid_rows = 0;
        int continuity_cost = 0;
        int previous_point = 0;
        uint8 previous_valid = 0;

        for(i = 0; i < REDBLOCK_VISUAL_CONTROL_WINDOW; i++)
        {
            int boundary_point = 0;
            if(RedBlock_GetCurrentBoundaryPoint(start_row + i, &boundary_point) == 0)
            {
                previous_valid = 0;
                continue;
            }

            valid_rows++;
            if(previous_valid != 0)
            {
                continuity_cost += func_abs(boundary_point - previous_point);
            }
            previous_point = boundary_point;
            previous_valid = 1;
        }

        if(roughness != nullptr)
        {
            *roughness = continuity_cost;
        }
        return valid_rows;
    }

    int RedBlock_GetWindowCachedRows(int start_row)
    {
        int i = 0;
        int cached_rows = 0;

        for(i = 0; i < REDBLOCK_VISUAL_CONTROL_WINDOW; i++)
        {
            if(redblock_visual_boundary_cache_valid[start_row + i] != 0)
            {
                cached_rows++;
            }
        }
        return cached_rows;
    }

    uint8 RedBlock_HasUsableCachedWindow(void)
    {
        if(redblock_visual_control_row_valid == 0)
        {
            return 0;
        }

        const int control_row = ClampInt(
            redblock_visual_control_row,
            REDBLOCK_VISUAL_CONTROL_ROW_MIN,
            Cut_ROW - REDBLOCK_VISUAL_CONTROL_WINDOW);
        return RedBlock_GetWindowCachedRows(control_row) >=
            REDBLOCK_VISUAL_VALID_ROWS_REQUIRED;
    }

    int32_t RedBlock_ApplyBoundaryLossSpeedLimit(int32_t requested_speed)
    {
        if(redblock_bypass_phase == RB_BYPASS_PHASE_RECOVER)
        {
            return requested_speed;
        }
        if(redblock_visual_boundary_lost_frames > REDBLOCK_VISUAL_BOUNDARY_LOST_SLOW_FRAMES)
        {
            return RedBlock_HasUsableCachedWindow() != 0 ?
                REDBLOCK_VISUAL_BOUNDARY_LOST_SPEED_CMD : 0;
        }
        if(redblock_visual_boundary_lost_frames > REDBLOCK_VISUAL_BOUNDARY_CACHE_FRAMES)
        {
            return REDBLOCK_VISUAL_BOUNDARY_LOST_SPEED_CMD;
        }
        return requested_speed;
    }

    int RedBlock_GetVisualBlendPercent(void)
    {
        int blend_percent = 0;

        switch(redblock_bypass_phase)
        {
            case RB_BYPASS_PHASE_SEEK_BOUNDARY:
            case RB_BYPASS_PHASE_BOUNDARY_HOLD:
            case RB_BYPASS_PHASE_POST_PASS_HOLD:
                blend_percent = 100;
                break;

            case RB_BYPASS_PHASE_BLEND_BACK:
                {
                const int32_t blend_progress =
                    func_abs(encoder_acc_avg - redblock_phase_start_encoder_avg);
                const float blend_ratio = ClampFloat(
                    (float)blend_progress / (float)REDBLOCK_VISUAL_EXIT_HOLD_DISTANCE,
                    0.0f,
                    1.0f);
                const float smooth_ratio =
                    blend_ratio * blend_ratio * (3.0f - 2.0f * blend_ratio);
                blend_percent = (int)((1.0f - smooth_ratio) * 100.0f + 0.5f);
                }
                break;

            case RB_BYPASS_PHASE_RECOVER:
                blend_percent = 0;
                break;

            case RB_BYPASS_PHASE_IDLE:
            default:
                blend_percent = 0;
                break;
        }

        return ClampInt(blend_percent, 0, 100);
    }

    uint8 RedBlock_SelectVisualControlWindow(void)
    {
        int row = 0;
        int best_row = Search_Stop_Line;
        int best_score = -1;
        uint8 found = 0;
        const int min_row = ClampInt(
            white_length_max[0] > REDBLOCK_VISUAL_CONTROL_ROW_MIN ?
                white_length_max[0] : REDBLOCK_VISUAL_CONTROL_ROW_MIN,
            0,
            Cut_ROW - REDBLOCK_VISUAL_CONTROL_WINDOW);
        const int max_row = Cut_ROW - REDBLOCK_VISUAL_CONTROL_WINDOW;

        if(redblock_bypass_phase == RB_BYPASS_PHASE_RECOVER)
        {
            return 0;
        }

        for(row = min_row; row <= max_row; row++)
        {
            int roughness = 0;
            const int valid_rows = RedBlock_GetWindowCurrentValidRows(row, &roughness);
            if(valid_rows < REDBLOCK_VISUAL_VALID_ROWS_REQUIRED)
            {
                continue;
            }

            // 有效点数量优先，其次选择更连续且更靠近上一控制窗口的位置。
            const int reference_row = redblock_visual_control_row_valid != 0 ?
                redblock_visual_control_row : (int)Search_Stop_Line;
            const int score = valid_rows * 1000 - roughness * 4
                              - func_abs(row - reference_row) * 2;
            if(found == 0 || score > best_score)
            {
                best_score = score;
                best_row = row;
                found = 1;
            }
        }

        if(found != 0)
        {
            int selected_row = best_row;
            if(redblock_visual_control_row_valid != 0)
            {
                selected_row = ClampInt(
                    best_row,
                    (int)redblock_visual_control_row - REDBLOCK_VISUAL_CONTROL_ROW_STEP,
                    (int)redblock_visual_control_row + REDBLOCK_VISUAL_CONTROL_ROW_STEP);
            }
            selected_row = ClampInt(selected_row, min_row, max_row);
            Search_Stop_Line = (int8)selected_row;
            redblock_visual_control_row = (int8)selected_row;
            redblock_visual_control_row_valid = 1;
        }

        if(redblock_visual_control_row_valid != 0)
        {
            const int control_row = ClampInt(
                redblock_visual_control_row,
                REDBLOCK_VISUAL_CONTROL_ROW_MIN,
                Cut_ROW - REDBLOCK_VISUAL_CONTROL_WINDOW);
            const int current_valid_rows = RedBlock_GetWindowCurrentValidRows(control_row, nullptr);
            const int cached_rows = RedBlock_GetWindowCachedRows(control_row);
            redblock_visual_boundary_valid_rows = (uint8)current_valid_rows;

            if(current_valid_rows >= REDBLOCK_VISUAL_VALID_ROWS_REQUIRED)
            {
                redblock_visual_boundary_lost_frames = 0;
                redblock_visual_target_source = RB_VIS_TARGET_CURRENT;
            }
            else
            {
                if(redblock_visual_boundary_lost_frames < 255)
                {
                    redblock_visual_boundary_lost_frames++;
                }
                redblock_visual_target_source =
                    (cached_rows >= REDBLOCK_VISUAL_VALID_ROWS_REQUIRED &&
                     redblock_visual_boundary_lost_frames <= REDBLOCK_VISUAL_BOUNDARY_CACHE_FRAMES) ?
                        RB_VIS_TARGET_CACHE : RB_VIS_TARGET_LOST;
            }
        }
        else
        {
            if(redblock_visual_boundary_lost_frames < 255)
            {
                redblock_visual_boundary_lost_frames++;
            }
            redblock_visual_boundary_valid_rows = 0;
            redblock_visual_target_source = RB_VIS_TARGET_LOST;
        }

        return found;
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

uint8 RedBlock_IsBypassActive(void)
{
    return redblock_bypass_active_flag;
}

int32_t RedBlock_GetBypassSpeedCmd(void)
{
    return RedBlock_ApplyBoundaryLossSpeedLimit(redblock_bypass_speed_cmd);
}

uint8 RedBlock_ShouldIgnoreBoundaryStop(void)
{
    return (
        (redblock_state == RB_DEC_MOTION_ACTIVE && redblock_bypass_active_flag != 0) ||
        redblock_boundary_stop_grace_frames > 0
    );
}

uint8 RedBlock_IsElementExclusive(void)
{
    return (
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
        redblock_state == RB_DEC_LOW_SPEED_SETTLE ||
        redblock_state == RB_DEC_MODEL_RECOGNIZING ||
        (redblock_state == RB_DEC_MOTION_ACTIVE && redblock_bypass_active_flag != 0)
    );
}

int32_t RedBlock_GetMotionSpeedCmd(void)
{
    if(redblock_state == RB_DEC_MOTION_ACTIVE && redblock_bypass_active_flag != 0)
    {
        return RedBlock_ApplyBoundaryLossSpeedLimit(redblock_bypass_speed_cmd);
    }
    return redblock_slowdown_speed_cmd;
}

float RedBlock_GetMotionDifSpeed(void)
{
    const float raw_motion_dif = (float)Servo_PID(err_new);
    float motion_dif = raw_motion_dif;

    if(redblock_state == RB_DEC_MOTION_ACTIVE && redblock_bypass_active_flag != 0)
    {
        float dif_limit = REDBLOCK_VISUAL_RECOVER_DIF_LIMIT;
        if(redblock_bypass_phase == RB_BYPASS_PHASE_SEEK_BOUNDARY)
        {
            dif_limit = REDBLOCK_VISUAL_SEEK_DIF_LIMIT;
        }
        else if(redblock_bypass_phase == RB_BYPASS_PHASE_BOUNDARY_HOLD ||
                redblock_bypass_phase == RB_BYPASS_PHASE_POST_PASS_HOLD)
        {
            dif_limit = REDBLOCK_VISUAL_HOLD_DIF_LIMIT;
        }
        float active_dif_limit = dif_limit;

        if(redblock_bypass_phase != RB_BYPASS_PHASE_RECOVER)
        {
            if(redblock_visual_boundary_lost_frames > REDBLOCK_VISUAL_BOUNDARY_LOST_SLOW_FRAMES)
            {
                if(RedBlock_HasUsableCachedWindow() != 0)
                {
                    active_dif_limit = REDBLOCK_VISUAL_BOUNDARY_LOST_DIF_LIMIT;
                }
                else
                {
                    motion_dif = 0.0f;
                    active_dif_limit = 0.0f;
                }
            }
            else if(redblock_visual_boundary_lost_frames > REDBLOCK_VISUAL_BOUNDARY_CACHE_FRAMES)
            {
                active_dif_limit = REDBLOCK_VISUAL_BOUNDARY_LOST_DIF_LIMIT;
            }
        }
        motion_dif = func_limit_ab(motion_dif, -active_dif_limit, active_dif_limit);

        redblock_motion_dif_diag_div++;
        if(redblock_motion_dif_diag_div >= 10)
        {
            redblock_motion_dif_diag_div = 0;
            printf("[RB_VIS_DIF] phase=%u mode=%u err=%.2f raw=%.2f dif=%.2f limit=%.2f speed=%ld lost=%u\n",
                   redblock_bypass_phase_flag,
                   redblock_bypass_mode_flag,
                   err_new,
                   raw_motion_dif,
                   motion_dif,
                   active_dif_limit,
                   (long)RedBlock_ApplyBoundaryLossSpeedLimit(redblock_bypass_speed_cmd),
                   (unsigned)redblock_visual_boundary_lost_frames);
        }
    }

    return motion_dif;
}

void RedBlock_StartFallbackLeftBypass(const char *reason)
{
    printf("[RB_DEC] recognition_failed reason=%s -> left_bypass\n", reason);
    RedBlock_StartBypassMode(RB_BYPASS_MODE_LEFT);
}

    void RedBlock_StartActionForClass(int coarse_index)
    {
        RedBlock_RecordConfirmedClass(coarse_index);

        switch(coarse_index)
    {
        case MODEL_CLASS_VEHICLE:
            printf("[RB_DEC] voting_result vehicle -> line_follow\n");
            RedBlock_ReturnToLineFollow("vehicle_class");
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

    cv::Scalar lower_red1(0, 110, 70);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(170, 110, 70);
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
    if(red_pixel_count < 25)
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
        if(area < RED_MIN_AREA || area > RED_MAX_AREA)
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
        // 仅在红块尚未确认前过滤贴边红砖。进入低速/模型阶段后保留检测结果，
        // 防止真实红块在车辆靠近时因投影位置变化而被错误取消。
        if((redblock_state == RB_DEC_IDLE || redblock_state == RB_DEC_CONFIRMING) &&
           RedBlock_ShouldIgnoreSideBrick(frame_width, frame_height) != 0)
        {
            RedBlock_ClearDetectionResult();
            return;
        }
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
    RedBlock_SaveRestoreSpeed();
    redblock_slowdown_speed_cmd = REDBLOCK_SLOWDOWN_SPEED_CMD;
    redblock_confirm_count = 0;
    redblock_low_speed_settle_count = 0;
    RedBlock_ResetModelVoting();
    printf(
        "RedBlock confirmed: area=%.0f -> low_speed_settle %u frames speed=%ld\n",
        redblock_area,
        REDBLOCK_LOW_SPEED_SETTLE_FRAMES,
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
    RedBlock_RestoreSpeedIfNeeded();
    RedBlock_ClearLocalState();
}

void RedBlock_ResetState(void)
{
    redblock_pause_flag = 0;
    model_request_flag = 0;
    model_running_flag = 0;
    redblock_slowdown_frame_count = 0;
    RedBlock_RestoreSpeedIfNeeded();
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
    RedBlock_ReturnToLineFollow("start_bypass_default");
}

void RedBlock_StartBypassMode(RedBlockBypassMode mode)
{
    if(mode == RB_BYPASS_MODE_NONE || mode == RB_BYPASS_MODE_STRAIGHT)
    {
        RedBlock_ReturnToLineFollow("straight_mode");
        return;
    }

    if(stop != 0 || zebra_flag == 3 || pwm0_flag != 0)
    {
        printf("[RB_MOT] clear_stop_before_bypass stop=%u zebra=%u pwm0=%u\n",
               (unsigned)stop,
               (unsigned)zebra_flag,
               (unsigned)pwm0_flag);
    }
    RedBlock_SaveRestoreSpeed();
    redblock_pause_flag = 0;
    model_request_flag = 0;
    model_running_flag = 0;
    time1 = 0;
    timestop = 0;
    stop = 0;
    zebra_flag = 0;
    pwm0_flag = 0;
    redblock_bypass_mode = mode;
    redblock_bypass_mode_flag = static_cast<uint8>(mode);
    redblock_bypass_active_flag = RedBlock_IsVisualBypassMode();
    redblock_phase_start_encoder_avg = encoder_acc_avg;
    redblock_recover_ready_count = 0;
    redblock_visual_diag_div = 0;
    RedBlock_ResetVisualTargetContext();
    redblock_bypass_speed_cmd = REDBLOCK_VISUAL_SEEK_SPEED_CMD;
    RedBlock_SetVisualPhase(RB_BYPASS_PHASE_SEEK_BOUNDARY);
    RedBlock_SetState(RB_DEC_MOTION_ACTIVE);
    printf("[RB_VIS] start mode=%d centerline=boundary\n",
           redblock_bypass_mode_flag);
}

void RedBlock_FinishBypass(void)
{
    if(redblock_pause_flag)
    {
        RedBlock_ReleasePause();
        return;
    }
    printf("[RB_MOT] finish\n");
    RedBlock_RestoreSpeedIfNeeded();
    RedBlock_ClearLocalState();
    RedBlock_StartBoundaryGrace();
    RedBlock_StartCooldown();
}

uint8 RedBlock_ApplyBypass(void)
{
    const int32_t encoder_progress = func_abs(encoder_acc_avg - redblock_phase_start_encoder_avg);

    if(redblock_state != RB_DEC_MOTION_ACTIVE || redblock_bypass_active_flag == 0)
    {
        return 0;
    }

    if(RedBlock_IsVisualBypassMode() == 0)
    {
        RedBlock_FinishBypass();
        return 0;
    }

    switch(redblock_bypass_phase)
    {
        case RB_BYPASS_PHASE_SEEK_BOUNDARY:
            redblock_bypass_speed_cmd =
                encoder_progress >= REDBLOCK_VISUAL_SEEK_SOFT_DISTANCE ?
                    REDBLOCK_VISUAL_SEEK_SLOW_SPEED_CMD :
                    REDBLOCK_VISUAL_SEEK_SPEED_CMD;
            break;

        case RB_BYPASS_PHASE_BOUNDARY_HOLD:
        case RB_BYPASS_PHASE_POST_PASS_HOLD:
            redblock_bypass_speed_cmd = REDBLOCK_VISUAL_HOLD_SPEED_CMD;
            break;

        case RB_BYPASS_PHASE_BLEND_BACK:
        case RB_BYPASS_PHASE_RECOVER:
        case RB_BYPASS_PHASE_IDLE:
        default:
            redblock_bypass_speed_cmd = REDBLOCK_RECOVER_SPEED_CMD;
            break;
    }

    if(redblock_bypass_phase_counter < 255)
    {
        redblock_bypass_phase_counter++;
    }

    switch(redblock_bypass_phase)
    {
        case RB_BYPASS_PHASE_SEEK_BOUNDARY:
            if(redblock_visual_target_source == RB_VIS_TARGET_LOST &&
               redblock_visual_boundary_lost_frames >= REDBLOCK_VISUAL_SEEK_LOST_EXIT_FRAMES)
            {
                printf("[RB_VIS] seek_abort boundary_lost=%u -> line_follow\n",
                       (unsigned)redblock_visual_boundary_lost_frames);
                RedBlock_FinishBypass();
                break;
            }

            if(redblock_visual_target_source == RB_VIS_TARGET_CURRENT &&
               redblock_visual_boundary_valid_rows >= REDBLOCK_VISUAL_VALID_ROWS_REQUIRED &&
               redblock_visual_tangent_ready != 0 &&
               func_abs(err_new) <= REDBLOCK_VISUAL_SEEK_READY_ERR)
            {
                if(redblock_visual_seek_ready_count < 255)
                {
                    redblock_visual_seek_ready_count++;
                }
            }
            else
            {
                redblock_visual_seek_ready_count = 0;
            }

            if(encoder_progress >= REDBLOCK_VISUAL_SEEK_SOFT_DISTANCE &&
               redblock_visual_seek_slow_logged == 0)
            {
                redblock_visual_seek_slow_logged = 1;
                printf("[RB_VIS] seek_slow err=%.2f distance=%ld enc=%ld\n",
                       err_new,
                       (long)encoder_progress,
                       (long)encoder_acc_avg);
            }

            if(redblock_visual_seek_ready_count >= REDBLOCK_VISUAL_SEEK_READY_FRAMES)
            {
                printf("[RB_VIS] boundary_hold_start err=%.2f tangent_gap=%u power=%.1f ready=%u enc=%ld\n",
                       err_new,
                       (unsigned)redblock_visual_tangent_gap,
                       redblock_visual_tangent_power_applied,
                       (unsigned)redblock_visual_seek_ready_count,
                       (long)encoder_acc_avg);
                redblock_phase_start_encoder_avg = encoder_acc_avg;
                redblock_visual_seek_ready_count = 0;
                RedBlock_SetVisualPhase(RB_BYPASS_PHASE_BOUNDARY_HOLD);
            }
            break;

        case RB_BYPASS_PHASE_BOUNDARY_HOLD:
            if(encoder_progress >= REDBLOCK_VISUAL_HOLD_DISTANCE)
            {
                redblock_phase_start_encoder_avg = encoder_acc_avg;
                RedBlock_SetVisualPhase(RB_BYPASS_PHASE_POST_PASS_HOLD);
            }
            break;

        case RB_BYPASS_PHASE_POST_PASS_HOLD:
            if(encoder_progress >= REDBLOCK_VISUAL_POST_PASS_HOLD_DISTANCE &&
               redblock_bypass_phase_counter >= REDBLOCK_VISUAL_POST_PASS_HOLD_MIN_FRAMES)
            {
                redblock_phase_start_encoder_avg = encoder_acc_avg;
                RedBlock_SetVisualPhase(RB_BYPASS_PHASE_BLEND_BACK);
            }
            break;

        case RB_BYPASS_PHASE_BLEND_BACK:
            // 同时满足距离与帧数：距离保证已绕过目标，帧数保证 blend 已衰减至 0
            if(encoder_progress >= REDBLOCK_VISUAL_EXIT_HOLD_DISTANCE &&
               redblock_bypass_phase_counter >= REDBLOCK_VISUAL_EXIT_HOLD_MAX_FRAMES)
            {
                RedBlock_SetVisualPhase(RB_BYPASS_PHASE_RECOVER);
            }
            break;

        case RB_BYPASS_PHASE_RECOVER:
            {
            redblock_bypass_speed_cmd = REDBLOCK_RECOVER_SPEED_CMD;
            redblock_recover_diag_div++;
            if(redblock_recover_diag_div >= 10)
            {
                redblock_recover_diag_div = 0;
                printf(
                    "[RB_RECOVER] ready_cnt=%u err=%.2f far=%.2f effect(L,R)=(%d,%d) active=%u phase=%u\n",
                    redblock_recover_ready_count,
                    err_new,
                    Farthest_distance,
                    l_effect_num,
                    r_effect_num,
                    redblock_bypass_active_flag,
                    redblock_bypass_phase_flag
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
            else if(redblock_bypass_phase_counter >= REDBLOCK_VISUAL_RECOVER_MAX_FRAMES)
            {
                RedBlock_FinishBypass();
            }
            }
            break;

        case RB_BYPASS_PHASE_IDLE:
        default:
            RedBlock_SetVisualPhase(RB_BYPASS_PHASE_SEEK_BOUNDARY);
            break;
    }

    return 1;
}

uint8 RedBlock_ApplyVisualCenterline(void)
{
    int blend_percent = 0;
    int tangent_control_row = 0;
    int tangent_far_row = 0;
    int tangent_near_row = 0;
    float tangent_power = 0.0f;
    uint8 row = 0;
    uint8 changed_rows = 0;
    int first_target = -1;
    int last_target = -1;
    uint8 selected_window = 0;

    if(redblock_state != RB_DEC_MOTION_ACTIVE || redblock_bypass_active_flag == 0)
    {
        return 0;
    }

    if(RedBlock_IsVisualBypassMode() == 0)
    {
        return 0;
    }

    blend_percent = RedBlock_GetVisualBlendPercent();
    if(blend_percent <= 0 &&
       redblock_bypass_phase != RB_BYPASS_PHASE_BLEND_BACK)
    {
        return 0;
    }

    RedBlock_UpdateVisualBoundaryCache();
    selected_window = RedBlock_SelectVisualControlWindow();
    tangent_control_row = ClampInt(
        redblock_visual_control_row_valid != 0 ?
            redblock_visual_control_row : (int)Search_Stop_Line,
        REDBLOCK_VISUAL_CONTROL_ROW_MIN,
        Cut_ROW - REDBLOCK_VISUAL_CONTROL_WINDOW);
    tangent_far_row = ClampInt(
        tangent_control_row - REDBLOCK_VISUAL_TANGENT_FAR_OFFSET,
        0,
        Cut_ROW - 1);
    tangent_near_row = ClampInt(
        tangent_control_row + REDBLOCK_VISUAL_TANGENT_NEAR_OFFSET,
        tangent_far_row + 1,
        Cut_ROW - 1);
    tangent_power = ClampFloat(
        REDBLOCK_VISUAL_TANGENT_POWER,
        REDBLOCK_VISUAL_TANGENT_POWER_MIN,
        REDBLOCK_VISUAL_TANGENT_POWER_MAX);
    redblock_visual_tangent_power_applied = tangent_power;
    redblock_visual_tangent_ready = 0;
    redblock_visual_tangent_normal_center = 0;
    redblock_visual_tangent_boundary_center = 0;
    redblock_visual_tangent_target_center = 0;
    redblock_visual_tangent_gap = 0;

    for(row = 0; row < Cut_ROW; row++)
    {
        int boundary_center = Center_point[row];
        const int normal_center = Center_point[row];
        int current_boundary = 0;
        int target_center = normal_center;
        uint8 current_boundary_valid = 0;

        if(RedBlock_GetCurrentBoundaryPoint(row, &current_boundary) != 0)
        {
            boundary_center = current_boundary;
            current_boundary_valid = 1;
        }
        else if(redblock_visual_boundary_cache_valid[row] != 0)
        {
            boundary_center = (int)redblock_visual_boundary_cache[row];
        }

        boundary_center = ClampColIndex(boundary_center);
        target_center = boundary_center;

        if(redblock_bypass_phase == RB_BYPASS_PHASE_SEEK_BOUNDARY)
        {
            float tangent_progress = 0.0f;
            if((int)row <= tangent_far_row)
            {
                tangent_progress = 1.0f;
            }
            else if((int)row < tangent_near_row)
            {
                tangent_progress =
                    (float)(tangent_near_row - (int)row) /
                    (float)(tangent_near_row - tangent_far_row);
            }

            // 先平滑切向进度，使普通中线端和边线端的斜率均为零；p 仍控制靠边速度。
            const float smooth_progress = tangent_progress * tangent_progress *
                (3.0f - 2.0f * tangent_progress);
            const float boundary_blend =
                1.0f - powf(1.0f - smooth_progress, tangent_power);
            target_center = ClampColIndex((int)(
                (float)normal_center +
                ((float)boundary_center - (float)normal_center) * boundary_blend +
                0.5f));
        }

        if(blend_percent >= 100)
        {
            Center_point[row] = (uint8)target_center;
        }
        else
        {
            const int blended_center =
                (normal_center * (100 - blend_percent) + target_center * blend_percent) / 100;
            Center_point[row] = (uint8)ClampColIndex(blended_center);
        }
        Center_err[row] = Cut_COL / 2 - Center_point[row];

        if((int)row == tangent_control_row)
        {
            redblock_visual_tangent_normal_center = (uint8)normal_center;
            redblock_visual_tangent_boundary_center = (uint8)boundary_center;
            redblock_visual_tangent_target_center = (uint8)target_center;
            redblock_visual_tangent_gap = (uint8)func_abs(target_center - boundary_center);
            if(redblock_bypass_phase == RB_BYPASS_PHASE_SEEK_BOUNDARY &&
               current_boundary_valid != 0 &&
               redblock_visual_tangent_gap <= REDBLOCK_VISUAL_TANGENT_READY_GAP)
            {
                redblock_visual_tangent_ready = 1;
            }
        }

        if(first_target < 0)
        {
            first_target = Center_point[row];
        }
        last_target = Center_point[row];
        changed_rows++;
    }

    redblock_visual_diag_div++;
    if(redblock_visual_diag_div >= 10)
    {
        redblock_visual_diag_div = 0;
        printf("[RB_VIS] phase=%u mode=%u blend=%d rows=%u first=%d last=%d err=%.2f ctrl=%u sel=%u\n",
               redblock_bypass_phase_flag,
               redblock_bypass_mode_flag,
               blend_percent,
               changed_rows,
               first_target,
               last_target,
               err_new,
               (unsigned)Search_Stop_Line,
               selected_window);
        printf("[RB_VIS_TARGET] phase=%u mode=%u row=%u valid=%u/%u lost=%u source=%s blend=%d p=%.1f normal=%u boundary=%u target=%u gap=%u tangent_ready=%u\n",
               redblock_bypass_phase_flag,
               redblock_bypass_mode_flag,
               (unsigned)Search_Stop_Line,
               (unsigned)redblock_visual_boundary_valid_rows,
               (unsigned)REDBLOCK_VISUAL_CONTROL_WINDOW,
               (unsigned)redblock_visual_boundary_lost_frames,
               RedBlock_VisualTargetSourceLabel(),
               blend_percent,
               tangent_power,
               (unsigned)redblock_visual_tangent_normal_center,
               (unsigned)redblock_visual_tangent_boundary_center,
               (unsigned)redblock_visual_tangent_target_center,
               (unsigned)redblock_visual_tangent_gap,
               (unsigned)redblock_visual_tangent_ready);
    }

    return (changed_rows > 0) ? 1 : 0;
}

void RedBlock_LogVisualControl(float current_err)
{
    int row0 = Search_Stop_Line;
    int row1 = Search_Stop_Line + 9;

    if(redblock_state != RB_DEC_MOTION_ACTIVE || redblock_bypass_active_flag == 0)
    {
        return;
    }

    if(RedBlock_IsVisualBypassMode() == 0)
    {
        return;
    }

    row0 = ClampInt(row0, 0, Cut_ROW - 1);
    row1 = ClampInt(row1, 0, Cut_ROW - 1);

    redblock_visual_err_diag_div++;
    if(redblock_visual_err_diag_div < 10)
    {
        return;
    }
    redblock_visual_err_diag_div = 0;

    printf("[RB_VIS_ERR] phase=%u mode=%u err=%.2f ctrl=%d row=%d cp=%u l=%u r=%u row2=%d cp2=%u l2=%u r2=%u\n",
           redblock_bypass_phase_flag,
           redblock_bypass_mode_flag,
           current_err,
           (int)Search_Stop_Line,
           row0,
           Center_point[row0],
           l_border[row0],
           r_border[row0],
           row1,
           Center_point[row1],
           l_border[row1],
           r_border[row1]);
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
    const int width_based_side =
        static_cast<int>(block_width * RED_MODEL_ROI_WIDTH_SCALE);
    int roi_side = total_height;
    if(roi_side < width_based_side)
    {
        roi_side = width_based_side;
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

    if(redblock_cooldown_count > 0)
    {
        RedBlock_TickCooldown();
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
                    RedBlock_SaveRestoreSpeed();
                    RedBlock_ResetModelVoting();
                    printf("[RB_DEC] confirmed area=%.0f -> low_speed_settle %u frames speed=%ld\n",
                           redblock_area,
                           REDBLOCK_LOW_SPEED_SETTLE_FRAMES,
                           (long)redblock_slowdown_speed_cmd);
                    RedBlock_SetState(RB_DEC_LOW_SPEED_SETTLE);
                }
            }
            else
            {
                redblock_confirm_count = 0;
                RedBlock_SetState(RB_DEC_IDLE);
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

void RedBlock_Update(void)
{
    if(REDBLOCK_DETECTION_ENABLE == 0)
    {
        // 支持在动作进行中关闭开关，避免残留的低速或绕行状态影响普通巡线。
        if(redblock_state != RB_DEC_IDLE || redblock_bypass_active_flag != 0)
        {
            RedBlock_ResetState();
        }
        else
        {
            RedBlock_ClearDetectionResult();
        }
        return;
    }

    RedBlock_UpdatePerception();
    RedBlock_UpdateDecision();
    if(redblock_boundary_stop_grace_frames > 0 &&
       !(redblock_state == RB_DEC_MOTION_ACTIVE && redblock_bypass_active_flag != 0))
    {
        redblock_boundary_stop_grace_frames--;
    }
}

void RedBlock_ReportClassificationSequenceIfStopped(void)
{
    RedBlock_ReportClassificationSequence();
}
