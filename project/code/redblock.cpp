#include "redblock.h"

#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "camera.h"
#include "zf_device_uvc.h"

extern uint8 redblock_pause_flag;
extern uint8 model_request_flag;
extern uint8 model_running_flag;
extern int time1;
extern int timestop;
extern uint8 pwm0_flag;

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

namespace
{
    // 每隔多少帧执行一次红块检测。
    // 数值越小：响应更快，但更吃性能，也更容易受瞬时噪声影响。
    // 数值越大：性能更稳，但触发会更慢。
    constexpr uint8 REDBLOCK_DETECT_INTERVAL = 3;

    // 连续命中多少次才认为红块成立。
    // 常用调参项：
    // 2 -> 响应更快；
    // 3 -> 更稳，更抗误检。
    constexpr uint8 REDBLOCK_CONFIRM_REQUIRED = 1;

    // 最小轮廓面积，过滤远处小红点和零散噪声。
    constexpr int RED_MIN_AREA = 190;

    // 形态学开闭运算核大小。
    // 增大：更能去噪，但可能吞掉小目标；
    // 减小：保留细节更多，但噪声也可能增多。
    constexpr int RED_KERNEL_SIZE = 5;

    // 外接矩形宽高比限制，避免把过细/过扁的区域当成红块。
    constexpr float RED_ASPECT_MIN = 0.7f;
    constexpr float RED_ASPECT_MAX = 5.0f;

    // 轮廓在外接矩形中的填充率下限。
    // 越大越严格，越能排除稀疏噪声。
    constexpr float RED_FILL_MIN = 0.50f;

    // 模型 ROI 改为“围绕红块生成动态方形框”。
    // 目标物体紧贴在红块上方，因此方形框需要同时容纳：
    // 1. 红块本体高度；
    // 2. 红块上方主体物；
    // 3. 红块底部少量留白。
    // 当前这组参数是按“更接近数据集构图”来收紧的：
    // - 方框整体更小，减少左右背景；
    // - 方框整体略微上移，让主体占比更大；
    // - 红块只保留少量底部区域，不再占太多面积。
    constexpr int RED_MODEL_ROI_MIN_SIDE = 80;
    constexpr int RED_MODEL_ROI_EDGE_PADDING = 2;
    constexpr float RED_MODEL_ROI_BOTTOM_MARGIN_RATIO = 0.08f;
    constexpr float RED_MODEL_ROI_OBJECT_HEIGHT_RATIO = 1.05f;
    constexpr float RED_MODEL_ROI_CENTER_UP_RATIO = 0.10f;

    // 红块检测区域改为用户指定的固定四点区域（320x240 坐标系）。
    // 用户原始给点顺序为：
    // (50,10)、(50,195)、(265,100)、(265,195)
    // 这里按顺时针顺序组织成四边形，便于后续生成 mask。
    constexpr int RED_SEARCH_POINT_LEFT_TOP_X = 50;
    constexpr int RED_SEARCH_POINT_LEFT_TOP_Y = 0;
    constexpr int RED_SEARCH_POINT_RIGHT_TOP_X = 265;
    constexpr int RED_SEARCH_POINT_RIGHT_TOP_Y = 0;
    constexpr int RED_SEARCH_POINT_RIGHT_BOTTOM_X = 265;
    constexpr int RED_SEARCH_POINT_RIGHT_BOTTOM_Y = 195;
    constexpr int RED_SEARCH_POINT_LEFT_BOTTOM_X = 50;
    constexpr int RED_SEARCH_POINT_LEFT_BOTTOM_Y = 195;

    constexpr uint8 REDBLOCK_BYPASS_APPROACH_FRAMES = 6;
    constexpr uint8 REDBLOCK_BYPASS_EXIT_HOLD_FRAMES = 8;
    constexpr uint8 REDBLOCK_BYPASS_RECOVER_FRAMES = 6;
    constexpr uint8 REDBLOCK_BYPASS_LOST_LIMIT = 4;
    constexpr int REDBLOCK_BYPASS_MIN_BLOCK_WIDTH = 8;
    constexpr int REDBLOCK_BYPASS_MIN_BLOCK_HEIGHT = 8;
    constexpr int REDBLOCK_BYPASS_ROW_EXTEND_UP = 6;
    constexpr int REDBLOCK_BYPASS_ROW_EXTEND_DOWN = 12;
    constexpr int REDBLOCK_BYPASS_INNER_MARGIN = 6;
    constexpr int REDBLOCK_BYPASS_OUTER_MARGIN = 3;
    constexpr int REDBLOCK_BYPASS_APPROACH_SHIFT = 8;

    RedBlockState redblock_state = RB_IDLE;
    uint8 redblock_detect_frame_counter = 0;
    RedBlockBypassMode redblock_bypass_mode = RB_BYPASS_MODE_NONE;
    RedBlockBypassPhase redblock_bypass_phase = RB_BYPASS_PHASE_IDLE;
    uint8 redblock_bypass_phase_counter = 0;
    uint8 redblock_bypass_lost_counter = 0;
    uint8 redblock_bypass_detect_valid = 0;
    uint8 redblock_bypass_last_row_top = 0;
    uint8 redblock_bypass_last_row_bottom = 0;
    uint8 redblock_bypass_last_col_left = 0;
    uint8 redblock_bypass_last_col_right = 0;

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

    void RedBlock_SetState(RedBlockState state)
    {
        redblock_state = state;
        redblock_state_flag = static_cast<uint8>(state);
    }

    void RedBlock_SetBypassPhase(RedBlockBypassPhase phase)
    {
        redblock_bypass_phase = phase;
        redblock_bypass_phase_flag = static_cast<uint8>(phase);
        redblock_bypass_phase_counter = 0;
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
        RedBlock_SetBypassPhase(RB_BYPASS_PHASE_IDLE);
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

uint8 RedBlock_ShouldIgnoreBoundaryStop(void)
{
    return (redblock_state == RB_BYPASS && redblock_bypass_active_flag != 0);
}

uint8 RedBlock_IsElementExclusive(void)
{
    return (
        redblock_state == RB_PAUSED ||
        redblock_state == RB_MODEL_WAIT ||
        redblock_state == RB_CONFIRMED ||
        (redblock_state == RB_BYPASS && redblock_bypass_active_flag != 0)
    );
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
        redblock_pause_flag = 1;
        model_request_flag = 1;
        redblock_confirm_count = 0;
        printf("RedBlock confirmed: area=%.0f, request model\n", redblock_area);
    }
    RedBlock_SetState(RB_PAUSED);
}

void RedBlock_ReleasePause(void)
{
    if(redblock_pause_flag == 1)
    {
        printf("RedBlock_ReleasePause: 红色色块检测已释放，恢复行驶\n");
    }
    redblock_pause_flag = 0;
    model_request_flag = 0;
    model_running_flag = 0;
    time1 = 0;
    timestop = 0;
    pwm0_flag = 0;
    RedBlock_ClearLocalState();
}

void RedBlock_ResetState(void)
{
    redblock_pause_flag = 0;
    model_request_flag = 0;
    model_running_flag = 0;
    RedBlock_ClearLocalState();
}

void RedBlock_OnModelStarted(void)
{
    model_running_flag = 1;
    if(redblock_state == RB_PAUSED || redblock_state == RB_MODEL_WAIT)
    {
        RedBlock_SetState(RB_MODEL_WAIT);
    }
}

void RedBlock_OnModelConfirmed(void)
{
    if(redblock_state == RB_PAUSED || redblock_state == RB_MODEL_WAIT)
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
    if(redblock_state == RB_CONFIRMED)
    {
        redblock_pause_flag = 0;
        model_request_flag = 0;
        model_running_flag = 0;
        time1 = 0;
        timestop = 0;
        pwm0_flag = 0;
        redblock_bypass_mode = mode;
        redblock_bypass_mode_flag = static_cast<uint8>(mode);
        redblock_bypass_active_flag = (mode != RB_BYPASS_MODE_NONE) ? 1 : 0;
        redblock_bypass_lost_counter = 0;
        redblock_bypass_detect_valid = 0;
        RedBlock_SetBypassPhase(RB_BYPASS_PHASE_APPROACH);
        RedBlock_SetState(RB_BYPASS);
        printf("RedBlock bypass start: mode=%d\n", redblock_bypass_mode_flag);
    }
}

void RedBlock_FinishBypass(void)
{
    if(redblock_pause_flag)
    {
        RedBlock_ReleasePause();
        return;
    }
    printf("RedBlock bypass finish\n");
    RedBlock_ClearLocalState();
}

uint8 RedBlock_ApplyBypass(void)
{
    uint8 applied = 0;

    if(redblock_state != RB_BYPASS || redblock_bypass_active_flag == 0)
    {
        return 0;
    }

    RedBlock_UpdateProjectedBox();

    if(redblock_bypass_detect_valid)
    {
        redblock_bypass_lost_counter = 0;
    }
    else if(redblock_bypass_lost_counter < 255)
    {
        redblock_bypass_lost_counter++;
    }

    switch(redblock_bypass_phase)
    {
        case RB_BYPASS_PHASE_APPROACH:
            if(redblock_bypass_detect_valid)
            {
                applied = RedBlock_ApplyBypassByMode(1);
            }
            redblock_bypass_phase_counter++;
            if(redblock_bypass_phase_counter >= REDBLOCK_BYPASS_APPROACH_FRAMES)
            {
                RedBlock_SetBypassPhase(RB_BYPASS_PHASE_COMMIT);
                printf("RedBlock bypass phase -> COMMIT\n");
            }
            break;

        case RB_BYPASS_PHASE_COMMIT:
            if(redblock_bypass_detect_valid)
            {
                applied = RedBlock_ApplyBypassByMode(0);
            }
            if(redblock_bypass_lost_counter >= REDBLOCK_BYPASS_LOST_LIMIT)
            {
                RedBlock_SetBypassPhase(RB_BYPASS_PHASE_EXIT_HOLD);
                printf("RedBlock bypass phase -> EXIT_HOLD\n");
            }
            break;

        case RB_BYPASS_PHASE_EXIT_HOLD:
            if(redblock_bypass_detect_valid)
            {
                applied = RedBlock_ApplyBypassByMode(0);
            }
            redblock_bypass_phase_counter++;
            if(redblock_bypass_phase_counter >= REDBLOCK_BYPASS_EXIT_HOLD_FRAMES)
            {
                RedBlock_SetBypassPhase(RB_BYPASS_PHASE_RECOVER);
                printf("RedBlock bypass phase -> RECOVER\n");
            }
            break;

        case RB_BYPASS_PHASE_RECOVER:
            redblock_bypass_phase_counter++;
            if(redblock_bypass_phase_counter >= REDBLOCK_BYPASS_RECOVER_FRAMES)
            {
                RedBlock_FinishBypass();
            }
            break;

        default:
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

void RedBlock_Update(void)
{
    if(redblock_detect_frame_counter != 0)
    {
        redblock_detect_frame_counter = (redblock_detect_frame_counter + 1) % REDBLOCK_DETECT_INTERVAL;
        if(redblock_state == RB_PAUSED)
        {
            RedBlock_SetState(RB_MODEL_WAIT);
        }
        return;
    }

    RedBlock_Detect();
    redblock_detect_frame_counter = (redblock_detect_frame_counter + 1) % REDBLOCK_DETECT_INTERVAL;

    if(redblock_state == RB_PAUSED)
    {
        RedBlock_SetState(RB_MODEL_WAIT);
        return;
    }

    if(redblock_state == RB_MODEL_WAIT || redblock_state == RB_CONFIRMED || redblock_state == RB_BYPASS)
    {
        return;
    }

    switch(redblock_state)
    {
        case RB_IDLE:
            if(redblock_flag)
            {
                redblock_confirm_count = 1;
                RedBlock_SetState(RB_CONFIRMING);
            }
            break;

        case RB_CONFIRMING:
            if(redblock_flag)
            {
                if(redblock_confirm_count < REDBLOCK_CONFIRM_REQUIRED)
                {
                    redblock_confirm_count++;
                }
                if(redblock_confirm_count >= REDBLOCK_CONFIRM_REQUIRED)
                {
                    RedBlock_RequestPause();
                }
            }
            else
            {
                redblock_confirm_count = 0;
                RedBlock_SetState(RB_IDLE);
            }
            break;

        default:
            break;
    }
}
