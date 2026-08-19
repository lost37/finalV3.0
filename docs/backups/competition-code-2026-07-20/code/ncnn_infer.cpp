#include "ncnn_infer.h"

#ifdef ENABLE_NCNN_INFER

#include "lq_ncnn.h"
#include "redblock.h"

#include <cstdio>
#include <opencv2/imgcodecs.hpp>

namespace
{
    constexpr char MODEL_PARAM_PATH[] = "tiny_classifier_fp32.ncnn.param";
    constexpr char MODEL_BIN_PATH[] = "tiny_classifier_fp32.ncnn.bin";
    constexpr int MODEL_INPUT_WIDTH = 96;
    constexpr int MODEL_INPUT_HEIGHT = 96;
    // 调试开关：设为 1 时保存模型最终接收的 96x96 BGR 输入图；比赛时保持 0，避免磁盘写入影响实时性。
    constexpr uint8 MODEL_INPUT_SAVE_ENABLE = 1;
    // 调参：单次程序运行最多保存的模型输入图数量。达到上限后停止写盘。
    constexpr uint8 MODEL_INPUT_SAVE_MAX_COUNT = 8;
    constexpr char MODEL_INPUT_SAVE_FILE_PREFIX[] = "redblock_model_input";

    // 顺序必须与训练产物 artifacts/labels.txt 完全一致。
    constexpr const char* MODEL_FINE_LABELS[] =
    {
        "suppliers/jijiubao",
        "suppliers/wangyuan",
        "vehicle/jiuhu",
        "vehicle/zhuangjia",
        "weapon/buqiang",
        "weapon/shouliu",
        "weapon/shouqiang"
    };
    constexpr const char* MODEL_COARSE_LABELS[] =
    {
        "suppliers",
        "vehicle",
        "weapon"
    };
    // fine index -> coarse index：suppliers / vehicle / weapon。
    constexpr int MODEL_FINE_TO_COARSE[] =
    {
        0, 0,
        1, 1,
        2, 2, 2
    };
    constexpr int MODEL_FINE_LABEL_COUNT = sizeof(MODEL_FINE_LABELS) / sizeof(MODEL_FINE_LABELS[0]);
    constexpr int MODEL_COARSE_LABEL_COUNT = sizeof(MODEL_COARSE_LABELS) / sizeof(MODEL_COARSE_LABELS[0]);
    constexpr int MODEL_FINE_TO_COARSE_COUNT = sizeof(MODEL_FINE_TO_COARSE) / sizeof(MODEL_FINE_TO_COARSE[0]);
    static_assert(MODEL_FINE_LABEL_COUNT == MODEL_FINE_TO_COARSE_COUNT, "fine label count must match mapping count");

    LQ_NCNN g_ncnn;
    bool g_ncnn_initialized = false;
    bool g_ncnn_init_failed = false;
    uint8 g_model_input_save_count = 0;
    uint8 g_model_input_save_limit_logged = 0;

    void SaveModelInputDebugImage(const cv::Mat &input_bgr)
    {
        if(MODEL_INPUT_SAVE_ENABLE == 0)
        {
            return;
        }

        if(g_model_input_save_count >= MODEL_INPUT_SAVE_MAX_COUNT)
        {
            if(g_model_input_save_limit_logged == 0)
            {
                g_model_input_save_limit_logged = 1;
                printf("[NCNN_DEBUG] model input save limit=%u reached\n",
                       (unsigned)MODEL_INPUT_SAVE_MAX_COUNT);
            }
            return;
        }

        char file_name[64] = {0};
        snprintf(file_name, sizeof(file_name), "%s_%02u.png",
                 MODEL_INPUT_SAVE_FILE_PREFIX,
                 (unsigned)g_model_input_save_count);
        const bool saved = cv::imwrite(file_name, input_bgr);
        printf("[NCNN_DEBUG] model_input file=%s saved=%u\n",
               file_name,
               saved ? 1U : 0U);
        g_model_input_save_count++;
    }

    NCNN_Infer_Result MakeDefaultResult(void)
    {
        NCNN_Infer_Result result;
        result.class_index = -1;
        result.coarse_index = -1;
        result.label = "None";
        result.fine_label = "None";
        result.confidence = 0.0f;
        result.valid = false;
        result.ready = false;
        return result;
    }
}

bool ncnn_infer_init(void)
{
    if(g_ncnn_initialized)
    {
        return true;
    }
    if(g_ncnn_init_failed)
    {
        return false;
    }

    float mean_vals[3] = {123.675f, 116.28f, 103.53f};
    float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};
    std::vector<std::string> labels(MODEL_FINE_LABELS, MODEL_FINE_LABELS + MODEL_FINE_LABEL_COUNT);

    g_ncnn.SetModelPath(MODEL_PARAM_PATH, MODEL_BIN_PATH);
    g_ncnn.SetInputSize(MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT);
    g_ncnn.SetLabels(labels);
    g_ncnn.SetNormalize(mean_vals, norm_vals);
    g_ncnn.SetApplySoftmax(true);
    g_ncnn.SetUseFp16(false);

    if(!g_ncnn.Init())
    {
        g_ncnn_init_failed = true;
        printf("NCNN infer init failed\n");
        return false;
    }

    g_ncnn_initialized = true;
    printf("NCNN infer init success\n");
    return true;
}

NCNN_Infer_Result ncnn_infer_run_once(void)
{
    NCNN_Infer_Result result = MakeDefaultResult();
    result.ready = ncnn_infer_init();
    if(!result.ready)
    {
        return result;
    }

    std::vector<unsigned char> input_buffer(
        static_cast<size_t>(MODEL_INPUT_WIDTH) * static_cast<size_t>(MODEL_INPUT_HEIGHT) * 3
    );
    if(RedBlock_PrepareModelInput(input_buffer.data(), MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT) == 0)
    {
        return result;
    }

    cv::Mat input_bgr(MODEL_INPUT_HEIGHT, MODEL_INPUT_WIDTH, CV_8UC3, input_buffer.data());
    SaveModelInputDebugImage(input_bgr);
    const LQ_NCNN_Result infer_result = g_ncnn.InferResult(input_bgr);

    result.class_index = infer_result.class_index;
    result.fine_label = infer_result.label;
    result.confidence = infer_result.confidence;
    result.valid = infer_result.valid;
    if(result.valid &&
       result.class_index >= 0 &&
       result.class_index < MODEL_FINE_LABEL_COUNT)
    {
        result.coarse_index = MODEL_FINE_TO_COARSE[result.class_index];
        if(result.coarse_index >= 0 &&
           result.coarse_index < MODEL_COARSE_LABEL_COUNT)
        {
            result.label = MODEL_COARSE_LABELS[result.coarse_index];
        }
        else
        {
            result.label = "Unknown";
            result.valid = false;
        }
    }
    else
    {
        result.label = "Unknown";
        result.valid = false;
    }
    return result;
}

void ncnn_infer_run(void)
{
    const NCNN_Infer_Result result = ncnn_infer_run_once();
    if(!result.ready)
    {
        printf("NCNN infer not ready\n");
        return;
    }
    if(!result.valid)
    {
        printf("NCNN infer skipped: roi unavailable\n");
        return;
    }

    printf(
        "NCNN infer: fine=%d %s coarse=%d %s confidence=%.4f\n",
        result.class_index,
        result.fine_label.c_str(),
        result.coarse_index,
        result.label.c_str(),
        result.confidence
    );
}

#else

namespace
{
    NCNN_Infer_Result MakeDefaultResult(void)
    {
        NCNN_Infer_Result result;
        result.class_index = -1;
        result.coarse_index = -1;
        result.label = "None";
        result.fine_label = "None";
        result.confidence = 0.0f;
        result.valid = false;
        result.ready = false;
        return result;
    }
}

bool ncnn_infer_init(void)
{
    return false;
}

NCNN_Infer_Result ncnn_infer_run_once(void)
{
    return MakeDefaultResult();
}

void ncnn_infer_run(void)
{
}

#endif
