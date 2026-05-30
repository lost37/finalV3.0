#include "ncnn_infer.h"

#ifdef ENABLE_NCNN_INFER

#include "lq_ncnn.h"
#include "redblock.h"

#include <cstdio>

namespace
{
    constexpr char MODEL_PARAM_PATH[] = "tiny_classifier_fp32.ncnn.param";
    constexpr char MODEL_BIN_PATH[] = "tiny_classifier_fp32.ncnn.bin";
    constexpr int MODEL_INPUT_WIDTH = 96;
    constexpr int MODEL_INPUT_HEIGHT = 96;

    constexpr const char* MODEL_LABELS[] =
    {
        "supplies",
        "vehicle",
        "weapon"
    };
    constexpr int MODEL_LABEL_COUNT = sizeof(MODEL_LABELS) / sizeof(MODEL_LABELS[0]);

    LQ_NCNN g_ncnn;
    bool g_ncnn_initialized = false;
    bool g_ncnn_init_failed = false;

    NCNN_Infer_Result MakeDefaultResult(void)
    {
        NCNN_Infer_Result result;
        result.class_index = -1;
        result.label = "None";
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
    std::vector<std::string> labels(MODEL_LABELS, MODEL_LABELS + MODEL_LABEL_COUNT);

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
    const LQ_NCNN_Result infer_result = g_ncnn.InferResult(input_bgr);

    result.class_index = infer_result.class_index;
    result.label = infer_result.label;
    result.confidence = infer_result.confidence;
    result.valid = infer_result.valid;
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
        "NCNN infer: class=%d label=%s confidence=%.4f\n",
        result.class_index,
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
        result.label = "None";
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
