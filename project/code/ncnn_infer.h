#ifndef CODE_NCNN_INFER_H_
#define CODE_NCNN_INFER_H_

#include <string>

// ============================================================
// NCNN 模型推理使能开关
// 取消下行注释 → 编译并启用模型推理
// 保持注释     → 接口返回未就绪结果，不引入 OpenCV/NCNN 依赖
// ============================================================
#define ENABLE_NCNN_INFER

struct NCNN_Infer_Result
{
    int class_index;
    std::string label;
    float confidence;
    bool valid;
    bool ready;
};

// 初始化模型（只会真正加载一次）
bool ncnn_infer_init(void);

// 基于红块 ROI 执行一次推理
NCNN_Infer_Result ncnn_infer_run_once(void);

// 兼容旧入口：执行一次推理并打印结果
void ncnn_infer_run(void);

#endif /* CODE_NCNN_INFER_H_ */#ifndef CODE_NCNN_INFER_H_
#define CODE_NCNN_INFER_H_

#include <string>

// ============================================================
// NCNN 模型推理使能开关
// 取消下行注释 → 编译并启用模型推理
// 保持注释     → 接口返回未就绪结果，不引入 OpenCV/NCNN 依赖
// ============================================================
#define ENABLE_NCNN_INFER

struct NCNN_Infer_Result
{
    int class_index;        // 模型原始输出：细类 index
    int coarse_index;       // 映射后的粗类 index，控制逻辑只使用该字段
    std::string label;      // 粗类 label：suppliers / vehicle / weapon
    std::string fine_label; // 细类 label：如 suppliers/jijiubao
    float confidence;
    bool valid;
    bool ready;
};

// 初始化模型（只会真正加载一次）
bool ncnn_infer_init(void);

// 基于红块 ROI 执行一次推理
NCNN_Infer_Result ncnn_infer_run_once(void);

// 兼容旧入口：执行一次推理并打印结果
void ncnn_infer_run(void);

#endif /* CODE_NCNN_INFER_H_ */

