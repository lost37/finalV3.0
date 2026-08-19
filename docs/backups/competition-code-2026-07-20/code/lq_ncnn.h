/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-03-03
@修改内容：整理代码风格，与其他驱动库保持一致
@注意事项：注意查看路径的修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <ncnn/net.h>

struct LQ_NCNN_Result
{
    int class_index;
    std::string label;
    float confidence;
    bool valid;
};

/*!
 * @brief NCNN推理类
 * @details 用于加载NCNN模型并进行图像分类推理
 */
class LQ_NCNN
{
public:
    /*!
     * @brief   NCNN无参构造函数
     */
    LQ_NCNN();

    /*!
     * @brief   初始化NCNN模型
     * @return  true:成功 false:失败
     */
    bool Init();

    /*!
     * @brief   结构化推理函数
     * @param   bgr_image : 输入BGR图像
     * @return  推理结果（类别索引、标签、置信度、有效位）
     */
    LQ_NCNN_Result InferResult(const cv::Mat& bgr_image);

    /*!
     * @brief   兼容旧接口，返回类别名称或编号
     * @param   bgr_image : 输入BGR图像
     * @return  推理结果（类别名称或编号）
     */
    std::string Infer(const cv::Mat& bgr_image);

    /*!
     * @brief   设置模型路径
     * @param   param_path : 模型参数文件路径(.param)
     * @param   bin_path   : 模型权重文件路径(.bin)
     */
    void SetModelPath(const std::string& param_path, const std::string& bin_path);

    /*!
     * @brief   设置输入尺寸
     * @param   width  : 输入宽度
     * @param   height : 输入高度
     */
    void SetInputSize(int width, int height);

    /*!
     * @brief   设置类别标签
     * @param   labels : 类别名称列表
     */
    void SetLabels(const std::vector<std::string>& labels);

    /*!
     * @brief   设置归一化参数
     * @param   mean_vals  : 均值
     * @param   norm_vals : 标准差
     */
    void SetNormalize(const float mean_vals[3], const float norm_vals[3]);

    /*!
     * @brief   设置是否对输出执行Softmax
     * @param   enabled : true执行 false跳过
     */
    void SetApplySoftmax(bool enabled);

    /*!
     * @brief   设置是否启用FP16优化选项
     * @param   enabled : true启用 false关闭
     */
    void SetUseFp16(bool enabled);

    /*!
     * @brief   NCNN析构函数
     */
    ~LQ_NCNN();

private:
    /*!
     * @brief   获取最大概率类别索引与概率
     * @param   logits : 模型输出
     * @param   probability : 输出对应概率
     * @return  类别索引
     */
    int Argmax(const ncnn::Mat& logits, float *probability);

private:
    ncnn::Net                 m_net;              // NCNN网络
    std::vector<std::string>  m_labels;           // 类别标签
    bool                      m_initialized;      // 初始化标志
    bool                      m_apply_softmax;    // 是否执行Softmax
    bool                      m_use_fp16;         // 是否启用FP16优化

    // 模型路径
    std::string               m_param_path;
    std::string               m_bin_path;

    // 输入参数
    int                       m_input_width;
    int                       m_input_height;
    float                     m_mean_vals[3];
    float                     m_norm_vals[3];

    // 输入输出blob名称
    std::string               m_input_name;
    std::string               m_output_name;
};
