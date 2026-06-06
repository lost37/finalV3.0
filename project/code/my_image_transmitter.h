#ifndef CODE_MY_IMAGE_TRANSMITTER_H_
#define CODE_MY_IMAGE_TRANSMITTER_H_

#include "zf_common_typedef.h"

/*
 * my_image_transmitter 模块说明
 * 1. 该模块只负责“调试图传”，不参与主控决策。
 * 2. 图传关闭或连接失败时，不应影响相机主处理链和电机控制链。
 * 3. 后期比赛时，如需释放性能，优先将总开关设为 0。
 */

/* 调试图传总开关
 * 1: 启用图传初始化与发送
 * 0: 编译期直接关闭整条图传调试链
 */
#define MY_IMAGE_TRANSMITTER_ENABLE 0

/* 调试图传模式
 * FULL_GRAY : 发送整幅灰度图，并叠加红色块搜索框/识别框/ROI
 * TRACK_GRAY: 发送巡线算法直接使用的裁剪灰度图
 * EDGE_GRAY : 发送 Canny 边缘裁剪图，用于观察边界和轮廓效果
 */
#define MY_IMAGE_TRANSMITTER_MODE_FULL_GRAY  0
#define MY_IMAGE_TRANSMITTER_MODE_TRACK_GRAY 1
#define MY_IMAGE_TRANSMITTER_MODE_EDGE_GRAY  2

/* 默认模式建议使用 TRACK_GRAY
 * 原因：最贴近主巡线输入，且传输与处理开销较低
 */
#define MY_IMAGE_TRANSMITTER_MODE MY_IMAGE_TRANSMITTER_MODE_TRACK_GRAY

/* 上位机网络配置
 * 如果后续更换热点、电脑或端口，只改这里即可
 */
#define MY_IMAGE_TRANSMITTER_SERVER_IP "192.168.43.9"
#define MY_IMAGE_TRANSMITTER_PORT      8086

/* 串口/终端状态打印开关
 * 1: 打印 connected / connect failed
 * 0: 静默运行，减少调试输出干扰
 */
#define MY_IMAGE_TRANSMITTER_PRINT_STATUS 1

uint8 my_image_transmitter_init(void);
void my_image_transmitter_send(void);
void my_image_transmitter_deinit(void);

#endif
