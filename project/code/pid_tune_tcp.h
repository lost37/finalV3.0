#ifndef CODE_PID_TUNE_TCP_H_
#define CODE_PID_TUNE_TCP_H_

#include "zf_common_typedef.h"

/* PID TCP 调试开关。
 * 1: 启用独立 TCP 调参通道
 * 0: 编译保留接口但不连接
 */
#define PID_TUNE_TCP_ENABLE 0

/* 上位机地址。先和图传电脑 IP 保持一致，端口独立。 */
#define PID_TUNE_TCP_SERVER_IP "192.168.43.9"
#define PID_TUNE_TCP_SERVER_PORT 9091

uint8 pid_tune_tcp_init(void);
void pid_tune_tcp_update(void);
void pid_tune_tcp_report(void);
void pid_tune_tcp_deinit(void);
uint8 pid_tune_tcp_is_connected(void);

#endif /* CODE_PID_TUNE_TCP_H_ */
