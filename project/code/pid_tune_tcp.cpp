#include "pid_tune_tcp.h"

#include "control.h"
#include "camera.h"
#include "gyroscope.h"
#include "key.h"
#include "menu_settings.h"
#include "motor.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

extern volatile float dif_speed;
extern volatile int camera_flag;
extern int start_flag;

#if (1 == PID_TUNE_TCP_ENABLE)

namespace
{
    int g_pid_tune_socket = -1;
    uint8 g_pid_tune_connected = 0;
    char g_rx_line[128];
    uint16 g_rx_len = 0;
    uint8 g_stream_enabled = 1;
    uint8 g_stream_side = 0; // 0: left, 1: right
    uint8 g_tune_loop = 0; // 0: speed loop, 1: position loop
    uint32 g_report_tick = 0;

    int SetNonBlocking(int fd)
    {
        const int flags = fcntl(fd, F_GETFL, 0);
        if(flags < 0)
        {
            return -1;
        }
        return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    int ConnectWithTimeout(int fd, const sockaddr *addr, socklen_t addr_len, int timeout_ms)
    {
        if(SetNonBlocking(fd) < 0)
        {
            return -1;
        }

        const int ret = connect(fd, addr, addr_len);
        if(ret == 0)
        {
            return 0;
        }
        if(errno != EINPROGRESS)
        {
            return -1;
        }

        fd_set write_set;
        FD_ZERO(&write_set);
        FD_SET(fd, &write_set);

        timeval timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        const int select_ret = select(fd + 1, nullptr, &write_set, nullptr, &timeout);
        if(select_ret <= 0)
        {
            errno = (select_ret == 0) ? ETIMEDOUT : errno;
            return -1;
        }

        int socket_error = 0;
        socklen_t socket_error_len = sizeof(socket_error);
        if(getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_len) < 0)
        {
            return -1;
        }
        if(socket_error != 0)
        {
            errno = socket_error;
            return -1;
        }
        return 0;
    }

    void SendText(const char *text)
    {
        if(g_pid_tune_socket < 0 || text == nullptr)
        {
            return;
        }
        (void)send(g_pid_tune_socket, text, strlen(text), MSG_NOSIGNAL);
    }

    void SendStatus(void)
    {
        char tx[420];
        snprintf(
            tx,
            sizeof(tx),
            "# STATUS: go=%u camera=%d start=%d loop=%s side=%c stream=%u LKp=%.4f LKi=%.4f LA=%.4f RKp=%.4f RKi=%.4f RA=%.4f PosKp=%.4f PosKp2=%.4f PosKd=%.4f PosGkd=%.4f err=%.2f dif=%.2f gyro_z=%.2f\n",
            (unsigned)go_flag,
            (int)camera_flag,
            start_flag,
            g_tune_loop == 0 ? "SPEED" : "POS",
            g_stream_side == 0 ? 'L' : 'R',
            (unsigned)g_stream_enabled,
            (double)motor_l_kp,
            (double)motor_l_ki,
            (double)motor_l_filter_a,
            (double)motor_r_kp,
            (double)motor_r_ki,
            (double)motor_r_filter_a,
            (double)servo_pid_kp,
            (double)servo_pid_kp2,
            (double)servo_pid_kd,
            (double)servo_pid_gkd,
            (double)err_new,
            (double)dif_speed,
            (double)gyro_z
        );
        SendText(tx);
    }

    void SendGoState(void)
    {
        char tx[96];
        snprintf(
            tx,
            sizeof(tx),
            "# GO: go=%u camera=%d start=%d\n",
            (unsigned)go_flag,
            (int)camera_flag,
            start_flag
        );
        SendText(tx);
    }

    void SendStreamMode(void)
    {
        char tx[96];
        snprintf(
            tx,
            sizeof(tx),
            "# STREAM: loop=%s side=%c enabled=%u\n",
            g_tune_loop == 0 ? "SPEED" : "POS",
            g_stream_side == 0 ? 'L' : 'R',
            (unsigned)g_stream_enabled
        );
        SendText(tx);
    }

    void SendSetSuccess(void)
    {
        char tx[256];
        snprintf(
            tx,
            sizeof(tx),
            "修改成功，现在对应参数为：LKp=%.4f LKi=%.4f LA=%.4f RKp=%.4f RKi=%.4f RA=%.4f\n",
            (double)motor_l_kp,
            (double)motor_l_ki,
            (double)motor_l_filter_a,
            (double)motor_r_kp,
            (double)motor_r_ki,
            (double)motor_r_filter_a
        );
        SendText(tx);
    }

    uint8 ParseNamedSpeedPidValues(
        const char *line,
        float *lp,
        float *li,
        float *la,
        float *rp,
        float *ri,
        float *ra
    )
    {
        return (sscanf(line, "SET LKp:%f LKi:%f LA:%f RKp:%f RKi:%f RA:%f", lp, li, la, rp, ri, ra) == 6 ||
                sscanf(line, "SET LKP:%f LKI:%f LA:%f RKP:%f RKI:%f RA:%f", lp, li, la, rp, ri, ra) == 6);
    }

    uint8 ParseActiveSideSpeedPidValues(const char *line, float *p, float *i, float *d)
    {
        return (sscanf(line, "SET P:%f I:%f D:%f", p, i, d) == 3 ||
                sscanf(line, "SET KP:%f KI:%f KD:%f", p, i, d) == 3);
    }

    uint8 ParseNamedPositionPidValues(const char *line, float *kp, float *kp2, float *kd, float *gkd)
    {
        return (sscanf(line, "SET POS KP:%f KP2:%f KD:%f GKD:%f", kp, kp2, kd, gkd) == 4 ||
                sscanf(line, "SET POS Kp:%f Kp2:%f Kd:%f GKD:%f", kp, kp2, kd, gkd) == 4);
    }

    void ApplySetCommand(const char *line)
    {
        float lp = 0.0f;
        float li = 0.0f;
        float la = 0.0f;
        float rp = 0.0f;
        float ri = 0.0f;
        float ra = 0.0f;
        float pos_kp = 0.0f;
        float pos_kp2 = 0.0f;
        float pos_kd = 0.0f;
        float pos_gkd = 0.0f;

        if(ParseNamedPositionPidValues(line, &pos_kp, &pos_kp2, &pos_kd, &pos_gkd))
        {
            servo_pid_kp = pos_kp;
            servo_pid_kp2 = pos_kp2;
            servo_pid_kd = pos_kd;
            servo_pid_gkd = pos_gkd;
            SendSetSuccess();
            SendStatus();
            printf("[PID-TCP] set position loop params\r\n");
        }
        else if(ParseNamedSpeedPidValues(line, &lp, &li, &la, &rp, &ri, &ra))
        {
            motor_l_kp = lp;
            motor_l_ki = li;
            motor_l_filter_a = la;
            motor_r_kp = rp;
            motor_r_ki = ri;
            motor_r_filter_a = ra;
            SendSetSuccess();
            printf("[PID-TCP] set left/right speed loop params\r\n");
        }
        else if(ParseActiveSideSpeedPidValues(line, &lp, &li, &la))
        {
            if(g_tune_loop == 1)
            {
                servo_pid_kp = lp;
                servo_pid_kd = li;
                servo_pid_gkd = la;
                printf("[PID-TCP] set position loop kp/kd/gkd from tuner command\r\n");
            }
            else if(g_stream_side == 0)
            {
                motor_l_kp = lp;
                motor_l_ki = li;
                printf("[PID-TCP] set active left speed loop kp/ki from tuner command\r\n");
            }
            else
            {
                motor_r_kp = lp;
                motor_r_ki = li;
                printf("[PID-TCP] set active right speed loop kp/ki from tuner command\r\n");
            }
            SendSetSuccess();
            SendStatus();
        }
        else
        {
            SendText("# ERROR: expected SET P:<p> I:<i> D:<d> or SET POS KP:<kp> KP2:<kp2> KD:<kd> GKD:<gkd> or SET LKp:<kp> LKi:<ki> LA:<filter> RKp:<kp> RKi:<ki> RA:<filter>\n");
        }
    }

    void ApplySaveCommand(void)
    {
        if(MenuSettings_Save())
        {
            SendText("保存成功，当前速度环参数已写入 /home/root/smartcar_menu.cfg\n");
            SendStatus();
            printf("[PID-TCP] save settings ok\r\n");
        }
        else
        {
            SendText("# ERROR: save failed\n");
            printf("[PID-TCP] save settings failed\r\n");
        }
    }

    void ApplyGoCommand(uint8 enabled)
    {
        key_set_go_flag(enabled);
        SendText(enabled ? "发车成功，当前 go_flag=1\n" : "停车成功，当前 go_flag=0\n");
        SendGoState();
        printf("[PID-TCP] %s by command\r\n", enabled ? "go" : "stop");
    }

    void SendHelp(void)
    {
        SendText(
            "# HELP: STATUS | GO/START | STOP | SAVE | STREAM ON/OFF | LOOP SPEED/POS | SIDE L/R(sample and speed SET target) | SET P:<p> I:<i> D:<d> | SET POS KP:<kp> KP2:<kp2> KD:<kd> GKD:<gkd> | SET LKp:<kp> LKi:<ki> LA:<filter> RKp:<kp> RKi:<ki> RA:<filter>\n"
        );
    }

    void ApplyStreamCommand(const char *line)
    {
        if(strcasecmp(line, "STREAM ON") == 0)
        {
            g_stream_enabled = 1;
            SendStreamMode();
            return;
        }
        if(strcasecmp(line, "STREAM OFF") == 0)
        {
            g_stream_enabled = 0;
            SendStreamMode();
            return;
        }
        SendText("# ERROR: expected STREAM ON or STREAM OFF\n");
    }

    void ApplySideCommand(const char *line)
    {
        if(strcasecmp(line, "SIDE L") == 0 || strcasecmp(line, "SIDE LEFT") == 0)
        {
            g_stream_side = 0;
            SendStreamMode();
            return;
        }
        if(strcasecmp(line, "SIDE R") == 0 || strcasecmp(line, "SIDE RIGHT") == 0)
        {
            g_stream_side = 1;
            SendStreamMode();
            return;
        }
        SendText("# ERROR: expected SIDE L or SIDE R\n");
    }

    void ApplyLoopCommand(const char *line)
    {
        if(strcasecmp(line, "LOOP SPEED") == 0)
        {
            g_tune_loop = 0;
            SendStatus();
            return;
        }
        if(strcasecmp(line, "LOOP POS") == 0 || strcasecmp(line, "LOOP POSITION") == 0)
        {
            g_tune_loop = 1;
            SendStatus();
            return;
        }
        SendText("# ERROR: expected LOOP SPEED or LOOP POS\n");
    }

    char *TrimCommand(char *line)
    {
        if(line == nullptr)
        {
            return line;
        }

        while(*line == ' ' || *line == '\t')
        {
            line++;
        }

        char *end = line + strlen(line);
        while(end > line && (end[-1] == ' ' || end[-1] == '\t'))
        {
            end--;
        }
        *end = '\0';
        return line;
    }

    void HandleLine(const char *line)
    {
        if(line == nullptr || line[0] == '\0')
        {
            return;
        }

        char command[128];
        snprintf(command, sizeof(command), "%s", line);
        char *trimmed = TrimCommand(command);

        if(strcasecmp(trimmed, "STATUS") == 0)
        {
            SendStatus();
            return;
        }

        if(strcasecmp(trimmed, "SAVE") == 0)
        {
            ApplySaveCommand();
            return;
        }

        if(strcasecmp(trimmed, "GO") == 0 || strcasecmp(trimmed, "START") == 0)
        {
            ApplyGoCommand(1);
            return;
        }

        if(strcasecmp(trimmed, "STOP") == 0)
        {
            ApplyGoCommand(0);
            return;
        }

        if(strcasecmp(trimmed, "HELP") == 0)
        {
            SendHelp();
            return;
        }

        if(strncasecmp(trimmed, "STREAM ", 7) == 0)
        {
            ApplyStreamCommand(trimmed);
            return;
        }

        if(strncasecmp(trimmed, "SIDE ", 5) == 0)
        {
            ApplySideCommand(trimmed);
            return;
        }

        if(strncasecmp(trimmed, "LOOP ", 5) == 0)
        {
            ApplyLoopCommand(trimmed);
            return;
        }

        if(strncasecmp(trimmed, "SET ", 4) == 0)
        {
            ApplySetCommand(trimmed);
            return;
        }

        SendText("# ERROR: unknown command\n");
    }
}

#endif

uint8 pid_tune_tcp_init(void)
{
#if (1 == PID_TUNE_TCP_ENABLE)
    if(g_pid_tune_socket >= 0)
    {
        return g_pid_tune_connected;
    }

    g_pid_tune_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(g_pid_tune_socket < 0)
    {
        printf("[PID-TCP] socket failed\r\n");
        return 0;
    }

    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(PID_TUNE_TCP_SERVER_IP);
    server_addr.sin_port = htons(PID_TUNE_TCP_SERVER_PORT);

    printf("[PID-TCP] connect %s:%d\r\n", PID_TUNE_TCP_SERVER_IP, PID_TUNE_TCP_SERVER_PORT);
    if(ConnectWithTimeout(g_pid_tune_socket, (struct sockaddr *)&server_addr, sizeof(server_addr), 1000) < 0)
    {
        perror("[PID-TCP] connect failed");
        close(g_pid_tune_socket);
        g_pid_tune_socket = -1;
        g_pid_tune_connected = 0;
        return 0;
    }

    g_pid_tune_connected = 1;
    g_rx_len = 0;
    g_report_tick = 0;
    g_stream_enabled = 1;
    SendStatus();
    printf("[PID-TCP] connected\r\n");
    return 1;
#else
    return 0;
#endif
}

void pid_tune_tcp_update(void)
{
#if (1 == PID_TUNE_TCP_ENABLE)
    if(!g_pid_tune_connected || g_pid_tune_socket < 0)
    {
        return;
    }

    char rx[64];
    const int recv_len = recv(g_pid_tune_socket, rx, sizeof(rx), 0);
    if(recv_len == 0)
    {
        printf("[PID-TCP] peer closed\r\n");
        pid_tune_tcp_deinit();
        return;
    }
    if(recv_len < 0)
    {
        if(errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return;
        }
        perror("[PID-TCP] recv failed");
        pid_tune_tcp_deinit();
        return;
    }

    for(int i = 0; i < recv_len; i++)
    {
        const char ch = rx[i];
        if(ch == '\r')
        {
            continue;
        }
        if(ch == '\n')
        {
            g_rx_line[g_rx_len] = '\0';
            HandleLine(g_rx_line);
            g_rx_len = 0;
            continue;
        }

        if(g_rx_len < sizeof(g_rx_line) - 1)
        {
            g_rx_line[g_rx_len++] = ch;
        }
        else
        {
            g_rx_len = 0;
            SendText("# ERROR: command too long\n");
        }
    }
#endif
}

void pid_tune_tcp_report(void)
{
#if (1 == PID_TUNE_TCP_ENABLE)
    if(!g_pid_tune_connected || g_pid_tune_socket < 0 || !g_stream_enabled)
    {
        return;
    }

    g_report_tick++;
    if((g_report_tick % 4) != 0)
    {
        return;
    }

    const uint32 timestamp_ms = g_report_tick * 5;
    int32_t target = 0;
    int32_t input = 0;
    int32_t pwm = 0;
    int32_t error = 0;
    float kp = 0.0f;
    float ki = 0.0f;
    float d = 0.0f;

    if(g_tune_loop == 1)
    {
        target = 0;
        input = (int32_t)err_new;
        pwm = (int32_t)dif_speed;
        error = -input;
        kp = servo_pid_kp;
        ki = servo_pid_kd;
        d = servo_pid_gkd;
    }
    else
    {
        target = (g_stream_side == 0) ? l_speed : r_speed;
        input = (g_stream_side == 0) ? enconder_left : enconder_right;
        pwm = (g_stream_side == 0) ? l_out : r_out;
        error = target - input;
        kp = (g_stream_side == 0) ? motor_l_kp : motor_r_kp;
        ki = (g_stream_side == 0) ? motor_l_ki : motor_r_ki;
        d = 0.0f;
    }

    char tx[160];
    snprintf(
        tx,
        sizeof(tx),
        "%lu,%ld,%ld,%ld,%ld,%.4f,%.4f,%.4f\n",
        (unsigned long)timestamp_ms,
        (long)target,
        (long)input,
        (long)pwm,
        (long)error,
        (double)kp,
        (double)ki,
        (double)d
    );
    SendText(tx);
#endif
}

void pid_tune_tcp_deinit(void)
{
#if (1 == PID_TUNE_TCP_ENABLE)
    if(g_pid_tune_socket >= 0)
    {
        close(g_pid_tune_socket);
    }
    g_pid_tune_socket = -1;
    g_pid_tune_connected = 0;
    g_rx_len = 0;
#endif
}

uint8 pid_tune_tcp_is_connected(void)
{
#if (1 == PID_TUNE_TCP_ENABLE)
    return g_pid_tune_connected;
#else
    return 0;
#endif
}
