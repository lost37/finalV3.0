#include "zf_common_headfile.h"
#include "my_image_transmitter.h"
#include "menu_app.h"
#include "pid_tune_tcp.h"
#include "zebra.h"

volatile int dec = 0;
volatile int motor_mode = 1;   // 0: 开环, 1: 闭环
volatile float dif_speed = 0;
// 这里只表示“相机链是否已就绪”，不再拿它表示图传是否连接成功。
// 这样后面即使关闭图传，也不会影响整车主控链启动。
volatile int camera_flag = 0;
int times_start = 0;
int start_flag = 0;

void imu_isr()
{
    get_gyro();
}

void encoder_feedback_task()
{
    /* 独立编码器反馈任务：
     * 当前先保持与电机闭环相同的 5ms 周期，
     * 先验证“任务独立”本身，再考虑后续继续提频。
     */
    Get_speed();
}

void motor_isr()
{
    // 发车门控仍然放在定时中断里：
    // go_flag 负责总启动，camera_flag 负责相机链就绪，start_flag 负责延时发车。
    if(go_flag && camera_flag && start_flag)
    {
        if(motor_mode == 0)
        {
            Motor_test();
        }
        else
        {
            dianji_control();
        }
    }
    else
    {
        pwm_set_duty(MOTOR1_PWM, 0);
        pwm_set_duty(MOTOR2_PWM, 0);
        go_delay_counter = 0;
    }
}

void show_isr()
{
    if(++times_start >= 2)
    {
        start_flag = 1;
    }
}

void sigint_handler(int signum)
{
    (void)signum;
    printf("收到 Ctrl+C, 程序即将退出\n");
    go_flag = 0;
    exit(0);
}

void cleanup()
{
    printf("程序退出, 执行清理操作\n");
    gpio_set_level(BEEP, 0x00);
    pwm_set_duty(MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR2_PWM, 0);
    my_image_transmitter_deinit();
    pid_tune_tcp_deinit();
}

int main(int, char**)
{
    atexit(cleanup);
    signal(SIGINT, sigint_handler);

#ifdef MENU_ONLY_TEST
    printf("MENU_ONLY_TEST enabled: only IPS200 menu and keys will run.\n");
    motor_init();
    MenuApp_Init();
    while(1)
    {
        key_operate();
        MenuApp_DrawActiveDisplay();
        system_delay_ms(20);
    }
    return 0;
#endif

    // 底层驱动初始化：先电机，再陀螺仪。
    motor_init();
#ifdef DISABLE_GYRO
    servo_pid_gkd = 0.0f;
    printf("DISABLE_GYRO enabled: skip gyroscope init/sample, GKD=0\n");
#else
    gyroscope_init();
#endif

    // 控制节拍：
    // 5ms 编码器反馈，5ms 电机闭环，500ms 发车延时，5ms 陀螺仪采样。
    pit_ms_init(2, encoder_feedback_task);
    pit_ms_init(5, motor_isr);
    pit_ms_init(500, show_isr);
#ifndef DISABLE_GYRO
    pit_ms_init(5, imu_isr);
#endif

    go_init();

    // 相机初始化成功后，主控制链才允许进入运行态。
    if(uvc_camera_init("/dev/video0") < 0)
    {
        printf("摄像头初始化失败\r\n");
        return -1;
    }

    camera_flag = 1;

    // 图传只用于调试。
    // 即使这里连接失败，主控链也仍然可以继续运行。
    my_image_transmitter_init();
    pid_tune_tcp_init();
    MenuApp_Init();

    while(1)
    {
        key_operate();

        if(wait_image_refresh() < 0)
        {
            printf("未获取到图像帧\r\n");
            exit(0);
        }

        // 图像主处理链：原始图 -> 裁剪/边缘 -> 中线/元素 -> err_new
        Camera_Function();
        RedBlock_ReportClassificationSequenceIfStopped();

        static uint8 menu_draw_div = 0;
        /* 发车后暂停 IPS 屏幕刷新，减少行驶过程中的 CPU 占用；停车后自动恢复显示。 */
        if(go_flag == 0 && ++menu_draw_div >= 5)
        {
            menu_draw_div = 0;
            MenuApp_DrawActiveDisplay();
        }
        else if(go_flag)
        {
            menu_draw_div = 0;
        }

        // 调试图传链与主控链解耦，后期可直接关闭以释放性能。
        my_image_transmitter_send();
        pid_tune_tcp_update();

        if(go_flag && !Car_ShouldPause())
        {
            static uint8 zebra_sprint_hold_logged = 0;

            // 斑马线延时停车期：不再跟随斑马线图像产生的异常中线，锁直冲线。
            if(Zebra_IsStopDelayActive())
            {
                dif_speed = 0.0f;
                if(zebra_sprint_hold_logged == 0)
                {
                    zebra_sprint_hold_logged = 1;
                    printf("[ZEBRA] sprint_hold dif=0 speed=%ld\n", (long)set_speed);
                }
            }
            // 外环位置误差先进入 Servo_PID，再把结果交给速度决策层。
            else if(RedBlock_ShouldUseLowSpeedHold())
            {
                zebra_sprint_hold_logged = 0;
                dif_speed = RedBlock_GetMotionDifSpeed();
                set_speed = RedBlock_GetMotionSpeedCmd();
            }
            else
            {
                zebra_sprint_hold_logged = 0;
                // Only run normal line-following after red-block takeover is inactive.
                dif_speed = Servo_PID(err_new);
                if(dec)
                {
                    Speed_decision();
                }
            }
        }
        else
        {
            gpio_set_level(BEEP, 0x00);
        }
    }
}
