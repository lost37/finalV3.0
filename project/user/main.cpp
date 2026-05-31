#include "zf_common_headfile.h"
#include "my_image_transmitter.h"

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
}

int main(int, char**)
{
    atexit(cleanup);
    signal(SIGINT, sigint_handler);

    // 底层驱动初始化：先电机，再陀螺仪。
    motor_init();
    gyroscope_init();

    // 控制节拍：
    // 5ms 编码器反馈，5ms 电机闭环，500ms 发车延时，5ms 陀螺仪采样。
    pit_ms_init(2, encoder_feedback_task);
    pit_ms_init(5, motor_isr);
    pit_ms_init(500, show_isr);
    pit_ms_init(5, imu_isr);

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

    while(1)
    {
        if(wait_image_refresh() < 0)
        {
            printf("未获取到图像帧\r\n");
            exit(0);
        }

        // 图像主处理链：原始图 -> 裁剪/边缘 -> 中线/元素 -> err_new
        Camera_Function();

        // 调试图传链与主控链解耦，后期可直接关闭以释放性能。
        my_image_transmitter_send();

        if(go_flag && !Car_ShouldPause())
        {
            // 外环位置误差先进入 Servo_PID，再把结果交给速度决策层。
            dif_speed = Servo_PID(err_new);
            if(RedBlock_IsBypassActive())
            {
                dif_speed = RedBlock_GetBypassDifSpeed();
            }

            if(dec)
            {
                Speed_decision();
            }
        }
        else
        {
            gpio_set_level(BEEP, 0x00);
        }
    }
}
