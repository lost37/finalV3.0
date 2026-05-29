#include "zf_common_headfile.h"
#include <opencv2/imgproc/imgproc.hpp>

#define SERVER_IP "192.168.43.9"
#define PORT 8086
#define mode 1 // 0不发送 1发送到逐飞助手

#define DEBUG_IMAGE_MODE_FULL_GRAY 0
#define DEBUG_IMAGE_MODE_TRACK_GRAY 1
// #define DEBUG_IMAGE_MODE DEBUG_IMAGE_MODE_FULL_GRAY
#define DEBUG_IMAGE_MODE DEBUG_IMAGE_MODE_TRACK_GRAY

// 上位机灰度图发送缓存：320x240（每像素1字节）
uint8 image_gray[UVC_RAW_HEIGHT][UVC_RAW_WIDTH];
volatile int dec = 0;       //决策模式 0关闭 1开启
volatile int motor_mode = 1; //电机模式 0开环 1闭环
volatile float dif_speed = 0;
volatile int camera_flag =0;
int times_start=0;
int start_flag=0;
void imu_isr()
{
    get_gyro(); //获取陀螺仪数据

}
void motor_isr()
{
    if(go_flag && camera_flag && start_flag)
    {
        //go_delay_counter++;
       // if(go_delay_counter>NEGATIVE_PRESSURE_DELAY)
        //{
            if(motor_mode ==0) //开环
            {Motor_test();} //电机测试
            else
            {dianji_control();
            }
       // }
    }
    else
    {
        pwm_set_duty(MOTOR1_PWM, 0);
        pwm_set_duty(MOTOR2_PWM, 0);
        go_delay_counter =0;
    }
}
void show_isr()
{
        if(++times_start>=2)
        {
            start_flag=1;
        }
}
void sigint_handler(int signum)
{
    printf("收到Ctrl+C,程序即将退出\n");
    go_flag=0;
    exit(0);
}

void cleanup()
{
    // 需要先停止定时器线程，后面才能稳定关闭电机，电调，舵机等
    printf("程序异常退出，执行清理操作\n");
    // 关闭电机
    gpio_set_level(BEEP, 0x00);      // 关闭蜂鸣器
    pwm_set_duty(MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR2_PWM, 0);
}


int main(int, char**)
{
     // 注册清理函数
    atexit(cleanup);

    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);
    //ips200_init("/dev/fb0");
    motor_init();

    //readParameters(); //flash初始化读参数
    pit_ms_init(5, motor_isr);      // 电机5ms控制
    //pit_ms_init(1000, time);      // 电机5ms控制
    pit_ms_init(500, show_isr);      // 每隔0
    pit_ms_init(5, imu_isr);        //陀螺仪 坡道
    go_init();
    // 初始化UVC摄像头
    if(uvc_camera_init("/dev/video0") < 0)
    {
        printf("摄像头初始化失败！\r\n");
        return -1;
    }

#if(1 == mode)


    // 初始化TCP客户端
    if(tcp_client_init(SERVER_IP, PORT) == 0)
    {
        printf("tcp_client连接成功\r\n");
        camera_flag=1;
    }
    else
    {
        printf("tcp_client连接失败\r\n");
        camera_flag=0;
        return -1;
    }
    // 初始化逐飞助手接口
    seekfree_assistant_interface_init(tcp_client_send_data, tcp_client_read_data);
#endif


    while(1)
    {
        //key_operate();
        // 阻塞等待图像刷新
        if(wait_image_refresh() < 0)
        {
            printf("未获取到图像帧\r\n");
            exit(0);
        }

        // 调用图像处理主函数
        Camera_Function();

        // Get_speed();
        // printf("zf_encoder_left = %d.\r\n", enconder_left);
        // printf("zf_encoder_right = %d.\r\n", enconder_right);


#if(1 == mode)
        if(DEBUG_IMAGE_MODE == DEBUG_IMAGE_MODE_FULL_GRAY)
        {
            if(!frame_rgb.empty())
            {
                cv::Mat frame_gray;
                cv::cvtColor(frame_rgb, frame_gray, cv::COLOR_BGR2GRAY);

                int16 search_x = 0;
                int16 search_y = 0;
                uint16 search_width = 0;
                uint16 search_height = 0;
                if(RedBlock_GetSearchRect(&search_x, &search_y, &search_width, &search_height))
                {
                    cv::rectangle(
                        frame_gray,
                        cv::Rect(search_x, search_y, search_width, search_height),
                        cv::Scalar(120),
                        1
                    );
                }

                int16 rect_x = 0;
                int16 rect_y = 0;
                uint16 rect_width = 0;
                uint16 rect_height = 0;
                if(RedBlock_GetRect(&rect_x, &rect_y, &rect_width, &rect_height))
                {
                    cv::rectangle(
                        frame_gray,
                        cv::Rect(rect_x, rect_y, rect_width, rect_height),
                        cv::Scalar(255),
                        2
                    );
                }

                int16 roi_x = 0;
                int16 roi_y = 0;
                uint16 roi_width = 0;
                uint16 roi_height = 0;
                if(RedBlock_GetModelRoi(&roi_x, &roi_y, &roi_width, &roi_height))
                {
                    cv::rectangle(
                        frame_gray,
                        cv::Rect(roi_x, roi_y, roi_width, roi_height),
                        cv::Scalar(180),
                        1
                    );
                }

                memcpy(image_gray[0], frame_gray.data, UVC_RAW_WIDTH * UVC_RAW_HEIGHT);
                seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_gray[0], UVC_RAW_WIDTH, UVC_RAW_HEIGHT);
                seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0, NULL, NULL, NULL, NULL, NULL, NULL);
                seekfree_assistant_camera_send();
            }
        }
        else if(DEBUG_IMAGE_MODE == DEBUG_IMAGE_MODE_TRACK_GRAY)
        {
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, (uint8 *)Cut_Image_Use, Cut_COL, Cut_ROW);
            seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0, NULL, NULL, NULL, NULL, NULL, NULL);
            seekfree_assistant_camera_send();
        }
#endif
        if(go_flag && !Car_ShouldPause()){

            dif_speed = Servo_PID(err_new);

            if(dec)
            {
                Speed_decision();
            }

            //chasu_calculation(); //单减差速控制 舵机输出

            // printf("po_flag=%d\n",barrier_flag);
        }else{
            gpio_set_level(BEEP , 0x00);
            //wm_set_duty(SERVO_MOTOR1_PWM, 0);
            //pwm_set_duty(FAN_1, 0);
            //pwm_set_duty(FAN_2, 0);
            //interface_display();//UI界面显示
        }
    }
}


