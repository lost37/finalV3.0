#include "zf_common_headfile.h"

#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"

//开关状态变量
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;
 
//上一次开关状态变量
uint8 key1_last_status = 1;
uint8 key2_last_status = 1;
uint8 key3_last_status = 1;
uint8 key4_last_status = 1;
 
//开关标志位
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;

uint8 go_flag=0;




void key_operate(void) //获取开关状态 
{

    //使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费

    //保存按键状态
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;

    //读取当前按键状态
    key4_status = gpio_get_level(KEY_3);
    key3_status = gpio_get_level(KEY_2);
    key2_status = gpio_get_level(KEY_1);
    key1_status = gpio_get_level(KEY_0);

    //检测到按键按下之后  并放开置位标志位
    if(!key1_status && key1_last_status)    key1_flag = 1;
    if(!key2_status && key2_last_status)    key2_flag = 1;
    if(!key3_status && key3_last_status)    key3_flag = 1;
    if(!key4_status && key4_last_status)    key4_flag = 1;

    //标志位 置位 之后，可以使用标志位执行自己想要做的事件
    if(key1_flag)//上
    {
    key1_flag = 0;   //清除标志位,用来释放按键
    key1_function();
    }

    // else if(key2_flag)//下
    // {
    // key2_flag = 0;    //清除标志位,用来释放按键
    // key2_function();

    // }

    // else if(key3_flag)//确定
    // {
    // key3_flag = 0;    //清除标志位,用来释放按键
    // key3_function();
    // }

    // else if(key4_flag)
    // {
    // key4_flag = 0; //清除标志位,用来释放按键
    // key4_function();
    // }
  }
void  key1_function(void)
{
    go_flag=!go_flag;
}

