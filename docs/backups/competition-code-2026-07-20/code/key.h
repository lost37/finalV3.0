 #ifndef CODE_KEY_UI_H_
 #define CODE_KEY_UI_H_
 
#include "zf_common_headfile.h"
 
 /*************************************
 *
 *
 *           变量及常数定义
 *
 *
 *************************************/
extern int Switch_Mode;
#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"
#define SWITCH_0    "/dev/zf_driver_gpio_switch_0"
#define SWITCH_1    "/dev/zf_driver_gpio_switch_1"
// 定义刷新间隔（毫秒）
//#define KEY_SCAN_INTERVAL 20   // 按键扫描间隔（可调整，如10-50ms）

// 记录上次刷新时间
// extern uint32_t last_display_time;
// extern uint32_t last_key_time;

// extern uint8 dis_page,dis_page_last;  //显示内容

// extern uint8 sw0_status;
// extern uint8 sw1_status;

//开关状态变量
extern uint8 key1_status;
extern uint8 key2_status;
extern uint8 key3_status;
extern uint8 key4_status;

//上一次开关状态变量
extern uint8 key1_last_status;
extern uint8 key2_last_status;
extern uint8 key3_last_status;
extern uint8 key4_last_status;

//开关标志位
extern uint8 key1_flag;
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;

extern uint8 go_flag;
void key_set_go_flag(uint8 enabled); // TCP/按键共用：设置发车状态，并取消 KEY0 延时切换。
// extern uint8 image_flag;
// extern uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];
// extern int err;
// extern char show_m[50];

//extern  uint8 cursor_position; //光标  
 /*************************************
  *
  *
  *            函数声明
  *
  *
  ************************************/
 void key_operate(void);
 
 //void UI(void); 
 //void interface_display(void);
 
 void key1_function(void); 
//  void key2_function(void);
//  void key3_function(void);
//  void key4_function(void);
 
 #endif /* CODE_KEY_H_ */
