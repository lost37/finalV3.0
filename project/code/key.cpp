#include "zf_common_headfile.h"
#include "menu_app.h"
#include <sys/time.h>

#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"

typedef enum
{
    KEY_SCAN_RELEASED = 0,
    KEY_SCAN_DEBOUNCE,
    KEY_SCAN_PRESSED,
    KEY_SCAN_LONG_PRESS,
} KeyScanState;

typedef struct
{
    const char *path;
    KeyScanState state;
    uint8 counter;
    uint8 short_press;
    uint8 long_press;
} KeyScanInfo;

// 调参：按键消抖扫描次数。主循环每帧调用时，数值越大越稳但响应越慢。
static const uint8 KEY_DEBOUNCE_SCAN_COUNT = 2;
// 调参：长按判定扫描次数。主循环每帧调用时，约等于 帧周期 * 该值。
static const uint8 KEY_LONG_PRESS_SCAN_COUNT = 25;

static KeyScanInfo key_scan_table[4] =
{
    {KEY_0, KEY_SCAN_RELEASED, 0, 0, 0},
    {KEY_1, KEY_SCAN_RELEASED, 0, 0, 0},
    {KEY_2, KEY_SCAN_RELEASED, 0, 0, 0},
    {KEY_3, KEY_SCAN_RELEASED, 0, 0, 0},
};

uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

uint8 key1_last_status = 1;
uint8 key2_last_status = 1;
uint8 key3_last_status = 1;
uint8 key4_last_status = 1;

uint8 key1_flag = 0;
uint8 key2_flag = 0;
uint8 key3_flag = 0;
uint8 key4_flag = 0;

uint8 go_flag = 0;

// 调参：KEY0 按下后延迟切换发车/停车的时间，单位 ms。
static const uint32 KEY0_GO_TOGGLE_DELAY_MS = 1000;
static uint8 key0_go_toggle_pending = 0;
static uint32 key0_go_toggle_deadline_ms = 0;

static uint32 KeyScan_NowMs(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

static void KeyScan_UpdateOne(KeyScanInfo *key)
{
    const uint8 pressed = (gpio_get_level(key->path) == 0) ? 1 : 0;

    switch(key->state)
    {
        case KEY_SCAN_RELEASED:
            if(pressed)
            {
                key->state = KEY_SCAN_DEBOUNCE;
                key->counter = 0;
            }
            break;

        case KEY_SCAN_DEBOUNCE:
            if(pressed)
            {
                if(++key->counter >= KEY_DEBOUNCE_SCAN_COUNT)
                {
                    key->state = KEY_SCAN_PRESSED;
                    key->counter = 0;
                }
            }
            else
            {
                key->state = KEY_SCAN_RELEASED;
                key->counter = 0;
            }
            break;

        case KEY_SCAN_PRESSED:
            if(!pressed)
            {
                key->short_press = 1;
                key->state = KEY_SCAN_RELEASED;
            }
            else if(++key->counter >= KEY_LONG_PRESS_SCAN_COUNT)
            {
                key->state = KEY_SCAN_LONG_PRESS;
            }
            break;

        case KEY_SCAN_LONG_PRESS:
            if(!pressed)
            {
                key->long_press = 1;
                key->state = KEY_SCAN_RELEASED;
                key->counter = 0;
            }
            break;

        default:
            key->state = KEY_SCAN_RELEASED;
            key->counter = 0;
            break;
    }
}

static uint8 KeyScan_TakeShortPress(uint8 index)
{
    if(index >= 4)
    {
        return 0;
    }

    if(key_scan_table[index].short_press)
    {
        key_scan_table[index].short_press = 0;
        return 1;
    }

    return 0;
}

static uint8 KeyScan_TakeLongPress(uint8 index)
{
    if(index >= 4)
    {
        return 0;
    }

    if(key_scan_table[index].long_press)
    {
        key_scan_table[index].long_press = 0;
        return 1;
    }

    return 0;
}

void key_operate(void)
{
    for(uint8 i = 0; i < 4; i++)
    {
        KeyScan_UpdateOne(&key_scan_table[i]);
    }

    key1_flag = KeyScan_TakeShortPress(0);
    key2_flag = KeyScan_TakeShortPress(1);
    key3_flag = KeyScan_TakeShortPress(2);
    key4_flag = KeyScan_TakeShortPress(3);

    if(key1_flag)
    {
        key1_function();
    }
    if(key2_flag)
    {
        MenuApp_HandleAction(MENU_CORE_ACTION_DOWN);
    }
    if(key3_flag)
    {
        MenuApp_HandleAction(MENU_CORE_ACTION_UP);
    }
    if(KeyScan_TakeLongPress(3))
    {
        MenuApp_HandleAction(MENU_CORE_ACTION_BACK);
    }
    else if(key4_flag)
    {
        MenuApp_HandleAction(MENU_CORE_ACTION_ENTER);
    }

    if(key0_go_toggle_pending)
    {
        const int32 remain_ms = (int32)(key0_go_toggle_deadline_ms - KeyScan_NowMs());
        if(remain_ms <= 0)
        {
            key0_go_toggle_pending = 0;
            go_flag = !go_flag;
        }
    }
}

void key1_function(void)
{
    key0_go_toggle_pending = 1;
    key0_go_toggle_deadline_ms = KeyScan_NowMs() + KEY0_GO_TOGGLE_DELAY_MS;
}
