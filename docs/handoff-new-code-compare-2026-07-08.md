# TestV3.0 新代码对比交接文档

生成时间：2026-07-08  
当前项目：`E:\Virtual Machines\temp\TestV3.0`  
新对话目标：拿一份新代码与当前智能车项目对比，筛选可迁移优化点。

## 使用偏好

- 用户已开启 caveman mode：回答短、技术重点、少废话。
- 中文注释使用 UTF-8。
- 不要一步到位大改，按模块小步验证。
- 频繁修改参数用中文注释说明含义。

## 建议技能

- `caveman`：保持短回答。
- `embedded-systems`：固件/实时控制/传感器/电机相关分析。
- `diagnose`：遇到实车行为不符时按复现、定位、验证走。

## 当前重要路径

- 主项目：`E:\Virtual Machines\temp\TestV3.0`
- 智能车代码：`E:\Virtual Machines\temp\TestV3.0\project\code`
- 主循环：`E:\Virtual Machines\temp\TestV3.0\project\user\main.cpp`
- AI PID 项目：`C:\Users\86189\Desktop\pid-ai\llm-pid-tuner-repo`
- 参考旧陀螺仪代码：`E:\Virtual Machines\temp\Abandoned-main\project\code\IMU963R.cpp`

## 编译状态

- 本轮未成功本地编译验证。
- PowerShell 没有 `make`。
- WSL 可用，但其环境里 `make` 缺失。
- 项目 `project/out` 有大量构建产物改动，不要把 build 输出当核心代码变更。

## 红块模块当前状态

核心文件：

- `project\code\redblock.cpp`
- `project\code\redblock.h`
- `project\code\camera.cpp`

红块颜色检测流程：

```text
BGR -> HSV
-> 两段红色阈值 inRange
-> 合并 red_mask
-> 多边形 ROI 限制
-> red_pixel_count < 25 直接退出
-> 开运算 OPEN
-> 闭运算 CLOSE
-> findContours
-> 面积/宽高比/填充率筛选
-> 最大有效轮廓作为红块
```

当前面积参数：

```cpp
constexpr int RED_MIN_AREA = 50;
constexpr int RED_MAX_AREA = 5000;
```

含义：

```text
area < RED_MIN_AREA -> 忽略小红点
area > RED_MAX_AREA -> 忽略过大红色区域，按普通巡线走
```

红块模型分类：

```text
suppliers -> right_bypass
weapon    -> left_bypass
vehicle   -> line_follow
unknown/invalid/lost 当前仍可能 fallback left_bypass
```

注意：当前模型没有 `redbrick` / `红砖` 类。红砖如果被 HSV 当红色目标，仍可能进入红块状态机。`RED_MAX_AREA` 只是临时几何过滤，不是可靠分类。

红块与其他元素互斥当前状态：

在 `camera.cpp` 中，现在只让 S 弯和坡道/障碍标志挡红块：

```cpp
const uint8 other_element_exclusive = (
    s_wan_flag != 0 ||
    barrier_flag != 0
);
```

所以当前允许：

```text
圆环处理中 -> 红块可检测/识别/绕行
斑马线处理中 -> 红块可检测/识别/绕行
十字处理中 -> 红块可检测/识别/绕行
```

但红块一旦进入互斥状态，会清其他元素：

```cpp
RB_DEC_BRAKING
RB_DEC_LOW_SPEED_SETTLE
RB_DEC_MODEL_RECOGNIZING
RB_DEC_MOTION_ACTIVE + bypass_active
```

绕行动作核心：

- `RedBlock_StartBypassMode()`
- `RedBlock_ApplyBypass()`
- `RedBlock_ApplyVisualCenterline()`

## 斑马线模块当前状态

核心文件：

- `project\code\zebra.cpp`
- `project\code\zebra.h`

当前默认：

```cpp
volatile int zebra_mode = 1;
volatile int zebra_target_laps = 1;
static const int32_t ZEBRA_STOP_DELAY_ENCODER = 6000;
```

含义：

```text
zebra_target_laps = 1 -> 斑马线前发车，跑一圈后停车
zebra_target_laps = 2 -> 跑两圈后停车
```

计数逻辑：

```text
第 1 次检测到斑马线 = 起点线
离开斑马线 30 帧后允许计下一次
目标检测次数 = zebra_target_laps + 1
目标线检测到后，继续跑 6000 编码器距离停车
```

检测逻辑已经改为“一帧只更新一次状态”，避免旧代码每行扫描时重复更新状态机。

## 图传/前瞻显示当前状态

核心文件：

- `project\code\my_image_transmitter.cpp`
- `project\code\my_image_transmitter.h`
- `project\code\menu_app.cpp`

图传总开关：

```cpp
#define MY_IMAGE_TRANSMITTER_ENABLE 1
#define MY_IMAGE_TRANSMITTER_MODE MY_IMAGE_TRANSMITTER_MODE_TRACK_GRAY
```

图传目标：

```text
TCP 上传巡线灰度图，并叠加 w 对应前瞻黄线
```

实现方式：

```text
不修改 Cut_Image_Use 原图
构建 RGB565 调试缓冲
把 w 行写成 RGB565_YELLOW
上传 SEEKFREE_ASSISTANT_RGB565
```

注意：之前中断过修改，若新对话继续，应先检查 `my_image_transmitter.cpp` 是否完整调用 `my_image_build_track_rgb565_with_foresight()`，并确认可编译。

IPS 屏幕本身已有前瞻黄线：

- `menu_app.cpp`
- `DrawForesightLineOverlay()`

## AI PID / TCP 调参状态

固件 TCP 调参文件：

- `project\code\pid_tune_tcp.cpp`
- `project\code\pid_tune_tcp.h`

当前固件是 TCP client，上位机是 server：

```cpp
PID_TUNE_TCP_SERVER_IP "192.168.43.9"
PID_TUNE_TCP_SERVER_PORT 9091
```

支持：

```text
STATUS
HELP
SAVE
STREAM ON/OFF
SIDE L/R
LOOP SPEED/POS
SET P:<p> I:<i> D:<d>
SET POS KP:<kp> KP2:<kp2> KD:<kd> GKD:<gkd>
SET LKp/LKi/LA/RKp/RKi/RA
```

速度环：

- 左轮：`motor_l_kp`, `motor_l_ki`, `motor_l_filter_a`
- 右轮：`motor_r_kp`, `motor_r_ki`, `motor_r_filter_a`

位置环：

- `servo_pid_kp`
- `servo_pid_kp2`
- `servo_pid_kd`
- `servo_pid_gkd`

AI PID 上位机已改为支持 `--loop pos`，位置环提示词说明：

```text
位置环不是经典 PID，而是 PD + gyro assist
p -> servo_pid_kp
i -> servo_pid_kd
d -> servo_pid_gkd
KP2 固定
```

## 陀螺仪状态

核心文件：

- `project\code\gyroscope.cpp`
- `project\code\motor.cpp`

已做：

- 启动预热。
- Y/Z 零漂校准。
- 运行时减零漂。
- `gyro_z` 一阶低通滤波。
- 位置环陀螺辅助项已改为：

```cpp
- gyro_z * GKD
```

而不是直接用 `imu_gyro_z * GKD`。

未做：

- 温漂补偿。
- 运行时零漂更新。
- 三轴完整校准。
- 磁力计融合。

## 坡道/障碍状态

代码里命名混乱：

```text
zhang_ai_flag / zhang_ai_judge() = 障碍物
barrier_flag / po_judge() = 坡道
```

障碍物 `zhang_ai_judge()` 当前存在但未启用：

```cpp
//zhang_ai_judge();
```

坡道 `po_judge()` 当前启用，并通过 `barrier_flag` 影响速度状态：

```text
barrier_flag != 0 -> STATE_SLOPE -> speed_chose = po_s
```

当前红块互斥里仍把 `barrier_flag != 0` 作为阻止红块更新的条件。

## 新代码对比建议

优先对比这些模块：

1. 红块检测：HSV 阈值、ROI、多边形区域、开闭运算顺序、面积/宽高比/填充率阈值。
2. 红砖/红块区分：新代码是否有独立类别、几何规则或独立元素入口。
3. 红块绕行：是否仍用边界替换中线，是否用编码器分阶段推进。
4. 元素互斥：新代码是否允许红块打断圆环/斑马线/十字。
5. 斑马线：是否有圈数选择、延迟停车距离、状态机是否一帧更新一次。
6. 图传：是否有前瞻线、是否污染算法原图。
7. 陀螺仪：是否有更完整零漂/滤波/温漂处理。
8. PID 调参：是否支持实时参数、保存、左右轮独立、位置环 PD + gyro assist。

## 当前重点风险

- 红砖与红块不能可靠区分。`RED_MAX_AREA` 只是临时过滤。
- 红块 fallback 仍偏激进：模型 invalid/lost/unknown 可能左绕。
- 红块允许在圆环/斑马线/十字期间检测后，一旦进入识别/绕行，会清其他元素状态，实车需验证优先级是否合适。
- `project/out` 构建产物很脏，新对话不要被 build diff 干扰。
- 本机编译链不完整，代码改动最好在车端或可用 LoongOS 工具链验证。

