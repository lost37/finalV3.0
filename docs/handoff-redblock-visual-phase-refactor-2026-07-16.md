# 红块视觉绕行 phase 重构交接

## 目标

下一轮只重构红块绕行的“视觉绕行 phase”部分。

不要大改红块检测、NCNN 分类、TCP PID、斑马线、圆环等模块。当前核心问题是：红块识别后进入 `left_bypass`，小车频繁直接左绕出界，视觉绕行动作不稳定。

用户期望动作：

```text
发现红块
-> 减速识别模型，保证图片清晰
-> 快速但可控地向左边界靠近
-> 贴着左边线行驶一段距离，把左边界当作临时中线绕过障碍
-> 逐步回到真实中线
```

## 建议技能

- `embedded-systems`：这是实车控制逻辑，注意状态机、实时性、编码器距离。
- `diagnose`：每次修改必须用日志验证，不要凭感觉继续堆参数。
- `tdd`：当前本地只有菜单单测；红块逻辑缺可测试 seam，重构时建议抽出纯函数/小状态机方便测试。

## 当前关键文件

- `project/code/redblock.cpp`
- `project/code/redblock.h`
- `project/code/camera.cpp`

调用链：

```text
camera.cpp
  RedBlock_Update()
  Model_Request_Process()
  search_center()
  weight_box()
  RedBlock_ApplyBypass()
  RedBlock_ApplyVisualCenterline()
  err_new = Err_Get()
  RedBlock_LogVisualControl(err_new)
```

`RedBlock_ApplyBypass()` 管 phase 状态推进。  
`RedBlock_ApplyVisualCenterline()` 按 phase 改写 `Center_point[] / Center_err[]`。  
`RedBlock_GetMotionDifSpeed()` 根据 `err_new` 算红块绕行差速限幅。

## 当前状态机

决策状态在 `redblock.h`：

```cpp
RB_DEC_IDLE
RB_DEC_CONFIRMING
RB_DEC_BRAKING
RB_DEC_LOW_SPEED_SETTLE
RB_DEC_MODEL_RECOGNIZING
RB_DEC_MOTION_ACTIVE
```

视觉绕行 phase：

```cpp
RB_BYPASS_PHASE_IDLE = 0
RB_BYPASS_PHASE_APPROACH = 1
RB_BYPASS_PHASE_COMMIT = 2
RB_BYPASS_PHASE_EXIT_HOLD = 3
RB_BYPASS_PHASE_RECOVER = 4
```

当前 `left_bypass` 使用：

```cpp
boundary_center = (int)l_border[row];
Center_point[row] = boundary_center;
Center_err[row] = Cut_COL / 2 - Center_point[row];
```

也就是说它选的是 `160x90` 裁剪图里的左边界：

```cpp
#define Cut_ROW 90
#define Cut_COL 160
extern uint8 l_border[Cut_ROW], r_border[Cut_ROW];
```

## 已做过的关键修改

1. 修过 phase3 控制行跳变问题。

旧问题：

```text
phase2 ctrl=76
phase3 ctrl=45
row=45 cp=79 l=0 r=159
```

当前逻辑：进入 `EXIT_HOLD` 时保留 phase2 选出的 `redblock_visual_control_row`，只有无效时才回退 `ROW_MIN=45`。

2. 增加 `hold_center`。

旧问题：phase3 边界丢失后 `cp` 回到 79，控制失效。  
当前逻辑：phase2 选中窗口时保存平均中心 `redblock_visual_hold_center`；phase3 控制窗口边界无效时使用它兜底。

3. phase3 已改成逐步回线。

`EXIT_HOLD` 中 `blend` 从 100 线性衰减到 0，不再一直 100% 贴左边界。

4. 增加 phase2 贴边判断。

当前参数大致为：

```cpp
REDBLOCK_BYPASS_APPROACH_FRAMES = 6;
REDBLOCK_VISUAL_BOUNDARY_READY_ERR = 20.0f;
REDBLOCK_VISUAL_BOUNDARY_READY_FRAMES = 3;
REDBLOCK_VISUAL_REVERSE_ERR_LIMIT = 15.0f;
REDBLOCK_VISUAL_BOUNDARY_SEEK_MAX_DISTANCE = 4500;
REDBLOCK_VISUAL_BOUNDARY_HOLD_DISTANCE = 2600;
REDBLOCK_VISUAL_APPROACH_DIF_LIMIT = 32.0f;
REDBLOCK_VISUAL_DIF_LIMIT = 32.0f;
REDBLOCK_VISUAL_RECOVER_DIF_LIMIT = 30.0f;
```

当前 phase2 流程：

```text
COMMIT:
  若反向误差过大 -> 立即进入 EXIT_HOLD
  若还没开始贴边保持:
    err 满足贴边阈值连续 3 帧，或追边距离超过 4500
    -> boundary_hold_start，重置编码器
  已开始贴边保持:
    编码器距离 >= 2600
    -> EXIT_HOLD
```

5. 修过 warning。

- `REDBLOCK_COOLDOWN_FRAMES = 560` 从 `uint8` 改为 `uint16`，否则会溢出成 48。
- 删除未用 `ClampRowIndex`。
- `pid_tune_tcp.cpp` 和 `my_image_transmitter.cpp` 做过条件编译清 warning。

## 仍然存在的问题

实车仍频繁左拐出界。最新日志片段：

```text
[RB_VIS] start mode=3 centerline=boundary speed=170
[RB_VIS_MOTOR] err=19.17 dif=32.00 set=170 target(L,R)=(133,206)
[RB_VIS_MOTOR] err=21.98 dif=32.00 set=170 target(L,R)=(133,206)
[RB_VIS] phase 1 -> 2 enc=269619
[RB_VIS_MOTOR] err=25.32 dif=32.00 set=170 target(L,R)=(133,206)
[RB_VIS_MOTOR] err=26.64 dif=16.00 set=170 target(L,R)=(152,187)
[RB_VIS] boundary_hold_start err=26.64 ready=3 seek=3733 enc=273352
[RB_VIS] phase=2 mode=3 blend=100 rows=90 first=79 last=48 err=30.36 ctrl=54 sel=0
[RB_VIS_ERR] phase=2 mode=3 err=-60.70 ctrl=54 row=54 cp=120 l=82 r=158 row2=63 cp2=140 l2=123 r2=158
[RB_VIS_DIF] phase=2 mode=3 err=-60.70 raw=-234.00 dif=-32.00 limit=32.00 speed=170
```

关键观察：

- 代码确实选的是 `160x90` 图里的 `l_border[]`。
- 出问题时 `l_border` 已经不再像真实左边界：`l=82`, `l2=123`，甚至跑到图像中线右侧。
- 左绕时 `err` 变成 `-60.70`，说明控制方向已经反向/失真。
- 当前刚加了 `reverse_err_escape` 保护，若 `err_new <= -15` 会进入 phase3，但尚未经过充分实车验证。

用户视觉反馈：

- 看不到稳定“贴左边线行驶一段时间”。
- 仍感觉车直接向左边界外拐出去。

## 为什么建议重构 phase

现在逻辑是在原状态机上连续打补丁：

- phase1 用帧数切换。
- phase2 同时承担“追边”和“贴边保持”。
- phase3 同时承担“退出保持”和“回中线”。
- `err_new`、`Search_Stop_Line`、`l_border`、`hold_center`、编码器距离混在一起作为切换依据。

结果：难判断车到底处于“靠边”“贴边”“回线”哪一步。日志能看出动作，但状态语义不干净。

## 推荐重构边界

只重构 `RB_BYPASS_PHASE_*` 对应视觉动作，不碰前置检测/分类。

保留：

```cpp
RedBlock_Detect()
RedBlock_UpdateDecision()
RedBlock_StartBypassMode()
RedBlock_GetMotionSpeedCmd()
RedBlock_GetMotionDifSpeed()
RedBlock_ApplyVisualCenterline() 的外部接口
```

重点改：

```cpp
RedBlock_GetVisualBlendPercent()
RedBlock_ApplyBypass()
RedBlock_ApplyVisualCenterline()
视觉绕行相关状态变量
```

## 推荐新 phase 设计

建议把 phase 语义改清楚：

```text
PHASE_APPROACH_NORMAL
  减速识别后进入绕行，短暂保持普通中线或低 blend，避免刚接管就猛打。

PHASE_SEEK_BOUNDARY
  快速向目标边界靠近。
  左绕时目标不是 l_border 本身，而是 l_border + safety_offset。
  目的：靠近左边线，但不把车中心压到线外。

PHASE_BOUNDARY_HOLD
  确认靠边后，重新清零编码器。
  跟随“安全边界中线”保持一段距离，真正绕过障碍。

PHASE_BLEND_BACK
  从安全边界中线逐步混回普通中线。

PHASE_RECOVER
  完全普通巡线，满足边界/误差稳定后退出红块模块。
```

注意：用户原话是“把左边界当中线”，但实车表现说明直接 `Center_point = l_border` 太危险。建议保留语义，但实现用安全偏移：

```cpp
left_bypass_center = l_border[row] + REDBLOCK_VISUAL_BOUNDARY_OFFSET;
right_bypass_center = r_border[row] - REDBLOCK_VISUAL_BOUNDARY_OFFSET;
```

可先取：

```cpp
REDBLOCK_VISUAL_BOUNDARY_OFFSET = 8~15
```

这样图像上仍贴左边线，但车体中心不会直接压到边界。

## 推荐重构后的关键参数

建议用少量清晰参数替代现在散乱参数：

```cpp
// 靠边安全偏移，防止把车中心直接压到边界上
REDBLOCK_VISUAL_BOUNDARY_OFFSET

// 追边阶段最大差速
REDBLOCK_VISUAL_SEEK_DIF_LIMIT

// 贴边保持阶段最大差速
REDBLOCK_VISUAL_HOLD_DIF_LIMIT

// 回线阶段最大差速
REDBLOCK_VISUAL_BACK_DIF_LIMIT

// 判断已靠近目标边界的误差阈值/连续帧数
REDBLOCK_VISUAL_BOUNDARY_READY_ERR
REDBLOCK_VISUAL_BOUNDARY_READY_FRAMES

// 真正贴边保持距离
REDBLOCK_VISUAL_BOUNDARY_HOLD_DISTANCE

// 回线 blend 帧数或距离
REDBLOCK_VISUAL_BLEND_BACK_FRAMES
```

## 建议日志格式

保留/新增这些日志，便于实车判断：

```text
[RB_VIS] phase A -> B enc=...
[RB_VIS_TARGET] phase=... mode=... row=... normal=... boundary=... target=... blend=...
[RB_VIS_ERR] phase=... err=... ctrl=... cp=... l=... r=...
[RB_VIS_DIF] phase=... err=... raw=... dif=... limit=... speed=...
[RB_VIS_MOTOR] err=... dif=... set=... target(L,R)=...
```

重构后必须能回答：

- 当前车在“追边/贴边/回线”哪一步？
- 当前目标中心是普通中线、边界线、还是混合线？
- `l_border` 是否可信？
- 若 `l_border` 不可信，当前 fallback 是什么？

## 实车验证指标

每次测试看这些：

```text
1. phase 是否按预期：
   SEEK_BOUNDARY -> BOUNDARY_HOLD -> BLEND_BACK -> RECOVER

2. BOUNDARY_HOLD 是否真的出现并持续：
   能看到 hold_start 后再跑一段编码器距离

3. 左绕时 err 不应突然变成大负数：
   err <= -15 是危险信号

4. l_border 不应长期 >= 80：
   若出现，说明左边界已失真，不能继续把它当目标

5. phase3/回线时 blend 应逐步降低：
   100 -> 80 -> 60 -> 30 -> 0
```

## 注意事项

- 不要再只靠调 `REDBLOCK_VISUAL_SIDE_DISTANCE` 解决。那会在“出界”和“贴边不明显”之间来回摆。
- 不要让 phase2 同时承担“追边”和“贴边保持”。这就是当前混乱来源。
- 不要大规模重构检测/模型分类。当前模型识别能进入 `weapon -> left_bypass`，问题在绕行动作。
- 不要删除 `hold_center` 思路；边界丢失时仍需要兜底。
- 若继续使用 `l_border`，必须加可信判断和安全偏移。

## 当前本地验证

Windows 本地只能跑已有菜单测试：

```text
menu_core tests passed
```

主工程编译在 LoongOS/交叉编译环境完成。此前出现过：

```text
redblock.cpp.o: file not recognized: file truncated
```

这是 object 损坏/中断残留，处理：

```bash
cd /home/lost37/Desktop/temp/TestV3.0/project/out
rm -f CMakeFiles/project.dir/home/lost37/Desktop/temp/TestV3.0/project/code/redblock.cpp.o
make -j$(nproc)
```

不行再：

```bash
make clean
make -j$(nproc)
```

