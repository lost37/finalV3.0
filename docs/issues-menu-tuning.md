# 菜单调参改造 Issues

## 1. 接入 4 按键事件系统

**Type**: AFK  
**Blocked by**: None

### What to build

把当前简单边沿检测升级为带防抖的短按/可扩展长按事件系统，并输出菜单可用的按键动作。

### Acceptance criteria

- [ ] KEY0/KEY1/KEY2/KEY3 都能产生稳定短按事件。
- [ ] 单次物理按下只触发一次动作。
- [ ] 保留 KEY0 当前发车/停车能力，或预留长按发车/停车接口。
- [ ] 主循环中周期调用按键扫描。

## 2. 抽取可调参数为真实运行变量

**Type**: AFK  
**Blocked by**: None

### What to build

把要调的参数从局部常量/局部变量改成菜单可绑定的模块级变量。

### Acceptance criteria

- [ ] `Servo_PID` 使用可调的 `Kp/Kp2/Kd/GKD`。
- [ ] 前瞻参数 `w/land_w` 可被外部菜单绑定。
- [ ] 速度类参数可被菜单读取和修改。
- [ ] 每个参数旁边有 UTF-8 中文调参注释。

## 3. 移植轻量菜单核心

**Type**: AFK  
**Blocked by**: Issue 1

### What to build

参考 v4.0 菜单的文件夹树、数值框、限幅、选中状态和步进调节，做一版适配当前工程的 C 风格菜单核心。

### Acceptance criteria

- [ ] 支持文件夹节点。
- [ ] 支持 `float/int/int32/uint8` 数值节点。
- [ ] 支持最小值/最大值限幅。
- [ ] 支持选中参数后再加减。
- [ ] 不依赖 `ST7789.h`。

## 4. 适配 IPS200 显示层

**Type**: AFK  
**Blocked by**: Issue 3

### What to build

使用当前工程已有 `ips200_show_string/int/float` 显示菜单路径、参数名、参数值、选中箭头和当前步进。

### Acceptance criteria

- [ ] 屏幕能显示当前菜单路径。
- [ ] 屏幕能显示当前文件夹下参数列表。
- [ ] 当前选中项有明显箭头。
- [ ] 被编辑参数用 `<value>` 标识。
- [ ] 不全速刷屏，只在按键变化或低频周期刷新。

## 5. 建立第一批调车菜单分组

**Type**: AFK  
**Blocked by**: Issue 2, Issue 3, Issue 4

### What to build

注册第一批实车常调参数到菜单树。

### Acceptance criteria

- [ ] `PID` 分组包含 `Kp/Kp2/Kd/GKD`。
- [ ] `Camera` 分组包含 `w/land_w`。
- [ ] `Speed` 分组包含基础速度、环岛速度、弯道速度。
- [ ] `Ackermann` 分组包含阿克曼敏感度相关参数。
- [ ] 每个参数有合理步进和限幅。

## 6. 定义 4 键菜单交互规则

**Type**: HITL  
**Blocked by**: Issue 1, Issue 3

### What to build

最终确定 4 个按键在菜单浏览、参数编辑、步进切换、发车/停车之间的分配。

### Acceptance criteria

- [ ] 菜单浏览能上/下/进入/返回。
- [ ] 参数编辑能增加/减少/退出编辑。
- [ ] 不会误触发发车/停车。
- [ ] 用户确认 KEY0 是继续短按发车，还是改成长按发车。

## 7. 主循环集成菜单系统

**Type**: AFK  
**Blocked by**: Issue 1, Issue 3, Issue 4, Issue 6

### What to build

在主循环中初始化菜单和屏幕，周期处理按键事件和菜单刷新，同时不影响图像、电机、红块状态机。

### Acceptance criteria

- [ ] 程序启动后屏幕显示菜单。
- [ ] 按键能改变菜单状态和参数值。
- [ ] 电机 5ms 控制中断不进行屏幕刷新。
- [ ] 图像处理链仍正常运行。
- [ ] 参数变化能实时影响控制逻辑。

## 8. 实车验证菜单调参链路

**Type**: HITL  
**Blocked by**: Issue 7

### What to build

在实车上验证从屏幕显示、按键修改、参数生效到车辆行为变化的完整链路。

### Acceptance criteria

- [ ] 屏幕显示稳定，无明显闪烁或卡顿。
- [ ] 按键响应稳定，无连跳。
- [ ] 修改 `Kp/Kd/w` 后车辆行为有可观察变化。
- [ ] 菜单刷新不会明显拖慢巡线。
- [ ] 记录需要调整的限幅、步进和按键手感。

## Recommended Order

`1 -> 2 -> 3 -> 4 -> 6 -> 5 -> 7 -> 8`
