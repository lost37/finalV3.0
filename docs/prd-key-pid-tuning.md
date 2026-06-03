# 按键调节位置环 PID PRD

## Problem Statement

当前智能车的位置环 PID 参数主要写在电机控制代码内部，实车调参时需要反复修改源码、编译、上传、运行，调试效率低。用户希望通过车上的 4 个按键，在程序运行时调节位置环的关键参数，方便快速观察车辆转向、稳定性、提前量和陀螺仪修正效果。

现有按键代码只做简单下降沿检测，缺少防抖、长短按状态、参数选择状态、上下限保护和串口反馈。并且当前主循环中没有调用按键扫描函数，即使扩展按键功能，也可能不会生效。

## Solution

新增一套轻量级按键调参功能，用于在运行时调节位置环 PID 参数。保留现有 C 风格代码结构，不直接搬运参考工程的 `MyKey` C++ 类，但吸收其按键状态机思想。

按键功能规划如下：

- KEY0：保留当前发车/停车功能，继续控制 `go_flag`。
- KEY1：切换当前正在调节的位置环参数。
- KEY2：增加当前选中参数。
- KEY3：减少当前选中参数。

位置环参数规划：

- `Kp`：位置误差线性比例项。
- `Kp2`：位置误差二次比例项，用于大误差时增强修正。
- `Kd`：图像误差差分项，用于抑制过冲。
- `GKD`：陀螺仪 Z 轴角速度修正项，用于抑制实际车身旋转。

每次切换或修改参数后，串口打印当前选中参数和值，便于实车调试记录。

## User Stories

1. As a smartcar developer, I want to adjust position-loop PID parameters using onboard keys, so that I can tune the car without editing source code every time.
2. As a smartcar developer, I want KEY0 to keep controlling start and stop, so that the existing driving workflow is not broken.
3. As a smartcar developer, I want one key to select which PID parameter is active, so that three remaining keys are enough for tuning.
4. As a smartcar developer, I want one key to increase the selected parameter, so that I can make steering response stronger during testing.
5. As a smartcar developer, I want one key to decrease the selected parameter, so that I can reduce oscillation or overcorrection during testing.
6. As a smartcar developer, I want every parameter change to print over serial, so that I know the exact value being tested.
7. As a smartcar developer, I want key input to be debounced, so that one physical press does not trigger multiple accidental changes.
8. As a smartcar developer, I want parameters to have min and max limits, so that repeated key presses cannot push PID into unsafe or invalid values.
9. As a smartcar developer, I want different parameters to have different step sizes, so that each parameter can be tuned with suitable precision.
10. As a smartcar developer, I want the key scanner to be called periodically in the main loop, so that key operations actually take effect during runtime.
11. As a smartcar developer, I want the first version to use short presses only, so that implementation risk stays low.
12. As a smartcar developer, I want the design to leave room for long press behavior later, so that fast adjustment or print-all functions can be added without rewriting the scanner.
13. As a smartcar developer, I want the implementation to reuse the existing project style, so that the new feature is easy to maintain with current files.
14. As a smartcar developer, I want UTF-8 Chinese comments near tunable values, so that later parameter changes are easier to understand.
15. As a smartcar developer, I want no persistent storage in the first version, so that the feature can be verified quickly without adding flash/file write risk.

## Implementation Decisions

- Keep KEY0 as the existing start/stop key. It should continue toggling the vehicle run flag.
- Use KEY1 as the PID parameter selector. Each short press cycles through `Kp -> Kp2 -> Kd -> GKD -> Kp`.
- Use KEY2 to increase the selected parameter.
- Use KEY3 to decrease the selected parameter.
- Convert the position-loop PID parameters from local variables inside the position-loop function into module-level tunable variables.
- Expose small tuning functions from the motor module:
  - select next parameter
  - increase selected parameter
  - decrease selected parameter
  - print current parameter or all parameters
- Add parameter bounds and step sizes:
  - `Kp` step `0.1`, recommended range `0.0 ~ 20.0`
  - `Kp2` step `0.01`, recommended range `0.0 ~ 1.0`
  - `Kd` step `0.05`, recommended range `0.0 ~ 10.0`
  - `GKD` step `0.005`, recommended range `0.0 ~ 1.0`
- Reference the abandoned project's key state machine structure:
  - released
  - debounce
  - pressed
  - long press
- Do not directly copy the abandoned project's `MyKey` class. The current project should keep a simple C-style implementation compatible with the existing `key.cpp` and `key.h`.
- Add a periodic key scan call in the main control loop. Without this, the key tuning feature will not run.
- Define key debounce and long-press thresholds as scan-count based constants, with comments explaining that the real time depends on scan period.
- For the first implementation, only short-press actions are required. Long-press state may be retained in the scanner for future expansion.
- Print tuning feedback using a stable prefix such as `[PID_TUNE]`, so logs are easy to search.

## Testing Decisions

- Prioritize external behavior over internal implementation details.
- Verify that KEY0 still toggles start/stop exactly as before.
- Verify that KEY1 cycles through all tunable PID parameters in order.
- Verify that KEY2 increases only the selected parameter.
- Verify that KEY3 decreases only the selected parameter.
- Verify that parameters are clamped at their configured minimum and maximum.
- Verify that one physical short press results in one logical action after debounce.
- Verify that repeated calls to the key scanner while a key is held do not continuously apply short-press actions.
- Verify that serial output prints the selected parameter name and value after each tuning action.
- On the real car, verify that changed parameters affect `Servo_PID` output without requiring recompilation.

## Out of Scope

- Saving PID parameters permanently to flash, file, or EEPROM.
- Building an OLED/menu UI for PID tuning.
- Tuning the motor speed-loop PI parameters.
- Tuning red-block state machine parameters.
- Adding remote tuning over serial, Wi-Fi, or image transmitter.
- Fully replacing the current key module with the abandoned project's C++ `MyKey` class.

## Further Notes

- The current key code appears to define or reference `key1_status` inconsistently because comments and code may be mojibake in PowerShell output. Before editing, confirm the actual file encoding and declarations carefully.
- The main loop must call the key scanner. This is currently the highest-risk omission because it makes all key planning ineffective.
- Parameter comments should be written in UTF-8 Chinese near the constants and step definitions so future实车调参 can quickly understand each value.
- If long press is added later, suggested actions are:
  - long KEY1: print all PID parameters
  - long KEY2: fast increase selected parameter
  - long KEY3: fast decrease selected parameter
