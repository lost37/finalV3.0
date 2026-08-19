# TestV3.0 Overall Optimization Handoff

Generated: 2026-07-18

Workspace: `E:\Virtual Machines\temp\TestV3.0`

Next-session goal: understand the whole smart-car codebase, then improve it incrementally. Do not begin with a large rewrite.

## User Preferences

- Caveman mode is active: concise Chinese responses, high signal.
- Chinese source comments must be UTF-8.
- Work in small modules and validate each step on the real vehicle.
- Frequently adjusted parameters need Chinese comments.
- Windows host cannot perform a reliable LoongOS cross-build. The user compiles and tests on their own environment/vehicle.
- Preserve unrelated dirty working-tree changes. The repository is intentionally very dirty.

## Project Shape

```text
project/user/main.cpp
  startup, camera frame loop, key/menu, display, motor control

project/code/camera.cpp
  line extraction, left/right borders, centerline, road-element orchestration

project/code/control.cpp + motor.cpp
  speed choice, wheel-speed PID, steering PD + gyro assist, PWM output

project/code/redblock.cpp
  red target perception, decision state machine, model handoff, visual bypass

project/code/zebra.cpp / circle.cpp / cross.cpp
  zebra stopping, roundabout, intersection elements

libraries/zf_device/zf_device_uvc.cpp
  UVC acquisition, raw RGB cache, resized line-follow image, camera controls
```

The project targets LoongOS/LS2K0300 and uses OpenCV plus NCNN. CMake sources every `.cpp` in `project/code`, `project/user`, and listed `libraries` directories.

## Main Runtime Flow

Per camera frame, `Camera_Function()` in `project/code/camera.cpp` currently does this order:

```text
raw grayscale/edge -> Cut_Image_Use / Canny_Cut_Image_Use
-> search_border() produces l_border[] / r_border[] and validity flags
-> RedBlock_Update() and Model_Request_Process() unless S-curve/slope excludes it
-> zebra/cross/circle/S-curve logic when red-block flow is not exclusive
-> search_center() and weight_box()
-> RedBlock_ApplyBypass() + RedBlock_ApplyVisualCenterline()
-> Err_Get() -> err_new
```

`control.cpp` selects `set_speed`; `camera.cpp::chasu_calculation()` converts steering error/differential speed into left/right wheel targets; `motor.cpp::l_pid/r_pid` generate PWM.

After `go_flag=1`, IPS menu drawing is skipped to reduce CPU. Key scan, image processing, TCP PID handling, and car control remain active. `STOP` or physical key stop restores display work.

## Image Coordinate Systems

This distinction is critical for visual work:

```text
frame_rgb: raw 320x240 BGR, used by red detection and NCNN
frame_line_resize: raw image resized to 160x120
Cut_Image_Use: top 160x90 of frame_line_resize, used by line following
l_border[] / r_border[] / Center_point[]: 160x90 coordinate system
```

Never compare a raw RGB `x/y` directly with `l_border/r_border`. Map raw RGB coordinates to the 160x90 line image first.

## Core Modules

### Camera and Line Following

- `project/code/camera.cpp`, `camera.h`
- `search_border()` fills `l_border[]`, `r_border[]`, `l_effect_flag[]`, `r_effect_flag[]`.
- `search_center()` creates `Center_point[]`.
- `w` is foresight/control-row related and is shown as a yellow line in both IPS overlay and image transmitter output.
- `ack_dif_full_scale` controls Ackermann differential speed scaling.

### Control and Motor

- `project/code/control.cpp`: normal/slope/red-block speed arbitration. Global `set_speed` is actual active speed command.
- `project/code/motor.cpp`:
  - left speed loop: `motor_l_kp`, `motor_l_ki`, `motor_l_filter_a`
  - right speed loop: `motor_r_kp`, `motor_r_ki`, `motor_r_filter_a`
  - steering: `servo_pid_kp`, `servo_pid_kp2`, `servo_pid_kd`, `servo_pid_gkd`
  - steering gyro term is `- gyro_z * GKD`, not raw `imu_gyro_z`.
- Left/right wheel PID are intentionally independently tunable because vehicle mechanics are asymmetric.

### Gyroscope

- `project/code/gyroscope.cpp`
- IMU660RA path: warmup -> startup zero calibration -> runtime publishing.
- Z-axis path: startup offset + online bias learner + `LPF_1` low-pass filter.
- Online bias learns only under near-1g and low angular-rate conditions.
- Current source has a documentation mismatch: comment says `130` startup samples, macro `GYRO_ZERO_SAMPLE_COUNT` is `200`, with `5 ms` delay. Treat macro as behavior and reconcile comments/intent before further tuning.
- No temperature table, magnetometer fusion, or full 3-axis runtime calibration.

### UVC Camera

- `libraries/zf_device/zf_device_uvc.cpp/.h`
- Raw target: 320x240. Line image: 160x120, only top 90 rows used by legacy line algorithm.
- Supports manual exposure, auto exposure mode, and readback.
- Defaults currently include exposure `-5`, brightness `0`, contrast `15`, saturation `31`.
- `frame_rgb` is the authoritative color frame for NCNN/red perception.

### Display and Image Transmission

- `project/code/menu_app.cpp`: IPS screen, menu and foresight yellow overlay.
- `project/code/my_image_transmitter.cpp`: sends RGB565/gray debug images; track mode draws yellow foresight row without changing `Cut_Image_Use` itself.
- Screen rendering is paused during driving. Image transmission is still available; further CPU optimization can gate it during races only after confirming it is not required.

### Road Elements

- `zebra.cpp`: zebra detection and delayed stop. `zebra_target_laps=1` means one-lap target; stop delay uses encoder progress.
- `circle.cpp`, `cross.cpp`: roundabout and intersection.
- `po_judge()` / `barrier_flag`: slope-related logic; naming is confusing because `zhang_ai_*` obstacle code exists but is currently not called.
- In `camera.cpp`, red-block processing is currently suppressed only by S-curve and slope/barrier states. It may run alongside zebra/cross/circle until red-block flow becomes exclusive.

## Red Block: Current Design

Relevant files: `project/code/redblock.cpp`, `redblock.h`, `camera.cpp`, `ncnn_infer.cpp`.

### Perception

```text
frame_rgb BGR
-> HSV dual red thresholds
-> search polygon
-> open + close morphology
-> contour selection
-> area / aspect / fill filters
-> largest accepted red contour
```

The contour supplies `redblock_area`, `redblock_x/y`, `redblock_center_x/y`, `redblock_width/height`.

### New Red-Brick Gate (Latest Change)

Goal: red bricks at left/right road edges must not cause red-block slowdown or model inference. Central real red blocks must remain safer than false rejection.

Before setting `redblock_flag`, the code maps the red contour center from raw RGB to the line-follow grid and checks:

```text
both current left/right edges valid
lane width >= 40
red center in outermost 12% of lane width
```

Only when all three are true does it clear the candidate as a side brick. Any uncertain case is passed into the original red-block flow. The gate runs only in `RB_DEC_IDLE` or `RB_DEC_CONFIRMING`; a confirmed red block is never cancelled merely because its projection moves as the vehicle approaches.

Parameters in `redblock.cpp`:

```cpp
REDBLOCK_SIDE_IGNORE_RATIO = 0.12f
REDBLOCK_SIDE_IGNORE_MIN_LANE_WIDTH = 40
REDBLOCK_SIDE_IGNORE_LOG_INTERVAL = 8
```

Expected test logs:

```text
Side red brick: [RB_GATE] side_brick_ignore ...
                no [RB_DEC] redblock_seen
                no [RB_MOT] save speed

Central red block: normal [RB_DEC] redblock_seen -> confirmed -> low_speed_settle
```

This is uncompiled on the Windows host. Test it before changing thresholds. If a real central red block is rejected, reduce the side-ignore ratio. Do not widen it casually.

### Decision and Recognition

Decision states currently are:

```text
RB_DEC_IDLE
RB_DEC_CONFIRMING
RB_DEC_LOW_SPEED_SETTLE
RB_DEC_MODEL_RECOGNIZING
RB_DEC_MOTION_ACTIVE
```

Current flow:

```text
red contour -> confirming -> save normal speed
-> low-speed settle (3 frames, speed 40)
-> rolling NCNN recognition / vote
-> vehicle: line follow
-> suppliers: right bypass
-> weapon: left bypass
-> invalid/lost/unknown: conservative left bypass fallback
```

There is no current reverse-brake state despite stale terms in `CONTEXT.md`. Treat the source code as truth.

### NCNN Model

- `project/code/ncnn_infer.cpp`
- Runtime files must be placed beside board executable:
  - `tiny_classifier_fp32.ncnn.param`
  - `tiny_classifier_fp32.ncnn.bin`
- Input: 96x96 BGR ROI built relative to red base.
- Current fine classes:

```text
suppliers/jijiubao
suppliers/wangyuan
vehicle/jiuhu
vehicle/zhuangjia
weapon/buqiang
weapon/shouliu
weapon/shouqiang
```

- Fine-to-coarse map is `suppliers / vehicle / weapon`.
- `weapon/c4` and `weapon/shuzhuang` were removed from current model mapping.
- Model has no `red_brick` class. The side-brick gate is intentionally rule-based and runs before slowdown.

### Bypass Risk

Visual bypass remains the highest-risk subsystem. Vehicle has repeatedly turned left out of bounds after a `weapon -> left_bypass` result.

Current visual phases:

```text
SEEK_BOUNDARY -> BOUNDARY_HOLD -> BLEND_BACK -> RECOVER
```

It rewrites `Center_point[]` from selected boundary data. Directly setting centerline equal to `l_border[]`/`r_border[]` is unsafe; historical logs show boundary loss/distortion can reverse the control direction. The existing detailed analysis is in:

- `docs/handoff-redblock-visual-phase-refactor-2026-07-16.md`

Do not refactor red perception, NCNN mapping, and visual bypass in the same step. When visual bypass is revisited, use a boundary-derived target with explicit safety offset, then create a deterministic test seam for phase transitions.

## TCP PID Tuning

Firmware files: `project/code/pid_tune_tcp.cpp/.h`.

Architecture:

```text
Board: TCP client
Host tuner: TCP server
Default server: 192.168.43.9:9091
```

Commands include `STATUS`, `HELP`, `SAVE`, `GO/START`, `STOP`, `STREAM ON/OFF`, `SIDE L/R`, speed/position loop commands, independent left/right speed PID setters, and position PID setters.

`SAVE` writes current values through menu settings. It saves the currently active parameters, not automatically the best run. Host tuner should track best metrics and send its chosen best values followed by `SAVE`.

Important: `PID_TUNE_TCP_ENABLE` is currently `0` in `pid_tune_tcp.h`; set it to `1` only for TCP tuning builds.

`GO/START` uses `key_set_go_flag()` so delayed physical-key toggling does not undo the TCP launch command.

AI host project: `C:\Users\86189\Desktop\pid-ai\llm-pid-tuner-repo`.

## Build and Test State

- No reliable Windows cross-build is available in this session.
- Local CMake caches point to old paths and use unavailable generators/toolchains. Do not trust `project/out` artifacts.
- Native MinGW syntax attempt stops at LoongOS-only `<sys/ioctl.h>` dependencies, before meaningful target compilation.
- User compiles in their own LoongOS/cross environment.
- `git diff --check -- project/code/redblock.cpp` passed after latest gate edit.
- The working tree has broad user changes and untracked files. Never reset or clean it globally.

## Recommended Optimization Order

1. Compile latest source on target environment. Fix actual compiler errors only; do not refactor in the same pass.
2. Test side-brick gate with one side red brick and one central real red block. Capture `[RB_GATE]`, `[RB_DEC]`, and speed logs.
3. Create lightweight pure-function test seam for red-block geometric gate and visual-bypass phase transitions. Use `tdd` only once these seams exist.
4. Refactor red-block module internally into perception, decision, motion sections/functions. Keep external API stable. Do not split files until behavior is stable.
5. Redesign visual bypass around a safe boundary offset. Validate each phase separately with logs and image output.
6. Measure CPU before optimization. Highest likely safe wins: suppress race-time debug prints, keep IPS paused after GO, optionally gate image transmission during racing, avoid duplicate color conversions/scans.
7. Reconcile parameter comments with actual values, especially gyroscope sampling and old `CONTEXT.md` vocabulary.
8. Only after vehicle behavior is stable, consider larger cleanup: naming, module extraction, stale build artifact handling, and test harnesses.

## Useful Logs

```text
[RB_GATE]          side brick gate
[RB_DEC]           red-block state / model decision
[RB_MOT]           saved/restored speed
[RB_VIS]           bypass phase and visual control
[RB_VIS_ERR]       centerline/boundary control evidence
[RB_VIS_DIF]       differential-speed evidence
[PID-TCP]          firmware TCP activity
```

## Previous Handoffs

- `docs/handoff-new-code-compare-2026-07-08.md`: earlier code comparison and subsystems.
- `docs/handoff-redblock-visual-phase-refactor-2026-07-16.md`: deep visual bypass failure analysis and proposed phase refactor.

## Suggested Skills for Next Session

- `caveman`: user communication mode.
- `embedded-systems`: firmware/control/resource changes.
- `diagnose`: any real-car wrong behavior; build a log-based reproduction loop first.
- `tdd`: only after extracting pure testable logic from camera/red-block state code.
- `improve-codebase-architecture`: after behavior is measured and stable, for module boundaries and naming cleanup.
