# camera 中各元素共享部分整理

本文只整理**当前拆分状态下**，仍然由 `camera` 提供、并被各元素模块共享或依赖的部分。

适用范围：
- `project/code/camera.h`
- `project/code/camera.cpp`
- `project/code/circle.cpp`
- `project/code/cross.cpp`
- `project/code/zebra.cpp`

---

## 1. 共享宏与基础类型

这些内容定义在 `project/code/camera.h`，属于各元素都会直接或间接用到的基础环境：

- `ROW` / `COL`
  - 原始图像尺寸宏
- `Cut_ROW` / `Cut_COL`
  - 裁剪图像尺寸宏
- `SEARCH_MAX` / `SEARCH_MIN`
  - 边界搜索范围
- `Direction`
  - 左右边判断用枚举，`compare_border_judge()` 依赖它
- `BEEP`
  - 蜂鸣器设备路径，元素模块里会直接调用
- `ZebraState`
  - 当前斑马线状态机枚举

这些内容虽然不属于某一个元素，但它们是元素代码运行的基础参数，因此仍由 `camera.h` 集中提供。

---

## 2. 图像与边界类共享数据

这部分是所有视觉元素最核心的公共输入，主要定义在：
- `project/code/camera.cpp`
- 对外声明在 `project/code/camera.h`

### 2.1 图像数据

- `Image_Use[COL][ROW]`
  - 原始图像缓存
- `Cut_Image_Use[Cut_ROW][Cut_COL]`
  - 裁剪后的灰度图
- `Canny_Image_Use[COL][ROW]`
  - Canny 图像
- `Canny_Cut_Image_Use[Cut_ROW][Cut_COL]`
  - 裁剪后的 Canny 图

### 2.2 边界/中线相关数据

- `white_length_max[2]`
  - 最长白列位置
- `white_length_max_Num`
  - 最长白列对应列号索引
- `white_length_start`
- `white_length_end`
- `l_start` / `r_start`
  - 左右边界起始行
- `l_effect_num` / `r_effect_num`
  - 左右有效边界点数量
- `l_border[Cut_ROW]` / `r_border[Cut_ROW]`
  - 左右边界数组
- `Center_point[Cut_ROW]`
  - 中线点
- `Center_err[Cut_ROW]`
  - 中线误差
- `width[Cut_ROW]`
  - 赛道宽度

这些变量本质上是**所有元素模块共享的赛道描述层**。当前圆环、十字、斑马线都直接依赖这批数据。

---

## 3. camera 中仍保留的通用处理函数

这些函数目前仍留在 `camera`，但它们并不是只服务于 `camera` 主流程，而是多个元素都会复用的公共能力。

### 3.1 图像获取

定义位置：`project/code/camera.cpp`

- `Get_Use_Image()`
- `Cut_Use_Image()`
- `Get_Use_rgbImage()`
- `Cut_Use_rgbImage()`

作用：准备元素识别所需的图像输入。

### 3.2 通用判断/补线工具

- `calc_diff_zebra()`
  - 差比和函数，当前斑马线直接依赖
- `fill_line()`
  - 通用补线函数，圆环和十字都会直接调用
- `compare_border_judge()`
  - 单边边界判定函数，圆环和十字都会直接调用

### 3.3 基础赛道搜索

- `search_longest_white_col()`
  - 搜最长白列
- `search_border()`
  - 搜左右边界
- `search_center()`
  - 根据边界计算中线

这些函数属于公共底座，不是某一个元素的私有逻辑。

### 3.4 后处理/控制相关公共函数

- `Err_Get()`
  - 误差计算
- `weight_box()`
  - 动态前瞻/权重调整
- `Furthest_judge()`
  - 距离转换
- `protect()`
  - 出界保护
- `chasu_calculation()`
  - 差速计算

这部分虽然更靠近控制层，但当前仍由 `camera` 统一维护，且元素状态会间接影响它们的行为。

---

## 4. 当前各元素对 camera 共享部分的依赖

下面只按你当前已经拆过/检查过的元素来整理。

## 4.1 圆环对 camera 的共享依赖

圆环实现位置：`project/code/circle.cpp`

### 直接依赖的共享数据

- `white_length_max`
- `l_border` / `r_border`
- `l_start` / `r_start`
- `r_effect_num`
- `Straight_track_width`
- `Cut_ROW` / `Cut_COL`
- 十字模块导出的角点状态：`left_up` / `left_down` / `right_up`
- 十字状态：`cross_flag`

### 直接依赖的共享函数/能力

- `fill_line()`
- `compare_border_judge()`
- `BEEP` + `gpio_set_level()`

### 说明

圆环虽然已经拆到 `circle.cpp/.h`，但它仍然深度依赖 `camera` 维护的：
- 边界数组
- 最长白列
- 赛道宽度
- 通用补线/边界判断能力

也就是说，**当前圆环是“功能拆分”，还不是“完全解耦”**。

---

## 4.2 十字对 camera 的共享依赖

十字实现位置：`project/code/cross.cpp`

### 直接依赖的共享数据

- `white_length_max`
- `l_border` / `r_border`
- `l_start` / `r_start`
- `r_effect_num`
- `l_land_flag` / `r_land_flag`
- `Cut_ROW` / `Cut_COL`

### 直接依赖的共享函数

- `fill_line()`
- `compare_border_judge()`

### 说明

十字当前本质上依赖 `camera` 先完成：
1. 最长白列搜索
2. 左右边界搜索

然后十字模块再基于这些结果做：
- 角点搜索
- 十字判定
- 十字补线

所以十字拆分后，仍然属于**建立在 camera 赛道基础数据之上的上层元素逻辑**。

---

## 4.3 斑马线对 camera 的共享依赖

斑马线实现位置：`project/code/zebra.cpp`

### 直接依赖的共享数据

- `Cut_Image_Use`
- `width`
- `Cut_ROW` / `Cut_COL`
- `l_land_num`
  - 这个变量现在归属在 `circle`，但斑马线仍会使用

### 直接依赖的共享函数/能力

- `calc_diff_zebra()`
- `BEEP` + `gpio_set_level()`

### 说明

斑马线对 `camera` 的依赖主要集中在：
- 图像数据
- 宽度数组
- 差比和工具函数

因此它对 `camera` 的耦合比圆环、十字稍浅，但仍没有完全脱离公共图像和工具层。

---

## 5. 当前仍适合留在 camera 的共享部分

如果只从“各元素共享”角度看，当前最适合继续留在 `camera` 的内容主要有四类：

### 5.1 图像输入层

- 图像缓存
- 裁剪图
- Canny 图
- 图像获取函数

### 5.2 赛道基础描述层

- 最长白列
- 左右边界
- 中线
- 宽度
- 有效点统计

### 5.3 通用工具层

- `fill_line()`
- `compare_border_judge()`
- `calc_diff_zebra()`

### 5.4 主流程调度层

- `Camera_Function()`
  - 统一调用各元素
- `weight_box()`
- `Furthest_judge()`
- `Err_Get()`

---

## 6. 当前不适合再算作 camera 私有的内容

从现在的拆分结果看，下面这些内容虽然还通过 `camera.h` 暴露，但实际上已经不是 `camera` 私有：

- 圆环状态：已经主要归 `circle`
- 十字角点与十字状态：已经主要归 `cross`
- 斑马线状态机状态：已经主要归 `zebra`

换句话说，`camera` 当前更像：
- 公共数据提供者
- 主流程调度者
- 通用工具承载者

而不是所有元素逻辑的唯一归属地。

---

## 7. 一句话总结

当前 `camera` 中被各元素共享的部分，核心就是三层：

1. **图像与赛道基础数据**：图像、最长白列、边界、中线、宽度
2. **通用工具函数**：补线、边界判断、差比和
3. **主流程与后处理**：统一调度、误差、前瞻、距离转换

而圆环、十字、斑马线虽然已经开始拆分，但都还建立在这套 `camera` 公共底座之上。
