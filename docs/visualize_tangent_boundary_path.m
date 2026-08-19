%% 红块左绕：切向贴边路径可视化
% 横轴 x 为车辆横向位置（mm），纵轴 s 为前进距离（mm）。
% 轨迹在起点位于普通中线，随后快速靠近左边线，末端与左边线相切。

clear;
close all;
clc;

%% 可调参数
ROAD_WIDTH_MM = 300;              % 赛道宽度，用于画图；不参与车端控制
PATH_LENGTH_MM = 900;             % 从普通中线靠到左边线的前进距离
ROAD_CURVE_AMPLITUDE_MM = 35;     % 赛道自身横向弯曲；设为 0 可看直道效果
SAMPLE_COUNT = 240;               % 绘图采样数，越大曲线越平滑
CURVE_POWER_LIST = [3.0, 5.0, 8.0, 12.0]; % p 越大，前段越快向左靠边；p 必须大于 1
SELECTED_POWER = 12.0;            % 加粗显示的当前车端使用曲线

%% 赛道边线和普通中线
s = linspace(0, PATH_LENGTH_MM, SAMPLE_COUNT);
t = s / PATH_LENGTH_MM;
road_center = ROAD_CURVE_AMPLITUDE_MM * sin(pi * t / 2);
left_boundary = road_center - ROAD_WIDTH_MM / 2;
right_boundary = road_center + ROAD_WIDTH_MM / 2;
normal_center = road_center;

% 左绕切向贴边轨迹：
% t=0 时为普通中线；t=1 时与左边线重合。
% (1-t)^p 的导数在 t=1 为 0（p>1），因此末端与边线平行。
target_paths = zeros(numel(CURVE_POWER_LIST), SAMPLE_COUNT);
for index = 1:numel(CURVE_POWER_LIST)
    curve_power = CURVE_POWER_LIST(index);
    target_paths(index, :) = left_boundary + ...
        (normal_center - left_boundary) .* (1 - t) .^ curve_power;
end

%% 图 1：赛道内的切向贴边轨迹
figure('Color', 'w', 'Name', 'Tangent boundary approach');
plot(left_boundary, s, 'k', 'LineWidth', 2.0);
hold on;
plot(right_boundary, s, 'k', 'LineWidth', 2.0);
plot(normal_center, s, '--', 'Color', [0.45, 0.45, 0.45], 'LineWidth', 1.4);

colors = lines(numel(CURVE_POWER_LIST));
for index = 1:numel(CURVE_POWER_LIST)
    curve_power = CURVE_POWER_LIST(index);
    line_width = 1.5;
    if abs(curve_power - SELECTED_POWER) < 1e-6
        line_width = 3.0;
    end
    plot(target_paths(index, :), s, 'Color', colors(index, :), ...
        'LineWidth', line_width);
end

plot(normal_center(1), 0, '^', 'MarkerSize', 9, ...
    'MarkerFaceColor', [0.85, 0.2, 0.1], 'MarkerEdgeColor', 'k');
grid on;
axis equal;
xlim([min(left_boundary) - 35, max(right_boundary) + 35]);
ylim([0, PATH_LENGTH_MM]);
xlabel('横向位置 x (mm)');
ylabel('前进距离 s (mm)');
title('左绕切向贴边：前段靠边，末端与左边线平行');
legend_entries = {'左边线', '右边线', '普通中线'};
for index = 1:numel(CURVE_POWER_LIST)
    legend_entries{end + 1} = sprintf('切向路径 p = %.1f', CURVE_POWER_LIST(index));
end
legend_entries{end + 1} = '车辆当前位置';
legend(legend_entries, 'Location', 'eastoutside');

%% 图 2：车辆目标到左边线的距离，验证末端渐近贴边趋势
figure('Color', 'w', 'Name', 'Distance to left boundary');
hold on;
for index = 1:numel(CURVE_POWER_LIST)
    distance_to_left = target_paths(index, :) - left_boundary;
    plot(s, distance_to_left, 'Color', colors(index, :), 'LineWidth', 2.0);
end
grid on;
xlabel('前进距离 s (mm)');
ylabel('目标到左边线的横向距离 (mm)');
title('目标到左边线的距离：终点收敛到 0');
legend(arrayfun(@(value) sprintf('p = %.1f', value), ...
    CURVE_POWER_LIST, 'UniformOutput', false), 'Location', 'northeast');

fprintf('切向贴边路径已生成。推荐先观察 p = %.1f。\n', SELECTED_POWER);
fprintf('p 增大：前段更快靠边；p 减小但大于 1：靠边更柔和。\n');
