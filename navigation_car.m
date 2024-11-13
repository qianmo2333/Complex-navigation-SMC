clc;
clear;
close all;

% 加载新环境地图
run('first_environment.m'); % 运行first environment.m加载地图数据

% 参数设置
v = 1;  % 速度
u_max = 3.0;  % 最大角速度
start_pos = [0, 0];  % 机器人的起点
end_pos = [10, 10];  % 机器人的终点
start_theta = 0;  % 机器人的起始方向
d0 = 2;  % 期望与障碍物边界的距离

% 导航法则参数
gamma = 1;  % 控制参数
delta = 0.5;  % 控制参数
v_star = gamma * delta;

% 仿真参数
dt = 0.1;  % 时间步长
total_time = 100;  % 总仿真时间

% 初始化位置和方向
pos = start_pos;
theta = start_theta;
trajectory = pos;

% 计算两点之间的距离函数
distance = @(p1, p2) sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2);

% 符号函数
sgn = @(x) (x > 0) - (x < 0);

% chi函数
chi = @(z) (abs(z) <= delta) .* (gamma * z) + (abs(z) > delta) .* (v_star * sgn(z));

% 定义平面的大小
x_limit = 10;
y_limit = 10;

% 获取机器人图形数据
data = my_gritsbot_patch();

% 仿真循环
figure;
axis equal;
grid on;
xlim([0 x_limit]);
ylim([0 y_limit]);
xlabel('X');
ylabel('Y');
title('Dubins Car Trajectory Patrolling the Obstacle and Reaching the Target in New Environment');
hold on;

for t = 0:dt:total_time
    u = 0;
    avoid_flag = false;
    
    % 计算避障控制输入
    for i = 1:size(obstacle_center, 1)
        d = distance(pos, obstacle_center(i, :)) - obstacle_radius;
        
        if d < d0
            avoid_flag = true;
            
            % 计算距离的变化率（近似）
            if size(trajectory, 1) < 2
                ddot = 0;
            else
                d_prev = distance(trajectory(end-1, :), obstacle_center(i, :)) - obstacle_radius;
                ddot = (d - d_prev) / dt;
            end
            
            % 计算角速度的控制输入
            u_i = u_max * sgn(ddot + chi(d - d0));
            u = u + u_i;
        end
    end
    % 计算到目标的距离
    target_distance = distance(pos, end_pos);

    % 当距离目标位置很近时，忽略避障逻辑
    if target_distance < 5.0  % 如果距离目标小于1米，优先导航到目标
        avoid_flag = false;  % 忽略避障逻辑
    end

    if avoid_flag
        u = u / size(obstacle_center, 1);  % 平均控制输入
    else
        % 导航到目标位置
        direction = end_pos - pos;
        target_theta = atan2(direction(2), direction(1));
        angle_diff = atan2(sin(target_theta - theta), cos(target_theta - theta));
    
        % 使用比例控制和死区避免振荡
        if abs(angle_diff) < 0.05
            u = 0;  % 如果角度差很小，不调整角度
        else
            u = u_max * (angle_diff / pi);  % 按比例调整角速度
        end
    end
    % 更新位置和方向（确保不超出平面边界）
    pos(1) = min(max(pos(1) + v * cos(theta) * dt, 0), x_limit);
    pos(2) = min(max(pos(2) + v * sin(theta) * dt, 0), y_limit);
    theta = theta + u * dt;
    
    % 记录轨迹
    trajectory = [trajectory; pos];
    
    % 检查是否到达目标位置
    if distance(pos, end_pos) < 0.1
        break;
    end
    
    % 绘制轨迹
    plot(trajectory(:, 1), trajectory(:, 2), '-');  % 绘制轨迹

    % 绘制多个障碍物
    for i = 1:size(obstacle_center, 1)
        rectangle('Position', [obstacle_center(i, 1) - obstacle_radius, obstacle_center(i, 2) - obstacle_radius, 2 * obstacle_radius, 2 * obstacle_radius], ...
                  'Curvature', [1, 1], 'LineStyle', '--');
    end

    % 绘制起点和终点
    plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);  % 绘制起点
    plot(end_pos(1), end_pos(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);  % 绘制终点
    
    % 绘制机器人
    delete(findobj(gca, 'Type', 'Patch'));
    th = theta - pi/2;
    rotation_matrix = [
        cos(th) -sin(th) pos(1);
        sin(th)  cos(th) pos(2);
        0 0 1];
    transformed = data.vertices * rotation_matrix';
    
    patch(... 
        'Vertices', transformed(:, 1:2), ...
        'Faces', data.faces, ...
        'FaceColor', 'flat', ...
        'FaceVertexCData', data.colors, ...
        'EdgeColor','none');  % 绘制小车
    
    drawnow;  % 刷新屏幕以更新绘图
end

% 显示最终位置和方向的调试信息
fprintf('Final Position: (%.2f, %.2f)\n', pos(1), pos(2));
fprintf('Final Orientation: %.2f\n', theta);

% my_gritsbot_patch 函数定义
function [patch_data] = my_gritsbot_patch()
    % Make it facing 0 rads
    robot_width = 0.2;
    robot_height = 0.4; 
    wheel_width = 0.04; 
    wheel_height = 0.08; 
    led_size = 0.02; 
    
    % Helper functions to generate vertex coordinates for a centered
    % rectangle and a helper function to shift a rectangle.
    rectangle = @(w, h) [w/2 h/2 1; -w/2 h/2 1; -w/2 -h/2 1; w/2 -h/2 1];
    shift = @(r, x, y) r + repmat([x, y, 0], size(r, 1), 1);
    
    % Create vertices for body, wheel, and led.
    body = rectangle(robot_width, robot_height);
    wheel = rectangle(wheel_width, wheel_height);
    led = rectangle(led_size, led_size);
    
    % Use pre-generated vertices and shift them around to create a robot
    left_wheel_1 = shift(wheel, -(robot_width + wheel_width)/2, -robot_height/3);
    right_wheel_1 = shift(wheel, (robot_width + wheel_width)/2, -robot_height/3);
    left_wheel_2 = shift(wheel, -(robot_width + wheel_width)/2, robot_height/3);
    right_wheel_2 = shift(wheel, (robot_width + wheel_width)/2, robot_height/3);
    left_led = shift(led,  robot_width/4, robot_height/2 - 2*led_size);
    right_led = shift(led,  -robot_width/4, robot_height/2 - 2*led_size);
    
    % Putting all the robot vertices together
    vertices = [
     body ; 
     left_wheel_1; 
     left_wheel_2;
     right_wheel_1;
     right_wheel_2;
     left_led;
     right_led
    ];

    % Only color the body of the robot. Everything else is black.
    colors = [
     [255, 0, 0]/255; 
     0 0 0;
     0 0 0;
     0 0 0;
     0 0 0;
     1 1 1;
     1 1 1
    ];

    % This seems weird, but it basically tells the patch function which
    % vertices to connect.
    faces = repmat([1 2 3 4 1], 7, 1);
    
    for i = 2:7
       faces(i, :) = faces(i, :) + (i-1)*4;
    end
    
   patch_data = []; 
   patch_data.vertices = vertices;
   patch_data.colors = colors;
   patch_data.faces = faces;
end