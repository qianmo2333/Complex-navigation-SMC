% 初始化障碍物的初始位置和速度
obstacles = struct('x', [2, 1, 3, 4], 'y', [2, 2, 3, 4], ...
                   'vx', [0.1, -0.1, 0.1, 0.2], 'vy', [0, 0.1, -0.1, 0.1]);

% 初始化参数
d_safe = 1.9; % 安全距离
gamma = 0.3;      % 控制参数 γ
delta = 0.9;      % 控制参数 δ
v = 1;
u_max = 2;
v_star = gamma * delta;
x = 0; % 小车初始位置
y = 0;
theta = 0; % 小车初始方向

dt = 0.1;
t_max = 20;
num_steps = t_max / dt;
target_position = [3, 4]; % 目标位置
min_distance_to_target = 0.05; % 目标距离阈值

% 记录数据
x_history = zeros(num_steps, 1);
y_history = zeros(num_steps, 1);
obstacle_traj = struct('x', zeros(num_steps, length(obstacles.x)), ...
                       'y', zeros(num_steps, length(obstacles.y)));
distances_history = zeros(num_steps, length(obstacles.x));

% 初始化障碍物的轨迹数组
for j = 1:length(obstacles.x)
    obstacle_traj.x(1, j) = obstacles.x(j);
    obstacle_traj.y(1, j) = obstacles.y(j);
end

prev_d_t = inf;

% 仿真主循环
for i = 2:num_steps
    % 更新障碍物位置
    for j = 1:length(obstacles.x)
        obstacles.x(j) = obstacles.x(j) + obstacles.vx(j) * dt;
        obstacles.y(j) = obstacles.y(j) + obstacles.vy(j) * dt;
        obstacle_traj.x(i, j) = obstacles.x(j);
        obstacle_traj.y(i, j) = obstacles.y(j);
    end
    
    % 判断是否到达目标点
    distance_to_target = sqrt((x - target_position(1))^2 + (y - target_position(2))^2);
    if distance_to_target < min_distance_to_target
        fprintf('Target reached at step %d (time = %.2f s)\n', i, i * dt);
        x_history(i:end) = x;
        y_history(i:end) = y;
        break;
    end
    
    % 计算到最近障碍物的距离
    distances = sqrt((x - obstacles.x).^2 + (y - obstacles.y).^2);
    d_t = min(distances);
    distances_history(i, :) = distances;
    if i > 1
        d_t_dot = (d_t - prev_d_t) / dt;
    else
        d_t_dot = 0;
    end
    prev_d_t = d_t;
    
    % 判断是否可以直线前往目标
    if d_t > d_safe + 0.2
        % 朝向目标
        theta_to_target = atan2(target_position(2) - y, target_position(1) - x);
        x = x + v * cos(theta_to_target) * dt;
        y = y + v * sin(theta_to_target) * dt;
        theta = theta_to_target;
        fprintf('Step %d: Decision - Moving Straight to Target\n', i);
    else
        % 启用避障控制（滑模控制律）
        z = d_t - d_safe;
        if abs(z) > delta
            chi_z = v_star * sign(z);
        else
            chi_z = gamma * z;
        end
        u = u_max * sign(d_t_dot + chi_z); % 控制角速度
        % 更新小车状态
        x = x + v * cos(theta) * dt;
        y = y + v * sin(theta) * dt;
        theta = theta + u * dt;
        fprintf('Step %d: Decision - Obstacle Avoidance\n', i);
    end
    fprintf('Step %d: Obstacle 1 -> x: %.2f, y: %.2f, vx: %.2f, vy: %.2f\n', ...
        i, obstacles.x(1), obstacles.y(1), obstacles.vx(1), obstacles.vy(1));

    % 保存当前状态
    x_history(i) = x;
    y_history(i) = y;
end

% 图 1：小车轨迹和目标
figure;
hold on;
plot(x_history, y_history, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Robot Trajectory');
plot(target_position(1), target_position(2), 'o', 'MarkerSize', 10, 'LineWidth', 2, ...
     'DisplayName', 'Target'); % 绘制目标点镂空圆
legend('Location', 'best');
title('Robot Trajectory');
xlabel('X Position');
ylabel('Y Position');
grid on;
axis equal;
hold off;

% 图 2：障碍物轨迹
figure;
hold on;
colors = lines(length(obstacles.x)); % 定义不同障碍物颜色
for j = 1:length(obstacles.x)
    % 去掉轨迹中的无效点 (0,0)
    valid_indices = ~(obstacle_traj.x(:, j) == 0 & obstacle_traj.y(:, j) == 0);
    traj_x = obstacle_traj.x(valid_indices, j);
    traj_y = obstacle_traj.y(valid_indices, j);
    
    % 绘制障碍物轨迹
    plot(traj_x, traj_y, '--', 'Color', colors(j, :), ...
         'LineWidth', 1.5, 'DisplayName', ['Obstacle ' num2str(j) ' Trajectory']);
    % 绘制起点
    plot(traj_x(1), traj_y(1), 'x', 'Color', colors(j, :), ...
         'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', ['Obstacle ' num2str(j) ' Start']);
    % 绘制终点
    plot(traj_x(end), traj_y(end), 'o', 'Color', colors(j, :), ...
         'MarkerSize', 8, 'MarkerFaceColor', colors(j, :), 'DisplayName', ['Obstacle ' num2str(j) ' End']);
end
legend('Location', 'best');
title('Obstacle Trajectories');
xlabel('X Position');
ylabel('Y Position');
grid on;
axis equal;
hold off;

% 图 3：小车轨迹和目标 + 障碍物轨迹
figure;
hold on;

% 绘制小车轨迹
plot(x_history, y_history, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Robot Trajectory');

% 绘制目标点
plot(target_position(1), target_position(2), 'o', 'MarkerSize', 10, 'LineWidth', 2, ...
     'DisplayName', 'Target'); % 绘制目标点镂空圆

% 定义障碍物颜色
colors = lines(length(obstacles.x));

% 绘制障碍物轨迹
for j = 1:length(obstacles.x)
    % 去掉轨迹中的无效点 (0,0)
    valid_indices = ~(obstacle_traj.x(:, j) == 0 & obstacle_traj.y(:, j) == 0);
    traj_x = obstacle_traj.x(valid_indices, j);
    traj_y = obstacle_traj.y(valid_indices, j);
    
    % 绘制障碍物轨迹
    plot(traj_x, traj_y, '--', 'Color', colors(j, :), ...
         'LineWidth', 1.5, 'DisplayName', ['Obstacle ' num2str(j) ' Trajectory']);
    % 绘制起点
    plot(traj_x(1), traj_y(1), 'x', 'Color', colors(j, :), ...
         'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', ['Obstacle ' num2str(j) ' Start']);
    % 绘制终点
    plot(traj_x(end), traj_y(end), 'o', 'Color', colors(j, :), ...
         'MarkerSize', 8, 'MarkerFaceColor', colors(j, :), 'DisplayName', ['Obstacle ' num2str(j) ' End']);
end

% 添加图例和图表设置
legend('Location', 'best');
title('Robot and Obstacle Trajectories');
xlabel('X Position');
ylabel('Y Position');
grid on;
axis equal;
hold off;

