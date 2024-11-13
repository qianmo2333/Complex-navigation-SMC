% 初始化障碍物的初始位置和速度
obstacles = struct('x', [2, 1, 3, 4], 'y', [2, 2, 3, 4], ...
                   'vx', [0.1, -0.1, 0.1, 0.2], 'vy', [0, 0.1, -0.1, 0.1]);

% 初始化参数
d_safe = 2; % 安全距离
gamma = 2;
delta = 2;
v = 1;
u_max = 1.5;
v_star = gamma * delta;
x = 0; % 小车初始位置
y = 0;
theta = 0; % 小车初始方向
dt = 0.1;
t_max = 20;
num_steps = t_max / dt;

% 记录数据
x_history = zeros(num_steps, 1);
y_history = zeros(num_steps, 1);
obstacle_traj = struct('x', zeros(num_steps, length(obstacles.x)), ...
                       'y', zeros(num_steps, length(obstacles.y)));
distances_history = zeros(num_steps, length(obstacles.x));

% 仿真主循环
prev_d_t = inf;
for i = 1:num_steps
    % 更新障碍物位置
    for j = 1:length(obstacles.x)
        obstacles.x(j) = obstacles.x(j) + obstacles.vx(j) * dt;
        obstacles.y(j) = obstacles.y(j) + obstacles.vy(j) * dt;
        obstacle_traj.x(i, j) = obstacles.x(j);
        obstacle_traj.y(i, j) = obstacles.y(j);
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
    
    % 滑模控制律
    z = d_t - d_safe;
    if abs(z) > delta
        chi_z = v_star * sign(z);
    else
        chi_z = gamma * z;
    end
    u = u_max * sign(d_t_dot + chi_z);
    
    % 更新小车状态
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + u * dt;
    x_history(i) = x;
    y_history(i) = y;
end

% 绘图
figure;

% 子图1：小车轨迹和障碍物轨迹
subplot(2, 1, 1);
hold on;
plot(x_history, y_history, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Robot Trajectory');
for j = 1:length(obstacles.x)
    plot(obstacle_traj.x(:, j), obstacle_traj.y(:, j), '--', 'LineWidth', 1.5, ...
         'DisplayName', ['Obstacle ' num2str(j) ' Trajectory']);
end
legend('Location', 'best');
title('Robot and Obstacles Trajectories');
xlabel('X Position');
ylabel('Y Position');
grid on;
axis equal;

% 子图2：障碍物与小车的距离变化
subplot(2, 1, 2);
hold on;
time = (0:num_steps-1) * dt;
for j = 1:length(obstacles.x)
    plot(time, distances_history(:, j), 'LineWidth', 1.5, 'DisplayName', ['Obstacle ' num2str(j)]);
end
legend('Location', 'best');
title('Robot-Obstacle Distance Profiles');
xlabel('Time (s)');
ylabel('Distance (m)');
grid on;
hold off;
