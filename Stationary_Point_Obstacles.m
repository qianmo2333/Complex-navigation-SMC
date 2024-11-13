% 初始化参数
d_safe = 2;     % 安全距离
gamma = 2;      % 控制参数 γ
delta = 2;      % 控制参数 δ
v = 1;          % 线速度 (保持常数)
u_max = 1.5;    % 角速度的最大值
v_star = gamma * delta;

% 封装障碍物信息
num_obstacles = 3;  % 障碍物数量
boundary = struct('x', [], 'y', []);
boundary(1:num_obstacles) = struct('x', 0, 'y', 0);
boundary(1).x = 1; boundary(1).y = 1;
boundary(2).x = 2; boundary(2).y = 2;
boundary(3).x = 3; boundary(3).y = 3;

% 小车的初始位置和方向
x = 0;          % 初始 x 位置
y = 0;          % 初始 y 位置
theta = 0;      % 初始方向角

% 仿真参数
dt = 0.1;       % 时间步长
t_max = 20;     % 仿真总时间
num_steps = t_max / dt;

% 记录小车位置和距离信息
x_history = zeros(num_steps, 1);
y_history = zeros(num_steps, 1);
distances_history = zeros(num_steps, num_obstacles);  % 初始化障碍物距离矩阵

prev_d_t = inf;  % 初始化前一步的最小距离

% 仿真循环
for i = 1:num_steps
    % 计算到每个障碍物的距离
    for b = 1:num_obstacles
        distances_history(i, b) = sqrt((x - boundary(b).x)^2 + (y - boundary(b).y)^2);
    end
    
    % 获取到最近障碍物的最小距离
    [d_t, closest_idx] = min(distances_history(i, :));
    
    % 计算距离变化率 d_t_dot
    if i > 1
        d_t_dot = (d_t - prev_d_t) / dt;  % 距离变化率
    else
        d_t_dot = 0;  % 初始时刻的变化率为0
    end
    prev_d_t = d_t;
    
    % 滑模控制律
    z = d_t - d_safe;
    if abs(z) > delta
        chi_z = v_star * sign(z);
    else
        chi_z = gamma * z;
    end
    u = u_max * sign(d_t_dot + chi_z);  % 控制角速度
    
    % 更新小车状态（运动学模型）
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + u * dt;
    
    % 保存当前状态
    x_history(i) = x;
    y_history(i) = y;
end

% 绘制轨迹图和距离变化曲线
figure;

% 子图1：小车轨迹图
subplot(2, 1, 1);
hold on;
plot(x_history, y_history, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Car Trajectory');
scatter([boundary.x], [boundary.y], 100, 'r', 'x', 'DisplayName', 'Obstacles');
title('Car Trajectory');
xlabel('X Position');
ylabel('Y Position');
legend('show');
grid on;
axis equal;
hold off;

% 子图2：小车与障碍物的距离变化
subplot(2, 1, 2);
hold on;
time = (0:num_steps-1) * dt;  % 时间轴
for b = 1:num_obstacles
    plot(time, distances_history(:, b), 'LineWidth', 1.5, 'DisplayName', ['Obstacle ' num2str(b)]);
end
title('Distance to Obstacles Over Time');
xlabel('Time (s)');
ylabel('Distance (m)');
legend('show');
grid on;
hold off;
