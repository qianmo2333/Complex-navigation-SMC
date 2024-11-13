% 初始化参数
d_safe = 2;     % 安全距离
gamma = 0.2;      % 控制参数 γ
delta = 0.8;      % 控制参数 δ
v = 1;          % 线速度 (保持常数)
u_max = 2;    % 角速度的最大值
v_star = gamma * delta;
R = v/u_max;

% 封装障碍物信息
num_obstacles = 3;  % 障碍物数量
boundary = struct('x', [], 'y', []);
boundary(1:num_obstacles) = struct('x', 0, 'y', 0);
boundary(1).x = 1; boundary(1).y = 1;
boundary(2).x = 2; boundary(2).y = 2;
boundary(3).x = 4; boundary(3).y = 3;

% 小车的初始位置和方向
x = 0;          % 初始 x 位置
y = 0;          % 初始 y 位置
theta = 0;      % 初始方向角

target_position = [5, 4];  % 目标位 T (x, y)


% 仿真参数
dt = 0.1;       % 时间步长
t_max = 20;     % 仿真总时间
num_steps = t_max / dt;

% 初始化轨迹记录
x_history = zeros(num_steps, 1);
y_history = zeros(num_steps, 1);

% 记录初始位置
x_history(1) = x;
y_history(1) = y;

% 记录小车位置和距离信息
x_history = zeros(num_steps, 1);
y_history = zeros(num_steps, 1);
distances_history = zeros(num_steps, num_obstacles);  % 初始化障碍物距离矩阵

prev_d_t = inf;  % 初始化前一步的最小距离

% 仿真循环
for i = 2:num_steps
    % 判断是否精确到达目标
    if abs(x - target_position(1)) <= 0.05 && abs(y - target_position(2)) <= 0.05
        x_history(i:end) = x;
        y_history(i:end) = y;
        break;
    else
        % 计算到每个障碍物的距离
        for b = 1:num_obstacles
            distances_history(i, b) = sqrt((x - boundary(b).x)^2 + (y - boundary(b).y)^2);
        end
        
        % 获取到最近障碍物的最小距离
        [d_t, closest_idx] = min(distances_history(i, :));
        distance_to_target = sqrt((x - target_position(1))^2 + (y - target_position(2))^2);
        fprintf('Step %d: Closest obstacle is #%d with distance d_t = %.2f, Distance to target = %.2f\n', ...
        i, closest_idx, d_t, distance_to_target);
        % 判断是否可以直线前往目标
        if d_t > d_safe - 1
            u = 0;
            % 直线前往目标
            theta_to_target = atan2(target_position(2) - y, target_position(1) - x);  % 朝向目标的方向角
            x = x + v * cos(theta_to_target) * dt;
            y = y + v * sin(theta_to_target) * dt;
            theta = theta_to_target;  % 更新方向角
            fprintf('Step %d: Decision - Straight to Target\n', i);

        else
            % 启用避障控制（滑模控制律）
            z = d_t - d_safe;
            % 计算距离变化率 d_t_dot
            if i > 1
                d_t_dot = (d_t - prev_d_t) / dt;  % 距离变化率
            else
                d_t_dot = 0;  % 初始时刻的变化率为0
            end
            prev_d_t = d_t;
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
            fprintf('Step %d: Decision - Obstacle Avoidance\n', i);

        end
    % 保存当前状态
    x_history(i) = x;
    y_history(i) = y;
    end 
end

% 初始化轨迹长度
trajectory_length = 0;

% 遍历每一步，计算相邻位置之间的距离
for i = 2:num_steps
    % 如果后续点没有更新（说明仿真已停止），则退出循环
    if x_history(i) == 0 && y_history(i) == 0
        break;
    end
    % 计算相邻点之间的欧几里得距离
    dx = x_history(i) - x_history(i-1);
    dy = y_history(i) - y_history(i-1);
    trajectory_length = trajectory_length + sqrt(dx^2 + dy^2);
end

% 输出轨迹长度
fprintf('Estimated Trajectory Length: %.4f units\n', trajectory_length);



figure; % 创建一个新的图形窗口
hold on; % 开启保持模式
plot(x_history, y_history, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Car Trajectory'); % 绘制小车轨迹
scatter([boundary.x], [boundary.y], 100, 'r', 'x', 'DisplayName', 'Obstacles'); % 绘制障碍物
plot(target_position(1), target_position(2), 'bo', 'MarkerSize', 8, 'DisplayName', 'Target'); % 绘制目标点

% 绘制以障碍物为圆心的虚线圆
theta = linspace(0, 2*pi, 100); % 用于绘制圆的角度
for b = 1:num_obstacles
    x_circle = boundary(b).x + (d_safe-1) * cos(theta); % 计算圆的 x 坐标
    y_circle = boundary(b).y + (d_safe-1) * sin(theta); % 计算圆的 y 坐标
    plot(x_circle, y_circle, '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1.2); % 绘制虚线圆
end

% 图形标题和坐标轴标签
title('Car Trajectory');
xlabel('X Position');
ylabel('Y Position');

% 添加图例
legend('Car Trajectory', 'Obstacles', 'Target', 'Location', 'best'); % 指定需要显示的图例
grid on; % 显示网格
axis equal; % 设置坐标轴比例相等
hold off; % 关闭保持模式
