% 起点和终点定义
start_position = [0, 0];  % 初始位置
target_position = [5, 4];  % 目标位置

% 障碍物定义
num_obstacles = 3;
boundary = struct('x', [], 'y', []);
boundary(1:num_obstacles) = struct('x', 0, 'y', 0);
boundary(1).x = 1; boundary(1).y = 1;
boundary(2).x = 2; boundary(2).y = 2;
boundary(3).x = 4; boundary(3).y = 3;

% 计算最短路径（直线距离）
shortest_path_length = sqrt((target_position(1) - start_position(1))^2 + ...
                            (target_position(2) - start_position(2))^2);
disp(['最短路径长度: ', num2str(shortest_path_length)]);

% 绘制图像
figure;
hold on;
% 绘制障碍物
scatter([boundary.x], [boundary.y], 200, 'yellow', 'filled', 'DisplayName', 'Obstacles');
% 绘制起点和终点
scatter(start_position(1), start_position(2), 150, 'green', 'filled', 'DisplayName', 'Start Point');
scatter(target_position(1), target_position(2), 150, 'magenta', 'filled', 'DisplayName', 'Target Point');
% 绘制最短路径直线
plot([start_position(1), target_position(1)], [start_position(2), target_position(2)], ...
    '--r', 'LineWidth', 2, 'DisplayName', 'Shortest Path');

% 图像设置
title('Simplified Shortest Path');
xlabel('X');
ylabel('Y');
legend show;
axis equal;
grid on;
hold off;
