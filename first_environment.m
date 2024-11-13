% 创建不规则边界
theta = linspace(0, 2*pi, 100);
x_boundary = cos(theta) + 0.1*sin(3*theta);
y_boundary = 0.9*sin(theta) + 0.2*cos(5*theta);

% 将图案平移到第一象限，并放大到适合的范围
x_boundary = x_boundary * 3 + 5; % 缩放和平移 x 坐标到 0-10 范围
y_boundary = y_boundary * 3 + 5; % 缩放和平移 y 坐标到 0-10 范围

fill(x_boundary, y_boundary, [0.9, 0.9, 0.9]); % 灰色填充
hold on;
axis equal;

% 绘制坐标轴
plot([0, 10], [0, 0], 'k-'); % x 轴
plot([0, 0], [0, 10], 'k-'); % y 轴

% 设置坐标轴范围
xlim([0, 10]);
ylim([0, 10]);

% 标注 x 轴和 y 轴
xlabel('X');
ylabel('Y');

% 添加刻度线
xticks(0:1:10); % 设置 x 轴刻度，每 1 为一个刻度
yticks(0:1:10); % 设置 y 轴刻度，每 1 为一个刻度

% 定义更多的障碍物圆心位置和较小的半径
obstacle_indices = 1:4:100; % 从边界坐标中每隔10个点选择一个点
obstacle_center = [x_boundary(obstacle_indices)', y_boundary(obstacle_indices)']; % 使用边界坐标作为圆心
obstacle_radius = 0.5; % 缩小半径

% 绘制障碍物
for i = 1:size(obstacle_center, 1)
    rectangle('Position', [obstacle_center(i, 1) - obstacle_radius, obstacle_center(i, 2) - obstacle_radius, 2 * obstacle_radius, 2 * obstacle_radius], ...
              'Curvature', [1, 1], 'LineStyle', '--', 'EdgeColor', 'k'); % 用虚线表示障碍物
end

hold off;
