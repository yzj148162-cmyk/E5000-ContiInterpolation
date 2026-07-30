function h = draw_cylinder_between_points(p1, p2, R, faceColor, alphaVal)
% DRAW_CYLINDER_BETWEEN_POINTS 在两点之间画圆柱
% 输入:
%   p1: 起点坐标 (底面圆心) [x, y, z]
%   p2: 终点坐标 (顶面圆心) [x, y, z]
%   R:  圆柱半径
%   faceColor: (可选) 颜色，例如 'b', 'r', [0.5 0.5 0.5]
%   alphaVal:  (可选) 透明度 (0-1)
% 输出:
%   h: 图形对象的句柄 (包含侧面和两个端面)

    if nargin < 4, faceColor = [0.8 0.8 0.8]; end % 默认灰色
    if nargin < 5, alphaVal = 1; end            % 默认不透明

    % 1. 计算轴向量和长度
    p1 = p1(:); % 确保列向量
    p2 = p2(:);
    vec = p2 - p1;
    L = norm(vec);
    
    if L < 1e-10
        warning('起点和终点重合，无法绘制圆柱。');
        return;
    end
    
    % 2. 构建圆柱的局部坐标系
    % 我们需要找到两个与 vec 垂直的向量 (n1, n2) 来画圆
    % 使用 null 函数求 vec 的零空间 (即垂直空间)
    Basis = null(vec'); % 返回 3x2 矩阵，列向量与 vec 垂直且互相垂直
    n1 = Basis(:, 1);
    n2 = Basis(:, 2);
    
    % 3. 生成圆柱侧面数据
    theta = linspace(0, 2*pi, 40); % 圆周细分
    
    % 计算底部圆周点
    x_bot = p1(1) + R * cos(theta) * n1(1) + R * sin(theta) * n2(1);
    y_bot = p1(2) + R * cos(theta) * n1(2) + R * sin(theta) * n2(2);
    z_bot = p1(3) + R * cos(theta) * n1(3) + R * sin(theta) * n2(3);
    
    % 计算顶部圆周点
    x_top = p2(1) + R * cos(theta) * n1(1) + R * sin(theta) * n2(1);
    y_top = p2(2) + R * cos(theta) * n1(2) + R * sin(theta) * n2(2);
    z_top = p2(3) + R * cos(theta) * n1(3) + R * sin(theta) * n2(3);
    
    % 组合侧面坐标矩阵 (2 x N)
    X = [x_bot; x_top];
    Y = [y_bot; y_top];
    Z = [z_bot; z_top];
    
    % 4. 绘图
    hold on;
    
    % 绘制侧面
    h_side = surf(X, Y, Z, 'FaceColor', faceColor, ...
                  'EdgeColor', 'none', 'FaceAlpha', alphaVal);
              
    % 绘制底面 (封口)
    h_bot = patch(x_bot, y_bot, z_bot, faceColor, ...
                  'EdgeColor', 'none', 'FaceAlpha', alphaVal);
              
    % 绘制顶面 (封口)
    h_top = patch(x_top, y_top, z_top, faceColor, ...
                  'EdgeColor', 'none', 'FaceAlpha', alphaVal);
    
    % 组合句柄返回
    h = [h_side; h_bot; h_top];
    
    % 设置光照使圆柱更有立体感
    if isempty(findobj(gca, 'Type', 'Light'))
        camlight; lighting gouraud;
    end
    axis equal; grid on;
end