function h_group = plot_cylinder_between(varargin)
    % 1. 解析参数 (保持不变)
    if isa(varargin{1}, 'matlab.graphics.axis.Axes')
        ax = varargin{1}; args = varargin(2:end);
    else
        ax = gca; args = varargin;
    end
    p1 = args{1}; p2 = args{2}; R  = args{3};
    if length(args) > 3, plot_props = args(4:end); else, plot_props = {'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none'}; end

    % 2. 几何计算 (保持不变)
    p1 = p1(:); p2 = p2(:); vec = p2 - p1; L = norm(vec);
    if L < 1e-12, h_group = hggroup('Parent', ax); return; end
    
    Basis = null(vec'); n1 = Basis(:, 1); n2 = Basis(:, 2);
    theta = linspace(0, 2*pi, 30); % 注意：这里的点数必须和 update 函数一致
    
    x_side = [p1(1) + R*cos(theta)*n1(1) + R*sin(theta)*n2(1); p2(1) + R*cos(theta)*n1(1) + R*sin(theta)*n2(1)];
    y_side = [p1(2) + R*cos(theta)*n1(2) + R*sin(theta)*n2(2); p2(2) + R*cos(theta)*n1(2) + R*sin(theta)*n2(2)];
    z_side = [p1(3) + R*cos(theta)*n1(3) + R*sin(theta)*n2(3); p2(3) + R*cos(theta)*n1(3) + R*sin(theta)*n2(3)];
    
    x_bot = x_side(1,:); y_bot = y_side(1,:); z_bot = z_side(1,:);
    x_top = x_side(2,:); y_top = y_side(2,:); z_top = z_side(2,:);
    
    % 3. 创建图形组 (关键修改：增加 Tag)
    h_group = hggroup('Parent', ax);
    
    % 给侧面加 Tag = 'CylinderSide'
    surf(x_side, y_side, z_side, 'Parent', h_group, 'EdgeColor', 'none', ...
        'Tag', 'CylinderSide', ... 
        plot_props{:});
    
    % 给底面加 Tag = 'CapBot'
    patch(x_bot, y_bot, z_bot, 1, 'Parent', h_group, 'EdgeColor', 'none', ...
        'Tag', 'CapBot', ...
        plot_props{:});
        
    % 给顶面加 Tag = 'CapTop'
    patch(x_top, y_top, z_top, 1, 'Parent', h_group, 'EdgeColor', 'none', ...
        'Tag', 'CapTop', ...
        plot_props{:});
end