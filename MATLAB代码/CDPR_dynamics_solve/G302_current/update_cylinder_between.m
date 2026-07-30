function update_cylinder_between(h_group, p1, p2, R)
    % 1. 几何计算 (必须与 plot 函数完全一致)
    p1 = p1(:); p2 = p2(:);
    vec = p2 - p1;
    L = norm(vec);
    if L < 1e-12, return; end
    
    Basis = null(vec'); n1 = Basis(:, 1); n2 = Basis(:, 2);
    theta = linspace(0, 2*pi, 30); % 确保这里的 30 和 plot 函数里的 30 一致！
    
    x_side = [p1(1) + R*cos(theta)*n1(1) + R*sin(theta)*n2(1); ...
              p2(1) + R*cos(theta)*n1(1) + R*sin(theta)*n2(1)];
    y_side = [p1(2) + R*cos(theta)*n1(2) + R*sin(theta)*n2(2); ...
              p2(2) + R*cos(theta)*n1(2) + R*sin(theta)*n2(2)];
    z_side = [p1(3) + R*cos(theta)*n1(3) + R*sin(theta)*n2(3); ...
              p2(3) + R*cos(theta)*n1(3) + R*sin(theta)*n2(3)];
          
    x_bot = x_side(1,:); y_bot = y_side(1,:); z_bot = z_side(1,:);
    x_top = x_side(2,:); y_top = y_side(2,:); z_top = z_side(2,:);
    
    % 2. 通过 Tag 精确更新对象 (不再盲目循环)
    
    % 更新侧面
    h_side = findobj(h_group, 'Tag', 'CylinderSide');
    if ~isempty(h_side)
        set(h_side, 'XData', x_side, 'YData', y_side, 'ZData', z_side);
    end
    
    % 更新底面 (CapBot)
    h_bot = findobj(h_group, 'Tag', 'CapBot');
    if ~isempty(h_bot)
        set(h_bot, 'XData', x_bot, 'YData', y_bot, 'ZData', z_bot);
    end
    
    % 更新顶面 (CapTop)
    h_top = findobj(h_group, 'Tag', 'CapTop');
    if ~isempty(h_top)
        set(h_top, 'XData', x_top, 'YData', y_top, 'ZData', z_top);
    end
end