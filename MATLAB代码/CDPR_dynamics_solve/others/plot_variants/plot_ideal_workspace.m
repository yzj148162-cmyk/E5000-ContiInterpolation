function plot_ideal_workspace(ideal_workspace)
    % PLOT_IDEAL_WORKSPACE 绘制4xn矩阵的云图
    % 输入:
    %   ideal_workspace - 4行n列矩阵，每列格式: [x; y; z; value]
    
    % --- 1. 数据提取 ---
    x = ideal_workspace(1, :); % 第1行：X坐标
    y = ideal_workspace(2, :); % 第2行：Y坐标
    z = ideal_workspace(3, :); % 第3行：Z坐标
    v = ideal_workspace(4, :); % 第4行：值大小

    % --- 2. 方式一：绘制二维插值云图 (X-Y 平面) ---
    figure('Name', '二维云图 (X-Y平面)');
    
    % 2.1 生成网格
    num_grid = 100; % 网格细度
    xi = linspace(min(x), max(x), num_grid);
    yi = linspace(min(y), max(y), num_grid);
    [Xi, Yi] = meshgrid(xi, yi);
    
    % 2.2 插值 (将散点值映射到网格上)
    Vi = griddata(x, y, v, Xi, Yi, 'linear'); % 可选: 'linear', 'cubic', 'nearest'
    
    % 2.3 绘图
    pcolor(Xi, Yi, Vi);   % 绘制伪彩色图
    shading interp;        % 平滑着色 (去除网格线)
    colorbar;              % 显示颜色条
    colormap jet;          % 设置颜色映射 (可选: parula, hot, cool)
    
    % 2.4 标签设置
    xlabel('X / m');
    ylabel('Y / m');
    title('CFEL / N');
    xlim([-1 1]);
    ylim([-1 1]);
    % axis equal tight;      % 紧凑显示

    % --- 3. 方式二：绘制三维散点云图 (带颜色映射) ---
    % figure('Name', '三维云图');
    % 
    % % 3.1 绘制散点
    % scatter3(x, y, z, ...
    %     30, ...      % 点的大小
    %     v, ...       % 点的颜色 (由值v决定)
    %     'filled');   % 填充点
    % 
    % % 3.2 修饰
    % colorbar;
    % colormap jet;
    % xlabel('X 坐标');
    % ylabel('Y 坐标');
    % zlabel('Z 坐标');
    % title('三维位置值大小云图');
    % grid on;
    % view(3); % 确保是3D视角
end