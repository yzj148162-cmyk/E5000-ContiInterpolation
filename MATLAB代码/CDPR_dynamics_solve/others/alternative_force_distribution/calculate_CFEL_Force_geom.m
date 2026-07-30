function [cable_force,num_vertex,CFEL,ideal_cf_exitflag] = calculate_CFEL_Force_geom(force_ee,moment_ee,jaco,force_min,force_max,ij_indx)
% 输出参数新增：
% L_min: 交点到形心的最短距离
% intersections: 交点坐标矩阵 [x, y, 斜率]
ideal_cf_exitflag = 1;
CFEL = 0;
x_vertex=zeros(120,2);
i2=0;
A=0;
sum1=0;
sum2=0;
% Particular solution of 'J'*F+w=0'
t_p=-jaco*inv(jaco'*jaco)*[force_ee;moment_ee];
% Null space of J'
n_s=null(jaco');
if (size(n_s,2)~=2)
   error('singular position')
end

N_C2 = inv(n_s(ij_indx,:));
n_star = n_s*N_C2;

tp_star = t_p - n_star*t_p(ij_indx);

% combntns has been removed in MATLAB R2023b, use nchoosek instead
Aeq1=nchoosek([n_star(:,1);n_star(:,1)],2);
Aeq2=nchoosek([n_star(:,2);n_star(:,2)],2);
Beq0=nchoosek([force_min-tp_star;force_max-tp_star],2);

% solve the C16,2=120 linear equations
for i1=1:120
   Aeq=[(Aeq1(i1,:))' (Aeq2(i1,:))'];
   Beq=(Beq0(i1,:))';
% if the two line are parallel
   if (abs(det(Aeq))<1e-6)
      continue
   end
   x=(Aeq)\Beq;
% examine x_vertex meet the inequation constrain
% when compare two numbers by matlab, calculation tolerance must be considered
   if (((n_star*x)<=(force_max-tp_star+1e-5))&((n_star*x)>=(force_min-tp_star-1e-5)))
   i2=i2+1;
   x_vertex(i2,:)=x;
   end
end
% calculate the area of the polygon
% sort all the vertex

if i2<3  %----力分配失败
    ideal_cf_exitflag = -1;
    cable_force = 0;
    num_vertex = 0;
else


    xs_vertex=sort_vertex(x_vertex(1:i2,:));
    num_vertex=size(xs_vertex,1);
    % increase one line; the last point is coinsidented with the first point
    xs_vertex(i2+1,:)=xs_vertex(1,:);
    for i3=1:i2
        A=xs_vertex(i3,1)*xs_vertex(i3+1,2)-xs_vertex(i3+1,1)*xs_vertex(i3,2)+A;
    end
    A=A/2;
    % calculate the bary center
    for i4=1:i2
        sum1=(xs_vertex(i4,1)+xs_vertex(i4+1,1))*...
            (xs_vertex(i4,1)*xs_vertex(i4+1,2)-xs_vertex(i4+1,1)*xs_vertex(i4,2))+sum1;
        sum2=(xs_vertex(i4,2)+xs_vertex(i4+1,2))*...
            (xs_vertex(i4,1)*xs_vertex(i4+1,2)-xs_vertex(i4+1,1)*xs_vertex(i4,2))+sum2;
    end
    b_c=[sum1/6/A;sum2/6/A];
    % cable force distribution
    cable_force=tp_star+n_star*b_c;
end
% ===================== 新增功能核心代码 =====================
if i2 > 0
    bx = b_c(1);
    by = b_c(2);
    m_list = [1, -1]; % 两条直线的斜率
    intersections = []; % 存储交点 [x, y, 斜率]
    
    % 遍历两条直线
    for m_idx = 1:length(m_list)
        m = m_list(m_idx);
        c = by - m * bx; % 直线方程: m*x - y + c = 0
        
        % 遍历凸多边形每条边
        for k = 1:i2
            P1 = xs_vertex(k, :);
            P2 = xs_vertex(k+1, :);
            x1 = P1(1); y1 = P1(2);
            x2 = P2(1); y2 = P2(2);
            dx = x2 - x1;
            dy = y2 - y1;
            
            % 计算直线与线段交点参数
            denominator = m * dx - dy;
            numerator = -(m * x1 - y1 + c);
            
            % 跳过平行情况
            if abs(denominator) < 1e-8
                continue;
            end
            
            t = numerator / denominator;
            
            % 检查交点是否在线段上
            if t >= -1e-5 && t <= 1 + 1e-5
                t_clamped = max(0, min(1, t));
                x_inter = x1 + t_clamped * dx;
                y_inter = y1 + t_clamped * dy;
                
                % 检查重复交点
                is_duplicate = false;
                for p = 1:size(intersections,1)
                    if norm([x_inter, y_inter] - intersections(p,1:2)) < 1e-5
                        is_duplicate = true;
                        break;
                    end
                end
                if ~is_duplicate
                    intersections = [intersections; x_inter, y_inter, m];
                end
            end
        end
    end
    
    % 计算交点到形心的距离
    num_inter = size(intersections, 1);
    distances = zeros(num_inter, 1);
    for p = 1:num_inter
        distances(p) = norm(intersections(p,1:2) - [bx, by]);
    end
    
    % 找到最短距离
    [L_min, min_idx] = min(distances);

    CFEL = L_min/1.41421;
    
    % % 输出结果
    % fprintf('=== 新增功能结果 ===\n');
    % fprintf('形心坐标：(%.4f, %.4f)\n', bx, by);
    % fprintf('交点数量：%d\n', num_inter);
    % for p = 1:num_inter
    %     fprintf('交点%d：(%.4f, %.4f)，斜率：%d，距离：%.4f\n', ...
    %         p, intersections(p,1), intersections(p,2), intersections(p,3), distances(p));
    % end
    % fprintf('最短距离L_min：%.4f\n', L_min);
    
    % % ===================== 绘制增强图形 =====================
    % figure('Name','张力可行域 + 斜率±1直线分析','Color','w');
    % hold on; grid on; axis equal;
    % 
    % % 绘制原约束直线
    % x_ext = linspace(-1e6, 1e6, 2);
    % n_rope = length(n_star(:,1));
    % for k = 1:n_rope
    %     a = n_star(k,1);
    %     b = n_star(k,2);
    %     fmin_k = force_min - tp_star(k);
    %     fmax_k = force_max - tp_star(k);
    %     y_lb = (fmin_k - a * x_ext) / b;
    %     y_ub = (fmax_k - a * x_ext) / b;
    %     plot(x_ext, y_lb, 'm--', 'LineWidth', 1.0, 'HandleVisibility','off');
    %     plot(x_ext, y_ub, 'c--', 'LineWidth', 1.0, 'HandleVisibility','off');
    % end
    % 
    % % 绘制凸多边形
    % fill(xs_vertex(:,1), xs_vertex(:,2), [0, 0.7, 0], 'FaceAlpha', 0.2, 'EdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName','可行域');
    % plot(xs_vertex(1:end-1,1), xs_vertex(1:end-1,2), 'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r', 'DisplayName','顶点');
    % plot(bx, by, 'bs', 'MarkerSize', 9, 'MarkerFaceColor', 'b', 'DisplayName','形心');
    % 
    % % 绘制斜率±1直线
    % x_poly = xs_vertex(1:end-1,1);
    % x_range = [min(x_poly)-10, max(x_poly)+10];
    % y_range1 = 1*(x_range - bx) + by;
    % y_range2 = -1*(x_range - bx) + by;
    % plot(x_range, y_range1, 'k-', 'LineWidth', 1.5, 'DisplayName','斜率 1');
    % plot(x_range, y_range2, 'k--', 'LineWidth', 1.5, 'DisplayName','斜率 -1');
    % 
    % % 绘制交点和最短距离
    % plot(intersections(:,1), intersections(:,2), 'gp', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName','交点');
    % plot([bx, intersections(min_idx,1)], [by, intersections(min_idx,2)], 'r-', 'LineWidth', 2, 'DisplayName',['L_{min} = ', num2str(L_min, '%.4f')]);
    % 
    % xlabel('T_1'); ylabel('T_2');
    % title('张力可行域：斜率±1直线及最短距离分析');
    % legend('Location','best');
    % hold off;
    % 
% else
    % warning('无可行顶点，无法执行新增功能！');
    % L_min = [];
    % intersections = [];
end

end