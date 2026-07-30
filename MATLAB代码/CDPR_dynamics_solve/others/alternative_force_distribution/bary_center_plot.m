function [cable_force,num_vertex] = bary_center_plot(force_ee,moment_ee,jaco,force_min,force_max)
% This programme use Mikelson's barycenter method to deal with the force
% distribution problem. 
% initialization
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

Aeq1=nchoosek([n_s(:,1);n_s(:,1)],2);
Aeq2=nchoosek([n_s(:,2);n_s(:,2)],2);
Beq0=nchoosek([force_min-t_p;force_max-t_p],2);

% solve the C16,2=120 linear equations
for i1=1:120
   Aeq=[(Aeq1(i1,:))' (Aeq2(i1,:))'];
   Beq=(Beq0(i1,:))';
   if (abs(det(Aeq))<1e-6)
      continue;
   end
   x=(Aeq)\Beq;
   if (((n_s*x)<=(force_max-t_p+1e-5))&((n_s*x)>=(force_min-t_p-1e-5)))
   i2=i2+1;
   x_vertex(i2,:)=x;
   end
end

xs_vertex=sort_vertex(x_vertex(1:i2,:));
num_vertex=size(xs_vertex,1);
xs_vertex(i2+1,:)=xs_vertex(1,:);
for i3=1:i2
   A=xs_vertex(i3,1)*xs_vertex(i3+1,2)-xs_vertex(i3+1,1)*xs_vertex(i3,2)+A;
end
A=A/2;
for i4=1:i2
   sum1=(xs_vertex(i4,1)+xs_vertex(i4+1,1))*...
      (xs_vertex(i4,1)*xs_vertex(i4+1,2)-xs_vertex(i4+1,1)*xs_vertex(i4,2))+sum1;
   sum2=(xs_vertex(i4,2)+xs_vertex(i4+1,2))*...
      (xs_vertex(i4,1)*xs_vertex(i4+1,2)-xs_vertex(i4+1,1)*xs_vertex(i4,2))+sum2;
end
b_c=[sum1/6/A;sum2/6/A];

% ===================== 画 16 条完整直线（8组平行线） =====================
if i2 > 0
    figure('Name','8 组平行线 · 16 条完整约束直线','Color','w');
    hold on; grid on; axis equal;
    
    % 延伸到足够大范围，看起来就是无限直线
    x_ext = linspace(-1e6, 1e6, 2);  % 足够大，视觉上无限长

    n_rope = length(n_s(:,1)); % 8 根绳索
    
    for k = 1:n_rope
        a = n_s(k,1);
        b = n_s(k,2);
        fmin_k = force_min - t_p(k);
        fmax_k = force_max - t_p(k);

        % ---------------- 画 拉力下限直线 fmin ----------------
        y_lb = (fmin_k - a * x_ext) / b;
        plot(x_ext, y_lb, 'm--', 'LineWidth', 1.0);
        hold on
        
        % ---------------- 画 拉力上限直线 fmax ----------------
        y_ub = (fmax_k - a * x_ext) / b;
        plot(x_ext, y_ub, 'c--', 'LineWidth', 1.0);
        hold on
    end


    % 绘制凸多边形可行域
    fill(xs_vertex(:,1), xs_vertex(:,2), [0, 0.7, 0], 'FaceAlpha', 0.2, 'EdgeColor', 'k', 'LineWidth', 1.5);
    % 顶点
    plot(xs_vertex(1:end-1,1), xs_vertex(1:end-1,2), 'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');
    % 重心
    plot(b_c(1), b_c(2), 'bs', 'MarkerSize', 9, 'MarkerFaceColor', 'b');
    axis equal

    xlabel('\lambda_1'); ylabel('\lambda_2');
    title('张力可行域：零空间8 组平行线（16 条完整直线）');
    legend('下限直线','上限直线','可行域','顶点','重心','Location','best');
    hold off;
else
    warning('无可行顶点，无法绘图！');
end

cable_force = t_p + n_s * b_c;
end