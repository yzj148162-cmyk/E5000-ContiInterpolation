function [hasInterference, report] = checkCableplatInterference( segments_curr)
    % checkCableInterferenceFinal 
    % 包含防误判机制的完整干涉检测（静态+动态）
    
    %% --- 参数配置 ---
    % 1. 数据格式定义
    isFixedPointFirst = true; % true: [定点A, 动点B]; false: [动点B, 定点A]
    
    % 2. 危险距离阈值 
    % 只有当两根绳索距离小于此值，且发生符号翻转时，才视为穿越。
    % proximity_threshold = 0.1; 
    
    hasInterference = false;
    report = [];
    n = size(segments_curr, 1);
    
    for i = 1:n-1
        for j = i+1:n
            %% 1. 计算当前时刻两线段的最短距离
            dist_curr = getMinDistBetweenSegments(segments_curr(i,:), segments_curr(j,:));
            
            % 如果当前已经撞上了 (距离近似0)
            if dist_curr < 8e-2 % 假设绳索视为无粗细，或者小于绳径
                hasInterference = true;
                report = [report; sprintf('静态碰撞: 绳索 与圆柱平台 实质接触')];
                continue; % 已经撞了，不用测穿越了
            end
            

        end
    end
    
    if hasInterference
        disp('检测到干涉:');
        disp(report);
    end
end



function d = getMinDistBetweenSegments(seg1, seg2)
    % 计算两个空间线段之间的最短欧氏距离
    % 这是过滤误判的核心函数
    
    P1 = seg1(1:3)'; Q1 = seg1(4:6)';
    P2 = seg2(1:3)'; Q2 = seg2(4:6)';
    
    u = Q1 - P1;
    v = Q2 - P2;
    w = P1 - P2;
    
    a = dot(u,u); b = dot(u,v); c = dot(v,v);
    d_dot = dot(u,w); e = dot(v,w);
    D = a*c - b*b;
    
    sc = 0; sN = 0; sD = D;
    tc = 0; tN = 0; tD = D;
    
    if D < 1e-8 % 平行
        sN = 0; sD = 1; tN = e; tD = c;
    else
        sN = (b*e - c*d_dot);
        tN = (a*e - b*d_dot);
        if sN < 0
            sN = 0; tN = e; tD = c;
        elseif sN > sD
            sN = sD; tN = e + b; tD = c;
        end
    end
    
    if tN < 0
        tN = 0;
        if -d_dot < 0
            sN = 0;
        elseif -d_dot > a
            sN = sD;
        else
            sN = -d_dot; sD = a;
        end
    elseif tN > tD
        tN = tD;
        if (-d_dot + b) < 0
            sN = 0;
        elseif (-d_dot + b) > a
            sN = sD;
        else
            sN = (-d_dot + b); sD = a;
        end
    end
    
    if abs(sN) < 1e-8, sc = 0; else, sc = sN / sD; end
    if abs(tN) < 1e-8, tc = 0; else, tc = tN / tD; end
    
    dP = w + (sc * u) - (tc * v);
    d = norm(dP);
end