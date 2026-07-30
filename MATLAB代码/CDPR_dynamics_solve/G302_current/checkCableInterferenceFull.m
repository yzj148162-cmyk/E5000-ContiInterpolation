function [hasInterference, report] = checkCableInterferenceFull(segments_prev, segments_curr)
    % checkCableInterferenceFinal 
    % 包含防误判机制的完整干涉检测（静态+动态）
    
    %% --- 参数配置 ---
    % 1. 数据格式定义
    isFixedPointFirst = true; % true: [定点A, 动点B]; false: [动点B, 定点A]
    
    % 2. 危险距离阈值 
    % 只有当两根绳索距离小于此值，且发生符号翻转时，才视为穿越。
    proximity_threshold = 0.01; 
    
    hasInterference = false;
    report = [];
    n = size(segments_curr, 1);
    
    for i = 1:n-1
        for j = i+1:n
            %% 1. 计算当前时刻两线段的最短距离
            dist_curr = getMinDistBetweenSegments(segments_curr(i,:), segments_curr(j,:));
            
            % 如果当前已经撞上了 (距离近似0)
            if dist_curr < 0.6e-2 % 假设绳索视为无粗细，或者小于绳径
                plot3([segments_curr(i,1),segments_curr(i,4)],[segments_curr(i,2),segments_curr(i,5)],[segments_curr(i,3),segments_curr(i,6)] , 'k','LineWidth',1.5);hold on
                plot3([segments_curr(j,1),segments_curr(j,4)],[segments_curr(j,2),segments_curr(j,5)],[segments_curr(j,3),segments_curr(j,6)] , 'k','LineWidth',1.5);hold on
                hasInterference = true;
                disp('静态碰撞: 绳索实质接触');
                % report = [report; sprintf('静态碰撞: 绳索 %d-%d 实质接触', i, j)];
                continue; % 已经撞了，不用测穿越了
            end
            
            %% 2. 动态穿越检测 (带距离门控)
            % 只有当距离处于“危险范围”内时，才进行复杂的穿越计算
            % 这样可以过滤掉 99% 的远距离误判
            
            % 为了保险，我们取当前距离和上一时刻距离的最小值
            dist_prev = getMinDistBetweenSegments(segments_prev(i,:), segments_prev(j,:));
            min_dist_interval = min(dist_curr, dist_prev);
            
            if min_dist_interval < proximity_threshold
                % 进入危险区，启用符号翻转检查
                
                % 检查 i 越过 j
                if checkCrossingLogic(i, j, segments_prev, segments_curr, isFixedPointFirst)
                    hasInterference = true;
                    report = [report; sprintf('穿越干涉: 绳索 %d/%d 在近距离发生位置跳变', i, j)];
                end
                
                % 检查 j 越过 i (反向)
                if checkCrossingLogic(j, i, segments_prev, segments_curr, isFixedPointFirst)
                    hasInterference = true;
                    report = [report; sprintf('穿越干涉: 绳索 %d/%d 在近距离发生位置跳变', j, i)];
                end
            end
        end
    end
    
    if hasInterference
        disp('检测到干涉:');
        disp(report);
    end
end

function isCrossed = checkCrossingLogic(idx_i, idx_j, segs_prev, segs_curr, isFixedFirst)
    % 纯粹的数学符号检查 (公式 4-25/4-26)
    [Ai_old, Bi_old] = getAB(segs_prev(idx_i, :), isFixedFirst);
    [Aj_old, Bj_old] = getAB(segs_prev(idx_j, :), isFixedFirst);
    
    [Ai_new, Bi_new] = getAB(segs_curr(idx_i, :), isFixedFirst);
    [Aj_new, Bj_new] = getAB(segs_curr(idx_j, :), isFixedFirst);
    
    mu_prev = calculateMu(Ai_old, Bi_old, Aj_old, Bj_old);
    mu_curr = calculateMu(Ai_new, Bi_new, Aj_new, Bj_new);
    
    % 符号翻转判定
    if (mu_prev * mu_curr < -1e-6)
        isCrossed = true;
    else
        isCrossed = false;
    end
end

function mu = calculateMu(Ai, Bi, Aj, Bj)
    % 几何定义：以Bi为基准点的四面体体积符号
    vec_li = Ai - Bi; 
    vec_eta = Aj - Bi;
    vec_kappa = Bj - Bi;
    
    vol = dot(cross(vec_li, vec_eta), vec_kappa);
    mu = sign(vol);
    if abs(vol) < 1e-9, mu = 0; end
end

function [A, B] = getAB(row, isFixedFirst)
    if isFixedFirst
        A = row(1:3)'; B = row(4:6)';
    else
        A = row(4:6)'; B = row(1:3)';
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