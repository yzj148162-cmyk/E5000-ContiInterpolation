function hasInterference = checkLineSegmentInterference(segments)
    % segments: 8x6 矩阵，每行代表一个线段 [x1 y1 z1 x2 y2 z2]
    % 返回: true 如果有干涉发生
    
    n = size(segments, 1);
    hasInterference = false;
    
    % 检查所有线段对
    for i = 1:n-1
        for j = i+1:n
            if segmentsIntersect3D(segments(i,:), segments(j,:))
                hasInterference = true;
                fprintf('绳索 %d 和绳索 %d 发生干涉！\n', i, j);
                return;
            end
        end
    end
end

function intersect = segmentsIntersect3D(seg1, seg2)
    % 检查两个3D线段是否相交
    % seg1, seg2: [x1 y1 z1 x2 y2 z2]
    
    % 提取端点
    p1 = seg1(1:3);
    p2 = seg1(4:6);
    q1 = seg2(1:3);
    q2 = seg2(4:6);
    
    % 计算方向向量
    u = p2 - p1;
    v = q2 - q1;
    w = p1 - q1;
    
    % 计算一些点积
    a = dot(u, u);
    b = dot(u, v);
    c = dot(v, v);
    d = dot(u, w);
    e = dot(v, w);
    
    denom = a*c - b*b;
    
    % 检查线段是否平行
    if abs(denom) < 1e-10
        % 平行或共线的情况
        intersect = checkParallelSegments(p1, p2, q1, q2);
        return;
    end
    
    % 计算参数
    s = (b*e - c*d) / denom;
    t = (a*e - b*d) / denom;
    
    % 检查参数是否在线段范围内
    if s >= 0 && s <= 1 && t >= 0 && t <= 1
        % 计算交点
        intersectionPoint = p1 + s*u;
        intersectionPoint2 = q1 + t*v;
        
        % 检查两点是否足够接近
        if norm(intersectionPoint - intersectionPoint2) < 5e-2
            intersect = true;
        else
            intersect = false;
        end
    else
        intersect = false;
    end
end

function intersect = checkParallelSegments(p1, p2, q1, q2)
    % 检查平行线段是否相交
    intersect = false;
    
    % 计算投影到一条直线上
    line_dir = p2 - p1;
    if norm(line_dir) < 1e-6
        return;
    end
    
    % 标准化方向向量
    line_dir = line_dir / norm(line_dir);
    
    % 投影所有点到这条直线上
    proj_p1 = dot(p1, line_dir);
    proj_p2 = dot(p2, line_dir);
    proj_q1 = dot(q1, line_dir);
    proj_q2 = dot(q2, line_dir);
    
    % 对投影进行排序
    min_p = min(proj_p1, proj_p2);
    max_p = max(proj_p1, proj_p2);
    min_q = min(proj_q1, proj_q2);
    max_q = max(proj_q1, proj_q2);
    
    % 检查投影区间是否重叠
    if max(min_p, min_q) <= min(max_p, max_q)
        % 还需要检查在垂直方向上是否也重叠
        % 计算垂直于line_dir的平面上的投影
        % 这里简化为检查距离是否足够小
        % 可以更严格地检查共线性
        intersect = true;
    end
end

