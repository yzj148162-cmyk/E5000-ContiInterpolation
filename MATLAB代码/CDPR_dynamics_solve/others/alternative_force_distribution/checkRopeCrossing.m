function hasCrossing = checkRopeCrossing(prevSegs, currSegs)
% 检查相邻两个时刻是否发生绳索穿越

    n = size(prevSegs,1);
    hasCrossing = false;

    for i = 1:n-1
        for j = i+1:n
            mu_prev = ropeRelativePosition(prevSegs(i,:), prevSegs(j,:));
            mu_curr = ropeRelativePosition(currSegs(i,:), currSegs(j,:));

            % 公式 (4-26)
            if mu_prev * mu_curr < 0
                fprintf('绳索 %d 与 %d 发生穿越干涉！\n', i, j);
                hasCrossing = true;
                return;
            end
        end
    end
end
