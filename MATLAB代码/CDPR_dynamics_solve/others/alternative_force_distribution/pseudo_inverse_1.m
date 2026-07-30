function [ideal_cf, ideal_cf_exitflag] = pseudo_inverse_1(force_ee, moment_ee, jaco, force_min, force_max)
    % 确保输入向量为列向量
    force_ee = force_ee(:);
    moment_ee = moment_ee(:);
    w = [force_ee; moment_ee]; % 组装广义力向量 (6x1)
    n_cables = 8; % 绳索数量

    % --------------------------
    % 1. 计算伪逆解（提高数值稳定性）
    % --------------------------
    f_pinv = -pinv(jaco') * w; % 用 pinv 替代 inv 避免奇异问题

    % --------------------------
    % 2. 设置绳索力上下界
    % --------------------------
    % 支持 force_min/force_max 为标量或 8 维向量
    if isscalar(force_min)
        lb = force_min * ones(n_cables, 1);
    else
        lb = force_min(:);
    end
    if isscalar(force_max)
        ub = force_max * ones(n_cables, 1);
    else
        ub = force_max(:);
    end

    % --------------------------
    % 3. 检查伪逆解是否直接可行
    % --------------------------
    % 加 1e-6 容差避免数值误差导致误判
    is_feasible_pinv = all(f_pinv >= lb - 1e-6) && all(f_pinv <= ub + 1e-6);
    if is_feasible_pinv
        ideal_cf = f_pinv;
        ideal_cf_exitflag = 1;
        return;
    end

    % --------------------------
    % 4. 伪逆解不可行，用二次规划找可行解
    % --------------------------
    % 优化目标：最小化 ||f - f_pinv||^2（最接近伪逆解的可行解）
    % 转化为 quadprog 标准形式：min 0.5*f'*H*f + c'*f
    H = eye(n_cables); % H 为单位矩阵
    c = -f_pinv;       % 线性项系数

    % 等式约束：jaco' * f = -w（与原力平衡逻辑一致）
    Aeq = jaco';
    beq = -w;

    % 设置优化选项（关闭显示，选择凸问题算法）
    options = optimoptions('quadprog', ...
        'Display', 'off', ...
        'Algorithm', 'interior-point-convex');

    % 调用二次规划求解
    try
        [f_opt, ~, exitflag_qp] = quadprog(H, c, [], [], Aeq, beq, lb, ub, [], options);
    catch ME
        % 捕获错误（如未安装 Optimization Toolbox）
        warning('优化失败: %s。返回伪逆解。', ME.message);
        ideal_cf = f_pinv;
        ideal_cf_exitflag = -1;
        return;
    end

    % --------------------------
    % 5. 验证优化结果并返回
    % --------------------------
    if exitflag_qp > 0
        % 二次规划成功，再次验证可行性（避免数值误差）
        is_feasible_opt = all(f_opt >= lb - 1e-6) && all(f_opt <= ub + 1e-6);
        if is_feasible_opt
            ideal_cf = f_opt;
            ideal_cf_exitflag = 1;
        else
            ideal_cf = f_pinv;
            ideal_cf_exitflag = -1;
        end
    else
        % 二次规划未找到可行解
        ideal_cf = f_pinv;
        ideal_cf_exitflag = -1;
    end
end