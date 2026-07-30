function [ideal_cf, ideal_cf_exitflag] = pseudo_inverse_new( ...
    force_ee, moment_ee, jaco, force_min, force_max)

ideal_cf_exitflag = 1;

% 外力 wrench
w = [force_ee; moment_ee];

% ---------- Step 1: 伪逆解 ----------
T_pinv = -jaco * ((jaco' * jaco) \ w);

% ---------- Step 2: 检查是否满足约束 ----------
if all(T_pinv >= force_min) && all(T_pinv <= force_max)
    ideal_cf = T_pinv;
    return;
end

% ---------- Step 3: 若不满足，用QP重新求解 ----------
m = size(jaco,1);   % 一般是 8

% QP: min (1/2) T' H T
H = eye(m);        % 最小范数
f = zeros(m,1);

% 等式约束：J' T + w = 0  →  J' T = -w
Aeq = jaco';
beq = -w;

% 上下界
lb = force_min * ones(m,1);
ub = force_max * ones(m,1);

% 求解
options = optimoptions('quadprog','Display','off');

[T_sol,~,exitflag] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],options);

if exitflag ~= 1
    if ~isempty(T_sol) && norm(Aeq*T_sol - beq) < 1e-3
        ideal_cf = T_sol;
    else
        ideal_cf_exitflag = -1;
        ideal_cf = [];
    end
else
    ideal_cf = T_sol;
end

end
