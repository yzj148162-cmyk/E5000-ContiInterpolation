function mu = ropeRelativePosition(seg_i, seg_j)
% 严格对应论文式 (4-25)

    % 锚点
    Bi = seg_i(1:3);
    Ai = seg_i(4:6);
    Bj = seg_j(1:3);
    Aj = seg_j(4:6);

    % 向量定义（不是中点！）
    li  = Ai - Bi;      % 第 i 根绳方向
    eta = Aj - Bi;      % B_i -> A_j
    kappa = Bj - Bi;    % B_i -> B_j

    % 三重积
    val = dot(cross(li, eta), kappa);

    tol = 1e-9;
    if abs(val) < tol
        mu = 0;
    else
        mu = sign(val);
    end
end
