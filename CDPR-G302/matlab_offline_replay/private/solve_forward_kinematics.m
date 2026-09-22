function result = solve_forward_kinematics(measuredLength, initialPose, cfg)
lower = [double(cfg.workspace.frame_minimum_mm(:)).' -pi -pi -pi];
upper = [double(cfg.workspace.frame_maximum_mm(:)).'  pi  pi  pi];
if logical(cfg.workspace.orientation_bounds_enabled)
    lower(4:6) = double(cfg.workspace.orientation_minimum_rad(:)).';
    upper(4:6) = double(cfg.workspace.orientation_maximum_rad(:)).';
end
x0 = min(max(double(initialPose(:)).', lower), upper);
residual = @(x) double(measuredLength(:)).' - cdpr_cable_lengths(x, cfg);

if exist('lsqnonlin', 'file') == 2
    opt = optimoptions('lsqnonlin', 'Display', 'off', ...
        'FunctionTolerance', 1e-12, 'StepTolerance', 1e-10, ...
        'MaxIterations', 80, 'MaxFunctionEvaluations', 1000);
    [pose,~,r,exitFlag,output] = lsqnonlin(residual, x0, lower, upper, opt);
else
    penalty = @(x) sum(residual(min(max(x,lower),upper)).^2) + ...
        1e6*sum((x-min(max(x,lower),upper)).^2);
    opt = optimset('Display','off','MaxIter',400,'MaxFunEvals',3000, ...
        'TolX',1e-9,'TolFun',1e-9);
    [pose,~,exitFlag,output] = fminsearch(penalty, x0, opt);
    pose = min(max(pose,lower),upper);
    r = residual(pose);
end
r = r(:);
result = struct('pose',pose(:).', 'success',exitFlag>0 && all(isfinite(pose)), ...
    'rmsResidualMm',sqrt(mean(r.^2)), 'maxResidualMm',max(abs(r)), ...
    'exitFlag',exitFlag, 'iterations',output.iterations);
end
