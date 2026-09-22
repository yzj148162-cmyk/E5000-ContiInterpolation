function result = replay_force_interaction_run(configFile, options)
%REPLAY_FORCE_INTERACTION_RUN Reconstruct the virtual CDPR motion from Qt logs.
% Qt and MATLAB remain completely independent: this function only reads the
% exported CSV/JSON pair and writes offline analysis files.

arguments
    configFile (1,1) string = ""
    options.Animate (1,1) logical = true
    options.IncludeBraking (1,1) logical = false
    options.WriteVideo (1,1) logical = true
    options.FrameStride (1,1) double {mustBePositive,mustBeInteger} = 1
    options.MaxFrames (1,1) double {mustBePositive} = inf
end

if strlength(configFile) == 0
    initialSelection = find_latest_replay_config(fileparts(mfilename('fullpath')));
    [name, folder] = uigetfile('*_replay_config.json', ...
        '选择 Qt 导出的六维力交互回放配置', initialSelection);
    if isequal(name, 0)
        error('CDPR:Replay:Cancelled', '未选择回放配置。');
    end
    configFile = string(fullfile(folder, name));
end

[cfg, raw, csvFile] = load_replay_export(configFile);
[sample, rejected] = extract_replay_samples(raw, cfg, options.IncludeBraking);
if isempty(sample.timeS)
    error('CDPR:Replay:NoSamples', 'CSV中没有满足条件的八轴同帧Trace数据。');
end
if isfinite(options.MaxFrames) && numel(sample.timeS) > options.MaxFrames
    keep = 1:floor(options.MaxFrames);
    sample = subset_sample(sample, keep);
end

axisCfg = cfg.axes;
n = numel(sample.timeS);
actualPose = nan(n, 6);
cableLength = nan(n, 8);
rmsResidual = nan(n, 1);
maxResidual = nan(n, 1);
solverExit = zeros(n, 1);
solverIterations = zeros(n, 1);
poseGuess = double(cfg.geometry.initial_pose_mm_rad(:)).';

fprintf('CDPR离线回放：逐帧正运动学 %d 帧……\n', n);
for k = 1:n
    for axis = 1:8
        motorTheta = sample.axisRelative(axis, k) / ...
            double(axisCfg(axis).motor_unit_per_radian);
        platformDelta = winch_platform_delta_from_motor_theta( ...
            axisCfg(axis), motorTheta);
        cableLength(k, axis) = ...
            double(cfg.geometry.reference_cable_length_mm(axis)) - platformDelta;
    end
    solved = solve_forward_kinematics(cableLength(k, :), poseGuess, cfg);
    if solved.success
        actualPose(k, :) = solved.pose;
        rmsResidual(k) = solved.rmsResidualMm;
        maxResidual(k) = solved.maxResidualMm;
        solverExit(k) = solved.exitFlag;
        solverIterations(k) = solved.iterations;
        poseGuess = solved.pose;
    end
    if mod(k, max(1, floor(n / 20))) == 0 || k == n
        fprintf('  %d/%d\n', k, n);
    end
end

valid = all(isfinite(actualPose), 2);
if ~any(valid)
    error('CDPR:Replay:ForwardKinematics', ...
        '全部%d帧正运动学均未收敛；请检查JSON几何参数和轴方向。', n);
end
translationError = vecnorm(actualPose(:,1:3) - sample.desiredPose(:,1:3), 2, 2);
angleDelta = atan2(sin(actualPose(:,4:6) - sample.desiredPose(:,4:6)), ...
                   cos(actualPose(:,4:6) - sample.desiredPose(:,4:6)));
orientationErrorDeg = rad2deg(vecnorm(angleDelta, 2, 2));

[folder, base] = fileparts(csvFile);
outputFolder = fullfile(folder, base + "_matlab_replay");
if ~isfolder(outputFolder), mkdir(outputFolder); end
outputCsv = fullfile(outputFolder, "pose_history.csv");
outputMat = fullfile(outputFolder, "replay_result.mat");
out = table(sample.step, sample.traceSequence, sample.traceTimeUs, sample.timeS, ...
    sample.interactionSegment, sample.controlledStopCause, ...
    'VariableNames', {'step_index','trace_sequence','trace_time_us', ...
    'aligned_time_s','interaction_segment','controlled_stop_cause'});
for i = 1:6
    out.(sprintf('desired_pose_mm_rad_%d', i-1)) = sample.desiredPose(:,i);
    out.(sprintf('actual_pose_mm_rad_%d', i-1)) = actualPose(:,i);
end
for i = 1:3
    out.(sprintf('platform_force_n_%d', i-1)) = sample.platformForceN(:,i);
end
for i = 1:8
    out.(sprintf('measured_cable_length_mm_%d', i-1)) = cableLength(:,i);
end
out.translation_error_mm = translationError;
out.orientation_error_deg = orientationErrorDeg;
out.rms_cable_residual_mm = rmsResidual;
out.maximum_cable_residual_mm = maxResidual;
out.solver_success = valid;
out.solver_exit_flag = solverExit;
out.solver_iterations = solverIterations;
writetable(out, outputCsv);

metrics = struct;
metrics.totalRows = height(raw);
metrics.acceptedRows = n;
metrics.rejectedRows = rejected;
metrics.solvedRows = nnz(valid);
metrics.failedRows = nnz(~valid);
metrics.translationRmsMm = sqrt(mean(translationError(valid).^2, 'omitnan'));
metrics.translationMaximumMm = max(translationError(valid), [], 'omitnan');
metrics.orientationRmsDeg = sqrt(mean(orientationErrorDeg(valid).^2, 'omitnan'));
metrics.orientationMaximumDeg = max(orientationErrorDeg(valid), [], 'omitnan');
metrics.cableResidualRmsMm = sqrt(mean(rmsResidual(valid).^2, 'omitnan'));
metrics.cableResidualMaximumMm = max(maxResidual(valid), [], 'omitnan');

result = struct('configFile', configFile, 'csvFile', csvFile, ...
    'outputFolder', outputFolder, 'outputCsv', outputCsv, ...
    'outputMat', outputMat, 'config', cfg, ...
    'sample', sample, 'actualPoseMmRad', actualPose, ...
    'measuredCableLengthMm', cableLength, 'metrics', metrics);
save(outputMat, 'result', '-v7.3');
summaryFile = fullfile(outputFolder, "summary.json");
fid = fopen(summaryFile, 'w');
if fid >= 0
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, jsonencode(metrics, 'PrettyPrint', true), 'char');
    clear cleaner;
end
plot_replay_summary(result);

fprintf(['完成：有效/失败=%d/%d，平移RMS/最大=%.4f/%.4f mm，' ...
    '姿态RMS/最大=%.5f/%.5f deg，绳长残差RMS/最大=%.6f/%.6f mm。\n'], ...
    metrics.solvedRows, metrics.failedRows, ...
    metrics.translationRmsMm, metrics.translationMaximumMm, ...
    metrics.orientationRmsDeg, metrics.orientationMaximumDeg, ...
    metrics.cableResidualRmsMm, metrics.cableResidualMaximumMm);
fprintf('结果：%s\n', outputCsv);

if options.Animate
    animate_cdpr_replay(result, options);
end
end

function sample = subset_sample(sample, keep)
originalCount = numel(sample.timeS);
fields = fieldnames(sample);
for i = 1:numel(fields)
    value = sample.(fields{i});
    if ismatrix(value) && size(value,1) == originalCount
        sample.(fields{i}) = value(keep, :);
    elseif ismatrix(value) && size(value,2) == originalCount
        sample.(fields{i}) = value(:, keep);
    end
end
end
