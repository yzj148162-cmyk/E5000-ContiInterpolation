function [sample, rejected] = extract_replay_samples(data, cfg, includeBraking)
n = height(data);
relative = nan(n, 8);
desiredSi = nan(n, 6);
platformForceN = nan(n, 3);
for axis = 1:8
    relative(:,axis) = data.(sprintf( ...
        'axis_safety_relative_trace_position_%d', axis-1));
end
for dim = 1:6
    desiredSi(:,dim) = data.(sprintf('desired_pose_si_%d', dim-1));
end
% 使用经过坐标变换、滤波/门限处理后真正送入动力学模型的全局系平台力。
% 旧日志若没有该组字段则保留NaN，不影响正运动学回放。
for dim = 1:3
    name = sprintf('platform_wrench_%d', dim-1);
    if ismember(name, data.Properties.VariableNames)
        platformForceN(:,dim) = double(data.(name));
    end
end

interaction = numeric_column(data, 'interaction_segment', ones(n,1));
stopCause = numeric_column(data, 'controlled_stop_cause', zeros(n,1));
mask = data.trace_valid == 1 & all(isfinite(relative),2) & ...
       all(isfinite(desiredSi),2) & isfinite(data.trace_time_us) & ...
       isfinite(data.trace_sequence);
if ~includeBraking
    % Qt记录约定：0=正常力交互段，1=协同制动段。
    mask = mask & (interaction == 0);
end
index = find(mask);
if ~isempty(index)
    seq = data.trace_sequence(index);
    monotonic = [true; diff(seq) > 0];
    index = index(monotonic);
end
rejected = n - numel(index);

hostUs = double(data.host_monotonic_us(index));
traceUs = double(data.trace_time_us(index));
anchorOffsetUs = min(hostUs - traceUs);
alignedHostUs = traceUs + anchorOffsetUs;
desiredAlignedSi = nan(numel(index), 6);
allHostUs = double(data.host_monotonic_us);
for dim = 1:6
    values = desiredSi(:,dim);
    good = isfinite(allHostUs) & isfinite(values);
    [x, uniqueIndex] = unique(allHostUs(good), 'stable');
    y = values(good,:);
    y = y(uniqueIndex);
    if numel(x) == 1
        desiredAlignedSi(:,dim) = repmat(y(1), numel(alignedHostUs), 1);
    else
        desiredAlignedSi(:,dim) = interp1(x, y, alignedHostUs, ...
            'linear', 'extrap');
    end
end
desiredAlignedSi(:,1:3) = desiredAlignedSi(:,1:3) * 1000.0;

sample = struct;
sample.step = double(data.step_index(index));
sample.traceSequence = double(data.trace_sequence(index));
sample.traceTimeUs = traceUs;
sample.hostTimeUs = hostUs;
sample.traceHostAnchorOffsetUs = anchorOffsetUs;
sample.timeS = (alignedHostUs - alignedHostUs(1)) * 1e-6;
sample.axisRelative = relative(index,:).';
sample.desiredPose = desiredAlignedSi;
sample.platformForceN = platformForceN(index,:);
sample.interactionSegment = interaction(index);
sample.controlledStopCause = stopCause(index);
sample.translationOnly = logical(cfg.translation_only);
end

function value = numeric_column(tableData, name, fallback)
if ismember(name, tableData.Properties.VariableNames)
    value = double(tableData.(name));
else
    value = fallback;
end
end
