function [cfg, data, csvFile] = load_replay_export(configFile)
configFile = string(configFile);
if ~isfile(configFile)
    error('CDPR:Replay:MissingConfig', '回放配置不存在：%s', configFile);
end
cfg = jsondecode(fileread(configFile));
if ~isfield(cfg, 'schema') || ~strcmp(cfg.schema, 'cdpr_g302_matlab_replay_config_v1')
    error('CDPR:Replay:Schema', '不支持的回放JSON版本。');
end
csvFile = string(fullfile(fileparts(configFile), cfg.csv_file));
if ~isfile(csvFile)
    error('CDPR:Replay:MissingCsv', '找不到JSON对应的运行CSV：%s', csvFile);
end
if numel(cfg.axes) ~= 8 || numel(cfg.geometry.reference_cable_length_mm) ~= 8
    error('CDPR:Replay:AxisCount', '回放参数不是完整八轴数据。');
end

opts = detectImportOptions(csvFile, 'FileType', 'text', ...
    'CommentStyle', '#', 'VariableNamingRule', 'preserve');
data = readtable(csvFile, opts);
required = ["step_index","host_monotonic_us","trace_sequence", ...
    "trace_time_us","trace_valid","desired_pose_si_0", ...
    "axis_safety_relative_trace_position_0"];
missing = required(~ismember(required, string(data.Properties.VariableNames)));
if ~isempty(missing)
    error('CDPR:Replay:CsvColumns', 'CSV缺少字段：%s', strjoin(missing, ', '));
end
end
