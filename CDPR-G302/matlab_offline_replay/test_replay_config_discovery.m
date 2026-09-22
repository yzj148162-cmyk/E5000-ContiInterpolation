function selected = test_replay_config_discovery
moduleDirectory = fileparts(mfilename('fullpath'));
selected = find_latest_replay_config(moduleDirectory);
assert(isfile(selected), '未能从工程build目录发现已导出的回放JSON。');
assert(endsWith(selected,'_replay_config.json'), ...
    '发现结果不是回放配置JSON。');
fprintf('Latest replay config: %s\n', selected);
end
