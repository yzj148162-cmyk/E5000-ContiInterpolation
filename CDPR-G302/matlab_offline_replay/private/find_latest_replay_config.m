function initialSelection = find_latest_replay_config(moduleDirectory)
% Locate the newest Qt replay sidecar without coupling Qt to MATLAB.
moduleDirectory = string(moduleDirectory);
projectRoot = string(fileparts(moduleDirectory));
patterns = [ ...
    fullfile(projectRoot,'build','**','data','outputmsg', ...
             'force_interaction_runs','*_replay_config.json'); ...
    fullfile(projectRoot,'data','outputmsg','force_interaction_runs', ...
             '*_replay_config.json')];

files = [];
for pattern = patterns.'
    found = dir(pattern);
    if ~isempty(found)
        files = [files; found]; %#ok<AGROW>
    end
end
if isempty(files)
    % No completed run yet. Start from build instead of presenting an empty
    % source-code directory whenever the build directory is available.
    buildDirectory = fullfile(projectRoot,'build');
    if isfolder(buildDirectory)
        initialSelection = buildDirectory;
    else
        initialSelection = moduleDirectory;
    end
    return;
end

[~, newest] = max([files.datenum]);
initialSelection = string(fullfile(files(newest).folder, files(newest).name));
end
