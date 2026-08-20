function analysis = analyze_ready_comparison(inputFiles, outputDir, varargin)
%ANALYZE_READY_COMPARISON Compare ready target error with/without gravity.
parser = inputParser;
addRequired(parser, 'inputFiles');
addRequired(parser, 'outputDir');
addParameter(parser, 'ExperimentNames', strings(0, 1));
addParameter(parser, 'TargetRad', []);
addParameter(parser, 'PostureNames', strings(0, 1));
addParameter(parser, 'GravityEnabled', []);
addParameter(parser, 'Visible', 'off');
parse(parser, inputFiles, outputDir, varargin{:});

files = cellstr(string(inputFiles(:)));
count = numel(files);
names = string(parser.Results.ExperimentNames(:));
if isempty(names)
    names = strings(count, 1);
    for index = 1:count
        [~, stem] = fileparts(files{index});
        names(index) = string(stem);
    end
end
if numel(names) ~= count
    error('ready:NameCount', 'ExperimentNames must match input files.');
end
targets = parser.Results.TargetRad;
postures = string(parser.Results.PostureNames(:));
gravity = parser.Results.GravityEnabled;
if ~isempty(targets) && size(targets, 1) ~= count
    error('ready:TargetCount', 'TargetRad rows must match input files.');
end
if ~isempty(postures) && numel(postures) ~= count
    error('ready:PostureCount', 'PostureNames must match input files.');
end
if ~isempty(gravity) && numel(gravity) ~= count
    error('ready:GravityCount', 'GravityEnabled must match input files.');
end

runs = cell(count, 1);
for index = 1:count
    target = [];
    posture = "";
    enabled = nan;
    if ~isempty(targets), target = targets(index, :); end
    if ~isempty(postures), posture = postures(index); end
    if ~isempty(gravity), enabled = gravity(index); end
    runs{index} = read_ready_json(files{index}, names(index), ...
        'TargetRad', target, 'PostureName', posture, ...
        'GravityEnabled', enabled);
end
runs = vertcat(runs{:});
if ~isfolder(outputDir), mkdir(outputDir); end

summary = repmat(struct(), count, 1);
jointRows = struct('experiment', {}, 'posture_name', {}, ...
    'gravity_enabled', {}, 'joint', {}, 'target_rad', {}, ...
    'reference_rad', {}, 'feedback_rad', {}, 'error_rad', {});
row = 0;
for index = 1:count
    run = runs(index);
    summary(index).experiment = run.experiment;
    summary(index).source_file = run.source_file;
    summary(index).posture_name = run.posture_name;
    summary(index).gravity_enabled = run.gravity_enabled;
    summary(index).gravity_scale = run.gravity_scale;
    summary(index).termination = run.termination;
    summary(index).passed = run.passed;
    summary(index).max_abs_error_rad = run.max_abs_error_rad;
    summary(index).rms_error_rad = run.rms_error_rad;
    summary(index).settling_sec = run.settling_sec;
    summary(index).target_rad = run.target_rad;
    summary(index).reference_rad = run.reference_rad;
    summary(index).feedback_rad = run.feedback_rad;
    summary(index).error_rad = run.error_rad;
    summary(index).gravity_torque_nm = run.gravity_torque_nm;
    for joint = 1:numel(run.joint_names)
        row = row + 1;
        jointRows(row).experiment = run.experiment;
        jointRows(row).posture_name = run.posture_name;
        jointRows(row).gravity_enabled = run.gravity_enabled;
        jointRows(row).joint = run.joint_names(joint);
        jointRows(row).target_rad = run.target_rad(joint);
        jointRows(row).reference_rad = run.reference_rad(joint);
        jointRows(row).feedback_rad = run.feedback_rad(joint);
        jointRows(row).error_rad = run.error_rad(joint);
    end
end
summaryTable = struct2table(summary);
summaryTable.target_rad = [];
summaryTable.reference_rad = [];
summaryTable.feedback_rad = [];
summaryTable.error_rad = [];
summaryTable.gravity_torque_nm = [];
writetable(summaryTable, fullfile(outputDir, 'ready_summary.csv'));
writetable(struct2table(jointRows), fullfile(outputDir, 'ready_joint_errors.csv'));

keys = strings(count, 1);
for index = 1:count
    keys(index) = runs(index).posture_name + "|gravity=" + string(runs(index).gravity_enabled);
end
uniqueKeys = unique(keys, 'stable');
groups = repmat(struct('key', "", 'posture_name', "", ...
    'gravity_enabled', false, 'experiments', 0, ...
    'mean_max_abs_error_rad', nan, 'worst_error_rad', nan), numel(uniqueKeys), 1);
for index = 1:numel(uniqueKeys)
    mask = keys == uniqueKeys(index);
    values = [runs(mask).max_abs_error_rad];
    first = find(mask, 1);
    groups(index).key = uniqueKeys(index);
    groups(index).posture_name = runs(first).posture_name;
    groups(index).gravity_enabled = runs(first).gravity_enabled;
    groups(index).experiments = sum(mask);
    groups(index).mean_max_abs_error_rad = mean(values, 'omitnan');
    groups(index).worst_error_rad = max(values, [], 'omitnan');
end
writetable(struct2table(groups), fullfile(outputDir, 'ready_group_comparison.csv'));

generated = string(datetime('now', 'TimeZone', 'UTC', ...
    'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX'));
json = struct('generated_utc', generated, ...
    'generator', "analyze_ready_comparison", ...
    'input_files_are_unmodified', true, ...
    'experiments', summary, 'groups', groups);
fid = fopen(fullfile(outputDir, 'ready_analysis_summary.json'), 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(json, 'PrettyPrint', true));
figureHandle = figure('Visible', parser.Results.Visible, 'Color', 'w');
bar(categorical(names), [runs.max_abs_error_rad]);
ylabel('Worst joint error (rad)');
title('Ready target arrival error');
grid on;
exportgraphics(figureHandle, fullfile(outputDir, 'ready_target_error.png'), ...
    'Resolution', 180);
close(figureHandle);

analysis = struct('generated_utc', generated, 'experiments', runs, ...
    'summary', summary, 'groups', groups, ...
    'output_directory', string(java.io.File(outputDir).getCanonicalPath()));
% Rewrite MAT now that the returned analysis structure exists.
save(fullfile(outputDir, 'ready_analysis.mat'), 'analysis', 'runs', ...
    'summaryTable', 'groups', '-v7');
end
