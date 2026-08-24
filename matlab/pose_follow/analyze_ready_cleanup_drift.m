function analysis = analyze_ready_cleanup_drift(afterReadyFile, postCleanupFile, outputDir, varargin)
%ANALYZE_READY_CLEANUP_DRIFT Separate pre-cleanup ready error from later sag.
parser = inputParser;
addRequired(parser, 'afterReadyFile');
addRequired(parser, 'postCleanupFile');
addRequired(parser, 'outputDir');
addParameter(parser, 'ExperimentName', "ready-cleanup-drift");
addParameter(parser, 'Visible', 'off');
parse(parser, afterReadyFile, postCleanupFile, outputDir, varargin{:});

name = string(parser.Results.ExperimentName);
pre = read_ready_json(afterReadyFile, name + "-pre-cleanup");
post = read_ready_json(postCleanupFile, name + "-post-cleanup", ...
    'TargetRad', pre.target_rad, 'PostureName', pre.posture_name, ...
    'GravityEnabled', false);
if ~isequal(pre.joint_names, post.joint_names)
    error('ready:JointNames', 'Pre/post cleanup joint names differ.');
end

drift = post.feedback_rad - pre.feedback_rad;
postError = post.feedback_rad - pre.target_rad;
[preWorst, preIndex] = max(abs(pre.error_rad));
[driftWorst, driftIndex] = max(abs(drift));
[postWorst, postIndex] = max(abs(postError));
jointTable = table(pre.joint_names, pre.target_rad(:), ...
    pre.feedback_rad(:), pre.error_rad(:), post.feedback_rad(:), ...
    drift(:), postError(:), ...
    'VariableNames', {'joint', 'target_rad', 'pre_cleanup_feedback_rad', ...
    'pre_cleanup_target_error_rad', 'post_cleanup_feedback_rad', ...
    'post_cleanup_drift_rad', 'post_cleanup_target_error_rad'});

summary = struct();
summary.experiment = name;
summary.pre_cleanup_source = pre.source_file;
summary.post_cleanup_source = post.source_file;
summary.posture_name = pre.posture_name;
summary.cleanup_recorded_attempted = false;
summary.cleanup_recorded_zero_published = false;
raw = jsondecode(fileread(char(string(afterReadyFile))));
if isfield(raw, 'ready_result') && isstruct(raw.ready_result) && ...
        isfield(raw.ready_result, 'gravity_cleanup')
    cleanup = raw.ready_result.gravity_cleanup;
    if isfield(cleanup, 'attempted')
        summary.cleanup_recorded_attempted = logical(cleanup.attempted);
    end
    if isfield(cleanup, 'zero_published')
        summary.cleanup_recorded_zero_published = logical(cleanup.zero_published);
    end
end
summary.pre_cleanup_worst_target_error_rad = preWorst;
summary.pre_cleanup_worst_joint = pre.joint_names(preIndex);
summary.post_cleanup_worst_drift_rad = driftWorst;
summary.post_cleanup_worst_drift_joint = pre.joint_names(driftIndex);
summary.post_cleanup_worst_target_error_rad = postWorst;
summary.post_cleanup_worst_target_error_joint = pre.joint_names(postIndex);
summary.pre_cleanup_feedback_rad = pre.feedback_rad;
summary.pre_cleanup_target_error_rad = pre.error_rad;
summary.post_cleanup_feedback_rad = post.feedback_rad;
summary.post_cleanup_drift_rad = drift;
summary.post_cleanup_target_error_rad = postError;
summary.classification = "ready performance uses pre-cleanup feedback; " + ...
    "post-cleanup change is gravity-zero drift";

if ~isfolder(outputDir), mkdir(outputDir); end
writetable(jointTable, fullfile(outputDir, 'ready_cleanup_joint_drift.csv'));
summaryTable = struct2table(rmfield(summary, { ...
    'pre_cleanup_feedback_rad', 'pre_cleanup_target_error_rad', ...
    'post_cleanup_feedback_rad', 'post_cleanup_drift_rad', ...
    'post_cleanup_target_error_rad'}));
writetable(summaryTable, fullfile(outputDir, 'ready_cleanup_summary.csv'));
jsonPath = fullfile(outputDir, 'ready_cleanup_analysis.json');
fid = fopen(jsonPath, 'w');
cleanupFile = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(summary, 'PrettyPrint', true));

figureHandle = figure('Visible', parser.Results.Visible, 'Color', 'w');
bar(categorical(pre.joint_names), [abs(pre.error_rad(:)), abs(drift(:)), ...
    abs(postError(:))]);
ylabel('Absolute angle (rad)');
title('Ready error before cleanup vs post-cleanup gravity-zero drift');
legend('pre-cleanup target error', 'post-cleanup drift', ...
    'post-cleanup target error', 'Location', 'northwest');
grid on;
exportgraphics(figureHandle, ...
    fullfile(outputDir, 'ready_cleanup_drift.png'), 'Resolution', 180);
exportgraphics(figureHandle, ...
    fullfile(outputDir, 'ready_cleanup_drift.pdf'), 'ContentType', 'vector');
close(figureHandle);

analysis = struct('summary', summary, 'joint_table', jointTable, ...
    'pre_cleanup', pre, 'post_cleanup', post, ...
    'output_directory', string(java.io.File(outputDir).getCanonicalPath()));
save(fullfile(outputDir, 'ready_cleanup_analysis.mat'), 'analysis', ...
    'jointTable', 'summaryTable', '-v7');
end
