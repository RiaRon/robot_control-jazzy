function validate_pose_follow_parser(legacyJson, currentJson, outputDir)
%VALIDATE_POSE_FOLLOW_PARSER Exercise both supported pose-follow variants.
%   The files are supplied by the caller and are never copied or modified.
if nargin < 3
    outputDir = fullfile(tempdir, 'openarm-pose-follow-analysis-validation');
end
toolDir = fileparts(fileparts(mfilename('fullpath')));
addpath(toolDir);

legacy = read_pose_follow_json(legacyJson, 'legacy-real');
current = read_pose_follow_json(currentJson, 'current-fake');
assert(legacy.schema_variant == "legacy-2026-08-18");
assert(current.schema_variant == "extended");
assert(all(isfinite(legacy.position_error_signed_projection_m(:, [1, 3:6])), ...
    'all'), 'Legacy signed projections were not reconstructed.');
for phase = ["ramp", "hold", "return", "origin-hold"]
    assert(any(current.phase == phase), ...
        'Current fake JSON is missing canonical phase %s.', phase);
end

analysis = analyze_pose_follow( ...
    [string(legacyJson); string(currentJson)], outputDir, ...
    'ExperimentNames', ["legacy-real"; "current-fake"]);
assert(numel(analysis.experiments) == 2);

required = [ ...
    "summary.csv"; "analysis_summary.json"; "analysis.mat"; ...
    "tcp_error_timeseries.png"; "error_layers.png"; ...
    "joint_tracking.png"; "j1_j4_j7_detail.png"; ...
    "ik_events.png"; "phase_comparison.png"; "research_report.pdf"];
for index = 1:numel(required)
    assert(isfile(fullfile(outputDir, required(index))), ...
        'Missing bundle file %s.', required(index));
end
fprintf('Validated legacy and extended pose-follow JSON: %s\n', outputDir);
end
