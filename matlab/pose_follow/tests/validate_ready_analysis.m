function validate_ready_analysis(dJson, aprimeJson, gravityJson, outputDir)
%VALIDATE_READY_ANALYSIS Validate legacy no-gravity and new gravity snapshots.
addpath(fileparts(fileparts(mfilename('fullpath'))));
files = [string(dJson); string(aprimeJson); string(gravityJson)];
names = ["D-no-gravity"; "Aprime-no-gravity"; "Aprime-gravity"];
d = [0.15,0.55,0.15,0.8,-0.1,0.15,0.1];
a = [0.0,0.2,0.0,0.6,0.0,0.0,0.0];
analysis = analyze_ready_comparison(files, outputDir, ...
    'ExperimentNames', names, 'TargetRad', [d; a; a], ...
    'PostureNames', ["openarm_right_ready_v1"; ...
        "openarm_right_ready_v2"; "openarm_right_ready_v2"], ...
    'GravityEnabled', [false; false; true]);
assert(numel(analysis.experiments) == 3);
assert(analysis.experiments(1).max_abs_error_rad > 0.20);
assert(analysis.experiments(2).max_abs_error_rad > 0.14);
assert(analysis.experiments(3).gravity_enabled);
required = ["ready_summary.csv"; "ready_joint_errors.csv"; ...
    "ready_group_comparison.csv"; "ready_analysis_summary.json"; ...
    "ready_analysis.mat"; "ready_target_error.png"];
for index = 1:numel(required)
    assert(isfile(fullfile(outputDir, required(index))), ...
        'Missing ready analysis output %s.', required(index));
end
fprintf('Validated ready target/gravity comparison: %s\n', outputDir);
end
