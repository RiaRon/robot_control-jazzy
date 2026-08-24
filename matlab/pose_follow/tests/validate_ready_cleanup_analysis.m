function validate_ready_cleanup_analysis(afterReadyFile, postCleanupFile, outputDir)
%VALIDATE_READY_CLEANUP_ANALYSIS Validate cleanup/pre-drift separation bundle.
addpath(fileparts(fileparts(mfilename('fullpath'))));
analysis = analyze_ready_cleanup_drift( ...
    afterReadyFile, postCleanupFile, outputDir, 'Visible', 'off');
summary = analysis.summary;
assert(abs(summary.pre_cleanup_worst_target_error_rad - ...
    0.047051194018461806) < 1e-12);
assert(summary.pre_cleanup_worst_joint == "r_aj_4");
assert(abs(summary.post_cleanup_worst_drift_rad - ...
    0.072861829556725155) < 1e-12);
assert(summary.post_cleanup_worst_drift_joint == "r_aj_4");
assert(summary.cleanup_recorded_attempted);
assert(summary.cleanup_recorded_zero_published);
required = ["ready_cleanup_summary.csv"; ...
    "ready_cleanup_joint_drift.csv"; "ready_cleanup_analysis.json"; ...
    "ready_cleanup_analysis.mat"; "ready_cleanup_drift.png"; ...
    "ready_cleanup_drift.pdf"];
for index = 1:numel(required)
    assert(isfile(fullfile(outputDir, required(index))), ...
        'Missing ready cleanup output %s.', required(index));
end
fprintf('Validated ready cleanup drift separation: %s\n', outputDir);
end
