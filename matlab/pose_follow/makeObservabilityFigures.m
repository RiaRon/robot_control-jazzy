function [figures, names] = makeObservabilityFigures( ...
        runs, summaryTable, jointSummaryTable, style, visible)
%MAKEOBSERVABILITYFIGURES Build the twelve pose-follow observability views.
% All angles are computed from relative, sign-invariant quaternions.  RPY
% subtraction is intentionally not used.

names = [ ...
    "01_tcp_translation_layers"; ...
    "02_tcp_orientation_layers"; ...
    "03_translation_error_decomposition"; ...
    "04_orientation_error_decomposition"; ...
    "05_joint_positions"; "06_joint_velocities"; ...
    "07_handoff_convergence_zoom"; ...
    "08_hold_command_lead_overshoot"; ...
    "09_limiter_activation"; "10_phase_statistics"; ...
    "11_joint_max_error_heatmap"; "12_profile_comparison"];
figures = cell(numel(names), 1);
run = runs(end);
figures{1} = tcpTranslation(run, style, visible);
figures{2} = tcpOrientation(run, style, visible);
figures{3} = translationDecomposition(run, style, visible);
figures{4} = orientationDecomposition(run, style, visible);
figures{5} = jointPositions(run, style, visible);
figures{6} = jointVelocities(run, style, visible);
figures{7} = handoffZoom(run, style, visible);
figures{8} = holdZoom(run, style, visible);
figures{9} = limiterActivation(run, style, visible);
figures{10} = phaseStatistics(run, summaryTable, style, visible);
figures{11} = jointHeatmap(run, jointSummaryTable, style, visible);
figures{12} = profileComparison(runs, summaryTable, style, visible);
end


function f = tcpTranslation(run, style, visible)
f = newFigure('TCP translation layers', visible, 3);
t = run.time_sec;
layers = {'live_marker', 'ik_target', 'command', 'measured'};
labels = {'target', 'IK FK', 'command FK', 'measured'};
lineStyles = {'--', '-.', '-', ':'};
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact');
axisLabels = ["x", "y", "z"];
for axisIndex = 1:3
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    for layerIndex = 1:numel(layers)
        values = run.tcp_positions_m.(layers{layerIndex})(:, axisIndex);
        plot(ax, t, values, 'LineStyle', lineStyles{layerIndex}, ...
            'LineWidth', 1.25, 'DisplayName', labels{layerIndex});
    end
    ylabel(ax, char("TCP " + axisLabels(axisIndex) + " [m]"));
    addBoundaries(ax, t, run.phase);
    if axisIndex == 1, legend(ax, 'Location', 'best'); end
end
xlabel(nexttile(tl, 3), 'Run-relative time [s]');
title(tl, metadataTitle(run, 'TCP translation target / IK / command / measured'));
end


function f = tcpOrientation(run, style, visible)
f = newFigure('TCP orientation layers', visible, 2);
t = run.time_sec;
measured = run.tcp_orientations_xyzw.measured;
layers = {'live_marker', 'accepted_marker', 'ik_target', 'command'};
labels = {'target to measured', 'accepted to measured', ...
    'IK FK to measured', 'command FK to measured'};
lineStyles = {'--', '-.', '-', ':'};
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
for index = 1:numel(layers)
    angle = quaternionAngle(run.tcp_orientations_xyzw.(layers{index}), measured);
    plot(ax, t, rad2deg(angle), 'LineStyle', lineStyles{index}, ...
        'LineWidth', 1.25, 'DisplayName', labels{index});
end
ylabel(ax, 'Relative orientation error [deg]'); legend(ax, 'Location', 'best');
addBoundaries(ax, t, run.phase);
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
plot(ax, t, rad2deg(run.orientation_error_rad(:, 4)), '--', ...
    'DisplayName', 'accepted to IK');
plot(ax, t, rad2deg(run.orientation_error_rad(:, 5)), '-.', ...
    'DisplayName', 'IK to command');
plot(ax, t, rad2deg(run.orientation_error_rad(:, 6)), '-', ...
    'DisplayName', 'command to measured');
ylabel(ax, 'Layer orientation error [deg]'); xlabel(ax, 'Run-relative time [s]');
legend(ax, 'Location', 'best'); addBoundaries(ax, t, run.phase);
title(tl, metadataTitle(run, 'Relative-rotation orientation observability'));
end


function f = translationDecomposition(run, style, visible)
f = newFigure('Translation error decomposition', visible, 1);
ax = axes(f); hold(ax, 'on'); grid(ax, 'on');
columns = [1, 3, 4, 5, 6];
labels = {'live to measured', 'live to accepted', 'accepted to IK', ...
    'IK to command', 'command to measured'};
lineStyles = {'--', ':', '-.', '-', '--'};
for index = 1:numel(columns)
    plot(ax, run.time_sec, ...
        1000 * run.position_error_signed_projection_m(:, columns(index)), ...
        'LineStyle', lineStyles{index}, 'LineWidth', 1.25, ...
        'DisplayName', labels{index});
end
xlabel(ax, 'Run-relative time [s]'); ylabel(ax, 'Signed translation error [mm]');
legend(ax, 'Location', 'best'); addBoundaries(ax, run.time_sec, run.phase);
title(ax, metadataTitle(run, 'Signed translation error layer decomposition'));
end


function f = orientationDecomposition(run, style, visible)
f = newFigure('Orientation error decomposition', visible, 1);
ax = axes(f); hold(ax, 'on'); grid(ax, 'on');
columns = [1, 3, 4, 5, 6];
labels = {'live to measured', 'live to accepted', 'accepted to IK', ...
    'IK to command', 'command to measured'};
lineStyles = {'--', ':', '-.', '-', '--'};
for index = 1:numel(columns)
    plot(ax, run.time_sec, rad2deg(run.orientation_error_rad(:, columns(index))), ...
        'LineStyle', lineStyles{index}, 'LineWidth', 1.25, ...
        'DisplayName', labels{index});
end
xlabel(ax, 'Run-relative time [s]'); ylabel(ax, 'Relative orientation angle [deg]');
legend(ax, 'Location', 'best'); addBoundaries(ax, run.time_sec, run.phase);
title(ax, metadataTitle(run, 'Orientation error layer decomposition'));
end


function f = jointPositions(run, style, visible)
f = newFigure('Joint positions', visible, 4);
tl = tiledlayout(f, 4, 2, 'TileSpacing', 'compact');
for jointIndex = 1:numel(run.joint_names)
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    plot(ax, run.time_sec, run.joint_positions_rad.ik_target(:, jointIndex), ...
        '--', 'DisplayName', 'IK target');
    plot(ax, run.time_sec, run.joint_positions_rad.command(:, jointIndex), ...
        '-', 'DisplayName', 'command');
    plot(ax, run.time_sec, run.joint_positions_rad.measured(:, jointIndex), ...
        ':', 'LineWidth', 1.25, 'DisplayName', 'measured');
    ylabel(ax, char(run.joint_names(jointIndex) + " [rad]"));
    addBoundaries(ax, run.time_sec, run.phase);
    if jointIndex == 1, legend(ax, 'Location', 'best'); end
end
xlabel(nexttile(tl, 7), 'Run-relative time [s]');
title(tl, metadataTitle(run, 'J1-J7 IK / command / measured'));
end


function f = jointVelocities(run, style, visible)
f = newFigure('Joint velocities', visible, 4);
tl = tiledlayout(f, 4, 2, 'TileSpacing', 'compact');
for jointIndex = 1:numel(run.joint_names)
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    plot(ax, run.time_sec, run.joint_velocity_rad_s.command(:, jointIndex), ...
        '-', 'DisplayName', 'command velocity');
    plot(ax, run.time_sec, run.joint_velocity_rad_s.measured(:, jointIndex), ...
        ':', 'LineWidth', 1.25, 'DisplayName', 'measured velocity');
    ylabel(ax, char(run.joint_names(jointIndex) + " [rad/s]"));
    addBoundaries(ax, run.time_sec, run.phase);
    if jointIndex == 1, legend(ax, 'Location', 'best'); end
end
xlabel(nexttile(tl, 7), 'Run-relative time [s]');
title(tl, metadataTitle(run, 'J1-J7 command and measured velocity'));
end


function f = handoffZoom(run, style, visible)
f = newFigure('Handoff convergence zoom', visible, 2);
mask = ismember(run.phase, ["handoff", "startup-alignment", "convergence-gate"]);
if ~any(mask)
    finiteTime = run.time_sec(isfinite(run.time_sec));
    mask = run.time_sec <= min(finiteTime) + ...
        0.25 * (max(finiteTime) - min(finiteTime));
end
t = run.time_sec(mask);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
plot(ax, t, 1000 * run.position_error_m(mask, 1), '-', ...
    'DisplayName', 'marker to measured');
ylabel(ax, 'TCP error [mm]'); legend(ax, 'Location', 'best');
addBoundaries(ax, t, run.phase(mask));
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
ikError = max(abs(run.joint_positions_rad.ik_target(mask, :) - ...
    run.joint_positions_rad.command(mask, :)), [], 2, 'omitnan');
cmdError = max(abs(run.joint_positions_rad.command(mask, :) - ...
    run.joint_positions_rad.measured(mask, :)), [], 2, 'omitnan');
plot(ax, t, ikError, '--', 'DisplayName', 'max |IK-command|');
plot(ax, t, cmdError, '-', 'DisplayName', 'max |command-measured|');
ylabel(ax, 'Joint error [rad]'); xlabel(ax, 'Run-relative time [s]');
legend(ax, 'Location', 'best'); addBoundaries(ax, t, run.phase(mask));
title(tl, metadataTitle(run, 'Ready handoff and bounded convergence gate'));
end


function f = holdZoom(run, style, visible)
f = newFigure('Hold command lead and overshoot', visible, 2);
mask = run.phase == "hold";
if ~any(mask), mask = run.phase == "origin-hold"; end
t = run.time_sec(mask);
error = run.joint_positions_rad.command(mask, :) - ...
    run.joint_positions_rad.measured(mask, :);
[~, jointIndex] = max(max(abs(error), [], 1, 'omitnan'));
if isempty(jointIndex), jointIndex = 1; end
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
plot(ax, t, run.joint_positions_rad.ik_target(mask, jointIndex), '--', ...
    'DisplayName', 'IK target');
plot(ax, t, run.joint_positions_rad.command(mask, jointIndex), '-', ...
    'DisplayName', 'command');
plot(ax, t, run.joint_positions_rad.measured(mask, jointIndex), ':', ...
    'DisplayName', 'measured');
ylabel(ax, char(run.joint_names(jointIndex) + " [rad]")); legend(ax, 'Location', 'best');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
plot(ax, t, error(:, jointIndex), '-o', 'MarkerIndices', sparseMarkers(t), ...
    'DisplayName', 'command lead'); yline(ax, 0, ':');
ylabel(ax, 'Command - measured [rad]'); xlabel(ax, 'Run-relative time [s]');
legend(ax, 'Location', 'best');
title(tl, metadataTitle(run, 'Hold command lead and overshoot zoom'));
end


function f = limiterActivation(run, style, visible)
f = newFigure('Limiter activation', visible, 1);
ax = axes(f); hold(ax, 'on'); grid(ax, 'on');
stairs(ax, run.time_sec, double(run.limits.cartesian_speed), '-', ...
    'LineWidth', 1.25, 'DisplayName', 'Cartesian linear step');
stairs(ax, run.time_sec, 1.2 * double(run.limits.cartesian_angular_speed), '--', ...
    'LineWidth', 1.25, 'DisplayName', 'Cartesian angular step');
if isfield(run.limits, 'joint_velocity')
    stairs(ax, run.time_sec, 1.4 * double(run.limits.joint_velocity), '-.', ...
        'DisplayName', 'joint velocity');
    stairs(ax, run.time_sec, 1.6 * double(run.limits.lead), ':', ...
        'DisplayName', 'lead');
    stairs(ax, run.time_sec, 1.8 * double(run.limits.position), '-o', ...
        'DisplayName', 'position');
end
ylim(ax, [-0.1, 2.1]); xlabel(ax, 'Run-relative time [s]');
ylabel(ax, 'Activation (offset for visibility)'); legend(ax, 'Location', 'best');
addBoundaries(ax, run.time_sec, run.phase);
title(ax, metadataTitle(run, 'Limiter activation overlay'));
end


function f = phaseStatistics(run, summaryTable, style, visible)
f = newFigure('Phase statistics', visible, 1);
ax = axes(f); grid(ax, 'on');
phases = ["handoff", "startup-alignment", "convergence-gate", ...
    "profile-only", "ramp", "hold", "return", "origin-hold"];
maskRun = summaryTable.experiment == run.experiment;
values = nan(numel(phases), 4);
fields = {'tcp_position_mean_mm', 'tcp_position_rms_mm', ...
    'tcp_position_max_mm', 'tcp_position_p95_mm'};
for phaseIndex = 1:numel(phases)
    rowMask = maskRun & summaryTable.phase == phases(phaseIndex);
    if any(rowMask)
        for metricIndex = 1:4
            values(phaseIndex, metricIndex) = ...
                summaryTable.(fields{metricIndex})(find(rowMask, 1));
        end
    end
end
bar(ax, values); xticks(ax, 1:numel(phases)); xticklabels(ax, phases);
xtickangle(ax, 25); ylabel(ax, 'Target to measured translation error [mm]');
legend(ax, {'mean', 'RMS', 'max', 'p95'}, 'Location', 'best');
title(ax, metadataTitle(run, 'Phase-separated mean / RMS / max / p95'));
end


function f = jointHeatmap(run, jointSummaryTable, style, visible)
f = newFigure('Joint maximum error heatmap', visible, 1);
mask = jointSummaryTable.experiment == run.experiment & ...
    jointSummaryTable.phase == "profile-only";
rows = jointSummaryTable(mask, :);
values = nan(2, numel(run.joint_names));
layers = ["ik_to_measured", "command_to_measured"];
for layerIndex = 1:2
    for jointIndex = 1:numel(run.joint_names)
        selected = rows.error_layer == layers(layerIndex) & ...
            rows.joint == run.joint_names(jointIndex);
        if any(selected)
            values(layerIndex, jointIndex) = rows.max_abs_rad(find(selected, 1));
        end
    end
end
ax = axes(f);
imagesc(ax, values); colorbar(ax); colormap(ax, parula);
xticks(ax, 1:numel(run.joint_names)); xticklabels(ax, run.joint_names);
yticks(ax, 1:2); yticklabels(ax, {'IK-measured', 'command-measured'});
xlabel(ax, 'Joint'); ylabel(ax, 'Error layer');
title(ax, metadataTitle(run, 'Profile-only joint maximum absolute error [rad]'));
end


function f = profileComparison(runs, summaryTable, style, visible)
f = newFigure('Profile comparison', visible, 1);
ax = axes(f); grid(ax, 'on'); hold(ax, 'on');
metrics = {'tcp_position_rms_mm', 'tcp_position_max_mm', ...
    'tcp_orientation_rms_deg', 'tcp_orientation_max_deg'};
values = nan(numel(runs), numel(metrics)); labels = strings(numel(runs), 1);
for runIndex = 1:numel(runs)
    row = summaryTable.experiment == runs(runIndex).experiment & ...
        summaryTable.phase == "profile-only";
    labels(runIndex) = runs(runIndex).profile;
    if strlength(labels(runIndex)) == 0, labels(runIndex) = runs(runIndex).experiment; end
    for metricIndex = 1:numel(metrics)
        values(runIndex, metricIndex) = summaryTable.(metrics{metricIndex})(find(row, 1));
    end
end
yyaxis(ax, 'left'); bar(ax, (1:numel(runs)) - 0.18, values(:, 1:2), 0.35);
ylabel(ax, 'Translation error [mm]');
yyaxis(ax, 'right'); bar(ax, (1:numel(runs)) + 0.18, values(:, 3:4), 0.35);
ylabel(ax, 'Orientation error [deg]');
xticks(ax, 1:numel(runs)); xticklabels(ax, labels); xtickangle(ax, 20);
legend(ax, {'translation RMS', 'translation max', ...
    'orientation RMS', 'orientation max'}, 'Location', 'best');
title(ax, "Translation / rotation / combined profile-only comparison | " + ...
    strjoin(string({runs.experiment}), ', '));
end


function angle = quaternionAngle(qTarget, qMeasured)
qTarget = normalizeQuaternion(qTarget);
qMeasured = normalizeQuaternion(qMeasured);
dotProduct = sum(qTarget .* qMeasured, 2);
dotProduct = min(1, max(-1, abs(dotProduct)));
angle = 2 * acos(dotProduct);
invalid = any(~isfinite(qTarget), 2) | any(~isfinite(qMeasured), 2);
angle(invalid) = nan;
end


function q = normalizeQuaternion(q)
norms = sqrt(sum(q .^ 2, 2));
valid = isfinite(norms) & norms > eps;
q(valid, :) = q(valid, :) ./ norms(valid);
q(~valid, :) = nan;
end


function titleText = metadataTitle(run, purpose)
[~, sourceName, sourceExt] = fileparts(char(run.source_file));
valid = run.time_sec(isfinite(run.time_sec));
if isempty(valid)
    window = "window unavailable";
else
    window = sprintf('window %.3f-%.3f s', min(valid), max(valid));
end
titleText = string(purpose) + " | " + sourceName + sourceExt + ...
    " | " + window;
end


function addBoundaries(ax, t, phase)
valid = isfinite(t);
t = t(valid); phase = phase(valid);
if numel(t) < 2, return; end
starts = [1; find(phase(2:end) ~= phase(1:end-1)) + 1];
for index = starts(2:end).'
    boundary = t(index);
    if isempty(boundary) || ~isfinite(boundary), continue; end
    xline(ax, boundary, ':', 'HandleVisibility', 'off');
end
end


function indices = sparseMarkers(t)
count = numel(t);
if count == 0
    indices = [];
else
    indices = unique(round(linspace(1, count, min(12, count))));
end
end
