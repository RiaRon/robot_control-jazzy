function figureHandle = plotJointTracking(runs, style, visible)
%PLOTJOINTTRACKING Plot maximum decomposed tracking error across all joints.
figureHandle = newFigure('Joint tracking overview', visible, numel(runs));
layout = tiledlayout(figureHandle, numel(runs), 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, ...
    'Maximum joint tracking decomposition across J1-J7 by profile phase', ...
    'Interpreter', 'none');
limit = 0;
series = cell(numel(runs), 2);
for index = 1:numel(runs)
    target = runs(index).joint_positions_rad.ik_target;
    command = runs(index).joint_positions_rad.command;
    measured = runs(index).joint_positions_rad.measured;
    series{index, 1} = max(abs(target - command), [], 2, 'omitnan');
    series{index, 2} = max(abs(command - measured), [], 2, 'omitnan');
    limit = max(limit, maxFinite([series{index, 1}; series{index, 2}]));
end
limit = positiveLimit(limit);
timeLimit = globalTimeLimit(runs);

for index = 1:numel(runs)
    run = runs(index);
    ax = nexttile(layout);
    configureTimeAxes(ax, timeLimit, [0, limit]);
    phaseHandles = addPhaseBands(ax, run.time_sec, run.phase, style);
    first = plot(ax, run.time_sec, series{index, 1}, '--', ...
        'LineWidth', 1.25, 'Color', style.joint_state_colors_rgb(1, :), ...
        'DisplayName', 'max |IK target - command|');
    second = plot(ax, run.time_sec, series{index, 2}, '-', ...
        'LineWidth', 1.25, 'Color', style.joint_state_colors_rgb(2, :), ...
        'DisplayName', 'max |command - measured|');
    title(ax, run.experiment + " — all joints", 'Interpreter', 'none');
    xlabel(ax, 'Elapsed time (s)');
    ylabel(ax, 'Maximum absolute error (rad)');
    legend(ax, [first; second; phaseHandles], 'Location', 'best', ...
        'Interpreter', 'none');
end
end
