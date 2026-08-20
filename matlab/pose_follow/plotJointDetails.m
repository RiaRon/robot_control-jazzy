function figureHandle = plotJointDetails(runs, style, visible)
%PLOTJOINTDETAILS Plot J1/J4/J7 target-command-measured and jump markers.
selectedOrdinals = [1, 4, 7];
figureHandle = newFigure('J1 J4 J7 detail', visible, numel(runs));
layout = tiledlayout(figureHandle, numel(runs), numel(selectedOrdinals), ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, ...
    'J1/J4/J7 IK target-command-measured tracking with IK target jumps', ...
    'Interpreter', 'none');
timeLimit = globalTimeLimit(runs);
jointLimits = nan(numel(selectedOrdinals), 2);
for selectedIndex = 1:numel(selectedOrdinals)
    values = [];
    jointIndex = selectedOrdinals(selectedIndex);
    for runIndex = 1:numel(runs)
        if jointIndex <= numel(runs(runIndex).joint_names)
            values = [values; ... %#ok<AGROW>
                runs(runIndex).joint_positions_rad.ik_target(:, jointIndex); ...
                runs(runIndex).joint_positions_rad.command(:, jointIndex); ...
                runs(runIndex).joint_positions_rad.measured(:, jointIndex)];
        end
    end
    jointLimits(selectedIndex, :) = finiteRange(values);
end

for runIndex = 1:numel(runs)
    run = runs(runIndex);
    for selectedIndex = 1:numel(selectedOrdinals)
        jointIndex = selectedOrdinals(selectedIndex);
        ax = nexttile(layout);
        configureTimeAxes(ax, timeLimit, jointLimits(selectedIndex, :));
        addPhaseBands(ax, run.time_sec, run.phase, style);
        if jointIndex > numel(run.joint_names)
            text(ax, 0.5, 0.5, sprintf('J%d unavailable', jointIndex), ...
                'Units', 'normalized', 'HorizontalAlignment', 'center');
            continue;
        end
        handles = gobjects(3, 1);
        handles(1) = plot(ax, run.time_sec, ...
            run.joint_positions_rad.ik_target(:, jointIndex), '--', ...
            'LineWidth', 1.1, 'Color', style.joint_state_colors_rgb(1, :), ...
            'DisplayName', 'IK target');
        handles(2) = plot(ax, run.time_sec, ...
            run.joint_positions_rad.command(:, jointIndex), '-', ...
            'LineWidth', 1.1, 'Color', style.joint_state_colors_rgb(2, :), ...
            'DisplayName', 'command');
        handles(3) = plot(ax, run.time_sec, ...
            run.joint_positions_rad.measured(:, jointIndex), ':', ...
            'LineWidth', 1.4, 'Color', style.joint_state_colors_rgb(3, :), ...
            'DisplayName', 'measured');
        addJumpLines(ax, run, jointIndex, style);
        title(ax, run.experiment + " — " + run.joint_names(jointIndex), ...
            'Interpreter', 'none');
        xlabel(ax, 'Elapsed time (s)');
        ylabel(ax, 'Joint position (rad)');
        legend(ax, handles, 'Location', 'best', 'Interpreter', 'none');
    end
end
end
