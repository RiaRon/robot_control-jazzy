function figureHandle = plotPhaseComparison(runs, summaryTable, style, visible)
%PLOTPHASECOMPARISON Compare canonical phase metrics with fixed colors.
figureHandle = newFigure('Phase comparison', visible, 1);
layout = tiledlayout(figureHandle, 2, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, ...
    'Cross-experiment comparison with fixed experiment colors and canonical phases', ...
    'Interpreter', 'none');
phases = ["ramp"; "hold"; "return"; "origin-hold"];
metrics = { ...
    'tcp_position_rms_mm', 'TCP position RMS (mm)'; ...
    'tcp_position_p95_mm', 'TCP position p95 (mm)'; ...
    'tcp_orientation_rms_deg', 'TCP orientation RMS (deg)'; ...
    'tcp_orientation_p95_deg', 'TCP orientation p95 (deg)'};

for metricIndex = 1:size(metrics, 1)
    ax = nexttile(layout);
    hold(ax, 'on');
    handles = gobjects(numel(runs), 1);
    for runIndex = 1:numel(runs)
        values = nan(numel(phases), 1);
        for phaseIndex = 1:numel(phases)
            mask = summaryTable.experiment == runs(runIndex).experiment & ...
                summaryTable.phase == phases(phaseIndex);
            if any(mask)
                values(phaseIndex) = summaryTable.(metrics{metricIndex, 1})(mask);
            end
        end
        handles(runIndex) = plot(ax, 1:numel(phases), values, '-o', ...
            'LineWidth', 1.4, 'MarkerSize', 5, ...
            'Color', style.experiment_colors_rgb(runIndex, :), ...
            'DisplayName', char(runs(runIndex).experiment));
    end
    grid(ax, 'on');
    xlim(ax, [0.75, numel(phases) + 0.25]);
    xticks(ax, 1:numel(phases));
    xticklabels(ax, cellstr(phases));
    xlabel(ax, 'Profile phase');
    ylabel(ax, metrics{metricIndex, 2});
    title(ax, metrics{metricIndex, 2}, 'Interpreter', 'none');
    legend(ax, handles, 'Location', 'best', 'Interpreter', 'none');
end
end
