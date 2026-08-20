function figureHandle = plotTcpErrors(runs, style, visible)
%PLOTTCPERRORS Plot position/orientation errors with shared axes.
figureHandle = newFigure('TCP error time series', visible, numel(runs));
layout = tiledlayout(figureHandle, numel(runs), 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'TCP position and orientation error by experiment and profile phase', ...
    'Interpreter', 'none');
positionLimit = 0;
orientationLimit = 0;
for index = 1:numel(runs)
    positionLimit = max(positionLimit, ...
        maxFinite(runs(index).position_error_m(:, 1) * 1000));
    orientationLimit = max(orientationLimit, ...
        maxFinite(rad2deg(runs(index).orientation_error_rad(:, 1))));
end
positionLimit = positiveLimit(positionLimit);
orientationLimit = positiveLimit(orientationLimit);
timeLimit = globalTimeLimit(runs);

for index = 1:numel(runs)
    run = runs(index);
    color = style.experiment_colors_rgb(index, :);
    ax = nexttile(layout);
    configureTimeAxes(ax, timeLimit, [0, positionLimit]);
    phaseHandles = addPhaseBands(ax, run.time_sec, run.phase, style);
    lineHandle = plot(ax, run.time_sec, run.position_error_m(:, 1) * 1000, ...
        'LineWidth', 1.25, 'Color', color, ...
        'DisplayName', char(run.experiment + " TCP position"));
    title(ax, run.experiment + " — position", 'Interpreter', 'none');
    xlabel(ax, 'Elapsed time (s)');
    ylabel(ax, 'Position error (mm)');
    legend(ax, [lineHandle; phaseHandles], 'Location', 'best', ...
        'Interpreter', 'none');

    ax = nexttile(layout);
    configureTimeAxes(ax, timeLimit, [0, orientationLimit]);
    addPhaseBands(ax, run.time_sec, run.phase, style);
    plot(ax, run.time_sec, rad2deg(run.orientation_error_rad(:, 1)), ...
        'LineWidth', 1.25, 'Color', color, ...
        'DisplayName', char(run.experiment + " TCP orientation"));
    title(ax, run.experiment + " — orientation", 'Interpreter', 'none');
    xlabel(ax, 'Elapsed time (s)');
    ylabel(ax, 'Orientation error (deg)');
    legend(ax, 'Location', 'best', 'Interpreter', 'none');
end
end
