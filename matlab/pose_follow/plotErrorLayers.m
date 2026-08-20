function figureHandle = plotErrorLayers(runs, style, visible)
%PLOTERRORLAYERS Plot norm layers and signed live-error projections.
figureHandle = newFigure('TCP error layers', visible, numel(runs));
layout = tiledlayout(figureHandle, numel(runs), 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, ...
    'TCP position layer distance and signed projection by profile phase', ...
    'Interpreter', 'none');
columns = style.layer_columns;
distanceMax = 0;
projectionMax = 0;
for index = 1:numel(runs)
    distanceMax = max(distanceMax, ...
        maxFinite(runs(index).position_error_m(:, columns) * 1000));
    projectionMax = max(projectionMax, ...
        maxFinite(abs(runs(index).position_error_signed_projection_m(:, columns) * 1000)));
end
distanceLimit = positiveLimit(distanceMax);
projectionLimit = positiveLimit(projectionMax);
timeLimit = globalTimeLimit(runs);

for runIndex = 1:numel(runs)
    run = runs(runIndex);
    ax = nexttile(layout);
    configureTimeAxes(ax, timeLimit, [0, distanceLimit]);
    addPhaseBands(ax, run.time_sec, run.phase, style);
    handles = gobjects(numel(columns), 1);
    for layerIndex = 1:numel(columns)
        handles(layerIndex) = plot(ax, run.time_sec, ...
            run.position_error_m(:, columns(layerIndex)) * 1000, ...
            'LineWidth', 1.0, 'Color', style.layer_colors_rgb(layerIndex, :), ...
            'DisplayName', char(style.layer_names(layerIndex)));
    end
    title(ax, run.experiment + " — layer distances", 'Interpreter', 'none');
    xlabel(ax, 'Elapsed time (s)');
    ylabel(ax, 'Layer distance (mm)');
    legend(ax, handles, 'Location', 'best', 'Interpreter', 'none');

    ax = nexttile(layout);
    configureTimeAxes(ax, timeLimit, [-projectionLimit, projectionLimit]);
    addPhaseBands(ax, run.time_sec, run.phase, style);
    handles = gobjects(numel(columns), 1);
    for layerIndex = 1:numel(columns)
        handles(layerIndex) = plot(ax, run.time_sec, ...
            run.position_error_signed_projection_m(:, columns(layerIndex)) * 1000, ...
            'LineWidth', 1.0, 'Color', style.layer_colors_rgb(layerIndex, :), ...
            'DisplayName', char(style.layer_names(layerIndex)));
    end
    yline(ax, 0, 'k:', 'HandleVisibility', 'off');
    title(ax, run.experiment + " — signed projection", 'Interpreter', 'none');
    xlabel(ax, 'Elapsed time (s)');
    ylabel(ax, 'Projection on live-error direction (mm)');
    legend(ax, handles, 'Location', 'best', 'Interpreter', 'none');
end
end
