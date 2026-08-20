function figureHandle = newFigure(name, visible, rowCount)
%NEWFIGURE Consistent report figure sizing and background.
height = min(1600, max(650, 330 * rowCount));
figureHandle = figure('Name', name, 'Visible', visible, 'Color', 'white', ...
    'Position', [100, 100, 1400, height]);
end
