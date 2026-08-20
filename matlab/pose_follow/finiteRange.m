function range = finiteRange(values)
%FINITERANGE Produce padded finite y-axis limits.
values = values(isfinite(values));
if isempty(values)
    range = [-1, 1];
    return;
end
minimum = min(values);
maximum = max(values);
span = maximum - minimum;
if span <= 1e-12
    span = max(abs(maximum), 1) * 0.1;
end
range = [minimum - 0.05 * span, maximum + 0.05 * span];
end
