function limit = positiveLimit(value)
%POSITIVELIMIT Produce a non-degenerate upper plot limit.
value = maxFinite(value);
if ~isfinite(value) || value <= 0
    limit = 1;
else
    limit = value * 1.05;
end
end
