function value = maxFinite(values)
%MAXFINITE Maximum finite element, or zero for an empty/non-finite input.
values = values(isfinite(values));
if isempty(values)
    value = 0;
else
    value = max(values(:));
end
end
