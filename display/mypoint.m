function [] = mypoint(p, color, varargin)

point_size = 12;
if ~isempty(varargin)
    point_size = varargin{1};
end


if (length(p) == 3)
    scatter3(p(1), p(2), p(3), point_size, color, 'o', 'filled');
end
if (length(p) == 2)
    scatter(p(1), p(2), point_size, color, 'o', 'filled');
end

