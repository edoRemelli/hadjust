function [] = myline(a, b, color, varargin)

line_width = 2;
if ~isempty(varargin)
    line_width = varargin{1};
end

if length(a) == 3
    line([a(1) b(1)], [a(2) b(2)], [a(3) b(3)], 'lineWidth', line_width, 'color', color);
end

if length(a) == 2
    line([a(1) b(1)], [a(2) b(2)], 'lineWidth', line_width, 'color', color);
end