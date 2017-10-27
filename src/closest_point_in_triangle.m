function [t, index] = closest_point_in_triangle(v1, v2, v3, p, index1, index2, index3)

n = cross(v1 - v2, v1 - v3);
n = n / norm(n);
distance = (p - v1)' * n;
t = p - n * distance;
t_plane = t;

is_in_triangle = is_point_in_triangle(t, v1, v2, v3);

if is_in_triangle == true
    index = [index1, index2, index3];
else
    [t12, index12] = closest_point_in_segment(v1, v2, p, index1, index2);
    [t13, index13] = closest_point_in_segment(v1, v3, p, index1, index3);
    [t23, index23] = closest_point_in_segment(v2, v3, p, index2, index3);
    d12 = norm(p - t12); d13 = norm(p - t13); d23 = norm(p - t23);
    t = [t12, t13, t23];
    indices{1} = index12; indices{2} = index13; indices{3} = index23;
    [~, i] = min([d12, d13, d23]);
    t = t(:, i);
    index = indices{i};
end

% figure; hold on;
% line([v1(1) v2(1)], [v1(2) v2(2)], [v1(3) v2(3)], 'lineWidth', 2);
% line([v1(1) v3(1)], [v1(2) v3(2)], [v1(3) v3(3)], 'lineWidth', 2);
% line([v2(1) v3(1)], [v2(2) v3(2)], [v2(3) v3(3)], 'lineWidth', 2);
% scatter3(p(1), p(2), p(3), 30, 'filled');
% scatter3(t(1), t(2), t(3), 30, 'filled', 'm');
% line([p(1) t_plane(1)], [p(2) t_plane(2)], [p(3) t_plane(3)], 'lineWidth', 2, 'color', 'g');
% line([p(1) t(1)], [p(2) t(2)], [p(3) t(3)], 'lineWidth', 2, 'color', 'm');
% axis equal;

end





