function [extremal_points] = compute_extremal_points(centers, radii)
RAND_MAX = 32767;
D = length(centers{1});
min_x = RAND_MAX;
min_y = RAND_MAX;
min_z = RAND_MAX;
max_x = -RAND_MAX;
max_y = -RAND_MAX;
max_z = -RAND_MAX;

for i = 1:length(centers)
    if centers{i}(1) - radii{i} < min_x
        min_x = centers{i}(1) - radii{i};
        extremal_points.min_x = centers{i};
        extremal_points.min_x(1) = centers{i}(1) - radii{i};
    end
    if centers{i}(1) + radii{i} > max_x
        max_x = centers{i}(1) + radii{i};
        extremal_points.max_x = centers{i};
        extremal_points.max_x(1) = max_x;
    end
    if centers{i}(2) - radii{i} < min_y
        min_y = centers{i}(2) - radii{i};
        extremal_points.min_y = centers{i};
        extremal_points.min_y(2) = min_y;
    end
    if centers{i}(2) + radii{i} > max_y
        max_y = centers{i}(2) + radii{i};
        extremal_points.max_y = centers{i};
        extremal_points.max_y(2) = max_y;
    end
    if D == 3
        if centers{i}(3) - radii{i} < min_z
            min_z = centers{i}(3) - radii{i};
            extremal_points.min_z = centers{i};
            extremal_points.min_z(3) = min_z;
        end
        if centers{i}(3) + radii{i} > max_z
            max_z = centers{i}(3) + radii{i};
            extremal_points.max_z = centers{i};
            extremal_points.max_z(3) = max_z;
        end
    end
end
