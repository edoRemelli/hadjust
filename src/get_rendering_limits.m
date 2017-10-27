function [bounding_box] = get_rendering_limits(centers, radii, fov, data_bounding_box, view_axis)

D = length(centers{1});

model_bounding_box = compute_model_bounding_box(centers, radii);
extremal_points = compute_extremal_points(centers, radii);

max_x = max(model_bounding_box.max_x, data_bounding_box.max_x); min_x = min(model_bounding_box.min_x, data_bounding_box.min_x);
min_y = min(model_bounding_box.min_y, data_bounding_box.min_y); max_y = max(model_bounding_box.max_y, data_bounding_box.max_y);
if (D == 3)
    min_z = min(model_bounding_box.min_z, data_bounding_box.min_z); max_z = max(model_bounding_box.max_z, data_bounding_box.max_z);
end

if (strcmp(view_axis, 'X'))
    max_y = model_bounding_box.max_y + extremal_points.max_y(1) * tand(fov/2);
    max_y = max(max_y, data_bounding_box.max_y);
    min_y = model_bounding_box.min_y - extremal_points.min_y(1) * tand(fov/2);
    min_y = min(min_y, data_bounding_box.min_y);
    if D == 3
        max_z = model_bounding_box.max_z + extremal_points.max_z(1) * tand(fov/2);
        max_z = max(max_z, data_bounding_box.max_z);
        min_z = model_bounding_box.min_z - extremal_points.min_z(1) * tand(fov/2);
        min_z = min(min_z, data_bounding_box.min_z);
    end
end

if (strcmp(view_axis, 'Y'))
    max_x = model_bounding_box.max_x + extremal_points.max_y(2) * tand(fov/2);
    max_x = max(max_x, data_bounding_box.max_x);
    min_x = model_bounding_box.min_x - extremal_points.min_y(2) * tand(fov/2);
    min_x = min(min_x, data_bounding_box.min_x);
    if (D == 3)
        max_z = model_bounding_box.max_z + extremal_points.max_z(2) * tand(fov/2);
        max_z = max(max_z, data_bounding_box.max_z);
        min_z = model_bounding_box.min_z - extremal_points.min_z(2) * tand(fov/2);
        min_z = min(min_z, data_bounding_box.min_z);
    end
end


if (strcmp(view_axis, 'Z'))
    max_x = model_bounding_box.max_x; + extremal_points.max_z(3) * tand(fov/2);
    max_x = max(max_x, data_bounding_box.max_x);
    min_x = model_bounding_box.min_x - extremal_points.min_z(3) * tand(fov/2);
    min_x = min(min_x, data_bounding_box.min_x);
    max_y = model_bounding_box.max_y + extremal_points.max_y(3) * tand(fov/2);
    max_y = max(max_y, data_bounding_box.max_y);
    min_y = model_bounding_box.min_y - extremal_points.min_y(3) * tand(fov/2);
    min_y = min(min_y, data_bounding_box.min_y);
end

bounding_box.min_x = min_x; bounding_box.max_x = max_x;
bounding_box.min_y = min_y; bounding_box.max_y = max_y;
if (D == 3)
    bounding_box.min_z = min_z; bounding_box.max_z = max_z;
end

% bounding_box.max_x = bounding_box.max_x + 10;
% bounding_box.max_y = bounding_box.max_y + 10;
% bounding_box.min_x = bounding_box.min_x - 10;
% bounding_box.min_y = bounding_box.min_y - 10;
