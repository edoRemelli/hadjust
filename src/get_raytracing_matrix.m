function [raytracing_matrix, camera_axis, camera_center] = get_raytracing_matrix(centers, radii, data_bounding_box, view_axis, settings, side)
fov = settings.fov; H = settings.H; W = settings.W; D = settings.D;
f = H/2/tand(fov/2);

if D == 3
    
    if (strcmp(view_axis, 'X'))
        [bb] = get_rendering_limits(centers, radii, fov, data_bounding_box, view_axis);
        delta = max(bb.max_y - bb.min_y, bb.max_z - bb.min_z);
        offset = delta / tand(fov/2);
        z = [0; 0; 1];
        camera_axis = [-1; 0; 0];          
    end
    if (strcmp(view_axis, 'Y'))
        [bb] = get_rendering_limits(centers, radii, fov, data_bounding_box, view_axis);
        delta = max(bb.max_x - bb.min_x, bb.max_z - bb.min_z);
        offset = delta / tand(fov/2);
        z = [0; 0; 1];
        camera_axis = [0; -1; 0];
    end
    if (strcmp(view_axis, 'Z'))
        [bb] = get_rendering_limits(centers, radii, fov, data_bounding_box, view_axis);
        delta = max(bb.max_x - bb.min_x, bb.max_y - bb.min_y);
        offset = delta / tand(fov/2);
        z = [0; 1; 0];
        camera_axis = [0; 0; -1];
    end
    
    if strcmp(side, 'back'), camera_axis = -camera_axis; end
    S = f * tand(fov/2) / H;
    Mean = [mean([bb.min_x, bb.max_x]); ...
        mean([bb.min_y, bb.max_y]); mean([bb.min_z, bb.max_z])];
    camera_center = Mean + offset * camera_axis;
    
    w = camera_axis;
    v = z - (w' * z) * w; v = v / norm(v);
    u = cross(v, w); u = u / norm(u);
    
    n0 = W/2; m0 = H/2;
    A = zeros(3, 3);
    A(1, 1) = - S / f; A(2, 2) = - S / f;
    A(1, 3) = n0 * S / f; A(2, 3) = m0 * S / f;
    A(3, 3) = 1;
    raytracing_matrix = [u, v, w] * A;
    
else
    
    if (strcmp(view_axis, 'X'))
        [bb] = get_rendering_limits(centers, radii, fov, data_bounding_box, view_axis);
        delta = bb.max_y - bb.min_y;
        offset = delta / tand(fov/2);
        S = f * tand(fov/2) / H;
        z = [0; 1];
        camera_axis = [-1; 0];
    end
    if (strcmp(view_axis, 'Y'))
        [bb] = get_rendering_limits(centers, radii, fov, data_bounding_box, view_axis);
        delta = max(bb.max_x - bb.min_x);
        offset = delta / tand(fov/2);
        S = f * tand(fov/2) / H;
        z = [1; 0];
        camera_axis = [0; -1];
    end
    Mean = [mean([bb.min_x, bb.max_x]); mean([bb.min_y, bb.max_y])];
    camera_center = Mean + offset * camera_axis;
    
    w = camera_axis;
    v = z - (w' * z) * w; v = v / norm(v);
    
    n0 = H/2;
    A = zeros(2, 2);
    A(1, 1) = - S / f; A(2, 2) = 1;
    A(1, 2) = n0 * S / f; A(2, 1) = 0;
    
    raytracing_matrix = [v, w] * A;
    
end