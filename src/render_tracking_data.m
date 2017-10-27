function [rendered_data, back_map_for_rendered_data,P] = ...
    render_tracking_data(points, camera_axis, camera_center, view_axis, closing_radius, dialation_radius, settings)

fov = settings.fov; H = settings.H; W = settings.W;

RAND_MAX = 32767;
focal = H/tand(fov/2);
a = camera_axis; t = camera_center;
A = [focal, 0, W/2; 0, focal, H/2; 0, 0, 1];
rendered_data = zeros(H, W);
back_map_for_rendered_data = zeros(H, W, 3);
min_r = RAND_MAX; max_r = -RAND_MAX;
min_c = RAND_MAX; max_c = -RAND_MAX;
for i = 1:length(points)
    p = [points{i}; 1];
    if strcmp(view_axis, 'Z'), b = [0; 0; -1]; R = vrrotvec2mat(vrrotvec(a, b));
        P = A * [R -R*t]; m = P * p; m = m ./ m(3); m = [W - m(1); m(2)]; end
    if strcmp(view_axis, 'Y'), b = [0; -1; 0]; R = vrrotvec2mat(vrrotvec(a, b)); R = [R(1, :); R(3, :); R(2, :)];
        P = A * [R -R*t]; m = P * p; m = m ./ m(3); m = [m(1); m(2)]; end
    if strcmp(view_axis, 'X'), b = [-1; 0; 0]; R = vrrotvec2mat(vrrotvec(a, b)); R = [R(2, :); R(3, :); R(1, :)];
        P = A * [R -R*t]; m = P * p; m = m ./ m(3); m = [W - m(1); m(2)]; end
    if (m(2) < 1 || m(2) > H || m(1) < 1 || m(1) > W), continue; end
    rendered_data(round(m(2)), round(m(1))) = 1;
    if round(m(2)) < min_r, min_r = round(m(2)); end;
    if round(m(2)) > max_r, max_r = round(m(2)); end;
    if round(m(1)) < min_c, min_c = round(m(1)); end;
    if round(m(1)) > max_c, max_c = round(m(1)); end;
    back_map_for_rendered_data(round(m(2)), round(m(1)), :) = points{i};
end

rendered_data = imfill(imclose(rendered_data, strel('disk', closing_radius, 0)));
rendered_data = imdilate(rendered_data,  strel('disk', dialation_radius, 8));

min_r = min_r - 8;
max_r = max_r + 8;
min_c = min_c - 8;
max_c = max_c + 8;
rendered_data([1:min_r, max_r:end], :) = 0; rendered_data(:, [1:min_c, max_c:end]) = 0;
%figure; imshow(rendered_data, []); drawnow;

%% Improve boundary

if settings.sparse_data
    boarder_points = zeros(size(rendered_data));
    [I, J] = find(rendered_data);
    for i = 1:length(I)
        count = 0;
        for u = -1:1
            for v = -1:1
                if u == 0 && v == 0, continue; end
                if I(i) + u < 1 || I(i) + u > settings.H || ...
                        J(i) + v < 1 || J(i) + v > settings.W, continue; end
                count = count + rendered_data(I(i) + u, J(i) + v);
            end
        end
        if (count <= 3), boarder_points(I(i), J(i)) = 1; end
    end
    boundary_indices = bwboundaries(rendered_data);
    boundary_indices = boundary_indices{1};
    previous = [boundary_indices(end, 1); boundary_indices(end, 2)];
    for i = 1:length(boundary_indices)
        current = [boundary_indices(i, 1); boundary_indices(i, 2)];
        if boarder_points(current(1), current(2)) == 0, continue; end
        num_samples = ceil(1.5 * norm(current - previous));
        x = linspace(current(1), previous(1), num_samples);
        y = linspace(current(2), previous(2), num_samples);
        index = sub2ind(size(rendered_data),round(x),round(y));
        rendered_data(index) = 1;
        previous = current;
    end
    rendered_data = imfill(rendered_data);
    
    %% Display results
%     boundary = zeros(size(rendered_data));
%     for i = 1:length(boundary_indices)
%         boundary(boundary_indices(i, 1), boundary_indices(i, 2)) = 1;
%     end
%     rendered_data_and_data = zeros(settings.H, settings.W, 3);
%     rendered_data_and_data(:, :, 2) = boundary;
%     rendered_data_and_data(:, :, 3) = rendered_data;
%     rendered_data_and_data(:, :, 1) = boarder_points;
%     figure; imshow(rendered_data_and_data); drawnow;
end

