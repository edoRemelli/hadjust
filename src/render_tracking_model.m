function [rendered_model, I] = render_tracking_model(centers, blocks, radii, camera_matrix, camera_center, settings)
 H = settings.H; W = settings.W;

D = 3;

%% Put data in matrix form
RAND_MAX = 32767;
R = zeros(length(radii), 1);
C = zeros(length(centers), D);
B = RAND_MAX * ones(length(blocks), 3);
T = RAND_MAX * ones(length(blocks), 6 * D);
tangent_points = blocks_tangent_points(centers, blocks, radii);
for j = 1:length(radii)
    R(j) = radii{j};
    C(j, :) = centers{j}';
end
for j = 1:length(blocks)
    for k = 1:length(blocks{j})
        B(j, k) = blocks{j}(k) - 1;
    end
    if ~isempty(tangent_points{j})
        T(j, 1:3) = tangent_points{j}.v1';
        T(j, 4:6) = tangent_points{j}.v2';
        T(j, 7:9) = tangent_points{j}.v3';
        T(j, 10:12) = tangent_points{j}.u1';
        T(j, 13:15) = tangent_points{j}.u2';
        T(j, 16:18) = tangent_points{j}.u3';
    end
end
M = camera_matrix;
p = camera_center;

[U, V, D, I] = render_model_mex(C, R, B, T, M, W, H, p);

%[U, V, D] = render_model_matlab(centers, blocks, radii, tangent_points, W, H, M, p);


rendered_model = zeros(H, W, 3);
rendered_model(:, :, 1) = U;
rendered_model(:, :, 2) = V;
rendered_model(:, :, 3) = D;




