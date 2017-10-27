function [F, Jtheta, Jbeta, Jr,Jcenters_fingers, Jr_palm, Jcenters_palm, Js_membrane, Jglobal_rotation,Jglobal_translation,Joffset_fold, Jr_fold] = COMPUTE_JACOBIAN_M2D(hand_model, data_points, settings,display )

%% prepare hand model
centers = hand_model.palm_wrist_centers;
centers{end+1} = hand_model.fold_center;
%convert segments into centers
for i = 1:length(hand_model.segments)
    finger_segments =  hand_model.segments{i};
    for j = 1:length(finger_segments)
        centers{end+1} = finger_segments{j}.global(1:3, 4);
    end
end

for i = 1:length(hand_model.membrane_centers)
    centers{end+1} = hand_model.membrane_centers{i}; 
end

radii = {};
 for i = 1: length(hand_model.palm_wrist_radii)
    radii{end +1} = hand_model.palm_wrist_radii{i};
 end
radii{end+1} = hand_model.fold_radii;
 
 for i = 1: length(hand_model.finger_radii)
    radii{end +1} = hand_model.finger_radii{i};
 end
 for i = 1: length(hand_model.membrane_radii)
    radii{end +1} = hand_model.membrane_radii{i};
 end
blocks = hand_model.blocks;


% compute data bounding box

data_bounding_box  = compute_data_bounding_box(data_points);


%% Render model and data

% paramters necessary for rendering data e model
closing_radius = 2;
dialation_radius = 1;

[raytracing_matrix, camera_axis, camera_center] = ...
    get_raytracing_matrix(centers, radii, data_bounding_box, settings.view_axis, settings, settings.side);

[rendered_data, back_map_for_rendered_data, P] = ...
    render_tracking_data(data_points, camera_axis, camera_center, settings.view_axis, closing_radius, dialation_radius, settings);

rendered_model = render_tracking_model(centers, blocks, radii, raytracing_matrix, camera_center, settings);

% extract only indices of the rendered model making sense and where there is no rendered data,
% hence the distance of the model from the data is different than zero 
[I, J] = find((rendered_model(:, :, 3) > - settings.RAND_MAX) & (rendered_data == 0));
% they are going to be the model points 
model_points = cell(length(I), 1);
for k = 1:length(I)
    model_points{k} = squeeze(rendered_model(I(k), J(k), :));
end

% determine to which segment rendered model points belong
[indices, ~, block_indices] = compute_projections(model_points, hand_model);

[model_normals] = compute_model_normals(centers, radii, blocks, model_points, indices);

% get world coordinates of closest data point in pc, data_point and model
% point in screen uv coordinates - used only for displaying results
[closest_data_points, model_points_2D, data_points_2D] = my_find_silhouette_constraints(model_points, back_map_for_rendered_data, rendered_data, P, settings.view_axis);

%% Compute camera ray normals
camera_ray_normals = cell(length(model_points), 1);
for i = 1:length(model_points)
    m = model_points{i};
    d =  closest_data_points{i};
    if isempty(m) || isempty(d), continue; end
    q = project_point_on_line(m, d, camera_center);
    camera_ray_normals{i} = (q - m) / norm(q - m);
end

%% Compute Jacobian
[F, Jtheta, Jbeta, Jr ,Jcenters_fingers, Jr_palm, Jcenters_palm, Js_membrane, Jglobal_rotation,Jglobal_translation,Jr_fold,Joffset_fold] = jacobian_realsense(hand_model, closest_data_points, model_points, indices, block_indices);

% project along camera ray normal
[F, Jtheta, Jbeta, Jr, Jcenters_fingers, Jcenters_palm, Jr_palm, Js_membrane, Jglobal_rotation,Jglobal_translation,Jr_fold,Joffset_fold] = compute_normal_distance(camera_ray_normals, model_normals, F, Jtheta, Jbeta, Jr, Jcenters_fingers, Jcenters_palm,Jr_palm,Js_membrane, Jglobal_rotation,Jglobal_translation,Jr_fold,Joffset_fold, settings.D);

%% Display 
if (display)  
    rendered_intersection = zeros(size(rendered_model));
    rendered_intersection(:, :, 3) = (rendered_model(:, :, 3) > -settings.RAND_MAX);
    rendered_intersection(:, :, 1) = rendered_data;
    figure; imshow(rendered_intersection); hold on;
    mypoints(model_points_2D, [0, 0.7, 1]);
    mypoints(data_points_2D, [1, 0.7, 0.1]);      
end