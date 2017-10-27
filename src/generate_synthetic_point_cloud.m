function [data_points] = generate_synthetic_point_cloud(hand_model, downscaling_factor,view_axis, sigma_noise)

D = 3; 
settings.fov = 15;
settings.H = floor(480/downscaling_factor);
settings.W = floor(636/downscaling_factor);
settings.D = D;
settings.sparse_data = false;
settings.RAND_MAX = 32767;
settings.side = 'front';
settings.view_axis = view_axis;

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



%% Render the data
data_bounding_box = compute_model_bounding_box(centers, radii);

[raytracing_matrix, ~, camera_center] = get_raytracing_matrix(centers, radii, data_bounding_box, settings.view_axis, settings, settings.side);
[rendered_model, I] = render_tracking_model(centers, blocks, radii, raytracing_matrix, camera_center, settings);

[I, J] = find((rendered_model(:, :, 3) > - settings.RAND_MAX));

data_points  = cell(length(I), 1);
for k = 1:length(I)
    data_points{k} = squeeze(rendered_model(I(k), J(k), :)) + sigma_noise * ( 2*rand(3,1)-1);
end