function [img] = RENDER_TAN_PC(hand_model,data_points, settings )

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
blocks = hand_model.blocks(3:end);


% compute data bounding box
bounding_box  = compute_data_bounding_box(data_points);


% paramters necessary for rendering data e model- maybe ask Anastasia?
closing_radius = 2;
dialation_radius = 1;

[raytracing_matrix, camera_axis, camera_center] = ...
    get_raytracing_matrix(centers, radii, bounding_box, settings.view_axis, settings, settings.side);

[rendered_data, back_map_for_rendered_data, P] = ...
    render_tracking_data(data_points, camera_axis, camera_center, settings.view_axis, closing_radius, dialation_radius, settings);

% [uvd] = convert_xyz_to_uvd(tx, ty, fx, fy, xyz)
% for i = 1 : size(rendered_data,1)
%     for j = 1: size(rendered_data,2)
%         if(rendered
%     end
% end

rendered_model = render_tracking_model(centers, blocks, radii, raytracing_matrix, camera_center, settings);

% rendered_model(:, :, 3) is the depth 

% extract only indices of the rendered model making sense
[I, J] = find((rendered_model(:, :, 3) > - settings.RAND_MAX));
% they are going to be the model points 
model_points = cell(length(I), 1);
for k = 1:length(I)
    model_points{k} = squeeze(rendered_model(I(k), J(k), :));
end

% determine to which segment rendered model points belong

%[indices, ~, block_indices] = compute_projections(model_points, hand_model);
%[model_normals] = compute_model_normals(centers, radii, blocks, model_points, indices);

% get world coordinates of closest data point in pc, data_point and model
% point in screen uv coordinates - used only for displaying results
%[closest_data_points, model_points_2D, data_points_2D] = my_find_silhouette_constraints(model_points, back_map_for_rendered_data, rendered_data, P, settings.view_axis);

% depth map associated to hand model
D_hm = zeros(size(rendered_model(:, :, 3)));
D_hm = (rendered_model(:, :, 3) > -settings.RAND_MAX).*rendered_model(:, :, 3);
%not sure
D_pc = back_map_for_rendered_data(:,:,3);

%compute position of model wrt depth data
D_hm = flipud(D_hm);
D_pc = flipud(D_pc);
rendered_data = flipud(rendered_data);
%delta_depth = D_hm - tgt_D_hm;
delta_depth = D_hm - D_pc;

factor = max(max(abs(delta_depth)));
rendered_model = flipud(rendered_model);
rendered_image = 255*ones(size(rendered_model));
nn = 0.6;

%tgt_rendered_model(i, j, 3) == -settings.RAND_MAX

for i = 1 : size(delta_depth,1)
    for j = 1: size(delta_depth,2)
        if (delta_depth(i,j)>0 && (rendered_model(i, j, 3) == -settings.RAND_MAX || rendered_data(i,j) == 0 ))
           %rendered_image(i,j,1) = delta_depth(i,j);
           rendered_image(i,j,1) = 255;
           rendered_image(i,j,2) = 0;
           rendered_image(i,j,3) = 0;
        elseif (delta_depth(i,j)>0)
           rendered_image(i,j,1) = 255;
           rendered_image(i,j,2) = 255-255*(delta_depth(i,j)/factor)^nn;
           rendered_image(i,j,3) = 255-255*(delta_depth(i,j)/factor)^nn;
        elseif (delta_depth(i,j)<0 && (rendered_model(i, j, 3) == -settings.RAND_MAX || rendered_data(i,j) == 0 ))
            rendered_image(i,j,1) = 0;
            rendered_image(i,j,2) = 0;
            %rendered_image(i,j,3) = abs(delta_depth(i,j));
            rendered_image(i,j,3) = 255.0;
        elseif (delta_depth(i,j)<0 )
            rendered_image(i,j,1) = 255-255*(abs(delta_depth(i,j))/factor)^nn;
            rendered_image(i,j,2) = 255-255*(abs(delta_depth(i,j))/factor)^nn;
            %rendered_image(i,j,3) = abs(delta_depth(i,j));
            rendered_image(i,j,3) = 255.0;
        %elseif (delta_depth(i,j) == 0 && D_hm(i,j)==0 )
        elseif (delta_depth(i,j) == 0 && rendered_model(i, j, 3) == -settings.RAND_MAX )
            rendered_image(i,j,:)=170;
        end
    end
end

figure; img = imshow(rendered_image/255.0);