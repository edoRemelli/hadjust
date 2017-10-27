%% SETUP

clc
clear
close all
addpath('../src/')
addpath('../display/')
addpath('../gradient/')
addpath('../mex/')
addpath('../test/')
addpath('../template/')

data_path = 'data\USER1\';
image_path = '..\output\';

% choose frames from folder
frames = {1,2,3,4,5,6}; 
frames = {1}

% settings
settings.stage1 = true;
settings.stage2 = true;
settings.display = true;
% OPTIMIZATION PARAMETERS
tol = 1e-03;
settings.fov = 15;
settings.H = 480/1;
settings.W = 636/1;
settings.D = 3;
settings.sparse_data = false;
settings.RAND_MAX = 32767;
settings.side = 'front';
settings.view_axis = 'Y';
settings.theta_factor = 0.2;
settings.block_safety_factor = 1.1;
w_bounds = 400;
w_val = 2000;
w_stabilization_theta = 9000;
w_stabilization_radii = 1000;



% re-size template
scale_x = 1.00;
scale_y = 1.00;
scale_z = 1.00;
fat_factor = 1.00;
finger_length_factor = 1.00;
alpha = [scale_x; scale_y; scale_z];

% DEFINE USEFUL TRANSFORMS
Rx = @(alpha) [
    1, 0, 0;
    0, cos(alpha), -sin(alpha);
    0, sin(alpha), cos(alpha)];

Ry = @(alpha) [
    cos(alpha), 0, sin(alpha);
    0, 1, 0;
    -sin(alpha), 0, cos(alpha)];

Rz = @(alpha) [
    cos(alpha), -sin(alpha), 0;
    sin(alpha), cos(alpha), 0
    0, 0, 1];

% transform from my orientation to htrack's one
R = Rx(-pi/2)*Rz(-pi);
% transform from htrack's orientation to mine
Rinv = Rz(pi) * Rx(pi/2);

% LOAD TEMPLATE HAND MODEL
load('hand_model.mat');
[hand_model] = reindex_fullhand(hand_model);

% Scale template model
scaled_hand_model = hand_model;
scaled_hand_model.beta = { finger_length_factor*scaled_hand_model.beta{1}, finger_length_factor*scaled_hand_model.beta{2},  finger_length_factor*scaled_hand_model.beta{3}, finger_length_factor*scaled_hand_model.beta{4}, finger_length_factor*scaled_hand_model.beta{5} };
for j = 1:length(scaled_hand_model.finger_radii)
    scaled_hand_model.finger_radii{j} = fat_factor*scaled_hand_model.finger_radii{j};
end

for j = 1:length(scaled_hand_model.palm_wrist_radii)
    scaled_hand_model.palm_wrist_radii{j} = fat_factor*scaled_hand_model.palm_wrist_radii{j};
    scaled_hand_model.palm_wrist_centers_relative{j} = alpha.*scaled_hand_model.palm_wrist_centers_relative{j};
end
% update centers   
[ scaled_hand_model ] = update_centers(scaled_hand_model);
% perturbate finger centers
delta_finger_center = {(alpha-1).*scaled_hand_model.segments{1}{1}.local(1:3,4), (alpha-1).*scaled_hand_model.segments{2}{1}.local(1:3,4), (alpha-1).*scaled_hand_model.segments{3}{1}.local(1:3,4), (alpha-1).*scaled_hand_model.segments{4}{1}.local(1:3,4), (alpha-1).*scaled_hand_model.segments{5}{1}.local(1:3,4)};
% update shape
[ scaled_hand_model.segments ] = update_fingers_shape(scaled_hand_model.segments, scaled_hand_model.beta );    
delta_theta = {zeros(4,1),zeros(4,1),zeros(4,1),zeros(4,1),zeros(4,1)};
% update model pose
[ scaled_hand_model ] = update_fingers_pose(scaled_hand_model, delta_theta, delta_finger_center );
% update membranes
[ scaled_hand_model ] = update_membranes( scaled_hand_model );  
hand_model = scaled_hand_model;

%% READ htrack INPUT

v_hand_model = {};
v_data_points = {};

for i = 1 : length(frames)

    frame_path = [data_path, num2str(frames{i}) ,'\'];
    %read cpp model
    [centers, radii, blocks, theta, phalanges, mean_centers] = read_cpp_model(frame_path);

    v_hand_model{i} = hand_model;
    %READ INITIAL TRANSFORMATIONS from htrack
    v_hand_model{i}.segments{1}{1}.local(1:3,1:3) = Rinv* phalanges{2}.local(1:3,1:3)*R;
    v_hand_model{i}.segments{1}{2}.local(1:3,1:3) = Rinv* phalanges{3}.local(1:3,1:3)*R;
    v_hand_model{i}.segments{1}{3}.local(1:3,1:3) = Rinv* phalanges{4}.local(1:3,1:3)*R;

    v_hand_model{i}.segments{5}{1}.local(1:3,1:3) = Rinv*  phalanges{5}.local(1:3,1:3)*R;
    v_hand_model{i}.segments{5}{2}.local(1:3,1:3) =  Rinv*  phalanges{6}.local(1:3,1:3)*R;
    v_hand_model{i}.segments{5}{3}.local(1:3,1:3) =  Rinv*  phalanges{7}.local(1:3,1:3)*R;

    v_hand_model{i}.segments{4}{1}.local(1:3,1:3) = Rinv*  phalanges{8}.local(1:3,1:3)*R;
    v_hand_model{i}.segments{4}{2}.local(1:3,1:3) =  Rinv*  phalanges{9}.local(1:3,1:3)*R;
    v_hand_model{i}.segments{4}{3}.local(1:3,1:3) =  Rinv*  phalanges{10}.local(1:3,1:3)*R;

    v_hand_model{i}.segments{3}{1}.local(1:3,1:3) = Rinv*  phalanges{11}.local(1:3,1:3)*R;
    v_hand_model{i}.segments{3}{2}.local(1:3,1:3) =  Rinv*  phalanges{12}.local(1:3,1:3)*R;
    v_hand_model{i}.segments{3}{3}.local(1:3,1:3) =  Rinv*  phalanges{13}.local(1:3,1:3)*R;

    v_hand_model{i}.segments{2}{1}.local(1:3,1:3) = Rinv*  phalanges{14}.local(1:3,1:3)*R;
    v_hand_model{i}.segments{2}{2}.local(1:3,1:3) =  Rinv*  phalanges{15}.local(1:3,1:3)*R;
    v_hand_model{i}.segments{2}{3}.local(1:3,1:3) =  Rinv*  phalanges{16}.local(1:3,1:3)*R;

    % POSE MODEL
    dtheta = { +[-theta(11); +theta(10); -theta(12:13)],[-theta(15); +theta(14); -theta(16:17)],-[+theta(19); -theta(18); +theta(20:21)],-[+theta(23); -theta(22); +theta(24:25)],-[+theta(27); -theta(26);+ theta(28:29)]};
    v_hand_model{i}.global_pose = [Rinv,zeros(3,1),;zeros(1,3),1] * makehgtform('translate',theta(1:3))*makehgtform('axisrotate', [1;0;0], theta(4))*makehgtform('axisrotate', [0;1;0], theta(5))*makehgtform('axisrotate', [0;0;1], theta(6))*[R,zeros(3,1),;zeros(1,3),1];
    v_hand_model{i}.global_translation = v_hand_model{i}.global_pose(1:3,4);
    % this should be improved, but should not really cause any problem like that
    v_hand_model{i}.global_rotation = [-theta(4);theta(5);theta(6)];
    v_hand_model{i}.theta = dtheta;
    pose.global_translation = zeros(3,1);
    pose.global_rotation = zeros(3,1);
    v_hand_model{i} = pose_model(v_hand_model{i}, dtheta, pose);
    

    % READ POINT CLOUD

    % camera parameters
    tx = 640 / 4; ty = 480 / 4; fx = 287.26; fy = 287.26;

    filename = [frame_path, 'depth.png']; D = imread(filename);
    filename = [frame_path, 'mask.png']; M = imread(filename);

    % set depth value of pixels not belonging to mask to zero
    D(M == 0) = 0;
    [U, V] = meshgrid(1:size(D, 2), 1:size(D, 1));
    UVD = zeros(size(D, 1), size(D, 2), 3);
    UVD(:, :, 1) = U;
    UVD(:, :, 2) = V;
    UVD(:, :, 3) = D;
    uvd = reshape(UVD, size(UVD, 1) * size(UVD, 2), 3)';
    I = convert_uvd_to_xyz(tx, ty, fx, fy, uvd);
    data_points = {};
    data_pointsR = {};
    for j = 1:size(I, 2)
        if ~any(isnan(I(:, j)))
            data_points{end + 1} = I(:, j);
        end
    end

    %% FILTER DATA
    depth_image = reshape(I, 3, ty * 2, tx * 2);
    depth_image = shiftdim(depth_image, 1);
    depth = depth_image(:, :, 3);
    max_depth = max(depth(:));
    depth = depth ./ max_depth;
    depth = bfilter2(depth, 5, [2 0.1]);
    depth = depth .* max_depth;
    depth_image(:, :, 3) = depth;
    depth_image = shiftdim(depth_image, 2);
    I2 = reshape(depth_image, 3, ty * 2 * tx * 2);
    data_points = {};
    for j = 1:size(I2, 2)
        if ~any(isnan(I2(:, j)))
            data_points{end + 1} = I2(:, j);
        end
    end
    % ROTATE POINT CLOUD
    for j = 1:length(data_points)
        data_pointsR{end+1} = Rinv*data_points{j};
    end
    % save pc
    v_data_points{i} = data_pointsR;
    v_data_points_4rend{i} = data_points;
end

%% VISUALIZE TO CHECK EVERYTHING IS ALLRIGHT

if(settings.display)
    ds_renderer = 3;
    settings_renderer.fov = 15;
    settings_renderer.H = 480/ds_renderer;
    settings_renderer.W = 636/ds_renderer;
    settings_renderer.D=3;
    settings_renderer.sparse_data = false;
    settings_renderer.RAND_MAX = 32767;
    settings_renderer.side = 'front';
    settings_renderer.view_axis = 'Z';
    v_hand_model_4rend = v_hand_model;
    
    path = [image_path,'init\'];
    for i = 1 : length(frames)
        P.global_rotation = [pi/2;pi;0.0];     
        P.global_translation = zeros(3,1);
        DT = {zeros(4,1),zeros(4,1),zeros(4,1),zeros(4,1),zeros(4,1)};
        [ v_hand_model_4rend{i} ] = pose_model(v_hand_model_4rend{i}, DT,P );
        [ v_hand_model_4rend{i} ] = update_membranes( v_hand_model_4rend{i} );
        img = RENDER_TAN_PC(v_hand_model_4rend{i},v_data_points_4rend{i}, settings_renderer );
        frame_path = [path,num2str(i),'.png'];
        saveas(img,frame_path);
    end
end

for i = 1 :length(frames)
   [v_indices{i}, v_model_points{i}, v_block_indices{i}] = compute_projections(v_data_points{i}, v_hand_model{i});
end



%% REDUCED PARAMETERS OPTIMIZATION

lambda = 100.0;
n_iter = 0;
target_delta = 2.0;
err = tol + 1;



if(settings.stage1)

disp('Optimization over reduced parameters')


while err > tol && n_iter < 20
    
    n_iter = n_iter + 1
    
     % compute Jacobian matrix and right hand side
    J_full = [];
    F_full = [];
    
    for i = 1 :length(frames)
        % compute Jacobian matrix and right hand side
        [F_d2m, Jtheta_d2m, Jbeta_d2m, Jr_d2m, Jcenters_fingers_d2m, Jcenters_palm_d2m, Jr_palm_d2m, Js_membrane_d2m, Jglobal_rotation_d2m, Jglobal_translation_d2m, Joffset_fold_d2m, Jr_fold_d2m] = COMPUTE_D2M_ENERGY(v_hand_model{i}, v_data_points{i}, v_model_points{i}, v_indices{i}, v_block_indices{i},true,false );
        [F_m2d, Jtheta_m2d, Jbeta_m2d, Jr_m2d, Jcenters_fingers_m2d, Jr_palm_m2d, Jcenters_palm_m2d, Js_membrane_m2d, Jglobal_rotation_m2d, Jglobal_translation_m2d, Joffset_fold_m2d, Jr_fold_m2d] = COMPUTE_JACOBIAN_M2D(v_hand_model{i}, v_data_points{i}, settings,false );
        % start by stacking full jacobian
        J = [Jr_d2m, Jr_palm_d2m,Jr_fold_d2m, Jbeta_d2m, Jcenters_palm_d2m, Jcenters_fingers_d2m,zeros(length(F_d2m),26*(i-1)), Jtheta_d2m, Jglobal_rotation_d2m, Jglobal_translation_d2m,zeros(length(F_d2m),26*(length(frames)-i)); Jr_m2d, Jr_palm_m2d,Jr_fold_m2d, Jbeta_m2d, Jcenters_palm_m2d, Jcenters_fingers_m2d,zeros(length(F_m2d),26*(i-1)),Jtheta_m2d, Jglobal_rotation_m2d, Jglobal_translation_m2d,zeros(length(F_m2d),26*(length(frames)-i)) ] ;
        F = [F_d2m;F_m2d];

        param_value = zeros( size(J));

        for j = 1:length(v_hand_model{i}.finger_radii)
            param_value(:,j) = v_hand_model{i}.finger_radii{j};
        end
        l1 = length(v_hand_model{i}.finger_radii);
        for j = 1:length(v_hand_model{i}.palm_wrist_radii)
           param_value(:,l1 + j) = v_hand_model{i}.palm_wrist_radii{j};
        end
        l1 = l1 + length(v_hand_model{i}.palm_wrist_radii);
        absolute_size(:,l1 + 1) = v_hand_model{i}.fold_radii;
        l1 = l1 + 1;        
        param_value(:, l1 + 1: l1 + 1 + length(v_hand_model{i}.beta)*length(v_hand_model{i}.beta{2})) = [ v_hand_model{i}.beta{1}', v_hand_model{i}.beta{2}',  v_hand_model{i}.beta{3}', v_hand_model{i}.beta{4}', v_hand_model{i}.beta{5}' ].* ones(length(F),16);
        l2 = l1 + length(v_hand_model{i}.beta)*length(v_hand_model{i}.beta{2}) + 1;

        for j = 1:length( v_hand_model{i}.palm_wrist_centers_relative)
            for k = 1:3
                param_value(:,l2 + 3*(j-1) + k) = v_hand_model{i}.palm_wrist_centers_relative{j}(k);
            end
        end
        l3 = l2 + 3*length( v_hand_model{i}.palm_wrist_centers_relative);
        for j = 1:length(v_hand_model{i}.segments)
            for k = 1:3
            param_value(:,l3 + 3*(j-1) + k) = v_hand_model{i}.segments{j}{1}.local(k,4);
            end
        end
        l3 = l3 + 3*length(v_hand_model{i}.segments);

        param_value(:,l3+1:end) = 1;

        J = J.*param_value;
        J = [ sum(J(:,1:l1),2), sum(J(:,1+l1:l2),2), sum(J(:,1+l2:3:l3),2), sum(J(:,2+l2:3:l3),2), sum(J(:,3+l2:3:l3),2), J(:,l3+1:end); ];
        
        J_full = [J_full;J];
        F_full = [F_full;F];      
    end
    
    % add stabilization energy
    J_full = [J_full; zeros(length(frames)*26,5),w_stabilization_theta*eye(length(frames)*26,length(frames)*26); w_stabilization_radii,zeros(1,4+26*length(frames))];
    F_full = [F_full; zeros(26*length(frames) +1,1)];   
        
        
    % perform descent step      
    JtJ = J_full' * J_full;
    LHS = JtJ + lambda*eye(size(JtJ));
    delta = LHS \ (J_full' * F_full);

    err = norm(delta)
    lambda = target_delta/err * lambda;


    delta_radii = delta(1);
    delta_beta = delta(2);
    delta_alpha = delta(3:5);
    
    delta_theta = {};
    delta_pose = {};
    
    for i = 1 :length(frames)
        delta_theta{i} = { delta(26*(i-1)+6:26*(i-1)+9), delta(26*(i-1)+10:26*(i-1)+13), delta(26*(i-1)+14:26*(i-1)+17), delta(26*(i-1)+18:26*(i-1)+21), delta(26*(i-1)+22:26*(i-1)+25)};
        delta_pose{i}.global_rotation = delta(26*(i-1)+26:26*(i-1)+28);
        delta_pose{i}.global_translation = delta(26*(i-1)+29:26*(i-1)+31);
    end

    % update
    for i = 1:length(frames)
    
        for j = 1:length(v_hand_model{i}.finger_radii)
            v_hand_model{i}.finger_radii{j} = v_hand_model{i}.finger_radii{j}*(1 + delta_radii);
        end

        for j = 1:length(v_hand_model{i}.palm_wrist_radii)
            v_hand_model{i}.palm_wrist_radii{j} = v_hand_model{i}.palm_wrist_radii{j}*(1 + delta_radii);
            v_hand_model{i}.palm_wrist_centers_relative{j} = v_hand_model{i}.palm_wrist_centers_relative{j}.*(1 + delta_alpha);
        end
        v_hand_model{i}.fold_radii = v_hand_model{i}.fold_radii*(1 + delta_radii);
        v_hand_model{i}.fold_offset = v_hand_model{i}.fold_offset.*(ones(3,1) + delta_alpha);
        v_hand_model{i}.beta = { v_hand_model{i}.beta{1}*(1 + delta_beta), v_hand_model{i}.beta{2}*(1 + delta_beta),  v_hand_model{i}.beta{3}*(1 + delta_beta), v_hand_model{i}.beta{4}*(1 + delta_beta), v_hand_model{i}.beta{5}*(1 + delta_beta) };

        delta_finger_center = {delta_alpha.*v_hand_model{i}.segments{1}{1}.local(1:3,4), delta_alpha.*v_hand_model{i}.segments{2}{1}.local(1:3,4), delta_alpha.*v_hand_model{i}.segments{3}{1}.local(1:3,4), delta_alpha.*v_hand_model{i}.segments{4}{1}.local(1:3,4), delta_alpha.*v_hand_model{i}.segments{5}{1}.local(1:3,4)};

        % update parameters
        for j = 1:length(v_hand_model{i}.theta)
            v_hand_model{i}.theta{j} = v_hand_model{i}.theta{j} + delta_theta{i}{j};
        end

        % update centers
        [ v_hand_model{i} ] = update_centers(v_hand_model{i});
        % update shape
        [ v_hand_model{i}.segments ] = update_fingers_shape(v_hand_model{i}.segments, v_hand_model{i}.beta ); 

        % update model pose
        [ v_hand_model{i} ] = update_fingers_pose(v_hand_model{i}, delta_theta{i}, delta_finger_center );
        delta_theta{i} = {zeros(4,1), zeros(4,1), zeros(4,1), zeros(4,1), zeros(4,1)};
        [ v_hand_model{i} ] = pose_model(v_hand_model{i}, delta_theta{i},delta_pose{i} );

        % update membranes
        [ v_hand_model{i} ] = update_membranes( v_hand_model{i} );   

        % compute updated correspondencies
        [v_indices{i}, v_model_points{i}, v_block_indices{i}] = compute_projections(v_data_points{i}, v_hand_model{i});      
    end   
    if(settings.display)
            v_hand_model_4rend = v_hand_model;
            path = [image_path,num2str(n_iter),'\'];
            for i = 1 : length(frames)
                P.global_rotation = [pi/2;pi;0.0];     
                P.global_translation = zeros(3,1);
                DT = {zeros(4,1),zeros(4,1),zeros(4,1),zeros(4,1),zeros(4,1)};
                [ v_hand_model_4rend{i} ] = pose_model(v_hand_model_4rend{i}, DT,P );
                [ v_hand_model_4rend{i} ] = update_membranes( v_hand_model_4rend{i} );
                img = RENDER_TAN_PC(v_hand_model_4rend{i},v_data_points_4rend{i}, settings_renderer );
                frame_path = [path,num2str(i),'.png'];
                saveas(img,frame_path);
            end  
    end        
end

 save('calib.mat','v_hand_model')
 save('phalanges.mat','phalanges')
 save('data_path.mat','data_path');
end
% %% OPTIMIZE OVER THUMB FOLD
% 
% lambda = 100.0;
% n_iter = 0;
% target_delta = 1.0;
% err = tol + 1;
% 
% disp('Fold optimization')
% 
% while err > tol && n_iter < 5
%     
%     n_iter = n_iter + 1
%     
%     compute Jacobian matrix and right hand side
%     J_full = [];
%     F_full = [];
%     
%     for i = 1:length(frames)
%     
%         compute Jacobian matrix and right hand side
%         [F_d2m, Jtheta_d2m, Jbeta_d2m, Jr_d2m, Jcenters_fingers_d2m, Jcenters_palm_d2m, Jr_palm_d2m, Js_membrane_d2m, Jglobal_rotation_d2m, Jglobal_translation_d2m, Joffset_fold_d2m, Jr_fold_d2m] = COMPUTE_D2M_ENERGY(v_hand_model{i}, v_data_points{i}, v_model_points{i}, v_indices{i}, v_block_indices{i},true,false );
%         [F_m2d, Jtheta_m2d, Jbeta_m2d, Jr_m2d, Jcenters_fingers_m2d, Jr_palm_m2d, Jcenters_palm_m2d, Js_membrane_m2d, Jglobal_rotation_m2d, Jglobal_translation_m2d, Joffset_fold_m2d, Jr_fold_m2d] = COMPUTE_JACOBIAN_M2D(v_hand_model{i}, v_data_points{i}, settings,false );
%         
%         assemble matrix
%         J = [Jbeta_d2m, zeros(length(F_d2m),26*(i-1)), Jtheta_d2m, Jglobal_rotation_d2m, Jglobal_translation_d2m,zeros(length(F_d2m),26*(length(frames)-i)),Joffset_fold_d2m; Jbeta_m2d,zeros(length(F_m2d),26*(i-1)), Jtheta_m2d, Jglobal_rotation_m2d, Jglobal_translation_m2d,zeros(length(F_m2d),26*(length(frames)-i)),Joffset_fold_m2d] ;
%         F = [F_d2m;F_m2d;];
%         constrain to 2 dof centers
%                
%         store in full matrix
%         J_full = [J_full;J];
%         F_full = [F_full;F];       
%     end
%     
%     add stabilization energy
%     J_full = [J_full; zeros(length(frames)*26,16),w_stabilization_theta*eye(length(frames)*26,length(frames)*26),zeros(length(frames)*26,3)];
%     F_full = [F_full; zeros(26*length(frames),1)];   
%                 
%     perform descent step      
%     JtJ = J_full' * J_full;
%     LHS = JtJ + lambda*eye(size(JtJ));
%     delta = LHS \ (J_full' * F_full);
%         
%     err = norm(delta)
%     lambda = err/target_delta * lambda;
%        
%     
%     delta_theta = {};
%     delta_pose = {};
%     for i = 1 :length(frames)        
%         delta_theta{i} = { delta(26*(i-1)+17:26*(i-1)+20), delta(26*(i-1)+21:26*(i-1)+24), delta(26*(i-1)+25:26*(i-1)+28), delta(26*(i-1)+29:26*(i-1)+32), delta(26*(i-1)+33:26*(i-1)+36)};
%         delta_pose{i}.global_rotation = delta(26*(i-1)+37:26*(i-1)+39);
%         delta_pose{i}.global_translation = delta(26*(i-1)+40:26*(i-1)+42);
%     end
%     delta_fold_offset = delta(26*(length(frames)-1)+43:26*(length(frames)-1)+45)
%     
%     update stuff
%     for i = 1 :length(frames)        
%          v_hand_model{i}.beta = { v_hand_model{i}.beta{1} + delta(1:4), v_hand_model{i}.beta{2} + delta(5:7),  v_hand_model{i}.beta{3} + delta(8:10), v_hand_model{i}.beta{4} + delta(11:13), v_hand_model{i}.beta{5} + delta(14:16) };
%         update parameters
%         for j = 1:length(v_hand_model{i}.theta)
%             v_hand_model{i}.theta{j} = v_hand_model{i}.theta{j} + delta_theta{i}{j};
%         end
%         
%         v_hand_model{i}.fold_offset = v_hand_model{i}.fold_offset + delta_fold_offset;
% 
%         delta_finger_center = {zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1)};
%         update centers
%         [ v_hand_model{i} ] = update_centers(v_hand_model{i});
%         update shape
%         [ v_hand_model{i}.segments ] = update_fingers_shape(v_hand_model{i}.segments, v_hand_model{i}.beta );    
%         update model pose
%         [ v_hand_model{i} ] = update_fingers_pose(v_hand_model{i}, delta_theta{i}, delta_finger_center );
%         delta_theta{i}= {zeros(4,1), zeros(4,1), zeros(4,1), zeros(4,1), zeros(4,1)};
%         [ v_hand_model{i} ] = pose_model(v_hand_model{i}, delta_theta{i},delta_pose{i} );
%         update membranes
%         [ v_hand_model{i} ] = update_membranes( v_hand_model{i} );  
%         [v_hand_model{i}] = reindex_fullhand(v_hand_model{i});
%         compute updated correspondencies
%         [v_indices{i}, v_model_points{i}, v_block_indices{i}] = compute_projections(v_data_points{i}, v_hand_model{i});
%     end    
% end
% 
% 

%% FULL OPTIMIZATION OVER CENTERS

if(settings.stage2)
    
    lambda = 300.0;
    target_delta = 3.5;
    err = tol + 1;
    w_stabilization_radii = w_stabilization_radii/2;
    w = 1;

    bound_factor = 1.1;
    weight = 800;
    settings.fov = 15;
    settings.H = 480;
    settings.W = 636;
    settings.D = 3;
    settings.sparse_data = false;
    settings.RAND_MAX = 32767;
    settings.side = 'front';
    settings.view_axis = 'Y';

    disp('Full optimization')

    while err > tol && n_iter <20



        n_iter = n_iter + 1

        % compute Jacobian matrix and right hand side
        J_full = [];
        F_full = [];

        for i = 1:length(frames)

            % compute Jacobian matrix and right hand side
            [F_d2m, Jtheta_d2m, Jbeta_d2m, Jr_d2m, Jcenters_fingers_d2m, Jcenters_palm_d2m, Jr_palm_d2m, Js_membrane_d2m, Jglobal_rotation_d2m, Jglobal_translation_d2m, Joffset_fold_d2m, Jr_fold_d2m] = COMPUTE_D2M_ENERGY(v_hand_model{i}, v_data_points{i}, v_model_points{i}, v_indices{i}, v_block_indices{i},true,false );
            [F_m2d, Jtheta_m2d, Jbeta_m2d, Jr_m2d, Jcenters_fingers_m2d, Jr_palm_m2d, Jcenters_palm_m2d, Js_membrane_m2d, Jglobal_rotation_m2d, Jglobal_translation_m2d, Joffset_fold_m2d, Jr_fold_m2d] = COMPUTE_JACOBIAN_M2D(v_hand_model{i}, v_data_points{i}, settings,false );

            J = [Jbeta_d2m, Jr_d2m, Jcenters_palm_d2m,Joffset_fold_d2m, Jr_palm_d2m,Jr_fold_d2m, Jcenters_fingers_d2m, zeros(length(F_d2m),26*(i-1)), Jtheta_d2m, Jglobal_rotation_d2m, Jglobal_translation_d2m,zeros(length(F_d2m),26*(length(frames)-i)); w*[Jbeta_m2d, Jr_m2d, Jcenters_palm_m2d,Joffset_fold_m2d, Jr_palm_m2d,Jr_fold_m2d, Jcenters_fingers_m2d, zeros(length(F_m2d),26*(i-1)), Jtheta_m2d, Jglobal_rotation_m2d, Jglobal_translation_m2d,zeros(length(F_m2d),26*(length(frames)-i))]] ;
            F = [F_d2m;w*F_m2d;];

            % store in full matrix
            J_full = [J_full;J];
            F_full = [F_full;F];       
        end

        % add stabilization energies
        [F_rb,Jc_f_rb,Jc_p_rb] = COMPUTE_REALISTIC_BOUNDS_ENERGY(v_hand_model{1}, bound_factor ,weight);

        J_full = [J_full; zeros(length(frames)*26,104),w_stabilization_theta*eye(length(frames)*26,length(frames)*26); w_stabilization_radii*[zeros(21,16),eye(21,21),zeros(21,67),zeros(21,length(frames)*26)];[zeros(4,16),zeros(4,21),Jc_p_rb,zeros(4,13),Jc_f_rb, zeros(4,length(frames)*26)] ];
        F_full = [F_full; zeros(26*length(frames),1); zeros(21,1);F_rb];  

        % perform descent step      
        JtJ = J_full' * J_full;
        LHS = JtJ + lambda*eye(size(JtJ));
        delta = LHS \ (J_full' * F_full);

        err = norm(delta);
        lambda = err/target_delta * lambda;

        delta_radii = delta(17:37);
        delta_centers = delta (38: 73);
        delta_fold_offset = delta (74: 76);
        delta_radii_palm = delta(77 : 88);
        delta_fold_radii = delta (89);
        delta_finger_center_vec = delta( 90 : 104 );

        delta_theta = {};
        delta_pose = {};
        for i = 1 :length(frames)

            delta_theta{i} = { delta(26*(i-1)+105:26*(i-1)+108), delta(26*(i-1)+109:26*(i-1)+112), delta(26*(i-1)+113:26*(i-1)+116), delta(26*(i-1)+117:26*(i-1)+120), delta(26*(i-1)+121:26*(i-1)+124)};
            delta_pose{i}.global_rotation = delta(26*(i-1)+125:26*(i-1)+127);
            delta_pose{i}.global_translation = delta(26*(i-1)+128:26*(i-1)+130);
        end

        %update stuff
        for i = 1 :length(frames)

             v_hand_model{i}.beta = { v_hand_model{i}.beta{1} + delta(1:4), v_hand_model{i}.beta{2} + delta(5:7),  v_hand_model{i}.beta{3} + delta(8:10), v_hand_model{i}.beta{4} + delta(11:13), v_hand_model{i}.beta{5} + delta(14:16) };
            % update parameters
            for j = 1:length(v_hand_model{i}.theta)
                v_hand_model{i}.theta{j} = v_hand_model{i}.theta{j} + delta_theta{i}{j};
            end

            for j = 1:length(v_hand_model{i}.finger_radii)
                v_hand_model{i}.finger_radii{j} = max(v_hand_model{i}.finger_radii{j} + delta_radii(j),0.1);
            end

            indices_ok = [v_hand_model{i}.palm_wrist_names_map('wrist_bottom_left'),v_hand_model{i}.palm_wrist_names_map( 'wrist_bottom_right'),v_hand_model{i}.palm_wrist_names_map('wrist_top_left'),v_hand_model{i}.palm_wrist_names_map( 'wrist_top_right')];

            for j = 1:length(v_hand_model{i}.palm_wrist_radii)
                if ~ismember(j,indices_ok)
                    v_hand_model{i}.palm_wrist_radii{j} = v_hand_model{i}.palm_wrist_radii{j} + delta_radii_palm(j);
                    v_hand_model{i}.palm_wrist_centers_relative{j} = v_hand_model{i}.palm_wrist_centers_relative{j} +  delta_centers( (3*(j-1) +1) : 3*j);
                end
            end

            v_hand_model{i}.fold_offset = v_hand_model{i}.fold_offset + delta_fold_offset;
            delta_finger_center = {delta_finger_center_vec(1:3), delta_finger_center_vec(4:6), delta_finger_center_vec(7:9), delta_finger_center_vec(10:12), delta_finger_center_vec(13:15)};

            % update centers
            [ v_hand_model{i} ] = update_centers(v_hand_model{i});
            % update shape
            [ v_hand_model{i}.segments ] = update_fingers_shape(v_hand_model{i}.segments, v_hand_model{i}.beta );    
            % update model pose
            [ v_hand_model{i} ] = update_fingers_pose(v_hand_model{i}, delta_theta{i}, delta_finger_center );
            delta_theta{i}= {zeros(4,1), zeros(4,1), zeros(4,1), zeros(4,1), zeros(4,1)};
            [ v_hand_model{i} ] = pose_model(v_hand_model{i}, delta_theta{i},delta_pose{i} );
            % update membranes
            [ v_hand_model{i} ] = update_membranes( v_hand_model{i} );  
            [v_hand_model{i}] = reindex_fullhand(v_hand_model{i});
            % compute updated correspondencies
            [v_indices{i}, v_model_points{i}, v_block_indices{i}] = compute_projections(v_data_points{i}, v_hand_model{i});
        end  
        w_stabilization_theta = w_stabilization_theta/1.1;
                
        if(settings.display)
            v_hand_model_4rend = v_hand_model;
            path = [image_path,num2str(n_iter),'\'];
            for i = 1 : length(frames)
                P.global_rotation = [pi/2;pi;0.0];     
                P.global_translation = zeros(3,1);
                DT = {zeros(4,1),zeros(4,1),zeros(4,1),zeros(4,1),zeros(4,1)};
                [ v_hand_model_4rend{i} ] = pose_model(v_hand_model_4rend{i}, DT,P );
                [ v_hand_model_4rend{i} ] = update_membranes( v_hand_model_4rend{i} );
                img = RENDER_TAN_PC(v_hand_model_4rend{i},v_data_points_4rend{i}, settings_renderer );
                frame_path = [path,num2str(i),'.png'];
                saveas(img,frame_path);
            end
        end
    end   
end

%% SAVE STUFF

% save for storing later
 save('calib.mat','v_hand_model')
 save('phalanges.mat','phalanges')
 save('data_path.mat','data_path');
