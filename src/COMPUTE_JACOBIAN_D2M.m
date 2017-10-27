function [F, Jtheta, Jbeta, Jr] = COMPUTE_JACOBIAN_D2M(hand_model, data_points, model_points, indices, block_indices )

%% prepare hand model
centers = hand_model.palm_wrist_centers;
%convert segments into centers
for i = 1:length(hand_model.segments)
    finger_segments =  hand_model.segments{i};
    for j = 1:length(finger_segments)
        centers{end+1} = finger_segments{j}.global(1:3, 4);
    end
end

radii = {};
 for i = 1: length(hand_model.palm_wrist_radii)
    radii{end +1} = hand_model.palm_wrist_radii{i};
 end
 for i = 1: length(hand_model.finger_radii)
    radii{end +1} = hand_model.finger_radii{i};
 end
blocks = hand_model.blocks;

%% setup

num_points = length(data_points);

% TO MODIFY WHEN LOOKING AT PALM AS WELL

F = zeros( num_points, 1);
Jtheta = zeros( num_points, 20);
Jbeta = zeros( num_points, 15);
Jr = zeros( num_points, 20);

%% Compute tangent points
for i = 1:num_points
    
    q = model_points{i};
    p = data_points{i};
    %% what to do with index
    index =  indices{i};
    block_index = block_indices{i};
    
    %check if point belongs to segment, in that case map to segment
    %indices
    [is_on_segment, index_i,index_j, segment_index ] = block_index2segment_id( hand_model,index, block_index );
       
    if isempty(index) || isempty(p) || isempty(q) || ~is_on_segment 
        continue;
    end  
    
    n = (p-q)/norm(p-q);
    
    % take current segment
    
    segment = hand_model.segments{index_i}{index_j};
    
        
    %% COMPUTE JACOBIAN DUE TO THETA
    
    point_jacobian_theta = zeros(3, 20);
    

    % loop over all elements of kinematic chain
    for l = 1:length(segment.kinematic_chain)
        % extract joint
        loop_segment_id = segment.kinematic_chain(l);
        % joint center position
        joint_position = hand_model.segments{index_i}{loop_segment_id}.global(1:3, 4);
        % global transform
        T = hand_model.segments{index_i}{loop_segment_id}.global(1:3, 1:3);

        for k = 1: length(hand_model.segments{index_i}{loop_segment_id}.joints_id)
            joint_id = hand_model.segments{index_i}{loop_segment_id}.joints_id{k};  
            % joint axis
            u = hand_model.joints{index_i}{joint_id}.axis;           
            % transform axis from local to global ref frame
            v = T * u;
        
            switch hand_model.joints{index_i}{joint_id}.type
                case 'R'
                    point_jacobian_theta(:,  joint_id + (index_i-1)* 4) = cross(v, q - joint_position)';
                case 'T'
                    point_jacobian_theta(:, joint_id + (index_i-1)* 4) = v;
            end
        end
    end    
    
    %% COMPUTE JACOBIAN DUE TO BETA
       
    point_jacobian_beta = zeros(3, 15);
    %default orientation for length increment
    u = [0; 0; 1];
 
    for l = 1:length(segment.shape_chain)
        %extract segment id on shape chain
        loop_segment_id = segment.shape_chain(l);
        % segment orientation transform
        T = hand_model.segments{index_i}{loop_segment_id}.global(1:3,1:3);
        %retrieve orientation of segment
        v = T * u;       
        %write in our vector 
        if (index_j ~= loop_segment_id)
            point_jacobian_beta(:, loop_segment_id + (index_i-1)* 3) = v;
        else
            if (length(segment_index) == 2)
                variables = {'c2'};
                c1 = hand_model.segments{index_i}{index_j}.global(1:3, 4);
                c2 = hand_model.segments{index_i}{index_j +1}.global(1:3, 4);
                r1 = hand_model.finger_radii{index_j + 4*(index_i -1)};
                r2 = hand_model.finger_radii{index_j + 1 + 4*(index_i -1)};
                [~,dq] = jacobian_convsegment(q,c1,c2,r1,r2,variables);
                point_jacobian_beta(:, loop_segment_id + (index_i-1)* 3) = dq.dc2 * v;
            elseif(length(segment_index) == 1 && segment_index(1) == (index_j + 1))
               variables = {'c1'};
                c1 = hand_model.segments{index_i}{index_j +1}.global(1:3, 4);
                r1 = hand_model.finger_radii{index_j + 1 + 4*(index_i -1)};
                [~, dq] = jacobian_sphere(q, c1, r1, variables);
                point_jacobian_beta(:, loop_segment_id + (index_i-1)* 3) = dq.dc1 * v; 
            end
        end
    end
           
    %% COMPUTE JACOBIAN DUE TO RADI
    
    if length(index) == 1
        variables = {'r1'};
        [~, dq] = jacobian_sphere(q, centers{12 + segment_index + 4*(index_i -1)}, radii{ 12 + segment_index + 4*(index_i -1)}, variables);
    end
      
    if length(index) == 2
        variables = {'r1', 'r2'};
        [~, dq] = jacobian_convsegment(q, centers{12 + index_j  + 4*(index_i -1)}, centers{12 +index_j + 1 + 4*(index_i -1)}, radii{12 + index_j + 4*(index_i -1)}, radii{12 + index_j + 1 + 4*(index_i -1)}, variables);
    end
    
    %% STORE WHAT HAS BEEN COMPUTED   
    
    F(i) = n'*(p-q);      
    Jr(i, 4*(index_i -1)+ segment_index(1)) = n'*dq.dr1;
    if length(index) == 2
        Jr(i, 4*(index_i -1)+ segment_index(2)) = n'*dq.dr2;
    end
    
    Jtheta(i, :) = n'*point_jacobian_theta;
    Jbeta(i, :) = n'*point_jacobian_beta;
    
end
      