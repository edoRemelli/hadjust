function [F, Jtheta, Jbeta, Jr, Jcenters_fingers, Jcenters_palm, Jr_palm] = COMPUTE_FULL_JACOBIAN_D2M(hand_model, data_points, model_points, indices, block_indices )

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
D = 3;

%discard back facing points
model_normals = compute_model_normals(centers, radii, blocks, data_points, indices);
camera_ray = [0; 1; 0];
for i = 1:length(model_points)
    if isempty(model_normals{i}), continue; end
    if camera_ray' * model_normals{i} > 0
        model_points{i} = [];
    end
end


%% Compute tangent points to wedges
[tangent_gradients] = jacobian_tangent_planes(centers, blocks, radii, {'c1', 'c2', 'c3', 'r1', 'r2', 'r3'});

%% setup

num_points = length(data_points);

% prepare matrices & vectors for optimization
F = zeros( num_points, 1);
Jtheta = zeros( num_points, 20);
Jbeta = zeros( num_points, 15);
Jr = zeros( num_points, 20);
Jcenters_fingers = zeros( num_points, 15 );

Jr_palm = zeros( num_points, length(hand_model.palm_wrist_radii) );
Jcenters_palm = zeros( num_points, 3*length(hand_model.palm_wrist_radii) );

%% Compute tangent points
for i = 1:num_points
    
    q = model_points{i};
    p = data_points{i};
    index =  indices{i};
    block_index = block_indices{i};   
    tangent_gradient = tangent_gradients{block_index};
    
    
    %check if point belongs to segment, in that case map to segment
    %indices
    [is_on_segment, index_i,index_j, segment_index ] = block_index2segment_id( hand_model,index, block_index );
       
    if isempty(index) || isempty(p) || isempty(q) 
        continue;
    end  
    
    n = (p-q)/norm(p-q);
        
    % check if model point belongs to a segment or not
    if(is_on_segment)
        
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
                    [~,dq] = jacobian_convsegment(p,c1,c2,r1,r2,variables);
                    point_jacobian_beta(:, loop_segment_id + (index_i-1)* 3) = dq.dc2 * v;
                elseif(length(segment_index) == 1 && segment_index(1) == (index_j + 1))
                   variables = {'c1'};
                    c1 = hand_model.segments{index_i}{index_j +1}.global(1:3, 4);
                    r1 = hand_model.finger_radii{index_j + 1 + 4*(index_i -1)};
                    [~, dq] = jacobian_sphere(p, c1, r1, variables);
                    point_jacobian_beta(:, loop_segment_id + (index_i-1)* 3) = dq.dc1 * v; 
                end
            end
        end

        %% COMPUTE JACOBIAN DUE TO RADI

        if length(index) == 1
            variables = {'r1'};
            [~, dq] = jacobian_sphere(p, centers{12 + segment_index + 4*(index_i -1)}, radii{ 12 + segment_index + 4*(index_i -1)}, variables);
        end

        if length(index) == 2
            variables = {'r1', 'r2'};
            [~, dq] = jacobian_convsegment(p, centers{12 + index_j  + 4*(index_i -1)}, centers{12 +index_j + 1 + 4*(index_i -1)}, radii{12 + index_j + 4*(index_i -1)}, radii{12 + index_j + 1 + 4*(index_i -1)}, variables);
        end

        % PROJECT ALONG NORMAL AND STORE WHAT HAS BEEN COMPUTED   
    
        Jr(i, 4*(index_i -1)+ segment_index(1)) = n'*dq.dr1;
        if length(index) == 2
            Jr(i, 4*(index_i -1)+ segment_index(2)) = n'*dq.dr2;
        end

        Jtheta(i, :) = n'*point_jacobian_theta;
        Jbeta(i, :) = n'*point_jacobian_beta;
        
        Jcenters_fingers(i, (3*(index_i -1) +1) : 3*index_i ) = n'*eye(3,3);
    else
        
    % then we are working with the old parametrization    
    if length(index) == 1
        variables = {'c1', 'r1'};
        [q, dq] = jacobian_sphere(p, centers{index(1)}, radii{index(1)}, variables);
        
    end
    if length(index) == 2
        variables = {'c1', 'c2', 'r1', 'r2'};
        [q, dq] = jacobian_convsegment(p, centers{index(1)}, centers{index(2)}, radii{index(1)}, radii{index(2)}, variables);
    end
    if length(index) == 3
        variables = {'c1', 'c2', 'c3', 'r1', 'r2', 'r3'};
        v1 = tangent_gradient.v1; v2 = tangent_gradient.v2; v3 = tangent_gradient.v3;
        u1 = tangent_gradient.u1; u2 = tangent_gradient.u2; u3 = tangent_gradient.u3;
        Jv1 = tangent_gradient.Jv1; Jv2 = tangent_gradient.Jv2; Jv3 = tangent_gradient.Jv3;
        Ju1 = tangent_gradient.Ju1; Ju2 = tangent_gradient.Ju2; Ju3 = tangent_gradient.Ju3;
        if (index(1) > 0)
            [~, dq] = jacobian_convtriangle(p, v1, v2, v3, Jv1, Jv2, Jv3, variables);
        else
            [~, dq] = jacobian_convtriangle(p, u1, u2, u3, Ju1, Ju2, Ju3, variables);
        end
    end
        
    % PROJECT ALONG NORMAL AND STORE WHAT HAS BEEN COMPUTED      
    if length(index) == 1
            Jcenters_palm(i, D * index(1) - D + 1:D * index(1)) = n'*dq.dc1;
            Jr_palm(i, index(1)) = n'*dq.dr1;
    end
    if length(index) == 2
            Jcenters_palm(i, D * index(1) - D + 1:D * index(1)) = n'*dq.dc1;
            Jcenters_palm(i, D * index(2) - D + 1:D * index(2)) = n'*dq.dc2;
            Jr_palm(i, index(1)) = n'*dq.dr1;
            Jr_palm(i, index(2)) = n'*dq.dr2;
    end
    if length(index) == 3
            index = abs(index);
            Jcenters_palm(i, D * index(1) - D + 1:D * index(1)) = n'*dq.dc1;
            Jcenters_palm(i, D * index(2) - D + 1:D * index(2)) = n'*dq.dc2;
            Jcenters_palm(i, D * index(3) - D + 1:D * index(3)) = n'*dq.dc3;
            Jr_palm(i, index(1)) = n'*dq.dr1;
            Jr_palm(i, index(2)) = n'*dq.dr2;
            Jr_palm(i, index(3)) = n'*dq.dr3;
    end
         
    end
    
    % PROJECT RHS ALONG NORMAL
    F(i) = n'*(p-q);  
    
end