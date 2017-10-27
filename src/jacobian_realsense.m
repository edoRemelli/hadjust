function [F, Jtheta, Jbeta, Jr, Jcenters_fingers, Jr_palm, Jcenters_palm, Js_membrane, Jglobal_rotation,Jglobal_translation,Jr_fold,Joffset_fold  ] = jacobian_realsense(hand_model, data_points, model_points, indices, block_indices)

% prepare hand model
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

D = length(centers{1});

%% setup

num_points = length(data_points);
% prepare matrices & vectors for optimization
F = zeros(D * num_points, 1);
Jtheta = zeros(D * num_points, 20);
Jbeta = zeros(D * num_points, 16);
Jr = zeros(D * num_points, 21);
Jr_palm = zeros(D * num_points, length(hand_model.palm_wrist_radii) );
Jcenters_palm = zeros(D * num_points, 3*length(hand_model.palm_wrist_radii) );
Jcenters_fingers = zeros(D * num_points, 15 );
Js_membrane = zeros( D*num_points, 4 );
Jglobal_rotation = zeros( D*num_points, 3 );
Jglobal_translation = zeros( D*num_points, 3 );
Jr_fold = zeros(D*num_points, 1);
Joffset_fold = zeros(D*num_points, 3);

%% Compute tangent points
[tangent_gradients] = jacobian_tangent_planes(centers, blocks, radii, {'c1', 'c2', 'c3', 'r1', 'r2', 'r3'});

for i = 1:num_points   
    q = model_points{i};
    p = data_points{i};
    index =  indices{i};
    block_index = block_indices{i};   
    tangent_gradient = tangent_gradients{block_index};
    
    bool_degenerate = false;
    if (~isempty(tangent_gradient))
        bool_degenerate = ~isreal(tangent_gradient.u1);
    end
    if isempty(index) || isempty(p) || isempty(q) || bool_degenerate
        continue;
    end  
    
    %check if point belongs to segment, in that case map to segment
    %indices
    [is_on_segment, index_i,index_j, segment_index ] = block_index2segment_id( hand_model,index, block_index );
    
    %check if point belongs to membrane, in that case map to membrane
    %indices
    [bool_is_on_membrane, is_membrane_center, membrane_index ] = is_on_membrane( hand_model,index, block_index ) ;
    
    thumb_correction = 0;
    if (index_i > 1)
       thumb_correction = 1;
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

        point_jacobian_beta = zeros(3, 16);
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
                point_jacobian_beta(:,thumb_correction +  loop_segment_id + (index_i-1)* 3) = v;
            else
                if (length(segment_index) == 2)
                    variables = {'c2'};
                    c1 = hand_model.segments{index_i}{index_j}.global(1:3, 4);
                    c2 = hand_model.segments{index_i}{index_j +1}.global(1:3, 4);
                    r1 = hand_model.finger_radii{thumb_correction+ index_j + 4*(index_i -1)};
                    r2 = hand_model.finger_radii{thumb_correction+index_j + 1 + 4*(index_i -1)};
                    [~,dq] = jacobian_convsegment(q,c1,c2,r1,r2,variables);
                    point_jacobian_beta(:,thumb_correction+ loop_segment_id + (index_i-1)* 3) = dq.dc2 * v;
                elseif(length(segment_index) == 1 && segment_index(1) == (index_j + 1))
                   variables = {'c1'};
                    c1 = hand_model.segments{index_i}{index_j +1}.global(1:3, 4);
                    r1 = hand_model.finger_radii{thumb_correction + index_j + 1 + 4*(index_i -1)};
                    [~, dq] = jacobian_sphere(q, c1, r1, variables);
                    point_jacobian_beta(:, thumb_correction+ loop_segment_id + (index_i-1)* 3) = dq.dc1 * v; 
                end
            end
        end

        %% COMPUTE JACOBIAN DUE TO RADI

        if length(index) == 1
            variables = {'r1'};
            [~, dq] = jacobian_sphere(q, centers{13 + segment_index + 4*(index_i -1)+thumb_correction}, radii{ 13 + segment_index + 4*(index_i -1)+thumb_correction}, variables);
        end

        if length(index) == 2
            variables = {'r1', 'r2'};
            [~, dq] = jacobian_convsegment(q, centers{13 + index_j  + 4*(index_i -1)+thumb_correction}, centers{13 +index_j + 1 + 4*(index_i -1)+thumb_correction}, radii{13 + index_j + 4*(index_i -1)+thumb_correction}, radii{13 + index_j + 1 + 4*(index_i -1)+thumb_correction}, variables);
        end

        % PROJECT ALONG NORMAL AND STORE WHAT HAS BEEN COMPUTED   
    
        Jr(D * i - D + 1:D * i,thumb_correction+ 4*(index_i -1)+ segment_index(1)) = dq.dr1;
        if length(index) == 2
            Jr(D * i - D + 1:D * i,thumb_correction+ 4*(index_i -1)+ segment_index(2)) = dq.dr2;
        end

        Jtheta(D * i - D + 1:D * i, :) = point_jacobian_theta;
        Jbeta(D * i - D + 1:D * i, :) = point_jacobian_beta;
        
        
        T =  hand_model.global_pose;
        T = T(1:3,1:3);
        
        Jcenters_fingers(D * i - D + 1:D * i, (3*(index_i -1) +1) : 3*index_i ) = T*eye(3,3);
                       
    
    elseif(bool_is_on_membrane)
            indices_for_centers = [hand_model.palm_wrist_names_map('palm_index'), hand_model.palm_wrist_names_map('palm_middle'), hand_model.palm_wrist_names_map('palm_ring'), hand_model.palm_wrist_names_map('palm_pinky')];

                T = hand_model.global_pose;
                T = T(1:3,1:3);
                
                % determine left and right finger
                [ ~,  ~, full_membrane_index ] = is_on_membrane( hand_model,blocks{block_index}, block_index ) ;
                left_finger_index = min(full_membrane_index(3),min(full_membrane_index(2),full_membrane_index(1)));
                right_finger_index = max(full_membrane_index(3),max(full_membrane_index(2),full_membrane_index(1)));    
                % PREPARE PARAMS TO OCOMPUTE MEMBRANE DERIVATIVE
                c1 = hand_model.segments{left_finger_index}{1}.global(1:3,4);
                c2 = hand_model.segments{right_finger_index}{1}.global(1:3,4);
            
                beta1 = hand_model.beta{left_finger_index}(1);
                beta2 = hand_model.beta{right_finger_index}(1);
                R1 = hand_model.segments{left_finger_index}{1}.global(1:3,1:3);
                R2 = hand_model.segments{right_finger_index}{1}.global(1:3,1:3);
                r11 = hand_model.finger_radii{1 + 4*( left_finger_index - 1) + 1};
                r12 = hand_model.finger_radii{1 + 4*( left_finger_index - 1) + 2};
                r21 = hand_model.finger_radii{1 + 4*( right_finger_index - 1) + 1};
                r22 = hand_model.finger_radii{1 + 4*( right_finger_index - 1) + 2};
                s1 = hand_model.membrane_position{left_finger_index-1};
                s2 = hand_model.membrane_position{right_finger_index-1};        

                variables = {'c1b','c2b','c1','c2','beta1','beta2','r1','r2','r11','r12','r21','r22','s1','s2','theta11', 'theta12', 'theta21', 'theta22'};

                [dc1b, dc2b,m_dc1, m_dc2, m_dcm1, m_dcm2, m_dc1_r, m_dc2_r,m_dc1b_r, m_dc2b_r, m_dcm1_r, m_dcm2_r ] = jacobian_membrane(hand_model.global_pose(1:3,1:3), c1,c2, beta1, beta2, R1, R2, r11, r12, r21, r22, s1, s2, variables);
        

        % IDENTIFY LEFT AND RIGHT FINGER WE ARE DEALING WITH    
        if (length(index) == 3)
               
            %% FIGURE OUT WHICH MEMBRANE TRIANGLE TYPE ARE WE LOOKING AT 

             if (sum(is_membrane_center) == 2)
                dc1 = m_dc1;
                dr1 = m_dc1_r;
                if membrane_index(1) == membrane_index(2)
                    dc2 = m_dcm1;
                    dr2 = m_dcm1_r;
                    dc3 = m_dcm2;
                    dr3 = m_dcm2_r;               
                else
                    dc3 = m_dcm1;
                    dr3 = m_dcm1_r;
                    dc2 = m_dcm2;
                    dr2 = m_dcm2_r;      
                end

             else
                dc3 = m_dcm2;
                dr3 = m_dcm2_r;
                if membrane_index(3) == membrane_index(2)
                    dc2 = m_dc2;
                    dr2 = m_dc2_r;
                    dc1 = m_dc1;
                    dr1 = m_dc1_r;               
                else
                    dc1 = m_dc2;
                    dr1 = m_dc2_r;
                    dc2 = m_dc1;
                    dr2 = m_dc1_r;     
                end
             end  

            variables = {'c1', 'c2', 'c3', 'r1', 'r2', 'r3'};
            v1 = tangent_gradient.v1; v2 = tangent_gradient.v2; v3 = tangent_gradient.v3;
            u1 = tangent_gradient.u1; u2 = tangent_gradient.u2; u3 = tangent_gradient.u3;
            Jv1 = tangent_gradient.Jv1; Jv2 = tangent_gradient.Jv2; Jv3 = tangent_gradient.Jv3;
            Ju1 = tangent_gradient.Ju1; Ju2 = tangent_gradient.Ju2; Ju3 = tangent_gradient.Ju3;
            if (index(1) > 0)
                [~, dq] = jacobian_convtriangle(q, v1, v2, v3, Jv1, Jv2, Jv3, variables);
            else
                [~, dq] = jacobian_convtriangle(q, u1, u2, u3, Ju1, Ju2, Ju3, variables);
            end

             % derivative with respect to finger centers
                Jcenters_fingers(D * i - D + 1:D * i,(3*(left_finger_index -1) +1) : 3*left_finger_index) =  (dq.dc1*dc1.dc1b + dq.dc2*dc2.dc1b + dq.dc3*dc3.dc1b + dq.dr1*dr1.dc1b + dq.dr2*dr2.dc1b + dq.dr3*dr3.dc1b)*T;
                Jcenters_fingers(D * i - D + 1:D * i,(3*(right_finger_index -1) +1) : 3*right_finger_index) = (dq.dc1*dc1.dc2b + dq.dc2*dc2.dc2b + dq.dc3*dc3.dc2b + dq.dr1*dr1.dc2b + dq.dr2*dr2.dc2b + dq.dr3*dr3.dc2b)*T;
                % derivative with respect to c1 
                Jcenters_palm(D * i - D + 1:D * i,(3*(indices_for_centers(left_finger_index-1) -1) +1) : 3*indices_for_centers(left_finger_index-1)) =  (dq.dc1*dc1.dc1 + dq.dc2*dc2.dc1 + dq.dc3*dc3.dc1 + dq.dr1*dr1.dc1 + dq.dr2*dr2.dc1 + dq.dr3*dr3.dc1)*T;
                Jcenters_palm(D * i - D + 1:D * i,(3*(indices_for_centers(right_finger_index-1) -1) +1) : 3*indices_for_centers(right_finger_index-1)) = (dq.dc1*dc1.dc2 + dq.dc2*dc2.dc2 + dq.dc3*dc3.dc2 + dq.dr1*dr1.dc2 + dq.dr2*dr2.dc2 + dq.dr3*dr3.dc2)*T;                
                % beta derivative
                Jbeta(D * i - D + 1:D * i, 1 +3*(left_finger_index -1) +1) = (dq.dc1*dc1.dbeta1 + dq.dc2*dc2.dbeta1 + dq.dc3*dc3.dbeta1 + dq.dr1*dr1.dbeta1 + dq.dr2*dr2.dbeta1 + dq.dr3*dr3.dbeta1);
                Jbeta(D * i - D + 1:D * i, 1 +3*(right_finger_index -1) +1) = (dq.dc1*dc1.dbeta2 + dq.dc2*dc2.dbeta2 + dq.dc3*dc3.dbeta2 + dq.dr1*dr1.dbeta2 + dq.dr2*dr2.dbeta2 + dq.dr3*dr3.dbeta2);
                % radi derivative
                Jr_palm (D * i - D + 1:D * i, indices_for_centers(left_finger_index-1) ) = (dq.dc1*dc1.dr1 + dq.dc2*dc2.dr1 + dq.dc3*dc3.dr1 + dq.dr1*dr1.dr1 + dq.dr2*dr2.dr1 + dq.dr3*dr3.dr1);
                Jr (D * i - D + 1:D * i, 1 + 4*(left_finger_index -1) +1  )= (dq.dc1*dc1.dr11 + dq.dc2*dc2.dr11 + dq.dc3*dc3.dr11 + dq.dr1*dr1.dr11 + dq.dr2*dr2.dr11 + dq.dr3*dr3.dr11);
                Jr (D * i - D + 1:D * i, 1 + 4*(left_finger_index -1) +2  )= (dq.dc1*dc1.dr12 + dq.dc2*dc2.dr12 + dq.dc3*dc3.dr12 + dq.dr1*dr1.dr12 + dq.dr2*dr2.dr12 + dq.dr3*dr3.dr12);
                Jr_palm (D * i - D + 1:D * i, indices_for_centers(right_finger_index-1) ) = (dq.dc1*dc1.dr2 + dq.dc2*dc2.dr2 + dq.dc3*dc3.dr2 + dq.dr1*dr1.dr2 + dq.dr2*dr2.dr2 + dq.dr3*dr3.dr2);
                Jr (D * i - D + 1:D * i, 1 +4*(right_finger_index -1) +1  )  = (dq.dc1*dc1.dr21 + dq.dc2*dc2.dr21 + dq.dc3*dc3.dr21 + dq.dr1*dr1.dr21 + dq.dr2*dr2.dr21 + dq.dr3*dr3.dr21);
                Jr (D * i - D + 1:D * i, 1 +4*(right_finger_index -1) +2  )  = (dq.dc1*dc1.dr22 + dq.dc2*dc2.dr22 + dq.dc3*dc3.dr22 + dq.dr1*dr1.dr22 + dq.dr2*dr2.dr22 + dq.dr3*dr3.dr22);
                % other stuff
                Jtheta(D * i - D + 1:D * i, 1 + (left_finger_index-1)* 4) = (dq.dc1*dc1.dtheta11 + dq.dc2*dc2.dtheta11 + dq.dc3*dc3.dtheta11 + dq.dr1*dr1.dtheta11 + dq.dr2*dr2.dtheta11 + dq.dr3*dr3.dtheta11);
                Jtheta(D * i - D + 1:D * i, 2 + (left_finger_index-1)* 4) = (dq.dc1*dc1.dtheta12 + dq.dc2*dc2.dtheta12 + dq.dc3*dc3.dtheta12 + dq.dr1*dr1.dtheta12 + dq.dr2*dr2.dtheta12 + dq.dr3*dr3.dtheta12);
                Jtheta(D * i - D + 1:D * i, 1 + (right_finger_index-1)* 4) = (dq.dc1*dc1.dtheta21 + dq.dc2*dc2.dtheta21 + dq.dc3*dc3.dtheta21 + dq.dr1*dr1.dtheta21 + dq.dr2*dr2.dtheta21 + dq.dr3*dr3.dtheta21);
                Jtheta(D * i - D + 1:D * i, 2 + (right_finger_index-1)* 4) =(dq.dc1*dc1.dtheta22 + dq.dc2*dc2.dtheta22 + dq.dc3*dc3.dtheta22 + dq.dr1*dr1.dtheta22 + dq.dr2*dr2.dtheta22 + dq.dr3*dr3.dtheta22);
                Js_membrane (D * i - D + 1:D * i, left_finger_index - 1) = (dq.dc1*dc1.ds1 + dq.dc2*dc2.ds1 + dq.dc3*dc3.ds1 + dq.dr1*dr1.ds1 + dq.dr2*dr2.ds1 + dq.dr3*dr3.ds1);
                Js_membrane (D * i - D + 1:D * i, right_finger_index - 1) = (dq.dc1*dc1.ds2 + dq.dc2*dc2.ds2 + dq.dc3*dc3.ds2 + dq.dr1*dr1.ds2 + dq.dr2*dr2.ds2 + dq.dr3*dr3.ds2);           
            
            
            %Js_membrane (D * i - D + 1:D * i, right_finger_index - 1) = (dq.dc1*dc1.ds2 + dq.dc2*dc2.ds2 + dq.dc3*dc3.ds2 + dq.dr1*dr1.ds2 + dq.dr2*dr2.ds2 + dq.dr3*dr3.ds2);           
            
            %Jgx = dq.dc1*cross([1;0;0], centers{index(1)}) + dq.dc2*cross([1;0;0], centers{index(2)}) + dq.dc3*cross([1;0;0], centers{index(3)});
            %Jgy = dq.dc1*cross([0;1;0], centers{index(1)}) + dq.dc2*cross([0;1;0], centers{index(2)}) + dq.dc3*cross([0;1;0], centers{index(3)});
            %Jgz = dq.dc1*cross([0;0;1], centers{index(1)}) + dq.dc2*cross([0;0;1], centers{index(2)}) + dq.dc3*cross([0;0;1], centers{index(3)});
            %Jglobal_rotation(i,:) = n'*[ Jgx, Jgy, Jgz];
        end
                 
        if length(index) == 2
            
            if (sum(is_membrane_center) == 2)
                if membrane_index(1) == left_finger_index
                    dc1 = m_dcm1;
                    dr1 = m_dcm1_r;
                    dc2 = m_dcm2;
                    dr2 = m_dcm2_r;               
                else
                    dc2 = m_dcm1;
                    dr2 = m_dcm1_r;
                    dc1 = m_dcm2;
                    dr1 = m_dcm2_r;       
                end  
            elseif (sum(is_membrane_center) == 0)
                if membrane_index(1) == left_finger_index
                    dc1 = m_dc1;
                    dr1 = m_dc1_r;
                    dc2 = m_dc2;
                    dr2 = m_dc2_r;               
                else
                    dc2 = m_dc1;
                    dr2 = m_dc1_r;
                    dc1 = m_dc2;
                    dr1 = m_dc2_r;       
                end    
            else
                if membrane_index(1) == left_finger_index && membrane_index(2) == left_finger_index
                   dc1 = m_dc1;
                   dr1 = m_dc1_r;
                   dc2 = m_dcm1;
                   dr2 = m_dcm1_r;
                elseif membrane_index(1) == right_finger_index && membrane_index(2) == right_finger_index
                   dc1 = m_dc2;
                   dr1 = m_dc2_r;
                   dc2 = m_dcm2;
                   dr2 = m_dcm2_r;
                else
                   dc1 = m_dc1;
                   dr1 = m_dc1_r;
                   dc2 = m_dcm2;
                   dr2 = m_dcm2_r; 
                end
            end
            variables = {'c1', 'c2', 'r1', 'r2'};
            [q, dq] = jacobian_convsegment(q, centers{index(1)}, centers{index(2)}, radii{index(1)}, radii{index(2)}, variables);
            
            % derivative with respect to finger centers
                Jcenters_fingers(D * i - D + 1:D * i,(3*(left_finger_index -1) +1) : 3*left_finger_index) =  (dq.dc1*dc1.dc1b + dq.dc2*dc2.dc1b  + dq.dr1*dr1.dc1b + dq.dr2*dr2.dc1b )*T;
                Jcenters_fingers(D * i - D + 1:D * i,(3*(right_finger_index -1) +1) : 3*right_finger_index) = (dq.dc1*dc1.dc2b + dq.dc2*dc2.dc2b  + dq.dr1*dr1.dc2b + dq.dr2*dr2.dc2b )*T;
                % derivative with respect to c1 
                Jcenters_palm(D * i - D + 1:D * i,(3*(indices_for_centers(left_finger_index-1) -1) +1) : 3*indices_for_centers(left_finger_index-1)) =  (dq.dc1*dc1.dc1 + dq.dc2*dc2.dc1 + dq.dr1*dr1.dc1 + dq.dr2*dr2.dc1 )*T;
                Jcenters_palm(D * i - D + 1:D * i,(3*(indices_for_centers(right_finger_index-1) -1) +1) : 3*indices_for_centers(right_finger_index-1)) = (dq.dc1*dc1.dc2 + dq.dc2*dc2.dc2 +  dq.dr1*dr1.dc2 + dq.dr2*dr2.dc2 )*T;                
                % beta derivative
                Jbeta(D * i - D + 1:D * i, 1 +3*(left_finger_index -1) +1) =(dq.dc1*dc1.dbeta1 + dq.dc2*dc2.dbeta1  + dq.dr1*dr1.dbeta1 + dq.dr2*dr2.dbeta1 );
                Jbeta(D * i - D + 1:D * i, 1 +3*(right_finger_index -1) +1) =(dq.dc1*dc1.dbeta2 + dq.dc2*dc2.dbeta2  + dq.dr1*dr1.dbeta2 + dq.dr2*dr2.dbeta2 );
                % radi derivative
                Jr_palm (D * i - D + 1:D * i, indices_for_centers(left_finger_index-1) ) = (dq.dc1*dc1.dr1 + dq.dc2*dc2.dr1  + dq.dr1*dr1.dr1 + dq.dr2*dr2.dr1);
                Jr (D * i - D + 1:D * i, 1 + 4*(left_finger_index -1) +1  )= (dq.dc1*dc1.dr11 + dq.dc2*dc2.dr11  + dq.dr1*dr1.dr11 + dq.dr2*dr2.dr11 );
                Jr (D * i - D + 1:D * i, 1 + 4*(left_finger_index -1) +2  )= (dq.dc1*dc1.dr12 + dq.dc2*dc2.dr12 + dq.dr1*dr1.dr12 + dq.dr2*dr2.dr12 );
                Jr_palm (D * i - D + 1:D * i, indices_for_centers(right_finger_index-1) ) = (dq.dc1*dc1.dr2 + dq.dc2*dc2.dr2  + dq.dr1*dr1.dr2 + dq.dr2*dr2.dr2 );
                Jr (D * i - D + 1:D * i, 1 +4*(right_finger_index -1) +1  )  = (dq.dc1*dc1.dr21 + dq.dc2*dc2.dr21 + dq.dr1*dr1.dr21 + dq.dr2*dr2.dr21 );
                Jr (D * i - D + 1:D * i, 1 +4*(right_finger_index -1) +2  )  = (dq.dc1*dc1.dr22 + dq.dc2*dc2.dr22  + dq.dr1*dr1.dr22 + dq.dr2*dr2.dr22 );
                % other stuff
                Jtheta(D * i - D + 1:D * i, 1 + (left_finger_index-1)* 4) = (dq.dc1*dc1.dtheta11 + dq.dc2*dc2.dtheta11  + dq.dr1*dr1.dtheta11 + dq.dr2*dr2.dtheta11 );
                Jtheta(D * i - D + 1:D * i, 2 + (left_finger_index-1)* 4) = (dq.dc1*dc1.dtheta12 + dq.dc2*dc2.dtheta12  + dq.dr1*dr1.dtheta12 + dq.dr2*dr2.dtheta12 );
                Jtheta(D * i - D + 1:D * i, 1 + (right_finger_index-1)* 4) = (dq.dc1*dc1.dtheta21 + dq.dc2*dc2.dtheta21  + dq.dr1*dr1.dtheta21 + dq.dr2*dr2.dtheta21 );
                Jtheta(D * i - D + 1:D * i, 2 + (right_finger_index-1)* 4) = (dq.dc1*dc1.dtheta22 + dq.dc2*dc2.dtheta22  + dq.dr1*dr1.dtheta22 + dq.dr2*dr2.dtheta22 );
                Js_membrane (D * i - D + 1:D * i, left_finger_index - 1) = (dq.dc1*dc1.ds1 + dq.dc2*dc2.ds1  + dq.dr1*dr1.ds1 + dq.dr2*dr2.ds1 );
                Js_membrane (D * i - D + 1:D * i, right_finger_index - 1) = (dq.dc1*dc1.ds2 + dq.dc2*dc2.ds2  + dq.dr1*dr1.ds2 + dq.dr2*dr2.ds2 );           
            
            
            
            %Js_membrane (D * i - D + 1:D * i, right_finger_index - 1) = (dq.dc1*dc1.ds2 + dq.dc2*dc2.ds2 + dq.dr1*dr1.ds2 + dq.dr2*dr2.ds2 );                   
        
            %Jgx = dq.dc1*cross([1;0;0], centers{index(1)}) + dq.dc2*cross([1;0;0], centers{index(2)}) ;
            %Jgy = dq.dc1*cross([0;1;0], centers{index(1)}) + dq.dc2*cross([0;1;0], centers{index(2)}) ;
            %Jgz = dq.dc1*cross([0;0;1], centers{index(1)}) + dq.dc2*cross([0;0;1], centers{index(2)}) ;
            %Jglobal_rotation(i,:) = n'*[ Jgx, Jgy, Jgz];
        
        
        end
                 
        %TO DO: figure it out
        if length(index) == 1
            if(is_membrane_center == 1 && membrane_index == left_finger_index)
                dc1 = m_dcm1;
                dr1 = m_dcm1_r;
            elseif (is_membrane_center == 1 && membrane_index == right_finger_index)
                dc1 = m_dcm2;
                dr1 = m_dcm2_r;
            elseif (is_membrane_center == 0 && membrane_index == right_finger_index)
                dc1 = m_dc2;
                dr1 = m_dc2_r;
            else
                dc1 = m_dc1;
                dr1 = m_dc1_r;
                
            end

            variables = {'c1', 'r1'};
            [q, dq] = jacobian_sphere(q, centers{index(1)}, radii{index(1)}, variables);
            
            % derivative with respect to finger centers
                Jcenters_fingers(D * i - D + 1:D * i,(3*(left_finger_index -1) +1) : 3*left_finger_index) =  (dq.dc1*dc1.dc1b + dq.dr1*dr1.dc1b )*T;
                Jcenters_fingers(D * i - D + 1:D * i,(3*(right_finger_index -1) +1) : 3*right_finger_index) = (dq.dc1*dc1.dc2b + dq.dr1*dr1.dc2b )*T;
                % derivative with respect to c1 
                Jcenters_palm(D * i - D + 1:D * i,(3*(indices_for_centers(left_finger_index-1) -1) +1) : 3*indices_for_centers(left_finger_index-1)) =  (dq.dc1*dc1.dc1 +  dq.dr1*dr1.dc1 )*T;
                Jcenters_palm(D * i - D + 1:D * i,(3*(indices_for_centers(right_finger_index-1) -1) +1) : 3*indices_for_centers(right_finger_index-1)) = (dq.dc1*dc1.dc2  + dq.dr1*dr1.dc2 )*T;                
                % beta derivative
                Jbeta(D * i - D + 1:D * i, 1 +3*(left_finger_index -1) +1) = (dq.dc1*dc1.dbeta1 +  dq.dr1*dr1.dbeta1 );
                Jbeta(D * i - D + 1:D * i, 1 +3*(right_finger_index -1) +1) = (dq.dc1*dc1.dbeta2 + dq.dr1*dr1.dbeta2 );
                % radi derivative
                Jr_palm (D * i - D + 1:D * i, indices_for_centers(left_finger_index-1) ) = (dq.dc1*dc1.dr1  + dq.dr1*dr1.dr1 );
                Jr (D * i - D + 1:D * i, 1 + 4*(left_finger_index -1) +1  )= (dq.dc1*dc1.dr11 + dq.dr1*dr1.dr11 );
                Jr (D * i - D + 1:D * i, 1 + 4*(left_finger_index -1) +2  )= (dq.dc1*dc1.dr12  + dq.dr1*dr1.dr12 );
                Jr_palm (D * i - D + 1:D * i, indices_for_centers(right_finger_index-1) ) = (dq.dc1*dc1.dr2  + dq.dr1*dr1.dr2 );
                Jr (D * i - D + 1:D * i, 1 +4*(right_finger_index -1) +1  )  = (dq.dc1*dc1.dr21 + dq.dr1*dr1.dr21 );
                Jr (D * i - D + 1:D * i, 1 +4*(right_finger_index -1) +2  )  = (dq.dc1*dc1.dr22  + dq.dr1*dr1.dr22 );
                % other stuff
                Jtheta(D * i - D + 1:D * i, 1 + (left_finger_index-1)* 4) = (dq.dc1*dc1.dtheta11  + dq.dr1*dr1.dtheta11  );
                Jtheta(D * i - D + 1:D * i, 2 + (left_finger_index-1)* 4) = (dq.dc1*dc1.dtheta12 + dq.dr1*dr1.dtheta12 );
                Jtheta(D * i - D + 1:D * i, 1 + (right_finger_index-1)* 4) = (dq.dc1*dc1.dtheta21 + dq.dr1*dr1.dtheta21 );
                Jtheta(D * i - D + 1:D * i, 2 + (right_finger_index-1)* 4) = (dq.dc1*dc1.dtheta22 + dq.dr1*dr1.dtheta22);
                Js_membrane (D * i - D + 1:D * i, left_finger_index - 1) = (dq.dc1*dc1.ds1  + dq.dr1*dr1.ds1 );
                Js_membrane (D * i - D + 1:D * i, right_finger_index - 1) = (dq.dc1*dc1.ds2 + dq.dr1*dr1.ds2 );   
            
            %Jgx = dq.dc1*cross([1;0;0], centers{index(1)});
            %Jgy = dq.dc1*cross([0;1;0], centers{index(1)});
            %Jgz = dq.dc1*cross([0;0;1], centers{index(1)});
            %Jglobal_rotation(i,:) = n'*[ Jgx, Jgy, Jgz];
        end
 
    else     
    % then we are working with the old parametrization    
    if length(index) == 1
        variables = {'c1', 'r1'};
        [q, dq] = jacobian_sphere(q, centers{index(1)}, radii{index(1)}, variables);
        
    end
    if length(index) == 2
        variables = {'c1', 'c2', 'r1', 'r2'};
        [q, dq] = jacobian_convsegment(q, centers{index(1)}, centers{index(2)}, radii{index(1)}, radii{index(2)}, variables);
    end
    if length(index) == 3
        variables = {'c1', 'c2', 'c3', 'r1', 'r2', 'r3'};
        v1 = tangent_gradient.v1; v2 = tangent_gradient.v2; v3 = tangent_gradient.v3;
        u1 = tangent_gradient.u1; u2 = tangent_gradient.u2; u3 = tangent_gradient.u3;
        Jv1 = tangent_gradient.Jv1; Jv2 = tangent_gradient.Jv2; Jv3 = tangent_gradient.Jv3;
        Ju1 = tangent_gradient.Ju1; Ju2 = tangent_gradient.Ju2; Ju3 = tangent_gradient.Ju3;
        if (index(1) > 0)
            [~, dq] = jacobian_convtriangle(q, v1, v2, v3, Jv1, Jv2, Jv3, variables);
        else
            [~, dq] = jacobian_convtriangle(q, u1, u2, u3, Ju1, Ju2, Ju3, variables);
        end
    end
    
    
        T =  hand_model.global_pose;
        T = T(1:3,1:3);    
        
        if(block_index == 9 )
            % this is the block having two centers belong to a segment
            if length(index) == 1
                
                if(index(1) == 13)
                    Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc1*hand_model.segments{1}{1}.global(1:3,1:3);
                    Jr_fold(D * i - D + 1:D * i, 1) = dq.dr1;
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc1*cross(R*[1; 0; 0], centers{13} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc1*cross(R*[0; 1; 0], centers{13} - centers{14});
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc1*T;
                elseif(index(1) == 14)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc1*T;
                    Jr(D * i - D + 1:D * i, 1) = dq.dr1;
                elseif(index(1) == 15)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc1*T;
                    Jr(D * i - D + 1:D * i, 2) = dq.dr1;
                    R = hand_model.segments{1}{1}.global(1:3,1:3);
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc1*cross(R*[1; 0; 0], centers{15} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc1*cross(R*[0; 1; 0], centers{15} - centers{14});
                    Jbeta(D * i - D + 1:D * i,1) = dq.dc1*R*[0; 0; 1];                   
                end

        
            elseif length(index) == 2
                
                if(index(1) == 13)
                    Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc1*hand_model.segments{1}{1}.global(1:3,1:3);
                    Jr_fold(D * i - D + 1:D * i, 1) = dq.dr1;
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc1*cross(R*[1; 0; 0], centers{13} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc1*cross(R*[0; 1; 0], centers{13} - centers{14});
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc1*T;
                elseif(index(1) == 14)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc1*T;
                    Jr(D * i - D + 1:D * i, 1) = dq.dr1;
                elseif(index(1) == 15)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc1*T;
                    Jr(D * i - D + 1:D * i, 2) = dq.dr1;
                    R = hand_model.segments{1}{1}.global(1:3,1:3);
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc1*cross(R*[1; 0; 0], centers{15} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc1*cross(R*[0; 1; 0], centers{15} - centers{14});
                    Jbeta(D * i - D + 1:D * i,1) = dq.dc1*R*[0; 0; 1];                   
                end

                if(index(2) == 13)
                    Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc2*hand_model.segments{1}{1}.global(1:3,1:3);
                    Jr_fold(D * i - D + 1:D * i, 1) = dq.dr2;
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc2*cross(R*[1; 0; 0], centers{13} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc2*cross(R*[0; 1; 0], centers{13} - centers{14});
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc2*T;
                elseif(index(2) == 14)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc2*T;
                    Jr(D * i - D + 1:D * i, 1) = dq.dr2;
                elseif(index(2) == 15)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc2*T;
                    Jr(D * i - D + 1:D * i, 2) = dq.dr2;
                    R = hand_model.segments{1}{1}.global(1:3,1:3);
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc2*cross(R*[1; 0; 0], centers{15} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc2*cross(R*[0; 1; 0], centers{15} - centers{14});
                    Jbeta(D * i - D + 1:D * i,1) = dq.dc2*R*[0; 0; 1];                   
                end


        
            elseif length(index) == 3
                
                index = abs(index);
                
                if(index(1) == 13)
                    Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc1*hand_model.segments{1}{1}.global(1:3,1:3);
                    Jr_fold(D * i - D + 1:D * i, 1) = dq.dr1;
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc1*cross(R*[1; 0; 0], centers{13} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc1*cross(R*[0; 1; 0], centers{13} - centers{14});
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc1*T;
                elseif(index(1) == 14)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc1*T;
                    Jr(D * i - D + 1:D * i, 1) = dq.dr1;
                elseif(index(1) == 15)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc1*T;
                    Jr(D * i - D + 1:D * i, 2) = dq.dr1;
                    R = hand_model.segments{1}{1}.global(1:3,1:3);
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc1*cross(R*[1; 0; 0], centers{15} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc1*cross(R*[0; 1; 0], centers{15} - centers{14});
                    Jbeta(D * i - D + 1:D * i,1) = dq.dc1*R*[0; 0; 1];                   
                end

                if(index(2) == 13)
                    Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc2*hand_model.segments{1}{1}.global(1:3,1:3);
                    Jr_fold(D * i - D + 1:D * i, 1) = dq.dr2;
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc2*cross(R*[1; 0; 0], centers{13} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc2*cross(R*[0; 1; 0], centers{13} - centers{14});
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc2*T;
                elseif(index(2) == 14)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc2*T;
                    Jr(D * i - D + 1:D * i, 1) = dq.dr2;
                elseif(index(2) == 15)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3) + dq.dc2*T;
                    Jr(D * i - D + 1:D * i, 2) = dq.dr2;
                    R = hand_model.segments{1}{1}.global(1:3,1:3);
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc2*cross(R*[1; 0; 0], centers{15} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc2*cross(R*[0; 1; 0], centers{15} - centers{14});
                    Jbeta(D * i - D + 1:D * i,1) = dq.dc2*R*[0; 0; 1];                   
                end
                
                
                if(index(3) == 13)
                    Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc3*hand_model.segments{1}{1}.global(1:3,1:3);
                    Jr_fold(D * i - D + 1:D * i, 1) = dq.dr3;
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc3*cross(R*[1; 0; 0], centers{13} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc3*cross(R*[0; 1; 0], centers{13} - centers{14});
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3)  + dq.dc3*T;
                elseif(index(3) == 14)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3)  + dq.dc3*T;
                    Jr(D * i - D + 1:D * i, 1) = dq.dr3;
                elseif(index(3) == 15)
                    Jcenters_fingers(D * i - D + 1:D * i, 1:3) = Jcenters_fingers(D * i - D + 1:D * i, 1:3)  + dq.dc3*T;
                    Jr(D * i - D + 1:D * i, 2) = dq.dr3;
                    R = hand_model.segments{1}{1}.global(1:3,1:3);
                    Jtheta(D * i - D + 1:D * i,1) =Jtheta(D * i - D + 1:D * i,1) + dq.dc3*cross(R*[1; 0; 0], centers{15} - centers{14});
                    Jtheta(D * i - D + 1:D * i,2) =Jtheta(D * i - D + 1:D * i,2) + dq.dc3*cross(R*[0; 1; 0], centers{15} - centers{14});
                    Jbeta(D * i - D + 1:D * i,1) = dq.dc3*R*[0; 0; 1];                   
                end
            end
                     
        else
            % then we are dealing with the standard case
        R = hand_model.segments{1}{1}.global(1:3,1:3);
        % PROJECT ALONG NORMAL AND STORE WHAT HAS BEEN COMPUTED      
            if length(index) == 1
                    if(index(1) < 13)
                        Jcenters_palm(D * i - D + 1:D * i, D * index(1) - D + 1:D * index(1)) = dq.dc1*T;
                        Jr_palm(D * i - D + 1:D * i, index(1)) = dq.dr1;
                    elseif(index(1)== 14)
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc1*T;
                        Jr(D * i - D + 1:D * i, 1) = dq.dr1;
                    else
                        Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc1*hand_model.segments{1}{1}.global(1:3,1:3);
                        Jr_fold(D * i - D + 1:D * i, 1) = dq.dr1;
                        Jtheta(D * i - D + 1:D * i,1) = dq.dc1*cross(R*[1; 0; 0], centers{13} - centers{14});
                        Jtheta(D * i - D + 1:D * i,2) = dq.dc1*cross(R*[0; 1; 0], centers{13} - centers{14});
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc1*T;
                    end

            end
            if length(index) == 2
                    if(index(1) < 13)
                        Jcenters_palm(D * i - D + 1:D * i, D * index(1) - D + 1:D * index(1)) = dq.dc1*T;
                        Jr_palm(D * i - D + 1:D * i, index(1)) = dq.dr1;
                    elseif(index(1)== 14)
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc1*T;
                        Jr(D * i - D + 1:D * i, 1) = dq.dr1;
                    else
                        Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc1*hand_model.segments{1}{1}.global(1:3,1:3);
                        Jr_fold(D * i - D + 1:D * i, 1) = dq.dr1;
                        Jtheta(D * i - D + 1:D * i,1) = dq.dc1*cross(R*[1; 0; 0], centers{13} - centers{14});
                        Jtheta(D * i - D + 1:D * i,2) = dq.dc1*cross(R*[0; 1; 0], centers{13} - centers{14});
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc1*T;
                    end

                    if(index(2) < 13)
                        Jcenters_palm(D * i - D + 1:D * i, D * index(2) - D + 1:D * index(2)) = dq.dc2*T;
                        Jr_palm(D * i - D + 1:D * i, index(2)) = dq.dr2;
                    elseif(index(2)== 14)
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc2*T;
                        Jr(D * i - D + 1:D * i, 1) = dq.dr2;
                    else
                        Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc2*hand_model.segments{1}{1}.global(1:3,1:3);
                        Jr_fold(D * i - D + 1:D * i, 1) = dq.dr2;
                        Jtheta(D * i - D + 1:D * i,1) = dq.dc2*cross(R*[1; 0; 0], centers{13} - centers{14});
                        Jtheta(D * i - D + 1:D * i,2) = dq.dc2*cross(R*[0; 1; 0], centers{13} - centers{14});
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc2*T;
                    end
            end
            if length(index) == 3
                    index = abs(index);
                    if(index(1) < 13)
                        Jcenters_palm(D * i - D + 1:D * i, D * index(1) - D + 1:D * index(1)) = dq.dc1*T;
                        Jr_palm(D * i - D + 1:D * i, index(1)) = dq.dr1;
                    elseif(index(1)== 14)
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc1*T;
                        Jr(D * i - D + 1:D * i, 1) = dq.dr1;
                    else
                        Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc1*hand_model.segments{1}{1}.global(1:3,1:3);
                        Jr_fold(D * i - D + 1:D * i, 1) = dq.dr1;
                        Jtheta(D * i - D + 1:D * i,1) = dq.dc1*cross(R*[1; 0; 0], centers{13} - centers{14});
                        Jtheta(D * i - D + 1:D * i,2) = dq.dc1*cross(R*[0; 1; 0], centers{13} - centers{14});
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc1*T;
                    end

                    if(index(2) < 13)
                        Jcenters_palm(D * i - D + 1:D * i, D * index(2) - D + 1:D * index(2)) = dq.dc2*T;
                        Jr_palm(D * i - D + 1:D * i, index(2)) = dq.dr2;
                    elseif(index(2)== 14)
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc2*T;
                        Jr(D * i - D + 1:D * i, 1) = dq.dr2;
                    else
                        Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc2*hand_model.segments{1}{1}.global(1:3,1:3);
                        Jr_fold(D * i - D + 1:D * i, 1) = dq.dr2;
                        Jtheta(D * i - D + 1:D * i,1) = dq.dc2*cross(R*[1; 0; 0], centers{13} - centers{14});
                        Jtheta(D * i - D + 1:D * i,2) = dq.dc2*cross(R*[0; 1; 0], centers{13} - centers{14});
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc2*T;
                    end

                    if(index(3) < 13)
                        Jcenters_palm(D * i - D + 1:D * i, D * index(3) - D + 1:D * index(3)) = dq.dc3*T;            
                        Jr_palm(D * i - D + 1:D * i, index(3)) = dq.dr3;
                    elseif(index(3) == 14)
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc3*T;
                        Jr(D * i - D + 1:D * i, 1) = dq.dr3;
                    else
                        Joffset_fold(D * i - D + 1:D * i, 1:3) = dq.dc3*hand_model.segments{1}{1}.global(1:3,1:3);
                        Jr_fold(D * i - D + 1:D * i, 1) = dq.dr3;
                        Jtheta(D * i - D + 1:D * i,1) = dq.dc3*cross(R*[1; 0; 0], centers{13} - centers{14});
                        Jtheta(D * i - D + 1:D * i,2) = dq.dc3*cross(R*[0; 1; 0], centers{13} - centers{14});
                        Jcenters_fingers(D * i - D + 1:D * i, 1:3) =Jcenters_fingers(D * i - D + 1:D * i, 1:3)+ dq.dc3*T;
                    end
            end
        end   
    end 
    Jglobal_rotation(D * i - D + 1:D * i,:) = [ cross([1;0;0], q), cross([0;1;0], q), cross([0;0;1], q)];
    Jglobal_translation(D * i - D + 1:D * i,:) = eye(3);
    F(D * i - D + 1:D * i) = (p-q);   
end