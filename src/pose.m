function [segments] = pose(segments, joints, theta, delta_center)

% add a zero at the end of the vector so that we have 1 to 1 matching
% joints-theta
theta = [theta; 0];


segments{1}.local(1:3,4) = segments{1}.local(1:3,4) + delta_center;
segments{1}.global(1:3,4) = segments{1}.global(1:3,4) + delta_center;

%% Pose segments
% loop over joints
for i = 1:length(joints)
    % take current segment
    segment = segments{joints{i}.segment_id};
    % initialize transform 
    T = [];
    % discriminate between roational and translational joint
    switch joints{i}.type
        case 'R'
            % makehgttransform gives us global tranform, convert it to
            % local frame of reference using local matrix property
            T = segment.local * makehgtform('axisrotate', joints{i}.axis, theta(i));
        case 'T'
            % makehgttransform gives us global tranform, convert it to
            % local frame of reference using local matrix property
            T = segment.local * makehgtform('translate', joints{i}.axis * theta(i));
    end 
    
    if (segment.parent_id > 0 )   
        % update local to new local tranform
        segment.local = T;
        % set global tranform to global of parents times local
        segment.global = segments{segment.parent_id}.global * T;   
    else
        segment.local = T;
        segment.global = T;
    end
    
    % store new oriented segment 
    segments{joints{i}.segment_id} = segment;
end