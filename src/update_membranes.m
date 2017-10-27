function [ hand_model ] = update_membranes( hand_model )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


%indices_for_centers = {hand_model.palm_wrist_names_map('palm_index'), hand_model.palm_wrist_names_map('palm_middle'), hand_model.palm_wrist_names_map('palm_ring'), hand_model.palm_wrist_names_map('palm_pinky')};

for i = 1:4
    
    s = hand_model.membrane_position{i};
    
    %index
    %base = hand_model.segments{1+i}{1}.global(1:3,4);
    
    % find base and bottom for membrane
    base = hand_model.segments{1+i}{1}.global(1:3,4);   
    bottom = hand_model.segments{1+i}{2}.global(1:3,4);
    % set up location
    location = base + s*(bottom - base);
    % retrieve segment orientation
    %[ R ] = RotationFromVectors([0;0;1], (base-bottom)/norm(base-bottom) );
    
    R = hand_model.segments{1+i}{1}.global(1:3,1:3);
    
    u1 = R*[0;1;0];
    u2 = hand_model.global_pose(1:3,1:3)*[0;1;0];
    if (u1'*u2 <= 0)
        membrane_direction = [0;1;0];
    else
        membrane_direction = [0;-1;0];
    end
        
    
    
    d  = R* membrane_direction;

    location_ray = (1-s)*hand_model.finger_radii{1+4*i+1} + s*hand_model.finger_radii{1+4*i+2};

    new_location = location + d* (location_ray - 3.0);

    hand_model.membrane_centers{i} = new_location;    
end

% update thumb membrane

hand_model.fold_center = hand_model.segments{1}{1}.global(1:3,4) + hand_model.segments{1}{1}.global(1:3,1:3)*hand_model.fold_offset;


end

