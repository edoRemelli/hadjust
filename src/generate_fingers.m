function [segments, joints, fingers_name_map] = generate_fingers(beta,base_centers,finger_names)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
finger_indices = {}; 
segments = {};
joints = {};

for i = 1 : length(beta)
    if i == 1
        [segments{i}, joints{i}] = segments_and_joints(beta{i},base_centers{i},true);  
    else
        [segments{i}, joints{i}] = segments_and_joints(beta{i},base_centers{i},false); 
    end    
    finger_indices{i} = i;
end

fingers_name_map = containers.Map( finger_names, finger_indices);
