function [ hand_model ] = pose_model(hand_model, delta_theta_finger, pose )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


delta_pose_matrix = makehgtform('translate',pose.global_translation(1),pose.global_translation(2),pose.global_translation(3))*makehgtform('axisrotate', [1;0;0], pose.global_rotation(1))*makehgtform('axisrotate', [0;1;0], pose.global_rotation(2))*makehgtform('axisrotate', [0;0;1], pose.global_rotation(3));
hand_model.global_pose = delta_pose_matrix * hand_model.global_pose;

for i = 1:length(hand_model.segments)
    [hand_model.segments{i}] = pose_NEW(hand_model.segments{i}, hand_model.joints{i}, delta_theta_finger{i}, zeros(3,1), hand_model.global_pose);
end

for i = 1:length(hand_model.palm_wrist_centers)
     new_position = hand_model.global_pose * [hand_model.palm_wrist_centers_relative{i}; 1];
     hand_model.palm_wrist_centers{i} = new_position(1:3);
end

[ hand_model ] = update_membranes( hand_model );

hand_model.global_rotation = hand_model.global_rotation + pose.global_rotation;
hand_model.global_translation = hand_model.global_translation +pose.global_translation;