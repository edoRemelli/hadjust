function [ hand_model ] = update_centers(hand_model)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


for i = 1:length(hand_model.palm_wrist_centers)
     new_position = hand_model.global_pose * [hand_model.palm_wrist_centers_relative{i}; 1];
     hand_model.palm_wrist_centers{i} = new_position(1:3);
end

