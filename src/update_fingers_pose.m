function [ hand_model ] = update_fingers_pose(hand_model, delta_theta, delta_center )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

segments = hand_model.segments;
joints = hand_model.joints;

for i = 1:length(segments)
    [hand_model.segments{i}] = pose_NEW(segments{i}, joints{i}, delta_theta{i}, delta_center{i}, hand_model.global_pose);
end



