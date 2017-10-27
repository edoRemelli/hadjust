function [ segments ] = update_fingers_shape(segments, beta )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

for i = 1:length(segments)
    [segments{i}] = shape(segments{i}, beta{i});
end

