function [segments] = shape(segments, beta)

% loop over segments 
for i = 2:length(segments)
    % update local transform taking into account new position of frame -
    % reads beta to displace joint accordingly along y_axis
    segments{i}.local(3, 4) = beta(segments{i}.parent_id);
end
 
end
    


