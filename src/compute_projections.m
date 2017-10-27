function [indices, projections, block_indices] = compute_projections(points, hand_model)

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
tangent_points = blocks_tangent_points(centers, blocks, radii);

RAND_MAX = 32767;
R = zeros(length(radii), 1);
C = zeros(length(centers), D);
B = RAND_MAX * ones(length(blocks), 3);
T = RAND_MAX * ones(length(blocks), 6 * D);
for j = 1:length(radii)
    R(j) = radii{j};
    C(j, :) = centers{j}';
end
for j = 1:length(blocks)
    for k = 1:length(blocks{j})
        B(j, k) = blocks{j}(k) - 1;
    end
    if ~isempty(tangent_points{j})
        T(j, 1:3) = tangent_points{j}.v1';
        T(j, 4:6) = tangent_points{j}.v2';
        T(j, 7:9) = tangent_points{j}.v3';
        T(j, 10:12) = tangent_points{j}.u1';
        T(j, 13:15) = tangent_points{j}.u2';
        T(j, 16:18) = tangent_points{j}.u3';
    end
end
P =  zeros(length(points), D);
for j = 1:length(points)
    P(j, :) = points{j}';
end

[I, Pr, Bi] = compute_projections_mex(P, C, R, B, T);

num_points = length(points);
indices = cell(num_points, 1);
block_indices = cell(num_points, 1);
projections = cell(num_points, 1);

for i = 1:length(points)
    if (i == 4)
        disp('');
    end
    index = [];
    for j = 1:D
        if I(i, j) >= RAND_MAX, break; end
        index = [index; I(i, j)];
    end
    indices{i} = index;
    if (~isempty(index))
        projections{i} = Pr(i, :)';
    end
    block_indices{i} = Bi(i);
end

