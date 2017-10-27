function [hand_model] = reindex_fullhand(hand_model)

radii = hand_model.palm_wrist_radii;
radii{end +1} = hand_model.fold_radii;
 for i = 1: length(hand_model.finger_radii)
    radii{end +1} = hand_model.finger_radii{i};
 end
 for i = 1: length(hand_model.membrane_radii)
    radii{end +1} = hand_model.membrane_radii{i};
 end
blocks = hand_model.blocks;

for i = 1:length(blocks)
    I = zeros(length(blocks{i}), 1);
    R = zeros(length(blocks{i}), 1);
    for j = 1:length(blocks{i})
        I(j) = blocks{i}(j);
        R(j) = radii{blocks{i}(j)};
    end
    [~, J] = sort(R, 'descend');
    for j = 1:length(blocks{i})           
        blocks{i}(j) = I(J(j));
    end
end

hand_model.blocks = blocks;

