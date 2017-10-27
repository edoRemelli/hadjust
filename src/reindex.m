function [blocks] = reindex(radii, blocks)

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

