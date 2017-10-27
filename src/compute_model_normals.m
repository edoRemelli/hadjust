function [model_normals] = compute_model_normals(centers, radii, blocks, points, model_indices)

tangent_points = blocks_tangent_points(centers, blocks, radii);
model_normals = cell(length(points), 1);
tangent_point = [];
for i = 1:length(points)
    if isempty(model_indices{i}), continue; end
    p = points{i};
    if length(model_indices{i}) == 1
        index = model_indices{i}(1);
        c1 = centers{index}; r1 = radii{index}; s = c1;
        q = c1 + r1 * (p - c1) / norm(p - c1);
    else
        if length(model_indices{i}) == 3
            for b = 1:length(blocks)
                if (length(blocks{b}) < 3), continue; end
                abs_index = [abs(model_indices{i}(1)), abs(model_indices{i}(2)), abs(model_indices{i}(3))];
                indicator = ismember(blocks{b}, abs_index);
                if sum(indicator) == 3
                    tangent_point = tangent_points{b};
                    break;
                end
            end
            model_indices{i} = abs_index;
        end
        [~, q, s, ~] = projection(p, model_indices{i}, radii, centers, tangent_point);
    end    
    model_normals{i} = (q - s) / norm(q - s);   
end