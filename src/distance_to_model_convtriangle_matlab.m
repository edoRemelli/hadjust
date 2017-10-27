function [distances] = distance_to_model_convtriangle_matlab(c1, c2, c3, r1, r2, r3, v1, v2, v3, u1, u2, u3, P)

distances = zeros(length(P), 1);
for i = 1:length(P)
    p = P(:, i);
    [~, q, ~, is_inside] = projection_convtriangle_matlab(p, c1, c2, c3, r1, r2, r3, v1, v2, v3, u1, u2, u3, 1, 2, 3);    
    distances(i) = norm(p - q);
    if is_inside
        distances(i) = - distances(i);
    end
end