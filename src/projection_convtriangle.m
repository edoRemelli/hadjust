function [index, q, s, is_inside] = projection_convtriangle(p, c1, c2, c3, r1, r2, r3, v1, v2, v3, u1, u2, u3, index1, index2, index3)

%% Find the tangent plane
% z12 = c1 + (c2 - c1) * r1 / (r1 - r2);
% z13 = c1 + (c3 - c1) * r1 / (r1 - r3);
%
% l = (z12 - z13) / norm(z12 - z13);
% projection = (c1 - z12)' * l;
% z = z12 + projection * l;
%
% beta = asin(r1/norm(c1 - z));
%
% g = rotate_around_axis(l, c1 - z, beta);
% v1 = z + norm(c1 - z) * cos(beta) * g;
% n = v1  - c1; n = n / norm(n);
% v2 = c2 + r2 * n;
% v3 = c3 + r3 * n;
%
% g = rotate_around_axis(l, c1 - z, -beta);
% u1 = z + norm(c1 - z) * cos(beta) * g;
% m = c1 - u1; m = m / norm(m);
% u2 = c2 - r2 * m;
% u3 = c3 - r3 * m;

%% Compute projection to a convtriangle
[q1, indexa] = closest_point_in_triangle_mex(v1, v2, v3, p, index1, index2, index3);
[q2, indexb] = closest_point_in_triangle_mex(u1, u2, u3, p, -index1, -index2, -index3);
n1 = cross(v1 - v2, v1 - v3); n1 = n1/norm(n1);
n2 = cross(u1 - u2, u1 - u3); n2 = n2/norm(n2);

%[s, ~] = closest_point_in_triangle_mex(c1, c2, c3, p, 0, 0, 0);

%n = cross(c1 - c2, c1 - c3); n = n / norm(n); distance = (p - c1)' * n; s = p - n * distance;

I{1} = indexa; I{2} = indexb;
N{1} = n1; N{2} = n2;
[~, k] = min([norm(q1 - p), norm(q2 - p)]);
Q{1} = q1; Q{2} = q2; q = Q{k};
index = I{k}; n = N{k};

if length(index) == 3
    s = ray_triangle_intersection (c1, c2, c3, p, n);
    if norm(s) == Inf
        s = ray_triangle_intersection (c1, c2, c3, p, -n);
    end
    if norm(s) == Inf
        %disp('still outside');
        [s, ~] = closest_point_in_triangle_mex(c1, c2, c3, p, 0, 0, 0);
    end
end

%% Compute projection to a capsule
if length(index) < 3
    
    [index12, q12, s12] = projection_convsegment_mex(p, c1, c2, r1, r2, index1, index2);
    [index13, q13, s13] = projection_convsegment_mex(p, c1, c3, r1, r3, index1, index3);
    [index23, q23, s23] = projection_convsegment_mex(p, c2, c3, r2, r3, index2, index3);
    
    Q{1} = q12; Q{2} = q23; Q{3} = q13;
    S{1} = s12; S{2} = s23; S{3} = s13;
    I{1} = index12; I{2} = index23; I{3} = index13;
    
    is_inside = zeros(3, 1);
    for j = 1:3
        if norm(p - S{j}) < norm(Q{j} - S{j})
            is_inside(j) = 1;
        end
    end
    
    %% If the point is simultaneously inside two capsules
    if (sum(is_inside) > 1)
        if (is_inside(1) == 1 && is_inside(2) == 1 )
            [~, q, s] = projection_convsegment_mex(q23, c1, c2, r1, r2, index1, index2);
            if ~test_insideness(q23, q, s)
                q = q23; s = s23;
                index = index23;
            else
                q = q12; s = s12;
                index = index12;
            end
        elseif (is_inside(1) == 1 && is_inside(3) == 1 )
            [~, q, s] = projection_convsegment_mex(q13, c1, c2, r1, r2, index1, index2);
            if ~test_insideness(q13, q, s)
                q = q13; s = s13;
                index = index13;
            else
                q = q12; s = s12;
                index = index12;
            end
        elseif (is_inside(2) == 1 && is_inside(3) == 1 )
            [~, q, s] = projection_convsegment_mex(q13, c2, c3, r2, r3, index2, index3);
            if ~test_insideness(q13, q, s)
                q = q13; s = s13;
                index = index13;
            else
                q = q23; s = s23;
                index = index23;
            end
        end
        
        %% If point is inside one capsule
    elseif sum(is_inside) == 1
        q = Q{find(is_inside)};
        s = S{find(is_inside)};
        index = I{find(is_inside)};
        
        %% If point is outside
    else
        [~, k] = min([norm(p - q12), norm(p - q23), norm(p - q13)]);
        q = Q{k};
        s = S{k};
        index = I{k};
    end
end

%% Cheek is inside or outside
if norm(p - s) - norm(q - s) > 10e-7 || abs(norm(p - s) - norm(q - s)) < 10e-10
    is_inside = false;
else
    is_inside = true;
end




