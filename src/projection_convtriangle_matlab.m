function [index, q, s, is_inside] = projection_convtriangle_matlab(p, c1, c2, c3, r1, r2, r3, v1, v2, v3, u1, u2, u3, index1, index2, index3)


%% Compute projection to a convtriangle
[q1, indexa] = closest_point_in_triangle(v1, v2, v3, p, index1, index2, index3);
[q2, indexb] = closest_point_in_triangle(u1, u2, u3, p, -index1, -index2, -index3);
[s, ~] = closest_point_in_triangle(c1, c2, c3, p, 0, 0, 0);

I{1} = indexa; I{2} = indexb;
[~, k] = min([norm(q1 - p), norm(q2 - p)]);
Q{1} = q1; Q{2} = q2; q = Q{k};
index = I{k};

%% Compute projection to a capsule
if length(index) < 3    

    [index12, q12, s12] = projection_convsegment(p, c1, c2, r1, r2, index1, index2);
    [index13, q13, s13] = projection_convsegment(p, c1, c3, r1, r3, index1, index3);
    [index23, q23, s23] = projection_convsegment(p, c2, c3, r2, r3, index2, index3);
    
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
            [~, q, s] = projection_convsegment(q23, c1, c2, r1, r2, index1, index2);
            if ~test_insideness(q23, q, s)
                q = q23; s = s23;
                index = index23;
            else
                q = q12; s = s12; 
                index = index12;
            end
        elseif (is_inside(1) == 1 && is_inside(3) == 1 )
            [~, q, s] = projection_convsegment(q13, c1, c2, r1, r2, index1, index2);
            if ~test_insideness(q13, q, s)
                q = q13; s = s13;
                index = index13;
            else
                q = q12; s = s12; 
                index = index12;
            end
        elseif (is_inside(2) == 1 && is_inside(3) == 1 )
            [~, q, s] = projection_convsegment(q13, c2, c3, r2, r3, index2, index3);
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
if norm(p - s) - norm(q - s) > 10e-7
    is_inside = false;
else
    is_inside = true;
end




