function [index, q, s, is_inside] = projection_convsegment(p, c1, c2, r1, r2, index1, index2)

if r2 > r1
    temp = r1;
    r1 = r2;
    r2 = temp;
    
    temp = c1;
    c1 = c2;
    c2 = temp;
    
    temp = index1;
    index1 = index2;
    index2 = temp;
end

index = [];

u = c2 - c1;
v = p - c1;

alpha = u' * v / (u' * u);
t = c1 + alpha * u;

omega = sqrt(u' * u - (r1 - r2)^2);
delta =  norm(p - t) * (r1 - r2) / omega;

if alpha <= 0
    s = c1;
    q =  c1 + r1 * (p - c1) / norm(p - c1);
    index = [index1];
end
if (alpha > 0 && alpha < 1)
    if (norm(c1 - t) < delta)
        s = c1;
        q = c1 + r1 * (p - c1) / norm(p - c1);
        index = [index1];
    end
end
if (alpha >= 1)
    if (norm(t - c2) > delta)
        s = c2;
        q = c2 + r2 * (p - c2) / norm(p - c2);
        index = [index2];
    end    
    if norm(c1 - c2) < delta
        s = c1;
        q =  c1 + r1 * (p - c1) / norm(p - c1);     
        index = [index2];
    end
end   

if isempty(index)
    s = t - delta * (c2 - c1) / norm(c2 - c1);
    gamma = (r1 - r2) * norm(c2 - t + delta * u / norm(u))/ sqrt(u' * u);
    q = s + (p - s) / norm(p - s) * (gamma + r2);
    index = [index1, index2];
end

%% Cheek is inside or outside
if  norm(p - s) - norm(q - s) > 10e-7 || abs(norm(p - s) - norm(q - s)) < 10e-10
    is_inside = false;
else
    is_inside = true;
end

