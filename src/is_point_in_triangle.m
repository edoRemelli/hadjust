function [is_in_triangle] = is_point_in_triangle(p, a, b, c)

v0 = b - a;
v1 = c - a;
v2 = p - a;
d00 = v0' * v0;
d01 = v0' * v1;
d11 = v1' * v1;
d20 = v2' * v0;
d21 = v2' * v1;
denom = d00 * d11 - d01 * d01;
alpha = (d11 * d20 - d01 * d21) / denom;
beta = (d00 * d21 - d01 * d20) / denom;
gamma = 1.0 - alpha - beta;
if alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1
    is_in_triangle = true;
else
    is_in_triangle = false;
end

end