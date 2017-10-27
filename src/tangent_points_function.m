function [v1, v2, v3, u1, u2, u3, n, m] = tangent_points_function(c1, c2, c3, r1, r2, r3)

epsilon = 10e-9;
if r1 - r2 < epsilon && r1 - r3 < epsilon
    n = cross(c1 - c2, c1 - c3);
    n = n / norm(n);
    v1 = c1 + r1 * n;
    v2 = c2 + r2 * n;
    v3 = c3 + r3 * n;
    m = -n;
    u1 = c1 + r1 * m;
    u2 = c2 + r2 * m;
    u3 = c3 + r3 * m;
    return;
end

z12 = c1 + (c2 - c1) * r1 / (r1 - r2);
z13 = c1 + (c3 - c1) * r1 / (r1 - r3);

l = (z12 - z13) / norm(z12 - z13);
projection = (c1 - z12)' * l;
z = z12 + projection * l;

eta = norm(c1 - z);
sin_beta = r1/eta;
nu = sqrt(eta^2 - r1^2);
cos_beta = nu/eta;

f = (c1 - z) / eta;
h = cross(l, f);
h = h / norm(h);

g = sin_beta * h + cos_beta * f;
v1 = z + nu * g;
n = (v1  - c1) / norm(v1  - c1);
v2 = c2 + r2 * n;
v3 = c3 + r3 * n;

g = - sin_beta * h + cos_beta * f;
u1 = z + nu * g;
m = (u1  - c1) / norm(u1  - c1);
u2 = c2 + r2 * m;
u3 = c3 + r3 * m;