function [i, normal]  = ray_triangle_intersection (p0, p1, p2, o, d)

epsilon = 0.00001;
i = Inf * ones(3, 1);
normal = zeros(3, 1);

e1 = p1 - p0;
e2 = p2 - p0;
q  = cross(d,e2);
a  = dot(e1, q); 

%% the vector is parallel to the plane (the intersection is at infinity)
if (a > -epsilon && a < epsilon)   
    return;
end;

f = 1/a;
s = o - p0;
u = f * dot(s, q);

%% the intersection is outside of the triangle
if (u < 0.0)    
    return;
end;

r = cross(s, e1);
v = f*dot(d, r);

%% the intersection is outside of the triangle
if (v < 0.0 || u + v > 1.0)
    return;
end;

t = f * dot(e2, r); 
i = o + t * d;
normal = cross(e1, e2);
normal = normal / norm(normal);
