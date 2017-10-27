function [t, index] = closest_point_in_segment(c1, c2, p, index1, index2)

index = [];

u = c2 - c1;
v = p - c1;

q = u' * v / (u' * u);

if q <= 0
    t = c1;
    index = [index1];
end
if q > 0 && q < 1
    t = c1 + q * u;
    index = [index1, index2];
end
if q >= 1
    t = c2;
    index = [index2];
end

end
