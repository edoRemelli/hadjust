function [index, q, s, is_inside] = projection(p, block, radii, centers, tangent_points)

if length(block) == 3
    c1 = centers{block(1)}; c2 = centers{block(2)}; c3 = centers{block(3)};
    r1 = radii{block(1)}; r2 = radii{block(2)}; r3 = radii{block(3)};
    v1 = tangent_points.v1; v2 = tangent_points.v2; v3 = tangent_points.v3;
    u1 = tangent_points.u1; u2 = tangent_points.u2; u3 = tangent_points.u3;
    index1 = block(1); index2 = block(2); index3 = block(3);
    [index, q, s, is_inside] = projection_convtriangle(p, c1, c2, c3, r1, r2, r3, v1, v2, v3, u1, u2, u3, index1, index2, index3);
end

if length(block) == 2
    c1 = centers{block(1)}; c2 = centers{block(2)};
    r1 = radii{block(1)}; r2 = radii{block(2)};
    index1 = block(1); index2 = block(2);
    [index, q, s, is_inside] = projection_convsegment(p, c1, c2, r1, r2, index1, index2);
end

if length(block) == 1
    c1 = centers{block(1)};
    r1 = radii{block(1)};
    index1 = block(1);    
    [index, q, s, is_inside] = projection_sphere(p, c1, r1, index1);
end