function [tangent_points] = blocks_tangent_points(centers, blocks, radii)

tangent_points = cell(length(blocks), 1);

for i = 1:length(blocks)
    
    if length(blocks{i}) == 3
        
        c1 = centers{blocks{i}(1)};
        c2 = centers{blocks{i}(2)};
        c3 = centers{blocks{i}(3)};
        
        r1 = radii{blocks{i}(1)};
        r2 = radii{blocks{i}(2)};
        r3 = radii{blocks{i}(3)};
        
        [v1, v2, v3, u1, u2, u3, n, m] = tangent_points_function(c1, c2, c3, r1, r2, r3);
        
        tangent_points{i}.v1 = v1;
        tangent_points{i}.v2 = v2;
        tangent_points{i}.v3 = v3;
        tangent_points{i}.u1 = u1;
        tangent_points{i}.u2 = u2;
        tangent_points{i}.u3 = u3; 
        tangent_points{i}.n = n;
        tangent_points{i}.m = m;              
    end
end
