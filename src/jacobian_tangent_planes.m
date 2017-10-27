function [tangent_gradients] = jacobian_tangent_planes(centers, blocks, radii, variables)
if length(centers{1}) == 2, return; end

tangent_gradients = cell(length(blocks), 1);

for i = 1:length(blocks)
    
    if length(blocks{i}) == 3
        
        c1 = centers{blocks{i}(1)};
        c2 = centers{blocks{i}(2)};
        c3 = centers{blocks{i}(3)};
        
        r1 = radii{blocks{i}(1)};
        r2 = radii{blocks{i}(2)};
        r3 = radii{blocks{i}(3)};
        
        [v1, v2, v3, Jv1, Jv2, Jv3, u1, u2, u3, Ju1, Ju2, Ju3] = jacobian_tangent_plane(c1, c2, c3, r1, r2, r3, variables);
        
        tangent_gradients{i}.v1 = v1;
        tangent_gradients{i}.v2 = v2;
        tangent_gradients{i}.v3 = v3;
        tangent_gradients{i}.Jv1 = Jv1;
        tangent_gradients{i}.Jv2 = Jv2;
        tangent_gradients{i}.Jv3 = Jv3;
        
        tangent_gradients{i}.v3 = v3;
        tangent_gradients{i}.u1 = u1;
        tangent_gradients{i}.u2 = u2;
        tangent_gradients{i}.u3 = u3;  
        tangent_gradients{i}.Ju1 = Ju1;
        tangent_gradients{i}.Ju2 = Ju2;
        tangent_gradients{i}.Ju3 = Ju3;       
       
    end
end
