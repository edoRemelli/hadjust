function [F,Jc,Jr, Jc_fingers, Jr_fingers, Jtheta, Jbeta] = COMPUTE_VALIDITY_ENERGY(hand_model, settings)

centers = hand_model.palm_wrist_centers_relative;
centers{end+1} = hand_model.segments{1}{1}.local(1:3, 4);
local_center =  hand_model.segments{1}{1}.local* hand_model.segments{1}{2}.local;
centers{end+1} = local_center(1:3, 4);
radii = hand_model.palm_wrist_radii;
radii{end +1} = hand_model.finger_radii{1};
radii{end +1} = hand_model.finger_radii{2};
blocks = hand_model.blocks(1:9);

D = settings.D;
factor = settings.block_safety_factor;

num_constaints = 9;
F  = zeros(num_constaints, 1);
Jc = zeros(num_constaints, 13 * D);
Jr = zeros(num_constaints, 13);
Jr_fingers = zeros( num_constaints, 21);
Jc_fingers = zeros( num_constaints, 15 );
Jtheta = zeros( num_constaints, 20 );
Jbeta = zeros( num_constaints, 16 );

for i = 1:length(radii)
    radii{i} = radii{i} + 0.001*rand;
end


T =  eye(3);


count = 1;
for b = 1:length(blocks)-1
    
    %% Tangent cone
    indices = nchoosek(blocks{b}, 2);
    index1 = indices(:, 1);
    index2 = indices(:, 2);
    for i = 1:length(index1)
        [r1, max_index] = max([radii{index1(i)}, radii{index2(i)}]);
        [r2, min_index] = min([radii{index1(i)}, radii{index2(i)}]);
        indices = [index1(i), index2(i)]; i1 = indices(max_index); i2 = indices(min_index);
        c1 = centers{i1}; c2 = centers{i2};
        if norm(c1 - c2) - factor * (r1 - r2) > 0
            continue;
        end

        [f, df] = jacobian_tangent_cone_existence(c1, c2, r1, r2, factor, {'c1', 'c2', 'r1', 'r2'});
        
        if (i1 < 14)
            Jr(count, i1) = df.dr1;
            Jc(count, D * i1 - D + 1:D * i1) = df.dc1*T;
        else
            Jr_fingers(count, 1) = df.dr1;
            Jc_fingers(count, 1:3) = df.dc1*T;    
        end
            
        if (i2 < 14)
            Jr(count, i2) = df.dr2;
            Jc(count, D * i2 - D + 1:D * i2) = df.dc2*T;
        else
            Jr_fingers(count, 1) = df.dr2;
            Jc_fingers(count, 1:3) = df.dc2*T;    
        end
        F(count) = f;
        count = count + 1;
    end
    
    %% Tangent plane
    if length(blocks{b}) == 3
        indices = nchoosek(blocks{b}, 2);
        index1 = indices(:, 1);
        index2 = indices(:, 2);
        for i = 1:length(index1)
            [r1, max_index] = max([radii{index1(i)}, radii{index2(i)}]);
            [r2, min_index] = min([radii{index1(i)}, radii{index2(i)}]);
            indices = [index1(i), index2(i)]; i1 = indices(max_index); i2 = indices(min_index);
            i3 = sum([blocks{b}(1), blocks{b}(2), blocks{b}(3)]) - i1 - i2; r3 = radii{i3};
            c1 = centers{i1}; c2 = centers{i2}; c3 = centers{i3};
            
            %% Compute objective function
            z = c1 + (c2 - c1) * r1 / (r1 - r2);
            gamma = (c2 - c1)' * (c3 - c1) / ((c2 - c1)' * (c2 - c1)); t = c1 + gamma * (c2 - c1);
            if (t - c1)' * (z - c1) > 0 && norm(t - c1) > norm(z - c1), t = c1 + (z - c1) + (z - t); end
            delta_r = norm(c2 - t) * (r1 - r2) / norm(c2 - c1);
            if (t - c1)' * (c2 - c1) > 0 && norm(t - c1) > norm(c2 - c1), delta_r = -delta_r; end
            r_tilde = delta_r + r2; beta = asin((r1 - r2) / norm(c2 - c1));
            r = r_tilde/cos(beta); eta = r3 + norm(c3 - t); f = eta - factor * r;
            if f > 0
                continue;
            end
            
            [f, df] = jacobian_tangent_plane_existence(c1, c2, c3, r1, r2, r3, factor, {'c1', 'c2', 'c3', 'r1', 'r2', 'r3'});
            
            if (i1<14)
            Jr(count, i1) = df.dr1;
            Jc(count, D * i1 - D + 1:D * i1) = df.dc1*T;
            else
            Jr(count, i1) = df.dr1;
            Jc(count, D * i1 - D + 1:D * i1) = df.dc1*T;    
                
            end
            
            if(i2<14)
            Jr(count, i2) = df.dr2;
            Jc(count, D * i2 - D + 1:D * i2) = df.dc2*T;
            else
            Jr_fingers(count, 1) = df.dr1;
            Jc_fingers(count, 1:3) = df.dc1*T;              
            end
            
            
            if(i3<14)
            Jr(count, i3) = df.dr3;    
            Jc(count, D * i3 - D + 1:D * i3) = df.dc3*T;
            else
            Jc_fingers(count, 1:3) = df.dr1;
            Jc_fingers(count, 1:3) = df.dc1*T;    
            end
            
            F(count) = f;
            count = count + 1;
        end
    end
end

%% SPECIAL CARE FOR BLOCK 9

b = 9;

%% Tangent cone
    indices = nchoosek(blocks{b}, 2);
    index1 = indices(:, 1);
    index2 = indices(:, 2);
    
    R = hand_model.segments{1}{1}.global(1:3,1:3);
    
    
    for i = 1:length(index1)
        [r1, max_index] = max([radii{index1(i)}, radii{index2(i)}]);
        [r2, min_index] = min([radii{index1(i)}, radii{index2(i)}]);
        indices = [index1(i), index2(i)]; i1 = indices(max_index); i2 = indices(min_index);
        c1 = centers{i1}; c2 = centers{i2};
        if norm(c1 - c2) - factor * (r1 - r2) > 0
            continue;
        end

        [f, df] = jacobian_tangent_cone_existence(c1, c2, r1, r2, factor, {'c1', 'c2', 'r1', 'r2'});
        
        if (i1 == 13)
            Jr(count, i1) = df.dr1;
            Jc(count, D * i1 - D + 1:D * i1) = df.dc1*T;
        elseif (i1 == 14)
            Jr_fingers(count, 1) = df.dr1;
            Jc_fingers(count, 1:3) = df.dc1*T; 
        elseif (i1 == 15)
            Jr_fingers(count, 2) = df.dr1;                                     
            Jtheta(count,1) = df.dc1*cross(R*[1; 0; 0], centers{15} - centers{14});
            Jtheta(count,2) = df.dc1*cross(R*[0; 1; 0], centers{15} - centers{14}); 
            Jbeta(count, 1) = df.dc1*R*[0; 0; 1]; 
        end
            
        if (i2 == 13)
            Jr(count, i2) = df.dr2;
            Jc(count, D * i2 - D + 1:D * i2) = df.dc2*T;
        elseif (i2 == 14)
            Jr_fingers(count, 1) = df.dr2;
            Jc_fingers(count, 1:3) = df.dc2*T; 
        elseif (i2 == 15)
            Jr_fingers(count, 2) = df.dr2;                                     
            Jtheta(count,1) = df.dc2*cross(R*[1; 0; 0], centers{15} - centers{14});
            Jtheta(count,2) = df.dc2*cross(R*[0; 1; 0], centers{15} - centers{14}); 
            Jbeta(count, 1) = df.dc2*R*[0; 0; 1]; 
        end
        
                
        F(count) = f;
        count = count + 1;
    end
    
    %% Tangent plane
    if length(blocks{b}) == 3
        indices = nchoosek(blocks{b}, 2);
        index1 = indices(:, 1);
        index2 = indices(:, 2);
        for i = 1:length(index1)
            [r1, max_index] = max([radii{index1(i)}, radii{index2(i)}]);
            [r2, min_index] = min([radii{index1(i)}, radii{index2(i)}]);
            indices = [index1(i), index2(i)]; i1 = indices(max_index); i2 = indices(min_index);
            i3 = sum([blocks{b}(1), blocks{b}(2), blocks{b}(3)]) - i1 - i2; r3 = radii{i3};
            c1 = centers{i1}; c2 = centers{i2}; c3 = centers{i3};
            
            %% Compute objective function
            z = c1 + (c2 - c1) * r1 / (r1 - r2);
            gamma = (c2 - c1)' * (c3 - c1) / ((c2 - c1)' * (c2 - c1)); t = c1 + gamma * (c2 - c1);
            if (t - c1)' * (z - c1) > 0 && norm(t - c1) > norm(z - c1), t = c1 + (z - c1) + (z - t); end
            delta_r = norm(c2 - t) * (r1 - r2) / norm(c2 - c1);
            if (t - c1)' * (c2 - c1) > 0 && norm(t - c1) > norm(c2 - c1), delta_r = -delta_r; end
            r_tilde = delta_r + r2; beta = asin((r1 - r2) / norm(c2 - c1));
            r = r_tilde/cos(beta); eta = r3 + norm(c3 - t); f = eta - factor * r;
            if f > 0
                continue;
            end
            
            [f, df] = jacobian_tangent_plane_existence(c1, c2, c3, r1, r2, r3, factor, {'c1', 'c2', 'c3', 'r1', 'r2', 'r3'});
            
            if (i1 == 13)
                Jr(count, i1) = df.dr1;
                Jc(count, D * i1 - D + 1:D * i1) = df.dc1*T;
            elseif (i1 == 14)
                Jr_fingers(count, 1) = df.dr1;
                Jc_fingers(count, 1:3) = df.dc1*T; 
            elseif (i1 == 15)
                Jr_fingers(count, 2) = df.dr1;                                     
                Jtheta(count,1) = df.dc1*cross(R*[1; 0; 0], centers{15} - centers{14});
                Jtheta(count,2) = df.dc1*cross(R*[0; 1; 0], centers{15} - centers{14}); 
                Jbeta(count, 1) = df.dc1*R*[0; 0; 1]; 
            end

            if (i2 == 13)
                Jr(count, i2) = df.dr2;
                Jc(count, D * i2 - D + 1:D * i2) = df.dc2*T;
            elseif (i2 == 14)
                Jr_fingers(count, 1) = df.dr2;
                Jc_fingers(count, 1:3) = df.dc2*T; 
            elseif (i2 == 15)
                Jr_fingers(count, 2) = df.dr2;                                     
                Jtheta(count,1) = df.dc2*cross(R*[1; 0; 0], centers{15} - centers{14});
                Jtheta(count,2) = df.dc2*cross(R*[0; 1; 0], centers{15} - centers{14}); 
                Jbeta(count, 1) = df.dc2*R*[0; 0; 1]; 
            end

             if (i3 == 13)
                Jr(count, i3) = df.dr3;
                Jc(count, D * i3 - D + 1:D * i3) = df.dc3*T;
            elseif (i3 == 14)
                Jr_fingers(count, 1) = df.dr3;
                Jc_fingers(count, 1:3) = df.dc3*T; 
            elseif (i3 == 15)
                Jr_fingers(count, 2) = df.dr3;                                     
                Jtheta(count,1) = df.dc3*cross(R*[1; 0; 0], centers{15} - centers{14});
                Jtheta(count,2) = df.dc3*cross(R*[0; 1; 0], centers{15} - centers{14}); 
                Jbeta(count, 1) = df.dc3*R*[0; 0; 1]; 
            end
            
            F(count) = f;
            count = count + 1;
        end
    end


