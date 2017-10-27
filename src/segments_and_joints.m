function [segments, joints] = segments_and_joints(beta,base_center,is_thumb)

%% Segments
segments{1}.shape_chain = 1;
segments{1}.kinematic_chain = 1;
segments{1}.length = beta(1);
segments{1}.parent_id = 0;
segments{1}.joints_id = {1,2};
segments{1}.global = eye(4, 4);
segments{1}.global(1:3,4) = base_center;
segments{1}.local = eye(4, 4);
segments{1}.local(1:3,4) = base_center;

segments{2}.shape_chain = 1:2;
segments{2}.kinematic_chain = 1:2;
segments{2}.length = beta(2);
segments{2}.parent_id = 1;
segments{2}.joints_id = {3};
segments{2}.local = eye(4, 4);
segments{2}.local(3, 4) = segments{segments{2}.parent_id}.length;

segments{3}.shape_chain = 1:3;
segments{3}.kinematic_chain = 1:3;
segments{3}.length = beta(3);
segments{3}.parent_id = 2;
segments{3}.joints_id = {4};
segments{3}.local = eye(4, 4);
segments{3}.local(3, 4) = segments{segments{3}.parent_id}.length;

if (is_thumb)

segments{4}.shape_chain = 1:4;
segments{4}.kinematic_chain = 1:4;
segments{4}.length = beta(4);
segments{4}.parent_id = 3;
segments{4}.joints_id = {};
segments{4}.local = eye(4, 4);
segments{4}.local(3, 4) = segments{segments{4}.parent_id}.length;

segments{5}.kinematic_chain = 1:4;
segments{5}.length = 0;
segments{5}.parent_id = 4;
segments{5}.joints_id = {};
segments{5}.local = eye(4, 4);
segments{5}.local(3, 4) = segments{segments{5}.parent_id}.length;
       
else
 
segments{4}.kinematic_chain = 1:3;
segments{4}.length = 0;
segments{4}.parent_id = 3;
segments{4}.joints_id = {};
segments{4}.local = eye(4, 4);
segments{4}.local(3, 4) = segments{segments{4}.parent_id}.length;

end

%% Joints

joints{1}.segment_id = 1;
joints{1}.axis = [1; 0; 0];  
joints{1}.type = 'R';

joints{2}.segment_id = 1;
joints{2}.axis = [0; 1; 0];  
joints{2}.type = 'R';

joints{3}.segment_id = 2;
joints{3}.axis = [1; 0; 0];
joints{3}.type = 'R';

joints{4}.segment_id = 3;
joints{4}.axis = [1; 0; 0]; 
joints{4}.type = 'R';

joints{5}.segment_id = 4;
joints{5}.axis = [1; 0; 0];  
joints{5}.type = 'R';

if (is_thumb)
    joints{6}.segment_id = 5;
    joints{6}.axis = [1; 0; 0];  
    joints{6}.type = 'R';
end

