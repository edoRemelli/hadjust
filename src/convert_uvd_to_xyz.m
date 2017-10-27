function [xyz] = convert_uvd_to_xyz(tx, ty, fx, fy, uvd)

uvd = double(uvd);

P = [fx 0   tx;
    0  fy  -ty;
    0  0   1];

Pinv = [1/fx   0      -tx/fx;
        0       1/fy  -ty/fy;
        0       0       1];

d = uvd(3, :);
uvd(3, :) = uvd(3, :) ./ uvd(3, :);

XYZ = Pinv * uvd;

z = d;
x = XYZ(1, :) .* z;
y = -XYZ(2, :) .* z;

xyz = [x; y; z];

% R = makehgtform('axisrotate', [0;0;1], pi)*makehgtform('axisrotate', [1;0;0], pi/2);
% R = R(1:3,1:3);
% xyz = (R * xyz);
% 
% xyz = double(xyz);


%xyz = [x,y,z];
%xyz = [-x; z; y];

