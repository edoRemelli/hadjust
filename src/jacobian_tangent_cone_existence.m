function [f, df] = jacobian_tangent_cone_existence(c1, c2, r1, r2, factor, variables)

D = length(c1);

c1 = c1;
c2 = c2;
r1 = r1;
r2 = r2;
dfactor = 0;

for var = 1:length(variables)
    variable = variables{var};
    switch variable
        case 'c1', dc1 = eye(D, D); dc2 = zeros(D, D);
            dr1 = zeros(1, D); dr2 = zeros(1, D);
        case 'c2',  dc1 = zeros(D, D); dc2 = eye(D, D);
            dr1 = zeros(1, D); dr2 = zeros(1, D);
        case 'r1', dc1 = zeros(D, 1); dc2 = zeros(D, 1);
            dr1 = 1; dr2 = 0;
        case 'r2', dc1 = zeros(D, 1); dc2 = zeros(D, 1);
            dr1 = 0; dr2 = 1;
    end
    
    %% norm(c1 - c2) - factor * (r1 - r2) = 0;
    [a, da] = difference_derivative(c1, dc1, c2, dc2);
    [b, db] = norm_derivative(a, da);
    [c, dc] = difference_derivative(r1, dr1, r2, dr2);
    [d, dd] = product_derivative(factor, dfactor, c, dc);
    [r, dr] = difference_derivative(b, db, d, dd);
    
    f = r;
    switch variable
        case 'c1', df.dc1 = dr;
        case 'c2', df.dc2 = dr;
        case 'r1', df.dr1 = dr;
        case 'r2', df.dr2 = dr;
    end
end
%O(r2)
%Jnumerical
%Janalytical
