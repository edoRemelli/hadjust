function [f, df] = jacobian_sphere(p, c, r, variables)

D = length(p);

for v = 1:length(variables)
    variable = variables{v};
    switch variable
        case 'c1'
            dc = eye(D, D);
            dr = zeros(1, D);
            dp = zeros(D, D);
        case 'r1'
            dc = zeros(D, 1);
            dr = 1;
            dp = zeros(D, 1);
    end
    
    [m, dm] = difference_derivative(p, dp, c, dc);
    [n, dn] = normalize_derivative(m, dm);
    [l, dl] = product_derivative(r, dr, n, dn);
    [q, dq] = sum_derivative(c, dc, l, dl);
    f = q;
    
    switch variable
        case 'c1'  
            df.dc1 = dq;            
        case 'r1'        
            df.dr1 = dq;
    end
end



