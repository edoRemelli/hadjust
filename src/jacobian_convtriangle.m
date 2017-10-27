function [f, df] = jacobian_convtriangle(p, v1, v2, v3, Jv1, Jv2, Jv3, variables)

D = length(p);

for v = 1:length(variables)
    variable = variables{v};
    switch variable
        case 'c1'
            dv1 = Jv1.dc1; dv2 = Jv2.dc1; dv3 = Jv3.dc1; dp =  zeros(D, D);
        case 'c2'
            dv1 = Jv1.dc2; dv2 = Jv2.dc2; dv3 = Jv3.dc2; dp =  zeros(D, D);
        case 'c3'
            dv1 = Jv1.dc3; dv2 = Jv2.dc3; dv3 = Jv3.dc3; dp =  zeros(D, D);
        case 'r1'
            dv1 = Jv1.dr1; dv2 = Jv2.dr1; dv3 = Jv3.dr1; dp =  zeros(D, 1);
        case 'r2'
            dv1 = Jv1.dr2; dv2 = Jv2.dr2; dv3 = Jv3.dr2; dp =  zeros(D, 1);
        case 'r3'
            dv1 = Jv1.dr3; dv2 = Jv2.dr3; dv3 = Jv3.dr3; dp =  zeros(D, 1);
    end
    
    % m = cross(v1 - v2, v1 - v3);
    [O1, dO1] = difference_derivative(v1, dv1, v2, dv2);
    [O2, dO2] = difference_derivative(v1, dv1, v3, dv3);
    [m, dm] = cross_derivative(O1, dO1, O2, dO2);
    
    % m = m / norm(m);
    [m, dm] = normalize_derivative(m, dm);
    
    % distance = (p - v1)' * m;
    [O1, dO1] = difference_derivative(p, dp, v1, dv1);
    [distance, ddistance] = dot_derivative(O1, dO1, m, dm);
    
    % t = p - distance * m;
    [O1, dO1] = product_derivative(distance, ddistance, m, dm);
    [q, dq] = difference_derivative(p, dp, O1, dO1);
    f = q;
    switch variable
        case 'c1', df.dc1 = dq;
        case 'c2', df.dc2 = dq;
        case 'c3', df.dc3 = dq;            
        case 'r1', df.dr1 = dq;
        case 'r2', df.dr2 = dq;
        case 'r3', df.dr3 = dq; 
    end     
end












