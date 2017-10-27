function [dcenter1b, dcenter2b,dcenter1, dcenter2, dcenter_m1, dcenter_m2, dcenter1_r, dcenter2_r,dcenter1b_r, dcenter2b_r, dcenter_m1_r, dcenter_m2_r ] = jacobian_membrane(P, c1b,c2b, beta1, beta2, R1, R2, r11, r12, r21, r22, s1, s2, variables)

D = length(c1b);
f1_rm = 3.0;

for v = 1:length(variables)
    variable = variables{v};
    switch variable
        case 'c1b'
            dc1b = eye(D); dc2b = zeros(D); dc1 = zeros(D); dc2 = zeros(D); dbeta1 = zeros(1,D); dbeta2 = zeros(1,D);dr1 = zeros(1,D); dr2 =  zeros(1,D); dr11 = zeros(1,D); dr12 =  zeros(1,D); dr21 = zeros(1,D); dr22 =  zeros(1,D); ds1 = zeros(1,D); ds2 =  zeros(1,D);
        case 'c2b'
            dc1b = zeros(D); dc2b = eye(D); dc1 = zeros(D); dc2 = zeros(D); dbeta1 = zeros(1,D); dbeta2 =  zeros(1,D);dr1 = zeros(1,D); dr2 =  zeros(1,D); dr11 = zeros(1,D); dr12 =  zeros(1,D); dr21 = zeros(1,D); dr22 =  zeros(1,D); ds1 = zeros(1,D); ds2 =  zeros(1,D);
        case 'c1'
            dc1b = zeros(D); dc2b = zeros(D); dc1 = eye(D); dc2 = zeros(D); dbeta1 = zeros(1,D); dbeta2 = zeros(1,D);dr1 = zeros(1,D); dr2 =  zeros(1,D); dr11 = zeros(1,D); dr12 =  zeros(1,D); dr21 = zeros(1,D); dr22 =  zeros(1,D); ds1 = zeros(1,D); ds2 =  zeros(1,D);
        case 'c2'
            dc1b = zeros(D); dc2b = zeros(D); dc1 = zeros(D); dc2 = eye(D); dbeta1 = zeros(1,D); dbeta2 =  zeros(1,D);dr1 = zeros(1,D); dr2 =  zeros(1,D); dr11 = zeros(1,D); dr12 =  zeros(1,D); dr21 = zeros(1,D); dr22 =  zeros(1,D); ds1 = zeros(1,D); ds2 =  zeros(1,D);        
        case 'beta1'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 1.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 =  0.0; ds1 = 0.0; ds2 =  0.0;
        case 'beta2'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1); dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  1.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 =  0.0; ds1 = 0.0; ds2 =  0.0;        
        case 'r1'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0; dr1 = 1.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 =  0.0; ds1 = 0.0; ds2 =  0.0;
        case 'r2'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0; dr1 = 0.0; dr2 =  1.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 = 0.0; ds1 = 0.0; ds2 =  0.0;        
        case 'r11'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 1.0; dr12 =  0.0; dr21 = 0.0; dr22 =  0.0; ds1 = 0.0; ds2 =  0.0;
        case 'r12'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  1.0; dr21 = 0.0; dr22 = 0.0; ds1 = 0.0; ds2 =  0.0;
        case 'r21'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 1.0; dr22 =  0.0; ds1 = 0.0; ds2 =  0.0;
        case 'r22'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 =  1.0; ds1 = 0.0; ds2 =  0.0;
        case 's1'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 =  0.0; ds1 = 1.0; ds2 =  0.0;
        case 's2'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 =  0.0; ds1 = 0.0; ds2 =  1.0;
        case 'theta11'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 =  0.0; ds1 = 0.0; ds2 =  0.0;
        case 'theta12'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 = 0.0; ds1 = 0.0; ds2 =  0.0;
        case 'theta21'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 =  0.0; ds1 = 0.0; ds2 =  0.0;
        case 'theta22'
            dc1b = zeros(D, 1); dc2b = zeros(D, 1);dc1 = zeros(D, 1); dc2 = zeros(D, 1); dbeta1 = 0.0; dbeta2 =  0.0;dr1 = 0.0; dr2 =  0.0; dr11 = 0.0; dr12 =  0.0; dr21 = 0.0; dr22 =  0.0; ds1 = 0.0; ds2 =  0.0;    
    end                  
            
    %% COMPUTE FIRST MEMBRANE CENTER   
    
    %f1_c2b = f1_l * f1_R(1:3,1:3)* [0;0;1] + f1_c1b ;    
    [ct1, dct1] = sum_derivative(beta1 * R1(1:3,1:3)* [0;0;1],   R1(1:3,1:3)* [0;0;1]* dbeta1 , c1b, dc1b);    
    %location = base + f1_s*(bottom - base);
    [l,dl] = difference_derivative(ct1, dct1, c1b, dc1b);
    [t,dt] = product_derivative(s1,ds1, l,dl);
    [l,dl] = sum_derivative(c1b,dc1b,t,dt); 
    
    
    u1 = R1(1:3,1:3)*[0;1;0];
    u2 = P(1:3,1:3)*[0;1;0];
    if (u1'*u2 <= 0)
        membrane_direction = [0;1;0];
    else
        membrane_direction = [0;-1;0];
    end
    
    d1  = R1(1:3,1:3)*membrane_direction;    
    %r_cm1 = (1-f1_s)*f1_r1 + f1_s*f1_r2;    
    [a,da] = product_derivative ( (1-s1), -ds1, r11, dr11);
    [b,db] = product_derivative ( s1,ds1, r12 , dr12);   
    [r,dr] = sum_derivative( a,da,b,db);
    %f1_cm = location + d1* (location_ray-f1_rm);    
    [cm1,dcm1] = sum_derivative(l,dl, d1* (r-f1_rm), d1*dr);
    
    
    %% COMPUTE SECOND MEMBRANE CENTER
    
    %f1_c2b = f1_l * f1_R(1:3,1:3)* [0;0;1] + f1_c1b ;    
    [ct2, dct2] = sum_derivative(beta2 * R2(1:3,1:3)* [0;0;1], dbeta2 * R2(1:3,1:3)* [0;0;1], c2b, dc2b);    
    %location = base + f1_s*(bottom - base);
    [l,dl] = difference_derivative(ct2, dct2, c2b, dc2b);
    [t,dt] = product_derivative(s2,ds2, l,dl);
    [l,dl] = sum_derivative(c2b,dc2b,t,dt); 
    
    u1 = R2(1:3,1:3)*[0;1;0];
    u2 = P(1:3,1:3)*[0;1;0];
    if (u1'*u2 <= 0)
        membrane_direction = [0;1;0];
    else
        membrane_direction = [0;-1;0];
    end
    
    d2  = R2(1:3,1:3)*membrane_direction;    
    %r_cm1 = (1-f1_s)*f1_r1 + f1_s*f1_r2;    
    [a,da] = product_derivative ( (1-s2), -ds2, r21, dr21);
    [b,db] = product_derivative ( s2,ds2, r22 , dr22);   
    [r,dr] = sum_derivative( a,da,b,db);
    %f1_cm = location + d1* (location_ray-f1_rm);    
    [cm2,dcm2] = sum_derivative(l,dl, d2* (r-f1_rm), d2*dr);

    switch variable
        case 'c1b'
            dcenter1b.dc1b = dc1b; dcenter2b.dc1b = dc2b; dcenter1.dc1b = dc1; dcenter2.dc1b = dc2; dcenter_m1.dc1b = dcm1; dcenter_m2.dc1b = dcm2;dcenter1_r.dc1b = dr1; dcenter2_r.dc1b = dr2; dcenter1b_r.dc1b = dr11; dcenter2b_r.dc1b = dr21; dcenter_m1_r.dc1b = zeros(1,D); dcenter_m2_r.dc1b = zeros(1,D);
        case 'c2b'
            dcenter1b.dc2b = dc1b; dcenter2b.dc2b = dc2b;dcenter1.dc2b = dc1; dcenter2.dc2b = dc2; dcenter_m1.dc2b = dcm1; dcenter_m2.dc2b = dcm2;dcenter1_r.dc2b = dr1; dcenter2_r.dc2b = dr2;  dcenter1b_r.dc2b = dr11; dcenter2b_r.dc2b = dr21; dcenter_m1_r.dc2b = zeros(1,D); dcenter_m2_r.dc2b = zeros(1,D);
        case 'c1'
            dcenter1b.dc1 = dc1b; dcenter2b.dc1 = dc2b; dcenter1.dc1 = dc1; dcenter2.dc1 = dc2; dcenter_m1.dc1 = dcm1; dcenter_m2.dc1 = dcm2;dcenter1_r.dc1 = dr1; dcenter2_r.dc1 = dr2; dcenter1b_r.dc1 = dr11; dcenter2b_r.dc1 = dr21; dcenter_m1_r.dc1 = zeros(1,D); dcenter_m2_r.dc1 = zeros(1,D);
        case 'c2'
            dcenter1b.dc2 = dc1b; dcenter2b.dc2 = dc2b;dcenter1.dc2 = dc1; dcenter2.dc2 = dc2; dcenter_m1.dc2 = dcm1; dcenter_m2.dc2 = dcm2;dcenter1_r.dc2 = dr1; dcenter2_r.dc2 = dr2;  dcenter1b_r.dc2 = dr11; dcenter2b_r.dc2 = dr21; dcenter_m1_r.dc2 = zeros(1,D); dcenter_m2_r.dc2 = zeros(1,D);       
        case 'beta1'
            dcenter1b.dbeta1 = dc1b; dcenter2b.dbeta1 = dc2b;dcenter1.dbeta1 = dc1; dcenter2.dbeta1 = dc2; dcenter_m1.dbeta1 = dcm1; dcenter_m2.dbeta1 = dcm2;dcenter1_r.dbeta1 = dr1; dcenter2_r.dbeta1 = dr2;   dcenter1b_r.dbeta1 = dr11; dcenter2b_r.dbeta1 = dr21; dcenter_m1_r.dbeta1 = 0.0; dcenter_m2_r.dbeta1 = 0.0; 
        case 'beta2'
            dcenter1b.dbeta2 = dc1b; dcenter2b.dbeta2 = dc2b;dcenter1.dbeta2 = dc1; dcenter2.dbeta2 = dc2; dcenter_m1.dbeta2 = dcm1; dcenter_m2.dbeta2 = dcm2;dcenter1_r.dbeta2 = dr1; dcenter2_r.dbeta2 = dr2;  dcenter1b_r.dbeta2 = dr11; dcenter2b_r.dbeta2 = dr21; dcenter_m1_r.dbeta2 = 0.0; dcenter_m2_r.dbeta2 = 0.0;  
        case 'r11'
            dcenter1b.dr11 = dc1b; dcenter2b.dr11 = dc2b;dcenter1.dr11 = dc1; dcenter2.dr11 = dc2; dcenter_m1.dr11 = dcm1; dcenter_m2.dr11 = dcm2;dcenter1_r.dr11 = dr1; dcenter2_r.dr11 = dr2;  dcenter1b_r.dr11 = dr11; dcenter2b_r.dr11 = dr21; dcenter_m1_r.dr11 = 0.0; dcenter_m2_r.dr11 = 0.0;  
        case 'r12'
            dcenter1b.dr12 = dc1b; dcenter2b.dr12 = dc2b;dcenter1.dr12 = dc1; dcenter2.dr12 = dc2; dcenter_m1.dr12 = dcm1; dcenter_m2.dr12 = dcm2;dcenter1_r.dr12 = dr1; dcenter2_r.dr12 = dr2;   dcenter1b_r.dr12 = dr11; dcenter2b_r.dr12 = dr21; dcenter_m1_r.dr12 = 0.0; dcenter_m2_r.dr12 = 0.0; 
       case 'r1'
            dcenter1b.dr1 = dc1b; dcenter2b.dr1 = dc2b;dcenter1.dr1 = dc1; dcenter2.dr1 = dc2; dcenter_m1.dr1 = dcm1; dcenter_m2.dr1 = dcm2;dcenter1_r.dr1 = dr1; dcenter2_r.dr1 = dr2;  dcenter1b_r.dr1 = dr11; dcenter2b_r.dr1 = dr21; dcenter_m1_r.dr1 = 0.0; dcenter_m2_r.dr1 = 0.0;  
        case 'r2'
            dcenter1b.dr2 = dc1b; dcenter2b.dr2 = dc2b;dcenter1.dr2 = dc1; dcenter2.dr2 = dc2; dcenter_m1.dr2 = dcm1; dcenter_m2.dr2 = dcm2;dcenter1_r.dr2 = dr1; dcenter2_r.dr2 = dr2;   dcenter1b_r.dr2 = dr11; dcenter2b_r.dr2 = dr21; dcenter_m1_r.dr2 = 0.0; dcenter_m2_r.dr2 = 0.0;        
        case 'r21'
            dcenter1b.dr21 = dc1b; dcenter2b.dr21 = dc2b;dcenter1.dr21 = dc1; dcenter2.dr21 = dc2; dcenter_m1.dr21 = dcm1; dcenter_m2.dr21 = dcm2;dcenter1_r.dr21 = dr1; dcenter2_r.dr21 = dr2;   dcenter1b_r.dr21 = dr11; dcenter2b_r.dr21 = dr21; dcenter_m1_r.dr21 = 0.0; dcenter_m2_r.dr21 = 0.0; 
        case 'r22'
            dcenter1b.dr22 = dc1b; dcenter2b.dr22 = dc2b;dcenter1.dr22 = dc1; dcenter2.dr22 = dc2; dcenter_m1.dr22 = dcm1; dcenter_m2.dr22 = dcm2;dcenter1_r.dr22 = dr1; dcenter2_r.dr22 = dr2;    dcenter1b_r.dr22 = dr11; dcenter2b_r.dr22 = dr21; dcenter_m1_r.dr22 = 0.0; dcenter_m2_r.dr22 = 0.0;
        case 's1'
            dcenter1b.ds1 = dc1b; dcenter2b.ds1 = dc2b;dcenter1.ds1 = dc1; dcenter2.ds1 = dc2; dcenter_m1.ds1 = dcm1; dcenter_m2.ds1 = dcm2;dcenter1_r.ds1 = dr1; dcenter2_r.ds1 = dr2;    dcenter1b_r.ds1 = dr11; dcenter2b_r.ds1 = dr21; dcenter_m1_r.ds1 = 0.0; dcenter_m2_r.ds1 = 0.0;
        case 's2'
            dcenter1b.ds2 = dc1b; dcenter2b.ds2 = dc2b;dcenter1.ds2 = dc1; dcenter2.ds2 = dc2; dcenter_m1.ds2 = dcm1; dcenter_m2.ds2 = dcm2;dcenter1_r.ds2 = dr1; dcenter2_r.ds2 = dr2;    dcenter1b_r.ds2 = dr11; dcenter2b_r.ds2 = dr21; dcenter_m1_r.ds2 = 0.0; dcenter_m2_r.ds2 = 0.0;
        case 'theta11'
            dcenter1b.dtheta11 = dc1b; dcenter2b.dtheta11 = dc2b;dcenter1.dtheta11 = dc1; dcenter2.dtheta11 = dc2; dcenter_m1.dtheta11 = dcm1 + cross(R1(1:3,1:3)*[1; 0; 0], cm1 - c1b); dcenter_m2.dtheta11 = dcm2;dcenter1_r.dtheta11 = dr1; dcenter2_r.dtheta11 = dr2;  dcenter1b_r.dtheta11 = dr11; dcenter2b_r.dtheta11 = dr21; dcenter_m1_r.dtheta11 = 0.0; dcenter_m2_r.dtheta11 = 0.0;
        case 'theta12'
            dcenter1b.dtheta12 = dc1b; dcenter2b.dtheta12 = dc2b;dcenter1.dtheta12 = dc1; dcenter2.dtheta12 = dc2; dcenter_m1.dtheta12 = dcm1 + cross(R1(1:3,1:3)*[0; 1; 0], cm1 - c1b); dcenter_m2.dtheta12 = dcm2;dcenter1_r.dtheta12 = dr1; dcenter2_r.dtheta12 = dr2;  dcenter1b_r.dtheta12 = dr11; dcenter2b_r.dtheta12 = dr21; dcenter_m1_r.dtheta12 = 0.0; dcenter_m2_r.dtheta12 = 0.0;
        case 'theta21'
            dcenter1b.dtheta21 = dc1b; dcenter2b.dtheta21 = dc2b;dcenter1.dtheta21 = dc1; dcenter2.dtheta21 = dc2; dcenter_m1.dtheta21 = dcm1 ; dcenter_m2.dtheta21 = dcm2 + cross(R2(1:3,1:3)*[1; 0; 0], cm2 - c2b);dcenter1_r.dtheta21 = dr1; dcenter2_r.dtheta21 = dr2;  dcenter1b_r.dtheta21 = dr11; dcenter2b_r.dtheta21 = dr21; dcenter_m1_r.dtheta21 = 0.0; dcenter_m2_r.dtheta21 = 0.0;
        case 'theta22'
            dcenter1b.dtheta22 = dc1b; dcenter2b.dtheta22 = dc2b;dcenter1.dtheta22 = dc1; dcenter2.dtheta22 = dc2; dcenter_m1.dtheta22 = dcm1 ; dcenter_m2.dtheta22 = dcm2 + cross(R2(1:3,1:3)*[0; 1; 0], cm2 - c2b);dcenter1_r.dtheta22 = dr1; dcenter2_r.dtheta22 = dr2;  dcenter1b_r.dtheta22 = dr11; dcenter2b_r.dtheta22 = dr21; dcenter_m1_r.dtheta22 = 0.0; dcenter_m2_r.dtheta22 = 0.0;
    end      

end
