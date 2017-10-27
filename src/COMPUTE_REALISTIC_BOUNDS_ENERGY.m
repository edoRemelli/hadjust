function [F,Jc_f,Jc_p] = COMPUTE_REALISTIC_BOUNDS_ENERGY(hand_model, bound_factor ,weight)


num_constrains = 4;
F  = zeros(num_constrains, 1);
Jc_f = zeros(num_constrains, 15);
Jc_p = zeros(num_constrains, 39);

indices_palm = [hand_model.palm_wrist_names_map('palm_index'),hand_model.palm_wrist_names_map('palm_middle'),hand_model.palm_wrist_names_map('palm_ring'),hand_model.palm_wrist_names_map('palm_pinky')];

%% compute bounds for beta

for i = 1:4

    c_p = hand_model.palm_wrist_centers_relative{indices_palm(i)};
    c_f = hand_model.segments{i+1}{1}.local(1:3,4);

    
    
    residual =  c_f(2)-bound_factor*c_p(2);
    
    % indicator function
    if (residual <= 0)
            Jc_f(i,3*i + 2) = weight;
            Jc_p(i,3*(indices_palm(i)-1) +2) =-bound_factor*weight;
            F(i) = -weight*residual;
    end
            
end

    
