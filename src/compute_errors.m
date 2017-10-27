function [ error, error_r_f, error_r_p, error_c_f, error_c_p, error_beta ] = compute_errors( hand_model, target_hand_model )

error_r_f = 0;
error_r_p = 0;
error_c_f = 0;
error_c_p = 0;
error_beta = 0;

% palm centers
for i = 1 : length ( hand_model.palm_wrist_centers_relative) 
    error_c_p = error_c_p + norm(hand_model.palm_wrist_centers_relative{i} - target_hand_model.palm_wrist_centers_relative{i})^2;
end

% finger centers
for i = 1 : length ( hand_model.segments) 
    error_c_f = error_c_f + norm(hand_model.segments{i}{1}.global(1:3,4) - target_hand_model.segments{i}{1}.global(1:3,4))^2;
end

for i = 1:length(hand_model.palm_wrist_radii)
    error_r_p = error_r_p + ( hand_model.palm_wrist_radii{i} - target_hand_model.palm_wrist_radii{i})^2;
end

for i = 1:length(hand_model.finger_radii)
    error_r_f = error_r_f + ( hand_model.finger_radii{i} - target_hand_model.finger_radii{i})^2;
end

for i = 1:length(hand_model.beta)
    error_beta = error_beta + norm( hand_model.beta{i} - target_hand_model.beta{i}) ^ 2;
end

error = error_c_p + error_c_f + error_r_p + error_r_f + error_beta;

error = sqrt(error);
error_c_p = sqrt(error_c_p);
error_c_f = sqrt(error_c_f);
error_r_p = sqrt(error_r_p);
error_r_f = sqrt(error_r_f);
error_beta = sqrt(error_beta);




