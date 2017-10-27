function [F,Jtheta] = COMPUTE_BOUNDS_ENERGY(hand_model, settings)


num_constrains = length(hand_model.theta)*length(hand_model.theta{1});
F  = zeros(num_constrains, 1);
Jtheta = zeros(num_constrains, num_constrains);
factor = settings.theta_factor;

% COULD NOT FIND VALUES ON PAPER so guessing them
THETA_HIGH = { -[-0.3;-1.1;-0.35;-0.35],-[-0.4;-0.5;-0.3;-0.3], -[-0.4;-0.5;-0.3;-0.3], -[-0.4;-0.2;-0.3;-0.3], -[-0.4;-0.3;-0.3;-0.3]} ;
THETA_LOW = { -[1.8;-0.2;1.70;1.50],-[1.0;+0.35;+pi/2;+pi/2], -[+pi/2;+0.25;+pi/2;+pi/2], -[+pi/2;+0.3;+pi/2;+pi/2], -[+pi/2;+0.6;+pi/2;+pi/2]}; 

for s = 1:length(hand_model.theta)
    
    theta = hand_model.theta{s};
    
    for i = 1: length(theta)
        
        % LOWER BOUND
        r_low = theta(i) - THETA_LOW{s}(i)-factor;
        r_high = theta(i) - THETA_HIGH{s}(i)+factor;
        
        if (r_low <= 0)
            Jtheta(4*(s-1) + i,4*(s-1) + i) = 1;
            F(4*(s-1) + i) = -r_low;
        elseif (r_high >= 0)
            Jtheta(4*(s-1) + i,4*(s-1) + i) = 1;
            F(4*(s-1) + i) = -r_high;
        end
            
    end
end
    
    
   
