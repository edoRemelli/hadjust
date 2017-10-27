function [df] = my_gradient(f, x)

e = 10e-10;

%% If f is a scalar function

if isscalar(f(x))
    
    df = zeros(size(x'));
    
    for i = 1:length(x)
        delta_x = zeros(size(x));
        delta_x(i) = e;
        df(i) = (f(x + delta_x) - f(x - delta_x))/ (2 * e);
    end
    
%% If f is a vector function
    
else
    
    df = zeros(length(f(x)), length(x));
    
    for i = 1:length(x)
        delta_x = zeros(size(x));
        delta_x(i) = e;
        df(:, i) = (f(x + delta_x) - f(x - delta_x))/ (2 * e);
    end
    
end