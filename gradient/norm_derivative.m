function [f, df]  = norm_derivative(a, da)

f = sqrt(a' * a);
df = a' * da / sqrt(a' * a);


