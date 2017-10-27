function [f, df]  = sqrt_derivative(s, ds)

f = sqrt(s);
df = ds / 2 / sqrt(s);

