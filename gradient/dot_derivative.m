function [f, df]  = dot_derivative(a, da, b, db)

f = a' * b;
df = a' * db + b' * da;

