function [f, df]  = ratio_derivative(a, da, b, db)

f = a  / b;
df =  (b * da - a * db) / b^2;