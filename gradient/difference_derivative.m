function [f, df]  = difference_derivative(a, da, b, db)

f =  a - b;
df = da - db;