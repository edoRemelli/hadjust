function [f, df] = cross_derivative(a, da, b, db)

f = cross(a, b);
df = my_cross(a, db) - my_cross(b, da);
