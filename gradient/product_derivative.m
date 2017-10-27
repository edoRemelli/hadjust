function [f, df] = product_derivative(s, ds, a, da)

f  = s * a;
df = a * ds + s * da;