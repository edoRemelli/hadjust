function [f, df] = normalize_derivative(a, da)


f = a / sqrt(a' * a);
df = (da * sqrt(a' * a) - a * a' * da / sqrt(a' * a)) /  (a' * a);