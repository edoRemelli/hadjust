function [q] = project_point_on_line(p, c1, c2)
u = c2 - c1;
v = p - c1;

alpha = u' * v / (u' * u);
q = c1 + alpha * u;
