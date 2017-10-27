function [is_inside] = test_insideness(p, q, s)
if  norm(p - s) - norm(q - s) > 10e-7
    is_inside = false;
else
    is_inside = true;
end
