function c = my_cross(a, b)

c = zeros(size(b));
for i = 1:size(b, 2)
    c(:, i) = cross(a, b(:, i));
end

