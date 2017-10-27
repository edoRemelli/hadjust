function [] = mylines(first_points, second_points, color)

if isempty(first_points) || isempty(second_points), return; end

D = 1;
for i = 1:length(first_points)
    if ~isempty(first_points{i})
        D = length(first_points{i});
        break;
    end
end

if D == 3
    P = zeros(length(first_points), 3);
    Q = zeros(length(first_points), 3);
    L = zeros(length(first_points) * 3, 3);
    k = 0;
    for i = 1:length(first_points)
        if ~isempty(first_points{i}) && ~isempty(second_points{i})
            k = k + 1;
            P(k, :) =  first_points{i}';
            Q(k, :) =  second_points{i}';
            L(3 * (k - 1) + 1, :) = first_points{i}';
            L(3 * (k - 1) + 2, :) = second_points{i}';
            L(3 * (k - 1) + 3, :) = [NaN, NaN, NaN];
        end
    end
    if (k > 0)
        L = L(1:3*k, :);
        line(L(1:3*k, 1), L(1:3*k, 2), L(1:3*k, 3), 'lineWidth', 2, 'color', color);
    end
end

if D == 2
    P = zeros(length(first_points), 2);
    Q = zeros(length(first_points), 2);
    L = zeros(length(first_points) * 3, 2);
    k = 0;
    for i = 1:length(first_points)
        if ~isempty(first_points{i}) && ~isempty(second_points{i})
            k = k + 1;
            P(k, :) =  first_points{i}';
            Q(k, :) =  second_points{i}';
            L(3 * (k - 1) + 1, :) = first_points{i}';
            L(3 * (k - 1) + 2, :) = second_points{i}';
            L(3 * (k - 1) + 3, :) = [NaN, NaN];
        end
    end
    if (k > 0)
        L = L(1:3*k, :);
        line(L(1:3*k, 1), L(1:3*k, 2), 'lineWidth', 2, 'color', color);
    end
end