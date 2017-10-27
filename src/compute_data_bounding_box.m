function [data_bounding_box] = compute_data_bounding_box(points)
RAND_MAX = 32767;
D = length(points{1});
data_bounding_box.min_x = RAND_MAX;
data_bounding_box.min_y = RAND_MAX;
data_bounding_box.max_x = -RAND_MAX;
data_bounding_box.max_y = -RAND_MAX;

if D == 3
    data_bounding_box.min_z = RAND_MAX;
    data_bounding_box.max_z = -RAND_MAX;
end

for i = 1:length(points)
    if points{i}(1) < data_bounding_box.min_x
        data_bounding_box.min_x = points{i}(1);
    end
    if points{i}(1) > data_bounding_box.max_x
        data_bounding_box.max_x = points{i}(1);
    end
    if points{i}(2) <data_bounding_box. min_y
        data_bounding_box.min_y = points{i}(2);
    end
    if points{i}(2) > data_bounding_box.max_y
        data_bounding_box.max_y = points{i}(2);
    end
    if D == 3
        if points{i}(3) > data_bounding_box.max_z
            data_bounding_box.max_z = points{i}(3);
        end
        if points{i}(3) < data_bounding_box.min_z
            data_bounding_box.min_z = points{i}(3);
        end
    end    
end