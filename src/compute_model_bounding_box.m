function model_bounding_box = compute_model_bounding_box(centers, radii)
RAND_MAX = 32767;
D = length(centers{1});
model_bounding_box.min_x = RAND_MAX;
model_bounding_box.max_x = -RAND_MAX;
model_bounding_box.min_y = RAND_MAX;
model_bounding_box.max_y = -RAND_MAX;
if D == 3
    model_bounding_box.min_z = RAND_MAX;
    model_bounding_box.max_z = -RAND_MAX;
end

for i = 1:length(centers)
    if centers{i}(1) - radii{i} < model_bounding_box.min_x
        model_bounding_box.min_x = centers{i}(1) - radii{i};
    end
    if centers{i}(2) - radii{i} < model_bounding_box.min_y
        model_bounding_box.min_y = centers{i}(2) - radii{i};
    end
    if centers{i}(1) + radii{i} > model_bounding_box.max_x
        model_bounding_box.max_x = centers{i}(1) + radii{i};
    end
    if centers{i}(2) + radii{i} > model_bounding_box.max_y
        model_bounding_box.max_y = centers{i}(2) + radii{i};
    end
    if D == 3
        if centers{i}(3) - radii{i} < model_bounding_box.min_z
            model_bounding_box.min_z = centers{i}(3) - radii{i};
        end
        if centers{i}(3) + radii{i} > model_bounding_box.max_z
            model_bounding_box.max_z = centers{i}(3) + radii{i};
        end
    end
end