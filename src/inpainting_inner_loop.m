function [u, v] = inpainting_inner_loop(I, h, w)
[H, W] = size(I);
for offset = 1:3
    for u = -offset:offset
        for v = -offset:offset
            if (abs(u) < offset && abs(v) < offset), continue; end
            if h + u < 1 || h + u > H || w + v < 1 || w + v > W, continue; end
            if I(h + u, w + v) ~= 0
                return
            end
        end
    end
end

u = 0; v = 0;