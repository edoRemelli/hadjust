function [] = display_model(hand_model, face_alpha, figure_mode, varargin)


centers = hand_model.palm_wrist_centers;
centers{end+1} = hand_model.fold_center;

%convert segments into centers
for i = 1:length(hand_model.segments)
    finger_segments =  hand_model.segments{i};
    for j = 1:length(finger_segments)
        centers{end+1} = finger_segments{j}.global(1:3, 4);
    end
end

for i = 1:length(hand_model.membrane_centers)
    centers{end+1} = hand_model.membrane_centers{i}; 
end

radii = {};
 for i = 1: length(hand_model.palm_wrist_radii)
    radii{end +1} = hand_model.palm_wrist_radii{i};
 end
radii{end+1} = hand_model.fold_radii;
 
 for i = 1: length(hand_model.finger_radii)
    radii{end +1} = hand_model.finger_radii{i};
 end
 for i = 1: length(hand_model.membrane_radii)
    radii{end +1} = hand_model.membrane_radii{i};
 end
blocks = hand_model.blocks(3:end);


% Generating the volumetric domain data:
n = 75; 
color = double([240; 189; 157]./255);

if ~isempty(varargin) 
    color = varargin{1};
end


model_bounding_box = compute_model_bounding_box(centers, radii);
xm = linspace(model_bounding_box.min_x, model_bounding_box.max_x, n);
ym = linspace(model_bounding_box.min_y, model_bounding_box.max_y, n);
zm = linspace(model_bounding_box.min_z, model_bounding_box.max_z, n);
[x, y, z] = meshgrid(xm,ym,zm);
N = numel(x);
P = [reshape(x, N, 1), reshape(y, N, 1), reshape(z, N, 1)];
distances = zeros(N, 1);

if strcmp(figure_mode, 'small')
    figure; hold on;
end
if strcmp(figure_mode, 'big')
    figure('units','normalized','outerposition',[0.0 0.085 1 0.873]); hold on;
end
set(gcf,'color','w');

tangent_points = blocks_tangent_points(centers, blocks, radii);
RAND_MAX = 32767;
min_distances = RAND_MAX * ones(N, 1);

for i = 1:length(blocks)
    if length(blocks{i}) == 3
        c1 = centers{blocks{i}(1)}; c2 = centers{blocks{i}(2)}; c3 = centers{blocks{i}(3)};
        r1 = radii{blocks{i}(1)}; r2 = radii{blocks{i}(2)}; r3 = radii{blocks{i}(3)};
        v1 = tangent_points{i}.v1; v2 = tangent_points{i}.v2; v3 = tangent_points{i}.v3;
        u1 = tangent_points{i}.u1; u2 = tangent_points{i}.u2; u3 = tangent_points{i}.u3;
        distances = distance_to_model_convtriangle(c1, c2, c3, r1, r2, r3, v1, v2, v3, u1, u2, u3, P');
    end
    
    if length(blocks{i}) == 2
        c1 = centers{blocks{i}(1)}; c2 = centers{blocks{i}(2)};
        r1 = radii{blocks{i}(1)}; r2 = radii{blocks{i}(2)};
        distances = distance_to_model_convsegment(c1, c2, r1, r2, P');
    end
    if length(blocks{i}) == 1
        c1 = centers{blocks{i}(1)};
        r1 = radii{blocks{i}(1)};
        distances = distance_to_model_sphere(c1, r1, P');
    end
    min_distances = min(min_distances, distances);
end

%% Making the 3D graph of the 0-level surface of the 4D function "fun":
min_distances = reshape(min_distances, size(x));
h = patch(isosurface(x, y, z, min_distances,0));
isonormals(x, y, z, min_distances, h);

% FaceAplha is used for transparency
set(h,'FaceColor',color,'EdgeColor','none', 'FaceAlpha', face_alpha);
material([0.7, 0.35, 0.05, 5, 0.1]); 

grid off;
axis equal;
lighting gouraud;
axis off; 

%camzoom(1.5)

view([0,0])
if ~strcmp(figure_mode, 'none')
    camlight; 
end

end
