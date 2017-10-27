function [centers, radii, blocks, theta, phalanges, mean_centers] = read_cpp_model(path, varargin)

frame_number = 0;
if ~isempty(varargin)
    frame_number = varargin{1};
end

RAND_MAX = 32767;

%% Read centers
if ~isempty(varargin)
    fileID = fopen([path, 'C-', num2str(frame_number), '.txt'], 'r');
else
    fileID = fopen([path, 'C.txt'], 'r');
end
C = fscanf(fileID, '%f');
C = C(2:end);
C = reshape(C, 3, length(C)/3);
centers = cell(0, 1);
mean_centers = [0; 0; 0];
for i = 1:size(C, 2);
    centers{end + 1} = C(:, i);
end
%{
for i = 1:size(C, 2);
    centers{end + 1} = C(:, i);
    mean_centers = mean_centers + centers{end};
end
mean_centers = mean_centers ./ length(centers);
for i = 1:length(centers)
    centers{i} = centers{i} - mean_centers;
end
%}

%% Read radii
if ~isempty(varargin)
    fileID = fopen([path, 'R-', num2str(frame_number), '.txt'], 'r');
else
    fileID = fopen([path, 'R.txt'], 'r');
end
R = fscanf(fileID, '%f');
R = R(2:end);
radii = cell(0, 1);
for i = 1:length(R);
    radii{end + 1} = R(i);
end
%% Read blocks
if ~isempty(varargin)
    fileID = fopen([path, 'B-', num2str(frame_number), '.txt'], 'r');
else
    fileID = fopen([path, 'B.txt'], 'r');
end
B = fscanf(fileID, '%f');
B = B(2:end);
B = reshape(B, 3, length(B)/3);
blocks = cell(0, 1);
for i = 1:size(B, 2);
    if B(3, i) == RAND_MAX
        blocks{end + 1} = B(1:2, i) + 1;
    else
        blocks{end + 1} = B(:, i) + 1;
    end
end
blocks = reindex(radii, blocks);

%% Read theta
if exist([path, 'T.txt'], 'file')
    if ~isempty(varargin)
        fileID = fopen([path, 'T-', num2str(frame_number), '.txt'], 'r');
    else
        fileID = fopen([path, 'T.txt'], 'r');
    end
    T = fscanf(fileID, '%f');
    %T = textscan(fileID, ['%f' 'f,' ], 'HeaderLines',1);
    T = T(2:end);
    theta = zeros(length(T), 1);
    for i = 1:length(T);
        theta(i) = T(i);
    end
else
    disp('no thetas in the folder');
    theta = zeros(29, 1);
end

%% Read phalanges
if exist([path, 'I.txt'], 'file')
    fileID = fopen([path, 'I.txt'], 'r');
    I = fscanf(fileID, '%f');
    I = I(2:end);
    I = reshape(I, 16, length(I)/16)';
    [phalanges, ~] = hmodel_parameters();
    for i = 1:size(I, 1)
        M = reshape(I(i, :), 4, 4);
        if (M(4,2) == 0)
            phalanges{i}.local = M;
        else
            phalanges{i}.local = M';
        end
    end
else
    disp('no transformations in the folder');
    phalanges = [];
end