%% SAVE CALIBRATED HAND TO FILE
clc
%clear
close all
%LOAD CALIBRATION OUTPUT
load('calib.mat');
%choose one it doesn't really matter which one
%hand_model = v_hand_model{1};
load('phalanges.mat');
load('data_path')

%% REPUT ORIGINALS TRANSFORMS ON FINGERS

Rx = @(alpha) [
    1, 0, 0;
    0, cos(alpha), -sin(alpha);
    0, sin(alpha), cos(alpha)];

Ry = @(alpha) [
    cos(alpha), 0, sin(alpha);
    0, 1, 0;
    -sin(alpha), 0, cos(alpha)];

Rz = @(alpha) [
    cos(alpha), -sin(alpha), 0;
    sin(alpha), cos(alpha), 0
    0, 0, 1];

% transform from my orientation to htrack's one
R = Rx(-pi/2)*Rz(-pi);
% transform from htrack's orientation to mine
Rinv = Rz(pi) * Rx(pi/2);

%READ INITIAL TRANSFORMATIONS
hand_model.segments{1}{1}.local(1:3,1:3) = Rinv* phalanges{2}.local(1:3,1:3)*R;
hand_model.segments{1}{2}.local(1:3,1:3) = Rinv* phalanges{3}.local(1:3,1:3)*R;
hand_model.segments{1}{3}.local(1:3,1:3) = Rinv* phalanges{4}.local(1:3,1:3)*R;

hand_model.segments{5}{1}.local(1:3,1:3) = Rinv*  phalanges{5}.local(1:3,1:3)*R;
hand_model.segments{5}{2}.local(1:3,1:3) =  Rinv*  phalanges{6}.local(1:3,1:3)*R;
hand_model.segments{5}{3}.local(1:3,1:3) =  Rinv*  phalanges{7}.local(1:3,1:3)*R;

hand_model.segments{4}{1}.local(1:3,1:3) = Rinv*  phalanges{8}.local(1:3,1:3)*R;
hand_model.segments{4}{2}.local(1:3,1:3) =  Rinv*  phalanges{9}.local(1:3,1:3)*R;
hand_model.segments{4}{3}.local(1:3,1:3) =  Rinv*  phalanges{10}.local(1:3,1:3)*R;

hand_model.segments{3}{1}.local(1:3,1:3) = Rinv*  phalanges{11}.local(1:3,1:3)*R;
hand_model.segments{3}{2}.local(1:3,1:3) =  Rinv*  phalanges{12}.local(1:3,1:3)*R;
hand_model.segments{3}{3}.local(1:3,1:3) =  Rinv*  phalanges{13}.local(1:3,1:3)*R;

hand_model.segments{2}{1}.local(1:3,1:3) = Rinv*  phalanges{14}.local(1:3,1:3)*R;
hand_model.segments{2}{2}.local(1:3,1:3) =  Rinv*  phalanges{15}.local(1:3,1:3)*R;
hand_model.segments{2}{3}.local(1:3,1:3) =  Rinv*  phalanges{16}.local(1:3,1:3)*R;


%% READ RELATIVE CENTERS,RADII FROM OUR MODEL
centers = hand_model.palm_wrist_centers_relative;
centers{end+1} = hand_model.segments{1}{1}.local(1:3,4) + hand_model.segments{1}{1}.local(1:3,1:3)*hand_model.fold_offset;
%convert segments into centers
for i = 1:length(hand_model.segments)
    finger_segments =  hand_model.segments{i};
    LOCAL = eye(4);
    for j = 1:length(finger_segments)        
         LOCAL = LOCAL*finger_segments{j}.local;
         centers{end+1} = LOCAL(1:3, 4);
    end
end
for i = 1:length(hand_model.membrane_centers)
    s = hand_model.membrane_position{i};
    base = hand_model.segments{1+i}{1}.local(1:3,4);   
    bottom =  hand_model.segments{1+i}{1}.local*hand_model.segments{1+i}{2}.local;
    bottom=bottom(1:3,4);
    location = base + s*(bottom - base);    
    Rm = hand_model.segments{1+i}{1}.local(1:3,1:3);    
    u1 = Rm*[0;1;0];
    u2 = hand_model.global_pose(1:3,1:3)*[0;1;0];
    if (u1'*u2 <= 0)
        membrane_direction = [0;1;0];
    else
        membrane_direction = [0;-1;0];
    end   
    d  = Rm* membrane_direction;
    location_ray = (1-s)*hand_model.finger_radii{1+4*i+1} + s*hand_model.finger_radii{1+4*i+2};
    new_location = location + d* (location_ray - 3.0);   
    centers{end+1} = new_location; 
end
radii = {};
 for i = 1: length(hand_model.palm_wrist_radii)
    radii{end +1} = hand_model.palm_wrist_radii{i};
 end
 radii{end +1} = hand_model.fold_radii;
 for i = 1: length(hand_model.finger_radii)
    radii{end +1} = hand_model.finger_radii{i};
 end
 for i = 1: length(hand_model.membrane_radii)
    radii{end +1} = hand_model.membrane_radii{i};
 end

% display for debugging
display_model_debug(centers,hand_model.blocks,radii,1.0,'big');


%% HOW CENTERS ARE ORDERED IN YOUR MODEL

load('names_map.mat');

index_htrack2hmodel = {} ;
index_htrack2hmodel{names_map('index_base')} = 19; %'index_base' 
index_htrack2hmodel{names_map('index_bottom')} = 20;  %'index_bottom'
index_htrack2hmodel{names_map('index_membrane')} = 35;    %'index_membrane'
index_htrack2hmodel{names_map('index_middle')} = 21;    %'index_middle'
index_htrack2hmodel{names_map('index_top')} = 22;    %'index_top'
index_htrack2hmodel{names_map('middle_base')} = 23;    %'middle_base'
index_htrack2hmodel{names_map('middle_bottom')} = 24;   %'middle_bottom'
index_htrack2hmodel{names_map('middle_membrane')} = 36;    %'middle_membrane'
index_htrack2hmodel{names_map('middle_middle')} = 25;    %'middle_middle'
index_htrack2hmodel{names_map('middle_top')} = 26;    %'middle_top'
index_htrack2hmodel{names_map('palm_back')} = 2;    %'palm_back'  
index_htrack2hmodel{names_map('palm_index')} = 7;    %'palm_index'   
index_htrack2hmodel{names_map('palm_left')} = 1;    %'palm_left'   
index_htrack2hmodel{names_map('palm_middle')} = 6;    %'palm_middle' 
index_htrack2hmodel{names_map('palm_pinky')} = 4;    %'palm_pinky'  
index_htrack2hmodel{names_map('palm_right')} = 3;    %'palm_right'
index_htrack2hmodel{names_map('palm_ring')} = 5;    %'palm_ring'
index_htrack2hmodel{names_map('palm_thumb')} = 8;    %'palm_thumb'  
index_htrack2hmodel{names_map('pinky_base')} = 31;    %'pinky_base'
index_htrack2hmodel{names_map('pinky_bottom')} = 32;    %'pinky_bottom'
index_htrack2hmodel{names_map('pinky_membrane')} = 38;    %'pinky_membrane'
index_htrack2hmodel{names_map('pinky_middle')} = 33;    %'pinky_middle'
index_htrack2hmodel{names_map('pinky_top')} = 34;    %'pinky_top'  
index_htrack2hmodel{names_map('ring_base')} = 27;    %'ring_base' 
index_htrack2hmodel{names_map('ring_bottom')} = 28;    %'ring_bottom'
index_htrack2hmodel{names_map('ring_membrane')} = 37;    %'ring_membrane' 
index_htrack2hmodel{names_map('ring_middle')} = 29;    %'ring_middle'
index_htrack2hmodel{names_map('ring_top')} = 30;    %'ring_top' 
index_htrack2hmodel{names_map('thumb_additional')} = 18;    %'thumb_additional' 
index_htrack2hmodel{names_map('thumb_base')} = 14;    %'thumb_base'
index_htrack2hmodel{names_map('thumb_bottom')} = 15;    %'thumb_bottom'
index_htrack2hmodel{names_map('thumb_fold')} = 13;    %'thumb_fold'
index_htrack2hmodel{names_map('thumb_middle')} = 16;    %'thumb_middle'
index_htrack2hmodel{names_map('thumb_top')} = 17;    %'thumb_top'
index_htrack2hmodel{names_map('wrist_bottom_left')} = 11;    %'wrist_bottom_left'
index_htrack2hmodel{names_map('wrist_bottom_right')} = 12;    %'wrist_bottom_right' 
index_htrack2hmodel{names_map('wrist_top_left')} = 9;    %'wrist_top_left'
index_htrack2hmodel{names_map('wrist_top_right')} = 10;    %'wrist_top_right'

%% COPY TO NEW MODEL & ROTATE ACCORDINGLY

centers_htrack = {};
radii_htrack = {};

for i = 1:length(index_htrack2hmodel)
    index = index_htrack2hmodel(i);
    centers_htrack{i} = R*centers{index{1}};
    radii_htrack{i} = radii{index{1}};
end

% UPDATE INPUT FALANGES
phalanges{2}.local(1:3,4) = R* hand_model.segments{1}{1}.local(1:3,4);
phalanges{3}.local(2,4) = hand_model.segments{1}{2}.local(3,4);
phalanges{4}.local(2,4) = hand_model.segments{1}{3}.local(3,4);

phalanges{5}.local(1:3,4) = R* hand_model.segments{5}{1}.local(1:3,4);
phalanges{6}.local(2,4) = hand_model.segments{5}{2}.local(3,4);
phalanges{7}.local(2,4) = hand_model.segments{5}{3}.local(3,4);

phalanges{8}.local(1:3,4) = R* hand_model.segments{4}{1}.local(1:3,4);
phalanges{9}.local(2,4) = hand_model.segments{4}{2}.local(3,4);
phalanges{10}.local(2,4) = hand_model.segments{4}{3}.local(3,4);

phalanges{11}.local(1:3,4) = R* hand_model.segments{3}{1}.local(1:3,4);
phalanges{12}.local(2,4) = hand_model.segments{3}{2}.local(3,4);
phalanges{13}.local(2,4) = hand_model.segments{3}{3}.local(3,4);

phalanges{14}.local(1:3,4) = R* hand_model.segments{2}{1}.local(1:3,4);
phalanges{15}.local(2,4) = hand_model.segments{2}{2}.local(3,4);
phalanges{16}.local(2,4) = hand_model.segments{2}{3}.local(3,4);


%% WRITE TO FILE

write_cpp_model(data_path, centers_htrack, radii_htrack, phalanges)
