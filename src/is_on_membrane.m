function [bool_is_on_membrane, is_membrane_center, membrane_index ] = is_on_membrane( hand_model,index, block_index )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here


bool_is_on_membrane = false;
is_membrane_center = false(size(index));
membrane_index = -ones(size(index));

% the membrane blocks
block_index_list = [13,17,21,22,26,27];
% the membrane center indices
index_list = [35,36,37,38];


block = hand_model.blocks{block_index};

% check if we are really looking at a membrane block
if (length(block) == 3 && ismember(block_index,block_index_list))   
    bool_is_on_membrane = true;
    %loop over where we are projecting
    for i = 1:length(index)       
        if ismember( abs(index(i)),index_list)
            is_membrane_center(i) = true;
            membrane_index(i) = abs(index(i)) - 34 + 1;
        else
            membrane_index(i) = 6-(abs(index(i)) - 3);       
        end
    end        
end
