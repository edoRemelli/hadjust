function [is_on_segment, i,j, segment_index ] = block_index2segment_id( hand_model,index, block_index )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here



% in the case we are dealing with segment
i=0;
j=0;
segment_index = 0;
is_on_segment = false;
block = hand_model.blocks{block_index};
min_index = min(block(1),block(2));

% check if we are really looking at a segment
if (length(block) < 3 && min_index> 13 && min_index< 35  )
    
    % DEALING WITH THUMB
    if(min_index <= 18)
        remapped_min_index = min_index - 13;
        i = 1;
        j = remapped_min_index;
        is_on_segment = true;     
    end
    
    % DEALING WITH REST OF FINGERS
    if (min_index > 18)
        remapped_min_index = min_index - 18;
        i = floor(remapped_min_index/4) +2;
        j = mod(remapped_min_index,4);
        is_on_segment = true;
    end
    
    % check if we project over segment or if we project over sphere
    if (length(index) == 2)
            segment_index = [j,j+1];
    elseif (length(index) == 1)
            if min_index == index
                segment_index = j;
            else
                segment_index = j+1;
            end
    end
        
end


    







