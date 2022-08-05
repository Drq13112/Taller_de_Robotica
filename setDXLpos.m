function [] = setDXLpos(id, P_GOAL_POSITION, GoalPos)
    if mod(GoalPos,1) ~= 0
        disp('input position must have an integer value!!');
    else
        if GoalPos >= 1024  || GoalPos < 0
            disp('GoalPos must have a value between 0 ~ 1023 !!')
        else
            calllib('dynamixel','dxl_write_word',id,P_GOAL_POSITION,GoalPos);  
        end        
    end   
end