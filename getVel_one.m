function vel = getVel_one(id)
P_GOAL_SPEED	= 32;
COMM_RXSUCCESS = 1;

res = calllib('dynamixel','dxl_read_word',id,P_GOAL_SPEED); 

CommStatus = int32(calllib('dynamixel','dxl_get_result'));

if CommStatus == COMM_RXSUCCESS % check receiving is okay
    PrintErrorCode();
    vel = res*(2/3); 
else
    PrintCommStatus(CommStatus);
    vel = -1;
end

end