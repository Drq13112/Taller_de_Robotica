function ComStat = setVel_one(id,vel2move)
P_GOAL_SPEED	= 32;
COMM_RXSUCCESS = 1;
vel2move = vel2move*(3/2);

calllib('dynamixel','dxl_write_word',id,P_GOAL_SPEED,vel2move); 

CommStatus = int32(calllib('dynamixel','dxl_get_result'));

if CommStatus == COMM_RXSUCCESS % check receiving is okay
    PrintErrorCode();
    ComStat = 1;
else
    PrintCommStatus(CommStatus);
    ComStat = -1;
end

end