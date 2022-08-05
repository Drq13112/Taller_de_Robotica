function pos = getPos_one(id)
P_GOAL_POSITION=20;
COMM_RXSUCCESS = 1;

res = calllib('dynamixel','dxl_read_word',id,P_GOAL_POSITION); 

CommStatus = int32(calllib('dynamixel','dxl_get_result'));

if CommStatus == COMM_RXSUCCESS % check receiving is okay
    PrintErrorCode();
    pos = res*(300/1024); 
else
    PrintCommStatus(CommStatus);
    pos = -1;
end

end