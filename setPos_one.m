function ComStat = setPos_one(id, pos2move)
P_GOAL_POSITION	= 30;
COMM_RXSUCCESS = 1;

pos2move = round( pos2move * (1024/300) );

calllib('dynamixel','dxl_write_word',id,P_GOAL_POSITION,pos2move); 

CommStatus = int32(calllib('dynamixel','dxl_get_result'));

if CommStatus == COMM_RXSUCCESS % check receiving is okay
    PrintErrorCode();
    ComStat = 1;
else
    PrintCommStatus(CommStatus);
    ComStat = -1;
end

end