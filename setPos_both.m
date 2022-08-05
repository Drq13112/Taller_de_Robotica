function ComStat = setPos_both(id1, id2, pos1, pos2)
P_GOAL_POSITION	= 30;
COMM_RXSUCCESS = 1;
BROADCAST_ID    = 254;
NUM_ACTUATOR = 2;

INST_SYNC_WRITE		=131;
id = [id1 id2];
pos1 = round( pos1 * (1024/300) );
pos2 = round( pos2 * (1024/300) );
pos = [pos1 pos2];
% Make syncwrite packet
calllib('dynamixel','dxl_set_txpacket_id',BROADCAST_ID);
calllib('dynamixel','dxl_set_txpacket_instruction',INST_SYNC_WRITE);
calllib('dynamixel','dxl_set_txpacket_parameter',0,P_GOAL_POSITION);
calllib('dynamixel','dxl_set_txpacket_parameter',1,2);

for i = 0:1:NUM_ACTUATOR-1
    calllib('dynamixel','dxl_set_txpacket_parameter',2+3*i,id(i+1));
    
    GoalPos = pos(i+1);
    
    low = calllib('dynamixel','dxl_get_lowbyte',GoalPos);
    calllib('dynamixel','dxl_set_txpacket_parameter',2+3*i+1,low);
    high = calllib('dynamixel','dxl_get_highbyte',GoalPos);
    calllib('dynamixel','dxl_set_txpacket_parameter',2+3*i+2,high);
end

calllib('dynamixel','dxl_set_txpacket_length',(2+1)*NUM_ACTUATOR+4);

calllib('dynamixel','dxl_txrx_packet'); % makes move!

%%%% Check Communication Status %%%%
CommStatus = int32(calllib('dynamixel','dxl_get_result'));

if CommStatus == COMM_RXSUCCESS % check receiving is okay
    PrintErrorCode();
    ComStat = 1;
else
    PrintCommStatus(CommStatus);
    ComStat = -1;
end

end
