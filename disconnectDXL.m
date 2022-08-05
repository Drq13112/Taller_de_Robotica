function [status] = disconnectDXL()

calllib('dynamixel','dxl_terminate'); 
unloadlibrary('dynamixel');

disp('disconnected from USB2Dynamixel!');
status=0;
end