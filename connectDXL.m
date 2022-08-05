function res = connectDXL(DEFAULT_PORTNUM, DEFAULT_BAUDNUM)

res = calllib('dynamixel','dxl_initialize',DEFAULT_PORTNUM,DEFAULT_BAUDNUM);

if res == 1
    disp('Succeed to open USB2Dynmixel!');
else
    disp('Failed to open USB2Dynamixel!');
end


end

