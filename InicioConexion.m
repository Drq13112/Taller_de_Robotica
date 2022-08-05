%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 
function status=InicioConexion(current_pos,Port,Baud,velocidad_servo,velocidad_motor)
    %Iniciar conexion con dynamixel
    global_vars;
    
    loadlibrary('dynamixel', 'dynamixel.h');
    loadlibrary('dynamixel','dxl_matlab.h');
    libfunctions('dynamixel');

    % Settings for connection
    DEFAULT_PORTNUM		= 5; % COM5
    DEFAULT_BAUDNUM		= 1; % 1Mbps

    % Settings for move actuators
    NUM_ACTUATOR		= 7; % Number of actuator300/
    CONTROL_PERIOD		= 0.01; % 10msec (Large value is more slow)

    %%%%% vel (deg/sec) is between [0 682], pos (deg) is between [0 300]
    Default_Vel = 30; % 0 for maximum torque
    Default_Pos = 150;
    
    res = connectDXL(Port, Baud);
    
    %Ponemos los motores en la posicion inicial
    %Seteamos el motor 1 como una rueda, el resto vienen predefinidos como
    %servos

    if res == 1
        %MoverTornillo(150,0,1000)
        HomePosition(current_pos*1000,velocidad_servo,velocidad_motor);
        status=1;
    else
        disp('Failed to open USB2Dynamixel!');
        status=0;
    end
end