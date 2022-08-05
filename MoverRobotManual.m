%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
function MoverRobotManual(pose_objetivo,current_pos,velocidad_motor,velocidad_servo)
    global_vars;
    %Sincronizamos los ceros de lar articulaciones con los de los motores
    
    pose_objetivo(2)=-pose_objetivo(2)+150
    pose_objetivo(3)=230-pose_objetivo(3)
    pose_objetivo(4)=155+pose_objetivo(4)
    pose_objetivo(5)=pose_objetivo(5)+150
    
    
    SetPoint=pose_objetivo(1)*1000;
    %Movemos el tornillo
    MoverTornillo(current_pos*1000,SetPoint,velocidad_motor);
    
    pause(3);

    SetPoint=pose_objetivo(2);
    %Movemos el resto de motores
    comm_stat = setVel_one(2,velocidad_servo);
    comm_stat = setPos_one(2,SetPoint);
    %pause(0.5);    

    SetPoint=pose_objetivo(3);
    %Movemos el motor doble
    Mover_DobleMotor(SetPoint,velocidad_servo);
    %pause(0.5);

    SetPoint=pose_objetivo(4);
    %Movemos el resto de motores
    comm_stat = setVel_one(4,velocidad_servo);
    comm_stat = setPos_one(4,SetPoint);
    %pause(0.5);    

    SetPoint= pose_objetivo(5);
    comm_stat = setVel_one(5,velocidad_servo);
    comm_stat = setPos_one(5,SetPoint);
    %GirarTCP(current_pinza,SetPoint,250);
    
end
