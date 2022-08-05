%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
function [nueva_pose]=MoverRobot(pose_inicial,puntos,velocidad_motor,velocidad_servo,modo,tcp)

    %Puntos=    q1,q2,q3,q4,q5 en t0
    %           q1,q2,q3,q4,q5 en t1...
    global_vars;
    puntos
    %Sincronizamos los ceros de lar articulaciones con los de los motores
    current_pos=pose_inicial(1);
    pose_inicial(2)=pose_inicial(2)+150;
    pose_inicial(3)=220-pose_inicial(3);
    pose_inicial(4)=150+pose_inicial(4);
    pose_inicial(5)=pose_inicial(5)+150;
    pose_inicial
    %Leemos los puntos de la trayectoria
    if modo==1 
        [filas,columnas]=size(puntos);
        for fila= 1:filas
            for columna=1:columnas
                switch columna
                    case 1
                        SetPoint=puntos(fila,columna)*1000;
                        %Movemos el tornillo
                        MoverTornillo(current_pos*1000,SetPoint,velocidad_motor);
                          %pause(0.1);
                          disp('moviendo');
                        current_pos=puntos(fila,1);
                end
            end 
        end
    end
    pause(3); 
    [filas,columnas]=size(puntos);
    for fila= 1:filas
        for columna=1:columnas
            switch columna
               case 1
                    if modo ==0
                        SetPoint=puntos(fila,columna)*1000;
                        %Movemos el tornillo
                        MoverTornillo(current_pos*1000,SetPoint,velocidad_motor);
                        current_pos=puntos(fila,1);
                        %pause(2);
                    end
                case 2
                    SetPoint=pose_inicial(2)-puntos(fila,columna)*180/pi;
                    comm_stat = setVel_one(2,velocidad_servo);
                    comm_stat = setPos_one(2,SetPoint);
                    %pause(0.2);    
                case 3
                    SetPoint=pose_inicial(3)-puntos(fila,columna)*180/pi;
                    %Movemos el motor doble
                    Mover_DobleMotor(SetPoint,velocidad_servo);
                    pause(0.2);
                case 4
                    SetPoint=pose_inicial(4)-puntos(fila,columna)*180/pi;
                    %Movemos el resto de motores
                    comm_stat = setVel_one(4,velocidad_servo);
                    comm_stat = setPos_one(4,SetPoint);
                    %pause(0.2);    
                case 5
                    if tcp==1
                        Setpoint=pose_inicial(5)+puntos(fila,5)*180/pi;
                        
                        comm_stat = setVel_one(5,velocidad_servo);
                        comm_stat = setPos_one(5,SetPoint);
                    end
            end
        end
    end
    nueva_pose=zeros(5,1);
    nueva_pose(1)=current_pos;
    puntos(filas,3);
    nueva_pose(2)=pose_inicial(2)+puntos(filas,2)*180/pi;
    nueva_pose(3)=pose_inicial(3)+puntos(filas,3)*180/pi;
    nueva_pose(4)=pose_inicial(4)+puntos(filas,4)*180/pi;
    nueva_pose
    nueva_pose(2)=nueva_pose(2)-150;
    nueva_pose(3)=-220+nueva_pose(3);
    nueva_pose(4)=nueva_pose(4)-150;
    if tcp==1
        nueva_pose(5)=pose_inicial(5)+puntos(filas,5)*180/pi-150;
    else
        nueva_pose(5)=pose_inicial(5);
    end
    nueva_pose;
    disp('Punto alcanzado');
end