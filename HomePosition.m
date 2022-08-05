%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 
function HomePosition(pos_actual,velocidad_servo,velocidad_motor)
%Puntos=[0,150,220,80,10,0,150];
%MoverRobot(Puntos,150,200)


Mover_DobleMotor(230, velocidad_servo)

comm_stat = setVel_one(4,velocidad_servo);
comm_stat = setPos_one(4,155);
pause(5);

comm_stat = setVel_one(2,velocidad_servo);
comm_stat = setPos_one(2,165);
pause(3);

comm_stat = setVel_one(2,velocidad_servo);
comm_stat = setPos_one(2,120);
pause(3);
comm_stat = setVel_one(2,velocidad_servo);
comm_stat = setPos_one(2,150);
pause(3);

comm_stat = setVel_one(5,velocidad_servo);

comm_stat = setPos_one(5,150);
pause(3);
comm_stat = setPos_one(5,200);
pause(3);
comm_stat = setPos_one(5,100);
pause(5);
comm_stat = setPos_one(5,155);

MoverTornillo(pos_actual,0,velocidad_motor);

AbrirPinza();
pause(2);
CerrarPinza();
pause(2);
AbrirPinza();

end
