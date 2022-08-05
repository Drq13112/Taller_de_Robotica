%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
function Mover_DobleMotor(posicion, velocidad)

%Esta función se encarga de mover simultaneamente los dos motores de la articulación 3

posicion2=300-posicion;
comm_stat = setVel_one(3,velocidad);
comm_stat = setPos_one(3,posicion);

comm_stat = setVel_one(8,velocidad);
comm_stat = setPos_one(8,posicion2);

end