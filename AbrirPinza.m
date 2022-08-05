%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 

function AbrirPinza
    global_vars;
    disp('Abriendo Pinza');
    comm_stat = setVel_one(6,30);
    comm_stat = setPos_one(6,150);
end
