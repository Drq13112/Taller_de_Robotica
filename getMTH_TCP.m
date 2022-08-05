%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 
function MTH_TCP = getMTH_TCP(miRobot,q1,q2,q3,q4,q5)

%   Se resuelve el problema de la cinemática directa
    conf = transpose([q1 q2 q3 q4 q5]);
    endEffector = 'tool';

%   Se devuelve la Matriz de Transformación Homogénea
    MTH_TCP = getTransform(miRobot, conf, endEffector);
    
end