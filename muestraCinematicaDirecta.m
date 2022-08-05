%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
function muestraCinematicaDirecta(q1,q2,q3,q4,q5)
    miRobot = creaRobot_v2();
    conf = transpose([q1 q2 q3 q4 q5]);
    show(miRobot, conf, 'Collisions','off','Visuals','on');
    camup([1 0 0]);
end