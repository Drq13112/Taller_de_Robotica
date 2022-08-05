%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
function muestraCinematicaDirecta_v2(q1,q2,q3,q4,q5)
    q2=q2*pi/180;
    q3=q3*pi/180;
    q4=q4*pi/180;
    q5=q5*pi/180;
    miRobot = creaRobot_v2();
    conf = transpose([q1 q2 q3 q4 q5]);
%     show (miRobot, conf);
    show(miRobot, conf, 'Collisions','off','Visuals','on');
    camup([1 0 0]);
end