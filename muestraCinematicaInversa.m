%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
function muestraCinematicaInversa(x,y,z,angulo)
    l1=0.1725;l2=0.08898;l3=0.260486;
    miRobot=creaRobot_v2();
    ik = inverseKinematics('RigidBodyTree',miRobot);
    weights = [0.25 0.25 0.25 1 1 1];
    initialguess = miRobot.homeConfiguration;

    %El angulo se introduce en radianes
    MTH_TCP = trvec2tform ([x y z])*axang2tform ([0 0 1 angulo]);
    [configSoln,solnInfo] = ik('tool',MTH_TCP,weights,initialguess);
    show(miRobot,configSoln);
    
    %Vista más representativa
    view([135 0])
    camup([1 0 0]);
    axis([-0.2 (l1+l2+l3)*1.10 -(l2+l3)*1.10 (l2+l3)*1.10 -(l2+l3)*1.10 (0.25+l2+l3)*1.10]);
    solnInfo
    configSoln
end