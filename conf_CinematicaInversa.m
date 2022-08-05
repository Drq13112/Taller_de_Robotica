%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 
function configSoln = conf_CinematicaInversa(robot,q,x,y,z)
    ik = inverseKinematics('RigidBodyTree',robot);
    weights = [0 0 0 1 1 1];
    initialguess = robot.homeConfiguration;

    %El angulo se introduce en radianes
    MTH_TCP = trvec2tform ([x y z]);
    [configSoln,solnInfo] = ik('tool',MTH_TCP,weights,initialguess);
end