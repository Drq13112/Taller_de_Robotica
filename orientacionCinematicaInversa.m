%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
function M_orientacion=orientacionCinematicaInversa(robot,x,y,z)
    ik = inverseKinematics('RigidBodyTree',robot);
    weights = [1 1 1 0 0 0];
    initialguess = robot.homeConfiguration;

    %El angulo se introduce en radianes
    MTH_TCP = trvec2tform ([x y z]);
    [configSoln,solnInfo] = ik('tool',MTH_TCP,weights,initialguess);
    M_orientacion=getMTH_TCP(robot,configSoln(1,1),configSoln(2,1),configSoln(3,1),configSoln(4,1),configSoln(5,1));
    
end