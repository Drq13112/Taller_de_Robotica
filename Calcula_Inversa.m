%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 
function configSoln=Calcula_Inversa(miRobot,q,x,y,z,angulo)

    l1=0.1725;l2=0.08898;l3=0.260486;
    ik = inverseKinematics('RigidBodyTree',miRobot);
    weights = [0.25 0.25 0.25 1 1 1];
    initialguess = q;

    %El angulo se introduce en radianes
    MTH_TCP = trvec2tform ([x y z])*axang2tform ([0 0 1 angulo]);
    [configSoln,solnInfo] = ik('tool',MTH_TCP,weights,initialguess);

end