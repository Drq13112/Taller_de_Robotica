%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
function configuracion = moveL_conf_v3(miRobot,t,t_sample,q1,q2,q3,q4,q5,x,y,z)
    l1=0.1725;l2=0.08898;l3=0.260486;
    endEffector = 'tool';
    q0 = homeConfiguration(miRobot);
    nJoint = numel (q0);
    %Definimos la orientación del punto final con la cinemática inversa
    orientacionF = orientacionCinematicaInversa(miRobot,x,y,z);

    fixOrientationF = [orientacionF(1,1) orientacionF(1,2) orientacionF(1,3);
                       orientacionF(2,1) orientacionF(2,2) orientacionF(3,2);
                       orientacionF(3,1) orientacionF(3,2) orientacionF(3,3)];
                   
    pose0 = getMTH_TCP(miRobot,q1,q2,q3,q4,q5);
    poseF = trvec2tform([x y z]) * rotm2tform(fixOrientationF);

    tIni = 0;
    tFin = t;
    tSample = t_sample;
    tInterval = [tIni tFin];         %time interval
    tvec = tIni:tSample:tFin;           %time sample
    nTraj = length (tvec); 

    %Calculamos los puntos intermedios desde el punto inicial al punto final
    %mediante interpolación lineal
    [tfInterp, v1, a1] = transformtraj(pose0,poseF,tInterval,tvec);

    %Aplicamos la cinemática inversa para todos los puntos de paso
    ik = inverseKinematics('RigidBodyTree',miRobot);
    weights = [0 0 0 1 1 1]';
    ikInitGuess = q0;
    joints_TS = zeros (nJoint, nTraj); 
    for i = 1:nTraj
        qSol = ik(endEffector, tfInterp(:,:,i),weights,ikInitGuess);
        joints_TS(:,i) = qSol';
        ikInitGuess = qSol;
    end
    
    traj_TS = tform2trvec(tfInterp); 
    
    configuracion = joints_TS';
    
%     figure
%     set(gcf,'Visible','on')
%     show(miRobot,q0,'Frames','off','PreservePlot',false);
%     hold all
%     axis([-0.2 (l1+l2+l3)*1.10 -(l2+l3)*1.10 (l2+l3)*1.10 -(l2+l3)*1.10 (0.25+l2+l3)*1.10])
% 
%     plot3 (traj_TS (:,1), traj_TS (:,2), traj_TS (:,3),':','LineWidth',2);
%     for i = 1:nTraj
%         show(miRobot, joints_TS(:,i),'Frames','off','PreservePlot',false);
%         pause(0.05);
%     end
    
end