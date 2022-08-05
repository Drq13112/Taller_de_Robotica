%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
function refPose=moveL_conf_v2(miRobot,t,sample_t,q1,q2,q3,q4,q5,x,y,z)
    robot = miRobot; 
    endEffector = "tool";
    nJoints = numel (robot.homeConfiguration);
    
    %Crear vector de tiempo
    tspan = 0:sample_t:t; 
    
    initial_conf = [q1;q2;q3;q4;q5];
    
    %Matriz de posiciones y velocidades articulares iniciales
%     initialState = [homeConfiguration(robot);zeros(nJoints,1)];
    initialState = [initial_conf;zeros(nJoints,1)];
    
    %Matriz de posiciones articulares final
    final_conf = conf_CinematicaInversa(robot,x,y,z);
    
    %Matriz homogénea del end-effector
    refPose = getTransform (robot, final_conf, endEffector);
    
    %Creación del modelo de movimiento en el espacio articular
    motionModel_TS = taskSpaceMotionModel("RigidBodyTree",robot,"EndEffectorName",endEffector);
    
    % Definir la velocidad objetivo del end-effector en el espacio de trabajo [omega v]
    refVel = zeros(6,1);
    [t,robotState_TS] = ode15s(@(t,state)derivative(motionModel_TS,state,refPose,refVel),tspan,initialState);
    
    
    %Visualizamos el robot
    figure
    set(gcf,'Visible','on')
    show(robot,initialState(1:nJoints));
    hold all
    initPose = getTransform (robot, initialState(1:nJoints), endEffector);
    plot3(initPose(1,4),initPose(2,4),initPose(3,4),'x','Color','g',"MarkerSize",20)
    plot3(refPose(1,4),refPose(2,4),refPose(3,4),"x",'Color','g',"MarkerSize",20)
    r = rateControl(5);
    
    %TASK-SPACE MODEL
    for i = 1:size(robotState_TS,1)
        poseNow = getTransform(robot,robotState_TS(i,1:nJoints)',endEffector);
        show(robot,robotState_TS(i,1:nJoints)',"PreservePlot",false);
        taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',10);
        waitfor(r);
    end
    
    % Add a legend and title
    legend(taskSpaceMarker,'Defined in Task-Space');
    title('Manipulator Trajectories')
%     
%     % Añadimos la leyenda y el título
%     legend(jointSpaceMarker,'Defined in Joint-Space');
%     title('Trayectoria seguida')
%     figure
%     set(gcf,'Visible','on')
%     plot(t,robotState_JS(:,nJoints+1:nJoints*2));
%     title("JOINT-SPACE: Joint Velocity");
%     xlabel("Time (s)")
%     ylabel("Velocity (rad/s)");
    
    configuraciones_articulares = robotState_TS(:,1:5);
    velocidades_articulares = robotState_TS(:,6:10);
end