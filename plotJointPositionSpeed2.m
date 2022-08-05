function plotJointPositionSpeed2(t, y, tRef, yRef,targetq, numJoints)
    figure
    subplot(2,1,1)
    plot(t,targetq*ones(1,length(t)),'--') % Joint setpoint    
    hold all
    plot(t,y(:,1:numJoints)) % Joint simulated position
    plot(tRef,yRef(:,1:numJoints),':') % Joint trayecotry
    title('Joint Position')
    xlabel('Time (s)')
    ylabel('Position (rad)')
    subplot(2,1,2)
    plot(t,y(:,numJoints+1:end)) % Joint velocity.
    hold on
    plot(tRef,yRef(:,numJoints+1:end),':') % Joint setpoint
    title('Joint Velocity')
    xlabel('Time (s)')
    ylabel('Velocity (rad/s)')

end