function plotJointPositionSpeed(t, y, targetq, numJoints)
    figure
    subplot(2,1,1)
    plot(t,y(:,1:numJoints)) % Joint position
    hold all
    plot(t,targetq*ones(1,length(t)),'--') % Joint setpoint
    title('Joint Position')
    xlabel('Time (s)')
    ylabel('Position (rad)')
    subplot(2,1,2)
    plot(t,y(:,numJoints+1:end)) % Joint velocity
    title('Joint Velocity')
    xlabel('Time (s)')
    ylabel('Velocity (rad/s)')

end