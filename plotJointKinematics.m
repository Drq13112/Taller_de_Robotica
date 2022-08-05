function plotJointKinematics(joint, trajTimes, q, qd, qdd)

figure
subplot (3,1,1);
plot (trajTimes, q(joint,:));
title ('Joint position (rad)')
%xaxis ([trajTimes(1) trajTimes(end)])
subplot (3,1,2);
plot (trajTimes, qd(joint,:));
title ('Joint velocity (rad/s)')
%xaxis ([trajTimes(1) trajTimes(end)])
subplot (3,1,3);
plot (trajTimes, qdd(joint,:));
title ('Joint acceleration (rad/s2)')
%xaxis ([trajTimes(1) trajTimes(end)])

end