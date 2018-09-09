%% Lab 1 Task 1 Move the Robot
% pause(6);
vl = 0.05;
vr = 0.05;
time_elapse = 0.0;
tic
while time_elapse < 4
    robot.sendVelocity(vl, vr);
    time_elapse = toc;
    pause(0.05);
end
robot.sendVelocity(0,0);