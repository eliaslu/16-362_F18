robot = raspbot('Raspbot-6');
pause(6);
vl = 0.05;
vr = 0.05;
leftStart = robot.encoders.LatestMessage.Vector.X;
rghtStart = robot.encoders.LatestMessage.Vector.Y;
distance_togo = 12 * 2.54 /100.0;
signedDistance = 0.0;
timeArray = zeros(1,1);
leftArray = zeros(1,1);
rghtArray = zeros(1,1);
tic
while signedDistance < distance_togo
    time_elapse = toc;
    timeArray(end+1) = time_elapse;
    leftEncoder = robot.encoders.LatestMessage.Vector.X;
    rghtEncoder = robot.encoders.LatestMessage.Vector.Y;
    if leftEncoder - leftStart > distance_togo | rghtEncoder - rghtStart > distance_togo
        robot.sendVelocity(0, 0);
    else
        robot.sendVelocity(vl, vr);
    end
    leftArray(end+1) = leftEncoder - leftStart;
    rghtArray(end+1) = rghtEncoder - rghtStart;
    signedDistance = (leftEncoder + rghtEncoder - leftStart - rghtStart)/2;
    pause(0.05);
end

first_loop_time = toc;
robot.sendVelocity(0,0);
pause(1);

signedDistance = 0.0;
leftStart2 = robot.encoders.LatestMessage.Vector.X;
rghtStart2 = robot.encoders.LatestMessage.Vector.Y;
tic
while signedDistance < distance_togo
    time_elapse = toc;
    timeArray(end+1) = time_elapse + first_loop_time;
    leftEncoder = robot.encoders.LatestMessage.Vector.X;
    rghtEncoder = robot.encoders.LatestMessage.Vector.Y;
    if leftStart2 - leftEncoder > distance_togo | rghtStart2 - rghtEncoder > distance_togo
        robot.sendVelocity(0, 0);
    else
        robot.sendVelocity(-vl, -vr);
    end
    leftArray(end+1) = leftEncoder - leftStart;
    rghtArray(end+1) = rghtEncoder - rghtStart;
    signedDistance = -(leftEncoder + rghtEncoder - leftStart2 - rghtStart2)/2;
    pause(0.05);
end

robot.sendVelocity(0,0);

figure;
subplot(2, 1, 1);
plot(timeArray,leftArray);
xlabel('time elapsed (s)')
ylabel('distance travelled (m)')
title('left encoder plot')
subplot(2, 1, 2);
plot(timeArray,rghtArray);
xlabel('time elapsed (s)')
ylabel('distance travelled (m)')
title('right encoder plot')