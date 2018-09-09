leftStart = 6934;
rghtStart = 4396;
distance_togo = 12 * 2.54 /100.0;
signedDistance = 0.0;
timeArray = zeros(1,1);
leftArray = zeros(1,1);
rghtArray = zeros(1,1);
tic
while signedDistance < distance_togo
    time_elapse = toc;
    timeArray(end+1) = time_elapse;
    leftEncoder = leftStart + 50 * time_elapse;
    rghtEncoder = rghtStart + 50 * time_elapse;
    leftArray(end+1) = leftEncoder - leftStart;
    rghtArray(end+1) = rghtEncoder - rghtStart;
    signedDistance = (leftEncoder + rghtEncoder - leftStart - rghtStart)/2000;
    pause(0.001);
end

figure;
subplot(2, 1, 1);
plot(timeArray,leftArray);
xlabel('time elapsed (s)')
ylabel('distance travelled (mm)')
title('left encoder plot')
subplot(2, 1, 2);
plot(timeArray,rghtArray);
xlabel('time elapsed (s)')
ylabel('distance travelled (mm)')
title('right encoder plot')
% disp(timeArray);
% disp(leftEncoder);
% disp(rghtEncoder);