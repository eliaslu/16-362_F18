robot = raspbot('Raspbot-7');
% pause(5);
%% Warm Up 1: Robot Listener
% robot.encoders.NewMessageFcn = @encoderEventListener;
% robot.sendVelocity(0.02, 0.02);
% tic;
% time = toc;
% INITIAL_TIME = time;
% disp(INITIAL_TIME);
% curTime = 0;
% 
% while time < 5
%     curTime = toc;
%     pause(0.2);
% end
% 
% robot.encoders.NewMessageFcn = [];

%% Warm Up 2: Measuring Velocity
% startPos = robot.encoders.LatestMessage.Vector.X;
% tic;
% toc;
% startTime = toc;
% % startTime = robot.encoders.LatestMessage.Header.Stamp.Sec + ...
%  %   (robot.encoders.LatestMessage.Header.Stamp.Nsec / 1000000000.0);
% timeArray = [];
% velocityArray = [];
% myPlot = plot(timeArray, velocityArray, 'b-');
% xlim([0.0 3]);
% ylim([-.25 .25]);
% for i = 1:20
%     toc;
%     newDistance = robot.encoders.LatestMessage.Vector.X;
%     newTime = toc;
%     %newStart = robot.encoders.LatestMessage.Header.Stamp.Sec + ...
%     %(robot.encoders.LatestMessage.Header.Stamp.Nsec / 1000000000.0);
%     robot.sendVelocity(0.15,0.15);
%     toc;
%     endDistance = robot.encoders.LatestMessage.Vector.X;
%     endTime = toc;
%     %endTime = robot.encoders.LatestMessage.Header.Stamp.Sec + ...
%     % (robot.encoders.LatestMessage.Header.Stamp.Nsec / 1000000000.0);
%     ds = endDistance - newDistance;
%     dt = endTime - newTime;
%     V = ds/dt;
%     totalTime = endTime - startTime;
%     pause(0.005);
%     timeArray(end+1) = totalTime;
%     velocityArray(end+1) = V;
%     set(myPlot, 'xdata', [get(myPlot,'xdata') totalTime], ...
%                 'ydata', [get(myPlot,'ydata') V]);
% end


%% Warm Up 3:Basic Simulation
% v = 0.1;
% kw = 0.125;
% W = 0.085;
% 
% tf = sqrt(32*pi);
% t0 = 0.0;
% t = 0.0;
% prev_t = 0.0;

% v = 0.2;
% sf = 1;
% W = 0.088;
% tf = sf/v;
% kt = 2*pi/sf;
% kk = 15.1084;
% ks = 3;
% t0 = 0.0;
% Tf = ks*tf;
% t = 0.0;
% prev_t = 0.0;
% 
% vl_array = [];
% vr_array = [];
% dt = [];
% 
% 
% % robot.sendVelocity(0.2, 0.2);
% % pause(0.4);
% % % robot.sendVelocity(0, 0);
% 
% global INITIAL_TIME;
% INITIAL_TIME = double(robot.encoders.LatestMessage.Header.Stamp.Sec) ...
%         + double(robot.encoders.LatestMessage.Header.Stamp.Nsec) /1000000000.0;
%  
% prev_t = INITIAL_TIME;
% while t < tf
%     t = double(robot.encoders.LatestMessage.Header.Stamp.Sec) ...
%         + double(robot.encoders.LatestMessage.Header.Stamp.Nsec) /1000000000.0 - INITIAL_TIME;
%     
%     t = t/ks;
%     
%     dt(end+1) = t - prev_t;
%     s = v * t;
%     curvature = kk/ks * sin(kt*s);
%     omega = curvature * v;
%     vr = v + W * omega/2;
%     vl = v - W * omega/2;
%     robot.sendVelocity(vl, vr);
%     
%     
% %     vl = (v - W*kw*t/2);
% %     vr = (v + W*kw*t/2);
% %     
% %     robot.sendVelocity(vl,vr);
%     
%     vl_array(end+1) = vl * 1000;
%     vr_array(end+1) = vr * 1000;
%     
%     pause(0.001);
%     prev_t = t;
% end
% 
% robot.sendVelocity(0,0);
% 
% robot.sendVelocity(0.2,0.2);
% pause(0.5);
% robot.sendVelocity(0,0);
% %plot trajectory
% [x, y, th] = modelDiffSteerRobot(vl_array, vr_array, t0, tf, dt);
% disp(x(1000))
% x = x./1000;
% y = y./1000;
% plot(x, y, 'b-');
% xlim([0 0.5]);
% ylim([0 0.5]);
% 
% robot.sendVelocity(0,0);




%% challenge task
%set up the listener 
robot.encoders.NewMessageFcn=@encoderEventListener;
%set up vr and vl and t
global timeArray;
%global leftArray;
%global rightArray;
global vlSoFar;
global vrSoFar;
global timeDiff;

vlSoFar = zeros(1);
vrSoFar = zeros(1);


W = 0.087;
v = 0.2;
sf = 1;
tf = sf/v;
kt = 2*pi/sf;
kk = 15.8;
ks = 3;
t0 = 0.0;
Tf = ks*tf;
t = 0.0;
prev_t = 0.0;
tic;

xArray = zeros(1,500);
yArray = zeros(1,500);

myPlot = plot(xArray,yArray, 'b-');
title('State Estimate');
xlabel('y-direction (m)');
ylabel('x-direction (m)');
xlim([-0.6 0.6]);
ylim([-0.6 0.6]);
vl = 0;
vr = 0;
  
%loop to move the robot by unpdating vl and vr
while (t < tf)
    %drive the robot
    robot.sendVelocity(vl,vr);
    pause(0.001);
    T = toc;
    t = T/ks;
    s = v * t;
    curvature = kk/ks * sin(kt*s);
    omega = curvature * v;
    vr = v + W * omega/2;
    vl = v - W * omega/2;
  
    if(~isempty(timeArray))
        tfinal = timeArray(size(timeArray)) - timeArray(1);
        dt = timeDiff;
        [x, y, th] = modelDiffSteerRobot(vlSoFar, vrSoFar, t0, tfinal, dt);

        %set(myPlot, 'Xdata', x.*dt, 'Ydata', y.*dt);
        set(myPlot, 'Xdata', x, 'Ydata', y);
        pause(0.01);
    end
    
    % put that into modelDiffSteerRobot
    % plot x,y real time
end

robot.sendVelocity(0,0);
robot.encoders.NewMessageFcn = [];

%%

robot.stop();
clear all;