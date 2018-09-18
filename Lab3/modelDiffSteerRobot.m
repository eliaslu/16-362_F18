function [x, y, th] = modelDiffSteerRobot(vl, vr, t0, tf, dt)

% dt = (tf-t0)/(length(vr));
% dt = dt(2);
dt = 0.02;
%   
% time_period = round((tf-t0)./dt);
% l = length(vr);
W = 0.087; % robot base width

% create empty vectors to optimize odometry loop
x  = [];
y  = [];
th = [];
  
%% Initialize Robot Starting Position and Angle 
% initialize all to 0
x(end+1) = 0;  
y(end+1) = 0; 
th(end+1) = 0;
  
%% Update Robot Position
% update linear and angular velocity, then update position starting
% with the new angle
  
% myPlot = plot(x,y);
% xlim([-0.5 0.5]);
% ylim([-0.5 0.5]);
  
for k = 1:length(vr)-1
    % update linear and angular velocity of robot
    V = (vr(k) + vl(k))/2;
    if k < 200
        omega = (vr(k) - vl(k))/W + 0.25;
    else 
        omega = (vr(k) - vl(k))/W - 0.1;
    end
    
    % calculate next position    
    th(end+1) = th(k) + omega       *dt;
    x(end+1)  = x(k)  + V*cos(th(k))*dt;
    y(end+1)  = y(k)  + V*sin(th(k))*dt;
    
   
end



end

