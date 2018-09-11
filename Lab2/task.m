%% Initialize robot
robot = raspbot('Raspbot-6');
pause(2);
robot.startLaser();
pause(3);

%% Define consts and vars
idealObjectRange = 0.5;
gain = 0.5;
gain_b = 0.8;
width = 0.25;
V = 0;
tmpx=0;
tmpy=0;
tmpth=0;

%% Collect and parse data
tic;
a = toc;
prev_time = 0;
while toc < 150
    objectRanges = robot.laser.LatestMessage.Ranges;
    x = [];
    y = [];
    th = [];
    for index = 1:size(objectRanges)
        if objectRanges(index) <= 0.06
            objectRanges(index) = 10; % or maybe change it to 2 so they don't get interfered when tracking
        end
    end
    [objectDist, min_index] = min(objectRanges);
    if objectDist > 1
        a = toc;
        prev_time = a;
        continue;
    end
    for idx = 1:size(objectRanges)
        if objectRanges(idx) <= 1
            [tmpx,tmpy,tmpth]= irToXy(idx,objectRanges(idx));
            if abs(tmpth) <=  deg2rad(90)
                [x(end+1),y(end+1),th(end+1)] = irToXy(idx,objectRanges(idx));
            end
        end
        if idx == min_index
            y_dist = tmpy;
            x_dist = tmpx;
%             min_index = size(th);
        end
    end
    
%% plot
   
    f = scatter(-y,x,"x");
    hold on;
    xlim([-2 2]);
    ylim([-2 2]);

%% Operations
    if objectDist > idealObjectRange
%         a = toc;
%         cur_time = a;
%         dt = cur_time - prev_time;
%         dth = min_index - prev_index;
%         omega = dth/dt;
        curvature = y_dist/(objectDist^2);
        V = (objectDist - idealObjectRange) * gain;
        omega = curvature * V;
        vr = V + width * omega/2;
        vl = V - width * omega/2;
        robot.sendVelocity(vl, vr);
    end
    if objectDist == 0 || objectDist == 0.5
        robot.sendVelocity(0,0);
    end
    if objectDist < idealObjectRange
%         a = toc;
%         cur_time = a;
%         dt = cur_time - prev_time;
%         dth = min_index - prev_index;
%         omega = dth/dt;
        curvature = y_dist/(objectDist^2);
        V = (objectDist - idealObjectRange) * gain_b;
        omega = curvature * V;
        vr = V - width * omega/2;
        vl = V + width * omega/2;
        robot.sendVelocity(vl, vr);
    end
    
    scatter(-y_dist,x_dist,"x", "r");
    hold off;
    
    prev_index = min_index;
    prev_time = toc;
    pause(0.2);
end
