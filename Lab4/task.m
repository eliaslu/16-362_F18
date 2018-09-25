% clear all;
robot = raspbot('Raspbot-7');
% robot = raspbot('sim');
pause(3);
% /* Feedforward control constants */
FEED_WEIGHT = 0.5;
BALANCE_WEIGHT = 1;

% /* PID control constants */
PID_ON = 0;
PID_PTERM = 1.11;
PID_ITERM = 0.0;
PID_DTERM = 0.08;
PID_I_CAP = 10000; % maximum error added to I term
MAX_PID_ADJ = 100;
MIN_PID_DIFF = 0; % minimum error considered by control

% /* Global control variables */
balance_output = 0;
in_place_feedforward = 0;
pid_integral = 0;

% /* List(Array) variables */
time_array = zeros(1);
dist_array = zeros(1);
vel_array = zeros(1);
real_dist_array = zeros(1);
diff_array = zeros(1);

% initialize encoder values
left_initial = robot.encoders.LatestMessage.Vector.X;
right_initial = robot.encoders.LatestMessage.Vector.Y;

% claim variables
goal = 1.0;
new_time = 0.0;
prev_error = 0.0;
amax = 0.75;
vmax = 0.25;
sgn = 1;
cur_dist = 0.0;
cur_vel = 0.0;
prev_vel = 0.0;
if PID_ON
    t_delay = 0.1;
else
    t_delay = 0.385;
end

% Continuous plot
figure;
f1 = plot(time_array,dist_array);
hold on 
f2 = plot(time_array, real_dist_array);
hold off
legend('ref', 'feedback')
xlabel('time(s)')
ylabel('distance(m)')
title('ref and feedback distances over time')
figure;
f3 = plot(time_array, diff_array);
xlabel('time(s)')
ylabel('distance(m)')
title('error of distances over time')
% initialize time
tic;
prev_time = toc;

while new_time < 5.33 + t_delay
    encoder_left = robot.encoders.LatestMessage.Vector.X;
    encoder_right = robot.encoders.LatestMessage.Vector.Y;
    new_time = toc;
    left_elapse = encoder_left - left_initial;
    right_elapse = encoder_right - right_initial;
    dt = new_time - prev_time;
    left_error = goal - left_elapse;
    right_error = goal - right_elapse;
    if abs(left_error) < 0.0001 && abs(right_error) < 0.0001
        robot.sendVelocity(0,0);
        disp('stop');
        break;
    end
    if (left_error + right_error)/2 >= 0
        sgn = 1;
    else
        sgn = -1;
    end

    u_feedforward = trapezoidalVelocityProfile(new_time, amax, vmax, goal, sgn);
    u_ref = trapezoidalVelocityProfile(new_time-t_delay, amax, vmax, goal, sgn);
    cur_vel = u_ref;
    cur_dist = cur_dist + prev_vel * dt;
    real_dist = (left_elapse + right_elapse)/2;
    error = cur_dist - real_dist;
    d_error = (error - prev_error)/dt;
    u_pid = PID_PTERM * error + PID_DTERM * d_error;
    if PID_ON
        u = u_feedforward + u_pid;
    else
        u = u_feedforward;
    end
    
    robot.sendVelocity(u, u);
    
%     Plot arrays over time
    vel_array(end+1) = prev_vel;
    dist_array(end+1) = cur_dist;
    real_dist_array(end+1) = real_dist;
    diff_array(end+1) = error;
    time_array(end+1) = new_time;
    set(f1, 'XData', time_array);
    set(f1, 'YData', dist_array);
    set(f2, 'XData', time_array);
    set(f2, 'YData', real_dist_array);
    set(f3,'YData', diff_array);
    set(f3,'XData', time_array);
    prev_error = error;
    prev_time = new_time;
    prev_vel = u_ref;
    pause(0.002);
end
robot.sendVelocity(0,0);
pause(0.02);
robot.shutdown();
