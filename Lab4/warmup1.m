% clear all;
robot = raspbot('Raspbot-8');
% robot = raspbot('sim');
pause(3);
% /* Feedforward control constants */
FEED_WEIGHT = 0.5;
BALANCE_WEIGHT = 1;

% /* PID control constants */
PID_PTERM = 2.0;
PID_ITERM = 0.0;
PID_DTERM = 0.2;
PID_I_CAP = 10000; % maximum error added to I term
MAX_PID_ADJ = 100;
MIN_PID_DIFF = 0; % minimum error considered by control

% /* Global control variables */
balance_output = 0;
in_place_feedforward = 0;
pid_integral = 0;

% /* List(Array) variables */
time_array = zeros(1);
error_array = zeros(1);
dist_array = zeros(1);
vel_array = zeros(1);
real_vel_array = zeros(1);
real_dist_array = zeros(1);
diff_array = zeros(1);

% initialize encoder values
left_initial = robot.encoders.LatestMessage.Vector.X;
right_initial = robot.encoders.LatestMessage.Vector.Y;

% claim variables
goal = 1.0;
new_time = 0.0;
left_error_prev = 0.0;
right_error_prev = 0.0;
vel = 0.0;
amax = 0.75;
vmax = 0.25;
sgn = 1;
cur_dist = 0.0;
cur_vel = 0.0;
prev_vel = 0.0;
t_delay = 0.21;

% Continuous plot
figure;
f = plot(time_array,error_array);
hold on 
f1 = plot(time_array,dist_array);
f2 = plot(time_array, real_dist_array);
% hold off
% figure;
f3 = plot(time_array, diff_array);
hold off
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
    d_error_left = (left_error - left_error_prev)/dt;
    d_error_right = (right_error - right_error_prev)/dt;
    if (left_error + right_error)/2 >= 0
        sgn = 1;
    else
        sgn = -1;
    end
    
%     u_left = PID_PTERM * left_error + PID_DTERM * d_error_left;
%     u_right = PID_PTERM * right_error + PID_DTERM * d_error_right;
%     v_left = vel + min(u_left, vamx);
%     v_right = vel + min(u_right, vmax);
%     robot.sendVelocity(v_left, v_right);

    u_ref = trapezoidalVelocityProfile(new_time-t_delay, amax, vmax, goal, sgn);
    u_feedforward = trapezoidalVelocityProfile(new_time, amax, vmax, goal, sgn);
    robot.sendVelocity(u_feedforward, u_feedforward);
    
%     Plot arrays over time
    cur_vel = u_ref;
    cur_dist = cur_dist + prev_vel * dt;
    vel_array(end+1) = cur_vel;
    real_vel_array(end+1) = u_feedforward;
    dist_array(end+1) = cur_dist;
    real_dist_array(end+1) = (left_elapse + right_elapse)/2;
    error_array(end+1) = (left_error + right_error)/2;
    diff_array(end+1) = cur_dist - (left_elapse + right_elapse)/2;
    time_array(end+1) = new_time;
    set(f, 'XData', time_array);
    set(f, 'YData', vel_array);
    set(f1, 'XData', time_array);
    set(f1, 'YData', dist_array);
    set(f2, 'XData', time_array);
    set(f2, 'YData', real_dist_array);
    set(f3,'YData', real_vel_array);
    set(f3,'XData', time_array);
%     store errors and time
    left_error_prev = left_error;
    right_error_prev = right_error;
    prev_time = new_time;
    prev_vel = cur_vel;
    pause(0.02);
end
robot.sendVelocity(0,0);
pause(0.2);
robot.shutdown();
