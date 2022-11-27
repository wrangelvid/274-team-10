% Set Inputs

% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = -pi/2;
angle2_init = -pi/2; 

% Total experiment time is buffer,trajectory,buffer
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 2;
traj_time         = 5;

motor1_act_time = 2;
motor2_act_time = 2.5;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 0.75;

% Run Experiment
[output_data] = Experiment_trajectory( angle1_init, angle2_init,...
                                       traj_time, pre_buffer_time, post_buffer_time,...
                                       motor1_act_time, motor2_act_time,...
                                       duty_max);