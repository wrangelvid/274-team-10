function output_data = Experiment_trajectory( angle1_init, angle2_init, traj_time, pre_buffer_time, post_buffer_time, motor1_act_time, motor2_act_time, duty_max)
    
    % Figure for plotting motor data
    figure(1);  clf;       
    a1 = subplot(421);
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Angle 1 (rad)');
    title('Joint 1');
    
    a2 = subplot(423);
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity 1 (rad/s)');
    
    a3 = subplot(425);
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Current 1 (A)');
    hold on;
    subplot(425);
    h4 = plot([0],[0],'r');
    h4.XData = []; h4.YData = [];
    hold off;
    
    a4 = subplot(427);
    h5 = plot([0],[0]);
    h5.XData = []; h5.YData = [];
    ylabel('Duty Cycle 1');
    

    a5 = subplot(422);
    h21 = plot([0],[0]);
    h21.XData = []; h21.YData = [];
    ylabel('Angle 2 (rad)');
    title('Joint 2');
    
    a6 = subplot(424);
    h22 = plot([0],[0]);
    h22.XData = []; h22.YData = [];
    ylabel('Velocity 2 (rad/s)');
    
    a7 = subplot(426);
    h23 = plot([0],[0]);
    h23.XData = []; h23.YData = [];
    ylabel('Current 2 (A)');
    hold on;
    subplot(426);
    h24 = plot([0],[0],'r');
    h24.XData = []; h24.YData = [];
    hold off;
    
    a8 = subplot(428);
    h25 = plot([0],[0]);
    h25.XData = []; h25.YData = [];
    ylabel('Duty Cycle 2');
    
%     % Figure for plotting state of the leg
%     figure(2)
%     clf
%     hold on
%     axis equal
%     axis([-.25 .25 -.25 .1]);
%    
%     h_OB = plot([0],[0],'LineWidth',2);
%     h_AC = plot([0],[0],'LineWidth',2);
%     h_BD = plot([0],[0],'LineWidth',2);
%     h_CE = plot([0],[0],'LineWidth',2);
%     
%     h_foot= plot([0],[0],'k');
%     h_des = plot([0],[0],'k--');
%     h_des.XData=[];
%     h_des.YData=[];
%     h_foot.XData=[];
%     h_foot.YData=[];
%     
%     % Define leg length parameters
%     l_OA = 0.011; 
%     l_OB = 0.042; 
%     l_AC = 0.096; 
%     l_DE = 0.091;
% 
%     p   = [l_OA l_OB l_AC l_DE]';
%     
    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        % Parse new data
        t = new_data(:,1);          % time

        pos1 = new_data(:,2);       % position
        vel1 = new_data(:,3);       % velocity
        cur1 = new_data(:,4);       % current
        dcur1 = new_data(:,5);      % desired current
        duty1 = new_data(:,6);      % command
        
        pos2 = new_data(:,7);       % position
        vel2 = new_data(:,8);       % velocity
        cur2 = new_data(:,9);       % current
        dcur2 = new_data(:,10);     % desired current
        duty2 = new_data(:,11);     % command  
        
        N = length(pos1);
        
        % Update motor data plots
        h1.XData(end+1:end+N) = t;   
        h1.YData(end+1:end+N) = pos1; % switch sign on all plotted values due to direction motors are mounted
        h2.XData(end+1:end+N) = t;   
        h2.YData(end+1:end+N) = vel1;
        h3.XData(end+1:end+N) = t;   
        h3.YData(end+1:end+N) = cur1;
        h4.XData(end+1:end+N) = t;   
        h4.YData(end+1:end+N) = dcur1;
        h5.XData(end+1:end+N) = t;   
        h5.YData(end+1:end+N) = duty1;
        
        h21.XData(end+1:end+N) = t;   
        h21.YData(end+1:end+N) = pos2;
        h22.XData(end+1:end+N) = t;   
        h22.YData(end+1:end+N) = vel2;
        h23.XData(end+1:end+N) = t;   
        h23.YData(end+1:end+N) = cur2;
        h24.XData(end+1:end+N) = t;   
        h24.YData(end+1:end+N) = dcur2;
        h25.XData(end+1:end+N) = t;   
        h25.YData(end+1:end+N) = duty2;
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    %params.timeout  = 2;            % end of experiment timeout
    
    % Parameters for tuning
    start_period                = pre_buffer_time;    % In seconds 
    end_period                  = post_buffer_time;   % In seconds
    
    % Specify inputs
    input = [start_period traj_time end_period];
    input = [input angle1_init angle2_init];
    input = [input motor1_act_time motor2_act_time];
    input = [input duty_max];
    
    params.timeout  = (start_period+traj_time+end_period);  
    
    output_size = 11;    % number of outputs expected
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    linkaxes([a1 a2 a3 a4],'x')
    
end