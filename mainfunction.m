function [Miu, Sigma] = mainfunction(tmax,t)
% MAINFUNCTION: control the motion of the robot. The function read sensor
%               data and use localization algorithm to determine the
%               robot's position and orientation in 2D plane. 
% 
% Inputs:
% 
%       tmax: integer, max running time (sec) of the main control loop
% 
%       t:    tcp object, created before calling this function
% 
% Outputs:
% 
%       Miu:  n-by-3 matrix, time history of robot's position and
%             orientation estimation.
% 
%       Sigma: one cell of n 3-by-3 matrices, time history of estimation
%              confidence
% 
%   Cornell University
%   CCTA-p project
%   Lou, Wenbo


% initialize data store
global IMU_data
global heading
global encoder_data
IMU_data = [];
encoder_data = [];
heading = [];


% initialize localization output
Miu(:,1) = [0;0;0]; % EKF initial belief
Miu_dead(:,1) = [0;0;0]; % Dead-reckoning initial belief
Sigma = {}; % EKF initial confidence
Sigma{1} = 0.0001*eye(3);


% ============================ Main Loop =================================
tic 
while toc < tmax
    
%   ======================== request sensor data ========================
    sensor_data = requestdata(t);
    ax = -sensor_data(1);  ay = -sensor_data(2);  az = sensor_data(3);    
    IMU_data = [IMU_data; ax,ay,az];
    heading = [heading; sensor_data(4)];
    encoder_data = [encoder_data; sensor_data(5:8)];

%   ==========================   Localization   ==========================
    u = wheel2FwdAngVel((encoder_data(end,1) + encoder_data(end,3))/2,(encoder_data(end,2) + encoder_data(end,4))/2);
    measurement = heading(end); %  measurement data
    miu_prev = Miu(:,end); %  EKF previous belief
    sigma_prev = Sigma{end}; %  EKF previous confidence
    q = Miu_dead(:,end); %  Dead-reckoning previous belief

%   EKF
    [miu_update, sigma_update] = EKFHeadEncoder(u,measurement,miu_prev,sigma_prev,0.1);
    
%   Dead-rockoning
    q_predict = diffDrivePredict_3state(q,u,0.1);
    
%   store updated belief and confidence
    Miu = [Miu, miu_update];
    Miu_dead = [Miu_dead, q_predict];
    Sigma{end+1} = sigma_update;
    
%   ============================== plot ==================================
    plot(Miu(1,:),Miu(2,:),'r-','Linewidth',3)
    hold on
    plot(Miu_dead(1,:),Miu_dead(2,:),'k-','Linewidth',1)
    axis([-0.2,2,-0.2,2])
    hold off

%   ============================= control ================================
    setFwdAngVel(0.06,0,t);

end

% stop the robot
setFwdAngVel(0,0,t);

end