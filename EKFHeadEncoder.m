function [miu_update, sigma_update] = EKFHeadEncoder(u,measurement,miu_prev,sigma_prev,t_delta)
% EKFHEADENCODER: Extended-Kalman Filter. Use encoder's data and heading data
%                 to estimate robot's 2D position and orienation.
% 
% Inputs:
% 
%       u:    1-by-2 array, [v,w], robot's forward and angular velocity,
%             computed from encoders' data, input to prediction model
% 
%       measurement:    double, robot's heading computed from IMU data
% 
%       miu_prev:    3-by-1 array, [x,y,theta]', previous belief of state
% 
%       sigma_prev:    3-by-3 matrix, previous confidence of belief
% 
%       t_delta:    double, time elapse (sec) since previous call of this function
% 
% Outputs:
% 
%       miu_update:    3-by-1 array, updated belief
%             
%       sigma_update:    3-by-3 matrix, updated confidence matrix
% 
% 
%   Cornell University
%   CCTA-p project
%   Lou, Wenbo


% set noise covariance matrices
R(1:3,1:3) = [0.01  , 0.001 , 0.002;
              0.001 , 0.01  , 0.002;
              0.002 , 0.002 , 0.02]; % process noise
Q = 0.01; % sensor noise

% ========================= Prediction  ==================================
miu_predict = diffDrivePredict_3state(miu_prev,u,t_delta); % prediction model
G = diffDriveJac_3state(miu_prev,u,t_delta); % prediction model Jacobian
sigma_predict = G * sigma_prev * G' + R; % predict covariance

% ========================= Measurement ==================================
z_actual = sin(deg2rad(measurement)); % call measurement data
z_expect = headMeasureFun(miu_predict); % compute expected measurement
H = HjacHead(miu_predict); % compute measurement model Jacobian

% ============================ Update ====================================
K = sigma_predict * H' * inv(H * sigma_predict * H' + Q); % Kalman gain
miu_update = miu_predict + K * (z_actual - z_expect); % updates belief
sigma_update = (eye(3) - K * H) * sigma_predict; % updates covariance


end