function q_predict = diffDrivePredict_3state(q,u,delta_t)
% DIFFDRIVEPREDICT_3STATE: prediction model based on differential drive
%                          kinematic. Predicts robot's pose based on
%                          previous pose and encoder's data
% 
% Inputs:  
% 
%     q:    1 x 3 array, robot's pose, [x,y,theta]
% 
%     u:    1 x 2 array, odometry information, [v,w]
% 
%     delta_t:    double, the sample time (sec)
% 
% 
% Outputs:
% 
%     q_predict:    1 x 3 array, robot's predicted pose, [x,y,theta]
% 
%   Cornell University
%   CCTA-p project
%   Lou, Wenbo

% ============================ Main function ==============================
x = q(1); % robot's previous x position
y = q(2); % robot's previous y position
t = q(3); % robot's previous heading
V = u(1); % robot's forward velocity, encoder data
W = u(2); % robot's angular velocity, encoder data

s = sin(t);
c = cos(t);

% predict robot's position and orientation based on previous pose
[r,~] = size(q);
q_predict = zeros(r,1);
q_predict(1) = x + V * c * delta_t;
q_predict(2) = y + V * s * delta_t;
q_predict(3) = t + W * delta_t;


end