function y = wheel2FwdAngVel(wr,wl)
% WHEEL2FWDANGVEL: convert wheels' angular velocity to robot's forward and
%                  angular velocity.
% 
% Inputs:
% 
%       wr: double, right wheel's average angular rate, (rad/s)
% 
%       wl: double, left wheel's average angular rate, (rad/s)
% 
% Outputs:
% 
%       y: 1-by-2 array, [v,w], where v is the robot's forward velocity
%          (m/s), w is the robot's angular rate (rad/s)
% 
%   Cornell University
%   CCTA-p project
%   Lou, Wenbo

r = 0.0143; % wheel's radius
L = 0.16; % base width

% convert wheels' angular rate to robot's forward and angular velocity
w = (r/L) * (wr-wl);
v = (r/2) * (wr+wl);

y = [v,w];
end