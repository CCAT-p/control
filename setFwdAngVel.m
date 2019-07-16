function setFwdAngVel(FwdVel,AngVel,t)
% MAINFUNCTION: send velocity command to off-board motor driver. The
%               function takes robot's forward and angular velocity command.
% 
% Inputs:
% 
%       FwdVel:    double, forward velocity commmand, m/s
% 
%       AngVel:    double, angular velocity commmand, rad/s
% 
%       t:    tcp object, created before calling this function
% 
% 
%   Cornell University
%   CCTA-p project
%   Lou, Wenbo

% =========================== Main function =============================

% convert forward and angular velocity command to wheels' angular rate
wheelspeed = vel2wheel(FwdVel,AngVel);

% send 12 characters, dicard the rest if there are more than 12
str1 = num2str(wheelspeed(1));
str2 = num2str(wheelspeed(2));
n1 = length(str1);
n2 = length(str2);
if n1 > 5
    n1 = 5;
end
if n2 > 5
    n2 = 5;
end
speedcommand = strcat(str1(1:n1),',',str2(1:n2),',');

% send speed command through tcp/ip
fwrite(t, speedcommand);

end