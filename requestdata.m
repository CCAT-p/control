function sensorData = requestdata(t)
% REQUESTDATA: send request to server and read sensor data.
% 
% Inputs:
% 
%       t:    tcp object, created before calling this function
% 
% Outputs:
% 
%       sensorData:  1-by-8 array, [ax,ay,az,heading,w1,w2,w3,w4], where
%                    the first 3 entries are accelerometer data, 
%                    4th entry is the robot's heading, the rest are the 
%                    wheels' angular velocities (w1,w3:right, w2,w4:left).
% 
% 
%   Cornell University
%   CCTA-p project
%   Eddie


% write to server to request sensor data
fwrite(t, 'SSSSSSSSSSSS');

% initialize a flag varibale, in case sensor data is not immediately available 
i = 0;
while ~i
    if t.BytesAvailable
        bytes = fread(t, [1,t.BytesAvailable]);
        data = typecast(uint8(bytes),'single');
        sensorData = data;
        i = 1;

    else
        i = 0;
    end
end
    
    
end
