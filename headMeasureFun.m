function measurement_predict = headMeasureFun(q_predict)
% DIFFDRIVEPREDICT_3STATE: measurement model of heading
% 
% Inputs:  
% 
%         q_predict:    1 x 3 array, robot's predicted pose, [x,y,theta]
% 
% Outputs:
% 
%         measurement_predict:  double, robot's predicted heading
% 
%   Cornell University
%   CCTA-p project
%   Lou, Wenbo

% take sin of the robot's orientation
measurement_predict = sin(q_predict(3));

end