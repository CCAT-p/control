function G = diffDriveJac_3state(q,u,t_delta)
% DIFFDRIVEJAC_3STATE: Compute the Jacobian of prediction model using
%                      finite difference method.
% 
% Inputs:  
% 
%     q:    1 x 3 array, robot's previous pose, [x,y,theta]
% 
%     u:    1 x 2 array, odometry information, [v,w]
% 
%     t_delta:     double, the sample time (sec)
% 
% Outputs:
% 
%     G:    3 x 3 matrix, Jacobian of prediction model
% 
%   Cornell University
%   CCTA-p project
%   Lou, Wenbo

% ============================== Main function ============================
% finite difference
delta = 0.0001;

miu_predict = diffDrivePredict_3state(q,u,t_delta);
G = zeros(3,3);
for j = 1:3 % perturb each of the 3 states and see how the output changes
    q_perturbed = q;
    q_perturbed(j) = q_perturbed(j) + delta; % perturb each of the 3 states
    G(:,j) = (diffDrivePredict_3state(q_perturbed,u,t_delta) - miu_predict) / delta; % finite differnece
end


end