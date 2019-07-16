function H = HjacHead(miu_predict)
% DIFFDRIVEJAC_3STATE: Compute the Jacobian of measurement model using
%                      finite difference method.
% 
% Inputs:  
% 
%     miu_predict:    1 x 3 array, robot's prediected pose, [x,y,theta]
% 
% Outputs:
% 
%     G:    1 x 3 matrix, Jacobian of measurement model
% 
%   Cornell University
%   CCTA-p project
%   Lou, Wenbo

% ============================ Main function =============================
H = zeros(1,3); %  initialize output matrix

q_perturbed = miu_predict;
q_perturbed(3) = miu_predict(3) + 0.0001; %  perturb heading
measurement_predict_new = headMeasureFun(q_perturbed);
measurement_predict = headMeasureFun(miu_predict);
H(3) = (measurement_predict_new - measurement_predict) / 0.0001; %  finite difference

end