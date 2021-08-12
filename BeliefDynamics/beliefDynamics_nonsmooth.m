function b_next = beliefDynamics_nonsmooth(b, u, motionModel, obsModel)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Propagate beliefs according to approach given in Section 4.1 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control input
%   motionModel: Robot motion model
%   obsModel: Observation model
%
% Outputs:
%   b_next: Updated belief vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

horizon = size(b,2);

b_next = zeros(size(b));

for i=1:horizon    
    b_next(:,i) = updateSingleBelief(b(:,i), u(:,i), motionModel, obsModel);
end


end

function b_next = updateSingleBelief(b, u, motionModel, obsModel)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Propagate single belief according to approach given in Section 4.1 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control input
%   motionModel: Robot motion model
%   obsModel: Observation model
%
% Outputs:
%   b_next: Updated belief vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isnan(u(1,:))
    u = zeros(size(u));
end

% get the state space dimension
stDim = motionModel.stDim;

% Extract robot state
x = b(1:stDim,1);



% Extract columns of principal sqrt of covariance matrix
% right now we are not exploiting symmetry
D = motionModel.D;
vecP = D*b(stDim+1:end);
P = reshape(vecP,stDim,stDim); % Covariance Matrix

% update state
processNoise = motionModel.zeroNoise; % 0 process noise
x_next = motionModel.evolve(x,u,processNoise); 

% Get motion model jacobians
A = motionModel.getStateTransitionJacobian(x,u,processNoise);
G = motionModel.getProcessNoiseJacobian(x,u,processNoise);
Q = motionModel.getProcessNoiseCovariance(x,u);

% Get observation model jacobians
[z,vis] = obsModel.getObservation(x_next, 'nonoise');
obsNoise = zeros(size(z));
H = obsModel.getObservationJacobian(x,obsNoise);
H = H(vis==1,:); % only visible features
z = z(vis==1); % Only visible features
% variances_all_j = repmat([obsModel.var],length(z)/obsModel.obsDim,1);
% R = diag(variances_all_j);

variances_all_j = repmat(obsModel.var,length(obsModel.landmarkIDs),1);
R = diag(variances_all_j);

R = zeros(length(obsModel.landmarkIDs)*obsModel.obsDim + 2, length(obsModel.landmarkIDs)*obsModel.obsDim + 2);
R(1:end-2,1:end-2) = diag(variances_all_j);
R(end-1:end,end-1:end) = [ obsModel.encoder_std^2, 0;
                                        0, obsModel.encoder_std^2];
R = R(vis==1,vis==1);

% % update P 
P_prd = A*P*A' + G*Q*G';

if isempty(z)
        P_next = P_prd;
else
%     S = H*P_prd*H' + M*R*M';
%     K = (P_prd*H')/S;
%     P_next = (eye(stDim) - K*H)*P_prd;
    %information form
    innovation_info = (H'/R)*H;
    P_next_inv = inv(P_prd) + innovation_info;
    P_next = inv(P_next_inv);
end



% update belief
% W = zeros(stDim+stDim^2,2);
% W(1:stDim,:) = sqrtm(K*H*T);
% 
% w = mvnrnd(zeros(1,stDim), eye(stDim),1);
% w = w';

g_b_u = zeros(size(b));
g_b_u(1:stDim,1) = x_next;

%Store diagonal and below of covariance matrix

g_b_u(stDim+1:end,1) = motionModel.D_pseudoinv*P_next(:);

% for d = 1:stDim
%     g_b_u(d*stDim+1:(d+1)*stDim,1) = P_next(:,d);
% end

b_next = g_b_u ;%+ W*w;

end
% 
% function b_next = updateSingleBelief(b, u, motionModel, obsModel)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Propagate single belief according to approach given in Section 4.1 
% % of Van Den Berg et al. IJRR 2012
% %
% % Input:
% %   b: Current belief vector
% %   u: Control input
% %   motionModel: Robot motion model
% %   obsModel: Observation model
% %
% % Outputs:
% %   b_next: Updated belief vector
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% if isnan(u(1,:))
%     u = zeros(size(u));
% end
% 
% % get the state space dimension
% stDim = motionModel.stDim;
% 
% % Extract robot state
% x = b(1:stDim,1);
% 
% sqrtSigma = zeros(stDim, stDim); % principal sqrt of covariance matrix
% 
% % Extract columns of principal sqrt of covariance matrix
% % right now we are not exploiting symmetry
% for d = 1:stDim
%     sqrtSigma(:,d) = b(d*stDim+1:(d+1)*stDim, 1);
% end
% 
% % update state
% processNoise = motionModel.zeroNoise; % 0 process noise
% x_next = motionModel.evolve(x,u,processNoise); 
% 
% % Get motion model jacobians
% A = motionModel.getStateTransitionJacobian(x,u,processNoise);
% G = motionModel.getProcessNoiseJacobian(x,u,processNoise);
% 
% % Get observation model jacobians
% z = obsModel.getObservation(x_next, 'nonoise');
% obsNoise = zeros(size(z));
% H = obsModel.getObservationJacobian(x,obsNoise);
% M = obsModel.getObservationNoiseJacobian(x,obsNoise,z);
% 
% % update sqrtSigma 
% T = A*sqrtSigma*(A*sqrtSigma)' + G*G';
% K = (T*H')/(H*T*H' + M*M');
% sqrtSigma_next = sqrtm(T - K*H*T);
% 
% % update belief
% % W = zeros(stDim+stDim^2,2);
% % W(1:stDim,:) = sqrtm(K*H*T);
% % 
% % w = mvnrnd(zeros(1,stDim), eye(stDim),1);
% % w = w';
% 
% g_b_u = zeros(size(b));
% g_b_u(1:stDim,1) = x_next;
% 
% for d = 1:stDim
%     g_b_u(d*stDim+1:(d+1)*stDim,1) = sqrtSigma_next(:,d);
% end
% 
% b_next = g_b_u ;%+ W*w;
% 
% end