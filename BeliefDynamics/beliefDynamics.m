function b_next = beliefDynamics(b, u, motionModel, obsModel)
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
z = obsModel.getObservation(x_next, 'nonoise');
obsNoise = zeros(size(z));
H = obsModel.getObservationJacobian(x,obsNoise);
%M = obsModel.getObservationNoiseJacobian(x,obsNoise,z);
%R = obsModel.getObservationNoiseCovariance(x,z);
R_inv = obsModel.getObservationNoiseCovarianceInverse(x,z);

% % update P 
P_prd = A*P*A' + G*Q*G';

% %information form
% innovation_info = H'*R_inv*H;
% if rank(innovation_info)<stDim
%     P_next = P_prd;
% else
%     P_next_inv = inv(P_prd) + innovation_info;
%     P_next = inv(P_next_inv);
% end

%information form
innovation_info = H'*R_inv*H;
P_next_inv = inv(P_prd) + innovation_info;
P_next = inv(P_next_inv);

if nnz(eigs(P_next)<0)>0
    b
end


P_next = 0.5.*(P_next + P_next.');


g_b_u = zeros(size(b));
g_b_u(1:stDim,1) = x_next;

%Store diagonal and below of covariance matrix

g_b_u(stDim+1:end,1) = motionModel.D_pseudoinv*P_next(:);


b_next = g_b_u ;%+ W*w;

end
