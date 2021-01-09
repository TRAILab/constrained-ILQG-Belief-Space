function c = costFunction(b, u, goal, L, stDim, stateValidityChecker, map, D, varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for vector of states according to cost model given in Section 6 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control
%   goal: target state
%   L: Total segments
%   stDim: state space dimension for robot
%   stateValidityChecker: checks if state is in collision or not
% Outputs:
%   c: cost estimate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

c = zeros(1,size(b,2));

for i=1:size(b,2)
    if isempty(varargin)
        c(i) =  evaluateCost(b(:,i),u(:,i), goal, stDim, L, stateValidityChecker,map, D);
    else
        c(i) =  evaluateCost(b(:,i),u(:,i), goal, stDim, L, stateValidityChecker, map, D, varargin{1});
    end
end

end

function cost = evaluateCost(b, u, goal, stDim, L, stateValidityChecker, map,D, varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for a states according to cost model given in Section 6 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control
%   goal: target state
%   stDim: State dimension
%   L: Number of steps in horizon
%   stateValidityChecker: checks if state is in collision or not
% Outputs:
%   c: cost estimate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

final = isnan(u(1,:));
u(:,final)  = 0;

ctrlDim = size(u,1);

x = b(1:stDim,1);

vecP = D*b(stDim+1:end);
P = reshape(vecP,stDim,stDim); % Covariance Matrix

Q_t = 1000; % penalize uncertainty
Q_l = 100*L*eye(stDim); % penalize terminal error

if ctrlDim == 2 % quadPlane robot
    R_t = 4*eye(ctrlDim);
    Q_l(3,3) = 0;
end
if ctrlDim == 3
    R_t = 3*eye(ctrlDim);
    R_t(3,3) = 1;
    Q_l(3,3) = 0;
end
if ctrlDim == 4 % carPanTilt
	R_t = 4*eye(ctrlDim); % penalize control effort
    R_t(3:4,3:4) = eye(2);
    Q_l(3:5,3:5) = zeros(3,3);
end



w_cc = 1.0; % penalize collision

% deviation from goal
delta_x = zeros(stDim,1);
delta_x(1:2) = goal(1:2)-x(1:2);
att_x = [cos(x(3));sin(x(3))];
att_goal = [cos(goal(3));sin(goal(3))];
delta_x(3) = acos(att_goal.'*att_x);

% collision cost
cc = 0;

% State Cost
% sc = 0.001.*(delta_x'*Q_l*delta_x);
sc = 0;

% information cost
ic = 0;

% control cost
uc = 0;

% final cost
if any(final)
    
  sc = delta_x'*Q_l*delta_x;
  
  ic = Q_t*sum(diag(P));
  
else
      
  ic = Q_t*sum(diag(P));
  
  uc = u'*R_t*u;
  
end
  % if extra arg is 1, get collision cost or if no extra arg, default behaviour
% if ~isempty(varargin)
%   if varargin{1} == 1
%     nSigma = sigmaToCollide(b,stDim,stateValidityChecker);
% %     nSigma = sigmaToCollide2(b,stDim,map);
% 
%     cc = -log(chi2cdf(nSigma^2, stDim-1));     
%   end
% else
%   nSigma = sigmaToCollide(b,stDim,stateValidityChecker);
% %     nSigma = sigmaToCollide2(b,stDim,map);
%   cc = -log(chi2cdf(nSigma^2, stDim-1));      
% end

if ~isempty(varargin)
  if varargin{1} == 1
%     nSigma = sigmaToCollideAll(b,stDim,stateValidityChecker,map);
%     cc = sum(exp(-nSigma));
%     nSigma = sigmaToCollide(b,stDim,D,stateValidityChecker);
    nSigma = sigmaToCollide2(b,stDim,D,map);
%     cc = -log(chi2cdf(nSigma^2, stDim-1));
     cc = exp(-nSigma);

  end
else
%   nSigma = sigmaToCollideAll(b,stDim,stateValidityChecker,map);
%   cc = sum(exp(-nSigma));
%     nSigma = sigmaToCollide(b,stDim,D,stateValidityChecker);
    nSigma = sigmaToCollide2(b,stDim,D,map);
%     cc = -log(chi2cdf(nSigma^2, stDim-1));
    cc = exp(-nSigma);
end
  


cost = sc + ic + uc + 350*w_cc*cc;

end