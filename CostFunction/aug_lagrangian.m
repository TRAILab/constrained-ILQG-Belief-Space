function c = aug_lagrangian(b, u, lagMultiplier, mu, goal,  L, stDim, stateValidityChecker, conFunc,map, D, k, varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute augmented Lagrangian for vector of states according to cost model given in Section 6 
    % of Van Den Berg et al. IJRR 2012 and 
    %
    % Input:
    %   b: Current belief vector
    %   u: Control
    %   goal: target state
    %   lagMultiplier: Lagrange multiplier estimate
    %   mu: Penalty parameter
    %   L: Total segments
    %   stDim: state space dimension for robot
    %   stateValidityChecker: checks if state is in collision or not
    %   map: map containing obstacles and landmarks
    %   D: Duplication matrix for creating full covariance matrix from minimal
    %      representation
    % Outputs:
    %   c: cost estimate
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    c = zeros(1,size(b,2));

    for i=1:size(b,2)
        if isempty(varargin)
            c(1,i) =  evaluateCost(b(:,i),u(:,i), lagMultiplier, mu, goal,  stDim, L, stateValidityChecker,conFunc, map, D,k);
        else
            c(1,i) =  evaluateCost(b(:,i),u(:,i), lagMultiplier, mu, goal,  stDim, L, stateValidityChecker, conFunc, map, D, k, varargin{1});
         end
    end


end

function cost = evaluateCost(b, u, lagMultiplier, mu, goal,  stDim, L, stateValidityChecker, conFunc, map,D,k, varargin)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute cost for a states according to cost model given in Section 6 
    % of Van Den Berg et al. IJRR 2012
    %
    % Input:
    %   b: Current belief vector
    %   u: Control
    %   lagMultiplier: Lagrange multiplier estimate
    %   mu: Penalty parameter
    %   goal: target state
    %   L: Total segments
    %   stDim: state space dimension for robot
    %   stateValidityChecker: checks if state is in collision or not
    %   map: map containing obstacles and landmarks
    %   D: Duplication matrix for creating full covariance matrix from minimal
    %      representation
    %   k: Timestep
    % Outputs:
    %   c: cost estimate
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    final = isnan(u(1,:));
    u(:,final)  = 0;

    ctrlDim = size(u,1);

    x = b(1:stDim,1);

    vecP = D*b(stDim+1:end);
    P = reshape(vecP,stDim,stDim); % Covariance Matrix

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

    % control cost
    uc = 0;

    % final cost
    if any(final)

      sc = delta_x'*Q_l*delta_x;

      pc = penalty_func(b, 0, lagMultiplier, mu, k,conFunc);

    else

      uc = u'*R_t*u;

      if k == 1
          pc = 0;
      else
          pc = penalty_func(b, u, lagMultiplier, mu, k, conFunc);
      end

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
        nSigma = sigmaToCollide2(b,stDim,D,map);
    %     cc = sum(exp(-nSigma));
%         nSigma = sigmaToCollide(b,stDim,D,stateValidityChecker);
        cc = exp(-nSigma);   
%     cc = -log(chi2cdf(nSigma^2, stDim-1));

      end
    else
      nSigma = sigmaToCollide2(b,stDim,D,map);
    %   cc = sum(exp(-nSigma));
%         nSigma = sigmaToCollide(b,stDim,D,stateValidityChecker);
        cc = exp(-nSigma);
%     cc = -log(chi2cdf(nSigma^2, stDim-1));
    end



    cost = sc + uc + 350*w_cc*cc + pc;

end

function penalty = penalty_func(b, u, lagMultiplier, mu, k, conFunc)
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute penalty function based on Aoyama et al (2020)
%
% Input:
%   b: Current belief vector
%   u: Control
%   lagMultiplier: Lagrange multiplier estimate
%   mu: Penalty parameter
%   goal: target state
%   L: Total segments
%   stDim: state space dimension for robot
%   stateValidityChecker: checks if state is in collision or not
%   map: map containing obstacles and landmarks
%   D: Duplication matrix for creating full covariance matrix from minimal
%      representation
% Outputs:
%   c: cost estimate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    const_dim = length(lagMultiplier);
    
    con_values = conFunc(b,u, k);
    
    penalty = 0;
    
    for j = 1:const_dim
        t = (mu(j)/lagMultiplier(j))*con_values(j);
       if t > -0.5
           phi = 0.5*t^2 + t;
       else
           phi = -(1/4)*log(-2*t)-(3/8);
       end
       penalty = penalty + (lagMultiplier(j)^2/mu(j))*phi;
           
    end
       
end