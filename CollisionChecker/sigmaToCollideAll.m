function nSigma = sigmaToCollideAll(b,stDim,stateValidityChecker,map)
%%%%%%%%%%%%%%%%%%%%%%%
% Compute std devs to collision based on 
% Section 5 of Van den Berg et al. IJRR 2012
%
% Inputs:
%   b: belief state
%   stateValidityChecker: function for checking collision
%
% Output:
%   nSigma: number of std devs robot should deviate to collide
%%%%%%%%%%%%%%%%%%%%%%

nSigma = 5.23*ones(size(map.obstacles,2),size(b,2));

global ROBOT_RADIUS
R_orig =  ROBOT_RADIUS; % save robot radius
    
for i = 1:size(b,2)
    
    x = b(1:stDim,i);

    P = zeros(stDim, stDim); % covariance matrix

    % Extract columns of principal sqrt of covariance matrix
    % right now we are not exploiting symmetry
    for d = 1:stDim
        P(:,d) = b(d*stDim+1:(d+1)*stDim, i);
    end
    
    
    eigval = eig(P(1:stDim-1,1:stDim-1)); % get eigen values

    lambda = max(eigval); % get largest eigen val


    d = sqrt(lambda); % distance along 1 std dev
    

    % number of standard deviations at which robot collides
    % at s = 0, f goes to infinite so not good -> better to use small value of 0.01
%     for s = 0.01:0.05:5.22
% 
%         % inflate robot radius 
%         ROBOT_RADIUS = R_orig + s*d;
% 
%         % if robot collided
%         [coll_free,distances] = stateValidityChecker(x);
%         if coll_free == 0    
%             
%             nSigma(i) = s;            
%             
%             break;
%         end
%     end

    [coll_free, c2c_dist] = stateValidityChecker(x);
    nSigma(:,i) = c2c_dist./d;
    if ~isreal(nSigma(:,i))
        i
    end

    
end

ROBOT_RADIUS = R_orig; % reset robot radius
end