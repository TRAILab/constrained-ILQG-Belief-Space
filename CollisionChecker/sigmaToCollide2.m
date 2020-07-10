function min_ct = sigmaToCollide2(b,stDim,map)
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
    
    L = chol(P(1:2,1:2));
    max_std =sqrt(max(eig(P(1:2,1:2))));
    
    x_tf = L\x(1:2);
    
    min_ct = 100;
    
    for j = 1:length(map.obstacles(1,:))
        obs_j = L\(map.obstacles(:,j)-0.5.*[map.obstacle_sizes(j);map.obstacle_sizes(j)]);
        ct = norm(obs_j - x_tf) - (R_orig + (sqrt(2)/2)*map.obstacle_sizes(j))/max_std;
        if ct<0
            min_ct = 0.01;
            break
        end
        if ct < min_ct
           min_ct = ct; 
        end
    end

    
end

end