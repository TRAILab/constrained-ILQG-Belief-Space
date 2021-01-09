function nSigma = sigmaToCollide2(b,stDim,D, map)
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

nSigma = 5.23*ones(1,size(b,2));

for i = 1:size(b,2)
    
    x = b(1:stDim,i);

    vecP = D*b(stDim+1:end,i);
    P = reshape(vecP,stDim,stDim); % Covariance Matrix
    
%     if min(eigs(P),[],'all')<=0
%        i 
%     end
%     L = chol(P(1:2,1:2));
    max_std =sqrt(max(eig(P(1:2,1:2))));
    
    
%     x_tf = L\x(1:2);
    
    min_ct = 100;
    
    for j = 1:length(map.obstacles(1,:))
%         obs_j = L\(map.obstacles(:,j)-0.5.*[map.obstacle_sizes(j);map.obstacle_sizes(j)]);
        obs_j = (map.obstacles(:,j)-0.5.*[map.obstacle_sizes(j);map.obstacle_sizes(j)]);
        ct = (norm(obs_j - x(1:2)) - (R_orig + (sqrt(2)/2)*map.obstacle_sizes(j)))/max_std;
        
%         if ct<0
%             min_ct = 0.001;
%             break
%         end
        if ct < min_ct
           min_ct = ct; 
        end
    end
    
    nSigma(i) = min_ct;

    
end

end