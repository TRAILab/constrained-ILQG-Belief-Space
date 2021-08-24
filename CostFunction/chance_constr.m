function constr_values = chance_constr(b,u,k,map,stDim, D, del_r)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constraint function 
%
% Input:
%   b: Current belief vector
%   u: Control input
%   k: Timestep
%
% Outputs:
%   constr_values: constraint violation values,
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    global ROBOT_RADIUS
    r_k =  ROBOT_RADIUS; % save robot radius
    x = b(1:2);

    vecP = D*b(stDim+1:end);
    P = reshape(vecP,stDim,stDim); % Covariance Matrix
    P = P(1:2,1:2);
    J = length(map.obstacles(1,:));
    
    obs = map.obstacles-0.5.*[map.obstacle_sizes;map.obstacle_sizes];
    r_ko_mean = x(1:2) - obs;
    dist_mean = vecnorm(r_ko_mean);
    a_ko = r_ko_mean./dist_mean;
    var_normal = zeros(J,1);
    
    for j = 1:J
        cov_normal_j = 2*a_ko(:,j).'*P*a_ko(:,j);
        if cov_normal_j<0
            var_normal(j) = sqrt(2*max(diag(P)));
        else
            var_normal(j) = sqrt(cov_normal_j);
        end
    end
    
    constr_values = repmat(r_k,J,1) + (sqrt(2)/2).*map.obstacle_sizes.' + erfinv(1 - 2*del_r).*var_normal - dist_mean.';
    
    
end
