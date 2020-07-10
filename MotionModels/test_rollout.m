function test_rollout(map,figh,motion_model,x_nom,u_nom)
%TEST_ROLLOUT Summary of this function goes here
%   Detailed explanation goes here


    T = length(u_nom(1,:));
    %plot(x_nom(1,:),x_nom(2,:),'b-')
    
    x_k = map.start;
    x_rollout = zeros(motion_model.stDim,T+1);
    x_rollout(:,1) = x_k;
    for k = 1:T
        x_k_plus = motion_model.evolve(x_k,u_nom(:,k),motion_model.zeroNoise);
        x_rollout(:,k+1) = x_k_plus;
        x_k = x_k_plus;
    end
    plot(x_rollout(1,:),x_rollout(2,:),'r-')
    
end

