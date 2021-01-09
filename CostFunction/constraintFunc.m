function constr_values = constraintFunc(b,u,k)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constraint function 
%
% Input:
%   b: Current belief vector
%   u: Control input
%   k: Timestep
%
% Outputs:
%   b_next: Updated belief vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     x_bound_3sig = 0.15; %0.13 tight 0.5 loose
%     y_bound_3sig = 0.15; %0.13 tight 0.5 loose
%     th_bound_3sig = 0.15; % 0.05 tight 0.1 loose
    
    x_bound_3sig = 0.15; %0.13 tight 0.5 loose
    y_bound_3sig = 0.15; %0.13 tight 0.5 loose
    th_bound_3sig = 0.15; % 0.05 tight 0.1 loose
    
    
    unc_bound = ([x_bound_3sig;y_bound_3sig;th_bound_3sig]./3).^2;
    if k <= 4
       constr_values = -ones(3,1);
    else
        
        if length(b(:,1)) == 9 % quadPlane or unicycle robot
            constr_values = [b(4);b(7);b(9)] - unc_bound;
        end
        if length(b(:,1)) == 20
            constr_values = [b(6);b(11);b(15)] - unc_bound;
        end

    end

end
