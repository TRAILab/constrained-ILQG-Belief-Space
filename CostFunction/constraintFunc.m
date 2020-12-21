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
%     x_bound_3sig = 0.13; %0.13 tight 0.5 loose
%     y_bound_3sig = 0.13; %0.13 tight 0.5 loose
%     th_bound_3sig = 0.05; % 0.05 tight 0.1 loose
    
    x_bound_3sig = 0.25; %0.13 tight 0.5 loose
    y_bound_3sig = 0.25; %0.13 tight 0.5 loose
    th_bound_3sig = 0.2; % 0.05 tight 0.1 loose
    
    
    unc_bound = ([x_bound_3sig;y_bound_3sig;th_bound_3sig]./3).^2;
    if k <= 4
       constr_values = -ones(3,1);
    else
        constr_values = [b(4);b(7);b(9)] - unc_bound;
    end

end
