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
    x_bound_3sig = 0.5;
    y_bound_3sig = 0.5;
    th_bound_3sig = 0.5;
    
    unc_bound = ([x_bound_3sig;y_bound_3sig;th_bound_3sig]./3).^2;
    if k <= 2
       constr_values = -ones(3,1);
    else
        constr_values = [b(4);b(7);b(9)] - unc_bound;
    end

end
