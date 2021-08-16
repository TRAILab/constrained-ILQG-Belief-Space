function plot_convergence(traces)
%PLOT_CONVERGENCE Plots convergence plots of multiple runs
%   Inputs: 
%   Traces - array of trace structures 

    no_runs = length(traces(1,:));
    
    for i = 1:no_runs
        % Retrieve traces
        trace= traces(:,i);
        iters = [0, trace.iter];
        costs = [trace(1).init_cost, trace.cost];
        plot(iters,costs);
        hold on       
    end
    xlabel('Iterations','Interpreter','Latex')
    ylabel('Cost','Interpreter','Latex')
    legend({'iLQG with visibility modelling, holonomic', 'iLQG, holonomic',...
        'iLQG with visibility smoothing, nonholonomic', 'iLQG, nonholonomic'},'Interpreter','Latex')
end

