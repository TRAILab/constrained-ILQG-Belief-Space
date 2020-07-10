function [g,c,gb,gu,gbb,gbu,guu,cb,cu,cbb,cbu,cuu] = beliefDynCost2(b,u,xf,L,full_DDP,motionModel,obsModel, collisionChecker, map)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A utility function that combines belief dynamics and cost
% uses helper function finite_difference() to compute derivatives
% Inputs:
%   b: belief
%   u: controls
%   xf: target state
%   L: Total segments
%   full_DDP: whether to use 2nd order derivates of dynamics
%   motionModel: robot's motion model
%   obsModel: Sensing model
%   collisionChecker: collision checking with obstacles
%
% Outputs:
%   g: belief update using belief dynamics
%   c: cost 
%   gx: belief dynamics derivate w.r.t belief state
%   gu: belief dynamics derivate w.r.t control
%   gbb: 2-nd order derivate of belief dynamics derivate w.r.t belief state
%   gbu: belief dynamics derivate w.r.t belief state and control
%   guu: 2-nd order derivate of belief dynamics derivate w.r.t control
%   cb: cost func derivate w.r.t belief state
%   cu: cost func derivate w.r.t control
%   cbb: 2-nd order derivate of cost func derivate w.r.t belief state
%   cbu: cost func derivate w.r.t belief state and control
%   cuu: 2-nd order derivate of cost func derivate w.r.t control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

beliefDim = size(b,1);
ctDim = motionModel.ctDim;

if nargout == 2
    g = beliefDynamics(b, u, motionModel, obsModel);
    c = costFunction(b, u, xf, L, motionModel.stDim, collisionChecker, map);
else
    % belief state and control indices
    ib = 1:beliefDim;
    iu = beliefDim+1:beliefDim+ctDim;
    
    M = size(map.obstacles,2); % no of obstacles
    
    % dynamics first derivatives
    xu_dyn  = @(xu) beliefDynamics(xu(ib,:),xu(iu,:),motionModel, obsModel);
    J       = finiteDifference(xu_dyn, [b; u]);
    gb      = J(:,ib,:);
    gu      = J(:,iu,:);

    % dynamics second derivatives
    if full_DDP
        xu_Jcst = @(xu) finite_difference(xu_dyn, xu);
        JJ      = finiteDifference(xu_Jcst, [b; u]);
        JJ      = reshape(JJ, [4 6 size(J)]);
        JJ      = 0.5*(JJ + permute(JJ,[1 3 2 4])); %symmetrize
        gbb     = JJ(:,ib,ib,:);
        gbu     = JJ(:,ib,iu,:);
        guu     = JJ(:,iu,iu,:);
    else
        [gbb,gbu,guu] = deal([]);
    end
    
%      if motionModel.stDim ~= 2
%         error('This partial of f w.r.t sigma is only valid for robot with state dimension 2')
%      end
    
    %% First derivative of sigmaToCollide (jacobian of sigma[b])
    tStart = tic;
    %xu_sigma =  @(x,obs_id) sigmaToCollideAll(x, motionModel.stDim, collisionChecker,map);
    dsigma_db = zeros(M,size(b,1),size(b,2));
    for m = 1:M
        xu_sigma = @(x) xu_sigma_m(x, m, motionModel.stDim, collisionChecker,map);
        dsigma_db(m,:,:)  = squeeze(finiteDifference(xu_sigma, b,1e-3)); % need to have a large step size to see derivative in collision
    end
    
    dsigma_db = [dsigma_db;zeros(M,motionModel.ctDim,size(dsigma_db,3))]; % jacobian w.r.t u is zero for collision
    
    nSigma = sigmaToCollideAll(b, motionModel.stDim, collisionChecker,map);
    fprintf('Time to do sigma derivative and compute sigma: %f seconds\n', toc(tStart))
    
    %% cost first derivatives
    xu_cost = @(xu) costFunction(xu(ib,:),xu(iu,:),xf,L,motionModel.stDim, collisionChecker,map,0);    
    J       = squeeze(finiteDifference(xu_cost, [b; u]));
    
    % construct Jacobian adding collision cost
    for i = 1:size(dsigma_db,3)
        
%         J(:,i) = J(:,i) + 300.*((-1/2)/(exp(nSigma(i)/2)-1)) * dsigma_db(:,i);
        J(:,i) = J(:,i) + 300.*(-exp(-nSigma)).' * dsigma_db(:,i);
    end
    
    cb      = J(ib,:);
    cu      = J(iu,:);
    
    %% cost second derivatives
    
    
    % first calculate Hessian excluding collision cost
    xu_cost_nocc = @(xu) costFunction(xu(ib,:),xu(iu,:),xf,L,motionModel.stDim, collisionChecker,map,0);
    xu_Jcst_nocc = @(xu) squeeze(finiteDifference(xu_cost_nocc, xu));    
    JJ      = finiteDifference(xu_Jcst_nocc, [b; u]);
    JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize                      
    
    % construct Hessian adding collision cost
    for i = 1:size(dsigma_db,2)
        for m = 1:size(dsigma_db,1)
            jjt = dsigma_db(m,i)*dsigma_db(m,i)';        
            JJ(:,:,i) = JJ(:,:,i) + 300.*((1/4)*exp(nSigma(i)/2)/(exp(nSigma(i)/2)-1)^2) * 0.5*(jjt+jjt');
        end
    end
    
    cbb     = JJ(ib,ib,:);
    cbu     = JJ(ib,iu,:);
    cuu     = JJ(iu,iu,:);            

    [g,c] = deal([]);
end


end

function n_sigma_m = xu_sigma_m(x,m, stDim, collisionChecker,map)
    n_sigma_all =  sigmaToCollideAll(x, stDim, collisionChecker,map);
    n_sigma_m = n_sigma_all(m,:);
end