function [g,c,constr_values, gb,gu,gbb,gbu,guu,cb,cu,cbb,cbu,cuu] = beliefDynCostConstr(b,u,lagMultiplier, mu, k, xf, L,full_DDP,motionModel,obsModel, collisionChecker, conFunc, map)
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

if nargout == 3
    g = beliefDynamics(b, u, motionModel, obsModel);
    c = aug_lagrangian(b, u, lagMultiplier, mu, xf,  L, motionModel.stDim, collisionChecker, conFunc, map,  motionModel.D, k); 
    constr_values = conFunc(b,u,k);
else
    % belief state and control indices
    ib = 1:beliefDim;
    iu = beliefDim+1:beliefDim+ctDim;
    
    tStart = tic;
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
    
    fprintf('Time to do dynamics Jacobians: %f seconds\n', toc(tStart))
%      if motionModel.stDim ~= 2
%         error('This partial of f w.r.t sigma is only valid for robot with state dimension 2')
%      end
    
    %% First derivative of sigmaToCollide (jacobian of sigma[b])
    tStart = tic;
    %xu_sigma =  @(x) sigmaToCollide(x, motionModel.stDim, motionModel.D,collisionChecker);
    xu_sigma =  @(x) sigmaToCollide2(x, motionModel.stDim, motionModel.D,map);
    dsigma_db  = squeeze(finiteDifference(xu_sigma, b,1e-2)); % need to have a large step size to see derivative in collision
    dsigma_db = [dsigma_db;zeros(motionModel.ctDim,size(dsigma_db,2))]; % jacobian w.r.t u is zero for collision
    
    %nSigma = sigmaToCollide(b, motionModel.stDim, motionModel.D, collisionChecker);
    nSigma = sigmaToCollide2(b, motionModel.stDim, motionModel.D, map);
%     fprintf('Time to do sigma derivative and compute sigma: %f seconds\n', toc(tStart))
    
    %% cost first derivatives
    tStart = tic;
    J = zeros(beliefDim + ctDim,L+1);
    for i = 1:L+1
        xu_cost = @(xu) aug_lagrangian(xu(ib,:),xu(iu,:),lagMultiplier(:,i), mu(:,i), xf, L,motionModel.stDim, collisionChecker,conFunc, map,motionModel.D,i,0);    
        J(:,i)       = squeeze(finiteDifference(xu_cost, [b(:,i); u(:,i)]));
    end
    
    
    % construct Jacobian adding collision cost
    for i = 1:size(dsigma_db,2)               
        J(:,i) = J(:,i) + 350.*((-1/2)/(exp(nSigma(i)/2)-1)) * dsigma_db(:,i);
    end
    
    cb      = J(ib,:);
    cu      = J(iu,:);
%     fprintf('Time to do cost Jacobian: %f seconds\n', toc(tStart))
    
    %% cost second derivatives
    
    tStart = tic;
    % first calculate Hessian excluding collision cost
    JJ = zeros(beliefDim + ctDim,beliefDim + ctDim,L+1);
    for i = 1:L+1
        xu_cost_nocc = @(xu) aug_lagrangian(xu(ib,:),xu(iu,:),lagMultiplier(:,i), mu(:,i), xf, L,motionModel.stDim, collisionChecker,conFunc, map,motionModel.D,i,0);    
        xu_Jcst_nocc = @(xu) squeeze(finiteDifference(xu_cost_nocc, xu));    
        JJ(:,:,i)      = finiteDifference(xu_Jcst_nocc, [b(:,i); u(:,i)]);
    end
    JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize
    
    
    % construct Hessian adding collision cost
    for i = 1:size(dsigma_db,2)
        jjt = dsigma_db(:,i)*dsigma_db(:,i)';        
        JJ(:,:,i) = JJ(:,:,i) + 350.*((1/4)*exp(nSigma(i)/2)/(exp(nSigma(i)/2)-1)^2) * 0.5*(jjt+jjt');
    end
    
    cbb     = JJ(ib,ib,:);
    cbu     = JJ(ib,iu,:);
    cuu     = JJ(iu,iu,:);            
%     fprintf('Time to do cost Hessian: %f seconds\n', toc(tStart))
    
    [g,c,constr_values] = deal([]);
end
end