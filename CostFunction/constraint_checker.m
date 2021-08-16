function [c,violations] = constraint_checker(b,u, tol)
%CONSTRAINT_CHECKER Summary of this function goes here
%   Detailed explanation goes here
    c = zeros(3,size(u,2));
    violations = zeros(3,size(u,2));
    for i=1:size(u,2)
        c(:,i) = constraintFunc(b(:,i),u(:,i),i);
        violations(:,i) = c(:,i)>tol;
    end
end

