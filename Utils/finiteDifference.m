function J = finiteDifference(fun, x, h)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simple finite-difference derivatives
% assumes the function fun() is vectorized
% Inputs:
%   fun: function to differentiate
%   x: point at which to diff
%   h: step-size
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin < 3
    h = 2^-17;
end

[n, K]  = size(x);
% H       = [zeros(n,1) h*eye(n)];
% H       = permute(H, [1 3 2]);
% X       = pp(x, H);
% X       = reshape(X, n, K*(n+1));
% Y       = fun(X);
% m       = numel(Y)/(K*(n+1));
% Y       = reshape(Y, m, K, n+1);
% J       = pp(Y(:,:,2:end), -Y(:,:,1)) / h;
% J       = permute(J, [1 3 2]);


H2       = [(-h/2)*eye(n) (h/2)*eye(n)];
H2       = permute(H2, [1 3 2]);
X2       = pp(x, H2);
X2       = reshape(X2, n, K*(2*n));
Y2       = fun(X2);
m2       = numel(Y2)/(K*(2*n));
Y2       = reshape(Y2, m2, K, 2*n);
J2       = pp(Y2(:,:,n+1:end), -Y2(:,:,1:n)) / h;
J2       = permute(J2, [1 3 2]);
J = J2;
end