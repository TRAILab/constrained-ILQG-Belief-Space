function vech = vech(matA)
% Function return elements from and below the main diagonal, then
% stacking by column to have a vector of size K*(K+1)/2.
% For example:  matA = [b11 b12 b13;b21 b22 b23;b31 b32 b33] is (3x3)
%               vech(matA) returns: [b11 b12 b13 b22 b23 b33]' is (6x1)
% Author: Binh T. Pham. Date: 2018-02-15.
[M,N] = size(matA);

if (M == N)
    vech  = [];
    for ii=1:M
        vech = [vech; matA(ii:end,ii)];
    end
else
     error('Input must be a symmetric matrix.')
end
end