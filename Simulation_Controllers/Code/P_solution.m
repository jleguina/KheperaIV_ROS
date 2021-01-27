function P = P_solution(i,j,m,b)
% P = P_solution(i,j,m,b) computes the algebraic P solution for agents i, j
% according to cost functionals (6) in Mylvaganam 2019
%
% Inputs:
%   - i(scalar): agent i
%   - j(scalar): agent j
%   - m(scalar): number of dimensions
%   - b(vector): barrier functions
%
% Output:
%   - P(matrix): algebraic P solution
% 
% JLP 17/02/2020

% Determine P solutions
if i == j & i ~= 1
    P = blkdiag(2*eye(m)+sqrt(1+b(j))*eye(m),2*eye(m));
elseif i-j  == 1 & i ~= 1
    P = blkdiag(2*eye(m),3*eye(m));
else
    error("Matrix not required");
end

end