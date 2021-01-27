function Q = Q_weight(i,j,N,b)
% Q = Q_weight(i,j,N,b) assembles the m-by-m Q weight matrix
%
% Inputs:
%   - i(scalar): agent i
%   - j(scalar): agent j
%   - N(scalar): number of agents
%   - b(vector): barrier functions
%
% Output:
%   - Q(matrix): weight matrix
% 
% JLP 17/02/2020

% Determine Q weights
if i == j & i == 1
    Q = eye(N);
    
elseif i == j & i ~= 1
    Q = blkdiag((1+b(j))*eye(N),zeros(N));
    
elseif i-j  == 1 & i ~= 1
    Q = blkdiag(zeros(N),(1+b(j))*eye(N));
else
    error("Matrix not required");
end

end