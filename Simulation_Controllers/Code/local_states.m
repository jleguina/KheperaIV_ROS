function X = local_states(x,N,m)
% X = local_states(N,x) computes the local states P solution for agents i, j
% according to cost functionals (6) in Mylvaganam 2019
%
% Inputs:
%   - x(vector): state vector
%   - N(scalar): number of agents
%   - N(scalar): number of dimensions
%
% Output:
%   - X(matrix): x-by-j local states for agent j
%
% JLP 17/02/2020


X = sym(zeros(2*m,N-1));

X(1:m,1) = x(1:m);
for i=2:N
    % Must take elements in d and x as m sized vectors
    ini = m*i - (m-1);
    fin = m*i;
    
    X(:,i) = [x(ini:fin);x(ini-m:fin-m)];
    
end

end