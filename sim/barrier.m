function B = barrier(N,x,d,r)
% B = barrier(N,x,d,r) computes the barrier function expressions for collision
% avoidance among N agents in m dimensions. Number of dimension m is
% implicitly declared by the size of vector d.
%
% Inputs:
%   - N(scalar): number of agents
%   - x(vector): N*m vector of state vectors for each agent
%   - d(vector): N*m vector containing desired distance vectors between agents
%                first vector element constitutes distance between agent 1
%                and the origin
%   - r(scalar): minimum admissible distance between agents
%
% Output:
%   - B(vector): N*m vector containing barrier functions
% 
% JLP 17/02/2020

% Number of dimensions
m = length(d)/N;

% Initialise vectors
P = sym(zeros(m,N));
B = sym(zeros(N,1));

% No barrier for first agent
B(1) = 0;

for i = 2:N
    
    % Must take elements in d and x as m sized vectors
    ini = m*i - (m-1);
    fin = m*i;
    
    % Store barrier function first elements before summation
    P(1:m,i) = x(ini:fin) + d(ini:fin);
    
end

% Assemble summation of fractional elements of barrier functions
for k = 2:N
    for i = 2:k
        B(k) = B(k) + (norm(sum(P(:,i:k),2))^2 - r^2)^(-2);    
    end
end

end