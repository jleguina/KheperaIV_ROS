function zeta = dynamic_vector_time(m,N)
% x = dynamic_vector(m,N) assembles the whole dynamic extension vector with
% time-dependent variables
% of a system composed of N agents in m dimensions.
%
% Inputs:
%   - N(scalar): number of agents
%   - m(scalar): number of dimensions
%
% Output:
%   - x(vector): state vector
%
% JLP 17/02/2020

% Initialise symbolic variables
syms t

zeta = sym(zeros(2*m,(N-1)));
for j = 2:N
    for i=1:2*m
        ZZ = symfun(str2sym(sprintf('zeta%d_%d(t)', j, i)), t); %declare each element to a symbolic "handle"
        zeta(i,j) = ZZ; %paste the symbolic "handle" into an array
    end
end

end
