function x = state_vector_time(m,N)
% x = state_vector(m,N) assembles the whole state vector of a system with
% time-dependent variables
% composed of N agents in m dimensions.
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
x = sym('x',[N m]).';


syms t

x = sym(zeros(m,N));
for j = 1:N
    for i=1:m
        XX = symfun(str2sym(sprintf('x%d_%d(t)', j, i)), t); %declare each element to a symbolic "handle"
        x(i,j) = XX; %paste the symbolic "handle" into an array
    end
end

end