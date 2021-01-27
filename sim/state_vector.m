function x = state_vector(m,N)
% x = state_vector(m,N) assembles the whole state vector of a system
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
end