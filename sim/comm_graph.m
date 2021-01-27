function G = comm_graph(N,flag)
% G = comm_graph(N,flag) creates the communications graph for an N agent
% system 
%
% Inputs:
%   - N(scalar) : number of agents
%   - flag(bool): determines if vertices should be named or numbered
%       - 1 = names 
%       - 0 = numbered
%
% Output:
%   - G(graph): comms graph
% 
% JLP 17/02/2020

% Initialise adjacency matrix
adj = zeros(N);

% Fill adjacency matrix for acyclic directed graph
for i =1:N-1
    adj(i,i+1) = 1;
end

% Generate graph
G = digraph(adj);

% Name graph vertices
if flag == 1
    for i =1:N
        Names(i,1) = sprintf("Agent_%d",i);
    end
    G.Nodes.Name = Names;
end

end