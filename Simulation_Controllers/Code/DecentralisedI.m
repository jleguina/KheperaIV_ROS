%% Part 0: Initialisation
clear; close all; clc;

%% Part 1: Problem Formulation
% Number of agents
N = 1;

% Dimensions
m = 2;

% Barrier parameters
d = ones(N*m,1);
r = 1;

% Optimisation weights
S = 30*eye(2*m);
R = eye(m);
g = eye(m);
k = 10;

%% Part 2: Initial Conditions
p1_0 = [0; 0];

x1_0 = p1_0 - d(1:2);
y0 = [x1_0];
tspan = [0, 30];

%% Part 3: Variable definition
% State Vector
x = state_vector(m,N);

% Dinamic Extension vector
zeta = dynamic_vector(m,N);

% Local States
x_hat = local_states(x(:),N,m);
zeta_hat = local_states(zeta(:),N,m);

% Communication graph
G = comm_graph(N,0);

% Barrier functions
b = barrier(N,zeta(:),d,r);

%% Part 4: Value function V
for i = 2:N
    Hop_1 = [predecessors(G,i), i];
    for k = 1:length(Hop_1)
        j = Hop_1(k);
        
        V(i,j) = 0.5*x_hat(:,i).'*P_solution(i,j,m,b)*x_hat(:,i)...
            + 0.5*(x_hat(:,i) - zeta_hat(:,i)).'*S*(x_hat(:,i) - zeta_hat(:,i));
        
    end
end

%% Part 5: ODEs
% First agent
u(:,1) = -x(:,1);

zeta_dot = sym(zeros(2*m,N));
for i = 2:N
    Hop_1 = [predecessors(G,i), i];
    for t = 1:length(Hop_1)
        j = Hop_1(t);
        zeta_dot(:,i) = zeta_dot(:,i) + -k * jacobian(V(i,j), zeta_hat(:,i)).'; 
    end
    u(:,i) = -inv(R) * g * jacobian(V(i,i), x(:,i)).' - u(:,i-1);
end


U = reshape(u(:, 1:end), [], 1);
Z = reshape(zeta_dot(:, 2:end), [], 1);


x_time = state_vector_time(m,N);

zeta_time = dynamic_vector_time(m,N);
zeta_time = local_states(zeta_time(:),N,m);

y = [x_time(:)];

for i = 2:N
   y = [y; zeta_time(:,i)];
end

syms t
ode = diff(y,t) == [U;Z];

symvar(ode);


%% Part 6: ODE45
[L,S] = odeToVectorField(ode);
F = matlabFunction(L,'vars',{'t','Y'});

[t, sol] = ode45(F, tspan, y0);


% [-Y(1);
%     -Y(2);
%     -(sqrt(1.0./(abs(Y(5)+Y(6)+2.0).^2-1.0).^2+1.0)+2.0).*Y(3)+Y(2)-Y(3).*3.0e+1+Y(5).*3.0e+1;
%     -(sqrt(1.0./(abs(Y(5)+Y(6)+2.0).^2-1.0).^2+1.0)+2.0).*Y(4)+Y(1)-Y(4).*3.0e+1+Y(6).*3.0e+1;
%     Y(3).*1.2e+2-Y(5).*1.2e+2+abs(Y(5)+Y(6)+2.0).*sign(Y(5)+Y(6)+2.0).*1.0./sqrt(1.0./(abs(Y(5)+Y(6)+2.0).^2-1.0).^2+1.0).*Y(3).^2.*1.0./(abs(Y(5)+Y(6)+2.0).^2-1.0).^3.*2.0+abs(Y(5)+Y(6)+2.0).*sign(Y(5)+Y(6)+2.0).*1.0./sqrt(1.0./(abs(Y(5)+Y(6)+2.0).^2-1.0).^2+1.0).*Y(4).^2.*1.0./(abs(Y(5)+Y(6)+2.0).^2-1.0).^3.*2.0;
%     Y(4).*1.2e+2-Y(6).*1.2e+2+abs(Y(5)+Y(6)+2.0).*sign(Y(5)+Y(6)+2.0).*1.0./sqrt(1.0./(abs(Y(5)+Y(6)+2.0).^2-1.0).^2+1.0).*Y(3).^2.*1.0./(abs(Y(5)+Y(6)+2.0).^2-1.0).^3.*2.0+abs(Y(5)+Y(6)+2.0).*sign(Y(5)+Y(6)+2.0).*1.0./sqrt(1.0./(abs(Y(5)+Y(6)+2.0).^2-1.0).^2+1.0).*Y(4).^2.*1.0./(abs(Y(5)+Y(6)+2.0).^2-1.0).^3.*2.0;
%     Y(2).*1.2e+2-Y(7).*1.2e+2;
%     Y(1).*1.2e+2-Y(8).*1.2e+2];




