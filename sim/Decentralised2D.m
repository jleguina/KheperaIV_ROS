%% Part 0: Initialisation
clear; close all; clc;

%% Part 1: Problem Formulation
% Number of agents
N = 3;

% Dimensions
m = 2;

% Barrier parameters
r = 1;

% Optimisation weights
S = 30*eye(2*m);
R = eye(m);
Q = eye(m);
g(:,:,3) = [1 0; 0 1; 1 0; 0 1];%rot90(eye(6,3),2);
g(:,:,2) = eye(2*m,m);

%% Part 2: Initial & Final Conditions
p1_0 = [0; 0];
p2_0 = [1; 0];
p3_0 = [2; 0];

d1 = [1;1];
d2 = [0;1];
d3 = [-1;1];
d = [d1; d2; d3];

zeta2_0 = [1; 1; 1; 1];
zeta3_0 = [1; 1; 1; 1];

x1_0 = p1_0 - d1;
x2_0 = p2_0 - d2;
x3_0 = p3_0 - d3;


%% Part 3: Definition of Variables
% State Vector
x = state_vector(m,N);

% Dinamic Extension vector
zeta = dynamic_vector(m,N);

% Local States
x_hat = local_states(x(:),N,m);
zeta_hat = dynamic_estimate(m,N); % Estimates for all agents. Agent 1 is 0

% Communication graph
G = comm_graph(N,0);

% Barrier functions
b = barrier(N,zeta(:),d,r);

% Construct B matrices
% B = zeros(2*N,m,N);
% for i = 1:N
%     ini = m*i-1;
%     fin = m*i;
%     B(ini:fin,1:m,i) = eye(m);
% end

%% Part 4: Value function V
V = sym(zeros(N,m));
for i = 1:N
    Hop_1 = [predecessors(G,i), i];
    for k = 1:length(Hop_1)
        j = Hop_1(k);
        
        if i == 1
            V(i,j) = 0;
        else
            V(i,j) = 0.5*x_hat(:,i).'*P_solution(i,j,m,b)*x_hat(:,i)...
                + 0.5*(x_hat(:,i) - zeta_hat(:,i)).'*S*(x_hat(:,i) - zeta_hat(:,i));
        end
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
        zeta_dot(:,i) = zeta_dot(:,i) - 1 * jacobian(V(i,j), zeta_hat(:,i)).';
    end
    u(:,i) = -inv(R) * g(:,:,i).' * jacobian(V(i,i), x_hat(:,i)).' - u(:,i-1);
end


U = reshape(u(:, 1:end), [], 1);
Z = reshape(zeta_dot(:, 2:end), [], 1);


x_time = state_vector_time(m,N);

zeta_time = dynamic_vector_time(m,N);

y = sym(zeros(N*m+(N-1)*2*m,1));
y = x_time(:);

for i = 2:N
    y = [y; zeta_time(:,i)];
end

ode = diff(y) == [U;Z];

% symvar(ode);

%% Part 6: ODE45
% TODO - Error arises because there are two incompatible sets of equations for diff zeta2
[L,S] = odeToVectorField(ode);
F = matlabFunction(L,'vars',{'t','Y'});

tspan = [0, 10];

% Variables are not transformed to a vector field in the same order they
% are given. They need to be found and assigned their corresponding
% initial condition
y0(S=="x1_1",1) = x1_0(1);
y0(S=="x1_2",1) = x1_0(2);
% y0(S=="x1_3",1) = x1_0(3);
y0(S=="x2_1",1) = x2_0(1);
y0(S=="x2_2",1) = x2_0(2);
% y0(S=="x2_3",1) = x2_0(3);
y0(S=="x3_1",1) = x3_0(1);
y0(S=="x3_2",1) = x3_0(2);
% y0(S=="x3_3",1) = x3_0(3);
y0(S=="zeta2_1",1) = zeta2_0(1);
y0(S=="zeta2_2",1) = zeta2_0(2);
y0(S=="zeta2_3",1) = zeta2_0(3);
y0(S=="zeta2_4",1) = zeta2_0(4);
% y0(S=="zeta2_5",1) = zeta2_0(5);
% y0(S=="zeta2_6",1) = zeta2_0(6);
y0(S=="zeta3_1",1) = zeta3_0(1);
y0(S=="zeta3_2",1) = zeta3_0(2);
y0(S=="zeta3_3",1) = zeta3_0(3);
y0(S=="zeta3_4",1) = zeta3_0(4);
% y0(S=="zeta3_5",1) = zeta3_0(5);
% y0(S=="zeta3_6",1) = zeta3_0(6);

[t, sol] = ode45(F, tspan, y0);

% Variables are not transformed to a vector field in the same order they
% are given. They need to be found and assigned to the corresponding column
% in the sol vector
x1_1_sol = sol(:,S=="x1_1");
x1_2_sol = sol(:,S=="x1_2");
% x1_3_sol = sol(:,S=="x1_3");
x2_1_sol = sol(:,S=="x2_1");
x2_2_sol = sol(:,S=="x2_2");
% x2_3_sol = sol(:,S=="x2_3");
x3_1_sol = sol(:,S=="x3_1");
x3_2_sol = sol(:,S=="x3_2");
% x3_3_sol = sol(:,S=="x3_3");

x1_sol = [x1_1_sol, x1_2_sol];
x2_sol = [x2_1_sol, x2_2_sol];
x3_sol = [x3_1_sol, x3_2_sol];

p1 = x1_sol + d1';
p2 = x2_sol + d2';
p3 = x3_sol + d3';


%% Part 7: Plotting
LW = 2;

hold on
plot(p1(:,1),p1(:,2),"LineWidth", LW);
plot(p2(:,1),p2(:,2),"LineWidth", LW);
plot(p3(:,1),p3(:,2),"LineWidth", LW);
hold off
xlabel("x-axis")
ylabel("y-axis")
zlabel("z-axis")
% xlim([-2 3]);
% ylim([-2 3]);
% zlim([-3 2]);
grid on


title("Decentralised Simulation",'Interpreter','latex')
xlabel("x-coordinate",'Interpreter','latex')
ylabel("y-coordinate",'Interpreter','latex')
zlabel("z-coordinate",'Interpreter','latex')

legend("Agent 1", "Agent 2",'Interpreter','latex')
set(gca, "FontSize", 18)




% %% Part 7: Check HJ inequalities
% for i = 2:N
%     Hop_1 = [predecessors(G,i), i];
%     for t = 1:length(Hop_1)
%         j = Hop_1(t);
%         q(i,j) = x(:,i).'*Q*x(:,i);
%
%         HJ(i,j) = 0.5*q(i,j)+0.5*jacobian(V(i,j),x(:,i))*g*inv(R)*g'*jacobian(V(i,j),x(:,i)).'
%
%
%     end
% end


