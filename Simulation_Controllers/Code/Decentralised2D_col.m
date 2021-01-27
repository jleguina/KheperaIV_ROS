%% Part 0: Initialisation
clear; close all; clc;

%% Part 1: Problem Formulation
% Number of agents
N = 3;

% Dimensions
m = 2;

% Barrier parameters
r = 0.07;

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

zeta2_0 = [-2; 0; 2; 0];
zeta3_0 = [-0.25; 0.15; 1; 0];
% zeta3_0 = [-0; 0; -0.5; 0];

p1_0 = p1_0 - d1;
p2_0 = p2_0 - d2;
p3_0 = p3_0 - d3;


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

%% Barrier functions
obs = [0.3, 0.7]';
rho = 0.1;

obs2 = [-0.65, 0.7]';
rho2 = 0.1;

b = sym(zeros(N,1));

b(1) = 0;
b(2) = (norm(zeta(:,2) + d2)^2 - rho^2 - r^2)^(-2);
% b(3) = (norm(zeta(:,3) + d3 - obs2)^2 - rho2^2 - r^2)^(-2) + (norm(zeta(:,3) + d3 - obs)^2 - rho^2 - r^2)^(-2) + (norm(zeta(:,3) + zeta(:,2) + d3 + d2 - obs)^2 - rho^2 - r^2)^(-2);
b(3) = (norm(zeta(:,3) + d3 - obs)^2 - rho^2 - r^2)^(-2) + (norm(zeta(:,3) + zeta(:,2) + d3 + d2 - obs)^2 - rho^2 - r^2)^(-2);


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

tspan = [0, 5];

% Variables are not transformed to a vector field in the same order they
% are given. They need to be found and assigned their corresponding
% initial condition
y0(S=="x1_1",1) = p1_0(1);
y0(S=="x1_2",1) = p1_0(2);
% y0(S=="x1_3",1) = x1_0(3);
y0(S=="x2_1",1) = p2_0(1);
y0(S=="x2_2",1) = p2_0(2);
% y0(S=="x2_3",1) = x2_0(3);
y0(S=="x3_1",1) = p3_0(1);
y0(S=="x3_2",1) = p3_0(2);
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


%% Part 7: Plotting simulink
LW = 2;

load out.mat
load out_online1.mat


myred = [0.85, 0.33, 0.1];
myblue = [0, 0.45, 0.74];
myyell = [0.93, 0.69, 0.13];

figure
hold on

a(1) = plot(p1(:,1),p1(:,2),"Color",myblue,"LineWidth",LW);
a(2) = plot(p2(:,1),p2(:,2),"Color",myred,"LineWidth", LW);
a(3) = plot(p3(1:123,1),p3(1:123,2),"Color",myyell,"LineWidth", LW);
plot(p3_online(:,1),p3_online(:,2),"Color",myyell,"LineWidth", LW);

% p_new = [p3(1:123,1),p3(1:123,2)
%     p3_online(:,1),p3_online(:,2)];
% 
% p3_first = [p3(1:123,1),p3(1:123,2)];
% for i = 1:550
%     
%     p3_first = [p3(1,1),p3(1,2)
%         p3_first];
%     
% end

a(4) = plot(out.x1_on,out.y1_on,':',"Color",myyell,"LineWidth",LW);
a(5) = plot(out.x2_on,out.y2_on,':',"Color",myblue,"LineWidth",LW);
a(6) = plot(out.x3_on,out.y3_on,':',"Color",myred,"LineWidth",LW);

load out_online2.mat

plot(out.x1_on,out.y1_on,':',"Color",myyell,"LineWidth",LW);
plot(out.x2_on,out.y2_on,':',"Color",myblue,"LineWidth",LW);
plot(out.x3_on,out.y3_on,':',"Color",myred,"LineWidth",LW);

circle(p1(end,1),p1(end,2),0.05,'k',0.25);
circle(p2(end,1),p2(end,2),0.05,'k',0.25);
circle(p3(end,1),p3(end,2),0.05,'k',0.25);


circle(p1(170,1),p1(170,2),r,myblue,1);
circle(p2(170,1),p2(170,2),r,myred,1);
circle(p3_online(47,1),p3_online(47,2),r,myyell,1);

% circle(p1(95,2),p1(95,3),r,myblue,1);
% circle(p2(95,2),p2(95,3),r,myred,1);
% circle(p3(95,2),p3(95,3),r,myyell,1);


circle(obs(1),obs(2),rho,'k',1);
circle(obs2(1),obs2(2),rho2,'k',1);
hold off
xlabel("x-axis")
ylabel("y-axis")
zlabel("z-axis")
% xlim([-2 3]);
% ylim([-2 3]);
% zlim([-3 2]);
grid on
box on

% title("Decentralised Simulation",'Interpreter','latex')
xlabel("x-coordinate",'Interpreter','latex')
ylabel("y-coordinate",'Interpreter','latex')
zlabel("z-coordinate",'Interpreter','latex')

legend([a(1) a(2) a(3) a(4) a(5) a(6)],"Reference 1", "Reference 2", "Reference 3","Gazebo 1", "Gazebo 2", "Gazebo 3")
set(gca, "FontSize", 18)

% TT = 12;
% 
% figure
% for i=1:length(t)
%     clf
%     hold on
%     circle(p1(i,1),p1(i,2),r/2,'r',0);
%     circle(p2(i,1),p2(i,2),r/2,'b',0);
%     circle(p3(i,1),p3(i,2),r/2,'g',0);
%     circle(obs(1),obs(2),rho,'k',0);
%     
% %     if i > TT/3 
% %     circle(p1(i-TT/3,1),p1(i-TT/3,2),r/2,'r',0.25);
% %     circle(p2(i-TT/3,1),p2(i-TT/3,2),r/2,'b',0.25);
% %     circle(p3(i-TT/3,1),p3(i-TT/3,2),r/2,'g',0.25);
% %     circle(obs(1),obs(2),rho,'k',0);
% %     end
% %     
% %     if i > TT/2
% %     circle(p1(i-TT/2,1),p1(i-TT/2,2),r/2,'r',0.5);
% %     circle(p2(i-TT/2,1),p2(i-TT/2,2),r/2,'b',0.5);
% %     circle(p3(i-TT/2,1),p3(i-TT/2,2),r/2,'g',0.5);
% %     circle(obs(1),obs(2),rho,'k',0);
% %     end
% %    
% %     if i>150
% %     circle(p1(i-TT,1),p1(i-TT,2),r/2,'r',0.75);
% %     circle(p2(i-TT,1),p2(i-TT,2),r/2,'b',0.75);
% %     circle(p3(i-TT,1),p3(i-TT,2),r/2,'g',0.75);
% %     circle(obs(1),obs(2),rho,'k',0);
% %     end
% 
%     plot(p1(:,1),p1(:,2),"LineWidth",LW);
%     plot(p2(:,1),p2(:,2),"LineWidth", LW);
%     plot(p3(:,1),p3(:,2),"LineWidth", LW);
%     
%     drawnow
%     hold off
% end


