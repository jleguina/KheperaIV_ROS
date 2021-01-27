clear
clc

syms x_tilda1 x_tilda2 x_tilda3 x_tilda4...
    zeta1 zeta2 zeta3 zeta4...
    x1 x2 x3 x4

x_tilda = [x_tilda1; x_tilda2; x_tilda3; x_tilda4];
zeta = [zeta1; zeta2; zeta3; zeta4];
x = [x1; x2; x3; x4];

%% Problem formulation
N = 2;

% xf = [30; 0; -30; 0];
% x0 = [-30; 0; 30; 0];
% zeta0 = [60; -5; -5; -1];

x0 = [0; 0; 20; 20];
xf = [20; 20; 0; 0];
zeta0 = [15; -25; 0; 0];

% zeta0 = [10; -40; 0; 0];

tspan = [0, 15];

x_tilda0 = x0 - xf;

p = [6; 4];
rho = 2;

k = 1;
r = 1;
R = eye(4);

alpha = [1; 1]; % alpha>0
betaS = [1; 1]; % betaS>0
betaD = [1; 1]; % betaD>0
gamma = [4; 1]; % gamma>0
E = eye(2);
RE = eye(4);
kappa = 1; % kappa > kappa_star


%% Barrier functions
for i =1:N
    for j = 1:1
        gS(j,i) = 1./(eNorm(zeta((2*i-1):(2*i))+xf((2*i-1):(2*i))-p, E).^2 - rho^2);
    end
end

gS = sum(gS,1);

for i =1:N
    for j = 1:N
        if i ~= j
            gD(:,i) = 1/(norm((zeta((2*i-1):(2*i))+xf((2*i-1):(2*i))) - (zeta((2*j-1):(2*j))+xf((2*j-1):(2*j))))^2 - r^2);
        end
    end
end

gS = gS.';
gD = gD.';

for i=1:N
    diff_gS_zeta(:,i) = jacobian(gS(i), zeta);
    diff_gD_zeta(:,i) = jacobian(gD(i), zeta);
    diff_sum(:,i) = (betaS(i)*diff_gS_zeta(:,i) + betaD(i)*diff_gD_zeta(:,1));
end

%% Construct P and V to check HJ inequality later
P = sym([]);
for i=1:N
    P(:,:,i) = sqrt(alpha(i)+betaS(i)*gS(i)+betaD(i)*gD(i))*eye(4);
    %P = blkdiag(P,Pacc(:,:,i));
    
    V(i) = 0.5.* (x_tilda.'*P(i)*x_tilda + eNorm(x_tilda-zeta,RE).^2);
    diff_v_x(i,:) = jacobian(V(i), x_tilda);
    diff_v_zeta(i,:) = jacobian(V(i), zeta);
end

% Check if zeta0 conditions are valid
sym_vars = symvar(P);
P_check = double(subs(P,sym_vars,zeta0.'));
if P_check < abs(inf)
    disp("zeta_0 valid");
else
    error("zeta_0 NOT valid");
end

% Construct B matrices
B = zeros(2*N,2,N);
for i = 1:N
    B((2*i-1):(2*i),1:2,i) = eye(2);
end

% Construct q function
for i = 1:N
    q(i) = (alpha(i)+betaS(i)*gS(i)+betaD(i)*gD(i))*...
        (x_tilda((2*i-1):(2*i)).'*x_tilda((2*i-1):(2*i)));
end


%% ODEs
syms x_tilda1(t) x_tilda2(t) x_tilda3(t) x_tilda4(t)...
    zeta1(t) zeta2(t) zeta3(t) zeta4(t)

x_tilda = [x_tilda1(t); x_tilda2(t); x_tilda3(t); x_tilda4(t)];
zeta = [zeta1(t); zeta2(t); zeta3(t); zeta4(t)];

for i = 1:N
    summation(:,i) = x_tilda((2*i-1):(2*i))-zeta((2*i-1):(2*i));
    varsum(:,i) = N^i*summation(:,i);
end

varsum = sum(varsum,2);

for i = 1:N
    root = (sqrt(alpha(i) + betaS(i)*gS(i) + betaD(i)*gD(i)) + gamma(i));
    ode1((2*i-1):(2*i),1) = diff(x_tilda((2*i-1):(2*i)),t) == -x_tilda((2*i-1):(2*i)).*root - (varsum);
    ode2sum(:,i) = x_tilda((2*i-1):(2*i)).'*x_tilda((2*i-1):(2*i))/(2*root) * ...
        diff_sum(:,i) - R*(x_tilda-zeta);
end

zeta_dot = -kappa*sum(ode2sum,2);
ode2 = diff(zeta,t) == zeta_dot;

symvar(ode1)
symvar(ode2)

[L,S] = odeToVectorField(ode1,ode2);
F = matlabFunction(L,'vars',{'t','Y'});

y0 = [x_tilda0; zeta0];
[t, sol] = ode45(F, tspan, y0);


%% Construct HJ inequalities
for i = 1:N
    for j = 1:N
        if j~=i
            sum_HJ(i) = diff_v_x(i,:)*B(:,:,j)*B(:,:,j).'*diff_v_x(j,:).'
        end
    end
end

for i = 1:N
    HJ(i) = 0.5*diff_v_x(i,:)*B(:,:,i)*B(:,:,i).'*diff_v_x(i,:).' + ...
        0.5*q(i) - sum_HJ(i) + diff_v_zeta(i,:) * zeta_dot;;
end

HJ = HJ.';

x_tilda1 = sol(:,1);
x_tilda2 = sol(:,2);
x_tilda3 = sol(:,3);
x_tilda4 = sol(:,4);

zeta1 = sol(:,5);
zeta2 = sol(:,6);
zeta3 = sol(:,7);
zeta4 = sol(:,8);

sym_vars = symvar(HJ);

for k = 1:length(sol)
    HJ_subs(:,k) = double(subs(HJ,sym_vars,...
        [0, x_tilda1(k), x_tilda2(k), x_tilda3(k), x_tilda4(k), zeta1(k), zeta2(k), zeta3(k), zeta4(k)]));
    
    progress = k*100 / length(sol);
    if HJ_subs(:,k)>0
        error("HJ inquality not satisfied. Progress: %f", progress);
        break
    else
        fprintf("HJ inquality satisfied. Progress: %f", progress);
        fprintf('\n')
    end
end


%% Plotting
LW = 2;

sol_x = sol(:,1:4) + xf.';

x_pos_sol1 = sol_x(:,1);
y_pos_sol1 = sol_x(:,2);

x_pos_sol2 = sol_x(:,3);
y_pos_sol2 = sol_x(:,4);

plot(x_pos_sol1,y_pos_sol1,'r',"LineWidth",LW);
hold on
plot(x_pos_sol2,y_pos_sol2,'b',"LineWidth",LW);
hold on
circle(p(1),p(2),rho,'k');
circle(8.474,1.349,1,'b');
circle(2.655,6.915,1,'r');
grid on


title("Centralised Simulation",'Interpreter','latex')
xlabel("x-coordinate [$m$]",'Interpreter','latex')
ylabel("y-coordinate [$m$]",'Interpreter','latex')
axis([-5,25,-5,25]);

legend("Agent 1", "Agent 2",'Interpreter','latex')
set(gca, "FontSize", 18)
set(gca,'TickLabelInterpreter','latex')

% % Crop margin
% ax = gca;
% outerpos = ax.OuterPosition;
% ti = ax.TightInset; 
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax.Position = [left bottom ax_width ax_height];

%% Diastance plots
dist1 = abs(sqrt(x_pos_sol1.^2 + y_pos_sol1.^2) - sqrt(xf(1)^2 + xf(2)^2));
dist2 = abs(sqrt(x_pos_sol2.^2 + y_pos_sol2.^2) - sqrt(xf(3)^2 + xf(4)^2));


plot(t, dist1,'r',"LineWidth",LW);
hold on
plot(t, dist2,'b',"LineWidth",LW);
grid on

xlabel("Time [$s$]",'Interpreter','latex')
ylabel("Euclidean distance [$m$]",'Interpreter','latex')
axis([0,8,0,30]);

legend("Agent 2", "Agent 1",'Interpreter','latex')
set(gca, "FontSize", 18)
set(gca,'TickLabelInterpreter','latex')



